// Offline paired comparison: read saved raw maps once, filter once, then correct a copy.
// Does not regenerate depth, modify inputs, run voxel fusion, or change the display.
#include "PlaneSweepCrossViewCorrection.h"
#include "PathUtility.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <set>
#include <stdexcept>

using namespace graphicsFX;
using Sweep = MultiViewUtility::MultiViewPlaneSweepUtility;
using Depth = MultiViewUtility::MultiViewDepthPointCloudUtility;
namespace fs = std::filesystem;

static void SavePly(const fs::path &file, const std::vector<Sweep::PlaneSweepPointCandidate> &points)
{
    std::ofstream out(file, std::ios::binary);
    out << "ply\nformat binary_little_endian 1.0\nelement vertex " << points.size()
        << "\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
    for (const auto &point : points)
    {
        const float xyz[]{ point.m_position.x, point.m_position.y, point.m_position.z };
        const uint8_t rgb[]{ uint8_t(std::lround(std::clamp(point.m_color.x, 0.f, 1.f) * 255)),
            uint8_t(std::lround(std::clamp(point.m_color.y, 0.f, 1.f) * 255)),
            uint8_t(std::lround(std::clamp(point.m_color.z, 0.f, 1.f) * 255)) };
        out.write(reinterpret_cast<const char *>(xyz), sizeof(xyz));
        out.write(reinterpret_cast<const char *>(rgb), sizeof(rgb));
    }
    if (!out) throw std::runtime_error("Cannot write comparison PLY");
}

int main(int argc, char **argv)
{
    try
    {
        if (argc < 4 || argc > 5) throw std::runtime_error(
            "Usage: PlaneSweepCrossViewCompare setup.ini raw_depth_folder NEW_output_folder [reference_count=6]");
        const fs::path output = fs::absolute(argv[3]);
        if (fs::exists(output)) throw std::runtime_error("Comparison output folder must be new");
        auto setup = LoadProgramSetup(argv[1]).m_colmap;
        if (!setup.m_planeSweepCoarseToFineEnabled) throw std::runtime_error("Offline comparison currently requires pyramid depth maps");
        const auto data = MultiViewUtility::LoadColmapCameras(setup);
        if (data.m_cameras.empty()) throw std::runtime_error("No input cameras");
        const std::size_t step = std::size_t(std::max(1, setup.m_planeSweepReferenceStep));
        std::vector<std::size_t> available;
        for (std::size_t i = 0; i < data.m_cameras.size(); i += step) available.push_back(i);
        const std::size_t count = std::min(available.size(), argc == 5 ? std::size_t(std::stoul(argv[4])) : std::size_t(6));
        if (!count) throw std::runtime_error("Reference count must be positive");
        std::vector<std::size_t> references;
        for (std::size_t i = 0; i < count; ++i) references.push_back(available[i * available.size() / count]);
        std::set<std::size_t> needed(references.begin(), references.end());
        // Load the exact nearest comparison maps for these references, not all full-size maps.
        for (auto r : references)
        {
            std::vector<std::pair<float, std::size_t>> neighbors;
            const auto &ref = data.m_cameras[r];
            for (auto i : available)
            {
                if (i == r) continue;
                const auto &other = data.m_cameras[i];
                const float alignment = std::max(0.f, dot(ref.m_forward, other.m_forward));
                neighbors.push_back({ length(ref.m_position - other.m_position) / (.25f + alignment), i });
            }
            std::sort(neighbors.begin(), neighbors.end());
            const auto n = std::min(neighbors.size(), std::size_t(std::max(1, setup.m_planeSweepCrossViewCount)));
            for (std::size_t i = 0; i < n; ++i) needed.insert(neighbors[i].second);
        }
        fs::create_directories(output);
        fs::copy_file(argv[1], output / "source_setup.ini");
        std::ofstream manifest(output / "inputs.csv");
        manifest << "camera_id,width,height,file_size,modified_ticks,path\n";
        const std::vector<std::string> extensions{ ".PNG", ".JPG", ".JPEG", ".BMP", ".TIF", ".TIFF" };
        const auto images = Depth::BuildFileLookup(setup.m_cameraImageFolder, extensions);
        const auto masks = Depth::BuildFileLookup(setup.m_cameraMaskFolder, extensions);
        std::vector<Sweep::PlaneSweepDepthMap> maps;
        std::vector<Sweep::PlaneSweepPointCandidate> points;
        for (auto i : needed)
        {
            const auto &camera = data.m_cameras[i];
            Sweep::PlaneSweepImage image;
            if (!Sweep::LoadPlaneSweepImage(data, setup, images, masks, i, image)) throw std::runtime_error("Cannot load reference image/mask");
            const auto path = fs::absolute(fs::path(argv[2]) / ("PlaneSweep_" + std::to_string(camera.m_imageId) + "_" + fs::path(camera.m_name).stem().string() + ".bin"));
            std::ifstream in(path, std::ios::binary);
            Sweep::PlaneSweepDepthMap map;
            map.m_camera = image.m_camera; map.m_intrinsic = image.m_intrinsic;
            map.m_imageWidth = image.m_texture.m_width; map.m_imageHeight = image.m_texture.m_height;
            in.read(reinterpret_cast<char *>(&map.m_width), sizeof(map.m_width));
            in.read(reinterpret_cast<char *>(&map.m_height), sizeof(map.m_height));
            const auto expectedWidth = std::max(1u, uint32_t(std::lround(map.m_imageWidth * setup.m_planeSweepImageScale)));
            const auto expectedHeight = std::max(1u, uint32_t(std::lround(map.m_imageHeight * setup.m_planeSweepImageScale)));
            if (!in || map.m_width != expectedWidth || map.m_height != expectedHeight ||
                fs::file_size(path) != 8 + uint64_t(map.m_width) * map.m_height * 4)
                throw std::runtime_error("Missing map or dimensions differ from active image scale: " + path.string());
            map.m_depthValues.resize(std::size_t(map.m_width) * map.m_height);
            in.read(reinterpret_cast<char *>(map.m_depthValues.data()), std::streamsize(map.m_depthValues.size() * sizeof(float)));
            if (!in) throw std::runtime_error("Cannot read raw map");
            manifest << camera.m_imageId << ',' << map.m_width << ',' << map.m_height << ',' << fs::file_size(path) << ','
                << fs::last_write_time(path).time_since_epoch().count() << ',' << std::quoted(path.string()) << '\n';
            const auto index = maps.size();
            if (std::find(references.begin(), references.end(), i) != references.end())
            {
                const auto stride = uint32_t(std::max(1, setup.m_planeSweepPointStep));
                for (uint32_t y = 0; y < map.m_height; y += stride)
                    for (uint32_t x = 0; x < map.m_width; x += stride)
                    {
                        const float z = map.m_depthValues[std::size_t(y) * map.m_width + x];
                        if (!(z > 0) || !std::isfinite(z)) continue;
                        const float ix = (float(x) + .5f) * map.m_imageWidth / map.m_width - .5f;
                        const float iy = (float(y) + .5f) * map.m_imageHeight / map.m_height - .5f;
                        Float4 color;
                        if (!Sweep::SamplePlaneSweepPixel(image, ix, iy, color) || color.w <= setup.m_planeSweepAlphaThreshold) continue;
                        points.push_back({ index, Sweep::UnprojectPixel(map.m_camera, map.m_intrinsic, ix, iy, z), color });
                    }
            }
            maps.push_back(std::move(map));
        }
        const auto rawCount = points.size();
        setup.m_planeSweepCrossViewConsistencyEnabled = true;
        setup.m_planeSweepCrossViewCorrectionEnabled = false;
        ColmapPlaneSweepResult result;
        Sweep::ApplyCrossViewConsistency(setup, maps, points, result);
        SavePly(output / "filter_only.ply", points);
        auto corrected = points;
        setup.m_planeSweepCrossViewCorrectionEnabled = true;
        const auto start = std::chrono::steady_clock::now();
        const auto stats = CorrectPlaneSweepCrossViewDepths(setup, maps, corrected);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        SavePly(output / "corrected.ply", corrected);
        std::ofstream shifts(output / "point_movements.csv");
        shifts << "point,reference_id,before_depth,after_depth,relative_change\n" << std::setprecision(9);
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            if (length(points[i].m_position - corrected[i].m_position) == 0) continue;
            const auto &camera = maps[points[i].m_depthMapIndex].m_camera;
            const float before = dot(points[i].m_position - camera.m_position, camera.m_forward);
            const float after = dot(corrected[i].m_position - camera.m_position, camera.m_forward);
            shifts << i << ',' << camera.m_imageId << ',' << before << ',' << after << ',' << (after - before) / before << '\n';
        }
        std::ofstream report(output / "summary.txt");
        report << std::setprecision(10) << "Same saved inputs for both arms; no depth estimation, voxel fusion, color correction or rendering change.\n"
            << "Saved inputs have no original run manifest: input timestamps and a copy of the supplied calibration/settings are recorded;"
            << " provenance of historical predictions cannot be independently established.\n"
            << "references=" << count << "\nloaded_maps=" << maps.size() << "\nraw_points=" << rawCount
            << "\nfiltered_points=" << points.size() << "\ncorrected_output_points=" << corrected.size()
            << "\npoints_moved=" << stats.m_corrected << "\ninsufficient_support=" << stats.m_insufficientSupport
            << "\nambiguous=" << stats.m_ambiguous << "\nunchanged=" << stats.m_unchanged
            << "\nmean_relative_movement=" << stats.MeanRelativeMovement() << "\nmaximum_relative_movement=" << stats.m_maxRelativeMovement
            << "\nmean_agreement_score_before=" << stats.MeanScoreBefore() << "\nmean_agreement_score_after=" << stats.MeanScoreAfter()
            << "\nsurface_supported=" << stats.m_surfaceSupported << "\nsurface_regularized=" << stats.m_regularized
            << "\nsurface_missing=" << stats.m_surfaceMissing << "\nsurface_edge_guard=" << stats.m_surfaceEdges
            << "\nconsensus_only=" << stats.m_consensusOnly << "\nrejected_proposals=" << stats.m_proposalsRejected
            << "\nmean_joint_objective_before=" << stats.MeanObjectiveBefore() << "\nmean_joint_objective_after=" << stats.MeanObjectiveAfter()
            << "\nmean_relative_surface_residual_before=" << stats.MeanSurfaceErrorBefore()
            << "\nmean_relative_surface_residual_after=" << stats.MeanSurfaceErrorAfter()
            << "\ncorrection_ms=" << ms << "\nAgreement scores are geometric proxies, not ground-truth accuracy.\n";
        if (!report || !manifest || !shifts) throw std::runtime_error("Cannot finish comparison reports");
        std::cout << "Paired comparison saved to " << output.string() << "; moved " << stats.m_corrected << " / " << points.size() << " points.\n";
        return 0;
    }
    catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
