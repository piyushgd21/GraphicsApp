// Controlled real-data GPU experiments: tilt-only ablation or three matching modes on identical inputs.
#include "PlaneSweepGpu.h"
#include "PlaneSweepGpuTestContext.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <unordered_set>

using namespace graphicsFX;
using DepthUtility = MultiViewUtility::MultiViewDepthPointCloudUtility;

static void Vector(std::ostream &out, const Float3 &v)
{ out << '[' << v.x << ',' << v.y << ',' << v.z << ']'; }

static void SaveArray(const std::filesystem::path &file, uint32_t width, uint32_t height, const std::vector<float> &values)
{
    if (values.size() != size_t(width) * height) throw std::runtime_error("Invalid benchmark array dimensions.");
    std::ofstream out(file, std::ios::binary);
    out.write(reinterpret_cast<const char *>(&width), sizeof(width));
    out.write(reinterpret_cast<const char *>(&height), sizeof(height));
    out.write(reinterpret_cast<const char *>(values.data()), std::streamsize(values.size() * sizeof(float)));
    if (!out) throw std::runtime_error("Cannot save benchmark array.");
}

static double Quantile(std::vector<float> values, double q)
{
    if (values.empty()) return 0;
    std::sort(values.begin(), values.end());
    return values[size_t(q * double(values.size() - 1))];
}

int main(int argc, char **argv)
{
    try
    {
        if (argc < 3) throw std::runtime_error("Usage: PlaneSweepTiltBenchmark setup.ini output_folder [maximum_reference_views] [modes]");
        const bool compareModes = argc > 4 && std::string(argv[4]) == "modes";
        if (argc > 4 && !compareModes) throw std::runtime_error("Unknown comparison: expected modes.");
        struct Variant { const char *name; PlaneSweepMatchingMode mode; bool tilt; };
        std::vector<Variant> variants;
        if (compareModes)
            variants = { { "planesweep", PlaneSweepMatchingMode::PlaneSweep, false },
                { "slanted_front", PlaneSweepMatchingMode::PatchMatchSlanted, false }, { "slanted", PlaneSweepMatchingMode::PatchMatchSlanted, true } };
        else
            variants = { { "tilt_off", PlaneSweepMatchingMode::PatchMatchSlanted, false }, { "tilt_on", PlaneSweepMatchingMode::PatchMatchSlanted, true } };
        const std::filesystem::path output = std::filesystem::absolute(argv[2]);
        if (std::filesystem::exists(output / "views.jsonl")) throw std::runtime_error("Output already contains a benchmark. Choose a new folder.");
        for (const auto &variant : variants) std::filesystem::create_directories(output / variant.name);
        std::filesystem::copy_file(argv[1], output / "source_setup.ini", std::filesystem::copy_options::overwrite_existing);
        auto setup = LoadProgramSetup(argv[1]).m_colmap;
        if (std::filesystem::path(setup.m_cameraIntrinsicFile).parent_path().filename() != "scan9")
            throw std::runtime_error("Expected the scan9 paths under #house data in setup.ini.");
        setup.m_planeSweepMatchingMode = PlaneSweepMatchingMode::PatchMatchSlanted;
        setup.m_planeSweepExecutionMode = PlaneSweepExecutionMode::Gpu;
        setup.m_planeSweepCoarseToFineEnabled = true;
        setup.m_planeSweepSaveDepth = setup.m_planeSweepSavePyramidDepth = false;
        setup.m_planeSweepDebugEnabled = setup.m_planeSweepExperimentMode = false;
        // Preserve raw per-view depths. Identical geometric checks are evaluated offline for both arms.
        setup.m_planeSweepCrossViewConsistencyEnabled = false;
        setup.m_planeSweepPointFusionEnabled = setup.m_planeSweepOutlierFilterEnabled = false;
        ValidatePlaneSweepSlantedSetup(setup);
        const auto cameras = MultiViewUtility::LoadColmapCameras(setup);
        if (cameras.m_cameras.empty()) throw std::runtime_error("House cameras are unavailable.");
        const auto sparse = DepthUtility::LoadSparseScalePoints(setup);
        const auto lookup = DepthUtility::BuildSparsePointLookup(sparse);
        const auto indices = SweepUtility::BuildCameraIndexLookup(cameras);
        const std::vector<std::string> extensions{ ".PNG", ".JPG", ".JPEG", ".BMP", ".TIF", ".TIFF" };
        const auto imageFiles = DepthUtility::BuildFileLookup(setup.m_cameraImageFolder, extensions);
        const auto maskFiles = DepthUtility::BuildFileLookup(setup.m_cameraMaskFolder, extensions);
        std::ofstream settings(output / "effective_settings.txt");
        settings << std::setprecision(12) << "dataset=" << setup.m_cameraImageFolder << "\nreconstruction=GPU_ONLY\n"
            << "registered_views=" << cameras.m_cameras.size() << "\nsparse_points=" << sparse.size()
            << "\nimage_scale=" << setup.m_planeSweepImageScale << "\nsource_count=" << setup.m_planeSweepSourceCount
            << "\nmin_valid_sources=" << setup.m_planeSweepMinValidSourceViews << "\nplane_count=" << setup.m_planeSweepPlaneCount
            << "\npyramid_levels=" << setup.m_planeSweepPyramidLevels << "\nlocal_samples=" << setup.m_planeSweepLocalSamples
            << "\npatch_radius=" << setup.m_planeSweepPatchRadius << "\ncost_method_enum=" << int(setup.m_planeSweepPatchCostMethod)
            << "\ncolor_sigma=" << setup.m_planeSweepPatchMatchColorSigma << "\nspatial_sigma=" << setup.m_planeSweepPatchMatchSpatialSigma
            << "\niterations=" << setup.m_planeSweepPatchMatchIterations << "\ncost_threshold=" << setup.m_planeSweepCostThreshold
            << "\nconfidence_enabled=" << setup.m_planeSweepConfidenceEnabled << "\nconfidence_gap=" << setup.m_planeSweepConfidenceMinimumDifference
            << "\ntexture_rejection_enabled=" << setup.m_planeSweepTextureRejectionEnabled << "\ntexture_stddev=" << setup.m_planeSweepTextureMinimumStdDev
            << "\nraw_depths_before_crossview_filter=true\ncomparison=" << (compareModes ? "matching_modes" : "tilt_only")
            << "\nonly_varied_parameter=" << (compareModes ? "PlaneSweepMatchingMode_and_PlaneSweepPatchMatchTiltEnabled" : "PlaneSweepPatchMatchTiltEnabled") << '\n';
        PlaneSweepGpuTestContext context; context.Initialize();
        PlaneSweepGpu gpu;
        gpu.Initialize(context.m_device, context.m_physicalDevice, context.m_queue, context.m_commandPool, context.m_shaderFloat64Enabled);
        VkPhysicalDeviceProperties device{}; vkGetPhysicalDeviceProperties(context.m_physicalDevice, &device);
        settings << "gpu=" << device.deviceName << '\n'; settings.flush();
        std::ofstream metadata(output / "views.jsonl"), sparseSamples(output / "sparse_samples.csv"), metrics(output / "gpu_metrics.csv");
        metadata << std::setprecision(12); sparseSamples << std::setprecision(12);
        sparseSamples << "image_id,point_index,x,y,depth\n";
        metrics << "image_id,tilt,valid_pixels,pixels,wall_ms,evaluations,median_tilt_degrees,p90_tilt_degrees,max_control_sine,variant\n";
        metrics << std::setprecision(12);
        const size_t requested = argc > 3 ? size_t(std::stoul(argv[3])) : cameras.m_cameras.size();
        const size_t count = std::min(requested, cameras.m_cameras.size());
        for (size_t view = 0; view < count; ++view)
        {
            const size_t referenceIndex = count == cameras.m_cameras.size() ? view : view * cameras.m_cameras.size() / count;
            SweepUtility::PlaneSweepImage reference;
            if (!SweepUtility::LoadPlaneSweepImage(cameras, setup, imageFiles, maskFiles, referenceIndex, reference))
                throw std::runtime_error("Cannot load a selected reference image.");
            const auto candidates = SweepUtility::SelectSourceCameraIndices(cameras, setup, indices, lookup, sparse, referenceIndex);
            std::vector<SweepUtility::PlaneSweepImage> sources;
            for (size_t index : candidates)
            {
                SweepUtility::PlaneSweepImage image;
                if (SweepUtility::LoadPlaneSweepImage(cameras, setup, imageFiles, maskFiles, index, image)) sources.push_back(std::move(image));
                if (sources.size() >= size_t(setup.m_planeSweepSourceCount)) break;
            }
            if (sources.size() < size_t(setup.m_planeSweepMinValidSourceViews)) throw std::runtime_error("Too few source images.");
            const auto range = SweepUtility::EstimateDepthRange(reference.m_camera, reference.m_intrinsic, setup, lookup, sparse);
            const auto &camera = reference.m_camera;
            const auto &intrinsic = reference.m_intrinsic;
            metadata << "{\"id\":" << camera.m_imageId << ",\"name\":" << std::quoted(camera.m_name) << ",\"position\":";
            Vector(metadata, camera.m_position); metadata << ",\"forward\":"; Vector(metadata, camera.m_forward);
            metadata << ",\"up\":"; Vector(metadata, camera.m_up); metadata << ",\"right\":"; Vector(metadata, camera.m_right);
            metadata << ",\"intrinsics\":[" << intrinsic.m_fx << ',' << intrinsic.m_fy << ',' << intrinsic.m_cx << ',' << intrinsic.m_cy << ','
                << intrinsic.m_k1 << ',' << intrinsic.m_k2 << ',' << intrinsic.m_p1 << ',' << intrinsic.m_p2 << "],\"image_size\":["
                << intrinsic.m_width << ',' << intrinsic.m_height << "],\"range\":[" << range.m_nearDepth << ',' << range.m_farDepth << "],\"sources\":[";
            for (size_t i = 0; i < sources.size(); ++i) metadata << (i ? "," : "") << sources[i].m_camera.m_imageId;
            metadata << "]}\n"; metadata.flush();
            const auto visible = lookup.find(camera.m_imageId);
            if (visible != lookup.end())
            {
                std::unordered_set<size_t> seen;
                for (size_t index : visible->second)
                {
                    if (!seen.insert(index).second) continue;
                    float x{}, y{}, z{};
                    if (SweepUtility::ProjectPoint(camera, intrinsic, sparse[index].m_position, x, y, z))
                        sparseSamples << camera.m_imageId << ',' << index << ',' << x - 0.5f << ',' << y - 0.5f << ',' << z << '\n';
                }
            }
            sparseSamples.flush();
            for (size_t arm = 0; arm < variants.size(); ++arm)
            {
                // Rotate the order in mode comparisons to distribute pipeline warmup and timing bias.
                const auto &variant = variants[(arm + (compareModes ? view : 0)) % variants.size()];
                const bool tilt = variant.tilt;
                setup.m_planeSweepMatchingMode = variant.mode;
                setup.m_planeSweepPatchMatchTiltEnabled = tilt;
                SweepUtility::PlaneSweepDepthMap map;
                PlaneSweepSlantedDiagnostics diagnostics;
                std::vector<PlaneSweepPyramidStats> stats;
                std::string failure;
                const auto start = std::chrono::steady_clock::now();
                // There is deliberately no CPU reconstruction call or fallback in this executable.
                if (!gpu.RunCoarseToFineDepthMap(reference, sources, setup, range, map, &stats, &failure, &diagnostics))
                    throw std::runtime_error("GPU benchmark failed: " + failure);
                const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
                const auto folder = output / variant.name;
                const std::string stem = std::to_string(camera.m_imageId);
                SaveArray(folder / (stem + ".depth.bin"), map.m_width, map.m_height, map.m_depthValues);
                SaveArray(folder / (stem + ".cost.bin"), map.m_width, map.m_height, diagnostics.m_costs);
                size_t valid = 0;
                float maximumSine = 0;
                std::vector<float> angles;
                std::vector<float> tiltMap(map.m_depthValues.size(), 0);
                for (size_t i = 0; i < map.m_depthValues.size(); ++i)
                {
                    if (map.m_depthValues[i] <= 0) continue;
                    ++valid;
                    if (variant.mode == PlaneSweepMatchingMode::PlaneSweep) continue;
                    const auto &p = diagnostics.m_planes[i];
                    const Float3 k{ p.x, p.y, p.z };
                    const float size = length(k);
                    if (size <= 0) throw std::runtime_error("Invalid diagnostic plane.");
                    const float cosine = std::clamp(dot(k, camera.m_forward) / (size * length(camera.m_forward)), -1.0f, 1.0f);
                    maximumSine = std::max(maximumSine, length(cross(k, camera.m_forward)) / (size * length(camera.m_forward)));
                    angles.push_back(std::acos(cosine) * 180.0f / Pi);
                    tiltMap[i] = angles.back();
                }
                if (compareModes) SaveArray(folder / (stem + ".tilt.bin"), map.m_width, map.m_height, tiltMap);
                if (!tilt && maximumSine > 1e-4f) throw std::runtime_error("Tilt-off control contains tilted planes.");
                uint64_t evaluations = 0;
                for (const auto &level : stats) evaluations += level.m_evaluations;
                metrics << camera.m_imageId << ',' << tilt << ',' << valid << ',' << map.m_depthValues.size() << ',' << elapsed << ',' << evaluations
                    << ',' << Quantile(angles, 0.5) << ',' << Quantile(angles, 0.9) << ',' << (!tilt ? maximumSine : 0) << ',' << variant.name << '\n';
                metrics.flush();
                std::cout << "VIEW " << view + 1 << '/' << count << " id=" << camera.m_imageId << " variant=" << variant.name
                    << " valid=" << valid << " ms=" << elapsed << " median_tilt=" << Quantile(angles, 0.5) << std::endl;
            }
        }
        std::cout << "GPU comparison completed: " << output.string() << std::endl;
        return 0;
    }
    catch (const std::exception &error) { std::cerr << error.what() << std::endl; return 1; }
}
