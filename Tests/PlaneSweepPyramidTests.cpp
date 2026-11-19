#include "PlaneSweepGpu.h"
#include "PlaneSweepGpuTestContext.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <chrono>
#include <stdexcept>

using namespace graphicsFX;

static void Require(bool condition, const char *message)
{ if (!condition) throw std::runtime_error(message); }

static SweepUtility::PlaneSweepImage Plane(float cameraX, float depth, uint32_t width = 129, uint32_t height = 97, bool foreground = false)
{
    SweepUtility::PlaneSweepImage image;
    image.m_camera.m_position.x = cameraX;
    image.m_intrinsic.m_width = width; image.m_intrinsic.m_height = height;
    image.m_intrinsic.m_fx = image.m_intrinsic.m_fy = 96;
    image.m_intrinsic.m_cx = width * 0.5; image.m_intrinsic.m_cy = height * 0.5;
    image.m_texture.m_width = width; image.m_texture.m_height = height;
    image.m_texture.m_pixels.resize(size_t(width) * height * 4, 255);
    for (uint32_t y = 0; y < height; ++y)
        for (uint32_t x = 0; x < width; ++x)
        {
            const double ray = (x + 0.5 - width * 0.5) / 96;
            const double d = foreground && std::fabs(cameraX + ray * 1.9) < 0.25 ? 1.9 : depth;
            const double wx = cameraX + ray * d;
            const double wy = (y + 0.5 - height * 0.5) / 96 * d;
            const size_t i = (size_t(y) * width + x) * 4;
            for (int c = 0; c < 3; ++c)
                image.m_texture.m_pixels[i + c] = uint8_t(std::lround(128 + 45 * std::sin(wx * (6 + c * 2) + wy * 2) + 38 * std::cos(wy * 9 - wx * (3 + c))));
        }
    return image;
}

static void Compare(PlaneSweepGpu &gpu, const SweepUtility::PlaneSweepImage &reference,
    const std::vector<SweepUtility::PlaneSweepImage> &sources, const ColmapSetup &setup,
    const SweepUtility::PlaneSweepDepthRange &range, bool capture = false)
{
    SweepUtility::PlaneSweepDepthMap actual;
    std::string error;
    auto gpuSetup = setup; gpuSetup.m_planeSweepSavePyramidDepth = capture; gpuSetup.m_planeSweepOutputFolder = "Output/BoundaryParity/GPU";
    auto cpuSetup = setup; cpuSetup.m_planeSweepSavePyramidDepth = capture; cpuSetup.m_planeSweepOutputFolder = "Output/BoundaryParity/CPU";
    Require(gpu.RunCoarseToFineDepthMap(reference, sources, gpuSetup, range, actual, nullptr, &error), error.c_str());
    const auto expected = RunPlaneSweepPyramidCpu(reference, sources, cpuSetup, range);
    Require(actual.m_depthValues.size() == expected.m_depthValues.size(), "CPU/GPU dimensions differ");
    size_t differences = 0;
    for (size_t i = 0; i < expected.m_depthValues.size(); ++i)
        if (!std::isfinite(actual.m_depthValues[i]) || std::fabs(actual.m_depthValues[i] - expected.m_depthValues[i]) > 2e-4f)
        {
            if (differences < 12) std::cout << "Difference at " << i % actual.m_width << "," << i / actual.m_width << ": CPU=" << expected.m_depthValues[i] << " GPU=" << actual.m_depthValues[i] << '\n';
            ++differences;
        }
    std::cout << "Cost " << int(setup.m_planeSweepPatchCostMethod) << ": " << differences << " CPU/GPU differences" << std::endl;
    Require(differences == 0, "CPU/GPU candidate schedule or matching differs");
    if (capture)
    {
        std::vector<float> foregroundErrors;
        for (uint32_t y = 8; y <= 16; ++y)
            for (uint32_t x = 14; x <= 18; ++x)
            {
                const float depth = actual.m_depthValues[size_t(y) * actual.m_width + x];
                if (depth > 0) foregroundErrors.push_back(std::fabs(depth - 1.9f));
            }
        Require(foregroundErrors.size() > 30, "Foreground surface disappeared during refinement");
        std::sort(foregroundErrors.begin(), foregroundErrors.end());
        Require(foregroundErrors[foregroundErrors.size() / 2] < 0.08f, "Foreground depth accuracy failed");
    }
}

static void RealScene(PlaneSweepGpu &gpu, const char *configuration)
{
    auto setup = LoadProgramSetup(configuration).m_colmap;
    const auto cameras = MultiViewUtility::LoadColmapCameras(setup);
    Require(!cameras.m_cameras.empty(), "Real-scene cameras are unavailable");
    setup.m_planeSweepReferenceStep = int(cameras.m_cameras.size()); // One reference, original source selection.
    setup.m_planeSweepExecutionMode = PlaneSweepExecutionMode::Gpu;
    setup.m_planeSweepPointStep = 1; // Equal dense output grids for the comparison.
    setup.m_planeSweepCrossViewConsistencyEnabled = false;
    setup.m_planeSweepPointFusionEnabled = false; setup.m_planeSweepOutlierFilterEnabled = false;
    setup.m_planeSweepFusionMode = PlaneSweepFusionMode::None;
    setup.m_planeSweepExperimentMode = false; setup.m_planeSweepDebugEnabled = false;
    for (bool pyramid : { true, false })
    {
        setup.m_planeSweepCoarseToFineEnabled = pyramid;
        setup.m_planeSweepSaveDepth = false; setup.m_planeSweepSavePyramidDepth = false;
        setup.m_planeSweepOutputFolder = std::string("Output/PlaneSweepPyramidSmoke/") + (pyramid ? "Pyramid" : "Exhaustive");
        for (int run = 0; run < 3; ++run)
        {
            PlaneSweepExperimentMetrics metrics;
            const auto start = std::chrono::steady_clock::now();
            const auto result = MultiViewUtility::BuildColmapPlaneSweepPointCloud(cameras, setup, &metrics, &gpu);
            const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            std::cout << (pyramid ? "Pyramid" : "Exhaustive") << (run == 0 ? " warmup" : " measured") << " real reference: points=" << result.m_points.size()
                << ", references=" << result.m_referenceImagesProcessed << ", wall ms=" << elapsed
                << ", pyramid candidates=" << metrics.m_pyramidCandidates << ", widened=" << metrics.m_pyramidWidenedSearches
                << ", global recoveries=" << metrics.m_pyramidGlobalRecoveries << std::endl;
            Require(result.m_referenceImagesProcessed == 1 && !result.m_points.empty(), "Real scene reconstruction failed");
        }
    }
}

int main(int argc, char **argv)
{
    try
    {
        PlaneSweepGpuTestContext context; context.Initialize();
        PlaneSweepGpu gpu;
        gpu.Initialize(context.m_device, context.m_physicalDevice, context.m_queue, context.m_commandPool, context.m_shaderFloat64Enabled);
        if (argc == 3 && std::string(argv[1]) == "--real") { RealScene(gpu, argv[2]); return 0; }
        ColmapSetup setup;
        setup.m_planeSweepCoarseToFineEnabled = true;
        setup.m_planeSweepImageScale = 0.5f; setup.m_planeSweepPlaneCount = 16;
        setup.m_planeSweepPatchRadius = 1; setup.m_planeSweepMinValidSourceViews = 1;
        setup.m_planeSweepConfidenceEnabled = false; setup.m_planeSweepTextureRejectionEnabled = false;
        setup.m_planeSweepSaveDepth = false; setup.m_planeSweepCostThreshold = 0.2f;
        const auto reference = Plane(0, 2.37f), source = Plane(0.35f, 2.37f);
        const auto pyramid = BuildPlaneSweepPyramid(reference, { source }, setup);
        Require(pyramid.size() == 3, "Expected three pyramid levels");
        for (const auto &level : pyramid)
        {
            const auto &intrinsic = level.m_reference.m_intrinsic;
            Require(std::fabs(intrinsic.m_fx / 96 - double(intrinsic.m_width) / 129) < 1e-12, "Incorrect odd-size intrinsic scaling");
            const auto p = SweepUtility::UnprojectPixel(level.m_reference.m_camera, intrinsic, 3, 2, 2.37f);
            const float ox = (3.5f * 129 / float(intrinsic.m_width)) - 0.5f;
            const float oy = (2.5f * 97 / float(intrinsic.m_height)) - 0.5f;
            const auto q = SweepUtility::UnprojectPixel(reference.m_camera, reference.m_intrinsic, ox, oy, 2.37f);
            Require(std::fabs(p.x - q.x) < 1e-5f && std::fabs(p.y - q.y) < 1e-5f, "Pyramid rays do not align");
        }
        SweepUtility::PlaneSweepDepthRange range; range.m_nearDepth = 1.5f; range.m_farDepth = 4;
        SweepUtility::PlaneSweepDepthMap map;
        std::vector<PlaneSweepPyramidStats> stats;
        std::string error;
        Require(gpu.RunCoarseToFineDepthMap(reference, { source }, setup, range, map, &stats, &error), error.c_str());
        std::vector<float> errors;
        for (uint32_t y = map.m_height / 4; y < map.m_height * 3 / 4; ++y)
            for (uint32_t x = map.m_width / 4; x < map.m_width * 3 / 4; ++x)
            {
                float depth = map.m_depthValues[size_t(y) * map.m_width + x];
                if (depth > 0) errors.push_back(std::fabs(depth - 2.37f));
            }
        Require(errors.size() > 400, "Insufficient valid known-plane depths");
        std::sort(errors.begin(), errors.end());
        std::cout << "Known plane median absolute depth error: " << errors[errors.size() / 2] << '\n';
        Require(errors[errors.size() / 2] < 0.08f, "Known-plane accuracy failed");
        Require(stats.back().m_evaluations < uint64_t(map.m_width) * map.m_height * 16 * 2, "Local refinement unexpectedly expensive");
        const auto cpu = RunPlaneSweepPyramidCpu(reference, { source }, setup, range);
        size_t differences = 0;
        float maxDifference = 0;
        for (size_t i = 0; i < cpu.m_depthValues.size(); ++i)
        {
            const float difference = std::fabs(cpu.m_depthValues[i] - map.m_depthValues[i]);
            maxDifference = std::max(maxDifference, difference);
            if (difference > 2e-4f)
            {
                if (differences < 15) std::cout << "Mismatch " << i % map.m_width << "," << i / map.m_width << ": CPU=" << cpu.m_depthValues[i] << " GPU=" << map.m_depthValues[i] << '\n';
                ++differences;
            }
        }
        std::cout << "MAD CPU/GPU differences: " << differences << ", max=" << maxDifference << std::endl;
        Require(differences == 0, "MAD CPU/GPU pyramid mismatch");
        auto transparent = source;
        for (size_t i = 3; i < transparent.m_texture.m_pixels.size(); i += 4) transparent.m_texture.m_pixels[i] = 0;
        Require(gpu.RunCoarseToFineDepthMap(reference, { transparent }, setup, range, map), "Transparent-source dispatch failed");
        Require(std::all_of(map.m_depthValues.begin(), map.m_depthValues.end(), [](float d) { return d == 0; }), "Transparent sources produced depths");
        Require(stats.back().m_globalFallbacks > 0, "Expected recovery near invalid parents/image boundaries");
        const auto smallReference = Plane(0, 2.37f, 65, 49), smallSource = Plane(0.2f, 2.37f, 65, 49);
        for (int method = 0; method < 6; ++method)
        {
            if ((method == 1 || method == 2) && !context.m_shaderFloat64Enabled) continue;
            setup.m_planeSweepPatchCostMethod = static_cast<PatchCostMethod>(method);
            Compare(gpu, smallReference, { smallSource }, setup, range);
        }
        setup.m_planeSweepPatchCostMethod = PatchCostMethod::PatchCostMethodMeanAbsoluteDifference;
        setup.m_planeSweepConfidenceEnabled = true; setup.m_planeSweepConfidenceMinimumDifference = 0.0001f;
        setup.m_planeSweepTextureRejectionEnabled = true; setup.m_planeSweepTextureMinimumStdDev = 0.01f;
        Compare(gpu, smallReference, { smallSource }, setup, range);
        auto masked = smallReference;
        masked.m_hasMask = true; masked.m_maskUsesAlpha = true;
        masked.m_maskTexture = masked.m_texture;
        for (uint32_t y = 0; y < masked.m_maskTexture.m_height; ++y)
            for (uint32_t x = 0; x < masked.m_maskTexture.m_width; ++x)
                masked.m_maskTexture.m_pixels[(size_t(y) * masked.m_maskTexture.m_width + x) * 4 + 3] = x < 20 ? 0 : 255;
        Compare(gpu, masked, { smallSource }, setup, range);
        const auto maskedPyramid = BuildPlaneSweepPyramid(masked, { smallSource }, setup);
        Require(maskedPyramid.back().m_reference.m_texture.m_pixels[3] == 0, "Mask was lost during downsampling");
        setup.m_planeSweepPyramidLevels = 1;
        Compare(gpu, smallReference, { smallSource }, setup, range);
        setup.m_planeSweepPyramidLevels = 3;
        setup.m_planeSweepConfidenceEnabled = false; setup.m_planeSweepTextureRejectionEnabled = false;
        Compare(gpu, Plane(0, 3.2f, 65, 49, true), { Plane(0.2f, 3.2f, 65, 49, true) }, setup, range, true);
        setup.m_planeSweepImageScale = 0.39f;
        Compare(gpu, smallReference, { Plane(0.2f, 2.37f, 73, 55), Plane(-0.2f, 2.37f, 65, 49) }, setup, range);
        Require(!gpu.RunCoarseToFineDepthMap(reference, std::vector<SweepUtility::PlaneSweepImage>(9, source), setup, range, map), "Unsupported source count was silently truncated");
        std::cout << "GPU pyramid geometry, local refinement and invalid-source tests passed.\n";
        return 0;
    }
    catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
