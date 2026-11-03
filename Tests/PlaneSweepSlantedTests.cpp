#include "PlaneSweepGpu.h"
#include "PlaneSweepGpuTestContext.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace graphicsFX;

static void Require(bool condition, const char *message)
{ if (!condition) throw std::runtime_error(message); }

// Analytically render a textured 3D plane into calibrated cameras, including lens distortion.
static SweepUtility::PlaneSweepImage Render(float cameraX, bool edge = false, bool distorted = false, float yaw = 0)
{
    SweepUtility::PlaneSweepImage image;
    image.m_camera.m_position.x = cameraX;
    image.m_camera.m_forward = { std::sin(yaw), 0, -std::cos(yaw) };
    image.m_camera.m_right = { std::cos(yaw), 0, std::sin(yaw) };
    auto &intrinsic = image.m_intrinsic;
    intrinsic.m_width = 49; intrinsic.m_height = 37;
    intrinsic.m_fx = intrinsic.m_fy = 55;
    intrinsic.m_cx = 24.5; intrinsic.m_cy = 18.5;
    if (distorted) { intrinsic.m_k1 = 0.06; intrinsic.m_p1 = 0.001; }
    image.m_texture.m_width = intrinsic.m_width; image.m_texture.m_height = intrinsic.m_height;
    image.m_texture.m_pixels.resize(size_t(intrinsic.m_width) * intrinsic.m_height * 4, 255);
    for (uint32_t y = 0; y < intrinsic.m_height; ++y)
        for (uint32_t x = 0; x < intrinsic.m_width; ++x)
        {
            const auto ray = SweepUtility::UnprojectPixel(image.m_camera, intrinsic, float(x), float(y), 1) - image.m_camera.m_position;
            float depth = (2.4f + 0.9f * cameraX) / (-ray.z - 0.9f * ray.x - 0.2f * ray.y);
            const bool foreground = edge && cameraX + ray.x * 1.8f < 0;
            if (foreground) depth = 1.8f;
            const auto p = image.m_camera.m_position + ray * depth;
            const size_t i = (size_t(y) * intrinsic.m_width + x) * 4;
            for (int c = 0; c < 3; ++c)
            {
                float value = 125 + 38 * std::sin(p.x * (13 + c * 2) + p.y * 4) + 30 * std::cos(p.y * 17 - p.x * 3);
                if (edge) value = value * 0.55f + (foreground ? (c == 0 ? 100.0f : 5.0f) : (c == 2 ? 100.0f : 5.0f));
                image.m_texture.m_pixels[i + size_t(c)] = uint8_t(std::lround(std::clamp(value, 0.0f, 255.0f)));
            }
        }
    return image;
}

static float Error(const SweepUtility::PlaneSweepDepthMap &map, const SweepUtility::PlaneSweepImage &reference, bool edge)
{
    std::vector<float> errors;
    for (uint32_t y = 7; y + 7 < map.m_height; ++y)
        for (uint32_t x = 7; x + 7 < map.m_width; ++x)
        {
            auto ray = SweepUtility::UnprojectPixel(reference.m_camera, reference.m_intrinsic, float(x), float(y), 1);
            float expected = edge && ray.x < 0 ? 1.8f : 2.4f / (-ray.z - 0.9f * ray.x - 0.2f * ray.y);
            float depth = map.m_depthValues[size_t(y) * map.m_width + x];
            if (depth > 0) errors.push_back(std::fabs(depth - expected));
        }
    Require(errors.size() > 450, "Too few reconstructed surface pixels");
    std::sort(errors.begin(), errors.end());
    return errors[errors.size() / 2];
}

// Inherited tilted planes must initialize fine pixels even without sweep seeds.
static void CheckPlaneInheritance()
{
    const auto reference = Render(0);
    PlaneSweepPyramidLevel level{ reference, { Render(0.22f), Render(-0.22f) } };
    ColmapSetup setup;
    setup.m_planeSweepMatchingMode = PlaneSweepMatchingMode::PatchMatchSlanted;
    setup.m_planeSweepPlaneCount = 24; setup.m_planeSweepPatchRadius = 3;
    setup.m_planeSweepMinValidSourceViews = 2; setup.m_planeSweepPatchMatchColorSigma = 0.45f;
    setup.m_planeSweepConfidenceEnabled = setup.m_planeSweepTextureRejectionEnabled = false;
    setup.m_planeSweepCostThreshold = 0.3f;
    SweepUtility::PlaneSweepDepthRange range; range.m_nearDepth = 1.2f; range.m_farDepth = 4.5f;
    const uint32_t width = reference.m_texture.m_width, height = reference.m_texture.m_height;
    const float spacing = (1.0f / range.m_nearDepth - 1.0f / range.m_farDepth) / float(setup.m_planeSweepPlaneCount - 1);
    const PlaneSweepPyramidState empty{ 0, std::numeric_limits<float>::max(), 0, spacing };
    // Odd parent dimensions exercise pixel-center mapping. The stored cost is deliberately stale.
    const uint32_t parentWidth = 13, parentHeight = 10;
    const std::vector<Float4> parents(size_t(parentWidth) * parentHeight, { -0.9f / 2.4f, -0.2f / 2.4f, -1.0f / 2.4f, 0.9f });
    std::vector<PlaneSweepPyramidState> state(size_t(width) * height, empty);
    PlaneSweepPyramidStats statistics;
    const auto planes = RefinePlaneSweepSlantedLevelCpu(level, setup, range, state, parents, parentWidth, parentHeight, true, statistics);
    Require(planes.size() == state.size() && statistics.m_accepted > 450, "Parent planes did not initialize fine pixels without sweep seeds");
    Require(Error(MakePlaneSweepPyramidDepthMap(reference, width, height, state), reference, false) < 0.07f,
        "Inherited planes were scaled or transferred incorrectly");
    setup.m_planeSweepConfidenceEnabled = true; setup.m_planeSweepConfidenceMinimumDifference = 10;
    state.assign(state.size(), empty);
    statistics = {};
    RefinePlaneSweepSlantedLevelCpu(level, setup, range, state, parents, parentWidth, parentHeight, false, statistics);
    const auto provisional = std::count_if(state.begin(), state.end(), [](const auto &pixel) { return pixel[0] > 0; });
    Require(provisional > 450 && statistics.m_confidenceRejected > 0, "Intermediate rejection discarded provisional geometry");
    state.assign(state.size(), empty);
    statistics = {};
    RefinePlaneSweepSlantedLevelCpu(level, setup, range, state, parents, parentWidth, parentHeight, true, statistics);
    Require(std::count_if(state.begin(), state.end(), [](const auto &pixel) { return pixel[0] > 0; }) < provisional,
        "Final level did not apply confidence rejection");
}

static void CheckConfig(bool gpu)
{
    const std::string folder = gpu ? "Testing/SlantedGpu" : "Testing/SlantedCpu";
    std::filesystem::create_directories(folder);
    const std::string file = folder + "/setup.ini";
    { std::ofstream out(file); out << "[COLMAP]\nPlaneSweepMatchingMode=PatchMatchSlanted\nPlaneSweepPatchMatchIterations=4\nPlaneSweepPatchMatchColorSigma=0.2\n"; }
    const auto parsed = LoadProgramSetup(file).m_colmap;
    Require(parsed.m_planeSweepMatchingMode == PlaneSweepMatchingMode::PatchMatchSlanted && parsed.m_planeSweepPatchMatchIterations == 4,
        "Slanted matching setup was not parsed");
    Require(ColmapSetup{}.m_planeSweepMatchingMode == PlaneSweepMatchingMode::PlaneSweep, "Legacy default changed");
    { std::ofstream out(file); out << "[COLMAP]\nPlaneSweepMatchingMode=Unknown\n"; }
    bool rejected = false;
    try { LoadProgramSetup(file); } catch (const std::invalid_argument &) { rejected = true; }
    Require(rejected, "Invalid matching mode silently changed algorithms");
    ColmapSetup unsupported;
    unsupported.m_planeSweepPatchCostMethod = PatchCostMethod::PatchCostMethodCensusTransform;
    rejected = false;
    try { ValidatePlaneSweepSlantedSetup(unsupported); } catch (const std::invalid_argument &) { rejected = true; }
    Require(rejected, "Unsupported weighted cost was silently accepted");
}

int main(int argc, char **argv)
{
    try
    {
        const bool useGpu = argc > 1 && std::string(argv[1]) == "--gpu";
        CheckConfig(useGpu);
        CheckPlaneInheritance();
        PlaneSweepGpuTestContext context;
        PlaneSweepGpu gpu;
        if (useGpu)
        {
            context.Initialize();
            gpu.Initialize(context.m_device, context.m_physicalDevice, context.m_queue, context.m_commandPool, context.m_shaderFloat64Enabled);
        }
        ColmapSetup setup;
        setup.m_planeSweepMatchingMode = PlaneSweepMatchingMode::PatchMatchSlanted;
        setup.m_planeSweepImageScale = 1;
        setup.m_planeSweepPyramidLevels = 2; setup.m_planeSweepPlaneCount = 24;
        setup.m_planeSweepLocalSamples = 9;
        setup.m_planeSweepPatchRadius = 3; setup.m_planeSweepMinValidSourceViews = 2;
        setup.m_planeSweepPatchMatchIterations = 3;
        setup.m_planeSweepPatchMatchColorSigma = 0.45f;
        setup.m_planeSweepConfidenceEnabled = setup.m_planeSweepTextureRejectionEnabled = false;
        setup.m_planeSweepSaveDepth = false;
        setup.m_planeSweepCostThreshold = 0.3f;
        SweepUtility::PlaneSweepDepthRange range; range.m_nearDepth = 1.2f; range.m_farDepth = 4.5f;
        for (int method : { 0, 1, 2 })
        {
            if (useGpu && method != 0 && !context.m_shaderFloat64Enabled) continue;
            setup.m_planeSweepPatchCostMethod = static_cast<PatchCostMethod>(method);
            for (bool edge : { false, true })
            {
                const auto reference = Render(0, edge, edge);
                std::vector<SweepUtility::PlaneSweepImage> sources{ Render(0.22f, edge, edge, 0.04f), Render(-0.22f, edge, edge, -0.03f) };
                uint64_t coarseSweepEvaluations = 0;
                std::vector<PlaneSweepPyramidStats> cpuStatistics;
                const auto cpu = RunPlaneSweepPyramidCpu(reference, sources, setup, range, &cpuStatistics);
                const float error = Error(cpu, reference, edge);
                std::cout << "Slanted method=" << method << " edge=" << edge << " median depth error=" << error << std::endl;
                Require(error < 0.07f, "Slanted reconstruction failed known geometry");
                if (!edge && method == 0)
                {
                    auto legacy = setup; legacy.m_planeSweepMatchingMode = PlaneSweepMatchingMode::PlaneSweep;
                    std::vector<PlaneSweepPyramidStats> baselineStatistics;
                    const auto baseline = RunPlaneSweepPyramidCpu(reference, sources, legacy, range, &baselineStatistics);
                    coarseSweepEvaluations = baselineStatistics.front().m_evaluations;
                    Require(cpuStatistics.size() == 2 && cpuStatistics.front().m_evaluations > coarseSweepEvaluations,
                        "Coarse pyramid level did not run slanted refinement");
                    const float baselineError = Error(baseline, reference, false);
                    std::cout << "Fronto-parallel median depth error=" << baselineError << std::endl;
                    Require(error < baselineError, "Tilted-plane refinement did not improve the sloping surface");
                    const auto repeated = RunPlaneSweepPyramidCpu(reference, sources, setup, range);
                    Require(cpu.m_depthValues == repeated.m_depthValues, "CPU slanted results are not reproducible");
                }
                if (useGpu)
                {
                    SweepUtility::PlaneSweepDepthMap actual;
                    std::string failure;
                    std::vector<PlaneSweepPyramidStats> gpuStatistics;
                    Require(gpu.RunCoarseToFineDepthMap(reference, sources, setup, range, actual, &gpuStatistics, &failure), failure.c_str());
                    Require(gpuStatistics.size() == cpuStatistics.size(), "CPU/GPU pyramid level counts differ");
                    if (coarseSweepEvaluations)
                        Require(gpuStatistics.front().m_evaluations > coarseSweepEvaluations, "GPU coarse pyramid level did not run slanted refinement");
                    const float gpuError = Error(actual, reference, edge);
                    Require(gpuError < 0.07f, "GPU slanted reconstruction failed known geometry");
                    size_t different = 0;
                    for (size_t i = 0; i < actual.m_depthValues.size(); ++i)
                        if (std::fabs(actual.m_depthValues[i] - cpu.m_depthValues[i]) > 2e-4f) ++different;
                    std::cout << "GPU median error=" << gpuError << ", CPU/GPU depth differences=" << different << std::endl;
                    Require(different < actual.m_depthValues.size() / 100, "CPU/GPU slanted plane refinement differs");
                }
            }
        }
        setup.m_planeSweepPatchCostMethod = PatchCostMethod::PatchCostMethodMeanAbsoluteDifference;
        auto reference = Render(0);
        auto transparent = Render(0.22f);
        for (size_t i = 3; i < transparent.m_texture.m_pixels.size(); i += 4) transparent.m_texture.m_pixels[i] = 0;
        const auto empty = RunPlaneSweepPyramidCpu(reference, { transparent, transparent }, setup, range);
        Require(std::all_of(empty.m_depthValues.begin(), empty.m_depthValues.end(), [](float z) { return z == 0; }), "Transparent source produced geometry");
        if (useGpu)
        {
            SweepUtility::PlaneSweepDepthMap actual;
            Require(gpu.RunCoarseToFineDepthMap(reference, { transparent, transparent }, setup, range, actual), "Transparent GPU dispatch failed");
            Require(std::all_of(actual.m_depthValues.begin(), actual.m_depthValues.end(), [](float z) { return z == 0; }), "Transparent GPU source produced geometry");
        }
        // Reference masks must survive initialization, propagation and final extraction.
        auto masked = reference;
        masked.m_hasMask = masked.m_maskUsesAlpha = true;
        masked.m_maskTexture = masked.m_texture;
        for (uint32_t y = 0; y < masked.m_texture.m_height; ++y)
            for (uint32_t x = 0; x < 15; ++x) masked.m_maskTexture.m_pixels[(size_t(y) * masked.m_texture.m_width + x) * 4 + 3] = 0;
        const std::vector<SweepUtility::PlaneSweepImage> validSources{ Render(0.22f), Render(-0.22f) };
        const auto maskedCpu = RunPlaneSweepPyramidCpu(masked, validSources, setup, range);
        SweepUtility::PlaneSweepDepthMap maskedGpu;
        if (useGpu) Require(gpu.RunCoarseToFineDepthMap(masked, validSources, setup, range, maskedGpu), "Masked GPU dispatch failed");
        for (uint32_t y = 0; y < maskedCpu.m_height; ++y)
            for (uint32_t x = 0; x < 15; ++x)
            {
                const size_t index = size_t(y) * maskedCpu.m_width + x;
                Require(maskedCpu.m_depthValues[index] == 0 && (!useGpu || maskedGpu.m_depthValues[index] == 0), "Plane propagation filled masked pixels");
            }
        // Textureless patches are rejected using the weighted support variance.
        auto flat = reference;
        for (size_t i = 0; i < flat.m_texture.m_pixels.size(); i += 4)
            flat.m_texture.m_pixels[i] = flat.m_texture.m_pixels[i + 1] = flat.m_texture.m_pixels[i + 2] = 128;
        setup.m_planeSweepTextureRejectionEnabled = true;
        const auto flatCpu = RunPlaneSweepPyramidCpu(flat, { flat, flat }, setup, range);
        Require(std::all_of(flatCpu.m_depthValues.begin(), flatCpu.m_depthValues.end(), [](float z) { return z == 0; }), "Weighted texture rejection failed");
        if (useGpu)
        {
            SweepUtility::PlaneSweepDepthMap actual;
            Require(gpu.RunCoarseToFineDepthMap(flat, { flat, flat }, setup, range, actual), "Flat GPU dispatch failed");
            Require(std::all_of(actual.m_depthValues.begin(), actual.m_depthValues.end(), [](float z) { return z == 0; }), "GPU weighted texture rejection failed");
        }
        // Exercise local-depth confidence on useful geometry and check a deliberately strict threshold.
        setup.m_planeSweepTextureRejectionEnabled = false;
        setup.m_planeSweepPatchCostMethod = PatchCostMethod::PatchCostMethodZeroMeanNormalizedCrossCorrelation;
        const auto flatZncc = RunPlaneSweepPyramidCpu(flat, { flat, flat }, setup, range);
        Require(std::all_of(flatZncc.m_depthValues.begin(), flatZncc.m_depthValues.end(), [](float z) { return z == 0; }), "Weighted ZNCC accepted a zero-variance patch");
        if (useGpu && context.m_shaderFloat64Enabled)
        {
            SweepUtility::PlaneSweepDepthMap actual;
            Require(gpu.RunCoarseToFineDepthMap(flat, { flat, flat }, setup, range, actual), "Flat ZNCC GPU dispatch failed");
            Require(std::all_of(actual.m_depthValues.begin(), actual.m_depthValues.end(), [](float z) { return z == 0; }), "GPU weighted ZNCC accepted a zero-variance patch");
        }
        setup.m_planeSweepPatchCostMethod = PatchCostMethod::PatchCostMethodMeanAbsoluteDifference;
        setup.m_planeSweepConfidenceEnabled = true;
        setup.m_planeSweepConfidenceMinimumDifference = 0.0001f;
        const auto confident = RunPlaneSweepPyramidCpu(reference, validSources, setup, range);
        Require(Error(confident, reference, false) < 0.07f, "Local-depth confidence removed the known surface");
        setup.m_planeSweepConfidenceMinimumDifference = 10;
        const auto ambiguous = RunPlaneSweepPyramidCpu(reference, validSources, setup, range);
        Require(std::count_if(ambiguous.m_depthValues.begin(), ambiguous.m_depthValues.end(), [](float z) { return z > 0; }) <
            std::count_if(confident.m_depthValues.begin(), confident.m_depthValues.end(), [](float z) { return z > 0; }), "Confidence threshold has no effect");
        std::cout << "Slanted mode, geometry, adaptive support and deterministic refinement tests passed.\n";
        return 0;
    }
    catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
