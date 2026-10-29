///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	PlaneSweepPatchCostTests.cpp
//	Description	:
//		Checks patch costs through synthetic plane sweep reconstruction.
//
//
///////////////////////////////////////////////////////////////////////

#include "MultiViewUtility.h"
#include "PlaneSweepGpu.h"
#include "PlaneSweepGpuTypes.h"
#include "PlaneSweepGpuTestContext.h"

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace graphicsFX;

using PlaneSweepUtility = MultiViewUtility::MultiViewPlaneSweepUtility;

//Builds a textured plane at depth 2 and checks public reconstruction and setup APIs.
class PlaneSweepPatchCostTests final
{

    public:

        //Runs a synthetic scene for each cost method and checks important invalid inputs.
        static void Run(const std::filesystem::path &folder, bool useHomographyWarping,
            PlaneSweepGpu *gpuPtr = nullptr);

    private:

        //Compares production GPU output against the CPU depth map.
        static void CompareGpu(
            PlaneSweepGpu &gpu,
            const ColmapSetup &setup,
            const PlaneSweepUtility::PlaneSweepImage &referenceImage,
            const std::vector<PlaneSweepUtility::PlaneSweepImage> &sourceImages);

        //Fails a test with a message even when assertions are disabled in Release builds.
        static void Require(bool condition, const std::string &message);

        //Creates a repeatable grayscale texture, shifted eight pixels in the neighbor image.
        static TextureData MakeImage(bool neighbor, int gain, int offset, bool flat, bool masked, bool inverted);

        //Returns intensities divisible by four so bilinear averages remain exact image bytes.
        static int PlaneIntensity(uint32_t x, uint32_t y);

        //Runs the actual plane sweep with only the central patch fully inside both images.
        static ColmapPlaneSweepResult Reconstruct(const std::filesystem::path &folder, PatchCostMethod method, int gain, int offset, bool flat, bool masked, bool inverted, float weight, int radius, float threshold);

        //Requires exactly one point at the center of the known plane at depth 2.
        static void RequireCorrectDepth(const ColmapPlaneSweepResult &result, const std::string &message);

        static bool s_useHomographyWarping; //Selects whether the current test pass uses homography warping.
        static PlaneSweepGpu *s_gpuPtr;
};

bool PlaneSweepPatchCostTests::s_useHomographyWarping = false;
PlaneSweepGpu *PlaneSweepPatchCostTests::s_gpuPtr = nullptr;

//Fails a test with a message even when assertions are disabled in Release builds.
void PlaneSweepPatchCostTests::Require(bool condition, const std::string &message)
{
    if (!condition) throw std::runtime_error(message);
}

//Returns intensities divisible by four so bilinear averages remain exact image bytes.
int PlaneSweepPatchCostTests::PlaneIntensity(uint32_t x, uint32_t y)
{
    const uint32_t hash = ((x + 3U) * 73856093U) ^ ((y + 5U) * 19349663U) ^ (x * y * 83492791U);
    return 16 + static_cast<int>((hash ^ (hash >> 13U)) % 12U) * 4;
}

//Creates a repeatable grayscale texture, shifted eight pixels in the neighbor image.
TextureData PlaneSweepPatchCostTests::MakeImage(bool neighbor, int gain, int offset, bool flat, bool masked, bool inverted)
{
    TextureData texture;
    texture.m_width = 33U;
    texture.m_height = 33U;
    texture.m_pixels.resize(33U * 33U * 4U, 255U);
    for (uint32_t y = 0U; y < 33U; ++y)
    {
        for (uint32_t x = 0U; x < 33U; ++x)
        {
            const uint32_t planeX = x + (neighbor ? 8U : 0U);
            //The existing unprojection adds half a pixel, so the neighbor is sampled halfway between four pixels.
            int intensity = neighbor ? PlaneIntensity(planeX, y) : (PlaneIntensity(x, y) + PlaneIntensity(x + 1U, y) + PlaneIntensity(x, y + 1U) + PlaneIntensity(x + 1U, y + 1U)) / 4;
            if (flat) intensity = 0;
            const int value = neighbor ? (inverted ? 96 - intensity : intensity * gain + offset) : intensity;
            const std::size_t index = (static_cast<std::size_t>(y) * 33U + x) * 4U;
            for (std::size_t channel = 0U; channel < 3U; ++channel) texture.m_pixels[index + channel] = static_cast<uint8_t>(value);
            if (neighbor && masked && !(x == 8U && y == 16U)) texture.m_pixels[index + 3U] = 0U;
        }
    }

    return texture;
}

//Compares production GPU output against the CPU depth map.
void PlaneSweepPatchCostTests::CompareGpu(
    PlaneSweepGpu &gpu,
    const ColmapSetup &setup,
    const PlaneSweepUtility::PlaneSweepImage &referenceImage,
    const std::vector<PlaneSweepUtility::PlaneSweepImage> &sourceImages)
{
    PlaneSweepUtility::PlaneSweepDepthRange depthRange;
    depthRange.m_nearDepth = setup.m_planeSweepNearDepth;
    depthRange.m_farDepth = setup.m_planeSweepFarDepth;

    PlaneSweepUtility::PlaneSweepDepthMap cpuDepthMap;
    std::vector<PlaneSweepUtility::PlaneSweepPointCandidate> candidates;
    ColmapPlaneSweepResult cpuCounters;
    PlaneSweepUtility::BuildReferencePoints(
        setup, referenceImage, sourceImages, depthRange,
        0U, cpuDepthMap, candidates, cpuCounters);

    PlaneSweepGpuParams params{};
    params.m_inverseNear = 1.0f / depthRange.m_nearDepth;
    params.m_inverseFar = 1.0f / depthRange.m_farDepth;
    params.m_planeCount = setup.m_planeSweepPlaneCount;
    params.m_patchRadius = setup.m_planeSweepPatchRadius;
    params.m_minValidSourceViews = setup.m_planeSweepMinValidSourceViews;
    params.m_alphaThreshold = setup.m_planeSweepAlphaThreshold;
    params.m_minimumOpaquePatchRatio = setup.m_planeSweepMinimumOpaquePatchRatio;
    params.m_costThreshold = setup.m_planeSweepCostThreshold;
    params.m_confidenceMinimumDifference =
        setup.m_planeSweepConfidenceMinimumDifference;
    params.m_textureMinimumStdDev = setup.m_planeSweepTextureMinimumStdDev;
    params.m_sourceCount = static_cast<uint32_t>(sourceImages.size());
    params.m_textureRejectionEnabled =
        setup.m_planeSweepTextureRejectionEnabled ? 1U : 0U;
    params.m_confidenceEnabled = setup.m_planeSweepConfidenceEnabled ? 1U : 0U;
    params.m_pointStep = static_cast<uint32_t>(setup.m_planeSweepPointStep);
    params.m_scaledWidth = cpuDepthMap.m_width;
    params.m_scaledHeight = cpuDepthMap.m_height;
    params.m_patchCostMethod =
        static_cast<uint32_t>(setup.m_planeSweepPatchCostMethod);
    params.m_adCensusWeight = setup.m_planeSweepADCensusWeight;

    PlaneSweepUtility::PlaneSweepDepthMap gpuDepthMap;
    std::vector<uint32_t> statuses;
    PlaneSweepRealComparisonStats stats;
    const bool succeeded = gpu.RunProductionDepthMap(
        referenceImage, sourceImages, setup, params, depthRange,
        cpuDepthMap, candidates, candidates.size(), cpuCounters,
        0U, 0.0, true, gpuDepthMap, statuses, &stats);
    Require(succeeded, "GPU dispatch failed: " + stats.m_failureReason);

    Require(gpuDepthMap.m_depthValues.size() == cpuDepthMap.m_depthValues.size(),
        "GPU depth-map size differs from CPU.");

    for (std::size_t index = 0; index < cpuDepthMap.m_depthValues.size(); ++index)
    {
        const float expected = cpuDepthMap.m_depthValues[index];
        const float actual = gpuDepthMap.m_depthValues[index];
        const bool matches = std::isfinite(actual) &&
            ((expected == 0.0f && actual == 0.0f) ||
             (expected > 0.0f && actual > 0.0f &&
              std::fabs(expected - actual) <= 0.00001f));
        Require(matches, "GPU depth mismatch at pixel " + std::to_string(index));
    }
}

//Runs the actual plane sweep with only the central patch fully inside both images.
ColmapPlaneSweepResult PlaneSweepPatchCostTests::Reconstruct(const std::filesystem::path &folder, PatchCostMethod method, int gain, int offset, bool flat, bool masked, bool inverted, float weight, int radius, float threshold)
{
    Require(SaveTexture((folder / "depth_image.png").string(), MakeImage(false, 1, 0, flat, false, false)), "Save depth reconstruction image");
    Require(SaveTexture((folder / "neighbor.png").string(), MakeImage(true, gain, offset, flat, masked, inverted)), "Save neighbor image");

    ColmapCameraReadResult cameras;
    ColmapCameraIntrinsicData intrinsic;
    intrinsic.m_cameraId = 1U;
    intrinsic.m_supportedModel = true;
    intrinsic.m_width = 33U;
    intrinsic.m_height = 33U;
    intrinsic.m_fx = 16.0;
    intrinsic.m_fy = 16.0;
    intrinsic.m_cx = 16.5;
    intrinsic.m_cy = 16.5;
    cameras.m_intrinsics.push_back(intrinsic);
    ColmapCameraData camera;
    camera.m_cameraId = 1U;
    camera.m_imageId = 1U;
    camera.m_name = "depth_image.png";
    cameras.m_cameras.push_back(camera);
    camera.m_imageId = 2U;
    camera.m_name = "neighbor.png";
    camera.m_position.x = 1.0f;
    cameras.m_cameras.push_back(camera);

    ColmapSetup setup;
    setup.m_enabled = true;
    setup.m_planeSweepUseHomographyWarping = s_useHomographyWarping;
    setup.m_cameraImageFolder = folder.string();
    setup.m_planeSweepSourceCount = 1;
    setup.m_planeSweepMinValidSourceViews = 1;
    setup.m_planeSweepPlaneCount = 2;
    setup.m_planeSweepImageScale = 1.0f;
    setup.m_planeSweepPointStep = 16;
    setup.m_planeSweepPatchRadius = radius;
    setup.m_planeSweepPatchCostMethod = method;
    setup.m_planeSweepADCensusWeight = weight;
    setup.m_planeSweepMinimumOpaquePatchRatio = 1.0f;
    setup.m_planeSweepCostThreshold = threshold;
    setup.m_planeSweepDepthRangeMode = "Manual";
    setup.m_planeSweepNearDepth = 2.0f;
    setup.m_planeSweepFarDepth = 4.0f;
    setup.m_planeSweepConfidenceEnabled = false;
    setup.m_planeSweepTextureRejectionEnabled = false;
    setup.m_planeSweepCrossViewConsistencyEnabled = false;
    setup.m_planeSweepPointFusionEnabled = false;
    setup.m_planeSweepOutlierFilterEnabled = false;
    setup.m_planeSweepSaveDepth = false;
    const bool hasGpuMethod =
        method == PatchCostMethod::PatchCostMethodMeanAbsoluteDifference ||
        method == PatchCostMethod::PatchCostMethodRankTransform ||
        method == PatchCostMethod::PatchCostMethodCensusTransform ||
        method == PatchCostMethod::PatchCostMethodADCensus ||
        (s_gpuPtr != nullptr && s_gpuPtr->ShaderFloat64Enabled() &&
         (method == PatchCostMethod::PatchCostMethodNormalizedCrossCorrelation ||
          method == PatchCostMethod::PatchCostMethodZeroMeanNormalizedCrossCorrelation));

    if (s_gpuPtr != nullptr && hasGpuMethod)
    {
        PlaneSweepUtility::PlaneSweepImage referenceImage;
        referenceImage.m_camera = cameras.m_cameras[0];
        referenceImage.m_intrinsic = cameras.m_intrinsics[0];
        referenceImage.m_texture = MakeImage(false, 1, 0, flat, false, false);

        PlaneSweepUtility::PlaneSweepImage sourceImage;
        sourceImage.m_camera = cameras.m_cameras[1];
        sourceImage.m_intrinsic = cameras.m_intrinsics[0];
        sourceImage.m_texture =
            MakeImage(true, gain, offset, flat, masked, inverted);
        sourceImage.m_cameraIndex = 1U;

        CompareGpu(*s_gpuPtr, setup, referenceImage, { sourceImage });
    }
    return MultiViewUtility::BuildColmapPlaneSweepPointCloud(cameras, setup);
}

//Requires exactly one point at the center of the known plane at depth 2.
void PlaneSweepPatchCostTests::RequireCorrectDepth(const ColmapPlaneSweepResult &result, const std::string &message)
{
    Require(result.m_points.size() == 1U, message + ": expected one central point; got " + std::to_string(result.m_points.size()) + ", processed " + std::to_string(result.m_referenceImagesProcessed));
    const Float3 &point = result.m_points.front().m_position;
    Require(std::fabs(point.x) < 0.0001f && std::fabs(point.y) < 0.0001f && std::fabs(point.z + 2.0f) < 0.0001f, message + ": incorrect world position");
}

//Runs a synthetic scene for each cost method and checks important invalid inputs.
void PlaneSweepPatchCostTests::Run(const std::filesystem::path &folder,
    bool useHomographyWarping, PlaneSweepGpu *gpuPtr)
{
    s_useHomographyWarping = useHomographyWarping;
    s_gpuPtr = gpuPtr;
    std::filesystem::create_directories(folder);
    const std::array<PatchCostMethod, 6U> methods{
        PatchCostMethod::PatchCostMethodMeanAbsoluteDifference,
        PatchCostMethod::PatchCostMethodNormalizedCrossCorrelation,
        PatchCostMethod::PatchCostMethodZeroMeanNormalizedCrossCorrelation,
        PatchCostMethod::PatchCostMethodRankTransform,
        PatchCostMethod::PatchCostMethodCensusTransform,
        PatchCostMethod::PatchCostMethodADCensus
    };
    const std::array<std::string, 6U> names{ "MeanAbsoluteDifference", "NormalizedCrossCorrelation", "ZeroMeanNormalizedCrossCorrelation", "RankTransform", "CensusTransform", "ADCensus" };
    const std::filesystem::path configPath = folder / "patch_cost_setup.ini";
    for (std::size_t index = 0U; index < methods.size(); ++index)
    {
        const PatchCostMethod method = methods[index];
        const std::string &name = names[index];
        RequireCorrectDepth(Reconstruct(folder, method, 1, 0, false, false, false, 0.5f, 2, 0.0001f), name + " identical patches");
        Require(Reconstruct(folder, method, 1, 0, false, true, false, 0.5f, 2, 0.0001f).m_points.empty(), name + " rejects transparent neighbor");

        for (const std::string &prefix : { std::string(), std::string("PatchCostMethod") })
        {
            std::ofstream config(configPath);
            config << "[COLMAP]\nPlaneSweepPatchCostMethod = " << prefix << name << "\nPlaneSweepADCensusWeight = 0.25\nPlaneSweepUseHomographyWarping = true\n";
            config.close();
            const ColmapSetup loaded = LoadProgramSetup(configPath.string()).m_colmap;
            Require(loaded.m_planeSweepPatchCostMethod == method && loaded.m_planeSweepADCensusWeight == 0.25f, name + " parses setup");
            Require(loaded.m_planeSweepUseHomographyWarping, "Parses homography warping flag");
        }

        if (index >= 1U && index <= 4U) RequireCorrectDepth(Reconstruct(folder, method, 2, 0, false, false, false, 0.5f, 2, 0.0001f), name + " gain invariance");
        if (index >= 2U && index <= 4U) RequireCorrectDepth(Reconstruct(folder, method, 2, 16, false, false, false, 0.5f, 2, 0.0001f), name + " gain and offset invariance");
        if (index >= 1U && index <= 2U) Require(Reconstruct(folder, method, 1, 0, true, false, false, 0.5f, 2, 0.0001f).m_points.empty(), name + " rejects undefined black correlation");
        if (index >= 2U && index <= 4U) Require(Reconstruct(folder, method, 1, 0, false, false, false, 0.5f, 0, 0.0001f).m_points.empty(), name + " rejects a single-pixel descriptor");
        if (index >= 2U && index <= 4U) Require(Reconstruct(folder, method, 1, 0, false, false, true, 0.5f, 2, 0.0001f).m_points.empty(), name + " rejects inverted intensities");
        std::cout << "Passed " << name << '\n';
    }

    const PatchCostMethod absolute = methods[0U];
    const PatchCostMethod combined = methods[5U];
    RequireCorrectDepth(Reconstruct(folder, methods[4U], 1, 0, false, false, false, 0.5f, 5, 0.0001f), "Census supports patches with more than 64 comparisons");
    Require(Reconstruct(folder, methods[2U], 1, 32, true, false, false, 0.5f, 2, 0.0001f).m_points.empty(), "ZNCC rejects a flat nonzero neighbor patch");
    RequireCorrectDepth(Reconstruct(folder, absolute, 1, 32, true, false, false, 0.5f, 2, 0.126f), "RGB absolute cost below threshold");
    Require(Reconstruct(folder, absolute, 1, 32, true, false, false, 0.5f, 2, 0.125f).m_points.empty(), "RGB absolute cost above threshold");
    RequireCorrectDepth(Reconstruct(folder, combined, 1, 32, true, false, false, 0.5f, 2, 0.063f), "AD-Census weighted cost below threshold");
    Require(Reconstruct(folder, combined, 1, 32, true, false, false, 0.5f, 2, 0.062f).m_points.empty(), "AD-Census weighted cost above threshold");
    RequireCorrectDepth(Reconstruct(folder, combined, 1, 32, false, false, false, 1.0f, 2, 0.0001f), "AD-Census weight 1 uses Census");
    Require(Reconstruct(folder, combined, 1, 32, false, false, false, 0.0f, 2, 0.0001f).m_points.empty(), "AD-Census weight 0 uses absolute difference");
    Require(!Reconstruct(folder, combined, 1, 0, false, false, false, 0.0f, 0, 0.0001f).m_points.empty(), "AD-Census weight 0 supports a single pixel");
    Require(Reconstruct(folder, combined, 1, 0, false, false, false, 1.0f, 0, 0.0001f).m_points.empty(), "AD-Census weight 1 needs Census comparisons");

    std::ofstream config(configPath);
    config << "[COLMAP]\nPlaneSweepPatchCostMethod = Unknown\nPlaneSweepADCensusWeight = 2\n";
    config.close();
    const ColmapSetup loaded = LoadProgramSetup(configPath.string()).m_colmap;
    Require(loaded.m_planeSweepPatchCostMethod == absolute && loaded.m_planeSweepADCensusWeight == 0.5f, "Invalid configuration preserves defaults");
    Require(!loaded.m_planeSweepUseHomographyWarping, "Homography warping defaults to false");
}

//Runs the regression tests without opening the graphics application.
int main(int argc, char **argv)
{
    try
    {
        if (argc < 2 || argc > 3 ||
            (argc == 3 && std::string(argv[2]) != "--gpu"))
        {
            throw std::runtime_error(
                "Usage: PlaneSweepPatchCostTests <output-directory> [--gpu]");
        }

        const std::filesystem::path folder = std::filesystem::absolute(argv[1]);
        if (argc == 3)
        {
            PlaneSweepGpuTestContext context;
            context.Initialize();

            PlaneSweepGpu gpu;
            gpu.Initialize(context.m_device, context.m_physicalDevice,
                context.m_queue, context.m_commandPool,
                context.m_shaderFloat64Enabled);

            VkPhysicalDeviceProperties properties{};
            vkGetPhysicalDeviceProperties(context.m_physicalDevice, &properties);
            std::cout << "GPU comparisons using: " << properties.deviceName << '\n';
            std::cout << "GPU shaderFloat64 enabled: "
                << (gpu.ShaderFloat64Enabled() ? "yes" : "no") << '\n';

            PlaneSweepPatchCostTests::Run(folder, false, &gpu);
            if (gpu.ShaderFloat64Enabled())
            {
                std::cout << "GPU comparisons passed for all six patch cost methods.\n";
            }
            else
            {
                std::cout << "GPU comparisons passed for MAD, Rank, Census, and AD-Census; "
                             "NCC/ZNCC were tested on CPU only.\n";
            }
        }
        else
        {
            PlaneSweepPatchCostTests::Run(folder, false);
        }
        std::cout << "All plane sweep patch cost tests passed.\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
