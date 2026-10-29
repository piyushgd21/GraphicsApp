#pragma once

#include <vulkan/vulkan.h>
#include <vector>
#include <string>
#include <cstdint>
#include <array>

#include "MultiViewUtility.h"

namespace graphicsFX
{
	struct PlaneSweepMismatchExample
	{
		uint32_t m_referenceId{ 0 };
		std::string m_filename;
		std::size_t m_reconstructionIndex{ 0 };
		std::size_t m_depthIndex{ 0 };
		uint32_t m_scaledX{ 0 };
		uint32_t m_scaledY{ 0 };
		uint32_t m_referenceX{ 0 };
		uint32_t m_referenceY{ 0 };
		float m_cpuDepth{ 0.0f };
		float m_gpuDepth{ 0.0f };
		uint32_t m_cpuPlane{ UINT32_MAX };
		uint32_t m_gpuPlane{ UINT32_MAX };
		bool m_cpuValid{ false };
		bool m_gpuValid{ false };
		uint32_t m_gpuStatus{ UINT32_MAX };
	};

	struct PlaneSweepRealComparisonStats
	{
		uint32_t m_referenceId{ 0 };
		std::string m_filename;
		std::size_t m_reconstructionIndex{ 0 };
		std::size_t m_totalDifferences{ 0 };
		std::size_t m_totalDepthEntries{ 0 };
		std::size_t m_meaningfulSamples{ 0 };
		std::size_t m_cpuAccepted{ 0 };
		std::size_t m_gpuAccepted{ 0 };
		std::size_t m_bothInvalid{ 0 };
		std::size_t m_cpuOnlyValid{ 0 };
		std::size_t m_gpuOnlyValid{ 0 };
		std::size_t m_samePlanePositive{ 0 };
		std::size_t m_differentPlane{ 0 };
		std::size_t m_unsampledEntries{ 0 };
		std::size_t m_unsampledZeroMatches{ 0 };
		std::size_t m_gpuNonFinite{ 0 };
		std::size_t m_classificationMismatches{ 0 };
		double m_maximumAbsoluteError{ 0.0 };
		double m_maximumRelativeError{ 0.0 };
		double m_meanAbsoluteError{ 0.0 };
		double m_uploadSetupMilliseconds{ 0.0 };
		double m_dispatchMilliseconds{ 0.0 };
		double m_readbackComparisonMilliseconds{ 0.0 };
		double m_cpuBuildMilliseconds{ 0.0 };
		VkResult m_failureResult{ VK_SUCCESS };
		std::string m_failureReason;
		bool m_succeeded{ false };
	};

	struct TextureData;
	struct GpuCameraIntrinsics;
	struct GpuCameraPose;
	struct PlaneSweepGpuParams;

	class PlaneSweepGpu
	{
	public:
		PlaneSweepGpu(void);
		~PlaneSweepGpu(void);

		void Initialize(VkDevice device, VkPhysicalDevice physicalDevice, VkQueue queue,
			VkCommandPool commandPool, bool shaderFloat64Enabled = false);
		void RunComputeSmokeTest(void);
		bool RunStorageBufferSmokeTest(void);
		bool RunCameraLayoutSmokeTest(void);
		bool RunCameraConversionSmokeTest(void);
		bool RunImageSamplingSmokeTest(void);
		bool RunBilinearSamplingSmokeTest(void);
		bool RunPlaneSweepPixelSamplingSmokeTest(void);
		bool RunMultiSourceDescriptorSmokeTest(void);
		bool RunUnprojectPixelSmokeTest(
			const std::vector<GpuCameraIntrinsics> &intrinsics,
			const std::vector<GpuCameraPose> &poses,
			const std::vector<uint32_t> &cameraIndices,
			const std::vector<float> &sampleValues,
			const std::vector<float> &expectedPositions
		);
		bool RunProjectPointSmokeTest(
			const std::vector<GpuCameraIntrinsics> &intrinsics,
			const std::vector<GpuCameraPose> &poses,
			const std::vector<uint32_t> &cameraIndices,
			const std::vector<float> &worldPoints,
			const std::vector<uint32_t> &expectedValidity,
			const std::vector<float> &expectedProjections
		);
		bool RunSingleSourceCostSmokeTest(
			const std::vector<uint8_t> &referencePixels,
			const std::vector<uint8_t> &sourcePixels,
			const std::vector<GpuCameraIntrinsics> &intrinsics,
			const std::vector<GpuCameraPose> &poses,
			const std::vector<float> &caseValues,
			const std::vector<uint32_t> &patchRadii,
			const std::vector<uint32_t> &expectedSampleCounts,
			const std::vector<uint32_t> &expectedMinimumSamples,
			const std::vector<uint32_t> &expectedValidity,
			const std::vector<float> &expectedCosts
		);
		bool RunDepthPlaneSearchSmokeTest(
			const std::vector<uint8_t> &referencePixels,
			const std::vector<std::vector<uint8_t>> &scenarioSourcePixels,
			const std::vector<GpuCameraIntrinsics> &intrinsics,
			const std::vector<GpuCameraPose> &poses,
			const std::vector<float> &scenarioFloats,
			const std::vector<uint32_t> &scenarioParameters,
			const std::vector<float> &expectedPlaneDepths,
			const std::vector<uint32_t> &expectedPlaneValidity,
			const std::vector<float> &expectedPlaneCosts,
			const std::vector<uint32_t> &expectedStatuses,
			const std::vector<float> &expectedFinalDepths,
			const std::vector<float> &expectedBestDepths,
			const std::vector<float> &expectedBestCosts,
			const std::vector<float> &expectedSecondBestCosts,
			const std::vector<int32_t> &expectedBestPlanes
		);
		bool RunDepthMapSmokeTest(
			const std::vector<uint8_t> &referencePixels,
			const std::vector<uint8_t> &sourcePixels,
			const std::vector<GpuCameraIntrinsics> &intrinsics,
			const std::vector<GpuCameraPose> &poses,
			const PlaneSweepGpuParams &params,
			const std::vector<uint32_t> &sourceMetadata,
			const std::vector<float> &expectedDepthMap,
			const std::vector<uint32_t> &expectedSampleMappings,
			const std::vector<uint32_t> &expectedSampleStatuses,
			const std::vector<float> &expectedTextureStdDevs = {},
			const std::vector<uint32_t> &expectedTextureSampleCounts = {},
			uint32_t expectedTextureRejectedCount = 0,
			uint32_t expectedAcceptedCount = UINT32_MAX,
			uint32_t expectedCandidateCount = UINT32_MAX,
			float *maximumTextureStdDevError = nullptr,
			float *maximumPositiveDepthError = nullptr
		);
		bool RunProductionDepthMap(
			const MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepImage &referenceImage,
			const std::vector<MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepImage> &sourceImages,
			const ColmapSetup &setup,
			const PlaneSweepGpuParams &params,
			const MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepDepthRange &depthRange,
			const MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepDepthMap &cpuDepthMap,
			const std::vector<MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepPointCandidate> &cpuPointCandidates,
			std::size_t cpuPointCandidateCount,
			const ColmapPlaneSweepResult &cpuCounters,
			std::size_t referenceReconstructionIndex,
			double cpuBuildMilliseconds,
			bool compareCpuResult,
			MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepDepthMap &gpuDepthMap,
			std::vector<uint32_t> &gpuSampleStatuses,
			PlaneSweepRealComparisonStats *stats = nullptr,
			uint32_t bucketIndex = 0
		);
		bool RunTextureStdDevSmokeTest(
			const std::vector<std::vector<uint8_t>> &casePixels,
			const std::vector<float> &caseParameters,
			const std::vector<uint32_t> &expectedValidity,
			const std::vector<float> &expectedStdDevs,
			const std::vector<uint32_t> &expectedSampleCounts,
			float *maximumStdDevError = nullptr
		);
		bool RunMultiSourceCostSmokeTest(
			const std::vector<uint8_t> &referencePixels,
			const std::vector<uint8_t> &source0Pixels,
			const std::vector<uint8_t> &source1Pixels,
			const std::vector<uint8_t> &source2Pixels,
			const std::vector<uint8_t> &source2MaskPixels,
			const std::vector<uint8_t> &source3Pixels,
			const std::vector<GpuCameraIntrinsics> &intrinsics,
			const std::vector<GpuCameraPose> &poses,
			const std::vector<float> &caseValues,
			const std::vector<uint32_t> &caseParameters,
			const std::vector<uint32_t> &sourceMetadata,
			const std::vector<uint32_t> &expectedSampleCounts,
			const std::vector<uint32_t> &expectedValidViews,
			const std::vector<uint32_t> &expectedMinimumSamples,
			const std::vector<uint32_t> &expectedValidity,
			const std::vector<float> &expectedCosts
		);
		bool RunRealImageSamplingSmokeTest(
			const TextureData &texture,
			const TextureData *maskTexturePtr,
			bool hasMask,
			bool maskUsesAlpha,
			const std::vector<float> &xyPairs,
			std::vector<uint32_t> &gpuValidity,
			std::vector<float> &gpuRgba,
			uint32_t cameraIndex
		);
		void Clean(void);
		bool ShaderSampledImageArrayDynamicIndexingEnabled(void) const;
		bool ShaderFloat64Enabled(void) const { return m_shaderFloat64Enabled; }

	private:
		// helpers
		std::vector<char> ReadShaderFile(const std::string &relativePath) const;

		bool m_initialized{ false };
		VkDevice m_device{ VK_NULL_HANDLE };
		VkPhysicalDevice m_physicalDevice{ VK_NULL_HANDLE };
		VkQueue m_queue{ VK_NULL_HANDLE };
		VkCommandPool m_commandPool{ VK_NULL_HANDLE };
		bool m_shaderSampledImageArrayDynamicIndexingEnabled{ false };
		bool m_shaderFloat64Enabled{ false };

		VkShaderModule m_shaderModule{ VK_NULL_HANDLE };
		VkPipelineLayout m_pipelineLayout{ VK_NULL_HANDLE };
		VkPipeline m_computePipeline{ VK_NULL_HANDLE };
	};
}
