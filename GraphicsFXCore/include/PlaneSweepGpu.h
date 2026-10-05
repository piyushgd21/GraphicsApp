#pragma once

#include <vulkan/vulkan.h>
#include <vector>
#include <string>
#include <cstdint>

namespace graphicsFX
{
	struct TextureData;
	struct GpuCameraIntrinsics;
	struct GpuCameraPose;

	class PlaneSweepGpu
	{
	public:
		PlaneSweepGpu(void);
		~PlaneSweepGpu(void);

		void Initialize(VkDevice device, VkPhysicalDevice physicalDevice, VkQueue queue, VkCommandPool commandPool);
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

	private:
		// helpers
		std::vector<char> ReadShaderFile(const std::string &relativePath) const;

		bool m_initialized{ false };
		VkDevice m_device{ VK_NULL_HANDLE };
		VkPhysicalDevice m_physicalDevice{ VK_NULL_HANDLE };
		VkQueue m_queue{ VK_NULL_HANDLE };
		VkCommandPool m_commandPool{ VK_NULL_HANDLE };
		bool m_shaderSampledImageArrayDynamicIndexingEnabled{ false };

		VkShaderModule m_shaderModule{ VK_NULL_HANDLE };
		VkPipelineLayout m_pipelineLayout{ VK_NULL_HANDLE };
		VkPipeline m_computePipeline{ VK_NULL_HANDLE };
	};
}
