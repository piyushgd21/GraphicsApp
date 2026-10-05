#pragma once

#include <vulkan/vulkan.h>
#include <vector>
#include <string>

namespace graphicsFX
{
	class PlaneSweepGpu
	{
	public:
		PlaneSweepGpu(void);
		~PlaneSweepGpu(void);

		void Initialize(VkDevice device, VkPhysicalDevice physicalDevice, uint32_t queueFamilyIndex, VkQueue queue, VkCommandPool commandPool);
		void RunComputeSmokeTest(void);
		bool RunStorageBufferSmokeTest(void);
		bool RunCameraLayoutSmokeTest(void);
		bool RunCameraConversionSmokeTest(void);
		bool RunImageSamplingSmokeTest(void);
		void Clean(void);

	private:
		// helpers
		std::vector<char> ReadShaderFile(const std::string &relativePath) const;

		bool m_initialized{ false };
		VkDevice m_device{ VK_NULL_HANDLE };
		VkPhysicalDevice m_physicalDevice{ VK_NULL_HANDLE };
		uint32_t m_queueFamilyIndex{ 0 };
		VkQueue m_queue{ VK_NULL_HANDLE };
		VkCommandPool m_commandPool{ VK_NULL_HANDLE };

		VkShaderModule m_shaderModule{ VK_NULL_HANDLE };
		VkPipelineLayout m_pipelineLayout{ VK_NULL_HANDLE };
		VkPipeline m_computePipeline{ VK_NULL_HANDLE };
	};
}
