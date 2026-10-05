#include "PlaneSweepGpu.h"

#include "GraphicsUtility.h"
#include "UIUtility.h"
#include "PathUtility.h"
#include "PlaneSweepGpuTypes.h"

#include <Windows.h>
#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

#include <stdexcept>
#include <cmath>

namespace graphicsFX
{

PlaneSweepGpu::PlaneSweepGpu(void)
{
}

bool PlaneSweepGpu::RunCameraConversionSmokeTest(void)
{
	if (!m_initialized)
	{
		throw std::runtime_error("PlaneSweepGpu not initialized.");
	}

	// Use 3 synthetic Colmap camera records (intrinsics + pose)
	constexpr uint32_t kCount = 3u;

	// Build synthetic Colmap CPU-side source data
	std::vector<ColmapCameraIntrinsicData> cpuIntrinsics(kCount);
	std::vector<ColmapCameraData> cpuCameras(kCount);

	// Record 0
	cpuIntrinsics[0].m_width = 641; cpuIntrinsics[0].m_height = 479;
	cpuIntrinsics[0].m_fx = 501.25; cpuIntrinsics[0].m_fy = 502.50;
	cpuIntrinsics[0].m_cx = 319.75; cpuIntrinsics[0].m_cy = 240.125;
	cpuIntrinsics[0].m_k1 = 0.011; cpuIntrinsics[0].m_k2 = -0.022; cpuIntrinsics[0].m_p1 = 0.0035; cpuIntrinsics[0].m_p2 = -0.0045;

	cpuCameras[0].m_position = Float3{ 1.25f, -2.5f, 3.75f };
	cpuCameras[0].m_forward = Float3{ 0.11f, 0.22f, 0.93f };
	cpuCameras[0].m_up = Float3{ -0.17f, 0.98f, 0.07f };
	cpuCameras[0].m_right = Float3{ 0.97f, 0.19f, -0.13f };

	// Record 1
	cpuIntrinsics[1].m_width = 1283; cpuIntrinsics[1].m_height = 721;
	cpuIntrinsics[1].m_fx = 812.75; cpuIntrinsics[1].m_fy = 810.125;
	cpuIntrinsics[1].m_cx = 640.5; cpuIntrinsics[1].m_cy = 360.25;
	cpuIntrinsics[1].m_k1 = -0.013; cpuIntrinsics[1].m_k2 = 0.027; cpuIntrinsics[1].m_p1 = -0.0055; cpuIntrinsics[1].m_p2 = 0.0065;

	cpuCameras[1].m_position = Float3{ -10.0f, 5.5f, 0.125f };
	cpuCameras[1].m_forward = Float3{ -0.3f, 0.4f, 0.866f };
	cpuCameras[1].m_up = Float3{ 0.0f, 1.0f, 0.0f };
	cpuCameras[1].m_right = Float3{ 0.953f, -0.289f, 0.089f };

	// Record 2
	cpuIntrinsics[2].m_width = 1920; cpuIntrinsics[2].m_height = 1080;
	cpuIntrinsics[2].m_fx = 1000.0; cpuIntrinsics[2].m_fy = 999.5;
	cpuIntrinsics[2].m_cx = 960.0; cpuIntrinsics[2].m_cy = 540.0;
	cpuIntrinsics[2].m_k1 = 0.0; cpuIntrinsics[2].m_k2 = 0.0; cpuIntrinsics[2].m_p1 = 0.0; cpuIntrinsics[2].m_p2 = 0.0;

	cpuCameras[2].m_position = Float3{ 100.0f, -100.0f, 50.5f };
	cpuCameras[2].m_forward = Float3{ 0.0f, 0.0f, 1.0f };
	cpuCameras[2].m_up = Float3{ 0.0f, 1.0f, 0.0f };
	cpuCameras[2].m_right = Float3{ 1.0f, 0.0f, 0.0f };

	// Convert on CPU using existing helpers
	const std::vector<GpuCameraIntrinsics> gpuIntrinsics = MakeGpuCameraIntrinsicsArray(cpuIntrinsics);
	const std::vector<GpuCameraPose> gpuPoses = MakeGpuCameraPoseArray(cpuCameras);

	// CPU-side verification: compare converted GPU PODs against source
	for (uint32_t i = 0; i < kCount; ++i)
	{
		const auto &srcI = cpuIntrinsics[i];
		const auto &dstI = gpuIntrinsics[i];
		if (dstI.m_width != static_cast<int32_t>(srcI.m_width)) { UIUtility::Print("Conversion mismatch: intrinsics width\n"); return false; }
		if (dstI.m_height != static_cast<int32_t>(srcI.m_height)) { UIUtility::Print("Conversion mismatch: intrinsics height\n"); return false; }
		if (fabs(dstI.m_fx - static_cast<float>(srcI.m_fx)) > 1e-6f) { UIUtility::Print("Conversion mismatch: intrinsics fx\n"); return false; }
		if (fabs(dstI.m_fy - static_cast<float>(srcI.m_fy)) > 1e-6f) { UIUtility::Print("Conversion mismatch: intrinsics fy\n"); return false; }
		if (fabs(dstI.m_cx - static_cast<float>(srcI.m_cx)) > 1e-6f) { UIUtility::Print("Conversion mismatch: intrinsics cx\n"); return false; }
		if (fabs(dstI.m_cy - static_cast<float>(srcI.m_cy)) > 1e-6f) { UIUtility::Print("Conversion mismatch: intrinsics cy\n"); return false; }
		if (fabs(dstI.m_k1 - static_cast<float>(srcI.m_k1)) > 1e-6f) { UIUtility::Print("Conversion mismatch: intrinsics k1\n"); return false; }
		if (fabs(dstI.m_k2 - static_cast<float>(srcI.m_k2)) > 1e-6f) { UIUtility::Print("Conversion mismatch: intrinsics k2\n"); return false; }
		if (fabs(dstI.m_p1 - static_cast<float>(srcI.m_p1)) > 1e-6f) { UIUtility::Print("Conversion mismatch: intrinsics p1\n"); return false; }
		if (fabs(dstI.m_p2 - static_cast<float>(srcI.m_p2)) > 1e-6f) { UIUtility::Print("Conversion mismatch: intrinsics p2\n"); return false; }
	}

	for (uint32_t i = 0; i < kCount; ++i)
	{
		const auto &srcP = cpuCameras[i];
		const auto &dstP = gpuPoses[i];
		for (int m = 0; m < 3; ++m)
		{
			const float srcValPos = (&srcP.m_position.x)[m];
			if (fabs(dstP.m_position[m] - srcValPos) > 1e-6f) { UIUtility::Print("Conversion mismatch: pose position\n"); return false; }
			const float srcValFwd = (&srcP.m_forward.x)[m];
			if (fabs(dstP.m_forward[m] - srcValFwd) > 1e-6f) { UIUtility::Print("Conversion mismatch: pose forward\n"); return false; }
			const float srcValUp = (&srcP.m_up.x)[m];
			if (fabs(dstP.m_up[m] - srcValUp) > 1e-6f) { UIUtility::Print("Conversion mismatch: pose up\n"); return false; }
			const float srcValRight = (&srcP.m_right.x)[m];
			if (fabs(dstP.m_right[m] - srcValRight) > 1e-6f) { UIUtility::Print("Conversion mismatch: pose right\n"); return false; }
		}
		// check w components == 0
		if (fabs(dstP.m_position[3] - 0.0f) > 1e-6f) { UIUtility::Print("Conversion mismatch: pose position.w\n"); return false; }
		if (fabs(dstP.m_forward[3] - 0.0f) > 1e-6f) { UIUtility::Print("Conversion mismatch: pose forward.w\n"); return false; }
		if (fabs(dstP.m_up[3] - 0.0f) > 1e-6f) { UIUtility::Print("Conversion mismatch: pose up.w\n"); return false; }
		if (fabs(dstP.m_right[3] - 0.0f) > 1e-6f) { UIUtility::Print("Conversion mismatch: pose right.w\n"); return false; }
	}

	// Reuse existing GPU round-trip by uploading gpuIntrinsics and gpuPoses to the same pipeline created in RunCameraLayoutSmokeTest.
	// We'll serialize the converted POD arrays into the staging/device buffers exactly like the layout test and dispatch the same shader.

	// For simplicity, call RunCameraLayoutSmokeTest's internal flow by reusing the shader and code paths; but we must upload the already-converted PODs.
	// Implement a minimal inlined copy of the proven path that uploads deviceIntrinsics/devicePoses from these converted vectors.

	// The upload / dispatch / readback code is highly similar to RunCameraLayoutSmokeTest. To avoid duplicating too much logic, we'll temporarily
	// reuse the same buffer creation and command sequence as the layout test but replace the staging contents with gpuIntrinsics/gpuPoses.

	// Create staging buffers and populate from gpuIntrinsics/gpuPoses
	const VkDeviceSize intrinsicsSize = sizeof(GpuCameraIntrinsics) * kCount;
	const VkDeviceSize posesSize = sizeof(GpuCameraPose) * kCount;

	auto createHostBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkBuffer &buffer, VkDeviceMemory &memory)
	{
		VkBufferCreateInfo bufInfo{};
		bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bufInfo.size = size;
		bufInfo.usage = usage;
		bufInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

		if (vkCreateBuffer(m_device, &bufInfo, nullptr, &buffer) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to create staging/readback buffer.");
		}

		VkMemoryRequirements memReq{};
		vkGetBufferMemoryRequirements(m_device, buffer, &memReq);

		VkPhysicalDeviceMemoryProperties memProps{};
		vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memProps);

		uint32_t memType = UINT32_MAX;
		for (uint32_t i = 0; i < memProps.memoryTypeCount; ++i)
		{
			if ((memReq.memoryTypeBits & (1u << i)) != 0 && (memProps.memoryTypes[i].propertyFlags & (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)) == (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT))
			{
				memType = i; break;
			}
		}

		if (memType == UINT32_MAX)
		{
			vkDestroyBuffer(m_device, buffer, nullptr);
			throw std::runtime_error("No suitable memory type for staging buffer.");
		}

		VkMemoryAllocateInfo alloc{}; alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; alloc.allocationSize = memReq.size; alloc.memoryTypeIndex = memType;
		if (vkAllocateMemory(m_device, &alloc, nullptr, &memory) != VK_SUCCESS) { vkDestroyBuffer(m_device, buffer, nullptr); throw std::runtime_error("Failed to allocate staging memory."); }
		vkBindBufferMemory(m_device, buffer, memory, 0);
	};

	VkBuffer stagingIntrinsics = VK_NULL_HANDLE; VkDeviceMemory stagingIntrinsicsMem = VK_NULL_HANDLE;
	VkBuffer stagingPoses = VK_NULL_HANDLE; VkDeviceMemory stagingPosesMem = VK_NULL_HANDLE;
	createHostBuffer(intrinsicsSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, stagingIntrinsics, stagingIntrinsicsMem);
	createHostBuffer(posesSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, stagingPoses, stagingPosesMem);

	// copy converted PODs into staging
	void *mapPtr = nullptr;
	vkMapMemory(m_device, stagingIntrinsicsMem, 0, intrinsicsSize, 0, &mapPtr);
	memcpy(mapPtr, gpuIntrinsics.data(), static_cast<size_t>(intrinsicsSize));
	vkUnmapMemory(m_device, stagingIntrinsicsMem);

	vkMapMemory(m_device, stagingPosesMem, 0, posesSize, 0, &mapPtr);
	memcpy(mapPtr, gpuPoses.data(), static_cast<size_t>(posesSize));
	vkUnmapMemory(m_device, stagingPosesMem);

	// Create device-local input and output, readback buffers (reuse the same usage sets as layout test)
	auto createDeviceLocalBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkBuffer &buffer, VkDeviceMemory &memory)
	{
		VkBufferCreateInfo bufInfo{}; bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO; bufInfo.size = size; bufInfo.usage = usage; bufInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
		if (vkCreateBuffer(m_device, &bufInfo, nullptr, &buffer) != VK_SUCCESS) { throw std::runtime_error("Failed to create device-local buffer."); }
		VkMemoryRequirements memReq{}; vkGetBufferMemoryRequirements(m_device, buffer, &memReq);
		VkPhysicalDeviceMemoryProperties memProps{}; vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memProps);
		uint32_t memType = UINT32_MAX; for (uint32_t i = 0; i < memProps.memoryTypeCount; ++i) { if ((memReq.memoryTypeBits & (1u << i)) != 0 && (memProps.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) == VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) { memType = i; break; } }
		if (memType == UINT32_MAX) { vkDestroyBuffer(m_device, buffer, nullptr); throw std::runtime_error("No suitable device-local memory type."); }
		VkMemoryAllocateInfo alloc{}; alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; alloc.allocationSize = memReq.size; alloc.memoryTypeIndex = memType;
		if (vkAllocateMemory(m_device, &alloc, nullptr, &memory) != VK_SUCCESS) { vkDestroyBuffer(m_device, buffer, nullptr); throw std::runtime_error("Failed to allocate device-local memory."); }
		vkBindBufferMemory(m_device, buffer, memory, 0);
	};

	VkBuffer deviceIntrinsics = VK_NULL_HANDLE; VkDeviceMemory deviceIntrinsicsMem = VK_NULL_HANDLE;
	VkBuffer devicePoses = VK_NULL_HANDLE; VkDeviceMemory devicePosesMem = VK_NULL_HANDLE;
	createDeviceLocalBuffer(intrinsicsSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, deviceIntrinsics, deviceIntrinsicsMem);
	createDeviceLocalBuffer(posesSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, devicePoses, devicePosesMem);

	VkBuffer deviceIntrinsicsOut = VK_NULL_HANDLE; VkDeviceMemory deviceIntrinsicsOutMem = VK_NULL_HANDLE;
	VkBuffer devicePosesOut = VK_NULL_HANDLE; VkDeviceMemory devicePosesOutMem = VK_NULL_HANDLE;
	createDeviceLocalBuffer(intrinsicsSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, deviceIntrinsicsOut, deviceIntrinsicsOutMem);
	createDeviceLocalBuffer(posesSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, devicePosesOut, devicePosesOutMem);

	VkBuffer readIntrinsics = VK_NULL_HANDLE; VkDeviceMemory readIntrinsicsMem = VK_NULL_HANDLE;
	VkBuffer readPoses = VK_NULL_HANDLE; VkDeviceMemory readPosesMem = VK_NULL_HANDLE;
	createHostBuffer(intrinsicsSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT, readIntrinsics, readIntrinsicsMem);
	createHostBuffer(posesSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT, readPoses, readPosesMem);

	// Build descriptor set layout, descriptor sets, pipeline like the layout test
	// (reuse shader file: Shaders/planeSweepCameraLayoutSmokeTest.comp.spv)
	const std::string shaderRelativePath = "shaders/planeSweepCameraLayoutSmokeTest.comp.spv";
	std::vector<char> code = ReadShaderFile(shaderRelativePath);
	if (code.empty()) { UIUtility::Print("Compute shader SPIR-V not found for conversion test\n"); throw std::runtime_error("Failed to load compute shader SPIR-V"); }

	VkShaderModuleCreateInfo shaderCreate{}; shaderCreate.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO; shaderCreate.codeSize = code.size(); shaderCreate.pCode = reinterpret_cast<const uint32_t *>(code.data());
	VkShaderModule shaderModule = VK_NULL_HANDLE;
	if (vkCreateShaderModule(m_device, &shaderCreate, nullptr, &shaderModule) != VK_SUCCESS) { throw std::runtime_error("Failed to create shader module for conversion test."); }

	// Descriptor layout bindings 0..3
	std::array<VkDescriptorSetLayoutBinding, 4> bindings{};
	for (uint32_t i = 0; i < 4; ++i) { bindings[i].binding = i; bindings[i].descriptorCount = 1; bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT; bindings[i].pImmutableSamplers = nullptr; }

	VkDescriptorSetLayoutCreateInfo layoutCreate{}; layoutCreate.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO; layoutCreate.bindingCount = static_cast<uint32_t>(bindings.size()); layoutCreate.pBindings = bindings.data();
	VkDescriptorSetLayout descriptorSetLayout = VK_NULL_HANDLE; if (vkCreateDescriptorSetLayout(m_device, &layoutCreate, nullptr, &descriptorSetLayout) != VK_SUCCESS) { vkDestroyShaderModule(m_device, shaderModule, nullptr); throw std::runtime_error("Failed to create descriptor set layout for conversion test."); }

	VkPipelineLayoutCreateInfo pipelineLayoutCreate{}; pipelineLayoutCreate.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO; pipelineLayoutCreate.setLayoutCount = 1; pipelineLayoutCreate.pSetLayouts = &descriptorSetLayout;
	VkPipelineLayout pipelineLayout = VK_NULL_HANDLE; if (vkCreatePipelineLayout(m_device, &pipelineLayoutCreate, nullptr, &pipelineLayout) != VK_SUCCESS) { vkDestroyDescriptorSetLayout(m_device, descriptorSetLayout, nullptr); vkDestroyShaderModule(m_device, shaderModule, nullptr); throw std::runtime_error("Failed to create pipeline layout for conversion test."); }

	VkComputePipelineCreateInfo pipelineCreate{}; pipelineCreate.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO; VkPipelineShaderStageCreateInfo stage{}; stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = shaderModule; stage.pName = "main"; pipelineCreate.stage = stage; pipelineCreate.layout = pipelineLayout;
	VkPipeline pipeline = VK_NULL_HANDLE; if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipelineCreate, nullptr, &pipeline) != VK_SUCCESS) { vkDestroyPipelineLayout(m_device, pipelineLayout, nullptr); vkDestroyDescriptorSetLayout(m_device, descriptorSetLayout, nullptr); vkDestroyShaderModule(m_device, shaderModule, nullptr); throw std::runtime_error("Failed to create compute pipeline for conversion test."); }

	// Descriptor pool + set
	std::array<VkDescriptorPoolSize,1> poolSizes{}; poolSizes[0].type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; poolSizes[0].descriptorCount = 4;
	VkDescriptorPoolCreateInfo poolCreate{}; poolCreate.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO; poolCreate.maxSets = 1; poolCreate.poolSizeCount = 1; poolCreate.pPoolSizes = poolSizes.data();
	VkDescriptorPool descriptorPool = VK_NULL_HANDLE; if (vkCreateDescriptorPool(m_device, &poolCreate, nullptr, &descriptorPool) != VK_SUCCESS) { vkDestroyPipeline(m_device, pipeline, nullptr); vkDestroyPipelineLayout(m_device, pipelineLayout, nullptr); vkDestroyDescriptorSetLayout(m_device, descriptorSetLayout, nullptr); vkDestroyShaderModule(m_device, shaderModule, nullptr); throw std::runtime_error("Failed to create descriptor pool for conversion test."); }

	VkDescriptorSetAllocateInfo allocInfo{}; allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO; allocInfo.descriptorPool = descriptorPool; allocInfo.descriptorSetCount = 1; allocInfo.pSetLayouts = &descriptorSetLayout;
	VkDescriptorSet descriptorSet = VK_NULL_HANDLE; if (vkAllocateDescriptorSets(m_device, &allocInfo, &descriptorSet) != VK_SUCCESS) { vkDestroyDescriptorPool(m_device, descriptorPool, nullptr); vkDestroyPipeline(m_device, pipeline, nullptr); vkDestroyPipelineLayout(m_device, pipelineLayout, nullptr); vkDestroyDescriptorSetLayout(m_device, descriptorSetLayout, nullptr); vkDestroyShaderModule(m_device, shaderModule, nullptr); throw std::runtime_error("Failed to allocate descriptor set for conversion test."); }

	std::array<VkDescriptorBufferInfo,4> descInfos{};
	descInfos[0].buffer = deviceIntrinsics; descInfos[0].offset = 0; descInfos[0].range = intrinsicsSize;
	descInfos[1].buffer = devicePoses; descInfos[1].offset = 0; descInfos[1].range = posesSize;
	descInfos[2].buffer = deviceIntrinsicsOut; descInfos[2].offset = 0; descInfos[2].range = intrinsicsSize;
	descInfos[3].buffer = devicePosesOut; descInfos[3].offset = 0; descInfos[3].range = posesSize;

	std::array<VkWriteDescriptorSet,4> writes{};
	for (uint32_t i = 0; i < 4; ++i) { writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; writes[i].dstSet = descriptorSet; writes[i].dstBinding = i; writes[i].dstArrayElement = 0; writes[i].descriptorCount = 1; writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; writes[i].pBufferInfo = &descInfos[i]; }
	vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);

	// Record command buffer: copy staging -> device inputs, barrier, bind pipeline, dispatch, barrier, copy outputs -> readbacks
	VkCommandBufferAllocateInfo allocCmd{}; allocCmd.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO; allocCmd.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; allocCmd.commandPool = m_commandPool; allocCmd.commandBufferCount = 1;
	VkCommandBuffer cmd = VK_NULL_HANDLE; if (vkAllocateCommandBuffers(m_device, &allocCmd, &cmd) != VK_SUCCESS) { throw std::runtime_error("Failed to allocate command buffer for conversion test."); }

	VkCommandBufferBeginInfo beginInfo{}; beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO; beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	if (vkBeginCommandBuffer(cmd, &beginInfo) != VK_SUCCESS) { throw std::runtime_error("Failed to begin command buffer for conversion test."); }

	VkBufferCopy copyIntr{}; copyIntr.srcOffset = 0; copyIntr.dstOffset = 0; copyIntr.size = intrinsicsSize;
	vkCmdCopyBuffer(cmd, stagingIntrinsics, deviceIntrinsics, 1, &copyIntr);
	VkBufferCopy copyPos{}; copyPos.srcOffset = 0; copyPos.dstOffset = 0; copyPos.size = posesSize;
	vkCmdCopyBuffer(cmd, stagingPoses, devicePoses, 1, &copyPos);

	// upload -> compute barrier
	VkBufferMemoryBarrier uploadBarriers[2]{};
	uploadBarriers[0].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; uploadBarriers[0].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; uploadBarriers[0].dstAccessMask = VK_ACCESS_SHADER_READ_BIT; uploadBarriers[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; uploadBarriers[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; uploadBarriers[0].buffer = deviceIntrinsics; uploadBarriers[0].offset = 0; uploadBarriers[0].size = intrinsicsSize;
	uploadBarriers[1] = uploadBarriers[0]; uploadBarriers[1].buffer = devicePoses; uploadBarriers[1].size = posesSize;

	vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 2, uploadBarriers, 0, nullptr);

	vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
	vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
	vkCmdDispatch(cmd, kCount, 1, 1);

	VkBufferMemoryBarrier outputBarriers[2]{};
	outputBarriers[0].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; outputBarriers[0].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT; outputBarriers[0].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT; outputBarriers[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; outputBarriers[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; outputBarriers[0].buffer = deviceIntrinsicsOut; outputBarriers[0].offset = 0; outputBarriers[0].size = intrinsicsSize;
	outputBarriers[1] = outputBarriers[0]; outputBarriers[1].buffer = devicePosesOut; outputBarriers[1].size = posesSize;
	vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 2, outputBarriers, 0, nullptr);

	VkBufferCopy copyOutIntr{}; copyOutIntr.srcOffset = 0; copyOutIntr.dstOffset = 0; copyOutIntr.size = intrinsicsSize;
	vkCmdCopyBuffer(cmd, deviceIntrinsicsOut, readIntrinsics, 1, &copyOutIntr);
	VkBufferCopy copyOutPos{}; copyOutPos.srcOffset = 0; copyOutPos.dstOffset = 0; copyOutPos.size = posesSize;
	vkCmdCopyBuffer(cmd, devicePosesOut, readPoses, 1, &copyOutPos);

	if (vkEndCommandBuffer(cmd) != VK_SUCCESS) { throw std::runtime_error("Failed to record command buffer for conversion test."); }

	VkFenceCreateInfo fenceInfo{}; fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO; VkFence fence = VK_NULL_HANDLE; if (vkCreateFence(m_device, &fenceInfo, nullptr, &fence) != VK_SUCCESS) { throw std::runtime_error("Failed to create fence for conversion test."); }
	VkSubmitInfo submit{}; submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO; submit.commandBufferCount = 1; submit.pCommandBuffers = &cmd; if (vkQueueSubmit(m_queue, 1, &submit, fence) != VK_SUCCESS) { if (fence != VK_NULL_HANDLE) { vkDestroyFence(m_device, fence, nullptr); fence = VK_NULL_HANDLE; } throw std::runtime_error("Failed to submit conversion test command buffer."); }
	if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS) { if (fence != VK_NULL_HANDLE) { vkDestroyFence(m_device, fence, nullptr); fence = VK_NULL_HANDLE; } throw std::runtime_error("Failed to wait for fence for conversion test."); }
	if (fence != VK_NULL_HANDLE) { vkDestroyFence(m_device, fence, nullptr); fence = VK_NULL_HANDLE; }

	// Map readbacks and compare against gpuIntrinsics/gpuPoses
	void *mapped = nullptr;
	vkMapMemory(m_device, readIntrinsicsMem, 0, intrinsicsSize, 0, &mapped);
	GpuCameraIntrinsics *intrRead = reinterpret_cast<GpuCameraIntrinsics *>(mapped);
	bool intrAll = true;
	for (uint32_t i = 0; i < kCount; ++i)
	{
		const GpuCameraIntrinsics &exp = gpuIntrinsics[i];
		const GpuCameraIntrinsics &act = intrRead[i];
		if (exp.m_width != act.m_width || exp.m_height != act.m_height || fabs(exp.m_fx - act.m_fx) > 1e-6f || fabs(exp.m_fy - act.m_fy) > 1e-6f || fabs(exp.m_cx - act.m_cx) > 1e-6f || fabs(exp.m_cy - act.m_cy) > 1e-6f || fabs(exp.m_k1 - act.m_k1) > 1e-6f || fabs(exp.m_k2 - act.m_k2) > 1e-6f || fabs(exp.m_p1 - act.m_p1) > 1e-6f || fabs(exp.m_p2 - act.m_p2) > 1e-6f)
		{
			UIUtility::Print(std::string("Camera conversion smoke test mismatch\nrecord: ") + std::to_string(i) + " type: GpuCameraIntrinsics\n"); intrAll = false; break;
		}
	}
	vkUnmapMemory(m_device, readIntrinsicsMem);

	vkMapMemory(m_device, readPosesMem, 0, posesSize, 0, &mapped);
	GpuCameraPose *poseRead = reinterpret_cast<GpuCameraPose *>(mapped);
	bool poseAll = true;
	for (uint32_t i = 0; i < kCount; ++i)
	{
		const GpuCameraPose &exp = gpuPoses[i];
		const GpuCameraPose &act = poseRead[i];
		for (int m = 0; m < 4; ++m)
		{
			if (fabs(exp.m_position[m] - act.m_position[m]) > 1e-6f) { UIUtility::Print(std::string("Camera conversion smoke test mismatch\nrecord: ") + std::to_string(i) + " type: GpuCameraPose field: position[" + std::to_string(m) + "]\n"); poseAll = false; break; }
			if (fabs(exp.m_forward[m] - act.m_forward[m]) > 1e-6f) { UIUtility::Print(std::string("Camera conversion smoke test mismatch\nrecord: ") + std::to_string(i) + " type: GpuCameraPose field: forward[" + std::to_string(m) + "]\n"); poseAll = false; break; }
			if (fabs(exp.m_up[m] - act.m_up[m]) > 1e-6f) { UIUtility::Print(std::string("Camera conversion smoke test mismatch\nrecord: ") + std::to_string(i) + " type: GpuCameraPose field: up[" + std::to_string(m) + "]\n"); poseAll = false; break; }
			if (fabs(exp.m_right[m] - act.m_right[m]) > 1e-6f) { UIUtility::Print(std::string("Camera conversion smoke test mismatch\nrecord: ") + std::to_string(i) + " type: GpuCameraPose field: right[" + std::to_string(m) + "]\n"); poseAll = false; break; }
		}
		if (!poseAll) break;
	}
	vkUnmapMemory(m_device, readPosesMem);

	if (intrAll && poseAll)
	{
		UIUtility::Print("PlaneSweep GPU camera-conversion smoke test succeeded:\n3 / 3 intrinsic conversions matched.\n3 / 3 pose conversions matched.\n3 / 3 GPU intrinsic round trips matched.\n3 / 3 GPU pose round trips matched.\n");
	}

	// Cleanup same as layout test
	vkFreeCommandBuffers(m_device, m_commandPool, 1, &cmd);
	vkDestroyPipeline(m_device, pipeline, nullptr);
	vkDestroyPipelineLayout(m_device, pipelineLayout, nullptr);
	vkDestroyShaderModule(m_device, shaderModule, nullptr);
	vkDestroyDescriptorPool(m_device, descriptorPool, nullptr);
	vkDestroyDescriptorSetLayout(m_device, descriptorSetLayout, nullptr);

	vkFreeMemory(m_device, readPosesMem, nullptr); vkDestroyBuffer(m_device, readPoses, nullptr);
	vkFreeMemory(m_device, readIntrinsicsMem, nullptr); vkDestroyBuffer(m_device, readIntrinsics, nullptr);

	vkFreeMemory(m_device, devicePosesOutMem, nullptr); vkDestroyBuffer(m_device, devicePosesOut, nullptr);
	vkFreeMemory(m_device, deviceIntrinsicsOutMem, nullptr); vkDestroyBuffer(m_device, deviceIntrinsicsOut, nullptr);

	vkFreeMemory(m_device, devicePosesMem, nullptr); vkDestroyBuffer(m_device, devicePoses, nullptr);
	vkFreeMemory(m_device, deviceIntrinsicsMem, nullptr); vkDestroyBuffer(m_device, deviceIntrinsics, nullptr);

	vkFreeMemory(m_device, stagingPosesMem, nullptr); vkDestroyBuffer(m_device, stagingPoses, nullptr);
	vkFreeMemory(m_device, stagingIntrinsicsMem, nullptr); vkDestroyBuffer(m_device, stagingIntrinsics, nullptr);

	return intrAll && poseAll;
}

// Loads a compiled shader file using the same search strategy used elsewhere in the project.
std::vector<char> PlaneSweepGpu::ReadShaderFile(const std::string &relativePath) const
{
	std::vector<std::filesystem::path> candidates;

	wchar_t modulePath[MAX_PATH]{};
	const DWORD modulePathLength = GetModuleFileNameW(nullptr, modulePath, MAX_PATH);
	if (modulePathLength > 0)
	{
		candidates.push_back(std::filesystem::path(modulePath).parent_path() / relativePath);
	}

	candidates.push_back(PathUtility::ShaderOutputFolder() / std::filesystem::path(relativePath).filename());
	candidates.push_back(std::filesystem::current_path() / relativePath);
	candidates.push_back(std::filesystem::path(relativePath));

	for (const auto &candidate : candidates)
	{
		std::ifstream file(candidate, std::ios::ate | std::ios::binary);
		if (!file.is_open())
		{
			continue;
		}

		const std::streampos endPosition = file.tellg();
		if (endPosition == std::streampos(-1))
		{
			file.close();
			continue;
		}

		const std::size_t fileSize = static_cast<std::size_t>(endPosition);
		if (fileSize == 0)
		{
			file.close();
			continue;
		}

		std::vector<char> buffer(fileSize);
		file.seekg(0);
		file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
		file.close();
		return buffer;
	}

	return {};
}

PlaneSweepGpu::~PlaneSweepGpu(void)
{
	Clean();
}

void PlaneSweepGpu::Initialize(VkDevice device, VkPhysicalDevice physicalDevice, uint32_t queueFamilyIndex, VkQueue queue, VkCommandPool commandPool)
{
	if (m_initialized)
	{
		throw std::runtime_error("PlaneSweepGpu already initialized.");
	}

	m_device = device;
	m_physicalDevice = physicalDevice;
	m_queueFamilyIndex = queueFamilyIndex;
	m_queue = queue;
	m_commandPool = commandPool;

	m_initialized = true;
}

void PlaneSweepGpu::Clean(void)
{
	if (!m_initialized)
	{
		return;
	}

	if (m_computePipeline != VK_NULL_HANDLE)
	{
		vkDestroyPipeline(m_device, m_computePipeline, nullptr);
		m_computePipeline = VK_NULL_HANDLE;
	}

	if (m_pipelineLayout != VK_NULL_HANDLE)
	{
		vkDestroyPipelineLayout(m_device, m_pipelineLayout, nullptr);
		m_pipelineLayout = VK_NULL_HANDLE;
	}

	if (m_shaderModule != VK_NULL_HANDLE)
	{
		vkDestroyShaderModule(m_device, m_shaderModule, nullptr);
		m_shaderModule = VK_NULL_HANDLE;
	}

	m_initialized = false;
}

// Runs a trivial compute smoke test: create shader module, pipeline, record a one-time command buffer, dispatch, submit, and wait for completion.
void PlaneSweepGpu::RunComputeSmokeTest(void)
{
	if (!m_initialized)
	{
		throw std::runtime_error("PlaneSweepGpu not initialized.");
	}

	// Load SPIR-V from compiled shader file using the same search strategy used elsewhere.
	const std::string shaderRelativePath = "shaders/planeSweepSmokeTest.comp.spv";
	std::vector<char> code = ReadShaderFile(shaderRelativePath);
	if (code.empty())
	{
		UIUtility::Print("Compute shader SPIR-V not found: " + shaderRelativePath + " - ensure ShaderUtility compiles .comp to Output/Shaders at startup.\n");
		throw std::runtime_error("Failed to load compute shader SPIR-V: " + shaderRelativePath);
	}

	VkShaderModuleCreateInfo createInfo{};
	createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	createInfo.codeSize = code.size();
	createInfo.pCode = reinterpret_cast<const uint32_t *>(code.data());

	if (vkCreateShaderModule(m_device, &createInfo, nullptr, &m_shaderModule) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to create compute shader module.");
	}

	VkPipelineShaderStageCreateInfo shaderStage{};
	shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
	shaderStage.module = m_shaderModule;
	shaderStage.pName = "main";

	VkPipelineLayoutCreateInfo layoutInfo{};
	layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;

	if (vkCreatePipelineLayout(m_device, &layoutInfo, nullptr, &m_pipelineLayout) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to create pipeline layout.");
	}

	VkComputePipelineCreateInfo pipelineInfo{};
	pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
	pipelineInfo.stage = shaderStage;
	pipelineInfo.layout = m_pipelineLayout;

	if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_computePipeline) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to create compute pipeline.");
	}

	// Allocate a one-time command buffer from the provided command pool.
	VkCommandBufferAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandPool = m_commandPool;
	allocInfo.commandBufferCount = 1;

	VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
	if (vkAllocateCommandBuffers(m_device, &allocInfo, &commandBuffer) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to allocate command buffer for compute smoke test.");
	}

	VkCommandBufferBeginInfo beginInfo{};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

	if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to begin compute command buffer.");
	}

	vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_computePipeline);
	vkCmdDispatch(commandBuffer, 1, 1, 1);

	if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to record compute command buffer.");
	}

	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &commandBuffer;

	VkFenceCreateInfo fenceInfo{};
	fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;

	VkFence fence = VK_NULL_HANDLE;
	if (vkCreateFence(m_device, &fenceInfo, nullptr, &fence) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to create fence for compute submission.");
	}

	if (vkQueueSubmit(m_queue, 1, &submitInfo, fence) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to submit compute command buffer.");
	}

	// Wait for completion to keep this smoke test simple and synchronous.
	if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to wait for compute fence.");
	}

	vkDestroyFence(m_device, fence, nullptr);

	vkFreeCommandBuffers(m_device, m_commandPool, 1, &commandBuffer);
}

bool PlaneSweepGpu::RunStorageBufferSmokeTest(void)
{
	if (!m_initialized)
	{
		throw std::runtime_error("PlaneSweepGpu not initialized.");
	}

	constexpr uint32_t kCount = 64;
	const VkDeviceSize bufferSize = sizeof(float) * kCount;

	// Create device-local storage buffer (GPU writes)
	VkBuffer storageBuffer = VK_NULL_HANDLE;
	VkDeviceMemory storageBufferMemory = VK_NULL_HANDLE;
	{
		VkBufferCreateInfo bufferInfo{};
		bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bufferInfo.size = bufferSize;
		bufferInfo.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
		bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

		if (vkCreateBuffer(m_device, &bufferInfo, nullptr, &storageBuffer) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to create storage buffer.");
		}

		VkMemoryRequirements memReq{};
		vkGetBufferMemoryRequirements(m_device, storageBuffer, &memReq);

		VkPhysicalDeviceMemoryProperties memProperties{};
		vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memProperties);

		uint32_t memoryTypeIndex = UINT32_MAX;
		for (uint32_t i = 0; i < memProperties.memoryTypeCount; ++i)
		{
			if ((memReq.memoryTypeBits & (1u << i)) != 0 && (memProperties.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) == VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)
			{
				memoryTypeIndex = i;
				break;
			}
		}

		if (memoryTypeIndex == UINT32_MAX)
		{
			vkDestroyBuffer(m_device, storageBuffer, nullptr);
			throw std::runtime_error("Failed to find suitable memory type for storage buffer.");
		}

		VkMemoryAllocateInfo allocInfo{};
		allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		allocInfo.allocationSize = memReq.size;
		allocInfo.memoryTypeIndex = memoryTypeIndex;

		if (vkAllocateMemory(m_device, &allocInfo, nullptr, &storageBufferMemory) != VK_SUCCESS)
		{
			vkDestroyBuffer(m_device, storageBuffer, nullptr);
			throw std::runtime_error("Failed to allocate storage buffer memory.");
		}

		vkBindBufferMemory(m_device, storageBuffer, storageBufferMemory, 0);
	}

	// Create host-visible readback buffer (transfer dst)
	VkBuffer readbackBuffer = VK_NULL_HANDLE;
	VkDeviceMemory readbackBufferMemory = VK_NULL_HANDLE;
	{
		VkBufferCreateInfo bufferInfo{};
		bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bufferInfo.size = bufferSize;
		bufferInfo.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
		bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

		if (vkCreateBuffer(m_device, &bufferInfo, nullptr, &readbackBuffer) != VK_SUCCESS)
		{
			// cleanup storage
			vkFreeMemory(m_device, storageBufferMemory, nullptr);
			vkDestroyBuffer(m_device, storageBuffer, nullptr);
			throw std::runtime_error("Failed to create readback buffer.");
		}

		VkMemoryRequirements memReq{};
		vkGetBufferMemoryRequirements(m_device, readbackBuffer, &memReq);

		VkPhysicalDeviceMemoryProperties memProperties{};
		vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memProperties);

		uint32_t memoryTypeIndex = UINT32_MAX;
		for (uint32_t i = 0; i < memProperties.memoryTypeCount; ++i)
		{
			if ((memReq.memoryTypeBits & (1u << i)) != 0 && (memProperties.memoryTypes[i].propertyFlags & (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)) == (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT))
			{
				memoryTypeIndex = i;
				break;
			}
		}

		if (memoryTypeIndex == UINT32_MAX)
		{
			vkDestroyBuffer(m_device, readbackBuffer, nullptr);
			vkFreeMemory(m_device, storageBufferMemory, nullptr);
			vkDestroyBuffer(m_device, storageBuffer, nullptr);
			throw std::runtime_error("Failed to find suitable memory type for readback buffer.");
		}

		VkMemoryAllocateInfo allocInfo{};
		allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		allocInfo.allocationSize = memReq.size;
		allocInfo.memoryTypeIndex = memoryTypeIndex;

		if (vkAllocateMemory(m_device, &allocInfo, nullptr, &readbackBufferMemory) != VK_SUCCESS)
		{
			vkDestroyBuffer(m_device, readbackBuffer, nullptr);
			vkFreeMemory(m_device, storageBufferMemory, nullptr);
			vkDestroyBuffer(m_device, storageBuffer, nullptr);
			throw std::runtime_error("Failed to allocate readback buffer memory.");
		}

		vkBindBufferMemory(m_device, readbackBuffer, readbackBufferMemory, 0);
	}

	// Descriptor layout
	VkDescriptorSetLayoutBinding binding{};
	binding.binding = 0;
	binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	binding.descriptorCount = 1;
	binding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
	binding.pImmutableSamplers = nullptr;

	VkDescriptorSetLayoutCreateInfo layoutInfo{};
	layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	layoutInfo.bindingCount = 1;
	layoutInfo.pBindings = &binding;

	VkDescriptorSetLayout descriptorSetLayout = VK_NULL_HANDLE;
	if (vkCreateDescriptorSetLayout(m_device, &layoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to create descriptor set layout.");
	}

	// Descriptor pool
	VkDescriptorPoolSize poolSize{};
	poolSize.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	poolSize.descriptorCount = 1;

	VkDescriptorPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	poolInfo.maxSets = 1;
	poolInfo.poolSizeCount = 1;
	poolInfo.pPoolSizes = &poolSize;

	VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
	if (vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS)
	{
		vkDestroyDescriptorSetLayout(m_device, descriptorSetLayout, nullptr);
		throw std::runtime_error("Failed to create descriptor pool.");
	}

	VkDescriptorSetAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	allocInfo.descriptorPool = descriptorPool;
	allocInfo.descriptorSetCount = 1;
	allocInfo.pSetLayouts = &descriptorSetLayout;

	VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
	if (vkAllocateDescriptorSets(m_device, &allocInfo, &descriptorSet) != VK_SUCCESS)
	{
		vkDestroyDescriptorPool(m_device, descriptorPool, nullptr);
		vkDestroyDescriptorSetLayout(m_device, descriptorSetLayout, nullptr);
		throw std::runtime_error("Failed to allocate descriptor set.");
	}

	VkDescriptorBufferInfo bufferInfo{};
	bufferInfo.buffer = storageBuffer;
	bufferInfo.offset = 0;
	bufferInfo.range = bufferSize;

	VkWriteDescriptorSet descriptorWrite{};
	descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
	descriptorWrite.dstSet = descriptorSet;
	descriptorWrite.dstBinding = 0;
	descriptorWrite.dstArrayElement = 0;
	descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	descriptorWrite.descriptorCount = 1;
	descriptorWrite.pBufferInfo = &bufferInfo;

	vkUpdateDescriptorSets(m_device, 1, &descriptorWrite, 0, nullptr);

	// Load shader
	const std::string shaderRelativePath = "shaders/planeSweepBufferSmokeTest.comp.spv";
	std::vector<char> code = ReadShaderFile(shaderRelativePath);
	if (code.empty())
	{
		UIUtility::Print("Compute shader SPIR-V not found: " + shaderRelativePath + " - ensure ShaderUtility compiles .comp to Output/Shaders at startup.\n");
		vkDestroyDescriptorPool(m_device, descriptorPool, nullptr);
		vkDestroyDescriptorSetLayout(m_device, descriptorSetLayout, nullptr);
		vkFreeMemory(m_device, readbackBufferMemory, nullptr);
		vkDestroyBuffer(m_device, readbackBuffer, nullptr);
		vkFreeMemory(m_device, storageBufferMemory, nullptr);
		vkDestroyBuffer(m_device, storageBuffer, nullptr);
		return false;
	}

	VkShaderModuleCreateInfo shaderCreateInfo{};
	shaderCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	shaderCreateInfo.codeSize = code.size();
	shaderCreateInfo.pCode = reinterpret_cast<const uint32_t *>(code.data());

	VkShaderModule shaderModule = VK_NULL_HANDLE;
	if (vkCreateShaderModule(m_device, &shaderCreateInfo, nullptr, &shaderModule) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to create shader module for storage-buffer test.");
	}

	// Pipeline layout
	VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
	pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	pipelineLayoutInfo.setLayoutCount = 1;
	pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout;

	VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
	if (vkCreatePipelineLayout(m_device, &pipelineLayoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS)
	{
		vkDestroyShaderModule(m_device, shaderModule, nullptr);
		throw std::runtime_error("Failed to create pipeline layout for storage-buffer test.");
	}

	VkPipelineShaderStageCreateInfo stageInfo{};
	stageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	stageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
	stageInfo.module = shaderModule;
	stageInfo.pName = "main";

	VkComputePipelineCreateInfo pipelineInfo{};
	pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
	pipelineInfo.stage = stageInfo;
	pipelineInfo.layout = pipelineLayout;

	VkPipeline pipeline = VK_NULL_HANDLE;
	if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline) != VK_SUCCESS)
	{
		vkDestroyPipelineLayout(m_device, pipelineLayout, nullptr);
		vkDestroyShaderModule(m_device, shaderModule, nullptr);
		throw std::runtime_error("Failed to create compute pipeline for storage-buffer test.");
	}

	// Command buffer
	VkCommandBufferAllocateInfo allocInfoCmd{};
	allocInfoCmd.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfoCmd.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfoCmd.commandPool = m_commandPool;
	allocInfoCmd.commandBufferCount = 1;

	VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
	if (vkAllocateCommandBuffers(m_device, &allocInfoCmd, &commandBuffer) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to allocate command buffer for storage-buffer test.");
	}

	VkCommandBufferBeginInfo beginInfo{};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

	if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to begin command buffer for storage-buffer test.");
	}

	vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
	vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
	vkCmdDispatch(commandBuffer, 1, 1, 1);

	// Barrier: ensure compute shader writes are visible to transfer
	VkBufferMemoryBarrier bufferBarrier{};
	bufferBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
	bufferBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
	bufferBarrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
	bufferBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	bufferBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	bufferBarrier.buffer = storageBuffer;
	bufferBarrier.offset = 0;
	bufferBarrier.size = bufferSize;

	vkCmdPipelineBarrier(
		commandBuffer,
		VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
		VK_PIPELINE_STAGE_TRANSFER_BIT,
		0,
		0, nullptr,
		1, &bufferBarrier,
		0, nullptr
	);

	// Copy to readback buffer
	VkBufferCopy copyRegion{};
	copyRegion.srcOffset = 0;
	copyRegion.dstOffset = 0;
	copyRegion.size = bufferSize;
	vkCmdCopyBuffer(commandBuffer, storageBuffer, readbackBuffer, 1, &copyRegion);

	if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to record command buffer for storage-buffer test.");
	}

	VkFenceCreateInfo fenceInfo{};
	fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;

	VkFence fence = VK_NULL_HANDLE;
	if (vkCreateFence(m_device, &fenceInfo, nullptr, &fence) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to create fence for storage-buffer test.");
	}

	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &commandBuffer;

	if (vkQueueSubmit(m_queue, 1, &submitInfo, fence) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to submit storage-buffer test command buffer.");
	}

	if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to wait for fence for storage-buffer test.");
	}

	vkDestroyFence(m_device, fence, nullptr);

	// Map readback memory and verify
	void *mapped = nullptr;
	vkMapMemory(m_device, readbackBufferMemory, 0, bufferSize, 0, &mapped);

	float *data = reinterpret_cast<float *>(mapped);
	bool allMatched = true;
	constexpr float kEps = 1e-6f;
	for (uint32_t i = 0; i < kCount; ++i)
	{
		const float expected = 123.0f + static_cast<float>(i) * 0.25f;
		const float actual = data[i];
		const float absErr = std::fabs(expected - actual);
		if (absErr > kEps)
		{
			UIUtility::Print(std::string("PlaneSweep GPU storage-buffer smoke test failed at index ") + std::to_string(i) + ": expected " + std::to_string(expected) + ", actual " + std::to_string(actual) + ", absErr " + std::to_string(absErr) + "\n");
			allMatched = false;
			break;
		}
	}

	vkUnmapMemory(m_device, readbackBufferMemory);

	if (allMatched)
	{
		UIUtility::Print("PlaneSweep GPU storage-buffer smoke test succeeded:\n64 / 64 values matched.\n");
	}

	// Cleanup
	vkFreeCommandBuffers(m_device, m_commandPool, 1, &commandBuffer);
	vkDestroyPipeline(m_device, pipeline, nullptr);
	vkDestroyPipelineLayout(m_device, pipelineLayout, nullptr);
	vkDestroyShaderModule(m_device, shaderModule, nullptr);
	vkDestroyDescriptorPool(m_device, descriptorPool, nullptr);
	vkDestroyDescriptorSetLayout(m_device, descriptorSetLayout, nullptr);
	vkFreeMemory(m_device, readbackBufferMemory, nullptr);
	vkDestroyBuffer(m_device, readbackBuffer, nullptr);
	vkFreeMemory(m_device, storageBufferMemory, nullptr);
	vkDestroyBuffer(m_device, storageBuffer, nullptr);

	return allMatched;
}

bool PlaneSweepGpu::RunCameraLayoutSmokeTest(void)
{
	if (!m_initialized)
	{
		throw std::runtime_error("PlaneSweepGpu not initialized.");
	}

	// Prepare 3 synthetic records for intrinsics and poses
	constexpr uint32_t kCount = 3u;
	const VkDeviceSize intrinsicsSize = sizeof(GpuCameraIntrinsics) * kCount;
	const VkDeviceSize posesSize = sizeof(GpuCameraPose) * kCount;

	// Create and populate staging buffers (host-visible coherent)
	auto createHostBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkBuffer &buffer, VkDeviceMemory &memory)
	{
		VkBufferCreateInfo bufInfo{};
		bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bufInfo.size = size;
		bufInfo.usage = usage;
		bufInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

		if (vkCreateBuffer(m_device, &bufInfo, nullptr, &buffer) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to create staging buffer.");
		}

		VkMemoryRequirements memReq{};
		vkGetBufferMemoryRequirements(m_device, buffer, &memReq);

		VkPhysicalDeviceMemoryProperties memProps{};
		vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memProps);

		uint32_t memType = UINT32_MAX;
		for (uint32_t i = 0; i < memProps.memoryTypeCount; ++i)
		{
			if ((memReq.memoryTypeBits & (1u << i)) != 0 && (memProps.memoryTypes[i].propertyFlags & (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)) == (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT))
			{
				memType = i;
				break;
			}
		}

		if (memType == UINT32_MAX)
		{
			vkDestroyBuffer(m_device, buffer, nullptr);
			throw std::runtime_error("No suitable memory type for staging buffer.");
		}

		VkMemoryAllocateInfo alloc{};
		alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		alloc.allocationSize = memReq.size;
		alloc.memoryTypeIndex = memType;

		if (vkAllocateMemory(m_device, &alloc, nullptr, &memory) != VK_SUCCESS)
		{
			vkDestroyBuffer(m_device, buffer, nullptr);
			throw std::runtime_error("Failed to allocate staging memory.");
		}

		vkBindBufferMemory(m_device, buffer, memory, 0);
	};

	VkBuffer stagingIntrinsics = VK_NULL_HANDLE; VkDeviceMemory stagingIntrinsicsMem = VK_NULL_HANDLE;
	VkBuffer stagingPoses = VK_NULL_HANDLE; VkDeviceMemory stagingPosesMem = VK_NULL_HANDLE;
	createHostBuffer(intrinsicsSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, stagingIntrinsics, stagingIntrinsicsMem);
	createHostBuffer(posesSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, stagingPoses, stagingPosesMem);

	// Fill staging with deterministic synthetic data
	// Zero-initialize then set fields
	std::vector<GpuCameraIntrinsics> intrinsics(kCount);
	std::vector<GpuCameraPose> poses(kCount);

	// Record 0
	intrinsics[0] = GpuCameraIntrinsics{};
	intrinsics[0].m_width = 641; intrinsics[0].m_height = 479;
	intrinsics[0].m_fx = 501.25f; intrinsics[0].m_fy = 502.50f;
	intrinsics[0].m_cx = 319.75f; intrinsics[0].m_cy = 240.125f;
	intrinsics[0].m_k1 = 0.011f; intrinsics[0].m_k2 = -0.022f; intrinsics[0].m_p1 = 0.0035f; intrinsics[0].m_p2 = -0.0045f;

	poses[0] = GpuCameraPose{};
	poses[0].m_position[0] = 1.25f; poses[0].m_position[1] = -2.5f; poses[0].m_position[2] = 3.75f; poses[0].m_position[3] = 0.0f;
	poses[0].m_forward[0] = 0.11f; poses[0].m_forward[1] = 0.22f; poses[0].m_forward[2] = 0.93f; poses[0].m_forward[3] = 0.0f;
	poses[0].m_up[0] = -0.17f; poses[0].m_up[1] = 0.98f; poses[0].m_up[2] = 0.07f; poses[0].m_up[3] = 0.0f;
	poses[0].m_right[0] = 0.97f; poses[0].m_right[1] = 0.19f; poses[0].m_right[2] = -0.13f; poses[0].m_right[3] = 0.0f;

	// Record 1
	intrinsics[1] = GpuCameraIntrinsics{};
	intrinsics[1].m_width = 1283; intrinsics[1].m_height = 721;
	intrinsics[1].m_fx = 812.75f; intrinsics[1].m_fy = 810.125f;
	intrinsics[1].m_cx = 640.5f; intrinsics[1].m_cy = 360.25f;
	intrinsics[1].m_k1 = -0.013f; intrinsics[1].m_k2 = 0.027f; intrinsics[1].m_p1 = -0.0055f; intrinsics[1].m_p2 = 0.0065f;

	poses[1] = GpuCameraPose{};
	poses[1].m_position[0] = -10.0f; poses[1].m_position[1] = 5.5f; poses[1].m_position[2] = 0.125f; poses[1].m_position[3] = 0.0f;
	poses[1].m_forward[0] = -0.3f; poses[1].m_forward[1] = 0.4f; poses[1].m_forward[2] = 0.866f; poses[1].m_forward[3] = 0.0f;
	poses[1].m_up[0] = 0.0f; poses[1].m_up[1] = 1.0f; poses[1].m_up[2] = 0.0f; poses[1].m_up[3] = 0.0f;
	poses[1].m_right[0] = 0.953f; poses[1].m_right[1] = -0.289f; poses[1].m_right[2] = 0.089f; poses[1].m_right[3] = 0.0f;

	// Record 2
	intrinsics[2] = GpuCameraIntrinsics{};
	intrinsics[2].m_width = 1920; intrinsics[2].m_height = 1080;
	intrinsics[2].m_fx = 1000.0f; intrinsics[2].m_fy = 999.5f;
	intrinsics[2].m_cx = 960.0f; intrinsics[2].m_cy = 540.0f;
	intrinsics[2].m_k1 = 0.0f; intrinsics[2].m_k2 = 0.0f; intrinsics[2].m_p1 = 0.0f; intrinsics[2].m_p2 = 0.0f;

	poses[2] = GpuCameraPose{};
	poses[2].m_position[0] = 100.0f; poses[2].m_position[1] = -100.0f; poses[2].m_position[2] = 50.5f; poses[2].m_position[3] = 0.0f;
	poses[2].m_forward[0] = 0.0f; poses[2].m_forward[1] = 0.0f; poses[2].m_forward[2] = 1.0f; poses[2].m_forward[3] = 0.0f;
	poses[2].m_up[0] = 0.0f; poses[2].m_up[1] = 1.0f; poses[2].m_up[2] = 0.0f; poses[2].m_up[3] = 0.0f;
	poses[2].m_right[0] = 1.0f; poses[2].m_right[1] = 0.0f; poses[2].m_right[2] = 0.0f; poses[2].m_right[3] = 0.0f;

	// Copy data into staging buffers
	void *mapPtr = nullptr;
	vkMapMemory(m_device, stagingIntrinsicsMem, 0, intrinsicsSize, 0, &mapPtr);
	memcpy(mapPtr, intrinsics.data(), static_cast<size_t>(intrinsicsSize));
	vkUnmapMemory(m_device, stagingIntrinsicsMem);

	vkMapMemory(m_device, stagingPosesMem, 0, posesSize, 0, &mapPtr);
	memcpy(mapPtr, poses.data(), static_cast<size_t>(posesSize));
	vkUnmapMemory(m_device, stagingPosesMem);

	// Create device-local input buffers (transfer dst | storage buffer)
	auto createDeviceLocalBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkBuffer &buffer, VkDeviceMemory &memory)
	{
		VkBufferCreateInfo bufInfo{};
		bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bufInfo.size = size;
		bufInfo.usage = usage;
		bufInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

		if (vkCreateBuffer(m_device, &bufInfo, nullptr, &buffer) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to create device-local buffer.");
		}

		VkMemoryRequirements memReq{};
		vkGetBufferMemoryRequirements(m_device, buffer, &memReq);

		VkPhysicalDeviceMemoryProperties memProps{};
		vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memProps);

		uint32_t memType = UINT32_MAX;
		for (uint32_t i = 0; i < memProps.memoryTypeCount; ++i)
		{
			if ((memReq.memoryTypeBits & (1u << i)) != 0 && (memProps.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) == VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)
			{
				memType = i;
				break;
			}
		}

		if (memType == UINT32_MAX)
		{
			vkDestroyBuffer(m_device, buffer, nullptr);
			throw std::runtime_error("No suitable device-local memory type.");
		}

		VkMemoryAllocateInfo alloc{};
		alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		alloc.allocationSize = memReq.size;
		alloc.memoryTypeIndex = memType;

		if (vkAllocateMemory(m_device, &alloc, nullptr, &memory) != VK_SUCCESS)
		{
			vkDestroyBuffer(m_device, buffer, nullptr);
			throw std::runtime_error("Failed to allocate device-local memory.");
		}

		vkBindBufferMemory(m_device, buffer, memory, 0);
	};

	VkBuffer deviceIntrinsics = VK_NULL_HANDLE; VkDeviceMemory deviceIntrinsicsMem = VK_NULL_HANDLE;
	VkBuffer devicePoses = VK_NULL_HANDLE; VkDeviceMemory devicePosesMem = VK_NULL_HANDLE;
	createDeviceLocalBuffer(intrinsicsSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, deviceIntrinsics, deviceIntrinsicsMem);
	createDeviceLocalBuffer(posesSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, devicePoses, devicePosesMem);

	// Create device-local output buffers (storage | transfer src)
	VkBuffer deviceIntrinsicsOut = VK_NULL_HANDLE; VkDeviceMemory deviceIntrinsicsOutMem = VK_NULL_HANDLE;
	VkBuffer devicePosesOut = VK_NULL_HANDLE; VkDeviceMemory devicePosesOutMem = VK_NULL_HANDLE;
	createDeviceLocalBuffer(intrinsicsSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, deviceIntrinsicsOut, deviceIntrinsicsOutMem);
	createDeviceLocalBuffer(posesSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, devicePosesOut, devicePosesOutMem);

	// Create readback buffers (host-visible coherent)
	VkBuffer readIntrinsics = VK_NULL_HANDLE; VkDeviceMemory readIntrinsicsMem = VK_NULL_HANDLE;
	VkBuffer readPoses = VK_NULL_HANDLE; VkDeviceMemory readPosesMem = VK_NULL_HANDLE;
	createHostBuffer(intrinsicsSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT, readIntrinsics, readIntrinsicsMem);
	createHostBuffer(posesSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT, readPoses, readPosesMem);

	// Descriptor layout: bindings 0..3
	std::array<VkDescriptorSetLayoutBinding,4> bindings{};
	for (uint32_t i=0;i<4;++i)
	{
		bindings[i].binding = i;
		bindings[i].descriptorCount = 1;
		bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
		bindings[i].pImmutableSamplers = nullptr;
	}

	VkDescriptorSetLayoutCreateInfo dslInfo{};
	dslInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	dslInfo.bindingCount = static_cast<uint32_t>(bindings.size());
	dslInfo.pBindings = bindings.data();

	VkDescriptorSetLayout descriptorSetLayout = VK_NULL_HANDLE;
	if (vkCreateDescriptorSetLayout(m_device, &dslInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to create descriptor set layout for camera test.");
	}

	VkDescriptorPoolSize poolSizes[1];
	poolSizes[0].type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	poolSizes[0].descriptorCount = 4;

	VkDescriptorPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	poolInfo.maxSets = 1;
	poolInfo.poolSizeCount = 1;
	poolInfo.pPoolSizes = poolSizes;

	VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
	if (vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS)
	{
		vkDestroyDescriptorSetLayout(m_device, descriptorSetLayout, nullptr);
		throw std::runtime_error("Failed to create descriptor pool for camera test.");
	}

	VkDescriptorSetAllocateInfo dsa{};
	dsa.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	dsa.descriptorPool = descriptorPool;
	dsa.descriptorSetCount = 1;
	dsa.pSetLayouts = &descriptorSetLayout;

	VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
	if (vkAllocateDescriptorSets(m_device, &dsa, &descriptorSet) != VK_SUCCESS)
	{
		vkDestroyDescriptorPool(m_device, descriptorPool, nullptr);
		vkDestroyDescriptorSetLayout(m_device, descriptorSetLayout, nullptr);
		throw std::runtime_error("Failed to allocate descriptor set for camera test.");
	}

	VkDescriptorBufferInfo descInfos[4];
	descInfos[0].buffer = deviceIntrinsics; descInfos[0].offset = 0; descInfos[0].range = intrinsicsSize;
	descInfos[1].buffer = devicePoses; descInfos[1].offset = 0; descInfos[1].range = posesSize;
	descInfos[2].buffer = deviceIntrinsicsOut; descInfos[2].offset = 0; descInfos[2].range = intrinsicsSize;
	descInfos[3].buffer = devicePosesOut; descInfos[3].offset = 0; descInfos[3].range = posesSize;

	std::array<VkWriteDescriptorSet,4> writes{};
	for (uint32_t i=0;i<4;++i)
	{
		writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writes[i].dstSet = descriptorSet;
		writes[i].dstBinding = i;
		writes[i].dstArrayElement = 0;
		writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		writes[i].descriptorCount = 1;
		writes[i].pBufferInfo = &descInfos[i];
	}

	vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);

	// Load shader
	const std::string shaderRelativePath = "shaders/planeSweepCameraLayoutSmokeTest.comp.spv";
	std::vector<char> code = ReadShaderFile(shaderRelativePath);
	if (code.empty())
	{
		UIUtility::Print("Camera layout compute shader SPIR-V not found: " + shaderRelativePath + "\n");
		// cleanup
		vkDestroyDescriptorPool(m_device, descriptorPool, nullptr);
		vkDestroyDescriptorSetLayout(m_device, descriptorSetLayout, nullptr);
		// free buffers/memory (staging + device)
		vkFreeMemory(m_device, readPosesMem, nullptr); vkDestroyBuffer(m_device, readPoses, nullptr);
		vkFreeMemory(m_device, readIntrinsicsMem, nullptr); vkDestroyBuffer(m_device, readIntrinsics, nullptr);
		vkFreeMemory(m_device, devicePosesOutMem, nullptr); vkDestroyBuffer(m_device, devicePosesOut, nullptr);
		vkFreeMemory(m_device, deviceIntrinsicsOutMem, nullptr); vkDestroyBuffer(m_device, deviceIntrinsicsOut, nullptr);
		vkFreeMemory(m_device, devicePosesMem, nullptr); vkDestroyBuffer(m_device, devicePoses, nullptr);
		vkFreeMemory(m_device, deviceIntrinsicsMem, nullptr); vkDestroyBuffer(m_device, deviceIntrinsics, nullptr);
		vkFreeMemory(m_device, stagingPosesMem, nullptr); vkDestroyBuffer(m_device, stagingPoses, nullptr);
		vkFreeMemory(m_device, stagingIntrinsicsMem, nullptr); vkDestroyBuffer(m_device, stagingIntrinsics, nullptr);
		return false;
	}

	VkShaderModuleCreateInfo shaderCreate{};
	shaderCreate.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	shaderCreate.codeSize = code.size();
	shaderCreate.pCode = reinterpret_cast<const uint32_t *>(code.data());

	VkShaderModule shaderModule = VK_NULL_HANDLE;
	if (vkCreateShaderModule(m_device, &shaderCreate, nullptr, &shaderModule) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to create shader module for camera test.");
	}

	VkPipelineLayoutCreateInfo plInfo{};
	plInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	plInfo.setLayoutCount = 1;
	plInfo.pSetLayouts = &descriptorSetLayout;

	VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
	if (vkCreatePipelineLayout(m_device, &plInfo, nullptr, &pipelineLayout) != VK_SUCCESS)
	{
		vkDestroyShaderModule(m_device, shaderModule, nullptr);
		throw std::runtime_error("Failed to create pipeline layout for camera test.");
	}

	VkPipelineShaderStageCreateInfo stageInfo{};
	stageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	stageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
	stageInfo.module = shaderModule;
	stageInfo.pName = "main";

	VkComputePipelineCreateInfo pipelineInfo{};
	pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
	pipelineInfo.stage = stageInfo;
	pipelineInfo.layout = pipelineLayout;

	VkPipeline pipeline = VK_NULL_HANDLE;
	if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline) != VK_SUCCESS)
	{
		vkDestroyPipelineLayout(m_device, pipelineLayout, nullptr);
		vkDestroyShaderModule(m_device, shaderModule, nullptr);
		throw std::runtime_error("Failed to create compute pipeline for camera test.");
	}

	// Command buffer: copy staging -> device inputs, barrier transfer->compute read, dispatch, barrier compute->transfer, copy outputs -> readbacks
	VkCommandBufferAllocateInfo allocCmd{};
	allocCmd.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocCmd.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocCmd.commandPool = m_commandPool;
	allocCmd.commandBufferCount = 1;

	VkCommandBuffer cmd = VK_NULL_HANDLE;
	if (vkAllocateCommandBuffers(m_device, &allocCmd, &cmd) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to allocate command buffer for camera test.");
	}

	VkCommandBufferBeginInfo begin{};
	begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	if (vkBeginCommandBuffer(cmd, &begin) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to begin command buffer for camera test.");
	}

	// Copy staging -> device inputs
	VkBufferCopy copyIntr{}; copyIntr.srcOffset = 0; copyIntr.dstOffset = 0; copyIntr.size = intrinsicsSize;
	vkCmdCopyBuffer(cmd, stagingIntrinsics, deviceIntrinsics, 1, &copyIntr);
	VkBufferCopy copyPos{}; copyPos.srcOffset = 0; copyPos.dstOffset = 0; copyPos.size = posesSize;
	vkCmdCopyBuffer(cmd, stagingPoses, devicePoses, 1, &copyPos);

	// Barrier: transfer write -> compute read for inputs
	VkBufferMemoryBarrier inputBarriers[2]{};
	inputBarriers[0].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
	inputBarriers[0].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
	inputBarriers[0].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
	inputBarriers[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	inputBarriers[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	inputBarriers[0].buffer = deviceIntrinsics; inputBarriers[0].offset = 0; inputBarriers[0].size = intrinsicsSize;

	inputBarriers[1] = inputBarriers[0];
	inputBarriers[1].buffer = devicePoses; inputBarriers[1].size = posesSize;

	vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 2, inputBarriers, 0, nullptr);

	// Bind and dispatch
	vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
	vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
	vkCmdDispatch(cmd, kCount, 1, 1);

	// Barrier: compute write -> transfer read for outputs
	VkBufferMemoryBarrier outputBarriers[2]{};
	outputBarriers[0].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
	outputBarriers[0].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
	outputBarriers[0].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
	outputBarriers[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	outputBarriers[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	outputBarriers[0].buffer = deviceIntrinsicsOut; outputBarriers[0].offset = 0; outputBarriers[0].size = intrinsicsSize;

	outputBarriers[1] = outputBarriers[0];
	outputBarriers[1].buffer = devicePosesOut; outputBarriers[1].size = posesSize;

	vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 2, outputBarriers, 0, nullptr);

	// Copy outputs -> readbacks
	VkBufferCopy copyOutIntr{}; copyOutIntr.srcOffset = 0; copyOutIntr.dstOffset = 0; copyOutIntr.size = intrinsicsSize;
	vkCmdCopyBuffer(cmd, deviceIntrinsicsOut, readIntrinsics, 1, &copyOutIntr);
	VkBufferCopy copyOutPos{}; copyOutPos.srcOffset = 0; copyOutPos.dstOffset = 0; copyOutPos.size = posesSize;
	vkCmdCopyBuffer(cmd, devicePosesOut, readPoses, 1, &copyOutPos);

	if (vkEndCommandBuffer(cmd) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to record command buffer for camera test.");
	}

	VkFenceCreateInfo fenceInfo{}; fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
	VkFence fence = VK_NULL_HANDLE;
	if (vkCreateFence(m_device, &fenceInfo, nullptr, &fence) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to create fence for camera test.");
	}

	VkSubmitInfo submit{}; submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO; submit.commandBufferCount = 1; submit.pCommandBuffers = &cmd;
	if (vkQueueSubmit(m_queue, 1, &submit, fence) != VK_SUCCESS)
	{
		// ensure we clean up the fence if submit fails
		if (fence != VK_NULL_HANDLE) { vkDestroyFence(m_device, fence, nullptr); fence = VK_NULL_HANDLE; }
		throw std::runtime_error("Failed to submit camera test command buffer.");
	}

	if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS)
	{
		// wait failed: destroy fence then report
		if (fence != VK_NULL_HANDLE) { vkDestroyFence(m_device, fence, nullptr); fence = VK_NULL_HANDLE; }
		throw std::runtime_error("Failed to wait for fence for camera test.");
	}

	// Destroy fence once and mark handle null to avoid accidental double-destroy
	if (fence != VK_NULL_HANDLE) { vkDestroyFence(m_device, fence, nullptr); fence = VK_NULL_HANDLE; }

	// Map readbacks and compare
	void *mapped = nullptr;
	vkMapMemory(m_device, readIntrinsicsMem, 0, intrinsicsSize, 0, &mapped);
	GpuCameraIntrinsics *intrRead = reinterpret_cast<GpuCameraIntrinsics *>(mapped);

	bool intrAll = true;
	for (uint32_t i=0;i<kCount;++i)
	{
		const GpuCameraIntrinsics &exp = intrinsics[i];
		const GpuCameraIntrinsics &act = intrRead[i];
		if (exp.m_width != act.m_width || exp.m_height != act.m_height || fabs(exp.m_fx - act.m_fx) > 1e-6f || fabs(exp.m_fy - act.m_fy) > 1e-6f || fabs(exp.m_cx - act.m_cx) > 1e-6f || fabs(exp.m_cy - act.m_cy) > 1e-6f || fabs(exp.m_k1 - act.m_k1) > 1e-6f || fabs(exp.m_k2 - act.m_k2) > 1e-6f || fabs(exp.m_p1 - act.m_p1) > 1e-6f || fabs(exp.m_p2 - act.m_p2) > 1e-6f)
		{
			UIUtility::Print(std::string("Camera layout smoke test mismatch\nrecord: ") + std::to_string(i) + " type: GpuCameraIntrinsics\n");
			intrAll = false;
			break;
		}
	}
	vkUnmapMemory(m_device, readIntrinsicsMem);

	vkMapMemory(m_device, readPosesMem, 0, posesSize, 0, &mapped);
	GpuCameraPose *poseRead = reinterpret_cast<GpuCameraPose *>(mapped);
	bool poseAll = true;
	for (uint32_t i=0;i<kCount;++i)
	{
		const GpuCameraPose &exp = poses[i];
		const GpuCameraPose &act = poseRead[i];
		for (int m=0;m<4;++m)
		{
			if (fabs(exp.m_position[m] - act.m_position[m]) > 1e-6f)
			{
				UIUtility::Print(std::string("Camera layout smoke test mismatch\nrecord: ") + std::to_string(i) + " type: GpuCameraPose field: position[" + std::to_string(m) + "]\n");
				poseAll = false; break;
			}
			if (fabs(exp.m_forward[m] - act.m_forward[m]) > 1e-6f)
			{
				UIUtility::Print(std::string("Camera layout smoke test mismatch\nrecord: ") + std::to_string(i) + " type: GpuCameraPose field: forward[" + std::to_string(m) + "]\n");
				poseAll = false; break;
			}
			if (fabs(exp.m_up[m] - act.m_up[m]) > 1e-6f)
			{
				UIUtility::Print(std::string("Camera layout smoke test mismatch\nrecord: ") + std::to_string(i) + " type: GpuCameraPose field: up[" + std::to_string(m) + "]\n");
				poseAll = false; break;
			}
			if (fabs(exp.m_right[m] - act.m_right[m]) > 1e-6f)
			{
				UIUtility::Print(std::string("Camera layout smoke test mismatch\nrecord: ") + std::to_string(i) + " type: GpuCameraPose field: right[" + std::to_string(m) + "]\n");
				poseAll = false; break;
			}
		}
		if (!poseAll) break;
	}
	vkUnmapMemory(m_device, readPosesMem);

	if (intrAll && poseAll)
	{
		UIUtility::Print("PlaneSweep GPU camera-layout smoke test succeeded:\n3 / 3 intrinsics records matched.\n3 / 3 pose records matched.\n");
	}

	// Cleanup all temporary resources
	vkFreeCommandBuffers(m_device, m_commandPool, 1, &cmd);
	vkDestroyPipeline(m_device, pipeline, nullptr);
	vkDestroyPipelineLayout(m_device, pipelineLayout, nullptr);
	vkDestroyShaderModule(m_device, shaderModule, nullptr);
	vkDestroyDescriptorPool(m_device, descriptorPool, nullptr);
	vkDestroyDescriptorSetLayout(m_device, descriptorSetLayout, nullptr);

	vkFreeMemory(m_device, readPosesMem, nullptr); vkDestroyBuffer(m_device, readPoses, nullptr);
	vkFreeMemory(m_device, readIntrinsicsMem, nullptr); vkDestroyBuffer(m_device, readIntrinsics, nullptr);

	vkFreeMemory(m_device, devicePosesOutMem, nullptr); vkDestroyBuffer(m_device, devicePosesOut, nullptr);
	vkFreeMemory(m_device, deviceIntrinsicsOutMem, nullptr); vkDestroyBuffer(m_device, deviceIntrinsicsOut, nullptr);

	vkFreeMemory(m_device, devicePosesMem, nullptr); vkDestroyBuffer(m_device, devicePoses, nullptr);
	vkFreeMemory(m_device, deviceIntrinsicsMem, nullptr); vkDestroyBuffer(m_device, deviceIntrinsics, nullptr);

	vkFreeMemory(m_device, stagingPosesMem, nullptr); vkDestroyBuffer(m_device, stagingPoses, nullptr);
	vkFreeMemory(m_device, stagingIntrinsicsMem, nullptr); vkDestroyBuffer(m_device, stagingIntrinsics, nullptr);

	vkDestroyFence(m_device, fence, nullptr);

	return intrAll && poseAll;
}

bool PlaneSweepGpu::RunImageSamplingSmokeTest(void)
{
	if (!m_initialized)
	{
		throw std::runtime_error("PlaneSweepGpu not initialized.");
	}

	constexpr uint32_t kWidth = 4;
	constexpr uint32_t kHeight = 4;
	constexpr uint32_t kPixelCount = kWidth * kHeight;
	constexpr VkDeviceSize kPixelBytes = kPixelCount * 4;
	constexpr VkDeviceSize kOutputBytes = kPixelCount * 4 * sizeof(float);
	constexpr VkFormat kFormat = VK_FORMAT_R8G8B8A8_UNORM;

	VkFormatProperties formatProperties{};
	vkGetPhysicalDeviceFormatProperties(m_physicalDevice, kFormat, &formatProperties);
	const VkFormatFeatureFlags requiredFeatures = VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT | VK_FORMAT_FEATURE_TRANSFER_DST_BIT;
	if ((formatProperties.optimalTilingFeatures & requiredFeatures) != requiredFeatures)
	{
		UIUtility::Print("Image sampling smoke test unsupported: VK_FORMAT_R8G8B8A8_UNORM optimal tiling lacks sampled-image or transfer-destination support.\n");
		return false;
	}

	std::array<uint8_t, kPixelBytes> pixels{};
	for (uint32_t y = 0; y < kHeight; ++y)
	{
		for (uint32_t x = 0; x < kWidth; ++x)
		{
			const uint32_t index = y * kWidth + x;
			pixels[index * 4 + 0] = static_cast<uint8_t>(10 + x * 50);
			pixels[index * 4 + 1] = static_cast<uint8_t>(20 + y * 50);
			pixels[index * 4 + 2] = static_cast<uint8_t>(30 + index * 7);
			pixels[index * 4 + 3] = static_cast<uint8_t>(255 - index * 9);
		}
	}

	VkBuffer stagingBuffer = VK_NULL_HANDLE;
	VkDeviceMemory stagingMemory = VK_NULL_HANDLE;
	VkImage image = VK_NULL_HANDLE;
	VkDeviceMemory imageMemory = VK_NULL_HANDLE;
	VkImageView imageView = VK_NULL_HANDLE;
	VkSampler sampler = VK_NULL_HANDLE;
	VkBuffer deviceOutput = VK_NULL_HANDLE;
	VkDeviceMemory deviceOutputMemory = VK_NULL_HANDLE;
	VkBuffer readbackBuffer = VK_NULL_HANDLE;
	VkDeviceMemory readbackMemory = VK_NULL_HANDLE;
	VkDescriptorSetLayout descriptorSetLayout = VK_NULL_HANDLE;
	VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
	VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
	VkShaderModule shaderModule = VK_NULL_HANDLE;
	VkPipeline pipeline = VK_NULL_HANDLE;
	VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
	VkFence fence = VK_NULL_HANDLE;
	bool submitted = false;

	auto cleanup = [&]()
	{
		if (submitted)
		{
			vkDeviceWaitIdle(m_device);
			submitted = false;
		}
		if (fence != VK_NULL_HANDLE) { vkDestroyFence(m_device, fence, nullptr); fence = VK_NULL_HANDLE; }
		if (commandBuffer != VK_NULL_HANDLE) { vkFreeCommandBuffers(m_device, m_commandPool, 1, &commandBuffer); commandBuffer = VK_NULL_HANDLE; }
		if (pipeline != VK_NULL_HANDLE) { vkDestroyPipeline(m_device, pipeline, nullptr); pipeline = VK_NULL_HANDLE; }
		if (shaderModule != VK_NULL_HANDLE) { vkDestroyShaderModule(m_device, shaderModule, nullptr); shaderModule = VK_NULL_HANDLE; }
		if (pipelineLayout != VK_NULL_HANDLE) { vkDestroyPipelineLayout(m_device, pipelineLayout, nullptr); pipelineLayout = VK_NULL_HANDLE; }
		if (descriptorPool != VK_NULL_HANDLE) { vkDestroyDescriptorPool(m_device, descriptorPool, nullptr); descriptorPool = VK_NULL_HANDLE; }
		if (descriptorSetLayout != VK_NULL_HANDLE) { vkDestroyDescriptorSetLayout(m_device, descriptorSetLayout, nullptr); descriptorSetLayout = VK_NULL_HANDLE; }
		if (sampler != VK_NULL_HANDLE) { vkDestroySampler(m_device, sampler, nullptr); sampler = VK_NULL_HANDLE; }
		if (imageView != VK_NULL_HANDLE) { vkDestroyImageView(m_device, imageView, nullptr); imageView = VK_NULL_HANDLE; }
		if (image != VK_NULL_HANDLE) { vkDestroyImage(m_device, image, nullptr); image = VK_NULL_HANDLE; }
		if (imageMemory != VK_NULL_HANDLE) { vkFreeMemory(m_device, imageMemory, nullptr); imageMemory = VK_NULL_HANDLE; }
		if (deviceOutput != VK_NULL_HANDLE) { vkDestroyBuffer(m_device, deviceOutput, nullptr); deviceOutput = VK_NULL_HANDLE; }
		if (deviceOutputMemory != VK_NULL_HANDLE) { vkFreeMemory(m_device, deviceOutputMemory, nullptr); deviceOutputMemory = VK_NULL_HANDLE; }
		if (readbackBuffer != VK_NULL_HANDLE) { vkDestroyBuffer(m_device, readbackBuffer, nullptr); readbackBuffer = VK_NULL_HANDLE; }
		if (readbackMemory != VK_NULL_HANDLE) { vkFreeMemory(m_device, readbackMemory, nullptr); readbackMemory = VK_NULL_HANDLE; }
		if (stagingBuffer != VK_NULL_HANDLE) { vkDestroyBuffer(m_device, stagingBuffer, nullptr); stagingBuffer = VK_NULL_HANDLE; }
		if (stagingMemory != VK_NULL_HANDLE) { vkFreeMemory(m_device, stagingMemory, nullptr); stagingMemory = VK_NULL_HANDLE; }
	};

	auto findMemoryType = [&](uint32_t typeBits, VkMemoryPropertyFlags requiredFlags)
	{
		VkPhysicalDeviceMemoryProperties memoryProperties{};
		vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memoryProperties);
		for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; ++i)
		{
			if ((typeBits & (1u << i)) != 0 && (memoryProperties.memoryTypes[i].propertyFlags & requiredFlags) == requiredFlags)
			{
				return i;
			}
		}
		return UINT32_MAX;
	};

	auto createBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags memoryFlags, VkBuffer &buffer, VkDeviceMemory &memory)
	{
		VkBufferCreateInfo bufferInfo{};
		bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bufferInfo.size = size;
		bufferInfo.usage = usage;
		bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
		if (vkCreateBuffer(m_device, &bufferInfo, nullptr, &buffer) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to create image-sampling buffer.");
		}

		VkMemoryRequirements requirements{};
		vkGetBufferMemoryRequirements(m_device, buffer, &requirements);
		const uint32_t memoryType = findMemoryType(requirements.memoryTypeBits, memoryFlags);
		if (memoryType == UINT32_MAX)
		{
			throw std::runtime_error("No suitable memory type for image-sampling buffer.");
		}

		VkMemoryAllocateInfo allocation{};
		allocation.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		allocation.allocationSize = requirements.size;
		allocation.memoryTypeIndex = memoryType;
		if (vkAllocateMemory(m_device, &allocation, nullptr, &memory) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to allocate image-sampling buffer memory.");
		}
		if (vkBindBufferMemory(m_device, buffer, memory, 0) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to bind image-sampling buffer memory.");
		}
	};

	try
	{
		createBuffer(kPixelBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
			stagingBuffer, stagingMemory);
		void *mappedPixels = nullptr;
		if (vkMapMemory(m_device, stagingMemory, 0, kPixelBytes, 0, &mappedPixels) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to map image-sampling staging memory.");
		}
		std::memcpy(mappedPixels, pixels.data(), static_cast<size_t>(kPixelBytes));
		vkUnmapMemory(m_device, stagingMemory);

		VkImageCreateInfo imageInfo{};
		imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
		imageInfo.imageType = VK_IMAGE_TYPE_2D;
		imageInfo.format = kFormat;
		imageInfo.extent = VkExtent3D{ kWidth, kHeight, 1 };
		imageInfo.mipLevels = 1;
		imageInfo.arrayLayers = 1;
		imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
		imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
		imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
		imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
		imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		if (vkCreateImage(m_device, &imageInfo, nullptr, &image) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to create image-sampling image.");
		}

		VkMemoryRequirements imageRequirements{};
		vkGetImageMemoryRequirements(m_device, image, &imageRequirements);
		const uint32_t imageMemoryType = findMemoryType(imageRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
		if (imageMemoryType == UINT32_MAX)
		{
			throw std::runtime_error("No device-local memory type for image-sampling image.");
		}
		VkMemoryAllocateInfo imageAllocation{};
		imageAllocation.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		imageAllocation.allocationSize = imageRequirements.size;
		imageAllocation.memoryTypeIndex = imageMemoryType;
		if (vkAllocateMemory(m_device, &imageAllocation, nullptr, &imageMemory) != VK_SUCCESS || vkBindImageMemory(m_device, image, imageMemory, 0) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to allocate or bind image-sampling image memory.");
		}

		VkImageViewCreateInfo viewInfo{};
		viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
		viewInfo.image = image;
		viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
		viewInfo.format = kFormat;
		viewInfo.components = VkComponentMapping{};
		viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		viewInfo.subresourceRange.baseMipLevel = 0;
		viewInfo.subresourceRange.levelCount = 1;
		viewInfo.subresourceRange.baseArrayLayer = 0;
		viewInfo.subresourceRange.layerCount = 1;
		if (vkCreateImageView(m_device, &viewInfo, nullptr, &imageView) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to create image-sampling image view.");
		}

		VkSamplerCreateInfo samplerInfo{};
		samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
		samplerInfo.magFilter = VK_FILTER_NEAREST;
		samplerInfo.minFilter = VK_FILTER_NEAREST;
		samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
		samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
		samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
		samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
		samplerInfo.mipLodBias = 0.0f;
		samplerInfo.anisotropyEnable = VK_FALSE;
		samplerInfo.maxAnisotropy = 1.0f;
		samplerInfo.compareEnable = VK_FALSE;
		samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
		samplerInfo.minLod = 0.0f;
		samplerInfo.maxLod = 0.0f;
		samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK;
		samplerInfo.unnormalizedCoordinates = VK_FALSE;
		if (vkCreateSampler(m_device, &samplerInfo, nullptr, &sampler) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to create image-sampling sampler.");
		}

		createBuffer(kOutputBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, deviceOutput, deviceOutputMemory);
		createBuffer(kOutputBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
			readbackBuffer, readbackMemory);

		std::array<VkDescriptorSetLayoutBinding, 2> bindings{};
		bindings[0].binding = 0;
		bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		bindings[0].descriptorCount = 1;
		bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
		bindings[1].binding = 1;
		bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		bindings[1].descriptorCount = 1;
		bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
		VkDescriptorSetLayoutCreateInfo descriptorLayoutInfo{};
		descriptorLayoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
		descriptorLayoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
		descriptorLayoutInfo.pBindings = bindings.data();
		if (vkCreateDescriptorSetLayout(m_device, &descriptorLayoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to create image-sampling descriptor set layout.");
		}

		VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
		pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
		pipelineLayoutInfo.setLayoutCount = 1;
		pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout;
		if (vkCreatePipelineLayout(m_device, &pipelineLayoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to create image-sampling pipeline layout.");
		}

		const std::string shaderRelativePath = "shaders/planeSweepImageSamplingSmokeTest.comp.spv";
		const std::vector<char> shaderCode = ReadShaderFile(shaderRelativePath);
		if (shaderCode.empty())
		{
			throw std::runtime_error("Failed to load image-sampling compute shader SPIR-V.");
		}
		VkShaderModuleCreateInfo shaderInfo{};
		shaderInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
		shaderInfo.codeSize = shaderCode.size();
		shaderInfo.pCode = reinterpret_cast<const uint32_t *>(shaderCode.data());
		if (vkCreateShaderModule(m_device, &shaderInfo, nullptr, &shaderModule) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to create image-sampling shader module.");
		}

		VkPipelineShaderStageCreateInfo shaderStage{};
		shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
		shaderStage.module = shaderModule;
		shaderStage.pName = "main";
		VkComputePipelineCreateInfo pipelineInfo{};
		pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
		pipelineInfo.stage = shaderStage;
		pipelineInfo.layout = pipelineLayout;
		if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to create image-sampling compute pipeline.");
		}

		std::array<VkDescriptorPoolSize, 2> poolSizes{};
		poolSizes[0] = VkDescriptorPoolSize{ VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1 };
		poolSizes[1] = VkDescriptorPoolSize{ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1 };
		VkDescriptorPoolCreateInfo poolInfo{};
		poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
		poolInfo.maxSets = 1;
		poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
		poolInfo.pPoolSizes = poolSizes.data();
		if (vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to create image-sampling descriptor pool.");
		}
		VkDescriptorSetAllocateInfo descriptorAllocateInfo{};
		descriptorAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
		descriptorAllocateInfo.descriptorPool = descriptorPool;
		descriptorAllocateInfo.descriptorSetCount = 1;
		descriptorAllocateInfo.pSetLayouts = &descriptorSetLayout;
		VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
		if (vkAllocateDescriptorSets(m_device, &descriptorAllocateInfo, &descriptorSet) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to allocate image-sampling descriptor set.");
		}
		VkDescriptorImageInfo descriptorImageInfo{};
		descriptorImageInfo.sampler = sampler;
		descriptorImageInfo.imageView = imageView;
		descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		VkDescriptorBufferInfo descriptorBufferInfo{};
		descriptorBufferInfo.buffer = deviceOutput;
		descriptorBufferInfo.offset = 0;
		descriptorBufferInfo.range = kOutputBytes;
		std::array<VkWriteDescriptorSet, 2> descriptorWrites{};
		descriptorWrites[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		descriptorWrites[0].dstSet = descriptorSet;
		descriptorWrites[0].dstBinding = 0;
		descriptorWrites[0].descriptorCount = 1;
		descriptorWrites[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		descriptorWrites[0].pImageInfo = &descriptorImageInfo;
		descriptorWrites[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		descriptorWrites[1].dstSet = descriptorSet;
		descriptorWrites[1].dstBinding = 1;
		descriptorWrites[1].descriptorCount = 1;
		descriptorWrites[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		descriptorWrites[1].pBufferInfo = &descriptorBufferInfo;
		vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);

		VkCommandBufferAllocateInfo commandAllocateInfo{};
		commandAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		commandAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		commandAllocateInfo.commandPool = m_commandPool;
		commandAllocateInfo.commandBufferCount = 1;
		if (vkAllocateCommandBuffers(m_device, &commandAllocateInfo, &commandBuffer) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to allocate image-sampling command buffer.");
		}
		VkCommandBufferBeginInfo commandBeginInfo{};
		commandBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
		commandBeginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
		if (vkBeginCommandBuffer(commandBuffer, &commandBeginInfo) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to begin image-sampling command buffer.");
		}

		VkImageMemoryBarrier toTransfer{};
		toTransfer.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		toTransfer.srcAccessMask = 0;
		toTransfer.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		toTransfer.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		toTransfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		toTransfer.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		toTransfer.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		toTransfer.image = image;
		toTransfer.subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &toTransfer);

		VkBufferImageCopy imageCopy{};
		imageCopy.bufferOffset = 0;
		imageCopy.bufferRowLength = 0;
		imageCopy.bufferImageHeight = 0;
		imageCopy.imageSubresource = VkImageSubresourceLayers{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
		imageCopy.imageOffset = VkOffset3D{ 0, 0, 0 };
		imageCopy.imageExtent = VkExtent3D{ kWidth, kHeight, 1 };
		vkCmdCopyBufferToImage(commandBuffer, stagingBuffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &imageCopy);

		VkImageMemoryBarrier toShaderRead{};
		toShaderRead.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		toShaderRead.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		toShaderRead.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
		toShaderRead.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		toShaderRead.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		toShaderRead.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		toShaderRead.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		toShaderRead.image = image;
		toShaderRead.subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &toShaderRead);

		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
		vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
		vkCmdDispatch(commandBuffer, 1, 1, 1);

		VkBufferMemoryBarrier outputBarrier{};
		outputBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
		outputBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
		outputBarrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
		outputBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		outputBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		outputBarrier.buffer = deviceOutput;
		outputBarrier.offset = 0;
		outputBarrier.size = kOutputBytes;
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 1, &outputBarrier, 0, nullptr);

		VkBufferCopy outputCopy{};
		outputCopy.srcOffset = 0;
		outputCopy.dstOffset = 0;
		outputCopy.size = kOutputBytes;
		vkCmdCopyBuffer(commandBuffer, deviceOutput, readbackBuffer, 1, &outputCopy);
		if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to record image-sampling command buffer.");
		}

		VkFenceCreateInfo fenceInfo{};
		fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
		if (vkCreateFence(m_device, &fenceInfo, nullptr, &fence) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to create image-sampling fence.");
		}
		VkSubmitInfo submitInfo{};
		submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
		submitInfo.commandBufferCount = 1;
		submitInfo.pCommandBuffers = &commandBuffer;
		if (vkQueueSubmit(m_queue, 1, &submitInfo, fence) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to submit image-sampling command buffer.");
		}
		submitted = true;
		if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to wait for image-sampling fence.");
		}
		submitted = false;
		vkDestroyFence(m_device, fence, nullptr);
		fence = VK_NULL_HANDLE;

		void *mappedOutput = nullptr;
		if (vkMapMemory(m_device, readbackMemory, 0, kOutputBytes, 0, &mappedOutput) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to map image-sampling readback memory.");
		}
		const float *actualValues = static_cast<const float *>(mappedOutput);
		bool matched = true;
		constexpr float kTolerance = 1e-5f;
		constexpr const char *channelNames[] = { "R", "G", "B", "A" };
		for (uint32_t i = 0; i < kPixelCount && matched; ++i)
		{
			const uint32_t x = i % kWidth;
			const uint32_t y = i / kWidth;
			for (uint32_t channel = 0; channel < 4; ++channel)
			{
				const uint8_t original = pixels[i * 4 + channel];
				const float expected = static_cast<float>(original) / 255.0f;
				const float actual = actualValues[i * 4 + channel];
				const float error = std::fabs(expected - actual);
				if (error > kTolerance)
				{
					UIUtility::Print(std::string("PlaneSweep GPU image-sampling mismatch: pixel ") + std::to_string(i) + " (x=" + std::to_string(x) + ", y=" + std::to_string(y) + "), channel " + channelNames[channel] + ", original uint8=" + std::to_string(original) + ", expected=" + std::to_string(expected) + ", actual=" + std::to_string(actual) + ", absolute error=" + std::to_string(error) + ". Check row orientation, X/Y orientation, channel order, and UNORM normalization.\n");
					matched = false;
					break;
				}
			}
		}
		vkUnmapMemory(m_device, readbackMemory);

		if (matched)
		{
			UIUtility::Print("PlaneSweep GPU image-sampling smoke test succeeded:\n16 / 16 texel-center samples matched RGBA8_UNORM source data.\n");
		}
		cleanup();
		return matched;
	}
	catch (...)
	{
		cleanup();
		throw;
	}
}

bool PlaneSweepGpu::RunBilinearSamplingSmokeTest(void)
{
	if (!m_initialized)
	{
		throw std::runtime_error("PlaneSweepGpu not initialized.");
	}

	constexpr uint32_t kWidth = 4;
	constexpr uint32_t kHeight = 4;
	constexpr uint32_t kSampleCount = 16;
	constexpr VkDeviceSize kImageBytes = kWidth * kHeight * 4;
	constexpr VkDeviceSize kCoordinateBytes = kSampleCount * 2 * sizeof(float);
	constexpr VkDeviceSize kColorBytes = kSampleCount * 4 * sizeof(float);
	constexpr VkDeviceSize kValidityBytes = kSampleCount * sizeof(uint32_t);
	constexpr VkFormat kFormat = VK_FORMAT_R8G8B8A8_UNORM;

	struct PixelCoordinate { float x; float y; };
	const std::array<PixelCoordinate, kSampleCount> coordinates{{
		{ 0.0f, 0.0f }, { 3.0f, 3.0f }, { 1.0f, 2.0f }, { 1.5f, 1.5f },
		{ 0.25f, 0.75f }, { 2.75f, 0.125f }, { 0.0f, 2.5f }, { 3.0f, 1.5f },
		{ 1.25f, 3.0f }, { 2.999f, 2.001f }, { 0.001f, 0.001f }, { -0.001f, 1.0f },
		{ 1.0f, -0.001f }, { 3.001f, 1.0f }, { 1.0f, 3.001f }, { 4.0f, 4.0f }
	}};

	VkFormatProperties formatProperties{};
	vkGetPhysicalDeviceFormatProperties(m_physicalDevice, kFormat, &formatProperties);
	const VkFormatFeatureFlags requiredFeatures = VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT | VK_FORMAT_FEATURE_TRANSFER_DST_BIT;
	if ((formatProperties.optimalTilingFeatures & requiredFeatures) != requiredFeatures)
	{
		UIUtility::Print("Bilinear sampling smoke test unsupported: VK_FORMAT_R8G8B8A8_UNORM optimal tiling lacks sampled-image or transfer-destination support.\n");
		return false;
	}

	std::array<uint8_t, kImageBytes> pixels{};
	for (uint32_t y = 0; y < kHeight; ++y)
	{
		for (uint32_t x = 0; x < kWidth; ++x)
		{
			const uint32_t index = y * kWidth + x;
			pixels[index * 4 + 0] = static_cast<uint8_t>(10 + x * 50);
			pixels[index * 4 + 1] = static_cast<uint8_t>(20 + y * 50);
			pixels[index * 4 + 2] = static_cast<uint8_t>(30 + index * 7);
			pixels[index * 4 + 3] = static_cast<uint8_t>(255 - index * 9);
		}
	}

	std::array<float, kSampleCount * 2> coordinateFloats{};
	for (uint32_t i = 0; i < kSampleCount; ++i)
	{
		coordinateFloats[i * 2] = coordinates[i].x;
		coordinateFloats[i * 2 + 1] = coordinates[i].y;
	}

	VkBuffer imageStaging = VK_NULL_HANDLE, coordinateStaging = VK_NULL_HANDLE;
	VkDeviceMemory imageStagingMemory = VK_NULL_HANDLE, coordinateStagingMemory = VK_NULL_HANDLE;
	VkBuffer coordinateInput = VK_NULL_HANDLE, colorOutput = VK_NULL_HANDLE, validOutput = VK_NULL_HANDLE;
	VkDeviceMemory coordinateInputMemory = VK_NULL_HANDLE, colorOutputMemory = VK_NULL_HANDLE, validOutputMemory = VK_NULL_HANDLE;
	VkBuffer colorReadback = VK_NULL_HANDLE, validReadback = VK_NULL_HANDLE;
	VkDeviceMemory colorReadbackMemory = VK_NULL_HANDLE, validReadbackMemory = VK_NULL_HANDLE;
	VkImage image = VK_NULL_HANDLE;
	VkDeviceMemory imageMemory = VK_NULL_HANDLE;
	VkImageView imageView = VK_NULL_HANDLE;
	VkSampler sampler = VK_NULL_HANDLE;
	VkDescriptorSetLayout descriptorSetLayout = VK_NULL_HANDLE;
	VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
	VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
	VkShaderModule shaderModule = VK_NULL_HANDLE;
	VkPipeline pipeline = VK_NULL_HANDLE;
	VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
	VkFence fence = VK_NULL_HANDLE;
	bool submitted = false;

	auto cleanup = [&]()
	{
		if (submitted) { vkDeviceWaitIdle(m_device); submitted = false; }
		if (fence != VK_NULL_HANDLE) { vkDestroyFence(m_device, fence, nullptr); fence = VK_NULL_HANDLE; }
		if (commandBuffer != VK_NULL_HANDLE) { vkFreeCommandBuffers(m_device, m_commandPool, 1, &commandBuffer); commandBuffer = VK_NULL_HANDLE; }
		if (pipeline != VK_NULL_HANDLE) { vkDestroyPipeline(m_device, pipeline, nullptr); pipeline = VK_NULL_HANDLE; }
		if (shaderModule != VK_NULL_HANDLE) { vkDestroyShaderModule(m_device, shaderModule, nullptr); shaderModule = VK_NULL_HANDLE; }
		if (pipelineLayout != VK_NULL_HANDLE) { vkDestroyPipelineLayout(m_device, pipelineLayout, nullptr); pipelineLayout = VK_NULL_HANDLE; }
		if (descriptorPool != VK_NULL_HANDLE) { vkDestroyDescriptorPool(m_device, descriptorPool, nullptr); descriptorPool = VK_NULL_HANDLE; }
		if (descriptorSetLayout != VK_NULL_HANDLE) { vkDestroyDescriptorSetLayout(m_device, descriptorSetLayout, nullptr); descriptorSetLayout = VK_NULL_HANDLE; }
		if (sampler != VK_NULL_HANDLE) { vkDestroySampler(m_device, sampler, nullptr); sampler = VK_NULL_HANDLE; }
		if (imageView != VK_NULL_HANDLE) { vkDestroyImageView(m_device, imageView, nullptr); imageView = VK_NULL_HANDLE; }
		if (image != VK_NULL_HANDLE) { vkDestroyImage(m_device, image, nullptr); image = VK_NULL_HANDLE; }
		if (imageMemory != VK_NULL_HANDLE) { vkFreeMemory(m_device, imageMemory, nullptr); imageMemory = VK_NULL_HANDLE; }
		auto destroyBuffer = [&](VkBuffer &buffer, VkDeviceMemory &memory)
		{
			if (buffer != VK_NULL_HANDLE) { vkDestroyBuffer(m_device, buffer, nullptr); buffer = VK_NULL_HANDLE; }
			if (memory != VK_NULL_HANDLE) { vkFreeMemory(m_device, memory, nullptr); memory = VK_NULL_HANDLE; }
		};
		destroyBuffer(colorReadback, colorReadbackMemory);
		destroyBuffer(validReadback, validReadbackMemory);
		destroyBuffer(imageStaging, imageStagingMemory);
		destroyBuffer(coordinateStaging, coordinateStagingMemory);
		destroyBuffer(coordinateInput, coordinateInputMemory);
		destroyBuffer(colorOutput, colorOutputMemory);
		destroyBuffer(validOutput, validOutputMemory);
	};

	auto findMemoryType = [&](uint32_t typeBits, VkMemoryPropertyFlags requiredFlags)
	{
		VkPhysicalDeviceMemoryProperties properties{};
		vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &properties);
		for (uint32_t i = 0; i < properties.memoryTypeCount; ++i)
		{
			if ((typeBits & (1u << i)) != 0 && (properties.memoryTypes[i].propertyFlags & requiredFlags) == requiredFlags) return i;
		}
		return UINT32_MAX;
	};
	auto createBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer &buffer, VkDeviceMemory &memory)
	{
		VkBufferCreateInfo info{};
		info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		info.size = size;
		info.usage = usage;
		info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
		if (vkCreateBuffer(m_device, &info, nullptr, &buffer) != VK_SUCCESS) throw std::runtime_error("Failed to create bilinear test buffer.");
		VkMemoryRequirements requirements{};
		vkGetBufferMemoryRequirements(m_device, buffer, &requirements);
		const uint32_t type = findMemoryType(requirements.memoryTypeBits, properties);
		if (type == UINT32_MAX) throw std::runtime_error("No suitable memory type for bilinear test buffer.");
		VkMemoryAllocateInfo allocation{};
		allocation.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		allocation.allocationSize = requirements.size;
		allocation.memoryTypeIndex = type;
		if (vkAllocateMemory(m_device, &allocation, nullptr, &memory) != VK_SUCCESS) throw std::runtime_error("Failed to allocate bilinear test buffer memory.");
		if (vkBindBufferMemory(m_device, buffer, memory, 0) != VK_SUCCESS) throw std::runtime_error("Failed to bind bilinear test buffer memory.");
	};
	auto mapAndCopy = [&](VkDeviceMemory memory, VkDeviceSize size, const void *data)
	{
		void *mapped = nullptr;
		if (vkMapMemory(m_device, memory, 0, size, 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map bilinear test staging memory.");
		std::memcpy(mapped, data, static_cast<size_t>(size));
		vkUnmapMemory(m_device, memory);
	};

	try
	{
		const VkMemoryPropertyFlags hostFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
		createBuffer(kImageBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, hostFlags, imageStaging, imageStagingMemory);
		mapAndCopy(imageStagingMemory, kImageBytes, pixels.data());
		createBuffer(kCoordinateBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, hostFlags, coordinateStaging, coordinateStagingMemory);
		mapAndCopy(coordinateStagingMemory, kCoordinateBytes, coordinateFloats.data());
		createBuffer(kCoordinateBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, coordinateInput, coordinateInputMemory);
		createBuffer(kColorBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, colorOutput, colorOutputMemory);
		createBuffer(kValidityBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, validOutput, validOutputMemory);
		createBuffer(kColorBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, hostFlags, colorReadback, colorReadbackMemory);
		createBuffer(kValidityBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, hostFlags, validReadback, validReadbackMemory);

		VkImageCreateInfo imageInfo{};
		imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
		imageInfo.imageType = VK_IMAGE_TYPE_2D;
		imageInfo.format = kFormat;
		imageInfo.extent = VkExtent3D{ kWidth, kHeight, 1 };
		imageInfo.mipLevels = 1;
		imageInfo.arrayLayers = 1;
		imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
		imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
		imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
		imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
		imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		if (vkCreateImage(m_device, &imageInfo, nullptr, &image) != VK_SUCCESS) throw std::runtime_error("Failed to create bilinear test image.");
		VkMemoryRequirements imageRequirements{};
		vkGetImageMemoryRequirements(m_device, image, &imageRequirements);
		const uint32_t imageType = findMemoryType(imageRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
		if (imageType == UINT32_MAX) throw std::runtime_error("No device-local memory type for bilinear test image.");
		VkMemoryAllocateInfo imageAllocation{};
		imageAllocation.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		imageAllocation.allocationSize = imageRequirements.size;
		imageAllocation.memoryTypeIndex = imageType;
		if (vkAllocateMemory(m_device, &imageAllocation, nullptr, &imageMemory) != VK_SUCCESS || vkBindImageMemory(m_device, image, imageMemory, 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate or bind bilinear test image memory.");

		VkImageViewCreateInfo viewInfo{};
		viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
		viewInfo.image = image;
		viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
		viewInfo.format = kFormat;
		viewInfo.subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
		if (vkCreateImageView(m_device, &viewInfo, nullptr, &imageView) != VK_SUCCESS) throw std::runtime_error("Failed to create bilinear test image view.");
		VkSamplerCreateInfo samplerInfo{};
		samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
		samplerInfo.magFilter = VK_FILTER_NEAREST;
		samplerInfo.minFilter = VK_FILTER_NEAREST;
		samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
		samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
		samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
		samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
		samplerInfo.anisotropyEnable = VK_FALSE;
		samplerInfo.compareEnable = VK_FALSE;
		samplerInfo.minLod = 0.0f;
		samplerInfo.maxLod = 0.0f;
		samplerInfo.unnormalizedCoordinates = VK_FALSE;
		if (vkCreateSampler(m_device, &samplerInfo, nullptr, &sampler) != VK_SUCCESS) throw std::runtime_error("Failed to create bilinear test sampler.");

		std::array<VkDescriptorSetLayoutBinding, 4> bindings{};
		bindings[0] = VkDescriptorSetLayoutBinding{ 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
		for (uint32_t i = 1; i < 4; ++i) bindings[i] = VkDescriptorSetLayoutBinding{ i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
		VkDescriptorSetLayoutCreateInfo descriptorLayoutInfo{};
		descriptorLayoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
		descriptorLayoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
		descriptorLayoutInfo.pBindings = bindings.data();
		if (vkCreateDescriptorSetLayout(m_device, &descriptorLayoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create bilinear descriptor set layout.");
		VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
		pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
		pipelineLayoutInfo.setLayoutCount = 1;
		pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout;
		if (vkCreatePipelineLayout(m_device, &pipelineLayoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create bilinear pipeline layout.");

		const std::string shaderPath = "shaders/planeSweepBilinearSamplingSmokeTest.comp.spv";
		const std::vector<char> shaderCode = ReadShaderFile(shaderPath);
		if (shaderCode.empty()) throw std::runtime_error("Failed to load bilinear sampling shader SPIR-V.");
		VkShaderModuleCreateInfo shaderInfo{};
		shaderInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
		shaderInfo.codeSize = shaderCode.size();
		shaderInfo.pCode = reinterpret_cast<const uint32_t *>(shaderCode.data());
		if (vkCreateShaderModule(m_device, &shaderInfo, nullptr, &shaderModule) != VK_SUCCESS) throw std::runtime_error("Failed to create bilinear shader module.");
		VkPipelineShaderStageCreateInfo shaderStage{};
		shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
		shaderStage.module = shaderModule;
		shaderStage.pName = "main";
		VkComputePipelineCreateInfo pipelineInfo{};
		pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
		pipelineInfo.stage = shaderStage;
		pipelineInfo.layout = pipelineLayout;
		if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline) != VK_SUCCESS) throw std::runtime_error("Failed to create bilinear compute pipeline.");

		std::array<VkDescriptorPoolSize, 2> poolSizes{{
			{ VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1 },
			{ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 3 }
		}};
		VkDescriptorPoolCreateInfo poolInfo{};
		poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
		poolInfo.maxSets = 1;
		poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
		poolInfo.pPoolSizes = poolSizes.data();
		if (vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS) throw std::runtime_error("Failed to create bilinear descriptor pool.");
		VkDescriptorSetAllocateInfo descriptorAllocateInfo{};
		descriptorAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
		descriptorAllocateInfo.descriptorPool = descriptorPool;
		descriptorAllocateInfo.descriptorSetCount = 1;
		descriptorAllocateInfo.pSetLayouts = &descriptorSetLayout;
		VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
		if (vkAllocateDescriptorSets(m_device, &descriptorAllocateInfo, &descriptorSet) != VK_SUCCESS) throw std::runtime_error("Failed to allocate bilinear descriptor set.");
		VkDescriptorImageInfo imageDescriptor{};
		imageDescriptor.sampler = sampler;
		imageDescriptor.imageView = imageView;
		imageDescriptor.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		std::array<VkDescriptorBufferInfo, 3> bufferDescriptors{{
			{ coordinateInput, 0, kCoordinateBytes },
			{ colorOutput, 0, kColorBytes },
			{ validOutput, 0, kValidityBytes }
		}};
		std::array<VkWriteDescriptorSet, 4> writes{};
		writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writes[0].dstSet = descriptorSet;
		writes[0].dstBinding = 0;
		writes[0].descriptorCount = 1;
		writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		writes[0].pImageInfo = &imageDescriptor;
		for (uint32_t i = 1; i < 4; ++i)
		{
			writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writes[i].dstSet = descriptorSet;
			writes[i].dstBinding = i;
			writes[i].descriptorCount = 1;
			writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
			writes[i].pBufferInfo = &bufferDescriptors[i - 1];
		}
		vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);

		VkCommandBufferAllocateInfo commandAllocateInfo{};
		commandAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		commandAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		commandAllocateInfo.commandPool = m_commandPool;
		commandAllocateInfo.commandBufferCount = 1;
		if (vkAllocateCommandBuffers(m_device, &commandAllocateInfo, &commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to allocate bilinear command buffer.");
		VkCommandBufferBeginInfo commandBeginInfo{};
		commandBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
		commandBeginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
		if (vkBeginCommandBuffer(commandBuffer, &commandBeginInfo) != VK_SUCCESS) throw std::runtime_error("Failed to begin bilinear command buffer.");

		VkImageMemoryBarrier imageToTransfer{};
		imageToTransfer.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		imageToTransfer.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		imageToTransfer.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		imageToTransfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		imageToTransfer.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		imageToTransfer.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		imageToTransfer.image = image;
		imageToTransfer.subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &imageToTransfer);
		VkBufferImageCopy imageCopy{};
		imageCopy.imageSubresource = VkImageSubresourceLayers{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
		imageCopy.imageExtent = VkExtent3D{ kWidth, kHeight, 1 };
		vkCmdCopyBufferToImage(commandBuffer, imageStaging, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &imageCopy);
		VkImageMemoryBarrier imageToRead{};
		imageToRead.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		imageToRead.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		imageToRead.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
		imageToRead.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		imageToRead.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		imageToRead.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		imageToRead.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		imageToRead.image = image;
		imageToRead.subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &imageToRead);

		VkBufferCopy coordinateCopy{};
		coordinateCopy.size = kCoordinateBytes;
		vkCmdCopyBuffer(commandBuffer, coordinateStaging, coordinateInput, 1, &coordinateCopy);
		VkBufferMemoryBarrier coordinateBarrier{};
		coordinateBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
		coordinateBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		coordinateBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
		coordinateBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		coordinateBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		coordinateBarrier.buffer = coordinateInput;
		coordinateBarrier.size = kCoordinateBytes;
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 1, &coordinateBarrier, 0, nullptr);

		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
		vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
		vkCmdDispatch(commandBuffer, 1, 1, 1);

		VkBufferMemoryBarrier outputBarriers[2]{};
		outputBarriers[0].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
		outputBarriers[0].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
		outputBarriers[0].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
		outputBarriers[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		outputBarriers[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		outputBarriers[0].buffer = colorOutput;
		outputBarriers[0].size = kColorBytes;
		outputBarriers[1] = outputBarriers[0];
		outputBarriers[1].buffer = validOutput;
		outputBarriers[1].size = kValidityBytes;
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 2, outputBarriers, 0, nullptr);
		VkBufferCopy colorCopy{};
		colorCopy.size = kColorBytes;
		vkCmdCopyBuffer(commandBuffer, colorOutput, colorReadback, 1, &colorCopy);
		VkBufferCopy validCopy{};
		validCopy.size = kValidityBytes;
		vkCmdCopyBuffer(commandBuffer, validOutput, validReadback, 1, &validCopy);
		if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to record bilinear command buffer.");

		VkFenceCreateInfo fenceInfo{};
		fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
		if (vkCreateFence(m_device, &fenceInfo, nullptr, &fence) != VK_SUCCESS) throw std::runtime_error("Failed to create bilinear fence.");
		VkSubmitInfo submitInfo{};
		submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
		submitInfo.commandBufferCount = 1;
		submitInfo.pCommandBuffers = &commandBuffer;
		if (vkQueueSubmit(m_queue, 1, &submitInfo, fence) != VK_SUCCESS) throw std::runtime_error("Failed to submit bilinear command buffer.");
		submitted = true;
		if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS) throw std::runtime_error("Failed to wait for bilinear fence.");
		submitted = false;
		vkDestroyFence(m_device, fence, nullptr);
		fence = VK_NULL_HANDLE;

		std::array<bool, kSampleCount> expectedValid{};
		std::array<std::array<float, 4>, kSampleCount> expectedColors{};
		uint32_t validSampleCount = 0;
		for (uint32_t i = 0; i < kSampleCount; ++i)
		{
			const float x = coordinates[i].x;
			const float y = coordinates[i].y;
			if (x < 0.0f || y < 0.0f || x > static_cast<float>(kWidth - 1) || y > static_cast<float>(kHeight - 1)) continue;
			expectedValid[i] = true;
			++validSampleCount;
			const uint32_t x0 = static_cast<uint32_t>(std::floor(x));
			const uint32_t y0 = static_cast<uint32_t>(std::floor(y));
			const uint32_t x1 = std::min(kWidth - 1, x0 + 1);
			const uint32_t y1 = std::min(kHeight - 1, y0 + 1);
			const float tx = x - static_cast<float>(x0);
			const float ty = y - static_cast<float>(y0);
			for (uint32_t channel = 0; channel < 4; ++channel)
			{
				auto sample = [&](uint32_t sx, uint32_t sy)
				{
					const size_t index = (static_cast<size_t>(sy) * kWidth + sx) * 4 + channel;
					return static_cast<float>(pixels[index]) / 255.0f;
				};
				const float top = (sample(x0, y0) * (1.0f - tx)) + (sample(x1, y0) * tx);
				const float bottom = (sample(x0, y1) * (1.0f - tx)) + (sample(x1, y1) * tx);
				expectedColors[i][channel] = (top * (1.0f - ty)) + (bottom * ty);
			}
		}

		void *validMapped = nullptr;
		void *colorMapped = nullptr;
		if (vkMapMemory(m_device, validReadbackMemory, 0, kValidityBytes, 0, &validMapped) != VK_SUCCESS) throw std::runtime_error("Failed to map bilinear validity readback.");
		const uint32_t *actualValidity = static_cast<const uint32_t *>(validMapped);
		bool matched = true;
		for (uint32_t i = 0; i < kSampleCount; ++i)
		{
			const bool actualValid = actualValidity[i] != 0;
			if (expectedValid[i] != actualValid)
			{
				UIUtility::Print(std::string("Bilinear validity mismatch: sample ") + std::to_string(i) + " x=" + std::to_string(coordinates[i].x) + " y=" + std::to_string(coordinates[i].y) + " expected valid=" + (expectedValid[i] ? "true" : "false") + " actual valid=" + (actualValid ? "true" : "false") + "\n");
				matched = false;
				break;
			}
		}
		vkUnmapMemory(m_device, validReadbackMemory);

		float maximumAbsoluteError = 0.0f;
		if (matched)
		{
			if (vkMapMemory(m_device, colorReadbackMemory, 0, kColorBytes, 0, &colorMapped) != VK_SUCCESS) throw std::runtime_error("Failed to map bilinear color readback.");
			const float *actualColors = static_cast<const float *>(colorMapped);
			constexpr const char *channelNames[] = { "R", "G", "B", "A" };
			for (uint32_t i = 0; i < kSampleCount && matched; ++i)
			{
				if (!expectedValid[i]) continue;
				for (uint32_t channel = 0; channel < 4; ++channel)
				{
					const float actual = actualColors[i * 4 + channel];
					const float expected = expectedColors[i][channel];
					const float error = std::fabs(expected - actual);
					maximumAbsoluteError = std::max(maximumAbsoluteError, error);
					if (error > 1e-5f)
					{
						UIUtility::Print(std::string("Bilinear color mismatch: sample ") + std::to_string(i) + " x=" + std::to_string(coordinates[i].x) + " y=" + std::to_string(coordinates[i].y) + " channel=" + channelNames[channel] + " expected=" + std::to_string(expected) + " actual=" + std::to_string(actual) + " absolute error=" + std::to_string(error) + "\n");
						matched = false;
						break;
					}
				}
				if (matched && (coordinates[i].x != std::floor(coordinates[i].x) || coordinates[i].y != std::floor(coordinates[i].y)))
				{
					UIUtility::Print(std::string("Bilinear valid sample ") + std::to_string(i) + " CPU RGBA=(" + std::to_string(expectedColors[i][0]) + "," + std::to_string(expectedColors[i][1]) + "," + std::to_string(expectedColors[i][2]) + "," + std::to_string(expectedColors[i][3]) + ") GPU RGBA=(" + std::to_string(actualColors[i * 4]) + "," + std::to_string(actualColors[i * 4 + 1]) + "," + std::to_string(actualColors[i * 4 + 2]) + "," + std::to_string(actualColors[i * 4 + 3]) + ")\n");
				}
			}
			vkUnmapMemory(m_device, colorReadbackMemory);
		}

		if (matched)
		{
			UIUtility::Print(std::string("PlaneSweep GPU bilinear-sampling smoke test succeeded:\n16 / 16 validity results matched CPU SampleBilinear semantics.\n") + std::to_string(validSampleCount) + " / " + std::to_string(validSampleCount) + " valid RGBA samples matched CPU bilinear results.\n");
			UIUtility::Print(std::string("Maximum valid RGBA component absolute error: ") + std::to_string(maximumAbsoluteError) + "\n");
		}
		cleanup();
		return matched;
	}
	catch (...)
	{
		cleanup();
		throw;
	}
}

}
