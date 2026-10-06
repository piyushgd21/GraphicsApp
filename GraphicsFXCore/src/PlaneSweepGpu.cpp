#include "PlaneSweepGpu.h"

#include "GraphicsUtility.h"
#include "UIUtility.h"
#include "PathUtility.h"
#include "PlaneSweepGpuTypes.h"
#include "TextureUtility.h"

#include <Windows.h>
#include <array>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <chrono>

#include <stdexcept>
#include <cmath>

namespace graphicsFX
{

static uint32_t FloatBits(float value)
{
	uint32_t bits = 0;
	static_assert(sizeof(bits) == sizeof(value));
	std::memcpy(&bits, &value, sizeof(bits));
	return bits;
}

static float FloatFromBits(uint32_t bits)
{
	float value = 0.0f;
	std::memcpy(&value, &bits, sizeof(value));
	return value;
}

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

void PlaneSweepGpu::Initialize(VkDevice device, VkPhysicalDevice physicalDevice, VkQueue queue, VkCommandPool commandPool)
{
	if (m_initialized)
	{
		throw std::runtime_error("PlaneSweepGpu already initialized.");
	}

	m_device = device;
	m_physicalDevice = physicalDevice;
	m_queue = queue;
	m_commandPool = commandPool;
	m_shaderSampledImageArrayDynamicIndexingEnabled = false;

	m_initialized = true;
}

bool PlaneSweepGpu::ShaderSampledImageArrayDynamicIndexingEnabled(void) const
{
	return m_shaderSampledImageArrayDynamicIndexingEnabled;
}

bool PlaneSweepGpu::RunMultiSourceDescriptorSmokeTest(void)
{
	if (!m_initialized) throw std::runtime_error("PlaneSweepGpu not initialized.");

	constexpr uint32_t kSourceCount = MAX_PLANE_SWEEP_SOURCE_IMAGES;
	constexpr uint32_t kImageCount = 2 + 2 * kSourceCount;
	constexpr VkDeviceSize kOutputBytes = 18 * 4 * sizeof(float);
	constexpr VkFormat kFormat = VK_FORMAT_R8G8B8A8_UNORM;
	const std::array<std::array<uint8_t, 4>, kImageCount> colors{{
		{{11, 12, 13, 14}}, {{21, 22, 23, 24}},
		{{31, 32, 33, 34}}, {{41, 42, 43, 44}}, {{51, 52, 53, 54}}, {{61, 62, 63, 64}},
		{{71, 72, 73, 74}}, {{81, 82, 83, 84}}, {{91, 92, 93, 94}}, {{101, 102, 103, 104}},
		{{111, 112, 113, 114}}, {{121, 122, 123, 124}}, {{131, 132, 133, 134}}, {{141, 142, 143, 144}},
		{{151, 152, 153, 154}}, {{161, 162, 163, 164}}, {{171, 172, 173, 174}}, {{181, 182, 183, 184}}
	}};

	VkPhysicalDeviceFeatures supportedFeatures{};
	vkGetPhysicalDeviceFeatures(m_physicalDevice, &supportedFeatures);
	VkPhysicalDeviceProperties properties{};
	vkGetPhysicalDeviceProperties(m_physicalDevice, &properties);
	const VkPhysicalDeviceLimits &limits = properties.limits;
	const bool supported = supportedFeatures.shaderSampledImageArrayDynamicIndexing == VK_TRUE;
	const bool enabled = ShaderSampledImageArrayDynamicIndexingEnabled();
	UIUtility::Print("PlaneSweep GPU descriptor limits for " + std::string(properties.deviceName) + ":\n");
	UIUtility::Print("maxPerStageDescriptorSamplers: " + std::to_string(limits.maxPerStageDescriptorSamplers) + "\n");
	UIUtility::Print("maxPerStageDescriptorSampledImages: " + std::to_string(limits.maxPerStageDescriptorSampledImages) + "\n");
	UIUtility::Print("maxDescriptorSetSamplers: " + std::to_string(limits.maxDescriptorSetSamplers) + "\n");
	UIUtility::Print("maxDescriptorSetSampledImages: " + std::to_string(limits.maxDescriptorSetSampledImages) + "\n");
	UIUtility::Print("maxPerStageDescriptorStorageBuffers: " + std::to_string(limits.maxPerStageDescriptorStorageBuffers) + "\n");
	UIUtility::Print("maxDescriptorSetStorageBuffers: " + std::to_string(limits.maxDescriptorSetStorageBuffers) + "\n");
	UIUtility::Print("maxPerStageResources: " + std::to_string(limits.maxPerStageResources) + "\n");
	UIUtility::Print(std::string("shaderSampledImageArrayDynamicIndexing supported: ") + (supported ? "yes\n" : "no\n"));
	UIUtility::Print(std::string("shaderSampledImageArrayDynamicIndexing enabled by logical device: ") + (enabled ? "yes\n" : "no\n"));

	bool limitsSufficient = true;
	auto checkLimit = [&](const char *name, uint32_t actual, uint32_t required)
	{
		if (actual < required)
		{
			UIUtility::Print(std::string("PlaneSweep GPU descriptor smoke test stopped: limiting property ") + name + "=" + std::to_string(actual) + ", required=" + std::to_string(required) + ".\n");
			limitsSufficient = false;
		}
	};
	checkLimit("maxPerStageDescriptorSamplers", limits.maxPerStageDescriptorSamplers, 18);
	checkLimit("maxPerStageDescriptorSampledImages", limits.maxPerStageDescriptorSampledImages, 18);
	checkLimit("maxDescriptorSetSamplers", limits.maxDescriptorSetSamplers, 18);
	checkLimit("maxDescriptorSetSampledImages", limits.maxDescriptorSetSampledImages, 18);
	checkLimit("maxPerStageDescriptorStorageBuffers", limits.maxPerStageDescriptorStorageBuffers, 1);
	checkLimit("maxDescriptorSetStorageBuffers", limits.maxDescriptorSetStorageBuffers, 1);
	checkLimit("maxPerStageResources", limits.maxPerStageResources, 19);
	if (!limitsSufficient) return false;

	VkFormatProperties formatProperties{};
	vkGetPhysicalDeviceFormatProperties(m_physicalDevice, kFormat, &formatProperties);
	const VkFormatFeatureFlags requiredFormatFeatures = VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT | VK_FORMAT_FEATURE_TRANSFER_DST_BIT;
	if ((formatProperties.optimalTilingFeatures & requiredFormatFeatures) != requiredFormatFeatures)
	{
		UIUtility::Print("PlaneSweep GPU descriptor smoke test stopped: VK_FORMAT_R8G8B8A8_UNORM lacks optimal sampled-image or transfer-destination support.\n");
		return false;
	}

	VkBuffer stagingBuffer = VK_NULL_HANDLE, outputBuffer = VK_NULL_HANDLE, readbackBuffer = VK_NULL_HANDLE;
	VkDeviceMemory stagingMemory = VK_NULL_HANDLE, outputMemory = VK_NULL_HANDLE, readbackMemory = VK_NULL_HANDLE;
	std::array<VkImage, kImageCount> images{};
	std::array<VkDeviceMemory, kImageCount> imageMemories{};
	std::array<VkImageView, kImageCount> imageViews{};
	VkSampler sampler = VK_NULL_HANDLE;
	VkDescriptorSetLayout descriptorSetLayout = VK_NULL_HANDLE;
	VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
	VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
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
		if (descriptorPool != VK_NULL_HANDLE) { vkDestroyDescriptorPool(m_device, descriptorPool, nullptr); descriptorPool = VK_NULL_HANDLE; descriptorSet = VK_NULL_HANDLE; }
		if (descriptorSetLayout != VK_NULL_HANDLE) { vkDestroyDescriptorSetLayout(m_device, descriptorSetLayout, nullptr); descriptorSetLayout = VK_NULL_HANDLE; }
		if (sampler != VK_NULL_HANDLE) { vkDestroySampler(m_device, sampler, nullptr); sampler = VK_NULL_HANDLE; }
		for (uint32_t i = 0; i < kImageCount; ++i)
		{
			if (imageViews[i] != VK_NULL_HANDLE) { vkDestroyImageView(m_device, imageViews[i], nullptr); imageViews[i] = VK_NULL_HANDLE; }
			if (images[i] != VK_NULL_HANDLE) { vkDestroyImage(m_device, images[i], nullptr); images[i] = VK_NULL_HANDLE; }
			if (imageMemories[i] != VK_NULL_HANDLE) { vkFreeMemory(m_device, imageMemories[i], nullptr); imageMemories[i] = VK_NULL_HANDLE; }
		}
		auto destroyBuffer = [&](VkBuffer &buffer, VkDeviceMemory &memory)
		{
			if (buffer != VK_NULL_HANDLE) { vkDestroyBuffer(m_device, buffer, nullptr); buffer = VK_NULL_HANDLE; }
			if (memory != VK_NULL_HANDLE) { vkFreeMemory(m_device, memory, nullptr); memory = VK_NULL_HANDLE; }
		};
		destroyBuffer(stagingBuffer, stagingMemory);
		destroyBuffer(outputBuffer, outputMemory);
		destroyBuffer(readbackBuffer, readbackMemory);
	};
	auto findMemoryType = [&](uint32_t bits, VkMemoryPropertyFlags required)
	{
		VkPhysicalDeviceMemoryProperties memoryProperties{};
		vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memoryProperties);
		for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; ++i)
			if ((bits & (1u << i)) && (memoryProperties.memoryTypes[i].propertyFlags & required) == required) return i;
		return UINT32_MAX;
	};
	auto createBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags flags, VkBuffer &buffer, VkDeviceMemory &memory)
	{
		VkBufferCreateInfo info{}; info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO; info.size = size; info.usage = usage; info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
		if (vkCreateBuffer(m_device, &info, nullptr, &buffer) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source descriptor buffer.");
		VkMemoryRequirements requirements{}; vkGetBufferMemoryRequirements(m_device, buffer, &requirements);
		const uint32_t type = findMemoryType(requirements.memoryTypeBits, flags);
		if (type == UINT32_MAX) throw std::runtime_error("No suitable memory type for multi-source descriptor buffer.");
		VkMemoryAllocateInfo allocation{}; allocation.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; allocation.allocationSize = requirements.size; allocation.memoryTypeIndex = type;
		if (vkAllocateMemory(m_device, &allocation, nullptr, &memory) != VK_SUCCESS || vkBindBufferMemory(m_device, buffer, memory, 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate or bind multi-source descriptor buffer memory.");
	};
	try
	{
		constexpr VkDeviceSize stagingBytes = kImageCount * 4;
		createBuffer(stagingBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingMemory);
		void *mapped = nullptr;
		if (vkMapMemory(m_device, stagingMemory, 0, stagingBytes, 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map synthetic image staging memory.");
		std::memcpy(mapped, colors.data(), static_cast<size_t>(stagingBytes));
		vkUnmapMemory(m_device, stagingMemory);

		for (uint32_t i = 0; i < kImageCount; ++i)
		{
			VkImageCreateInfo imageInfo{}; imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO; imageInfo.imageType = VK_IMAGE_TYPE_2D; imageInfo.format = kFormat;
			imageInfo.extent = VkExtent3D{ 1, 1, 1 }; imageInfo.mipLevels = 1; imageInfo.arrayLayers = 1; imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
			imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL; imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT; imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE; imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			if (vkCreateImage(m_device, &imageInfo, nullptr, &images[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create synthetic descriptor image.");
			VkMemoryRequirements requirements{}; vkGetImageMemoryRequirements(m_device, images[i], &requirements);
			const uint32_t type = findMemoryType(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
			if (type == UINT32_MAX) throw std::runtime_error("No device-local memory for synthetic descriptor image.");
			VkMemoryAllocateInfo allocation{}; allocation.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; allocation.allocationSize = requirements.size; allocation.memoryTypeIndex = type;
			if (vkAllocateMemory(m_device, &allocation, nullptr, &imageMemories[i]) != VK_SUCCESS || vkBindImageMemory(m_device, images[i], imageMemories[i], 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate or bind synthetic descriptor image memory.");
			VkImageViewCreateInfo view{}; view.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO; view.image = images[i]; view.viewType = VK_IMAGE_VIEW_TYPE_2D; view.format = kFormat; view.subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
			if (vkCreateImageView(m_device, &view, nullptr, &imageViews[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create synthetic descriptor image view.");
		}
		VkSamplerCreateInfo samplerInfo{}; samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO; samplerInfo.magFilter = VK_FILTER_NEAREST; samplerInfo.minFilter = VK_FILTER_NEAREST; samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
		samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE; samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE; samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
		samplerInfo.anisotropyEnable = VK_FALSE; samplerInfo.compareEnable = VK_FALSE; samplerInfo.minLod = 0.0f; samplerInfo.maxLod = 0.0f; samplerInfo.unnormalizedCoordinates = VK_FALSE;
		if (vkCreateSampler(m_device, &samplerInfo, nullptr, &sampler) != VK_SUCCESS) throw std::runtime_error("Failed to create synthetic descriptor sampler.");

		std::array<VkDescriptorSetLayoutBinding, 5> bindings{};
		for (uint32_t i = 0; i < bindings.size(); ++i) { bindings[i].binding = i; bindings[i].descriptorType = i == 4 ? VK_DESCRIPTOR_TYPE_STORAGE_BUFFER : VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; bindings[i].descriptorCount = (i == 2 || i == 3) ? kSourceCount : 1; bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT; }
		VkDescriptorSetLayoutCreateInfo setLayoutInfo{}; setLayoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO; setLayoutInfo.bindingCount = static_cast<uint32_t>(bindings.size()); setLayoutInfo.pBindings = bindings.data();
		if (vkCreateDescriptorSetLayout(m_device, &setLayoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source descriptor set layout.");
		std::array<VkDescriptorPoolSize, 2> poolSizes{{ { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 18 }, { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1 } }};
		VkDescriptorPoolCreateInfo poolInfo{}; poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO; poolInfo.maxSets = 1; poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size()); poolInfo.pPoolSizes = poolSizes.data();
		if (vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source descriptor pool.");
		VkDescriptorSetAllocateInfo setAllocate{}; setAllocate.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO; setAllocate.descriptorPool = descriptorPool; setAllocate.descriptorSetCount = 1; setAllocate.pSetLayouts = &descriptorSetLayout;
		if (vkAllocateDescriptorSets(m_device, &setAllocate, &descriptorSet) != VK_SUCCESS) throw std::runtime_error("Failed to allocate multi-source descriptor set.");

		createBuffer(kOutputBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, outputBuffer, outputMemory);
		createBuffer(kOutputBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, readbackBuffer, readbackMemory);
		std::array<VkDescriptorImageInfo, 2> referenceInfos{};
		for (uint32_t i = 0; i < 2; ++i) { referenceInfos[i].sampler = sampler; referenceInfos[i].imageView = imageViews[i]; referenceInfos[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; }
		std::array<VkDescriptorImageInfo, kSourceCount> sourceImageInfos{}, sourceMaskInfos{};
		for (uint32_t i = 0; i < kSourceCount; ++i)
		{
			sourceImageInfos[i] = VkDescriptorImageInfo{ sampler, imageViews[2 + i], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
			sourceMaskInfos[i] = VkDescriptorImageInfo{ sampler, imageViews[10 + i], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
		}
		VkDescriptorBufferInfo outputInfo{ outputBuffer, 0, kOutputBytes };
		std::array<VkWriteDescriptorSet, 5> writes{};
		for (uint32_t i = 0; i < 5; ++i) { writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; writes[i].dstSet = descriptorSet; writes[i].dstBinding = i; writes[i].descriptorType = i == 4 ? VK_DESCRIPTOR_TYPE_STORAGE_BUFFER : VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; }
		writes[0].descriptorCount = 1; writes[0].pImageInfo = &referenceInfos[0];
		writes[1].descriptorCount = 1; writes[1].pImageInfo = &referenceInfos[1];
		writes[2].dstArrayElement = 0; writes[2].descriptorCount = kSourceCount; writes[2].pImageInfo = sourceImageInfos.data();
		writes[3].dstArrayElement = 0; writes[3].descriptorCount = kSourceCount; writes[3].pImageInfo = sourceMaskInfos.data();
		writes[4].descriptorCount = 1; writes[4].pBufferInfo = &outputInfo;
		vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);

		const std::vector<char> shaderCode = ReadShaderFile("shaders/planeSweepMultiSourceDescriptorSmokeTest.comp.spv");
		if (shaderCode.empty()) throw std::runtime_error("Failed to load multi-source descriptor smoke-test SPIR-V.");
		VkShaderModuleCreateInfo shaderInfo{}; shaderInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO; shaderInfo.codeSize = shaderCode.size(); shaderInfo.pCode = reinterpret_cast<const uint32_t *>(shaderCode.data());
		if (vkCreateShaderModule(m_device, &shaderInfo, nullptr, &shaderModule) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source descriptor shader module.");
		VkPipelineLayoutCreateInfo layoutInfo{}; layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO; layoutInfo.setLayoutCount = 1; layoutInfo.pSetLayouts = &descriptorSetLayout;
		if (vkCreatePipelineLayout(m_device, &layoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source descriptor pipeline layout.");
		VkPipelineShaderStageCreateInfo stage{}; stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = shaderModule; stage.pName = "main";
		VkComputePipelineCreateInfo pipelineInfo{}; pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO; pipelineInfo.stage = stage; pipelineInfo.layout = pipelineLayout;
		if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source descriptor compute pipeline.");

		VkCommandBufferAllocateInfo commandAllocate{}; commandAllocate.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO; commandAllocate.commandPool = m_commandPool; commandAllocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; commandAllocate.commandBufferCount = 1;
		if (vkAllocateCommandBuffers(m_device, &commandAllocate, &commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to allocate multi-source descriptor command buffer.");
		VkCommandBufferBeginInfo commandBegin{}; commandBegin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO; commandBegin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
		if (vkBeginCommandBuffer(commandBuffer, &commandBegin) != VK_SUCCESS) throw std::runtime_error("Failed to begin multi-source descriptor command buffer.");
		std::array<VkImageMemoryBarrier, kImageCount> toTransfer{};
		for (uint32_t i = 0; i < kImageCount; ++i) { toTransfer[i].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; toTransfer[i].dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; toTransfer[i].oldLayout = VK_IMAGE_LAYOUT_UNDEFINED; toTransfer[i].newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; toTransfer[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toTransfer[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toTransfer[i].image = images[i]; toTransfer[i].subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 }; }
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, kImageCount, toTransfer.data());
		for (uint32_t i = 0; i < kImageCount; ++i)
		{
			VkBufferImageCopy copy{}; copy.bufferOffset = i * 4; copy.imageSubresource = VkImageSubresourceLayers{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 }; copy.imageExtent = VkExtent3D{ 1, 1, 1 };
			vkCmdCopyBufferToImage(commandBuffer, stagingBuffer, images[i], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
		}
		std::array<VkImageMemoryBarrier, kImageCount> toShaderRead{};
		for (uint32_t i = 0; i < kImageCount; ++i) { toShaderRead[i].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; toShaderRead[i].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; toShaderRead[i].dstAccessMask = VK_ACCESS_SHADER_READ_BIT; toShaderRead[i].oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; toShaderRead[i].newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; toShaderRead[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toShaderRead[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toShaderRead[i].image = images[i]; toShaderRead[i].subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 }; }
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 0, nullptr, kImageCount, toShaderRead.data());
		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
		vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
		vkCmdDispatch(commandBuffer, 1, 1, 1);
		VkBufferMemoryBarrier outputBarrier{}; outputBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; outputBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT; outputBarrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT; outputBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; outputBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; outputBarrier.buffer = outputBuffer; outputBarrier.size = kOutputBytes;
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 1, &outputBarrier, 0, nullptr);
		VkBufferCopy outputCopy{}; outputCopy.size = kOutputBytes; vkCmdCopyBuffer(commandBuffer, outputBuffer, readbackBuffer, 1, &outputCopy);
		VkBufferMemoryBarrier readbackBarrier{}; readbackBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; readbackBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; readbackBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT; readbackBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; readbackBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; readbackBarrier.buffer = readbackBuffer; readbackBarrier.size = kOutputBytes;
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 0, nullptr, 1, &readbackBarrier, 0, nullptr);
		if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to end multi-source descriptor command buffer.");
		VkFenceCreateInfo fenceInfo{}; fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
		if (vkCreateFence(m_device, &fenceInfo, nullptr, &fence) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source descriptor fence.");
		VkSubmitInfo submit{}; submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO; submit.commandBufferCount = 1; submit.pCommandBuffers = &commandBuffer;
		if (vkQueueSubmit(m_queue, 1, &submit, fence) != VK_SUCCESS) throw std::runtime_error("Failed to submit multi-source descriptor command buffer.");
		submitted = true;
		if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS) throw std::runtime_error("Failed to wait for multi-source descriptor fence.");
		submitted = false;
		vkDestroyFence(m_device, fence, nullptr); fence = VK_NULL_HANDLE;
		std::array<float, 18 * 4> actual{};
		if (vkMapMemory(m_device, readbackMemory, 0, kOutputBytes, 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map multi-source descriptor readback memory.");
		std::memcpy(actual.data(), mapped, static_cast<size_t>(kOutputBytes));
		vkUnmapMemory(m_device, readbackMemory);
		const char *categories[kImageCount]{ "reference image", "reference mask", "source image", "source image", "source image", "source image", "source image", "source image", "source image", "source image", "source mask", "source mask", "source mask", "source mask", "source mask", "source mask", "source mask", "source mask" };
		for (uint32_t entry = 0; entry < kImageCount; ++entry)
		{
			for (uint32_t channel = 0; channel < 4; ++channel)
				if (std::fabs(actual[entry * 4 + channel] - static_cast<float>(colors[entry][channel]) / 255.0f) > 1e-6f)
				{
					const uint32_t sourceIndex = entry >= 10 ? entry - 10 : (entry >= 2 ? entry - 2 : 0);
					UIUtility::Print(std::string("PlaneSweep GPU multi-source descriptor mismatch: category=") + categories[entry] + (entry >= 2 ? ", source index=" + std::to_string(sourceIndex) : std::string()) + ", channel=" + std::to_string(channel) + ", expected uint8=" + std::to_string(colors[entry][channel]) + ", expected normalized=" + std::to_string(static_cast<float>(colors[entry][channel]) / 255.0f) + ", actual GPU=" + std::to_string(actual[entry * 4 + channel]) + ", absolute error=" + std::to_string(std::fabs(actual[entry * 4 + channel] - static_cast<float>(colors[entry][channel]) / 255.0f)) + ".\n");
					cleanup();
					return false;
				}
		}
		cleanup();
		UIUtility::Print("PlaneSweep GPU multi-source descriptor smoke test succeeded:\nreference image descriptor matched.\nreference mask descriptor matched.\n8 / 8 source image descriptors matched.\n8 / 8 source mask descriptors matched.\nStatic switch-based source selection succeeded without dynamic descriptor indexing.\n");
		return true;
	}
	catch (const std::exception &ex)
	{
		cleanup();
		UIUtility::Print(std::string("PlaneSweep GPU multi-source descriptor smoke test failed: ") + ex.what() + "\n");
		return false;
	}
}

bool PlaneSweepGpu::RunUnprojectPixelSmokeTest(
	const std::vector<GpuCameraIntrinsics> &intrinsics,
	const std::vector<GpuCameraPose> &poses,
	const std::vector<uint32_t> &cameraIndices,
	const std::vector<float> &sampleValues,
	const std::vector<float> &expectedPositions)
{
	if (!m_initialized) throw std::runtime_error("PlaneSweepGpu not initialized.");
	constexpr uint32_t kSampleCount = 16;
	if (intrinsics.size() != 3 || poses.size() != 3 || cameraIndices.size() != kSampleCount ||
		sampleValues.size() != kSampleCount * 4 || expectedPositions.size() != kSampleCount * 3)
		throw std::invalid_argument("UnprojectPixel smoke test requires exactly 3 cameras and 16 complete cases.");

	const std::array<VkDeviceSize, 4> inputSizes{{
		static_cast<VkDeviceSize>(intrinsics.size() * sizeof(GpuCameraIntrinsics)),
		static_cast<VkDeviceSize>(poses.size() * sizeof(GpuCameraPose)),
		static_cast<VkDeviceSize>(sampleValues.size() * sizeof(float)),
		static_cast<VkDeviceSize>(cameraIndices.size() * sizeof(uint32_t))
	}};
	const std::array<const void *, 4> inputData{{ intrinsics.data(), poses.data(), sampleValues.data(), cameraIndices.data() }};
	constexpr VkDeviceSize outputBytes = kSampleCount * 4 * sizeof(float);
	std::array<VkBuffer, 4> stagingBuffers{};
	std::array<VkDeviceMemory, 4> stagingMemories{};
	std::array<VkBuffer, 4> inputBuffers{};
	std::array<VkDeviceMemory, 4> inputMemories{};
	VkBuffer outputBuffer = VK_NULL_HANDLE, readbackBuffer = VK_NULL_HANDLE;
	VkDeviceMemory outputMemory = VK_NULL_HANDLE, readbackMemory = VK_NULL_HANDLE;
	VkDescriptorSetLayout descriptorSetLayout = VK_NULL_HANDLE;
	VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
	VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
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
		auto destroyBuffer = [&](VkBuffer &buffer, VkDeviceMemory &memory)
		{
			if (buffer != VK_NULL_HANDLE) { vkDestroyBuffer(m_device, buffer, nullptr); buffer = VK_NULL_HANDLE; }
			if (memory != VK_NULL_HANDLE) { vkFreeMemory(m_device, memory, nullptr); memory = VK_NULL_HANDLE; }
		};
		for (uint32_t i = 0; i < 4; ++i) { destroyBuffer(stagingBuffers[i], stagingMemories[i]); destroyBuffer(inputBuffers[i], inputMemories[i]); }
		destroyBuffer(outputBuffer, outputMemory);
		destroyBuffer(readbackBuffer, readbackMemory);
	};
	auto findMemoryType = [&](uint32_t bits, VkMemoryPropertyFlags required)
	{
		VkPhysicalDeviceMemoryProperties memoryProperties{}; vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memoryProperties);
		for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; ++i)
			if ((bits & (1u << i)) && (memoryProperties.memoryTypes[i].propertyFlags & required) == required) return i;
		return UINT32_MAX;
	};
	auto createBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags flags, VkBuffer &buffer, VkDeviceMemory &memory)
	{
		VkBufferCreateInfo info{}; info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO; info.size = size; info.usage = usage; info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
		if (vkCreateBuffer(m_device, &info, nullptr, &buffer) != VK_SUCCESS) throw std::runtime_error("Failed to create UnprojectPixel buffer.");
		VkMemoryRequirements requirements{}; vkGetBufferMemoryRequirements(m_device, buffer, &requirements);
		const uint32_t type = findMemoryType(requirements.memoryTypeBits, flags);
		if (type == UINT32_MAX) throw std::runtime_error("No suitable memory type for UnprojectPixel buffer.");
		VkMemoryAllocateInfo allocation{}; allocation.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; allocation.allocationSize = requirements.size; allocation.memoryTypeIndex = type;
		if (vkAllocateMemory(m_device, &allocation, nullptr, &memory) != VK_SUCCESS || vkBindBufferMemory(m_device, buffer, memory, 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate or bind UnprojectPixel buffer memory.");
	};
	try
	{
		const VkMemoryPropertyFlags host = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
		for (uint32_t i = 0; i < 4; ++i)
		{
			createBuffer(inputSizes[i], VK_BUFFER_USAGE_TRANSFER_SRC_BIT, host, stagingBuffers[i], stagingMemories[i]);
			void *mapped = nullptr;
			if (vkMapMemory(m_device, stagingMemories[i], 0, inputSizes[i], 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map UnprojectPixel staging memory.");
			std::memcpy(mapped, inputData[i], static_cast<size_t>(inputSizes[i])); vkUnmapMemory(m_device, stagingMemories[i]);
			createBuffer(inputSizes[i], VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, inputBuffers[i], inputMemories[i]);
		}
		createBuffer(outputBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, outputBuffer, outputMemory);
		createBuffer(outputBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, host, readbackBuffer, readbackMemory);

		std::array<VkDescriptorSetLayoutBinding, 5> bindings{};
		for (uint32_t i = 0; i < 5; ++i) { bindings[i].binding = i; bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; bindings[i].descriptorCount = 1; bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT; }
		VkDescriptorSetLayoutCreateInfo setLayoutInfo{}; setLayoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO; setLayoutInfo.bindingCount = static_cast<uint32_t>(bindings.size()); setLayoutInfo.pBindings = bindings.data();
		if (vkCreateDescriptorSetLayout(m_device, &setLayoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create UnprojectPixel descriptor layout.");
		VkDescriptorPoolSize poolSize{ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 5 };
		VkDescriptorPoolCreateInfo poolInfo{}; poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO; poolInfo.maxSets = 1; poolInfo.poolSizeCount = 1; poolInfo.pPoolSizes = &poolSize;
		if (vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS) throw std::runtime_error("Failed to create UnprojectPixel descriptor pool.");
		VkDescriptorSetAllocateInfo setAllocate{}; setAllocate.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO; setAllocate.descriptorPool = descriptorPool; setAllocate.descriptorSetCount = 1; setAllocate.pSetLayouts = &descriptorSetLayout;
		if (vkAllocateDescriptorSets(m_device, &setAllocate, &descriptorSet) != VK_SUCCESS) throw std::runtime_error("Failed to allocate UnprojectPixel descriptor set.");
		std::array<VkDescriptorBufferInfo, 5> bufferInfos{};
		for (uint32_t i = 0; i < 4; ++i) bufferInfos[i] = VkDescriptorBufferInfo{ inputBuffers[i], 0, inputSizes[i] };
		bufferInfos[4] = VkDescriptorBufferInfo{ outputBuffer, 0, outputBytes };
		std::array<VkWriteDescriptorSet, 5> writes{};
		for (uint32_t i = 0; i < 5; ++i) { writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; writes[i].dstSet = descriptorSet; writes[i].dstBinding = i; writes[i].descriptorCount = 1; writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; writes[i].pBufferInfo = &bufferInfos[i]; }
		vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);

		const std::vector<char> shaderCode = ReadShaderFile("shaders/planeSweepUnprojectPixelSmokeTest.comp.spv");
		if (shaderCode.empty()) throw std::runtime_error("Failed to load UnprojectPixel smoke-test SPIR-V.");
		VkShaderModuleCreateInfo shaderInfo{}; shaderInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO; shaderInfo.codeSize = shaderCode.size(); shaderInfo.pCode = reinterpret_cast<const uint32_t *>(shaderCode.data());
		if (vkCreateShaderModule(m_device, &shaderInfo, nullptr, &shaderModule) != VK_SUCCESS) throw std::runtime_error("Failed to create UnprojectPixel shader module.");
		VkPipelineLayoutCreateInfo layoutInfo{}; layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO; layoutInfo.setLayoutCount = 1; layoutInfo.pSetLayouts = &descriptorSetLayout;
		if (vkCreatePipelineLayout(m_device, &layoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create UnprojectPixel pipeline layout.");
		VkPipelineShaderStageCreateInfo stage{}; stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = shaderModule; stage.pName = "main";
		VkComputePipelineCreateInfo pipelineInfo{}; pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO; pipelineInfo.stage = stage; pipelineInfo.layout = pipelineLayout;
		if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline) != VK_SUCCESS) throw std::runtime_error("Failed to create UnprojectPixel compute pipeline.");

		VkCommandBufferAllocateInfo commandAllocate{}; commandAllocate.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO; commandAllocate.commandPool = m_commandPool; commandAllocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; commandAllocate.commandBufferCount = 1;
		if (vkAllocateCommandBuffers(m_device, &commandAllocate, &commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to allocate UnprojectPixel command buffer.");
		VkCommandBufferBeginInfo commandBegin{}; commandBegin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO; commandBegin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
		if (vkBeginCommandBuffer(commandBuffer, &commandBegin) != VK_SUCCESS) throw std::runtime_error("Failed to begin UnprojectPixel command buffer.");
		for (uint32_t i = 0; i < 4; ++i) { VkBufferCopy copy{}; copy.size = inputSizes[i]; vkCmdCopyBuffer(commandBuffer, stagingBuffers[i], inputBuffers[i], 1, &copy); }
		std::array<VkBufferMemoryBarrier, 4> inputBarriers{};
		for (uint32_t i = 0; i < 4; ++i) { inputBarriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; inputBarriers[i].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; inputBarriers[i].dstAccessMask = VK_ACCESS_SHADER_READ_BIT; inputBarriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; inputBarriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; inputBarriers[i].buffer = inputBuffers[i]; inputBarriers[i].size = inputSizes[i]; }
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, static_cast<uint32_t>(inputBarriers.size()), inputBarriers.data(), 0, nullptr);
		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
		vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
		vkCmdDispatch(commandBuffer, 1, 1, 1);
		VkBufferMemoryBarrier outputBarrier{}; outputBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; outputBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT; outputBarrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT; outputBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; outputBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; outputBarrier.buffer = outputBuffer; outputBarrier.size = outputBytes;
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 1, &outputBarrier, 0, nullptr);
		VkBufferCopy outputCopy{}; outputCopy.size = outputBytes; vkCmdCopyBuffer(commandBuffer, outputBuffer, readbackBuffer, 1, &outputCopy);
		VkBufferMemoryBarrier hostBarrier{}; hostBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; hostBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; hostBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT; hostBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; hostBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; hostBarrier.buffer = readbackBuffer; hostBarrier.size = outputBytes;
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 0, nullptr, 1, &hostBarrier, 0, nullptr);
		if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to end UnprojectPixel command buffer.");
		VkFenceCreateInfo fenceInfo{}; fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
		if (vkCreateFence(m_device, &fenceInfo, nullptr, &fence) != VK_SUCCESS) throw std::runtime_error("Failed to create UnprojectPixel fence.");
		VkSubmitInfo submit{}; submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO; submit.commandBufferCount = 1; submit.pCommandBuffers = &commandBuffer;
		if (vkQueueSubmit(m_queue, 1, &submit, fence) != VK_SUCCESS) throw std::runtime_error("Failed to submit UnprojectPixel command buffer.");
		submitted = true;
		if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS) throw std::runtime_error("Failed to wait for UnprojectPixel fence.");
		submitted = false;
		vkDestroyFence(m_device, fence, nullptr); fence = VK_NULL_HANDLE;
		std::array<float, kSampleCount * 4> actual{};
		void *mapped = nullptr;
		if (vkMapMemory(m_device, readbackMemory, 0, outputBytes, 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map UnprojectPixel readback memory.");
		std::memcpy(actual.data(), mapped, static_cast<size_t>(outputBytes)); vkUnmapMemory(m_device, readbackMemory);
		float maximumError = 0.0f;
		for (uint32_t i = 0; i < kSampleCount; ++i)
		{
			std::array<float, 3> errors{};
			bool matched = actual[i * 4 + 3] == 0.0f;
			for (uint32_t component = 0; component < 3; ++component)
			{
				errors[component] = std::fabs(actual[i * 4 + component] - expectedPositions[i * 3 + component]);
				maximumError = std::max(maximumError, errors[component]);
				matched = matched && errors[component] <= 1e-5f;
			}
			if (!matched)
			{
				const size_t offset = static_cast<size_t>(i) * 4;
				UIUtility::Print("UnprojectPixel mismatch: test=" + std::to_string(i) + ", camera=" + std::to_string(cameraIndices[i]) + ", x=" + std::to_string(sampleValues[offset]) + ", y=" + std::to_string(sampleValues[offset + 1]) + ", depth=" + std::to_string(sampleValues[offset + 2]) +
					" CPU=(" + std::to_string(expectedPositions[i * 3]) + "," + std::to_string(expectedPositions[i * 3 + 1]) + "," + std::to_string(expectedPositions[i * 3 + 2]) + ") GPU=(" + std::to_string(actual[i * 4]) + "," + std::to_string(actual[i * 4 + 1]) + "," + std::to_string(actual[i * 4 + 2]) + ") errors=(" + std::to_string(errors[0]) + "," + std::to_string(errors[1]) + "," + std::to_string(errors[2]) + ") w=" + std::to_string(actual[i * 4 + 3]) + ".\n");
				cleanup();
				return false;
			}
		}
		cleanup();
		UIUtility::Print("PlaneSweep GPU UnprojectPixel smoke test succeeded:\n16 / 16 world positions matched CPU UnprojectPixel.\nMaximum world-position component absolute error: " + std::to_string(maximumError) + "\n");
		return true;
	}
	catch (const std::exception &ex)
	{
		cleanup();
		UIUtility::Print(std::string("PlaneSweep GPU UnprojectPixel smoke test failed: ") + ex.what() + "\n");
		return false;
	}
}

bool PlaneSweepGpu::RunProjectPointSmokeTest(
	const std::vector<GpuCameraIntrinsics> &intrinsics,
	const std::vector<GpuCameraPose> &poses,
	const std::vector<uint32_t> &cameraIndices,
	const std::vector<float> &worldPoints,
	const std::vector<uint32_t> &expectedValidity,
	const std::vector<float> &expectedProjections)
{
	if (!m_initialized) throw std::runtime_error("PlaneSweepGpu not initialized.");
	constexpr uint32_t kSampleCount = 16;
	if (intrinsics.size() != 3 || poses.size() != 3 || cameraIndices.size() != kSampleCount || worldPoints.size() != kSampleCount * 4 ||
		expectedValidity.size() != kSampleCount || expectedProjections.size() != kSampleCount * 3)
		throw std::invalid_argument("ProjectPoint smoke test requires exactly 3 cameras and 16 complete cases.");

	const std::array<VkDeviceSize, 4> inputSizes{{
		static_cast<VkDeviceSize>(intrinsics.size() * sizeof(GpuCameraIntrinsics)),
		static_cast<VkDeviceSize>(poses.size() * sizeof(GpuCameraPose)),
		static_cast<VkDeviceSize>(worldPoints.size() * sizeof(float)),
		static_cast<VkDeviceSize>(cameraIndices.size() * sizeof(uint32_t))
	}};
	const std::array<const void *, 4> inputData{{ intrinsics.data(), poses.data(), worldPoints.data(), cameraIndices.data() }};
	constexpr VkDeviceSize projectionBytes = kSampleCount * 4 * sizeof(float);
	constexpr VkDeviceSize validityBytes = kSampleCount * sizeof(uint32_t);
	std::array<VkBuffer, 4> stagingBuffers{}, inputBuffers{};
	std::array<VkDeviceMemory, 4> stagingMemories{}, inputMemories{};
	VkBuffer projectionOutput = VK_NULL_HANDLE, validityOutput = VK_NULL_HANDLE, projectionReadback = VK_NULL_HANDLE, validityReadback = VK_NULL_HANDLE;
	VkDeviceMemory projectionOutputMemory = VK_NULL_HANDLE, validityOutputMemory = VK_NULL_HANDLE, projectionReadbackMemory = VK_NULL_HANDLE, validityReadbackMemory = VK_NULL_HANDLE;
	VkDescriptorSetLayout descriptorSetLayout = VK_NULL_HANDLE;
	VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
	VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
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
		auto destroyBuffer = [&](VkBuffer &buffer, VkDeviceMemory &memory)
		{
			if (buffer != VK_NULL_HANDLE) { vkDestroyBuffer(m_device, buffer, nullptr); buffer = VK_NULL_HANDLE; }
			if (memory != VK_NULL_HANDLE) { vkFreeMemory(m_device, memory, nullptr); memory = VK_NULL_HANDLE; }
		};
		for (uint32_t i = 0; i < 4; ++i) { destroyBuffer(stagingBuffers[i], stagingMemories[i]); destroyBuffer(inputBuffers[i], inputMemories[i]); }
		destroyBuffer(projectionOutput, projectionOutputMemory); destroyBuffer(validityOutput, validityOutputMemory);
		destroyBuffer(projectionReadback, projectionReadbackMemory); destroyBuffer(validityReadback, validityReadbackMemory);
	};
	auto findMemoryType = [&](uint32_t bits, VkMemoryPropertyFlags required)
	{
		VkPhysicalDeviceMemoryProperties memoryProperties{}; vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memoryProperties);
		for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; ++i)
			if ((bits & (1u << i)) && (memoryProperties.memoryTypes[i].propertyFlags & required) == required) return i;
		return UINT32_MAX;
	};
	auto createBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags flags, VkBuffer &buffer, VkDeviceMemory &memory)
	{
		VkBufferCreateInfo info{}; info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO; info.size = size; info.usage = usage; info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
		if (vkCreateBuffer(m_device, &info, nullptr, &buffer) != VK_SUCCESS) throw std::runtime_error("Failed to create ProjectPoint buffer.");
		VkMemoryRequirements requirements{}; vkGetBufferMemoryRequirements(m_device, buffer, &requirements);
		const uint32_t type = findMemoryType(requirements.memoryTypeBits, flags);
		if (type == UINT32_MAX) throw std::runtime_error("No suitable memory type for ProjectPoint buffer.");
		VkMemoryAllocateInfo allocation{}; allocation.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; allocation.allocationSize = requirements.size; allocation.memoryTypeIndex = type;
		if (vkAllocateMemory(m_device, &allocation, nullptr, &memory) != VK_SUCCESS || vkBindBufferMemory(m_device, buffer, memory, 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate or bind ProjectPoint buffer memory.");
	};
	try
	{
		const VkMemoryPropertyFlags host = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
		for (uint32_t i = 0; i < 4; ++i)
		{
			createBuffer(inputSizes[i], VK_BUFFER_USAGE_TRANSFER_SRC_BIT, host, stagingBuffers[i], stagingMemories[i]);
			void *mapped = nullptr;
			if (vkMapMemory(m_device, stagingMemories[i], 0, inputSizes[i], 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map ProjectPoint staging memory.");
			std::memcpy(mapped, inputData[i], static_cast<size_t>(inputSizes[i])); vkUnmapMemory(m_device, stagingMemories[i]);
			createBuffer(inputSizes[i], VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, inputBuffers[i], inputMemories[i]);
		}
		createBuffer(projectionBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, projectionOutput, projectionOutputMemory);
		createBuffer(validityBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, validityOutput, validityOutputMemory);
		createBuffer(projectionBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, host, projectionReadback, projectionReadbackMemory);
		createBuffer(validityBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, host, validityReadback, validityReadbackMemory);

		std::array<VkDescriptorSetLayoutBinding, 6> bindings{};
		for (uint32_t i = 0; i < 6; ++i) { bindings[i].binding = i; bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; bindings[i].descriptorCount = 1; bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT; }
		VkDescriptorSetLayoutCreateInfo setLayoutInfo{}; setLayoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO; setLayoutInfo.bindingCount = static_cast<uint32_t>(bindings.size()); setLayoutInfo.pBindings = bindings.data();
		if (vkCreateDescriptorSetLayout(m_device, &setLayoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create ProjectPoint descriptor layout.");
		VkDescriptorPoolSize poolSize{ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 6 };
		VkDescriptorPoolCreateInfo poolInfo{}; poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO; poolInfo.maxSets = 1; poolInfo.poolSizeCount = 1; poolInfo.pPoolSizes = &poolSize;
		if (vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS) throw std::runtime_error("Failed to create ProjectPoint descriptor pool.");
		VkDescriptorSetAllocateInfo setAllocate{}; setAllocate.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO; setAllocate.descriptorPool = descriptorPool; setAllocate.descriptorSetCount = 1; setAllocate.pSetLayouts = &descriptorSetLayout;
		if (vkAllocateDescriptorSets(m_device, &setAllocate, &descriptorSet) != VK_SUCCESS) throw std::runtime_error("Failed to allocate ProjectPoint descriptor set.");
		std::array<VkDescriptorBufferInfo, 6> bufferInfos{};
		for (uint32_t i = 0; i < 4; ++i) bufferInfos[i] = VkDescriptorBufferInfo{ inputBuffers[i], 0, inputSizes[i] };
		bufferInfos[4] = VkDescriptorBufferInfo{ projectionOutput, 0, projectionBytes };
		bufferInfos[5] = VkDescriptorBufferInfo{ validityOutput, 0, validityBytes };
		std::array<VkWriteDescriptorSet, 6> writes{};
		for (uint32_t i = 0; i < 6; ++i) { writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; writes[i].dstSet = descriptorSet; writes[i].dstBinding = i; writes[i].descriptorCount = 1; writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; writes[i].pBufferInfo = &bufferInfos[i]; }
		vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);

		const std::vector<char> shaderCode = ReadShaderFile("shaders/planeSweepProjectPointSmokeTest.comp.spv");
		if (shaderCode.empty()) throw std::runtime_error("Failed to load ProjectPoint smoke-test SPIR-V.");
		VkShaderModuleCreateInfo shaderInfo{}; shaderInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO; shaderInfo.codeSize = shaderCode.size(); shaderInfo.pCode = reinterpret_cast<const uint32_t *>(shaderCode.data());
		if (vkCreateShaderModule(m_device, &shaderInfo, nullptr, &shaderModule) != VK_SUCCESS) throw std::runtime_error("Failed to create ProjectPoint shader module.");
		VkPipelineLayoutCreateInfo layoutInfo{}; layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO; layoutInfo.setLayoutCount = 1; layoutInfo.pSetLayouts = &descriptorSetLayout;
		if (vkCreatePipelineLayout(m_device, &layoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create ProjectPoint pipeline layout.");
		VkPipelineShaderStageCreateInfo stage{}; stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = shaderModule; stage.pName = "main";
		VkComputePipelineCreateInfo pipelineInfo{}; pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO; pipelineInfo.stage = stage; pipelineInfo.layout = pipelineLayout;
		if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline) != VK_SUCCESS) throw std::runtime_error("Failed to create ProjectPoint compute pipeline.");

		VkCommandBufferAllocateInfo commandAllocate{}; commandAllocate.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO; commandAllocate.commandPool = m_commandPool; commandAllocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; commandAllocate.commandBufferCount = 1;
		if (vkAllocateCommandBuffers(m_device, &commandAllocate, &commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to allocate ProjectPoint command buffer.");
		VkCommandBufferBeginInfo commandBegin{}; commandBegin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO; commandBegin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
		if (vkBeginCommandBuffer(commandBuffer, &commandBegin) != VK_SUCCESS) throw std::runtime_error("Failed to begin ProjectPoint command buffer.");
		for (uint32_t i = 0; i < 4; ++i) { VkBufferCopy copy{}; copy.size = inputSizes[i]; vkCmdCopyBuffer(commandBuffer, stagingBuffers[i], inputBuffers[i], 1, &copy); }
		std::array<VkBufferMemoryBarrier, 4> inputBarriers{};
		for (uint32_t i = 0; i < 4; ++i) { inputBarriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; inputBarriers[i].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; inputBarriers[i].dstAccessMask = VK_ACCESS_SHADER_READ_BIT; inputBarriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; inputBarriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; inputBarriers[i].buffer = inputBuffers[i]; inputBarriers[i].size = inputSizes[i]; }
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, static_cast<uint32_t>(inputBarriers.size()), inputBarriers.data(), 0, nullptr);
		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
		vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
		vkCmdDispatch(commandBuffer, 1, 1, 1);
		std::array<VkBufferMemoryBarrier, 2> outputBarriers{};
		for (uint32_t i = 0; i < 2; ++i) { outputBarriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; outputBarriers[i].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT; outputBarriers[i].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT; outputBarriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; outputBarriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; }
		outputBarriers[0].buffer = projectionOutput; outputBarriers[0].size = projectionBytes; outputBarriers[1].buffer = validityOutput; outputBarriers[1].size = validityBytes;
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, static_cast<uint32_t>(outputBarriers.size()), outputBarriers.data(), 0, nullptr);
		VkBufferCopy projectionCopy{}; projectionCopy.size = projectionBytes; vkCmdCopyBuffer(commandBuffer, projectionOutput, projectionReadback, 1, &projectionCopy);
		VkBufferCopy validityCopy{}; validityCopy.size = validityBytes; vkCmdCopyBuffer(commandBuffer, validityOutput, validityReadback, 1, &validityCopy);
		std::array<VkBufferMemoryBarrier, 2> hostBarriers{};
		for (uint32_t i = 0; i < 2; ++i) { hostBarriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; hostBarriers[i].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; hostBarriers[i].dstAccessMask = VK_ACCESS_HOST_READ_BIT; hostBarriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; hostBarriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; }
		hostBarriers[0].buffer = projectionReadback; hostBarriers[0].size = projectionBytes; hostBarriers[1].buffer = validityReadback; hostBarriers[1].size = validityBytes;
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 0, nullptr, static_cast<uint32_t>(hostBarriers.size()), hostBarriers.data(), 0, nullptr);
		if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to end ProjectPoint command buffer.");
		VkFenceCreateInfo fenceInfo{}; fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
		if (vkCreateFence(m_device, &fenceInfo, nullptr, &fence) != VK_SUCCESS) throw std::runtime_error("Failed to create ProjectPoint fence.");
		VkSubmitInfo submit{}; submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO; submit.commandBufferCount = 1; submit.pCommandBuffers = &commandBuffer;
		if (vkQueueSubmit(m_queue, 1, &submit, fence) != VK_SUCCESS) throw std::runtime_error("Failed to submit ProjectPoint command buffer.");
		submitted = true;
		if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS) throw std::runtime_error("Failed to wait for ProjectPoint fence.");
		submitted = false;
		vkDestroyFence(m_device, fence, nullptr); fence = VK_NULL_HANDLE;
		std::array<float, kSampleCount * 4> actualProjections{};
		std::array<uint32_t, kSampleCount> actualValidity{};
		void *mapped = nullptr;
		if (vkMapMemory(m_device, projectionReadbackMemory, 0, projectionBytes, 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map ProjectPoint projection readback.");
		std::memcpy(actualProjections.data(), mapped, static_cast<size_t>(projectionBytes)); vkUnmapMemory(m_device, projectionReadbackMemory);
		if (vkMapMemory(m_device, validityReadbackMemory, 0, validityBytes, 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map ProjectPoint validity readback.");
		std::memcpy(actualValidity.data(), mapped, static_cast<size_t>(validityBytes)); vkUnmapMemory(m_device, validityReadbackMemory);

		for (uint32_t i = 0; i < kSampleCount; ++i)
		{
			const size_t pointOffset = static_cast<size_t>(i) * 4;
			const bool cpuValid = expectedValidity[i] != 0;
			const bool gpuValid = actualValidity[i] != 0;
			if (cpuValid != gpuValid)
			{
				UIUtility::Print("ProjectPoint validity mismatch: test=" + std::to_string(i) + ", camera=" + std::to_string(cameraIndices[i]) + ", point=(" + std::to_string(worldPoints[pointOffset]) + "," + std::to_string(worldPoints[pointOffset + 1]) + "," + std::to_string(worldPoints[pointOffset + 2]) + "), CPU valid=" + (cpuValid ? "true" : "false") + ", GPU valid=" + (gpuValid ? "true" : "false") + ", CPU depth=" + std::to_string(expectedProjections[i * 3 + 2]) + ", GPU depth=" + std::to_string(actualProjections[i * 4 + 2]) + ".\n");
				cleanup();
				return false;
			}
		}
		float maximumXError = 0.0f, maximumYError = 0.0f, maximumDepthError = 0.0f;
		uint32_t validCount = 0;
		bool outOfImageValidCase = false;
		const auto pixelTolerance = [](float cpuValue, float gpuValue)
		{
			return std::max(
				1e-4f,
				2.0f * std::numeric_limits<float>::epsilon() *
					std::max(std::fabs(cpuValue), std::fabs(gpuValue)));
		};
		for (uint32_t i = 0; i < kSampleCount; ++i)
		{
			if (expectedValidity[i] == 0) continue;
			++validCount;
			const float cpuX = expectedProjections[i * 3];
			const float gpuX = actualProjections[i * 4];
			const float cpuY = expectedProjections[i * 3 + 1];
			const float gpuY = actualProjections[i * 4 + 1];
			const float xError = std::fabs(gpuX - cpuX);
			const float yError = std::fabs(gpuY - cpuY);
			const float toleranceX = pixelTolerance(cpuX, gpuX);
			const float toleranceY = pixelTolerance(cpuY, gpuY);
			const float depthError = std::fabs(actualProjections[i * 4 + 2] - expectedProjections[i * 3 + 2]);
			maximumXError = std::max(maximumXError, xError); maximumYError = std::max(maximumYError, yError); maximumDepthError = std::max(maximumDepthError, depthError);
			if (i == 8) UIUtility::Print("ProjectPoint tolerance diagnostic:\nerrorY=" + std::to_string(yError) + "\ntoleranceY=" + std::to_string(toleranceY) + "\n");
			const GpuCameraIntrinsics &intrinsic = intrinsics[cameraIndices[i]];
			if (expectedProjections[i * 3] < 0.0f || expectedProjections[i * 3] > static_cast<float>(intrinsic.m_width - 1) ||
				expectedProjections[i * 3 + 1] < 0.0f || expectedProjections[i * 3 + 1] > static_cast<float>(intrinsic.m_height - 1)) outOfImageValidCase = true;
			if (xError > toleranceX || yError > toleranceY || depthError > 1e-5f || actualProjections[i * 4 + 3] != 0.0f)
			{
				UIUtility::Print("ProjectPoint projection mismatch: test=" + std::to_string(i) + ", camera=" + std::to_string(cameraIndices[i]) + ", point=(" + std::to_string(worldPoints[i * 4]) + "," + std::to_string(worldPoints[i * 4 + 1]) + "," + std::to_string(worldPoints[i * 4 + 2]) + "), CPU=(" + std::to_string(expectedProjections[i * 3]) + "," + std::to_string(expectedProjections[i * 3 + 1]) + "," + std::to_string(expectedProjections[i * 3 + 2]) + "), GPU=(" + std::to_string(actualProjections[i * 4]) + "," + std::to_string(actualProjections[i * 4 + 1]) + "," + std::to_string(actualProjections[i * 4 + 2]) + "), errors=(" + std::to_string(xError) + "," + std::to_string(yError) + "," + std::to_string(depthError) + ").\n");
				cleanup();
				return false;
			}
		}
		for (uint32_t i = 0; i < kSampleCount; ++i)
		{
			if (expectedValidity[i] != 0)
			{
				UIUtility::Print("ProjectPoint case " + std::to_string(i) + ", camera=" + std::to_string(cameraIndices[i]) + ", point=(" + std::to_string(worldPoints[i * 4]) + "," + std::to_string(worldPoints[i * 4 + 1]) + "," + std::to_string(worldPoints[i * 4 + 2]) + "), CPU valid=true projection=(" + std::to_string(expectedProjections[i * 3]) + "," + std::to_string(expectedProjections[i * 3 + 1]) + "," + std::to_string(expectedProjections[i * 3 + 2]) + "), GPU valid=true projection=(" + std::to_string(actualProjections[i * 4]) + "," + std::to_string(actualProjections[i * 4 + 1]) + "," + std::to_string(actualProjections[i * 4 + 2]) + ").\n");
			}
			else
			{
				UIUtility::Print("ProjectPoint case " + std::to_string(i) + ", camera=" + std::to_string(cameraIndices[i]) + ", point=(" + std::to_string(worldPoints[i * 4]) + "," + std::to_string(worldPoints[i * 4 + 1]) + "," + std::to_string(worldPoints[i * 4 + 2]) + "), CPU valid=false depth=" + std::to_string(expectedProjections[i * 3 + 2]) + ", GPU valid=false depth=" + std::to_string(actualProjections[i * 4 + 2]) + ".\n");
			}
		}
		UIUtility::Print("PlaneSweep GPU ProjectPoint smoke test succeeded:\n16 / 16 validity results matched CPU ProjectPoint.\n" + std::to_string(validCount) + " / " + std::to_string(validCount) + " valid projected points matched.\nMaximum projected-X absolute error: " + std::to_string(maximumXError) + "\nMaximum projected-Y absolute error: " + std::to_string(maximumYError) + "\nMaximum depth absolute error: " + std::to_string(maximumDepthError) + "\n");
		if (outOfImageValidCase) UIUtility::Print("ProjectPoint out-of-image validity semantics confirmed.\n");
		cleanup();
		return true;
	}
	catch (const std::exception &ex)
	{
		cleanup();
		UIUtility::Print(std::string("PlaneSweep GPU ProjectPoint smoke test failed: ") + ex.what() + "\n");
		return false;
	}
}

bool PlaneSweepGpu::RunSingleSourceCostSmokeTest(
	const std::vector<uint8_t> &referencePixels,
	const std::vector<uint8_t> &sourcePixels,
	const std::vector<GpuCameraIntrinsics> &intrinsics,
	const std::vector<GpuCameraPose> &poses,
	const std::vector<float> &caseValues,
	const std::vector<uint32_t> &patchRadii,
	const std::vector<uint32_t> &expectedSampleCounts,
	const std::vector<uint32_t> &expectedMinimumSamples,
	const std::vector<uint32_t> &expectedValidity,
	const std::vector<float> &expectedCosts)
{
	if (!m_initialized) throw std::runtime_error("PlaneSweepGpu not initialized.");
	constexpr uint32_t kCaseCount = 8;
	constexpr VkFormat kFormat = VK_FORMAT_R8G8B8A8_UNORM;
	constexpr VkDeviceSize kImageBytes = 7 * 7 * 4;
	constexpr VkDeviceSize kDummyBytes = 4;
	constexpr VkDeviceSize kOutputFloatBytes = kCaseCount * sizeof(float);
	constexpr VkDeviceSize kOutputUintBytes = kCaseCount * sizeof(uint32_t);
	if (referencePixels.size() != kImageBytes || sourcePixels.size() != kImageBytes || intrinsics.size() != 2 || poses.size() != 2 ||
		caseValues.size() != kCaseCount * 8 || patchRadii.size() != kCaseCount || expectedSampleCounts.size() != kCaseCount || expectedMinimumSamples.size() != kCaseCount || expectedValidity.size() != kCaseCount || expectedCosts.size() != kCaseCount)
		throw std::invalid_argument("Single-source cost smoke test received invalid input dimensions.");

	VkFormatProperties formatProperties{}; vkGetPhysicalDeviceFormatProperties(m_physicalDevice, kFormat, &formatProperties);
	const VkFormatFeatureFlags requiredFeatures = VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT | VK_FORMAT_FEATURE_TRANSFER_DST_BIT;
	if ((formatProperties.optimalTilingFeatures & requiredFeatures) != requiredFeatures)
	{
		UIUtility::Print("Single-source cost smoke test unsupported: VK_FORMAT_R8G8B8A8_UNORM optimal tiling lacks sampled-image or transfer-destination support.\n");
		return false;
	}

	const std::array<VkDeviceSize, 4> inputSizes{{ 2 * sizeof(GpuCameraIntrinsics), 2 * sizeof(GpuCameraPose), kCaseCount * 8 * sizeof(float), kCaseCount * sizeof(uint32_t) }};
	const std::array<const void *, 4> inputData{{ intrinsics.data(), poses.data(), caseValues.data(), patchRadii.data() }};
	const VkDeviceSize stagingBytes = kImageBytes * 2 + kDummyBytes;
	VkBuffer imageStaging = VK_NULL_HANDLE;
	VkDeviceMemory imageStagingMemory = VK_NULL_HANDLE;
	std::array<VkImage, 3> images{};
	std::array<VkDeviceMemory, 3> imageMemories{};
	std::array<VkImageView, 3> imageViews{};
	VkSampler sampler = VK_NULL_HANDLE;
	std::array<VkBuffer, 4> inputStaging{}, inputBuffers{};
	std::array<VkDeviceMemory, 4> inputStagingMemories{}, inputMemories{};
	std::array<VkBuffer, 4> outputBuffers{}, readbackBuffers{};
	std::array<VkDeviceMemory, 4> outputMemories{}, readbackMemories{};
	VkDescriptorSetLayout descriptorSetLayout = VK_NULL_HANDLE;
	VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
	VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
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
		for (uint32_t i = 0; i < 3; ++i)
		{
			if (imageViews[i] != VK_NULL_HANDLE) { vkDestroyImageView(m_device, imageViews[i], nullptr); imageViews[i] = VK_NULL_HANDLE; }
			if (images[i] != VK_NULL_HANDLE) { vkDestroyImage(m_device, images[i], nullptr); images[i] = VK_NULL_HANDLE; }
			if (imageMemories[i] != VK_NULL_HANDLE) { vkFreeMemory(m_device, imageMemories[i], nullptr); imageMemories[i] = VK_NULL_HANDLE; }
		}
		auto destroyBuffer = [&](VkBuffer &buffer, VkDeviceMemory &memory)
		{
			if (buffer != VK_NULL_HANDLE) { vkDestroyBuffer(m_device, buffer, nullptr); buffer = VK_NULL_HANDLE; }
			if (memory != VK_NULL_HANDLE) { vkFreeMemory(m_device, memory, nullptr); memory = VK_NULL_HANDLE; }
		};
		destroyBuffer(imageStaging, imageStagingMemory);
		for (uint32_t i = 0; i < 4; ++i)
		{
			destroyBuffer(inputStaging[i], inputStagingMemories[i]);
			destroyBuffer(inputBuffers[i], inputMemories[i]);
			destroyBuffer(outputBuffers[i], outputMemories[i]);
			destroyBuffer(readbackBuffers[i], readbackMemories[i]);
		}
	};
	auto findMemoryType = [&](uint32_t bits, VkMemoryPropertyFlags required)
	{
		VkPhysicalDeviceMemoryProperties properties{}; vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &properties);
		for (uint32_t i = 0; i < properties.memoryTypeCount; ++i)
			if ((bits & (1u << i)) && (properties.memoryTypes[i].propertyFlags & required) == required) return i;
		return UINT32_MAX;
	};
	auto createBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags flags, VkBuffer &buffer, VkDeviceMemory &memory)
	{
		VkBufferCreateInfo info{}; info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO; info.size = size; info.usage = usage; info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
		if (vkCreateBuffer(m_device, &info, nullptr, &buffer) != VK_SUCCESS) throw std::runtime_error("Failed to create single-source cost buffer.");
		VkMemoryRequirements requirements{}; vkGetBufferMemoryRequirements(m_device, buffer, &requirements);
		const uint32_t type = findMemoryType(requirements.memoryTypeBits, flags);
		if (type == UINT32_MAX) throw std::runtime_error("No suitable memory type for single-source cost buffer.");
		VkMemoryAllocateInfo allocation{}; allocation.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; allocation.allocationSize = requirements.size; allocation.memoryTypeIndex = type;
		if (vkAllocateMemory(m_device, &allocation, nullptr, &memory) != VK_SUCCESS || vkBindBufferMemory(m_device, buffer, memory, 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate or bind single-source cost buffer memory.");
	};
	try
	{
		const std::array<uint8_t, 4> dummyPixel{{ 0, 0, 0, 255 }};
		createBuffer(stagingBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, imageStaging, imageStagingMemory);
		void *mapped = nullptr;
		if (vkMapMemory(m_device, imageStagingMemory, 0, stagingBytes, 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map single-source image staging memory.");
		std::memcpy(mapped, referencePixels.data(), static_cast<size_t>(kImageBytes));
		std::memcpy(static_cast<uint8_t *>(mapped) + kImageBytes, sourcePixels.data(), static_cast<size_t>(kImageBytes));
		std::memcpy(static_cast<uint8_t *>(mapped) + kImageBytes * 2, dummyPixel.data(), dummyPixel.size());
		vkUnmapMemory(m_device, imageStagingMemory);
		const std::array<VkExtent3D, 3> extents{{ VkExtent3D{7, 7, 1}, VkExtent3D{7, 7, 1}, VkExtent3D{1, 1, 1} }};
		for (uint32_t i = 0; i < 3; ++i)
		{
			VkImageCreateInfo imageInfo{}; imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO; imageInfo.imageType = VK_IMAGE_TYPE_2D; imageInfo.format = kFormat; imageInfo.extent = extents[i]; imageInfo.mipLevels = 1; imageInfo.arrayLayers = 1; imageInfo.samples = VK_SAMPLE_COUNT_1_BIT; imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL; imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT; imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE; imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			if (vkCreateImage(m_device, &imageInfo, nullptr, &images[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create single-source cost image.");
			VkMemoryRequirements requirements{}; vkGetImageMemoryRequirements(m_device, images[i], &requirements);
			const uint32_t type = findMemoryType(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
			if (type == UINT32_MAX) throw std::runtime_error("No device-local memory for single-source cost image.");
			VkMemoryAllocateInfo allocation{}; allocation.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; allocation.allocationSize = requirements.size; allocation.memoryTypeIndex = type;
			if (vkAllocateMemory(m_device, &allocation, nullptr, &imageMemories[i]) != VK_SUCCESS || vkBindImageMemory(m_device, images[i], imageMemories[i], 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate or bind single-source cost image memory.");
			VkImageViewCreateInfo view{}; view.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO; view.image = images[i]; view.viewType = VK_IMAGE_VIEW_TYPE_2D; view.format = kFormat; view.subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
			if (vkCreateImageView(m_device, &view, nullptr, &imageViews[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create single-source cost image view.");
		}
		VkSamplerCreateInfo samplerInfo{}; samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO; samplerInfo.magFilter = VK_FILTER_NEAREST; samplerInfo.minFilter = VK_FILTER_NEAREST; samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST; samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE; samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE; samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE; samplerInfo.anisotropyEnable = VK_FALSE; samplerInfo.compareEnable = VK_FALSE; samplerInfo.minLod = 0.0f; samplerInfo.maxLod = 0.0f; samplerInfo.unnormalizedCoordinates = VK_FALSE;
		if (vkCreateSampler(m_device, &samplerInfo, nullptr, &sampler) != VK_SUCCESS) throw std::runtime_error("Failed to create single-source cost sampler.");

		const VkMemoryPropertyFlags host = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
		for (uint32_t i = 0; i < 4; ++i)
		{
			createBuffer(inputSizes[i], VK_BUFFER_USAGE_TRANSFER_SRC_BIT, host, inputStaging[i], inputStagingMemories[i]);
			if (vkMapMemory(m_device, inputStagingMemories[i], 0, inputSizes[i], 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map single-source case staging memory.");
			std::memcpy(mapped, inputData[i], static_cast<size_t>(inputSizes[i])); vkUnmapMemory(m_device, inputStagingMemories[i]);
			createBuffer(inputSizes[i], VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, inputBuffers[i], inputMemories[i]);
		}
		const std::array<VkDeviceSize, 4> outputSizes{{ kOutputUintBytes, kOutputFloatBytes, kOutputUintBytes, kOutputUintBytes }};
		for (uint32_t i = 0; i < 4; ++i)
		{
			createBuffer(outputSizes[i], VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, outputBuffers[i], outputMemories[i]);
			createBuffer(outputSizes[i], VK_BUFFER_USAGE_TRANSFER_DST_BIT, host, readbackBuffers[i], readbackMemories[i]);
		}

		std::array<VkDescriptorSetLayoutBinding, 12> bindings{};
		for (uint32_t i = 0; i < bindings.size(); ++i) { bindings[i].binding = i; bindings[i].descriptorType = i < 4 ? VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; bindings[i].descriptorCount = (i == 2 || i == 3) ? 8 : 1; bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT; }
		VkDescriptorSetLayoutCreateInfo layoutInfo{}; layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO; layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size()); layoutInfo.pBindings = bindings.data();
		if (vkCreateDescriptorSetLayout(m_device, &layoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create single-source cost descriptor layout.");
		std::array<VkDescriptorPoolSize, 2> poolSizes{{ { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 18 }, { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 8 } }};
		VkDescriptorPoolCreateInfo poolInfo{}; poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO; poolInfo.maxSets = 1; poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size()); poolInfo.pPoolSizes = poolSizes.data();
		if (vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS) throw std::runtime_error("Failed to create single-source cost descriptor pool.");
		VkDescriptorSetAllocateInfo setAllocate{}; setAllocate.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO; setAllocate.descriptorPool = descriptorPool; setAllocate.descriptorSetCount = 1; setAllocate.pSetLayouts = &descriptorSetLayout;
		if (vkAllocateDescriptorSets(m_device, &setAllocate, &descriptorSet) != VK_SUCCESS) throw std::runtime_error("Failed to allocate single-source cost descriptor set.");
		std::array<VkDescriptorImageInfo, 1> referenceImageInfo{{ { sampler, imageViews[0], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL } }};
		std::array<VkDescriptorImageInfo, 1> referenceMaskInfo{{ { sampler, imageViews[2], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL } }};
		std::array<VkDescriptorImageInfo, 8> sourceImageInfos{}, sourceMaskInfos{};
		for (uint32_t i = 0; i < 8; ++i)
		{
			sourceImageInfos[i] = VkDescriptorImageInfo{ sampler, i == 0 ? imageViews[1] : imageViews[2], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
			sourceMaskInfos[i] = VkDescriptorImageInfo{ sampler, imageViews[2], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
		}
		std::array<VkDescriptorBufferInfo, 8> bufferInfos{};
		for (uint32_t i = 0; i < 4; ++i) bufferInfos[i] = VkDescriptorBufferInfo{ inputBuffers[i], 0, inputSizes[i] };
		for (uint32_t i = 0; i < 4; ++i) bufferInfos[4 + i] = VkDescriptorBufferInfo{ outputBuffers[i], 0, outputSizes[i] };
		std::array<VkWriteDescriptorSet, 12> writes{};
		for (uint32_t i = 0; i < 12; ++i) { writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; writes[i].dstSet = descriptorSet; writes[i].dstBinding = i; writes[i].descriptorType = i < 4 ? VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; }
		writes[0].descriptorCount = 1; writes[0].pImageInfo = referenceImageInfo.data();
		writes[1].descriptorCount = 1; writes[1].pImageInfo = referenceMaskInfo.data();
		writes[2].descriptorCount = 8; writes[2].pImageInfo = sourceImageInfos.data();
		writes[3].descriptorCount = 8; writes[3].pImageInfo = sourceMaskInfos.data();
		for (uint32_t i = 0; i < 8; ++i) { writes[4 + i].descriptorCount = 1; writes[4 + i].pBufferInfo = &bufferInfos[i]; }
		vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);

		const std::vector<char> shaderCode = ReadShaderFile("shaders/planeSweepSingleSourceCostSmokeTest.comp.spv");
		if (shaderCode.empty()) throw std::runtime_error("Failed to load single-source cost smoke-test SPIR-V.");
		VkShaderModuleCreateInfo shaderInfo{}; shaderInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO; shaderInfo.codeSize = shaderCode.size(); shaderInfo.pCode = reinterpret_cast<const uint32_t *>(shaderCode.data());
		if (vkCreateShaderModule(m_device, &shaderInfo, nullptr, &shaderModule) != VK_SUCCESS) throw std::runtime_error("Failed to create single-source cost shader module.");
		VkPipelineLayoutCreateInfo pipelineLayoutInfo{}; pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO; pipelineLayoutInfo.setLayoutCount = 1; pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout;
		if (vkCreatePipelineLayout(m_device, &pipelineLayoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create single-source cost pipeline layout.");
		VkPipelineShaderStageCreateInfo stage{}; stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = shaderModule; stage.pName = "main";
		VkComputePipelineCreateInfo pipelineInfo{}; pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO; pipelineInfo.stage = stage; pipelineInfo.layout = pipelineLayout;
		if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline) != VK_SUCCESS) throw std::runtime_error("Failed to create single-source cost compute pipeline.");

		VkCommandBufferAllocateInfo commandAllocate{}; commandAllocate.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO; commandAllocate.commandPool = m_commandPool; commandAllocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; commandAllocate.commandBufferCount = 1;
		if (vkAllocateCommandBuffers(m_device, &commandAllocate, &commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to allocate single-source cost command buffer.");
		VkCommandBufferBeginInfo commandBegin{}; commandBegin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO; commandBegin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
		if (vkBeginCommandBuffer(commandBuffer, &commandBegin) != VK_SUCCESS) throw std::runtime_error("Failed to begin single-source cost command buffer.");
		std::array<VkImageMemoryBarrier, 3> toTransfer{};
		for (uint32_t i = 0; i < 3; ++i) { toTransfer[i].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; toTransfer[i].dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; toTransfer[i].oldLayout = VK_IMAGE_LAYOUT_UNDEFINED; toTransfer[i].newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; toTransfer[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toTransfer[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toTransfer[i].image = images[i]; toTransfer[i].subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 }; }
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 3, toTransfer.data());
		for (uint32_t i = 0; i < 3; ++i)
		{
			VkBufferImageCopy copy{}; copy.bufferOffset = i == 0 ? 0 : (i == 1 ? kImageBytes : kImageBytes * 2); copy.imageSubresource = VkImageSubresourceLayers{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 }; copy.imageExtent = extents[i];
			vkCmdCopyBufferToImage(commandBuffer, imageStaging, images[i], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
		}
		std::array<VkImageMemoryBarrier, 3> toShaderRead{};
		for (uint32_t i = 0; i < 3; ++i) { toShaderRead[i].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; toShaderRead[i].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; toShaderRead[i].dstAccessMask = VK_ACCESS_SHADER_READ_BIT; toShaderRead[i].oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; toShaderRead[i].newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; toShaderRead[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toShaderRead[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toShaderRead[i].image = images[i]; toShaderRead[i].subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 }; }
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 3, toShaderRead.data());
		for (uint32_t i = 0; i < 4; ++i) { VkBufferCopy copy{}; copy.size = inputSizes[i]; vkCmdCopyBuffer(commandBuffer, inputStaging[i], inputBuffers[i], 1, &copy); }
		std::array<VkBufferMemoryBarrier, 4> inputBarriers{};
		for (uint32_t i = 0; i < 4; ++i) { inputBarriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; inputBarriers[i].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; inputBarriers[i].dstAccessMask = VK_ACCESS_SHADER_READ_BIT; inputBarriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; inputBarriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; inputBarriers[i].buffer = inputBuffers[i]; inputBarriers[i].size = inputSizes[i]; }
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 4, inputBarriers.data(), 0, nullptr);
		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
		vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
		vkCmdDispatch(commandBuffer, 1, 1, 1);
		std::array<VkBufferMemoryBarrier, 4> outputBarriers{};
		for (uint32_t i = 0; i < 4; ++i) { outputBarriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; outputBarriers[i].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT; outputBarriers[i].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT; outputBarriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; outputBarriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; outputBarriers[i].buffer = outputBuffers[i]; outputBarriers[i].size = outputSizes[i]; }
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 4, outputBarriers.data(), 0, nullptr);
		for (uint32_t i = 0; i < 4; ++i) { VkBufferCopy copy{}; copy.size = outputSizes[i]; vkCmdCopyBuffer(commandBuffer, outputBuffers[i], readbackBuffers[i], 1, &copy); }
		std::array<VkBufferMemoryBarrier, 4> hostBarriers{};
		for (uint32_t i = 0; i < 4; ++i) { hostBarriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; hostBarriers[i].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; hostBarriers[i].dstAccessMask = VK_ACCESS_HOST_READ_BIT; hostBarriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; hostBarriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; hostBarriers[i].buffer = readbackBuffers[i]; hostBarriers[i].size = outputSizes[i]; }
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 0, nullptr, 4, hostBarriers.data(), 0, nullptr);
		if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to end single-source cost command buffer.");
		VkFenceCreateInfo fenceInfo{}; fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
		if (vkCreateFence(m_device, &fenceInfo, nullptr, &fence) != VK_SUCCESS) throw std::runtime_error("Failed to create single-source cost fence.");
		VkSubmitInfo submit{}; submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO; submit.commandBufferCount = 1; submit.pCommandBuffers = &commandBuffer;
		if (vkQueueSubmit(m_queue, 1, &submit, fence) != VK_SUCCESS) throw std::runtime_error("Failed to submit single-source cost command buffer.");
		submitted = true;
		if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS) throw std::runtime_error("Failed to wait for single-source cost fence.");
		submitted = false;
		vkDestroyFence(m_device, fence, nullptr); fence = VK_NULL_HANDLE;
		std::array<uint32_t, kCaseCount> gpuValidity{}, gpuSampleCounts{}, gpuMinimumSamples{};
		std::array<float, kCaseCount> gpuCosts{};
		const std::array<void *, 4> readbackData{{ gpuValidity.data(), gpuCosts.data(), gpuSampleCounts.data(), gpuMinimumSamples.data() }};
		for (uint32_t i = 0; i < 4; ++i)
		{
			if (vkMapMemory(m_device, readbackMemories[i], 0, outputSizes[i], 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map single-source cost readback.");
			std::memcpy(readbackData[i], mapped, static_cast<size_t>(outputSizes[i])); vkUnmapMemory(m_device, readbackMemories[i]);
		}
		for (uint32_t i = 0; i < kCaseCount; ++i)
		{
			if (gpuSampleCounts[i] != expectedSampleCounts[i] || gpuMinimumSamples[i] != expectedMinimumSamples[i])
			{
				UIUtility::Print("Single-source cost sample-count diagnostic mismatch: case=" + std::to_string(i) + ", expected accepted=" + std::to_string(expectedSampleCounts[i]) + ", GPU accepted=" + std::to_string(gpuSampleCounts[i]) + ", expected minimum=" + std::to_string(expectedMinimumSamples[i]) + ", GPU minimum=" + std::to_string(gpuMinimumSamples[i]) + ".\n");
				cleanup(); return false;
			}
			if ((gpuValidity[i] != 0) != (expectedValidity[i] != 0))
			{
				UIUtility::Print("Single-source cost validity mismatch: case=" + std::to_string(i) + ", x=" + std::to_string(caseValues[i * 8]) + ", y=" + std::to_string(caseValues[i * 8 + 1]) + ", depth=" + std::to_string(caseValues[i * 8 + 2]) + ", patchRadius=" + std::to_string(patchRadii[i]) + ", alphaThreshold=" + std::to_string(caseValues[i * 8 + 3]) + ", minimumOpaquePatchRatio=" + std::to_string(caseValues[i * 8 + 4]) + ", GPU acceptedSampleCount=" + std::to_string(gpuSampleCounts[i]) + ", GPU minimumOpaqueSamples=" + std::to_string(gpuMinimumSamples[i]) + ", CPU validity=" + std::to_string(expectedValidity[i]) + ", GPU validity=" + std::to_string(gpuValidity[i]) + ".\n");
				cleanup(); return false;
			}
		}
		float maximumCostError = 0.0f;
		for (uint32_t i = 0; i < kCaseCount; ++i)
		{
			if (expectedValidity[i] != 0)
			{
				const float error = std::fabs(gpuCosts[i] - expectedCosts[i]);
				maximumCostError = std::max(maximumCostError, error);
				if (error > 1e-6f)
				{
					UIUtility::Print("Single-source cost mismatch: case=" + std::to_string(i) + ", CPU cost=" + std::to_string(expectedCosts[i]) + ", GPU cost=" + std::to_string(gpuCosts[i]) + ", absolute error=" + std::to_string(error) + ".\n");
					cleanup(); return false;
				}
			}
		}
		const uint32_t cpuValidCount = static_cast<uint32_t>(std::count(expectedValidity.begin(), expectedValidity.end(), 1u));
		UIUtility::Print("PlaneSweep GPU single-source patch-cost smoke test succeeded:\n8 / 8 validity results matched CPU ComputePlaneCost.\n" + std::to_string(cpuValidCount) + " / " + std::to_string(cpuValidCount) + " valid costs matched.\nMaximum cost absolute error: " + std::to_string(maximumCostError) + "\n");
		for (uint32_t i = 0; i < kCaseCount; ++i) UIUtility::Print("case " + std::to_string(i) + ": " + std::to_string(gpuSampleCounts[i]) + "\n");
		UIUtility::Print("Theoretical controlled per-sample cost: " + std::to_string(18.0f / (255.0f * 3.0f)) + "\n");
		cleanup();
		return true;
	}
	catch (const std::exception &ex)
	{
		cleanup();
		UIUtility::Print(std::string("PlaneSweep GPU single-source patch-cost smoke test failed: ") + ex.what() + "\n");
		return false;
	}
}

bool PlaneSweepGpu::RunMultiSourceCostSmokeTest(
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
	const std::vector<float> &expectedCosts)
{
	if (!m_initialized) throw std::runtime_error("PlaneSweepGpu not initialized.");
	constexpr uint32_t kCases = 10;
	constexpr uint32_t kImages = 7;
	constexpr VkDeviceSize kImageBytes = 7 * 7 * 4;
	constexpr VkDeviceSize kDummyBytes = 4;
	constexpr VkFormat kFormat = VK_FORMAT_R8G8B8A8_UNORM;
	if (referencePixels.size() != kImageBytes || source0Pixels.size() != kImageBytes || source1Pixels.size() != kImageBytes || source2Pixels.size() != kImageBytes || source2MaskPixels.size() != kImageBytes || source3Pixels.size() != kImageBytes ||
		intrinsics.size() != 5 || poses.size() != 5 || caseValues.size() != kCases * 8 || caseParameters.size() != kCases * 4 || sourceMetadata.size() != 16 ||
		expectedSampleCounts.size() != kCases * 8 || expectedValidViews.size() != kCases || expectedMinimumSamples.size() != kCases || expectedValidity.size() != kCases || expectedCosts.size() != kCases)
		throw std::invalid_argument("Multi-source cost smoke test received invalid input dimensions.");
	for (uint32_t i = 0; i < kCases; ++i)
		if (caseParameters[i * 4 + 1] > MAX_PLANE_SWEEP_SOURCE_IMAGES) throw std::invalid_argument("Multi-source cost smoke test sourceCount exceeds GPU capacity 8.");

	VkFormatProperties formatProperties{}; vkGetPhysicalDeviceFormatProperties(m_physicalDevice, kFormat, &formatProperties);
	const VkFormatFeatureFlags required = VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT | VK_FORMAT_FEATURE_TRANSFER_DST_BIT;
	if ((formatProperties.optimalTilingFeatures & required) != required) { UIUtility::Print("Multi-source cost smoke test unsupported: RGBA8 optimal tiling lacks sampled-image or transfer-destination support.\n"); return false; }
	VkPhysicalDeviceProperties deviceProperties{}; vkGetPhysicalDeviceProperties(m_physicalDevice, &deviceProperties);
	const VkPhysicalDeviceLimits &limits = deviceProperties.limits;
	if (limits.maxPerStageDescriptorSamplers < 18 || limits.maxPerStageDescriptorSampledImages < 18 || limits.maxPerStageDescriptorStorageBuffers < 11 ||
		limits.maxDescriptorSetSamplers < 18 || limits.maxDescriptorSetSampledImages < 18 || limits.maxDescriptorSetStorageBuffers < 11 || limits.maxPerStageResources < 29)
	{
		UIUtility::Print("Multi-source cost smoke test unsupported: device descriptor limits are below 18 sampled images/samplers, 11 storage buffers, or 29 total per-stage resources.\n");
		return false;
	}

	const std::array<const std::vector<uint8_t> *, 6> inputImages{{ &referencePixels, &source0Pixels, &source1Pixels, &source2Pixels, &source3Pixels, &source2MaskPixels }};
	const VkDeviceSize imageStagingBytes = kImageBytes * 6 + kDummyBytes;
	const std::array<VkDeviceSize, 5> inputSizes{{ 5 * sizeof(GpuCameraIntrinsics), 5 * sizeof(GpuCameraPose), kCases * 8 * sizeof(float), kCases * 4 * sizeof(uint32_t), 16 * sizeof(uint32_t) }};
	const std::array<const void *, 5> inputData{{ intrinsics.data(), poses.data(), caseValues.data(), caseParameters.data(), sourceMetadata.data() }};
	const std::array<VkDeviceSize, 6> outputSizes{{ kCases * sizeof(uint32_t), kCases * sizeof(float), kCases * sizeof(uint32_t), kCases * sizeof(uint32_t), kCases * 8 * sizeof(uint32_t), kCases * 8 * sizeof(float) }};
	VkBuffer imageStaging = VK_NULL_HANDLE; VkDeviceMemory imageStagingMemory = VK_NULL_HANDLE;
	std::array<VkImage, kImages> images{}; std::array<VkDeviceMemory, kImages> imageMemories{}; std::array<VkImageView, kImages> imageViews{};
	VkSampler sampler = VK_NULL_HANDLE;
	std::array<VkBuffer, 5> inputStaging{}, inputBuffers{}; std::array<VkDeviceMemory, 5> inputStagingMemories{}, inputMemories{};
	std::array<VkBuffer, 6> outputBuffers{}, readbackBuffers{}; std::array<VkDeviceMemory, 6> outputMemories{}, readbackMemories{};
	VkDescriptorSetLayout setLayout = VK_NULL_HANDLE; VkDescriptorPool pool = VK_NULL_HANDLE; VkDescriptorSet set = VK_NULL_HANDLE;
	VkPipelineLayout pipelineLayout = VK_NULL_HANDLE; VkShaderModule shader = VK_NULL_HANDLE; VkPipeline pipeline = VK_NULL_HANDLE;
	VkCommandBuffer commandBuffer = VK_NULL_HANDLE; VkFence fence = VK_NULL_HANDLE; bool submitted = false;
	auto cleanup = [&]()
	{
		if (submitted) { vkDeviceWaitIdle(m_device); submitted = false; }
		if (fence) vkDestroyFence(m_device, fence, nullptr);
		if (commandBuffer) vkFreeCommandBuffers(m_device, m_commandPool, 1, &commandBuffer);
		if (pipeline) vkDestroyPipeline(m_device, pipeline, nullptr);
		if (shader) vkDestroyShaderModule(m_device, shader, nullptr);
		if (pipelineLayout) vkDestroyPipelineLayout(m_device, pipelineLayout, nullptr);
		if (pool) vkDestroyDescriptorPool(m_device, pool, nullptr);
		if (setLayout) vkDestroyDescriptorSetLayout(m_device, setLayout, nullptr);
		if (sampler) vkDestroySampler(m_device, sampler, nullptr);
		for (uint32_t i = 0; i < kImages; ++i) { if (imageViews[i]) vkDestroyImageView(m_device, imageViews[i], nullptr); if (images[i]) vkDestroyImage(m_device, images[i], nullptr); if (imageMemories[i]) vkFreeMemory(m_device, imageMemories[i], nullptr); }
		auto destroyBuffer = [&](VkBuffer &b, VkDeviceMemory &m) { if (b) vkDestroyBuffer(m_device, b, nullptr); if (m) vkFreeMemory(m_device, m, nullptr); b = VK_NULL_HANDLE; m = VK_NULL_HANDLE; };
		destroyBuffer(imageStaging, imageStagingMemory);
		for (uint32_t i = 0; i < 5; ++i) { destroyBuffer(inputStaging[i], inputStagingMemories[i]); destroyBuffer(inputBuffers[i], inputMemories[i]); }
		for (uint32_t i = 0; i < 6; ++i) { destroyBuffer(outputBuffers[i], outputMemories[i]); destroyBuffer(readbackBuffers[i], readbackMemories[i]); }
	};
	auto findMemory = [&](uint32_t bits, VkMemoryPropertyFlags flags) { VkPhysicalDeviceMemoryProperties p{}; vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &p); for (uint32_t i = 0; i < p.memoryTypeCount; ++i) if ((bits & (1u << i)) && (p.memoryTypes[i].propertyFlags & flags) == flags) return i; return UINT32_MAX; };
	auto makeBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags flags, VkBuffer &b, VkDeviceMemory &m) { VkBufferCreateInfo ci{}; ci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO; ci.size = size; ci.usage = usage; ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE; if (vkCreateBuffer(m_device, &ci, nullptr, &b) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source buffer."); VkMemoryRequirements r{}; vkGetBufferMemoryRequirements(m_device, b, &r); uint32_t mt = findMemory(r.memoryTypeBits, flags); if (mt == UINT32_MAX) throw std::runtime_error("No memory type for multi-source buffer."); VkMemoryAllocateInfo ai{}; ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; ai.allocationSize = r.size; ai.memoryTypeIndex = mt; if (vkAllocateMemory(m_device, &ai, nullptr, &m) != VK_SUCCESS || vkBindBufferMemory(m_device, b, m, 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate multi-source buffer memory."); };
	try
	{
		const VkMemoryPropertyFlags host = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
		makeBuffer(imageStagingBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, host, imageStaging, imageStagingMemory);
		void *mapped = nullptr; if (vkMapMemory(m_device, imageStagingMemory, 0, imageStagingBytes, 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map image staging.");
		for (uint32_t i = 0; i < 6; ++i) std::memcpy(static_cast<uint8_t *>(mapped) + i * kImageBytes, inputImages[i]->data(), static_cast<size_t>(kImageBytes));
		const std::array<uint8_t, 4> dummy{{ 0, 0, 0, 255 }}; std::memcpy(static_cast<uint8_t *>(mapped) + 6 * kImageBytes, dummy.data(), dummy.size()); vkUnmapMemory(m_device, imageStagingMemory);
		for (uint32_t i = 0; i < kImages; ++i)
		{
			VkImageCreateInfo ci{}; ci.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO; ci.imageType = VK_IMAGE_TYPE_2D; ci.format = kFormat; ci.extent = i == 6 ? VkExtent3D{1, 1, 1} : VkExtent3D{7, 7, 1}; ci.mipLevels = 1; ci.arrayLayers = 1; ci.samples = VK_SAMPLE_COUNT_1_BIT; ci.tiling = VK_IMAGE_TILING_OPTIMAL; ci.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT; ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE; ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			if (vkCreateImage(m_device, &ci, nullptr, &images[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source image.");
			VkMemoryRequirements r{}; vkGetImageMemoryRequirements(m_device, images[i], &r); uint32_t mt = findMemory(r.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT); if (mt == UINT32_MAX) throw std::runtime_error("No device-local memory for multi-source image.");
			VkMemoryAllocateInfo ai{}; ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; ai.allocationSize = r.size; ai.memoryTypeIndex = mt; if (vkAllocateMemory(m_device, &ai, nullptr, &imageMemories[i]) != VK_SUCCESS || vkBindImageMemory(m_device, images[i], imageMemories[i], 0) != VK_SUCCESS) throw std::runtime_error("Failed to bind multi-source image memory.");
			VkImageViewCreateInfo vi{}; vi.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO; vi.image = images[i]; vi.viewType = VK_IMAGE_VIEW_TYPE_2D; vi.format = kFormat; vi.subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 }; if (vkCreateImageView(m_device, &vi, nullptr, &imageViews[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source image view.");
		}
		VkSamplerCreateInfo si{}; si.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO; si.magFilter = VK_FILTER_NEAREST; si.minFilter = VK_FILTER_NEAREST; si.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST; si.addressModeU = si.addressModeV = si.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE; si.anisotropyEnable = VK_FALSE; si.compareEnable = VK_FALSE; si.minLod = 0.0f; si.maxLod = 0.0f; if (vkCreateSampler(m_device, &si, nullptr, &sampler) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source sampler.");
		for (uint32_t i = 0; i < 5; ++i) { makeBuffer(inputSizes[i], VK_BUFFER_USAGE_TRANSFER_SRC_BIT, host, inputStaging[i], inputStagingMemories[i]); if (vkMapMemory(m_device, inputStagingMemories[i], 0, inputSizes[i], 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map multi-source input staging."); std::memcpy(mapped, inputData[i], static_cast<size_t>(inputSizes[i])); vkUnmapMemory(m_device, inputStagingMemories[i]); makeBuffer(inputSizes[i], VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, inputBuffers[i], inputMemories[i]); }
		for (uint32_t i = 0; i < 6; ++i) { makeBuffer(outputSizes[i], VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, outputBuffers[i], outputMemories[i]); makeBuffer(outputSizes[i], VK_BUFFER_USAGE_TRANSFER_DST_BIT, host, readbackBuffers[i], readbackMemories[i]); }
		std::array<VkDescriptorSetLayoutBinding, 15> bindings{};
		for (uint32_t i = 0; i < 15; ++i) { bindings[i].binding = i; bindings[i].descriptorType = i < 4 ? VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; bindings[i].descriptorCount = (i == 2 || i == 3) ? 8 : 1; bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT; }
		VkDescriptorSetLayoutCreateInfo li{}; li.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO; li.bindingCount = static_cast<uint32_t>(bindings.size()); li.pBindings = bindings.data(); if (vkCreateDescriptorSetLayout(m_device, &li, nullptr, &setLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source descriptor layout.");
		std::array<VkDescriptorPoolSize, 2> ps{{ { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 18 }, { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 11 } }}; VkDescriptorPoolCreateInfo pi{}; pi.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO; pi.maxSets = 1; pi.poolSizeCount = static_cast<uint32_t>(ps.size()); pi.pPoolSizes = ps.data(); if (vkCreateDescriptorPool(m_device, &pi, nullptr, &pool) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source descriptor pool.");
		VkDescriptorSetAllocateInfo sai{}; sai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO; sai.descriptorPool = pool; sai.descriptorSetCount = 1; sai.pSetLayouts = &setLayout; if (vkAllocateDescriptorSets(m_device, &sai, &set) != VK_SUCCESS) throw std::runtime_error("Failed to allocate multi-source descriptor set.");
		std::array<VkDescriptorImageInfo, 1> refInfo{{ { sampler, imageViews[0], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL } }}, refMaskInfo{{ { sampler, imageViews[6], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL } }};
		std::array<VkDescriptorImageInfo, 8> sourceInfos{}, maskInfos{};
		for (uint32_t i = 0; i < 8; ++i) { sourceInfos[i] = VkDescriptorImageInfo{ sampler, i < 4 ? imageViews[i + 1] : imageViews[6], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL }; maskInfos[i] = VkDescriptorImageInfo{ sampler, i == 2 ? imageViews[5] : imageViews[6], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL }; }
		std::array<VkDescriptorBufferInfo, 11> bi{}; for (uint32_t i = 0; i < 5; ++i) bi[i] = VkDescriptorBufferInfo{ inputBuffers[i], 0, inputSizes[i] }; for (uint32_t i = 0; i < 6; ++i) bi[5 + i] = VkDescriptorBufferInfo{ outputBuffers[i], 0, outputSizes[i] };
		std::array<VkWriteDescriptorSet, 15> writes{}; for (uint32_t i = 0; i < 15; ++i) { writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; writes[i].dstSet = set; writes[i].dstBinding = i; writes[i].descriptorType = i < 4 ? VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; }
		writes[0].descriptorCount = 1; writes[0].pImageInfo = refInfo.data(); writes[1].descriptorCount = 1; writes[1].pImageInfo = refMaskInfo.data(); writes[2].descriptorCount = 8; writes[2].pImageInfo = sourceInfos.data(); writes[3].descriptorCount = 8; writes[3].pImageInfo = maskInfos.data(); for (uint32_t i = 0; i < 11; ++i) { writes[4 + i].descriptorCount = 1; writes[4 + i].pBufferInfo = &bi[i]; } vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
		const std::vector<char> code = ReadShaderFile("shaders/planeSweepMultiSourceCostSmokeTest.comp.spv"); if (code.empty()) throw std::runtime_error("Failed to load multi-source cost SPIR-V."); VkShaderModuleCreateInfo sci{}; sci.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO; sci.codeSize = code.size(); sci.pCode = reinterpret_cast<const uint32_t *>(code.data()); if (vkCreateShaderModule(m_device, &sci, nullptr, &shader) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source cost shader.");
		VkPipelineLayoutCreateInfo pli{}; pli.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO; pli.setLayoutCount = 1; pli.pSetLayouts = &setLayout; if (vkCreatePipelineLayout(m_device, &pli, nullptr, &pipelineLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source cost pipeline layout."); VkPipelineShaderStageCreateInfo stage{}; stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = shader; stage.pName = "main"; VkComputePipelineCreateInfo pci{}; pci.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO; pci.stage = stage; pci.layout = pipelineLayout; if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pci, nullptr, &pipeline) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source cost pipeline.");
		VkCommandBufferAllocateInfo cai{}; cai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO; cai.commandPool = m_commandPool; cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; cai.commandBufferCount = 1; if (vkAllocateCommandBuffers(m_device, &cai, &commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to allocate multi-source command buffer."); VkCommandBufferBeginInfo cbi{}; cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO; cbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT; if (vkBeginCommandBuffer(commandBuffer, &cbi) != VK_SUCCESS) throw std::runtime_error("Failed to begin multi-source command buffer.");
		std::array<VkImageMemoryBarrier, kImages> toTransfer{}; for (uint32_t i = 0; i < kImages; ++i) { toTransfer[i].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; toTransfer[i].dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; toTransfer[i].oldLayout = VK_IMAGE_LAYOUT_UNDEFINED; toTransfer[i].newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; toTransfer[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toTransfer[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toTransfer[i].image = images[i]; toTransfer[i].subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 }; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, kImages, toTransfer.data());
		for (uint32_t i = 0; i < kImages; ++i) { VkBufferImageCopy copy{}; copy.bufferOffset = i == 6 ? kImageBytes * 6 : kImageBytes * i; copy.imageSubresource = VkImageSubresourceLayers{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 }; copy.imageExtent = i == 6 ? VkExtent3D{1, 1, 1} : VkExtent3D{7, 7, 1}; vkCmdCopyBufferToImage(commandBuffer, imageStaging, images[i], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy); }
		std::array<VkImageMemoryBarrier, kImages> toRead{}; for (uint32_t i = 0; i < kImages; ++i) { toRead[i].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; toRead[i].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; toRead[i].dstAccessMask = VK_ACCESS_SHADER_READ_BIT; toRead[i].oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; toRead[i].newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; toRead[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toRead[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toRead[i].image = images[i]; toRead[i].subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 }; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 0, nullptr, kImages, toRead.data());
		for (uint32_t i = 0; i < 5; ++i) { VkBufferCopy copy{}; copy.size = inputSizes[i]; vkCmdCopyBuffer(commandBuffer, inputStaging[i], inputBuffers[i], 1, &copy); } std::array<VkBufferMemoryBarrier, 5> inputBarriers{}; for (uint32_t i = 0; i < 5; ++i) { inputBarriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; inputBarriers[i].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; inputBarriers[i].dstAccessMask = VK_ACCESS_SHADER_READ_BIT; inputBarriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; inputBarriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; inputBarriers[i].buffer = inputBuffers[i]; inputBarriers[i].size = inputSizes[i]; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 5, inputBarriers.data(), 0, nullptr);
		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline); vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &set, 0, nullptr); vkCmdDispatch(commandBuffer, 1, 1, 1);
		std::array<VkBufferMemoryBarrier, 6> outputBarriers{}; for (uint32_t i = 0; i < 6; ++i) { outputBarriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; outputBarriers[i].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT; outputBarriers[i].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT; outputBarriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; outputBarriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; outputBarriers[i].buffer = outputBuffers[i]; outputBarriers[i].size = outputSizes[i]; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 6, outputBarriers.data(), 0, nullptr);
		for (uint32_t i = 0; i < 6; ++i) { VkBufferCopy copy{}; copy.size = outputSizes[i]; vkCmdCopyBuffer(commandBuffer, outputBuffers[i], readbackBuffers[i], 1, &copy); } std::array<VkBufferMemoryBarrier, 6> hostBarriers{}; for (uint32_t i = 0; i < 6; ++i) { hostBarriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; hostBarriers[i].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; hostBarriers[i].dstAccessMask = VK_ACCESS_HOST_READ_BIT; hostBarriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; hostBarriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; hostBarriers[i].buffer = readbackBuffers[i]; hostBarriers[i].size = outputSizes[i]; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 0, nullptr, 6, hostBarriers.data(), 0, nullptr);
		if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to end multi-source command buffer."); VkFenceCreateInfo fi{}; fi.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO; if (vkCreateFence(m_device, &fi, nullptr, &fence) != VK_SUCCESS) throw std::runtime_error("Failed to create multi-source fence."); VkSubmitInfo submit{}; submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO; submit.commandBufferCount = 1; submit.pCommandBuffers = &commandBuffer; if (vkQueueSubmit(m_queue, 1, &submit, fence) != VK_SUCCESS) throw std::runtime_error("Failed to submit multi-source test."); submitted = true; if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS) throw std::runtime_error("Failed to wait for multi-source test fence."); submitted = false;
		std::array<uint32_t, kCases> gpuValidity{}, gpuViews{}, gpuMinimum{}; std::array<float, kCases> gpuCosts{}; std::array<uint32_t, kCases * 8> gpuCounts{}; std::array<float, kCases * 8> gpuAverages{};
		const std::array<void *, 6> targets{{ gpuValidity.data(), gpuCosts.data(), gpuViews.data(), gpuMinimum.data(), gpuCounts.data(), gpuAverages.data() }}; for (uint32_t i = 0; i < 6; ++i) { if (vkMapMemory(m_device, readbackMemories[i], 0, outputSizes[i], 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map multi-source readback."); std::memcpy(targets[i], mapped, static_cast<size_t>(outputSizes[i])); vkUnmapMemory(m_device, readbackMemories[i]); }
		for (uint32_t i = 0; i < kCases; ++i)
		{
			if (gpuMinimum[i] != expectedMinimumSamples[i] || gpuViews[i] != expectedValidViews[i]) { UIUtility::Print("Multi-source diagnostics mismatch case=" + std::to_string(i) + ", GPU views=" + std::to_string(gpuViews[i]) + ", expected=" + std::to_string(expectedValidViews[i]) + ", GPU minimum=" + std::to_string(gpuMinimum[i]) + ", expected=" + std::to_string(expectedMinimumSamples[i]) + ".\n"); cleanup(); return false; }
			for (uint32_t s = 0; s < 8; ++s) if (gpuCounts[i * 8 + s] != expectedSampleCounts[i * 8 + s]) { UIUtility::Print("Multi-source sample count mismatch case=" + std::to_string(i) + ", source=" + std::to_string(s) + ", GPU=" + std::to_string(gpuCounts[i * 8 + s]) + ", expected=" + std::to_string(expectedSampleCounts[i * 8 + s]) + ".\n"); cleanup(); return false; }
			if ((gpuValidity[i] != 0) != (expectedValidity[i] != 0)) { std::string msg = "Multi-source validity mismatch case=" + std::to_string(i) + ", sourceCount=" + std::to_string(caseParameters[i * 4 + 1]) + ", minValidSourceViews=" + std::to_string(caseParameters[i * 4 + 2]) + ", patchRadius=" + std::to_string(caseParameters[i * 4]) + ", minimumOpaquePatchRatio=" + std::to_string(caseValues[i * 8 + 4]) + ", alphaThreshold=" + std::to_string(caseValues[i * 8 + 3]) + ", CPU=" + std::to_string(expectedValidity[i]) + ", GPU=" + std::to_string(gpuValidity[i]) + ", views=" + std::to_string(gpuViews[i]) + ", minimum=" + std::to_string(gpuMinimum[i]) + ", sampleCounts=["; for (uint32_t s = 0; s < 8; ++s) msg += (s ? "," : "") + std::to_string(gpuCounts[i * 8 + s]); UIUtility::Print(msg + "].\n"); cleanup(); return false; }
		}
		float maximumError = 0.0f; uint32_t validCount = 0;
		for (uint32_t i = 0; i < kCases; ++i) if (expectedValidity[i]) { ++validCount; float e = std::fabs(gpuCosts[i] - expectedCosts[i]); maximumError = std::max(maximumError, e); if (e > 1e-6f) { std::string msg = "Multi-source cost mismatch case=" + std::to_string(i) + ", CPU=" + std::to_string(expectedCosts[i]) + ", GPU=" + std::to_string(gpuCosts[i]) + ", error=" + std::to_string(e) + ", sourceAverageCosts=["; for (uint32_t s = 0; s < 8; ++s) msg += (s ? "," : "") + std::to_string(gpuAverages[i * 8 + s]); UIUtility::Print(msg + "].\n"); cleanup(); return false; } }
		UIUtility::Print("PlaneSweep GPU multi-source ComputePlaneCost smoke test succeeded:\n10 / 10 validity results matched CPU ComputePlaneCost.\n" + std::to_string(validCount) + " / " + std::to_string(validCount) + " valid costs matched.\nMaximum final-cost absolute error: " + std::to_string(maximumError) + "\n");
		for (uint32_t i = 0; i < kCases; ++i) { std::string line = "case " + std::to_string(i) + ": sourceCount=" + std::to_string(caseParameters[i * 4 + 1]) + ", validSourceViews=" + std::to_string(gpuViews[i]) + ", minimumOpaqueSamples=" + std::to_string(gpuMinimum[i]) + ", sampleCounts=["; for (uint32_t s = 0; s < 8; ++s) line += (s ? "," : "") + std::to_string(gpuCounts[i * 8 + s]); line += "], sourceAverageCosts=["; for (uint32_t s = 0; s < 8; ++s) line += (s ? "," : "") + std::to_string(gpuAverages[i * 8 + s]); line += "]"; if (expectedValidity[i]) line += ", CPU cost=" + std::to_string(expectedCosts[i]) + ", GPU cost=" + std::to_string(gpuCosts[i]); UIUtility::Print(line + "\n"); }
		UIUtility::Print("Theoretical controlled source costs: source0 = " + std::to_string(6.0f / 255.0f) + ", source1 = " + std::to_string(3.0f / 255.0f) + ", source2 = " + std::to_string(9.0f / 255.0f) + "\n");
		cleanup(); return true;
	}
	catch (const std::exception &ex) { cleanup(); UIUtility::Print(std::string("PlaneSweep GPU multi-source ComputePlaneCost failed: ") + ex.what() + "\n"); return false; }
}

bool PlaneSweepGpu::RunDepthPlaneSearchSmokeTest(
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
	const std::vector<int32_t> &expectedBestPlanes)
{
	if (!m_initialized) throw std::runtime_error("PlaneSweepGpu not initialized.");
	constexpr uint32_t kScenarios = 6;
	constexpr uint32_t kPlanes = 4;
	constexpr uint32_t kImages = 8;
	constexpr VkDeviceSize kImageBytes = 8 * 4;
	constexpr VkFormat kFormat = VK_FORMAT_R8G8B8A8_UNORM;
	if (referencePixels.size() != 4 || scenarioSourcePixels.size() != kScenarios || intrinsics.size() != 2 || poses.size() != 2 ||
		scenarioFloats.size() != kScenarios * 8 || scenarioParameters.size() != kScenarios * 4 || expectedPlaneDepths.size() != kScenarios * kPlanes ||
		expectedPlaneValidity.size() != kScenarios * kPlanes || expectedPlaneCosts.size() != kScenarios * kPlanes || expectedStatuses.size() != kScenarios ||
		expectedFinalDepths.size() != kScenarios || expectedBestDepths.size() != kScenarios || expectedBestCosts.size() != kScenarios ||
		expectedSecondBestCosts.size() != kScenarios || expectedBestPlanes.size() != kScenarios)
		throw std::invalid_argument("Depth-plane search smoke test received invalid input dimensions.");
	for (const auto &pixels : scenarioSourcePixels) if (pixels.size() != kImageBytes) throw std::invalid_argument("Depth-plane search scenario source image must be 8x1 RGBA8.");
	for (uint32_t i = 0; i < kScenarios; ++i) if (scenarioParameters[i * 4] != kPlanes) throw std::invalid_argument("Depth-plane search requires exactly four planes per scenario.");
	VkFormatProperties formatProperties{}; vkGetPhysicalDeviceFormatProperties(m_physicalDevice, kFormat, &formatProperties);
	if ((formatProperties.optimalTilingFeatures & (VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT | VK_FORMAT_FEATURE_TRANSFER_DST_BIT)) != (VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT | VK_FORMAT_FEATURE_TRANSFER_DST_BIT))
	{
		UIUtility::Print("Depth-plane search smoke test unsupported: RGBA8 optimal tiling lacks sampled-image or transfer-destination support.\n"); return false;
	}
	VkPhysicalDeviceProperties deviceProperties{}; vkGetPhysicalDeviceProperties(m_physicalDevice, &deviceProperties);
	const VkPhysicalDeviceLimits &limits = deviceProperties.limits;
	if (limits.maxPerStageDescriptorSamplers < 18 || limits.maxPerStageDescriptorSampledImages < 18 || limits.maxPerStageDescriptorStorageBuffers < 13 ||
		limits.maxDescriptorSetSamplers < 18 || limits.maxDescriptorSetSampledImages < 18 || limits.maxDescriptorSetStorageBuffers < 13 || limits.maxPerStageResources < 31)
	{
		UIUtility::Print("Depth-plane search smoke test unsupported: device descriptor limits are below 18 sampled images/samplers, 13 storage buffers, or 31 total per-stage resources.\n"); return false;
	}
	const VkDeviceSize stagingBytes = kImageBytes * 7 + 4;
	const std::array<const void *, 4> inputData{{ intrinsics.data(), poses.data(), scenarioFloats.data(), scenarioParameters.data() }};
	const std::array<VkDeviceSize, 4> inputSizes{{ 2 * sizeof(GpuCameraIntrinsics), 2 * sizeof(GpuCameraPose), scenarioFloats.size() * sizeof(float), scenarioParameters.size() * sizeof(uint32_t) }};
	const std::array<VkDeviceSize, 9> outputSizes{{ kScenarios * sizeof(uint32_t), kScenarios * sizeof(float), kScenarios * sizeof(float), kScenarios * sizeof(float), kScenarios * sizeof(float), kScenarios * sizeof(int32_t), kScenarios * kPlanes * sizeof(float), kScenarios * kPlanes * sizeof(uint32_t), kScenarios * kPlanes * sizeof(float) }};
	VkBuffer imageStaging = VK_NULL_HANDLE; VkDeviceMemory imageStagingMemory = VK_NULL_HANDLE;
	std::array<VkImage, kImages> images{}; std::array<VkDeviceMemory, kImages> imageMemories{}; std::array<VkImageView, kImages> imageViews{};
	VkSampler sampler = VK_NULL_HANDLE; std::array<VkBuffer, 4> inputStaging{}, inputBuffers{}; std::array<VkDeviceMemory, 4> inputStagingMemories{}, inputMemories{};
	std::array<VkBuffer, 9> outputBuffers{}, readbackBuffers{}; std::array<VkDeviceMemory, 9> outputMemories{}, readbackMemories{};
	VkDescriptorSetLayout setLayout = VK_NULL_HANDLE; VkDescriptorPool pool = VK_NULL_HANDLE; VkDescriptorSet set = VK_NULL_HANDLE;
	VkPipelineLayout pipelineLayout = VK_NULL_HANDLE; VkShaderModule shader = VK_NULL_HANDLE; VkPipeline pipeline = VK_NULL_HANDLE;
	VkCommandBuffer commandBuffer = VK_NULL_HANDLE; VkFence fence = VK_NULL_HANDLE; bool submitted = false;
	auto cleanup = [&]()
	{
		if (submitted) { vkDeviceWaitIdle(m_device); submitted = false; }
		if (fence) vkDestroyFence(m_device, fence, nullptr); if (commandBuffer) vkFreeCommandBuffers(m_device, m_commandPool, 1, &commandBuffer);
		if (pipeline) vkDestroyPipeline(m_device, pipeline, nullptr); if (shader) vkDestroyShaderModule(m_device, shader, nullptr); if (pipelineLayout) vkDestroyPipelineLayout(m_device, pipelineLayout, nullptr);
		if (pool) vkDestroyDescriptorPool(m_device, pool, nullptr); if (setLayout) vkDestroyDescriptorSetLayout(m_device, setLayout, nullptr); if (sampler) vkDestroySampler(m_device, sampler, nullptr);
		for (uint32_t i = 0; i < kImages; ++i) { if (imageViews[i]) vkDestroyImageView(m_device, imageViews[i], nullptr); if (images[i]) vkDestroyImage(m_device, images[i], nullptr); if (imageMemories[i]) vkFreeMemory(m_device, imageMemories[i], nullptr); }
		auto destroyBuffer = [&](VkBuffer &buffer, VkDeviceMemory &memory) { if (buffer) vkDestroyBuffer(m_device, buffer, nullptr); if (memory) vkFreeMemory(m_device, memory, nullptr); buffer = VK_NULL_HANDLE; memory = VK_NULL_HANDLE; };
		destroyBuffer(imageStaging, imageStagingMemory);
		for (uint32_t i = 0; i < 4; ++i) { destroyBuffer(inputStaging[i], inputStagingMemories[i]); destroyBuffer(inputBuffers[i], inputMemories[i]); }
		for (uint32_t i = 0; i < 9; ++i) { destroyBuffer(outputBuffers[i], outputMemories[i]); destroyBuffer(readbackBuffers[i], readbackMemories[i]); }
	};
	auto findMemory = [&](uint32_t bits, VkMemoryPropertyFlags flags) { VkPhysicalDeviceMemoryProperties p{}; vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &p); for (uint32_t i = 0; i < p.memoryTypeCount; ++i) if ((bits & (1u << i)) && (p.memoryTypes[i].propertyFlags & flags) == flags) return i; return UINT32_MAX; };
	auto makeBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags flags, VkBuffer &buffer, VkDeviceMemory &memory) { VkBufferCreateInfo ci{}; ci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO; ci.size = size; ci.usage = usage; ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE; if (vkCreateBuffer(m_device, &ci, nullptr, &buffer) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-plane buffer."); VkMemoryRequirements req{}; vkGetBufferMemoryRequirements(m_device, buffer, &req); uint32_t type = findMemory(req.memoryTypeBits, flags); if (type == UINT32_MAX) throw std::runtime_error("No memory type for depth-plane buffer."); VkMemoryAllocateInfo ai{}; ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; ai.allocationSize = req.size; ai.memoryTypeIndex = type; if (vkAllocateMemory(m_device, &ai, nullptr, &memory) != VK_SUCCESS || vkBindBufferMemory(m_device, buffer, memory, 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate depth-plane buffer memory."); };
	try
	{
		const VkMemoryPropertyFlags host = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
		makeBuffer(stagingBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, host, imageStaging, imageStagingMemory);
		void *mapped = nullptr; if (vkMapMemory(m_device, imageStagingMemory, 0, stagingBytes, 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map depth-plane image staging.");
		std::memcpy(mapped, referencePixels.data(), 4); for (uint32_t i = 0; i < kScenarios; ++i) std::memcpy(static_cast<uint8_t *>(mapped) + kImageBytes * (i + 1), scenarioSourcePixels[i].data(), static_cast<size_t>(kImageBytes));
		const std::array<uint8_t, 4> dummy{{0, 0, 0, 255}}; std::memcpy(static_cast<uint8_t *>(mapped) + kImageBytes * 7, dummy.data(), dummy.size()); vkUnmapMemory(m_device, imageStagingMemory);
		for (uint32_t i = 0; i < kImages; ++i)
		{
			VkImageCreateInfo ci{}; ci.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO; ci.imageType = VK_IMAGE_TYPE_2D; ci.format = kFormat; ci.extent = (i == 0 || i == 7) ? VkExtent3D{1, 1, 1} : VkExtent3D{8, 1, 1}; ci.mipLevels = 1; ci.arrayLayers = 1; ci.samples = VK_SAMPLE_COUNT_1_BIT; ci.tiling = VK_IMAGE_TILING_OPTIMAL; ci.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT; ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE; ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			if (vkCreateImage(m_device, &ci, nullptr, &images[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-plane image."); VkMemoryRequirements req{}; vkGetImageMemoryRequirements(m_device, images[i], &req); uint32_t type = findMemory(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT); if (type == UINT32_MAX) throw std::runtime_error("No device-local memory for depth-plane image."); VkMemoryAllocateInfo ai{}; ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; ai.allocationSize = req.size; ai.memoryTypeIndex = type; if (vkAllocateMemory(m_device, &ai, nullptr, &imageMemories[i]) != VK_SUCCESS || vkBindImageMemory(m_device, images[i], imageMemories[i], 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate depth-plane image.");
			VkImageViewCreateInfo vi{}; vi.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO; vi.image = images[i]; vi.viewType = VK_IMAGE_VIEW_TYPE_2D; vi.format = kFormat; vi.subresourceRange = VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}; if (vkCreateImageView(m_device, &vi, nullptr, &imageViews[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-plane image view.");
		}
		VkSamplerCreateInfo si{}; si.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO; si.magFilter = VK_FILTER_NEAREST; si.minFilter = VK_FILTER_NEAREST; si.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST; si.addressModeU = si.addressModeV = si.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE; si.anisotropyEnable = VK_FALSE; si.compareEnable = VK_FALSE; si.minLod = 0.0f; si.maxLod = 0.0f; if (vkCreateSampler(m_device, &si, nullptr, &sampler) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-plane sampler.");
		for (uint32_t i = 0; i < 4; ++i) { makeBuffer(inputSizes[i], VK_BUFFER_USAGE_TRANSFER_SRC_BIT, host, inputStaging[i], inputStagingMemories[i]); if (vkMapMemory(m_device, inputStagingMemories[i], 0, inputSizes[i], 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map depth-plane input staging."); std::memcpy(mapped, inputData[i], static_cast<size_t>(inputSizes[i])); vkUnmapMemory(m_device, inputStagingMemories[i]); makeBuffer(inputSizes[i], VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, inputBuffers[i], inputMemories[i]); }
		for (uint32_t i = 0; i < 9; ++i) { makeBuffer(outputSizes[i], VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, outputBuffers[i], outputMemories[i]); makeBuffer(outputSizes[i], VK_BUFFER_USAGE_TRANSFER_DST_BIT, host, readbackBuffers[i], readbackMemories[i]); }
		std::array<VkDescriptorSetLayoutBinding, 17> bindings{}; for (uint32_t i = 0; i < bindings.size(); ++i) { bindings[i].binding = i; bindings[i].descriptorType = i < 4 ? VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; bindings[i].descriptorCount = (i == 2 || i == 3) ? 8 : 1; bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT; }
		VkDescriptorSetLayoutCreateInfo lci{}; lci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO; lci.bindingCount = static_cast<uint32_t>(bindings.size()); lci.pBindings = bindings.data(); if (vkCreateDescriptorSetLayout(m_device, &lci, nullptr, &setLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-plane descriptor layout.");
		std::array<VkDescriptorPoolSize, 2> poolSizes{{ {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 18}, {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 13} }}; VkDescriptorPoolCreateInfo pci{}; pci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO; pci.maxSets = 1; pci.poolSizeCount = static_cast<uint32_t>(poolSizes.size()); pci.pPoolSizes = poolSizes.data(); if (vkCreateDescriptorPool(m_device, &pci, nullptr, &pool) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-plane descriptor pool.");
		VkDescriptorSetAllocateInfo sai{}; sai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO; sai.descriptorPool = pool; sai.descriptorSetCount = 1; sai.pSetLayouts = &setLayout; if (vkAllocateDescriptorSets(m_device, &sai, &set) != VK_SUCCESS) throw std::runtime_error("Failed to allocate depth-plane descriptor set.");
		std::array<VkDescriptorImageInfo, 1> referenceInfo{{{sampler, imageViews[0], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}}}, dummyInfo{{{sampler, imageViews[7], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}}};
		std::array<VkDescriptorImageInfo, 8> sourceInfos{}, maskInfos{}; for (uint32_t i = 0; i < 8; ++i) { sourceInfos[i] = {sampler, i < 6 ? imageViews[i + 1] : imageViews[7], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}; maskInfos[i] = {sampler, imageViews[7], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}; }
		std::array<VkDescriptorBufferInfo, 13> bufferInfos{}; for (uint32_t i = 0; i < 4; ++i) bufferInfos[i] = {inputBuffers[i], 0, inputSizes[i]}; for (uint32_t i = 0; i < 9; ++i) bufferInfos[4 + i] = {outputBuffers[i], 0, outputSizes[i]};
		std::array<VkWriteDescriptorSet, 17> writes{}; for (uint32_t i = 0; i < writes.size(); ++i) { writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; writes[i].dstSet = set; writes[i].dstBinding = i; writes[i].descriptorType = i < 4 ? VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; }
		writes[0].descriptorCount = 1; writes[0].pImageInfo = referenceInfo.data(); writes[1].descriptorCount = 1; writes[1].pImageInfo = dummyInfo.data(); writes[2].descriptorCount = 8; writes[2].pImageInfo = sourceInfos.data(); writes[3].descriptorCount = 8; writes[3].pImageInfo = maskInfos.data(); for (uint32_t i = 0; i < 13; ++i) { writes[4 + i].descriptorCount = 1; writes[4 + i].pBufferInfo = &bufferInfos[i]; } vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
		const std::vector<char> code = ReadShaderFile("shaders/planeSweepDepthPlaneSearchSmokeTest.comp.spv"); if (code.empty()) throw std::runtime_error("Failed to load depth-plane search SPIR-V."); VkShaderModuleCreateInfo sci{}; sci.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO; sci.codeSize = code.size(); sci.pCode = reinterpret_cast<const uint32_t *>(code.data()); if (vkCreateShaderModule(m_device, &sci, nullptr, &shader) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-plane shader.");
		VkPipelineLayoutCreateInfo pli{}; pli.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO; pli.setLayoutCount = 1; pli.pSetLayouts = &setLayout; if (vkCreatePipelineLayout(m_device, &pli, nullptr, &pipelineLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-plane pipeline layout."); VkPipelineShaderStageCreateInfo stage{}; stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = shader; stage.pName = "main"; VkComputePipelineCreateInfo cpci{}; cpci.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO; cpci.stage = stage; cpci.layout = pipelineLayout; if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &cpci, nullptr, &pipeline) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-plane pipeline.");
		VkCommandBufferAllocateInfo cai{}; cai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO; cai.commandPool = m_commandPool; cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; cai.commandBufferCount = 1; if (vkAllocateCommandBuffers(m_device, &cai, &commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to allocate depth-plane command buffer."); VkCommandBufferBeginInfo cbi{}; cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO; cbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT; if (vkBeginCommandBuffer(commandBuffer, &cbi) != VK_SUCCESS) throw std::runtime_error("Failed to begin depth-plane command buffer.");
		std::array<VkImageMemoryBarrier, kImages> imageToTransfer{}; for (uint32_t i = 0; i < kImages; ++i) { auto &b = imageToTransfer[i]; b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; b.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED; b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; b.image = images[i]; b.subresourceRange = VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, kImages, imageToTransfer.data());
		for (uint32_t i = 0; i < kImages; ++i) { VkBufferImageCopy copy{}; copy.bufferOffset = i == 0 ? 0 : kImageBytes * i; copy.imageSubresource = VkImageSubresourceLayers{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1}; copy.imageExtent = (i == 0 || i == 7) ? VkExtent3D{1, 1, 1} : VkExtent3D{8, 1, 1}; vkCmdCopyBufferToImage(commandBuffer, imageStaging, images[i], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy); }
		std::array<VkImageMemoryBarrier, kImages> imageToRead{}; for (uint32_t i = 0; i < kImages; ++i) { auto &b = imageToRead[i]; b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT; b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; b.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; b.image = images[i]; b.subresourceRange = VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 0, nullptr, kImages, imageToRead.data());
		for (uint32_t i = 0; i < 4; ++i) { VkBufferCopy copy{}; copy.size = inputSizes[i]; vkCmdCopyBuffer(commandBuffer, inputStaging[i], inputBuffers[i], 1, &copy); } std::array<VkBufferMemoryBarrier, 4> inputBarriers{}; for (uint32_t i = 0; i < 4; ++i) { auto &b = inputBarriers[i]; b.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT; b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; b.buffer = inputBuffers[i]; b.size = inputSizes[i]; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 4, inputBarriers.data(), 0, nullptr);
		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline); vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &set, 0, nullptr); vkCmdDispatch(commandBuffer, 1, 1, 1);
		std::array<VkBufferMemoryBarrier, 9> outputBarriers{}; for (uint32_t i = 0; i < 9; ++i) { auto &b = outputBarriers[i]; b.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; b.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT; b.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT; b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; b.buffer = outputBuffers[i]; b.size = outputSizes[i]; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 9, outputBarriers.data(), 0, nullptr);
		for (uint32_t i = 0; i < 9; ++i) { VkBufferCopy copy{}; copy.size = outputSizes[i]; vkCmdCopyBuffer(commandBuffer, outputBuffers[i], readbackBuffers[i], 1, &copy); } std::array<VkBufferMemoryBarrier, 9> hostBarriers{}; for (uint32_t i = 0; i < 9; ++i) { auto &b = hostBarriers[i]; b.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; b.dstAccessMask = VK_ACCESS_HOST_READ_BIT; b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; b.buffer = readbackBuffers[i]; b.size = outputSizes[i]; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 0, nullptr, 9, hostBarriers.data(), 0, nullptr);
		if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to end depth-plane command buffer."); VkFenceCreateInfo fi{}; fi.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO; if (vkCreateFence(m_device, &fi, nullptr, &fence) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-plane fence."); VkSubmitInfo submit{}; submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO; submit.commandBufferCount = 1; submit.pCommandBuffers = &commandBuffer; if (vkQueueSubmit(m_queue, 1, &submit, fence) != VK_SUCCESS) throw std::runtime_error("Failed to submit depth-plane search."); submitted = true; if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS) throw std::runtime_error("Failed waiting for depth-plane fence."); submitted = false;
		std::array<uint32_t, kScenarios> gpuStatuses{}; std::array<uint32_t, kScenarios * kPlanes> gpuPlaneValidity{}; std::array<float, kScenarios> gpuFinalDepths{}, gpuBestDepths{}, gpuBestCosts{}, gpuSecondBestCosts{}; std::array<int32_t, kScenarios> gpuBestPlane{}; std::array<float, kScenarios * kPlanes> gpuPlaneDepths{}, gpuPlaneCosts{};
		std::array<void *, 9> targets{{gpuStatuses.data(), gpuFinalDepths.data(), gpuBestDepths.data(), gpuBestCosts.data(), gpuSecondBestCosts.data(), gpuBestPlane.data(), gpuPlaneDepths.data(), gpuPlaneValidity.data(), gpuPlaneCosts.data()}};
		for (uint32_t i = 0; i < 9; ++i) { if (vkMapMemory(m_device, readbackMemories[i], 0, outputSizes[i], 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map depth-plane readback."); std::memcpy(targets[i], mapped, static_cast<size_t>(outputSizes[i])); vkUnmapMemory(m_device, readbackMemories[i]); }
		constexpr uint32_t kCostRejected = 1u;
		constexpr uint32_t kConfidenceRejected = 2u;
		constexpr uint32_t kAccepted = 3u;
		constexpr float kTolerance = 1e-6f; float maximumDepthError = 0.0f, maximumCostError = 0.0f, maximumFinalDepthError = 0.0f; uint32_t validPlaneCount = 0; bool tieOrderingMatched = true, thresholdRejected = gpuStatuses[1] == kCostRejected, confidenceRejected = gpuStatuses[2] == kConfidenceRejected;
		for (uint32_t i = 0; i < kScenarios; ++i)
		{
			if (gpuStatuses[i] != expectedStatuses[i] || gpuBestPlane[i] != expectedBestPlanes[i] || std::fabs(gpuFinalDepths[i] - expectedFinalDepths[i]) > kTolerance || std::fabs(gpuBestDepths[i] - expectedBestDepths[i]) > kTolerance || std::fabs(gpuBestCosts[i] - expectedBestCosts[i]) > kTolerance || std::fabs(gpuSecondBestCosts[i] - expectedSecondBestCosts[i]) > kTolerance)
			{
				UIUtility::Print("Depth-plane selection mismatch case=" + std::to_string(i) + ", status=" + std::to_string(gpuStatuses[i]) + ", bestPlane=" + std::to_string(gpuBestPlane[i]) + ", bestDepth=" + std::to_string(gpuBestDepths[i]) + ", bestCost=" + std::to_string(gpuBestCosts[i]) + ", secondBestCost=" + std::to_string(gpuSecondBestCosts[i]) + ", finalDepth=" + std::to_string(gpuFinalDepths[i]) + ".\n"); cleanup(); return false;
			}
			maximumFinalDepthError = std::max(maximumFinalDepthError, std::fabs(gpuFinalDepths[i] - expectedFinalDepths[i]));
			for (uint32_t p = 0; p < kPlanes; ++p)
			{
				const uint32_t index = i * kPlanes + p; const float depthError = std::fabs(gpuPlaneDepths[index] - expectedPlaneDepths[index]); maximumDepthError = std::max(maximumDepthError, depthError);
				if (depthError > kTolerance || gpuPlaneValidity[index] != expectedPlaneValidity[index]) { UIUtility::Print("Depth-plane diagnostic mismatch case=" + std::to_string(i) + ", plane=" + std::to_string(p) + ", CPU depth=" + std::to_string(expectedPlaneDepths[index]) + ", GPU depth=" + std::to_string(gpuPlaneDepths[index]) + ", CPU valid=" + std::to_string(expectedPlaneValidity[index]) + ", GPU valid=" + std::to_string(gpuPlaneValidity[index]) + ".\n"); cleanup(); return false; }
				if (expectedPlaneValidity[index]) { ++validPlaneCount; const float error = std::fabs(gpuPlaneCosts[index] - expectedPlaneCosts[index]); maximumCostError = std::max(maximumCostError, error); if (error > kTolerance) { UIUtility::Print("Depth-plane cost mismatch case=" + std::to_string(i) + ", plane=" + std::to_string(p) + ", CPU=" + std::to_string(expectedPlaneCosts[index]) + ", GPU=" + std::to_string(gpuPlaneCosts[index]) + ".\n"); cleanup(); return false; } }
			}
			if ((i == 2 || i == 5) && gpuBestPlane[i] != 2) tieOrderingMatched = false;
		}
		if (!tieOrderingMatched || !thresholdRejected || !confidenceRejected) { UIUtility::Print("Depth-plane strict tie/threshold/confidence diagnostics failed.\n"); cleanup(); return false; }
		UIUtility::Print("PlaneSweep GPU depth-plane search smoke test succeeded:\n6 / 6 final depth results matched CPU BuildReferencePoints.\n24 / 24 plane depths matched inverse-depth CPU interpolation.\n24 / 24 plane validity results matched CPU ComputePlaneCost.\n" + std::to_string(validPlaneCount) + " / " + std::to_string(validPlaneCount) + " valid plane costs matched.\n6 / 6 best/second-best selections matched.\nTie ordering matched strict CPU '<' semantics.\nCost-threshold rejection matched.\nConfidence rejection matched.\nMaximum plane-depth absolute error: " + std::to_string(maximumDepthError) + "\nMaximum valid plane-cost absolute error: " + std::to_string(maximumCostError) + "\nMaximum final-depth absolute error: " + std::to_string(maximumFinalDepthError) + "\n");
		for (uint32_t i = 0; i < kScenarios; ++i) { const char *status = gpuStatuses[i] == kAccepted ? "Accepted" : gpuStatuses[i] == kCostRejected ? "CostRejected" : gpuStatuses[i] == kConfidenceRejected ? "ConfidenceRejected" : "NoValidPlane"; std::string line = "case " + std::to_string(i) + ": status=" + status + ", bestPlane=" + std::to_string(gpuBestPlane[i]) + ", bestDepth=" + std::to_string(gpuBestDepths[i]) + ", bestCost=" + std::to_string(gpuBestCosts[i]) + ", secondBestCost=" + std::to_string(gpuSecondBestCosts[i]) + ", finalDepth=" + std::to_string(gpuFinalDepths[i]) + ", planeDepths=["; for (uint32_t p = 0; p < kPlanes; ++p) line += (p ? "," : "") + std::to_string(gpuPlaneDepths[i * kPlanes + p]); line += "], planeCosts=["; for (uint32_t p = 0; p < kPlanes; ++p) { const uint32_t j = i * kPlanes + p; line += (p ? "," : "") + (gpuPlaneValidity[j] ? std::to_string(gpuPlaneCosts[j]) : std::string("invalid")); } UIUtility::Print(line + "]\n"); }
		cleanup(); return true;
	}
	catch (const std::exception &ex) { cleanup(); UIUtility::Print(std::string("PlaneSweep GPU depth-plane search failed: ") + ex.what() + "\n"); return false; }
}

// Keep the direct texture-standard-deviation smoke-test implementation alongside
// the depth-map smoke-test dispatch path for easier maintenance and comparison.
bool PlaneSweepGpu::RunTextureStdDevSmokeTest(
	const std::vector<std::vector<uint8_t>> &casePixels,
	const std::vector<float> &caseParameters,
	const std::vector<uint32_t> &expectedValidity,
	const std::vector<float> &expectedStdDevs,
	const std::vector<uint32_t> &expectedSampleCounts,
	float *maximumStdDevError)
{
	if (!m_initialized) throw std::runtime_error("PlaneSweepGpu not initialized.");
	constexpr uint32_t kCases = 7; constexpr uint32_t kWidth = 5; constexpr uint32_t kHeight = 5; constexpr uint32_t kImageBytes = kWidth * kHeight * 4; constexpr uint32_t kTraceWordsPerCase = 100; constexpr VkFormat kFormat = VK_FORMAT_R8G8B8A8_UNORM;
	if (casePixels.size() != kCases || caseParameters.size() != kCases * 4 || expectedValidity.size() != kCases || expectedStdDevs.size() != kCases || expectedSampleCounts.size() != kCases) throw std::invalid_argument("Texture standard-deviation smoke test expects seven cases.");
	for (const auto &pixels : casePixels) if (pixels.size() != kImageBytes) throw std::invalid_argument("Texture standard-deviation test images must be 5x5 RGBA8.");
	VkBuffer staging = VK_NULL_HANDLE; VkDeviceMemory stagingMemory = VK_NULL_HANDLE;
	std::array<VkImage, 8> images{}; std::array<VkDeviceMemory, 8> imageMemories{}; std::array<VkImageView, 8> imageViews{}; VkSampler sampler = VK_NULL_HANDLE;
	std::array<VkBuffer, 5> buffers{}; std::array<VkDeviceMemory, 5> memories{};
	VkDescriptorSetLayout setLayout = VK_NULL_HANDLE; VkDescriptorPool pool = VK_NULL_HANDLE; VkDescriptorSet set = VK_NULL_HANDLE; VkPipelineLayout pipelineLayout = VK_NULL_HANDLE; VkShaderModule shader = VK_NULL_HANDLE; VkPipeline pipeline = VK_NULL_HANDLE; VkCommandBuffer commandBuffer = VK_NULL_HANDLE; VkFence fence = VK_NULL_HANDLE; bool submitted = false;
	auto cleanup = [&]() { if (submitted) { vkDeviceWaitIdle(m_device); submitted = false; } if (fence) vkDestroyFence(m_device, fence, nullptr); if (commandBuffer) vkFreeCommandBuffers(m_device, m_commandPool, 1, &commandBuffer); if (pipeline) vkDestroyPipeline(m_device, pipeline, nullptr); if (shader) vkDestroyShaderModule(m_device, shader, nullptr); if (pipelineLayout) vkDestroyPipelineLayout(m_device, pipelineLayout, nullptr); if (pool) vkDestroyDescriptorPool(m_device, pool, nullptr); if (setLayout) vkDestroyDescriptorSetLayout(m_device, setLayout, nullptr); if (sampler) vkDestroySampler(m_device, sampler, nullptr); for (uint32_t i = 0; i < images.size(); ++i) { if (imageViews[i]) vkDestroyImageView(m_device, imageViews[i], nullptr); if (images[i]) vkDestroyImage(m_device, images[i], nullptr); if (imageMemories[i]) vkFreeMemory(m_device, imageMemories[i], nullptr); } if (staging) vkDestroyBuffer(m_device, staging, nullptr); if (stagingMemory) vkFreeMemory(m_device, stagingMemory, nullptr); for (uint32_t i = 0; i < buffers.size(); ++i) { if (buffers[i]) vkDestroyBuffer(m_device, buffers[i], nullptr); if (memories[i]) vkFreeMemory(m_device, memories[i], nullptr); } };
	auto findMemory = [&](uint32_t bits, VkMemoryPropertyFlags flags) { VkPhysicalDeviceMemoryProperties p{}; vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &p); for (uint32_t i = 0; i < p.memoryTypeCount; ++i) if ((bits & (1u << i)) && (p.memoryTypes[i].propertyFlags & flags) == flags) return i; return UINT32_MAX; };
	auto makeBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkBuffer &buffer, VkDeviceMemory &memory) { VkBufferCreateInfo ci{}; ci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO; ci.size = size; ci.usage = usage; ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE; if (vkCreateBuffer(m_device, &ci, nullptr, &buffer) != VK_SUCCESS) throw std::runtime_error("Failed to create texture standard-deviation buffer."); VkMemoryRequirements req{}; vkGetBufferMemoryRequirements(m_device, buffer, &req); uint32_t type = findMemory(req.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT); if (type == UINT32_MAX) throw std::runtime_error("No host-coherent memory for texture standard-deviation buffer."); VkMemoryAllocateInfo ai{}; ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; ai.allocationSize = req.size; ai.memoryTypeIndex = type; if (vkAllocateMemory(m_device, &ai, nullptr, &memory) != VK_SUCCESS || vkBindBufferMemory(m_device, buffer, memory, 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate texture standard-deviation buffer memory."); };
	try
	{
		const VkDeviceSize stagingBytes = kImageBytes * kCases + 4; makeBuffer(stagingBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, staging, stagingMemory); void *mapped = nullptr; if (vkMapMemory(m_device, stagingMemory, 0, stagingBytes, 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map texture test staging buffer."); for (uint32_t i = 0; i < kCases; ++i) std::memcpy(static_cast<uint8_t *>(mapped) + i * kImageBytes, casePixels[i].data(), kImageBytes); const std::array<uint8_t, 4> dummy{{0, 0, 0, 255}}; std::memcpy(static_cast<uint8_t *>(mapped) + kImageBytes * kCases, dummy.data(), dummy.size()); vkUnmapMemory(m_device, stagingMemory);
		for (uint32_t i = 0; i < images.size(); ++i) { VkImageCreateInfo ci{}; ci.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO; ci.imageType = VK_IMAGE_TYPE_2D; ci.format = kFormat; ci.extent = i == kCases ? VkExtent3D{1, 1, 1} : VkExtent3D{kWidth, kHeight, 1}; ci.mipLevels = 1; ci.arrayLayers = 1; ci.samples = VK_SAMPLE_COUNT_1_BIT; ci.tiling = VK_IMAGE_TILING_OPTIMAL; ci.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT; ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE; ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED; if (vkCreateImage(m_device, &ci, nullptr, &images[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create texture test image."); VkMemoryRequirements req{}; vkGetImageMemoryRequirements(m_device, images[i], &req); uint32_t type = findMemory(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT); if (type == UINT32_MAX) throw std::runtime_error("No device-local memory for texture test image."); VkMemoryAllocateInfo ai{}; ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; ai.allocationSize = req.size; ai.memoryTypeIndex = type; if (vkAllocateMemory(m_device, &ai, nullptr, &imageMemories[i]) != VK_SUCCESS || vkBindImageMemory(m_device, images[i], imageMemories[i], 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate texture test image memory."); VkImageViewCreateInfo vi{}; vi.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO; vi.image = images[i]; vi.viewType = VK_IMAGE_VIEW_TYPE_2D; vi.format = kFormat; vi.subresourceRange = VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}; if (vkCreateImageView(m_device, &vi, nullptr, &imageViews[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create texture test image view."); }
		VkSamplerCreateInfo si{}; si.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO; si.magFilter = VK_FILTER_NEAREST; si.minFilter = VK_FILTER_NEAREST; si.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST; si.addressModeU = si.addressModeV = si.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE; si.maxLod = 0.0f; if (vkCreateSampler(m_device, &si, nullptr, &sampler) != VK_SUCCESS) throw std::runtime_error("Failed to create texture test sampler.");
		// sizes[4] is the direct trace buffer: kCases * kTraceWordsPerCase uint32 words (currently 100 words per case).
		const std::array<VkDeviceSize, 5> sizes{{caseParameters.size() * sizeof(float), kCases * sizeof(uint32_t), kCases * sizeof(float), kCases * sizeof(uint32_t), kCases * kTraceWordsPerCase * sizeof(uint32_t)}}; for (uint32_t i = 0; i < buffers.size(); ++i) makeBuffer(sizes[i], VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, buffers[i], memories[i]);
		for (uint32_t i = 0; i < buffers.size(); ++i) { if (vkMapMemory(m_device, memories[i], 0, sizes[i], 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map texture test buffer."); if (i == 0) std::memcpy(mapped, caseParameters.data(), static_cast<size_t>(sizes[i])); else std::memset(mapped, 0, static_cast<size_t>(sizes[i])); vkUnmapMemory(m_device, memories[i]); }
		std::array<VkDescriptorSetLayoutBinding, 7> bindings{}; for (uint32_t i = 0; i < bindings.size(); ++i) { bindings[i].binding = i; bindings[i].descriptorType = i < 2 ? VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; bindings[i].descriptorCount = i < 2 ? 8 : 1; bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT; } VkDescriptorSetLayoutCreateInfo lci{}; lci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO; lci.bindingCount = static_cast<uint32_t>(bindings.size()); lci.pBindings = bindings.data(); if (vkCreateDescriptorSetLayout(m_device, &lci, nullptr, &setLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create texture test descriptor layout.");
		constexpr uint32_t kStorageBufferDescriptorCount = 5;
		std::array<VkDescriptorPoolSize, 2> poolSizes{{{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 16}, {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, kStorageBufferDescriptorCount}}}; VkDescriptorPoolCreateInfo pci{}; pci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO; pci.maxSets = 1; pci.poolSizeCount = static_cast<uint32_t>(poolSizes.size()); pci.pPoolSizes = poolSizes.data(); if (vkCreateDescriptorPool(m_device, &pci, nullptr, &pool) != VK_SUCCESS) throw std::runtime_error("Failed to create texture test descriptor pool."); VkDescriptorSetAllocateInfo sai{}; sai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO; sai.descriptorPool = pool; sai.descriptorSetCount = 1; sai.pSetLayouts = &setLayout; if (vkAllocateDescriptorSets(m_device, &sai, &set) != VK_SUCCESS) throw std::runtime_error("Failed to allocate texture test descriptor set.");
		std::array<VkDescriptorImageInfo, 8> imageInfos{}, maskInfos{}; for (uint32_t i = 0; i < 8; ++i) { imageInfos[i] = {sampler, imageViews[std::min(i, kCases - 1)], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}; maskInfos[i] = {sampler, imageViews[kCases], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}; } std::array<VkDescriptorBufferInfo, 5> bufferInfos{}; for (uint32_t i = 0; i < buffers.size(); ++i) bufferInfos[i] = {buffers[i], 0, sizes[i]}; std::array<VkWriteDescriptorSet, 7> writes{}; for (uint32_t i = 0; i < writes.size(); ++i) { writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; writes[i].dstSet = set; writes[i].dstBinding = i; writes[i].descriptorType = i < 2 ? VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; } writes[0].descriptorCount = writes[1].descriptorCount = 8; writes[0].pImageInfo = imageInfos.data(); writes[1].pImageInfo = maskInfos.data(); for (uint32_t i = 0; i < 5; ++i) { writes[i + 2].descriptorCount = 1; writes[i + 2].pBufferInfo = &bufferInfos[i]; } vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
		const auto code = ReadShaderFile("shaders/planeSweepTextureStdDevSmokeTest.comp.spv"); if (code.empty()) throw std::runtime_error("Failed to load texture standard-deviation shader."); VkShaderModuleCreateInfo sci{}; sci.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO; sci.codeSize = code.size(); sci.pCode = reinterpret_cast<const uint32_t *>(code.data()); if (vkCreateShaderModule(m_device, &sci, nullptr, &shader) != VK_SUCCESS) throw std::runtime_error("Failed to create texture standard-deviation shader."); VkPipelineLayoutCreateInfo pli{}; pli.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO; pli.setLayoutCount = 1; pli.pSetLayouts = &setLayout; if (vkCreatePipelineLayout(m_device, &pli, nullptr, &pipelineLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create texture test pipeline layout."); VkPipelineShaderStageCreateInfo stage{}; stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = shader; stage.pName = "main"; VkComputePipelineCreateInfo cpci{}; cpci.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO; cpci.stage = stage; cpci.layout = pipelineLayout; if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &cpci, nullptr, &pipeline) != VK_SUCCESS) throw std::runtime_error("Failed to create texture test pipeline.");
		VkCommandBufferAllocateInfo cai{}; cai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO; cai.commandPool = m_commandPool; cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; cai.commandBufferCount = 1; if (vkAllocateCommandBuffers(m_device, &cai, &commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to allocate texture test command buffer."); VkCommandBufferBeginInfo cbi{}; cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO; cbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT; if (vkBeginCommandBuffer(commandBuffer, &cbi) != VK_SUCCESS) throw std::runtime_error("Failed to begin texture test command buffer.");
		std::array<VkImageMemoryBarrier, 8> imageBarriers{}; for (uint32_t i = 0; i < images.size(); ++i) { auto &b = imageBarriers[i]; b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; b.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED; b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; b.image = images[i]; b.subresourceRange = VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, static_cast<uint32_t>(imageBarriers.size()), imageBarriers.data()); for (uint32_t i = 0; i < images.size(); ++i) { VkBufferImageCopy copy{}; copy.bufferOffset = i == kCases ? kImageBytes * kCases : kImageBytes * i; copy.imageSubresource = VkImageSubresourceLayers{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1}; copy.imageExtent = i == kCases ? VkExtent3D{1, 1, 1} : VkExtent3D{kWidth, kHeight, 1}; vkCmdCopyBufferToImage(commandBuffer, staging, images[i], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy); } for (auto &b : imageBarriers) { b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT; b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; b.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 0, nullptr, static_cast<uint32_t>(imageBarriers.size()), imageBarriers.data()); vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline); vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &set, 0, nullptr); vkCmdDispatch(commandBuffer, 2, 1, 1); if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to record texture test command buffer."); VkFenceCreateInfo fci{}; fci.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO; if (vkCreateFence(m_device, &fci, nullptr, &fence) != VK_SUCCESS) throw std::runtime_error("Failed to create texture test fence."); VkSubmitInfo submit{}; submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO; submit.commandBufferCount = 1; submit.pCommandBuffers = &commandBuffer; if (vkQueueSubmit(m_queue, 1, &submit, fence) != VK_SUCCESS) throw std::runtime_error("Failed to submit texture test."); submitted = true; if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS) throw std::runtime_error("Failed to wait for texture test."); submitted = false;
		VkBufferMemoryBarrier outputBarrier{}; outputBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; outputBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT; outputBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT; outputBarrier.srcQueueFamilyIndex = outputBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; outputBarrier.buffer = buffers[1]; outputBarrier.size = sizes[1]; std::array<VkBufferMemoryBarrier, 4> outputBarriers{}; for (uint32_t i = 0; i < outputBarriers.size(); ++i) { outputBarriers[i] = outputBarrier; outputBarriers[i].buffer = buffers[i + 1]; outputBarriers[i].size = sizes[i + 1]; }
		VkCommandBuffer readBarrierCommand = VK_NULL_HANDLE; VkCommandBufferAllocateInfo readBarrierAlloc{}; readBarrierAlloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO; readBarrierAlloc.commandPool = m_commandPool; readBarrierAlloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; readBarrierAlloc.commandBufferCount = 1; if (vkAllocateCommandBuffers(m_device, &readBarrierAlloc, &readBarrierCommand) != VK_SUCCESS) throw std::runtime_error("Failed to allocate texture test visibility command buffer."); VkCommandBufferBeginInfo readBarrierBegin{}; readBarrierBegin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO; readBarrierBegin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT; if (vkBeginCommandBuffer(readBarrierCommand, &readBarrierBegin) != VK_SUCCESS) { vkFreeCommandBuffers(m_device, m_commandPool, 1, &readBarrierCommand); throw std::runtime_error("Failed to begin texture test visibility command buffer."); } vkCmdPipelineBarrier(readBarrierCommand, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 0, nullptr, static_cast<uint32_t>(outputBarriers.size()), outputBarriers.data(), 0, nullptr); if (vkEndCommandBuffer(readBarrierCommand) != VK_SUCCESS) { vkFreeCommandBuffers(m_device, m_commandPool, 1, &readBarrierCommand); throw std::runtime_error("Failed to end texture test visibility command buffer."); } VkSubmitInfo readBarrierSubmit{}; readBarrierSubmit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO; readBarrierSubmit.commandBufferCount = 1; readBarrierSubmit.pCommandBuffers = &readBarrierCommand; if (vkQueueSubmit(m_queue, 1, &readBarrierSubmit, VK_NULL_HANDLE) != VK_SUCCESS || vkQueueWaitIdle(m_queue) != VK_SUCCESS) { vkFreeCommandBuffers(m_device, m_commandPool, 1, &readBarrierCommand); throw std::runtime_error("Failed to submit texture test visibility barrier."); } vkFreeCommandBuffers(m_device, m_commandPool, 1, &readBarrierCommand);
		std::array<uint32_t, kCases> validity{}, sampleCounts{}; std::array<float, kCases> stdDevs{}; std::vector<uint32_t> traces(kCases * kTraceWordsPerCase); for (uint32_t i = 0; i < 4; ++i) { if (vkMapMemory(m_device, memories[i + 1], 0, sizes[i + 1], 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map texture test readback."); if (i == 0) std::memcpy(validity.data(), mapped, static_cast<size_t>(sizes[i + 1])); else if (i == 1) std::memcpy(stdDevs.data(), mapped, static_cast<size_t>(sizes[i + 1])); else if (i == 2) std::memcpy(sampleCounts.data(), mapped, static_cast<size_t>(sizes[i + 1])); else std::memcpy(traces.data(), mapped, static_cast<size_t>(sizes[i + 1])); vkUnmapMemory(m_device, memories[i + 1]); }
	uint32_t validityMatches = 0; uint32_t validValueMatches = 0; bool success = true; float maxError = 0.0f;
	for (uint32_t i = 0; i < kCases; ++i)
	{
		const bool gpuValid = validity[i] != 0; const bool cpuValid = expectedValidity[i] != 0; const float error = cpuValid ? std::abs(stdDevs[i] - expectedStdDevs[i]) : 0.0f;
		if (i < 6u)
		{
			maxError = std::max(maxError, error);
			if (gpuValid == cpuValid) ++validityMatches;
			if (gpuValid != cpuValid || (cpuValid && error > 1.0e-6f) || sampleCounts[i] != expectedSampleCounts[i]) success = false;
			if (cpuValid && error <= 1.0e-6f) ++validValueMatches;
		}
		else if (gpuValid != cpuValid || sampleCounts[i] != expectedSampleCounts[i])
		{
			success = false;
		}
		std::ostringstream detail; detail << std::setprecision(std::numeric_limits<float>::max_digits10) << "Texture stdDev case " << i << ": CPU valid=" << cpuValid << " stdDev=" << (cpuValid ? std::to_string(expectedStdDevs[i]) : "not-valid") << " CPU exact=" << expectedStdDevs[i] << ", GPU valid=" << gpuValid << " stdDev=" << stdDevs[i] << ", absError=" << error << ", CPU samples=" << expectedSampleCounts[i] << ", GPU samples=" << sampleCounts[i] << "\n"; UIUtility::Print(detail.str());
		if (i == 6u)
		{
			for (float threshold : {0.0f, 0.00005f, 0.0002f})
			{
				std::ostringstream classification; classification << std::setprecision(std::numeric_limits<float>::max_digits10) << "Six-sample threshold sensitivity threshold=" << threshold << ": CPU TextureRejected=" << (expectedStdDevs[i] < threshold) << ", GPU TextureRejected=" << (stdDevs[i] < threshold) << "\n"; UIUtility::Print(classification.str());
			}
			const size_t traceBase = static_cast<size_t>(i) * kTraceWordsPerCase;
			if (traceBase + kTraceWordsPerCase > traces.size()) throw std::out_of_range("Standalone texture diagnostic case exceeds its host trace buffer.");
			auto printBits = [](const char *label, uint32_t bits) { std::ostringstream line; line << label << "=0x" << std::hex << std::setw(8) << std::setfill('0') << bits << std::dec; return line.str(); };
			std::ostringstream trace; trace << std::setprecision(std::numeric_limits<float>::max_digits10) << "Standalone six-sample case bit trace: CPU stdDev=" << expectedStdDevs[i] << " " << printBits("bits", FloatBits(expectedStdDevs[i])) << ", GPU stdDev=" << stdDevs[i] << " " << printBits("bits", FloatBits(stdDevs[i])) << ", accepted=" << sampleCounts[i] << "\n";
			const uint32_t accepted = std::min(sampleCounts[i], 9u);
			for (uint32_t sample = 0; sample < accepted; ++sample)
			{
				const size_t base = traceBase + 10u + sample * 10u;
				trace << "GPU sample " << sample << ": coord=(" << FloatFromBits(traces[base]) << "," << FloatFromBits(traces[base + 1u]) << "), coordBits=(" << printBits("x", traces[base]) << "," << printBits("y", traces[base + 1u]) << "), rgba=(" << FloatFromBits(traces[base + 2u]) << "," << FloatFromBits(traces[base + 3u]) << "," << FloatFromBits(traces[base + 4u]) << "," << FloatFromBits(traces[base + 5u]) << "), rgbaBits=(" << printBits("r", traces[base + 2u]) << "," << printBits("g", traces[base + 3u]) << "," << printBits("b", traces[base + 4u]) << "," << printBits("a", traces[base + 5u]) << "), luminance=" << FloatFromBits(traces[base + 6u]) << " " << printBits("bits", traces[base + 6u]) << ", luminanceSquared=" << FloatFromBits(traces[base + 7u]) << " " << printBits("bits", traces[base + 7u]) << ", runningSum=" << FloatFromBits(traces[base + 8u]) << " " << printBits("bits", traces[base + 8u]) << ", runningSquaredSum=" << FloatFromBits(traces[base + 9u]) << " " << printBits("bits", traces[base + 9u]) << "\n";
				if (base + 9u >= traces.size()) throw std::out_of_range("Standalone texture diagnostic sample trace exceeds its host buffer.");
			}
			const char *names[] = {"sampleCount", "luminanceSum", "luminanceSquaredSum", "sampleCountFloat", "mean", "meanSquared", "averageSquared", "rawVariance", "clampedVariance", "standardDeviation"};
			for (uint32_t field = 0; field < 10; ++field)
			{
				const uint32_t bits = traces[traceBase + field]; const float value = field == 0 ? static_cast<float>(bits) : FloatFromBits(bits);
				trace << "GPU " << names[field] << "=" << (field == 0 ? static_cast<float>(bits) : value) << " " << printBits("bits", bits) << "\n";
			}
			UIUtility::Print(trace.str());
		}
	}
	if (maximumStdDevError) *maximumStdDevError = maxError;
	std::ostringstream summary; summary << std::setprecision(std::numeric_limits<float>::max_digits10) << "Direct texture standard-deviation regression: " << validityMatches << " / 6 existing validity results; " << validValueMatches << " / 5 existing valid values within 1e-6; max existing-case abs error=" << maxError << ".\n"; UIUtility::Print(summary.str()); cleanup(); return success;
	}
	catch (const std::exception &ex) { cleanup(); UIUtility::Print(std::string("PlaneSweep GPU direct texture standard-deviation test failed: ") + ex.what() + "\n"); return false; }
}

bool PlaneSweepGpu::RunDepthMapSmokeTest(
	const std::vector<uint8_t> &referencePixels,
	const std::vector<uint8_t> &sourcePixels,
	const std::vector<GpuCameraIntrinsics> &intrinsics,
	const std::vector<GpuCameraPose> &poses,
	const PlaneSweepGpuParams &params,
	const std::vector<uint32_t> &sourceMetadata,
	const std::vector<float> &expectedDepthMap,
	const std::vector<uint32_t> &expectedSampleMappings,
	const std::vector<uint32_t> &expectedSampleStatuses,
	const std::vector<float> &expectedTextureStdDevs,
	const std::vector<uint32_t> &expectedTextureSampleCounts,
	uint32_t expectedTextureRejectedCount,
	uint32_t expectedAcceptedCount,
	uint32_t expectedCandidateCount,
	float *maximumTextureStdDevError,
	float *maximumPositiveDepthError)
{
	if (!m_initialized) throw std::runtime_error("PlaneSweepGpu not initialized.");
	const uint32_t imageWidth = intrinsics.size() == 2 ? static_cast<uint32_t>(intrinsics[0].m_width) : 0u;
	const uint32_t imageHeight = intrinsics.size() == 2 ? static_cast<uint32_t>(intrinsics[0].m_height) : 0u;
	const uint32_t gridWidth = params.m_pointStep == 0 ? 0u : (params.m_scaledWidth + params.m_pointStep - 1u) / params.m_pointStep;
	const uint32_t gridHeight = params.m_pointStep == 0 ? 0u : (params.m_scaledHeight + params.m_pointStep - 1u) / params.m_pointStep;
	const uint32_t kSampleCount = gridWidth * gridHeight;
	constexpr uint32_t kImages = 3;
	constexpr VkFormat kFormat = VK_FORMAT_R8G8B8A8_UNORM;
	if ((imageWidth != 5 || imageHeight != 5) && (imageWidth != 8 || imageHeight != 6)) throw std::invalid_argument("Depth-map smoke test supports the 5x5 texture-rejection and 8x6 legacy images.");
	if (referencePixels.size() != static_cast<size_t>(imageWidth) * imageHeight * 4 || sourcePixels.size() != referencePixels.size() || poses.size() != 2 || sourceMetadata.size() != 18 ||
		params.m_scaledWidth == 0 || params.m_scaledHeight == 0 || params.m_pointStep == 0 || expectedDepthMap.size() != static_cast<size_t>(params.m_scaledWidth) * params.m_scaledHeight ||
		expectedSampleMappings.size() != kSampleCount * 7 || expectedSampleStatuses.size() != kSampleCount || (!expectedTextureStdDevs.empty() && expectedTextureStdDevs.size() != kSampleCount) || (!expectedTextureSampleCounts.empty() && expectedTextureSampleCounts.size() != kSampleCount))
		throw std::invalid_argument("Depth-map smoke test received invalid input dimensions.");
	if (params.m_sourceCount != 1 || (params.m_textureRejectionEnabled != 0u && (params.m_scaledWidth != 5 || params.m_scaledHeight != 5 || gridWidth != 3 || gridHeight != 3)))
		throw std::invalid_argument("Texture-rejection depth-map test expects a 5x5 map, 3x3 sample grid, and one source.");
	const VkDeviceSize imageBytes = referencePixels.size();
	const VkDeviceSize stagingBytes = imageBytes * 2 + 4;
	const VkDeviceSize depthBytes = expectedDepthMap.size() * sizeof(float);
	const VkDeviceSize mappingBytes = expectedSampleMappings.size() * sizeof(uint32_t);
	const VkDeviceSize statusBytes = expectedSampleStatuses.size() * sizeof(uint32_t);
	const std::array<VkDeviceSize, 4> inputSizes{{ intrinsics.size() * sizeof(GpuCameraIntrinsics), poses.size() * sizeof(GpuCameraPose), sizeof(PlaneSweepGpuParams), sourceMetadata.size() * sizeof(uint32_t) }};
	const std::array<const void *, 4> inputData{{ intrinsics.data(), poses.data(), &params, sourceMetadata.data() }};
	const VkDeviceSize textureStdDevBytes = static_cast<VkDeviceSize>(kSampleCount) * 100u * sizeof(uint32_t);
	const VkDeviceSize textureSampleCountBytes = static_cast<VkDeviceSize>(kSampleCount) * sizeof(uint32_t);
	const VkDeviceSize minimumOpaqueSamplesBytes = static_cast<VkDeviceSize>(kSampleCount) * sizeof(uint32_t);
	const VkDeviceSize planeCostSampleCountBytes = static_cast<VkDeviceSize>(kSampleCount) * sizeof(uint32_t);
	const std::array<VkDeviceSize, 7> outputSizes{{ depthBytes, mappingBytes, statusBytes, textureStdDevBytes, textureSampleCountBytes, minimumOpaqueSamplesBytes, planeCostSampleCountBytes }};
	VkBuffer imageStaging = VK_NULL_HANDLE; VkDeviceMemory imageStagingMemory = VK_NULL_HANDLE;
	std::array<VkImage, kImages> images{}; std::array<VkDeviceMemory, kImages> imageMemories{}; std::array<VkImageView, kImages> imageViews{};
	VkSampler sampler = VK_NULL_HANDLE;
	std::array<VkBuffer, 4> inputStaging{}, inputBuffers{}; std::array<VkDeviceMemory, 4> inputStagingMemories{}, inputMemories{};
	std::array<VkBuffer, 7> outputBuffers{}, readbackBuffers{}; std::array<VkDeviceMemory, 7> outputMemories{}, readbackMemories{};
	VkDescriptorSetLayout setLayout = VK_NULL_HANDLE; VkDescriptorPool pool = VK_NULL_HANDLE; VkDescriptorSet set = VK_NULL_HANDLE;
	VkPipelineLayout pipelineLayout = VK_NULL_HANDLE; VkShaderModule shader = VK_NULL_HANDLE; VkPipeline pipeline = VK_NULL_HANDLE;
	VkCommandBuffer commandBuffer = VK_NULL_HANDLE; VkFence fence = VK_NULL_HANDLE; bool submitted = false;
	auto cleanup = [&]()
	{
		if (submitted) { vkDeviceWaitIdle(m_device); submitted = false; }
		if (fence) vkDestroyFence(m_device, fence, nullptr); if (commandBuffer) vkFreeCommandBuffers(m_device, m_commandPool, 1, &commandBuffer);
		if (pipeline) vkDestroyPipeline(m_device, pipeline, nullptr); if (shader) vkDestroyShaderModule(m_device, shader, nullptr); if (pipelineLayout) vkDestroyPipelineLayout(m_device, pipelineLayout, nullptr);
		if (pool) vkDestroyDescriptorPool(m_device, pool, nullptr); if (setLayout) vkDestroyDescriptorSetLayout(m_device, setLayout, nullptr); if (sampler) vkDestroySampler(m_device, sampler, nullptr);
		for (uint32_t i = 0; i < kImages; ++i) { if (imageViews[i]) vkDestroyImageView(m_device, imageViews[i], nullptr); if (images[i]) vkDestroyImage(m_device, images[i], nullptr); if (imageMemories[i]) vkFreeMemory(m_device, imageMemories[i], nullptr); }
		auto destroyBuffer = [&](VkBuffer &buffer, VkDeviceMemory &memory) { if (buffer) vkDestroyBuffer(m_device, buffer, nullptr); if (memory) vkFreeMemory(m_device, memory, nullptr); buffer = VK_NULL_HANDLE; memory = VK_NULL_HANDLE; };
		destroyBuffer(imageStaging, imageStagingMemory);
		for (uint32_t i = 0; i < 4; ++i) { destroyBuffer(inputStaging[i], inputStagingMemories[i]); destroyBuffer(inputBuffers[i], inputMemories[i]); }
		for (uint32_t i = 0; i < outputBuffers.size(); ++i) { destroyBuffer(outputBuffers[i], outputMemories[i]); destroyBuffer(readbackBuffers[i], readbackMemories[i]); }
	};
	auto findMemory = [&](uint32_t bits, VkMemoryPropertyFlags flags) { VkPhysicalDeviceMemoryProperties p{}; vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &p); for (uint32_t i = 0; i < p.memoryTypeCount; ++i) if ((bits & (1u << i)) && (p.memoryTypes[i].propertyFlags & flags) == flags) return i; return UINT32_MAX; };
	auto makeBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags flags, VkBuffer &buffer, VkDeviceMemory &memory) { VkBufferCreateInfo ci{}; ci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO; ci.size = size; ci.usage = usage; ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE; if (vkCreateBuffer(m_device, &ci, nullptr, &buffer) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-map buffer."); VkMemoryRequirements req{}; vkGetBufferMemoryRequirements(m_device, buffer, &req); uint32_t type = findMemory(req.memoryTypeBits, flags); if (type == UINT32_MAX) throw std::runtime_error("No memory type for depth-map buffer."); VkMemoryAllocateInfo ai{}; ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; ai.allocationSize = req.size; ai.memoryTypeIndex = type; if (vkAllocateMemory(m_device, &ai, nullptr, &memory) != VK_SUCCESS || vkBindBufferMemory(m_device, buffer, memory, 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate depth-map buffer memory."); };
	try
	{
		VkPhysicalDeviceProperties properties{}; vkGetPhysicalDeviceProperties(m_physicalDevice, &properties);
		const auto &limits = properties.limits;
		if (limits.maxPerStageDescriptorSamplers < 18 || limits.maxPerStageDescriptorSampledImages < 18 || limits.maxPerStageDescriptorStorageBuffers < 11 || limits.maxDescriptorSetSamplers < 18 || limits.maxDescriptorSetSampledImages < 18 || limits.maxDescriptorSetStorageBuffers < 11 || limits.maxPerStageResources < 29)
			throw std::runtime_error("Device descriptor limits cannot support depth-map test resources.");
		const VkMemoryPropertyFlags host = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
		makeBuffer(stagingBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, host, imageStaging, imageStagingMemory);
		void *mapped = nullptr; if (vkMapMemory(m_device, imageStagingMemory, 0, stagingBytes, 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map depth-map image staging.");
		std::memcpy(mapped, referencePixels.data(), static_cast<size_t>(imageBytes)); std::memcpy(static_cast<uint8_t *>(mapped) + imageBytes, sourcePixels.data(), static_cast<size_t>(imageBytes)); const std::array<uint8_t, 4> dummy{{0, 0, 0, 255}}; std::memcpy(static_cast<uint8_t *>(mapped) + imageBytes * 2, dummy.data(), dummy.size()); vkUnmapMemory(m_device, imageStagingMemory);
		for (uint32_t i = 0; i < kImages; ++i)
		{
			VkImageCreateInfo ci{}; ci.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO; ci.imageType = VK_IMAGE_TYPE_2D; ci.format = kFormat; ci.extent = i == 2 ? VkExtent3D{1, 1, 1} : VkExtent3D{5, 5, 1}; ci.mipLevels = 1; ci.arrayLayers = 1; ci.samples = VK_SAMPLE_COUNT_1_BIT; ci.tiling = VK_IMAGE_TILING_OPTIMAL; ci.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT; ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE; ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			if (vkCreateImage(m_device, &ci, nullptr, &images[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-map image."); VkMemoryRequirements req{}; vkGetImageMemoryRequirements(m_device, images[i], &req); uint32_t type = findMemory(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT); if (type == UINT32_MAX) throw std::runtime_error("No device-local memory for depth-map image."); VkMemoryAllocateInfo ai{}; ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; ai.allocationSize = req.size; ai.memoryTypeIndex = type; if (vkAllocateMemory(m_device, &ai, nullptr, &imageMemories[i]) != VK_SUCCESS || vkBindImageMemory(m_device, images[i], imageMemories[i], 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate depth-map image.");
			VkImageViewCreateInfo vi{}; vi.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO; vi.image = images[i]; vi.viewType = VK_IMAGE_VIEW_TYPE_2D; vi.format = kFormat; vi.subresourceRange = VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}; if (vkCreateImageView(m_device, &vi, nullptr, &imageViews[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-map image view.");
		}
		VkSamplerCreateInfo si{}; si.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO; si.magFilter = VK_FILTER_NEAREST; si.minFilter = VK_FILTER_NEAREST; si.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST; si.addressModeU = si.addressModeV = si.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE; si.anisotropyEnable = VK_FALSE; si.compareEnable = VK_FALSE; si.minLod = 0.0f; si.maxLod = 0.0f; if (vkCreateSampler(m_device, &si, nullptr, &sampler) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-map sampler.");
		for (uint32_t i = 0; i < 4; ++i) { makeBuffer(inputSizes[i], VK_BUFFER_USAGE_TRANSFER_SRC_BIT, host, inputStaging[i], inputStagingMemories[i]); if (vkMapMemory(m_device, inputStagingMemories[i], 0, inputSizes[i], 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map depth-map input staging."); std::memcpy(mapped, inputData[i], static_cast<size_t>(inputSizes[i])); vkUnmapMemory(m_device, inputStagingMemories[i]); makeBuffer(inputSizes[i], VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, inputBuffers[i], inputMemories[i]); }
		for (uint32_t i = 0; i < outputBuffers.size(); ++i) { const VkBufferUsageFlags usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | (i == 0 ? VK_BUFFER_USAGE_TRANSFER_DST_BIT : 0); makeBuffer(outputSizes[i], usage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, outputBuffers[i], outputMemories[i]); makeBuffer(outputSizes[i], VK_BUFFER_USAGE_TRANSFER_DST_BIT, host, readbackBuffers[i], readbackMemories[i]); }
		std::array<VkDescriptorSetLayoutBinding, 15> bindings{}; for (uint32_t i = 0; i < bindings.size(); ++i) { bindings[i].binding = i; bindings[i].descriptorType = i < 4 ? VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; bindings[i].descriptorCount = i == 2 || i == 3 ? 8 : 1; bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT; }
		VkDescriptorSetLayoutCreateInfo lci{}; lci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO; lci.bindingCount = static_cast<uint32_t>(bindings.size()); lci.pBindings = bindings.data(); if (vkCreateDescriptorSetLayout(m_device, &lci, nullptr, &setLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-map descriptor layout.");
		std::array<VkDescriptorPoolSize, 2> poolSizes{{{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 18}, {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 11}}}; VkDescriptorPoolCreateInfo pci{}; pci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO; pci.maxSets = 1; pci.poolSizeCount = static_cast<uint32_t>(poolSizes.size()); pci.pPoolSizes = poolSizes.data(); if (vkCreateDescriptorPool(m_device, &pci, nullptr, &pool) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-map descriptor pool.");
		VkDescriptorSetAllocateInfo sai{}; sai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO; sai.descriptorPool = pool; sai.descriptorSetCount = 1; sai.pSetLayouts = &setLayout; if (vkAllocateDescriptorSets(m_device, &sai, &set) != VK_SUCCESS) throw std::runtime_error("Failed to allocate depth-map descriptor set.");
		std::array<VkDescriptorImageInfo, 1> referenceInfo{{{sampler, imageViews[0], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}}}, dummyInfo{{{sampler, imageViews[2], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}}};
		std::array<VkDescriptorImageInfo, 8> sourceInfos{}, maskInfos{}; for (uint32_t i = 0; i < 8; ++i) { sourceInfos[i] = {sampler, i == 0 ? imageViews[1] : imageViews[2], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}; maskInfos[i] = {sampler, imageViews[2], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}; }
		std::array<VkDescriptorBufferInfo, 11> bufferInfos{}; for (uint32_t i = 0; i < 4; ++i) bufferInfos[i] = {inputBuffers[i], 0, inputSizes[i]}; for (uint32_t i = 0; i < outputBuffers.size(); ++i) bufferInfos[4 + i] = {outputBuffers[i], 0, outputSizes[i]};
		std::array<VkWriteDescriptorSet, 15> writes{}; for (uint32_t i = 0; i < writes.size(); ++i) { writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; writes[i].dstSet = set; writes[i].dstBinding = i; writes[i].descriptorType = i < 4 ? VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; }
		writes[0].descriptorCount = 1; writes[0].pImageInfo = referenceInfo.data(); writes[1].descriptorCount = 1; writes[1].pImageInfo = dummyInfo.data(); writes[2].descriptorCount = 8; writes[2].pImageInfo = sourceInfos.data(); writes[3].descriptorCount = 8; writes[3].pImageInfo = maskInfos.data(); for (uint32_t i = 0; i < 11; ++i) { writes[4 + i].descriptorCount = 1; writes[4 + i].pBufferInfo = &bufferInfos[i]; } vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
		const std::vector<char> code = ReadShaderFile("shaders/planeSweepDepthMapSmokeTest.comp.spv"); if (code.empty()) throw std::runtime_error("Failed to load depth-map smoke-test SPIR-V."); VkShaderModuleCreateInfo sci{}; sci.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO; sci.codeSize = code.size(); sci.pCode = reinterpret_cast<const uint32_t *>(code.data()); if (vkCreateShaderModule(m_device, &sci, nullptr, &shader) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-map shader.");
		VkPipelineLayoutCreateInfo pli{}; pli.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO; pli.setLayoutCount = 1; pli.pSetLayouts = &setLayout; if (vkCreatePipelineLayout(m_device, &pli, nullptr, &pipelineLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-map pipeline layout."); VkPipelineShaderStageCreateInfo stage{}; stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = shader; stage.pName = "main"; VkComputePipelineCreateInfo cpci{}; cpci.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO; cpci.stage = stage; cpci.layout = pipelineLayout; if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &cpci, nullptr, &pipeline) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-map pipeline.");
		VkCommandBufferAllocateInfo cai{}; cai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO; cai.commandPool = m_commandPool; cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; cai.commandBufferCount = 1; if (vkAllocateCommandBuffers(m_device, &cai, &commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to allocate depth-map command buffer."); VkCommandBufferBeginInfo cbi{}; cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO; cbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT; if (vkBeginCommandBuffer(commandBuffer, &cbi) != VK_SUCCESS) throw std::runtime_error("Failed to begin depth-map command buffer.");
		std::array<VkImageMemoryBarrier, kImages> toTransfer{}; for (uint32_t i = 0; i < kImages; ++i) { auto &b = toTransfer[i]; b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; b.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED; b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; b.image = images[i]; b.subresourceRange = VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, kImages, toTransfer.data());
		for (uint32_t i = 0; i < kImages; ++i) { VkBufferImageCopy copy{}; copy.bufferOffset = i == 0 ? 0 : i == 1 ? imageBytes : imageBytes * 2; copy.imageSubresource = VkImageSubresourceLayers{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1}; copy.imageExtent = i == 2 ? VkExtent3D{1, 1, 1} : VkExtent3D{5, 5, 1}; vkCmdCopyBufferToImage(commandBuffer, imageStaging, images[i], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy); }
		std::array<VkImageMemoryBarrier, kImages> toRead{}; for (uint32_t i = 0; i < kImages; ++i) { auto &b = toRead[i]; b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT; b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; b.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; b.image = images[i]; b.subresourceRange = VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 0, nullptr, kImages, toRead.data());
		for (uint32_t i = 0; i < 4; ++i) { VkBufferCopy copy{}; copy.size = inputSizes[i]; vkCmdCopyBuffer(commandBuffer, inputStaging[i], inputBuffers[i], 1, &copy); } std::array<VkBufferMemoryBarrier, 4> inputBarriers{}; for (uint32_t i = 0; i < 4; ++i) { auto &b = inputBarriers[i]; b.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT; b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; b.buffer = inputBuffers[i]; b.size = inputSizes[i]; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 4, inputBarriers.data(), 0, nullptr);
		vkCmdFillBuffer(commandBuffer, outputBuffers[0], 0, VK_WHOLE_SIZE, 0u); VkBufferMemoryBarrier zeroBarrier{}; zeroBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; zeroBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; zeroBarrier.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT; zeroBarrier.srcQueueFamilyIndex = zeroBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; zeroBarrier.buffer = outputBuffers[0]; zeroBarrier.size = depthBytes; vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 1, &zeroBarrier, 0, nullptr);
		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline); vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &set, 0, nullptr); vkCmdDispatch(commandBuffer, (gridWidth + 7u) / 8u, (gridHeight + 7u) / 8u, 1);
		std::array<VkBufferMemoryBarrier, 7> outputBarriers{}; for (uint32_t i = 0; i < outputBarriers.size(); ++i) { auto &b = outputBarriers[i]; b.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; b.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT; b.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT; b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; b.buffer = outputBuffers[i]; b.size = outputSizes[i]; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, static_cast<uint32_t>(outputBarriers.size()), outputBarriers.data(), 0, nullptr);
		for (uint32_t i = 0; i < outputBuffers.size(); ++i) { VkBufferCopy copy{}; copy.size = outputSizes[i]; vkCmdCopyBuffer(commandBuffer, outputBuffers[i], readbackBuffers[i], 1, &copy); } std::array<VkBufferMemoryBarrier, 7> hostBarriers{}; for (uint32_t i = 0; i < hostBarriers.size(); ++i) { auto &b = hostBarriers[i]; b.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; b.dstAccessMask = VK_ACCESS_HOST_READ_BIT; b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; b.buffer = readbackBuffers[i]; b.size = outputSizes[i]; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 0, nullptr, static_cast<uint32_t>(hostBarriers.size()), hostBarriers.data(), 0, nullptr);
		if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to end depth-map command buffer."); VkFenceCreateInfo fi{}; fi.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO; if (vkCreateFence(m_device, &fi, nullptr, &fence) != VK_SUCCESS) throw std::runtime_error("Failed to create depth-map fence."); VkSubmitInfo submit{}; submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO; submit.commandBufferCount = 1; submit.pCommandBuffers = &commandBuffer; if (vkQueueSubmit(m_queue, 1, &submit, fence) != VK_SUCCESS) throw std::runtime_error("Failed to submit depth-map dispatch."); submitted = true; if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS) throw std::runtime_error("Failed waiting for depth-map fence."); submitted = false;
		std::vector<float> gpuDepthMap(expectedDepthMap.size()); std::vector<uint32_t> gpuTextureStdDevs(static_cast<size_t>(kSampleCount) * 100u), gpuMappings(expectedSampleMappings.size()), gpuStatuses(kSampleCount), gpuTextureSampleCounts(kSampleCount), gpuMinimumOpaqueSamples(kSampleCount), gpuPlaneCostSampleCounts(kSampleCount); const std::array<void *, 7> targets{{gpuDepthMap.data(), gpuMappings.data(), gpuStatuses.data(), gpuTextureStdDevs.data(), gpuTextureSampleCounts.data(), gpuMinimumOpaqueSamples.data(), gpuPlaneCostSampleCounts.data()}}; const std::array<size_t, 7> targetBytes{{gpuDepthMap.size() * sizeof(float), gpuMappings.size() * sizeof(uint32_t), gpuStatuses.size() * sizeof(uint32_t), gpuTextureStdDevs.size() * sizeof(uint32_t), gpuTextureSampleCounts.size() * sizeof(uint32_t), gpuMinimumOpaqueSamples.size() * sizeof(uint32_t), gpuPlaneCostSampleCounts.size() * sizeof(uint32_t)}};
		for (uint32_t i = 0; i < targets.size(); ++i) { if (outputSizes[i] > static_cast<VkDeviceSize>(std::numeric_limits<size_t>::max()) || static_cast<size_t>(outputSizes[i]) != targetBytes[i]) throw std::runtime_error("Depth-map readback size does not match its host destination allocation."); if (vkMapMemory(m_device, readbackMemories[i], 0, outputSizes[i], 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map depth-map readback."); std::memcpy(targets[i], mapped, targetBytes[i]); vkUnmapMemory(m_device, readbackMemories[i]); }
		constexpr uint32_t statusReferenceRejected = 4u, statusTextureRejected = 5u;
		float maximumStdDevError = 0.0f;
	uint32_t gpuTextureRejected = 0;
	for (uint32_t i = 0; i < kSampleCount; ++i)
	{
		const auto gpuMappingBegin = gpuMappings.begin() + i * 7;
		const auto expectedMappingBegin = expectedSampleMappings.begin() + i * 7;
		if (!std::equal(gpuMappingBegin, gpuMappingBegin + 7, expectedMappingBegin) || gpuStatuses[i] != expectedSampleStatuses[i])
		{
			std::string mappingDetails;
			for (uint32_t field = 0; field < 7; ++field) mappingDetails += (field == 0 ? "" : ",") + std::string(std::to_string(expectedMappingBegin[field]) + "/" + std::to_string(gpuMappingBegin[field]));
			UIUtility::Print("Depth-map mapping/status mismatch sample=" + std::to_string(i) + " (CPU/GPU mapping fields=" + mappingDetails + ", CPU/GPU status=" + std::to_string(expectedSampleStatuses[i]) + "/" + std::to_string(gpuStatuses[i]) + ").\n"); cleanup(); return false;
		}
		if (gpuStatuses[i] == statusTextureRejected) ++gpuTextureRejected;
		const bool textureRan = gpuStatuses[i] != statusReferenceRejected;
		const float gpuStdDev = textureRan ? FloatFromBits(gpuTextureStdDevs[i * 100u + 9u]) : 0.0f;
		const float stdDevError = textureRan && !expectedTextureStdDevs.empty() ? std::fabs(gpuStdDev - expectedTextureStdDevs[i]) : 0.0f;
		const bool sixSampleCancellation = textureRan && !expectedTextureSampleCounts.empty() && expectedTextureSampleCounts[i] == 6u && expectedTextureStdDevs[i] > 0.0f && gpuStdDev == 0.0f;
		if (textureRan && !expectedTextureStdDevs.empty() && !sixSampleCancellation) maximumStdDevError = std::max(maximumStdDevError, stdDevError);
		if (sixSampleCancellation)
		{
			std::ostringstream diagnostic; diagnostic.precision(9); diagnostic << "Known six-sample cancellation stdDev difference sample=" << i << ": CPU=" << expectedTextureStdDevs[i] << ", GPU=" << gpuStdDev << ", absError=" << stdDevError << "; decision status remains independently checked.\n"; UIUtility::Print(diagnostic.str());
		}
		else if (textureRan && !expectedTextureStdDevs.empty() && stdDevError != 0.0f)
		{
			std::ostringstream diagnostic; diagnostic.precision(9); diagnostic << "Texture stdDev numeric difference sample=" << i << ": CPU=" << expectedTextureStdDevs[i] << ", GPU=" << gpuStdDev << ", absError=" << stdDevError << "; decision status remains independently checked.\n"; UIUtility::Print(diagnostic.str());
		}
		if (textureRan && !expectedTextureSampleCounts.empty() && gpuTextureSampleCounts[i] != expectedTextureSampleCounts[i])
		{
			if (i == 1u)
			{
				const size_t db = static_cast<size_t>(i) * 100u;
				if (gpuTextureSampleCounts[i] > 9u || db + 100u > gpuTextureStdDevs.size()) throw std::out_of_range("Depth-map texture diagnostic record exceeds its host trace buffer.");
				std::ostringstream diagnostic; diagnostic.precision(9);
				auto printFloat = [&](const char *name, size_t offset) { diagnostic << name << "=" << FloatFromBits(gpuTextureStdDevs[db + offset]) << " (0x" << std::hex << std::setw(8) << std::setfill('0') << gpuTextureStdDevs[db + offset] << std::dec << ") "; };
				printFloat("luminanceSum", 1u); printFloat("luminanceSquaredSum", 2u); printFloat("sampleCountFloat", 3u); printFloat("mean", 4u); printFloat("meanSquared", 5u); printFloat("averageSquared", 6u); printFloat("rawVariance", 7u); printFloat("clampedVariance", 8u); printFloat("standardDeviation", 9u); diagnostic << "\n";
				for (uint32_t s = 0; s < gpuTextureSampleCounts[i]; ++s)
				{
					const size_t sb = db + 10u + s * 10u;
					if (sb + 10u > db + 100u || sb + 10u > gpuTextureStdDevs.size()) throw std::out_of_range("Depth-map texture diagnostic sample exceeds its host trace record.");
					diagnostic << "GPU sample " << s << ": coord=(" << FloatFromBits(gpuTextureStdDevs[sb]) << "," << FloatFromBits(gpuTextureStdDevs[sb + 1u]) << "), rgba=(" << FloatFromBits(gpuTextureStdDevs[sb + 2u]) << "," << FloatFromBits(gpuTextureStdDevs[sb + 3u]) << "," << FloatFromBits(gpuTextureStdDevs[sb + 4u]) << "," << FloatFromBits(gpuTextureStdDevs[sb + 5u]) << "), "; printFloat("luminance", sb - db + 6u); printFloat("luminanceSquared", sb - db + 7u); printFloat("runningSum", sb - db + 8u); printFloat("runningSquaredSum", sb - db + 9u); diagnostic << "\n";
				}
				UIUtility::Print(diagnostic.str());
			}
			std::ostringstream mismatch; mismatch.precision(9); mismatch << "Depth-map texture sample-count mismatch sample=" << i << ", CPU samples=" << expectedTextureSampleCounts[i] << ", GPU samples=" << gpuTextureSampleCounts[i] << ".\n"; UIUtility::Print(mismatch.str()); cleanup(); return false;
		}
	}
		if (gpuTextureRejected != expectedTextureRejectedCount) { UIUtility::Print("TextureRejected status count mismatch CPU=" + std::to_string(expectedTextureRejectedCount) + ", GPU=" + std::to_string(gpuTextureRejected) + ".\n"); cleanup(); return false; }
		uint32_t cpuAccepted = 0, gpuAccepted = 0; float maximumError = 0.0f;
		for (size_t i = 0; i < gpuDepthMap.size(); ++i) { if (std::isfinite(expectedDepthMap[i]) && expectedDepthMap[i] > 0.0f) ++cpuAccepted; if (std::isfinite(gpuDepthMap[i]) && gpuDepthMap[i] > 0.0f) ++gpuAccepted; if (expectedDepthMap[i] == 0.0f) { if (gpuDepthMap[i] != 0.0f) { UIUtility::Print("Depth-map zero mismatch index=" + std::to_string(i) + ", GPU=" + std::to_string(gpuDepthMap[i]) + ".\n"); cleanup(); return false; } } else { const float error = std::fabs(gpuDepthMap[i] - expectedDepthMap[i]); maximumError = std::max(maximumError, error); if (error > 1e-6f) { UIUtility::Print("Depth-map positive-depth mismatch index=" + std::to_string(i) + ", CPU=" + std::to_string(expectedDepthMap[i]) + ", GPU=" + std::to_string(gpuDepthMap[i]) + ", error=" + std::to_string(error) + ".\n"); cleanup(); return false; } } }
		if (maximumTextureStdDevError) *maximumTextureStdDevError = maximumStdDevError;
		if (maximumPositiveDepthError) *maximumPositiveDepthError = maximumError;
		if (cpuAccepted != gpuAccepted || (expectedAcceptedCount != UINT32_MAX && cpuAccepted != expectedAcceptedCount) || (expectedCandidateCount != UINT32_MAX && cpuAccepted != expectedCandidateCount)) { UIUtility::Print("Depth-map accepted/candidate count mismatch CPU=" + std::to_string(cpuAccepted) + ", GPU=" + std::to_string(gpuAccepted) + ", candidates=" + std::to_string(expectedCandidateCount) + ".\n"); cleanup(); return false; }
		UIUtility::Print("PlaneSweep GPU full depth-map dispatch smoke test succeeded:\nGPU scaled dimensions: " + std::to_string(params.m_scaledWidth) + " x " + std::to_string(params.m_scaledHeight) + ".\nSample grid dimensions: " + std::to_string(gridWidth) + " x " + std::to_string(gridHeight) + ".\n9 / 9 sampled reference-coordinate mappings matched CPU semantics.\n" + std::to_string(gpuDepthMap.size()) + " / " + std::to_string(gpuDepthMap.size()) + " depth-buffer entries matched CPU BuildReferencePoints.\n" + std::to_string(cpuAccepted) + " / " + std::to_string(cpuAccepted) + " accepted depths matched.\nTextureRejected statuses: " + std::to_string(gpuTextureRejected) + " CPU count: " + std::to_string(expectedTextureRejectedCount) + ".\nMaximum positive-depth absolute error: " + std::to_string(maximumError) + "\n");
		auto printMap = [&](const char *label, const std::vector<float> &values) { UIUtility::Print(std::string(label) + " depth map:\n"); for (uint32_t y = 0; y < params.m_scaledHeight; ++y) { std::string row = "row " + std::to_string(y) + ": "; for (uint32_t x = 0; x < params.m_scaledWidth; ++x) row += (x ? "," : "") + std::to_string(values[static_cast<size_t>(y) * params.m_scaledWidth + x]); UIUtility::Print(row + "\n"); } };
		printMap("CPU", expectedDepthMap); printMap("GPU", gpuDepthMap);
		for (uint32_t i = 0; i < kSampleCount; ++i) { const size_t base = i * 7; const bool textureRan = gpuStatuses[i] != statusReferenceRejected; UIUtility::Print("sample " + std::to_string(i) + ": scaled=(" + std::to_string(gpuMappings[base + 2]) + "," + std::to_string(gpuMappings[base + 3]) + "), reference=(" + std::to_string(gpuMappings[base + 4]) + "," + std::to_string(gpuMappings[base + 5]) + "), stdDev=" + (textureRan ? std::to_string(FloatFromBits(gpuTextureStdDevs[i * 100u + 9u])) : "not-run") + ", textureSamples=" + (textureRan ? std::to_string(gpuTextureSampleCounts[i]) : "not-run") + ", minimumOpaqueSamples=" + std::to_string(gpuMinimumOpaqueSamples[i]) + ", planeCostSamples=" + std::to_string(gpuPlaneCostSampleCounts[i]) + ", CPU depth=" + std::to_string(expectedDepthMap[gpuMappings[base + 6]]) + ", GPU depth=" + std::to_string(gpuDepthMap[gpuMappings[base + 6]]) + ", status=" + std::to_string(gpuStatuses[i]) + "\n"); }
		if (params.m_patchRadius == 1 && params.m_scaledWidth == 5 && params.m_scaledHeight == 5 && gpuStatuses[0] != statusReferenceRejected && gpuStatuses[0] != statusTextureRejected && gpuStatuses[4] != statusReferenceRejected && gpuStatuses[4] != statusTextureRejected)
		{
			if (gpuMinimumOpaqueSamples[0] != 4u || gpuPlaneCostSampleCounts[0] != 4u || gpuMinimumOpaqueSamples[4] != 4u || gpuPlaneCostSampleCounts[4] != 9u)
			{
				UIUtility::Print("Full-patch cost diagnostic failed: boundary sample=(0,0) minimum=" + std::to_string(gpuMinimumOpaqueSamples[0]) + " samples=" + std::to_string(gpuPlaneCostSampleCounts[0]) + "; interior sample=(2,2) minimum=" + std::to_string(gpuMinimumOpaqueSamples[4]) + " samples=" + std::to_string(gpuPlaneCostSampleCounts[4]) + ". Expected minima=4 and sample counts=4/9.\n"); cleanup(); return false;
			}
			UIUtility::Print("Full-patch cost diagnostics passed: boundary (0,0) minimumOpaqueSamples=4 sampleCount=4; interior (2,2) minimumOpaqueSamples=4 sampleCount=9.\n");
		}
		cleanup(); return true;
	}
	catch (const std::exception &ex) { cleanup(); UIUtility::Print(std::string("PlaneSweep GPU full depth-map dispatch failed: ") + ex.what() + "\n"); return false; }
}

bool PlaneSweepGpu::CompareRealDepthMap(
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
	double cpuBuildMilliseconds)
{
	if (!m_initialized) throw std::runtime_error("PlaneSweepGpu not initialized.");
	if (sourceImages.empty() || sourceImages.size() > MAX_PLANE_SWEEP_SOURCE_IMAGES || params.m_sourceCount != sourceImages.size()) throw std::invalid_argument("Real depth-map comparison requires 1..8 sources in exact CPU order.");
	const uint32_t scaledWidth = params.m_scaledWidth, scaledHeight = params.m_scaledHeight;
	const uint32_t pointStep = std::max(1u, params.m_pointStep);
	const uint32_t gridWidth = (scaledWidth + pointStep - 1u) / pointStep, gridHeight = (scaledHeight + pointStep - 1u) / pointStep;
	const size_t entryCount = static_cast<size_t>(scaledWidth) * scaledHeight;
	const size_t sampleCount = static_cast<size_t>(gridWidth) * gridHeight;
	if (scaledWidth == 0 || scaledHeight == 0 || cpuDepthMap.m_width != scaledWidth || cpuDepthMap.m_height != scaledHeight || cpuDepthMap.m_depthValues.size() != entryCount ||
		params.m_planeCount < 2 || params.m_patchRadius < 0 || params.m_scaledWidth == 0 || params.m_scaledHeight == 0 ||
		referenceImage.m_texture.m_width == 0 || referenceImage.m_texture.m_height == 0 || referenceImage.m_texture.m_channelCount != 4 ||
		referenceImage.m_texture.m_pixels.size() != static_cast<size_t>(referenceImage.m_texture.m_width) * referenceImage.m_texture.m_height * 4)
		throw std::invalid_argument("Real depth-map comparison received invalid reference/map dimensions or RGBA8 bytes.");
	for (const auto &image : sourceImages)
		if (image.m_texture.m_width == 0 || image.m_texture.m_height == 0 || image.m_texture.m_channelCount != 4 || image.m_texture.m_pixels.size() != static_cast<size_t>(image.m_texture.m_width) * image.m_texture.m_height * 4)
			throw std::invalid_argument("Real depth-map source is not a valid RGBA8 texture.");
	if (referenceImage.m_hasMask && (referenceImage.m_maskTexture.m_width == 0 || referenceImage.m_maskTexture.m_height == 0 || referenceImage.m_maskTexture.m_channelCount != 4 || referenceImage.m_maskTexture.m_pixels.size() != static_cast<size_t>(referenceImage.m_maskTexture.m_width) * referenceImage.m_maskTexture.m_height * 4)) throw std::invalid_argument("Real reference mask is not a valid RGBA8 texture.");
	for (const auto &image : sourceImages) if (image.m_hasMask && (image.m_maskTexture.m_width == 0 || image.m_maskTexture.m_height == 0 || image.m_maskTexture.m_channelCount != 4 || image.m_maskTexture.m_pixels.size() != static_cast<size_t>(image.m_maskTexture.m_width) * image.m_maskTexture.m_height * 4)) throw std::invalid_argument("Real source mask is not a valid RGBA8 texture.");
	const uint32_t refWidth = referenceImage.m_texture.m_width, refHeight = referenceImage.m_texture.m_height;
	const uint32_t expectedWidth = std::max(1u, static_cast<uint32_t>(std::lround(static_cast<float>(refWidth) * setup.m_planeSweepImageScale)));
	const uint32_t expectedHeight = std::max(1u, static_cast<uint32_t>(std::lround(static_cast<float>(refHeight) * setup.m_planeSweepImageScale)));
	if (expectedWidth != cpuDepthMap.m_width || expectedHeight != cpuDepthMap.m_height) throw std::runtime_error("Production scaled dimensions differ from the raw CPU depth map; Phase 2P comparison stopped.");
	const uint64_t patchWidth = static_cast<uint64_t>(params.m_patchRadius) * 2u + 1u, patchArea = patchWidth * patchWidth;
	const uint64_t workload = static_cast<uint64_t>(sampleCount) * static_cast<uint64_t>(params.m_planeCount) * sourceImages.size() * patchArea;
	const uint32_t referenceId = referenceImage.m_camera.m_imageId;
	const std::string referenceName = referenceImage.m_camera.m_name;
	std::ostringstream summary; summary << "Phase 2P real input summary:\nReference reconstruction index: " << referenceReconstructionIndex << "\nReference ID: " << referenceId << "\nReference filename: " << referenceName << "\nReference texture: " << refWidth << " x " << refHeight << "\nReference hasMask: " << referenceImage.m_hasMask << ", maskUsesAlpha: " << referenceImage.m_maskUsesAlpha;
	if (referenceImage.m_hasMask) summary << ", mask: " << referenceImage.m_maskTexture.m_width << " x " << referenceImage.m_maskTexture.m_height;
	summary << "\nSource count: " << sourceImages.size() << "\n";
	for (size_t i = 0; i < sourceImages.size(); ++i) { const auto &src = sourceImages[i]; summary << "Source slot " << i << ": ID=" << src.m_camera.m_imageId << ", filename=" << src.m_camera.m_name << ", texture=" << src.m_texture.m_width << " x " << src.m_texture.m_height << ", hasMask=" << src.m_hasMask << ", maskUsesAlpha=" << src.m_maskUsesAlpha; if (src.m_hasMask) summary << ", mask=" << src.m_maskTexture.m_width << " x " << src.m_maskTexture.m_height; summary << "\n"; }
	summary << "Setup: imageScale=" << setup.m_planeSweepImageScale << ", pointStep=" << setup.m_planeSweepPointStep << ", planeCount=" << setup.m_planeSweepPlaneCount << ", patchRadius=" << setup.m_planeSweepPatchRadius << ", minValidSourceViews=" << setup.m_planeSweepMinValidSourceViews << ", alphaThreshold=" << setup.m_planeSweepAlphaThreshold << ", minimumOpaquePatchRatio=" << setup.m_planeSweepMinimumOpaquePatchRatio << ", costThreshold=" << setup.m_planeSweepCostThreshold << ", confidenceEnabled=" << setup.m_planeSweepConfidenceEnabled << ", confidenceMinimumDifference=" << setup.m_planeSweepConfidenceMinimumDifference << ", textureRejectionEnabled=" << setup.m_planeSweepTextureRejectionEnabled << ", textureMinimumStdDev=" << setup.m_planeSweepTextureMinimumStdDev << "\nDepth range: near=" << depthRange.m_nearDepth << ", far=" << depthRange.m_farDepth << ", fromSparsePoints=" << depthRange.m_fromSparsePoints << "\nScaled map: " << scaledWidth << " x " << scaledHeight << ", sample grid: " << gridWidth << " x " << gridHeight << "\nMeaningful invocations=" << sampleCount << ", planeCount=" << params.m_planeCount << ", sourceCount=" << sourceImages.size() << ", patchArea=" << patchArea << ", estimated workload=" << workload << "\n";
	UIUtility::Print(summary.str());
	constexpr uint32_t kImageCount = 18;
	constexpr VkFormat kFormat = VK_FORMAT_R8G8B8A8_UNORM;
	struct UploadImage { const std::vector<uint8_t> *pixels; uint32_t width, height; };
	std::array<UploadImage, kImageCount> upload{};
	std::array<uint32_t, 18> metadata{};
	upload[0] = { &referenceImage.m_texture.m_pixels, refWidth, refHeight };
	const std::vector<uint8_t> dummyPixels{0, 0, 0, 255};
	upload[1] = { referenceImage.m_hasMask ? &referenceImage.m_maskTexture.m_pixels : &dummyPixels, referenceImage.m_hasMask ? referenceImage.m_maskTexture.m_width : 1u, referenceImage.m_hasMask ? referenceImage.m_maskTexture.m_height : 1u };
	metadata[16] = referenceImage.m_hasMask ? 1u : 0u; metadata[17] = referenceImage.m_maskUsesAlpha ? 1u : 0u;
	for (uint32_t i = 0; i < 8; ++i)
	{
		if (i < sourceImages.size())
		{
			const auto &src = sourceImages[i]; upload[2 + i] = { &src.m_texture.m_pixels, src.m_texture.m_width, src.m_texture.m_height };
			upload[10 + i] = { src.m_hasMask ? &src.m_maskTexture.m_pixels : &dummyPixels, src.m_hasMask ? src.m_maskTexture.m_width : 1u, src.m_hasMask ? src.m_maskTexture.m_height : 1u };
			metadata[i * 2] = src.m_hasMask ? 1u : 0u; metadata[i * 2 + 1] = src.m_maskUsesAlpha ? 1u : 0u;
		}
		else { upload[2 + i] = { &dummyPixels, 1u, 1u }; upload[10 + i] = { &dummyPixels, 1u, 1u }; }
	}
	std::vector<GpuCameraIntrinsics> intrinsics; std::vector<GpuCameraPose> poses;
	intrinsics.reserve(sourceImages.size() + 1); poses.reserve(sourceImages.size() + 1);
	intrinsics.push_back(MakeGpuCameraIntrinsics(referenceImage.m_intrinsic)); poses.push_back(MakeGpuCameraPose(referenceImage.m_camera));
	for (const auto &src : sourceImages) { intrinsics.push_back(MakeGpuCameraIntrinsics(src.m_intrinsic)); poses.push_back(MakeGpuCameraPose(src.m_camera)); }
	const float inverseNear = 1.0f / std::max(depthRange.m_nearDepth, 0.0001f);
	const float inverseFar = 1.0f / std::max(depthRange.m_farDepth, depthRange.m_nearDepth + 0.0001f);
	if (std::fabs(params.m_inverseNear - inverseNear) > 1e-6f || std::fabs(params.m_inverseFar - inverseFar) > 1e-6f) throw std::invalid_argument("GPU inverse depth range does not match the production depth range.");
	std::vector<uint32_t> mappings(sampleCount * 7u, 0u), statuses(sampleCount, 0u), textureSamples(sampleCount, 0u), opaqueSamples(sampleCount, 0u), planeSamples(sampleCount, 0u);
	const uint32_t textureTraceStride = std::max(100u, 10u + static_cast<uint32_t>(patchArea) * 10u);
	const std::vector<uint32_t> textureTrace(sampleCount * textureTraceStride, 0u);
	const std::array<const void *, 4> inputData{{intrinsics.data(), poses.data(), &params, metadata.data()}};
	const std::array<VkDeviceSize, 4> inputSizes{{intrinsics.size() * sizeof(GpuCameraIntrinsics), poses.size() * sizeof(GpuCameraPose), sizeof(params), metadata.size() * sizeof(uint32_t)}};
	const VkDeviceSize depthBytes = entryCount * sizeof(float);
	const std::array<VkDeviceSize, 7> outputSizes{{depthBytes, mappings.size() * sizeof(uint32_t), statuses.size() * sizeof(uint32_t), textureTrace.size() * sizeof(uint32_t), textureSamples.size() * sizeof(uint32_t), opaqueSamples.size() * sizeof(uint32_t), planeSamples.size() * sizeof(uint32_t)}};
	std::array<VkBuffer, 4> inputStaging{}, inputBuffers{}; std::array<VkDeviceMemory, 4> inputStagingMemory{}, inputMemory{};
	std::array<VkBuffer, 7> outputBuffers{}, readbackBuffers{}; std::array<VkDeviceMemory, 7> outputMemory{}, readbackMemory{};
	std::array<VkImage, kImageCount> images{}; std::array<VkDeviceMemory, kImageCount> imageMemory{}; std::array<VkImageView, kImageCount> imageViews{};
	VkBuffer imageStaging = VK_NULL_HANDLE; VkDeviceMemory imageStagingMemory = VK_NULL_HANDLE; VkSampler sampler = VK_NULL_HANDLE;
	VkDescriptorSetLayout setLayout = VK_NULL_HANDLE; VkDescriptorPool pool = VK_NULL_HANDLE; VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
	VkPipelineLayout pipelineLayout = VK_NULL_HANDLE; VkShaderModule shader = VK_NULL_HANDLE; VkPipeline pipeline = VK_NULL_HANDLE;
	VkCommandBuffer commandBuffer = VK_NULL_HANDLE; VkFence fence = VK_NULL_HANDLE; bool submitted = false;
	auto cleanup = [&]()
	{
		if (submitted) { vkDeviceWaitIdle(m_device); submitted = false; }
		if (fence) vkDestroyFence(m_device, fence, nullptr); if (commandBuffer) vkFreeCommandBuffers(m_device, m_commandPool, 1, &commandBuffer);
		if (pipeline) vkDestroyPipeline(m_device, pipeline, nullptr); if (shader) vkDestroyShaderModule(m_device, shader, nullptr); if (pipelineLayout) vkDestroyPipelineLayout(m_device, pipelineLayout, nullptr);
		if (pool) vkDestroyDescriptorPool(m_device, pool, nullptr); if (setLayout) vkDestroyDescriptorSetLayout(m_device, setLayout, nullptr); if (sampler) vkDestroySampler(m_device, sampler, nullptr);
		for (uint32_t i = 0; i < kImageCount; ++i) { if (imageViews[i]) vkDestroyImageView(m_device, imageViews[i], nullptr); if (images[i]) vkDestroyImage(m_device, images[i], nullptr); if (imageMemory[i]) vkFreeMemory(m_device, imageMemory[i], nullptr); }
		auto destroyBuffer = [&](VkBuffer &buffer, VkDeviceMemory &memory) { if (buffer) vkDestroyBuffer(m_device, buffer, nullptr); if (memory) vkFreeMemory(m_device, memory, nullptr); buffer = VK_NULL_HANDLE; memory = VK_NULL_HANDLE; };
		destroyBuffer(imageStaging, imageStagingMemory);
		for (uint32_t i = 0; i < 4; ++i) { destroyBuffer(inputStaging[i], inputStagingMemory[i]); destroyBuffer(inputBuffers[i], inputMemory[i]); }
		for (uint32_t i = 0; i < 7; ++i) { destroyBuffer(outputBuffers[i], outputMemory[i]); destroyBuffer(readbackBuffers[i], readbackMemory[i]); }
	};
	const auto gpuSetupStart = std::chrono::steady_clock::now();
	try
	{
		VkPhysicalDeviceProperties deviceProperties{}; vkGetPhysicalDeviceProperties(m_physicalDevice, &deviceProperties);
		const auto &limits = deviceProperties.limits;
		if (limits.maxPerStageDescriptorSamplers < 18 || limits.maxPerStageDescriptorSampledImages < 18 || limits.maxPerStageDescriptorStorageBuffers < 11 || limits.maxPerStageResources < 29) throw std::runtime_error("Vulkan descriptor limits cannot support the fixed PlaneSweep descriptor contract.");
		VkPhysicalDeviceMemoryProperties memoryProperties{}; vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memoryProperties);
		auto memoryType = [&](uint32_t bits, VkMemoryPropertyFlags flags) { for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; ++i) if ((bits & (1u << i)) && (memoryProperties.memoryTypes[i].propertyFlags & flags) == flags) return i; return UINT32_MAX; };
		auto createBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags flags, VkBuffer &buffer, VkDeviceMemory &memory) { VkBufferCreateInfo ci{}; ci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO; ci.size = std::max<VkDeviceSize>(size, 4); ci.usage = usage; ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE; if (vkCreateBuffer(m_device, &ci, nullptr, &buffer) != VK_SUCCESS) throw std::runtime_error("Failed to create Phase 2P buffer."); VkMemoryRequirements req{}; vkGetBufferMemoryRequirements(m_device, buffer, &req); uint32_t type = memoryType(req.memoryTypeBits, flags); if (type == UINT32_MAX) throw std::runtime_error("No memory type for Phase 2P buffer."); VkMemoryAllocateInfo ai{}; ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; ai.allocationSize = req.size; ai.memoryTypeIndex = type; if (vkAllocateMemory(m_device, &ai, nullptr, &memory) != VK_SUCCESS || vkBindBufferMemory(m_device, buffer, memory, 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate Phase 2P buffer memory."); };
		std::vector<VkDeviceSize> offsets(kImageCount); VkDeviceSize stagingSize = 0;
		for (uint32_t i = 0; i < kImageCount; ++i) { const VkDeviceSize bytes = static_cast<VkDeviceSize>(upload[i].pixels->size()); offsets[i] = stagingSize; stagingSize += bytes; }
		createBuffer(stagingSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, imageStaging, imageStagingMemory);
		void *mapped = nullptr; if (vkMapMemory(m_device, imageStagingMemory, 0, stagingSize, 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map Phase 2P image staging.");
		for (uint32_t i = 0; i < kImageCount; ++i) std::memcpy(static_cast<uint8_t *>(mapped) + offsets[i], upload[i].pixels->data(), upload[i].pixels->size()); vkUnmapMemory(m_device, imageStagingMemory);
		for (uint32_t i = 0; i < kImageCount; ++i)
		{
			VkImageCreateInfo ci{}; ci.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO; ci.imageType = VK_IMAGE_TYPE_2D; ci.format = kFormat; ci.extent = {upload[i].width, upload[i].height, 1}; ci.mipLevels = 1; ci.arrayLayers = 1; ci.samples = VK_SAMPLE_COUNT_1_BIT; ci.tiling = VK_IMAGE_TILING_OPTIMAL; ci.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT; ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE; ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			if (vkCreateImage(m_device, &ci, nullptr, &images[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create Phase 2P image."); VkMemoryRequirements req{}; vkGetImageMemoryRequirements(m_device, images[i], &req); uint32_t type = memoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT); if (type == UINT32_MAX) throw std::runtime_error("No device-local memory for Phase 2P image."); VkMemoryAllocateInfo ai{}; ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; ai.allocationSize = req.size; ai.memoryTypeIndex = type; if (vkAllocateMemory(m_device, &ai, nullptr, &imageMemory[i]) != VK_SUCCESS || vkBindImageMemory(m_device, images[i], imageMemory[i], 0) != VK_SUCCESS) throw std::runtime_error("Failed to bind Phase 2P image memory.");
			VkImageViewCreateInfo vi{}; vi.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO; vi.image = images[i]; vi.viewType = VK_IMAGE_VIEW_TYPE_2D; vi.format = kFormat; vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}; if (vkCreateImageView(m_device, &vi, nullptr, &imageViews[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create Phase 2P image view.");
		}
		VkFormatProperties formatProperties{}; vkGetPhysicalDeviceFormatProperties(m_physicalDevice, kFormat, &formatProperties); if ((formatProperties.optimalTilingFeatures & (VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT | VK_FORMAT_FEATURE_TRANSFER_DST_BIT)) != (VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT | VK_FORMAT_FEATURE_TRANSFER_DST_BIT)) throw std::runtime_error("VK_FORMAT_R8G8B8A8_UNORM sampled upload is unsupported.");
		VkSamplerCreateInfo si{}; si.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO; si.magFilter = VK_FILTER_NEAREST; si.minFilter = VK_FILTER_NEAREST; si.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST; si.addressModeU = si.addressModeV = si.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE; si.maxLod = 0.0f; if (vkCreateSampler(m_device, &si, nullptr, &sampler) != VK_SUCCESS) throw std::runtime_error("Failed to create Phase 2P sampler.");
		for (uint32_t i = 0; i < 4; ++i) { createBuffer(inputSizes[i], VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, inputStaging[i], inputStagingMemory[i]); if (vkMapMemory(m_device, inputStagingMemory[i], 0, inputSizes[i], 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map Phase 2P input."); std::memcpy(mapped, inputData[i], static_cast<size_t>(inputSizes[i])); vkUnmapMemory(m_device, inputStagingMemory[i]); createBuffer(inputSizes[i], VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, inputBuffers[i], inputMemory[i]); }
		for (uint32_t i = 0; i < 7; ++i) { createBuffer(outputSizes[i], VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | (i == 0 ? VK_BUFFER_USAGE_TRANSFER_DST_BIT : 0), VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, outputBuffers[i], outputMemory[i]); createBuffer(outputSizes[i], VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, readbackBuffers[i], readbackMemory[i]); }
		std::array<VkDescriptorSetLayoutBinding, 15> bindings{}; for (uint32_t i = 0; i < 15; ++i) { bindings[i].binding = i; bindings[i].descriptorType = i < 4 ? VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; bindings[i].descriptorCount = (i == 2 || i == 3) ? 8 : 1; bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT; }
		VkDescriptorSetLayoutCreateInfo lci{}; lci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO; lci.bindingCount = 15; lci.pBindings = bindings.data(); if (vkCreateDescriptorSetLayout(m_device, &lci, nullptr, &setLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create Phase 2P descriptor layout.");
		std::array<VkDescriptorPoolSize, 2> poolSizes{{{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 18}, {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 11}}}; VkDescriptorPoolCreateInfo pci{}; pci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO; pci.maxSets = 1; pci.poolSizeCount = 2; pci.pPoolSizes = poolSizes.data(); if (vkCreateDescriptorPool(m_device, &pci, nullptr, &pool) != VK_SUCCESS) throw std::runtime_error("Failed to create Phase 2P descriptor pool."); VkDescriptorSetAllocateInfo sai{}; sai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO; sai.descriptorPool = pool; sai.descriptorSetCount = 1; sai.pSetLayouts = &setLayout; if (vkAllocateDescriptorSets(m_device, &sai, &descriptorSet) != VK_SUCCESS) throw std::runtime_error("Failed to allocate Phase 2P descriptor set.");
		std::array<VkDescriptorImageInfo, 1> referenceInfo{{{sampler, imageViews[0], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}}}, referenceMaskInfo{{{sampler, imageViews[1], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}}}; std::array<VkDescriptorImageInfo, 8> sourceInfo{}, maskInfo{}; for (uint32_t i = 0; i < 8; ++i) { sourceInfo[i] = {sampler, imageViews[2 + i], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}; maskInfo[i] = {sampler, imageViews[10 + i], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}; }
		std::array<VkDescriptorBufferInfo, 11> bufferInfo{}; for (uint32_t i = 0; i < 4; ++i) bufferInfo[i] = {inputBuffers[i], 0, inputSizes[i]}; for (uint32_t i = 0; i < 7; ++i) bufferInfo[4 + i] = {outputBuffers[i], 0, outputSizes[i]}; std::array<VkWriteDescriptorSet, 15> writes{}; for (uint32_t i = 0; i < 15; ++i) { writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; writes[i].dstSet = descriptorSet; writes[i].dstBinding = i; writes[i].descriptorType = i < 4 ? VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; } writes[0].descriptorCount = writes[1].descriptorCount = 1; writes[0].pImageInfo = referenceInfo.data(); writes[1].pImageInfo = referenceMaskInfo.data(); writes[2].descriptorCount = writes[3].descriptorCount = 8; writes[2].pImageInfo = sourceInfo.data(); writes[3].pImageInfo = maskInfo.data(); for (uint32_t i = 0; i < 11; ++i) { writes[4 + i].descriptorCount = 1; writes[4 + i].pBufferInfo = &bufferInfo[i]; } vkUpdateDescriptorSets(m_device, 15, writes.data(), 0, nullptr);
		const std::vector<char> code = ReadShaderFile("shaders/planeSweepDepthMapSmokeTest.comp.spv"); if (code.empty()) throw std::runtime_error("Phase 2P depth-map SPIR-V is missing."); VkShaderModuleCreateInfo sci{}; sci.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO; sci.codeSize = code.size(); sci.pCode = reinterpret_cast<const uint32_t *>(code.data()); if (vkCreateShaderModule(m_device, &sci, nullptr, &shader) != VK_SUCCESS) throw std::runtime_error("Failed to create Phase 2P shader."); VkPipelineLayoutCreateInfo pli{}; pli.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO; pli.setLayoutCount = 1; pli.pSetLayouts = &setLayout; if (vkCreatePipelineLayout(m_device, &pli, nullptr, &pipelineLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create Phase 2P pipeline layout."); VkPipelineShaderStageCreateInfo stage{}; stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = shader; stage.pName = "main"; VkComputePipelineCreateInfo cpci{}; cpci.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO; cpci.stage = stage; cpci.layout = pipelineLayout; if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &cpci, nullptr, &pipeline) != VK_SUCCESS) throw std::runtime_error("Failed to create Phase 2P compute pipeline.");
		VkCommandBufferAllocateInfo cai{}; cai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO; cai.commandPool = m_commandPool; cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; cai.commandBufferCount = 1; if (vkAllocateCommandBuffers(m_device, &cai, &commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to allocate Phase 2P command buffer."); VkCommandBufferBeginInfo cbi{}; cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO; cbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT; if (vkBeginCommandBuffer(commandBuffer, &cbi) != VK_SUCCESS) throw std::runtime_error("Failed to begin Phase 2P command buffer.");
		std::array<VkImageMemoryBarrier, kImageCount> imageToTransfer{}; for (uint32_t i = 0; i < kImageCount; ++i) { auto &b = imageToTransfer[i]; b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; b.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED; b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; b.image = images[i]; b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, kImageCount, imageToTransfer.data());
		for (uint32_t i = 0; i < kImageCount; ++i) { VkBufferImageCopy copy{}; copy.bufferOffset = offsets[i]; copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1}; copy.imageExtent = {upload[i].width, upload[i].height, 1}; vkCmdCopyBufferToImage(commandBuffer, imageStaging, images[i], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy); }
		std::array<VkImageMemoryBarrier, kImageCount> imageToRead{}; for (uint32_t i = 0; i < kImageCount; ++i) { auto &b = imageToRead[i]; b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT; b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; b.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; b.image = images[i]; b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 0, nullptr, kImageCount, imageToRead.data());
		for (uint32_t i = 0; i < 4; ++i) { VkBufferCopy copy{}; copy.size = inputSizes[i]; vkCmdCopyBuffer(commandBuffer, inputStaging[i], inputBuffers[i], 1, &copy); } std::array<VkBufferMemoryBarrier, 4> inputBarriers{}; for (uint32_t i = 0; i < 4; ++i) { auto &b = inputBarriers[i]; b.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT; b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; b.buffer = inputBuffers[i]; b.size = inputSizes[i]; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 4, inputBarriers.data(), 0, nullptr);
		vkCmdFillBuffer(commandBuffer, outputBuffers[0], 0, depthBytes, 0); VkBufferMemoryBarrier zeroBarrier{}; zeroBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; zeroBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; zeroBarrier.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT; zeroBarrier.srcQueueFamilyIndex = zeroBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; zeroBarrier.buffer = outputBuffers[0]; zeroBarrier.size = depthBytes; vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 1, &zeroBarrier, 0, nullptr);
		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline); vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr); vkCmdDispatch(commandBuffer, (gridWidth + 7u) / 8u, (gridHeight + 7u) / 8u, 1);
		std::array<VkBufferMemoryBarrier, 7> outputBarriers{}; for (uint32_t i = 0; i < 7; ++i) { auto &b = outputBarriers[i]; b.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; b.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT; b.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT; b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; b.buffer = outputBuffers[i]; b.size = outputSizes[i]; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 7, outputBarriers.data(), 0, nullptr);
		for (uint32_t i = 0; i < 7; ++i) { VkBufferCopy copy{}; copy.size = outputSizes[i]; vkCmdCopyBuffer(commandBuffer, outputBuffers[i], readbackBuffers[i], 1, &copy); } std::array<VkBufferMemoryBarrier, 7> hostBarriers{}; for (uint32_t i = 0; i < 7; ++i) { auto &b = hostBarriers[i]; b.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; b.dstAccessMask = VK_ACCESS_HOST_READ_BIT; b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; b.buffer = readbackBuffers[i]; b.size = outputSizes[i]; } vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 0, nullptr, 7, hostBarriers.data(), 0, nullptr);
		if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to end Phase 2P command buffer."); VkFenceCreateInfo fi{}; fi.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO; if (vkCreateFence(m_device, &fi, nullptr, &fence) != VK_SUCCESS) throw std::runtime_error("Failed to create Phase 2P fence.");
		const double setupMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - gpuSetupStart).count();
		const auto dispatchStart = std::chrono::steady_clock::now(); VkSubmitInfo submit{}; submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO; submit.commandBufferCount = 1; submit.pCommandBuffers = &commandBuffer; if (vkQueueSubmit(m_queue, 1, &submit, fence) != VK_SUCCESS) throw std::runtime_error("Phase 2P queue submit failed (possible device loss). workload=" + std::to_string(workload)); submitted = true; const VkResult waitResult = vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX); if (waitResult != VK_SUCCESS) throw std::runtime_error("Phase 2P fence wait failed VkResult=" + std::to_string(waitResult) + ", dimensions=" + std::to_string(scaledWidth) + "x" + std::to_string(scaledHeight) + ", grid=" + std::to_string(gridWidth) + "x" + std::to_string(gridHeight) + ", planes=" + std::to_string(params.m_planeCount) + ", sources=" + std::to_string(sourceImages.size()) + ", patchRadius=" + std::to_string(params.m_patchRadius) + ", workload=" + std::to_string(workload)); submitted = false; const double dispatchMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - dispatchStart).count();
		const auto compareStart = std::chrono::steady_clock::now(); std::vector<float> gpuDepth(entryCount); std::vector<uint32_t> gpuStatuses(sampleCount), gpuTextureTrace(sampleCount * textureTraceStride), gpuTextureSamples(sampleCount); std::array<void *, 7> destinations{{gpuDepth.data(), mappings.data(), gpuStatuses.data(), gpuTextureTrace.data(), gpuTextureSamples.data(), opaqueSamples.data(), planeSamples.data()}};
		for (uint32_t i = 0; i < 7; ++i) { if (vkMapMemory(m_device, readbackMemory[i], 0, outputSizes[i], 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map Phase 2P readback."); std::memcpy(destinations[i], mapped, static_cast<size_t>(outputSizes[i])); vkUnmapMemory(m_device, readbackMemory[i]); }
		std::vector<float> planeDepths(static_cast<size_t>(params.m_planeCount)); for (int i = 0; i < params.m_planeCount; ++i) { const float t = static_cast<float>(i) / static_cast<float>(params.m_planeCount - 1); const float inv = inverseNear * (1.0f - t) + inverseFar * t; planeDepths[i] = 1.0f / std::max(inv, 0.000001f); }
		auto closestPlane = [&](float d) { uint32_t index = 0; float error = std::numeric_limits<float>::max(); for (uint32_t i = 0; i < planeDepths.size(); ++i) { const float e = std::fabs(d - planeDepths[i]); if (e < error) { index = i; error = e; } } const float tol = std::max(1e-5f, 4.0f * std::numeric_limits<float>::epsilon() * std::max({1.0f, std::fabs(d), std::fabs(planeDepths[index])})); return std::pair<uint32_t, bool>{index, error <= tol}; };
		size_t bothInvalid = 0, cpuOnly = 0, gpuOnly = 0, samePlane = 0, differentPlane = 0, unsampled = 0, unsampledNonzero = 0, gpuNonfinite = 0, cpuPositive = 0, gpuPositive = 0; double maxAbs = 0.0, maxRel = 0.0; std::vector<size_t> mismatches;
		for (size_t index = 0; index < entryCount; ++index)
		{
			const uint32_t x = static_cast<uint32_t>(index % scaledWidth), y = static_cast<uint32_t>(index / scaledWidth); const bool sampled = x % pointStep == 0 && y % pointStep == 0; const float c = cpuDepthMap.m_depthValues[index], g = gpuDepth[index]; const bool cv = std::isfinite(c) && c > 0.0f, gv = std::isfinite(g) && g > 0.0f;
			if (!std::isfinite(g)) { ++gpuNonfinite; mismatches.push_back(index); } if (cv) ++cpuPositive; if (gv) ++gpuPositive;
			if (!sampled) { ++unsampled; if (c != 0.0f || g != 0.0f) { ++unsampledNonzero; mismatches.push_back(index); } }
			if (!cv && !gv) ++bothInvalid; else if (cv && !gv) { ++cpuOnly; mismatches.push_back(index); } else if (!cv && gv) { ++gpuOnly; mismatches.push_back(index); } else { const auto cp = closestPlane(c), gp = closestPlane(g); if (!cp.second || !gp.second) mismatches.push_back(index); if (cp.first == gp.first && cp.second && gp.second) { ++samePlane; const double absolute = std::fabs(static_cast<double>(g) - c); maxAbs = std::max(maxAbs, absolute); maxRel = std::max(maxRel, absolute / std::max(std::fabs(static_cast<double>(c)), std::numeric_limits<double>::min())); } else { ++differentPlane; mismatches.push_back(index); } }
		}
		const size_t classificationMismatches = cpuOnly + gpuOnly;
		std::array<size_t, 6> gpuStatusCounts{}; for (uint32_t status : gpuStatuses) if (status < gpuStatusCounts.size()) ++gpuStatusCounts[status];
		std::ostringstream report; report.precision(9); report << "Phase 2P diagnostic timing ms: upload/resource setup=" << setupMs << ", dispatch+fence=" << dispatchMs << ", readback+comparison=" << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - compareStart).count() << ", CPU BuildReferencePoints=" << cpuBuildMilliseconds << "\nCPU accepted depths=" << cpuPositive << ", GPU accepted depths=" << gpuPositive << ", CPU pointCandidates for this map=" << cpuPointCandidateCount << " (candidate vector total=" << cpuPointCandidates.size() << ")\nCPU rejection deltas: texture=" << cpuCounters.m_pointsRejectedTexture << ", confidence=" << cpuCounters.m_pointsRejectedConfidence << "; GPU sampled status counts: no-valid-plane=" << gpuStatusCounts[0] << ", cost-rejected=" << gpuStatusCounts[1] << ", confidence-rejected=" << gpuStatusCounts[2] << ", accepted=" << gpuStatusCounts[3] << ", reference-invalid=" << gpuStatusCounts[4] << ", texture-rejected=" << gpuStatusCounts[5] << "\nRejection counter comparisons (CPU vs GPU sampled): texture=" << cpuCounters.m_pointsRejectedTexture << "/" << gpuStatusCounts[5] << ", confidence=" << cpuCounters.m_pointsRejectedConfidence << "/" << gpuStatusCounts[2] << "\n";
		const size_t mutuallyValid = samePlane + differentPlane;
		const bool success = gpuNonfinite == 0 && classificationMismatches == 0 && differentPlane == 0 && unsampledNonzero == 0 && mismatches.empty();
		if (success) report << "PlaneSweep GPU real-data depth-map comparison succeeded:\nReference: " << referenceId << " " << referenceName << "\nSource views: " << sourceImages.size() << "\nReference texture: " << refWidth << " x " << refHeight << "\nScaled depth map: " << scaledWidth << " x " << scaledHeight << "\nSample grid: " << gridWidth << " x " << gridHeight << "\nPlane count: " << params.m_planeCount << "\nPatch radius: " << params.m_patchRadius << "\n" << entryCount << " / " << entryCount << " valid/invalid classifications matched.\n" << samePlane << " / " << mutuallyValid << " positive depths selected the same plane.\n" << unsampled - unsampledNonzero << " / " << unsampled << " unsampled pointStep entries remained zero.\nMaximum positive-depth absolute error: " << maxAbs << "\nMaximum positive-depth relative error: " << maxRel << "\nVulkan validation: inspect runtime output; this method does not maintain an error counter.\n";
		else report << "PlaneSweep GPU real-data depth-map comparison NOT YET EQUIVALENT:\ntotal entries=" << entryCount << ", classification mismatch count=" << classificationMismatches << ", different-plane count=" << differentPlane << ", CPU-only-valid count=" << cpuOnly << ", GPU-only-valid count=" << gpuOnly << ", matching-positive count=" << samePlane << ", maximum same-plane depth error=" << maxAbs << ", GPU NaN/Inf=" << gpuNonfinite << ", unsampled nonzero=" << unsampledNonzero << "\n";
		for (size_t n = 0; n < std::min<size_t>(20, mismatches.size()); ++n) { const size_t index = mismatches[n]; const uint32_t x = static_cast<uint32_t>(index % scaledWidth), y = static_cast<uint32_t>(index / scaledWidth); const uint32_t rx = static_cast<uint32_t>((static_cast<size_t>(x) * refWidth) / scaledWidth), ry = static_cast<uint32_t>((static_cast<size_t>(y) * refHeight) / scaledHeight); const auto cp = std::isfinite(cpuDepthMap.m_depthValues[index]) && cpuDepthMap.m_depthValues[index] > 0.0f ? closestPlane(cpuDepthMap.m_depthValues[index]) : std::pair<uint32_t, bool>{UINT32_MAX, false}; const auto gp = std::isfinite(gpuDepth[index]) && gpuDepth[index] > 0.0f ? closestPlane(gpuDepth[index]) : std::pair<uint32_t, bool>{UINT32_MAX, false}; const size_t sampleIndex = static_cast<size_t>(y / pointStep) * gridWidth + x / pointStep; report << "Mismatch " << n << ": depthIndex=" << index << ", scaled=(" << x << "," << y << "), reference=(" << rx << "," << ry << "), CPU=" << cpuDepthMap.m_depthValues[index] << ", GPU=" << gpuDepth[index] << ", CPU valid=" << (cp.first != UINT32_MAX) << ", GPU valid=" << (gp.first != UINT32_MAX) << ", CPU plane=" << cp.first << ", GPU plane=" << gp.first; if (x % pointStep == 0 && y % pointStep == 0 && sampleIndex < gpuStatuses.size()) { report << ", GPU status=" << gpuStatuses[sampleIndex]; const float stdDev = FloatFromBits(gpuTextureTrace[sampleIndex * textureTraceStride + 9u]); report << ", GPU textureStdDev=" << stdDev; } report << "\n"; }
		UIUtility::Print(report.str()); cleanup(); return success;
	}
	catch (const std::exception &ex) { cleanup(); UIUtility::Print(std::string("PlaneSweep GPU real-data comparison failed: ") + ex.what() + "\n"); return false; }
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

bool PlaneSweepGpu::RunPlaneSweepPixelSamplingSmokeTest(void)
{
	if (!m_initialized)
	{
		throw std::runtime_error("PlaneSweepGpu not initialized.");
	}

	constexpr uint32_t kImageWidth = 4;
	constexpr uint32_t kImageHeight = 4;
	constexpr uint32_t kMaskWidth = 3;
	constexpr uint32_t kMaskHeight = 2;
	constexpr uint32_t kCaseCount = 16;
	constexpr VkFormat kFormat = VK_FORMAT_R8G8B8A8_UNORM;
	constexpr VkDeviceSize kImageBytes = kImageWidth * kImageHeight * 4;
	constexpr VkDeviceSize kMaskBytes = kMaskWidth * kMaskHeight * 4;
	constexpr VkDeviceSize kCoordinateBytes = kCaseCount * 2 * sizeof(float);
	constexpr VkDeviceSize kModeBytes = kCaseCount * 2 * sizeof(uint32_t);
	constexpr VkDeviceSize kColorBytes = kCaseCount * 4 * sizeof(float);
	constexpr VkDeviceSize kValidityBytes = kCaseCount * sizeof(uint32_t);
	struct SampleCase { float x; float y; uint32_t hasMask; uint32_t maskUsesAlpha; };
	const std::array<SampleCase, kCaseCount> cases{{
		{ 0.0f, 0.0f, 0, 0 }, { 1.5f, 1.5f, 0, 0 }, { 3.0f, 3.0f, 0, 0 }, { -0.001f, 1.0f, 0, 0 },
		{ 3.001f, 2.0f, 0, 0 }, { 0.0f, 0.0f, 1, 1 }, { 1.5f, 1.5f, 1, 1 }, { 3.0f, 3.0f, 1, 1 },
		{ 0.75f, 2.25f, 1, 1 }, { 2.999f, 0.001f, 1, 1 }, { 1.0f, 3.001f, 1, 1 }, { 0.0f, 0.0f, 1, 0 },
		{ 1.5f, 1.5f, 1, 0 }, { 3.0f, 3.0f, 1, 0 }, { 2.25f, 0.75f, 1, 0 }, { 1.0f, -0.001f, 1, 0 }
	}};

	VkFormatProperties formatProperties{};
	vkGetPhysicalDeviceFormatProperties(m_physicalDevice, kFormat, &formatProperties);
	const VkFormatFeatureFlags requiredFeatures = VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT | VK_FORMAT_FEATURE_TRANSFER_DST_BIT;
	if ((formatProperties.optimalTilingFeatures & requiredFeatures) != requiredFeatures)
	{
		UIUtility::Print("SamplePlaneSweepPixel smoke test unsupported: VK_FORMAT_R8G8B8A8_UNORM optimal tiling lacks sampled-image or transfer-destination support.\n");
		return false;
	}

	std::array<uint8_t, kImageBytes> imagePixels{};
	for (uint32_t y = 0; y < kImageHeight; ++y)
	{
		for (uint32_t x = 0; x < kImageWidth; ++x)
		{
			const uint32_t index = y * kImageWidth + x;
			imagePixels[index * 4 + 0] = static_cast<uint8_t>(10 + x * 50);
			imagePixels[index * 4 + 1] = static_cast<uint8_t>(20 + y * 50);
			imagePixels[index * 4 + 2] = static_cast<uint8_t>(30 + index * 7);
			imagePixels[index * 4 + 3] = static_cast<uint8_t>(255 - index * 9);
		}
	}
	const std::array<uint8_t, kMaskBytes> maskPixels{{
		20, 40, 60, 25, 80, 100, 120, 75, 140, 160, 180, 125,
		30, 70, 110, 175, 90, 130, 170, 225, 150, 190, 230, 250
	}};
	std::array<float, kCaseCount * 2> coordinateValues{};
	std::array<uint32_t, kCaseCount * 2> modeValues{};
	for (uint32_t i = 0; i < kCaseCount; ++i)
	{
		coordinateValues[i * 2] = cases[i].x;
		coordinateValues[i * 2 + 1] = cases[i].y;
		modeValues[i * 2] = cases[i].hasMask;
		modeValues[i * 2 + 1] = cases[i].maskUsesAlpha;
	}

	struct CpuExpected { bool valid{ false }; Float4 imageColor{}; Float4 maskColor{}; Float4 color{}; float maskX{ 0.0f }; float maskY{ 0.0f }; };
	std::array<CpuExpected, kCaseCount> expected{};
	auto sampleBilinear = [](const uint8_t *source, uint32_t width, uint32_t height, float x, float y, Float4 &color)
	{
		const size_t sourceBytes = static_cast<size_t>(width) * height * 4;
		if (width == 0 || height == 0 || sourceBytes == 0 || x < 0.0f || y < 0.0f || x > static_cast<float>(width - 1) || y > static_cast<float>(height - 1)) return false;
		const uint32_t x0 = static_cast<uint32_t>(std::floor(x));
		const uint32_t y0 = static_cast<uint32_t>(std::floor(y));
		const uint32_t x1 = std::min(width - 1, x0 + 1);
		const uint32_t y1 = std::min(height - 1, y0 + 1);
		const float tx = x - static_cast<float>(x0);
		const float ty = y - static_cast<float>(y0);
		float channels[4]{};
		for (uint32_t channel = 0; channel < 4; ++channel)
		{
			auto sample = [&](uint32_t sx, uint32_t sy)
			{
				const size_t index = ((static_cast<size_t>(sy) * width + sx) * 4) + channel;
				return index < sourceBytes ? static_cast<float>(source[index]) / 255.0f : 0.0f;
			};
			const float top = (sample(x0, y0) * (1.0f - tx)) + (sample(x1, y0) * tx);
			const float bottom = (sample(x0, y1) * (1.0f - tx)) + (sample(x1, y1) * tx);
			channels[channel] = (top * (1.0f - ty)) + (bottom * ty);
		}
		color = Float4{ channels[0], channels[1], channels[2], channels[3] };
		return true;
	};
	uint32_t validCount = 0;
	for (uint32_t i = 0; i < kCaseCount; ++i)
	{
		CpuExpected &result = expected[i];
		result.valid = sampleBilinear(imagePixels.data(), kImageWidth, kImageHeight, cases[i].x, cases[i].y, result.imageColor);
		if (!result.valid) continue;
		result.color = result.imageColor;
		if (cases[i].hasMask != 0)
		{
			const float imageWidth = static_cast<float>(std::max(1U, kImageWidth - 1));
			const float imageHeight = static_cast<float>(std::max(1U, kImageHeight - 1));
			result.maskX = (cases[i].x / imageWidth) * static_cast<float>(std::max(1U, kMaskWidth - 1));
			result.maskY = (cases[i].y / imageHeight) * static_cast<float>(std::max(1U, kMaskHeight - 1));
			if (!sampleBilinear(maskPixels.data(), kMaskWidth, kMaskHeight, result.maskX, result.maskY, result.maskColor))
			{
				result.valid = false;
				continue;
			}
			result.color.w = cases[i].maskUsesAlpha != 0
				? result.maskColor.w
				: (result.maskColor.x * 0.299f) + (result.maskColor.y * 0.587f) + (result.maskColor.z * 0.114f);
		}
		++validCount;
	}

	VkBuffer imageStaging[2]{ VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkDeviceMemory imageStagingMemory[2]{ VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkImage images[2]{ VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkDeviceMemory imageMemory[2]{ VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkImageView imageViews[2]{ VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkSampler samplers[2]{ VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkBuffer coordinateStaging = VK_NULL_HANDLE, modeStaging = VK_NULL_HANDLE;
	VkDeviceMemory coordinateStagingMemory = VK_NULL_HANDLE, modeStagingMemory = VK_NULL_HANDLE;
	VkBuffer coordinateInput = VK_NULL_HANDLE, modeInput = VK_NULL_HANDLE, colorOutput = VK_NULL_HANDLE, validOutput = VK_NULL_HANDLE;
	VkDeviceMemory coordinateInputMemory = VK_NULL_HANDLE, modeInputMemory = VK_NULL_HANDLE, colorOutputMemory = VK_NULL_HANDLE, validOutputMemory = VK_NULL_HANDLE;
	VkBuffer colorReadback = VK_NULL_HANDLE, validReadback = VK_NULL_HANDLE;
	VkDeviceMemory colorReadbackMemory = VK_NULL_HANDLE, validReadbackMemory = VK_NULL_HANDLE;
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
		for (uint32_t i = 0; i < 2; ++i)
		{
			if (samplers[i] != VK_NULL_HANDLE) { vkDestroySampler(m_device, samplers[i], nullptr); samplers[i] = VK_NULL_HANDLE; }
			if (imageViews[i] != VK_NULL_HANDLE) { vkDestroyImageView(m_device, imageViews[i], nullptr); imageViews[i] = VK_NULL_HANDLE; }
			if (images[i] != VK_NULL_HANDLE) { vkDestroyImage(m_device, images[i], nullptr); images[i] = VK_NULL_HANDLE; }
			if (imageMemory[i] != VK_NULL_HANDLE) { vkFreeMemory(m_device, imageMemory[i], nullptr); imageMemory[i] = VK_NULL_HANDLE; }
		}
		auto destroyBuffer = [&](VkBuffer &buffer, VkDeviceMemory &memory)
		{
			if (buffer != VK_NULL_HANDLE) { vkDestroyBuffer(m_device, buffer, nullptr); buffer = VK_NULL_HANDLE; }
			if (memory != VK_NULL_HANDLE) { vkFreeMemory(m_device, memory, nullptr); memory = VK_NULL_HANDLE; }
		};
		destroyBuffer(imageStaging[0], imageStagingMemory[0]); destroyBuffer(imageStaging[1], imageStagingMemory[1]);
		destroyBuffer(coordinateStaging, coordinateStagingMemory); destroyBuffer(modeStaging, modeStagingMemory);
		destroyBuffer(coordinateInput, coordinateInputMemory); destroyBuffer(modeInput, modeInputMemory);
		destroyBuffer(colorOutput, colorOutputMemory); destroyBuffer(validOutput, validOutputMemory);
		destroyBuffer(colorReadback, colorReadbackMemory); destroyBuffer(validReadback, validReadbackMemory);
	};
	auto findMemoryType = [&](uint32_t typeBits, VkMemoryPropertyFlags required)
	{
		VkPhysicalDeviceMemoryProperties properties{};
		vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &properties);
		for (uint32_t i = 0; i < properties.memoryTypeCount; ++i) if ((typeBits & (1u << i)) && (properties.memoryTypes[i].propertyFlags & required) == required) return i;
		return UINT32_MAX;
	};
	auto createBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer &buffer, VkDeviceMemory &memory)
	{
		VkBufferCreateInfo info{}; info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO; info.size = size; info.usage = usage; info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
		if (vkCreateBuffer(m_device, &info, nullptr, &buffer) != VK_SUCCESS) throw std::runtime_error("Failed to create SamplePlaneSweepPixel buffer.");
		VkMemoryRequirements requirements{}; vkGetBufferMemoryRequirements(m_device, buffer, &requirements);
		const uint32_t type = findMemoryType(requirements.memoryTypeBits, properties);
		if (type == UINT32_MAX) throw std::runtime_error("No suitable memory type for SamplePlaneSweepPixel buffer.");
		VkMemoryAllocateInfo allocation{}; allocation.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; allocation.allocationSize = requirements.size; allocation.memoryTypeIndex = type;
		if (vkAllocateMemory(m_device, &allocation, nullptr, &memory) != VK_SUCCESS) throw std::runtime_error("Failed to allocate SamplePlaneSweepPixel buffer memory.");
		if (vkBindBufferMemory(m_device, buffer, memory, 0) != VK_SUCCESS) throw std::runtime_error("Failed to bind SamplePlaneSweepPixel buffer memory.");
	};
	auto copyToMemory = [&](VkDeviceMemory memory, VkDeviceSize size, const void *data)
	{
		void *mapped = nullptr;
		if (vkMapMemory(m_device, memory, 0, size, 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map SamplePlaneSweepPixel staging memory.");
		std::memcpy(mapped, data, static_cast<size_t>(size));
		vkUnmapMemory(m_device, memory);
	};

	try
	{
		const VkMemoryPropertyFlags hostFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
		const uint32_t imageWidths[2]{ kImageWidth, kMaskWidth };
		const uint32_t imageHeights[2]{ kImageHeight, kMaskHeight };
		const uint8_t *imageData[2]{ imagePixels.data(), maskPixels.data() };
		const VkDeviceSize imageSizes[2]{ kImageBytes, kMaskBytes };
		for (uint32_t i = 0; i < 2; ++i)
		{
			createBuffer(imageSizes[i], VK_BUFFER_USAGE_TRANSFER_SRC_BIT, hostFlags, imageStaging[i], imageStagingMemory[i]);
			copyToMemory(imageStagingMemory[i], imageSizes[i], imageData[i]);
			VkImageCreateInfo info{}; info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO; info.imageType = VK_IMAGE_TYPE_2D; info.format = kFormat;
			info.extent = VkExtent3D{ imageWidths[i], imageHeights[i], 1 }; info.mipLevels = 1; info.arrayLayers = 1; info.samples = VK_SAMPLE_COUNT_1_BIT;
			info.tiling = VK_IMAGE_TILING_OPTIMAL; info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT; info.sharingMode = VK_SHARING_MODE_EXCLUSIVE; info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			if (vkCreateImage(m_device, &info, nullptr, &images[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create SamplePlaneSweepPixel image.");
			VkMemoryRequirements requirements{}; vkGetImageMemoryRequirements(m_device, images[i], &requirements);
			const uint32_t type = findMemoryType(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
			if (type == UINT32_MAX) throw std::runtime_error("No device-local memory for SamplePlaneSweepPixel image.");
			VkMemoryAllocateInfo allocation{}; allocation.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; allocation.allocationSize = requirements.size; allocation.memoryTypeIndex = type;
			if (vkAllocateMemory(m_device, &allocation, nullptr, &imageMemory[i]) != VK_SUCCESS || vkBindImageMemory(m_device, images[i], imageMemory[i], 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate or bind SamplePlaneSweepPixel image memory.");
			VkImageViewCreateInfo view{}; view.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO; view.image = images[i]; view.viewType = VK_IMAGE_VIEW_TYPE_2D; view.format = kFormat; view.subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
			if (vkCreateImageView(m_device, &view, nullptr, &imageViews[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create SamplePlaneSweepPixel image view.");
			VkSamplerCreateInfo samplerInfo{}; samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO; samplerInfo.magFilter = VK_FILTER_NEAREST; samplerInfo.minFilter = VK_FILTER_NEAREST; samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
			samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE; samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE; samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
			samplerInfo.anisotropyEnable = VK_FALSE; samplerInfo.compareEnable = VK_FALSE; samplerInfo.minLod = 0.0f; samplerInfo.maxLod = 0.0f; samplerInfo.unnormalizedCoordinates = VK_FALSE;
			if (vkCreateSampler(m_device, &samplerInfo, nullptr, &samplers[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create SamplePlaneSweepPixel sampler.");
		}

		createBuffer(kCoordinateBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, hostFlags, coordinateStaging, coordinateStagingMemory);
		copyToMemory(coordinateStagingMemory, kCoordinateBytes, coordinateValues.data());
		createBuffer(kModeBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, hostFlags, modeStaging, modeStagingMemory);
		copyToMemory(modeStagingMemory, kModeBytes, modeValues.data());
		createBuffer(kCoordinateBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, coordinateInput, coordinateInputMemory);
		createBuffer(kModeBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, modeInput, modeInputMemory);
		createBuffer(kColorBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, colorOutput, colorOutputMemory);
		createBuffer(kValidityBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, validOutput, validOutputMemory);
		createBuffer(kColorBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, hostFlags, colorReadback, colorReadbackMemory);
		createBuffer(kValidityBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, hostFlags, validReadback, validReadbackMemory);

		std::array<VkDescriptorSetLayoutBinding, 6> bindings{};
		bindings[0] = VkDescriptorSetLayoutBinding{ 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
		bindings[1] = VkDescriptorSetLayoutBinding{ 1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
		for (uint32_t i = 2; i < 6; ++i) bindings[i] = VkDescriptorSetLayoutBinding{ i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
		VkDescriptorSetLayoutCreateInfo descriptorLayoutInfo{}; descriptorLayoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO; descriptorLayoutInfo.bindingCount = 6; descriptorLayoutInfo.pBindings = bindings.data();
		if (vkCreateDescriptorSetLayout(m_device, &descriptorLayoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create SamplePlaneSweepPixel descriptor layout.");
		VkPipelineLayoutCreateInfo pipelineLayoutInfo{}; pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO; pipelineLayoutInfo.setLayoutCount = 1; pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout;
		if (vkCreatePipelineLayout(m_device, &pipelineLayoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create SamplePlaneSweepPixel pipeline layout.");
		const std::vector<char> shaderCode = ReadShaderFile("shaders/planeSweepPixelSamplingSmokeTest.comp.spv");
		if (shaderCode.empty()) throw std::runtime_error("Failed to load SamplePlaneSweepPixel shader SPIR-V.");
		VkShaderModuleCreateInfo shaderInfo{}; shaderInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO; shaderInfo.codeSize = shaderCode.size(); shaderInfo.pCode = reinterpret_cast<const uint32_t *>(shaderCode.data());
		if (vkCreateShaderModule(m_device, &shaderInfo, nullptr, &shaderModule) != VK_SUCCESS) throw std::runtime_error("Failed to create SamplePlaneSweepPixel shader module.");
		VkPipelineShaderStageCreateInfo stage{}; stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = shaderModule; stage.pName = "main";
		VkComputePipelineCreateInfo pipelineInfo{}; pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO; pipelineInfo.stage = stage; pipelineInfo.layout = pipelineLayout;
		if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline) != VK_SUCCESS) throw std::runtime_error("Failed to create SamplePlaneSweepPixel compute pipeline.");

		std::array<VkDescriptorPoolSize, 2> poolSizes{{ { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 2 }, { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 4 } }};
		VkDescriptorPoolCreateInfo poolInfo{}; poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO; poolInfo.maxSets = 1; poolInfo.poolSizeCount = 2; poolInfo.pPoolSizes = poolSizes.data();
		if (vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS) throw std::runtime_error("Failed to create SamplePlaneSweepPixel descriptor pool.");
		VkDescriptorSetAllocateInfo descriptorAllocate{}; descriptorAllocate.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO; descriptorAllocate.descriptorPool = descriptorPool; descriptorAllocate.descriptorSetCount = 1; descriptorAllocate.pSetLayouts = &descriptorSetLayout;
		VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
		if (vkAllocateDescriptorSets(m_device, &descriptorAllocate, &descriptorSet) != VK_SUCCESS) throw std::runtime_error("Failed to allocate SamplePlaneSweepPixel descriptor set.");
		VkDescriptorImageInfo imageInfos[2]{};
		for (uint32_t i = 0; i < 2; ++i) { imageInfos[i].sampler = samplers[i]; imageInfos[i].imageView = imageViews[i]; imageInfos[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; }
		VkDescriptorBufferInfo bufferInfos[4]{{ coordinateInput, 0, kCoordinateBytes }, { modeInput, 0, kModeBytes }, { colorOutput, 0, kColorBytes }, { validOutput, 0, kValidityBytes }};
		std::array<VkWriteDescriptorSet, 6> writes{};
		for (uint32_t i = 0; i < 2; ++i) { writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; writes[i].dstSet = descriptorSet; writes[i].dstBinding = i; writes[i].descriptorCount = 1; writes[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; writes[i].pImageInfo = &imageInfos[i]; }
		for (uint32_t i = 2; i < 6; ++i) { writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; writes[i].dstSet = descriptorSet; writes[i].dstBinding = i; writes[i].descriptorCount = 1; writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; writes[i].pBufferInfo = &bufferInfos[i - 2]; }
		vkUpdateDescriptorSets(m_device, 6, writes.data(), 0, nullptr);

		VkCommandBufferAllocateInfo commandAllocate{}; commandAllocate.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO; commandAllocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; commandAllocate.commandPool = m_commandPool; commandAllocate.commandBufferCount = 1;
		if (vkAllocateCommandBuffers(m_device, &commandAllocate, &commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to allocate SamplePlaneSweepPixel command buffer.");
		VkCommandBufferBeginInfo begin{}; begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO; begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
		if (vkBeginCommandBuffer(commandBuffer, &begin) != VK_SUCCESS) throw std::runtime_error("Failed to begin SamplePlaneSweepPixel command buffer.");
		for (uint32_t i = 0; i < 2; ++i)
		{
			VkImageMemoryBarrier toTransfer{}; toTransfer.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; toTransfer.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; toTransfer.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED; toTransfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
			toTransfer.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toTransfer.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toTransfer.image = images[i]; toTransfer.subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
			vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &toTransfer);
			VkBufferImageCopy copy{}; copy.imageSubresource = VkImageSubresourceLayers{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 }; copy.imageExtent = VkExtent3D{ imageWidths[i], imageHeights[i], 1 };
			vkCmdCopyBufferToImage(commandBuffer, imageStaging[i], images[i], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
			VkImageMemoryBarrier toRead{}; toRead.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; toRead.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; toRead.dstAccessMask = VK_ACCESS_SHADER_READ_BIT; toRead.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; toRead.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
			toRead.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toRead.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toRead.image = images[i]; toRead.subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
			vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &toRead);
		}
		VkBufferCopy coordinateCopy{}; coordinateCopy.size = kCoordinateBytes; vkCmdCopyBuffer(commandBuffer, coordinateStaging, coordinateInput, 1, &coordinateCopy);
		VkBufferCopy modeCopy{}; modeCopy.size = kModeBytes; vkCmdCopyBuffer(commandBuffer, modeStaging, modeInput, 1, &modeCopy);
		VkBufferMemoryBarrier inputBarriers[2]{};
		for (uint32_t i = 0; i < 2; ++i) { inputBarriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; inputBarriers[i].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; inputBarriers[i].dstAccessMask = VK_ACCESS_SHADER_READ_BIT; inputBarriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; inputBarriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; inputBarriers[i].offset = 0; }
		inputBarriers[0].buffer = coordinateInput; inputBarriers[0].size = kCoordinateBytes; inputBarriers[1].buffer = modeInput; inputBarriers[1].size = kModeBytes;
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 2, inputBarriers, 0, nullptr);
		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
		vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
		vkCmdDispatch(commandBuffer, 1, 1, 1);
		VkBufferMemoryBarrier outputBarriers[2]{};
		for (uint32_t i = 0; i < 2; ++i) { outputBarriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; outputBarriers[i].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT; outputBarriers[i].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT; outputBarriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; outputBarriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; outputBarriers[i].offset = 0; }
		outputBarriers[0].buffer = colorOutput; outputBarriers[0].size = kColorBytes; outputBarriers[1].buffer = validOutput; outputBarriers[1].size = kValidityBytes;
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 2, outputBarriers, 0, nullptr);
		VkBufferCopy colorCopy{}; colorCopy.size = kColorBytes; vkCmdCopyBuffer(commandBuffer, colorOutput, colorReadback, 1, &colorCopy);
		VkBufferCopy validCopy{}; validCopy.size = kValidityBytes; vkCmdCopyBuffer(commandBuffer, validOutput, validReadback, 1, &validCopy);
		if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to record SamplePlaneSweepPixel command buffer.");
		VkFenceCreateInfo fenceInfo{}; fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
		if (vkCreateFence(m_device, &fenceInfo, nullptr, &fence) != VK_SUCCESS) throw std::runtime_error("Failed to create SamplePlaneSweepPixel fence.");
		VkSubmitInfo submit{}; submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO; submit.commandBufferCount = 1; submit.pCommandBuffers = &commandBuffer;
		if (vkQueueSubmit(m_queue, 1, &submit, fence) != VK_SUCCESS) throw std::runtime_error("Failed to submit SamplePlaneSweepPixel command buffer.");
		submitted = true;
		if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS) throw std::runtime_error("Failed to wait for SamplePlaneSweepPixel fence.");
		submitted = false;
		vkDestroyFence(m_device, fence, nullptr); fence = VK_NULL_HANDLE;

		void *validMapped = nullptr;
		if (vkMapMemory(m_device, validReadbackMemory, 0, kValidityBytes, 0, &validMapped) != VK_SUCCESS) throw std::runtime_error("Failed to map SamplePlaneSweepPixel validity readback.");
		const uint32_t *actualValidity = static_cast<const uint32_t *>(validMapped);
		bool matched = true;
		for (uint32_t i = 0; i < kCaseCount; ++i)
		{
			const bool actual = actualValidity[i] != 0;
			if (expected[i].valid != actual)
			{
				UIUtility::Print(std::string("SamplePlaneSweepPixel validity mismatch: case ") + std::to_string(i) + " x=" + std::to_string(cases[i].x) + " y=" + std::to_string(cases[i].y) + " hasMask=" + std::to_string(cases[i].hasMask) + " maskUsesAlpha=" + std::to_string(cases[i].maskUsesAlpha) + " expected valid=" + (expected[i].valid ? "true" : "false") + " actual valid=" + (actual ? "true" : "false") + "\n");
				matched = false;
				break;
			}
		}
		vkUnmapMemory(m_device, validReadbackMemory);

		float maximumError = 0.0f;
		if (matched)
		{
			void *colorMapped = nullptr;
			if (vkMapMemory(m_device, colorReadbackMemory, 0, kColorBytes, 0, &colorMapped) != VK_SUCCESS) throw std::runtime_error("Failed to map SamplePlaneSweepPixel color readback.");
			const float *actualColors = static_cast<const float *>(colorMapped);
			constexpr const char *channelNames[] = { "R", "G", "B", "A" };
			for (uint32_t i = 0; i < kCaseCount && matched; ++i)
			{
				if (!expected[i].valid) continue;
				const float expectedChannels[4]{ expected[i].color.x, expected[i].color.y, expected[i].color.z, expected[i].color.w };
				for (uint32_t channel = 0; channel < 4; ++channel)
				{
					const float actual = actualColors[i * 4 + channel];
					const float error = std::fabs(expectedChannels[channel] - actual);
					maximumError = std::max(maximumError, error);
					if (error > 1e-5f)
					{
						UIUtility::Print(std::string("SamplePlaneSweepPixel color mismatch: case ") + std::to_string(i) + " x=" + std::to_string(cases[i].x) + " y=" + std::to_string(cases[i].y) + " mode hasMask=" + std::to_string(cases[i].hasMask) + " maskUsesAlpha=" + std::to_string(cases[i].maskUsesAlpha) + " channel=" + channelNames[channel] + " expected=" + std::to_string(expectedChannels[channel]) + " actual=" + std::to_string(actual) + " absolute error=" + std::to_string(error) + "\n");
						matched = false;
						break;
					}
				}
				if (matched && cases[i].hasMask != 0)
				{
					const float actualRgb[3]{ actualColors[i * 4], actualColors[i * 4 + 1], actualColors[i * 4 + 2] };
					const float sourceRgb[3]{ expected[i].imageColor.x, expected[i].imageColor.y, expected[i].imageColor.z };
					for (uint32_t channel = 0; channel < 3; ++channel)
					{
						if (std::fabs(actualRgb[channel] - sourceRgb[channel]) > 1e-5f)
						{
							UIUtility::Print(std::string("SamplePlaneSweepPixel masked RGB changed: case ") + std::to_string(i) + " channel=" + channelNames[channel] + " expected image RGB=" + std::to_string(sourceRgb[channel]) + " actual=" + std::to_string(actualRgb[channel]) + "\n");
							matched = false;
							break;
						}
					}
				}
				if (matched && cases[i].hasMask != 0)
				{
					const float *mask = &expected[i].maskColor.x;
					UIUtility::Print(std::string("SamplePlaneSweepPixel masked case ") + std::to_string(i) + " imageXY=(" + std::to_string(cases[i].x) + "," + std::to_string(cases[i].y) + ") maskXY=(" + std::to_string(expected[i].maskX) + "," + std::to_string(expected[i].maskY) + ") imageRGBA=(" + std::to_string(expected[i].imageColor.x) + "," + std::to_string(expected[i].imageColor.y) + "," + std::to_string(expected[i].imageColor.z) + "," + std::to_string(expected[i].imageColor.w) + ") maskRGBA=(" + std::to_string(mask[0]) + "," + std::to_string(mask[1]) + "," + std::to_string(mask[2]) + "," + std::to_string(mask[3]) + ") finalCPU=(" + std::to_string(expected[i].color.x) + "," + std::to_string(expected[i].color.y) + "," + std::to_string(expected[i].color.z) + "," + std::to_string(expected[i].color.w) + ") finalGPU=(" + std::to_string(actualColors[i * 4]) + "," + std::to_string(actualColors[i * 4 + 1]) + "," + std::to_string(actualColors[i * 4 + 2]) + "," + std::to_string(actualColors[i * 4 + 3]) + ")\n");
				}
			}
			vkUnmapMemory(m_device, colorReadbackMemory);
		}

		for (uint32_t i = 0; i < kCaseCount; ++i)
		{
			UIUtility::Print(std::string("SamplePlaneSweepPixel case ") + std::to_string(i) + " x=" + std::to_string(cases[i].x) + " y=" + std::to_string(cases[i].y) + " hasMask=" + std::to_string(cases[i].hasMask) + " maskUsesAlpha=" + std::to_string(cases[i].maskUsesAlpha) + " expected=" + (expected[i].valid ? "valid" : "invalid") + "\n");
		}
		if (matched)
		{
			UIUtility::Print(std::string("PlaneSweep GPU SamplePlaneSweepPixel smoke test succeeded:\n16 / 16 validity results matched CPU semantics.\n") + std::to_string(validCount) + " / " + std::to_string(validCount) + " valid RGBA results matched.\nMask alpha and luminance semantics matched.\nImage RGB remained unchanged by mask processing.\nMaximum valid RGBA component absolute error: " + std::to_string(maximumError) + "\n");
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

bool PlaneSweepGpu::RunRealImageSamplingSmokeTest(
	const TextureData &texture,
	const TextureData *maskTexturePtr,
	bool hasMask,
	bool maskUsesAlpha,
	const std::vector<float> &xyPairs,
	std::vector<uint32_t> &gpuValidity,
	std::vector<float> &gpuRgba,
	uint32_t cameraIndex)
{
	if (!m_initialized) throw std::runtime_error("PlaneSweepGpu not initialized.");
	if (xyPairs.size() != 32) throw std::invalid_argument("Real image sampling requires exactly 16 XY coordinate pairs.");
	if (hasMask && maskTexturePtr == nullptr) throw std::invalid_argument("Real image sampling mask flag requires mask texture data.");

	constexpr uint32_t kSampleCount = 16;
	constexpr VkFormat kFormat = VK_FORMAT_R8G8B8A8_UNORM;
	const TextureData dummyMask{ 1, 1, 4, std::vector<uint8_t>{ 0, 0, 0, 255 } };
	const TextureData &maskTexture = hasMask ? *maskTexturePtr : dummyMask;

	auto byteCount = [](const TextureData &data) -> VkDeviceSize
	{
		if (data.m_width == 0 || data.m_height == 0 || data.m_channelCount != 4)
			throw std::invalid_argument("Real image sampling requires non-empty RGBA texture data.");
		const uint64_t maxValue = std::numeric_limits<VkDeviceSize>::max();
		const uint64_t width = data.m_width;
		const uint64_t height = data.m_height;
		if (width > maxValue / height || width * height > maxValue / 4)
			throw std::overflow_error("Real image sampling texture byte count overflow.");
		const VkDeviceSize bytes = static_cast<VkDeviceSize>(width * height * 4);
		if (data.m_pixels.size() != static_cast<size_t>(bytes))
			throw std::invalid_argument("Real image sampling texture byte count does not match RGBA dimensions.");
		return bytes;
	};
	const VkDeviceSize imageBytes = byteCount(texture);
	const VkDeviceSize maskBytes = byteCount(maskTexture);
	const uint32_t imageWidths[2]{ texture.m_width, maskTexture.m_width };
	const uint32_t imageHeights[2]{ texture.m_height, maskTexture.m_height };
	const VkDeviceSize imageSizes[2]{ imageBytes, maskBytes };
	const TextureData *imageTextures[2]{ &texture, &maskTexture };
	constexpr VkDeviceSize coordinateBytes = kSampleCount * 2 * sizeof(float);
	constexpr VkDeviceSize modeBytes = kSampleCount * 2 * sizeof(uint32_t);
	constexpr VkDeviceSize colorBytes = kSampleCount * 4 * sizeof(float);
	constexpr VkDeviceSize validityBytes = kSampleCount * sizeof(uint32_t);
	std::array<uint32_t, kSampleCount * 2> modeValues{};
	for (uint32_t i = 0; i < kSampleCount; ++i)
	{
		modeValues[i * 2] = hasMask ? 1u : 0u;
		modeValues[i * 2 + 1] = maskUsesAlpha ? 1u : 0u;
	}

	VkFormatProperties formatProperties{};
	vkGetPhysicalDeviceFormatProperties(m_physicalDevice, kFormat, &formatProperties);
	const VkFormatFeatureFlags requiredFeatures = VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT | VK_FORMAT_FEATURE_TRANSFER_DST_BIT;
	if ((formatProperties.optimalTilingFeatures & requiredFeatures) != requiredFeatures)
	{
		UIUtility::Print("Real image sampling unsupported: VK_FORMAT_R8G8B8A8_UNORM lacks optimal sampled-image or transfer-destination support.\n");
		return false;
	}

	VkBuffer imageStaging[2]{ VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkDeviceMemory imageStagingMemory[2]{ VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkImage images[2]{ VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkDeviceMemory imageMemory[2]{ VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkImageView imageViews[2]{ VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkSampler samplers[2]{ VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkBuffer coordinateStaging = VK_NULL_HANDLE, modeStaging = VK_NULL_HANDLE;
	VkDeviceMemory coordinateStagingMemory = VK_NULL_HANDLE, modeStagingMemory = VK_NULL_HANDLE;
	VkBuffer coordinateInput = VK_NULL_HANDLE, modeInput = VK_NULL_HANDLE, colorOutput = VK_NULL_HANDLE, validOutput = VK_NULL_HANDLE;
	VkDeviceMemory coordinateInputMemory = VK_NULL_HANDLE, modeInputMemory = VK_NULL_HANDLE, colorOutputMemory = VK_NULL_HANDLE, validOutputMemory = VK_NULL_HANDLE;
	VkBuffer colorReadback = VK_NULL_HANDLE, validReadback = VK_NULL_HANDLE;
	VkDeviceMemory colorReadbackMemory = VK_NULL_HANDLE, validReadbackMemory = VK_NULL_HANDLE;
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
		for (uint32_t i = 0; i < 2; ++i)
		{
			if (samplers[i] != VK_NULL_HANDLE) { vkDestroySampler(m_device, samplers[i], nullptr); samplers[i] = VK_NULL_HANDLE; }
			if (imageViews[i] != VK_NULL_HANDLE) { vkDestroyImageView(m_device, imageViews[i], nullptr); imageViews[i] = VK_NULL_HANDLE; }
			if (images[i] != VK_NULL_HANDLE) { vkDestroyImage(m_device, images[i], nullptr); images[i] = VK_NULL_HANDLE; }
			if (imageMemory[i] != VK_NULL_HANDLE) { vkFreeMemory(m_device, imageMemory[i], nullptr); imageMemory[i] = VK_NULL_HANDLE; }
		}
		auto destroyBuffer = [&](VkBuffer &buffer, VkDeviceMemory &memory)
		{
			if (buffer != VK_NULL_HANDLE) { vkDestroyBuffer(m_device, buffer, nullptr); buffer = VK_NULL_HANDLE; }
			if (memory != VK_NULL_HANDLE) { vkFreeMemory(m_device, memory, nullptr); memory = VK_NULL_HANDLE; }
		};
		for (uint32_t i = 0; i < 2; ++i) destroyBuffer(imageStaging[i], imageStagingMemory[i]);
		destroyBuffer(coordinateStaging, coordinateStagingMemory); destroyBuffer(modeStaging, modeStagingMemory);
		destroyBuffer(coordinateInput, coordinateInputMemory); destroyBuffer(modeInput, modeInputMemory);
		destroyBuffer(colorOutput, colorOutputMemory); destroyBuffer(validOutput, validOutputMemory);
		destroyBuffer(colorReadback, colorReadbackMemory); destroyBuffer(validReadback, validReadbackMemory);
	};
	auto findMemoryType = [&](uint32_t bits, VkMemoryPropertyFlags required)
	{
		VkPhysicalDeviceMemoryProperties properties{}; vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &properties);
		for (uint32_t i = 0; i < properties.memoryTypeCount; ++i) if ((bits & (1u << i)) && (properties.memoryTypes[i].propertyFlags & required) == required) return i;
		return UINT32_MAX;
	};
	auto createBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer &buffer, VkDeviceMemory &memory)
	{
		VkBufferCreateInfo info{}; info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO; info.size = size; info.usage = usage; info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
		if (vkCreateBuffer(m_device, &info, nullptr, &buffer) != VK_SUCCESS) throw std::runtime_error("Failed to create real image sampling buffer.");
		VkMemoryRequirements requirements{}; vkGetBufferMemoryRequirements(m_device, buffer, &requirements);
		const uint32_t type = findMemoryType(requirements.memoryTypeBits, properties);
		if (type == UINT32_MAX) throw std::runtime_error("No suitable memory type for real image sampling buffer.");
		VkMemoryAllocateInfo allocation{}; allocation.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; allocation.allocationSize = requirements.size; allocation.memoryTypeIndex = type;
		if (vkAllocateMemory(m_device, &allocation, nullptr, &memory) != VK_SUCCESS) throw std::runtime_error("Failed to allocate real image sampling buffer memory.");
		if (vkBindBufferMemory(m_device, buffer, memory, 0) != VK_SUCCESS) throw std::runtime_error("Failed to bind real image sampling buffer memory.");
	};
	auto copyToMemory = [&](VkDeviceMemory memory, VkDeviceSize size, const void *data)
	{
		void *mapped = nullptr;
		if (vkMapMemory(m_device, memory, 0, size, 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map real image sampling staging memory.");
		std::memcpy(mapped, data, static_cast<size_t>(size)); vkUnmapMemory(m_device, memory);
	};

	try
	{
		const VkMemoryPropertyFlags host = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
		for (uint32_t i = 0; i < 2; ++i)
		{
			createBuffer(imageSizes[i], VK_BUFFER_USAGE_TRANSFER_SRC_BIT, host, imageStaging[i], imageStagingMemory[i]);
			copyToMemory(imageStagingMemory[i], imageSizes[i], imageTextures[i]->m_pixels.data());
			VkImageCreateInfo imageInfo{}; imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO; imageInfo.imageType = VK_IMAGE_TYPE_2D; imageInfo.format = kFormat;
			imageInfo.extent = VkExtent3D{ imageWidths[i], imageHeights[i], 1 }; imageInfo.mipLevels = 1; imageInfo.arrayLayers = 1; imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
			imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL; imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT; imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE; imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			if (vkCreateImage(m_device, &imageInfo, nullptr, &images[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create real image sampling VkImage.");
			VkMemoryRequirements requirements{}; vkGetImageMemoryRequirements(m_device, images[i], &requirements);
			const uint32_t type = findMemoryType(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
			if (type == UINT32_MAX) throw std::runtime_error("No device-local memory for real image sampling VkImage.");
			VkMemoryAllocateInfo allocation{}; allocation.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO; allocation.allocationSize = requirements.size; allocation.memoryTypeIndex = type;
			if (vkAllocateMemory(m_device, &allocation, nullptr, &imageMemory[i]) != VK_SUCCESS || vkBindImageMemory(m_device, images[i], imageMemory[i], 0) != VK_SUCCESS) throw std::runtime_error("Failed to allocate or bind real image sampling image memory.");
			VkImageViewCreateInfo view{}; view.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO; view.image = images[i]; view.viewType = VK_IMAGE_VIEW_TYPE_2D; view.format = kFormat; view.subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
			if (vkCreateImageView(m_device, &view, nullptr, &imageViews[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create real image sampling image view.");
			VkSamplerCreateInfo samplerInfo{}; samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO; samplerInfo.magFilter = VK_FILTER_NEAREST; samplerInfo.minFilter = VK_FILTER_NEAREST; samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
			samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE; samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE; samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
			samplerInfo.anisotropyEnable = VK_FALSE; samplerInfo.compareEnable = VK_FALSE; samplerInfo.minLod = 0.0f; samplerInfo.maxLod = 0.0f; samplerInfo.unnormalizedCoordinates = VK_FALSE;
			if (vkCreateSampler(m_device, &samplerInfo, nullptr, &samplers[i]) != VK_SUCCESS) throw std::runtime_error("Failed to create real image sampling sampler.");
		}
		createBuffer(coordinateBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, host, coordinateStaging, coordinateStagingMemory);
		copyToMemory(coordinateStagingMemory, coordinateBytes, xyPairs.data());
		createBuffer(modeBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, host, modeStaging, modeStagingMemory);
		copyToMemory(modeStagingMemory, modeBytes, modeValues.data());
		createBuffer(coordinateBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, coordinateInput, coordinateInputMemory);
		createBuffer(modeBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, modeInput, modeInputMemory);
		createBuffer(colorBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, colorOutput, colorOutputMemory);
		createBuffer(validityBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, validOutput, validOutputMemory);
		createBuffer(colorBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, host, colorReadback, colorReadbackMemory);
		createBuffer(validityBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, host, validReadback, validReadbackMemory);

		std::array<VkDescriptorSetLayoutBinding, 6> bindings{};
		bindings[0] = VkDescriptorSetLayoutBinding{ 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
		bindings[1] = VkDescriptorSetLayoutBinding{ 1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
		for (uint32_t i = 2; i < 6; ++i) bindings[i] = VkDescriptorSetLayoutBinding{ i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
		VkDescriptorSetLayoutCreateInfo descriptorLayout{}; descriptorLayout.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO; descriptorLayout.bindingCount = 6; descriptorLayout.pBindings = bindings.data();
		if (vkCreateDescriptorSetLayout(m_device, &descriptorLayout, nullptr, &descriptorSetLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create real image sampling descriptor layout.");
		VkPipelineLayoutCreateInfo pipelineLayoutInfo{}; pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO; pipelineLayoutInfo.setLayoutCount = 1; pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout;
		if (vkCreatePipelineLayout(m_device, &pipelineLayoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS) throw std::runtime_error("Failed to create real image sampling pipeline layout.");
		const std::vector<char> shaderCode = ReadShaderFile("shaders/planeSweepPixelSamplingSmokeTest.comp.spv");
		if (shaderCode.empty()) throw std::runtime_error("Failed to load proven SamplePlaneSweepPixel shader SPIR-V.");
		VkShaderModuleCreateInfo shaderInfo{}; shaderInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO; shaderInfo.codeSize = shaderCode.size(); shaderInfo.pCode = reinterpret_cast<const uint32_t *>(shaderCode.data());
		if (vkCreateShaderModule(m_device, &shaderInfo, nullptr, &shaderModule) != VK_SUCCESS) throw std::runtime_error("Failed to create real image sampling shader module.");
		VkPipelineShaderStageCreateInfo stage{}; stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO; stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = shaderModule; stage.pName = "main";
		VkComputePipelineCreateInfo pipelineInfo{}; pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO; pipelineInfo.stage = stage; pipelineInfo.layout = pipelineLayout;
		if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline) != VK_SUCCESS) throw std::runtime_error("Failed to create real image sampling compute pipeline.");
		std::array<VkDescriptorPoolSize, 2> poolSizes{{ { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 2 }, { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 4 } }};
		VkDescriptorPoolCreateInfo poolInfo{}; poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO; poolInfo.maxSets = 1; poolInfo.poolSizeCount = 2; poolInfo.pPoolSizes = poolSizes.data();
		if (vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS) throw std::runtime_error("Failed to create real image sampling descriptor pool.");
		VkDescriptorSetAllocateInfo descriptorAllocate{}; descriptorAllocate.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO; descriptorAllocate.descriptorPool = descriptorPool; descriptorAllocate.descriptorSetCount = 1; descriptorAllocate.pSetLayouts = &descriptorSetLayout;
		VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
		if (vkAllocateDescriptorSets(m_device, &descriptorAllocate, &descriptorSet) != VK_SUCCESS) throw std::runtime_error("Failed to allocate real image sampling descriptor set.");
		VkDescriptorImageInfo imageInfos[2]{};
		for (uint32_t i = 0; i < 2; ++i) { imageInfos[i].sampler = samplers[i]; imageInfos[i].imageView = imageViews[i]; imageInfos[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; }
		VkDescriptorBufferInfo bufferInfos[4]{{ coordinateInput, 0, coordinateBytes }, { modeInput, 0, modeBytes }, { colorOutput, 0, colorBytes }, { validOutput, 0, validityBytes }};
		std::array<VkWriteDescriptorSet, 6> writes{};
		for (uint32_t i = 0; i < 2; ++i) { writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; writes[i].dstSet = descriptorSet; writes[i].dstBinding = i; writes[i].descriptorCount = 1; writes[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER; writes[i].pImageInfo = &imageInfos[i]; }
		for (uint32_t i = 2; i < 6; ++i) { writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; writes[i].dstSet = descriptorSet; writes[i].dstBinding = i; writes[i].descriptorCount = 1; writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; writes[i].pBufferInfo = &bufferInfos[i - 2]; }
		vkUpdateDescriptorSets(m_device, 6, writes.data(), 0, nullptr);

		VkCommandBufferAllocateInfo commandAllocate{}; commandAllocate.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO; commandAllocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; commandAllocate.commandPool = m_commandPool; commandAllocate.commandBufferCount = 1;
		if (vkAllocateCommandBuffers(m_device, &commandAllocate, &commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to allocate real image sampling command buffer.");
		VkCommandBufferBeginInfo begin{}; begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO; begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
		if (vkBeginCommandBuffer(commandBuffer, &begin) != VK_SUCCESS) throw std::runtime_error("Failed to begin real image sampling command buffer.");
		for (uint32_t i = 0; i < 2; ++i)
		{
			VkImageMemoryBarrier toTransfer{}; toTransfer.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; toTransfer.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; toTransfer.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED; toTransfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
			toTransfer.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toTransfer.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toTransfer.image = images[i]; toTransfer.subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
			vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &toTransfer);
			VkBufferImageCopy copy{}; copy.imageSubresource = VkImageSubresourceLayers{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 }; copy.imageExtent = VkExtent3D{ imageWidths[i], imageHeights[i], 1 };
			vkCmdCopyBufferToImage(commandBuffer, imageStaging[i], images[i], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
			VkImageMemoryBarrier toRead{}; toRead.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER; toRead.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; toRead.dstAccessMask = VK_ACCESS_SHADER_READ_BIT; toRead.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; toRead.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
			toRead.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toRead.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; toRead.image = images[i]; toRead.subresourceRange = VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
			vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &toRead);
		}
		VkBufferCopy coordCopy{}; coordCopy.size = coordinateBytes; vkCmdCopyBuffer(commandBuffer, coordinateStaging, coordinateInput, 1, &coordCopy);
		VkBufferCopy modeCopy{}; modeCopy.size = modeBytes; vkCmdCopyBuffer(commandBuffer, modeStaging, modeInput, 1, &modeCopy);
		VkBufferMemoryBarrier inputs[2]{};
		for (uint32_t i = 0; i < 2; ++i) { inputs[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; inputs[i].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; inputs[i].dstAccessMask = VK_ACCESS_SHADER_READ_BIT; inputs[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; inputs[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; }
		inputs[0].buffer = coordinateInput; inputs[0].size = coordinateBytes; inputs[1].buffer = modeInput; inputs[1].size = modeBytes;
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 2, inputs, 0, nullptr);
		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
		vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
		vkCmdDispatch(commandBuffer, 1, 1, 1);
		VkBufferMemoryBarrier outputs[2]{};
		for (uint32_t i = 0; i < 2; ++i) { outputs[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER; outputs[i].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT; outputs[i].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT; outputs[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; outputs[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED; }
		outputs[0].buffer = colorOutput; outputs[0].size = colorBytes; outputs[1].buffer = validOutput; outputs[1].size = validityBytes;
		vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 2, outputs, 0, nullptr);
		VkBufferCopy colorCopy{}; colorCopy.size = colorBytes; vkCmdCopyBuffer(commandBuffer, colorOutput, colorReadback, 1, &colorCopy);
		VkBufferCopy validityCopy{}; validityCopy.size = validityBytes; vkCmdCopyBuffer(commandBuffer, validOutput, validReadback, 1, &validityCopy);
		if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) throw std::runtime_error("Failed to record real image sampling command buffer.");
		VkFenceCreateInfo fenceInfo{}; fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
		if (vkCreateFence(m_device, &fenceInfo, nullptr, &fence) != VK_SUCCESS) throw std::runtime_error("Failed to create real image sampling fence.");
		VkSubmitInfo submit{}; submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO; submit.commandBufferCount = 1; submit.pCommandBuffers = &commandBuffer;
		if (vkQueueSubmit(m_queue, 1, &submit, fence) != VK_SUCCESS) throw std::runtime_error("Failed to submit real image sampling command buffer.");
		submitted = true;
		if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS) throw std::runtime_error("Failed to wait for real image sampling fence.");
		submitted = false;
		vkDestroyFence(m_device, fence, nullptr); fence = VK_NULL_HANDLE;

		gpuValidity.resize(kSampleCount);
		gpuRgba.resize(kSampleCount * 4);
		void *mapped = nullptr;
		if (vkMapMemory(m_device, validReadbackMemory, 0, validityBytes, 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map real image validity readback.");
		std::memcpy(gpuValidity.data(), mapped, static_cast<size_t>(validityBytes)); vkUnmapMemory(m_device, validReadbackMemory);
		if (vkMapMemory(m_device, colorReadbackMemory, 0, colorBytes, 0, &mapped) != VK_SUCCESS) throw std::runtime_error("Failed to map real image color readback.");
		std::memcpy(gpuRgba.data(), mapped, static_cast<size_t>(colorBytes)); vkUnmapMemory(m_device, colorReadbackMemory);
		cleanup();
		return true;
	}
	catch (const std::exception &ex)
	{
		cleanup();
		UIUtility::Print(std::string("PlaneSweep GPU real-image sampling failed for camera/image index ") + std::to_string(cameraIndex) + ": " + ex.what() + "\n");
		return false;
	}
}

}
