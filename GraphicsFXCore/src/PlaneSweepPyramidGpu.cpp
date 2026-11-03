#include "PlaneSweepGpu.h"
#include "PlaneSweepGpuTypes.h"
#include "UIUtility.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace graphicsFX
{
    namespace
    {
        void Check(VkResult result, const char *operation)
        {
            if (result != VK_SUCCESS)
                throw std::runtime_error(std::string(operation) + ": VkResult=" + std::to_string(result));
        }

        // Owns all resources across all levels, including cleanup after partial allocation.
        struct Resources
        {
            VkDevice device;
            VkCommandPool commandPool;
            VkPhysicalDeviceMemoryProperties memoryProperties{};
            std::vector<VkBuffer> buffers;
            std::vector<VkDeviceMemory> memory;
            std::vector<VkImage> images;
            std::vector<VkImageView> views;
            VkSampler sampler{};
            VkDescriptorSetLayout layout{};
            VkDescriptorPool pool{};
            VkPipelineLayout pipelineLayout{};
            VkShaderModule shader{};
            VkPipeline pipeline{};
            VkCommandBuffer command{};
            VkFence fence{};
            bool submitted{ false };
            bool persistentPipeline{ false };

            Resources(VkDevice d, VkPhysicalDevice physical, VkCommandPool cp) : device(d), commandPool(cp)
            { vkGetPhysicalDeviceMemoryProperties(physical, &memoryProperties); }
            ~Resources()
            {
                if (submitted) vkDeviceWaitIdle(device);
                if (fence) vkDestroyFence(device, fence, nullptr);
                if (command) vkFreeCommandBuffers(device, commandPool, 1, &command);
                if (pipeline && !persistentPipeline) vkDestroyPipeline(device, pipeline, nullptr);
                if (shader) vkDestroyShaderModule(device, shader, nullptr);
                if (pipelineLayout && !persistentPipeline) vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
                if (pool) vkDestroyDescriptorPool(device, pool, nullptr);
                if (layout && !persistentPipeline) vkDestroyDescriptorSetLayout(device, layout, nullptr);
                if (sampler) vkDestroySampler(device, sampler, nullptr);
                for (auto v : views) vkDestroyImageView(device, v, nullptr);
                for (auto i : images) vkDestroyImage(device, i, nullptr);
                for (auto b : buffers) vkDestroyBuffer(device, b, nullptr);
                for (auto m : memory) vkFreeMemory(device, m, nullptr);
            }
            uint32_t MemoryType(uint32_t bits, VkMemoryPropertyFlags flags) const
            {
                for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; ++i)
                    if ((bits & (1u << i)) && (memoryProperties.memoryTypes[i].propertyFlags & flags) == flags) return i;
                throw std::runtime_error("No compatible pyramid memory type.");
            }
            VkDeviceMemory Allocate(const VkMemoryRequirements &requirements, VkMemoryPropertyFlags flags)
            {
                VkMemoryAllocateInfo info{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
                info.allocationSize = requirements.size; info.memoryTypeIndex = MemoryType(requirements.memoryTypeBits, flags);
                VkDeviceMemory result{};
                Check(vkAllocateMemory(device, &info, nullptr, &result), "Allocate pyramid memory");
                memory.push_back(result); return result;
            }
            struct Buffer { VkBuffer handle; VkDeviceMemory memory; VkDeviceSize bytes; };
            Buffer MakeBuffer(VkDeviceSize bytes, VkBufferUsageFlags usage, bool host, const void *data = nullptr)
            {
                VkBufferCreateInfo info{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
                info.size = std::max<VkDeviceSize>(bytes, 16); info.usage = usage;
                VkBuffer buffer{}; Check(vkCreateBuffer(device, &info, nullptr, &buffer), "Create pyramid buffer");
                buffers.push_back(buffer);
                VkMemoryRequirements requirements{}; vkGetBufferMemoryRequirements(device, buffer, &requirements);
                VkDeviceMemory allocation = Allocate(requirements, host ? VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT : VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
                Check(vkBindBufferMemory(device, buffer, allocation, 0), "Bind pyramid buffer");
                if (data) { void *mapped{}; Check(vkMapMemory(device, allocation, 0, bytes, 0, &mapped), "Map pyramid input"); std::memcpy(mapped, data, size_t(bytes)); vkUnmapMemory(device, allocation); }
                return { buffer, allocation, info.size };
            }
            void Read(const Buffer &buffer, void *destination, size_t bytes)
            {
                void *mapped{}; Check(vkMapMemory(device, buffer.memory, 0, bytes, 0, &mapped), "Read pyramid result");
                std::memcpy(destination, mapped, bytes); vkUnmapMemory(device, buffer.memory);
            }
            VkDescriptorImageInfo Upload(const TextureData &texture)
            {
                auto staging = MakeBuffer(texture.m_pixels.size(), VK_BUFFER_USAGE_TRANSFER_SRC_BIT, true, texture.m_pixels.data());
                VkImageCreateInfo info{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
                info.imageType = VK_IMAGE_TYPE_2D; info.format = VK_FORMAT_R8G8B8A8_UNORM;
                info.extent = { texture.m_width, texture.m_height, 1 }; info.mipLevels = 1; info.arrayLayers = 1;
                info.samples = VK_SAMPLE_COUNT_1_BIT; info.tiling = VK_IMAGE_TILING_OPTIMAL;
                info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
                VkImage image{}; Check(vkCreateImage(device, &info, nullptr, &image), "Create pyramid image"); images.push_back(image);
                VkMemoryRequirements requirements{}; vkGetImageMemoryRequirements(device, image, &requirements);
                auto allocation = Allocate(requirements, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
                Check(vkBindImageMemory(device, image, allocation, 0), "Bind pyramid image");
                VkImageMemoryBarrier barrier{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
                barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                barrier.srcQueueFamilyIndex = barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                barrier.image = image; barrier.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
                vkCmdPipelineBarrier(command, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);
                VkBufferImageCopy copy{}; copy.imageSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 }; copy.imageExtent = info.extent;
                vkCmdCopyBufferToImage(command, staging.handle, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
                barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT; barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
                barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                vkCmdPipelineBarrier(command, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);
                VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
                viewInfo.image = image; viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D; viewInfo.format = info.format;
                viewInfo.subresourceRange = barrier.subresourceRange;
                VkImageView view{}; Check(vkCreateImageView(device, &viewInfo, nullptr, &view), "Create pyramid image view"); views.push_back(view);
                return { sampler, view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            }
            void Barrier(const Buffer &buffer, VkAccessFlags source, VkAccessFlags destination,
                VkPipelineStageFlags sourceStage, VkPipelineStageFlags destinationStage)
            {
                VkBufferMemoryBarrier barrier{ VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER };
                barrier.srcAccessMask = source; barrier.dstAccessMask = destination;
                barrier.srcQueueFamilyIndex = barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                barrier.buffer = buffer.handle; barrier.size = buffer.bytes;
                vkCmdPipelineBarrier(command, sourceStage, destinationStage, 0, 0, nullptr, 1, &barrier, 0, nullptr);
            }
            Buffer CopyForReadback(const Buffer &buffer)
            {
                Buffer result = MakeBuffer(buffer.bytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT, true);
                Barrier(buffer, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);
                VkBufferCopy copy{ 0, 0, buffer.bytes }; vkCmdCopyBuffer(command, buffer.handle, result.handle, 1, &copy);
                Barrier(result, VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_HOST_READ_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT);
                return result;
            }
        };
    }

    bool PlaneSweepGpu::RunCoarseToFineDepthMap(const SweepUtility::PlaneSweepImage &reference,
        const std::vector<SweepUtility::PlaneSweepImage> &sources, const ColmapSetup &setup,
        const SweepUtility::PlaneSweepDepthRange &range, SweepUtility::PlaneSweepDepthMap &depthMap,
        std::vector<PlaneSweepPyramidStats> *statistics, std::string *failureReason, PlaneSweepSlantedDiagnostics *diagnostics)
    {
        const auto start = std::chrono::steady_clock::now();
        if (statistics) statistics->clear();
        if (failureReason) failureReason->clear();
        depthMap = {};
        if (diagnostics) *diagnostics = {};
        try
        {
            const bool slanted = setup.m_planeSweepMatchingMode == PlaneSweepMatchingMode::PatchMatchSlanted;
            if (slanted) ValidatePlaneSweepSlantedSetup(setup);
            if (!m_initialized || sources.empty() || sources.size() > MAX_PLANE_SWEEP_SOURCE_IMAGES)
                throw std::runtime_error("Pyramid GPU requires an initialized device and 1..8 sources.");
            const bool correlation = setup.m_planeSweepPatchCostMethod == PatchCostMethod::PatchCostMethodNormalizedCrossCorrelation ||
                setup.m_planeSweepPatchCostMethod == PatchCostMethod::PatchCostMethodZeroMeanNormalizedCrossCorrelation;
            if (correlation && !m_shaderFloat64Enabled) throw std::runtime_error("Pyramid NCC/ZNCC requires shaderFloat64.");
            if (!(range.m_nearDepth > 0 && range.m_farDepth > range.m_nearDepth) || setup.m_planeSweepPlaneCount < 2)
                throw std::invalid_argument("Invalid pyramid depth range or plane count.");
            auto pyramid = BuildPlaneSweepPyramid(reference, sources, setup);
            const uint32_t count = uint32_t(pyramid.size());
            Resources resources(m_device, m_physicalDevice, m_commandPool);
            VkPhysicalDeviceProperties properties{}; vkGetPhysicalDeviceProperties(m_physicalDevice, &properties);
            const uint32_t storageCount = slanted ? 9u : 7u;
            const uint32_t bindingCount = 4u + storageCount;
            if (properties.limits.maxPerStageDescriptorSamplers < 18 || properties.limits.maxPerStageDescriptorSampledImages < 18 ||
                properties.limits.maxPerStageDescriptorStorageBuffers < storageCount || properties.limits.maxPerStageResources < 18u + storageCount)
                throw std::runtime_error("Insufficient pyramid descriptor limits.");
            std::array<VkDescriptorSetLayoutBinding, 13> bindings{};
            for (uint32_t i = 0; i < bindings.size(); ++i)
                bindings[i] = { i, i < 4 ? VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                    i == 2 || i == 3 ? 8u : 1u, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
            VkDescriptorSetLayoutCreateInfo layout{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
            layout.bindingCount = bindingCount; layout.pBindings = bindings.data();
            const int variant = (slanted ? 2 : 0) + (correlation ? 1 : 0);
            if (m_pyramidPipelines[variant])
            {
                resources.layout = m_pyramidSetLayouts[variant];
                resources.pipelineLayout = m_pyramidPipelineLayouts[variant];
                resources.pipeline = m_pyramidPipelines[variant];
                resources.persistentPipeline = true;
            }
            else Check(vkCreateDescriptorSetLayout(m_device, &layout, nullptr, &resources.layout), "Create pyramid descriptor layout");
            const uint32_t setCount = count * (slanted ? uint32_t(setup.m_planeSweepPatchMatchIterations) + 3u : 1u);
            const std::array<VkDescriptorPoolSize, 2> sizes{{ { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 18 * setCount }, { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, storageCount * setCount } }};
            VkDescriptorPoolCreateInfo pool{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
            pool.maxSets = setCount; pool.poolSizeCount = uint32_t(sizes.size()); pool.pPoolSizes = sizes.data();
            Check(vkCreateDescriptorPool(m_device, &pool, nullptr, &resources.pool), "Create pyramid descriptor pool");
            struct LevelPush
            {
                uint32_t parentWidth, parentHeight, finalLevel, localSamples;
                uint32_t pass, iterations; float colorSigma, spatialSigma;
                float costThreshold, confidenceMinimum, textureMinimum; uint32_t flags;
            };
            static_assert(sizeof(LevelPush) == 48, "Pyramid push constant layout must match GLSL.");
            VkPushConstantRange push{ VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(LevelPush) };
            VkPipelineLayoutCreateInfo pipelineLayout{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
            pipelineLayout.setLayoutCount = 1; pipelineLayout.pSetLayouts = &resources.layout;
            pipelineLayout.pushConstantRangeCount = 1; pipelineLayout.pPushConstantRanges = &push;
            if (!resources.persistentPipeline)
            {
                Check(vkCreatePipelineLayout(m_device, &pipelineLayout, nullptr, &resources.pipelineLayout), "Create pyramid pipeline layout");
                const auto code = ReadShaderFile(slanted ?
                    (correlation ? "shaders/planeSweepPatchMatchFp64.comp.spv" : "shaders/planeSweepPatchMatch.comp.spv") :
                    (correlation ? "shaders/planeSweepPyramidFp64.comp.spv" : "shaders/planeSweepPyramid.comp.spv"));
                if (code.empty()) throw std::runtime_error("Pyramid SPIR-V is missing; compile the shaders.");
                VkShaderModuleCreateInfo module{ VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
                module.codeSize = code.size(); module.pCode = reinterpret_cast<const uint32_t *>(code.data());
                Check(vkCreateShaderModule(m_device, &module, nullptr, &resources.shader), "Create pyramid shader");
                VkComputePipelineCreateInfo pipeline{ VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO };
                pipeline.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                pipeline.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; pipeline.stage.module = resources.shader; pipeline.stage.pName = "main";
                pipeline.layout = resources.pipelineLayout;
                Check(vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipeline, nullptr, &resources.pipeline), "Create pyramid pipeline");
                m_pyramidSetLayouts[variant] = resources.layout;
                m_pyramidPipelineLayouts[variant] = resources.pipelineLayout;
                m_pyramidPipelines[variant] = resources.pipeline;
                resources.persistentPipeline = true;
            }
            VkSamplerCreateInfo sampler{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
            sampler.magFilter = sampler.minFilter = VK_FILTER_NEAREST;
            sampler.addressModeU = sampler.addressModeV = sampler.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
            Check(vkCreateSampler(m_device, &sampler, nullptr, &resources.sampler), "Create pyramid sampler");
            VkCommandBufferAllocateInfo command{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
            command.commandPool = m_commandPool; command.commandBufferCount = 1; command.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
            Check(vkAllocateCommandBuffers(m_device, &command, &resources.command), "Allocate pyramid commands");
            VkCommandBufferBeginInfo begin{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO }; begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
            Check(vkBeginCommandBuffer(resources.command, &begin), "Begin pyramid commands");
            std::vector<Resources::Buffer> stateReadbacks, counterReadbacks;
            Resources::Buffer planeReadback{};
            const PlaneSweepPyramidState empty{};
            auto parent = resources.MakeBuffer(sizeof(empty), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true, empty.data());
            auto parentPlanes = parent;
            TextureData dummy; dummy.m_width = dummy.m_height = 1; dummy.m_pixels = { 0, 0, 0, 255 };
            const auto dummyImage = resources.Upload(dummy);
            std::vector<PlaneSweepPyramidStats> stats(count);
            for (uint32_t l = 0; l < count; ++l)
            {
                const auto &images = pyramid[l];
                const uint32_t width = images.m_reference.m_texture.m_width, height = images.m_reference.m_texture.m_height;
                const VkDeviceSize entries = VkDeviceSize(width) * height;
                if (entries * 16 > properties.limits.maxStorageBufferRange || (width + 7) / 8 > properties.limits.maxComputeWorkGroupCount[0] ||
                    (height + 7) / 8 > properties.limits.maxComputeWorkGroupCount[1]) throw std::runtime_error("Pyramid level exceeds GPU limits.");
                stats[l].m_width = width; stats[l].m_height = height;
                std::array<VkDescriptorImageInfo, 18> imageInfo;
                imageInfo.fill(dummyImage); imageInfo[0] = resources.Upload(images.m_reference.m_texture);
                std::vector<GpuCameraIntrinsics> intrinsics{ MakeGpuCameraIntrinsics(images.m_reference.m_intrinsic) };
                std::vector<GpuCameraPose> poses{ MakeGpuCameraPose(images.m_reference.m_camera) };
                for (uint32_t i = 0; i < sources.size(); ++i)
                {
                    imageInfo[2 + i] = resources.Upload(images.m_sources[i].m_texture);
                    intrinsics.push_back(MakeGpuCameraIntrinsics(images.m_sources[i].m_intrinsic));
                    poses.push_back(MakeGpuCameraPose(images.m_sources[i].m_camera));
                }
                std::array<uint32_t, 18> metadata{}; // Masks were folded into pyramid image alpha.
                PlaneSweepGpuParams params{};
                params.m_inverseNear = 1.0f / range.m_nearDepth; params.m_inverseFar = 1.0f / range.m_farDepth;
                params.m_planeCount = setup.m_planeSweepPlaneCount; params.m_patchRadius = setup.m_planeSweepPatchRadius;
                params.m_minValidSourceViews = std::max(1, std::min(int(sources.size()), setup.m_planeSweepMinValidSourceViews));
                params.m_alphaThreshold = setup.m_planeSweepAlphaThreshold; params.m_minimumOpaquePatchRatio = setup.m_planeSweepMinimumOpaquePatchRatio;
                params.m_costThreshold = setup.m_planeSweepCostThreshold; params.m_confidenceMinimumDifference = setup.m_planeSweepConfidenceMinimumDifference;
                params.m_textureMinimumStdDev = setup.m_planeSweepTextureMinimumStdDev;
                params.m_sourceCount = uint32_t(sources.size()); params.m_textureRejectionEnabled = setup.m_planeSweepTextureRejectionEnabled;
                params.m_confidenceEnabled = setup.m_planeSweepConfidenceEnabled; params.m_pointStep = 1;
                params.m_scaledWidth = width; params.m_scaledHeight = height;
                params.m_patchCostMethod = uint32_t(setup.m_planeSweepPatchCostMethod); params.m_adCensusWeight = setup.m_planeSweepADCensusWeight;
                if (slanted)
                {
                    params.m_confidenceEnabled = params.m_textureRejectionEnabled = 0;
                    params.m_costThreshold = 10.0f;
                }
                std::array<Resources::Buffer, 9> buffer{{
                    resources.MakeBuffer(intrinsics.size() * sizeof(GpuCameraIntrinsics), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true, intrinsics.data()),
                    resources.MakeBuffer(poses.size() * sizeof(GpuCameraPose), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true, poses.data()),
                    resources.MakeBuffer(sizeof(params), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true, &params),
                    resources.MakeBuffer(sizeof(metadata), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true, metadata.data()),
                    resources.MakeBuffer(entries * 16, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, false),
                    parent,
                    resources.MakeBuffer(entries * 16, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, false),
                    parent, parent
                }};
                const auto makeSet = [&]()
                {
                VkDescriptorSetAllocateInfo allocation{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
                allocation.descriptorPool = resources.pool; allocation.descriptorSetCount = 1; allocation.pSetLayouts = &resources.layout;
                VkDescriptorSet set{}; Check(vkAllocateDescriptorSets(m_device, &allocation, &set), "Allocate pyramid descriptors");
                std::array<VkDescriptorBufferInfo, 9> bufferInfo{};
                for (uint32_t i = 0; i < 9; ++i) bufferInfo[i] = { buffer[i].handle, 0, buffer[i].bytes };
                std::array<VkWriteDescriptorSet, 13> writes{};
                for (uint32_t i = 0; i < bindingCount; ++i)
                {
                    writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; writes[i].dstSet = set; writes[i].dstBinding = i;
                    writes[i].descriptorCount = bindings[i].descriptorCount; writes[i].descriptorType = bindings[i].descriptorType;
                    if (i < 4) writes[i].pImageInfo = &imageInfo[i < 2 ? i : (i == 2 ? 2 : 10)];
                    else writes[i].pBufferInfo = &bufferInfo[i - 4];
                }
                vkUpdateDescriptorSets(m_device, bindingCount, writes.data(), 0, nullptr);
                return set;
                };
                VkDescriptorSet set = makeSet();
                LevelPush level{ l ? stats[l - 1].m_width : 0, l ? stats[l - 1].m_height : 0,
                    l + 1 == count ? 1u : 0u, uint32_t(setup.m_planeSweepLocalSamples),
                    0, uint32_t(setup.m_planeSweepPatchMatchIterations), setup.m_planeSweepPatchMatchColorSigma,
                    setup.m_planeSweepPatchMatchSpatialSigma, setup.m_planeSweepCostThreshold,
                    setup.m_planeSweepConfidenceMinimumDifference, setup.m_planeSweepTextureMinimumStdDev,
                    (setup.m_planeSweepTextureRejectionEnabled ? 1u : 0u) | (setup.m_planeSweepConfidenceEnabled ? 2u : 0u) |
                    (setup.m_planeSweepPatchMatchTiltEnabled ? 4u : 0u) };
                if (l) resources.Barrier(parent, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
                vkCmdBindPipeline(resources.command, VK_PIPELINE_BIND_POINT_COMPUTE, resources.pipeline);
                vkCmdBindDescriptorSets(resources.command, VK_PIPELINE_BIND_POINT_COMPUTE, resources.pipelineLayout, 0, 1, &set, 0, nullptr);
                vkCmdPushConstants(resources.command, resources.pipelineLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(level), &level);
                vkCmdDispatch(resources.command, (width + 7) / 8, (height + 7) / 8, 1);
                if (slanted)
                {
                    auto planeA = resources.MakeBuffer(entries * 16, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, false);
                    auto planeB = resources.MakeBuffer(entries * 16, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, false);
                    // Initialization reuses binding 9 for the preceding level's world-space planes.
                    buffer[5] = parentPlanes;
                    if (l) resources.Barrier(parentPlanes, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
                    buffer[7] = buffer[4]; buffer[8] = planeA;
                    // Separate descriptor sets preserve each dispatch's bindings until submission.
                    for (uint32_t pass = 1; pass <= level.iterations + 2u; ++pass)
                    {
                        resources.Barrier(buffer[6], VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
                            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
                        resources.Barrier(buffer[7], VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
                        resources.Barrier(buffer[8], VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_WRITE_BIT,
                            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
                        if (pass == level.iterations + 2u)
                        {
                            if (diagnostics && l + 1 == count) planeReadback = resources.CopyForReadback(buffer[7]);
                            resources.Barrier(buffer[4], VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_WRITE_BIT,
                                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
                            resources.Barrier(buffer[6], VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
                                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
                        }
                        level.pass = pass;
                        set = makeSet();
                        vkCmdBindDescriptorSets(resources.command, VK_PIPELINE_BIND_POINT_COMPUTE, resources.pipelineLayout, 0, 1, &set, 0, nullptr);
                        vkCmdPushConstants(resources.command, resources.pipelineLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(level), &level);
                        vkCmdDispatch(resources.command, (width + 7) / 8, (height + 7) / 8, 1);
                        // Finalization writes depth state, leaving the winning planes available to the next level.
                        if (pass <= level.iterations + 1u)
                        {
                            buffer[7] = buffer[8];
                            buffer[8] = buffer[7].handle == planeA.handle ? planeB : planeA;
                        }
                    }
                    parentPlanes = buffer[7];
                }
                parent = buffer[4];
                counterReadbacks.push_back(resources.CopyForReadback(buffer[6]));
                stateReadbacks.push_back(l + 1 == count || setup.m_planeSweepSavePyramidDepth ? resources.CopyForReadback(buffer[4]) : Resources::Buffer{});
            }
            Check(vkEndCommandBuffer(resources.command), "End pyramid commands");
            VkFenceCreateInfo fence{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
            Check(vkCreateFence(m_device, &fence, nullptr, &resources.fence), "Create pyramid fence");
            VkSubmitInfo submit{ VK_STRUCTURE_TYPE_SUBMIT_INFO }; submit.commandBufferCount = 1; submit.pCommandBuffers = &resources.command;
            Check(vkQueueSubmit(m_queue, 1, &submit, resources.fence), "Submit pyramid"); resources.submitted = true;
            Check(vkWaitForFences(m_device, 1, &resources.fence, VK_TRUE, UINT64_MAX), "Wait for pyramid"); resources.submitted = false;
            for (uint32_t l = 0; l < count; ++l)
            {
                const size_t entries = size_t(stats[l].m_width) * stats[l].m_height;
                std::vector<std::array<uint32_t, 4>> counters(entries);
                resources.Read(counterReadbacks[l], counters.data(), counters.size() * sizeof(counters[0]));
                for (const auto &c : counters)
                {
                    stats[l].m_evaluations += c[0]; stats[l].m_widened += c[1]; stats[l].m_globalFallbacks += c[2];
                    stats[l].m_accepted += c[3] == 3u ? 1u : 0u;
                    stats[l].m_confidenceRejected += c[3] == 2u ? 1u : 0u;
                    stats[l].m_textureRejected += c[3] == 5u ? 1u : 0u;
                }
                if (stateReadbacks[l].handle)
                {
                    std::vector<PlaneSweepPyramidState> state(entries);
                    resources.Read(stateReadbacks[l], state.data(), state.size() * sizeof(state[0]));
                    if (diagnostics && l + 1 == count)
                    {
                        diagnostics->m_costs.reserve(entries);
                        for (const auto &pixel : state) diagnostics->m_costs.push_back(pixel[1]);
                        if (planeReadback.handle)
                        {
                            diagnostics->m_planes.resize(entries);
                            resources.Read(planeReadback, diagnostics->m_planes.data(), entries * sizeof(Float4));
                        }
                    }
                    auto map = MakePlaneSweepPyramidDepthMap(reference, stats[l].m_width, stats[l].m_height, state);
                    if (setup.m_planeSweepSavePyramidDepth)
                    {
                        ColmapSetup output = setup; output.m_planeSweepSaveDepth = true;
                        output.m_planeSweepOutputFolder += "/Pyramid/Level" + std::to_string(l);
                        SweepUtility::SaveDepthOutput(output, reference.m_camera, map.m_depthValues, map.m_width, map.m_height);
                    }
                    if (l + 1 == count) depthMap = std::move(map);
                }
                UIUtility::Print("GPU pyramid level " + std::to_string(l) + " " + std::to_string(stats[l].m_width) + "x" +
                    std::to_string(stats[l].m_height) + ": candidates=" + std::to_string(stats[l].m_evaluations) +
                    ", widened=" + std::to_string(stats[l].m_widened) + ", global recovery=" + std::to_string(stats[l].m_globalFallbacks) + "\n");
            }
            const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            UIUtility::Print("GPU pyramid total including image preparation, upload and readback: " + std::to_string(elapsed) + " ms\n");
            stats.back().m_milliseconds = elapsed;
            if (statistics) *statistics = std::move(stats);
            return true;
        }
        catch (const std::exception &error)
        {
            if (failureReason) *failureReason = error.what();
            UIUtility::Print(std::string("GPU coarse-to-fine failed: ") + error.what() + "\n");
            return false;
        }
    }
}
