///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	SceneObject.cpp
//	Description	:
//		Implements shared scene object GPU resource behavior and line objects.
//
//
///////////////////////////////////////////////////////////////////////

#include "SceneObject.h"

#include "PathUtility.h"
#include "StringUtility.h"

#include <Windows.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace graphicsFX
{
    //Stores 16-bit mesh and object settings in the base object.
    void SceneObject::SetMesh(
        std::vector<Vertex> vertices,
        std::vector<uint16_t> indices,
        VkPrimitiveTopology topology,
        bool depthWrite,
        float lineWidth,
        const Float3 &position,
        const Float4x4 &orientation,
        float scale,
        const Float4 &color,
        SceneObjectRenderOptions renderOptions
    )
    {
        if (indices.size() > std::numeric_limits<uint32_t>::max())
        {
            throw std::runtime_error("Scene object index count is too large.");
        }

        m_vertices = std::move(vertices);
        m_indices16 = std::move(indices);
        m_indices32.clear();
        m_indexType = VK_INDEX_TYPE_UINT16;
        m_indexCount = static_cast<uint32_t>(m_indices16.size());

        m_topology = topology;
        m_depthWrite = depthWrite;
        m_lineWidth = lineWidth;
        m_position = position;
        m_orientation = orientation;
        m_scale = scale;
        m_color = color;
        m_renderOptions = renderOptions;
        NormalizeRenderOptions();
        UpdateSelectionBounds();
    }

    //Stores 32-bit mesh and object settings in the base object.
    void SceneObject::SetMesh(
        std::vector<Vertex> vertices,
        std::vector<uint32_t> indices,
        VkPrimitiveTopology topology,
        bool depthWrite,
        float lineWidth,
        const Float3 &position,
        const Float4x4 &orientation,
        float scale,
        const Float4 &color,
        SceneObjectRenderOptions renderOptions
    )
    {
        if (indices.size() > std::numeric_limits<uint32_t>::max())
        {
            throw std::runtime_error("Scene object index count is too large.");
        }

        m_vertices = std::move(vertices);
        m_indices16.clear();
        m_indices32 = std::move(indices);
        m_indexType = VK_INDEX_TYPE_UINT32;
        m_indexCount = static_cast<uint32_t>(m_indices32.size());

        m_topology = topology;
        m_depthWrite = depthWrite;
        m_lineWidth = lineWidth;
        m_position = position;
        m_orientation = orientation;
        m_scale = scale;
        m_color = color;
        m_renderOptions = renderOptions;
        NormalizeRenderOptions();
        UpdateSelectionBounds();
    }

    //Stores per-instance transform and color values.
    void SceneObject::SetInstances(std::vector<SceneObjectInstanceData> instances)
    {
        if (instances.size() > std::numeric_limits<uint32_t>::max())
        {
            throw std::runtime_error("Scene object instance count is too large.");
        }

        m_instances = std::move(instances);
        m_instanceCount = static_cast<uint32_t>(m_instances.size());
        m_instancingEnabled = true;
    }

    //Updates 32-bit mesh indices in the base object.
    void SceneObject::UpdateIndices(std::vector<uint32_t> indices)
    {
        if (indices.empty())
        {
            throw std::runtime_error("Scene object index update must contain at least one index.");
        }

        if (indices.size() > std::numeric_limits<uint32_t>::max())
        {
            throw std::runtime_error("Scene object index update count is too large.");
        }

        const bool rebuildIndexBuffer = m_indexType != VK_INDEX_TYPE_UINT32 || m_indices32.size() != indices.size();
        m_indices16.clear();
        m_indices32 = std::move(indices);
        m_indexType = VK_INDEX_TYPE_UINT32;
        m_indexCount = static_cast<uint32_t>(m_indices32.size());

        if (m_context.m_devicePtrGPU == VK_NULL_HANDLE || m_indexBufferPtrGPU == VK_NULL_HANDLE)
        {
            return;
        }

        if (rebuildIndexBuffer)
        {
            RetireBuffer(m_indexBufferPtrGPU, m_indexBufferMemoryPtrGPU);
            CreateIndexBuffer();
            return;
        }

        const VkDeviceSize bufferSize = static_cast<VkDeviceSize>(sizeof(m_indices32.front())) * static_cast<VkDeviceSize>(m_indices32.size());
        VkBuffer stagingBufferPtrGPU = VK_NULL_HANDLE;
        VkDeviceMemory stagingBufferMemoryPtrGPU = VK_NULL_HANDLE;
        CreateBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBufferPtrGPU, stagingBufferMemoryPtrGPU);

        void *dataPtr = nullptr;
        if (vkMapMemory(m_context.m_devicePtrGPU, stagingBufferMemoryPtrGPU, 0, bufferSize, 0, &dataPtr) != VK_SUCCESS)
        {
            vkDestroyBuffer(m_context.m_devicePtrGPU, stagingBufferPtrGPU, nullptr);
            vkFreeMemory(m_context.m_devicePtrGPU, stagingBufferMemoryPtrGPU, nullptr);
            throw std::runtime_error("Failed to map scene object index update staging buffer.");
        }

        std::memcpy(dataPtr, m_indices32.data(), static_cast<std::size_t>(bufferSize));
        vkUnmapMemory(m_context.m_devicePtrGPU, stagingBufferMemoryPtrGPU);

        CopyBuffer(stagingBufferPtrGPU, m_indexBufferPtrGPU, bufferSize);

        vkDestroyBuffer(m_context.m_devicePtrGPU, stagingBufferPtrGPU, nullptr);
        vkFreeMemory(m_context.m_devicePtrGPU, stagingBufferMemoryPtrGPU, nullptr);
    }

    //Updates 32-bit mesh vertices and indices in the base object.
    void SceneObject::UpdateMesh(std::vector<Vertex> vertices, std::vector<uint32_t> indices)
    {
        if (vertices.empty())
        {
            throw std::runtime_error("Scene object mesh update must contain at least one vertex.");
        }

        if (indices.empty())
        {
            throw std::runtime_error("Scene object mesh update must contain at least one index.");
        }

        if (indices.size() > std::numeric_limits<uint32_t>::max())
        {
            throw std::runtime_error("Scene object mesh update index count is too large.");
        }

        m_vertices = std::move(vertices);
        m_indices16.clear();
        m_indices32 = std::move(indices);
        m_indexType = VK_INDEX_TYPE_UINT32;
        m_indexCount = static_cast<uint32_t>(m_indices32.size());
        UpdateSelectionBounds();

        if (m_context.m_devicePtrGPU == VK_NULL_HANDLE)
        {
            return;
        }

        RetireBuffer(m_vertexBufferPtrGPU, m_vertexBufferMemoryPtrGPU);
        RetireBuffer(m_indexBufferPtrGPU, m_indexBufferMemoryPtrGPU);
        CreateVertexBuffer();
        CreateIndexBuffer();
    }

    //Checks whether a value fits in uint16_t.
    uint16_t SceneObject::CheckedU16(std::size_t value)
    {
        if (value > std::numeric_limits<uint16_t>::max())
        {
            throw std::runtime_error("Value does not fit in uint16_t.");
        }

        return static_cast<uint16_t>(value);
    }

    //Stores the optional texture file name.
    void SceneObject::SetTextureFileName(const std::string &textureFileName)
    {
        m_diffuseTexture.m_fileName = textureFileName;
    }

    //Stores the optional normal map file name.
    void SceneObject::SetNormalMapFileName(const std::string &normalMapFileName)
    {
        m_normalTexture.m_fileName = normalMapFileName;
    }

    //Stores the shader file names.
    void SceneObject::SetShaderFileNames(const std::string &vertexShader, const std::string &fragmentShader)
    {
        m_vertexShader = vertexShader;
        m_fragmentShader = fragmentShader;
    }

    //Initializes GPU resources for this object.
    void SceneObject::Init(const SceneObjectContext &context)
    {
        m_context = context;
        Update(0.0f, 0.0f);
        CreateTextureResources();
        CreatePipeline();
        CreateShadowPipeline();
        CreateVertexBuffer();
        CreateIndexBuffer();
        CreateInstanceBuffer();
    }

    //Updates this object's transform.
    void SceneObject::Update(float elapsedSeconds, float deltaSeconds)
    {
        (void)elapsedSeconds;
        (void)deltaSeconds;

        UpdateModelMatrix();
        ReleaseRetiredBuffers(false);
    }

    //Sets this object's initial orientation.
    void SceneObject::SetOrientation(const Float4x4 &orientation)
    {
        m_orientation = orientation;
        UpdateModelMatrix();
    }

    //Sets this object's position.
    void SceneObject::SetPosition(const Float3 &position)
    {
        m_position = position;
        UpdateModelMatrix();
    }

    //Sets this object's scale.
    void SceneObject::SetScale(float scaleValue)
    {
        m_scale = std::max(0.001f, scaleValue);
        UpdateModelMatrix();
    }

    //Sets whether this object can be selected.
    void SceneObject::SetClickable(bool clickable)
    {
        m_renderOptions.m_clickable = clickable;
        m_renderOptions.m_selectionMode = clickable ? SceneObjectSelectionMode::PerObject : SceneObjectSelectionMode::None;
    }

    //Sets this object's selection mode.
    void SceneObject::SetSelectionMode(SceneObjectSelectionMode selectionMode)
    {
        m_renderOptions.m_selectionMode = selectionMode;
        m_renderOptions.m_clickable = selectionMode != SceneObjectSelectionMode::None;
    }

    //Sets this object's debug name.
    void SceneObject::SetName(const std::string &name)
    {
        m_name = name.empty() ? "SceneObject" : name;
    }

    //Returns this object's position.
    const Float3 &SceneObject::Position(void) const
    {
        return m_position;
    }

    //Returns this object's debug name.
    const std::string &SceneObject::Name(void) const
    {
        return m_name;
    }

    //Returns this object's orientation.
    const Float4x4 &SceneObject::Orientation(void) const
    {
        return m_orientation;
    }

    //Returns this object's scale.
    float SceneObject::Scale(void) const
    {
        return m_scale;
    }

    //Returns whether this object can be selected.
    bool SceneObject::Clickable(void) const
    {
        return m_renderOptions.m_clickable && m_renderOptions.m_selectionMode != SceneObjectSelectionMode::None;
    }

    //Returns this object's selection mode.
    SceneObjectSelectionMode SceneObject::SelectionMode(void) const
    {
        return m_renderOptions.m_selectionMode;
    }

    //Returns this object's selection radius.
    float SceneObject::SelectionRadius(void) const
    {
        return std::max(0.1f, m_selectionRadius * std::max(0.001f, m_scale));
    }

    //Returns this object's selection center.
    Float3 SceneObject::SelectionCenter(void) const
    {
        const Float4 worldCenter = transform(m_model, { m_selectionCenter.x, m_selectionCenter.y, m_selectionCenter.z, 1.0f });
        return { worldCenter.x, worldCenter.y, worldCenter.z };
    }

    //Returns whether this object uses instancing.
    bool SceneObject::InstancingEnabled(void) const
    {
        return m_instancingEnabled;
    }

    //Returns this object's instance count.
    uint32_t SceneObject::InstanceCount(void) const
    {
        return m_instanceCount;
    }

    //Returns one instance position.
    Float3 SceneObject::InstancePosition(uint32_t instanceIndex) const
    {
        if (instanceIndex >= m_instances.size())
        {
            return { 0.0f, 0.0f, 0.0f };
        }

        const Float3 localPosition = MatrixPosition(m_instances[instanceIndex].m_model);
        const Float4 worldPosition = transform(m_model, { localPosition.x, localPosition.y, localPosition.z, 1.0f });
        return { worldPosition.x, worldPosition.y, worldPosition.z };
    }

    //Returns one instance orientation.
    Float4x4 SceneObject::InstanceOrientation(uint32_t instanceIndex) const
    {
        if (instanceIndex >= m_instances.size())
        {
            return identity();
        }

        return multiply(m_orientation, MatrixOrientation(m_instances[instanceIndex].m_model));
    }

    //Returns one instance scale.
    float SceneObject::InstanceScale(uint32_t instanceIndex) const
    {
        if (instanceIndex >= m_instances.size())
        {
            return 1.0f;
        }

        return std::max(0.001f, m_scale * MatrixScale(m_instances[instanceIndex].m_model));
    }

    //Returns one instance selection radius.
    float SceneObject::InstanceSelectionRadius(uint32_t instanceIndex) const
    {
        return std::max(0.1f, m_selectionRadius * InstanceScale(instanceIndex));
    }

    //Returns one instance selection center.
    Float3 SceneObject::InstanceSelectionCenter(uint32_t instanceIndex) const
    {
        if (instanceIndex >= m_instances.size())
        {
            return { 0.0f, 0.0f, 0.0f };
        }

        const Float4 instanceCenter = transform(m_instances[instanceIndex].m_model, { m_selectionCenter.x, m_selectionCenter.y, m_selectionCenter.z, 1.0f });
        const Float4 worldCenter = transform(m_model, instanceCenter);
        return { worldCenter.x, worldCenter.y, worldCenter.z };
    }

    //Sets one instance transform.
    void SceneObject::SetInstanceTransform(uint32_t instanceIndex, const Float3 &position, const Float4x4 &orientation, float scaleValue)
    {
        if (instanceIndex >= m_instances.size())
        {
            return;
        }

        m_instances[instanceIndex].m_model = ModelMatrixFromTransform(position, orientation, scaleValue);
        UpdateInstanceBuffer();
    }

    //Draws this object.
    void SceneObject::Show(VkCommandBuffer commandBuffer, VkDescriptorSet cameraDescriptorSet) const
    {
        Show(commandBuffer, cameraDescriptorSet, m_context.m_swapChainExtent);
    }

    //Draws this object with a render mode.
    void SceneObject::Show(VkCommandBuffer commandBuffer, VkDescriptorSet cameraDescriptorSet, bool wireframe) const
    {
        Show(commandBuffer, cameraDescriptorSet, m_context.m_swapChainExtent, wireframe);
    }

    //Draws this object with an explicit render extent.
    void SceneObject::Show(VkCommandBuffer commandBuffer, VkDescriptorSet cameraDescriptorSet, VkExtent2D renderExtent) const
    {
        Show(commandBuffer, cameraDescriptorSet, renderExtent, false);
    }

    //Draws this object with an explicit render extent and mode.
    void SceneObject::Show(VkCommandBuffer commandBuffer, VkDescriptorSet cameraDescriptorSet, VkExtent2D renderExtent, bool wireframe) const
    {
        if (m_pipelinePtrGPU == VK_NULL_HANDLE || m_vertexBufferPtrGPU == VK_NULL_HANDLE || m_indexBufferPtrGPU == VK_NULL_HANDLE || m_textureDescriptorSetPtrGPU == VK_NULL_HANDLE)
        {
            return;
        }

        if (m_instancingEnabled && (m_instanceCount == 0U || m_instanceBufferPtrGPU == VK_NULL_HANDLE))
        {
            return;
        }

        const bool drawWireframe = wireframe || m_renderOptions.m_wireframe;
        const VkPipeline pipelinePtrGPU = drawWireframe && m_wireframePipelinePtrGPU != VK_NULL_HANDLE ? m_wireframePipelinePtrGPU : m_pipelinePtrGPU;
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelinePtrGPU);

        VkViewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = static_cast<float>(renderExtent.width);
        viewport.height = static_cast<float>(renderExtent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        vkCmdSetViewport(commandBuffer, 0, 1, &viewport);

        VkRect2D scissor{};
        scissor.offset = { 0, 0 };
        scissor.extent = renderExtent;
        vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

        vkCmdSetLineWidth(commandBuffer, EffectiveLineWidth(drawWireframe));

        const VkDeviceSize offsets[] = { 0, 0 };
        if (m_instancingEnabled)
        {
            const VkBuffer vertexBuffers[] = { m_vertexBufferPtrGPU, m_instanceBufferPtrGPU };
            vkCmdBindVertexBuffers(commandBuffer, 0, 2, vertexBuffers, offsets);
        }
        else
        {
            const VkBuffer vertexBuffers[] = { m_vertexBufferPtrGPU };
            vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers, offsets);
        }

        vkCmdBindIndexBuffer(commandBuffer, m_indexBufferPtrGPU, 0, m_indexType);

        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipelineLayoutPtrGPU, 0, 1, &cameraDescriptorSet, 0, nullptr);

        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipelineLayoutPtrGPU, 1, 1, &m_textureDescriptorSetPtrGPU, 0, nullptr);

        const bool pointSizeEnabled = m_topology == VK_PRIMITIVE_TOPOLOGY_POINT_LIST && m_context.m_largePointsEnabled;
        const float pointSize = pointSizeEnabled ? std::clamp(m_renderOptions.m_pointSize, m_context.m_minimumPointSize, m_context.m_maximumPointSize) : 1.0f;
        const float requestedMaxPointSize = m_renderOptions.m_maxPointSize > 0.0f ? m_renderOptions.m_maxPointSize : pointSize;
        const float maxPointSize = pointSizeEnabled ? std::clamp(requestedMaxPointSize, pointSize, m_context.m_maximumPointSize) : pointSize;
        const float objectSettingX = m_topology == VK_PRIMITIVE_TOPOLOGY_POINT_LIST && m_renderOptions.m_maxPointSize > 0.0f ? maxPointSize : (m_renderOptions.m_receiveShadow ? 1.0f : 0.0f);

        PushConstants pushConstants{};
        pushConstants.m_model = m_model;
        pushConstants.m_color = m_color;
        pushConstants.m_objectSettings = {
            objectSettingX, pointSize, static_cast<float>(renderExtent.width), static_cast<float>(renderExtent.height)
        };

        vkCmdPushConstants(commandBuffer, m_pipelineLayoutPtrGPU, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushConstants), &pushConstants);

        vkCmdDrawIndexed(commandBuffer, m_indexCount, m_instancingEnabled ? m_instanceCount : 1U, 0, 0, 0);
    }

    //Draws this object into the shadow map.
    void SceneObject::ShowShadow(VkCommandBuffer commandBuffer, VkDescriptorSet cameraDescriptorSet, VkExtent2D shadowExtent) const
    {
        if (m_shadowPipelinePtrGPU == VK_NULL_HANDLE || m_vertexBufferPtrGPU == VK_NULL_HANDLE || m_indexBufferPtrGPU == VK_NULL_HANDLE || m_topology != VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST ||
            !m_depthWrite || !m_renderOptions.m_castShadow)
        {
            return;
        }

        if (m_instancingEnabled && (m_instanceCount == 0U || m_instanceBufferPtrGPU == VK_NULL_HANDLE))
        {
            return;
        }

        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_shadowPipelinePtrGPU);

        VkViewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = static_cast<float>(shadowExtent.width);
        viewport.height = static_cast<float>(shadowExtent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        vkCmdSetViewport(commandBuffer, 0, 1, &viewport);

        VkRect2D scissor{};
        scissor.offset = { 0, 0 };
        scissor.extent = shadowExtent;
        vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

        const VkDeviceSize offsets[] = { 0, 0 };
        if (m_instancingEnabled)
        {
            const VkBuffer vertexBuffers[] = { m_vertexBufferPtrGPU, m_instanceBufferPtrGPU };
            vkCmdBindVertexBuffers(commandBuffer, 0, 2, vertexBuffers, offsets);
        }
        else
        {
            const VkBuffer vertexBuffers[] = { m_vertexBufferPtrGPU };
            vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers, offsets);
        }

        vkCmdBindIndexBuffer(commandBuffer, m_indexBufferPtrGPU, 0, m_indexType);

        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_shadowPipelineLayoutPtrGPU, 0, 1, &cameraDescriptorSet, 0, nullptr);

        PushConstants pushConstants{};
        pushConstants.m_model = m_model;
        pushConstants.m_color = m_color;
        pushConstants.m_objectSettings = {
            m_renderOptions.m_receiveShadow ? 1.0f : 0.0f, m_renderOptions.m_pointSize, 0.0f, 0.0f
        };

        vkCmdPushConstants(commandBuffer, m_shadowPipelineLayoutPtrGPU, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PushConstants), &pushConstants);

        vkCmdDrawIndexed(commandBuffer, m_indexCount, m_instancingEnabled ? m_instanceCount : 1U, 0, 0, 0);
    }

    //Releases all GPU resources for this object.
    void SceneObject::Clean(void)
    {
        CleanPipeline();
        CleanShadowPipeline();

        if (m_pipelineLayoutPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyPipelineLayout(m_context.m_devicePtrGPU, m_pipelineLayoutPtrGPU, nullptr);
            m_pipelineLayoutPtrGPU = VK_NULL_HANDLE;
        }

        if (m_shadowPipelineLayoutPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyPipelineLayout(m_context.m_devicePtrGPU, m_shadowPipelineLayoutPtrGPU, nullptr);
            m_shadowPipelineLayoutPtrGPU = VK_NULL_HANDLE;
        }

        if (m_indexBufferPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyBuffer(m_context.m_devicePtrGPU, m_indexBufferPtrGPU, nullptr);
            m_indexBufferPtrGPU = VK_NULL_HANDLE;
        }

        if (m_indexBufferMemoryPtrGPU != VK_NULL_HANDLE)
        {
            vkFreeMemory(m_context.m_devicePtrGPU, m_indexBufferMemoryPtrGPU, nullptr);
            m_indexBufferMemoryPtrGPU = VK_NULL_HANDLE;
        }

        if (m_vertexBufferPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyBuffer(m_context.m_devicePtrGPU, m_vertexBufferPtrGPU, nullptr);
            m_vertexBufferPtrGPU = VK_NULL_HANDLE;
        }

        if (m_vertexBufferMemoryPtrGPU != VK_NULL_HANDLE)
        {
            vkFreeMemory(m_context.m_devicePtrGPU, m_vertexBufferMemoryPtrGPU, nullptr);
            m_vertexBufferMemoryPtrGPU = VK_NULL_HANDLE;
        }

        ReleaseRetiredBuffers(true);

        CleanInstanceBuffer();

        CleanTexture(m_diffuseTexture);
        CleanTexture(m_normalTexture);

        m_textureDescriptorSetPtrGPU = VK_NULL_HANDLE;
    }

    //Releases only the graphics pipeline.
    void SceneObject::CleanPipeline(void)
    {
        if (m_wireframePipelinePtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyPipeline(m_context.m_devicePtrGPU, m_wireframePipelinePtrGPU, nullptr);
            m_wireframePipelinePtrGPU = VK_NULL_HANDLE;
        }

        if (m_pipelinePtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyPipeline(m_context.m_devicePtrGPU, m_pipelinePtrGPU, nullptr);
            m_pipelinePtrGPU = VK_NULL_HANDLE;
        }
    }

    //Releases only the shadow graphics pipeline.
    void SceneObject::CleanShadowPipeline(void)
    {
        if (m_shadowPipelinePtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyPipeline(m_context.m_devicePtrGPU, m_shadowPipelinePtrGPU, nullptr);
            m_shadowPipelinePtrGPU = VK_NULL_HANDLE;
        }
    }

    //Recreates the pipeline after swapchain changes.
    void SceneObject::RefreshPipeline(const SceneObjectContext &context)
    {
        m_context = context;
        CleanPipeline();

        if (m_pipelineLayoutPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyPipelineLayout(m_context.m_devicePtrGPU, m_pipelineLayoutPtrGPU, nullptr);
            m_pipelineLayoutPtrGPU = VK_NULL_HANDLE;
        }

        CreatePipeline();
    }

    //Creates this object's graphics pipeline.
    void SceneObject::CreatePipeline(void)
    {
        std::string vertexShader = m_vertexShader;
        std::string fragmentShader = m_fragmentShader;

        if (m_context.m_shaderOverrideEnabled)
        {
            if (m_instancingEnabled && !m_context.m_instancedVertexShaderOverride.empty())
            {
                vertexShader = m_context.m_instancedVertexShaderOverride;
            }
            else if (!m_context.m_vertexShaderOverride.empty())
            {
                vertexShader = m_context.m_vertexShaderOverride;
            }

            if (!m_context.m_fragmentShaderOverride.empty())
            {
                fragmentShader = m_context.m_fragmentShaderOverride;
            }
        }

        const auto vertexShaderCode = ReadShaderFile(ShaderPath(vertexShader));
        const auto fragmentShaderCode = ReadShaderFile(ShaderPath(fragmentShader));

        if (vertexShaderCode.empty() || fragmentShaderCode.empty())
        {
            throw std::runtime_error("Shader file is empty.");
        }

        const VkShaderModule vertexShaderModule = CreateShaderModule(vertexShaderCode);
        const VkShaderModule fragmentShaderModule = CreateShaderModule(fragmentShaderCode);

        // Vertex specialization constant 0 describes the output transfer function.
        // Shaders that do not declare this constant ignore it.
        const VkBool32 colorAttachmentIsSrgb = m_context.m_colorAttachmentIsSrgb ? VK_TRUE : VK_FALSE;
        const VkSpecializationMapEntry colorAttachmentEntry{ 0U, 0U, sizeof(colorAttachmentIsSrgb) };
        VkSpecializationInfo colorAttachmentSpecialization{};
        colorAttachmentSpecialization.mapEntryCount = 1U;
        colorAttachmentSpecialization.pMapEntries = &colorAttachmentEntry;
        colorAttachmentSpecialization.dataSize = sizeof(colorAttachmentIsSrgb);
        colorAttachmentSpecialization.pData = &colorAttachmentIsSrgb;

        VkPipelineShaderStageCreateInfo vertexShaderStageInfo{};
        vertexShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        vertexShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertexShaderStageInfo.module = vertexShaderModule;
        vertexShaderStageInfo.pName = "main";
        vertexShaderStageInfo.pSpecializationInfo = &colorAttachmentSpecialization;

        VkPipelineShaderStageCreateInfo fragmentShaderStageInfo{};
        fragmentShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        fragmentShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        fragmentShaderStageInfo.module = fragmentShaderModule;
        fragmentShaderStageInfo.pName = "main";

        const VkPipelineShaderStageCreateInfo shaderStages[] = {
            vertexShaderStageInfo, fragmentShaderStageInfo
        };

        const auto vertexBindingDescription = Vertex::BindingDescription();
        const auto vertexAttributeDescriptions = Vertex::AttributeDescriptions();
        const auto instanceBindingDescription = SceneObjectInstanceData::BindingDescription();
        const auto instanceAttributeDescriptions = SceneObjectInstanceData::AttributeDescriptions();

        std::array<VkVertexInputBindingDescription, 2> bindingDescriptions{};
        bindingDescriptions[0] = vertexBindingDescription;

        std::array<VkVertexInputAttributeDescription, 10> attributeDescriptions{};
        std::copy(vertexAttributeDescriptions.begin(), vertexAttributeDescriptions.end(), attributeDescriptions.begin());

        uint32_t bindingDescriptionCount = 1U;
        uint32_t attributeDescriptionCount = static_cast<uint32_t>(vertexAttributeDescriptions.size());

        if (m_instancingEnabled)
        {
            bindingDescriptions[1] = instanceBindingDescription;
            std::copy(instanceAttributeDescriptions.begin(), instanceAttributeDescriptions.end(), attributeDescriptions.begin() + vertexAttributeDescriptions.size());
            bindingDescriptionCount = static_cast<uint32_t>(bindingDescriptions.size());
            attributeDescriptionCount += static_cast<uint32_t>(instanceAttributeDescriptions.size());
        }

        VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
        vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        vertexInputInfo.vertexBindingDescriptionCount = bindingDescriptionCount;
        vertexInputInfo.pVertexBindingDescriptions = bindingDescriptions.data();
        vertexInputInfo.vertexAttributeDescriptionCount = attributeDescriptionCount;
        vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

        VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
        inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        inputAssembly.topology = m_topology;
        inputAssembly.primitiveRestartEnable = VK_FALSE;

        VkViewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = static_cast<float>(m_context.m_swapChainExtent.width);
        viewport.height = static_cast<float>(m_context.m_swapChainExtent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;

        VkRect2D scissor{};
        scissor.offset = { 0, 0 };
        scissor.extent = m_context.m_swapChainExtent;

        VkPipelineViewportStateCreateInfo viewportState{};
        viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        viewportState.viewportCount = 1;
        viewportState.pViewports = &viewport;
        viewportState.scissorCount = 1;
        viewportState.pScissors = &scissor;

        VkPipelineRasterizationStateCreateInfo rasterizer{};
        rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        rasterizer.depthClampEnable = VK_FALSE;
        rasterizer.rasterizerDiscardEnable = VK_FALSE;
        rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
        rasterizer.lineWidth = 1.0f;
        rasterizer.cullMode = VK_CULL_MODE_NONE;
        rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rasterizer.depthBiasEnable = VK_FALSE;

        VkPipelineMultisampleStateCreateInfo multisampling{};
        multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        multisampling.sampleShadingEnable = VK_FALSE;
        multisampling.rasterizationSamples = m_context.m_sampleCount;

        VkPipelineDepthStencilStateCreateInfo depthStencil{};
        depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        depthStencil.depthTestEnable = m_renderOptions.m_depthTest ? VK_TRUE : VK_FALSE;
        depthStencil.depthWriteEnable = m_depthWrite ? VK_TRUE : VK_FALSE;
        depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
        depthStencil.depthBoundsTestEnable = VK_FALSE;
        depthStencil.stencilTestEnable = VK_FALSE;

        VkPipelineColorBlendAttachmentState colorBlendAttachment{};
        colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        colorBlendAttachment.blendEnable = m_renderOptions.m_alphaBlend ? VK_TRUE : VK_FALSE;
        colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
        colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;

        VkPipelineColorBlendStateCreateInfo colorBlending{};
        colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        colorBlending.logicOpEnable = VK_FALSE;
        colorBlending.attachmentCount = 1;
        colorBlending.pAttachments = &colorBlendAttachment;

        const std::array<VkDynamicState, 3> dynamicStates = {
            VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR, VK_DYNAMIC_STATE_LINE_WIDTH
        };

        VkPipelineDynamicStateCreateInfo dynamicState{};
        dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
        dynamicState.pDynamicStates = dynamicStates.data();

        if (m_pipelineLayoutPtrGPU == VK_NULL_HANDLE)
        {
            VkPushConstantRange pushConstantRange{};
            pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
            pushConstantRange.offset = 0;
            pushConstantRange.size = sizeof(PushConstants);

            const std::array<VkDescriptorSetLayout, 2> setLayouts = {
                m_context.m_descriptorSetLayoutPtrGPU, m_context.m_textureDescriptorSetLayoutPtrGPU
            };

            VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
            pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
            pipelineLayoutInfo.setLayoutCount = static_cast<uint32_t>(setLayouts.size());
            pipelineLayoutInfo.pSetLayouts = setLayouts.data();
            pipelineLayoutInfo.pushConstantRangeCount = 1;
            pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

            if (vkCreatePipelineLayout(m_context.m_devicePtrGPU, &pipelineLayoutInfo, nullptr, &m_pipelineLayoutPtrGPU) != VK_SUCCESS)
            {
                vkDestroyShaderModule(m_context.m_devicePtrGPU, fragmentShaderModule, nullptr);
                vkDestroyShaderModule(m_context.m_devicePtrGPU, vertexShaderModule, nullptr);
                throw std::runtime_error("Failed to create scene object pipeline layout.");
            }
        }

        VkGraphicsPipelineCreateInfo pipelineInfo{};
        pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        pipelineInfo.stageCount = 2;
        pipelineInfo.pStages = shaderStages;
        pipelineInfo.pVertexInputState = &vertexInputInfo;
        pipelineInfo.pInputAssemblyState = &inputAssembly;
        pipelineInfo.pViewportState = &viewportState;
        pipelineInfo.pRasterizationState = &rasterizer;
        pipelineInfo.pMultisampleState = &multisampling;
        pipelineInfo.pDepthStencilState = &depthStencil;
        pipelineInfo.pColorBlendState = &colorBlending;
        pipelineInfo.pDynamicState = &dynamicState;
        pipelineInfo.layout = m_pipelineLayoutPtrGPU;
        pipelineInfo.renderPass = m_context.m_renderPassPtrGPU;
        pipelineInfo.subpass = 0;

        if (vkCreateGraphicsPipelines(m_context.m_devicePtrGPU, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_pipelinePtrGPU) != VK_SUCCESS)
        {
            vkDestroyShaderModule(m_context.m_devicePtrGPU, fragmentShaderModule, nullptr);
            vkDestroyShaderModule(m_context.m_devicePtrGPU, vertexShaderModule, nullptr);
            throw std::runtime_error("Failed to create scene object graphics pipeline.");
        }

        if (m_context.m_wireframeEnabled && m_topology == VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST && !m_context.m_shaderOverrideEnabled)
        {
            rasterizer.polygonMode = VK_POLYGON_MODE_LINE;
            if (vkCreateGraphicsPipelines(m_context.m_devicePtrGPU, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_wireframePipelinePtrGPU) != VK_SUCCESS)
            {
                m_wireframePipelinePtrGPU = VK_NULL_HANDLE;
                OutputDebugStringA("Scene object wireframe pipeline creation failed. Fill pipeline will be used.\n");
            }
        }

        vkDestroyShaderModule(m_context.m_devicePtrGPU, fragmentShaderModule, nullptr);
        vkDestroyShaderModule(m_context.m_devicePtrGPU, vertexShaderModule, nullptr);
    }

    //Creates this object's shadow map graphics pipeline.
    void SceneObject::CreateShadowPipeline(void)
    {
        if (m_context.m_shadowRenderPassPtrGPU == VK_NULL_HANDLE || m_topology != VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST || !m_depthWrite || !m_renderOptions.m_castShadow)
        {
            return;
        }

        const std::string vertexShader = m_instancingEnabled ? "shaders/shadowDepthInstanced.vert" : "shaders/shadowDepth.vert";
        const auto vertexShaderCode = ReadShaderFile(ShaderPath(vertexShader));

        if (vertexShaderCode.empty())
        {
            throw std::runtime_error("Shadow shader file is empty.");
        }

        const VkShaderModule vertexShaderModule = CreateShaderModule(vertexShaderCode);

        VkPipelineShaderStageCreateInfo vertexShaderStageInfo{};
        vertexShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        vertexShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertexShaderStageInfo.module = vertexShaderModule;
        vertexShaderStageInfo.pName = "main";

        const auto vertexBindingDescription = Vertex::BindingDescription();
        const auto vertexAttributeDescriptions = Vertex::AttributeDescriptions();
        const auto instanceBindingDescription = SceneObjectInstanceData::BindingDescription();
        const auto instanceAttributeDescriptions = SceneObjectInstanceData::AttributeDescriptions();

        std::array<VkVertexInputBindingDescription, 2> bindingDescriptions{};
        bindingDescriptions[0] = vertexBindingDescription;

        std::array<VkVertexInputAttributeDescription, 10> attributeDescriptions{};
        std::copy(vertexAttributeDescriptions.begin(), vertexAttributeDescriptions.end(), attributeDescriptions.begin());

        uint32_t bindingDescriptionCount = 1U;
        uint32_t attributeDescriptionCount = static_cast<uint32_t>(vertexAttributeDescriptions.size());

        if (m_instancingEnabled)
        {
            bindingDescriptions[1] = instanceBindingDescription;
            std::copy(instanceAttributeDescriptions.begin(), instanceAttributeDescriptions.end(), attributeDescriptions.begin() + vertexAttributeDescriptions.size());
            bindingDescriptionCount = static_cast<uint32_t>(bindingDescriptions.size());
            attributeDescriptionCount += static_cast<uint32_t>(instanceAttributeDescriptions.size());
        }

        VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
        vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        vertexInputInfo.vertexBindingDescriptionCount = bindingDescriptionCount;
        vertexInputInfo.pVertexBindingDescriptions = bindingDescriptions.data();
        vertexInputInfo.vertexAttributeDescriptionCount = attributeDescriptionCount;
        vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

        VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
        inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        inputAssembly.topology = m_topology;
        inputAssembly.primitiveRestartEnable = VK_FALSE;

        VkViewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = static_cast<float>(std::max(m_context.m_shadowExtent.width, 1U));
        viewport.height = static_cast<float>(std::max(m_context.m_shadowExtent.height, 1U));
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;

        VkRect2D scissor{};
        scissor.offset = { 0, 0 };
        scissor.extent.width = std::max(m_context.m_shadowExtent.width, 1U);
        scissor.extent.height = std::max(m_context.m_shadowExtent.height, 1U);

        VkPipelineViewportStateCreateInfo viewportState{};
        viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        viewportState.viewportCount = 1;
        viewportState.pViewports = &viewport;
        viewportState.scissorCount = 1;
        viewportState.pScissors = &scissor;

        VkPipelineRasterizationStateCreateInfo rasterizer{};
        rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        rasterizer.depthClampEnable = VK_FALSE;
        rasterizer.rasterizerDiscardEnable = VK_FALSE;
        rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
        rasterizer.lineWidth = 1.0f;
        rasterizer.cullMode = VK_CULL_MODE_NONE;
        rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rasterizer.depthBiasEnable = VK_TRUE;
        rasterizer.depthBiasConstantFactor = 1.25f;
        rasterizer.depthBiasClamp = 0.0f;
        rasterizer.depthBiasSlopeFactor = 1.75f;

        VkPipelineMultisampleStateCreateInfo multisampling{};
        multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        multisampling.sampleShadingEnable = VK_FALSE;
        multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        VkPipelineDepthStencilStateCreateInfo depthStencil{};
        depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        depthStencil.depthTestEnable = VK_TRUE;
        depthStencil.depthWriteEnable = VK_TRUE;
        depthStencil.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        depthStencil.depthBoundsTestEnable = VK_FALSE;
        depthStencil.stencilTestEnable = VK_FALSE;

        VkPipelineColorBlendStateCreateInfo colorBlending{};
        colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        colorBlending.logicOpEnable = VK_FALSE;
        colorBlending.attachmentCount = 0;
        colorBlending.pAttachments = nullptr;

        const std::array<VkDynamicState, 2> dynamicStates = {
            VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR
        };

        VkPipelineDynamicStateCreateInfo dynamicState{};
        dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
        dynamicState.pDynamicStates = dynamicStates.data();

        if (m_shadowPipelineLayoutPtrGPU == VK_NULL_HANDLE)
        {
            VkPushConstantRange pushConstantRange{};
            pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
            pushConstantRange.offset = 0;
            pushConstantRange.size = sizeof(PushConstants);

            VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
            pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
            pipelineLayoutInfo.setLayoutCount = 1;
            pipelineLayoutInfo.pSetLayouts = &m_context.m_descriptorSetLayoutPtrGPU;
            pipelineLayoutInfo.pushConstantRangeCount = 1;
            pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

            if (vkCreatePipelineLayout(m_context.m_devicePtrGPU, &pipelineLayoutInfo, nullptr, &m_shadowPipelineLayoutPtrGPU) != VK_SUCCESS)
            {
                vkDestroyShaderModule(m_context.m_devicePtrGPU, vertexShaderModule, nullptr);
                throw std::runtime_error("Failed to create scene object shadow pipeline layout.");
            }
        }

        VkGraphicsPipelineCreateInfo pipelineInfo{};
        pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        pipelineInfo.stageCount = 1;
        pipelineInfo.pStages = &vertexShaderStageInfo;
        pipelineInfo.pVertexInputState = &vertexInputInfo;
        pipelineInfo.pInputAssemblyState = &inputAssembly;
        pipelineInfo.pViewportState = &viewportState;
        pipelineInfo.pRasterizationState = &rasterizer;
        pipelineInfo.pMultisampleState = &multisampling;
        pipelineInfo.pDepthStencilState = &depthStencil;
        pipelineInfo.pColorBlendState = &colorBlending;
        pipelineInfo.pDynamicState = &dynamicState;
        pipelineInfo.layout = m_shadowPipelineLayoutPtrGPU;
        pipelineInfo.renderPass = m_context.m_shadowRenderPassPtrGPU;
        pipelineInfo.subpass = 0;

        if (vkCreateGraphicsPipelines(m_context.m_devicePtrGPU, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_shadowPipelinePtrGPU) != VK_SUCCESS)
        {
            vkDestroyShaderModule(m_context.m_devicePtrGPU, vertexShaderModule, nullptr);
            throw std::runtime_error("Failed to create scene object shadow graphics pipeline.");
        }

        vkDestroyShaderModule(m_context.m_devicePtrGPU, vertexShaderModule, nullptr);
    }

    //Creates this object's vertex buffer.
    void SceneObject::CreateVertexBuffer(void)
    {
        if (m_vertices.empty())
        {
            throw std::runtime_error("Scene object has no vertices.");
        }

        const VkDeviceSize bufferSize = static_cast<VkDeviceSize>(sizeof(m_vertices.front())) * static_cast<VkDeviceSize>(m_vertices.size());

        VkBuffer stagingBuffer = VK_NULL_HANDLE;
        VkDeviceMemory stagingBufferMemory = VK_NULL_HANDLE;
        CreateBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

        void *data = nullptr;
        if (vkMapMemory(m_context.m_devicePtrGPU, stagingBufferMemory, 0, bufferSize, 0, &data) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to map scene object vertex staging buffer.");
        }

        std::memcpy(data, m_vertices.data(), static_cast<std::size_t>(bufferSize));
        vkUnmapMemory(m_context.m_devicePtrGPU, stagingBufferMemory);

        CreateBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, m_vertexBufferPtrGPU, m_vertexBufferMemoryPtrGPU);

        CopyBuffer(stagingBuffer, m_vertexBufferPtrGPU, bufferSize);

        vkDestroyBuffer(m_context.m_devicePtrGPU, stagingBuffer, nullptr);
        vkFreeMemory(m_context.m_devicePtrGPU, stagingBufferMemory, nullptr);
    }

    //Creates this object's index buffer.
    void SceneObject::CreateIndexBuffer(void)
    {
        const void *indexDataPtr = nullptr;
        VkDeviceSize bufferSize = 0;

        if (m_indexType == VK_INDEX_TYPE_UINT16)
        {
            if (m_indices16.empty())
            {
                throw std::runtime_error("Scene object has no 16-bit indices.");
            }

            indexDataPtr = m_indices16.data();
            bufferSize = static_cast<VkDeviceSize>(sizeof(m_indices16.front())) * static_cast<VkDeviceSize>(m_indices16.size());
        }
        else if (m_indexType == VK_INDEX_TYPE_UINT32)
        {
            if (m_indices32.empty())
            {
                throw std::runtime_error("Scene object has no 32-bit indices.");
            }

            indexDataPtr = m_indices32.data();
            bufferSize = static_cast<VkDeviceSize>(sizeof(m_indices32.front())) * static_cast<VkDeviceSize>(m_indices32.size());
        }
        else
        {
            throw std::runtime_error("Scene object index type is not supported.");
        }

        VkBuffer stagingBuffer = VK_NULL_HANDLE;
        VkDeviceMemory stagingBufferMemory = VK_NULL_HANDLE;
        CreateBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

        void *data = nullptr;
        if (vkMapMemory(m_context.m_devicePtrGPU, stagingBufferMemory, 0, bufferSize, 0, &data) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to map scene object index staging buffer.");
        }

        std::memcpy(data, indexDataPtr, static_cast<std::size_t>(bufferSize));
        vkUnmapMemory(m_context.m_devicePtrGPU, stagingBufferMemory);

        CreateBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, m_indexBufferPtrGPU, m_indexBufferMemoryPtrGPU);

        CopyBuffer(stagingBuffer, m_indexBufferPtrGPU, bufferSize);

        vkDestroyBuffer(m_context.m_devicePtrGPU, stagingBuffer, nullptr);
        vkFreeMemory(m_context.m_devicePtrGPU, stagingBufferMemory, nullptr);
    }

    //Creates this object's instance buffer.
    void SceneObject::CreateInstanceBuffer(void)
    {
        if (!m_instancingEnabled || m_instances.empty())
        {
            return;
        }

        const VkDeviceSize bufferSize = static_cast<VkDeviceSize>(sizeof(m_instances.front())) * static_cast<VkDeviceSize>(m_instances.size());

        VkBuffer stagingBufferPtrGPU = VK_NULL_HANDLE;
        VkDeviceMemory stagingBufferMemoryPtrGPU = VK_NULL_HANDLE;
        CreateBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBufferPtrGPU, stagingBufferMemoryPtrGPU);

        void *dataPtr = nullptr;
        if (vkMapMemory(m_context.m_devicePtrGPU, stagingBufferMemoryPtrGPU, 0, bufferSize, 0, &dataPtr) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to map scene object instance staging buffer.");
        }

        std::memcpy(dataPtr, m_instances.data(), static_cast<std::size_t>(bufferSize));
        vkUnmapMemory(m_context.m_devicePtrGPU, stagingBufferMemoryPtrGPU);

        CreateBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, m_instanceBufferPtrGPU, m_instanceBufferMemoryPtrGPU);

        CopyBuffer(stagingBufferPtrGPU, m_instanceBufferPtrGPU, bufferSize);

        vkDestroyBuffer(m_context.m_devicePtrGPU, stagingBufferPtrGPU, nullptr);
        vkFreeMemory(m_context.m_devicePtrGPU, stagingBufferMemoryPtrGPU, nullptr);
    }

    //Creates this object's texture resources.
    void SceneObject::CreateTextureResources(void)
    {
        const TextureData diffuseTexture = LoadTextureData(m_diffuseTexture, false);
        CreateTextureImage(m_diffuseTexture, diffuseTexture);
        CreateTextureImageView(m_diffuseTexture);
        CreateTextureSampler(m_diffuseTexture);

        const TextureData normalTexture = LoadTextureData(m_normalTexture, true);
        CreateTextureImage(m_normalTexture, normalTexture);
        CreateTextureImageView(m_normalTexture);
        CreateTextureSampler(m_normalTexture);

        CreateTextureDescriptorSet();
    }

    //Creates this object's texture image.
    void SceneObject::CreateTextureImage(MaterialTexture &textureResource, const TextureData &texture)
    {
        if (texture.m_width == 0 || texture.m_height == 0 || texture.m_channelCount != 4U || texture.m_pixels.empty())
        {
            throw std::runtime_error("Texture data must contain RGBA pixels.");
        }

        textureResource.m_mipLevels = MipLevelsForTexture(texture.m_width, texture.m_height);

        VkFormatProperties formatProperties{};
        vkGetPhysicalDeviceFormatProperties(m_context.m_physicalDevicePtrGPU, VK_FORMAT_R8G8B8A8_UNORM, &formatProperties);
        const VkFormatFeatureFlags mipmapFeatures = VK_FORMAT_FEATURE_BLIT_SRC_BIT | VK_FORMAT_FEATURE_BLIT_DST_BIT | VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT;
        if ((formatProperties.optimalTilingFeatures & mipmapFeatures) != mipmapFeatures)
        {
            textureResource.m_mipLevels = 1U;
        }

        const VkDeviceSize imageSize = static_cast<VkDeviceSize>(texture.m_pixels.size());

        VkBuffer stagingBufferPtrGPU = VK_NULL_HANDLE;
        VkDeviceMemory stagingBufferMemoryPtrGPU = VK_NULL_HANDLE;
        CreateBuffer(imageSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBufferPtrGPU, stagingBufferMemoryPtrGPU);

        void *dataPtr = nullptr;
        if (vkMapMemory(m_context.m_devicePtrGPU, stagingBufferMemoryPtrGPU, 0, imageSize, 0, &dataPtr) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to map scene object texture staging buffer.");
        }

        std::memcpy(dataPtr, texture.m_pixels.data(), texture.m_pixels.size());
        vkUnmapMemory(m_context.m_devicePtrGPU, stagingBufferMemoryPtrGPU);

        VkImageCreateInfo imageInfo{};
        imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.extent.width = texture.m_width;
        imageInfo.extent.height = texture.m_height;
        imageInfo.extent.depth = 1;
        imageInfo.mipLevels = textureResource.m_mipLevels;
        imageInfo.arrayLayers = 1;
        imageInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
        imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        if (vkCreateImage(m_context.m_devicePtrGPU, &imageInfo, nullptr, &textureResource.m_imagePtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create scene object texture image.");
        }

        VkMemoryRequirements memoryRequirements{};
        vkGetImageMemoryRequirements(m_context.m_devicePtrGPU, textureResource.m_imagePtrGPU, &memoryRequirements);

        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memoryRequirements.size;
        allocInfo.memoryTypeIndex = FindMemoryType(memoryRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        if (vkAllocateMemory(m_context.m_devicePtrGPU, &allocInfo, nullptr, &textureResource.m_imageMemoryPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to allocate scene object texture image memory.");
        }

        vkBindImageMemory(m_context.m_devicePtrGPU, textureResource.m_imagePtrGPU, textureResource.m_imageMemoryPtrGPU, 0);

        TransitionImageLayout(textureResource.m_imagePtrGPU, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, textureResource.m_mipLevels);
        CopyBufferToImage(stagingBufferPtrGPU, textureResource.m_imagePtrGPU, texture.m_width, texture.m_height);

        if (textureResource.m_mipLevels > 1U)
        {
            GenerateMipmaps(textureResource.m_imagePtrGPU, static_cast<int32_t>(texture.m_width), static_cast<int32_t>(texture.m_height), textureResource.m_mipLevels);
        }
        else
        {
            TransitionImageLayout(textureResource.m_imagePtrGPU, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, textureResource.m_mipLevels);
        }

        vkDestroyBuffer(m_context.m_devicePtrGPU, stagingBufferPtrGPU, nullptr);
        vkFreeMemory(m_context.m_devicePtrGPU, stagingBufferMemoryPtrGPU, nullptr);
    }

    //Creates this object's texture image view.
    void SceneObject::CreateTextureImageView(MaterialTexture &textureResource)
    {
        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = textureResource.m_imagePtrGPU;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.baseMipLevel = 0;
        viewInfo.subresourceRange.levelCount = textureResource.m_mipLevels;
        viewInfo.subresourceRange.baseArrayLayer = 0;
        viewInfo.subresourceRange.layerCount = 1;

        if (vkCreateImageView(m_context.m_devicePtrGPU, &viewInfo, nullptr, &textureResource.m_imageViewPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create scene object texture image view.");
        }
    }

    //Creates this object's texture sampler.
    void SceneObject::CreateTextureSampler(MaterialTexture &textureResource)
    {
        VkSamplerCreateInfo samplerInfo{};
        samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        samplerInfo.magFilter = VK_FILTER_LINEAR;
        samplerInfo.minFilter = VK_FILTER_LINEAR;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.anisotropyEnable = m_context.m_anisotropyEnabled ? VK_TRUE : VK_FALSE;
        samplerInfo.maxAnisotropy = m_context.m_maximumAnisotropy;
        samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_WHITE;
        samplerInfo.unnormalizedCoordinates = VK_FALSE;
        samplerInfo.compareEnable = VK_FALSE;
        samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        samplerInfo.mipLodBias = 0.0f;
        samplerInfo.minLod = 0.0f;
        samplerInfo.maxLod = static_cast<float>(textureResource.m_mipLevels);

        if (vkCreateSampler(m_context.m_devicePtrGPU, &samplerInfo, nullptr, &textureResource.m_samplerPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create scene object texture sampler.");
        }
    }

    //Creates this object's texture descriptor set.
    void SceneObject::CreateTextureDescriptorSet(void)
    {
        VkDescriptorSetAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        allocInfo.descriptorPool = m_context.m_descriptorPoolPtrGPU;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &m_context.m_textureDescriptorSetLayoutPtrGPU;

        if (vkAllocateDescriptorSets(m_context.m_devicePtrGPU, &allocInfo, &m_textureDescriptorSetPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to allocate scene object texture descriptor set.");
        }

        const std::array<VkDescriptorImageInfo, 2> imageInfos = {
            VkDescriptorImageInfo{
                m_diffuseTexture.m_samplerPtrGPU, m_diffuseTexture.m_imageViewPtrGPU, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
            }, VkDescriptorImageInfo{
                m_normalTexture.m_samplerPtrGPU, m_normalTexture.m_imageViewPtrGPU, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
            }
        };

        std::array<VkWriteDescriptorSet, 2> descriptorWrites{};
        for (std::size_t i = 0; i < descriptorWrites.size(); ++i)
        {
            descriptorWrites[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            descriptorWrites[i].dstSet = m_textureDescriptorSetPtrGPU;
            descriptorWrites[i].dstBinding = static_cast<uint32_t>(i);
            descriptorWrites[i].dstArrayElement = 0;
            descriptorWrites[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            descriptorWrites[i].descriptorCount = 1;
            descriptorWrites[i].pImageInfo = &imageInfos[i];
        }

        vkUpdateDescriptorSets(m_context.m_devicePtrGPU, static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);
    }

    //Releases one texture resource.
    void SceneObject::CleanTexture(MaterialTexture &textureResource)
    {
        if (textureResource.m_samplerPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroySampler(m_context.m_devicePtrGPU, textureResource.m_samplerPtrGPU, nullptr);
            textureResource.m_samplerPtrGPU = VK_NULL_HANDLE;
        }

        if (textureResource.m_imageViewPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyImageView(m_context.m_devicePtrGPU, textureResource.m_imageViewPtrGPU, nullptr);
            textureResource.m_imageViewPtrGPU = VK_NULL_HANDLE;
        }

        if (textureResource.m_imagePtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyImage(m_context.m_devicePtrGPU, textureResource.m_imagePtrGPU, nullptr);
            textureResource.m_imagePtrGPU = VK_NULL_HANDLE;
        }

        if (textureResource.m_imageMemoryPtrGPU != VK_NULL_HANDLE)
        {
            vkFreeMemory(m_context.m_devicePtrGPU, textureResource.m_imageMemoryPtrGPU, nullptr);
            textureResource.m_imageMemoryPtrGPU = VK_NULL_HANDLE;
        }
    }

    //Releases this object's instance buffer.
    void SceneObject::CleanInstanceBuffer(void)
    {
        if (m_instanceBufferPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyBuffer(m_context.m_devicePtrGPU, m_instanceBufferPtrGPU, nullptr);
            m_instanceBufferPtrGPU = VK_NULL_HANDLE;
        }

        if (m_instanceBufferMemoryPtrGPU != VK_NULL_HANDLE)
        {
            vkFreeMemory(m_context.m_devicePtrGPU, m_instanceBufferMemoryPtrGPU, nullptr);
            m_instanceBufferMemoryPtrGPU = VK_NULL_HANDLE;
        }
    }

    //Updates this object's instance buffer data.
    void SceneObject::UpdateInstanceBuffer(void)
    {
        if (!m_instancingEnabled || m_instances.empty() || m_context.m_devicePtrGPU == VK_NULL_HANDLE || m_instanceBufferPtrGPU == VK_NULL_HANDLE)
        {
            return;
        }

        const VkDeviceSize bufferSize = static_cast<VkDeviceSize>(sizeof(m_instances.front())) * static_cast<VkDeviceSize>(m_instances.size());

        VkBuffer stagingBufferPtrGPU = VK_NULL_HANDLE;
        VkDeviceMemory stagingBufferMemoryPtrGPU = VK_NULL_HANDLE;
        CreateBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBufferPtrGPU, stagingBufferMemoryPtrGPU);

        void *dataPtr = nullptr;
        if (vkMapMemory(m_context.m_devicePtrGPU, stagingBufferMemoryPtrGPU, 0, bufferSize, 0, &dataPtr) != VK_SUCCESS)
        {
            vkDestroyBuffer(m_context.m_devicePtrGPU, stagingBufferPtrGPU, nullptr);
            vkFreeMemory(m_context.m_devicePtrGPU, stagingBufferMemoryPtrGPU, nullptr);
            throw std::runtime_error("Failed to map scene object instance update staging buffer.");
        }

        std::memcpy(dataPtr, m_instances.data(), static_cast<std::size_t>(bufferSize));
        vkUnmapMemory(m_context.m_devicePtrGPU, stagingBufferMemoryPtrGPU);

        CopyBuffer(stagingBufferPtrGPU, m_instanceBufferPtrGPU, bufferSize);

        vkDestroyBuffer(m_context.m_devicePtrGPU, stagingBufferPtrGPU, nullptr);
        vkFreeMemory(m_context.m_devicePtrGPU, stagingBufferMemoryPtrGPU, nullptr);
    }

    //Normalizes clickable and selection mode settings.
    void SceneObject::NormalizeRenderOptions(void)
    {
        if (m_renderOptions.m_selectionMode != SceneObjectSelectionMode::None)
        {
            m_renderOptions.m_clickable = true;
            return;
        }

        if (m_renderOptions.m_clickable)
        {
            m_renderOptions.m_selectionMode = SceneObjectSelectionMode::PerObject;
        }
    }

    //Creates a Vulkan buffer and memory.
    void SceneObject::CreateBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer &buffer, VkDeviceMemory &bufferMemory) const
    {
        VkBufferCreateInfo bufferInfo{};
        bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferInfo.size = size;
        bufferInfo.usage = usage;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        if (vkCreateBuffer(m_context.m_devicePtrGPU, &bufferInfo, nullptr, &buffer) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create scene object buffer.");
        }

        VkMemoryRequirements memoryRequirements{};
        vkGetBufferMemoryRequirements(m_context.m_devicePtrGPU, buffer, &memoryRequirements);

        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memoryRequirements.size;
        allocInfo.memoryTypeIndex = FindMemoryType(memoryRequirements.memoryTypeBits, properties);

        if (vkAllocateMemory(m_context.m_devicePtrGPU, &allocInfo, nullptr, &bufferMemory) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to allocate scene object buffer memory.");
        }

        vkBindBufferMemory(m_context.m_devicePtrGPU, buffer, bufferMemory, 0);
    }

    //Copies one Vulkan buffer into another.
    void SceneObject::CopyBuffer(VkBuffer source, VkBuffer destination, VkDeviceSize size) const
    {
        VkCommandBuffer commandBufferPtrGPU = BeginSingleTimeCommands();

        VkBufferCopy copyRegion{};
        copyRegion.size = size;
        vkCmdCopyBuffer(commandBufferPtrGPU, source, destination, 1, &copyRegion);

        EndSingleTimeCommands(commandBufferPtrGPU);
    }

    //Copies one Vulkan buffer into an image.
    void SceneObject::CopyBufferToImage(VkBuffer source, VkImage destinationPtrGPU, uint32_t width, uint32_t height) const
    {
        VkCommandBuffer commandBufferPtrGPU = BeginSingleTimeCommands();

        VkBufferImageCopy region{};
        region.bufferOffset = 0;
        region.bufferRowLength = 0;
        region.bufferImageHeight = 0;
        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.imageSubresource.mipLevel = 0;
        region.imageSubresource.baseArrayLayer = 0;
        region.imageSubresource.layerCount = 1;
        region.imageOffset = { 0, 0, 0 };
        region.imageExtent = { width, height, 1 };

        vkCmdCopyBufferToImage(commandBufferPtrGPU, source, destinationPtrGPU, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

        EndSingleTimeCommands(commandBufferPtrGPU);
    }

    //Transitions a Vulkan image layout.
    void SceneObject::TransitionImageLayout(VkImage imagePtrGPU, VkImageLayout oldLayout, VkImageLayout newLayout, uint32_t mipLevels) const
    {
        VkCommandBuffer commandBufferPtrGPU = BeginSingleTimeCommands();

        VkImageMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = oldLayout;
        barrier.newLayout = newLayout;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = imagePtrGPU;
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        barrier.subresourceRange.baseMipLevel = 0;
        barrier.subresourceRange.levelCount = mipLevels;
        barrier.subresourceRange.baseArrayLayer = 0;
        barrier.subresourceRange.layerCount = 1;

        VkPipelineStageFlags sourceStage{};
        VkPipelineStageFlags destinationStage{};

        if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)
        {
            barrier.srcAccessMask = 0;
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
        }
        else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL && newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
        {
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
            destinationStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        }
        else
        {
            throw std::runtime_error("Unsupported scene object texture layout transition.");
        }

        vkCmdPipelineBarrier(commandBufferPtrGPU, sourceStage, destinationStage, 0, 0, nullptr, 0, nullptr, 1, &barrier);

        EndSingleTimeCommands(commandBufferPtrGPU);
    }

    //Generates mipmap levels for a texture image.
    void SceneObject::GenerateMipmaps(VkImage imagePtrGPU, int32_t width, int32_t height, uint32_t mipLevels) const
    {
        VkCommandBuffer commandBufferPtrGPU = BeginSingleTimeCommands();

        VkImageMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.image = imagePtrGPU;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        barrier.subresourceRange.baseArrayLayer = 0;
        barrier.subresourceRange.layerCount = 1;
        barrier.subresourceRange.levelCount = 1;

        int32_t mipWidth = width;
        int32_t mipHeight = height;

        for (uint32_t i = 1U; i < mipLevels; ++i)
        {
            barrier.subresourceRange.baseMipLevel = i - 1U;
            barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
            barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

            vkCmdPipelineBarrier(commandBufferPtrGPU, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);

            VkImageBlit blit{};
            blit.srcOffsets[0] = { 0, 0, 0 };
            blit.srcOffsets[1] = { mipWidth, mipHeight, 1 };
            blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            blit.srcSubresource.mipLevel = i - 1U;
            blit.srcSubresource.baseArrayLayer = 0;
            blit.srcSubresource.layerCount = 1;
            blit.dstOffsets[0] = { 0, 0, 0 };
            blit.dstOffsets[1] = {
                mipWidth > 1 ? mipWidth / 2 : 1, mipHeight > 1 ? mipHeight / 2 : 1, 1
            };
            blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            blit.dstSubresource.mipLevel = i;
            blit.dstSubresource.baseArrayLayer = 0;
            blit.dstSubresource.layerCount = 1;

            vkCmdBlitImage(commandBufferPtrGPU, imagePtrGPU, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, imagePtrGPU, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit, VK_FILTER_LINEAR);

            barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
            barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

            vkCmdPipelineBarrier(commandBufferPtrGPU, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);

            if (mipWidth > 1)
            {
                mipWidth /= 2;
            }

            if (mipHeight > 1)
            {
                mipHeight /= 2;
            }
        }

        barrier.subresourceRange.baseMipLevel = mipLevels - 1U;
        barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

        vkCmdPipelineBarrier(commandBufferPtrGPU, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);

        EndSingleTimeCommands(commandBufferPtrGPU);
    }

    //Begins a one-time command buffer.
    VkCommandBuffer SceneObject::BeginSingleTimeCommands(void) const
    {
        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandPool = m_context.m_commandPoolPtrGPU;
        allocInfo.commandBufferCount = 1;

        VkCommandBuffer commandBufferPtrGPU = VK_NULL_HANDLE;
        if (vkAllocateCommandBuffers(m_context.m_devicePtrGPU, &allocInfo, &commandBufferPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to allocate scene object one-time command buffer.");
        }

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

        if (vkBeginCommandBuffer(commandBufferPtrGPU, &beginInfo) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to begin scene object one-time command buffer.");
        }

        return commandBufferPtrGPU;
    }

    //Ends a one-time command buffer.
    void SceneObject::EndSingleTimeCommands(VkCommandBuffer commandBufferPtrGPU) const
    {
        if (vkEndCommandBuffer(commandBufferPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to end scene object one-time command buffer.");
        }

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBufferPtrGPU;

        vkQueueSubmit(m_context.m_graphicsQueuePtrGPU, 1, &submitInfo, VK_NULL_HANDLE);
        vkQueueWaitIdle(m_context.m_graphicsQueuePtrGPU);

        vkFreeCommandBuffers(m_context.m_devicePtrGPU, m_context.m_commandPoolPtrGPU, 1, &commandBufferPtrGPU);
    }

    //Loads this object's texture data.
    TextureData SceneObject::LoadTextureData(const MaterialTexture &textureResource, bool normalMap) const
    {
        if (textureResource.m_fileName.empty())
        {
            TextureData texture{};
            texture.m_width = 1;
            texture.m_height = 1;
            texture.m_channelCount = 4;
            if (normalMap)
            {
                texture.m_pixels = {
                    static_cast<uint8_t>(128), static_cast<uint8_t>(128), static_cast<uint8_t>(255), static_cast<uint8_t>(255)
                };
            }
            else
            {
                texture.m_pixels = {
                    static_cast<uint8_t>(255), static_cast<uint8_t>(255), static_cast<uint8_t>(255), static_cast<uint8_t>(255)
                };
            }
            return texture;
        }

        TextureData texture{};
        if (!LoadTexture(textureResource.m_fileName, texture))
        {
            throw std::runtime_error("Failed to load scene object texture data.");
        }

        return texture;
    }

    //Creates a Vulkan shader module.
    VkShaderModule SceneObject::CreateShaderModule(const std::vector<char> &code) const
    {
        VkShaderModuleCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = code.size();
        createInfo.pCode = reinterpret_cast<const uint32_t *>(code.data());

        VkShaderModule shaderModule = VK_NULL_HANDLE;
        if (vkCreateShaderModule(m_context.m_devicePtrGPU, &createInfo, nullptr, &shaderModule) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create scene object shader module.");
        }

        return shaderModule;
    }

    //Finds a matching memory type.
    uint32_t SceneObject::FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const
    {
        VkPhysicalDeviceMemoryProperties memoryProperties{};
        vkGetPhysicalDeviceMemoryProperties(m_context.m_physicalDevicePtrGPU, &memoryProperties);

        for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; ++i)
        {
            if ((typeFilter & (1U << i)) && (memoryProperties.memoryTypes[i].propertyFlags & properties) == properties)
            {
                return i;
            }
        }

        throw std::runtime_error("Failed to find a suitable scene object memory type.");
    }

    //Loads a compiled shader file.
    std::vector<char> SceneObject::ReadShaderFile(const std::string &relativePath) const
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

        throw std::runtime_error("Failed to read shader file: " + relativePath);
    }

    //Returns the Vulkan-safe line width for this object.
    float SceneObject::EffectiveLineWidth(bool wireframe) const
    {
        if (!m_context.m_wideLinesEnabled)
        {
            return 1.0f;
        }

        if (m_topology == VK_PRIMITIVE_TOPOLOGY_LINE_LIST)
        {
            return std::clamp(m_lineWidth, m_context.m_minimumLineWidth, m_context.m_maximumLineWidth);
        }

        if (wireframe)
        {
            const float wireframeLineWidth = m_renderOptions.m_wireframeLineWidth > 0.0f ? m_renderOptions.m_wireframeLineWidth : m_context.m_wireframeLineWidth;
            return std::clamp(wireframeLineWidth, m_context.m_minimumLineWidth, m_context.m_maximumLineWidth);
        }

        return 1.0f;
    }

    //Updates the cached model matrix.
    void SceneObject::UpdateModelMatrix(void)
    {
        m_model = multiply(translation(m_position.x, m_position.y, m_position.z), multiply(m_orientation, scale(m_scale)));
    }

    //Updates the object selection bounds.
    void SceneObject::UpdateSelectionBounds(void)
    {
        if (m_vertices.empty())
        {
            m_selectionCenter = { 0.0f, 0.0f, 0.0f };
            m_selectionRadius = 1.0f;
            return;
        }

        Float3 minimum = m_vertices.front().m_position;
        Float3 maximum = m_vertices.front().m_position;
        for (const Vertex &vertex : m_vertices)
        {
            minimum.x = std::min(minimum.x, vertex.m_position.x);
            minimum.y = std::min(minimum.y, vertex.m_position.y);
            minimum.z = std::min(minimum.z, vertex.m_position.z);
            maximum.x = std::max(maximum.x, vertex.m_position.x);
            maximum.y = std::max(maximum.y, vertex.m_position.y);
            maximum.z = std::max(maximum.z, vertex.m_position.z);
        }

        m_selectionCenter = (minimum + maximum) * 0.5f;

        float radiusSquared = 0.0f;
        for (const Vertex &vertex : m_vertices)
        {
            const Float3 offset = vertex.m_position - m_selectionCenter;
            radiusSquared = std::max(radiusSquared, dot(offset, offset));
        }

        m_selectionRadius = radiusSquared > 0.000001f ? std::sqrt(radiusSquared) : 1.0f;
    }

    //Retires one GPU buffer.
    void SceneObject::RetireBuffer(VkBuffer &bufferPtrGPU, VkDeviceMemory &bufferMemoryPtrGPU)
    {
        if (bufferPtrGPU == VK_NULL_HANDLE && bufferMemoryPtrGPU == VK_NULL_HANDLE)
        {
            return;
        }

        m_retiredBuffers.push_back(RetiredBuffer{ bufferPtrGPU, bufferMemoryPtrGPU, s_retiredBufferFrameCount });
        bufferPtrGPU = VK_NULL_HANDLE;
        bufferMemoryPtrGPU = VK_NULL_HANDLE;
    }

    //Releases retired GPU buffers.
    void SceneObject::ReleaseRetiredBuffers(bool releaseAll)
    {
        if (m_context.m_devicePtrGPU == VK_NULL_HANDLE || m_retiredBuffers.empty())
        {
            return;
        }

        for (RetiredBuffer &buffer : m_retiredBuffers)
        {
            if (!releaseAll)
            {
                --buffer.m_framesRemaining;
            }

            if (releaseAll || buffer.m_framesRemaining <= 0)
            {
                if (buffer.m_bufferPtrGPU != VK_NULL_HANDLE)
                {
                    vkDestroyBuffer(m_context.m_devicePtrGPU, buffer.m_bufferPtrGPU, nullptr);
                    buffer.m_bufferPtrGPU = VK_NULL_HANDLE;
                }

                if (buffer.m_bufferMemoryPtrGPU != VK_NULL_HANDLE)
                {
                    vkFreeMemory(m_context.m_devicePtrGPU, buffer.m_bufferMemoryPtrGPU, nullptr);
                    buffer.m_bufferMemoryPtrGPU = VK_NULL_HANDLE;
                }
            }
        }

        m_retiredBuffers.erase(
            std::remove_if(
                m_retiredBuffers.begin(),
                m_retiredBuffers.end(),
                [](const RetiredBuffer &buffer)
                {
                    return buffer.m_bufferPtrGPU == VK_NULL_HANDLE && buffer.m_bufferMemoryPtrGPU == VK_NULL_HANDLE;
                }
            ),
            m_retiredBuffers.end()
        );
    }

    //Returns a compiled SPIR-V shader path.
    std::string SceneObject::ShaderPath(const std::string &shaderName)
    {
        const std::filesystem::path shaderPath(shaderName);
        if (!shaderPath.has_parent_path())
        {
            throw std::runtime_error("Shader file names must include a folder.");
        }

        if (shaderPath.extension() == ".spv")
        {
            return shaderName;
        }

        return shaderName + ".spv";
    }

    //Returns the number of mip levels for a texture size.
    uint32_t SceneObject::MipLevelsForTexture(uint32_t width, uint32_t height)
    {
        uint32_t largestDimension = std::max(width, height);
        uint32_t mipLevels = 1U;

        while (largestDimension > 1U)
        {
            largestDimension /= 2U;
            ++mipLevels;
        }

        return mipLevels;
    }

    //Initializes the group from scene object children.
    bool SceneObjectGroup::Init(
        const std::string &name,
        const std::vector<SceneObject *> &sceneObjectPtrs,
        const Float3 &position,
        const Float4x4 &orientation,
        float scaleValue,
        const Float3 *localCenterPtr
    )
    {
        m_name = name;
        m_position = position;
        m_orientation = orientation;
        m_scale = std::max(0.001f, scaleValue);
        m_selectionRadius = 1.0f;
        m_localCenter = { 0.0f, 0.0f, 0.0f };
        m_children.clear();

        bool hasBounds = false;
        Float3 boundsMinimum{ 0.0f, 0.0f, 0.0f };
        Float3 boundsMaximum{ 0.0f, 0.0f, 0.0f };

        auto IncludeBounds = [&](const Float3 &center, float radius)
        {
            const Float3 minimum{ center.x - radius, center.y - radius, center.z - radius };
            const Float3 maximum{ center.x + radius, center.y + radius, center.z + radius };
            if (!hasBounds)
            {
                boundsMinimum = minimum;
                boundsMaximum = maximum;
                hasBounds = true;
                return;
            }

            boundsMinimum.x = std::min(boundsMinimum.x, minimum.x);
            boundsMinimum.y = std::min(boundsMinimum.y, minimum.y);
            boundsMinimum.z = std::min(boundsMinimum.z, minimum.z);
            boundsMaximum.x = std::max(boundsMaximum.x, maximum.x);
            boundsMaximum.y = std::max(boundsMaximum.y, maximum.y);
            boundsMaximum.z = std::max(boundsMaximum.z, maximum.z);
        };

        for (const SceneObject *sceneObjectPtr : sceneObjectPtrs)
        {
            if (sceneObjectPtr == nullptr)
            {
                continue;
            }

            if (sceneObjectPtr->InstancingEnabled())
            {
                for (uint32_t instanceIndex = 0; instanceIndex < sceneObjectPtr->InstanceCount(); ++instanceIndex)
                {
                    IncludeBounds(sceneObjectPtr->InstanceSelectionCenter(instanceIndex), sceneObjectPtr->InstanceSelectionRadius(instanceIndex));
                }
                continue;
            }

            IncludeBounds(sceneObjectPtr->SelectionCenter(), sceneObjectPtr->SelectionRadius());
        }

        if (!hasBounds)
        {
            return false;
        }

        m_localCenter = localCenterPtr != nullptr ? *localCenterPtr : (boundsMinimum + boundsMaximum) * 0.5f;
        m_children.reserve(sceneObjectPtrs.size());
        for (SceneObject *sceneObjectPtr : sceneObjectPtrs)
        {
            if (sceneObjectPtr == nullptr)
            {
                continue;
            }

            SceneObjectGroupChild childData;
            childData.m_objectPtr = sceneObjectPtr;
            childData.m_localPosition = sceneObjectPtr->Position() - m_localCenter;
            childData.m_localOrientation = sceneObjectPtr->Orientation();
            childData.m_localScale = sceneObjectPtr->Scale();
            m_children.push_back(childData);
        }

        for (const SceneObject *sceneObjectPtr : sceneObjectPtrs)
        {
            if (sceneObjectPtr == nullptr)
            {
                continue;
            }

            if (sceneObjectPtr->InstancingEnabled())
            {
                for (uint32_t instanceIndex = 0; instanceIndex < sceneObjectPtr->InstanceCount(); ++instanceIndex)
                {
                    const Float3 offset = sceneObjectPtr->InstanceSelectionCenter(instanceIndex) - m_localCenter;
                    m_selectionRadius = std::max(m_selectionRadius, std::sqrt(dot(offset, offset)) + sceneObjectPtr->InstanceSelectionRadius(instanceIndex));
                }
                continue;
            }

            const Float3 offset = sceneObjectPtr->SelectionCenter() - m_localCenter;
            m_selectionRadius = std::max(m_selectionRadius, std::sqrt(dot(offset, offset)) + sceneObjectPtr->SelectionRadius());
        }

        return !m_children.empty();
    }

    //Returns a group local point in world space.
    Float3 SceneObjectGroup::LocalToWorld(const Float3 &localPosition) const
    {
        const Float3 scaledPosition = localPosition * m_scale;
        const Float4 rotatedPosition = transform(m_orientation, { scaledPosition.x, scaledPosition.y, scaledPosition.z, 0.0f });
        return m_position + Float3{ rotatedPosition.x, rotatedPosition.y, rotatedPosition.z };
    }

    //Applies this group's transform to its children.
    void SceneObjectGroup::Apply(void)
    {
        for (const SceneObjectGroupChild &childData : m_children)
        {
            if (childData.m_objectPtr == nullptr)
            {
                continue;
            }

            childData.m_objectPtr->SetPosition(LocalToWorld(childData.m_localPosition));
            childData.m_objectPtr->SetOrientation(multiply(m_orientation, childData.m_localOrientation));
            childData.m_objectPtr->SetScale(m_scale * childData.m_localScale);
        }
    }

    //Returns the selected group debug popup message.
    std::string SceneObjectGroup::PopupMessage(void) const
    {
        const Float3 rotationDegrees = rotationDegreesFromMatrix(m_orientation);
        return
            "Name: " + m_name + "\n" + "Group Children: " + std::to_string(m_children.size()) + "\n" + "Position: " + StringUtility::FormatFloat3(m_position) + "\n" +
            "Rotation XYZ Degrees: " + StringUtility::FormatFloat3(rotationDegrees) + "\n" + "Scale: " + StringUtility::FormatFloat(m_scale);
    }

    //Returns a matrix position component.
    Float3 SceneObject::MatrixPosition(const Float4x4 &matrix)
    {
        return { matrix.m[12], matrix.m[13], matrix.m[14] };
    }

    //Returns a matrix uniform scale component.
    float SceneObject::MatrixScale(const Float4x4 &matrix)
    {
        const Float3 xAxis{ matrix.m[0], matrix.m[1], matrix.m[2] };
        const float scaleValue = std::sqrt(dot(xAxis, xAxis));
        return std::max(0.001f, scaleValue);
    }

    //Returns a matrix orientation component.
    Float4x4 SceneObject::MatrixOrientation(const Float4x4 &matrix)
    {
        const float scaleValue = MatrixScale(matrix);
        Float4x4 orientation = identity();

        orientation.m[0] = matrix.m[0] / scaleValue;
        orientation.m[1] = matrix.m[1] / scaleValue;
        orientation.m[2] = matrix.m[2] / scaleValue;
        orientation.m[4] = matrix.m[4] / scaleValue;
        orientation.m[5] = matrix.m[5] / scaleValue;
        orientation.m[6] = matrix.m[6] / scaleValue;
        orientation.m[8] = matrix.m[8] / scaleValue;
        orientation.m[9] = matrix.m[9] / scaleValue;
        orientation.m[10] = matrix.m[10] / scaleValue;

        return orientation;
    }

    //Returns a model matrix from transform values.
    Float4x4 SceneObject::ModelMatrixFromTransform(const Float3 &position, const Float4x4 &orientation, float scaleValue)
    {
        return multiply(translation(position.x, position.y, position.z), multiply(orientation, scale(std::max(0.001f, scaleValue))));
    }
}
