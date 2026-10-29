///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	SceneObject.h
//	Description	:
//		Defines the base scene object and shared context.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "global.h"

#include "RenderTypes.h"
#include "TextureUtility.h"

#include <vulkan/vulkan.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace graphicsFX
{
    //Stores Vulkan resources shared by scene objects.
    struct SceneObjectContext
    {
        VkDevice m_devicePtrGPU{ VK_NULL_HANDLE }; //Stores the Vulkan logical device.
        VkPhysicalDevice m_physicalDevicePtrGPU{ VK_NULL_HANDLE }; //Stores the Vulkan physical device.
        VkCommandPool m_commandPoolPtrGPU{ VK_NULL_HANDLE }; //Stores the command pool for copy commands.
        VkQueue m_graphicsQueuePtrGPU{ VK_NULL_HANDLE }; //Stores the graphics queue for copy commands.
        VkRenderPass m_renderPassPtrGPU{ VK_NULL_HANDLE }; //Stores the render pass used by pipelines.
        VkRenderPass m_shadowRenderPassPtrGPU{ VK_NULL_HANDLE }; //Stores the shadow map render pass.
        VkDescriptorSetLayout m_descriptorSetLayoutPtrGPU{ VK_NULL_HANDLE }; //Stores the camera descriptor layout.
        VkDescriptorSetLayout m_textureDescriptorSetLayoutPtrGPU{ VK_NULL_HANDLE }; //Stores the texture descriptor layout.
        VkDescriptorPool m_descriptorPoolPtrGPU{ VK_NULL_HANDLE }; //Stores the descriptor pool for object textures.
        VkExtent2D m_swapChainExtent{}; //Stores the active swapchain size.
        VkExtent2D m_shadowExtent{}; //Stores the shadow map size.
        bool m_wideLinesEnabled{ false }; //Stores whether wide lines are available.
        bool m_anisotropyEnabled{ false }; //Stores whether sampler anisotropy is available.
        bool m_wireframeEnabled{ false }; //Stores whether wireframe polygon mode is available.
        bool m_largePointsEnabled{ false }; //Stores whether large point sprites are available.
        bool m_shaderFloat64Enabled{ false }; //Stores whether double-precision shader arithmetic is enabled.
        float m_minimumLineWidth{ 1.0f }; //Stores the minimum supported line width.
        float m_maximumLineWidth{ 1.0f }; //Stores the maximum supported line width.
        float m_minimumPointSize{ 1.0f }; //Stores the minimum supported point size.
        float m_maximumPointSize{ 1.0f }; //Stores the maximum supported point size.
        float m_wireframeLineWidth{ 2.0f }; //Stores the requested wireframe line width.
        float m_maximumAnisotropy{ 1.0f }; //Stores the maximum sampler anisotropy.
        VkSampleCountFlagBits m_sampleCount{ VK_SAMPLE_COUNT_1_BIT }; //Stores the scene pipeline sample count.
        bool m_shaderOverrideEnabled{ false }; //Stores whether capture shaders override object shaders.
        std::string m_vertexShaderOverride; //Stores the non-instanced vertex shader override.
        std::string m_instancedVertexShaderOverride; //Stores the instanced vertex shader override.
        std::string m_fragmentShaderOverride; //Stores the fragment shader override.
    };

    //Stores scene object shader file names.
    struct SceneObjectShaderFiles
    {
        std::string m_vertexShader; //Stores the vertex shader file name.
        std::string m_fragmentShader; //Stores the fragment shader file name.
    };

    //Stores how a scene object can be selected.
    enum class SceneObjectSelectionMode
    {
        None, PerObject, PerInstance
    };

    //Stores optional scene object render behavior.
    struct SceneObjectRenderOptions
    {
        bool m_castShadow{ true }; //Stores whether this object casts shadows.
        bool m_receiveShadow{ true }; //Stores whether this object receives shadows.
        bool m_wireframe{ false }; //Stores whether this object draws in wireframe mode.
        bool m_clickable{ false }; //Stores whether this object can be selected.
        SceneObjectSelectionMode m_selectionMode{ SceneObjectSelectionMode::None }; //Stores object selection behavior.
        bool m_alphaBlend{ false }; //Stores whether alpha blending is enabled.
        bool m_depthTest{ true }; //Stores whether depth testing is enabled.
        float m_wireframeLineWidth{ 0.0f }; //Stores the optional per-object wireframe line width.
        float m_pointSize{ 1.0f }; //Stores the point primitive size.
        float m_maxPointSize{ 0.0f }; //Stores the optional maximum point primitive size.
    };

    //Defines shared scene object GPU resource behavior.
    class SceneObject
    {

        public:

            SceneObject(void) = default;
            virtual ~SceneObject() = default;

            SceneObject(const SceneObject &) = delete;
            SceneObject &operator=(const SceneObject &) = delete;

            //Initializes GPU resources for this object.
            virtual void Init(const SceneObjectContext &context);

            //Updates this object's transform.
            virtual void Update(float elapsedSeconds, float deltaSeconds);

            //Draws this object.
            virtual void Show(VkCommandBuffer commandBuffer, VkDescriptorSet cameraDescriptorSet) const;

            //Draws this object with a render mode.
            virtual void Show(VkCommandBuffer commandBuffer, VkDescriptorSet cameraDescriptorSet, bool wireframe) const;

            //Draws this object with an explicit render extent.
            virtual void Show(VkCommandBuffer commandBuffer, VkDescriptorSet cameraDescriptorSet, VkExtent2D renderExtent) const;

            //Draws this object with an explicit render extent and mode.
            virtual void Show(VkCommandBuffer commandBuffer, VkDescriptorSet cameraDescriptorSet, VkExtent2D renderExtent, bool wireframe) const;

            //Draws this object into the shadow map.
            virtual void ShowShadow(VkCommandBuffer commandBuffer, VkDescriptorSet cameraDescriptorSet, VkExtent2D shadowExtent) const;

            //Releases all GPU resources for this object.
            virtual void Clean(void);

            //Releases only the graphics pipeline.
            void CleanPipeline(void);

            //Recreates the pipeline after swapchain changes.
            void RefreshPipeline(const SceneObjectContext &context);

            //Sets this object's initial orientation.
            void SetOrientation(const Float4x4 &orientation);

            //Sets this object's position.
            void SetPosition(const Float3 &position);

            //Sets this object's scale.
            void SetScale(float scale);

            //Sets whether this object can be selected.
            void SetClickable(bool clickable);

            //Sets this object's selection mode.
            void SetSelectionMode(SceneObjectSelectionMode selectionMode);

            //Sets this object's debug name.
            void SetName(const std::string &name);

            //Returns this object's position.
            const Float3 &Position(void) const;

            //Returns this object's debug name.
            const std::string &Name(void) const;

            //Returns this object's orientation.
            const Float4x4 &Orientation(void) const;

            //Returns this object's scale.
            float Scale(void) const;

            //Returns whether this object can be selected.
            bool Clickable(void) const;

            //Returns this object's selection mode.
            SceneObjectSelectionMode SelectionMode(void) const;

            //Returns this object's selection radius.
            float SelectionRadius(void) const;

            //Returns this object's selection center.
            Float3 SelectionCenter(void) const;

            //Returns whether this object uses instancing.
            bool InstancingEnabled(void) const;

            //Returns this object's instance count.
            uint32_t InstanceCount(void) const;

            //Returns one instance position.
            Float3 InstancePosition(uint32_t instanceIndex) const;

            //Returns one instance orientation.
            Float4x4 InstanceOrientation(uint32_t instanceIndex) const;

            //Returns one instance scale.
            float InstanceScale(uint32_t instanceIndex) const;

            //Returns one instance selection radius.
            float InstanceSelectionRadius(uint32_t instanceIndex) const;

            //Returns one instance selection center.
            Float3 InstanceSelectionCenter(uint32_t instanceIndex) const;

            //Sets one instance transform.
            void SetInstanceTransform(uint32_t instanceIndex, const Float3 &position, const Float4x4 &orientation, float scaleValue);

        protected:

            //Stores 16-bit mesh and object settings in the base object.
            void SetMesh(
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
            );

            //Stores 32-bit mesh and object settings in the base object.
            void SetMesh(
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
            );

            //Stores per-instance transform and color values.
            void SetInstances(std::vector<SceneObjectInstanceData> instances);

            //Updates 32-bit mesh indices in the base object.
            void UpdateIndices(std::vector<uint32_t> indices);

            //Updates 32-bit mesh vertices and indices in the base object.
            void UpdateMesh(std::vector<Vertex> vertices, std::vector<uint32_t> indices);

            //Checks whether a value fits in uint16_t.
            static uint16_t CheckedU16(std::size_t value);

            //Stores the optional texture file name.
            void SetTextureFileName(const std::string &textureFileName);

            //Stores the optional normal map file name.
            void SetNormalMapFileName(const std::string &normalMapFileName);

            //Stores the shader file names.
            void SetShaderFileNames(const std::string &vertexShader, const std::string &fragmentShader);

        private:

            //Stores one material texture resource.
            struct MaterialTexture
            {
                std::string m_fileName; //Stores the texture file name.
                VkImage m_imagePtrGPU{ VK_NULL_HANDLE }; //Stores the texture image.
                VkDeviceMemory m_imageMemoryPtrGPU{ VK_NULL_HANDLE }; //Stores texture image memory.
                VkImageView m_imageViewPtrGPU{ VK_NULL_HANDLE }; //Stores the texture image view.
                VkSampler m_samplerPtrGPU{ VK_NULL_HANDLE }; //Stores the texture sampler.
                uint32_t m_mipLevels{ 1 }; //Stores the number of texture mip levels.
            };

            //Stores one retired GPU buffer waiting for in-flight frames to finish.
            struct RetiredBuffer
            {
                VkBuffer m_bufferPtrGPU{ VK_NULL_HANDLE }; //Stores the retired Vulkan buffer.
                VkDeviceMemory m_bufferMemoryPtrGPU{ VK_NULL_HANDLE }; //Stores the retired Vulkan buffer memory.
                int m_framesRemaining{ 0 }; //Stores remaining update frames before release.
            };

            static constexpr int s_retiredBufferFrameCount = 4; //Stores how long replaced buffers are kept alive.

            //Creates this object's graphics pipeline.
            void CreatePipeline(void);

            //Creates this object's shadow map graphics pipeline.
            void CreateShadowPipeline(void);

            //Creates this object's vertex buffer.
            void CreateVertexBuffer(void);

            //Creates this object's index buffer.
            void CreateIndexBuffer(void);

            //Creates this object's instance buffer.
            void CreateInstanceBuffer(void);

            //Creates this object's texture resources.
            void CreateTextureResources(void);

            //Creates this object's texture image.
            void CreateTextureImage(MaterialTexture &textureResource, const TextureData &texture);

            //Creates this object's texture image view.
            void CreateTextureImageView(MaterialTexture &textureResource);

            //Creates this object's texture sampler.
            void CreateTextureSampler(MaterialTexture &textureResource);

            //Creates this object's texture descriptor set.
            void CreateTextureDescriptorSet(void);

            //Releases one texture resource.
            void CleanTexture(MaterialTexture &textureResource);

            //Releases this object's instance buffer.
            void CleanInstanceBuffer(void);

            //Updates this object's instance buffer data.
            void UpdateInstanceBuffer(void);

            //Normalizes clickable and selection mode settings.
            void NormalizeRenderOptions(void);

            //Releases only the shadow graphics pipeline.
            void CleanShadowPipeline(void);

            //Creates a Vulkan buffer and memory.
            void CreateBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer &buffer, VkDeviceMemory &bufferMemory) const;

            //Copies one Vulkan buffer into another.
            void CopyBuffer(VkBuffer source, VkBuffer destination, VkDeviceSize size) const;

            //Copies one Vulkan buffer into an image.
            void CopyBufferToImage(VkBuffer source, VkImage destinationPtrGPU, uint32_t width, uint32_t height) const;

            //Transitions a Vulkan image layout.
            void TransitionImageLayout(VkImage imagePtrGPU, VkImageLayout oldLayout, VkImageLayout newLayout, uint32_t mipLevels) const;

            //Generates mipmap levels for a texture image.
            void GenerateMipmaps(VkImage imagePtrGPU, int32_t width, int32_t height, uint32_t mipLevels) const;

            //Begins a one-time command buffer.
            VkCommandBuffer BeginSingleTimeCommands(void) const;

            //Ends a one-time command buffer.
            void EndSingleTimeCommands(VkCommandBuffer commandBufferPtrGPU) const;

            //Loads this object's texture data.
            TextureData LoadTextureData(const MaterialTexture &textureResource, bool normalMap) const;

            //Creates a Vulkan shader module.
            VkShaderModule CreateShaderModule(const std::vector<char> &code) const;

            //Finds a matching memory type.
            uint32_t FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const;

            //Loads a compiled shader file.
            std::vector<char> ReadShaderFile(const std::string &relativePath) const;

            //Updates the cached model matrix.
            void UpdateModelMatrix(void);

            //Updates the object selection bounds.
            void UpdateSelectionBounds(void);

            //Retires one GPU buffer.
            void RetireBuffer(VkBuffer &bufferPtrGPU, VkDeviceMemory &bufferMemoryPtrGPU);

            //Releases retired GPU buffers.
            void ReleaseRetiredBuffers(bool releaseAll);

            //Returns the Vulkan-safe line width for this object.
            float EffectiveLineWidth(bool wireframe) const;

            //Returns a compiled SPIR-V shader path.
            static std::string ShaderPath(const std::string &shaderName);

            //Returns the number of mip levels for a texture size.
            static uint32_t MipLevelsForTexture(uint32_t width, uint32_t height);

            //Returns a matrix position component.
            static Float3 MatrixPosition(const Float4x4 &matrix);

            //Returns a matrix uniform scale component.
            static float MatrixScale(const Float4x4 &matrix);

            //Returns a matrix orientation component.
            static Float4x4 MatrixOrientation(const Float4x4 &matrix);

            //Returns a model matrix from transform values.
            static Float4x4 ModelMatrixFromTransform(const Float3 &position, const Float4x4 &orientation, float scaleValue);

            SceneObjectContext m_context{}; //Stores the Vulkan resources shared by the scene.
            std::vector<Vertex> m_vertices; //Stores mesh vertices.
            std::vector<SceneObjectInstanceData> m_instances; //Stores per-instance data.
            std::vector<uint16_t> m_indices16; //Stores 16-bit mesh indices.
            std::vector<uint32_t> m_indices32; //Stores 32-bit mesh indices.
            std::vector<RetiredBuffer> m_retiredBuffers; //Stores replaced GPU buffers waiting for release.
            std::string m_name{ "SceneObject" }; //Stores the object debug name.
            uint32_t m_indexCount{ 0 }; //Stores the number of mesh indices.
            uint32_t m_instanceCount{ 0 }; //Stores the number of object instances.
            VkIndexType m_indexType{ VK_INDEX_TYPE_UINT32 }; //Stores the Vulkan index type.
            bool m_instancingEnabled{ false }; //Stores whether instanced drawing is enabled.
            std::string m_vertexShader{ "shaders/default.vert" }; //Stores the vertex shader file name.
            std::string m_fragmentShader{ "shaders/default.frag" }; //Stores the fragment shader file name.
            MaterialTexture m_diffuseTexture; //Stores the diffuse texture resource.
            MaterialTexture m_normalTexture; //Stores the normal map texture resource.
            VkPrimitiveTopology m_topology{ VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST }; //Stores how vertices are assembled.
            bool m_depthWrite{ true }; //Stores whether this object writes depth.
            SceneObjectRenderOptions m_renderOptions{}; //Stores optional render behavior.
            float m_lineWidth{ 1.0f }; //Stores the line width for line drawing.
            Float3 m_position{}; //Stores the object position.
            float m_scale{ 1.0f }; //Stores the object scale.
            Float3 m_selectionCenter{}; //Stores the local selection center.
            float m_selectionRadius{ 1.0f }; //Stores the local selection radius.
            Float4 m_color{ 1.0f, 1.0f, 1.0f, 1.0f }; //Stores the object color.
            Float4x4 m_orientation{ identity() }; //Stores the static object orientation.
            Float4x4 m_model{ identity() }; //Stores the model transform.

            VkPipelineLayout m_pipelineLayoutPtrGPU{ VK_NULL_HANDLE }; //Stores the pipeline layout.
            VkPipeline m_pipelinePtrGPU{ VK_NULL_HANDLE }; //Stores the graphics pipeline.
            VkPipeline m_wireframePipelinePtrGPU{ VK_NULL_HANDLE }; //Stores the wireframe graphics pipeline.
            VkPipelineLayout m_shadowPipelineLayoutPtrGPU{ VK_NULL_HANDLE }; //Stores the shadow pipeline layout.
            VkPipeline m_shadowPipelinePtrGPU{ VK_NULL_HANDLE }; //Stores the shadow graphics pipeline.
            VkBuffer m_vertexBufferPtrGPU{ VK_NULL_HANDLE }; //Stores the vertex buffer.
            VkDeviceMemory m_vertexBufferMemoryPtrGPU{ VK_NULL_HANDLE }; //Stores vertex buffer memory.
            VkBuffer m_indexBufferPtrGPU{ VK_NULL_HANDLE }; //Stores the index buffer.
            VkDeviceMemory m_indexBufferMemoryPtrGPU{ VK_NULL_HANDLE }; //Stores index buffer memory.
            VkBuffer m_instanceBufferPtrGPU{ VK_NULL_HANDLE }; //Stores the instance buffer.
            VkDeviceMemory m_instanceBufferMemoryPtrGPU{ VK_NULL_HANDLE }; //Stores instance buffer memory.
            VkDescriptorSet m_textureDescriptorSetPtrGPU{ VK_NULL_HANDLE }; //Stores the texture descriptor set.
    };

    //Stores one scene object child inside a group.
    struct SceneObjectGroupChild
    {
        SceneObject *m_objectPtr{ nullptr }; //Stores the child scene object pointer.
        Float3 m_localPosition{ 0.0f, 0.0f, 0.0f }; //Stores the child local position.
        Float4x4 m_localOrientation{ identity() }; //Stores the child local orientation.
        float m_localScale{ 1.0f }; //Stores the child local scale.
    };

    //Stores a selectable group of scene objects.
    struct SceneObjectGroup
    {
        std::string m_name; //Stores the group name.
        Float3 m_position{ 0.0f, 0.0f, 0.0f }; //Stores the group position.
        Float4x4 m_orientation{ identity() }; //Stores the group orientation.
        float m_scale{ 1.0f }; //Stores the group scale.
        float m_selectionRadius{ 1.0f }; //Stores the group selection radius.
        Float3 m_localCenter{ 0.0f, 0.0f, 0.0f }; //Stores the original group local center.
        std::vector<SceneObjectGroupChild> m_children; //Stores grouped scene object children.

        //Initializes the group from scene object children.
        bool Init(const std::string &name, const std::vector<SceneObject *> &sceneObjectPtrs, const Float3 &position, const Float4x4 &orientation, float scale, const Float3 *localCenterPtr = nullptr);

        //Returns a group local point in world space.
        Float3 LocalToWorld(const Float3 &localPosition) const;

        //Applies this group's transform to its children.
        void Apply(void);

        //Returns the selected group debug popup message.
        std::string PopupMessage(void) const;
    };
}
