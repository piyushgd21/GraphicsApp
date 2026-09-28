///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	EngineCore.h
//	Description	:
//		Defines the engine core class.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "global.h"

#include "GraphicsUtility.h"
#include "InputUtility.h"
#include "MathUtility.h"
#include "OpenXRUtility.h"
#include "RenderTypes.h"
#include "SceneInterface.h"
#include "SetupUtility.h"
#include "TextureUtility.h"
#include "UIUtility.h"
#include "Window.h"

#include <vulkan/vulkan.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace graphicsFX
{
    //Defines the main graphics engine core.
    class EngineCore
    {

        public:

            //Creates the engine core and initializes graphics.
            EngineCore(HINSTANCE instancePtr, SceneInterface &scene, const std::string &setupFileName);

            //Destroys the engine core.
            ~EngineCore();

            EngineCore(const EngineCore &) = delete;
            EngineCore &operator=(const EngineCore &) = delete;

            //Runs the application loop.
            void Run(void);

        private:

            //Stores one-shot frame capture GPU resources.
            struct FrameCaptureResources
            {
                VkRenderPass m_renderPassPtrGPU{ VK_NULL_HANDLE }; //Stores the capture render pass.
                VkFramebuffer m_framebufferPtrGPU{ VK_NULL_HANDLE }; //Stores the capture framebuffer.
                VkImage m_colorImagePtrGPU{ VK_NULL_HANDLE }; //Stores the capture color image.
                VkDeviceMemory m_colorImageMemoryPtrGPU{ VK_NULL_HANDLE }; //Stores capture color image memory.
                VkImageView m_colorImageViewPtrGPU{ VK_NULL_HANDLE }; //Stores the capture color image view.
                VkImage m_depthImagePtrGPU{ VK_NULL_HANDLE }; //Stores the capture depth image.
                VkDeviceMemory m_depthImageMemoryPtrGPU{ VK_NULL_HANDLE }; //Stores capture depth image memory.
                VkImageView m_depthImageViewPtrGPU{ VK_NULL_HANDLE }; //Stores the capture depth image view.
                VkBuffer m_colorReadBufferPtrGPU{ VK_NULL_HANDLE }; //Stores the color readback buffer.
                VkDeviceMemory m_colorReadBufferMemoryPtrGPU{ VK_NULL_HANDLE }; //Stores color readback memory.
                VkBuffer m_depthReadBufferPtrGPU{ VK_NULL_HANDLE }; //Stores the depth readback buffer.
                VkDeviceMemory m_depthReadBufferMemoryPtrGPU{ VK_NULL_HANDLE }; //Stores depth readback memory.
                VkFormat m_colorFormat{}; //Stores the capture color image format.
                VkFormat m_depthFormat{}; //Stores the capture depth image format.
                VkDeviceSize m_colorByteCount{ 0 }; //Stores color readback byte count.
                VkDeviceSize m_depthByteCount{ 0 }; //Stores depth readback byte count.
            };

            //Stores one shadow map image set.
            struct ShadowMapResources
            {
                VkFramebuffer m_framebufferPtrGPU{ VK_NULL_HANDLE }; //Stores the shadow framebuffer.
                VkImage m_depthImagePtrGPU{ VK_NULL_HANDLE }; //Stores the shadow depth image.
                VkDeviceMemory m_depthImageMemoryPtrGPU{ VK_NULL_HANDLE }; //Stores shadow depth image memory.
                VkImageView m_depthImageViewPtrGPU{ VK_NULL_HANDLE }; //Stores the shadow depth image view.
            };

            //Stores a world-space camera basis.
            struct CameraBasis
            {
                Float3 m_position; //Stores world position.
                Float3 m_right; //Stores world right direction.
                Float3 m_up; //Stores world up direction.
                Float3 m_back; //Stores world back direction.
            };

            //Initializes graphics resources.
            void InitGraphics(void);

            //Runs the message and render loop.
            void MainLoop(void);

            //Releases application resources.
            void Cleanup(void);

            //Creates the swapchain.
            void CreateSwapChain(void);

            //Creates swapchain image views.
            void CreateImageViews(void);

            //Creates the render pass.
            void CreateRenderPass(void);

            //Creates the UI overlay render pass.
            void CreateUIRenderPass(void);

            //Creates descriptor set layouts.
            void CreateDescriptorSetLayout(void);

            //Creates the multisampled color image resources.
            void CreateColorResources(void);

            //Creates the depth image resources.
            void CreateDepthResources(void);

            //Creates shadow map resources.
            void CreateShadowResources(void);

            //Creates the shadow map render pass.
            void CreateShadowRenderPass(void);

            //Creates one shadow map image set.
            void CreateShadowMapResources(ShadowMapResources &resources);

            //Creates the shadow map sampler.
            void CreateShadowSampler(void);

            //Creates swapchain framebuffers.
            void CreateFramebuffers(void);

            //Creates UI overlay framebuffers.
            void CreateUIFramebuffers(void);

            //Creates camera uniform buffers.
            void CreateUniformBuffers(void);

            //Creates the descriptor pool.
            void CreateDescriptorPool(void);

            //Creates descriptor sets.
            void CreateDescriptorSets(void);

            //Creates command buffers.
            void CreateCommandBuffers(void);

            //Creates frame synchronization objects.
            void CreateSyncObjects(void);

            //Creates scene objects.
            void CreateScene(void);

            //Releases swapchain resources.
            void CleanupSwapChain(void);

            //Releases shadow map resources.
            void CleanShadowResources(void);

            //Releases one shadow map image set.
            void CleanShadowMapResources(ShadowMapResources &resources) const;

            //Rebuilds the swapchain after resize.
            void RecreateSwapChain(void);

            //Draws one frame.
            void DrawFrame(void);

            //Draws one XR frame.
            void DrawXRFrame(void);

            //Captures the frame when the trigger key is pressed.
            void CaptureFrameOnInput(void);

            //Returns the camera movement basis for input.
            const CameraMovementBasis *InputMovementBasis(void) const;

            //Updates the camera uniform buffer.
            void UpdateUniformBuffer(uint32_t currentImage);

            //Updates the camera uniform buffer with explicit values.
            void UpdateUniformBuffer(uint32_t currentImage, const CameraBuffer &camera);

            //Updates the XR environment depth descriptor.
            void UpdateXREnvironmentDepthDescriptor(uint32_t currentImage, const OpenXRUtility::EyeRenderInfo &renderInfo);

            //Returns the camera buffer for one XR eye.
            CameraBuffer XRCameraBuffer(const XrView &view) const;

            //Returns the camera buffer for one XR eye render.
            CameraBuffer XRCameraBuffer(const OpenXRUtility::EyeRenderInfo &renderInfo) const;

            //Returns the camera buffer for XR center-eye desktop mirror.
            CameraBuffer XRCenterEyeCameraBuffer(const std::vector<XrView> &views, float aspect) const;

            //Returns whether desktop should mirror the XR center eye.
            bool XRDesktopCenterEyeMirrorEnabled(void) const;

            //Records one frame command buffer.
            void RecordCommandBuffer(VkCommandBuffer commandBufferPtrGPU, uint32_t imageIndex);

            //Records the shadow map pass.
            void RecordShadowMap(VkCommandBuffer commandBufferPtrGPU);

            //Captures and saves one scene frame.
            void SaveFrameBuffers(void);

            //Creates one-shot frame capture resources.
            void CreateFrameCaptureResources(FrameCaptureResources &resources);

            //Releases one-shot frame capture resources.
            void CleanFrameCaptureResources(FrameCaptureResources &resources) const;

            //Records one frame capture command buffer.
            void RecordFrameCaptureCommandBuffer(VkCommandBuffer commandBufferPtrGPU, const FrameCaptureResources &resources, bool copyDepthBuffer);

            //Transitions an image layout inside a command buffer.
            void TransitionImageLayout(VkCommandBuffer commandBufferPtrGPU, VkImage imagePtrGPU, VkImageLayout oldLayout, VkImageLayout newLayout, VkImageAspectFlags aspectFlags) const;

            //Copies an image into a readback buffer.
            void CopyImageToBuffer(VkCommandBuffer commandBufferPtrGPU, VkImage imagePtrGPU, VkImageLayout imageLayout, VkImageAspectFlags aspectFlags, VkBuffer bufferPtrGPU) const;

            //Submits a finished one-shot command buffer.
            void SubmitSingleUseCommandBuffer(VkCommandBuffer commandBufferPtrGPU) const;

            //Reads the capture color buffer as texture pixels.
            TextureData ReadColorCapture(const FrameCaptureResources &resources) const;

            //Reads the capture depth buffer as texture pixels.
            TextureData ReadDepthCapture(const FrameCaptureResources &resources) const;

            //Returns an output file path under the solution folder.
            std::string OutputFilePath(const std::string &fileName) const;

            //Creates a context for frame capture rendering.
            SceneObjectContext FrameCaptureContextInfo(VkRenderPass renderPassPtrGPU, bool normalCapture) const;

            //Chooses the swapchain surface format.
            VkSurfaceFormatKHR ChooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR> &formats) const;

            //Chooses the present mode.
            VkPresentModeKHR ChooseSwapPresentMode(const std::vector<VkPresentModeKHR> &presentModes) const;

            //Chooses the swapchain extent.
            VkExtent2D ChooseSwapExtent(const VkSurfaceCapabilitiesKHR &capabilities) const;

            //Creates a context for scene objects.
            SceneObjectContext SceneObjectContextInfo(void) const;

            //Converts a normalized float to a byte.
            static uint8_t FloatToByte(float value);

            //Returns whether the format uses BGRA byte order.
            static bool FormatIsBgra(VkFormat format);

            //Returns whether the format uses RGBA byte order.
            static bool FormatIsRgba(VkFormat format);

            //Returns whether the depth format stores depth as a 32-bit float.
            static bool FormatIsFloatDepth(VkFormat format);

            //Returns whether the depth format stores depth as 24 normalized bits.
            static bool FormatIsDepth24Stencil8(VkFormat format);

            //Converts a normalized depth value to a grayscale byte.
            static uint8_t DepthToByte(float value);

            //Rotates a vector by an OpenXR quaternion.
            static Float3 RotateVector(const XrQuaternionf &quaternion, const Float3 &value);

            //Creates a view matrix from an OpenXR pose.
            static Float4x4 ViewFromBasis(const CameraBasis &basis);

            //Creates a camera basis from a camera buffer.
            static CameraBasis BasisFromCameraBuffer(const CameraBuffer &camera);

            //Creates a movement basis from a camera buffer.
            static CameraMovementBasis MovementBasisFromCameraBuffer(const CameraBuffer &camera);

            //Transforms a local direction by a rig basis.
            static Float3 TransformRigDirection(const CameraBasis &rigBasis, const Float3 &localDirection);

            //Transforms a local position by a rig basis.
            static Float3 TransformRigPosition(const CameraBasis &rigBasis, const Float3 &localPosition);

            //Creates a world-space XR eye basis from the shared rig.
            static CameraBasis BasisFromRigAndXrView(const CameraBasis &rigBasis, const XrView &view);

            //Creates a world-space XR basis from a rig and pose.
            static CameraBasis BasisFromRigAndXrPose(const CameraBasis &rigBasis, const XrPosef &pose);

            //Returns the XR depth debug mode value.
            static float XRDepthDebugModeValue(const std::string &debugMode);

            //Returns the XR depth value interpretation mode.
            static float XRDepthValueModeValue(const std::string &valueMode);

            //Returns the XR alpha debug mode value.
            static float XRAlphaDebugModeValue(const std::string &debugMode);

            //Returns whether XR depth debug fills the whole eye image.
            static bool XRFullScreenDepthDebugRequested(const std::string &debugMode);

            //Returns a finite XR environment depth near plane.
            static float XRDepthNearPlane(float nearPlane);

            //Returns a finite XR environment depth far plane.
            static float XRDepthFarPlane(float farPlane, float nearPlane, float fallbackFarPlane);

            //Creates a Vulkan projection matrix from an OpenXR field of view.
            static Float4x4 ProjectionFromXrFov(const XrFovf &fov, float nearPlane, float farPlane);

            static constexpr std::size_t s_maxFramesInFlight = 2; //Stores queued frame count.
            static constexpr std::size_t s_maxSceneObjects = 64; //Stores maximum texture descriptor count.
            static constexpr uint32_t s_shadowMapSize = 2048; //Stores shadow map width and height.

            ProgramSetup m_programSetup; //Stores setup values loaded from setup.ini.
            Window m_window; //Stores the application window.
            InputUtility m_inputUtility; //Stores keyboard and mouse input state.
            GraphicsUtility m_graphicsUtility; //Stores Vulkan device resources and helpers.
            OpenXRUtility m_openXRUtility; //Stores OpenXR runtime detection state.
            SceneInterface &m_scene; //Stores application scene callbacks.
            UIUtility m_uiUtility; //Stores in-graphics UI rendering.

            VkSwapchainKHR m_swapChainPtrGPU{ VK_NULL_HANDLE }; //Stores the swapchain.
            std::vector<VkImage> m_swapChainImagesPtrGPU; //Stores swapchain images.
            VkFormat m_swapChainImageFormat{}; //Stores the swapchain image format.
            VkExtent2D m_swapChainExtent{}; //Stores the swapchain size.
            std::vector<VkImageView> m_swapChainImageViewsPtrGPU; //Stores swapchain image views.
            std::vector<VkFramebuffer> m_swapChainFramebuffersPtrGPU; //Stores swapchain framebuffers.
            std::vector<VkFramebuffer> m_uiFramebuffersPtrGPU; //Stores UI overlay framebuffers.

            VkRenderPass m_renderPassPtrGPU{ VK_NULL_HANDLE }; //Stores the render pass.
            VkRenderPass m_uiRenderPassPtrGPU{ VK_NULL_HANDLE }; //Stores the UI overlay render pass.
            VkRenderPass m_shadowRenderPassPtrGPU{ VK_NULL_HANDLE }; //Stores the shadow map render pass.
            VkDescriptorSetLayout m_descriptorSetLayoutPtrGPU{ VK_NULL_HANDLE }; //Stores camera descriptor layout.
            VkDescriptorSetLayout m_textureDescriptorSetLayoutPtrGPU{ VK_NULL_HANDLE }; //Stores texture descriptor layout.
            std::vector<VkCommandBuffer> m_commandBuffersPtrGPU; //Stores command buffers.

            VkImage m_colorImagePtrGPU{ VK_NULL_HANDLE }; //Stores the multisampled color image.
            VkDeviceMemory m_colorImageMemoryPtrGPU{ VK_NULL_HANDLE }; //Stores multisampled color image memory.
            VkImageView m_colorImageViewPtrGPU{ VK_NULL_HANDLE }; //Stores the multisampled color image view.
            VkImage m_depthImagePtrGPU{ VK_NULL_HANDLE }; //Stores the depth image.
            VkDeviceMemory m_depthImageMemoryPtrGPU{ VK_NULL_HANDLE }; //Stores depth image memory.
            VkImageView m_depthImageViewPtrGPU{ VK_NULL_HANDLE }; //Stores the depth image view.
            VkSampleCountFlagBits m_sceneSampleCount{ VK_SAMPLE_COUNT_1_BIT }; //Stores the scene MSAA sample count.
            VkFormat m_shadowDepthFormat{}; //Stores the shadow map depth format.
            VkExtent2D m_shadowExtent{ s_shadowMapSize, s_shadowMapSize }; //Stores the shadow map extent.
            VkSampler m_shadowSamplerPtrGPU{ VK_NULL_HANDLE }; //Stores the shadow map sampler.
            std::vector<ShadowMapResources> m_shadowMapResources; //Stores per-frame shadow maps.

            std::vector<VkBuffer> m_uniformBuffersPtrGPU; //Stores camera uniform buffers.
            std::vector<VkDeviceMemory> m_uniformBuffersMemoryPtrGPU; //Stores uniform buffer memory.
            std::vector<void *> m_uniformBuffersMappedPtr; //Stores mapped uniform memory.

            VkDescriptorPool m_descriptorPoolPtrGPU{ VK_NULL_HANDLE }; //Stores the descriptor pool.
            std::vector<VkDescriptorSet> m_descriptorSetsPtrGPU; //Stores descriptor sets.

            std::vector<VkSemaphore> m_imageAvailableSemaphoresPtrGPU; //Stores image-available semaphores.
            std::vector<VkSemaphore> m_renderFinishedSemaphoresPtrGPU; //Stores render-finished semaphores.
            std::vector<VkFence> m_inFlightFencesPtrGPU; //Stores per-frame fences.
            uint32_t m_currentFrame{ 0 }; //Stores the active frame index.

            std::chrono::steady_clock::time_point m_startTime; //Stores app start time.
            std::chrono::steady_clock::time_point m_previousFrameTime; //Stores previous frame time.
            float m_framesPerSecond{ 0.0f }; //Stores displayed frames per second.
            float m_fpsElapsedSeconds{ 0.0f }; //Stores elapsed time used for FPS averaging.
            uint32_t m_fpsFrameCount{ 0 }; //Stores counted frames used for FPS averaging.
            CameraBuffer m_xrCenterEyeCamera{}; //Stores the XR center-eye desktop mirror camera.
            CameraMovementBasis m_xrCenterEyeMovementBasis{}; //Stores the XR center-eye input movement basis.
            bool m_xrCenterEyeCameraValid{ false }; //Stores whether the XR center-eye camera is valid.
    };
}

