///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	OpenXRUtility.h
//	Description	:
//		Defines OpenXR runtime, session, and stereo display helpers.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "global.h"

#include "MathUtility.h"
#include "RenderTypes.h"
#include "SetupUtility.h"

#include <vulkan/vulkan.h>

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#ifndef XR_FB_passthrough
#define XR_FB_passthrough 1
#define XR_FB_passthrough_SPEC_VERSION 5
#define XR_FB_PASSTHROUGH_EXTENSION_NAME "XR_FB_passthrough"
#define XR_TYPE_PASSTHROUGH_CREATE_INFO_FB static_cast<XrStructureType>(1000118001)
#define XR_TYPE_PASSTHROUGH_LAYER_CREATE_INFO_FB static_cast<XrStructureType>(1000118002)
#define XR_TYPE_COMPOSITION_LAYER_PASSTHROUGH_FB static_cast<XrStructureType>(1000118003)
#define XR_PASSTHROUGH_IS_RUNNING_AT_CREATION_BIT_FB 0x00000001ULL

typedef XrFlags64 XrPassthroughFlagsFB;

XR_DEFINE_HANDLE(XrPassthroughFB)
XR_DEFINE_HANDLE(XrPassthroughLayerFB)

//Stores passthrough layer purpose values.
typedef enum XrPassthroughLayerPurposeFB
{
    XR_PASSTHROUGH_LAYER_PURPOSE_RECONSTRUCTION_FB = 0, XR_PASSTHROUGH_LAYER_PURPOSE_PROJECTED_FB = 1, XR_PASSTHROUGH_LAYER_PURPOSE_MAX_ENUM_FB = 0x7FFFFFFF
} XrPassthroughLayerPurposeFB;

//Stores passthrough creation values.
typedef struct XrPassthroughCreateInfoFB
{
    XrStructureType type; //Stores the structure type.
    const void *next; //Stores the next extension structure.
    XrPassthroughFlagsFB flags; //Stores passthrough creation flags.
} XrPassthroughCreateInfoFB;

//Stores passthrough layer creation values.
typedef struct XrPassthroughLayerCreateInfoFB
{
    XrStructureType type; //Stores the structure type.
    const void *next; //Stores the next extension structure.
    XrPassthroughFB passthrough; //Stores the passthrough feature handle.
    XrPassthroughFlagsFB flags; //Stores passthrough layer creation flags.
    XrPassthroughLayerPurposeFB purpose; //Stores passthrough layer purpose.
} XrPassthroughLayerCreateInfoFB;

//Stores passthrough composition layer values.
typedef struct XrCompositionLayerPassthroughFB
{
    XrStructureType type; //Stores the structure type.
    const void *next; //Stores the next extension structure.
    XrCompositionLayerFlags layerFlags; //Stores composition layer flags.
    XrSpace space; //Stores optional layer space.
    XrPassthroughLayerFB layerHandle; //Stores passthrough layer handle.
} XrCompositionLayerPassthroughFB;

typedef XrResult(XRAPI_PTR *PFN_xrCreatePassthroughFB)(XrSession session, const XrPassthroughCreateInfoFB *createInfo, XrPassthroughFB *outPassthrough);
typedef XrResult(XRAPI_PTR *PFN_xrDestroyPassthroughFB)(XrPassthroughFB passthrough);
typedef XrResult(XRAPI_PTR *PFN_xrPassthroughStartFB)(XrPassthroughFB passthrough);
typedef XrResult(XRAPI_PTR *PFN_xrPassthroughPauseFB)(XrPassthroughFB passthrough);
typedef XrResult(XRAPI_PTR *PFN_xrCreatePassthroughLayerFB)(XrSession session, const XrPassthroughLayerCreateInfoFB *createInfo, XrPassthroughLayerFB *outLayer);
typedef XrResult(XRAPI_PTR *PFN_xrDestroyPassthroughLayerFB)(XrPassthroughLayerFB layer);
typedef XrResult(XRAPI_PTR *PFN_xrPassthroughLayerPauseFB)(XrPassthroughLayerFB layer);
typedef XrResult(XRAPI_PTR *PFN_xrPassthroughLayerResumeFB)(XrPassthroughLayerFB layer);
#endif

#ifndef XR_META_environment_depth
#define XR_META_environment_depth 1
#define XR_META_environment_depth_SPEC_VERSION 2
#define XR_META_ENVIRONMENT_DEPTH_EXTENSION_NAME "XR_META_environment_depth"
#define XR_TYPE_ENVIRONMENT_DEPTH_PROVIDER_CREATE_INFO_META static_cast<XrStructureType>(1000291000)
#define XR_TYPE_ENVIRONMENT_DEPTH_SWAPCHAIN_CREATE_INFO_META static_cast<XrStructureType>(1000291001)
#define XR_TYPE_ENVIRONMENT_DEPTH_SWAPCHAIN_STATE_META static_cast<XrStructureType>(1000291002)
#define XR_TYPE_ENVIRONMENT_DEPTH_IMAGE_ACQUIRE_INFO_META static_cast<XrStructureType>(1000291003)
#define XR_TYPE_ENVIRONMENT_DEPTH_IMAGE_VIEW_META static_cast<XrStructureType>(1000291004)
#define XR_TYPE_ENVIRONMENT_DEPTH_IMAGE_META static_cast<XrStructureType>(1000291005)
#define XR_TYPE_ENVIRONMENT_DEPTH_HAND_REMOVAL_SET_INFO_META static_cast<XrStructureType>(1000291006)
#define XR_TYPE_SYSTEM_ENVIRONMENT_DEPTH_PROPERTIES_META static_cast<XrStructureType>(1000291007)
#define XR_ENVIRONMENT_DEPTH_NOT_AVAILABLE_META static_cast<XrResult>(1000291000)

typedef XrFlags64 XrEnvironmentDepthProviderCreateFlagsMETA;
typedef XrFlags64 XrEnvironmentDepthSwapchainCreateFlagsMETA;

XR_DEFINE_HANDLE(XrEnvironmentDepthProviderMETA)
XR_DEFINE_HANDLE(XrEnvironmentDepthSwapchainMETA)

//Stores environment depth provider flag values.
typedef enum XrEnvironmentDepthProviderCreateFlagBitsMETA
{
    XR_ENVIRONMENT_DEPTH_PROVIDER_CREATE_FLAG_BITS_MAX_ENUM_META = 0x7FFFFFFF
} XrEnvironmentDepthProviderCreateFlagBitsMETA;

//Stores environment depth swapchain flag values.
typedef enum XrEnvironmentDepthSwapchainCreateFlagBitsMETA
{
    XR_ENVIRONMENT_DEPTH_SWAPCHAIN_CREATE_FLAG_BITS_MAX_ENUM_META = 0x7FFFFFFF
} XrEnvironmentDepthSwapchainCreateFlagBitsMETA;

//Stores environment depth provider creation values.
typedef struct XrEnvironmentDepthProviderCreateInfoMETA
{
    XrStructureType type; //Stores the structure type.
    const void *next; //Stores the next extension structure.
    XrEnvironmentDepthProviderCreateFlagsMETA createFlags; //Stores provider creation flags.
} XrEnvironmentDepthProviderCreateInfoMETA;

//Stores environment depth swapchain creation values.
typedef struct XrEnvironmentDepthSwapchainCreateInfoMETA
{
    XrStructureType type; //Stores the structure type.
    const void *next; //Stores the next extension structure.
    XrEnvironmentDepthSwapchainCreateFlagsMETA createFlags; //Stores swapchain creation flags.
} XrEnvironmentDepthSwapchainCreateInfoMETA;

//Stores environment depth swapchain state values.
typedef struct XrEnvironmentDepthSwapchainStateMETA
{
    XrStructureType type; //Stores the structure type.
    void *next; //Stores the next extension structure.
    uint32_t width; //Stores environment depth width.
    uint32_t height; //Stores environment depth height.
} XrEnvironmentDepthSwapchainStateMETA;

//Stores environment depth image acquisition values.
typedef struct XrEnvironmentDepthImageAcquireInfoMETA
{
    XrStructureType type; //Stores the structure type.
    const void *next; //Stores the next extension structure.
    XrSpace space; //Stores the space for the depth image.
    XrTime displayTime; //Stores the predicted display time.
} XrEnvironmentDepthImageAcquireInfoMETA;

//Stores one environment depth eye view.
typedef struct XrEnvironmentDepthImageViewMETA
{
    XrStructureType type; //Stores the structure type.
    const void *next; //Stores the next extension structure.
    XrFovf fov; //Stores the depth image field of view.
    XrPosef pose; //Stores the depth image pose.
} XrEnvironmentDepthImageViewMETA;

//Stores one acquired environment depth image.
typedef struct XrEnvironmentDepthImageMETA
{
    XrStructureType type; //Stores the structure type.
    const void *next; //Stores the next extension structure.
    uint32_t swapchainIndex; //Stores the acquired depth image index.
    float nearZ; //Stores the nearest environment depth distance.
    float farZ; //Stores the farthest environment depth distance.
    XrEnvironmentDepthImageViewMETA views[2]; //Stores per-eye environment depth views.
} XrEnvironmentDepthImageMETA;

//Stores environment depth hand removal values.
typedef struct XrEnvironmentDepthHandRemovalSetInfoMETA
{
    XrStructureType type; //Stores the structure type.
    const void *next; //Stores the next extension structure.
    XrBool32 enabled; //Stores whether hand removal is enabled.
} XrEnvironmentDepthHandRemovalSetInfoMETA;

//Stores environment depth system support values.
typedef struct XrSystemEnvironmentDepthPropertiesMETA
{
    XrStructureType type; //Stores the structure type.
    void *next; //Stores the next extension structure.
    XrBool32 supportsEnvironmentDepth; //Stores whether environment depth is supported.
    XrBool32 supportsHandRemoval; //Stores whether hand removal is supported.
} XrSystemEnvironmentDepthPropertiesMETA;

typedef XrResult(XRAPI_PTR *PFN_xrCreateEnvironmentDepthProviderMETA)(XrSession session, const XrEnvironmentDepthProviderCreateInfoMETA *createInfo, XrEnvironmentDepthProviderMETA *environmentDepthProvider);
typedef XrResult(XRAPI_PTR *PFN_xrDestroyEnvironmentDepthProviderMETA)(XrEnvironmentDepthProviderMETA environmentDepthProvider);
typedef XrResult(XRAPI_PTR *PFN_xrStartEnvironmentDepthProviderMETA)(XrEnvironmentDepthProviderMETA environmentDepthProvider);
typedef XrResult(XRAPI_PTR *PFN_xrStopEnvironmentDepthProviderMETA)(XrEnvironmentDepthProviderMETA environmentDepthProvider);
typedef XrResult(XRAPI_PTR *PFN_xrCreateEnvironmentDepthSwapchainMETA)(XrEnvironmentDepthProviderMETA environmentDepthProvider, const XrEnvironmentDepthSwapchainCreateInfoMETA *createInfo, XrEnvironmentDepthSwapchainMETA *swapchain);
typedef XrResult(XRAPI_PTR *PFN_xrDestroyEnvironmentDepthSwapchainMETA)(XrEnvironmentDepthSwapchainMETA swapchain);
typedef XrResult(XRAPI_PTR *PFN_xrEnumerateEnvironmentDepthSwapchainImagesMETA)(XrEnvironmentDepthSwapchainMETA swapchain, uint32_t imageCapacityInput, uint32_t *imageCountOutput, XrSwapchainImageBaseHeader *images);
typedef XrResult(XRAPI_PTR *PFN_xrGetEnvironmentDepthSwapchainStateMETA)(XrEnvironmentDepthSwapchainMETA swapchain, XrEnvironmentDepthSwapchainStateMETA *state);
typedef XrResult(XRAPI_PTR *PFN_xrAcquireEnvironmentDepthImageMETA)(XrEnvironmentDepthProviderMETA environmentDepthProvider, const XrEnvironmentDepthImageAcquireInfoMETA *acquireInfo, XrEnvironmentDepthImageMETA *environmentDepthImage);
typedef XrResult(XRAPI_PTR *PFN_xrSetEnvironmentDepthHandRemovalMETA)(XrEnvironmentDepthProviderMETA environmentDepthProvider, const XrEnvironmentDepthHandRemovalSetInfoMETA *setInfo);
#endif

namespace graphicsFX
{
    //Defines OpenXR runtime, session, and stereo display helpers.
    class OpenXRUtility
    {

        public:

            //Stores information for one XR eye render callback.
            struct EyeRenderInfo
            {
                uint32_t m_viewIndex{ 0 }; //Stores the XR eye index.
                VkCommandBuffer m_commandBufferPtrGPU{ VK_NULL_HANDLE }; //Stores the active command buffer.
                VkExtent2D m_extent{}; //Stores the eye render target size.
                XrView m_view{}; //Stores the OpenXR located view.
                bool m_environmentDepthAvailable{ false }; //Stores whether environment depth is available for this eye.
                uint32_t m_environmentDepthSwapchainIndex{ 0 }; //Stores the environment depth image index.
                VkImage m_environmentDepthImagePtrGPU{ VK_NULL_HANDLE }; //Stores the environment depth Vulkan image.
                VkImageView m_environmentDepthImageViewPtrGPU{ VK_NULL_HANDLE }; //Stores the environment depth image view.
                VkSampler m_environmentDepthSamplerPtrGPU{ VK_NULL_HANDLE }; //Stores the environment depth sampler.
                VkImageLayout m_environmentDepthImageLayout{ VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL }; //Stores environment depth image layout.
                VkExtent2D m_environmentDepthExtent{}; //Stores the environment depth image size.
                float m_environmentDepthNearZ{ 0.0f }; //Stores the environment depth near distance.
                float m_environmentDepthFarZ{ 0.0f }; //Stores the environment depth far distance.
                XrEnvironmentDepthImageViewMETA m_environmentDepthView{}; //Stores the environment depth view.
            };

            using EyeRenderCallback = std::function<void(const EyeRenderInfo &)>; //Stores the eye render callback type.

            //Creates the OpenXR utility.
            OpenXRUtility(void) = default;

            //Destroys the OpenXR utility.
            ~OpenXRUtility(void);

            OpenXRUtility(const OpenXRUtility &) = delete;
            OpenXRUtility &operator=(const OpenXRUtility &) = delete;

            //Initializes OpenXR runtime detection.
            void Init(const XRSetup &setup);

            //Initializes OpenXR session rendering with Vulkan.
            void InitSession(
                VkInstance instancePtrGPU,
                VkPhysicalDevice physicalDevicePtrGPU,
                VkDevice devicePtrGPU,
                uint32_t queueFamilyIndex,
                uint32_t queueIndex,
                VkQueue queuePtrGPU,
                VkCommandPool commandPoolPtrGPU,
                VkFormat preferredColorFormat
            );

            //Initializes OpenXR eye render resources.
            void InitRenderResources(VkPhysicalDevice physicalDevicePtrGPU, VkRenderPass renderPassPtrGPU, VkFormat colorFormat, VkFormat depthFormat, VkSampleCountFlagBits sampleCount);

            //Polls OpenXR events.
            void PollEvents(void);

            //Updates OpenXR controller action state.
            void UpdateControllerState(void);

            //Renders one OpenXR frame.
            void RenderFrame(const Float4 &clearColor);

            //Renders one OpenXR frame with a scene callback.
            void RenderFrame(const Float4 &clearColor, const EyeRenderCallback &renderCallback);

            //Releases OpenXR resources.
            void Clean(void);

            //Releases OpenXR eye render resources.
            void CleanRenderResources(void);

            //Returns whether XR was requested in setup.
            bool Enabled(void) const { return m_enabled; }

            //Returns whether the OpenXR loader and runtime are available.
            bool RuntimeAvailable(void) const { return m_runtimeAvailable; }

            //Returns whether an HMD system was detected.
            bool DeviceDetected(void) const { return m_deviceDetected; }

            //Returns whether PC mirror rendering should stay enabled.
            bool MirrorToWindow(void) const { return m_mirrorToWindow; }

            //Returns whether XR is ready for future stereo rendering.
            bool Active(void) const { return m_enabled && m_runtimeAvailable && m_deviceDetected; }

            //Returns whether the OpenXR session can render frames.
            bool SessionReady(void) const { return m_sessionReady; }

            //Returns whether OpenXR requested application exit.
            bool ExitRequested(void) const { return m_exitRequested; }

            //Returns the detected OpenXR runtime name.
            const std::string &RuntimeName(void) const { return m_runtimeName; }

            //Returns the detected OpenXR system name.
            const std::string &SystemName(void) const { return m_systemName; }

            //Returns the current OpenXR startup status.
            const std::string &StatusMessage(void) const { return m_statusMessage; }

            //Returns Vulkan instance extensions required by OpenXR.
            const std::vector<std::string> &VulkanInstanceExtensions(void) const { return m_vulkanInstanceExtensions; }

            //Returns Vulkan device extensions required by OpenXR.
            const std::vector<std::string> &VulkanDeviceExtensions(void) const { return m_vulkanDeviceExtensions; }

            //Queries the Vulkan physical device required by OpenXR.
            VkPhysicalDevice QueryRequiredPhysicalDevice(VkInstance instancePtrGPU);

            //Returns the minimum Vulkan API version required by OpenXR.
            uint32_t VulkanMinimumApiVersion(void) const { return m_vulkanMinimumApiVersion; }

            //Returns the current controller movement state.
            const ControllerMovementState &ControllerState(void) const { return m_controllerState; }

        private:

            //Stores one OpenXR swapchain and its Vulkan images.
            struct SwapchainData
            {
                XrSwapchain m_swapchainPtr{ XR_NULL_HANDLE }; //Stores the OpenXR swapchain.
                int64_t m_format{ 0 }; //Stores the Vulkan image format value.
                uint32_t m_width{ 0 }; //Stores the swapchain image width.
                uint32_t m_height{ 0 }; //Stores the swapchain image height.
                std::vector<XrSwapchainImageVulkanKHR> m_images; //Stores Vulkan swapchain images.
                std::vector<VkImageView> m_imageViewsPtrGPU; //Stores Vulkan swapchain image views.
                XrSwapchain m_depthSwapchainPtr{ XR_NULL_HANDLE }; //Stores the OpenXR depth swapchain.
                int64_t m_depthFormat{ 0 }; //Stores the Vulkan depth image format value.
                std::vector<XrSwapchainImageVulkanKHR> m_depthImages; //Stores Vulkan depth swapchain images.
                std::vector<VkImageView> m_depthImageViewsPtrGPU; //Stores Vulkan depth swapchain image views.
                std::vector<VkFramebuffer> m_framebuffersPtrGPU; //Stores Vulkan eye framebuffers.
                VkImage m_colorImagePtrGPU{ VK_NULL_HANDLE }; //Stores the multisampled color image.
                VkDeviceMemory m_colorImageMemoryPtrGPU{ VK_NULL_HANDLE }; //Stores multisampled color memory.
                VkImageView m_colorImageViewPtrGPU{ VK_NULL_HANDLE }; //Stores the multisampled color image view.
                VkImage m_depthImagePtrGPU{ VK_NULL_HANDLE }; //Stores the depth image.
                VkDeviceMemory m_depthImageMemoryPtrGPU{ VK_NULL_HANDLE }; //Stores depth image memory.
                VkImageView m_depthImageViewPtrGPU{ VK_NULL_HANDLE }; //Stores the depth image view.
            };

            //Stores full-screen depth debug push constants.
            struct DepthDebugConstants
            {
                Float4 m_settings; //Stores max depth, value mode, near, and far values.
            };

            //Sets and prints the OpenXR startup status.
            void SetStatus(const std::string &message);

            //Loads the OpenXR loader library.
            bool LoadLoader(void);

            //Loads global OpenXR function pointers.
            bool LoadGlobalFunctions(void);

            //Creates the OpenXR instance.
            bool CreateInstance(void);

            //Loads instance OpenXR function pointers.
            bool LoadInstanceFunctions(void);

            //Finds the head-mounted display system.
            bool FindHeadMountedDisplay(void);

            //Queries Vulkan extensions required by OpenXR.
            bool QueryVulkanExtensions(void);

            //Queries Vulkan graphics requirements required by OpenXR.
            bool QueryVulkanGraphicsRequirements(void);

            //Queries one OpenXR Vulkan extension string list.
            bool QueryVulkanExtensionList(
                PFN_xrGetVulkanInstanceExtensionsKHR instanceFunctionPtr,
                PFN_xrGetVulkanDeviceExtensionsKHR deviceFunctionPtr,
                bool queryInstanceExtensions,
                std::vector<std::string> &extensions
            );

            //Creates the OpenXR session.
            bool CreateSession(VkInstance instancePtrGPU, VkPhysicalDevice physicalDevicePtrGPU, VkDevice devicePtrGPU, uint32_t queueFamilyIndex, uint32_t queueIndex);

            //Creates the OpenXR reference space.
            bool CreateReferenceSpace(void);

            //Creates OpenXR controller actions.
            bool CreateControllerActions(void);

            //Releases OpenXR controller actions.
            void CleanControllerActions(void);

            //Creates OpenXR passthrough resources.
            bool CreatePassthrough(void);

            //Releases OpenXR passthrough resources.
            void CleanPassthrough(void);

            //Creates OpenXR environment depth resources.
            bool CreateEnvironmentDepth(void);

            //Releases OpenXR environment depth resources.
            void CleanEnvironmentDepth(void);

            //Acquires OpenXR environment depth for one frame.
            bool AcquireEnvironmentDepthImage(XrTime displayTime);

            //Creates Vulkan views for environment depth images.
            bool CreateEnvironmentDepthImageViews(void);

            //Creates Vulkan views for environment depth images with one format.
            bool TryCreateEnvironmentDepthImageViews(VkFormat format, VkImageAspectFlags aspectFlags);

            //Releases Vulkan views for environment depth images.
            void CleanEnvironmentDepthImageViews(void);

            //Creates the environment depth sampler.
            bool CreateEnvironmentDepthSampler(void);

            //Returns whether the full-screen depth debug view should draw.
            bool FullScreenDepthDebugEnabled(void) const;

            //Returns whether this frame needs Meta environment depth.
            bool EnvironmentDepthFrameNeeded(void) const;

            //Creates the full-screen environment depth debug resources.
            bool CreateDepthDebugResources(void);

            //Releases the full-screen environment depth debug resources.
            void CleanDepthDebugResources(void);

            //Draws a full-screen environment depth debug image.
            void DrawEnvironmentDepthDebug(VkCommandBuffer commandBufferPtrGPU, const EyeRenderInfo &renderInfo);

            //Clears the eye to a full-screen depth debug status color.
            void ClearDepthDebugStatus(VkCommandBuffer commandBufferPtrGPU, VkExtent2D extent, const Float4 &color) const;

            //Creates a Vulkan shader module.
            VkShaderModule CreateShaderModule(const std::vector<char> &code) const;

            //Loads a compiled shader file.
            std::vector<char> ReadShaderFile(const std::string &relativePath) const;

            //Converts an OpenXR result to readable text.
            std::string ResultToString(XrResult result) const;

            //Converts an OpenXR path string.
            bool StringToPath(const char *pathTextPtr, XrPath &path) const;

            //Creates OpenXR view and swapchain resources.
            bool CreateViewResources(VkFormat preferredColorFormat);

            //Creates one OpenXR swapchain.
            bool CreateSwapchain(uint32_t viewIndex, VkFormat preferredColorFormat);

            //Creates eye render resources for all swapchains.
            bool CreateRenderResources(void);

            //Creates one OpenXR depth swapchain.
            bool CreateDepthSwapchain(uint32_t viewIndex, SwapchainData &swapchain);

            //Creates eye render resources for one swapchain.
            bool CreateSwapchainRenderResources(SwapchainData &swapchain);

            //Releases one OpenXR depth swapchain.
            void CleanDepthSwapchain(SwapchainData &swapchain);

            //Creates one Vulkan image for XR rendering.
            bool CreateImage(uint32_t width, uint32_t height, VkFormat format, VkImageUsageFlags usage, VkSampleCountFlagBits sampleCount, VkImage &imagePtrGPU, VkDeviceMemory &imageMemoryPtrGPU) const;

            //Creates one Vulkan image view for XR rendering.
            VkImageView CreateImageView(VkImage imagePtrGPU, VkFormat format, VkImageAspectFlags aspectFlags) const;

            //Finds a matching Vulkan memory type.
            bool FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties, uint32_t &memoryTypeIndex) const;

            //Begins the OpenXR session.
            void BeginSession(void);

            //Ends the OpenXR session.
            void EndSession(void);

            //Renders one OpenXR eye swapchain image.
            bool RenderEye(uint32_t viewIndex, const Float4 &clearColor, const EyeRenderCallback &renderCallback);

            //Renders one OpenXR eye with scene content.
            bool RenderEyeScene(uint32_t viewIndex, uint32_t imageIndex, uint32_t depthImageIndex, const Float4 &clearColor, const EyeRenderCallback &renderCallback);

            //Clears one OpenXR Vulkan image.
            bool ClearImage(VkImage imagePtrGPU, uint32_t viewIndex, const Float4 &clearColor) const;

            //Submits one command buffer and waits for completion.
            bool SubmitCommandBuffer(VkCommandBuffer commandBufferPtrGPU) const;

            //Chooses an OpenXR swapchain color format.
            int64_t ChooseColorFormat(const std::vector<int64_t> &formats, VkFormat preferredColorFormat) const;

            //Returns a preferred OpenXR reference space type.
            XrReferenceSpaceType PreferredReferenceSpaceType(void) const;

            //Returns whether an OpenXR extension is available.
            static bool HasExtension(const std::vector<XrExtensionProperties> &extensions, const char *extensionNamePtr);

            //Copies a string into a fixed OpenXR character buffer.
            static void CopyText(char *targetPtr, std::size_t targetCount, const char *sourcePtr);

            //Converts an OpenXR Vulkan API version to a Vulkan API version.
            static uint32_t XrVersionToVulkanVersion(XrVersion version);

            //Formats an OpenXR version for debug output.
            static std::string FormatXrVersion(XrVersion version);

            //Converts a Vulkan sample count to a plain integer.
            static uint32_t SampleCountValue(VkSampleCountFlagBits sampleCount);

            //Returns the XR depth value interpretation mode.
            static float DepthValueModeValue(const std::string &valueMode);

            //Returns whether a depth debug mode fills the whole eye image.
            static bool FullScreenDepthDebugMode(const std::string &debugMode);

            //Returns whether the depth debug mode samples the depth texture.
            static bool DepthTextureDebugMode(const std::string &debugMode);

            //Returns a finite XR depth near plane.
            static float FiniteDepthNear(float nearPlane);

            //Returns a finite XR depth far plane.
            static float FiniteDepthFar(float farPlane, float nearPlane, float fallbackFarPlane);

            static constexpr std::array<const char *, 1> s_requiredExtensions{ XR_KHR_VULKAN_ENABLE_EXTENSION_NAME }; //Stores required OpenXR extension names.
            static constexpr float s_xrDepthNear = 0.05f; //Stores default XR depth near plane.
            static constexpr float s_xrDepthFar = 500.0f; //Stores default XR depth far plane.

            HMODULE m_loaderModulePtr{ nullptr }; //Stores the dynamically loaded OpenXR loader module.
            PFN_xrGetInstanceProcAddr m_xrGetInstanceProcAddrPtr{ nullptr }; //Stores xrGetInstanceProcAddr.
            PFN_xrEnumerateInstanceExtensionProperties m_xrEnumerateInstanceExtensionPropertiesPtr{ nullptr }; //Stores xrEnumerateInstanceExtensionProperties.
            PFN_xrCreateInstance m_xrCreateInstancePtr{ nullptr }; //Stores xrCreateInstance.
            PFN_xrDestroyInstance m_xrDestroyInstancePtr{ nullptr }; //Stores xrDestroyInstance.
            PFN_xrGetInstanceProperties m_xrGetInstancePropertiesPtr{ nullptr }; //Stores xrGetInstanceProperties.
            PFN_xrPollEvent m_xrPollEventPtr{ nullptr }; //Stores xrPollEvent.
            PFN_xrGetSystem m_xrGetSystemPtr{ nullptr }; //Stores xrGetSystem.
            PFN_xrGetSystemProperties m_xrGetSystemPropertiesPtr{ nullptr }; //Stores xrGetSystemProperties.
            PFN_xrResultToString m_xrResultToStringPtr{ nullptr }; //Stores xrResultToString.
            PFN_xrGetVulkanInstanceExtensionsKHR m_xrGetVulkanInstanceExtensionsKHRPtr{ nullptr }; //Stores xrGetVulkanInstanceExtensionsKHR.
            PFN_xrGetVulkanDeviceExtensionsKHR m_xrGetVulkanDeviceExtensionsKHRPtr{ nullptr }; //Stores xrGetVulkanDeviceExtensionsKHR.
            PFN_xrGetVulkanGraphicsDeviceKHR m_xrGetVulkanGraphicsDeviceKHRPtr{ nullptr }; //Stores xrGetVulkanGraphicsDeviceKHR.
            PFN_xrGetVulkanGraphicsRequirementsKHR m_xrGetVulkanGraphicsRequirementsKHRPtr{ nullptr }; //Stores xrGetVulkanGraphicsRequirementsKHR.
            PFN_xrCreateSession m_xrCreateSessionPtr{ nullptr }; //Stores xrCreateSession.
            PFN_xrDestroySession m_xrDestroySessionPtr{ nullptr }; //Stores xrDestroySession.
            PFN_xrCreateReferenceSpace m_xrCreateReferenceSpacePtr{ nullptr }; //Stores xrCreateReferenceSpace.
            PFN_xrDestroySpace m_xrDestroySpacePtr{ nullptr }; //Stores xrDestroySpace.
            PFN_xrEnumerateViewConfigurationViews m_xrEnumerateViewConfigurationViewsPtr{ nullptr }; //Stores xrEnumerateViewConfigurationViews.
            PFN_xrEnumerateSwapchainFormats m_xrEnumerateSwapchainFormatsPtr{ nullptr }; //Stores xrEnumerateSwapchainFormats.
            PFN_xrCreateSwapchain m_xrCreateSwapchainPtr{ nullptr }; //Stores xrCreateSwapchain.
            PFN_xrDestroySwapchain m_xrDestroySwapchainPtr{ nullptr }; //Stores xrDestroySwapchain.
            PFN_xrEnumerateSwapchainImages m_xrEnumerateSwapchainImagesPtr{ nullptr }; //Stores xrEnumerateSwapchainImages.
            PFN_xrAcquireSwapchainImage m_xrAcquireSwapchainImagePtr{ nullptr }; //Stores xrAcquireSwapchainImage.
            PFN_xrWaitSwapchainImage m_xrWaitSwapchainImagePtr{ nullptr }; //Stores xrWaitSwapchainImage.
            PFN_xrReleaseSwapchainImage m_xrReleaseSwapchainImagePtr{ nullptr }; //Stores xrReleaseSwapchainImage.
            PFN_xrBeginSession m_xrBeginSessionPtr{ nullptr }; //Stores xrBeginSession.
            PFN_xrEndSession m_xrEndSessionPtr{ nullptr }; //Stores xrEndSession.
            PFN_xrWaitFrame m_xrWaitFramePtr{ nullptr }; //Stores xrWaitFrame.
            PFN_xrBeginFrame m_xrBeginFramePtr{ nullptr }; //Stores xrBeginFrame.
            PFN_xrEndFrame m_xrEndFramePtr{ nullptr }; //Stores xrEndFrame.
            PFN_xrLocateViews m_xrLocateViewsPtr{ nullptr }; //Stores xrLocateViews.
            PFN_xrStringToPath m_xrStringToPathPtr{ nullptr }; //Stores xrStringToPath.
            PFN_xrCreateActionSet m_xrCreateActionSetPtr{ nullptr }; //Stores xrCreateActionSet.
            PFN_xrDestroyActionSet m_xrDestroyActionSetPtr{ nullptr }; //Stores xrDestroyActionSet.
            PFN_xrCreateAction m_xrCreateActionPtr{ nullptr }; //Stores xrCreateAction.
            PFN_xrDestroyAction m_xrDestroyActionPtr{ nullptr }; //Stores xrDestroyAction.
            PFN_xrSuggestInteractionProfileBindings m_xrSuggestInteractionProfileBindingsPtr{ nullptr }; //Stores xrSuggestInteractionProfileBindings.
            PFN_xrAttachSessionActionSets m_xrAttachSessionActionSetsPtr{ nullptr }; //Stores xrAttachSessionActionSets.
            PFN_xrSyncActions m_xrSyncActionsPtr{ nullptr }; //Stores xrSyncActions.
            PFN_xrGetActionStateVector2f m_xrGetActionStateVector2fPtr{ nullptr }; //Stores xrGetActionStateVector2f.
            PFN_xrGetActionStateBoolean m_xrGetActionStateBooleanPtr{ nullptr }; //Stores xrGetActionStateBoolean.
            PFN_xrCreatePassthroughFB m_xrCreatePassthroughFBPtr{ nullptr }; //Stores xrCreatePassthroughFB.
            PFN_xrDestroyPassthroughFB m_xrDestroyPassthroughFBPtr{ nullptr }; //Stores xrDestroyPassthroughFB.
            PFN_xrPassthroughStartFB m_xrPassthroughStartFBPtr{ nullptr }; //Stores xrPassthroughStartFB.
            PFN_xrPassthroughPauseFB m_xrPassthroughPauseFBPtr{ nullptr }; //Stores xrPassthroughPauseFB.
            PFN_xrCreatePassthroughLayerFB m_xrCreatePassthroughLayerFBPtr{ nullptr }; //Stores xrCreatePassthroughLayerFB.
            PFN_xrDestroyPassthroughLayerFB m_xrDestroyPassthroughLayerFBPtr{ nullptr }; //Stores xrDestroyPassthroughLayerFB.
            PFN_xrPassthroughLayerPauseFB m_xrPassthroughLayerPauseFBPtr{ nullptr }; //Stores xrPassthroughLayerPauseFB.
            PFN_xrPassthroughLayerResumeFB m_xrPassthroughLayerResumeFBPtr{ nullptr }; //Stores xrPassthroughLayerResumeFB.
            PFN_xrCreateEnvironmentDepthProviderMETA m_xrCreateEnvironmentDepthProviderMETAPtr{ nullptr }; //Stores xrCreateEnvironmentDepthProviderMETA.
            PFN_xrDestroyEnvironmentDepthProviderMETA m_xrDestroyEnvironmentDepthProviderMETAPtr{ nullptr }; //Stores xrDestroyEnvironmentDepthProviderMETA.
            PFN_xrStartEnvironmentDepthProviderMETA m_xrStartEnvironmentDepthProviderMETAPtr{ nullptr }; //Stores xrStartEnvironmentDepthProviderMETA.
            PFN_xrStopEnvironmentDepthProviderMETA m_xrStopEnvironmentDepthProviderMETAPtr{ nullptr }; //Stores xrStopEnvironmentDepthProviderMETA.
            PFN_xrCreateEnvironmentDepthSwapchainMETA m_xrCreateEnvironmentDepthSwapchainMETAPtr{ nullptr }; //Stores xrCreateEnvironmentDepthSwapchainMETA.
            PFN_xrDestroyEnvironmentDepthSwapchainMETA m_xrDestroyEnvironmentDepthSwapchainMETAPtr{ nullptr }; //Stores xrDestroyEnvironmentDepthSwapchainMETA.
            PFN_xrEnumerateEnvironmentDepthSwapchainImagesMETA m_xrEnumerateEnvironmentDepthSwapchainImagesMETAPtr{ nullptr }; //Stores xrEnumerateEnvironmentDepthSwapchainImagesMETA.
            PFN_xrGetEnvironmentDepthSwapchainStateMETA m_xrGetEnvironmentDepthSwapchainStateMETAPtr{ nullptr }; //Stores xrGetEnvironmentDepthSwapchainStateMETA.
            PFN_xrAcquireEnvironmentDepthImageMETA m_xrAcquireEnvironmentDepthImageMETAPtr{ nullptr }; //Stores xrAcquireEnvironmentDepthImageMETA.
            PFN_xrSetEnvironmentDepthHandRemovalMETA m_xrSetEnvironmentDepthHandRemovalMETAPtr{ nullptr }; //Stores xrSetEnvironmentDepthHandRemovalMETA.

            XrInstance m_instancePtr{ XR_NULL_HANDLE }; //Stores the OpenXR instance.
            XrSystemId m_systemId{ XR_NULL_SYSTEM_ID }; //Stores the detected OpenXR system id.
            XrSession m_sessionPtr{ XR_NULL_HANDLE }; //Stores the OpenXR session.
            XrSpace m_appSpacePtr{ XR_NULL_HANDLE }; //Stores the OpenXR app reference space.
            XrPassthroughFB m_passthroughPtr{ XR_NULL_HANDLE }; //Stores the OpenXR passthrough feature handle.
            XrPassthroughLayerFB m_passthroughLayerPtr{ XR_NULL_HANDLE }; //Stores the OpenXR passthrough layer handle.
            XrEnvironmentDepthProviderMETA m_environmentDepthProviderPtr{ XR_NULL_HANDLE }; //Stores the environment depth provider handle.
            XrEnvironmentDepthSwapchainMETA m_environmentDepthSwapchainPtr{ XR_NULL_HANDLE }; //Stores the environment depth swapchain handle.
            XrActionSet m_actionSetPtr{ XR_NULL_HANDLE }; //Stores the OpenXR controller action set.
            XrAction m_moveActionPtr{ XR_NULL_HANDLE }; //Stores the controller movement action.
            XrAction m_turnActionPtr{ XR_NULL_HANDLE }; //Stores the controller turn action.
            XrAction m_moveUpActionPtr{ XR_NULL_HANDLE }; //Stores the controller move-up action.
            XrAction m_moveDownActionPtr{ XR_NULL_HANDLE }; //Stores the controller move-down action.
            XrSessionState m_sessionState{ XR_SESSION_STATE_UNKNOWN }; //Stores the OpenXR session state.
            XrViewConfigurationType m_viewConfigurationType{ XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO }; //Stores the active view configuration type.
            VkPhysicalDevice m_requiredPhysicalDevicePtrGPU{ VK_NULL_HANDLE }; //Stores the Vulkan physical device required by OpenXR.
            VkPhysicalDevice m_physicalDevicePtrGPU{ VK_NULL_HANDLE }; //Stores the Vulkan physical device used for XR rendering.
            VkDevice m_devicePtrGPU{ VK_NULL_HANDLE }; //Stores the Vulkan device used for XR rendering.
            VkQueue m_queuePtrGPU{ VK_NULL_HANDLE }; //Stores the Vulkan queue used for XR rendering.
            VkCommandPool m_commandPoolPtrGPU{ VK_NULL_HANDLE }; //Stores the Vulkan command pool used for XR rendering.
            VkRenderPass m_renderPassPtrGPU{ VK_NULL_HANDLE }; //Stores the scene render pass used for XR.
            VkFormat m_colorFormat{ VK_FORMAT_UNDEFINED }; //Stores the XR scene color format.
            VkFormat m_depthFormat{ VK_FORMAT_UNDEFINED }; //Stores the XR scene depth format.
            VkSampleCountFlagBits m_sampleCount{ VK_SAMPLE_COUNT_1_BIT }; //Stores the XR scene sample count.
            std::vector<XrViewConfigurationView> m_viewConfigurationViews; //Stores per-eye recommended image settings.
            std::vector<XrView> m_views; //Stores per-eye located view poses.
            std::vector<XrCompositionLayerProjectionView> m_projectionViews; //Stores per-eye projection layer views.
            std::vector<XrCompositionLayerDepthInfoKHR> m_depthProjectionViews; //Stores per-eye projection depth layer views.
            std::vector<XrSwapchainImageVulkanKHR> m_environmentDepthImages; //Stores environment depth swapchain images.
            std::vector<VkImageView> m_environmentDepthImageViewsPtrGPU; //Stores environment depth image views.
            std::vector<SwapchainData> m_swapchains; //Stores per-eye OpenXR swapchains.
            std::vector<std::string> m_vulkanInstanceExtensions; //Stores Vulkan instance extensions required by OpenXR.
            std::vector<std::string> m_vulkanDeviceExtensions; //Stores Vulkan device extensions required by OpenXR.
            ControllerMovementState m_controllerState; //Stores controller movement values.
            uint32_t m_vulkanMinimumApiVersion{ VK_API_VERSION_1_0 }; //Stores minimum Vulkan API version required by OpenXR.
            uint32_t m_vulkanMaximumApiVersion{ VK_API_VERSION_1_0 }; //Stores maximum Vulkan API version supported by OpenXR.
            bool m_enabled{ false }; //Stores whether XR was requested.
            bool m_required{ false }; //Stores whether missing XR support is fatal.
            bool m_runtimeAvailable{ false }; //Stores whether an OpenXR runtime is available.
            bool m_deviceDetected{ false }; //Stores whether an HMD was found.
            bool m_mirrorToWindow{ true }; //Stores whether the PC mirror window remains enabled.
            bool m_sessionReady{ false }; //Stores whether OpenXR session resources are ready.
            bool m_sessionRunning{ false }; //Stores whether OpenXR frame rendering is running.
            bool m_renderResourcesReady{ false }; //Stores whether XR render resources are ready.
            bool m_controllerInputEnabled{ true }; //Stores whether OpenXR controller input is enabled.
            bool m_controllerActionsReady{ false }; //Stores whether controller actions are attached.
            bool m_passthroughRequested{ true }; //Stores whether passthrough is requested.
            bool m_passthroughExtensionEnabled{ false }; //Stores whether passthrough extension is enabled.
            bool m_passthroughReady{ false }; //Stores whether passthrough resources are ready.
            bool m_passthroughDepthTestRequested{ false }; //Stores whether passthrough depth testing is requested.
            bool m_depthLayerExtensionEnabled{ false }; //Stores whether OpenXR depth layer extension is enabled.
            bool m_passthroughDepthTestReady{ false }; //Stores whether passthrough depth testing is ready.
            bool m_environmentDepthRequested{ false }; //Stores whether environment depth is requested.
            bool m_environmentDepthExtensionEnabled{ false }; //Stores whether environment depth extension is enabled.
            bool m_environmentDepthReady{ false }; //Stores whether environment depth resources are ready.
            bool m_environmentDepthRunning{ false }; //Stores whether environment depth provider is running.
            bool m_environmentDepthAvailableThisFrame{ false }; //Stores whether environment depth was acquired this frame.
            bool m_environmentDepthHandRemovalSupported{ false }; //Stores whether hand removal is supported.
            bool m_environmentDepthHandRemovalRequested{ true }; //Stores whether hand removal is requested.
            bool m_environmentDepthAcquireFailureReported{ false }; //Stores whether acquire failure was reported.
            XrEnvironmentDepthImageMETA m_environmentDepthImage{}; //Stores the last acquired environment depth image.
            XrEnvironmentDepthSwapchainStateMETA m_environmentDepthSwapchainState{}; //Stores the environment depth swapchain state.
            VkSampler m_environmentDepthSamplerPtrGPU{ VK_NULL_HANDLE }; //Stores the environment depth sampler.
            VkFormat m_environmentDepthFormat{ VK_FORMAT_UNDEFINED }; //Stores the environment depth image view format.
            VkImageLayout m_environmentDepthImageLayout{ VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL }; //Stores the environment depth shader layout.
            VkDescriptorSetLayout m_depthDebugDescriptorSetLayoutPtrGPU{ VK_NULL_HANDLE }; //Stores the depth debug descriptor layout.
            VkDescriptorPool m_depthDebugDescriptorPoolPtrGPU{ VK_NULL_HANDLE }; //Stores the depth debug descriptor pool.
            VkDescriptorSet m_depthDebugDescriptorSetPtrGPU{ VK_NULL_HANDLE }; //Stores the depth debug descriptor set.
            VkPipelineLayout m_depthDebugPipelineLayoutPtrGPU{ VK_NULL_HANDLE }; //Stores the depth debug pipeline layout.
            VkPipeline m_depthDebugPipelinePtrGPU{ VK_NULL_HANDLE }; //Stores the depth debug graphics pipeline.
            bool m_exitRequested{ false }; //Stores whether OpenXR requested exit.
            float m_renderScale{ 1.0f }; //Stores the XR render scale from setup.
            float m_environmentDepthMaxMeters{ 8.0f }; //Stores the debug maximum environment depth.
            float m_environmentDepthValueMode{ 0.0f }; //Stores the depth value interpretation mode.
            std::string m_referenceSpaceName{ "LocalFloor" }; //Stores the preferred reference space name.
            std::string m_environmentDepthDebugView{ "Off" }; //Stores the environment depth debug view.
            std::string m_runtimeName; //Stores the detected runtime name.
            std::string m_systemName; //Stores the detected HMD system name.
            std::string m_statusMessage; //Stores the current startup status.
            bool m_environmentDepthLogEnabled{ true }; //Stores whether depth diagnostics are printed.
            bool m_environmentDepthFullScreenDebugRequested{ false }; //Stores whether raw depth fills the XR eye.
            bool m_environmentDepthTextureDebugRequested{ false }; //Stores whether depth debug samples the texture.
            bool m_environmentDepthTextureSamplingEnabled{ false }; //Stores whether shaders may sample depth textures.
            bool m_environmentDepthAcquireSuccessReported{ false }; //Stores whether first depth acquire was reported.
            bool m_environmentDepthUnavailableReported{ false }; //Stores whether unavailable depth was reported.
            bool m_depthDebugResourcesReady{ false }; //Stores whether depth debug drawing resources are ready.
            uint32_t m_environmentDepthUnavailableCount{ 0 }; //Stores unavailable depth frame count.
    };
}

