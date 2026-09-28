///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	OpenXRUtility.cpp
//	Description	:
//		Implements OpenXR runtime, session, and stereo display helpers.
//
//
///////////////////////////////////////////////////////////////////////

#include "OpenXRUtility.h"

#include "PathUtility.h"
#include "StringUtility.h"

#include <Windows.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace graphicsFX
{
    //Returns whether an OpenXR extension is available.
    bool OpenXRUtility::HasExtension(const std::vector<XrExtensionProperties> &extensions, const char *extensionNamePtr)
    {
        for (const XrExtensionProperties &extension : extensions)
        {
            if (std::strcmp(extension.extensionName, extensionNamePtr) == 0)
            {
                return true;
            }
        }

        return false;
    }

    //Copies a string into a fixed OpenXR character buffer.
    void OpenXRUtility::CopyText(char *targetPtr, std::size_t targetCount, const char *sourcePtr)
    {
        if (targetPtr == nullptr || targetCount == 0U)
        {
            return;
        }

        targetPtr[0] = '\0';
        if (sourcePtr != nullptr)
        {
            strncpy_s(targetPtr, targetCount, sourcePtr, _TRUNCATE);
        }
    }

    //Converts an OpenXR Vulkan API version to a Vulkan API version.
    uint32_t OpenXRUtility::XrVersionToVulkanVersion(XrVersion version)
    {
        return VK_MAKE_VERSION(XR_VERSION_MAJOR(version), XR_VERSION_MINOR(version), XR_VERSION_PATCH(version));
    }

    //Formats an OpenXR version for debug output.
    std::string OpenXRUtility::FormatXrVersion(XrVersion version)
    {
        return std::to_string(XR_VERSION_MAJOR(version)) + "." + std::to_string(XR_VERSION_MINOR(version)) + "." + std::to_string(XR_VERSION_PATCH(version));
    }

    //Converts a Vulkan sample count to a plain integer.
    uint32_t OpenXRUtility::SampleCountValue(VkSampleCountFlagBits sampleCount)
    {
        switch (sampleCount)
        {
            case VK_SAMPLE_COUNT_64_BIT: return 64U;
            case VK_SAMPLE_COUNT_32_BIT: return 32U;
            case VK_SAMPLE_COUNT_16_BIT: return 16U;
            case VK_SAMPLE_COUNT_8_BIT: return 8U;
            case VK_SAMPLE_COUNT_4_BIT: return 4U;
            case VK_SAMPLE_COUNT_2_BIT: return 2U;
            case VK_SAMPLE_COUNT_1_BIT:
            default: return 1U;
        }
    }

    //Returns the XR depth value interpretation mode.
    float OpenXRUtility::DepthValueModeValue(const std::string &valueMode)
    {
        const std::string mode = StringUtility::Uppercase(valueMode);
        if (mode == "NORMALIZED01" || mode == "NORMALIZED" || mode == "0TO1")
        {
            return 1.0f;
        }

        if (mode == "INVERSEZ" || mode == "INVERSE_Z")
        {
            return 2.0f;
        }

        return 0.0f;
    }

    //Returns whether a depth debug mode fills the whole eye image.
    bool OpenXRUtility::FullScreenDepthDebugMode(const std::string &debugMode)
    {
        const std::string mode = StringUtility::Uppercase(debugMode);
        return mode == "DEPTH" || mode == "RAWDEPTH" || mode == "RAW_DEPTH" || mode == "RAWDEPTHTEXTURE" || mode == "RAW_DEPTH_TEXTURE" || mode == "TEXTURE" || mode == "FULLSCREEN" ||
            mode == "FULL_SCREEN";
    }

    //Returns whether the depth debug mode samples the depth texture.
    bool OpenXRUtility::DepthTextureDebugMode(const std::string &debugMode)
    {
        const std::string mode = StringUtility::Uppercase(debugMode);
        return mode == "RAWDEPTHTEXTURE" || mode == "RAW_DEPTH_TEXTURE" || mode == "TEXTURE";
    }

    //Returns a finite XR depth near plane.
    float OpenXRUtility::FiniteDepthNear(float nearPlane)
    {
        if (std::isfinite(nearPlane) && nearPlane > 0.0f)
        {
            return nearPlane;
        }

        return 0.05f;
    }

    //Returns a finite XR depth far plane.
    float OpenXRUtility::FiniteDepthFar(float farPlane, float nearPlane, float fallbackFarPlane)
    {
        if (std::isfinite(farPlane) && farPlane > nearPlane)
        {
            return farPlane;
        }

        return std::max(fallbackFarPlane, nearPlane + 1.0f);
    }

    //Destroys the OpenXR utility.
    OpenXRUtility::~OpenXRUtility(void)
    {
        Clean();
    }

    //Initializes OpenXR runtime detection.
    void OpenXRUtility::Init(const XRSetup &setup)
    {
        m_enabled = setup.m_enabled;
        m_required = setup.m_required;
        m_mirrorToWindow = setup.m_mirrorToWindow;
        m_renderScale = setup.m_renderScale;
        m_referenceSpaceName = setup.m_referenceSpace;
        m_controllerInputEnabled = setup.m_controllerInput;
        m_passthroughRequested = setup.m_passthroughEnabled;
        m_passthroughDepthTestRequested = setup.m_useDepthLayer && setup.m_passthroughDepthTestEnabled;
        m_environmentDepthDebugView = setup.m_passthroughDepthDebugView;
        m_environmentDepthFullScreenDebugRequested = FullScreenDepthDebugMode(m_environmentDepthDebugView);
        m_environmentDepthTextureSamplingEnabled = setup.m_passthroughDepthTextureSampling;
        m_environmentDepthTextureDebugRequested = m_environmentDepthTextureSamplingEnabled && DepthTextureDebugMode(m_environmentDepthDebugView);
        if (!m_environmentDepthTextureSamplingEnabled && DepthTextureDebugMode(m_environmentDepthDebugView))
        {
            OutputDebugStringA("OpenXR environment depth texture sampling is disabled. " "RawDepthTexture will use the safe RawDepth status view.\n");
        }
        m_environmentDepthValueMode = DepthValueModeValue(setup.m_passthroughDepthValueMode);
        m_environmentDepthMaxMeters = setup.m_passthroughDepthMaxMeters;
        m_environmentDepthLogEnabled = setup.m_passthroughDepthLog;
        const std::string depthMode = StringUtility::Uppercase(setup.m_passthroughDepthMode);
        m_environmentDepthRequested = m_passthroughRequested && m_passthroughDepthTestRequested && (m_environmentDepthTextureSamplingEnabled || m_environmentDepthFullScreenDebugRequested) &&
            (depthMode == "OPENXRMETA" || depthMode == "META" || depthMode == "AUTO");
        m_environmentDepthHandRemovalRequested = setup.m_passthroughDepthHandRemoval;
        m_controllerState = {};

        if (!m_enabled)
        {
            SetStatus("OpenXR is disabled in setup.ini.");
            return;
        }

        if (!LoadLoader() || !LoadGlobalFunctions() || !CreateInstance() || !LoadInstanceFunctions() || !FindHeadMountedDisplay() || !QueryVulkanGraphicsRequirements() || !QueryVulkanExtensions())
        {
            const std::string failureMessage = m_statusMessage.empty() ? "OpenXR startup failed." : m_statusMessage;
            Clean();

            if (m_required)
            {
                throw std::runtime_error(failureMessage);
            }

            SetStatus(failureMessage + " Continuing with desktop Vulkan rendering.");
            return;
        }

        SetStatus("OpenXR device detected: " + m_systemName + ". Desktop mirror rendering remains enabled.");
    }

    //Initializes OpenXR session rendering with Vulkan.
    void OpenXRUtility::InitSession(
        VkInstance instancePtrGPU,
        VkPhysicalDevice physicalDevicePtrGPU,
        VkDevice devicePtrGPU,
        uint32_t queueFamilyIndex,
        uint32_t queueIndex,
        VkQueue queuePtrGPU,
        VkCommandPool commandPoolPtrGPU,
        VkFormat preferredColorFormat
    )
    {
        if (!Active())
        {
            return;
        }

        m_devicePtrGPU = devicePtrGPU;
        m_queuePtrGPU = queuePtrGPU;
        m_commandPoolPtrGPU = commandPoolPtrGPU;

        if (!CreateSession(instancePtrGPU, physicalDevicePtrGPU, devicePtrGPU, queueFamilyIndex, queueIndex) || !CreateReferenceSpace() || !CreateViewResources(preferredColorFormat))
        {
            if (m_required)
            {
                throw std::runtime_error(m_statusMessage);
            }

            SetStatus(m_statusMessage + " OpenXR session display is disabled.");
            return;
        }

        if (m_passthroughRequested && m_passthroughExtensionEnabled && !CreatePassthrough())
        {
            CleanPassthrough();
            OutputDebugStringA("OpenXR passthrough is disabled.\n");
        }

        if (m_environmentDepthRequested && m_environmentDepthExtensionEnabled && m_passthroughReady && !CreateEnvironmentDepth())
        {
            CleanEnvironmentDepth();
            OutputDebugStringA("OpenXR environment depth is disabled.\n");
        }

        if (m_controllerInputEnabled && !CreateControllerActions())
        {
            CleanControllerActions();
            m_controllerInputEnabled = false;
            OutputDebugStringA("OpenXR controller input is disabled.\n");
        }

        m_sessionReady = true;
        SetStatus("OpenXR session and stereo swapchains are ready.");
    }

    //Initializes OpenXR eye render resources.
    void OpenXRUtility::InitRenderResources(VkPhysicalDevice physicalDevicePtrGPU, VkRenderPass renderPassPtrGPU, VkFormat colorFormat, VkFormat depthFormat, VkSampleCountFlagBits sampleCount)
    {
        CleanRenderResources();

        if (!m_sessionReady)
        {
            return;
        }

        m_physicalDevicePtrGPU = physicalDevicePtrGPU;
        m_renderPassPtrGPU = renderPassPtrGPU;
        m_colorFormat = colorFormat;
        m_depthFormat = depthFormat;
        m_sampleCount = sampleCount;

        if (!CreateRenderResources())
        {
            CleanRenderResources();
            SetStatus("OpenXR scene render resources are disabled.");
            return;
        }

        m_renderResourcesReady = true;
        SetStatus("OpenXR scene render resources are ready.");
    }

    //Queries the Vulkan physical device required by OpenXR.
    VkPhysicalDevice OpenXRUtility::QueryRequiredPhysicalDevice(VkInstance instancePtrGPU)
    {
        m_requiredPhysicalDevicePtrGPU = VK_NULL_HANDLE;

        if (!Active() || instancePtrGPU == VK_NULL_HANDLE)
        {
            return VK_NULL_HANDLE;
        }

        XrResult result = m_xrGetVulkanGraphicsDeviceKHRPtr(m_instancePtr, m_systemId, instancePtrGPU, &m_requiredPhysicalDevicePtrGPU);
        if (XR_FAILED(result) || m_requiredPhysicalDevicePtrGPU == VK_NULL_HANDLE)
        {
            SetStatus("OpenXR could not query the required Vulkan physical device.");
            return VK_NULL_HANDLE;
        }

        OutputDebugStringA("OpenXR required Vulkan physical device was queried.\n");
        return m_requiredPhysicalDevicePtrGPU;
    }

    //Polls OpenXR events.
    void OpenXRUtility::PollEvents(void)
    {
        if (!Active() || m_xrPollEventPtr == nullptr)
        {
            return;
        }

        XrEventDataBuffer eventData{};
        eventData.type = XR_TYPE_EVENT_DATA_BUFFER;

        XrResult result = m_xrPollEventPtr(m_instancePtr, &eventData);
        while (result == XR_SUCCESS)
        {
            if (eventData.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED)
            {
                const auto *stateEventPtr = reinterpret_cast<const XrEventDataSessionStateChanged *>(&eventData);
                if (stateEventPtr->session == m_sessionPtr || m_sessionPtr == XR_NULL_HANDLE)
                {
                    m_sessionState = stateEventPtr->state;

                    if (m_sessionState == XR_SESSION_STATE_READY)
                    {
                        BeginSession();
                    }
                    else if (m_sessionState == XR_SESSION_STATE_STOPPING)
                    {
                        EndSession();
                    }
                    else if (m_sessionState == XR_SESSION_STATE_EXITING || m_sessionState == XR_SESSION_STATE_LOSS_PENDING)
                    {
                        m_exitRequested = true;
                    }
                }
            }

            eventData = {};
            eventData.type = XR_TYPE_EVENT_DATA_BUFFER;
            result = m_xrPollEventPtr(m_instancePtr, &eventData);
        }

        if (result != XR_EVENT_UNAVAILABLE && XR_FAILED(result))
        {
            SetStatus("OpenXR event polling failed.");
        }
    }

    //Updates OpenXR controller action state.
    void OpenXRUtility::UpdateControllerState(void)
    {
        m_controllerState = {};

        if (!m_controllerInputEnabled || !m_controllerActionsReady || !m_sessionRunning || m_sessionPtr == XR_NULL_HANDLE || m_actionSetPtr == XR_NULL_HANDLE)
        {
            return;
        }

        XrActiveActionSet activeActionSet{};
        activeActionSet.actionSet = m_actionSetPtr;
        activeActionSet.subactionPath = XR_NULL_PATH;

        XrActionsSyncInfo syncInfo{};
        syncInfo.type = XR_TYPE_ACTIONS_SYNC_INFO;
        syncInfo.countActiveActionSets = 1;
        syncInfo.activeActionSets = &activeActionSet;

        XrResult result = m_xrSyncActionsPtr(m_sessionPtr, &syncInfo);
        if (XR_FAILED(result))
        {
            return;
        }

        auto readVector = [&](XrAction actionPtr) -> Float2
        {
            if (actionPtr == XR_NULL_HANDLE)
            {
                return { 0.0f, 0.0f };
            }

            XrActionStateGetInfo getInfo{};
            getInfo.type = XR_TYPE_ACTION_STATE_GET_INFO;
            getInfo.action = actionPtr;
            getInfo.subactionPath = XR_NULL_PATH;

            XrActionStateVector2f state{};
            state.type = XR_TYPE_ACTION_STATE_VECTOR2F;
            result = m_xrGetActionStateVector2fPtr(m_sessionPtr, &getInfo, &state);
            if (XR_FAILED(result) || state.isActive == XR_FALSE)
            {
                return { 0.0f, 0.0f };
            }

            return { state.currentState.x, state.currentState.y };
        };

        auto readBoolean = [&](XrAction actionPtr) -> bool
        {
            if (actionPtr == XR_NULL_HANDLE)
            {
                return false;
            }

            XrActionStateGetInfo getInfo{};
            getInfo.type = XR_TYPE_ACTION_STATE_GET_INFO;
            getInfo.action = actionPtr;
            getInfo.subactionPath = XR_NULL_PATH;

            XrActionStateBoolean state{};
            state.type = XR_TYPE_ACTION_STATE_BOOLEAN;
            result = m_xrGetActionStateBooleanPtr(m_sessionPtr, &getInfo, &state);
            return XR_SUCCEEDED(result) && state.isActive == XR_TRUE && state.currentState == XR_TRUE;
        };

        m_controllerState.m_move = readVector(m_moveActionPtr);
        m_controllerState.m_turn = readVector(m_turnActionPtr);
        m_controllerState.m_moveUp = readBoolean(m_moveUpActionPtr);
        m_controllerState.m_moveDown = readBoolean(m_moveDownActionPtr);
        m_controllerState.m_active = std::abs(m_controllerState.m_move.x) > 0.0f || std::abs(m_controllerState.m_move.y) > 0.0f || std::abs(m_controllerState.m_turn.x) > 0.0f ||
            std::abs(m_controllerState.m_turn.y) > 0.0f || m_controllerState.m_moveUp || m_controllerState.m_moveDown;
    }

    //Renders one OpenXR frame.
    void OpenXRUtility::RenderFrame(const Float4 &clearColor)
    {
        RenderFrame(clearColor, EyeRenderCallback());
    }

    //Renders one OpenXR frame with a scene callback.
    void OpenXRUtility::RenderFrame(const Float4 &clearColor, const EyeRenderCallback &renderCallback)
    {
        if (!m_sessionReady || !m_sessionRunning)
        {
            return;
        }

        XrFrameWaitInfo waitInfo{};
        waitInfo.type = XR_TYPE_FRAME_WAIT_INFO;

        XrFrameState frameState{};
        frameState.type = XR_TYPE_FRAME_STATE;

        XrResult result = m_xrWaitFramePtr(m_sessionPtr, &waitInfo, &frameState);
        if (XR_FAILED(result))
        {
            SetStatus("OpenXR xrWaitFrame failed.");
            return;
        }

        XrFrameBeginInfo beginInfo{};
        beginInfo.type = XR_TYPE_FRAME_BEGIN_INFO;
        result = m_xrBeginFramePtr(m_sessionPtr, &beginInfo);
        if (XR_FAILED(result))
        {
            SetStatus("OpenXR xrBeginFrame failed.");
            return;
        }

        std::vector<const XrCompositionLayerBaseHeader *> layers;
        XrCompositionLayerPassthroughFB passthroughLayer{};
        XrCompositionLayerProjection projectionLayer{};
        projectionLayer.type = XR_TYPE_COMPOSITION_LAYER_PROJECTION;
        if (m_passthroughReady)
        {
            passthroughLayer.type = XR_TYPE_COMPOSITION_LAYER_PASSTHROUGH_FB;
            passthroughLayer.layerHandle = m_passthroughLayerPtr;
            layers.push_back(reinterpret_cast<const XrCompositionLayerBaseHeader *>(&passthroughLayer));

            projectionLayer.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT | XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
        }

        if (frameState.shouldRender == XR_TRUE)
        {
            if (m_environmentDepthReady && EnvironmentDepthFrameNeeded())
            {
                AcquireEnvironmentDepthImage(frameState.predictedDisplayTime);
            }

            XrViewLocateInfo locateInfo{};
            locateInfo.type = XR_TYPE_VIEW_LOCATE_INFO;
            locateInfo.viewConfigurationType = m_viewConfigurationType;
            locateInfo.displayTime = frameState.predictedDisplayTime;
            locateInfo.space = m_appSpacePtr;

            XrViewState viewState{};
            viewState.type = XR_TYPE_VIEW_STATE;

            uint32_t viewCount = 0;
            result = m_xrLocateViewsPtr(m_sessionPtr, &locateInfo, &viewState, static_cast<uint32_t>(m_views.size()), &viewCount, m_views.data());

            if (XR_SUCCEEDED(result) && viewCount == m_views.size())
            {
                bool renderedAllViews = true;
                Float4 renderClearColor = clearColor;
                if (m_passthroughReady)
                {
                    renderClearColor.w = 0.0f;
                }

                for (uint32_t viewIndex = 0; viewIndex < viewCount; ++viewIndex)
                {
                    renderedAllViews = RenderEye(viewIndex, renderClearColor, renderCallback) && renderedAllViews;

                    XrCompositionLayerProjectionView projectionView{};
                    projectionView.type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW;
                    projectionView.pose = m_views[viewIndex].pose;
                    projectionView.fov = m_views[viewIndex].fov;
                    projectionView.subImage.swapchain = m_swapchains[viewIndex].m_swapchainPtr;
                    projectionView.subImage.imageRect.offset = { 0, 0 };
                    projectionView.subImage.imageRect.extent = {
                        static_cast<int32_t>(m_swapchains[viewIndex].m_width), static_cast<int32_t>(m_swapchains[viewIndex].m_height)
                    };
                    projectionView.subImage.imageArrayIndex = 0;

                    if (m_depthLayerExtensionEnabled && m_passthroughDepthTestReady && renderCallback && m_swapchains[viewIndex].m_depthSwapchainPtr != XR_NULL_HANDLE)
                    {
                        XrCompositionLayerDepthInfoKHR depthInfo{};
                        depthInfo.type = XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR;
                        depthInfo.subImage.swapchain = m_swapchains[viewIndex].m_depthSwapchainPtr;
                        depthInfo.subImage.imageRect.offset = { 0, 0 };
                        depthInfo.subImage.imageRect.extent = projectionView.subImage.imageRect.extent;
                        depthInfo.subImage.imageArrayIndex = 0;
                        depthInfo.minDepth = 0.0f;
                        depthInfo.maxDepth = 1.0f;
                        depthInfo.nearZ = s_xrDepthNear;
                        depthInfo.farZ = s_xrDepthFar;
                        m_depthProjectionViews[viewIndex] = depthInfo;
                        projectionView.next = &m_depthProjectionViews[viewIndex];
                    }

                    m_projectionViews[viewIndex] = projectionView;
                }

                if (renderedAllViews)
                {
                    projectionLayer.space = m_appSpacePtr;
                    projectionLayer.viewCount = viewCount;
                    projectionLayer.views = m_projectionViews.data();
                    layers.push_back(reinterpret_cast<const XrCompositionLayerBaseHeader *>(&projectionLayer));
                }
            }
        }

        XrFrameEndInfo endInfo{};
        endInfo.type = XR_TYPE_FRAME_END_INFO;
        endInfo.displayTime = frameState.predictedDisplayTime;
        endInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        endInfo.layerCount = static_cast<uint32_t>(layers.size());
        endInfo.layers = layers.empty() ? nullptr : layers.data();

        result = m_xrEndFramePtr(m_sessionPtr, &endInfo);
        if (XR_FAILED(result))
        {
            SetStatus("OpenXR xrEndFrame failed.");
        }
    }

    //Releases OpenXR resources.
    void OpenXRUtility::Clean(void)
    {
        EndSession();
        CleanRenderResources();
        CleanControllerActions();
        CleanEnvironmentDepth();
        CleanPassthrough();

        for (SwapchainData &swapchain : m_swapchains)
        {
            if (swapchain.m_swapchainPtr != XR_NULL_HANDLE && m_xrDestroySwapchainPtr != nullptr)
            {
                m_xrDestroySwapchainPtr(swapchain.m_swapchainPtr);
                swapchain.m_swapchainPtr = XR_NULL_HANDLE;
            }

            if (swapchain.m_depthSwapchainPtr != XR_NULL_HANDLE && m_xrDestroySwapchainPtr != nullptr)
            {
                m_xrDestroySwapchainPtr(swapchain.m_depthSwapchainPtr);
                swapchain.m_depthSwapchainPtr = XR_NULL_HANDLE;
            }
        }
        m_swapchains.clear();

        if (m_appSpacePtr != XR_NULL_HANDLE && m_xrDestroySpacePtr != nullptr)
        {
            m_xrDestroySpacePtr(m_appSpacePtr);
            m_appSpacePtr = XR_NULL_HANDLE;
        }

        if (m_sessionPtr != XR_NULL_HANDLE && m_xrDestroySessionPtr != nullptr)
        {
            m_xrDestroySessionPtr(m_sessionPtr);
            m_sessionPtr = XR_NULL_HANDLE;
        }

        if (m_instancePtr != XR_NULL_HANDLE && m_xrDestroyInstancePtr != nullptr)
        {
            m_xrDestroyInstancePtr(m_instancePtr);
            m_instancePtr = XR_NULL_HANDLE;
        }

        if (m_loaderModulePtr != nullptr)
        {
            FreeLibrary(m_loaderModulePtr);
            m_loaderModulePtr = nullptr;
        }

        m_systemId = XR_NULL_SYSTEM_ID;
        m_requiredPhysicalDevicePtrGPU = VK_NULL_HANDLE;
        m_physicalDevicePtrGPU = VK_NULL_HANDLE;
        m_devicePtrGPU = VK_NULL_HANDLE;
        m_queuePtrGPU = VK_NULL_HANDLE;
        m_commandPoolPtrGPU = VK_NULL_HANDLE;
        m_renderPassPtrGPU = VK_NULL_HANDLE;
        m_colorFormat = VK_FORMAT_UNDEFINED;
        m_depthFormat = VK_FORMAT_UNDEFINED;
        m_sampleCount = VK_SAMPLE_COUNT_1_BIT;
        m_deviceDetected = false;
        m_runtimeAvailable = false;
        m_sessionReady = false;
        m_sessionRunning = false;
        m_renderResourcesReady = false;
        m_controllerInputEnabled = true;
        m_controllerActionsReady = false;
        m_passthroughRequested = true;
        m_passthroughExtensionEnabled = false;
        m_passthroughReady = false;
        m_passthroughDepthTestRequested = false;
        m_depthLayerExtensionEnabled = false;
        m_passthroughDepthTestReady = false;
        m_environmentDepthRequested = false;
        m_environmentDepthExtensionEnabled = false;
        m_environmentDepthReady = false;
        m_environmentDepthRunning = false;
        m_environmentDepthAvailableThisFrame = false;
        m_environmentDepthHandRemovalSupported = false;
        m_environmentDepthHandRemovalRequested = true;
        m_environmentDepthAcquireFailureReported = false;
        m_environmentDepthImage = {};
        m_environmentDepthSwapchainState = {};
        m_environmentDepthSamplerPtrGPU = VK_NULL_HANDLE;
        m_environmentDepthFormat = VK_FORMAT_UNDEFINED;
        m_environmentDepthImageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        m_depthDebugDescriptorSetLayoutPtrGPU = VK_NULL_HANDLE;
        m_depthDebugDescriptorPoolPtrGPU = VK_NULL_HANDLE;
        m_depthDebugDescriptorSetPtrGPU = VK_NULL_HANDLE;
        m_depthDebugPipelineLayoutPtrGPU = VK_NULL_HANDLE;
        m_depthDebugPipelinePtrGPU = VK_NULL_HANDLE;
        m_controllerState = {};
        m_vulkanInstanceExtensions.clear();
        m_vulkanDeviceExtensions.clear();
        m_depthProjectionViews.clear();
        m_environmentDepthImages.clear();
        m_environmentDepthImageViewsPtrGPU.clear();
        m_vulkanMinimumApiVersion = VK_API_VERSION_1_0;
        m_vulkanMaximumApiVersion = VK_API_VERSION_1_0;
        m_environmentDepthMaxMeters = 8.0f;
        m_environmentDepthValueMode = 0.0f;
        m_environmentDepthLogEnabled = true;
        m_environmentDepthFullScreenDebugRequested = false;
        m_environmentDepthTextureDebugRequested = false;
        m_environmentDepthTextureSamplingEnabled = false;
        m_environmentDepthAcquireSuccessReported = false;
        m_environmentDepthUnavailableReported = false;
        m_depthDebugResourcesReady = false;
        m_environmentDepthUnavailableCount = 0;
        m_environmentDepthDebugView = "Off";
        m_runtimeName.clear();
        m_systemName.clear();
    }

    //Sets and prints the OpenXR startup status.
    void OpenXRUtility::SetStatus(const std::string &message)
    {
        m_statusMessage = message;
        OutputDebugStringA((message + "\n").c_str());
    }

    //Loads the OpenXR loader library.
    bool OpenXRUtility::LoadLoader(void)
    {
        m_loaderModulePtr = LoadLibraryW(L"openxr_loader.dll");
        if (m_loaderModulePtr == nullptr)
        {
            SetStatus("OpenXR loader openxr_loader.dll was not found.");
            return false;
        }

        FARPROC functionPtr = GetProcAddress(m_loaderModulePtr, "xrGetInstanceProcAddr");
        if (functionPtr == nullptr)
        {
            SetStatus("OpenXR loader does not export xrGetInstanceProcAddr.");
            return false;
        }

        m_xrGetInstanceProcAddrPtr = reinterpret_cast<PFN_xrGetInstanceProcAddr>(functionPtr);
        return true;
    }

    //Loads global OpenXR function pointers.
    bool OpenXRUtility::LoadGlobalFunctions(void)
    {
        PFN_xrVoidFunction functionPtr = nullptr;

        XrResult result = m_xrGetInstanceProcAddrPtr(XR_NULL_HANDLE, "xrEnumerateInstanceExtensionProperties", &functionPtr);
        if (XR_FAILED(result) || functionPtr == nullptr)
        {
            SetStatus("OpenXR could not load xrEnumerateInstanceExtensionProperties.");
            return false;
        }
        m_xrEnumerateInstanceExtensionPropertiesPtr = reinterpret_cast<PFN_xrEnumerateInstanceExtensionProperties>(functionPtr);

        functionPtr = nullptr;
        result = m_xrGetInstanceProcAddrPtr(XR_NULL_HANDLE, "xrCreateInstance", &functionPtr);
        if (XR_FAILED(result) || functionPtr == nullptr)
        {
            SetStatus("OpenXR could not load xrCreateInstance.");
            return false;
        }
        m_xrCreateInstancePtr = reinterpret_cast<PFN_xrCreateInstance>(functionPtr);

        return true;
    }

    //Creates the OpenXR instance.
    bool OpenXRUtility::CreateInstance(void)
    {
        uint32_t extensionCount = 0;
        XrResult result = m_xrEnumerateInstanceExtensionPropertiesPtr(nullptr, 0, &extensionCount, nullptr);
        if (XR_FAILED(result))
        {
            SetStatus("OpenXR runtime is not available.");
            return false;
        }

        std::vector<XrExtensionProperties> extensions(extensionCount);
        for (XrExtensionProperties &extension : extensions)
        {
            extension = { XR_TYPE_EXTENSION_PROPERTIES };
        }

        if (!extensions.empty())
        {
            result = m_xrEnumerateInstanceExtensionPropertiesPtr(nullptr, extensionCount, &extensionCount, extensions.data());
            if (XR_FAILED(result))
            {
                SetStatus("OpenXR extension enumeration failed.");
                return false;
            }
        }

        for (const char *extensionNamePtr : s_requiredExtensions)
        {
            if (!HasExtension(extensions, extensionNamePtr))
            {
                SetStatus(std::string("OpenXR required extension is missing: ") + extensionNamePtr + ".");
                return false;
            }
        }

        std::vector<const char *> enabledExtensions(s_requiredExtensions.begin(), s_requiredExtensions.end());

        m_passthroughExtensionEnabled = m_passthroughRequested && HasExtension(extensions, XR_FB_PASSTHROUGH_EXTENSION_NAME);
        if (m_passthroughExtensionEnabled)
        {
            enabledExtensions.push_back(XR_FB_PASSTHROUGH_EXTENSION_NAME);
            if (m_environmentDepthLogEnabled)
            {
                OutputDebugStringA("OpenXR passthrough extension is enabled.\n");
            }
        }
        else if (m_passthroughRequested)
        {
            OutputDebugStringA("OpenXR passthrough extension is unavailable.\n");
        }

        m_depthLayerExtensionEnabled = false;
        if (m_passthroughRequested && m_passthroughDepthTestRequested)
        {
            OutputDebugStringA("OpenXR passthrough depth uses XR_META_environment_depth instead of XR_KHR_composition_layer_depth.\n");
        }

        m_environmentDepthExtensionEnabled = m_environmentDepthRequested && HasExtension(extensions, XR_META_ENVIRONMENT_DEPTH_EXTENSION_NAME);
        if (m_environmentDepthExtensionEnabled)
        {
            enabledExtensions.push_back(XR_META_ENVIRONMENT_DEPTH_EXTENSION_NAME);
            if (m_environmentDepthLogEnabled)
            {
                OutputDebugStringA("OpenXR environment depth extension is enabled.\n");
            }
        }
        else if (m_environmentDepthRequested)
        {
            OutputDebugStringA("OpenXR environment depth extension is unavailable.\n");
        }

        XrInstanceCreateInfo createInfo{};
        createInfo.type = XR_TYPE_INSTANCE_CREATE_INFO;
        CopyText(createInfo.applicationInfo.applicationName, sizeof(createInfo.applicationInfo.applicationName), "GraphicsApp00");
        createInfo.applicationInfo.applicationVersion = 1U;
        CopyText(createInfo.applicationInfo.engineName, sizeof(createInfo.applicationInfo.engineName), "GraphicsFXCore");
        createInfo.applicationInfo.engineVersion = 1U;
        createInfo.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;
        createInfo.enabledExtensionCount = static_cast<uint32_t>(enabledExtensions.size());
        createInfo.enabledExtensionNames = enabledExtensions.data();

        result = m_xrCreateInstancePtr(&createInfo, &m_instancePtr);
        if (XR_FAILED(result) || m_instancePtr == XR_NULL_HANDLE)
        {
            SetStatus("OpenXR instance creation failed.");
            return false;
        }

        m_runtimeAvailable = true;
        m_runtimeName = "OpenXR Runtime";
        return true;
    }

    //Loads instance OpenXR function pointers.
    bool OpenXRUtility::LoadInstanceFunctions(void)
    {
        auto loadFunction = [&](const char *namePtr, auto &targetPtr) -> bool
        {
            PFN_xrVoidFunction functionPtr = nullptr;
            const XrResult result = m_xrGetInstanceProcAddrPtr(m_instancePtr, namePtr, &functionPtr);
            if (XR_FAILED(result) || functionPtr == nullptr)
            {
                SetStatus(std::string("OpenXR could not load ") + namePtr + ".");
                return false;
            }

            using FunctionType = std::remove_reference_t<decltype(targetPtr)>;
            targetPtr = reinterpret_cast<FunctionType>(functionPtr);
            return true;
        };

        const bool loaded = loadFunction("xrDestroyInstance", m_xrDestroyInstancePtr) && loadFunction("xrGetInstanceProperties", m_xrGetInstancePropertiesPtr) &&
            loadFunction("xrPollEvent", m_xrPollEventPtr) && loadFunction("xrGetSystem", m_xrGetSystemPtr) && loadFunction("xrGetSystemProperties", m_xrGetSystemPropertiesPtr) &&
            loadFunction("xrResultToString", m_xrResultToStringPtr) && loadFunction("xrGetVulkanInstanceExtensionsKHR", m_xrGetVulkanInstanceExtensionsKHRPtr) &&
            loadFunction("xrGetVulkanDeviceExtensionsKHR", m_xrGetVulkanDeviceExtensionsKHRPtr) && loadFunction("xrGetVulkanGraphicsDeviceKHR", m_xrGetVulkanGraphicsDeviceKHRPtr) &&
            loadFunction("xrGetVulkanGraphicsRequirementsKHR", m_xrGetVulkanGraphicsRequirementsKHRPtr) && loadFunction("xrCreateSession", m_xrCreateSessionPtr) &&
            loadFunction("xrDestroySession", m_xrDestroySessionPtr) && loadFunction("xrCreateReferenceSpace", m_xrCreateReferenceSpacePtr) && loadFunction("xrDestroySpace", m_xrDestroySpacePtr) &&
            loadFunction("xrEnumerateViewConfigurationViews", m_xrEnumerateViewConfigurationViewsPtr) && loadFunction("xrEnumerateSwapchainFormats", m_xrEnumerateSwapchainFormatsPtr) &&
            loadFunction("xrCreateSwapchain", m_xrCreateSwapchainPtr) && loadFunction("xrDestroySwapchain", m_xrDestroySwapchainPtr) &&
            loadFunction("xrEnumerateSwapchainImages", m_xrEnumerateSwapchainImagesPtr) && loadFunction("xrAcquireSwapchainImage", m_xrAcquireSwapchainImagePtr) &&
            loadFunction("xrWaitSwapchainImage", m_xrWaitSwapchainImagePtr) && loadFunction("xrReleaseSwapchainImage", m_xrReleaseSwapchainImagePtr) &&
            loadFunction("xrBeginSession", m_xrBeginSessionPtr) && loadFunction("xrEndSession", m_xrEndSessionPtr) && loadFunction("xrWaitFrame", m_xrWaitFramePtr) &&
            loadFunction("xrBeginFrame", m_xrBeginFramePtr) && loadFunction("xrEndFrame", m_xrEndFramePtr) && loadFunction("xrLocateViews", m_xrLocateViewsPtr) &&
            loadFunction("xrStringToPath", m_xrStringToPathPtr) && loadFunction("xrCreateActionSet", m_xrCreateActionSetPtr) && loadFunction("xrDestroyActionSet", m_xrDestroyActionSetPtr) &&
            loadFunction("xrCreateAction", m_xrCreateActionPtr) && loadFunction("xrDestroyAction", m_xrDestroyActionPtr) &&
            loadFunction("xrSuggestInteractionProfileBindings", m_xrSuggestInteractionProfileBindingsPtr) && loadFunction("xrAttachSessionActionSets", m_xrAttachSessionActionSetsPtr) &&
            loadFunction("xrSyncActions", m_xrSyncActionsPtr) && loadFunction("xrGetActionStateVector2f", m_xrGetActionStateVector2fPtr) &&
            loadFunction("xrGetActionStateBoolean", m_xrGetActionStateBooleanPtr);

        if (loaded)
        {
            if (m_passthroughExtensionEnabled)
            {
                const bool passthroughLoaded = loadFunction("xrCreatePassthroughFB", m_xrCreatePassthroughFBPtr) && loadFunction("xrDestroyPassthroughFB", m_xrDestroyPassthroughFBPtr) &&
                    loadFunction("xrPassthroughStartFB", m_xrPassthroughStartFBPtr) && loadFunction("xrPassthroughPauseFB", m_xrPassthroughPauseFBPtr) &&
                    loadFunction("xrCreatePassthroughLayerFB", m_xrCreatePassthroughLayerFBPtr) && loadFunction("xrDestroyPassthroughLayerFB", m_xrDestroyPassthroughLayerFBPtr) &&
                    loadFunction("xrPassthroughLayerPauseFB", m_xrPassthroughLayerPauseFBPtr) && loadFunction("xrPassthroughLayerResumeFB", m_xrPassthroughLayerResumeFBPtr);
                if (!passthroughLoaded)
                {
                    m_passthroughExtensionEnabled = false;
                    OutputDebugStringA("OpenXR passthrough function loading failed.\n");
                }
            }

            if (m_environmentDepthExtensionEnabled)
            {
                const bool environmentDepthLoaded = loadFunction("xrCreateEnvironmentDepthProviderMETA", m_xrCreateEnvironmentDepthProviderMETAPtr) &&
                    loadFunction("xrDestroyEnvironmentDepthProviderMETA", m_xrDestroyEnvironmentDepthProviderMETAPtr) &&
                    loadFunction("xrStartEnvironmentDepthProviderMETA", m_xrStartEnvironmentDepthProviderMETAPtr) &&
                    loadFunction("xrStopEnvironmentDepthProviderMETA", m_xrStopEnvironmentDepthProviderMETAPtr) &&
                    loadFunction("xrCreateEnvironmentDepthSwapchainMETA", m_xrCreateEnvironmentDepthSwapchainMETAPtr) &&
                    loadFunction("xrDestroyEnvironmentDepthSwapchainMETA", m_xrDestroyEnvironmentDepthSwapchainMETAPtr) &&
                    loadFunction("xrEnumerateEnvironmentDepthSwapchainImagesMETA", m_xrEnumerateEnvironmentDepthSwapchainImagesMETAPtr) &&
                    loadFunction("xrGetEnvironmentDepthSwapchainStateMETA", m_xrGetEnvironmentDepthSwapchainStateMETAPtr) &&
                    loadFunction("xrAcquireEnvironmentDepthImageMETA", m_xrAcquireEnvironmentDepthImageMETAPtr) &&
                    loadFunction("xrSetEnvironmentDepthHandRemovalMETA", m_xrSetEnvironmentDepthHandRemovalMETAPtr);
                if (!environmentDepthLoaded)
                {
                    m_environmentDepthExtensionEnabled = false;
                    m_environmentDepthRequested = false;
                    OutputDebugStringA("OpenXR environment depth function loading failed.\n");
                }
            }

            XrInstanceProperties properties{};
            properties.type = XR_TYPE_INSTANCE_PROPERTIES;
            if (XR_SUCCEEDED(m_xrGetInstancePropertiesPtr(m_instancePtr, &properties)))
            {
                m_runtimeName = properties.runtimeName;
            }
        }

        return loaded;
    }

    //Finds the head-mounted display system.
    bool OpenXRUtility::FindHeadMountedDisplay(void)
    {
        XrSystemGetInfo getInfo{};
        getInfo.type = XR_TYPE_SYSTEM_GET_INFO;
        getInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;

        XrResult result = m_xrGetSystemPtr(m_instancePtr, &getInfo, &m_systemId);
        if (XR_FAILED(result) || m_systemId == XR_NULL_SYSTEM_ID)
        {
            SetStatus("OpenXR runtime did not report an available head-mounted display.");
            return false;
        }

        XrSystemEnvironmentDepthPropertiesMETA environmentDepthProperties{};
        environmentDepthProperties.type = XR_TYPE_SYSTEM_ENVIRONMENT_DEPTH_PROPERTIES_META;

        XrSystemProperties properties{};
        properties.type = XR_TYPE_SYSTEM_PROPERTIES;
        if (m_environmentDepthExtensionEnabled)
        {
            properties.next = &environmentDepthProperties;
        }

        result = m_xrGetSystemPropertiesPtr(m_instancePtr, m_systemId, &properties);
        if (XR_FAILED(result))
        {
            SetStatus("OpenXR system properties query failed.");
            return false;
        }

        m_systemName = properties.systemName;
        if (m_systemName.empty())
        {
            m_systemName = "OpenXR Head-Mounted Display";
        }

        if (m_environmentDepthExtensionEnabled)
        {
            if (environmentDepthProperties.supportsEnvironmentDepth == XR_FALSE)
            {
                m_environmentDepthExtensionEnabled = false;
                m_environmentDepthRequested = false;
                OutputDebugStringA("OpenXR system does not support environment depth.\n");
            }
            else
            {
                m_environmentDepthHandRemovalSupported = environmentDepthProperties.supportsHandRemoval == XR_TRUE;
                OutputDebugStringA("OpenXR environment depth is supported by the system.\n");
            }
        }

        m_deviceDetected = true;
        return true;
    }

    //Queries Vulkan graphics requirements required by OpenXR.
    bool OpenXRUtility::QueryVulkanGraphicsRequirements(void)
    {
        XrGraphicsRequirementsVulkanKHR requirements{};
        requirements.type = XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN_KHR;

        const XrResult result = m_xrGetVulkanGraphicsRequirementsKHRPtr(m_instancePtr, m_systemId, &requirements);
        if (XR_FAILED(result))
        {
            SetStatus("OpenXR Vulkan graphics requirements query failed.");
            return false;
        }

        m_vulkanMinimumApiVersion = XrVersionToVulkanVersion(requirements.minApiVersionSupported);
        m_vulkanMaximumApiVersion = XrVersionToVulkanVersion(requirements.maxApiVersionSupported);

        const std::string message = "OpenXR Vulkan API range: " + FormatXrVersion(requirements.minApiVersionSupported) + " to " + FormatXrVersion(requirements.maxApiVersionSupported) + "\n";
        OutputDebugStringA(message.c_str());
        return true;
    }

    //Queries Vulkan extensions required by OpenXR.
    bool OpenXRUtility::QueryVulkanExtensions(void)
    {
        if (!QueryVulkanExtensionList(m_xrGetVulkanInstanceExtensionsKHRPtr, m_xrGetVulkanDeviceExtensionsKHRPtr, true, m_vulkanInstanceExtensions))
        {
            return false;
        }

        if (!QueryVulkanExtensionList(m_xrGetVulkanInstanceExtensionsKHRPtr, m_xrGetVulkanDeviceExtensionsKHRPtr, false, m_vulkanDeviceExtensions))
        {
            return false;
        }

        return true;
    }

    //Queries one OpenXR Vulkan extension string list.
    bool OpenXRUtility::QueryVulkanExtensionList(
        PFN_xrGetVulkanInstanceExtensionsKHR instanceFunctionPtr,
        PFN_xrGetVulkanDeviceExtensionsKHR deviceFunctionPtr,
        bool queryInstanceExtensions,
        std::vector<std::string> &extensions
    )
    {
        extensions.clear();

        uint32_t bufferCount = 0;
        XrResult result = XR_SUCCESS;
        if (queryInstanceExtensions)
        {
            result = instanceFunctionPtr(m_instancePtr, m_systemId, 0, &bufferCount, nullptr);
        }
        else
        {
            result = deviceFunctionPtr(m_instancePtr, m_systemId, 0, &bufferCount, nullptr);
        }

        if (XR_FAILED(result))
        {
            SetStatus(queryInstanceExtensions ? "OpenXR Vulkan instance extension query failed." : "OpenXR Vulkan device extension query failed.");
            return false;
        }

        if (bufferCount == 0)
        {
            return true;
        }

        std::vector<char> extensionBuffer(bufferCount, '\0');
        if (queryInstanceExtensions)
        {
            result = instanceFunctionPtr(m_instancePtr, m_systemId, bufferCount, &bufferCount, extensionBuffer.data());
        }
        else
        {
            result = deviceFunctionPtr(m_instancePtr, m_systemId, bufferCount, &bufferCount, extensionBuffer.data());
        }

        if (XR_FAILED(result))
        {
            SetStatus(queryInstanceExtensions ? "OpenXR Vulkan instance extension enumeration failed." : "OpenXR Vulkan device extension enumeration failed.");
            return false;
        }

        const std::string extensionText(extensionBuffer.data());
        std::istringstream stream(extensionText);
        std::string extensionName;
        while (stream >> extensionName)
        {
            if (std::find(extensions.begin(), extensions.end(), extensionName) == extensions.end())
            {
                extensions.push_back(extensionName);
            }
        }

        const std::string message = queryInstanceExtensions ? "OpenXR Vulkan instance extensions: " + extensionText + "\n" : "OpenXR Vulkan device extensions: " + extensionText + "\n";
        OutputDebugStringA(message.c_str());
        return true;
    }

    //Creates the OpenXR session.
    bool OpenXRUtility::CreateSession(VkInstance instancePtrGPU, VkPhysicalDevice physicalDevicePtrGPU, VkDevice devicePtrGPU, uint32_t queueFamilyIndex, uint32_t queueIndex)
    {
        VkPhysicalDevice runtimePhysicalDevicePtrGPU = m_requiredPhysicalDevicePtrGPU;
        if (runtimePhysicalDevicePtrGPU == VK_NULL_HANDLE)
        {
            runtimePhysicalDevicePtrGPU = QueryRequiredPhysicalDevice(instancePtrGPU);
            if (runtimePhysicalDevicePtrGPU == VK_NULL_HANDLE)
            {
                return false;
            }
        }

        if (runtimePhysicalDevicePtrGPU != physicalDevicePtrGPU)
        {
            SetStatus("OpenXR requires a different Vulkan physical device than the desktop renderer selected.");
            return false;
        }

        XrGraphicsBindingVulkanKHR graphicsBinding{};
        graphicsBinding.type = XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR;
        graphicsBinding.instance = instancePtrGPU;
        graphicsBinding.physicalDevice = physicalDevicePtrGPU;
        graphicsBinding.device = devicePtrGPU;
        graphicsBinding.queueFamilyIndex = queueFamilyIndex;
        graphicsBinding.queueIndex = queueIndex;

        XrSessionCreateInfo sessionInfo{};
        sessionInfo.type = XR_TYPE_SESSION_CREATE_INFO;
        sessionInfo.next = &graphicsBinding;
        sessionInfo.systemId = m_systemId;

        XrResult result = m_xrCreateSessionPtr(m_instancePtr, &sessionInfo, &m_sessionPtr);
        if (XR_FAILED(result) || m_sessionPtr == XR_NULL_HANDLE)
        {
            SetStatus("OpenXR session creation failed.");
            return false;
        }

        return true;
    }

    //Creates the OpenXR reference space.
    bool OpenXRUtility::CreateReferenceSpace(void)
    {
        XrReferenceSpaceCreateInfo spaceInfo{};
        spaceInfo.type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO;
        spaceInfo.referenceSpaceType = PreferredReferenceSpaceType();
        spaceInfo.poseInReferenceSpace.orientation.w = 1.0f;

        XrResult result = m_xrCreateReferenceSpacePtr(m_sessionPtr, &spaceInfo, &m_appSpacePtr);
        if (XR_FAILED(result) && spaceInfo.referenceSpaceType != XR_REFERENCE_SPACE_TYPE_LOCAL)
        {
            spaceInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
            result = m_xrCreateReferenceSpacePtr(m_sessionPtr, &spaceInfo, &m_appSpacePtr);
        }

        if (XR_FAILED(result) || m_appSpacePtr == XR_NULL_HANDLE)
        {
            SetStatus("OpenXR reference space creation failed.");
            return false;
        }

        return true;
    }

    //Creates OpenXR controller actions.
    bool OpenXRUtility::CreateControllerActions(void)
    {
        if (!m_controllerInputEnabled)
        {
            return true;
        }

        XrActionSetCreateInfo actionSetInfo{};
        actionSetInfo.type = XR_TYPE_ACTION_SET_CREATE_INFO;
        CopyText(actionSetInfo.actionSetName, sizeof(actionSetInfo.actionSetName), "graphics_app_controls");
        CopyText(actionSetInfo.localizedActionSetName, sizeof(actionSetInfo.localizedActionSetName), "Graphics App Controls");
        actionSetInfo.priority = 0;

        XrResult result = m_xrCreateActionSetPtr(m_instancePtr, &actionSetInfo, &m_actionSetPtr);
        if (XR_FAILED(result) || m_actionSetPtr == XR_NULL_HANDLE)
        {
            SetStatus("OpenXR controller action set creation failed.");
            return false;
        }

        auto createAction = [&](const char *namePtr, const char *localizedNamePtr, XrActionType actionType, XrAction &actionPtr) -> bool
        {
            XrActionCreateInfo actionInfo{};
            actionInfo.type = XR_TYPE_ACTION_CREATE_INFO;
            actionInfo.actionType = actionType;
            CopyText(actionInfo.actionName, sizeof(actionInfo.actionName), namePtr);
            CopyText(actionInfo.localizedActionName, sizeof(actionInfo.localizedActionName), localizedNamePtr);

            const XrResult actionResult = m_xrCreateActionPtr(m_actionSetPtr, &actionInfo, &actionPtr);
            return XR_SUCCEEDED(actionResult) && actionPtr != XR_NULL_HANDLE;
        };

        if (!createAction("move", "Move", XR_ACTION_TYPE_VECTOR2F_INPUT, m_moveActionPtr) || !createAction("turn", "Turn", XR_ACTION_TYPE_VECTOR2F_INPUT, m_turnActionPtr) ||
            !createAction("move_up", "Move Up", XR_ACTION_TYPE_BOOLEAN_INPUT, m_moveUpActionPtr) || !createAction("move_down", "Move Down", XR_ACTION_TYPE_BOOLEAN_INPUT, m_moveDownActionPtr))
        {
            SetStatus("OpenXR controller action creation failed.");
            return false;
        }

        XrPath oculusTouchProfile = XR_NULL_PATH;
        if (!StringToPath("/interaction_profiles/oculus/touch_controller", oculusTouchProfile))
        {
            SetStatus("OpenXR Oculus Touch interaction profile path failed.");
            return false;
        }

        std::vector<XrActionSuggestedBinding> bindings;
        auto addBinding = [&](XrAction actionPtr, const char *pathTextPtr) -> bool
        {
            XrPath path = XR_NULL_PATH;
            if (!StringToPath(pathTextPtr, path))
            {
                return false;
            }

            XrActionSuggestedBinding binding{};
            binding.action = actionPtr;
            binding.binding = path;
            bindings.push_back(binding);
            return true;
        };

        if (!addBinding(m_moveActionPtr, "/user/hand/left/input/thumbstick") || !addBinding(m_turnActionPtr, "/user/hand/right/input/thumbstick") ||
            !addBinding(m_moveUpActionPtr, "/user/hand/right/input/a/click") || !addBinding(m_moveDownActionPtr, "/user/hand/right/input/b/click"))
        {
            SetStatus("OpenXR controller binding path creation failed.");
            return false;
        }

        XrInteractionProfileSuggestedBinding suggestedBindings{};
        suggestedBindings.type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING;
        suggestedBindings.interactionProfile = oculusTouchProfile;
        suggestedBindings.countSuggestedBindings = static_cast<uint32_t>(bindings.size());
        suggestedBindings.suggestedBindings = bindings.data();

        result = m_xrSuggestInteractionProfileBindingsPtr(m_instancePtr, &suggestedBindings);
        if (XR_FAILED(result))
        {
            SetStatus("OpenXR controller binding suggestion failed.");
            return false;
        }

        XrSessionActionSetsAttachInfo attachInfo{};
        attachInfo.type = XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO;
        attachInfo.countActionSets = 1;
        attachInfo.actionSets = &m_actionSetPtr;

        result = m_xrAttachSessionActionSetsPtr(m_sessionPtr, &attachInfo);
        if (XR_FAILED(result))
        {
            SetStatus("OpenXR controller action set attach failed.");
            return false;
        }

        m_controllerActionsReady = true;
        OutputDebugStringA("OpenXR controller actions are ready.\n");
        return true;
    }

    //Releases OpenXR controller actions.
    void OpenXRUtility::CleanControllerActions(void)
    {
        m_controllerActionsReady = false;
        m_controllerState = {};

        if (m_moveDownActionPtr != XR_NULL_HANDLE && m_xrDestroyActionPtr != nullptr)
        {
            m_xrDestroyActionPtr(m_moveDownActionPtr);
            m_moveDownActionPtr = XR_NULL_HANDLE;
        }

        if (m_moveUpActionPtr != XR_NULL_HANDLE && m_xrDestroyActionPtr != nullptr)
        {
            m_xrDestroyActionPtr(m_moveUpActionPtr);
            m_moveUpActionPtr = XR_NULL_HANDLE;
        }

        if (m_turnActionPtr != XR_NULL_HANDLE && m_xrDestroyActionPtr != nullptr)
        {
            m_xrDestroyActionPtr(m_turnActionPtr);
            m_turnActionPtr = XR_NULL_HANDLE;
        }

        if (m_moveActionPtr != XR_NULL_HANDLE && m_xrDestroyActionPtr != nullptr)
        {
            m_xrDestroyActionPtr(m_moveActionPtr);
            m_moveActionPtr = XR_NULL_HANDLE;
        }

        if (m_actionSetPtr != XR_NULL_HANDLE && m_xrDestroyActionSetPtr != nullptr)
        {
            m_xrDestroyActionSetPtr(m_actionSetPtr);
            m_actionSetPtr = XR_NULL_HANDLE;
        }
    }

    //Creates OpenXR passthrough resources.
    bool OpenXRUtility::CreatePassthrough(void)
    {
        if (!m_passthroughExtensionEnabled)
        {
            return false;
        }

        XrPassthroughCreateInfoFB passthroughInfo{};
        passthroughInfo.type = XR_TYPE_PASSTHROUGH_CREATE_INFO_FB;
        passthroughInfo.flags = XR_PASSTHROUGH_IS_RUNNING_AT_CREATION_BIT_FB;

        XrResult result = m_xrCreatePassthroughFBPtr(m_sessionPtr, &passthroughInfo, &m_passthroughPtr);
        if (XR_FAILED(result) || m_passthroughPtr == XR_NULL_HANDLE)
        {
            SetStatus("OpenXR passthrough creation failed.");
            return false;
        }

        XrPassthroughLayerCreateInfoFB layerInfo{};
        layerInfo.type = XR_TYPE_PASSTHROUGH_LAYER_CREATE_INFO_FB;
        layerInfo.passthrough = m_passthroughPtr;
        layerInfo.flags = XR_PASSTHROUGH_IS_RUNNING_AT_CREATION_BIT_FB;
        layerInfo.purpose = XR_PASSTHROUGH_LAYER_PURPOSE_RECONSTRUCTION_FB;

        result = m_xrCreatePassthroughLayerFBPtr(m_sessionPtr, &layerInfo, &m_passthroughLayerPtr);
        if (XR_FAILED(result) || m_passthroughLayerPtr == XR_NULL_HANDLE)
        {
            SetStatus("OpenXR passthrough layer creation failed.");
            return false;
        }

        m_passthroughReady = true;
        OutputDebugStringA("OpenXR passthrough is ready.\n");
        return true;
    }

    //Releases OpenXR passthrough resources.
    void OpenXRUtility::CleanPassthrough(void)
    {
        m_passthroughReady = false;

        if (m_passthroughLayerPtr != XR_NULL_HANDLE)
        {
            if (m_xrPassthroughLayerPauseFBPtr != nullptr)
            {
                m_xrPassthroughLayerPauseFBPtr(m_passthroughLayerPtr);
            }

            if (m_xrDestroyPassthroughLayerFBPtr != nullptr)
            {
                m_xrDestroyPassthroughLayerFBPtr(m_passthroughLayerPtr);
            }

            m_passthroughLayerPtr = XR_NULL_HANDLE;
        }

        if (m_passthroughPtr != XR_NULL_HANDLE)
        {
            if (m_xrPassthroughPauseFBPtr != nullptr)
            {
                m_xrPassthroughPauseFBPtr(m_passthroughPtr);
            }

            if (m_xrDestroyPassthroughFBPtr != nullptr)
            {
                m_xrDestroyPassthroughFBPtr(m_passthroughPtr);
            }

            m_passthroughPtr = XR_NULL_HANDLE;
        }
    }

    //Creates OpenXR environment depth resources.
    bool OpenXRUtility::CreateEnvironmentDepth(void)
    {
        if (!m_environmentDepthExtensionEnabled || m_xrCreateEnvironmentDepthProviderMETAPtr == nullptr || m_xrCreateEnvironmentDepthSwapchainMETAPtr == nullptr ||
            m_xrEnumerateEnvironmentDepthSwapchainImagesMETAPtr == nullptr || m_xrGetEnvironmentDepthSwapchainStateMETAPtr == nullptr || m_sessionPtr == XR_NULL_HANDLE)
        {
            return false;
        }

        XrEnvironmentDepthProviderCreateInfoMETA providerInfo{};
        providerInfo.type = XR_TYPE_ENVIRONMENT_DEPTH_PROVIDER_CREATE_INFO_META;

        XrResult result = m_xrCreateEnvironmentDepthProviderMETAPtr(m_sessionPtr, &providerInfo, &m_environmentDepthProviderPtr);
        if (XR_FAILED(result) || m_environmentDepthProviderPtr == XR_NULL_HANDLE)
        {
            SetStatus("OpenXR environment depth provider creation failed: " + ResultToString(result) + ".");
            return false;
        }

        if (m_environmentDepthLogEnabled)
        {
            OutputDebugStringA("OpenXR environment depth provider was created.\n");
        }

        XrEnvironmentDepthSwapchainCreateInfoMETA swapchainInfo{};
        swapchainInfo.type = XR_TYPE_ENVIRONMENT_DEPTH_SWAPCHAIN_CREATE_INFO_META;

        result = m_xrCreateEnvironmentDepthSwapchainMETAPtr(m_environmentDepthProviderPtr, &swapchainInfo, &m_environmentDepthSwapchainPtr);
        if (XR_FAILED(result) || m_environmentDepthSwapchainPtr == XR_NULL_HANDLE)
        {
            SetStatus("OpenXR environment depth swapchain creation failed: " + ResultToString(result) + ".");
            return false;
        }

        uint32_t imageCount = 0;
        result = m_xrEnumerateEnvironmentDepthSwapchainImagesMETAPtr(m_environmentDepthSwapchainPtr, 0, &imageCount, nullptr);
        if (XR_FAILED(result) || imageCount == 0)
        {
            SetStatus("OpenXR environment depth image query failed: " + ResultToString(result) + ".");
            return false;
        }

        m_environmentDepthImages.assign(imageCount, XrSwapchainImageVulkanKHR{ XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR });
        result = m_xrEnumerateEnvironmentDepthSwapchainImagesMETAPtr(m_environmentDepthSwapchainPtr, imageCount, &imageCount, reinterpret_cast<XrSwapchainImageBaseHeader *>(m_environmentDepthImages.data()));
        if (XR_FAILED(result))
        {
            SetStatus("OpenXR environment depth Vulkan image enumeration failed: " + ResultToString(result) + ".");
            return false;
        }

        m_environmentDepthSwapchainState = {};
        m_environmentDepthSwapchainState.type = XR_TYPE_ENVIRONMENT_DEPTH_SWAPCHAIN_STATE_META;
        result = m_xrGetEnvironmentDepthSwapchainStateMETAPtr(m_environmentDepthSwapchainPtr, &m_environmentDepthSwapchainState);
        if (XR_FAILED(result))
        {
            SetStatus("OpenXR environment depth swapchain state query failed: " + ResultToString(result) + ".");
            return false;
        }

        if (m_environmentDepthLogEnabled)
        {
            const std::string message = "OpenXR environment depth swapchain has " + std::to_string(imageCount) + " images at " + std::to_string(m_environmentDepthSwapchainState.width) + " x " +
                std::to_string(m_environmentDepthSwapchainState.height) + ".\n";
            OutputDebugStringA(message.c_str());
        }

        if (!CreateEnvironmentDepthImageViews() || !CreateEnvironmentDepthSampler())
        {
            SetStatus("OpenXR environment depth Vulkan sampling resources failed.");
            return false;
        }

        if (m_environmentDepthHandRemovalRequested && m_environmentDepthHandRemovalSupported && m_xrSetEnvironmentDepthHandRemovalMETAPtr != nullptr)
        {
            XrEnvironmentDepthHandRemovalSetInfoMETA handRemovalInfo{};
            handRemovalInfo.type = XR_TYPE_ENVIRONMENT_DEPTH_HAND_REMOVAL_SET_INFO_META;
            handRemovalInfo.enabled = XR_TRUE;
            m_xrSetEnvironmentDepthHandRemovalMETAPtr(m_environmentDepthProviderPtr, &handRemovalInfo);
        }

        m_environmentDepthReady = true;
        m_passthroughDepthTestReady = true;
        if (m_environmentDepthLogEnabled)
        {
            OutputDebugStringA("OpenXR environment depth is ready.\n");
        }
        return true;
    }

    //Releases OpenXR environment depth resources.
    void OpenXRUtility::CleanEnvironmentDepth(void)
    {
        if (m_environmentDepthRunning && m_environmentDepthProviderPtr != XR_NULL_HANDLE && m_xrStopEnvironmentDepthProviderMETAPtr != nullptr)
        {
            m_xrStopEnvironmentDepthProviderMETAPtr(m_environmentDepthProviderPtr);
        }

        m_environmentDepthRunning = false;
        m_environmentDepthReady = false;
        m_environmentDepthAvailableThisFrame = false;
        m_passthroughDepthTestReady = false;
        m_environmentDepthAcquireFailureReported = false;
        m_environmentDepthAcquireSuccessReported = false;
        m_environmentDepthUnavailableReported = false;
        m_environmentDepthUnavailableCount = 0;

        CleanEnvironmentDepthImageViews();

        if (m_environmentDepthSamplerPtrGPU != VK_NULL_HANDLE && m_devicePtrGPU != VK_NULL_HANDLE)
        {
            vkDestroySampler(m_devicePtrGPU, m_environmentDepthSamplerPtrGPU, nullptr);
            m_environmentDepthSamplerPtrGPU = VK_NULL_HANDLE;
        }

        if (m_environmentDepthSwapchainPtr != XR_NULL_HANDLE && m_xrDestroyEnvironmentDepthSwapchainMETAPtr != nullptr)
        {
            m_xrDestroyEnvironmentDepthSwapchainMETAPtr(m_environmentDepthSwapchainPtr);
            m_environmentDepthSwapchainPtr = XR_NULL_HANDLE;
        }

        if (m_environmentDepthProviderPtr != XR_NULL_HANDLE && m_xrDestroyEnvironmentDepthProviderMETAPtr != nullptr)
        {
            m_xrDestroyEnvironmentDepthProviderMETAPtr(m_environmentDepthProviderPtr);
            m_environmentDepthProviderPtr = XR_NULL_HANDLE;
        }

        m_environmentDepthImages.clear();
        m_environmentDepthImage = {};
        m_environmentDepthSwapchainState = {};
        m_environmentDepthFormat = VK_FORMAT_UNDEFINED;
        m_environmentDepthImageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    }

    //Acquires OpenXR environment depth for one frame.
    bool OpenXRUtility::AcquireEnvironmentDepthImage(XrTime displayTime)
    {
        m_environmentDepthAvailableThisFrame = false;

        if (!m_environmentDepthRunning || m_environmentDepthProviderPtr == XR_NULL_HANDLE || m_xrAcquireEnvironmentDepthImageMETAPtr == nullptr || m_appSpacePtr == XR_NULL_HANDLE)
        {
            return false;
        }

        XrEnvironmentDepthImageAcquireInfoMETA acquireInfo{};
        acquireInfo.type = XR_TYPE_ENVIRONMENT_DEPTH_IMAGE_ACQUIRE_INFO_META;
        acquireInfo.space = m_appSpacePtr;
        acquireInfo.displayTime = displayTime;

        m_environmentDepthImage = {};
        m_environmentDepthImage.type = XR_TYPE_ENVIRONMENT_DEPTH_IMAGE_META;
        for (XrEnvironmentDepthImageViewMETA &view : m_environmentDepthImage.views)
        {
            view.type = XR_TYPE_ENVIRONMENT_DEPTH_IMAGE_VIEW_META;
        }

        const XrResult result = m_xrAcquireEnvironmentDepthImageMETAPtr(m_environmentDepthProviderPtr, &acquireInfo, &m_environmentDepthImage);
        if (result == XR_ENVIRONMENT_DEPTH_NOT_AVAILABLE_META)
        {
            ++m_environmentDepthUnavailableCount;
            if (m_environmentDepthLogEnabled && (!m_environmentDepthUnavailableReported || (m_environmentDepthUnavailableCount % 240U) == 0U))
            {
                const std::string message = "OpenXR environment depth is not available for this frame. Count = " + std::to_string(m_environmentDepthUnavailableCount) + ".\n";
                OutputDebugStringA(message.c_str());
                m_environmentDepthUnavailableReported = true;
            }

            return false;
        }

        if (XR_FAILED(result))
        {
            if (!m_environmentDepthAcquireFailureReported)
            {
                const std::string message = "OpenXR environment depth acquire failed: " + ResultToString(result) + ".\n";
                OutputDebugStringA(message.c_str());
                m_environmentDepthAcquireFailureReported = true;
            }

            return false;
        }

        if (m_environmentDepthImage.swapchainIndex >= m_environmentDepthImages.size())
        {
            if (!m_environmentDepthAcquireFailureReported)
            {
                OutputDebugStringA("OpenXR environment depth returned an invalid image index.\n");
                m_environmentDepthAcquireFailureReported = true;
            }

            return false;
        }

        m_environmentDepthAcquireFailureReported = false;
        m_environmentDepthUnavailableCount = 0;
        m_environmentDepthUnavailableReported = false;
        m_environmentDepthAvailableThisFrame = true;

        if (m_environmentDepthLogEnabled && !m_environmentDepthAcquireSuccessReported)
        {
            const float nearPlane = FiniteDepthNear(m_environmentDepthImage.nearZ);
            const float farPlane = FiniteDepthFar(m_environmentDepthImage.farZ, nearPlane, m_environmentDepthMaxMeters);
            const std::string message = "OpenXR environment depth acquired. Index = " + std::to_string(m_environmentDepthImage.swapchainIndex) + ", near = " +
                std::to_string(m_environmentDepthImage.nearZ) + ", far = " + std::to_string(m_environmentDepthImage.farZ) + ", using near = " + std::to_string(nearPlane) + ", using far = " +
                std::to_string(farPlane) + ".\n";
            OutputDebugStringA(message.c_str());
            m_environmentDepthAcquireSuccessReported = true;
        }

        return true;
    }

    //Creates Vulkan views for environment depth images.
    bool OpenXRUtility::CreateEnvironmentDepthImageViews(void)
    {
        if (m_environmentDepthImages.empty())
        {
            return false;
        }

        const std::array<std::pair<VkFormat, VkImageAspectFlags>, 4> candidates = {
            std::pair<VkFormat, VkImageAspectFlags>{ VK_FORMAT_D32_SFLOAT, VK_IMAGE_ASPECT_DEPTH_BIT }, std::pair<VkFormat, VkImageAspectFlags>{ VK_FORMAT_D16_UNORM, VK_IMAGE_ASPECT_DEPTH_BIT },
            std::pair<VkFormat, VkImageAspectFlags>{ VK_FORMAT_R32_SFLOAT, VK_IMAGE_ASPECT_COLOR_BIT }, std::pair<VkFormat, VkImageAspectFlags>{ VK_FORMAT_R16_UNORM, VK_IMAGE_ASPECT_COLOR_BIT }
        };

        for (const auto &candidate : candidates)
        {
            if (TryCreateEnvironmentDepthImageViews(candidate.first, candidate.second))
            {
                const std::string message = "OpenXR environment depth Vulkan image views were created with format " + std::to_string(static_cast<int>(candidate.first)) + ".\n";
                OutputDebugStringA(message.c_str());
                return true;
            }
        }

        return false;
    }

    //Creates Vulkan views for environment depth images with one format.
    bool OpenXRUtility::TryCreateEnvironmentDepthImageViews(VkFormat format, VkImageAspectFlags aspectFlags)
    {
        CleanEnvironmentDepthImageViews();

        m_environmentDepthImageViewsPtrGPU.reserve(m_environmentDepthImages.size());
        for (const XrSwapchainImageVulkanKHR &image : m_environmentDepthImages)
        {
            const VkImageView imageViewPtrGPU = CreateImageView(image.image, format, aspectFlags);
            if (imageViewPtrGPU == VK_NULL_HANDLE)
            {
                CleanEnvironmentDepthImageViews();
                return false;
            }

            m_environmentDepthImageViewsPtrGPU.push_back(imageViewPtrGPU);
        }

        m_environmentDepthFormat = format;
        m_environmentDepthImageLayout = (aspectFlags & VK_IMAGE_ASPECT_DEPTH_BIT) != 0 ? VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        return true;
    }

    //Releases Vulkan views for environment depth images.
    void OpenXRUtility::CleanEnvironmentDepthImageViews(void)
    {
        if (m_devicePtrGPU != VK_NULL_HANDLE)
        {
            for (VkImageView &imageViewPtrGPU : m_environmentDepthImageViewsPtrGPU)
            {
                if (imageViewPtrGPU != VK_NULL_HANDLE)
                {
                    vkDestroyImageView(m_devicePtrGPU, imageViewPtrGPU, nullptr);
                    imageViewPtrGPU = VK_NULL_HANDLE;
                }
            }
        }

        m_environmentDepthImageViewsPtrGPU.clear();
    }

    //Creates the environment depth sampler.
    bool OpenXRUtility::CreateEnvironmentDepthSampler(void)
    {
        if (m_environmentDepthSamplerPtrGPU != VK_NULL_HANDLE)
        {
            return true;
        }

        VkSamplerCreateInfo samplerInfo{};
        samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        samplerInfo.magFilter = VK_FILTER_NEAREST;
        samplerInfo.minFilter = VK_FILTER_NEAREST;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.anisotropyEnable = VK_FALSE;
        samplerInfo.maxAnisotropy = 1.0f;
        samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK;
        samplerInfo.unnormalizedCoordinates = VK_FALSE;
        samplerInfo.compareEnable = VK_FALSE;
        samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        samplerInfo.mipLodBias = 0.0f;
        samplerInfo.minLod = 0.0f;
        samplerInfo.maxLod = 0.0f;

        return vkCreateSampler(m_devicePtrGPU, &samplerInfo, nullptr, &m_environmentDepthSamplerPtrGPU) == VK_SUCCESS;
    }

    //Returns whether the full-screen depth debug view should draw.
    bool OpenXRUtility::FullScreenDepthDebugEnabled(void) const
    {
        return m_environmentDepthFullScreenDebugRequested && m_depthDebugResourcesReady && m_depthDebugPipelinePtrGPU != VK_NULL_HANDLE && m_depthDebugPipelineLayoutPtrGPU != VK_NULL_HANDLE &&
            m_depthDebugDescriptorSetPtrGPU != VK_NULL_HANDLE;
    }

    //Returns whether this frame needs Meta environment depth.
    bool OpenXRUtility::EnvironmentDepthFrameNeeded(void) const
    {
        return m_environmentDepthTextureSamplingEnabled || m_environmentDepthFullScreenDebugRequested;
    }

    //Creates the full-screen environment depth debug resources.
    bool OpenXRUtility::CreateDepthDebugResources(void)
    {
        if (m_depthDebugResourcesReady)
        {
            return true;
        }

        if (m_devicePtrGPU == VK_NULL_HANDLE || m_renderPassPtrGPU == VK_NULL_HANDLE || m_environmentDepthSamplerPtrGPU == VK_NULL_HANDLE)
        {
            return false;
        }

        VkDescriptorSetLayoutBinding depthLayoutBinding{};
        depthLayoutBinding.binding = 0;
        depthLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        depthLayoutBinding.descriptorCount = 1;
        depthLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutCreateInfo descriptorLayoutInfo{};
        descriptorLayoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        descriptorLayoutInfo.bindingCount = 1;
        descriptorLayoutInfo.pBindings = &depthLayoutBinding;

        if (vkCreateDescriptorSetLayout(m_devicePtrGPU, &descriptorLayoutInfo, nullptr, &m_depthDebugDescriptorSetLayoutPtrGPU) != VK_SUCCESS)
        {
            return false;
        }

        VkDescriptorPoolSize poolSize{};
        poolSize.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        poolSize.descriptorCount = 1;

        VkDescriptorPoolCreateInfo poolInfo{};
        poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        poolInfo.poolSizeCount = 1;
        poolInfo.pPoolSizes = &poolSize;
        poolInfo.maxSets = 1;

        if (vkCreateDescriptorPool(m_devicePtrGPU, &poolInfo, nullptr, &m_depthDebugDescriptorPoolPtrGPU) != VK_SUCCESS)
        {
            return false;
        }

        VkDescriptorSetAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        allocInfo.descriptorPool = m_depthDebugDescriptorPoolPtrGPU;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &m_depthDebugDescriptorSetLayoutPtrGPU;

        if (vkAllocateDescriptorSets(m_devicePtrGPU, &allocInfo, &m_depthDebugDescriptorSetPtrGPU) != VK_SUCCESS)
        {
            return false;
        }

        const auto vertexShaderCode = ReadShaderFile("shaders/xrDepthDebug.vert.spv");
        const auto fragmentShaderCode = ReadShaderFile("shaders/xrDepthDebug.frag.spv");
        const VkShaderModule vertexShaderModulePtrGPU = CreateShaderModule(vertexShaderCode);
        const VkShaderModule fragmentShaderModulePtrGPU = CreateShaderModule(fragmentShaderCode);

        VkPipelineShaderStageCreateInfo vertexShaderStageInfo{};
        vertexShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        vertexShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertexShaderStageInfo.module = vertexShaderModulePtrGPU;
        vertexShaderStageInfo.pName = "main";

        VkPipelineShaderStageCreateInfo fragmentShaderStageInfo{};
        fragmentShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        fragmentShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        fragmentShaderStageInfo.module = fragmentShaderModulePtrGPU;
        fragmentShaderStageInfo.pName = "main";

        const std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages = {
            vertexShaderStageInfo, fragmentShaderStageInfo
        };

        VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
        vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

        VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
        inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

        VkPipelineViewportStateCreateInfo viewportState{};
        viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        viewportState.viewportCount = 1;
        viewportState.scissorCount = 1;

        VkPipelineRasterizationStateCreateInfo rasterizer{};
        rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
        rasterizer.lineWidth = 1.0f;
        rasterizer.cullMode = VK_CULL_MODE_NONE;
        rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;

        VkPipelineMultisampleStateCreateInfo multisampling{};
        multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        multisampling.rasterizationSamples = m_sampleCount;

        VkPipelineDepthStencilStateCreateInfo depthStencil{};
        depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        depthStencil.depthTestEnable = VK_FALSE;
        depthStencil.depthWriteEnable = VK_FALSE;
        depthStencil.depthCompareOp = VK_COMPARE_OP_ALWAYS;

        VkPipelineColorBlendAttachmentState colorBlendAttachment{};
        colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        colorBlendAttachment.blendEnable = VK_FALSE;

        VkPipelineColorBlendStateCreateInfo colorBlending{};
        colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        colorBlending.attachmentCount = 1;
        colorBlending.pAttachments = &colorBlendAttachment;

        const std::array<VkDynamicState, 2> dynamicStates = {
            VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR
        };

        VkPipelineDynamicStateCreateInfo dynamicState{};
        dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
        dynamicState.pDynamicStates = dynamicStates.data();

        VkPushConstantRange pushConstantRange{};
        pushConstantRange.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        pushConstantRange.offset = 0;
        pushConstantRange.size = sizeof(DepthDebugConstants);

        VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
        pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pipelineLayoutInfo.setLayoutCount = 1;
        pipelineLayoutInfo.pSetLayouts = &m_depthDebugDescriptorSetLayoutPtrGPU;
        pipelineLayoutInfo.pushConstantRangeCount = 1;
        pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

        if (vkCreatePipelineLayout(m_devicePtrGPU, &pipelineLayoutInfo, nullptr, &m_depthDebugPipelineLayoutPtrGPU) != VK_SUCCESS)
        {
            vkDestroyShaderModule(m_devicePtrGPU, fragmentShaderModulePtrGPU, nullptr);
            vkDestroyShaderModule(m_devicePtrGPU, vertexShaderModulePtrGPU, nullptr);
            return false;
        }

        VkGraphicsPipelineCreateInfo pipelineInfo{};
        pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        pipelineInfo.stageCount = static_cast<uint32_t>(shaderStages.size());
        pipelineInfo.pStages = shaderStages.data();
        pipelineInfo.pVertexInputState = &vertexInputInfo;
        pipelineInfo.pInputAssemblyState = &inputAssembly;
        pipelineInfo.pViewportState = &viewportState;
        pipelineInfo.pRasterizationState = &rasterizer;
        pipelineInfo.pMultisampleState = &multisampling;
        pipelineInfo.pDepthStencilState = &depthStencil;
        pipelineInfo.pColorBlendState = &colorBlending;
        pipelineInfo.pDynamicState = &dynamicState;
        pipelineInfo.layout = m_depthDebugPipelineLayoutPtrGPU;
        pipelineInfo.renderPass = m_renderPassPtrGPU;
        pipelineInfo.subpass = 0;

        const VkResult pipelineResult = vkCreateGraphicsPipelines(m_devicePtrGPU, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_depthDebugPipelinePtrGPU);

        vkDestroyShaderModule(m_devicePtrGPU, fragmentShaderModulePtrGPU, nullptr);
        vkDestroyShaderModule(m_devicePtrGPU, vertexShaderModulePtrGPU, nullptr);

        if (pipelineResult != VK_SUCCESS)
        {
            return false;
        }

        m_depthDebugResourcesReady = true;
        OutputDebugStringA("OpenXR full-screen environment depth debug view is ready.\n");
        return true;
    }

    //Releases the full-screen environment depth debug resources.
    void OpenXRUtility::CleanDepthDebugResources(void)
    {
        m_depthDebugResourcesReady = false;
        m_depthDebugDescriptorSetPtrGPU = VK_NULL_HANDLE;

        if (m_devicePtrGPU == VK_NULL_HANDLE)
        {
            return;
        }

        if (m_depthDebugPipelinePtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyPipeline(m_devicePtrGPU, m_depthDebugPipelinePtrGPU, nullptr);
            m_depthDebugPipelinePtrGPU = VK_NULL_HANDLE;
        }

        if (m_depthDebugPipelineLayoutPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyPipelineLayout(m_devicePtrGPU, m_depthDebugPipelineLayoutPtrGPU, nullptr);
            m_depthDebugPipelineLayoutPtrGPU = VK_NULL_HANDLE;
        }

        if (m_depthDebugDescriptorPoolPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorPool(m_devicePtrGPU, m_depthDebugDescriptorPoolPtrGPU, nullptr);
            m_depthDebugDescriptorPoolPtrGPU = VK_NULL_HANDLE;
        }

        if (m_depthDebugDescriptorSetLayoutPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyDescriptorSetLayout(m_devicePtrGPU, m_depthDebugDescriptorSetLayoutPtrGPU, nullptr);
            m_depthDebugDescriptorSetLayoutPtrGPU = VK_NULL_HANDLE;
        }
    }

    //Draws a full-screen environment depth debug image.
    void OpenXRUtility::DrawEnvironmentDepthDebug(VkCommandBuffer commandBufferPtrGPU, const EyeRenderInfo &renderInfo)
    {
        if (!m_environmentDepthFullScreenDebugRequested)
        {
            return;
        }

        if (!renderInfo.m_environmentDepthAvailable)
        {
            ClearDepthDebugStatus(commandBufferPtrGPU, renderInfo.m_extent, { 1.0f, 0.0f, 0.0f, 1.0f });
            return;
        }

        if (!m_environmentDepthTextureDebugRequested)
        {
            ClearDepthDebugStatus(commandBufferPtrGPU, renderInfo.m_extent, { 0.0f, 0.85f, 0.15f, 1.0f });
            return;
        }

        if (!FullScreenDepthDebugEnabled())
        {
            ClearDepthDebugStatus(commandBufferPtrGPU, renderInfo.m_extent, { 1.0f, 1.0f, 0.0f, 1.0f });
            return;
        }

        if (renderInfo.m_environmentDepthImageViewPtrGPU == VK_NULL_HANDLE || renderInfo.m_environmentDepthSamplerPtrGPU == VK_NULL_HANDLE)
        {
            ClearDepthDebugStatus(commandBufferPtrGPU, renderInfo.m_extent, { 1.0f, 0.0f, 1.0f, 1.0f });
            return;
        }

        VkDescriptorImageInfo imageInfo{};
        imageInfo.sampler = renderInfo.m_environmentDepthSamplerPtrGPU;
        imageInfo.imageView = renderInfo.m_environmentDepthImageViewPtrGPU;
        imageInfo.imageLayout = renderInfo.m_environmentDepthImageLayout;

        VkWriteDescriptorSet descriptorWrite{};
        descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        descriptorWrite.dstSet = m_depthDebugDescriptorSetPtrGPU;
        descriptorWrite.dstBinding = 0;
        descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        descriptorWrite.descriptorCount = 1;
        descriptorWrite.pImageInfo = &imageInfo;
        vkUpdateDescriptorSets(m_devicePtrGPU, 1, &descriptorWrite, 0, nullptr);

        VkViewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = static_cast<float>(renderInfo.m_extent.width);
        viewport.height = static_cast<float>(renderInfo.m_extent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;

        VkRect2D scissor{};
        scissor.offset = { 0, 0 };
        scissor.extent = renderInfo.m_extent;

        const float nearPlane = FiniteDepthNear(renderInfo.m_environmentDepthNearZ);
        const float farPlane = FiniteDepthFar(renderInfo.m_environmentDepthFarZ, nearPlane, m_environmentDepthMaxMeters);

        DepthDebugConstants constants{};
        constants.m_settings = {
            m_environmentDepthMaxMeters, m_environmentDepthValueMode, nearPlane, farPlane
        };

        vkCmdBindPipeline(commandBufferPtrGPU, VK_PIPELINE_BIND_POINT_GRAPHICS, m_depthDebugPipelinePtrGPU);
        vkCmdSetViewport(commandBufferPtrGPU, 0, 1, &viewport);
        vkCmdSetScissor(commandBufferPtrGPU, 0, 1, &scissor);
        vkCmdBindDescriptorSets(commandBufferPtrGPU, VK_PIPELINE_BIND_POINT_GRAPHICS, m_depthDebugPipelineLayoutPtrGPU, 0, 1, &m_depthDebugDescriptorSetPtrGPU, 0, nullptr);
        vkCmdPushConstants(commandBufferPtrGPU, m_depthDebugPipelineLayoutPtrGPU, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(constants), &constants);
        vkCmdDraw(commandBufferPtrGPU, 3, 1, 0, 0);
    }

    //Clears the eye to a full-screen depth debug status color.
    void OpenXRUtility::ClearDepthDebugStatus(VkCommandBuffer commandBufferPtrGPU, VkExtent2D extent, const Float4 &color) const
    {
        VkClearAttachment clearAttachment{};
        clearAttachment.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        clearAttachment.colorAttachment = 0;
        clearAttachment.clearValue.color.float32[0] = color.x;
        clearAttachment.clearValue.color.float32[1] = color.y;
        clearAttachment.clearValue.color.float32[2] = color.z;
        clearAttachment.clearValue.color.float32[3] = color.w;

        VkClearRect clearRect{};
        clearRect.rect.offset = { 0, 0 };
        clearRect.rect.extent = extent;
        clearRect.baseArrayLayer = 0;
        clearRect.layerCount = 1;

        vkCmdClearAttachments(commandBufferPtrGPU, 1, &clearAttachment, 1, &clearRect);
    }

    //Creates a Vulkan shader module.
    VkShaderModule OpenXRUtility::CreateShaderModule(const std::vector<char> &code) const
    {
        if (code.empty() || (code.size() % sizeof(uint32_t)) != 0U)
        {
            throw std::runtime_error("Invalid OpenXR debug shader code.");
        }

        VkShaderModuleCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = code.size();
        createInfo.pCode = reinterpret_cast<const uint32_t *>(code.data());

        VkShaderModule shaderModulePtrGPU = VK_NULL_HANDLE;
        if (vkCreateShaderModule(m_devicePtrGPU, &createInfo, nullptr, &shaderModulePtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create OpenXR debug shader module.");
        }

        return shaderModulePtrGPU;
    }

    //Loads a compiled shader file.
    std::vector<char> OpenXRUtility::ReadShaderFile(const std::string &relativePath) const
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
                continue;
            }

            const std::size_t fileSize = static_cast<std::size_t>(endPosition);
            if (fileSize == 0)
            {
                return {};
            }

            std::vector<char> buffer(fileSize);
            file.seekg(0);
            file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            file.close();
            return buffer;
        }

        throw std::runtime_error("Failed to open OpenXR debug shader file.");
    }

    //Converts an OpenXR result to readable text.
    std::string OpenXRUtility::ResultToString(XrResult result) const
    {
        if (m_instancePtr != XR_NULL_HANDLE && m_xrResultToStringPtr != nullptr)
        {
            char text[XR_MAX_RESULT_STRING_SIZE]{};
            if (XR_SUCCEEDED(m_xrResultToStringPtr(m_instancePtr, result, text)))
            {
                return text;
            }
        }

        return std::to_string(static_cast<int>(result));
    }

    //Converts an OpenXR path string.
    bool OpenXRUtility::StringToPath(const char *pathTextPtr, XrPath &path) const
    {
        path = XR_NULL_PATH;
        if (pathTextPtr == nullptr || m_xrStringToPathPtr == nullptr)
        {
            return false;
        }

        const XrResult result = m_xrStringToPathPtr(m_instancePtr, pathTextPtr, &path);
        return XR_SUCCEEDED(result) && path != XR_NULL_PATH;
    }

    //Creates OpenXR view and swapchain resources.
    bool OpenXRUtility::CreateViewResources(VkFormat preferredColorFormat)
    {
        uint32_t viewCount = 0;
        XrResult result = m_xrEnumerateViewConfigurationViewsPtr(m_instancePtr, m_systemId, m_viewConfigurationType, 0, &viewCount, nullptr);
        if (XR_FAILED(result) || viewCount == 0)
        {
            SetStatus("OpenXR view configuration query failed.");
            return false;
        }

        m_viewConfigurationViews.assign(viewCount, XrViewConfigurationView{ XR_TYPE_VIEW_CONFIGURATION_VIEW });
        result = m_xrEnumerateViewConfigurationViewsPtr(m_instancePtr, m_systemId, m_viewConfigurationType, viewCount, &viewCount, m_viewConfigurationViews.data());
        if (XR_FAILED(result))
        {
            SetStatus("OpenXR view configuration enumeration failed.");
            return false;
        }

        m_views.assign(viewCount, XrView{ XR_TYPE_VIEW });
        m_projectionViews.assign(viewCount, XrCompositionLayerProjectionView{ XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW });
        m_depthProjectionViews.assign(viewCount, XrCompositionLayerDepthInfoKHR{ XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR });
        m_swapchains.resize(viewCount);

        for (uint32_t viewIndex = 0; viewIndex < viewCount; ++viewIndex)
        {
            if (!CreateSwapchain(viewIndex, preferredColorFormat))
            {
                return false;
            }
        }

        return true;
    }

    //Creates one OpenXR swapchain.
    bool OpenXRUtility::CreateSwapchain(uint32_t viewIndex, VkFormat preferredColorFormat)
    {
        uint32_t formatCount = 0;
        XrResult result = m_xrEnumerateSwapchainFormatsPtr(m_sessionPtr, 0, &formatCount, nullptr);
        if (XR_FAILED(result) || formatCount == 0)
        {
            SetStatus("OpenXR swapchain format query failed.");
            return false;
        }

        std::vector<int64_t> formats(formatCount);
        result = m_xrEnumerateSwapchainFormatsPtr(m_sessionPtr, formatCount, &formatCount, formats.data());
        if (XR_FAILED(result))
        {
            SetStatus("OpenXR swapchain format enumeration failed.");
            return false;
        }

        SwapchainData &swapchain = m_swapchains[viewIndex];
        swapchain.m_format = ChooseColorFormat(formats, preferredColorFormat);

        const XrViewConfigurationView &view = m_viewConfigurationViews[viewIndex];
        const float scaledWidth = static_cast<float>(view.recommendedImageRectWidth) * m_renderScale;
        const float scaledHeight = static_cast<float>(view.recommendedImageRectHeight) * m_renderScale;
        swapchain.m_width = std::clamp(static_cast<uint32_t>(scaledWidth), 1U, view.maxImageRectWidth);
        swapchain.m_height = std::clamp(static_cast<uint32_t>(scaledHeight), 1U, view.maxImageRectHeight);

        XrSwapchainCreateInfo createInfo{};
        createInfo.type = XR_TYPE_SWAPCHAIN_CREATE_INFO;
        createInfo.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT;
        createInfo.format = swapchain.m_format;
        createInfo.sampleCount = 1;
        createInfo.width = swapchain.m_width;
        createInfo.height = swapchain.m_height;
        createInfo.faceCount = 1;
        createInfo.arraySize = 1;
        createInfo.mipCount = 1;

        result = m_xrCreateSwapchainPtr(m_sessionPtr, &createInfo, &swapchain.m_swapchainPtr);
        if (XR_FAILED(result) || swapchain.m_swapchainPtr == XR_NULL_HANDLE)
        {
            SetStatus("OpenXR swapchain creation failed.");
            return false;
        }

        uint32_t imageCount = 0;
        result = m_xrEnumerateSwapchainImagesPtr(swapchain.m_swapchainPtr, 0, &imageCount, nullptr);
        if (XR_FAILED(result) || imageCount == 0)
        {
            SetStatus("OpenXR swapchain image query failed.");
            return false;
        }

        swapchain.m_images.assign(imageCount, XrSwapchainImageVulkanKHR{ XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR });
        result = m_xrEnumerateSwapchainImagesPtr(swapchain.m_swapchainPtr, imageCount, &imageCount, reinterpret_cast<XrSwapchainImageBaseHeader *>(swapchain.m_images.data()));
        if (XR_FAILED(result))
        {
            SetStatus("OpenXR Vulkan swapchain image enumeration failed.");
            return false;
        }

        return true;
    }

    //Creates eye render resources for all swapchains.
    bool OpenXRUtility::CreateRenderResources(void)
    {
        m_passthroughDepthTestReady = false;

        if (m_devicePtrGPU == VK_NULL_HANDLE || m_physicalDevicePtrGPU == VK_NULL_HANDLE || m_renderPassPtrGPU == VK_NULL_HANDLE || m_colorFormat == VK_FORMAT_UNDEFINED ||
            m_depthFormat == VK_FORMAT_UNDEFINED)
        {
            return false;
        }

        bool useCompositionDepthSwapchain = false;

        if (useCompositionDepthSwapchain)
        {
            const uint32_t requestedSampleCount = SampleCountValue(m_sampleCount);
            for (const XrViewConfigurationView &view : m_viewConfigurationViews)
            {
                if (requestedSampleCount > view.maxSwapchainSampleCount)
                {
                    useCompositionDepthSwapchain = false;
                    OutputDebugStringA("OpenXR passthrough depth test is disabled because the runtime does not support the scene sample count.\n");
                    break;
                }
            }
        }

        if (useCompositionDepthSwapchain)
        {
            const uint32_t swapchainCount = static_cast<uint32_t>(m_swapchains.size());
            for (uint32_t viewIndex = 0; viewIndex < swapchainCount; ++viewIndex)
            {
                if (!CreateDepthSwapchain(viewIndex, m_swapchains[viewIndex]))
                {
                    useCompositionDepthSwapchain = false;
                    OutputDebugStringA("OpenXR passthrough depth test is disabled because a depth swapchain could not be created.\n");
                    break;
                }
            }
        }

        if (!useCompositionDepthSwapchain)
        {
            for (SwapchainData &swapchain : m_swapchains)
            {
                CleanDepthSwapchain(swapchain);
            }
        }

        for (SwapchainData &swapchain : m_swapchains)
        {
            if (static_cast<VkFormat>(swapchain.m_format) != m_colorFormat)
            {
                SetStatus("OpenXR swapchain format does not match the scene render pass format.");
                return false;
            }

            if (!CreateSwapchainRenderResources(swapchain))
            {
                return false;
            }
        }

        m_passthroughDepthTestReady = m_environmentDepthReady;
        if (m_passthroughDepthTestReady)
        {
            OutputDebugStringA("OpenXR environment depth is available to XR rendering.\n");
        }

        if (m_environmentDepthTextureDebugRequested && m_passthroughDepthTestReady && !CreateDepthDebugResources())
        {
            CleanDepthDebugResources();
            OutputDebugStringA("OpenXR full-screen environment depth debug view is disabled.\n");
        }

        return true;
    }

    //Creates one OpenXR depth swapchain.
    bool OpenXRUtility::CreateDepthSwapchain(uint32_t viewIndex, SwapchainData &swapchain)
    {
        uint32_t formatCount = 0;
        XrResult result = m_xrEnumerateSwapchainFormatsPtr(m_sessionPtr, 0, &formatCount, nullptr);
        if (XR_FAILED(result) || formatCount == 0)
        {
            return false;
        }

        std::vector<int64_t> formats(formatCount);
        result = m_xrEnumerateSwapchainFormatsPtr(m_sessionPtr, formatCount, &formatCount, formats.data());
        if (XR_FAILED(result))
        {
            return false;
        }

        const int64_t depthFormat = static_cast<int64_t>(m_depthFormat);
        if (std::find(formats.begin(), formats.end(), depthFormat) == formats.end())
        {
            OutputDebugStringA("OpenXR runtime did not report the app depth format as a swapchain format.\n");
            return false;
        }

        const XrViewConfigurationView &view = m_viewConfigurationViews[viewIndex];
        const uint32_t requestedSampleCount = SampleCountValue(m_sampleCount);

        XrSwapchainCreateInfo createInfo{};
        createInfo.type = XR_TYPE_SWAPCHAIN_CREATE_INFO;
        createInfo.usageFlags = XR_SWAPCHAIN_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
        createInfo.format = depthFormat;
        createInfo.sampleCount = requestedSampleCount;
        createInfo.width = swapchain.m_width;
        createInfo.height = swapchain.m_height;
        createInfo.faceCount = 1;
        createInfo.arraySize = 1;
        createInfo.mipCount = 1;

        if (requestedSampleCount > view.maxSwapchainSampleCount)
        {
            return false;
        }

        result = m_xrCreateSwapchainPtr(m_sessionPtr, &createInfo, &swapchain.m_depthSwapchainPtr);
        if (XR_FAILED(result) || swapchain.m_depthSwapchainPtr == XR_NULL_HANDLE)
        {
            return false;
        }

        uint32_t imageCount = 0;
        result = m_xrEnumerateSwapchainImagesPtr(swapchain.m_depthSwapchainPtr, 0, &imageCount, nullptr);
        if (XR_FAILED(result) || imageCount == 0)
        {
            return false;
        }

        swapchain.m_depthImages.assign(imageCount, XrSwapchainImageVulkanKHR{ XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR });
        result = m_xrEnumerateSwapchainImagesPtr(swapchain.m_depthSwapchainPtr, imageCount, &imageCount, reinterpret_cast<XrSwapchainImageBaseHeader *>(swapchain.m_depthImages.data()));
        if (XR_FAILED(result))
        {
            return false;
        }

        swapchain.m_depthFormat = depthFormat;
        return true;
    }

    //Creates eye render resources for one swapchain.
    bool OpenXRUtility::CreateSwapchainRenderResources(SwapchainData &swapchain)
    {
        const bool useMultisampling = m_sampleCount != VK_SAMPLE_COUNT_1_BIT;
        const bool useDepthSwapchain = swapchain.m_depthSwapchainPtr != XR_NULL_HANDLE && !swapchain.m_depthImages.empty();

        swapchain.m_imageViewsPtrGPU.reserve(swapchain.m_images.size());
        for (const XrSwapchainImageVulkanKHR &image : swapchain.m_images)
        {
            const VkImageView imageViewPtrGPU = CreateImageView(image.image, m_colorFormat, VK_IMAGE_ASPECT_COLOR_BIT);
            if (imageViewPtrGPU == VK_NULL_HANDLE)
            {
                return false;
            }

            swapchain.m_imageViewsPtrGPU.push_back(imageViewPtrGPU);
        }

        if (useMultisampling)
        {
            if (!CreateImage(swapchain.m_width, swapchain.m_height, m_colorFormat, VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT, m_sampleCount,
                swapchain.m_colorImagePtrGPU, swapchain.m_colorImageMemoryPtrGPU))
            {
                return false;
            }

            swapchain.m_colorImageViewPtrGPU = CreateImageView(swapchain.m_colorImagePtrGPU, m_colorFormat, VK_IMAGE_ASPECT_COLOR_BIT);
            if (swapchain.m_colorImageViewPtrGPU == VK_NULL_HANDLE)
            {
                return false;
            }
        }

        if (useDepthSwapchain)
        {
            swapchain.m_depthImageViewsPtrGPU.reserve(swapchain.m_depthImages.size());
            for (const XrSwapchainImageVulkanKHR &depthImage : swapchain.m_depthImages)
            {
                const VkImageView depthImageViewPtrGPU = CreateImageView(depthImage.image, m_depthFormat, VK_IMAGE_ASPECT_DEPTH_BIT);
                if (depthImageViewPtrGPU == VK_NULL_HANDLE)
                {
                    return false;
                }

                swapchain.m_depthImageViewsPtrGPU.push_back(depthImageViewPtrGPU);
            }
        }
        else
        {
            if (!CreateImage(swapchain.m_width, swapchain.m_height, m_depthFormat, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, m_sampleCount, swapchain.m_depthImagePtrGPU,
                swapchain.m_depthImageMemoryPtrGPU))
            {
                return false;
            }

            swapchain.m_depthImageViewPtrGPU = CreateImageView(swapchain.m_depthImagePtrGPU, m_depthFormat, VK_IMAGE_ASPECT_DEPTH_BIT);
            if (swapchain.m_depthImageViewPtrGPU == VK_NULL_HANDLE)
            {
                return false;
            }
        }

        const std::size_t depthViewCount = useDepthSwapchain ? swapchain.m_depthImageViewsPtrGPU.size() : 1U;
        swapchain.m_framebuffersPtrGPU.resize(swapchain.m_imageViewsPtrGPU.size() * depthViewCount, VK_NULL_HANDLE);
        for (std::size_t imageIndex = 0; imageIndex < swapchain.m_imageViewsPtrGPU.size(); ++imageIndex)
        {
            for (std::size_t depthIndex = 0; depthIndex < depthViewCount; ++depthIndex)
            {
                const VkImageView depthImageViewPtrGPU = useDepthSwapchain ? swapchain.m_depthImageViewsPtrGPU[depthIndex] : swapchain.m_depthImageViewPtrGPU;

                std::vector<VkImageView> attachments;
                if (useMultisampling)
                {
                    attachments = {
                        swapchain.m_colorImageViewPtrGPU, depthImageViewPtrGPU, swapchain.m_imageViewsPtrGPU[imageIndex]
                    };
                }
                else
                {
                    attachments = {
                        swapchain.m_imageViewsPtrGPU[imageIndex], depthImageViewPtrGPU
                    };
                }

                VkFramebufferCreateInfo framebufferInfo{};
                framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
                framebufferInfo.renderPass = m_renderPassPtrGPU;
                framebufferInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
                framebufferInfo.pAttachments = attachments.data();
                framebufferInfo.width = swapchain.m_width;
                framebufferInfo.height = swapchain.m_height;
                framebufferInfo.layers = 1;

                const std::size_t framebufferIndex = (imageIndex * depthViewCount) + depthIndex;
                if (vkCreateFramebuffer(m_devicePtrGPU, &framebufferInfo, nullptr, &swapchain.m_framebuffersPtrGPU[framebufferIndex]) != VK_SUCCESS)
                {
                    return false;
                }
            }
        }

        return true;
    }

    //Releases one OpenXR depth swapchain.
    void OpenXRUtility::CleanDepthSwapchain(SwapchainData &swapchain)
    {
        if (m_devicePtrGPU != VK_NULL_HANDLE)
        {
            for (VkImageView &imageViewPtrGPU : swapchain.m_depthImageViewsPtrGPU)
            {
                if (imageViewPtrGPU != VK_NULL_HANDLE)
                {
                    vkDestroyImageView(m_devicePtrGPU, imageViewPtrGPU, nullptr);
                    imageViewPtrGPU = VK_NULL_HANDLE;
                }
            }
        }

        swapchain.m_depthImageViewsPtrGPU.clear();
        swapchain.m_depthImages.clear();
        swapchain.m_depthFormat = 0;

        if (swapchain.m_depthSwapchainPtr != XR_NULL_HANDLE && m_xrDestroySwapchainPtr != nullptr)
        {
            m_xrDestroySwapchainPtr(swapchain.m_depthSwapchainPtr);
            swapchain.m_depthSwapchainPtr = XR_NULL_HANDLE;
        }
    }

    //Creates one Vulkan image for XR rendering.
    bool OpenXRUtility::CreateImage(
        uint32_t width,
        uint32_t height,
        VkFormat format,
        VkImageUsageFlags usage,
        VkSampleCountFlagBits sampleCount,
        VkImage &imagePtrGPU,
        VkDeviceMemory &imageMemoryPtrGPU
    ) const
    {
        VkImageCreateInfo imageInfo{};
        imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.extent.width = width;
        imageInfo.extent.height = height;
        imageInfo.extent.depth = 1;
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 1;
        imageInfo.format = format;
        imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        imageInfo.usage = usage;
        imageInfo.samples = sampleCount;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        if (vkCreateImage(m_devicePtrGPU, &imageInfo, nullptr, &imagePtrGPU) != VK_SUCCESS)
        {
            return false;
        }

        VkMemoryRequirements memoryRequirements{};
        vkGetImageMemoryRequirements(m_devicePtrGPU, imagePtrGPU, &memoryRequirements);

        uint32_t memoryTypeIndex = 0;
        if (!FindMemoryType(memoryRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, memoryTypeIndex))
        {
            return false;
        }

        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memoryRequirements.size;
        allocInfo.memoryTypeIndex = memoryTypeIndex;

        if (vkAllocateMemory(m_devicePtrGPU, &allocInfo, nullptr, &imageMemoryPtrGPU) != VK_SUCCESS)
        {
            return false;
        }

        return vkBindImageMemory(m_devicePtrGPU, imagePtrGPU, imageMemoryPtrGPU, 0) == VK_SUCCESS;
    }

    //Creates one Vulkan image view for XR rendering.
    VkImageView OpenXRUtility::CreateImageView(VkImage imagePtrGPU, VkFormat format, VkImageAspectFlags aspectFlags) const
    {
        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = imagePtrGPU;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = format;
        viewInfo.subresourceRange.aspectMask = aspectFlags;
        viewInfo.subresourceRange.baseMipLevel = 0;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.baseArrayLayer = 0;
        viewInfo.subresourceRange.layerCount = 1;

        VkImageView imageViewPtrGPU = VK_NULL_HANDLE;
        if (vkCreateImageView(m_devicePtrGPU, &viewInfo, nullptr, &imageViewPtrGPU) != VK_SUCCESS)
        {
            return VK_NULL_HANDLE;
        }

        return imageViewPtrGPU;
    }

    //Finds a matching Vulkan memory type.
    bool OpenXRUtility::FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties, uint32_t &memoryTypeIndex) const
    {
        VkPhysicalDeviceMemoryProperties memoryProperties{};
        vkGetPhysicalDeviceMemoryProperties(m_physicalDevicePtrGPU, &memoryProperties);

        for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; ++i)
        {
            if ((typeFilter & (1U << i)) && (memoryProperties.memoryTypes[i].propertyFlags & properties) == properties)
            {
                memoryTypeIndex = i;
                return true;
            }
        }

        return false;
    }

    //Begins the OpenXR session.
    void OpenXRUtility::BeginSession(void)
    {
        if (!m_sessionReady || m_sessionRunning || m_sessionPtr == XR_NULL_HANDLE)
        {
            return;
        }

        XrSessionBeginInfo beginInfo{};
        beginInfo.type = XR_TYPE_SESSION_BEGIN_INFO;
        beginInfo.primaryViewConfigurationType = m_viewConfigurationType;

        const XrResult result = m_xrBeginSessionPtr(m_sessionPtr, &beginInfo);
        if (XR_SUCCEEDED(result))
        {
            m_sessionRunning = true;
            if (m_passthroughReady)
            {
                m_xrPassthroughStartFBPtr(m_passthroughPtr);
                m_xrPassthroughLayerResumeFBPtr(m_passthroughLayerPtr);
            }

            if (m_environmentDepthReady && m_xrStartEnvironmentDepthProviderMETAPtr != nullptr && m_environmentDepthProviderPtr != XR_NULL_HANDLE)
            {
                const XrResult startResult = m_xrStartEnvironmentDepthProviderMETAPtr(m_environmentDepthProviderPtr);
                if (XR_SUCCEEDED(startResult))
                {
                    m_environmentDepthRunning = true;
                    if (m_environmentDepthLogEnabled)
                    {
                        OutputDebugStringA("OpenXR environment depth provider started.\n");
                    }
                }
                else
                {
                    const std::string message = "OpenXR environment depth provider start failed: " + ResultToString(startResult) + ".\n";
                    OutputDebugStringA(message.c_str());
                }
            }

            SetStatus("OpenXR session started.");
        }
        else
        {
            SetStatus("OpenXR session begin failed.");
        }
    }

    //Ends the OpenXR session.
    void OpenXRUtility::EndSession(void)
    {
        if (m_sessionRunning && m_passthroughReady)
        {
            m_xrPassthroughLayerPauseFBPtr(m_passthroughLayerPtr);
            m_xrPassthroughPauseFBPtr(m_passthroughPtr);
        }

        if (m_sessionRunning && m_environmentDepthRunning && m_environmentDepthProviderPtr != XR_NULL_HANDLE && m_xrStopEnvironmentDepthProviderMETAPtr != nullptr)
        {
            m_xrStopEnvironmentDepthProviderMETAPtr(m_environmentDepthProviderPtr);
            m_environmentDepthRunning = false;
            m_environmentDepthAvailableThisFrame = false;
        }

        if (m_sessionRunning && m_xrEndSessionPtr != nullptr && m_sessionPtr != XR_NULL_HANDLE)
        {
            m_xrEndSessionPtr(m_sessionPtr);
        }

        m_sessionRunning = false;
    }

    //Renders one OpenXR eye swapchain image.
    bool OpenXRUtility::RenderEye(uint32_t viewIndex, const Float4 &clearColor, const EyeRenderCallback &renderCallback)
    {
        SwapchainData &swapchain = m_swapchains[viewIndex];

        XrSwapchainImageAcquireInfo acquireInfo{};
        acquireInfo.type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO;

        uint32_t imageIndex = 0;
        XrResult result = m_xrAcquireSwapchainImagePtr(swapchain.m_swapchainPtr, &acquireInfo, &imageIndex);
        if (XR_FAILED(result) || imageIndex >= swapchain.m_images.size())
        {
            SetStatus("OpenXR swapchain image acquire failed.");
            return false;
        }

        XrSwapchainImageWaitInfo waitInfo{};
        waitInfo.type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO;
        waitInfo.timeout = XR_INFINITE_DURATION;
        result = m_xrWaitSwapchainImagePtr(swapchain.m_swapchainPtr, &waitInfo);
        if (XR_FAILED(result))
        {
            XrSwapchainImageReleaseInfo colorReleaseInfo{};
            colorReleaseInfo.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO;
            m_xrReleaseSwapchainImagePtr(swapchain.m_swapchainPtr, &colorReleaseInfo);
            SetStatus("OpenXR swapchain image wait failed.");
            return false;
        }

        bool depthAcquired = false;
        uint32_t depthImageIndex = 0;
        const bool useCompositionDepthSwapchain = m_depthLayerExtensionEnabled && m_passthroughDepthTestReady && renderCallback;
        if (useCompositionDepthSwapchain)
        {
            result = m_xrAcquireSwapchainImagePtr(swapchain.m_depthSwapchainPtr, &acquireInfo, &depthImageIndex);
            if (XR_FAILED(result) || depthImageIndex >= swapchain.m_depthImages.size())
            {
                XrSwapchainImageReleaseInfo colorReleaseInfo{};
                colorReleaseInfo.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO;
                m_xrReleaseSwapchainImagePtr(swapchain.m_swapchainPtr, &colorReleaseInfo);
                SetStatus("OpenXR depth swapchain image acquire failed.");
                return false;
            }

            depthAcquired = true;
            result = m_xrWaitSwapchainImagePtr(swapchain.m_depthSwapchainPtr, &waitInfo);
            if (XR_FAILED(result))
            {
                XrSwapchainImageReleaseInfo depthReleaseInfo{};
                depthReleaseInfo.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO;
                m_xrReleaseSwapchainImagePtr(swapchain.m_depthSwapchainPtr, &depthReleaseInfo);

                XrSwapchainImageReleaseInfo colorReleaseInfo{};
                colorReleaseInfo.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO;
                m_xrReleaseSwapchainImagePtr(swapchain.m_swapchainPtr, &colorReleaseInfo);

                SetStatus("OpenXR depth swapchain image wait failed.");
                return false;
            }
        }

        bool rendered = false;
        if (m_renderResourcesReady && renderCallback)
        {
            rendered = RenderEyeScene(viewIndex, imageIndex, depthImageIndex, clearColor, renderCallback);
        }
        else
        {
            rendered = ClearImage(swapchain.m_images[imageIndex].image, viewIndex, clearColor);
        }

        if (depthAcquired)
        {
            XrSwapchainImageReleaseInfo depthReleaseInfo{};
            depthReleaseInfo.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO;
            result = m_xrReleaseSwapchainImagePtr(swapchain.m_depthSwapchainPtr, &depthReleaseInfo);
            if (XR_FAILED(result))
            {
                SetStatus("OpenXR depth swapchain image release failed.");
                rendered = false;
            }
        }

        XrSwapchainImageReleaseInfo releaseInfo{};
        releaseInfo.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO;
        result = m_xrReleaseSwapchainImagePtr(swapchain.m_swapchainPtr, &releaseInfo);
        if (XR_FAILED(result))
        {
            SetStatus("OpenXR swapchain image release failed.");
            return false;
        }

        return rendered;
    }

    //Renders one OpenXR eye with scene content.
    bool OpenXRUtility::RenderEyeScene(uint32_t viewIndex, uint32_t imageIndex, uint32_t depthImageIndex, const Float4 &clearColor, const EyeRenderCallback &renderCallback)
    {
        SwapchainData &swapchain = m_swapchains[viewIndex];
        const bool useDepthSwapchain = m_depthLayerExtensionEnabled && m_passthroughDepthTestReady && !swapchain.m_depthImageViewsPtrGPU.empty();
        const std::size_t depthViewCount = useDepthSwapchain ? swapchain.m_depthImageViewsPtrGPU.size() : 1U;
        const std::size_t framebufferIndex = (static_cast<std::size_t>(imageIndex) * depthViewCount) + (useDepthSwapchain ? static_cast<std::size_t>(depthImageIndex) : 0U);

        if (static_cast<std::size_t>(imageIndex) >= swapchain.m_imageViewsPtrGPU.size() ||
            (useDepthSwapchain && static_cast<std::size_t>(depthImageIndex) >= swapchain.m_depthImageViewsPtrGPU.size()) || framebufferIndex >= swapchain.m_framebuffersPtrGPU.size())
        {
            return false;
        }

        VkCommandBuffer commandBufferPtrGPU = VK_NULL_HANDLE;

        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.commandPool = m_commandPoolPtrGPU;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = 1;

        if (vkAllocateCommandBuffers(m_devicePtrGPU, &allocInfo, &commandBufferPtrGPU) != VK_SUCCESS)
        {
            return false;
        }

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

        if (vkBeginCommandBuffer(commandBufferPtrGPU, &beginInfo) != VK_SUCCESS)
        {
            vkFreeCommandBuffers(m_devicePtrGPU, m_commandPoolPtrGPU, 1, &commandBufferPtrGPU);
            return false;
        }

        std::array<VkClearValue, 2> clearValues{};
        clearValues[0].color.float32[0] = clearColor.x;
        clearValues[0].color.float32[1] = clearColor.y;
        clearValues[0].color.float32[2] = clearColor.z;
        clearValues[0].color.float32[3] = clearColor.w;
        clearValues[1].depthStencil.depth = 1.0f;
        clearValues[1].depthStencil.stencil = 0;

        VkRenderPassBeginInfo renderPassInfo{};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        renderPassInfo.renderPass = m_renderPassPtrGPU;
        renderPassInfo.framebuffer = swapchain.m_framebuffersPtrGPU[framebufferIndex];
        renderPassInfo.renderArea.offset = { 0, 0 };
        renderPassInfo.renderArea.extent = { swapchain.m_width, swapchain.m_height };
        renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
        renderPassInfo.pClearValues = clearValues.data();

        vkCmdBeginRenderPass(commandBufferPtrGPU, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

        EyeRenderInfo renderInfo{};
        renderInfo.m_viewIndex = viewIndex;
        renderInfo.m_commandBufferPtrGPU = commandBufferPtrGPU;
        renderInfo.m_extent = { swapchain.m_width, swapchain.m_height };
        renderInfo.m_view = m_views[viewIndex];
        if (m_environmentDepthAvailableThisFrame && viewIndex < 2U && m_environmentDepthImage.swapchainIndex < m_environmentDepthImages.size() &&
            m_environmentDepthImage.swapchainIndex < m_environmentDepthImageViewsPtrGPU.size())
        {
            renderInfo.m_environmentDepthAvailable = true;
            renderInfo.m_environmentDepthSwapchainIndex = m_environmentDepthImage.swapchainIndex;
            renderInfo.m_environmentDepthImagePtrGPU = m_environmentDepthImages[m_environmentDepthImage.swapchainIndex].image;
            renderInfo.m_environmentDepthImageViewPtrGPU = m_environmentDepthImageViewsPtrGPU[m_environmentDepthImage.swapchainIndex];
            renderInfo.m_environmentDepthSamplerPtrGPU = m_environmentDepthSamplerPtrGPU;
            renderInfo.m_environmentDepthImageLayout = m_environmentDepthImageLayout;
            renderInfo.m_environmentDepthExtent = {
                m_environmentDepthSwapchainState.width, m_environmentDepthSwapchainState.height
            };
            renderInfo.m_environmentDepthNearZ = FiniteDepthNear(m_environmentDepthImage.nearZ);
            renderInfo.m_environmentDepthFarZ = FiniteDepthFar(m_environmentDepthImage.farZ, renderInfo.m_environmentDepthNearZ, m_environmentDepthMaxMeters);
            renderInfo.m_environmentDepthView = m_environmentDepthImage.views[viewIndex];
        }
        renderCallback(renderInfo);
        DrawEnvironmentDepthDebug(commandBufferPtrGPU, renderInfo);

        vkCmdEndRenderPass(commandBufferPtrGPU);

        const bool recorded = vkEndCommandBuffer(commandBufferPtrGPU) == VK_SUCCESS;
        const bool submitted = recorded && SubmitCommandBuffer(commandBufferPtrGPU);
        vkFreeCommandBuffers(m_devicePtrGPU, m_commandPoolPtrGPU, 1, &commandBufferPtrGPU);
        return submitted;
    }

    //Clears one OpenXR Vulkan image.
    bool OpenXRUtility::ClearImage(VkImage imagePtrGPU, uint32_t viewIndex, const Float4 &clearColor) const
    {
        VkCommandBuffer commandBufferPtrGPU = VK_NULL_HANDLE;

        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.commandPool = m_commandPoolPtrGPU;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = 1;

        if (vkAllocateCommandBuffers(m_devicePtrGPU, &allocInfo, &commandBufferPtrGPU) != VK_SUCCESS)
        {
            return false;
        }

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

        if (vkBeginCommandBuffer(commandBufferPtrGPU, &beginInfo) != VK_SUCCESS)
        {
            vkFreeCommandBuffers(m_devicePtrGPU, m_commandPoolPtrGPU, 1, &commandBufferPtrGPU);
            return false;
        }

        VkImageMemoryBarrier toTransferBarrier{};
        toTransferBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        toTransferBarrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        toTransferBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        toTransferBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        toTransferBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        toTransferBarrier.image = imagePtrGPU;
        toTransferBarrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        toTransferBarrier.subresourceRange.baseMipLevel = 0;
        toTransferBarrier.subresourceRange.levelCount = 1;
        toTransferBarrier.subresourceRange.baseArrayLayer = 0;
        toTransferBarrier.subresourceRange.layerCount = 1;
        toTransferBarrier.srcAccessMask = 0;
        toTransferBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

        vkCmdPipelineBarrier(commandBufferPtrGPU, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &toTransferBarrier);

        VkClearColorValue clearValue{};
        clearValue.float32[0] = 0.02f;
        clearValue.float32[1] = viewIndex == 0 ? 0.08f : 0.32f;
        clearValue.float32[2] = viewIndex == 0 ? 0.35f : 0.08f;
        clearValue.float32[3] = clearColor.w;

        VkImageSubresourceRange range{};
        range.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        range.baseMipLevel = 0;
        range.levelCount = 1;
        range.baseArrayLayer = 0;
        range.layerCount = 1;
        vkCmdClearColorImage(commandBufferPtrGPU, imagePtrGPU, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &clearValue, 1, &range);

        VkImageMemoryBarrier toColorBarrier{};
        toColorBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        toColorBarrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        toColorBarrier.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        toColorBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        toColorBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        toColorBarrier.image = imagePtrGPU;
        toColorBarrier.subresourceRange = range;
        toColorBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        toColorBarrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

        vkCmdPipelineBarrier(commandBufferPtrGPU, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, 0, 0, nullptr, 0, nullptr, 1, &toColorBarrier);

        const bool recorded = vkEndCommandBuffer(commandBufferPtrGPU) == VK_SUCCESS;
        const bool submitted = recorded && SubmitCommandBuffer(commandBufferPtrGPU);
        vkFreeCommandBuffers(m_devicePtrGPU, m_commandPoolPtrGPU, 1, &commandBufferPtrGPU);
        return submitted;
    }

    //Submits one command buffer and waits for completion.
    bool OpenXRUtility::SubmitCommandBuffer(VkCommandBuffer commandBufferPtrGPU) const
    {
        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBufferPtrGPU;

        if (vkQueueSubmit(m_queuePtrGPU, 1, &submitInfo, VK_NULL_HANDLE) != VK_SUCCESS)
        {
            return false;
        }

        vkQueueWaitIdle(m_queuePtrGPU);
        return true;
    }

    //Chooses an OpenXR swapchain color format.
    int64_t OpenXRUtility::ChooseColorFormat(const std::vector<int64_t> &formats, VkFormat preferredColorFormat) const
    {
        const std::array<VkFormat, 5> preferredFormats = {
            preferredColorFormat, VK_FORMAT_R8G8B8A8_SRGB, VK_FORMAT_B8G8R8A8_SRGB, VK_FORMAT_R8G8B8A8_UNORM, VK_FORMAT_B8G8R8A8_UNORM
        };

        for (VkFormat preferredFormat : preferredFormats)
        {
            const int64_t preferredValue = static_cast<int64_t>(preferredFormat);
            if (std::find(formats.begin(), formats.end(), preferredValue) != formats.end())
            {
                return preferredValue;
            }
        }

        return formats.front();
    }

    //Returns a preferred OpenXR reference space type.
    XrReferenceSpaceType OpenXRUtility::PreferredReferenceSpaceType(void) const
    {
        const std::string name = StringUtility::Uppercase(m_referenceSpaceName);
        if (name == "STAGE" || name == "LOCALFLOOR" || name == "LOCAL_FLOOR")
        {
            return XR_REFERENCE_SPACE_TYPE_STAGE;
        }

        return XR_REFERENCE_SPACE_TYPE_LOCAL;
    }

    //Releases OpenXR eye render resources.
    void OpenXRUtility::CleanRenderResources(void)
    {
        CleanDepthDebugResources();

        if (m_devicePtrGPU == VK_NULL_HANDLE)
        {
            m_renderResourcesReady = false;
            m_passthroughDepthTestReady = false;
            return;
        }

        for (SwapchainData &swapchain : m_swapchains)
        {
            for (VkFramebuffer &framebufferPtrGPU : swapchain.m_framebuffersPtrGPU)
            {
                if (framebufferPtrGPU != VK_NULL_HANDLE)
                {
                    vkDestroyFramebuffer(m_devicePtrGPU, framebufferPtrGPU, nullptr);
                    framebufferPtrGPU = VK_NULL_HANDLE;
                }
            }
            swapchain.m_framebuffersPtrGPU.clear();

            CleanDepthSwapchain(swapchain);

            if (swapchain.m_depthImageViewPtrGPU != VK_NULL_HANDLE)
            {
                vkDestroyImageView(m_devicePtrGPU, swapchain.m_depthImageViewPtrGPU, nullptr);
                swapchain.m_depthImageViewPtrGPU = VK_NULL_HANDLE;
            }

            if (swapchain.m_depthImagePtrGPU != VK_NULL_HANDLE)
            {
                vkDestroyImage(m_devicePtrGPU, swapchain.m_depthImagePtrGPU, nullptr);
                swapchain.m_depthImagePtrGPU = VK_NULL_HANDLE;
            }

            if (swapchain.m_depthImageMemoryPtrGPU != VK_NULL_HANDLE)
            {
                vkFreeMemory(m_devicePtrGPU, swapchain.m_depthImageMemoryPtrGPU, nullptr);
                swapchain.m_depthImageMemoryPtrGPU = VK_NULL_HANDLE;
            }

            if (swapchain.m_colorImageViewPtrGPU != VK_NULL_HANDLE)
            {
                vkDestroyImageView(m_devicePtrGPU, swapchain.m_colorImageViewPtrGPU, nullptr);
                swapchain.m_colorImageViewPtrGPU = VK_NULL_HANDLE;
            }

            if (swapchain.m_colorImagePtrGPU != VK_NULL_HANDLE)
            {
                vkDestroyImage(m_devicePtrGPU, swapchain.m_colorImagePtrGPU, nullptr);
                swapchain.m_colorImagePtrGPU = VK_NULL_HANDLE;
            }

            if (swapchain.m_colorImageMemoryPtrGPU != VK_NULL_HANDLE)
            {
                vkFreeMemory(m_devicePtrGPU, swapchain.m_colorImageMemoryPtrGPU, nullptr);
                swapchain.m_colorImageMemoryPtrGPU = VK_NULL_HANDLE;
            }

            for (VkImageView &imageViewPtrGPU : swapchain.m_imageViewsPtrGPU)
            {
                if (imageViewPtrGPU != VK_NULL_HANDLE)
                {
                    vkDestroyImageView(m_devicePtrGPU, imageViewPtrGPU, nullptr);
                    imageViewPtrGPU = VK_NULL_HANDLE;
                }
            }
            swapchain.m_imageViewsPtrGPU.clear();
        }

        m_renderResourcesReady = false;
        m_passthroughDepthTestReady = false;
    }
}

