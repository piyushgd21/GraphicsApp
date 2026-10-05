///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	EngineCore.cpp
//	Description	:
//		Implements the engine core frame loop.
//
//
///////////////////////////////////////////////////////////////////////

#include "EngineCore.h"

#include "PathUtility.h"
#include "ShaderUtility.h"
#include "StringUtility.h"
#include "PlaneSweepGpu.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

namespace graphicsFX
{
    //Converts a normalized float to a byte.
    uint8_t EngineCore::FloatToByte(float value)
    {
        const float clampedValue = std::clamp(value, 0.0f, 1.0f);
        return static_cast<uint8_t>((clampedValue * 255.0f) + 0.5f);
    }

    //Returns whether the format uses BGRA byte order.
    bool EngineCore::FormatIsBgra(VkFormat format)
    {
        return format == VK_FORMAT_B8G8R8A8_UNORM || format == VK_FORMAT_B8G8R8A8_SRGB || format == VK_FORMAT_B8G8R8A8_SNORM;
    }

    //Returns whether the format uses RGBA byte order.
    bool EngineCore::FormatIsRgba(VkFormat format)
    {
        return format == VK_FORMAT_R8G8B8A8_UNORM || format == VK_FORMAT_R8G8B8A8_SRGB || format == VK_FORMAT_R8G8B8A8_SNORM;
    }

    //Returns whether the depth format stores depth as a 32-bit float.
    bool EngineCore::FormatIsFloatDepth(VkFormat format)
    {
        return format == VK_FORMAT_D32_SFLOAT || format == VK_FORMAT_D32_SFLOAT_S8_UINT;
    }

    //Returns whether the depth format stores depth as 24 normalized bits.
    bool EngineCore::FormatIsDepth24Stencil8(VkFormat format)
    {
        return format == VK_FORMAT_D24_UNORM_S8_UINT;
    }

    //Converts a normalized depth value to a grayscale byte.
    uint8_t EngineCore::DepthToByte(float value)
    {
        if (!std::isfinite(value))
        {
            return 0U;
        }

        return FloatToByte(value);
    }

    //Rotates a vector by an OpenXR quaternion.
    Float3 EngineCore::RotateVector(const XrQuaternionf &quaternion, const Float3 &value)
    {
        const Float3 quaternionVector{ quaternion.x, quaternion.y, quaternion.z };
        const Float3 uv = cross(quaternionVector, value);
        const Float3 uuv = cross(quaternionVector, uv);
        return value + (uv * (2.0f * quaternion.w)) + (uuv * 2.0f);
    }

    //Creates a view matrix from an OpenXR pose.
    Float4x4 EngineCore::ViewFromBasis(const CameraBasis &basis)
    {
        Float4x4 result = identity();
        result.m[0] = basis.m_right.x;
        result.m[4] = basis.m_right.y;
        result.m[8] = basis.m_right.z;
        result.m[1] = basis.m_up.x;
        result.m[5] = basis.m_up.y;
        result.m[9] = basis.m_up.z;
        result.m[2] = basis.m_back.x;
        result.m[6] = basis.m_back.y;
        result.m[10] = basis.m_back.z;
        result.m[12] = -dot(basis.m_right, basis.m_position);
        result.m[13] = -dot(basis.m_up, basis.m_position);
        result.m[14] = -dot(basis.m_back, basis.m_position);
        return result;
    }

    //Creates a camera basis from a camera buffer.
    EngineCore::CameraBasis EngineCore::BasisFromCameraBuffer(const CameraBuffer &camera)
    {
        CameraBasis basis{};
        basis.m_position = {
            camera.m_cameraPosition.x, camera.m_cameraPosition.y, camera.m_cameraPosition.z
        };
        basis.m_right = normalizeOrDefault( { camera.m_view.m[0], camera.m_view.m[4], camera.m_view.m[8] }, { 1.0f, 0.0f, 0.0f });
        basis.m_up = normalizeOrDefault( { camera.m_view.m[1], camera.m_view.m[5], camera.m_view.m[9] }, { 0.0f, 1.0f, 0.0f });
        basis.m_back = normalizeOrDefault( { camera.m_view.m[2], camera.m_view.m[6], camera.m_view.m[10] }, { 0.0f, 0.0f, 1.0f });
        return basis;
    }

    //Creates a movement basis from a camera buffer.
    CameraMovementBasis EngineCore::MovementBasisFromCameraBuffer(const CameraBuffer &camera)
    {
        const CameraBasis basis = BasisFromCameraBuffer(camera);
        CameraMovementBasis movementBasis{};
        movementBasis.m_forward = basis.m_back * -1.0f;
        movementBasis.m_right = basis.m_right;
        movementBasis.m_up = basis.m_up;
        return movementBasis;
    }

    //Transforms a local direction by a rig basis.
    Float3 EngineCore::TransformRigDirection(const CameraBasis &rigBasis, const Float3 &localDirection)
    {
        return (rigBasis.m_right * localDirection.x) + (rigBasis.m_up * localDirection.y) + (rigBasis.m_back * localDirection.z);
    }

    //Transforms a local position by a rig basis.
    Float3 EngineCore::TransformRigPosition(const CameraBasis &rigBasis, const Float3 &localPosition)
    {
        return rigBasis.m_position + TransformRigDirection(rigBasis, localPosition);
    }

    //Creates a world-space XR eye basis from the shared rig.
    EngineCore::CameraBasis EngineCore::BasisFromRigAndXrView(const CameraBasis &rigBasis, const XrView &view)
    {
        CameraBasis basis{};
        const Float3 localPosition{
            view.pose.position.x, view.pose.position.y, view.pose.position.z
        };
        const Float3 localRight = RotateVector(view.pose.orientation, { 1.0f, 0.0f, 0.0f });
        const Float3 localUp = RotateVector(view.pose.orientation, { 0.0f, 1.0f, 0.0f });
        const Float3 localBack = RotateVector(view.pose.orientation, { 0.0f, 0.0f, 1.0f });

        basis.m_position = TransformRigPosition(rigBasis, localPosition);
        basis.m_right = normalizeOrDefault(TransformRigDirection(rigBasis, localRight), rigBasis.m_right);
        basis.m_up = normalizeOrDefault(TransformRigDirection(rigBasis, localUp), rigBasis.m_up);
        basis.m_back = normalizeOrDefault(TransformRigDirection(rigBasis, localBack), rigBasis.m_back);
        return basis;
    }

    //Creates a world-space XR basis from a rig and pose.
    EngineCore::CameraBasis EngineCore::BasisFromRigAndXrPose(const CameraBasis &rigBasis, const XrPosef &pose)
    {
        CameraBasis basis{};
        const Float3 localPosition{
            pose.position.x, pose.position.y, pose.position.z
        };
        const Float3 localRight = RotateVector(pose.orientation, { 1.0f, 0.0f, 0.0f });
        const Float3 localUp = RotateVector(pose.orientation, { 0.0f, 1.0f, 0.0f });
        const Float3 localBack = RotateVector(pose.orientation, { 0.0f, 0.0f, 1.0f });

        basis.m_position = TransformRigPosition(rigBasis, localPosition);
        basis.m_right = normalizeOrDefault(TransformRigDirection(rigBasis, localRight), rigBasis.m_right);
        basis.m_up = normalizeOrDefault(TransformRigDirection(rigBasis, localUp), rigBasis.m_up);
        basis.m_back = normalizeOrDefault(TransformRigDirection(rigBasis, localBack), rigBasis.m_back);
        return basis;
    }

    //Returns the XR depth debug mode value.
    float EngineCore::XRDepthDebugModeValue(const std::string &debugMode)
    {
        const std::string mode = StringUtility::Uppercase(debugMode);
        if (mode == "DEPTH" || mode == "RAWDEPTH" || mode == "RAW_DEPTH" || mode == "FULLSCREEN")
        {
            return 1.0f;
        }

        if (mode == "MASK")
        {
            return 2.0f;
        }

        return 0.0f;
    }

    //Returns the XR depth value interpretation mode.
    float EngineCore::XRDepthValueModeValue(const std::string &valueMode)
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

    //Returns the XR alpha debug mode value.
    float EngineCore::XRAlphaDebugModeValue(const std::string &debugMode)
    {
        const std::string mode = StringUtility::Uppercase(debugMode);
        if (mode == "FORCETRANSPARENT" || mode == "TRANSPARENT")
        {
            return 1.0f;
        }

        if (mode == "FORCEOPAQUE" || mode == "OPAQUE")
        {
            return 2.0f;
        }

        return 0.0f;
    }

    //Returns whether XR depth debug fills the whole eye image.
    bool EngineCore::XRFullScreenDepthDebugRequested(const std::string &debugMode)
    {
        const std::string mode = StringUtility::Uppercase(debugMode);
        return mode == "DEPTH" || mode == "RAWDEPTH" || mode == "RAW_DEPTH" || mode == "RAWDEPTHTEXTURE" || mode == "RAW_DEPTH_TEXTURE" || mode == "TEXTURE" || mode == "FULLSCREEN" ||
            mode == "FULL_SCREEN";
    }

    //Returns a finite XR environment depth near plane.
    float EngineCore::XRDepthNearPlane(float nearPlane)
    {
        if (std::isfinite(nearPlane) && nearPlane > 0.0f)
        {
            return nearPlane;
        }

        return 0.05f;
    }

    //Returns a finite XR environment depth far plane.
    float EngineCore::XRDepthFarPlane(float farPlane, float nearPlane, float fallbackFarPlane)
    {
        if (std::isfinite(farPlane) && farPlane > nearPlane)
        {
            return farPlane;
        }

        return std::max(fallbackFarPlane, nearPlane + 1.0f);
    }

    //Creates a Vulkan projection matrix from an OpenXR field of view.
    Float4x4 EngineCore::ProjectionFromXrFov(const XrFovf &fov, float nearPlane, float farPlane)
    {
        const float tanLeft = std::tan(fov.angleLeft);
        const float tanRight = std::tan(fov.angleRight);
        const float tanDown = std::tan(fov.angleDown);
        const float tanUp = std::tan(fov.angleUp);
        const float width = tanRight - tanLeft;
        const float height = tanUp - tanDown;

        Float4x4 result{};
        result.m[0] = 2.0f / width;
        result.m[5] = -2.0f / height;
        result.m[8] = (tanRight + tanLeft) / width;
        result.m[9] = -(tanUp + tanDown) / height;
        result.m[10] = farPlane / (nearPlane - farPlane);
        result.m[11] = -1.0f;
        result.m[14] = (farPlane * nearPlane) / (nearPlane - farPlane);
        return result;
    }

    //Creates the engine core and initializes graphics.
    EngineCore::EngineCore(HINSTANCE instancePtr, SceneInterface &scene, const std::string &setupFileName) : m_programSetup(LoadProgramSetup(setupFileName)),
          m_window(instancePtr, m_programSetup.m_window.m_width, m_programSetup.m_window.m_height, StringUtility::ToWide(m_programSetup.m_window.m_title, L"GraphicsApp")), m_scene(scene),
          m_startTime(std::chrono::steady_clock::now()), m_previousFrameTime(m_startTime)
    {
        PathUtility::SetPaths(m_programSetup.m_paths);
        m_window.SetInputUtility(&m_inputUtility);
        ShaderUtility::CompileStartupShaders(m_programSetup.m_shaders);
        m_openXRUtility.Init(m_programSetup.m_xr);
        InitGraphics();
    }

    //Destroys the engine core.
    EngineCore::~EngineCore()
    {
        Cleanup();
    }

    //Runs the application loop.
    void EngineCore::Run(void)
    {
        MainLoop();
    }

    //Initializes graphics resources.
    void EngineCore::InitGraphics(void)
    {
        m_graphicsUtility.SetExtraInstanceExtensions(m_openXRUtility.VulkanInstanceExtensions());
        m_graphicsUtility.SetExtraDeviceExtensions(m_openXRUtility.VulkanDeviceExtensions());
        m_graphicsUtility.SetMinimumApiVersion(m_openXRUtility.VulkanMinimumApiVersion());
        m_graphicsUtility.InitInstanceAndSurface(m_window);
        const VkPhysicalDevice xrPhysicalDevicePtrGPU = m_openXRUtility.QueryRequiredPhysicalDevice(m_graphicsUtility.Instance());
        m_graphicsUtility.InitDeviceAndCommandPool(xrPhysicalDevicePtrGPU);

        // Phase2A compute smoke-test: one-time invocation to validate existing graphics queue can run compute.
        {
            const GraphicsUtility::QueueFamilyIndices queueFamilyIndices = m_graphicsUtility.FindQueueFamilies(m_graphicsUtility.PhysicalDevice());
            if (queueFamilyIndices.m_graphicsFamily.has_value())
            {
                const uint32_t graphicsFamily = queueFamilyIndices.m_graphicsFamily.value();
                // Query queue family properties to determine compute capability.
                uint32_t queueFamilyCount = 0;
                vkGetPhysicalDeviceQueueFamilyProperties(m_graphicsUtility.PhysicalDevice(), &queueFamilyCount, nullptr);
                if (queueFamilyCount > 0)
                {
                    std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
                    vkGetPhysicalDeviceQueueFamilyProperties(m_graphicsUtility.PhysicalDevice(), &queueFamilyCount, queueFamilies.data());
                    if (graphicsFamily < queueFamilyCount)
                    {
                        const VkQueueFlags flags = queueFamilies[graphicsFamily].queueFlags;
                        if ((flags & VK_QUEUE_COMPUTE_BIT) == 0)
                        {
                            UIUtility::Print("Selected graphics queue family does not support compute;\nPlaneSweep compute smoke test skipped.\n");
                        }
                        else
                        {
                            UIUtility::Print("Selected graphics queue family supports compute; running PlaneSweep compute smoke test.\n");
                            PlaneSweepGpu smokeTest;
                            smokeTest.Initialize(
                                m_graphicsUtility.Device(),
                                m_graphicsUtility.PhysicalDevice(),
                                queueFamilyIndices.m_graphicsFamily.value(),
                                m_graphicsUtility.GraphicsQueue(),
                                m_graphicsUtility.CommandPool()
                            );

                            try
                            {
                                // Run only the newest bilinear-sampling smoke test at startup.
                                const bool bilinearSamplingOk = smokeTest.RunBilinearSamplingSmokeTest();
                                if (!bilinearSamplingOk)
                                {
                                    UIUtility::Print("PlaneSweep GPU bilinear-sampling smoke test failed.\n");
                                }
                            }
                            catch (const std::exception &ex)
                            {
                                UIUtility::Print(std::string("PlaneSweep GPU smoke test failed: ") + ex.what() + "\n");
                            }

                            smokeTest.Clean();
                        }
                    }
                }
            }
        }

        m_sceneSampleCount = m_graphicsUtility.ChooseSampleCount(m_programSetup.m_graphics.m_antiAliasingEnabled, m_programSetup.m_graphics.m_antiAliasingSamples);
        CreateSwapChain();
        const GraphicsUtility::QueueFamilyIndices queueFamilyIndices = m_graphicsUtility.FindQueueFamilies(m_graphicsUtility.PhysicalDevice());
        if (queueFamilyIndices.m_graphicsFamily.has_value())
        {
            m_openXRUtility.InitSession(
                m_graphicsUtility.Instance(),
                m_graphicsUtility.PhysicalDevice(),
                m_graphicsUtility.Device(),
                queueFamilyIndices.m_graphicsFamily.value(),
                0,
                m_graphicsUtility.GraphicsQueue(),
                m_graphicsUtility.CommandPool(),
                m_swapChainImageFormat
            );
        }
        else
        {
            UIUtility::Print("Warning: graphics queue family not found; skipping PlaneSweep compute smoke test.\n");
        }
        CreateImageViews();
        CreateRenderPass();
        CreateUIRenderPass();
        m_uiUtility.Init(m_graphicsUtility.Device(), m_graphicsUtility.PhysicalDevice(), m_uiRenderPassPtrGPU, m_swapChainExtent, s_maxFramesInFlight, m_programSetup.m_ui);
        CreateDescriptorSetLayout();
        CreateColorResources();
        CreateDepthResources();
        CreateShadowResources();
        m_openXRUtility.InitRenderResources(m_graphicsUtility.PhysicalDevice(), m_renderPassPtrGPU, m_swapChainImageFormat, m_graphicsUtility.FindDepthFormat(), m_sceneSampleCount);
        CreateFramebuffers();
        CreateUIFramebuffers();
        CreateUniformBuffers();
        CreateDescriptorPool();
        CreateDescriptorSets();
        CreateScene();
        CreateCommandBuffers();
        CreateSyncObjects();
    }

    //Runs the message and render loop.
    void EngineCore::MainLoop(void)
    {
        const auto targetFrameTime = std::chrono::milliseconds(16);

        while (!m_window.ShouldClose())
        {
            const auto frameStartTime = std::chrono::steady_clock::now();

            m_inputUtility.BeginFrame();
            m_window.PollEvents();
            m_openXRUtility.PollEvents();

            if (!m_window.ShouldClose())
            {
                DrawFrame();
                if (m_programSetup.m_graphics.m_vsyncEnabled)
                {
                    std::this_thread::sleep_until(frameStartTime + targetFrameTime);
                }
            }
        }

        m_graphicsUtility.WaitIdle();
    }

    //Releases application resources.
    void EngineCore::Cleanup(void)
    {
        if (m_graphicsUtility.DeviceReady())
        {
            const VkDevice devicePtrGPU = m_graphicsUtility.Device();

            m_graphicsUtility.WaitIdle();

            CleanupSwapChain();
            m_uiUtility.Clean();
            m_scene.Clean();
            CleanShadowResources();

            if (m_descriptorPoolPtrGPU != VK_NULL_HANDLE)
            {
                vkDestroyDescriptorPool(devicePtrGPU, m_descriptorPoolPtrGPU, nullptr);
                m_descriptorPoolPtrGPU = VK_NULL_HANDLE;
            }

            for (std::size_t i = 0; i < m_uniformBuffersPtrGPU.size(); ++i)
            {
                if (m_uniformBuffersMemoryPtrGPU[i] != VK_NULL_HANDLE)
                {
                    vkUnmapMemory(devicePtrGPU, m_uniformBuffersMemoryPtrGPU[i]);
                }

                if (m_uniformBuffersPtrGPU[i] != VK_NULL_HANDLE)
                {
                    vkDestroyBuffer(devicePtrGPU, m_uniformBuffersPtrGPU[i], nullptr);
                    m_uniformBuffersPtrGPU[i] = VK_NULL_HANDLE;
                }

                if (m_uniformBuffersMemoryPtrGPU[i] != VK_NULL_HANDLE)
                {
                    vkFreeMemory(devicePtrGPU, m_uniformBuffersMemoryPtrGPU[i], nullptr);
                    m_uniformBuffersMemoryPtrGPU[i] = VK_NULL_HANDLE;
                }
            }

            for (std::size_t i = 0; i < m_imageAvailableSemaphoresPtrGPU.size(); ++i)
            {
                if (m_renderFinishedSemaphoresPtrGPU[i] != VK_NULL_HANDLE)
                {
                    vkDestroySemaphore(devicePtrGPU, m_renderFinishedSemaphoresPtrGPU[i], nullptr);
                    m_renderFinishedSemaphoresPtrGPU[i] = VK_NULL_HANDLE;
                }

                if (m_imageAvailableSemaphoresPtrGPU[i] != VK_NULL_HANDLE)
                {
                    vkDestroySemaphore(devicePtrGPU, m_imageAvailableSemaphoresPtrGPU[i], nullptr);
                    m_imageAvailableSemaphoresPtrGPU[i] = VK_NULL_HANDLE;
                }

                if (m_inFlightFencesPtrGPU[i] != VK_NULL_HANDLE)
                {
                    vkDestroyFence(devicePtrGPU, m_inFlightFencesPtrGPU[i], nullptr);
                    m_inFlightFencesPtrGPU[i] = VK_NULL_HANDLE;
                }
            }

            if (m_descriptorSetLayoutPtrGPU != VK_NULL_HANDLE)
            {
                vkDestroyDescriptorSetLayout(devicePtrGPU, m_descriptorSetLayoutPtrGPU, nullptr);
                m_descriptorSetLayoutPtrGPU = VK_NULL_HANDLE;
            }

            if (m_textureDescriptorSetLayoutPtrGPU != VK_NULL_HANDLE)
            {
                vkDestroyDescriptorSetLayout(devicePtrGPU, m_textureDescriptorSetLayoutPtrGPU, nullptr);
                m_textureDescriptorSetLayoutPtrGPU = VK_NULL_HANDLE;
            }
        }

        m_openXRUtility.Clean();
        m_graphicsUtility.Clean();
    }

    //Creates the swapchain.
    void EngineCore::CreateSwapChain(void)
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();
        const VkPhysicalDevice physicalDevicePtrGPU = m_graphicsUtility.PhysicalDevice();
        const GraphicsUtility::SwapChainSupportDetails support = m_graphicsUtility.QuerySwapChainSupport(physicalDevicePtrGPU);

        const VkSurfaceFormatKHR surfaceFormat = ChooseSwapSurfaceFormat(support.m_formats);
        const VkPresentModeKHR presentMode = ChooseSwapPresentMode(support.m_presentModes);
        const VkExtent2D extent = ChooseSwapExtent(support.m_capabilities);

        uint32_t imageCount = support.m_capabilities.minImageCount + 1;
        if (support.m_capabilities.maxImageCount > 0 && imageCount > support.m_capabilities.maxImageCount)
        {
            imageCount = support.m_capabilities.maxImageCount;
        }

        VkSwapchainCreateInfoKHR createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
        createInfo.surface = m_graphicsUtility.Surface();
        createInfo.minImageCount = imageCount;
        createInfo.imageFormat = surfaceFormat.format;
        createInfo.imageColorSpace = surfaceFormat.colorSpace;
        createInfo.imageExtent = extent;
        createInfo.imageArrayLayers = 1;
        createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

        const GraphicsUtility::QueueFamilyIndices indices = m_graphicsUtility.FindQueueFamilies(physicalDevicePtrGPU);
        const uint32_t queueFamilyIndices[] = {
            indices.m_graphicsFamily.value(), indices.m_presentFamily.value()
        };

        if (indices.m_graphicsFamily != indices.m_presentFamily)
        {
            createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
            createInfo.queueFamilyIndexCount = 2;
            createInfo.pQueueFamilyIndices = queueFamilyIndices;
        }
        else
        {
            createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        }

        createInfo.preTransform = support.m_capabilities.currentTransform;
        createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
        createInfo.presentMode = presentMode;
        createInfo.clipped = VK_TRUE;
        createInfo.oldSwapchain = VK_NULL_HANDLE;

        if (vkCreateSwapchainKHR(devicePtrGPU, &createInfo, nullptr, &m_swapChainPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create Vulkan swapchain.");
        }

        vkGetSwapchainImagesKHR(devicePtrGPU, m_swapChainPtrGPU, &imageCount, nullptr);
        m_swapChainImagesPtrGPU.resize(imageCount);
        vkGetSwapchainImagesKHR(devicePtrGPU, m_swapChainPtrGPU, &imageCount, m_swapChainImagesPtrGPU.data());

        m_swapChainImageFormat = surfaceFormat.format;
        m_swapChainExtent = extent;
    }

    //Creates swapchain image views.
    void EngineCore::CreateImageViews(void)
    {
        m_swapChainImageViewsPtrGPU.resize(m_swapChainImagesPtrGPU.size());

        for (std::size_t i = 0; i < m_swapChainImagesPtrGPU.size(); ++i)
        {
            m_swapChainImageViewsPtrGPU[i] = m_graphicsUtility.CreateImageView(m_swapChainImagesPtrGPU[i], m_swapChainImageFormat, VK_IMAGE_ASPECT_COLOR_BIT);
        }
    }

    //Creates the render pass.
    void EngineCore::CreateRenderPass(void)
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();
        const bool useMultisampling = m_sceneSampleCount != VK_SAMPLE_COUNT_1_BIT;

        VkAttachmentDescription colorAttachment{};
        colorAttachment.format = m_swapChainImageFormat;
        colorAttachment.samples = m_sceneSampleCount;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        colorAttachment.storeOp = useMultisampling ? VK_ATTACHMENT_STORE_OP_DONT_CARE : VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        colorAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkAttachmentDescription depthAttachment{};
        depthAttachment.format = m_graphicsUtility.FindDepthFormat();
        depthAttachment.samples = m_sceneSampleCount;
        depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkAttachmentReference colorAttachmentRef{};
        colorAttachmentRef.attachment = 0;
        colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkAttachmentReference depthAttachmentRef{};
        depthAttachmentRef.attachment = 1;
        depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkAttachmentDescription resolveAttachment{};
        resolveAttachment.format = m_swapChainImageFormat;
        resolveAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        resolveAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        resolveAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        resolveAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        resolveAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        resolveAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        resolveAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkAttachmentReference resolveAttachmentRef{};
        resolveAttachmentRef.attachment = 2;
        resolveAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorAttachmentRef;
        subpass.pResolveAttachments = useMultisampling ? &resolveAttachmentRef : nullptr;
        subpass.pDepthStencilAttachment = &depthAttachmentRef;

        std::vector<VkAttachmentDescription> attachments = {
            colorAttachment, depthAttachment
        };

        if (useMultisampling)
        {
            attachments.push_back(resolveAttachment);
        }

        const std::array<VkSubpassDependency, 2> dependencies = {
            VkSubpassDependency{
                VK_SUBPASS_EXTERNAL, 0, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT,
                VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT, 0, VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT, 0
            }, VkSubpassDependency{
                0, VK_SUBPASS_EXTERNAL, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT, VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, 0
            }
        };

        VkRenderPassCreateInfo renderPassInfo{};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        renderPassInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
        renderPassInfo.pAttachments = attachments.data();
        renderPassInfo.subpassCount = 1;
        renderPassInfo.pSubpasses = &subpass;
        renderPassInfo.dependencyCount = static_cast<uint32_t>(dependencies.size());
        renderPassInfo.pDependencies = dependencies.data();

        if (vkCreateRenderPass(devicePtrGPU, &renderPassInfo, nullptr, &m_renderPassPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create Vulkan render pass.");
        }
    }

    //Creates the UI overlay render pass.
    void EngineCore::CreateUIRenderPass(void)
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();

        VkAttachmentDescription colorAttachment{};
        colorAttachment.format = m_swapChainImageFormat;
        colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
        colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        colorAttachment.initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

        VkAttachmentReference colorAttachmentRef{};
        colorAttachmentRef.attachment = 0;
        colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorAttachmentRef;

        const std::array<VkSubpassDependency, 2> dependencies = {
            VkSubpassDependency{
                VK_SUBPASS_EXTERNAL, 0, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
                VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, 0
            }, VkSubpassDependency{
                0, VK_SUBPASS_EXTERNAL, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, 0, 0
            }
        };

        VkRenderPassCreateInfo renderPassInfo{};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        renderPassInfo.attachmentCount = 1;
        renderPassInfo.pAttachments = &colorAttachment;
        renderPassInfo.subpassCount = 1;
        renderPassInfo.pSubpasses = &subpass;
        renderPassInfo.dependencyCount = static_cast<uint32_t>(dependencies.size());
        renderPassInfo.pDependencies = dependencies.data();

        if (vkCreateRenderPass(devicePtrGPU, &renderPassInfo, nullptr, &m_uiRenderPassPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create Vulkan UI render pass.");
        }
    }

    //Creates descriptor set layouts.
    void EngineCore::CreateDescriptorSetLayout(void)
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();

        VkDescriptorSetLayoutBinding cameraLayoutBinding{};
        cameraLayoutBinding.binding = 0;
        cameraLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        cameraLayoutBinding.descriptorCount = 1;
        cameraLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutBinding shadowLayoutBinding{};
        shadowLayoutBinding.binding = 1;
        shadowLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        shadowLayoutBinding.descriptorCount = 1;
        shadowLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutBinding xrEnvironmentDepthLayoutBinding{};
        xrEnvironmentDepthLayoutBinding.binding = 2;
        xrEnvironmentDepthLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        xrEnvironmentDepthLayoutBinding.descriptorCount = 1;
        xrEnvironmentDepthLayoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

        const std::array<VkDescriptorSetLayoutBinding, 3> cameraLayoutBindings = {
            cameraLayoutBinding, shadowLayoutBinding, xrEnvironmentDepthLayoutBinding
        };

        VkDescriptorSetLayoutCreateInfo layoutInfo{};
        layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        layoutInfo.bindingCount = static_cast<uint32_t>(cameraLayoutBindings.size());
        layoutInfo.pBindings = cameraLayoutBindings.data();

        if (vkCreateDescriptorSetLayout(devicePtrGPU, &layoutInfo, nullptr, &m_descriptorSetLayoutPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create Vulkan descriptor set layout.");
        }

        const std::array<VkDescriptorSetLayoutBinding, 2> textureLayoutBindings = {
            VkDescriptorSetLayoutBinding{
                0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr
            }, VkDescriptorSetLayoutBinding{
                1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr
            }
        };

        VkDescriptorSetLayoutCreateInfo textureLayoutInfo{};
        textureLayoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        textureLayoutInfo.bindingCount = static_cast<uint32_t>(textureLayoutBindings.size());
        textureLayoutInfo.pBindings = textureLayoutBindings.data();

        if (vkCreateDescriptorSetLayout(devicePtrGPU, &textureLayoutInfo, nullptr, &m_textureDescriptorSetLayoutPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create Vulkan texture descriptor set layout.");
        }
    }

    //Creates the multisampled color image resources.
    void EngineCore::CreateColorResources(void)
    {
        if (m_sceneSampleCount == VK_SAMPLE_COUNT_1_BIT)
        {
            return;
        }

        m_graphicsUtility.CreateImage(
            m_swapChainExtent.width,
            m_swapChainExtent.height,
            m_swapChainImageFormat,
            VK_IMAGE_TILING_OPTIMAL,
            VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
            m_sceneSampleCount,
            m_colorImagePtrGPU,
            m_colorImageMemoryPtrGPU
        );

        m_colorImageViewPtrGPU = m_graphicsUtility.CreateImageView(m_colorImagePtrGPU, m_swapChainImageFormat, VK_IMAGE_ASPECT_COLOR_BIT);
    }

    //Creates the depth image resources.
    void EngineCore::CreateDepthResources(void)
    {
        const VkFormat depthFormat = m_graphicsUtility.FindDepthFormat();

        m_graphicsUtility.CreateImage(
            m_swapChainExtent.width,
            m_swapChainExtent.height,
            depthFormat,
            VK_IMAGE_TILING_OPTIMAL,
            VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
            m_sceneSampleCount,
            m_depthImagePtrGPU,
            m_depthImageMemoryPtrGPU
        );

        m_depthImageViewPtrGPU = m_graphicsUtility.CreateImageView(m_depthImagePtrGPU, depthFormat, VK_IMAGE_ASPECT_DEPTH_BIT);
    }

    //Creates shadow map resources.
    void EngineCore::CreateShadowResources(void)
    {
        m_shadowExtent = { s_shadowMapSize, s_shadowMapSize };
        m_shadowDepthFormat = m_graphicsUtility.FindSupportedFormat(
            { VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT },
            VK_IMAGE_TILING_OPTIMAL,
            VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT
        );

        CreateShadowRenderPass();
        CreateShadowSampler();

        m_shadowMapResources.resize(s_maxFramesInFlight);
        for (ShadowMapResources &resources : m_shadowMapResources)
        {
            CreateShadowMapResources(resources);
        }
    }

    //Creates the shadow map render pass.
    void EngineCore::CreateShadowRenderPass(void)
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();

        VkAttachmentDescription depthAttachment{};
        depthAttachment.format = m_shadowDepthFormat;
        depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        depthAttachment.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkAttachmentReference depthAttachmentRef{};
        depthAttachmentRef.attachment = 0;
        depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 0;
        subpass.pDepthStencilAttachment = &depthAttachmentRef;

        const std::array<VkSubpassDependency, 2> dependencies = {
            VkSubpassDependency{
                VK_SUBPASS_EXTERNAL, 0, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT, VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT, 0
            }, VkSubpassDependency{
                0, VK_SUBPASS_EXTERNAL, VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT, 0
            }
        };

        VkRenderPassCreateInfo renderPassInfo{};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        renderPassInfo.attachmentCount = 1;
        renderPassInfo.pAttachments = &depthAttachment;
        renderPassInfo.subpassCount = 1;
        renderPassInfo.pSubpasses = &subpass;
        renderPassInfo.dependencyCount = static_cast<uint32_t>(dependencies.size());
        renderPassInfo.pDependencies = dependencies.data();

        if (vkCreateRenderPass(devicePtrGPU, &renderPassInfo, nullptr, &m_shadowRenderPassPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create Vulkan shadow render pass.");
        }
    }

    //Creates one shadow map image set.
    void EngineCore::CreateShadowMapResources(ShadowMapResources &resources)
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();

        m_graphicsUtility.CreateImage(
            m_shadowExtent.width,
            m_shadowExtent.height,
            m_shadowDepthFormat,
            VK_IMAGE_TILING_OPTIMAL,
            VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
            VK_SAMPLE_COUNT_1_BIT,
            resources.m_depthImagePtrGPU,
            resources.m_depthImageMemoryPtrGPU
        );

        resources.m_depthImageViewPtrGPU = m_graphicsUtility.CreateImageView(resources.m_depthImagePtrGPU, m_shadowDepthFormat, VK_IMAGE_ASPECT_DEPTH_BIT);

        VkFramebufferCreateInfo framebufferInfo{};
        framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        framebufferInfo.renderPass = m_shadowRenderPassPtrGPU;
        framebufferInfo.attachmentCount = 1;
        framebufferInfo.pAttachments = &resources.m_depthImageViewPtrGPU;
        framebufferInfo.width = m_shadowExtent.width;
        framebufferInfo.height = m_shadowExtent.height;
        framebufferInfo.layers = 1;

        if (vkCreateFramebuffer(devicePtrGPU, &framebufferInfo, nullptr, &resources.m_framebufferPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create Vulkan shadow framebuffer.");
        }
    }

    //Creates the shadow map sampler.
    void EngineCore::CreateShadowSampler(void)
    {
        VkSamplerCreateInfo samplerInfo{};
        samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        samplerInfo.magFilter = VK_FILTER_NEAREST;
        samplerInfo.minFilter = VK_FILTER_NEAREST;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        samplerInfo.anisotropyEnable = VK_FALSE;
        samplerInfo.maxAnisotropy = 1.0f;
        samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        samplerInfo.unnormalizedCoordinates = VK_FALSE;
        samplerInfo.compareEnable = VK_TRUE;
        samplerInfo.compareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        samplerInfo.mipLodBias = 0.0f;
        samplerInfo.minLod = 0.0f;
        samplerInfo.maxLod = 0.0f;

        if (vkCreateSampler(m_graphicsUtility.Device(), &samplerInfo, nullptr, &m_shadowSamplerPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create Vulkan shadow sampler.");
        }
    }

    //Creates swapchain framebuffers.
    void EngineCore::CreateFramebuffers(void)
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();
        const bool useMultisampling = m_sceneSampleCount != VK_SAMPLE_COUNT_1_BIT;
        m_swapChainFramebuffersPtrGPU.resize(m_swapChainImageViewsPtrGPU.size());

        for (std::size_t i = 0; i < m_swapChainImageViewsPtrGPU.size(); ++i)
        {
            std::vector<VkImageView> attachments;
            if (useMultisampling)
            {
                attachments = {
                    m_colorImageViewPtrGPU, m_depthImageViewPtrGPU, m_swapChainImageViewsPtrGPU[i]
                };
            }
            else
            {
                attachments = {
                    m_swapChainImageViewsPtrGPU[i], m_depthImageViewPtrGPU
                };
            }

            VkFramebufferCreateInfo framebufferInfo{};
            framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
            framebufferInfo.renderPass = m_renderPassPtrGPU;
            framebufferInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
            framebufferInfo.pAttachments = attachments.data();
            framebufferInfo.width = m_swapChainExtent.width;
            framebufferInfo.height = m_swapChainExtent.height;
            framebufferInfo.layers = 1;

            if (vkCreateFramebuffer(devicePtrGPU, &framebufferInfo, nullptr, &m_swapChainFramebuffersPtrGPU[i]) != VK_SUCCESS)
            {
                throw std::runtime_error("Failed to create Vulkan framebuffer.");
            }
        }
    }

    //Creates UI overlay framebuffers.
    void EngineCore::CreateUIFramebuffers(void)
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();
        m_uiFramebuffersPtrGPU.resize(m_swapChainImageViewsPtrGPU.size());

        for (std::size_t i = 0; i < m_swapChainImageViewsPtrGPU.size(); ++i)
        {
            const VkImageView attachments[] = {
                m_swapChainImageViewsPtrGPU[i]
            };

            VkFramebufferCreateInfo framebufferInfo{};
            framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
            framebufferInfo.renderPass = m_uiRenderPassPtrGPU;
            framebufferInfo.attachmentCount = 1;
            framebufferInfo.pAttachments = attachments;
            framebufferInfo.width = m_swapChainExtent.width;
            framebufferInfo.height = m_swapChainExtent.height;
            framebufferInfo.layers = 1;

            if (vkCreateFramebuffer(devicePtrGPU, &framebufferInfo, nullptr, &m_uiFramebuffersPtrGPU[i]) != VK_SUCCESS)
            {
                throw std::runtime_error("Failed to create Vulkan UI framebuffer.");
            }
        }
    }

    //Creates camera uniform buffers.
    void EngineCore::CreateUniformBuffers(void)
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();
        const VkDeviceSize bufferSize = sizeof(CameraBuffer);

        m_uniformBuffersPtrGPU.resize(s_maxFramesInFlight);
        m_uniformBuffersMemoryPtrGPU.resize(s_maxFramesInFlight);
        m_uniformBuffersMappedPtr.resize(s_maxFramesInFlight);

        for (std::size_t i = 0; i < s_maxFramesInFlight; ++i)
        {
            m_graphicsUtility.CreateBuffer(
                bufferSize,
                VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                m_uniformBuffersPtrGPU[i],
                m_uniformBuffersMemoryPtrGPU[i]
            );

            vkMapMemory(devicePtrGPU, m_uniformBuffersMemoryPtrGPU[i], 0, bufferSize, 0, &m_uniformBuffersMappedPtr[i]);
        }
    }

    //Creates the descriptor pool.
    void EngineCore::CreateDescriptorPool(void)
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();

        const std::array<VkDescriptorPoolSize, 2> poolSizes = {
            VkDescriptorPoolSize{
                VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, static_cast<uint32_t>(s_maxFramesInFlight)
            }, VkDescriptorPoolSize{
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, static_cast<uint32_t>((s_maxSceneObjects * 2) + (s_maxFramesInFlight * 2))
            }
        };

        VkDescriptorPoolCreateInfo poolInfo{};
        poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
        poolInfo.pPoolSizes = poolSizes.data();
        poolInfo.maxSets = static_cast<uint32_t>(s_maxFramesInFlight + s_maxSceneObjects);

        if (vkCreateDescriptorPool(devicePtrGPU, &poolInfo, nullptr, &m_descriptorPoolPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create Vulkan descriptor pool.");
        }
    }

    //Creates descriptor sets.
    void EngineCore::CreateDescriptorSets(void)
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();
        const std::vector<VkDescriptorSetLayout> layouts(s_maxFramesInFlight, m_descriptorSetLayoutPtrGPU);

        VkDescriptorSetAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        allocInfo.descriptorPool = m_descriptorPoolPtrGPU;
        allocInfo.descriptorSetCount = static_cast<uint32_t>(s_maxFramesInFlight);
        allocInfo.pSetLayouts = layouts.data();

        m_descriptorSetsPtrGPU.resize(s_maxFramesInFlight);
        if (vkAllocateDescriptorSets(devicePtrGPU, &allocInfo, m_descriptorSetsPtrGPU.data()) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to allocate Vulkan descriptor sets.");
        }

        for (std::size_t i = 0; i < s_maxFramesInFlight; ++i)
        {
            VkDescriptorBufferInfo bufferInfo{};
            bufferInfo.buffer = m_uniformBuffersPtrGPU[i];
            bufferInfo.offset = 0;
            bufferInfo.range = sizeof(CameraBuffer);

            VkDescriptorImageInfo shadowImageInfo{};
            shadowImageInfo.sampler = m_shadowSamplerPtrGPU;
            shadowImageInfo.imageView = m_shadowMapResources[i].m_depthImageViewPtrGPU;
            shadowImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

            std::array<VkWriteDescriptorSet, 3> descriptorWrites{};
            descriptorWrites[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            descriptorWrites[0].dstSet = m_descriptorSetsPtrGPU[i];
            descriptorWrites[0].dstBinding = 0;
            descriptorWrites[0].dstArrayElement = 0;
            descriptorWrites[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            descriptorWrites[0].descriptorCount = 1;
            descriptorWrites[0].pBufferInfo = &bufferInfo;

            descriptorWrites[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            descriptorWrites[1].dstSet = m_descriptorSetsPtrGPU[i];
            descriptorWrites[1].dstBinding = 1;
            descriptorWrites[1].dstArrayElement = 0;
            descriptorWrites[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            descriptorWrites[1].descriptorCount = 1;
            descriptorWrites[1].pImageInfo = &shadowImageInfo;

            descriptorWrites[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            descriptorWrites[2].dstSet = m_descriptorSetsPtrGPU[i];
            descriptorWrites[2].dstBinding = 2;
            descriptorWrites[2].dstArrayElement = 0;
            descriptorWrites[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            descriptorWrites[2].descriptorCount = 1;
            descriptorWrites[2].pImageInfo = &shadowImageInfo;

            vkUpdateDescriptorSets(devicePtrGPU, static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);
        }
    }

    //Creates command buffers.
    void EngineCore::CreateCommandBuffers(void)
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();
        m_commandBuffersPtrGPU.resize(s_maxFramesInFlight);

        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.commandPool = m_graphicsUtility.CommandPool();
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = static_cast<uint32_t>(m_commandBuffersPtrGPU.size());

        if (vkAllocateCommandBuffers(devicePtrGPU, &allocInfo, m_commandBuffersPtrGPU.data()) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to allocate Vulkan command buffers.");
        }
    }

    //Creates frame synchronization objects.
    void EngineCore::CreateSyncObjects(void)
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();

        m_imageAvailableSemaphoresPtrGPU.resize(s_maxFramesInFlight);
        m_renderFinishedSemaphoresPtrGPU.resize(s_maxFramesInFlight);
        m_inFlightFencesPtrGPU.resize(s_maxFramesInFlight);

        VkSemaphoreCreateInfo semaphoreInfo{};
        semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

        VkFenceCreateInfo fenceInfo{};
        fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

        for (std::size_t i = 0; i < s_maxFramesInFlight; ++i)
        {
            if (vkCreateSemaphore(devicePtrGPU, &semaphoreInfo, nullptr, &m_imageAvailableSemaphoresPtrGPU[i]) != VK_SUCCESS ||
                vkCreateSemaphore(devicePtrGPU, &semaphoreInfo, nullptr, &m_renderFinishedSemaphoresPtrGPU[i]) != VK_SUCCESS ||
                vkCreateFence(devicePtrGPU, &fenceInfo, nullptr, &m_inFlightFencesPtrGPU[i]) != VK_SUCCESS)
            {
                throw std::runtime_error("Failed to create Vulkan synchronization objects.");
            }
        }
    }

    //Creates scene objects.
    void EngineCore::CreateScene(void)
    {
        m_scene.Init(SceneObjectContextInfo(), m_programSetup, m_uiUtility);
    }

    //Releases swapchain resources.
    void EngineCore::CleanupSwapChain(void)
    {
        if (!m_graphicsUtility.DeviceReady())
        {
            return;
        }

        const VkDevice devicePtrGPU = m_graphicsUtility.Device();

        m_openXRUtility.CleanRenderResources();
        m_scene.CleanPipelines();
        m_uiUtility.CleanPipeline();

        for (VkFramebuffer framebufferPtrGPU : m_swapChainFramebuffersPtrGPU)
        {
            vkDestroyFramebuffer(devicePtrGPU, framebufferPtrGPU, nullptr);
        }
        m_swapChainFramebuffersPtrGPU.clear();

        for (VkFramebuffer framebufferPtrGPU : m_uiFramebuffersPtrGPU)
        {
            vkDestroyFramebuffer(devicePtrGPU, framebufferPtrGPU, nullptr);
        }
        m_uiFramebuffersPtrGPU.clear();

        if (m_colorImageViewPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyImageView(devicePtrGPU, m_colorImageViewPtrGPU, nullptr);
            m_colorImageViewPtrGPU = VK_NULL_HANDLE;
        }

        if (m_colorImagePtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyImage(devicePtrGPU, m_colorImagePtrGPU, nullptr);
            m_colorImagePtrGPU = VK_NULL_HANDLE;
        }

        if (m_colorImageMemoryPtrGPU != VK_NULL_HANDLE)
        {
            vkFreeMemory(devicePtrGPU, m_colorImageMemoryPtrGPU, nullptr);
            m_colorImageMemoryPtrGPU = VK_NULL_HANDLE;
        }

        if (m_depthImageViewPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyImageView(devicePtrGPU, m_depthImageViewPtrGPU, nullptr);
            m_depthImageViewPtrGPU = VK_NULL_HANDLE;
        }

        if (m_depthImagePtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyImage(devicePtrGPU, m_depthImagePtrGPU, nullptr);
            m_depthImagePtrGPU = VK_NULL_HANDLE;
        }

        if (m_depthImageMemoryPtrGPU != VK_NULL_HANDLE)
        {
            vkFreeMemory(devicePtrGPU, m_depthImageMemoryPtrGPU, nullptr);
            m_depthImageMemoryPtrGPU = VK_NULL_HANDLE;
        }

        if (m_renderPassPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyRenderPass(devicePtrGPU, m_renderPassPtrGPU, nullptr);
            m_renderPassPtrGPU = VK_NULL_HANDLE;
        }

        if (m_uiRenderPassPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyRenderPass(devicePtrGPU, m_uiRenderPassPtrGPU, nullptr);
            m_uiRenderPassPtrGPU = VK_NULL_HANDLE;
        }

        for (VkImageView imageViewPtrGPU : m_swapChainImageViewsPtrGPU)
        {
            vkDestroyImageView(devicePtrGPU, imageViewPtrGPU, nullptr);
        }
        m_swapChainImageViewsPtrGPU.clear();

        if (m_swapChainPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroySwapchainKHR(devicePtrGPU, m_swapChainPtrGPU, nullptr);
            m_swapChainPtrGPU = VK_NULL_HANDLE;
        }
    }

    //Releases shadow map resources.
    void EngineCore::CleanShadowResources(void)
    {
        if (!m_graphicsUtility.DeviceReady())
        {
            return;
        }

        const VkDevice devicePtrGPU = m_graphicsUtility.Device();

        for (ShadowMapResources &resources : m_shadowMapResources)
        {
            CleanShadowMapResources(resources);
        }
        m_shadowMapResources.clear();

        if (m_shadowSamplerPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroySampler(devicePtrGPU, m_shadowSamplerPtrGPU, nullptr);
            m_shadowSamplerPtrGPU = VK_NULL_HANDLE;
        }

        if (m_shadowRenderPassPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyRenderPass(devicePtrGPU, m_shadowRenderPassPtrGPU, nullptr);
            m_shadowRenderPassPtrGPU = VK_NULL_HANDLE;
        }
    }

    //Releases one shadow map image set.
    void EngineCore::CleanShadowMapResources(ShadowMapResources &resources) const
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();

        if (resources.m_framebufferPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyFramebuffer(devicePtrGPU, resources.m_framebufferPtrGPU, nullptr);
            resources.m_framebufferPtrGPU = VK_NULL_HANDLE;
        }

        if (resources.m_depthImageViewPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyImageView(devicePtrGPU, resources.m_depthImageViewPtrGPU, nullptr);
            resources.m_depthImageViewPtrGPU = VK_NULL_HANDLE;
        }

        if (resources.m_depthImagePtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyImage(devicePtrGPU, resources.m_depthImagePtrGPU, nullptr);
            resources.m_depthImagePtrGPU = VK_NULL_HANDLE;
        }

        if (resources.m_depthImageMemoryPtrGPU != VK_NULL_HANDLE)
        {
            vkFreeMemory(devicePtrGPU, resources.m_depthImageMemoryPtrGPU, nullptr);
            resources.m_depthImageMemoryPtrGPU = VK_NULL_HANDLE;
        }
    }

    //Rebuilds the swapchain after resize.
    void EngineCore::RecreateSwapChain(void)
    {
        while ((m_window.Width() == 0 || m_window.Height() == 0) && !m_window.ShouldClose())
        {
            m_window.WaitForEvents();
        }

        if (m_window.ShouldClose())
        {
            return;
        }

        m_graphicsUtility.WaitIdle();

        CleanupSwapChain();

        CreateSwapChain();
        CreateImageViews();
        CreateRenderPass();
        CreateUIRenderPass();
        m_scene.RefreshPipelines(SceneObjectContextInfo());
        m_uiUtility.RefreshPipeline(m_uiRenderPassPtrGPU, m_swapChainExtent);
        CreateColorResources();
        CreateDepthResources();
        m_openXRUtility.InitRenderResources(m_graphicsUtility.PhysicalDevice(), m_renderPassPtrGPU, m_swapChainImageFormat, m_graphicsUtility.FindDepthFormat(), m_sceneSampleCount);
        CreateFramebuffers();
        CreateUIFramebuffers();
    }

    //Draws one frame.
    void EngineCore::DrawFrame(void)
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();

        vkWaitForFences(devicePtrGPU, 1, &m_inFlightFencesPtrGPU[m_currentFrame], VK_TRUE, std::numeric_limits<uint64_t>::max());

        uint32_t imageIndex = 0;
        VkResult result = vkAcquireNextImageKHR(devicePtrGPU, m_swapChainPtrGPU, std::numeric_limits<uint64_t>::max(), m_imageAvailableSemaphoresPtrGPU[m_currentFrame], VK_NULL_HANDLE, &imageIndex);

        if (result == VK_ERROR_OUT_OF_DATE_KHR)
        {
            RecreateSwapChain();
            return;
        }

        if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR)
        {
            throw std::runtime_error("Failed to acquire Vulkan swapchain image.");
        }

        vkResetFences(devicePtrGPU, 1, &m_inFlightFencesPtrGPU[m_currentFrame]);

        const auto now = std::chrono::steady_clock::now();
        const float elapsedSeconds = std::chrono::duration<float>(now - m_startTime).count();
        const float deltaSeconds = std::chrono::duration<float>(now - m_previousFrameTime).count();
        m_previousFrameTime = now;
        m_fpsElapsedSeconds += deltaSeconds;
        ++m_fpsFrameCount;

        if (m_fpsElapsedSeconds >= 0.25f)
        {
            m_framesPerSecond = static_cast<float>(m_fpsFrameCount) / m_fpsElapsedSeconds;
            m_fpsElapsedSeconds = 0.0f;
            m_fpsFrameCount = 0;
        }

        m_openXRUtility.UpdateControllerState();
        m_scene.UpdateInput(m_inputUtility, deltaSeconds, InputMovementBasis(), &m_openXRUtility.ControllerState());
        UpdateUniformBuffer(m_currentFrame);
        m_scene.Update(elapsedSeconds, deltaSeconds);
        CaptureFrameOnInput();
        DrawXRFrame();
        UpdateUniformBuffer(m_currentFrame);

        m_uiUtility.BeginFrame(m_swapChainExtent, m_currentFrame);
        m_uiUtility.UpdateInput(m_inputUtility);
        m_uiUtility.DrawWindows();
        m_scene.ShowUI(m_uiUtility, m_programSetup.m_ui, m_framesPerSecond, m_swapChainExtent);

        vkResetCommandBuffer(m_commandBuffersPtrGPU[m_currentFrame], 0);
        RecordCommandBuffer(m_commandBuffersPtrGPU[m_currentFrame], imageIndex);

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

        const VkSemaphore waitSemaphoresPtrGPU[] = { m_imageAvailableSemaphoresPtrGPU[m_currentFrame] };
        const VkPipelineStageFlags waitStages[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };
        submitInfo.waitSemaphoreCount = 1;
        submitInfo.pWaitSemaphores = waitSemaphoresPtrGPU;
        submitInfo.pWaitDstStageMask = waitStages;

        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &m_commandBuffersPtrGPU[m_currentFrame];

        const VkSemaphore signalSemaphoresPtrGPU[] = { m_renderFinishedSemaphoresPtrGPU[m_currentFrame] };
        submitInfo.signalSemaphoreCount = 1;
        submitInfo.pSignalSemaphores = signalSemaphoresPtrGPU;

        if (vkQueueSubmit(m_graphicsUtility.GraphicsQueue(), 1, &submitInfo, m_inFlightFencesPtrGPU[m_currentFrame]) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to submit Vulkan draw command buffer.");
        }

        VkPresentInfoKHR presentInfo{};
        presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
        presentInfo.waitSemaphoreCount = 1;
        presentInfo.pWaitSemaphores = signalSemaphoresPtrGPU;

        const VkSwapchainKHR swapChainsPtrGPU[] = { m_swapChainPtrGPU };
        presentInfo.swapchainCount = 1;
        presentInfo.pSwapchains = swapChainsPtrGPU;
        presentInfo.pImageIndices = &imageIndex;

        result = vkQueuePresentKHR(m_graphicsUtility.PresentQueue(), &presentInfo);

        if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || m_window.WasResized())
        {
            m_window.ResetResizedFlag();
            RecreateSwapChain();
        }
        else if (result != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to present Vulkan swapchain image.");
        }

        m_currentFrame = (m_currentFrame + 1) % static_cast<uint32_t>(s_maxFramesInFlight);
    }

    //Draws one XR frame.
    void EngineCore::DrawXRFrame(void)
    {
        m_xrCenterEyeCameraValid = false;

        if (!m_openXRUtility.SessionReady() || m_descriptorSetsPtrGPU.empty())
        {
            return;
        }

        std::vector<XrView> renderedViews;
        const OpenXRUtility::EyeRenderCallback renderCallback = [this, &renderedViews](const OpenXRUtility::EyeRenderInfo &renderInfo)
            {
                UpdateXREnvironmentDepthDescriptor(m_currentFrame, renderInfo);
                const CameraBuffer camera = XRCameraBuffer(renderInfo);
                UpdateUniformBuffer(m_currentFrame, camera);
                m_scene.Show(renderInfo.m_commandBufferPtrGPU, m_descriptorSetsPtrGPU[m_currentFrame], renderInfo.m_extent);
                renderedViews.push_back(renderInfo.m_view);
            };

        m_openXRUtility.RenderFrame(m_programSetup.m_graphics.m_clearColor, renderCallback);

        if (!renderedViews.empty())
        {
            const float aspect = static_cast<float>(m_swapChainExtent.width) / static_cast<float>(m_swapChainExtent.height);
            m_xrCenterEyeCamera = XRCenterEyeCameraBuffer(renderedViews, aspect);
            m_xrCenterEyeMovementBasis = MovementBasisFromCameraBuffer(m_xrCenterEyeCamera);
            m_xrCenterEyeCameraValid = true;
        }
    }

    //Returns the camera movement basis for input.
    const CameraMovementBasis *EngineCore::InputMovementBasis(void) const
    {
        if (!XRDesktopCenterEyeMirrorEnabled() || !m_xrCenterEyeCameraValid)
        {
            return nullptr;
        }

        return &m_xrCenterEyeMovementBasis;
    }

    //Captures the frame when the trigger key is pressed.
    void EngineCore::CaptureFrameOnInput(void)
    {
        if (!m_scene.ConsumeCaptureFrameRequest())
        {
            return;
        }

        try
        {
            UpdateUniformBuffer(m_currentFrame);
            SaveFrameBuffers();
        }
        catch (const std::exception &error)
        {
            UIUtility::Print("Frame capture failed: " + std::string(error.what()) + "\n");
        }
    }

    //Updates the camera uniform buffer.
    void EngineCore::UpdateUniformBuffer(uint32_t currentImage)
    {
        const float aspect = static_cast<float>(m_swapChainExtent.width) / static_cast<float>(m_swapChainExtent.height);
        const CameraBuffer camera = XRDesktopCenterEyeMirrorEnabled() && m_xrCenterEyeCameraValid ? m_xrCenterEyeCamera : m_scene.CameraBufferInfo(aspect);

        UpdateUniformBuffer(currentImage, camera);
    }

    //Updates the camera uniform buffer with explicit values.
    void EngineCore::UpdateUniformBuffer(uint32_t currentImage, const CameraBuffer &camera)
    {
        std::memcpy(m_uniformBuffersMappedPtr[currentImage], &camera, sizeof(camera));
    }

    //Updates the XR environment depth descriptor.
    void EngineCore::UpdateXREnvironmentDepthDescriptor(uint32_t currentImage, const OpenXRUtility::EyeRenderInfo &renderInfo)
    {
        if (!renderInfo.m_environmentDepthAvailable || renderInfo.m_environmentDepthImageViewPtrGPU == VK_NULL_HANDLE || renderInfo.m_environmentDepthSamplerPtrGPU == VK_NULL_HANDLE ||
            currentImage >= m_descriptorSetsPtrGPU.size())
        {
            return;
        }

        VkDescriptorImageInfo imageInfo{};
        imageInfo.sampler = renderInfo.m_environmentDepthSamplerPtrGPU;
        imageInfo.imageView = renderInfo.m_environmentDepthImageViewPtrGPU;
        imageInfo.imageLayout = renderInfo.m_environmentDepthImageLayout;

        VkWriteDescriptorSet descriptorWrite{};
        descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        descriptorWrite.dstSet = m_descriptorSetsPtrGPU[currentImage];
        descriptorWrite.dstBinding = 2;
        descriptorWrite.dstArrayElement = 0;
        descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        descriptorWrite.descriptorCount = 1;
        descriptorWrite.pImageInfo = &imageInfo;

        vkUpdateDescriptorSets(m_graphicsUtility.Device(), 1, &descriptorWrite, 0, nullptr);
    }

    //Returns the camera buffer for one XR eye.
    CameraBuffer EngineCore::XRCameraBuffer(const XrView &view) const
    {
        CameraBuffer camera = m_scene.CameraBufferInfo(1.0f);
        const CameraBuffer rigCamera = m_scene.CameraBufferInfo(1.0f);
        const CameraBasis rigBasis = BasisFromCameraBuffer(rigCamera);
        const CameraBasis eyeBasis = BasisFromRigAndXrView(rigBasis, view);

        camera.m_view = ViewFromBasis(eyeBasis);
        camera.m_projection = ProjectionFromXrFov(view.fov, 0.05f, 500.0f);
        camera.m_cameraPosition = {
            eyeBasis.m_position.x, eyeBasis.m_position.y, eyeBasis.m_position.z, 1.0f
        };
        return camera;
    }

    //Returns the camera buffer for one XR eye render.
    CameraBuffer EngineCore::XRCameraBuffer(const OpenXRUtility::EyeRenderInfo &renderInfo) const
    {
        CameraBuffer camera = XRCameraBuffer(renderInfo.m_view);
        const bool fullScreenDepthDebug = XRFullScreenDepthDebugRequested(m_programSetup.m_xr.m_passthroughDepthDebugView);
        camera.m_xrDepthSettings = {
            0.0f, m_programSetup.m_xr.m_passthroughDepthBiasMeters, m_programSetup.m_xr.m_passthroughDepthMaxMeters,
            fullScreenDepthDebug ? 0.0f : XRDepthDebugModeValue(m_programSetup.m_xr.m_passthroughDepthDebugView)
        };
        camera.m_xrDepthModeSettings = {
            XRDepthValueModeValue(m_programSetup.m_xr.m_passthroughDepthValueMode), XRAlphaDebugModeValue(m_programSetup.m_xr.m_passthroughAlphaDebug), 0.0f, 0.0f
        };

        if (!renderInfo.m_environmentDepthAvailable || fullScreenDepthDebug || !m_programSetup.m_xr.m_passthroughDepthTextureSampling)
        {
            return camera;
        }

        const CameraBuffer rigCamera = m_scene.CameraBufferInfo(1.0f);
        const CameraBasis rigBasis = BasisFromCameraBuffer(rigCamera);
        const CameraBasis depthBasis = BasisFromRigAndXrPose(rigBasis, renderInfo.m_environmentDepthView.pose);
        const float nearPlane = XRDepthNearPlane(renderInfo.m_environmentDepthNearZ);
        const float farPlane = XRDepthFarPlane(renderInfo.m_environmentDepthFarZ, nearPlane, m_programSetup.m_xr.m_passthroughDepthMaxMeters);

        camera.m_xrEnvironmentDepthView = ViewFromBasis(depthBasis);
        camera.m_xrEnvironmentDepthViewProjection = multiply(ProjectionFromXrFov(renderInfo.m_environmentDepthView.fov, nearPlane, farPlane), camera.m_xrEnvironmentDepthView);
        camera.m_xrDepthSettings.x = 1.0f;
        camera.m_xrDepthExtent = {
            static_cast<float>(renderInfo.m_environmentDepthExtent.width), static_cast<float>(renderInfo.m_environmentDepthExtent.height), nearPlane, farPlane
        };
        return camera;
    }

    //Returns the camera buffer for XR center-eye desktop mirror.
    CameraBuffer EngineCore::XRCenterEyeCameraBuffer(const std::vector<XrView> &views, float aspect) const
    {
        CameraBuffer camera = m_scene.CameraBufferInfo(aspect);
        const CameraBuffer rigCamera = m_scene.CameraBufferInfo(aspect);
        const CameraBasis rigBasis = BasisFromCameraBuffer(rigCamera);

        Float3 position{ 0.0f, 0.0f, 0.0f };
        Float3 right{ 0.0f, 0.0f, 0.0f };
        Float3 up{ 0.0f, 0.0f, 0.0f };
        Float3 back{ 0.0f, 0.0f, 0.0f };

        for (const XrView &view : views)
        {
            const CameraBasis eyeBasis = BasisFromRigAndXrView(rigBasis, view);
            position += eyeBasis.m_position;
            right += eyeBasis.m_right;
            up += eyeBasis.m_up;
            back += eyeBasis.m_back;
        }

        const float scaleAmount = 1.0f / static_cast<float>(views.size());
        CameraBasis centerBasis{};
        centerBasis.m_position = position * scaleAmount;
        centerBasis.m_up = normalizeOrDefault(up * scaleAmount, rigBasis.m_up);
        centerBasis.m_back = normalizeOrDefault(back * scaleAmount, rigBasis.m_back);
        centerBasis.m_right = normalizeOrDefault(cross(centerBasis.m_up, centerBasis.m_back), right * scaleAmount);
        centerBasis.m_up = normalizeOrDefault(cross(centerBasis.m_back, centerBasis.m_right), rigBasis.m_up);

        camera.m_view = ViewFromBasis(centerBasis);
        camera.m_cameraPosition = {
            centerBasis.m_position.x, centerBasis.m_position.y, centerBasis.m_position.z, 1.0f
        };
        return camera;
    }

    //Returns whether desktop should mirror the XR center eye.
    bool EngineCore::XRDesktopCenterEyeMirrorEnabled(void) const
    {
        const std::string mirrorMode = StringUtility::Uppercase(m_programSetup.m_xr.m_mirrorMode);
        return m_programSetup.m_xr.m_mirrorToWindow && m_openXRUtility.SessionReady() && (mirrorMode == "CENTEREYE" || mirrorMode == "CENTER_EYE" || mirrorMode == "CENTER");
    }

    //Records one frame command buffer.
    void EngineCore::RecordCommandBuffer(VkCommandBuffer commandBufferPtrGPU, uint32_t imageIndex)
    {
        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

        if (vkBeginCommandBuffer(commandBufferPtrGPU, &beginInfo) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to begin recording Vulkan command buffer.");
        }

        RecordShadowMap(commandBufferPtrGPU);

        const Float4 clearColor = m_programSetup.m_graphics.m_clearColor;
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
        renderPassInfo.framebuffer = m_swapChainFramebuffersPtrGPU[imageIndex];
        renderPassInfo.renderArea.offset = { 0, 0 };
        renderPassInfo.renderArea.extent = m_swapChainExtent;
        renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
        renderPassInfo.pClearValues = clearValues.data();

        vkCmdBeginRenderPass(commandBufferPtrGPU, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

        m_scene.Show(commandBufferPtrGPU, m_descriptorSetsPtrGPU[m_currentFrame]);

        vkCmdEndRenderPass(commandBufferPtrGPU);

        VkRenderPassBeginInfo uiRenderPassInfo{};
        uiRenderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        uiRenderPassInfo.renderPass = m_uiRenderPassPtrGPU;
        uiRenderPassInfo.framebuffer = m_uiFramebuffersPtrGPU[imageIndex];
        uiRenderPassInfo.renderArea.offset = { 0, 0 };
        uiRenderPassInfo.renderArea.extent = m_swapChainExtent;

        vkCmdBeginRenderPass(commandBufferPtrGPU, &uiRenderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

        m_uiUtility.Show(commandBufferPtrGPU);

        vkCmdEndRenderPass(commandBufferPtrGPU);

        if (vkEndCommandBuffer(commandBufferPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to record Vulkan command buffer.");
        }
    }

    //Records the shadow map pass.
    void EngineCore::RecordShadowMap(VkCommandBuffer commandBufferPtrGPU)
    {
        if (m_shadowMapResources.empty())
        {
            return;
        }

        const ShadowMapResources &shadowResources = m_shadowMapResources[m_currentFrame];

        VkClearValue clearValue{};
        clearValue.depthStencil.depth = 1.0f;
        clearValue.depthStencil.stencil = 0;

        VkRenderPassBeginInfo renderPassInfo{};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        renderPassInfo.renderPass = m_shadowRenderPassPtrGPU;
        renderPassInfo.framebuffer = shadowResources.m_framebufferPtrGPU;
        renderPassInfo.renderArea.offset = { 0, 0 };
        renderPassInfo.renderArea.extent = m_shadowExtent;
        renderPassInfo.clearValueCount = 1;
        renderPassInfo.pClearValues = &clearValue;

        vkCmdBeginRenderPass(commandBufferPtrGPU, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

        if (m_programSetup.m_lighting.m_shadowsEnabled)
        {
            m_scene.ShowShadow(commandBufferPtrGPU, m_descriptorSetsPtrGPU[m_currentFrame], m_shadowExtent);
        }

        vkCmdEndRenderPass(commandBufferPtrGPU);
    }

    //Captures and saves one scene frame.
    void EngineCore::SaveFrameBuffers(void)
    {
        FrameCaptureResources frameResources{};
        FrameCaptureResources normalResources{};
        VkCommandBuffer commandBufferPtrGPU = VK_NULL_HANDLE;
        bool capturePipelinesActive = false;

        try
        {
            CreateFrameCaptureResources(frameResources);
            CreateFrameCaptureResources(normalResources);

            m_scene.RefreshPipelines(FrameCaptureContextInfo(frameResources.m_renderPassPtrGPU, false));
            capturePipelinesActive = true;

            VkCommandBufferAllocateInfo allocInfo{};
            allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
            allocInfo.commandPool = m_graphicsUtility.CommandPool();
            allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
            allocInfo.commandBufferCount = 1;

            if (vkAllocateCommandBuffers(m_graphicsUtility.Device(), &allocInfo, &commandBufferPtrGPU) != VK_SUCCESS)
            {
                throw std::runtime_error("Failed to allocate frame capture command buffer.");
            }

            RecordFrameCaptureCommandBuffer(commandBufferPtrGPU, frameResources, true);
            SubmitSingleUseCommandBuffer(commandBufferPtrGPU);

            vkFreeCommandBuffers(m_graphicsUtility.Device(), m_graphicsUtility.CommandPool(), 1, &commandBufferPtrGPU);
            commandBufferPtrGPU = VK_NULL_HANDLE;

            const TextureData colorTexture = ReadColorCapture(frameResources);
            const TextureData depthTexture = ReadDepthCapture(frameResources);

            m_scene.RefreshPipelines(FrameCaptureContextInfo(normalResources.m_renderPassPtrGPU, true));

            if (vkAllocateCommandBuffers(m_graphicsUtility.Device(), &allocInfo, &commandBufferPtrGPU) != VK_SUCCESS)
            {
                throw std::runtime_error("Failed to allocate normal capture command buffer.");
            }

            RecordFrameCaptureCommandBuffer(commandBufferPtrGPU, normalResources, false);
            SubmitSingleUseCommandBuffer(commandBufferPtrGPU);

            vkFreeCommandBuffers(m_graphicsUtility.Device(), m_graphicsUtility.CommandPool(), 1, &commandBufferPtrGPU);
            commandBufferPtrGPU = VK_NULL_HANDLE;

            const TextureData normalTexture = ReadColorCapture(normalResources);
            const std::string colorFileName = OutputFilePath("FrameColor.png");
            const std::string depthFileName = OutputFilePath("FrameDepth.png");
            const std::string normalFileName = OutputFilePath("FrameNormal.png");

            if (!SaveTexture(colorFileName, colorTexture, TextureFileType::Png))
            {
                throw std::runtime_error("Failed to save FrameColor.png.");
            }

            if (!SaveTexture(depthFileName, depthTexture, TextureFileType::Png))
            {
                throw std::runtime_error("Failed to save FrameDepth.png.");
            }

            if (!SaveTexture(normalFileName, normalTexture, TextureFileType::Png))
            {
                throw std::runtime_error("Failed to save FrameNormal.png.");
            }

            UIUtility::Print("Frame buffer saved.\n");

            m_scene.RefreshPipelines(SceneObjectContextInfo());
            capturePipelinesActive = false;
            CleanFrameCaptureResources(normalResources);
            CleanFrameCaptureResources(frameResources);
        }
        catch (...)
        {
            if (commandBufferPtrGPU != VK_NULL_HANDLE)
            {
                vkFreeCommandBuffers(m_graphicsUtility.Device(), m_graphicsUtility.CommandPool(), 1, &commandBufferPtrGPU);
            }

            if (capturePipelinesActive)
            {
                try
                {
                    m_graphicsUtility.WaitIdle();
                    m_scene.RefreshPipelines(SceneObjectContextInfo());
                }
                catch (...)
                {
                }
            }

            CleanFrameCaptureResources(normalResources);
            CleanFrameCaptureResources(frameResources);
            throw;
        }
    }

    //Creates one-shot frame capture resources.
    void EngineCore::CreateFrameCaptureResources(FrameCaptureResources &resources)
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();
        resources.m_colorFormat = m_swapChainImageFormat;
        resources.m_depthFormat = m_graphicsUtility.FindSupportedFormat(
            { VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT },
            VK_IMAGE_TILING_OPTIMAL,
            VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_FORMAT_FEATURE_TRANSFER_SRC_BIT
        );

        VkAttachmentDescription colorAttachment{};
        colorAttachment.format = resources.m_colorFormat;
        colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        colorAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkAttachmentDescription depthAttachment{};
        depthAttachment.format = resources.m_depthFormat;
        depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
        depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkAttachmentReference colorAttachmentRef{};
        colorAttachmentRef.attachment = 0;
        colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkAttachmentReference depthAttachmentRef{};
        depthAttachmentRef.attachment = 1;
        depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorAttachmentRef;
        subpass.pDepthStencilAttachment = &depthAttachmentRef;

        const std::array<VkAttachmentDescription, 2> attachments = {
            colorAttachment, depthAttachment
        };

        const std::array<VkSubpassDependency, 2> dependencies = {
            VkSubpassDependency{
                VK_SUBPASS_EXTERNAL, 0, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT,
                VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT, 0, VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT, 0
            }, VkSubpassDependency{
                0, VK_SUBPASS_EXTERNAL, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
                VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_TRANSFER_READ_BIT, 0
            }
        };

        VkRenderPassCreateInfo renderPassInfo{};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        renderPassInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
        renderPassInfo.pAttachments = attachments.data();
        renderPassInfo.subpassCount = 1;
        renderPassInfo.pSubpasses = &subpass;
        renderPassInfo.dependencyCount = static_cast<uint32_t>(dependencies.size());
        renderPassInfo.pDependencies = dependencies.data();

        if (vkCreateRenderPass(devicePtrGPU, &renderPassInfo, nullptr, &resources.m_renderPassPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create frame capture render pass.");
        }

        m_graphicsUtility.CreateImage(
            m_swapChainExtent.width,
            m_swapChainExtent.height,
            resources.m_colorFormat,
            VK_IMAGE_TILING_OPTIMAL,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
            VK_SAMPLE_COUNT_1_BIT,
            resources.m_colorImagePtrGPU,
            resources.m_colorImageMemoryPtrGPU
        );

        resources.m_colorImageViewPtrGPU = m_graphicsUtility.CreateImageView(resources.m_colorImagePtrGPU, resources.m_colorFormat, VK_IMAGE_ASPECT_COLOR_BIT);

        m_graphicsUtility.CreateImage(
            m_swapChainExtent.width,
            m_swapChainExtent.height,
            resources.m_depthFormat,
            VK_IMAGE_TILING_OPTIMAL,
            VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
            VK_SAMPLE_COUNT_1_BIT,
            resources.m_depthImagePtrGPU,
            resources.m_depthImageMemoryPtrGPU
        );

        resources.m_depthImageViewPtrGPU = m_graphicsUtility.CreateImageView(resources.m_depthImagePtrGPU, resources.m_depthFormat, VK_IMAGE_ASPECT_DEPTH_BIT);

        const std::array<VkImageView, 2> framebufferAttachments = {
            resources.m_colorImageViewPtrGPU, resources.m_depthImageViewPtrGPU
        };

        VkFramebufferCreateInfo framebufferInfo{};
        framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        framebufferInfo.renderPass = resources.m_renderPassPtrGPU;
        framebufferInfo.attachmentCount = static_cast<uint32_t>(framebufferAttachments.size());
        framebufferInfo.pAttachments = framebufferAttachments.data();
        framebufferInfo.width = m_swapChainExtent.width;
        framebufferInfo.height = m_swapChainExtent.height;
        framebufferInfo.layers = 1;

        if (vkCreateFramebuffer(devicePtrGPU, &framebufferInfo, nullptr, &resources.m_framebufferPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create frame capture framebuffer.");
        }

        const VkDeviceSize pixelCount = static_cast<VkDeviceSize>(m_swapChainExtent.width) * static_cast<VkDeviceSize>(m_swapChainExtent.height);
        resources.m_colorByteCount = pixelCount * 4U;
        resources.m_depthByteCount = pixelCount * 4U;

        m_graphicsUtility.CreateBuffer(
            resources.m_colorByteCount,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            resources.m_colorReadBufferPtrGPU,
            resources.m_colorReadBufferMemoryPtrGPU
        );

        m_graphicsUtility.CreateBuffer(
            resources.m_depthByteCount,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            resources.m_depthReadBufferPtrGPU,
            resources.m_depthReadBufferMemoryPtrGPU
        );
    }

    //Releases one-shot frame capture resources.
    void EngineCore::CleanFrameCaptureResources(FrameCaptureResources &resources) const
    {
        const VkDevice devicePtrGPU = m_graphicsUtility.Device();

        if (resources.m_depthReadBufferPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyBuffer(devicePtrGPU, resources.m_depthReadBufferPtrGPU, nullptr);
            resources.m_depthReadBufferPtrGPU = VK_NULL_HANDLE;
        }

        if (resources.m_depthReadBufferMemoryPtrGPU != VK_NULL_HANDLE)
        {
            vkFreeMemory(devicePtrGPU, resources.m_depthReadBufferMemoryPtrGPU, nullptr);
            resources.m_depthReadBufferMemoryPtrGPU = VK_NULL_HANDLE;
        }

        if (resources.m_colorReadBufferPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyBuffer(devicePtrGPU, resources.m_colorReadBufferPtrGPU, nullptr);
            resources.m_colorReadBufferPtrGPU = VK_NULL_HANDLE;
        }

        if (resources.m_colorReadBufferMemoryPtrGPU != VK_NULL_HANDLE)
        {
            vkFreeMemory(devicePtrGPU, resources.m_colorReadBufferMemoryPtrGPU, nullptr);
            resources.m_colorReadBufferMemoryPtrGPU = VK_NULL_HANDLE;
        }

        if (resources.m_framebufferPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyFramebuffer(devicePtrGPU, resources.m_framebufferPtrGPU, nullptr);
            resources.m_framebufferPtrGPU = VK_NULL_HANDLE;
        }

        if (resources.m_depthImageViewPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyImageView(devicePtrGPU, resources.m_depthImageViewPtrGPU, nullptr);
            resources.m_depthImageViewPtrGPU = VK_NULL_HANDLE;
        }

        if (resources.m_depthImagePtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyImage(devicePtrGPU, resources.m_depthImagePtrGPU, nullptr);
            resources.m_depthImagePtrGPU = VK_NULL_HANDLE;
        }

        if (resources.m_depthImageMemoryPtrGPU != VK_NULL_HANDLE)
        {
            vkFreeMemory(devicePtrGPU, resources.m_depthImageMemoryPtrGPU, nullptr);
            resources.m_depthImageMemoryPtrGPU = VK_NULL_HANDLE;
        }

        if (resources.m_colorImageViewPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyImageView(devicePtrGPU, resources.m_colorImageViewPtrGPU, nullptr);
            resources.m_colorImageViewPtrGPU = VK_NULL_HANDLE;
        }

        if (resources.m_colorImagePtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyImage(devicePtrGPU, resources.m_colorImagePtrGPU, nullptr);
            resources.m_colorImagePtrGPU = VK_NULL_HANDLE;
        }

        if (resources.m_colorImageMemoryPtrGPU != VK_NULL_HANDLE)
        {
            vkFreeMemory(devicePtrGPU, resources.m_colorImageMemoryPtrGPU, nullptr);
            resources.m_colorImageMemoryPtrGPU = VK_NULL_HANDLE;
        }

        if (resources.m_renderPassPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyRenderPass(devicePtrGPU, resources.m_renderPassPtrGPU, nullptr);
            resources.m_renderPassPtrGPU = VK_NULL_HANDLE;
        }
    }

    //Records one frame capture command buffer.
    void EngineCore::RecordFrameCaptureCommandBuffer(VkCommandBuffer commandBufferPtrGPU, const FrameCaptureResources &resources, bool copyDepthBuffer)
    {
        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

        if (vkBeginCommandBuffer(commandBufferPtrGPU, &beginInfo) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to begin frame capture command buffer.");
        }

        const Float4 clearColor = m_programSetup.m_graphics.m_clearColor;
        std::array<VkClearValue, 2> clearValues{};
        clearValues[0].color.float32[0] = clearColor.x;
        clearValues[0].color.float32[1] = clearColor.y;
        clearValues[0].color.float32[2] = clearColor.z;
        clearValues[0].color.float32[3] = clearColor.w;
        clearValues[1].depthStencil.depth = 1.0f;
        clearValues[1].depthStencil.stencil = 0;

        VkRenderPassBeginInfo renderPassInfo{};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        renderPassInfo.renderPass = resources.m_renderPassPtrGPU;
        renderPassInfo.framebuffer = resources.m_framebufferPtrGPU;
        renderPassInfo.renderArea.offset = { 0, 0 };
        renderPassInfo.renderArea.extent = m_swapChainExtent;
        renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
        renderPassInfo.pClearValues = clearValues.data();

        vkCmdBeginRenderPass(commandBufferPtrGPU, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
        m_scene.Show(commandBufferPtrGPU, m_descriptorSetsPtrGPU[m_currentFrame]);
        vkCmdEndRenderPass(commandBufferPtrGPU);

        TransitionImageLayout(commandBufferPtrGPU, resources.m_colorImagePtrGPU, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_ASPECT_COLOR_BIT);
        CopyImageToBuffer(commandBufferPtrGPU, resources.m_colorImagePtrGPU, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_ASPECT_COLOR_BIT, resources.m_colorReadBufferPtrGPU);

        if (copyDepthBuffer)
        {
            TransitionImageLayout(commandBufferPtrGPU, resources.m_depthImagePtrGPU, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_ASPECT_DEPTH_BIT);
            CopyImageToBuffer(commandBufferPtrGPU, resources.m_depthImagePtrGPU, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_ASPECT_DEPTH_BIT, resources.m_depthReadBufferPtrGPU);
        }

        if (vkEndCommandBuffer(commandBufferPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to record frame capture command buffer.");
        }
    }

    //Transitions an image layout inside a command buffer.
    void EngineCore::TransitionImageLayout(VkCommandBuffer commandBufferPtrGPU, VkImage imagePtrGPU, VkImageLayout oldLayout, VkImageLayout newLayout, VkImageAspectFlags aspectFlags) const
    {
        VkImageMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = oldLayout;
        barrier.newLayout = newLayout;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = imagePtrGPU;
        barrier.subresourceRange.aspectMask = aspectFlags;
        barrier.subresourceRange.baseMipLevel = 0;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.baseArrayLayer = 0;
        barrier.subresourceRange.layerCount = 1;

        VkPipelineStageFlags sourceStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        VkPipelineStageFlags destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;

        if ((aspectFlags & VK_IMAGE_ASPECT_DEPTH_BIT) != 0)
        {
            barrier.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
            sourceStage = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
        }
        else
        {
            barrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        }

        barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

        vkCmdPipelineBarrier(commandBufferPtrGPU, sourceStage, destinationStage, 0, 0, nullptr, 0, nullptr, 1, &barrier);
    }

    //Copies an image into a readback buffer.
    void EngineCore::CopyImageToBuffer(VkCommandBuffer commandBufferPtrGPU, VkImage imagePtrGPU, VkImageLayout imageLayout, VkImageAspectFlags aspectFlags, VkBuffer bufferPtrGPU) const
    {
        VkBufferImageCopy region{};
        region.bufferOffset = 0;
        region.bufferRowLength = 0;
        region.bufferImageHeight = 0;
        region.imageSubresource.aspectMask = aspectFlags;
        region.imageSubresource.mipLevel = 0;
        region.imageSubresource.baseArrayLayer = 0;
        region.imageSubresource.layerCount = 1;
        region.imageOffset = { 0, 0, 0 };
        region.imageExtent = {
            m_swapChainExtent.width, m_swapChainExtent.height, 1
        };

        vkCmdCopyImageToBuffer(commandBufferPtrGPU, imagePtrGPU, imageLayout, bufferPtrGPU, 1, &region);
    }

    //Submits a finished one-shot command buffer.
    void EngineCore::SubmitSingleUseCommandBuffer(VkCommandBuffer commandBufferPtrGPU) const
    {
        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBufferPtrGPU;

        if (vkQueueSubmit(m_graphicsUtility.GraphicsQueue(), 1, &submitInfo, VK_NULL_HANDLE) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to submit frame capture command buffer.");
        }

        vkQueueWaitIdle(m_graphicsUtility.GraphicsQueue());
    }

    //Reads the capture color buffer as texture pixels.
    TextureData EngineCore::ReadColorCapture(const FrameCaptureResources &resources) const
    {
        TextureData texture{};
        texture.m_width = m_swapChainExtent.width;
        texture.m_height = m_swapChainExtent.height;
        texture.m_channelCount = 4U;
        texture.m_pixels.resize(static_cast<std::size_t>(resources.m_colorByteCount));

        void *mappedDataPtr = nullptr;
        if (vkMapMemory(m_graphicsUtility.Device(), resources.m_colorReadBufferMemoryPtrGPU, 0, resources.m_colorByteCount, 0, &mappedDataPtr) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to map frame color buffer.");
        }

        const auto *sourcePixelsPtr = static_cast<const uint8_t *>(mappedDataPtr);
        const std::size_t pixelCount = static_cast<std::size_t>(texture.m_width) * static_cast<std::size_t>(texture.m_height);

        if (FormatIsBgra(resources.m_colorFormat))
        {
            for (std::size_t i = 0; i < pixelCount; ++i)
            {
                texture.m_pixels[(i * 4U) + 0U] = sourcePixelsPtr[(i * 4U) + 2U];
                texture.m_pixels[(i * 4U) + 1U] = sourcePixelsPtr[(i * 4U) + 1U];
                texture.m_pixels[(i * 4U) + 2U] = sourcePixelsPtr[(i * 4U) + 0U];
                texture.m_pixels[(i * 4U) + 3U] = sourcePixelsPtr[(i * 4U) + 3U];
            }
        }
        else if (FormatIsRgba(resources.m_colorFormat))
        {
            std::memcpy(texture.m_pixels.data(), sourcePixelsPtr, texture.m_pixels.size());
        }
        else
        {
            vkUnmapMemory(m_graphicsUtility.Device(), resources.m_colorReadBufferMemoryPtrGPU);
            throw std::runtime_error("Frame color capture format is not supported.");
        }

        vkUnmapMemory(m_graphicsUtility.Device(), resources.m_colorReadBufferMemoryPtrGPU);
        return texture;
    }

    //Reads the capture depth buffer as texture pixels.
    TextureData EngineCore::ReadDepthCapture(const FrameCaptureResources &resources) const
    {
        TextureData texture{};
        texture.m_width = m_swapChainExtent.width;
        texture.m_height = m_swapChainExtent.height;
        texture.m_channelCount = 4U;
        texture.m_pixels.resize(static_cast<std::size_t>(resources.m_depthByteCount));

        void *mappedDataPtr = nullptr;
        if (vkMapMemory(m_graphicsUtility.Device(), resources.m_depthReadBufferMemoryPtrGPU, 0, resources.m_depthByteCount, 0, &mappedDataPtr) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to map frame depth buffer.");
        }

        const auto *sourcePixelsPtr = static_cast<const uint8_t *>(mappedDataPtr);

        const std::size_t pixelCount = static_cast<std::size_t>(texture.m_width) * static_cast<std::size_t>(texture.m_height);

        if (FormatIsFloatDepth(resources.m_depthFormat))
        {
            for (std::size_t i = 0; i < pixelCount; ++i)
            {
                float depth = 1.0f;
                std::memcpy(&depth, sourcePixelsPtr + (i * 4U), sizeof(depth));
                const uint8_t depthByte = DepthToByte(depth);
                texture.m_pixels[(i * 4U) + 0U] = depthByte;
                texture.m_pixels[(i * 4U) + 1U] = depthByte;
                texture.m_pixels[(i * 4U) + 2U] = depthByte;
                texture.m_pixels[(i * 4U) + 3U] = 255U;
            }
        }
        else if (FormatIsDepth24Stencil8(resources.m_depthFormat))
        {
            constexpr float maxDepth24 = 16777215.0f;
            for (std::size_t i = 0; i < pixelCount; ++i)
            {
                uint32_t packedDepth = 0U;
                std::memcpy(&packedDepth, sourcePixelsPtr + (i * 4U), sizeof(packedDepth));
                const float depth = static_cast<float>(packedDepth & 0x00FFFFFFU) / maxDepth24;
                const uint8_t depthByte = DepthToByte(depth);
                texture.m_pixels[(i * 4U) + 0U] = depthByte;
                texture.m_pixels[(i * 4U) + 1U] = depthByte;
                texture.m_pixels[(i * 4U) + 2U] = depthByte;
                texture.m_pixels[(i * 4U) + 3U] = 255U;
            }
        }
        else
        {
            vkUnmapMemory(m_graphicsUtility.Device(), resources.m_depthReadBufferMemoryPtrGPU);
            throw std::runtime_error("Frame depth capture format is not supported.");
        }

        vkUnmapMemory(m_graphicsUtility.Device(), resources.m_depthReadBufferMemoryPtrGPU);
        return texture;
    }

    //Returns an output file path under the solution folder.
    std::string EngineCore::OutputFilePath(const std::string &fileName) const
    {
        const std::filesystem::path outputFolder = PathUtility::CaptureOutputFolder();
        std::filesystem::create_directories(outputFolder);
        return (outputFolder / fileName).string();
    }

    //Creates a context for frame capture rendering.
    SceneObjectContext EngineCore::FrameCaptureContextInfo(VkRenderPass renderPassPtrGPU, bool normalCapture) const
    {
        SceneObjectContext context = SceneObjectContextInfo();
        context.m_renderPassPtrGPU = renderPassPtrGPU;
        context.m_sampleCount = VK_SAMPLE_COUNT_1_BIT;

        if (normalCapture)
        {
            context.m_shaderOverrideEnabled = true;
            context.m_vertexShaderOverride = "shaders/normalCapture.vert";
            context.m_instancedVertexShaderOverride = "shaders/normalCaptureInstanced.vert";
            context.m_fragmentShaderOverride = "shaders/normalCapture.frag";
        }

        return context;
    }

    //Chooses the swapchain surface format.
    VkSurfaceFormatKHR EngineCore::ChooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR> &formats) const
    {
        for (const auto &availableFormat : formats)
        {
            if (availableFormat.format == VK_FORMAT_B8G8R8A8_SRGB && availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
            {
                return availableFormat;
            }
        }

        return formats[0];
    }

    //Chooses the present mode.
    VkPresentModeKHR EngineCore::ChooseSwapPresentMode(const std::vector<VkPresentModeKHR> &presentModes) const
    {
        if (m_programSetup.m_graphics.m_vsyncEnabled)
        {
            return VK_PRESENT_MODE_FIFO_KHR;
        }

        if (std::find(presentModes.begin(), presentModes.end(), VK_PRESENT_MODE_MAILBOX_KHR) != presentModes.end())
        {
            return VK_PRESENT_MODE_MAILBOX_KHR;
        }

        if (std::find(presentModes.begin(), presentModes.end(), VK_PRESENT_MODE_IMMEDIATE_KHR) != presentModes.end())
        {
            return VK_PRESENT_MODE_IMMEDIATE_KHR;
        }

        return VK_PRESENT_MODE_FIFO_KHR;
    }

    //Chooses the swapchain extent.
    VkExtent2D EngineCore::ChooseSwapExtent(const VkSurfaceCapabilitiesKHR &capabilities) const
    {
        if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max())
        {
            return capabilities.currentExtent;
        }

        VkExtent2D actualExtent = {
            m_window.Width(), m_window.Height()
        };

        actualExtent.width = std::clamp(actualExtent.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width);

        actualExtent.height = std::clamp(actualExtent.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height);

        return actualExtent;
    }

    //Creates a context for scene objects.
    SceneObjectContext EngineCore::SceneObjectContextInfo(void) const
    {
        SceneObjectContext context{};
        context.m_devicePtrGPU = m_graphicsUtility.Device();
        context.m_physicalDevicePtrGPU = m_graphicsUtility.PhysicalDevice();
        context.m_commandPoolPtrGPU = m_graphicsUtility.CommandPool();
        context.m_graphicsQueuePtrGPU = m_graphicsUtility.GraphicsQueue();
        context.m_renderPassPtrGPU = m_renderPassPtrGPU;
        context.m_shadowRenderPassPtrGPU = m_shadowRenderPassPtrGPU;
        context.m_descriptorSetLayoutPtrGPU = m_descriptorSetLayoutPtrGPU;
        context.m_textureDescriptorSetLayoutPtrGPU = m_textureDescriptorSetLayoutPtrGPU;
        context.m_descriptorPoolPtrGPU = m_descriptorPoolPtrGPU;
        context.m_swapChainExtent = m_swapChainExtent;
        context.m_shadowExtent = m_shadowExtent;
        context.m_wideLinesEnabled = m_graphicsUtility.WideLinesEnabled();
        context.m_anisotropyEnabled = m_graphicsUtility.AnisotropyEnabled();
        context.m_wireframeEnabled = m_graphicsUtility.WireframeEnabled();
        context.m_largePointsEnabled = m_graphicsUtility.LargePointsEnabled();
        context.m_minimumLineWidth = m_graphicsUtility.MinimumLineWidth();
        context.m_maximumLineWidth = m_graphicsUtility.MaximumLineWidth();
        context.m_minimumPointSize = m_graphicsUtility.MinimumPointSize();
        context.m_maximumPointSize = m_graphicsUtility.MaximumPointSize();
        context.m_wireframeLineWidth = m_programSetup.m_graphics.m_wireframeLineWidth;
        context.m_maximumAnisotropy = m_graphicsUtility.MaximumAnisotropy();
        context.m_sampleCount = m_sceneSampleCount;
        return context;
    }
}

