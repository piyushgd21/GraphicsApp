///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	GraphicsUtility.cpp
//	Description	:
//		Implements Vulkan device initialization and GPU resource helpers.
//
//
///////////////////////////////////////////////////////////////////////

#include "GraphicsUtility.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <set>
#include <stdexcept>
#include <string>
#include "UIUtility.h"

namespace graphicsFX
{
    //Initializes Vulkan device resources.
    void GraphicsUtility::Init(const Window &window)
    {
        InitInstanceAndSurface(window);
        InitDeviceAndCommandPool(VK_NULL_HANDLE);
    }

    //Initializes the Vulkan instance and window surface.
    void GraphicsUtility::InitInstanceAndSurface(const Window &window)
    {
        CreateInstance();
        SetupDebugMessenger();
        CreateSurface(window);
    }

    //Initializes the Vulkan device and command pool.
    void GraphicsUtility::InitDeviceAndCommandPool(VkPhysicalDevice preferredPhysicalDevicePtrGPU)
    {
        PickPhysicalDevice(preferredPhysicalDevicePtrGPU);
        CreateLogicalDevice();
        CreateCommandPool();
    }

    //Sets extra Vulkan instance extensions.
    void GraphicsUtility::SetExtraInstanceExtensions(const std::vector<std::string> &extensions)
    {
        m_extraInstanceExtensions = extensions;
    }

    //Sets extra Vulkan device extensions.
    void GraphicsUtility::SetExtraDeviceExtensions(const std::vector<std::string> &extensions)
    {
        m_extraDeviceExtensions = extensions;
    }

    //Sets the minimum Vulkan API version.
    void GraphicsUtility::SetMinimumApiVersion(uint32_t apiVersion)
    {
        m_minimumApiVersion = std::max(apiVersion, VK_API_VERSION_1_0);
    }

    //Releases Vulkan device resources.
    void GraphicsUtility::Clean(void)
    {
        WaitIdle();

        if (m_commandPoolPtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyCommandPool(m_devicePtrGPU, m_commandPoolPtrGPU, nullptr);
            m_commandPoolPtrGPU = VK_NULL_HANDLE;
        }

        if (m_devicePtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyDevice(m_devicePtrGPU, nullptr);
            m_devicePtrGPU = VK_NULL_HANDLE;
        }

        if (m_validationLayersEnabled && m_debugMessengerPtrGPU != VK_NULL_HANDLE)
        {
            DestroyDebugUtilsMessengerEXT(m_instancePtrGPU, m_debugMessengerPtrGPU, nullptr);
            m_debugMessengerPtrGPU = VK_NULL_HANDLE;
        }

        if (m_surfacePtrGPU != VK_NULL_HANDLE)
        {
            vkDestroySurfaceKHR(m_instancePtrGPU, m_surfacePtrGPU, nullptr);
            m_surfacePtrGPU = VK_NULL_HANDLE;
        }

        if (m_instancePtrGPU != VK_NULL_HANDLE)
        {
            vkDestroyInstance(m_instancePtrGPU, nullptr);
            m_instancePtrGPU = VK_NULL_HANDLE;
        }

        m_physicalDevicePtrGPU = VK_NULL_HANDLE;
        m_graphicsQueuePtrGPU = VK_NULL_HANDLE;
        m_presentQueuePtrGPU = VK_NULL_HANDLE;
        m_validationLayersEnabled = false;
        m_wideLinesEnabled = false;
        m_anisotropyEnabled = false;
        m_wireframeEnabled = false;
        m_largePointsEnabled = false;
        m_minimumLineWidth = 1.0f;
        m_maximumLineWidth = 1.0f;
        m_minimumPointSize = 1.0f;
        m_maximumPointSize = 1.0f;
        m_maximumAnisotropy = 1.0f;
    }

    //Waits for the logical device to become idle.
    void GraphicsUtility::WaitIdle(void) const
    {
        if (m_devicePtrGPU != VK_NULL_HANDLE)
        {
            vkDeviceWaitIdle(m_devicePtrGPU);
        }
    }

    //Chooses a supported multisample count.
    VkSampleCountFlagBits GraphicsUtility::ChooseSampleCount(bool antiAliasingEnabled, int requestedSamples) const
    {
        if (!antiAliasingEnabled || requestedSamples <= 1)
        {
            OutputDebugStringA("Vulkan MSAA sample count: 1x\n");
            return VK_SAMPLE_COUNT_1_BIT;
        }

        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(m_physicalDevicePtrGPU, &properties);

        const VkSampleCountFlags counts = properties.limits.framebufferColorSampleCounts & properties.limits.framebufferDepthSampleCounts;
        const VkSampleCountFlagBits requestedSampleCount = SampleCountFromInt(requestedSamples);

        constexpr std::array<VkSampleCountFlagBits, 7> choices = {
            VK_SAMPLE_COUNT_64_BIT, VK_SAMPLE_COUNT_32_BIT, VK_SAMPLE_COUNT_16_BIT, VK_SAMPLE_COUNT_8_BIT, VK_SAMPLE_COUNT_4_BIT, VK_SAMPLE_COUNT_2_BIT, VK_SAMPLE_COUNT_1_BIT
        };

        VkSampleCountFlagBits selectedSampleCount = VK_SAMPLE_COUNT_1_BIT;
        for (VkSampleCountFlagBits choice : choices)
        {
            if (SampleCountToInt(choice) <= SampleCountToInt(requestedSampleCount) && (counts & choice) != 0)
            {
                selectedSampleCount = choice;
                break;
            }
        }

        const std::string message = "Vulkan MSAA sample count: " + std::to_string(SampleCountToInt(selectedSampleCount)) + "x\n";
        OutputDebugStringA(message.c_str());
        return selectedSampleCount;
    }

    //Finds graphics and present queue families.
    GraphicsUtility::QueueFamilyIndices GraphicsUtility::FindQueueFamilies(VkPhysicalDevice devicePtrGPU) const
    {
        QueueFamilyIndices indices;

        uint32_t queueFamilyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(devicePtrGPU, &queueFamilyCount, nullptr);

        std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
        vkGetPhysicalDeviceQueueFamilyProperties(devicePtrGPU, &queueFamilyCount, queueFamilies.data());

        for (uint32_t i = 0; i < queueFamilyCount; ++i)
        {
            if (queueFamilies[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)
            {
                indices.m_graphicsFamily = i;
            }

            VkBool32 presentSupport = VK_FALSE;
            vkGetPhysicalDeviceSurfaceSupportKHR(devicePtrGPU, i, m_surfacePtrGPU, &presentSupport);

            if (presentSupport)
            {
                indices.m_presentFamily = i;
            }

            if (indices.Complete())
            {
                break;
            }
        }

        return indices;
    }

    //Queries swapchain support details.
    GraphicsUtility::SwapChainSupportDetails GraphicsUtility::QuerySwapChainSupport(VkPhysicalDevice devicePtrGPU) const
    {
        SwapChainSupportDetails details;

        vkGetPhysicalDeviceSurfaceCapabilitiesKHR(devicePtrGPU, m_surfacePtrGPU, &details.m_capabilities);

        uint32_t formatCount = 0;
        vkGetPhysicalDeviceSurfaceFormatsKHR(devicePtrGPU, m_surfacePtrGPU, &formatCount, nullptr);

        if (formatCount != 0)
        {
            details.m_formats.resize(formatCount);
            vkGetPhysicalDeviceSurfaceFormatsKHR(devicePtrGPU, m_surfacePtrGPU, &formatCount, details.m_formats.data());
        }

        uint32_t presentModeCount = 0;
        vkGetPhysicalDeviceSurfacePresentModesKHR(devicePtrGPU, m_surfacePtrGPU, &presentModeCount, nullptr);

        if (presentModeCount != 0)
        {
            details.m_presentModes.resize(presentModeCount);
            vkGetPhysicalDeviceSurfacePresentModesKHR(devicePtrGPU, m_surfacePtrGPU, &presentModeCount, details.m_presentModes.data());
        }

        return details;
    }

    //Finds the depth buffer format.
    VkFormat GraphicsUtility::FindDepthFormat(void) const
    {
        return FindSupportedFormat( { VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT }, VK_IMAGE_TILING_OPTIMAL, VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT);
    }

    //Finds a supported image format.
    VkFormat GraphicsUtility::FindSupportedFormat(const std::vector<VkFormat> &candidates, VkImageTiling tiling, VkFormatFeatureFlags features) const
    {
        for (VkFormat format : candidates)
        {
            VkFormatProperties properties{};
            vkGetPhysicalDeviceFormatProperties(m_physicalDevicePtrGPU, format, &properties);

            if (tiling == VK_IMAGE_TILING_LINEAR && (properties.linearTilingFeatures & features) == features)
            {
                return format;
            }

            if (tiling == VK_IMAGE_TILING_OPTIMAL && (properties.optimalTilingFeatures & features) == features)
            {
                return format;
            }
        }

        throw std::runtime_error("Failed to find a supported Vulkan image format.");
    }

    //Creates a Vulkan buffer and memory.
    void GraphicsUtility::CreateBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer &bufferPtrGPU, VkDeviceMemory &bufferMemoryPtrGPU) const
    {
        VkBufferCreateInfo bufferInfo{};
        bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferInfo.size = size;
        bufferInfo.usage = usage;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        if (vkCreateBuffer(m_devicePtrGPU, &bufferInfo, nullptr, &bufferPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create Vulkan buffer.");
        }

        VkMemoryRequirements memoryRequirements{};
        vkGetBufferMemoryRequirements(m_devicePtrGPU, bufferPtrGPU, &memoryRequirements);

        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memoryRequirements.size;
        allocInfo.memoryTypeIndex = FindMemoryType(memoryRequirements.memoryTypeBits, properties);

        if (vkAllocateMemory(m_devicePtrGPU, &allocInfo, nullptr, &bufferMemoryPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to allocate Vulkan buffer memory.");
        }

        vkBindBufferMemory(m_devicePtrGPU, bufferPtrGPU, bufferMemoryPtrGPU, 0);
    }

    //Creates a Vulkan image and memory.
    void GraphicsUtility::CreateImage(
        uint32_t width,
        uint32_t height,
        VkFormat format,
        VkImageTiling tiling,
        VkImageUsageFlags usage,
        VkMemoryPropertyFlags properties,
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
        imageInfo.tiling = tiling;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        imageInfo.usage = usage;
        imageInfo.samples = sampleCount;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        if (vkCreateImage(m_devicePtrGPU, &imageInfo, nullptr, &imagePtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create Vulkan image.");
        }

        VkMemoryRequirements memoryRequirements{};
        vkGetImageMemoryRequirements(m_devicePtrGPU, imagePtrGPU, &memoryRequirements);

        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memoryRequirements.size;
        allocInfo.memoryTypeIndex = FindMemoryType(memoryRequirements.memoryTypeBits, properties);

        if (vkAllocateMemory(m_devicePtrGPU, &allocInfo, nullptr, &imageMemoryPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to allocate Vulkan image memory.");
        }

        vkBindImageMemory(m_devicePtrGPU, imagePtrGPU, imageMemoryPtrGPU, 0);
    }

    //Creates a Vulkan image view.
    VkImageView GraphicsUtility::CreateImageView(VkImage imagePtrGPU, VkFormat format, VkImageAspectFlags aspectFlags) const
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
            throw std::runtime_error("Failed to create Vulkan image view.");
        }

        return imageViewPtrGPU;
    }

    //Finds a matching memory type.
    uint32_t GraphicsUtility::FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const
    {
        VkPhysicalDeviceMemoryProperties memoryProperties{};
        vkGetPhysicalDeviceMemoryProperties(m_physicalDevicePtrGPU, &memoryProperties);

        for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; ++i)
        {
            if ((typeFilter & (1U << i)) && (memoryProperties.memoryTypes[i].propertyFlags & properties) == properties)
            {
                return i;
            }
        }

        throw std::runtime_error("Failed to find a suitable Vulkan memory type.");
    }

    //Creates the Vulkan instance.
    void GraphicsUtility::CreateInstance(void)
    {
        m_validationLayersEnabled = s_enableValidationLayers && ValidationLayersSupported();

        if (s_enableValidationLayers && !m_validationLayersEnabled)
        {
            OutputDebugStringA("Vulkan validation layers are not available. Continuing without validation.\n");
        }

        VkApplicationInfo appInfo{};
        appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
        appInfo.pApplicationName = "GraphicsApp";
        appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
        appInfo.pEngineName = "No Engine";
        appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
        appInfo.apiVersion = m_minimumApiVersion;

        const auto extensions = RequiredExtensions();

        VkInstanceCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
        createInfo.pApplicationInfo = &appInfo;
        createInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
        createInfo.ppEnabledExtensionNames = extensions.data();

        VkDebugUtilsMessengerCreateInfoEXT debugCreateInfo{};
        if (m_validationLayersEnabled)
        {
            createInfo.enabledLayerCount = static_cast<uint32_t>(s_validationLayers.size());
            createInfo.ppEnabledLayerNames = s_validationLayers.data();
            PopulateDebugMessengerCreateInfo(debugCreateInfo);
            createInfo.pNext = &debugCreateInfo;
        }

        if (vkCreateInstance(&createInfo, nullptr, &m_instancePtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create Vulkan instance.");
        }
    }

    //Creates the debug messenger.
    void GraphicsUtility::SetupDebugMessenger(void)
    {
        if (!m_validationLayersEnabled)
        {
            return;
        }

        const auto createInfo = DebugMessengerCreateInfo();
        if (CreateDebugUtilsMessengerEXT(m_instancePtrGPU, &createInfo, nullptr, &m_debugMessengerPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to set up Vulkan debug messenger.");
        }
    }

    //Creates the Win32 Vulkan surface.
    void GraphicsUtility::CreateSurface(const Window &window)
    {
        VkWin32SurfaceCreateInfoKHR createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR;
        createInfo.hinstance = window.Instance();
        createInfo.hwnd = window.Hwnd();

        if (vkCreateWin32SurfaceKHR(m_instancePtrGPU, &createInfo, nullptr, &m_surfacePtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create Win32 Vulkan surface.");
        }
    }

    //Chooses the physical device.
    void GraphicsUtility::PickPhysicalDevice(VkPhysicalDevice preferredPhysicalDevicePtrGPU)
    {
        uint32_t deviceCount = 0;
        vkEnumeratePhysicalDevices(m_instancePtrGPU, &deviceCount, nullptr);

        if (deviceCount == 0)
        {
            throw std::runtime_error("No GPUs with Vulkan support were found.");
        }

        std::vector<VkPhysicalDevice> devicesPtrGPU(deviceCount);
        vkEnumeratePhysicalDevices(m_instancePtrGPU, &deviceCount, devicesPtrGPU.data());

        if (preferredPhysicalDevicePtrGPU != VK_NULL_HANDLE)
        {
            if (IsDeviceSuitable(preferredPhysicalDevicePtrGPU))
            {
                m_physicalDevicePtrGPU = preferredPhysicalDevicePtrGPU;
                return;
            }

            throw std::runtime_error("OpenXR required Vulkan physical device is not suitable for this app.");
        }

        for (VkPhysicalDevice devicePtrGPU : devicesPtrGPU)
        {
            if (IsDeviceSuitable(devicePtrGPU))
            {
                m_physicalDevicePtrGPU = devicePtrGPU;
                break;
            }
        }

        if (m_physicalDevicePtrGPU == VK_NULL_HANDLE)
        {
            throw std::runtime_error("Failed to find a suitable Vulkan GPU.");
        }
    }

    //Creates the logical device and queues.
    void GraphicsUtility::CreateLogicalDevice(void)
    {
        const QueueFamilyIndices indices = FindQueueFamilies(m_physicalDevicePtrGPU);
        const std::set<uint32_t> uniqueQueueFamilies = {
            indices.m_graphicsFamily.value(), indices.m_presentFamily.value()
        };

        std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
        const float queuePriority = 1.0f;

        for (uint32_t queueFamily : uniqueQueueFamilies)
        {
            VkDeviceQueueCreateInfo queueCreateInfo{};
            queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
            queueCreateInfo.queueFamilyIndex = queueFamily;
            queueCreateInfo.queueCount = 1;
            queueCreateInfo.pQueuePriorities = &queuePriority;
            queueCreateInfos.push_back(queueCreateInfo);
        }

        VkPhysicalDeviceFeatures supportedFeatures{};
        vkGetPhysicalDeviceFeatures(m_physicalDevicePtrGPU, &supportedFeatures);

        VkPhysicalDeviceProperties deviceProperties{};
        vkGetPhysicalDeviceProperties(m_physicalDevicePtrGPU, &deviceProperties);

        VkPhysicalDeviceFeatures deviceFeatures{};
        deviceFeatures.wideLines = supportedFeatures.wideLines;
        deviceFeatures.samplerAnisotropy = supportedFeatures.samplerAnisotropy;
        deviceFeatures.fillModeNonSolid = supportedFeatures.fillModeNonSolid;
        deviceFeatures.largePoints = supportedFeatures.largePoints;
        m_wideLinesEnabled = supportedFeatures.wideLines == VK_TRUE;
        m_anisotropyEnabled = supportedFeatures.samplerAnisotropy == VK_TRUE;
        m_wireframeEnabled = supportedFeatures.fillModeNonSolid == VK_TRUE;
        m_largePointsEnabled = supportedFeatures.largePoints == VK_TRUE;

        if (m_wideLinesEnabled)
        {
            m_minimumLineWidth = deviceProperties.limits.lineWidthRange[0];
            m_maximumLineWidth = deviceProperties.limits.lineWidthRange[1];
        }
        else
        {
            m_minimumLineWidth = 1.0f;
            m_maximumLineWidth = 1.0f;
        }

        if (m_anisotropyEnabled)
        {
            m_maximumAnisotropy = std::min(deviceProperties.limits.maxSamplerAnisotropy, 16.0f);
        }
        else
        {
            m_maximumAnisotropy = 1.0f;
        }

        if (m_largePointsEnabled)
        {
            m_minimumPointSize = deviceProperties.limits.pointSizeRange[0];
            m_maximumPointSize = deviceProperties.limits.pointSizeRange[1];
        }
        else
        {
            m_minimumPointSize = 1.0f;
            m_maximumPointSize = 1.0f;
        }

        VkDeviceCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        createInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
        createInfo.pQueueCreateInfos = queueCreateInfos.data();
        createInfo.pEnabledFeatures = &deviceFeatures;
        const std::vector<const char *> deviceExtensions = RequiredDeviceExtensions();
        createInfo.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());
        createInfo.ppEnabledExtensionNames = deviceExtensions.data();

        if (m_validationLayersEnabled)
        {
            createInfo.enabledLayerCount = static_cast<uint32_t>(s_validationLayers.size());
            createInfo.ppEnabledLayerNames = s_validationLayers.data();
        }

        if (vkCreateDevice(m_physicalDevicePtrGPU, &createInfo, nullptr, &m_devicePtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create Vulkan logical device.");
        }

        vkGetDeviceQueue(m_devicePtrGPU, indices.m_graphicsFamily.value(), 0, &m_graphicsQueuePtrGPU);
        vkGetDeviceQueue(m_devicePtrGPU, indices.m_presentFamily.value(), 0, &m_presentQueuePtrGPU);

        // Diagnostic: report whether the selected graphics queue family also supports compute.
        // This is a minimal, non-invasive check for Phase 2A smoke testing.
        uint32_t selectedGraphicsFamily = indices.m_graphicsFamily.value();
        uint32_t queueFamilyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(m_physicalDevicePtrGPU, &queueFamilyCount, nullptr);
        if (queueFamilyCount > 0)
        {
            std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
            vkGetPhysicalDeviceQueueFamilyProperties(m_physicalDevicePtrGPU, &queueFamilyCount, queueFamilies.data());
            if (selectedGraphicsFamily < queueFamilyCount)
            {
                const VkQueueFlags flags = queueFamilies[selectedGraphicsFamily].queueFlags;
                std::string message = "Graphics queue family index: " + std::to_string(selectedGraphicsFamily) + "\n";
                message += (flags & VK_QUEUE_GRAPHICS_BIT) ? "GRAPHICS: yes\n" : "GRAPHICS: no\n";
                message += (flags & VK_QUEUE_COMPUTE_BIT) ? "COMPUTE: yes\n" : "COMPUTE: no\n";
                UIUtility::Print(message);
            }
        }
    }

    //Creates the command pool.
    void GraphicsUtility::CreateCommandPool(void)
    {
        const QueueFamilyIndices queueFamilyIndices = FindQueueFamilies(m_physicalDevicePtrGPU);

        VkCommandPoolCreateInfo poolInfo{};
        poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        poolInfo.queueFamilyIndex = queueFamilyIndices.m_graphicsFamily.value();

        if (vkCreateCommandPool(m_devicePtrGPU, &poolInfo, nullptr, &m_commandPoolPtrGPU) != VK_SUCCESS)
        {
            throw std::runtime_error("Failed to create Vulkan command pool.");
        }
    }

    //Checks whether validation layers are available.
    bool GraphicsUtility::ValidationLayersSupported(void) const
    {
        uint32_t layerCount = 0;
        vkEnumerateInstanceLayerProperties(&layerCount, nullptr);

        std::vector<VkLayerProperties> availableLayers(layerCount);
        vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());

        for (const char *layerNamePtr : s_validationLayers)
        {
            bool layerFound = false;

            for (const auto &layerProperties : availableLayers)
            {
                if (std::strcmp(layerNamePtr, layerProperties.layerName) == 0)
                {
                    layerFound = true;
                    break;
                }
            }

            if (!layerFound)
            {
                return false;
            }
        }

        return true;
    }

    //Returns required instance extensions.
    std::vector<const char *> GraphicsUtility::RequiredExtensions(void) const
    {
        std::vector<const char *> extensions = {
            VK_KHR_SURFACE_EXTENSION_NAME, VK_KHR_WIN32_SURFACE_EXTENSION_NAME
        };

        if (m_validationLayersEnabled)
        {
            AddExtensionIfMissing(extensions, VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        }

        for (const std::string &extension : m_extraInstanceExtensions)
        {
            AddExtensionIfMissing(extensions, extension.c_str());
        }

        return extensions;
    }

    //Returns required device extensions.
    std::vector<const char *> GraphicsUtility::RequiredDeviceExtensions(void) const
    {
        std::vector<const char *> extensions;

        for (const char *extensionNamePtr : s_deviceExtensions)
        {
            AddExtensionIfMissing(extensions, extensionNamePtr);
        }

        for (const std::string &extension : m_extraDeviceExtensions)
        {
            AddExtensionIfMissing(extensions, extension.c_str());
        }

        return extensions;
    }

    //Checks whether a physical device can run the app.
    bool GraphicsUtility::IsDeviceSuitable(VkPhysicalDevice devicePtrGPU) const
    {
        const QueueFamilyIndices indices = FindQueueFamilies(devicePtrGPU);
        const bool extensionsSupported = CheckDeviceExtensionSupport(devicePtrGPU);

        bool swapChainAdequate = false;
        if (extensionsSupported)
        {
            const SwapChainSupportDetails swapChainSupport = QuerySwapChainSupport(devicePtrGPU);
            swapChainAdequate = !swapChainSupport.m_formats.empty() && !swapChainSupport.m_presentModes.empty();
        }

        return indices.Complete() && extensionsSupported && swapChainAdequate;
    }

    //Checks required device extensions.
    bool GraphicsUtility::CheckDeviceExtensionSupport(VkPhysicalDevice devicePtrGPU) const
    {
        uint32_t extensionCount = 0;
        vkEnumerateDeviceExtensionProperties(devicePtrGPU, nullptr, &extensionCount, nullptr);

        std::vector<VkExtensionProperties> availableExtensions(extensionCount);
        vkEnumerateDeviceExtensionProperties(devicePtrGPU, nullptr, &extensionCount, availableExtensions.data());

        const std::vector<const char *> deviceExtensions = RequiredDeviceExtensions();
        std::set<std::string> requiredExtensions(deviceExtensions.begin(), deviceExtensions.end());

        for (const auto &extension : availableExtensions)
        {
            requiredExtensions.erase(extension.extensionName);
        }

        return requiredExtensions.empty();
    }

    //Creates debug messenger settings.
    VkDebugUtilsMessengerCreateInfoEXT GraphicsUtility::DebugMessengerCreateInfo(void) const
    {
        VkDebugUtilsMessengerCreateInfoEXT createInfo{};
        PopulateDebugMessengerCreateInfo(createInfo);
        return createInfo;
    }

    //Fills debug messenger settings.
    void GraphicsUtility::PopulateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT &createInfo) const
    {
        createInfo = {};
        createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
        createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        createInfo.pfnUserCallback = DebugCallback;
    }

    //Handles Vulkan validation output.
    VKAPI_ATTR VkBool32 VKAPI_CALL GraphicsUtility::DebugCallback(
        VkDebugUtilsMessageSeverityFlagBitsEXT severity,
        VkDebugUtilsMessageTypeFlagsEXT,
        const VkDebugUtilsMessengerCallbackDataEXT *callbackDataPtr,
        void *
    )
    {
        if ((severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) != 0)
        {
            OutputDebugStringA("Vulkan validation: ");
            OutputDebugStringA(callbackDataPtr->pMessage);
            OutputDebugStringA("\n");
        }

        return VK_FALSE;
    }

    //Creates a Vulkan debug messenger through the extension entry point.
    VkResult GraphicsUtility::CreateDebugUtilsMessengerEXT(
        VkInstance instancePtrGPU,
        const VkDebugUtilsMessengerCreateInfoEXT *createInfoPtr,
        const VkAllocationCallbacks *allocatorPtr,
        VkDebugUtilsMessengerEXT *debugMessengerPtrGPU
    )
    {
        const auto functionPtr = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(vkGetInstanceProcAddr(instancePtrGPU, "vkCreateDebugUtilsMessengerEXT"));

        if (functionPtr)
        {
            return functionPtr(instancePtrGPU, createInfoPtr, allocatorPtr, debugMessengerPtrGPU);
        }

        return VK_ERROR_EXTENSION_NOT_PRESENT;
    }

    //Destroys a Vulkan debug messenger through the extension entry point.
    void GraphicsUtility::DestroyDebugUtilsMessengerEXT(VkInstance instancePtrGPU, VkDebugUtilsMessengerEXT debugMessengerPtrGPU, const VkAllocationCallbacks *allocatorPtr)
    {
        const auto functionPtr = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(vkGetInstanceProcAddr(instancePtrGPU, "vkDestroyDebugUtilsMessengerEXT"));

        if (functionPtr)
        {
            functionPtr(instancePtrGPU, debugMessengerPtrGPU, allocatorPtr);
        }
    }

    //Adds a Vulkan extension name once.
    void GraphicsUtility::AddExtensionIfMissing(std::vector<const char *> &extensions, const char *extensionNamePtr)
    {
        if (extensionNamePtr == nullptr || extensionNamePtr[0] == '\0')
        {
            return;
        }

        for (const char *currentExtensionNamePtr : extensions)
        {
            if (std::strcmp(currentExtensionNamePtr, extensionNamePtr) == 0)
            {
                return;
            }
        }

        extensions.push_back(extensionNamePtr);
    }

    //Returns the integer value for a Vulkan sample count.
    int GraphicsUtility::SampleCountToInt(VkSampleCountFlagBits sampleCount)
    {
        switch (sampleCount)
        {
            case VK_SAMPLE_COUNT_64_BIT: return 64;
            case VK_SAMPLE_COUNT_32_BIT: return 32;
            case VK_SAMPLE_COUNT_16_BIT: return 16;
            case VK_SAMPLE_COUNT_8_BIT: return 8;
            case VK_SAMPLE_COUNT_4_BIT: return 4;
            case VK_SAMPLE_COUNT_2_BIT: return 2;
            default: return 1;
        }
    }

    //Returns the requested Vulkan sample count.
    VkSampleCountFlagBits GraphicsUtility::SampleCountFromInt(int sampleCount)
    {
        if (sampleCount >= 64)
        {
            return VK_SAMPLE_COUNT_64_BIT;
        }

        if (sampleCount >= 32)
        {
            return VK_SAMPLE_COUNT_32_BIT;
        }

        if (sampleCount >= 16)
        {
            return VK_SAMPLE_COUNT_16_BIT;
        }

        if (sampleCount >= 8)
        {
            return VK_SAMPLE_COUNT_8_BIT;
        }

        if (sampleCount >= 4)
        {
            return VK_SAMPLE_COUNT_4_BIT;
        }

        if (sampleCount >= 2)
        {
            return VK_SAMPLE_COUNT_2_BIT;
        }

        return VK_SAMPLE_COUNT_1_BIT;
    }
}

