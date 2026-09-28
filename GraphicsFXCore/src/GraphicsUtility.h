///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	GraphicsUtility.h
//	Description	:
//		Defines Vulkan device initialization and GPU resource helpers.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "global.h"

#include "Window.h"

#include <vulkan/vulkan.h>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace graphicsFX
{
    //Defines Vulkan device initialization and GPU resource helpers.
    class GraphicsUtility
    {

        public:

            //Stores queue family indices for one physical device.
            struct QueueFamilyIndices
            {
                std::optional<uint32_t> m_graphicsFamily; //Stores the graphics queue family index.
                std::optional<uint32_t> m_presentFamily; //Stores the present queue family index.

                //Returns whether all queue families were found.
                bool Complete(void) const
                {
                    return m_graphicsFamily.has_value() && m_presentFamily.has_value();
                }
            };

            //Stores swapchain capabilities for one physical device.
            struct SwapChainSupportDetails
            {
                VkSurfaceCapabilitiesKHR m_capabilities{}; //Stores surface capabilities.
                std::vector<VkSurfaceFormatKHR> m_formats; //Stores supported surface formats.
                std::vector<VkPresentModeKHR> m_presentModes; //Stores supported present modes.
            };

            //Creates the graphics utility.
            GraphicsUtility(void) = default;

            //Destroys the graphics utility.
            ~GraphicsUtility(void) = default;

            GraphicsUtility(const GraphicsUtility &) = delete;
            GraphicsUtility &operator=(const GraphicsUtility &) = delete;

            //Initializes Vulkan device resources.
            void Init(const Window &window);

            //Initializes the Vulkan instance and window surface.
            void InitInstanceAndSurface(const Window &window);

            //Initializes the Vulkan device and command pool.
            void InitDeviceAndCommandPool(VkPhysicalDevice preferredPhysicalDevicePtrGPU);

            //Sets extra Vulkan instance extensions.
            void SetExtraInstanceExtensions(const std::vector<std::string> &extensions);

            //Sets extra Vulkan device extensions.
            void SetExtraDeviceExtensions(const std::vector<std::string> &extensions);

            //Sets the minimum Vulkan API version.
            void SetMinimumApiVersion(uint32_t apiVersion);

            //Releases Vulkan device resources.
            void Clean(void);

            //Waits for the logical device to become idle.
            void WaitIdle(void) const;

            //Returns whether the device is initialized.
            bool DeviceReady(void) const { return m_devicePtrGPU != VK_NULL_HANDLE; }

            //Returns the Vulkan instance.
            VkInstance Instance(void) const { return m_instancePtrGPU; }

            //Returns the Vulkan surface.
            VkSurfaceKHR Surface(void) const { return m_surfacePtrGPU; }

            //Returns the selected physical device.
            VkPhysicalDevice PhysicalDevice(void) const { return m_physicalDevicePtrGPU; }

            //Returns the logical device.
            VkDevice Device(void) const { return m_devicePtrGPU; }

            //Returns the graphics queue.
            VkQueue GraphicsQueue(void) const { return m_graphicsQueuePtrGPU; }

            //Returns the present queue.
            VkQueue PresentQueue(void) const { return m_presentQueuePtrGPU; }

            //Returns the command pool.
            VkCommandPool CommandPool(void) const { return m_commandPoolPtrGPU; }

            //Returns whether wide Vulkan lines are enabled.
            bool WideLinesEnabled(void) const { return m_wideLinesEnabled; }

            //Returns the minimum supported line width.
            float MinimumLineWidth(void) const { return m_minimumLineWidth; }

            //Returns the maximum supported line width.
            float MaximumLineWidth(void) const { return m_maximumLineWidth; }

            //Returns whether texture anisotropy is enabled.
            bool AnisotropyEnabled(void) const { return m_anisotropyEnabled; }

            //Returns whether wireframe polygon mode is enabled.
            bool WireframeEnabled(void) const { return m_wireframeEnabled; }

            //Returns whether large point sprites are enabled.
            bool LargePointsEnabled(void) const { return m_largePointsEnabled; }

            //Returns the maximum supported texture anisotropy.
            float MaximumAnisotropy(void) const { return m_maximumAnisotropy; }

            //Returns the minimum supported point size.
            float MinimumPointSize(void) const { return m_minimumPointSize; }

            //Returns the maximum supported point size.
            float MaximumPointSize(void) const { return m_maximumPointSize; }

            //Chooses a supported multisample count.
            VkSampleCountFlagBits ChooseSampleCount(bool antiAliasingEnabled, int requestedSamples) const;

            //Finds graphics and present queue families.
            QueueFamilyIndices FindQueueFamilies(VkPhysicalDevice devicePtrGPU) const;

            //Queries swapchain support details.
            SwapChainSupportDetails QuerySwapChainSupport(VkPhysicalDevice devicePtrGPU) const;

            //Finds the depth buffer format.
            VkFormat FindDepthFormat(void) const;

            //Finds a supported image format.
            VkFormat FindSupportedFormat(const std::vector<VkFormat> &candidates, VkImageTiling tiling, VkFormatFeatureFlags features) const;

            //Creates a Vulkan buffer and memory.
            void CreateBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer &bufferPtrGPU, VkDeviceMemory &bufferMemoryPtrGPU) const;

            //Creates a Vulkan image and memory.
            void CreateImage(
                uint32_t width,
                uint32_t height,
                VkFormat format,
                VkImageTiling tiling,
                VkImageUsageFlags usage,
                VkMemoryPropertyFlags properties,
                VkSampleCountFlagBits sampleCount,
                VkImage &imagePtrGPU,
                VkDeviceMemory &imageMemoryPtrGPU
            ) const;

            //Creates a Vulkan image view.
            VkImageView CreateImageView(VkImage imagePtrGPU, VkFormat format, VkImageAspectFlags aspectFlags) const;

            //Finds a matching memory type.
            uint32_t FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const;

        private:

            //Creates the Vulkan instance.
            void CreateInstance(void);

            //Creates the debug messenger.
            void SetupDebugMessenger(void);

            //Creates the Win32 Vulkan surface.
            void CreateSurface(const Window &window);

            //Chooses the physical device.
            void PickPhysicalDevice(VkPhysicalDevice preferredPhysicalDevicePtrGPU);

            //Creates the logical device and queues.
            void CreateLogicalDevice(void);

            //Creates the command pool.
            void CreateCommandPool(void);

            //Checks whether validation layers are available.
            bool ValidationLayersSupported(void) const;

            //Returns required instance extensions.
            std::vector<const char *> RequiredExtensions(void) const;

            //Returns required device extensions.
            std::vector<const char *> RequiredDeviceExtensions(void) const;

            //Checks whether a physical device can run the app.
            bool IsDeviceSuitable(VkPhysicalDevice devicePtrGPU) const;

            //Checks required device extensions.
            bool CheckDeviceExtensionSupport(VkPhysicalDevice devicePtrGPU) const;

            //Creates debug messenger settings.
            VkDebugUtilsMessengerCreateInfoEXT DebugMessengerCreateInfo(void) const;

            //Fills debug messenger settings.
            void PopulateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT &createInfo) const;

            //Handles Vulkan validation output.
            static VKAPI_ATTR VkBool32 VKAPI_CALL DebugCallback(
                VkDebugUtilsMessageSeverityFlagBitsEXT severity,
                VkDebugUtilsMessageTypeFlagsEXT messageType,
                const VkDebugUtilsMessengerCallbackDataEXT *callbackDataPtr,
                void *userDataPtr
            );

            //Creates a Vulkan debug messenger through the extension entry point.
            static VkResult CreateDebugUtilsMessengerEXT(
                VkInstance instancePtrGPU,
                const VkDebugUtilsMessengerCreateInfoEXT *createInfoPtr,
                const VkAllocationCallbacks *allocatorPtr,
                VkDebugUtilsMessengerEXT *debugMessengerPtrGPU
            );

            //Destroys a Vulkan debug messenger through the extension entry point.
            static void DestroyDebugUtilsMessengerEXT(VkInstance instancePtrGPU, VkDebugUtilsMessengerEXT debugMessengerPtrGPU, const VkAllocationCallbacks *allocatorPtr);

            //Adds a Vulkan extension name once.
            static void AddExtensionIfMissing(std::vector<const char *> &extensions, const char *extensionNamePtr);

            //Returns the integer value for a Vulkan sample count.
            static int SampleCountToInt(VkSampleCountFlagBits sampleCount);

            //Returns the requested Vulkan sample count.
            static VkSampleCountFlagBits SampleCountFromInt(int sampleCount);

            static constexpr std::array<const char *, 1> s_validationLayers{ "VK_LAYER_KHRONOS_validation" }; //Stores requested validation layer names.
            static constexpr std::array<const char *, 1> s_deviceExtensions{ VK_KHR_SWAPCHAIN_EXTENSION_NAME }; //Stores required Vulkan device extension names.

#ifdef NDEBUG
            static constexpr bool s_enableValidationLayers = false; //Stores whether validation layers should be requested.
#else
            static constexpr bool s_enableValidationLayers = true; //Stores whether validation layers should be requested.
#endif

            VkInstance m_instancePtrGPU{ VK_NULL_HANDLE }; //Stores the Vulkan instance.
            VkDebugUtilsMessengerEXT m_debugMessengerPtrGPU{ VK_NULL_HANDLE }; //Stores the debug messenger.
            VkSurfaceKHR m_surfacePtrGPU{ VK_NULL_HANDLE }; //Stores the window surface.
            VkPhysicalDevice m_physicalDevicePtrGPU{ VK_NULL_HANDLE }; //Stores the selected physical device.
            VkDevice m_devicePtrGPU{ VK_NULL_HANDLE }; //Stores the logical device.
            VkQueue m_graphicsQueuePtrGPU{ VK_NULL_HANDLE }; //Stores the graphics queue.
            VkQueue m_presentQueuePtrGPU{ VK_NULL_HANDLE }; //Stores the present queue.
            VkCommandPool m_commandPoolPtrGPU{ VK_NULL_HANDLE }; //Stores the command pool.
            std::vector<std::string> m_extraInstanceExtensions; //Stores extra Vulkan instance extensions.
            std::vector<std::string> m_extraDeviceExtensions; //Stores extra Vulkan device extensions.
            uint32_t m_minimumApiVersion{ VK_API_VERSION_1_0 }; //Stores the minimum Vulkan API version.
            bool m_validationLayersEnabled{ false }; //Stores whether validation is enabled.
            bool m_wideLinesEnabled{ false }; //Stores whether wide lines are enabled.
            bool m_anisotropyEnabled{ false }; //Stores whether sampler anisotropy is enabled.
            bool m_wireframeEnabled{ false }; //Stores whether wireframe polygon mode is enabled.
            bool m_largePointsEnabled{ false }; //Stores whether large point sprites are enabled.
            float m_minimumLineWidth{ 1.0f }; //Stores the minimum supported line width.
            float m_maximumLineWidth{ 1.0f }; //Stores the maximum supported line width.
            float m_minimumPointSize{ 1.0f }; //Stores the minimum supported point size.
            float m_maximumPointSize{ 1.0f }; //Stores the maximum supported point size.
            float m_maximumAnisotropy{ 1.0f }; //Stores the maximum supported sampler anisotropy.
    };
}

