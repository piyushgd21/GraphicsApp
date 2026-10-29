#pragma once

#include <vulkan/vulkan.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

struct PlaneSweepGpuTestContext final
{
    VkInstance m_instance{ VK_NULL_HANDLE };
    VkPhysicalDevice m_physicalDevice{ VK_NULL_HANDLE };
    VkDevice m_device{ VK_NULL_HANDLE };
    VkQueue m_queue{ VK_NULL_HANDLE };
    VkCommandPool m_commandPool{ VK_NULL_HANDLE };
    bool m_shaderFloat64Enabled{ false };

    PlaneSweepGpuTestContext() = default;
    PlaneSweepGpuTestContext(const PlaneSweepGpuTestContext &) = delete;
    PlaneSweepGpuTestContext &operator=(const PlaneSweepGpuTestContext &) = delete;

    ~PlaneSweepGpuTestContext()
    {
        if (m_device != VK_NULL_HANDLE)
        {
            vkDeviceWaitIdle(m_device);
            if (m_commandPool != VK_NULL_HANDLE)
            {
                vkDestroyCommandPool(m_device, m_commandPool, nullptr);
            }
            vkDestroyDevice(m_device, nullptr);
        }
        if (m_instance != VK_NULL_HANDLE)
        {
            vkDestroyInstance(m_instance, nullptr);
        }
    }

    static void Check(VkResult result, const char *operation)
    {
        if (result != VK_SUCCESS)
        {
            throw std::runtime_error(
                std::string(operation) + " failed: " + std::to_string(result));
        }
    }

    void Initialize()
    {
        if (m_instance != VK_NULL_HANDLE)
        {
            throw std::runtime_error("GPU test context already initialized.");
        }

        VkApplicationInfo application{};
        application.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
        application.pApplicationName = "PlaneSweep GPU regression tests";
        application.apiVersion = VK_API_VERSION_1_0;

        VkInstanceCreateInfo instanceInfo{};
        instanceInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
        instanceInfo.pApplicationInfo = &application;
        Check(vkCreateInstance(&instanceInfo, nullptr, &m_instance),
            "vkCreateInstance");

        uint32_t deviceCount = 0;
        Check(vkEnumeratePhysicalDevices(m_instance, &deviceCount, nullptr),
            "vkEnumeratePhysicalDevices");
        std::vector<VkPhysicalDevice> devices(deviceCount);
        if (deviceCount > 0)
        {
            Check(vkEnumeratePhysicalDevices(m_instance, &deviceCount, devices.data()),
                "vkEnumeratePhysicalDevices");
        }

        uint32_t queueFamily = UINT32_MAX;
        for (VkPhysicalDevice candidate : devices)
        {
            uint32_t familyCount = 0;
            vkGetPhysicalDeviceQueueFamilyProperties(candidate, &familyCount, nullptr);
            std::vector<VkQueueFamilyProperties> families(familyCount);
            vkGetPhysicalDeviceQueueFamilyProperties(
                candidate, &familyCount, families.data());

            for (uint32_t index = 0; index < familyCount; ++index)
            {
                const VkQueueFlags required =
                    VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT;
                if (families[index].queueCount > 0 &&
                    (families[index].queueFlags & required) == required)
                {
                    m_physicalDevice = candidate;
                    queueFamily = index;
                    break;
                }
            }
            if (m_physicalDevice != VK_NULL_HANDLE)
            {
                break;
            }
        }

        if (m_physicalDevice == VK_NULL_HANDLE)
        {
            throw std::runtime_error("No Vulkan graphics/compute device available.");
        }

        const float priority = 1.0f;
        VkDeviceQueueCreateInfo queueInfo{};
        queueInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queueInfo.queueFamilyIndex = queueFamily;
        queueInfo.queueCount = 1;
        queueInfo.pQueuePriorities = &priority;

        VkDeviceCreateInfo deviceInfo{};
        deviceInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        deviceInfo.queueCreateInfoCount = 1;
        deviceInfo.pQueueCreateInfos = &queueInfo;
        VkPhysicalDeviceFeatures supportedFeatures{};
        vkGetPhysicalDeviceFeatures(m_physicalDevice, &supportedFeatures);
        VkPhysicalDeviceFeatures enabledFeatures{};
        enabledFeatures.shaderFloat64 = supportedFeatures.shaderFloat64;
        deviceInfo.pEnabledFeatures = &enabledFeatures;
        Check(vkCreateDevice(m_physicalDevice, &deviceInfo, nullptr, &m_device),
            "vkCreateDevice");
        m_shaderFloat64Enabled = enabledFeatures.shaderFloat64 == VK_TRUE;
        vkGetDeviceQueue(m_device, queueFamily, 0, &m_queue);

        VkCommandPoolCreateInfo poolInfo{};
        poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        poolInfo.queueFamilyIndex = queueFamily;
        Check(vkCreateCommandPool(m_device, &poolInfo, nullptr, &m_commandPool),
            "vkCreateCommandPool");
    }
};
