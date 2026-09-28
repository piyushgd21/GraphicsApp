///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	RenderTypes.h
//	Description	:
//		Defines shared Vulkan render data structures.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "global.h"

#include "MathUtility.h"

#include <vulkan/vulkan.h>

#include <array>
#include <cstddef>
#include <cstdint>

namespace graphicsFX
{
    //Stores one renderable vertex.
    struct Vertex
    {
        Float3 m_position; //Stores vertex position.
        Float4 m_color; //Stores vertex color.
        Float2 m_texCoord{ 0.0f, 0.0f }; //Stores vertex texture coordinate.
        Float3 m_normal{ 0.0f, 1.0f, 0.0f }; //Stores vertex normal.
        Float4 m_tangent{ 1.0f, 0.0f, 0.0f, 1.0f }; //Stores vertex tangent and handedness.

        //Returns the Vulkan vertex binding layout.
        static VkVertexInputBindingDescription BindingDescription(void)
        {
            VkVertexInputBindingDescription binding{};
            binding.binding = 0;
            binding.stride = static_cast<uint32_t>(sizeof(Vertex));
            binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
            return binding;
        }

        //Returns the Vulkan vertex attribute layout.
        static std::array<VkVertexInputAttributeDescription, 5> AttributeDescriptions(void)
        {
            std::array<VkVertexInputAttributeDescription, 5> attributes{};

            attributes[0].binding = 0;
            attributes[0].location = 0;
            attributes[0].format = VK_FORMAT_R32G32B32_SFLOAT;
            attributes[0].offset = static_cast<uint32_t>(offsetof(Vertex, m_position));

            attributes[1].binding = 0;
            attributes[1].location = 1;
            attributes[1].format = VK_FORMAT_R32G32B32A32_SFLOAT;
            attributes[1].offset = static_cast<uint32_t>(offsetof(Vertex, m_color));

            attributes[2].binding = 0;
            attributes[2].location = 2;
            attributes[2].format = VK_FORMAT_R32G32_SFLOAT;
            attributes[2].offset = static_cast<uint32_t>(offsetof(Vertex, m_texCoord));

            attributes[3].binding = 0;
            attributes[3].location = 3;
            attributes[3].format = VK_FORMAT_R32G32B32_SFLOAT;
            attributes[3].offset = static_cast<uint32_t>(offsetof(Vertex, m_normal));

            attributes[4].binding = 0;
            attributes[4].location = 4;
            attributes[4].format = VK_FORMAT_R32G32B32A32_SFLOAT;
            attributes[4].offset = static_cast<uint32_t>(offsetof(Vertex, m_tangent));

            return attributes;
        }
    };

    //Stores one per-instance transform and color.
    struct SceneObjectInstanceData
    {
        Float4x4 m_model{ identity() }; //Stores the instance model transform.
        Float4 m_color{ 1.0f, 1.0f, 1.0f, 1.0f }; //Stores the instance color.

        //Returns the Vulkan instance binding layout.
        static VkVertexInputBindingDescription BindingDescription(void)
        {
            VkVertexInputBindingDescription binding{};
            binding.binding = 1;
            binding.stride = static_cast<uint32_t>(sizeof(SceneObjectInstanceData));
            binding.inputRate = VK_VERTEX_INPUT_RATE_INSTANCE;
            return binding;
        }

        //Returns the Vulkan instance attribute layout.
        static std::array<VkVertexInputAttributeDescription, 5> AttributeDescriptions(void)
        {
            std::array<VkVertexInputAttributeDescription, 5> attributes{};

            for (uint32_t column = 0; column < 4U; ++column)
            {
                attributes[column].binding = 1;
                attributes[column].location = 5U + column;
                attributes[column].format = VK_FORMAT_R32G32B32A32_SFLOAT;
                attributes[column].offset = static_cast<uint32_t>(offsetof(SceneObjectInstanceData, m_model)) + static_cast<uint32_t>(sizeof(Float4) * column);
            }

            attributes[4].binding = 1;
            attributes[4].location = 9;
            attributes[4].format = VK_FORMAT_R32G32B32A32_SFLOAT;
            attributes[4].offset = static_cast<uint32_t>(offsetof(SceneObjectInstanceData, m_color));

            return attributes;
        }
    };

    //Stores frame values for shaders.
    struct CameraBuffer
    {
        Float4x4 m_view; //Stores the camera view matrix.
        Float4x4 m_projection; //Stores the camera projection matrix.
        alignas(16) Float4 m_cameraPosition; //Stores the camera world position.
        alignas(16) Float4 m_lightDirection; //Stores the directional light pointing direction.
        alignas(16) Float4 m_ambientColor; //Stores ambient light color.
        alignas(16) Float4 m_diffuseColor; //Stores diffuse light color.
        alignas(16) Float4 m_specularColor; //Stores specular light color.
        alignas(16) Float4 m_lightSettings; //Stores specular power and reserved lighting values.
        Float4x4 m_lightViewProjection; //Stores the light view projection matrix.
        alignas(16) Float4 m_shadowSettings; //Stores shadow enabled, bias, normal bias, and softness.
        Float4x4 m_xrEnvironmentDepthView; //Stores the XR environment depth view matrix.
        Float4x4 m_xrEnvironmentDepthViewProjection; //Stores the XR environment depth view projection matrix.
        alignas(16) Float4 m_xrDepthSettings; //Stores XR depth enabled, bias, max depth, and debug mode.
        alignas(16) Float4 m_xrDepthExtent; //Stores XR depth width, height, near, and far values.
        alignas(16) Float4 m_xrDepthModeSettings; //Stores XR depth value mode, alpha debug, and reserved values.
    };

    //Stores directions used for camera input movement.
    struct CameraMovementBasis
    {
        Float3 m_forward{ 0.0f, 0.0f, -1.0f }; //Stores movement forward direction.
        Float3 m_right{ 1.0f, 0.0f, 0.0f }; //Stores movement right direction.
        Float3 m_up{ 0.0f, 1.0f, 0.0f }; //Stores movement up direction.
    };

    //Stores controller movement input values.
    struct ControllerMovementState
    {
        Float2 m_move{ 0.0f, 0.0f }; //Stores strafe and forward movement input.
        Float2 m_turn{ 0.0f, 0.0f }; //Stores yaw and pitch input.
        bool m_moveUp{ false }; //Stores whether upward movement is pressed.
        bool m_moveDown{ false }; //Stores whether downward movement is pressed.
        bool m_active{ false }; //Stores whether controller input has usable values.
    };

    //Stores per-object shader constants.
    struct PushConstants
    {
        Float4x4 m_model; //Stores the object model matrix.
        alignas(16) Float4 m_color; //Stores the object color.
        alignas(16) Float4 m_objectSettings; //Stores receive shadow and reserved object settings.
    };
}

