///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	SceneObjectSphere.h
//	Description	:
//		Defines the scene object used to draw colored spheres.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "SceneObject.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace graphicsFX
{
    //Defines the scene object used to draw colored spheres.
    class SceneObjectSphere final : public SceneObject
    {

        public:

            SceneObjectSphere(void) = default;

            //Creates sphere mesh data.
            void Init(
                const Float3 &position,
                float radius,
                uint32_t latitudeSegments,
                uint32_t longitudeSegments,
                const Float4 &color,
                const SceneObjectShaderFiles &shaderFiles,
                const std::string &textureFileName = "",
                const std::string &normalMapFileName = "",
                bool useLighting = true,
                SceneObjectRenderOptions renderOptions = {}
            )
            {
                if (radius <= 0.0f || latitudeSegments < 2U || longitudeSegments < 3U)
                {
                    throw std::runtime_error("Sphere radius and segment counts must be positive.");
                }

                SetShaderFileNames(shaderFiles.m_vertexShader, shaderFiles.m_fragmentShader);
                SetTextureFileName(textureFileName);
                SetNormalMapFileName(normalMapFileName);

                SphereMesh mesh = CreateMesh(radius, latitudeSegments, longitudeSegments, textureFileName, useLighting);

                SetMesh(std::move(mesh.m_vertices), std::move(mesh.m_indices), VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, true, 1.0f, position, identity(), 1.0f, color, renderOptions);
            }

            //Creates instanced sphere mesh data.
            void Init(
                std::vector<SceneObjectInstanceData> instances,
                uint32_t latitudeSegments,
                uint32_t longitudeSegments,
                const SceneObjectShaderFiles &shaderFiles,
                const std::string &textureFileName = "",
                const std::string &normalMapFileName = "",
                bool useLighting = true,
                SceneObjectRenderOptions renderOptions = {}
            )
            {
                SetShaderFileNames(shaderFiles.m_vertexShader, shaderFiles.m_fragmentShader);
                SetTextureFileName(textureFileName);
                SetNormalMapFileName(normalMapFileName);

                SphereMesh mesh = CreateMesh(1.0f, latitudeSegments, longitudeSegments, textureFileName, useLighting);

                SetMesh(std::move(mesh.m_vertices), std::move(mesh.m_indices), VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, true, 1.0f, { 0.0f, 0.0f, 0.0f }, identity(), 1.0f, { 1.0f, 1.0f, 1.0f, 1.0f }, renderOptions);
                SetInstances(std::move(instances));
            }

        private:

            //Stores generated sphere mesh data.
            struct SphereMesh
            {
                std::vector<Vertex> m_vertices; //Stores generated sphere vertices.
                std::vector<uint32_t> m_indices; //Stores generated sphere indices.
            };

            //Creates sphere vertex and index data.
            static SphereMesh CreateMesh(float radius, uint32_t latitudeSegments, uint32_t longitudeSegments, const std::string &textureFileName, bool useLighting)
            {
                if (radius <= 0.0f || latitudeSegments < 2U || longitudeSegments < 3U)
                {
                    throw std::runtime_error("Sphere radius and segment counts must be positive.");
                }

                SphereMesh mesh;
                mesh.m_vertices.reserve(static_cast<std::size_t>(latitudeSegments + 1U) * static_cast<std::size_t>(longitudeSegments + 1U));
                mesh.m_indices.reserve(static_cast<std::size_t>(latitudeSegments) * static_cast<std::size_t>(longitudeSegments) * 6U);

                for (uint32_t latitude = 0; latitude <= latitudeSegments; ++latitude)
                {
                    const float v = static_cast<float>(latitude) / static_cast<float>(latitudeSegments);
                    const float theta = v * Pi;
                    const float y = std::cos(theta) * radius;
                    const float ringRadius = std::sin(theta) * radius;

                    for (uint32_t longitude = 0; longitude <= longitudeSegments; ++longitude)
                    {
                        const float u = static_cast<float>(longitude) / static_cast<float>(longitudeSegments);
                        const float phi = u * Pi * 2.0f;
                        const float x = std::cos(phi) * ringRadius;
                        const float z = std::sin(phi) * ringRadius;
                        const Float3 normal = normalize({ x, y, z });
                        const Float4 tangent = { -std::sin(phi), 0.0f, std::cos(phi), 1.0f };
                        const Float4 vertexColor = !useLighting || !textureFileName.empty() ? Float4{ 1.0f, 1.0f, 1.0f, 1.0f }
                            : Float4{
                                0.35f + ((x / radius) + 1.0f) * 0.325f, 0.35f + ((y / radius) + 1.0f) * 0.325f, 0.35f + ((z / radius) + 1.0f) * 0.325f, 1.0f
                            };

                        mesh.m_vertices.push_back(Vertex{ { x, y, z }, vertexColor, { u, v }, normal, tangent });
                    }
                }

                const uint32_t rowVertexCount = longitudeSegments + 1U;
                for (uint32_t latitude = 0; latitude < latitudeSegments; ++latitude)
                {
                    for (uint32_t longitude = 0; longitude < longitudeSegments; ++longitude)
                    {
                        const uint32_t current = latitude * rowVertexCount + longitude;
                        const uint32_t next = current + rowVertexCount;

                        mesh.m_indices.push_back(current);
                        mesh.m_indices.push_back(next);
                        mesh.m_indices.push_back(current + 1U);

                        mesh.m_indices.push_back(current + 1U);
                        mesh.m_indices.push_back(next);
                        mesh.m_indices.push_back(next + 1U);
                    }
                }

                return mesh;
            }
    };
}

