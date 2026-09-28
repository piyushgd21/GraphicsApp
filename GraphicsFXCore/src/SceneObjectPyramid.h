///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	SceneObjectPyramid.h
//	Description	:
//		Defines the scene object used to show a pyramid position and direction marker.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "SceneObject.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace graphicsFX
{
    //Defines the scene object used to show a pyramid position and direction marker.
    class SceneObjectPyramid final : public SceneObject
    {

        public:

            SceneObjectPyramid(void) = default;

            //Creates pyramid mesh data.
            void Init(const Float3 &position, const Float3 &direction, float baseSize, float height, const Float4 &color, const SceneObjectShaderFiles &shaderFiles, SceneObjectRenderOptions renderOptions = {})
            {
                SetShaderFileNames(shaderFiles.m_vertexShader, shaderFiles.m_fragmentShader);
                SetMesh(CreateVertices(direction, baseSize, height, color), CreateIndices(), VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, true, 1.0f, position, identity(), 1.0f, { 1.0f, 1.0f, 1.0f, 1.0f }, renderOptions);
            }

            //Creates instanced pyramid mesh data.
            void Init(std::vector<SceneObjectInstanceData> instances, const Float3 &direction, float baseSize, float height, const SceneObjectShaderFiles &shaderFiles, SceneObjectRenderOptions renderOptions = {})
            {
                SetShaderFileNames(shaderFiles.m_vertexShader, shaderFiles.m_fragmentShader);
                SetMesh(
                    CreateVertices(direction, baseSize, height, { 1.0f, 1.0f, 1.0f, 1.0f }),
                    CreateIndices(),
                    VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST,
                    true,
                    1.0f,
                    { 0.0f, 0.0f, 0.0f },
                    identity(),
                    1.0f,
                    { 1.0f, 1.0f, 1.0f, 1.0f },
                    renderOptions
                );
                SetInstances(std::move(instances));
            }

        private:

            //Returns a safe normalized direction.
            static Float3 NormalizeOrDefault(const Float3 &value, const Float3 &fallback)
            {
                const float lengthSquared = dot(value, value);
                if (lengthSquared <= 0.000001f)
                {
                    return fallback;
                }

                const float inverseLength = 1.0f / std::sqrt(lengthSquared);
                return value * inverseLength;
            }

            //Creates pyramid vertices.
            static std::vector<Vertex> CreateVertices(const Float3 &direction, float baseSize, float height, const Float4 &color)
            {
                const Float3 forward = NormalizeOrDefault(direction, { 0.0f, -1.0f, 0.0f });
                const Float3 upSeed = std::fabs(dot(forward, { 0.0f, 1.0f, 0.0f })) > 0.95f ? Float3{ 1.0f, 0.0f, 0.0f }
                    : Float3{ 0.0f, 1.0f, 0.0f };
                const Float3 right = NormalizeOrDefault(cross(upSeed, forward), { 1.0f, 0.0f, 0.0f });
                const Float3 up = NormalizeOrDefault(cross(forward, right), { 0.0f, 1.0f, 0.0f });

                const float safeHeight = std::max(height, 0.0001f);
                const float halfBaseWidth = std::max(baseSize, 0.0001f) * 0.5f;

                const Float3 apex = { 0.0f, 0.0f, 0.0f };
                const Float3 baseCenter = forward * safeHeight;
                const Float3 base0 = baseCenter + right * halfBaseWidth + up * halfBaseWidth;
                const Float3 base1 = baseCenter - right * halfBaseWidth + up * halfBaseWidth;
                const Float3 base2 = baseCenter - right * halfBaseWidth - up * halfBaseWidth;
                const Float3 base3 = baseCenter + right * halfBaseWidth - up * halfBaseWidth;

                std::vector<Vertex> vertices;
                vertices.reserve(18);

                AddTriangle(vertices, apex, base0, base1, color);
                AddTriangle(vertices, apex, base1, base2, color);
                AddTriangle(vertices, apex, base2, base3, color);
                AddTriangle(vertices, apex, base3, base0, color);
                AddTriangle(vertices, base0, base3, base2, color);
                AddTriangle(vertices, base0, base2, base1, color);

                return vertices;
            }

            //Adds one flat-shaded pyramid triangle.
            static void AddTriangle(std::vector<Vertex> &vertices, const Float3 &a, const Float3 &b, const Float3 &c, const Float4 &color)
            {
                const Float2 texA = { 0.5f, 0.0f };
                const Float2 texB = { 1.0f, 1.0f };
                const Float2 texC = { 0.0f, 1.0f };
                const Float3 edge0 = b - a;
                const Float3 edge1 = c - a;
                const Float2 deltaTex0 = { texB.x - texA.x, texB.y - texA.y };
                const Float2 deltaTex1 = { texC.x - texA.x, texC.y - texA.y };
                const Float3 normal = NormalizeOrDefault(cross(edge1, edge0), { 0.0f, 1.0f, 0.0f });
                const float determinant = (deltaTex0.x * deltaTex1.y) - (deltaTex0.y * deltaTex1.x);
                Float3 tangent = NormalizeOrDefault(edge0, { 1.0f, 0.0f, 0.0f });
                Float3 bitangent = NormalizeOrDefault(edge1, { 0.0f, 1.0f, 0.0f });

                if (std::fabs(determinant) > 0.000001f)
                {
                    const float inverseDeterminant = 1.0f / determinant;
                    tangent = NormalizeOrDefault(((edge0 * deltaTex1.y) - (edge1 * deltaTex0.y)) * inverseDeterminant, tangent);
                    bitangent = NormalizeOrDefault(((edge1 * deltaTex0.x) - (edge0 * deltaTex1.x)) * inverseDeterminant, bitangent);
                }

                const float handedness = dot(cross(normal, tangent), bitangent) < 0.0f ? -1.0f : 1.0f;
                const Float4 tangentWithHandedness = { tangent.x, tangent.y, tangent.z, handedness };

                vertices.push_back(Vertex{ a, color, texA, normal, tangentWithHandedness });
                vertices.push_back(Vertex{ b, color, texB, normal, tangentWithHandedness });
                vertices.push_back(Vertex{ c, color, texC, normal, tangentWithHandedness });
            }

            //Creates pyramid indices.
            static std::vector<uint32_t> CreateIndices(void)
            {
                return {
                    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U, 13U, 14U, 15U, 16U, 17U
                };
            }
    };
}

