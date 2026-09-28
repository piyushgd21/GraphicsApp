///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	SceneObjectBox.h
//	Description	:
//		Defines the scene object used to draw colored boxes.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "SceneObject.h"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace graphicsFX
{
    //Defines the scene object used to draw colored boxes.
    class SceneObjectBox final : public SceneObject
    {

        public:

            SceneObjectBox(void) = default;

            //Creates box mesh data.
            void Init(
                const Float3 &position,
                const Float3 &boxSize,
                const Float4x4 &orientation,
                const Float4 &color,
                const SceneObjectShaderFiles &shaderFiles,
                const std::string &textureFileName = "",
                SceneObjectRenderOptions renderOptions = {}
            )
            {
                SetShaderFileNames(shaderFiles.m_vertexShader, shaderFiles.m_fragmentShader);
                SetTextureFileName(textureFileName);

                std::vector<Vertex> vertices = CreateVertices(boxSize);
                if (!textureFileName.empty())
                {
                    for (Vertex &vertex : vertices)
                    {
                        vertex.m_color = { 1.0f, 1.0f, 1.0f, 1.0f };
                    }
                }

                SetMesh(std::move(vertices), CreateIndices(), VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, true, 1.0f, position, orientation, 1.0f, color, renderOptions);
            }

            //Creates instanced box mesh data.
            void Init(
                std::vector<SceneObjectInstanceData> instances,
                const Float3 &boxSize,
                const SceneObjectShaderFiles &shaderFiles,
                const std::string &textureFileName = "",
                SceneObjectRenderOptions renderOptions = {}
            )
            {
                SetShaderFileNames(shaderFiles.m_vertexShader, shaderFiles.m_fragmentShader);
                SetTextureFileName(textureFileName);

                std::vector<Vertex> vertices = CreateVertices(boxSize);
                if (!textureFileName.empty())
                {
                    for (Vertex &vertex : vertices)
                    {
                        vertex.m_color = { 1.0f, 1.0f, 1.0f, 1.0f };
                    }
                }

                SetMesh(std::move(vertices), CreateIndices(), VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, true, 1.0f, { 0.0f, 0.0f, 0.0f }, identity(), 1.0f, { 1.0f, 1.0f, 1.0f, 1.0f }, renderOptions);
                SetInstances(std::move(instances));
            }

        private:

            //Creates box vertices.
            static std::vector<Vertex> CreateVertices(const Float3 &boxSize)
            {
                std::vector<Vertex> vertices;
                vertices.reserve(24);

                const Float3 halfSize = {
                    boxSize.x * 0.5f, boxSize.y * 0.5f, boxSize.z * 0.5f
                };

                const auto appendFace = [&vertices](const Float3 &a, const Float3 &b, const Float3 &c, const Float3 &d, const Float4 &color, const Float3 &normal, const Float3 &tangent)
                {
                    const Float4 tangentWithHandedness = { tangent.x, tangent.y, tangent.z, 1.0f };

                    vertices.push_back(Vertex{ a, color, { 0.0f, 1.0f }, normal, tangentWithHandedness });
                    vertices.push_back(Vertex{ b, color, { 1.0f, 1.0f }, normal, tangentWithHandedness });
                    vertices.push_back(Vertex{ c, color, { 1.0f, 0.0f }, normal, tangentWithHandedness });
                    vertices.push_back(Vertex{ d, color, { 0.0f, 0.0f }, normal, tangentWithHandedness });
                };

                appendFace(
                    { -halfSize.x, -halfSize.y,  halfSize.z },
                    {  halfSize.x, -halfSize.y,  halfSize.z },
                    {  halfSize.x,  halfSize.y,  halfSize.z },
                    { -halfSize.x,  halfSize.y,  halfSize.z },
                    { 1.0f, 0.2f, 0.2f, 1.0f },
                    { 0.0f, 0.0f, 1.0f },
                    { 1.0f, 0.0f, 0.0f }
                );

                appendFace(
                    {  halfSize.x, -halfSize.y,  halfSize.z },
                    {  halfSize.x, -halfSize.y, -halfSize.z },
                    {  halfSize.x,  halfSize.y, -halfSize.z },
                    {  halfSize.x,  halfSize.y,  halfSize.z },
                    { 0.2f, 1.0f, 0.3f, 1.0f },
                    { 1.0f, 0.0f, 0.0f },
                    { 0.0f, 0.0f, -1.0f }
                );

                appendFace(
                    {  halfSize.x, -halfSize.y, -halfSize.z },
                    { -halfSize.x, -halfSize.y, -halfSize.z },
                    { -halfSize.x,  halfSize.y, -halfSize.z },
                    {  halfSize.x,  halfSize.y, -halfSize.z },
                    { 0.2f, 0.5f, 1.0f, 1.0f },
                    { 0.0f, 0.0f, -1.0f },
                    { -1.0f, 0.0f, 0.0f }
                );

                appendFace(
                    { -halfSize.x, -halfSize.y, -halfSize.z },
                    { -halfSize.x, -halfSize.y,  halfSize.z },
                    { -halfSize.x,  halfSize.y,  halfSize.z },
                    { -halfSize.x,  halfSize.y, -halfSize.z },
                    { 1.0f, 0.9f, 0.2f, 1.0f },
                    { -1.0f, 0.0f, 0.0f },
                    { 0.0f, 0.0f, 1.0f }
                );

                appendFace(
                    { -halfSize.x,  halfSize.y,  halfSize.z },
                    {  halfSize.x,  halfSize.y,  halfSize.z },
                    {  halfSize.x,  halfSize.y, -halfSize.z },
                    { -halfSize.x,  halfSize.y, -halfSize.z },
                    { 0.8f, 0.3f, 1.0f, 1.0f },
                    { 0.0f, 1.0f, 0.0f },
                    { 1.0f, 0.0f, 0.0f }
                );

                appendFace(
                    { -halfSize.x, -halfSize.y, -halfSize.z },
                    {  halfSize.x, -halfSize.y, -halfSize.z },
                    {  halfSize.x, -halfSize.y,  halfSize.z },
                    { -halfSize.x, -halfSize.y,  halfSize.z },
                    { 0.2f, 1.0f, 1.0f, 1.0f },
                    { 0.0f, -1.0f, 0.0f },
                    { 1.0f, 0.0f, 0.0f }
                );

                return vertices;
            }

            //Creates box indices.
            static std::vector<uint32_t> CreateIndices(void)
            {
                return {
                    0, 1, 2, 2, 3, 0, 4, 5, 6, 6, 7, 4, 8, 9, 10, 10, 11, 8, 12, 13, 14, 14, 15, 12, 16, 17, 18, 18, 19, 16, 20, 21, 22, 22, 23, 20
                };
            }
    };
}

