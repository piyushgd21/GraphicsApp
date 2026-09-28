///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	SceneObjectAxis.h
//	Description	:
//		Defines the scene object used to draw the world axes.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "SceneObject.h"

#include <cstdint>
#include <utility>
#include <vector>

namespace graphicsFX
{
    //Defines the scene object used to draw the world axes.
    class SceneObjectAxis final : public SceneObject
    {

        public:

            SceneObjectAxis(void) = default;

            //Creates axis line mesh data.
            void Init(float minimum, float maximum, float lineWidth, const SceneObjectShaderFiles &shaderFiles, SceneObjectRenderOptions renderOptions = {})
            {
                SetShaderFileNames(shaderFiles.m_vertexShader, shaderFiles.m_fragmentShader);

                std::vector<Vertex> vertices;
                std::vector<uint32_t> indices;

                const auto appendLine = [&](const Vertex &start, const Vertex &end)
                {
                    const uint32_t first = static_cast<uint32_t>(vertices.size());
                    vertices.push_back(start);
                    vertices.push_back(end);
                    indices.push_back(first);
                    indices.push_back(first + 1U);
                };

                appendLine(Vertex{ { minimum, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } }, Vertex{ { maximum, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } });

                appendLine(Vertex{ { 0.0f, minimum, 0.0f }, { 0.0f, 0.75f, 0.0f, 1.0f } }, Vertex{ { 0.0f, maximum, 0.0f }, { 0.0f, 0.75f, 0.0f, 1.0f } });

                appendLine(Vertex{ { 0.0f, 0.0f, minimum }, { 0.0f, 0.2f, 1.0f, 1.0f } }, Vertex{ { 0.0f, 0.0f, maximum }, { 0.0f, 0.2f, 1.0f, 1.0f } });

                SetMesh(std::move(vertices), std::move(indices), VK_PRIMITIVE_TOPOLOGY_LINE_LIST, false, lineWidth, { 0.0f, 0.0f, 0.0f }, identity(), 1.0f, { 1.0f, 1.0f, 1.0f, 1.0f }, renderOptions);
            }
    };
}

