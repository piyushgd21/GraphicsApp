///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	SceneObjectGrid.h
//	Description	:
//		Defines the scene object used to draw grid lines.
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
    //Defines the scene object used to draw grid lines.
    class SceneObjectGrid final : public SceneObject
    {

        public:

            SceneObjectGrid(void) = default;

            //Creates grid line mesh data.
            void Init(float minimum, float maximum, float step, float lineWidth, const SceneObjectShaderFiles &shaderFiles, SceneObjectRenderOptions renderOptions = {})
            {
                SetShaderFileNames(shaderFiles.m_vertexShader, shaderFiles.m_fragmentShader);

                std::vector<Vertex> vertices;
                std::vector<uint32_t> indices;
                const float gray = 0.72f;

                const auto appendLine = [&](const Vertex &start, const Vertex &end)
                {
                    const uint32_t first = static_cast<uint32_t>(vertices.size());
                    vertices.push_back(start);
                    vertices.push_back(end);
                    indices.push_back(first);
                    indices.push_back(first + 1U);
                };

                for (float coordinate = minimum; coordinate <= maximum; coordinate += step)
                {
                    appendLine(Vertex{ { minimum, 0.0f, coordinate }, { gray, gray, gray, 1.0f } }, Vertex{ { maximum, 0.0f, coordinate }, { gray, gray, gray, 1.0f } });

                    appendLine(Vertex{ { coordinate, 0.0f, minimum }, { gray, gray, gray, 1.0f } }, Vertex{ { coordinate, 0.0f, maximum }, { gray, gray, gray, 1.0f } });
                }

                SetMesh(std::move(vertices), std::move(indices), VK_PRIMITIVE_TOPOLOGY_LINE_LIST, false, lineWidth, { 0.0f, 0.0f, 0.0f }, identity(), 1.0f, { 1.0f, 1.0f, 1.0f, 1.0f }, renderOptions);
            }
    };
}

