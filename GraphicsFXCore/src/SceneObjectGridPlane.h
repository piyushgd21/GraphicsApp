///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	SceneObjectGridPlane.h
//	Description	:
//		Defines the scene object used to draw a textured grid plane.
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
    //Defines the scene object used to draw a textured grid plane.
    class SceneObjectGridPlane final : public SceneObject
    {

        public:

            SceneObjectGridPlane(void) = default;

            //Creates textured grid plane mesh data.
            void Init(float minimum, float maximum, float step, const std::string &textureFileName, const SceneObjectShaderFiles &shaderFiles, SceneObjectRenderOptions renderOptions = {})
            {
                SetShaderFileNames(shaderFiles.m_vertexShader, shaderFiles.m_fragmentShader);
                SetTextureFileName(textureFileName);

                const float repeatCount = (maximum - minimum) / step;
                const Float4 color{ 1.0f, 1.0f, 1.0f, 1.0f };
                const Float3 normal{ 0.0f, 1.0f, 0.0f };
                const Float4 tangent{ 1.0f, 0.0f, 0.0f, -1.0f };

                std::vector<Vertex> vertices = {
                    Vertex{ { minimum, 0.0f, minimum }, color, { 0.0f, 0.0f }, normal, tangent }, Vertex{ { minimum, 0.0f, maximum }, color, { 0.0f, repeatCount }, normal, tangent },
                    Vertex{ { maximum, 0.0f, maximum }, color, { repeatCount, repeatCount }, normal, tangent }, Vertex{ { maximum, 0.0f, minimum }, color, { repeatCount, 0.0f }, normal, tangent }
                };

                std::vector<uint32_t> indices = {
                    0U, 1U, 2U, 2U, 3U, 0U
                };

                SetMesh(std::move(vertices), std::move(indices), VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, true, 1.0f, { 0.0f, 0.0f, 0.0f }, identity(), 1.0f, color, renderOptions);
            }
    };
}

