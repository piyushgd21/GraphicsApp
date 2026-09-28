///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	SceneObjectGaussianSplat.h
//	Description	:
//		Defines the scene object used to draw Gaussian splats.
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "GaussianSplatUtility.h"
#include "SceneObject.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace graphicsFX
{
    //Defines the scene object used to draw Gaussian splats.
    class SceneObjectGaussianSplat final : public SceneObject
    {

        private:

            //Stores one Gaussian sort record.
            struct GaussianSortEntry
            {
                uint32_t m_index{ 0 }; //Stores the splat index.
                float m_distanceSquared{ 0.0f }; //Stores squared distance from the camera.
            };

            std::vector<Float3> m_sortPositions; //Stores splat local positions used for sorting.
            std::vector<uint32_t> m_sortedIndices; //Stores current sorted index order.
            std::vector<GaussianSortEntry> m_sortEntries; //Stores temporary sort keys.
            bool m_sortEnabled{ true }; //Stores whether sort is enabled.
            bool m_sortEveryFrame{ true }; //Stores whether sort runs every frame.
            bool m_sortDirty{ true }; //Stores whether the object needs an initial sort.

        public:

            SceneObjectGaussianSplat(void) = default;

            //Creates Gaussian splat point sprite mesh data.
            void Init(
                std::vector<GaussianSplatData> splats,
                const Float3 &position,
                const Float4x4 &orientation,
                float scale,
                float pointSize,
                float maxPointSize,
                bool sortEnabled,
                bool sortEveryFrame,
                bool depthTest,
                bool depthWrite,
                const SceneObjectShaderFiles &shaderFiles,
                SceneObjectRenderOptions renderOptions = {}
            )
            {
                if (splats.empty())
                {
                    throw std::runtime_error("Gaussian splat object must contain at least one splat.");
                }

                m_sortEnabled = sortEnabled;
                m_sortEveryFrame = sortEveryFrame;
                m_sortDirty = true;
                m_sortPositions = CreateSortPositions(splats);
                m_sortedIndices = CreateIndices(splats.size());
                m_sortEntries.resize(splats.size());

                renderOptions.m_castShadow = false;
                renderOptions.m_receiveShadow = false;
                renderOptions.m_alphaBlend = true;
                renderOptions.m_depthTest = depthTest;
                renderOptions.m_pointSize = pointSize;
                renderOptions.m_maxPointSize = maxPointSize;
                SetShaderFileNames(shaderFiles.m_vertexShader, shaderFiles.m_fragmentShader);
                SetMesh(CreateVertices(splats), m_sortedIndices, VK_PRIMITIVE_TOPOLOGY_POINT_LIST, depthWrite, 1.0f, position, orientation, scale, { 1.0f, 1.0f, 1.0f, 1.0f }, renderOptions);
            }

            //Updates Gaussian splat point sprite mesh data.
            void UpdateSplats(std::vector<GaussianSplatData> splats)
            {
                if (splats.empty())
                {
                    throw std::runtime_error("Gaussian splat object update must contain at least one splat.");
                }

                m_sortDirty = true;
                m_sortPositions = CreateSortPositions(splats);
                m_sortedIndices = CreateIndices(splats.size());
                m_sortEntries.resize(splats.size());
                UpdateMesh(CreateVertices(splats), m_sortedIndices);
            }

            //Updates Gaussian splat index order from the camera position.
            void UpdateSort(const Float3 &cameraPosition)
            {
                if (!m_sortEnabled || m_sortPositions.empty())
                {
                    return;
                }

                if (!m_sortEveryFrame && !m_sortDirty)
                {
                    return;
                }

                const Float4x4 model = multiply(translation(Position().x, Position().y, Position().z), multiply(Orientation(), graphicsFX::scale(Scale())));
                for (std::size_t splatIndex = 0U; splatIndex < m_sortPositions.size(); ++splatIndex)
                {
                    const Float3 worldPosition = TransformPosition(model, m_sortPositions[splatIndex]);
                    const Float3 cameraToSplat = worldPosition - cameraPosition;
                    m_sortEntries[splatIndex].m_index = static_cast<uint32_t>(splatIndex);
                    m_sortEntries[splatIndex].m_distanceSquared = dot(cameraToSplat, cameraToSplat);
                }

                std::sort(
                    m_sortEntries.begin(),
                    m_sortEntries.end(),
                    [](const GaussianSortEntry &a, const GaussianSortEntry &b)
                    {
                        return a.m_distanceSquared > b.m_distanceSquared;
                    }
                );

                for (std::size_t sortIndex = 0U; sortIndex < m_sortEntries.size(); ++sortIndex)
                {
                    m_sortedIndices[sortIndex] = m_sortEntries[sortIndex].m_index;
                }

                UpdateIndices(m_sortedIndices);
                m_sortDirty = false;
            }

        private:

            //Creates Gaussian splat point vertices.
            static std::vector<Vertex> CreateVertices(const std::vector<GaussianSplatData> &splats)
            {
                std::vector<Vertex> vertices;
                vertices.reserve(splats.size());

                for (const GaussianSplatData &splat : splats)
                {
                    vertices.push_back(Vertex{ splat.m_position, splat.m_color, { 0.0f, 0.0f }, splat.m_scale, splat.m_rotation });
                }

                return vertices;
            }

            //Creates Gaussian splat positions used for sorting.
            static std::vector<Float3> CreateSortPositions(const std::vector<GaussianSplatData> &splats)
            {
                std::vector<Float3> positions;
                positions.reserve(splats.size());

                for (const GaussianSplatData &splat : splats)
                {
                    positions.push_back(splat.m_position);
                }

                return positions;
            }

            //Creates Gaussian splat point indices.
            static std::vector<uint32_t> CreateIndices(std::size_t splatCount)
            {
                if (splatCount > std::numeric_limits<uint32_t>::max())
                {
                    throw std::runtime_error("Gaussian splat object has too many splats.");
                }

                std::vector<uint32_t> indices;
                indices.reserve(splatCount);

                for (std::size_t splatIndex = 0U; splatIndex < splatCount; ++splatIndex)
                {
                    indices.push_back(static_cast<uint32_t>(splatIndex));
                }

                return indices;
            }

            //Transforms a local position to world position.
            static Float3 TransformPosition(const Float4x4 &model, const Float3 &position)
            {
                const Float4 transformedPosition = transform(model, { position.x, position.y, position.z, 1.0f });
                return { transformedPosition.x, transformedPosition.y, transformedPosition.z };
            }
    };
}
