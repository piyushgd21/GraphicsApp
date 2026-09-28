///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	SceneObjectPointCloud.h
//	Description	:
//		Defines the scene object used to draw point clouds.
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "SceneObject.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace graphicsFX
{
    //Stores one point cloud point.
    struct SceneObjectPointCloudPoint
    {
        Float3 m_position; //Stores the point position.
        Float4 m_color{ 1.0f, 1.0f, 1.0f, 1.0f }; //Stores the point color.
    };

    //Defines the scene object used to draw point clouds.
    class SceneObjectPointCloud final : public SceneObject
    {

        public:

            SceneObjectPointCloud(void) = default;

            //Creates point cloud mesh data.
            void Init(
                std::vector<SceneObjectPointCloudPoint> points,
                const Float3 &position,
                const Float4x4 &orientation,
                float scale,
                float pointSize,
                const Float4 &color,
                const SceneObjectShaderFiles &shaderFiles,
                SceneObjectRenderOptions renderOptions = {}
            )
            {
                if (points.empty())
                {
                    throw std::runtime_error("Point cloud must contain at least one point.");
                }

                renderOptions.m_castShadow = false;
                renderOptions.m_pointSize = pointSize;
                m_pickPointSize = pointSize;
                m_pickPositions.clear();
                m_pickPositions.reserve(points.size());
                for (const SceneObjectPointCloudPoint &point : points)
                {
                    m_pickPositions.push_back(point.m_position);
                }

                SetShaderFileNames(shaderFiles.m_vertexShader, shaderFiles.m_fragmentShader);
                SetMesh(CreateVertices(points), CreateIndices(points.size()), VK_PRIMITIVE_TOPOLOGY_POINT_LIST, true, 1.0f, position, orientation, scale, color, renderOptions);
            }

            //Returns whether the ray hits a rendered point in this cloud.
            bool PickRay(const Float3 &rayOrigin, const Float3 &rayDirection, float worldUnitsPerPixelAtUnitDistance, float &hitDistance) const
            {
                bool hit = false;
                float bestDistance = std::numeric_limits<float>::max();
                const float pointPixelRadius = std::max(6.0f, m_pickPointSize * 1.75f);
                const float safePixelScale = std::max(0.000001f, worldUnitsPerPixelAtUnitDistance);
                const Float3 &objectPosition = Position();
                const Float4x4 model = multiply(translation(objectPosition.x, objectPosition.y, objectPosition.z), multiply(Orientation(), scale(Scale())));

                for (const Float3 &point : m_pickPositions)
                {
                    const Float4 worldPoint = transform(model, { point.x, point.y, point.z, 1.0f });
                    const Float3 pointPosition{ worldPoint.x, worldPoint.y, worldPoint.z };
                    const Float3 offset = pointPosition - rayOrigin;
                    const float distanceAlongRay = dot(offset, rayDirection);
                    if (distanceAlongRay <= 0.0f)
                    {
                        continue;
                    }

                    const float pointRadius = std::max(0.01f, distanceAlongRay * safePixelScale * pointPixelRadius);
                    const float closestDistanceSquared = dot(offset, offset) - (distanceAlongRay * distanceAlongRay);
                    if (closestDistanceSquared <= pointRadius * pointRadius && distanceAlongRay < bestDistance)
                    {
                        hit = true;
                        bestDistance = distanceAlongRay;
                    }
                }

                hitDistance = bestDistance;
                return hit && std::isfinite(hitDistance);
            }

        private:

            //Creates point cloud vertices.
            static std::vector<Vertex> CreateVertices(const std::vector<SceneObjectPointCloudPoint> &points)
            {
                std::vector<Vertex> vertices;
                vertices.reserve(points.size());

                for (const SceneObjectPointCloudPoint &point : points)
                {
                    vertices.push_back(Vertex{ point.m_position, point.m_color, { 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 1.0f, 0.0f, 0.0f, 1.0f } });
                }

                return vertices;
            }

            //Creates point cloud indices.
            static std::vector<uint32_t> CreateIndices(std::size_t pointCount)
            {
                if (pointCount > std::numeric_limits<uint32_t>::max())
                {
                    throw std::runtime_error("Point cloud has too many points.");
                }

                std::vector<uint32_t> indices;
                indices.reserve(pointCount);

                for (std::size_t pointIndex = 0; pointIndex < pointCount; ++pointIndex)
                {
                    indices.push_back(static_cast<uint32_t>(pointIndex));
                }

                return indices;
            }

            std::vector<Float3> m_pickPositions; //Stores point positions used for selection.
            float m_pickPointSize{ 1.0f }; //Stores point size used for selection.
    };
}
