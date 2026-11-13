#include "PlaneSweepColorSelection.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace graphicsFX
{
    namespace
    {
        using Sweep = MultiViewUtility::MultiViewPlaneSweepUtility;
        constexpr uint8_t interiorPixels = 8; // Matching/depth-map pixels, not original-image pixels.
        constexpr float opaqueAlpha = 0.99f;

        bool ValidMap(const Sweep::PlaneSweepDepthMap &map)
        {
            return map.m_width && map.m_height && map.m_imageWidth && map.m_imageHeight &&
                map.m_intrinsic.m_fx > 0 && map.m_intrinsic.m_fy > 0 &&
                std::isfinite(map.m_intrinsic.m_fx) && std::isfinite(map.m_intrinsic.m_fy) &&
                map.m_depthValues.size() == size_t(map.m_width) * map.m_height;
        }

        bool ValidTexture(const TextureData &texture)
        {
            return texture.m_width && texture.m_height && texture.m_channelCount == 4 &&
                texture.m_pixels.size() == size_t(texture.m_width) * texture.m_height * 4;
        }

        struct ColorView
        {
            Sweep::PlaneSweepImage image;
            std::vector<uint8_t> clearance;

            bool Prepare(const Sweep::PlaneSweepDepthMap &map)
            {
                // Without a foreground mask there is no evidence of a silhouette to repair.
                if (!ValidMap(map) || !image.m_hasMask || !ValidTexture(image.m_texture) ||
                    !ValidTexture(image.m_maskTexture) || image.m_texture.m_width != map.m_imageWidth ||
                    image.m_texture.m_height != map.m_imageHeight) return false;
                const auto &texture = image.m_texture;
                const auto &mask = image.m_maskTexture;
                const bool directMask = mask.m_width == texture.m_width && mask.m_height == texture.m_height;
                clearance.assign(size_t(map.m_width) * map.m_height, 0);
                // A grid cell is foreground only if its whole original-image footprint is opaque.
                for (uint32_t y = 0; y < map.m_height; ++y)
                    for (uint32_t x = 0; x < map.m_width; ++x)
                    {
                        const uint32_t x0 = uint32_t(std::floor(double(x) * texture.m_width / map.m_width));
                        const uint32_t y0 = uint32_t(std::floor(double(y) * texture.m_height / map.m_height));
                        const uint32_t x1 = std::min(texture.m_width, uint32_t(std::ceil(double(x + 1) * texture.m_width / map.m_width)));
                        const uint32_t y1 = std::min(texture.m_height, uint32_t(std::ceil(double(y + 1) * texture.m_height / map.m_height)));
                        bool opaque = true;
                        for (uint32_t iy = y0; iy < y1 && opaque; ++iy)
                            for (uint32_t ix = x0; ix < x1; ++ix)
                            {
                                Float4 color;
                                if (directMask)
                                {
                                    const size_t i = (size_t(iy) * mask.m_width + ix) * 4;
                                    color.w = image.m_maskUsesAlpha ? mask.m_pixels[i + 3] / 255.0f :
                                        (mask.m_pixels[i] * 0.299f + mask.m_pixels[i + 1] * 0.587f + mask.m_pixels[i + 2] * 0.114f) / 255.0f;
                                }
                                else if (!Sweep::SamplePlaneSweepPixel(image, float(ix), float(iy), color))
                                { opaque = false; break; }
                                if (color.w < opaqueAlpha) { opaque = false; break; }
                            }
                        clearance[size_t(y) * map.m_width + x] = opaque ? uint8_t(255) : uint8_t(0);
                    }
                // Two-pass chessboard distance: conservative square clearance from mask/image boundaries.
                const auto relax = [&](int x, int y, int nx, int ny)
                {
                    auto &value = clearance[size_t(y) * map.m_width + x];
                    const int neighbor = nx < 0 || ny < 0 || nx >= int(map.m_width) || ny >= int(map.m_height)
                        ? 0 : clearance[size_t(ny) * map.m_width + nx];
                    value = uint8_t(std::min(int(value), neighbor + 1));
                };
                for (int y = 0; y < int(map.m_height); ++y)
                    for (int x = 0; x < int(map.m_width); ++x)
                    {
                        relax(x, y, x - 1, y); relax(x, y, x - 1, y - 1);
                        relax(x, y, x, y - 1); relax(x, y, x + 1, y - 1);
                    }
                for (int y = int(map.m_height) - 1; y >= 0; --y)
                    for (int x = int(map.m_width) - 1; x >= 0; --x)
                    {
                        relax(x, y, x + 1, y); relax(x, y, x + 1, y + 1);
                        relax(x, y, x, y + 1); relax(x, y, x - 1, y + 1);
                    }
                return true;
            }
        };

        struct Projection
        {
            float imageX, imageY, gridX, gridY, depth;
            int x0, x1, y0, y1;
        };

        bool Project(const ColmapSetup &setup, const Sweep::PlaneSweepDepthMap &map,
            const Float3 &point, Projection &p)
        {
            if (!ValidMap(map) || !Sweep::ProjectPoint(map.m_camera, map.m_intrinsic,
                point, p.imageX, p.imageY, p.depth) || !std::isfinite(p.depth)) return false;
            // ProjectPoint returns camera pixel-center coordinates; image sampling uses pixel indices.
            p.imageX -= 0.5f; p.imageY -= 0.5f;
            if (p.imageX < 0 || p.imageY < 0 || p.imageX > float(map.m_imageWidth - 1) ||
                p.imageY > float(map.m_imageHeight - 1)) return false;
            p.gridX = setup.m_planeSweepCoarseToFineEnabled
                ? (p.imageX + 0.5f) * float(map.m_width) / float(map.m_imageWidth) - 0.5f
                : p.imageX * float(map.m_width) / float(map.m_imageWidth);
            p.gridY = setup.m_planeSweepCoarseToFineEnabled
                ? (p.imageY + 0.5f) * float(map.m_height) / float(map.m_imageHeight) - 0.5f
                : p.imageY * float(map.m_height) / float(map.m_imageHeight);
            if (p.gridX < 0 || p.gridY < 0 || p.gridX > float(map.m_width - 1) || p.gridY > float(map.m_height - 1)) return false;
            p.x0 = int(std::floor(p.gridX)); p.x1 = int(std::ceil(p.gridX));
            p.y0 = int(std::floor(p.gridY)); p.y1 = int(std::ceil(p.gridY));
            return true;
        }

        uint8_t Clearance(const ColorView &view, const Sweep::PlaneSweepDepthMap &map, const Projection &p)
        {
            return std::min({ view.clearance[size_t(p.y0) * map.m_width + p.x0],
                view.clearance[size_t(p.y0) * map.m_width + p.x1],
                view.clearance[size_t(p.y1) * map.m_width + p.x0],
                view.clearance[size_t(p.y1) * map.m_width + p.x1] });
        }

        bool Visible(const ColmapSetup &setup, const Sweep::PlaneSweepDepthMap &source,
            const Projection &p, const Sweep::PlaneSweepDepthMap &reference, const Projection &r)
        {
            const float tolerance = std::max(0.0f, setup.m_planeSweepCrossViewRelativeDepthTolerance);
            // Require the projected footprint itself to agree. Never borrow a nearby foreground depth
            // across a hole/occlusion, or interpolate between foreground and background depths.
            for (int y : { p.y0, p.y1 })
                for (int x : { p.x0, p.x1 })
                {
                    const float d = source.m_depthValues[size_t(y) * source.m_width + x];
                    if (!std::isfinite(d) || d <= 0 || std::fabs(d - p.depth) / p.depth > tolerance) return false;
                }
            // Only the closest supporting sample makes the round trip; checking every corner would
            // reject exact geometry due to subpixel discretization at a one-pixel tolerance.
            const int x = int(std::lround(p.gridX)), y = int(std::lround(p.gridY));
            const float d = source.m_depthValues[size_t(y) * source.m_width + x];
            const float imageX = setup.m_planeSweepCoarseToFineEnabled
                ? (float(x) + 0.5f) * float(source.m_imageWidth) / float(source.m_width) - 0.5f
                : float(size_t(x) * source.m_imageWidth / source.m_width);
            const float imageY = setup.m_planeSweepCoarseToFineEnabled
                ? (float(y) + 0.5f) * float(source.m_imageHeight) / float(source.m_height) - 0.5f
                : float(size_t(y) * source.m_imageHeight / source.m_height);
            const Float3 support = Sweep::UnprojectPixel(source.m_camera, source.m_intrinsic, imageX, imageY, d);
            float rx, ry, rz;
            if (!Sweep::ProjectPoint(reference.m_camera, reference.m_intrinsic, support, rx, ry, rz) ||
                !std::isfinite(rz) || std::fabs(rz - r.depth) / r.depth > tolerance) return false;
            const float ex = (rx - (r.imageX + 0.5f)) * float(reference.m_width) / float(reference.m_imageWidth);
            const float ey = (ry - (r.imageY + 0.5f)) * float(reference.m_height) / float(reference.m_imageHeight);
            const float pixels = std::max(0.0f, setup.m_planeSweepCrossViewReprojectionTolerancePixels);
            if (ex * ex + ey * ey > pixels * pixels) return false;
            return true;
        }

        struct Observation { Float4 color; uint8_t clearance; };
        float ColorDistance(const Float4 &a, const Float4 &b)
        { return std::fabs(a.x - b.x) + std::fabs(a.y - b.y) + std::fabs(a.z - b.z); }
    }

    PlaneSweepColorSelectionStats SelectPlaneSweepBoundaryColors(const ColmapSetup &setup,
        const std::vector<Sweep::PlaneSweepDepthMap> &maps, std::vector<Sweep::PlaneSweepPointCandidate> &points,
        const std::function<bool(std::size_t, Sweep::PlaneSweepImage &)> &loadImage)
    {
        PlaneSweepColorSelectionStats stats;
        if (!setup.m_planeSweepBoundaryColorSelectionEnabled || points.empty() || !loadImage) return stats;
        // Reconstruction and consistency filtering preserve reference-contiguous candidate order.
        // Process each run without sorting/moving points or caching hundreds of full-resolution images.
        for (size_t begin = 0; begin < points.size();)
        {
            const size_t referenceIndex = points[begin].m_depthMapIndex;
            size_t end = begin + 1;
            while (end < points.size() && points[end].m_depthMapIndex == referenceIndex) ++end;
            if (referenceIndex >= maps.size() || !ValidMap(maps[referenceIndex])) { begin = end; continue; }
            const auto &reference = maps[referenceIndex];
            std::vector<size_t> boundary;
            std::vector<Projection> projections;
            {
                ColorView view;
                if (!loadImage(referenceIndex, view.image)) { ++stats.m_imagesUnavailable; begin = end; continue; }
                if (!view.Prepare(reference)) { begin = end; continue; }
                for (size_t i = begin; i < end; ++i)
                {
                    Projection p;
                    if (Project(setup, reference, points[i].m_position, p) && Clearance(view, reference, p) < interiorPixels)
                    { boundary.push_back(i); projections.push_back(p); }
                }
            } // Release reference textures before loading any comparison camera.
            stats.m_boundaryPoints += boundary.size();
            if (boundary.empty()) { begin = end; continue; }
            std::vector<Sweep::PlaneSweepSourceCandidate> neighbors;
            for (size_t i = 0; i < maps.size(); ++i)
                if (i != referenceIndex && ValidMap(maps[i]))
                {
                    const float distance = length(maps[i].m_camera.m_position - reference.m_camera.m_position);
                    const float alignment = std::max(0.0f, dot(reference.m_camera.m_forward, maps[i].m_camera.m_forward));
                    const float score = distance / (0.25f + alignment);
                    if (std::isfinite(score)) neighbors.push_back({ i, 0U, score });
                }
            std::stable_sort(neighbors.begin(), neighbors.end(), [](const auto &a, const auto &b) { return a.m_score < b.m_score; });
            const size_t count = std::min(neighbors.size(), size_t(std::max(1, setup.m_planeSweepBoundaryColorViewCount)));
            std::vector<std::vector<Observation>> observations(boundary.size());
            for (size_t n = 0; n < count; ++n)
            {
                const size_t index = neighbors[n].m_cameraIndex;
                const auto &source = maps[index];
                ColorView view;
                if (!loadImage(index, view.image)) { ++stats.m_imagesUnavailable; continue; }
                if (!view.Prepare(source)) continue;
                for (size_t b = 0; b < boundary.size(); ++b)
                {
                    Projection p;
                    if (!Project(setup, source, points[boundary[b]].m_position, p)) continue;
                    const uint8_t clearance = Clearance(view, source, p);
                    if (clearance < interiorPixels || !Visible(setup, source, p, reference, projections[b])) continue;
                    Float4 color;
                    if (Sweep::SamplePlaneSweepPixel(view.image, p.imageX, p.imageY, color) && color.w >= opaqueAlpha)
                        observations[b].push_back({ color, clearance });
                }
            }
            for (size_t b = 0; b < boundary.size(); ++b)
            {
                const auto &colors = observations[b];
                if (colors.size() < 2) { ++stats.m_insufficientObservations; continue; }
                size_t best = 0;
                float bestScore = std::numeric_limits<float>::infinity();
                for (size_t i = 0; i < colors.size(); ++i)
                {
                    float score = 0;
                    for (const auto &other : colors) score += ColorDistance(colors[i].color, other.color);
                    if (score < bestScore - 1e-6f || (std::fabs(score - bestScore) <= 1e-6f && colors[i].clearance > colors[best].clearance))
                    { best = i; bestScore = score; }
                }
                auto &color = points[boundary[b]].m_color;
                const auto &selected = colors[best].color;
                if (color.x != selected.x || color.y != selected.y || color.z != selected.z)
                {
                    color.x = selected.x; color.y = selected.y; color.z = selected.z;
                    ++stats.m_recoloredPoints;
                }
            }
            begin = end;
        }
        return stats;
    }
}
