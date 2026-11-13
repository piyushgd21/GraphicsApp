#include "PlaneSweepColorSelection.h"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace graphicsFX;
using Sweep = MultiViewUtility::MultiViewPlaneSweepUtility;

static void Require(bool condition, const char *message)
{ if (!condition) throw std::runtime_error(message); }

static Sweep::PlaneSweepDepthMap Map(float x, uint32_t width = 101, uint32_t height = 81)
{
    Sweep::PlaneSweepDepthMap map;
    map.m_camera.m_position.x = x;
    map.m_intrinsic.m_width = map.m_imageWidth = 101;
    map.m_intrinsic.m_height = map.m_imageHeight = 81;
    map.m_intrinsic.m_fx = map.m_intrinsic.m_fy = 200;
    map.m_intrinsic.m_cx = 50.5; map.m_intrinsic.m_cy = 40.5;
    map.m_width = width; map.m_height = height;
    map.m_depthValues.assign(size_t(width) * height, 5.0f);
    return map;
}

static Sweep::PlaneSweepImage Image(const Sweep::PlaneSweepDepthMap &map, std::array<uint8_t, 3> rgb, bool boundary)
{
    Sweep::PlaneSweepImage image;
    image.m_camera = map.m_camera; image.m_intrinsic = map.m_intrinsic;
    auto &texture = image.m_texture;
    texture.m_width = map.m_imageWidth; texture.m_height = map.m_imageHeight;
    texture.m_pixels.resize(size_t(texture.m_width) * texture.m_height * 4);
    for (size_t i = 0; i < texture.m_pixels.size(); i += 4)
    {
        texture.m_pixels[i] = rgb[0]; texture.m_pixels[i + 1] = rgb[1];
        texture.m_pixels[i + 2] = rgb[2]; texture.m_pixels[i + 3] = 255;
    }
    image.m_hasMask = true;
    image.m_maskTexture = texture;
    auto &mask = image.m_maskTexture;
    for (uint32_t y = 0; y < mask.m_height; ++y)
        for (uint32_t x = 0; x < mask.m_width; ++x)
        {
            const size_t i = (size_t(y) * mask.m_width + x) * 4;
            const uint8_t alpha = boundary && x < 49 ? uint8_t(0) : uint8_t(255);
            mask.m_pixels[i] = mask.m_pixels[i + 1] = mask.m_pixels[i + 2] = alpha;
            mask.m_pixels[i + 3] = 255;
        }
    return image;
}

static std::vector<Sweep::PlaneSweepPointCandidate> Points()
{ return { { 0, { 0.0f, 0.0f, -5.0f }, { 0.0f, 0.8f, 0.0f, 0.4f } } }; }

static void Unchanged(const std::vector<Sweep::PlaneSweepPointCandidate> &points)
{
    Require(points.size() == 1 && points[0].m_depthMapIndex == 0, "Selection changed point identity/count");
    const auto &p = points[0];
    Require(p.m_position.x == 0 && p.m_position.y == 0 && p.m_position.z == -5 && p.m_color.w == 0.4f,
        "Selection changed geometry or alpha");
    Require(p.m_color.x == 0 && p.m_color.y == 0.8f && p.m_color.z == 0, "Fallback changed original RGB");
}

int main()
{
    try
    {
        ColmapSetup setup;
        Require(!setup.m_planeSweepBoundaryColorSelectionEnabled, "Color selection must default off");
        Require(setup.m_planeSweepBoundaryColorViewCount == 4, "Omitted color camera count must preserve the four-view search");
        setup.m_planeSweepCoarseToFineEnabled = true;
        setup.m_planeSweepCrossViewRelativeDepthTolerance = 0.01f;
        setup.m_planeSweepCrossViewReprojectionTolerancePixels = 1.0f;
        setup.m_planeSweepCrossViewCount = 4;
        std::vector<Sweep::PlaneSweepDepthMap> maps{ Map(0), Map(0.25f), Map(-0.25f), Map(0.5f), Map(-0.5f) };
        const std::array<uint8_t, 3> brown{ 60, 45, 25 }, outlier{ 0, 190, 0 };
        std::vector<Sweep::PlaneSweepImage> images;
        for (size_t i = 0; i < maps.size(); ++i) images.push_back(Image(maps[i], i == 4 ? outlier : brown, i == 0));
        size_t loads = 0;
        const auto loader = [&](size_t index, Sweep::PlaneSweepImage &image)
        { ++loads; image = images.at(index); return true; };
        auto points = Points();
        SelectPlaneSweepBoundaryColors(setup, maps, points, loader);
        Require(loads == 0, "Disabled selection loaded images"); Unchanged(points);

        setup.m_planeSweepBoundaryColorSelectionEnabled = true;
        const auto stats = SelectPlaneSweepBoundaryColors(setup, maps, points, loader);
        Require(stats.m_boundaryPoints == 1 && stats.m_recoloredPoints == 1, "Boundary point was not repaired");
        Require(points.size() == 1 && points[0].m_position.z == -5 && points[0].m_color.w == 0.4f,
            "Repair changed geometry, alpha or count");
        Require(points[0].m_color.x == brown[0] / 255.0f && points[0].m_color.y == brown[1] / 255.0f &&
            points[0].m_color.z == brown[2] / 255.0f, "Medoid did not choose an observed consensus color");

        // Color search must not use the geometric camera limit in either direction.
        setup.m_planeSweepCrossViewCount = 1;
        points = Points();
        Require(SelectPlaneSweepBoundaryColors(setup, maps, points, loader).m_recoloredPoints == 1,
            "Geometric camera limit restricted color selection");
        setup.m_planeSweepCrossViewCount = 4;
        setup.m_planeSweepBoundaryColorViewCount = 1;
        points = Points(); SelectPlaneSweepBoundaryColors(setup, maps, points, loader); Unchanged(points);

        // A wider color search can find two reliable views beyond occluded nearest cameras.
        auto fartherSupport = maps;
        for (size_t i = 1; i <= 2; ++i)
            fartherSupport[i].m_depthValues.assign(fartherSupport[i].m_depthValues.size(), 4.0f);
        setup.m_planeSweepBoundaryColorViewCount = 2;
        points = Points(); SelectPlaneSweepBoundaryColors(setup, fartherSupport, points, loader); Unchanged(points);
        setup.m_planeSweepBoundaryColorViewCount = 4;
        points = Points();
        Require(SelectPlaneSweepBoundaryColors(setup, fartherSupport, points, loader).m_recoloredPoints == 1,
            "Color search failed to reach the farther reliable views");

        // A safely interior reference must retain its original RGB even if other cameras differ.
        images[0] = Image(maps[0], brown, false);
        points = Points(); SelectPlaneSweepBoundaryColors(setup, maps, points, loader); Unchanged(points);
        images[0] = Image(maps[0], brown, true);

        // One reliable alternative is insufficient; all other observations are occluded.
        auto occluded = maps;
        for (size_t i = 2; i < occluded.size(); ++i)
            occluded[i].m_depthValues.assign(occluded[i].m_depthValues.size(), 4.0f);
        points = Points(); SelectPlaneSweepBoundaryColors(setup, occluded, points, loader); Unchanged(points);

        // Missing nearest depth must not be replaced by a valid neighboring sample.
        auto holes = maps;
        for (size_t i = 1; i < holes.size(); ++i)
        {
            float x, y, z;
            Require(Sweep::ProjectPoint(holes[i].m_camera, holes[i].m_intrinsic, points[0].m_position, x, y, z),
                "Fixture point did not project into the source");
            const size_t px = size_t(std::lround(x - 0.5f)), py = size_t(std::lround(y - 0.5f));
            holes[i].m_depthValues[py * holes[i].m_width + px] = 0;
        }
        points = Points(); SelectPlaneSweepBoundaryColors(setup, holes, points, loader); Unchanged(points);

        // Foreground-only masks, including their alpha-channel form, are required for reliable colors.
        for (size_t i = 1; i < images.size(); ++i) images[i].m_hasMask = false;
        points = Points(); SelectPlaneSweepBoundaryColors(setup, maps, points, loader); Unchanged(points);
        for (size_t i = 1; i < images.size(); ++i)
        {
            images[i] = Image(maps[i], brown, false);
            images[i].m_maskUsesAlpha = true;
            for (size_t p = 3; p < images[i].m_maskTexture.m_pixels.size(); p += 4)
                images[i].m_maskTexture.m_pixels[p] = 0;
        }
        points = Points(); SelectPlaneSweepBoundaryColors(setup, maps, points, loader); Unchanged(points);

        // The algorithm does not key out green: valid interior green observations remain green.
        for (size_t i = 1; i < images.size(); ++i) images[i] = Image(maps[i], outlier, false);
        points = Points(); SelectPlaneSweepBoundaryColors(setup, maps, points, loader);
        Require(points[0].m_color.y == outlier[1] / 255.0f && points[0].m_color.w == 0.4f,
            "Selection removed a legitimate green consensus or changed alpha");

        // Pixel centers and clearance must work with odd, half-resolution maps as well.
        for (size_t i = 0; i < maps.size(); ++i)
        {
            maps[i] = Map(maps[i].m_camera.m_position.x, 51, 41);
            images[i] = Image(maps[i], brown, i == 0);
        }
        points = Points();
        Require(SelectPlaneSweepBoundaryColors(setup, maps, points, loader).m_recoloredPoints == 1,
            "Odd-sized matching grids prevented a valid repair");
        setup.m_planeSweepCoarseToFineEnabled = false;
        points = Points();
        Require(SelectPlaneSweepBoundaryColors(setup, maps, points, loader).m_recoloredPoints == 1,
            "Legacy sampling coordinates prevented a valid repair");

        points = Points();
        SelectPlaneSweepBoundaryColors(setup, maps, points, [](size_t, Sweep::PlaneSweepImage &) { return false; });
        Unchanged(points);
        points[0].m_depthMapIndex = maps.size();
        SelectPlaneSweepBoundaryColors(setup, maps, points, loader);
        Require(points[0].m_depthMapIndex == maps.size() && points[0].m_color.y == 0.8f, "Invalid reference index was modified");
        std::cout << "Plane sweep boundary color selection tests passed.\n";
        return 0;
    }
    catch (const std::exception &error)
    { std::cerr << error.what() << '\n'; return 1; }
}
