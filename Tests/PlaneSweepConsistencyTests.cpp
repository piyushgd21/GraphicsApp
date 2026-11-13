#include "MultiViewUtility.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace graphicsFX;
using Sweep = MultiViewUtility::MultiViewPlaneSweepUtility;

static void Require(bool condition, const char *message)
{ if (!condition) throw std::runtime_error(message); }

static Sweep::PlaneSweepDepthMap Map(float cameraX = 0.0f, uint32_t width = 101, uint32_t height = 81)
{
    Sweep::PlaneSweepDepthMap map;
    map.m_camera.m_position.x = cameraX;
    map.m_intrinsic.m_width = map.m_imageWidth = 101;
    map.m_intrinsic.m_height = map.m_imageHeight = 81;
    map.m_intrinsic.m_fx = map.m_intrinsic.m_fy = 200;
    map.m_intrinsic.m_cx = 50.5; map.m_intrinsic.m_cy = 40.5;
    map.m_width = width; map.m_height = height;
    map.m_depthValues.assign(size_t(width) * height, 5.0f);
    return map;
}

static ColmapSetup Settings()
{
    ColmapSetup setup;
    setup.m_planeSweepCoarseToFineEnabled = true;
    setup.m_planeSweepCrossViewConsistencyEnabled = true;
    setup.m_planeSweepCrossViewCount = 4;
    setup.m_planeSweepCrossViewMinimumViews = 1;
    setup.m_planeSweepCrossViewRelativeDepthTolerance = 0.01f;
    setup.m_planeSweepCrossViewReprojectionTolerancePixels = 1.0f;
    setup.m_planeSweepPointStep = 1;
    return setup;
}

static bool Kept(const std::vector<Sweep::PlaneSweepDepthMap> &maps, const ColmapSetup &setup,
    Float3 point = { 0.0f, 0.0f, -5.0f }, size_t referenceIndex = 0)
{
    const Float4 color{ 0.2f, 0.4f, 0.6f, 0.8f };
    std::vector<Sweep::PlaneSweepPointCandidate> points{ { referenceIndex, point, color } };
    ColmapPlaneSweepResult result;
    Sweep::ApplyCrossViewConsistency(setup, maps, points, result);
    Require(result.m_pointsRejectedCrossView == (points.empty() ? 1U : 0U), "Rejection counter differs from survivors");
    if (!points.empty())
    {
        Require(points[0].m_position.x == point.x && points[0].m_position.y == point.y && points[0].m_position.z == point.z,
            "Consistency filtering must not move a surviving point");
        Require(points[0].m_color.x == color.x && points[0].m_color.w == color.w, "Consistency filtering changed point color");
    }
    return !points.empty();
}

int main(int argc, char **argv)
{
    try
    {
        auto setup = Settings();
        const auto reference = Map();
        const auto source = Map(1.0f);
        setup.m_planeSweepCrossViewReprojectionTolerancePixels = 0.001f;
        Require(Kept({ reference, source }, setup), "Exact translated-camera correspondence was rejected");

        // A 3% depth error passes the old 5% test, but returns over one pixel away.
        auto wrongDepth = source;
        wrongDepth.m_depthValues.assign(wrongDepth.m_depthValues.size(), 0.0f);
        wrongDepth.m_depthValues[40U * wrongDepth.m_width + 10U] = 4.85f;
        setup.m_planeSweepCrossViewRelativeDepthTolerance = 0.05f;
        setup.m_planeSweepCrossViewReprojectionTolerancePixels = 1.0f;
        Require(!Kept({ reference, wrongDepth }, setup), "Depth-only false agreement survived the round-trip check");
        setup.m_planeSweepCrossViewReprojectionTolerancePixels = 1.5f;
        Require(Kept({ reference, wrongDepth }, setup), "Configured reprojection threshold was not applied");
        setup.m_planeSweepCrossViewRelativeDepthTolerance = 0.01f;
        setup.m_planeSweepCrossViewReprojectionTolerancePixels = 100.0f;
        Require(!Kept({ reference, wrongDepth }, setup), "Pixel tolerance bypassed the depth threshold");

        // Odd, differently sized maps must use pixel centers, not image corners.
        setup = Settings();
        setup.m_planeSweepCrossViewReprojectionTolerancePixels = 0.001f;
        auto resized = Map(0.0f, 51, 41);
        const float sampleX = (20.0f + 0.5f) * 101.0f / 51.0f - 0.5f;
        resized.m_camera.m_position.x = (50.0f - sampleX) * 5.0f / 200.0f;
        Require(Kept({ reference, resized }, setup), "Odd-sized pyramid map has a half-pixel alignment error");

        // The original sweep instead samples integer image coordinates after scaling.
        setup.m_planeSweepCoarseToFineEnabled = false;
        setup.m_planeSweepPointStep = 2;
        resized.m_camera.m_position.x = (50.0f - float((20U * 101U) / 51U)) * 5.0f / 200.0f;
        // Legacy floor sampling must align both axes: row 20 maps to image row 39, not 40.
        resized.m_camera.m_position.y = (float((20U * 81U) / 41U) - 40.0f) * 5.0f / 200.0f;
        Require(Kept({ reference, resized }, setup), "Legacy sweep coordinates were interpreted as pyramid centers");

        // Pixel tolerance is measured at the reference depth-map resolution.
        setup = Settings();
        setup.m_planeSweepCrossViewRelativeDepthTolerance = 0.05f;
        Require(Kept({ Map(0.0f, 51, 41), wrongDepth }, setup), "Reprojection tolerance was measured in full-image pixels");

        // A distorted, rotated view of the analytic world plane Z=-5.
        setup = Settings();
        auto rotated = Map(0.25f, 51, 41);
        const float yaw = 0.07f;
        rotated.m_camera.m_forward = { std::sin(yaw), 0.0f, -std::cos(yaw) };
        rotated.m_camera.m_right = { std::cos(yaw), 0.0f, std::sin(yaw) };
        rotated.m_intrinsic.m_k1 = 0.03; rotated.m_intrinsic.m_p1 = 0.001;
        for (uint32_t y = 0; y < rotated.m_height; ++y)
            for (uint32_t x = 0; x < rotated.m_width; ++x)
            {
                const float imageX = (float(x) + 0.5f) * 101.0f / float(rotated.m_width) - 0.5f;
                const float imageY = (float(y) + 0.5f) * 81.0f / float(rotated.m_height) - 0.5f;
                const Float3 ray = Sweep::UnprojectPixel(rotated.m_camera, rotated.m_intrinsic, imageX, imageY, 1.0f) - rotated.m_camera.m_position;
                rotated.m_depthValues[size_t(y) * rotated.m_width + x] = -5.0f / ray.z;
            }
        Require(Kept({ reference, rotated }, setup), "Rotated, distorted consistent plane was rejected");

        for (float invalid : { 0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity() })
        {
            auto missing = source;
            missing.m_depthValues.assign(missing.m_depthValues.size(), invalid);
            Require(!Kept({ reference, missing }, setup), "Invalid supporting depth was accepted");
        }
        auto malformed = source;
        malformed.m_depthValues.pop_back();
        Require(!Kept({ reference, malformed }, setup), "Malformed depth map was accepted");
        Require(!Kept({ reference, Map(100.0f) }, setup), "Out-of-view source supported a point");
        Require(!Kept({ reference, source }, setup, { 0.0f, 0.0f, 5.0f }), "Point behind the camera was accepted");
        Require(!Kept({ reference, source }, setup, { 0.0f, 0.0f, -5.0f }, 9), "Invalid reference index was accepted");
        Require(!Kept({ reference }, setup), "Enabled consistency kept a point without another depth map");
        setup.m_planeSweepCrossViewMinimumViews = 2;
        Require(!Kept({ reference, source }, setup), "Multiple neighbors in one view counted as multiple views");
        Require(Kept({ reference, source, Map(-1.0f) }, setup), "Two independent supporting views were rejected");
        setup.m_planeSweepCrossViewMinimumViews = 3;
        Require(!Kept({ reference, source, Map(-1.0f) }, setup), "Insufficient view count was accepted");
        setup.m_planeSweepCrossViewConsistencyEnabled = false;
        Require(Kept({}, setup), "Disabled consistency changed the candidates");

        // Use CTest's build-directory path so parser fixtures never overwrite repository settings.
        if (argc > 1)
        {
            const std::filesystem::path folder(argv[1]);
            std::filesystem::create_directories(folder);
            const auto file = folder / "round_trip.ini";
            {
                std::ofstream out(file);
                Require(bool(out), "Cannot create parser fixture");
                out << "[COLMAP]\nPlaneSweepCrossViewReprojectionTolerancePixels=0.75\n"
                    << "PlaneSweepCrossViewRelativeDepthTolerance=0.015\n"
                    << "PlaneSweepCrossViewCorrectionEnabled=true\n"
                    << "PlaneSweepBoundaryColorSelectionEnabled=true\n"
                    << "PlaneSweepCrossViewCount=4\nPlaneSweepBoundaryColorViewCount=16\n";
            }
            const auto parsed = LoadProgramSetup(file.string()).m_colmap;
            Require(parsed.m_planeSweepCrossViewReprojectionTolerancePixels == 0.75f, "Pixel threshold was not parsed");
            Require(parsed.m_planeSweepCrossViewRelativeDepthTolerance == 0.015f, "Depth threshold was not parsed");
            Require(parsed.m_planeSweepBoundaryColorSelectionEnabled, "Boundary color switch was not parsed");
            Require(parsed.m_planeSweepCrossViewCorrectionEnabled, "Depth correction switch was not parsed");
            Require(parsed.m_planeSweepBoundaryColorViewCount == 16 && parsed.m_planeSweepCrossViewCount == 4,
                "Color and geometric camera limits were not parsed independently");
            {
                std::ofstream out(file);
                out << "[COLMAP]\nPlaneSweepBoundaryColorSelectionEnabled=false\n";
            }
            Require(!LoadProgramSetup(file.string()).m_colmap.m_planeSweepBoundaryColorSelectionEnabled,
                "Boundary color switch could not be disabled");
            Require(LoadProgramSetup(file.string()).m_colmap.m_planeSweepBoundaryColorViewCount == 4,
                "Omitted color camera count did not preserve the previous limit");
            Require(!LoadProgramSetup(file.string()).m_colmap.m_planeSweepCrossViewCorrectionEnabled,
                "Omitted depth correction switch must default to false");
        }
        std::cout << "Plane sweep round-trip consistency tests passed.\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
