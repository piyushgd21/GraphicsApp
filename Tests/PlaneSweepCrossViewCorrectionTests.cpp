#include "PlaneSweepCrossViewCorrection.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace graphicsFX;
using Sweep = MultiViewUtility::MultiViewPlaneSweepUtility;
using Map = Sweep::PlaneSweepDepthMap;
using Point = Sweep::PlaneSweepPointCandidate;

static void Require(bool value, const char *message)
{ if (!value) throw std::runtime_error(message); }
static Map Plane(uint32_t id, float cameraX, bool slanted = false, bool distorted = false, bool odd = false)
{
    Map m;
    m.m_camera.m_imageId = id;
    m.m_camera.m_position = { cameraX, 0, 0 };
    m.m_imageWidth = m.m_intrinsic.m_width = 101;
    m.m_imageHeight = m.m_intrinsic.m_height = 81;
    m.m_intrinsic.m_fx = m.m_intrinsic.m_fy = 200;
    m.m_intrinsic.m_cx = 50.5; m.m_intrinsic.m_cy = 40.5;
    m.m_width = odd ? 51 : 101; m.m_height = odd ? 41 : 81;
    if (distorted)
    {
        const float yaw = 0.07f;
        m.m_camera.m_forward = { std::sin(yaw), 0, -std::cos(yaw) };
        m.m_camera.m_right = { std::cos(yaw), 0, std::sin(yaw) };
        m.m_intrinsic.m_k1 = .03; m.m_intrinsic.m_p1 = .001;
    }
    m.m_depthValues.resize(std::size_t(m.m_width) * m.m_height);
    const Float3 normal = slanted ? Float3{ .12f, -.08f, 1 } : Float3{ 0, 0, 1 };
    for (uint32_t y = 0; y < m.m_height; ++y)
        for (uint32_t x = 0; x < m.m_width; ++x)
        {
            const float ix = (float(x) + .5f) * m.m_imageWidth / m.m_width - .5f;
            const float iy = (float(y) + .5f) * m.m_imageHeight / m.m_height - .5f;
            const auto ray = Sweep::UnprojectPixel(m.m_camera, m.m_intrinsic, ix, iy, 1) - m.m_camera.m_position;
            m.m_depthValues[std::size_t(y) * m.m_width + x] = (-5 - dot(normal, m.m_camera.m_position)) / dot(normal, ray);
        }
    return m;
}
static ColmapSetup Settings()
{
    ColmapSetup s;
    s.m_planeSweepCrossViewCorrectionEnabled = true;
    s.m_planeSweepCoarseToFineEnabled = true;
    s.m_planeSweepCrossViewConsistencyEnabled = true;
    s.m_planeSweepCrossViewCount = 8;
    s.m_planeSweepCrossViewMinimumViews = 2;
    s.m_planeSweepCrossViewRelativeDepthTolerance = .01f;
    s.m_planeSweepCrossViewReprojectionTolerancePixels = 1;
    s.m_planeSweepPointStep = 1;
    return s;
}
static Point Candidate(float z = 5, std::size_t index = 0)
{ return { index, { 0, 0, -z }, { .2f, .4f, .6f, .8f } }; }
static void Same(const Point &a, const Point &b)
{
    Require(a.m_position.x == b.m_position.x && a.m_position.y == b.m_position.y && a.m_position.z == b.m_position.z,
        "Unexpected position change");
    Require(a.m_color.x == b.m_color.x && a.m_color.y == b.m_color.y && a.m_color.z == b.m_color.z && a.m_color.w == b.m_color.w,
        "Correction changed RGBA");
}
int main()
{
    try
    {
        auto s = Settings();
        std::vector<Map> maps{ Plane(1, 0), Plane(2, .05f), Plane(3, -.05f), Plane(4, .1f), Plane(5, -.1f) };
        const auto originalMaps = maps;
        std::vector<Point> points{ Candidate(5.02f) };
        auto disabled = s; disabled.m_planeSweepCrossViewCorrectionEnabled = false;
        auto copy = points;
        Require(CorrectPlaneSweepCrossViewDepths(disabled, maps, copy).m_examined == 0, "Disabled correction ran");
        Same(copy[0], points[0]);
        disabled = s; disabled.m_planeSweepCrossViewConsistencyEnabled = false;
        CorrectPlaneSweepCrossViewDepths(disabled, maps, copy); Same(copy[0], points[0]);

        const auto stats = CorrectPlaneSweepCrossViewDepths(s, maps, points);
        Require(stats.m_corrected == 1, "Noisy plane was not corrected");
        Require(std::fabs(points[0].m_position.z + 5) < .02f, "Correction failed to reduce known depth error");
        Require(stats.MeanScoreAfter() < stats.MeanScoreBefore(), "Correction did not improve agreement");
        Require(stats.m_maxRelativeMovement <= .005001, "Correction exceeded movement bound");
        Require(points[0].m_position.x == 0 && points[0].m_position.y == 0, "Correction moved sideways off the reference ray");
        Require(points[0].m_color.w == .8f && points[0].m_color.x == .2f, "Color changed");
        for (std::size_t i = 0; i < maps.size(); ++i)
            Require(maps[i].m_depthValues == originalMaps[i].m_depthValues, "Correction mutated an input map");
        auto integrated = std::vector<Point>{ Candidate(5.02f), Candidate(8) };
        ColmapPlaneSweepResult result;
        Sweep::ApplyCrossViewConsistency(s, maps, integrated, result);
        Require(integrated.size() == 1 && result.m_pointsRejectedCrossView == 1, "Correction rescued a rejected point or lost a valid point");
        Require(std::fabs(integrated[0].m_position.z - points[0].m_position.z) < 1e-6f, "Production integration differs");

        for (bool slanted : { false, true })
        {
            for (bool odd : { false, true })
            {
                auto analytic = std::vector<Map>{ Plane(1, 0, slanted), Plane(2, .05f, slanted, true, odd),
                    Plane(3, -.05f, slanted, true, odd), Plane(4, .1f, slanted, false, odd) };
                auto exact = std::vector<Point>{ Candidate() };
                CorrectPlaneSweepCrossViewDepths(s, analytic, exact);
                Require(std::fabs(exact[0].m_position.z + 5) < 2e-5f, "Exact analytic plane was degraded");
                exact = { Candidate(5.015f) };
                const auto improved = CorrectPlaneSweepCrossViewDepths(s, analytic, exact);
                Require(improved.m_corrected == 1 && std::fabs(exact[0].m_position.z + 5) < .015f,
                    "Rotated/distorted/resized plane error did not decrease");
            }
        }
        auto legacy = s; legacy.m_planeSweepCoarseToFineEnabled = false;
        copy = { Candidate(5.02f) };
        Require(CorrectPlaneSweepCrossViewDepths(legacy, maps, copy).m_corrected == 1, "Legacy pixel coordinates failed");

        auto outlierMaps = maps;
        outlierMaps[4].m_depthValues.assign(outlierMaps[4].m_depthValues.size(), 5.045f);
        copy = { Candidate(5.02f) };
        Require(CorrectPlaneSweepCrossViewDepths(s, outlierMaps, copy).m_corrected == 1 &&
            std::fabs(copy[0].m_position.z + 5) < .02f, "Outlier pulled correction away from true plane");
        auto splitMaps = maps;
        for (std::size_t i = 1; i < splitMaps.size(); ++i)
            splitMaps[i].m_depthValues.assign(splitMaps[i].m_depthValues.size(), i < 3 ? 4.98f : 5.02f);
        copy = { Candidate() };
        Require(CorrectPlaneSweepCrossViewDepths(s, splitMaps, copy).m_ambiguous == 1, "Two depth layers were combined");
        Same(copy[0], Candidate());
        for (float invalid : { 0.f, -1.f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity() })
        {
            auto missing = maps;
            for (std::size_t i = 1; i < missing.size(); ++i) missing[i].m_depthValues.assign(missing[i].m_depthValues.size(), invalid);
            copy = { Candidate(5.02f) };
            CorrectPlaneSweepCrossViewDepths(s, missing, copy); Same(copy[0], Candidate(5.02f));
        }
        auto duplicate = std::vector<Map>{ maps[0], maps[1], maps[1] };
        copy = { Candidate(5.02f) };
        Require(CorrectPlaneSweepCrossViewDepths(s, duplicate, copy).m_corrected == 0, "One source camera counted twice");
        auto boundary = std::vector<Map>{ Plane(1, 0), Plane(2, .055f), Plane(3, -.055f) };
        for (std::size_t i = 1; i < boundary.size(); ++i)
            for (uint32_t y = 0; y < boundary[i].m_height; ++y)
                for (uint32_t x = 0; x < boundary[i].m_width; ++x)
                    boundary[i].m_depthValues[std::size_t(y) * boundary[i].m_width + x] = x % 2 ? 4.98f : 5.02f;
        copy = { Candidate(5.01f) };
        CorrectPlaneSweepCrossViewDepths(s, boundary, copy); Same(copy[0], Candidate(5.01f));

        // Nearby valid samples may validate a point, but must not supply correction across a hole.
        auto holes = std::vector<Map>{ Plane(1, 0), Plane(2, .055f), Plane(3, -.055f) };
        for (uint32_t y = 0; y < holes[1].m_height; ++y)
        {
            holes[1].m_depthValues[std::size_t(y) * holes[1].m_width + 47] = 0;
            holes[2].m_depthValues[std::size_t(y) * holes[2].m_width + 53] = 0;
        }
        copy = { Candidate(5.01f) };
        ColmapPlaneSweepResult holeResult;
        Sweep::ApplyCrossViewConsistency(s, holes, copy, holeResult);
        Require(copy.size() == 1, "Discrete supporting samples should preserve the original filtered point");
        Same(copy[0], Candidate(5.01f));

        auto reorderedMaps = maps;
        std::reverse(reorderedMaps.begin() + 1, reorderedMaps.end());
        copy = { Candidate(5.02f) };
        CorrectPlaneSweepCrossViewDepths(s, reorderedMaps, copy);
        Same(copy[0], points[0]);
        auto batch = std::vector<Point>{ Candidate(5.02f), Candidate(5.01f) };
        auto reverse = batch; std::reverse(reverse.begin(), reverse.end());
        CorrectPlaneSweepCrossViewDepths(s, maps, batch); CorrectPlaneSweepCrossViewDepths(s, maps, reverse);
        Same(batch[0], reverse[1]); Same(batch[1], reverse[0]);
        auto strict = s; strict.m_planeSweepCrossViewRelativeDepthTolerance = 0;
        copy = { Candidate(5.02f) }; CorrectPlaneSweepCrossViewDepths(strict, maps, copy); Same(copy[0], Candidate(5.02f));
        std::cout << "Cross-view correction: known-plane accuracy, boundaries, invalid support, disabled path and determinism passed.\n";
        return 0;
    }
    catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
