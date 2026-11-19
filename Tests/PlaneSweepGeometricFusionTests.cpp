#include "PlaneSweepGeometricFusion.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace graphicsFX;
using Sweep = MultiViewUtility::MultiViewPlaneSweepUtility;
using Map = Sweep::PlaneSweepDepthMap;
using Point = Sweep::PlaneSweepPointCandidate;

static void Require(bool condition, const char *message)
{ if (!condition) throw std::runtime_error(message); }

static ColmapSetup Settings()
{
    ColmapSetup s;
    s.m_planeSweepFusionMode = PlaneSweepFusionMode::Geometric;
    s.m_planeSweepCoarseToFineEnabled = true;
    s.m_planeSweepCrossViewConsistencyEnabled = true;
    s.m_planeSweepCrossViewCorrectionEnabled = true;
    s.m_planeSweepCrossViewCount = 8;
    s.m_planeSweepCrossViewMinimumViews = 2;
    s.m_planeSweepCrossViewRelativeDepthTolerance = .01f;
    s.m_planeSweepCrossViewReprojectionTolerancePixels = .5f;
    s.m_planeSweepPointStep = 1;
    s.m_planeSweepOutlierFilterEnabled = false;
    return s;
}

static Map DepthMap(float cameraX = 0, float depth = 5, uint32_t width = 101, uint32_t height = 81)
{
    Map m;
    m.m_camera.m_position.x = cameraX;
    m.m_intrinsic.m_width = m.m_imageWidth = 101;
    m.m_intrinsic.m_height = m.m_imageHeight = 81;
    m.m_intrinsic.m_fx = m.m_intrinsic.m_fy = 200;
    m.m_intrinsic.m_cx = 50.5;
    m.m_intrinsic.m_cy = 40.5;
    m.m_width = width; m.m_height = height;
    m.m_depthValues.assign(std::size_t(width) * height, depth);
    return m;
}

static Point At(const std::vector<Map> &maps, std::size_t camera, uint32_t x, uint32_t y,
    bool pyramid = true, Float4 color = { .2f, .4f, .6f, .8f })
{
    const auto &m = maps[camera];
    const float imageX = pyramid ? (float(x) + .5f) * m.m_imageWidth / m.m_width - .5f
        : float((std::size_t(x) * m.m_imageWidth) / m.m_width);
    const float imageY = pyramid ? (float(y) + .5f) * m.m_imageHeight / m.m_height - .5f
        : float((std::size_t(y) * m.m_imageHeight) / m.m_height);
    const auto pixel = y * m.m_width + x;
    return { camera, Sweep::UnprojectPixel(m.m_camera, m.m_intrinsic, imageX, imageY, m.m_depthValues[pixel]), color, pixel };
}

static Map SlantedMap(float cameraX)
{
    auto map = DepthMap(cameraX);
    // Analytic plane world Z = -5 + .2 * world X, with non-integer source projections.
    for (uint32_t y = 0; y < map.m_height; ++y)
        for (uint32_t x = 0; x < map.m_width; ++x)
        {
            const float rayX = (float(x) - 50) / 200;
            map.m_depthValues[std::size_t(y) * map.m_width + x] = (5 - .2f * cameraX) / (1 + .2f * rayX);
        }
    return map;
}

static ColmapPlaneSweepResult Run(const ColmapSetup &s, const std::vector<Map> &maps, std::vector<Point> points)
{
    ColmapPlaneSweepResult result;
    Sweep::ApplyCrossViewConsistency(s, maps, points, result);
    Sweep::BuildFilteredPoints(s, points, result, &maps);
    return result;
}

int main(int argc, char **argv)
{
    try
    {
        auto s = Settings();
        std::vector<Map> maps{ DepthMap(0, 5.02f), DepthMap(.25f), DepthMap(-.25f) };
        std::vector<Point> points{ At(maps, 0, 50, 40), At(maps, 1, 40, 40), At(maps, 2, 60, 40) };
        const auto originalMaps = maps;
        auto filtered = points;
        ColmapPlaneSweepResult validation;
        Sweep::ApplyCrossViewConsistency(s, maps, filtered, validation);
        Require(filtered.size() == 3 && filtered[0].m_position.z == points[0].m_position.z,
            "Geometric mode must validate without also running standalone correction");
        auto result = Run(s, maps, points);
        Require(result.m_pointsBeforeFiltering == 3 && result.m_pointsAfterFusion == 1 && result.m_points.size() == 1,
            "Three corresponding observations were not consolidated into one point");
        Require(std::fabs(result.m_points[0].m_position.z + 5) < .02f,
            "Robust consensus did not improve the noisy seed depth");
        Require(result.m_points[0].m_position.x == points[0].m_position.x &&
            result.m_points[0].m_position.y == points[0].m_position.y, "Fusion left the reference camera ray");
        Require(result.m_points[0].m_color.x == points[0].m_color.x && result.m_points[0].m_color.w == points[0].m_color.w,
            "Fusion changed the selected seed RGBA");
        for (std::size_t i = 0; i < maps.size(); ++i)
            Require(maps[i].m_depthValues == originalMaps[i].m_depthValues, "Fusion mutated an input depth map");

        const auto noisyMaps = maps;
        const auto noisyPoints = points;
        maps = { SlantedMap(0), SlantedMap(.26f), SlantedMap(-.24f) };
        result = Run(s, maps, { At(maps, 0, 50, 40) });
        Require(result.m_points.size() == 1 && std::fabs(result.m_points[0].m_position.z + 5) < 2e-6f,
            "Discrete source sampling biased an already-correct slanted plane");

        // A nearby discrete sample can validate a point without safely providing an interpolated depth.
        maps = { DepthMap(), DepthMap(.26f, 0), DepthMap(-.24f, 0) };
        maps[1].m_depthValues[40 * 101 + 40] = 5.02f;
        maps[2].m_depthValues[40 * 101 + 60] = 5.02f;
        result = Run(s, maps, { At(maps, 0, 50, 40) });
        Require(result.m_points.size() == 1 && result.m_points[0].m_position.z == -5,
            "Fusion inferred a depth across a missing source footprint");
        maps = noisyMaps; points = noisyPoints;

        s.m_planeSweepCrossViewConsistencyEnabled = false;
        Require(Run(s, maps, points).m_points.size() == 1, "Geometric mode failed to enable its own validation");
        s.m_planeSweepCrossViewMinimumViews = 3;
        result = Run(s, maps, points);
        Require(result.m_points.empty() && result.m_pointsRejectedCrossView == 3,
            "Minimum views counted the reference camera as support");
        s = Settings();
        Require(Run(s, { maps[0], maps[1] }, { points[0], points[1], points[1] }).m_points.empty(),
            "Repeated pixels from one camera counted as independent cameras");

        // Adjacent samples belong to different groups even on a perfectly flat surface.
        maps = { DepthMap(), DepthMap(.25f), DepthMap(-.25f) };
        points = { At(maps, 0, 50, 40), At(maps, 0, 51, 40), At(maps, 1, 40, 40),
            At(maps, 1, 41, 40), At(maps, 2, 60, 40), At(maps, 2, 61, 40) };
        result = Run(s, maps, points);
        Require(result.m_points.size() == 2, "Adjacent pixels collapsed together or retained duplicate observations");
        Require(length(result.m_points[0].m_position - result.m_points[1].m_position) > .02f,
            "Fusion erased neighboring surface detail");

        // Claimed support may still validate a different seed, but cannot be averaged twice.
        s.m_planeSweepCrossViewReprojectionTolerancePixels = 1.1f;
        maps = { DepthMap(), DepthMap(.25f, 0), DepthMap(-.25f, 0) };
        maps[1].m_depthValues[40 * 101 + 40] = 5;
        maps[2].m_depthValues[40 * 101 + 60] = 5;
        points = { At(maps, 0, 50, 40), At(maps, 0, 51, 40), At(maps, 1, 40, 40), At(maps, 2, 60, 40) };
        result = Run(s, maps, points);
        Require(result.m_pointsBeforeFiltering == 4 && result.m_points.size() == 2,
            "Observation ownership rejected a separately validated seed or emitted duplicate support");

        // Avoid averaging two depth populations even when both fit the hard geometric limits.
        s = Settings();
        maps = { DepthMap(), DepthMap(0, 4.99f), DepthMap(0, 4.99f), DepthMap(0, 5.01f), DepthMap(0, 5.01f) };
        PlaneSweepGeometricFusionStats stats;
        const auto ambiguous = FusePlaneSweepGeometricPoints(s, maps, { At(maps, 0, 50, 40) }, stats);
        Require(ambiguous.size() == 1 && stats.ambiguous == 1 && ambiguous[0].m_position.z == -5,
            "Fusion averaged an ambiguous pair of surface layers");

        // Distinct depth layers must never be combined by spatial proximity alone.
        s.m_planeSweepCrossViewMinimumViews = 1;
        maps = { DepthMap(0, 5), DepthMap(.25f, 5), DepthMap(0, 6), DepthMap(.3f, 6) };
        points = { At(maps, 0, 50, 40), At(maps, 1, 40, 40), At(maps, 2, 50, 40), At(maps, 3, 40, 40) };
        result = Run(s, maps, points);
        Require(result.m_points.size() == 2, "Independent surface layers were merged");
        Require(std::fabs(result.m_points[0].m_position.z + 5) < 1e-5f &&
            std::fabs(result.m_points[1].m_position.z + 6) < 1e-5f, "Fusion averaged across an occlusion");

        // A nearby depth alone cannot pass the existing round-trip pixel requirement.
        maps = { DepthMap(), DepthMap(1, 0) };
        maps[1].m_depthValues[40 * 101 + 10] = 4.85f;
        s.m_planeSweepCrossViewRelativeDepthTolerance = .05f;
        s.m_planeSweepCrossViewReprojectionTolerancePixels = 1;
        Require(Run(s, maps, { At(maps, 0, 50, 40) }).m_points.empty(), "Fusion rescued a false depth-only match");
        s.m_planeSweepCrossViewReprojectionTolerancePixels = 1.5f;
        Require(Run(s, maps, { At(maps, 0, 50, 40) }).m_points.size() == 1, "Configured pixel tolerance was ignored");

        // Odd/unequal resolutions and legacy integer sampling retain the existing coordinate convention.
        for (const bool pyramid : { true, false })
        {
            s = Settings();
            s.m_planeSweepCoarseToFineEnabled = pyramid;
            s.m_planeSweepCrossViewMinimumViews = 1;
            s.m_planeSweepCrossViewReprojectionTolerancePixels = .001f;
            maps = { DepthMap(), DepthMap(0, 5, 51, 41) };
            const float x = pyramid ? 20.5f * 101 / 51 - .5f : float((20 * 101) / 51);
            const float y = pyramid ? 20.5f * 81 / 41 - .5f : float((20 * 81) / 41);
            maps[1].m_camera.m_position = { (50 - x) * 5 / 200, (y - 40) * 5 / 200, 0 };
            result = Run(s, maps, { At(maps, 0, 50, 40, pyramid), At(maps, 1, 20, 20, pyramid) });
            Require(result.m_points.size() == 1, "Scaled/legacy pixel coordinates produced duplicate or missing geometry");
        }

        // Consensus must resist a depth outlier rather than average all passing observations equally.
        std::vector<PlaneSweepDepthObservation> votes{ { 5, 0, true }, { 5, 0, true }, { 5.045f, 0, true } };
        const auto consensus = EstimatePlaneSweepDepthConsensus(5.01f, votes, 2);
        Require(consensus.valid && consensus.depth < 5.005f, "Depth consensus was biased by an outlier");

        // Missing/malformed/nonfinite depths never provide supporting observations.
        s = Settings(); s.m_planeSweepCrossViewMinimumViews = 1;
        for (float invalid : { 0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity() })
        {
            maps = { DepthMap(), DepthMap(.25f, invalid) };
            Require(Run(s, maps, { At(maps, 0, 50, 40) }).m_points.empty(), "Invalid depth supported a fused point");
        }
        maps[1].m_depthValues.pop_back();
        Require(Run(s, maps, { At(maps, 0, 50, 40) }).m_points.empty(), "Malformed depth map supported a fused point");

        // Explicit modes override the old boolean; omitted modes keep old programmatic behavior.
        s = Settings(); s.m_planeSweepCrossViewConsistencyEnabled = false;
        s.m_planeSweepCrossViewCorrectionEnabled = false;
        maps = { DepthMap() };
        points = { At(maps, 0, 50, 40), At(maps, 0, 51, 40) };
        s.m_planeSweepPointFusionEnabled = true;
        s.m_planeSweepFusionMode = PlaneSweepFusionMode::None;
        Require(Run(s, maps, points).m_points.size() == 2, "Explicit None mode still used voxel fusion");
        s.m_planeSweepFusionMode = PlaneSweepFusionMode::Voxel;
        s.m_planeSweepPointFusionEnabled = false;
        s.m_planeSweepFusionVoxelSize = 1;
        s.m_planeSweepFusionMinimumObservations = 2;
        Require(Run(s, maps, points).m_points.size() == 1, "Explicit Voxel mode did not override the old boolean");
        s.m_planeSweepFusionMode = PlaneSweepFusionMode::Legacy;
        Require(Run(s, maps, points).m_points.size() == 2, "Omitted selector ignored the disabled legacy boolean");
        s.m_planeSweepPointFusionEnabled = true;
        Require(Run(s, maps, points).m_points.size() == 1, "Omitted selector changed legacy voxel behavior");

        if (argc > 1)
        {
            const auto folder = std::filesystem::path(argv[1]);
            std::filesystem::create_directories(folder);
            const auto file = folder / "fusion.ini";
            const auto parse = [&](const char *body)
            {
                { std::ofstream out(file); out << "[COLMAP]\n" << body; }
                return LoadProgramSetup(file.string()).m_colmap;
            };
            Require(parse("PlaneSweepPointFusionEnabled=false\n").EffectiveFusionMode() == PlaneSweepFusionMode::None,
                "Legacy false config no longer disables fusion");
            Require(parse("").EffectiveFusionMode() == PlaneSweepFusionMode::Voxel, "Omitted selector changed the old default");
            Require(parse("PlaneSweepFusionMode=gEoMeTrIc\nPlaneSweepPointFusionEnabled=false\n").EffectiveFusionMode()
                == PlaneSweepFusionMode::Geometric, "Geometric mode parsing/precedence failed");
            Require(parse("PlaneSweepFusionMode=None\nPlaneSweepPointFusionEnabled=true\n").EffectiveFusionMode()
                == PlaneSweepFusionMode::None, "None mode parsing failed");
            Require(parse("PlaneSweepFusionMode=Voxel\nPlaneSweepPointFusionEnabled=false\n").EffectiveFusionMode()
                == PlaneSweepFusionMode::Voxel, "Voxel mode parsing failed");
            bool rejected = false;
            try { parse("PlaneSweepFusionMode=Invalid\n"); } catch (const std::invalid_argument &) { rejected = true; }
            Require(rejected, "Invalid mode silently selected a fusion algorithm");
        }
        std::cout << "Plane sweep geometric fusion tests passed.\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
