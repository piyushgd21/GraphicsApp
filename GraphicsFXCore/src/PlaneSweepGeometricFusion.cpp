#include "PlaneSweepGeometricFusion.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace graphicsFX
{
    namespace
    {
        using Sweep = MultiViewUtility::MultiViewPlaneSweepUtility;
        using Map = Sweep::PlaneSweepDepthMap;
        using Point = Sweep::PlaneSweepPointCandidate;

        float ImageCoordinate(int sample, uint32_t imageSize, uint32_t mapSize, bool pyramid)
        {
            return pyramid ? (float(sample) + .5f) * float(imageSize) / float(mapSize) - .5f
                : float((std::size_t(sample) * imageSize) / mapSize);
        }

        float Median(std::vector<float> &values)
        {
            std::sort(values.begin(), values.end());
            const auto n = values.size();
            return n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) * .5f;
        }

        bool InsideProjection(const ColmapSetup &setup, const Map &map, float x, float y)
        {
            if (setup.m_planeSweepCoarseToFineEnabled) { x -= .5f; y -= .5f; }
            return x >= 0 && y >= 0 && x < map.m_imageWidth && y < map.m_imageHeight;
        }

        bool CheckSample(const ColmapSetup &setup, const Map &reference, const Map &source,
            float referenceX, float referenceY, float referenceDepth,
            float projectedDepth, float depth, float imageX, float imageY, PlaneSweepDepthObservation &observation)
        {
            if (!(depth > 0) || !std::isfinite(depth)) return false;
            const float depthTolerance = std::max(0.0f, setup.m_planeSweepCrossViewRelativeDepthTolerance);
            const float pixelTolerance = std::max(0.0f, setup.m_planeSweepCrossViewReprojectionTolerancePixels);
            const float forward = std::fabs(projectedDepth - depth) / projectedDepth;
            if (forward > depthTolerance) return false;
            const auto world = Sweep::UnprojectPixel(source.m_camera, source.m_intrinsic, imageX, imageY, depth);
            float returnedX{}, returnedY{}, returnedDepth{};
            if (!Sweep::ProjectPoint(reference.m_camera, reference.m_intrinsic, world,
                returnedX, returnedY, returnedDepth)) return false;
            const float backward = std::fabs(returnedDepth - referenceDepth) / referenceDepth;
            if (backward > depthTolerance) return false;
            const float scaleX = float(reference.m_width) / float(reference.m_imageWidth);
            const float scaleY = float(reference.m_height) / float(reference.m_imageHeight);
            const float dx = (returnedX - referenceX) * scaleX;
            const float dy = (returnedY - referenceY) * scaleY;
            const float pixelError = dx * dx + dy * dy;
            if (pixelError > pixelTolerance * pixelTolerance) return false;
            // Zero tolerances still accept exact matches, without dividing by zero in the score.
            const double d = std::max(double(depthTolerance), 1e-12);
            const double p = std::max(double(pixelTolerance), 1e-12);
            const double score = (double(forward) * forward + double(backward) * backward) / (d * d)
                + double(pixelError) / (p * p);
            if (!std::isfinite(score) || score > std::numeric_limits<float>::max()) return false;
            observation.depth = returnedDepth;
            observation.score = float(score);
            observation.usable = true;
            return true;
        }

        bool CheckObservation(const ColmapSetup &setup, const Map &reference, const Map &source,
            float referenceX, float referenceY, float referenceDepth,
            float projectedDepth, uint32_t pixel, PlaneSweepDepthObservation &observation)
        {
            if (pixel >= source.m_depthValues.size()) return false;
            const float imageX = ImageCoordinate(int(pixel % source.m_width), source.m_imageWidth,
                source.m_width, setup.m_planeSweepCoarseToFineEnabled);
            const float imageY = ImageCoordinate(int(pixel / source.m_width), source.m_imageHeight,
                source.m_height, setup.m_planeSweepCoarseToFineEnabled);
            if (!CheckSample(setup, reference, source, referenceX, referenceY, referenceDepth,
                projectedDepth, source.m_depthValues[pixel], imageX, imageY, observation)) return false;
            observation.pixel = pixel;
            return true;
        }

        bool Bracket(float coordinate, uint32_t imageSize, uint32_t mapSize, bool pyramid,
            int &a, int &b, float &weight)
        {
            const auto at = [&](int i) { return ImageCoordinate(i, imageSize, mapSize, pyramid); };
            if (coordinate < at(0) - 1e-4f || coordinate > at(int(mapSize) - 1) + 1e-4f) return false;
            a = std::clamp(int(std::floor(coordinate * float(mapSize) / float(imageSize))), 0, int(mapSize) - 1);
            while (a > 0 && at(a) > coordinate + 1e-4f) --a;
            while (a + 1 < int(mapSize) && at(a + 1) <= coordinate + 1e-4f) ++a;
            if (std::fabs(coordinate - at(a)) <= 1e-4f) { b = a; weight = 0; return true; }
            b = a + 1;
            if (b >= int(mapSize) || at(b) <= at(a)) return false;
            weight = std::clamp((coordinate - at(a)) / (at(b) - at(a)), 0.0f, 1.0f);
            return true;
        }
    }

    bool ValidPlaneSweepDepthMap(const Map &map)
    {
        return map.m_width && map.m_height && map.m_imageWidth && map.m_imageHeight &&
            map.m_intrinsic.m_fx > 0 && map.m_intrinsic.m_fy > 0 &&
            std::isfinite(map.m_intrinsic.m_fx) && std::isfinite(map.m_intrinsic.m_fy) &&
            map.m_depthValues.size() == std::size_t(map.m_width) * map.m_height &&
            map.m_depthValues.size() < UINT32_MAX;
    }

    bool FindPlaneSweepDepthObservation(const ColmapSetup &setup, const Map &reference, const Map &source,
        const Float3 &point, float referenceX, float referenceY, float referenceDepth,
        PlaneSweepDepthObservation &observation, bool selectBest)
    {
        if (!ValidPlaneSweepDepthMap(reference) || !ValidPlaneSweepDepthMap(source) ||
            !(referenceDepth > 0) || !std::isfinite(referenceDepth)) return false;
        float x{}, y{}, projectedDepth{};
        if (!Sweep::ProjectPoint(source.m_camera, source.m_intrinsic, point, x, y, projectedDepth)) return false;
        if (!InsideProjection(setup, source, x, y)) return false;
        const bool pyramid = setup.m_planeSweepCoarseToFineEnabled;
        if (pyramid) { x -= .5f; y -= .5f; }
        const int centerX = pyramid ? int(std::lround((x + .5f) * source.m_width / source.m_imageWidth - .5f))
            : int(std::lround((x / float(source.m_imageWidth)) * source.m_width));
        const int centerY = pyramid ? int(std::lround((y + .5f) * source.m_height / source.m_imageHeight - .5f))
            : int(std::lround((y / float(source.m_imageHeight)) * source.m_height));
        const int radius = std::max(1, setup.m_planeSweepPointStep);
        bool found = false;
        for (int sampleY = std::max(0, centerY - radius); sampleY <= std::min(int(source.m_height) - 1, centerY + radius); ++sampleY)
            for (int sampleX = std::max(0, centerX - radius); sampleX <= std::min(int(source.m_width) - 1, centerX + radius); ++sampleX)
            {
                PlaneSweepDepthObservation candidate;
                const uint32_t pixel = uint32_t(sampleY) * source.m_width + uint32_t(sampleX);
                if (CheckObservation(setup, reference, source, referenceX, referenceY, referenceDepth,
                    projectedDepth, pixel, candidate) && (!found || candidate.score < observation.score))
                {
                    observation = candidate;
                    found = true;
                    if (!selectBest) return true;
                }
            }
        return found;
    }

    bool InterpolatePlaneSweepDepthObservation(const ColmapSetup &setup, const Map &reference, const Map &source,
        const Float3 &point, float referenceX, float referenceY, float referenceDepth,
        PlaneSweepDepthObservation &observation)
    {
        if (!ValidPlaneSweepDepthMap(reference) || !ValidPlaneSweepDepthMap(source)) return false;
        float px{}, py{}, projectedDepth{};
        if (!Sweep::ProjectPoint(source.m_camera, source.m_intrinsic, point, px, py, projectedDepth)) return false;
        const float targetX = px - .5f, targetY = py - .5f;
        const float tolerance = std::max(0.0f, setup.m_planeSweepCrossViewRelativeDepthTolerance);
        int x0{}, x1{}, y0{}, y1{}; float wx{}, wy{};
        if (!Bracket(targetX, source.m_imageWidth, source.m_width, setup.m_planeSweepCoarseToFineEnabled, x0, x1, wx) ||
            !Bracket(targetY, source.m_imageHeight, source.m_height, setup.m_planeSweepCoarseToFineEnabled, y0, y1, wy)) return false;
        float inverse = 0, low = std::numeric_limits<float>::max(), high = 0;
        for (int j = 0; j < 2; ++j)
            for (int i = 0; i < 2; ++i)
            {
                const float weight = (i ? wx : 1 - wx) * (j ? wy : 1 - wy);
                if (weight <= 0) continue;
                const float depth = source.m_depthValues[std::size_t(j ? y1 : y0) * source.m_width + (i ? x1 : x0)];
                if (!(depth > 0) || !std::isfinite(depth) || std::fabs(depth - projectedDepth) / projectedDepth > tolerance) return false;
                inverse += weight / depth;
                low = std::min(low, depth); high = std::max(high, depth);
            }
        if (!(inverse > 0) || high - low > .5f * tolerance * projectedDepth) return false;
        PlaneSweepDepthObservation interpolated;
        if (!CheckSample(setup, reference, source, referenceX, referenceY, referenceDepth, projectedDepth,
            1 / inverse, targetX, targetY, interpolated)) return false;
        observation.depth = interpolated.depth;
        observation.usable = true;
        return true;
    }

    std::vector<std::vector<std::size_t>> PlaneSweepComparisonViews(const ColmapSetup &setup, const std::vector<Map> &maps)
    {
        std::vector<std::vector<std::size_t>> neighbors(maps.size());
        for (std::size_t r = 0; r < maps.size(); ++r)
        {
            if (!ValidPlaneSweepDepthMap(maps[r])) continue;
            std::vector<Sweep::PlaneSweepSourceCandidate> ranking;
            for (std::size_t s = 0; s < maps.size(); ++s)
            {
                if (s == r || !ValidPlaneSweepDepthMap(maps[s])) continue;
                const float distance = length(maps[s].m_camera.m_position - maps[r].m_camera.m_position);
                const float alignment = std::max(0.0f, dot(maps[r].m_camera.m_forward, maps[s].m_camera.m_forward));
                ranking.push_back({ s, 0, distance / (.25f + alignment) });
            }
            std::sort(ranking.begin(), ranking.end(), [](const auto &a, const auto &b) { return a.m_score < b.m_score; });
            const auto count = std::min(ranking.size(), std::size_t(std::max(1, setup.m_planeSweepCrossViewCount)));
            for (std::size_t i = 0; i < count; ++i) neighbors[r].push_back(ranking[i].m_cameraIndex);
        }
        return neighbors;
    }

    PlaneSweepDepthConsensus EstimatePlaneSweepDepthConsensus(float referenceDepth,
        const std::vector<PlaneSweepDepthObservation> &observations, std::size_t minimum)
    {
        PlaneSweepDepthConsensus result;
        if (!(referenceDepth > 0) || !std::isfinite(referenceDepth)) return result;
        std::vector<float> depths;
        depths.reserve(observations.size());
        for (const auto &o : observations)
            if (o.usable && o.depth > 0 && std::isfinite(o.depth) && std::isfinite(o.score)) depths.push_back(o.depth);
        if (depths.empty() || depths.size() < minimum) return result;
        result.center = Median(depths);
        for (float &depth : depths) depth = std::fabs(depth - result.center);
        const float epsilon = 16 * std::numeric_limits<float>::epsilon() * referenceDepth;
        result.scale = std::max(epsilon, 1.4826f * Median(depths));
        double sum = referenceDepth, weightSum = 1;
        std::size_t inliers = 0;
        for (const auto &o : observations)
        {
            const float residual = std::fabs(o.depth - result.center);
            if (!o.usable || !(o.depth > 0) || !std::isfinite(o.depth) || !std::isfinite(o.score) ||
                o.score < 0 || residual > 3 * result.scale) continue;
            const double weight = std::min(1.0, double(result.scale) / std::max(double(residual), double(epsilon))) / (1.0 + o.score);
            sum += weight * o.depth;
            weightSum += weight;
            ++inliers;
        }
        result.depth = float(sum / weightSum);
        result.valid = inliers >= minimum && result.depth > 0 && std::isfinite(result.depth);
        return result;
    }

    std::vector<SceneObjectPointCloudPoint> FusePlaneSweepGeometricPoints(const ColmapSetup &setup, const std::vector<Map> &maps,
        const std::vector<Point> &points, PlaneSweepGeometricFusionStats &statistics)
    {
        statistics = {};
        // One ownership bit per depth pixel, not an all-pairs point graph. Claims are committed only for accepted groups.
        std::vector<std::vector<uint64_t>> owned(maps.size());
        const auto neighbors = PlaneSweepComparisonViews(setup, maps);
        for (std::size_t r = 0; r < maps.size(); ++r)
        {
            if (!ValidPlaneSweepDepthMap(maps[r])) continue;
            owned[r].resize((maps[r].m_depthValues.size() + 63) / 64);
        }
        const auto claimed = [&](std::size_t camera, uint32_t pixel)
        { return (owned[camera][pixel / 64] & (uint64_t(1) << (pixel % 64))) != 0; };
        const auto claim = [&](std::size_t camera, uint32_t pixel)
        { owned[camera][pixel / 64] |= uint64_t(1) << (pixel % 64); };
        const auto minimum = std::size_t(std::max(1, setup.m_planeSweepCrossViewMinimumViews));
        std::vector<SceneObjectPointCloudPoint> output;
        output.reserve(points.size() / std::max(std::size_t(1), std::min(minimum + 1, maps.size())) + 1);
        std::vector<PlaneSweepDepthObservation> observations, members;
        std::vector<float> depths;
        observations.reserve(std::size_t(std::max(1, setup.m_planeSweepCrossViewCount)));
        members.reserve(observations.capacity());
        for (const auto &seed : points)
        {
            const auto r = seed.m_depthMapIndex;
            if (r >= maps.size() || !ValidPlaneSweepDepthMap(maps[r])) continue;
            const auto &reference = maps[r];
            float x{}, y{}, z{};
            if (!Sweep::ProjectPoint(reference.m_camera, reference.m_intrinsic, seed.m_position, x, y, z)) continue;
            uint32_t pixel = seed.m_depthPixelIndex;
            if (pixel == UINT32_MAX)
            {
                // Compatibility for callers constructing candidates without a native pixel index.
                PlaneSweepDepthObservation self;
                if (!FindPlaneSweepDepthObservation(setup, reference, reference, seed.m_position, x, y, z, self)) continue;
                pixel = self.pixel;
            }
            if (pixel >= reference.m_depthValues.size()) continue;
            if (claimed(r, pixel)) { ++statistics.merged; continue; }
            observations.clear();
            for (const auto camera : neighbors[r])
            {
                PlaneSweepDepthObservation o;
                if (FindPlaneSweepDepthObservation(setup, reference, maps[camera], seed.m_position, x, y, z, o) &&
                    !claimed(camera, o.pixel))
                {
                    o.camera = camera;
                    o.usable = InterpolatePlaneSweepDepthObservation(setup, reference, maps[camera], seed.m_position, x, y, z, o);
                    observations.push_back(o);
                }
            }
            // The standalone correction pass also avoids two separated depth populations.
            // A gap is ambiguous only with at least two observations on either side.
            depths.clear();
            for (const auto &o : observations) if (o.usable) depths.push_back(o.depth);
            std::sort(depths.begin(), depths.end());
            bool split = false;
            for (std::size_t i = 2; i + 1 < depths.size(); ++i)
                if (depths[i] - depths[i - 1] > .25f * setup.m_planeSweepCrossViewRelativeDepthTolerance * z) split = true;
            if (split) ++statistics.ambiguous;
            const auto consensus = split ? PlaneSweepDepthConsensus{} : EstimatePlaneSweepDepthConsensus(z, observations, minimum);
            Point fused = seed;
            fused.m_depthPixelIndex = pixel;
            members.clear();
            if (consensus.valid)
            {
                // Preserve the correction pass's half-tolerance movement bound and the seed's camera ray.
                const float limit = .5f * std::max(0.0f, setup.m_planeSweepCrossViewRelativeDepthTolerance) * z;
                float depth = z + std::clamp(consensus.depth - z, -limit, limit);
                for (int attempt = 0; attempt < 2; ++attempt)
                {
                    const auto position = reference.m_camera.m_position + (seed.m_position - reference.m_camera.m_position) * (depth / z);
                    members.clear();
                    for (const auto &o : observations)
                    {
                        if (!o.usable || std::fabs(o.depth - consensus.center) > 3 * consensus.scale) continue;
                        float sx{}, sy{}, sz{};
                        PlaneSweepDepthObservation checked;
                        if (Sweep::ProjectPoint(maps[o.camera].m_camera, maps[o.camera].m_intrinsic, position, sx, sy, sz) &&
                            InsideProjection(setup, maps[o.camera], sx, sy) &&
                            CheckObservation(setup, reference, maps[o.camera], x, y, depth, sz, o.pixel, checked))
                            members.push_back(o);
                    }
                    if (members.size() >= minimum) { fused.m_position = position; break; }
                    depth = z; // A failed move can still consolidate observations valid at the original point.
                }
            }
            claim(r, pixel);
            if (members.size() >= minimum)
            {
                for (const auto &member : members) claim(member.camera, member.pixel);
                if (length(fused.m_position - seed.m_position) > 0) ++statistics.moved;
            }
            else ++statistics.unmerged; // Ownership must never turn a previously supported seed into a hole.
            // Retain the selected seed RGBA, including any preceding boundary color selection.
            output.push_back({ fused.m_position, fused.m_color });
        }
        return output;
    }
}
