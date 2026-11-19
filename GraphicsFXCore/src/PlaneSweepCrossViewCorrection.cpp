#include "PlaneSweepCrossViewCorrection.h"
#include "PlaneSweepGeometricFusion.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace graphicsFX
{
    namespace
    {
        using Sweep = MultiViewUtility::MultiViewPlaneSweepUtility;
        using Map = Sweep::PlaneSweepDepthMap;
        using Observation = PlaneSweepDepthObservation;
        struct Camera { std::size_t index; float score; };
        constexpr uint32_t NoCell = std::numeric_limits<uint32_t>::max();
        constexpr uint32_t DuplicateCell = NoCell - 1;
        // All neighborhood reads use this immutable per-reference snapshot, never an updated point.
        struct SurfacePoint
        {
            std::size_t point{};
            int x{}, y{};
            float imageX{}, imageY{}, depth{};
            double u{}, v{};
            bool valid{};
        };
        struct PlaneSample { double x{}, y{}, inverse{}, weight{}; int dx{}, dy{}; };
        struct SurfacePlane
        {
            double x{}, y{}, inverse{}, noise{}, effective{};
            bool valid{};
        };
        enum class SurfaceStatus { Missing, Edge, Valid };
        bool Valid(const Map &m)
        {
            return m.m_width && m.m_height && m.m_imageWidth && m.m_imageHeight &&
                m.m_intrinsic.m_fx > 0 && m.m_intrinsic.m_fy > 0 &&
                std::isfinite(m.m_intrinsic.m_fx) && std::isfinite(m.m_intrinsic.m_fy) &&
                m.m_depthValues.size() == std::size_t(m.m_width) * m.m_height;
        }
        float ImageCoordinate(int sample, uint32_t imageSize, uint32_t mapSize, bool pyramid)
        {
            return pyramid ? (float(sample) + 0.5f) * float(imageSize) / float(mapSize) - 0.5f :
                float((std::size_t(sample) * imageSize) / mapSize);
        }
        bool Observe(const ColmapSetup &s, const Map &ref, const Map &src, const Float3 &point,
            float refX, float refY, float refDepth, Observation &observation, bool interpolate = true)
        {
            float px{}, py{}, projectedDepth{};
            if (!Sweep::ProjectPoint(src.m_camera, src.m_intrinsic, point, px, py, projectedDepth)) return false;
            const bool pyramid = s.m_planeSweepCoarseToFineEnabled;
            const float targetX = px - 0.5f, targetY = py - 0.5f;
            if (targetX < -1e-4f || targetY < -1e-4f || targetX >= src.m_imageWidth || targetY >= src.m_imageHeight) return false;
            const float depthTolerance = s.m_planeSweepCrossViewRelativeDepthTolerance;
            const float pixelTolerance = s.m_planeSweepCrossViewReprojectionTolerancePixels;
            const auto check = [&](float depth, float imageX, float imageY, float &returnedDepth, float &score)
            {
                if (!(depth > 0) || !std::isfinite(depth)) return false;
                const float forward = std::fabs(depth - projectedDepth) / projectedDepth;
                if (forward > depthTolerance) return false;
                const auto world = Sweep::UnprojectPixel(src.m_camera, src.m_intrinsic, imageX, imageY, depth);
                float rx{}, ry{};
                if (!Sweep::ProjectPoint(ref.m_camera, ref.m_intrinsic, world, rx, ry, returnedDepth)) return false;
                const float backward = std::fabs(returnedDepth - refDepth) / refDepth;
                if (backward > depthTolerance) return false;
                const float dx = (rx - refX) * float(ref.m_width) / float(ref.m_imageWidth);
                const float dy = (ry - refY) * float(ref.m_height) / float(ref.m_imageHeight);
                const float pixelError = dx * dx + dy * dy;
                if (pixelError > pixelTolerance * pixelTolerance) return false;
                score = (forward * forward + backward * backward) / (depthTolerance * depthTolerance) +
                    pixelError / (pixelTolerance * pixelTolerance);
                return std::isfinite(score) && std::isfinite(returnedDepth);
            };
            // Preserve the filter's discrete neighborhood tests; choose its best passing sample.
            const int centerX = pyramid ? int(std::lround(px * float(src.m_width) / src.m_imageWidth - 0.5f)) :
                int(std::lround(px * float(src.m_width) / src.m_imageWidth));
            const int centerY = pyramid ? int(std::lround(py * float(src.m_height) / src.m_imageHeight - 0.5f)) :
                int(std::lround(py * float(src.m_height) / src.m_imageHeight));
            const int radius = std::max(1, s.m_planeSweepPointStep);
            observation.score = std::numeric_limits<float>::max();
            for (int y = std::max(0, centerY - radius); y <= std::min(int(src.m_height) - 1, centerY + radius); ++y)
                for (int x = std::max(0, centerX - radius); x <= std::min(int(src.m_width) - 1, centerX + radius); ++x)
                {
                    float returned{}, score{};
                    if (check(src.m_depthValues[std::size_t(y) * src.m_width + x],
                        ImageCoordinate(x, src.m_imageWidth, src.m_width, pyramid),
                        ImageCoordinate(y, src.m_imageHeight, src.m_height, pyramid), returned, score) && score < observation.score)
                        observation.score = score;
                }
            if (observation.score == std::numeric_limits<float>::max()) return false;
            if (!interpolate) return true;

            // Correction requires a valid footprint at the actual projection. Never infer a depth
            // from a nearby pixel across a hole. Inverse-depth interpolation preserves pinhole planes.
            observation.usable = InterpolatePlaneSweepDepthObservation(s, ref, src, point, refX, refY, refDepth, observation);
            return true;
        }
        double SmallMedian(std::array<double, 24> values, int count)
        {
            std::sort(values.begin(), values.begin() + count);
            return count % 2 ? values[count / 2] : .5 * (values[count / 2 - 1] + values[count / 2]);
        }
        bool SolvePlane(double matrix[3][4], double result[3])
        {
            const double scale = std::max({ std::fabs(matrix[0][0]), std::fabs(matrix[1][1]), std::fabs(matrix[2][2]) });
            if (!(scale > 0) || !std::isfinite(scale)) return false;
            for (int k = 0; k < 3; ++k)
            {
                int pivot = k;
                for (int i = k + 1; i < 3; ++i)
                    if (std::fabs(matrix[i][k]) > std::fabs(matrix[pivot][k])) pivot = i;
                if (std::fabs(matrix[pivot][k]) <= 1e-6 * scale) return false;
                for (int j = k; j < 4; ++j) std::swap(matrix[k][j], matrix[pivot][j]);
                const double divisor = matrix[k][k];
                for (int j = k; j < 4; ++j) matrix[k][j] /= divisor;
                for (int i = 0; i < 3; ++i)
                {
                    if (i == k) continue;
                    const double factor = matrix[i][k];
                    for (int j = k; j < 4; ++j) matrix[i][j] -= factor * matrix[k][j];
                }
            }
            for (int i = 0; i < 3; ++i)
            {
                result[i] = matrix[i][3];
                if (!std::isfinite(result[i])) return false;
            }
            return true;
        }
        SurfacePlane FitPlane(const std::array<PlaneSample, 24> &samples, int count, int half,
            int minimum, double referenceInverse)
        {
            // Half-window fits distinguish a shared plane from a crease or conflicting orientations.
            std::array<int, 24> selected{};
            std::array<double, 24> weights{}, residuals{};
            int size = 0;
            for (int i = 0; i < count; ++i)
            {
                const auto &p = samples[i];
                if ((half == 1 && p.dx > 0) || (half == 2 && p.dx < 0) ||
                    (half == 3 && p.dy > 0) || (half == 4 && p.dy < 0)) continue;
                selected[size] = i; weights[size] = p.weight; ++size;
            }
            if (size < minimum) return {};
            double coefficients[3]{};
            for (int iteration = 0; iteration < 3; ++iteration)
            {
                double matrix[3][4]{};
                for (int j = 0; j < size; ++j)
                {
                    const auto &p = samples[selected[j]];
                    const double row[3]{ p.x, p.y, 1 };
                    for (int a = 0; a < 3; ++a)
                    {
                        for (int b = 0; b < 3; ++b) matrix[a][b] += weights[j] * row[a] * row[b];
                        matrix[a][3] += weights[j] * row[a] * p.inverse;
                    }
                }
                if (!SolvePlane(matrix, coefficients) || !(coefficients[2] > 0)) return {};
                for (int j = 0; j < size; ++j)
                {
                    const auto &p = samples[selected[j]];
                    residuals[j] = std::fabs(coefficients[0] * p.x + coefficients[1] * p.y + coefficients[2] - p.inverse);
                }
                if (iteration == 2) break;
                const double sigma = std::max(referenceInverse * 1e-6, 1.4826 * SmallMedian(residuals, size));
                for (int j = 0; j < size; ++j)
                    weights[j] = samples[selected[j]].weight * std::min(1.0, 1.5 * sigma / std::max(residuals[j], referenceInverse * 1e-8));
            }
            double sum = 0, sumSquared = 0, error = 0;
            for (int i = 0; i < size; ++i)
            {
                sum += weights[i]; sumSquared += weights[i] * weights[i];
                error += weights[i] * residuals[i] * residuals[i];
            }
            const double effective = sumSquared > 0 ? sum * sum / sumSquared : 0;
            if (!(sum > 0) || effective < (half ? 3.5 : 6.0)) return {};
            return { coefficients[0], coefficients[1], coefficients[2], std::sqrt(error / sum), effective, true };
        }
        double ColorDifference(const Float4 &a, const Float4 &b)
        {
            return std::fabs(double(a.x) - b.x) + std::fabs(double(a.y) - b.y) + std::fabs(double(a.z) - b.z);
        }
        SurfaceStatus LocalSurface(const ColmapSetup &setup, const Map &map,
            const std::vector<Sweep::PlaneSweepPointCandidate> &points, const std::vector<SurfacePoint> &snapshot,
            const std::vector<uint32_t> &grid, uint32_t centerIndex, SurfacePlane &model)
        {
            const auto &center = snapshot[centerIndex];
            if (!center.valid || grid[std::size_t(center.y) * map.m_width + center.x] != centerIndex) return SurfaceStatus::Missing;
            const double colorSigma = std::isfinite(setup.m_planeSweepPatchMatchColorSigma) && setup.m_planeSweepPatchMatchColorSigma > 0 ?
                setup.m_planeSweepPatchMatchColorSigma : .15;
            const double tolerance = setup.m_planeSweepCrossViewRelativeDepthTolerance;
            const double fx = map.m_intrinsic.m_fx * map.m_width / map.m_imageWidth;
            const double fy = map.m_intrinsic.m_fy * map.m_height / map.m_imageHeight;
            if (!(fx > 0 && fy > 0)) return SurfaceStatus::Missing;
            const auto &centerColor = points[center.point].m_color;
            std::array<PlaneSample, 24> samples{};
            std::array<bool, 25> visited{};
            std::array<int, 25> queue{};
            int head = 0, tail = 1, count = 0, left = 0, right = 0, above = 0, below = 0;
            queue[0] = 12; visited[12] = true;
            // Connected support cannot jump over absent filtered pixels to another surface.
            while (head < tail)
            {
                const int cell = queue[head++], dx = cell % 5 - 2, dy = cell / 5 - 2;
                const int x = center.x + dx, y = center.y + dy;
                if (x < 0 || y < 0 || x >= int(map.m_width) || y >= int(map.m_height)) continue;
                const uint32_t index = grid[std::size_t(y) * map.m_width + x];
                if (index >= snapshot.size() || !snapshot[index].valid) continue;
                const auto &p = snapshot[index];
                const double color = ColorDifference(centerColor, points[p.point].m_color);
                if (!std::isfinite(color) || color > 2 * colorSigma ||
                    std::fabs(double(p.depth) - center.depth) > 2 * tolerance * center.depth) continue;
                if (cell != 12)
                {
                    samples[count++] = { (p.u - center.u) * fx, (p.v - center.v) * fy, 1.0 / p.depth,
                        std::exp(-double(dx * dx + dy * dy) / 4.5 - color / colorSigma), dx, dy };
                    left += dx < 0; right += dx > 0; above += dy < 0; below += dy > 0;
                }
                for (int direction = 0; direction < 4; ++direction)
                {
                    const int nx = cell % 5 + (direction == 0 ? -1 : direction == 1 ? 1 : 0);
                    const int ny = cell / 5 + (direction == 2 ? -1 : direction == 3 ? 1 : 0);
                    if (nx < 0 || ny < 0 || nx >= 5 || ny >= 5) continue;
                    const int next = ny * 5 + nx;
                    if (visited[next]) continue;
                    // Rejected cells do not propagate connectivity through a hole or color/depth edge.
                    const int ix = center.x + nx - 2, iy = center.y + ny - 2;
                    if (ix < 0 || iy < 0 || ix >= int(map.m_width) || iy >= int(map.m_height)) continue;
                    const uint32_t neighborIndex = grid[std::size_t(iy) * map.m_width + ix];
                    if (neighborIndex >= snapshot.size() || !snapshot[neighborIndex].valid) continue;
                    const auto &neighbor = snapshot[neighborIndex];
                    if (std::fabs(double(neighbor.depth) - p.depth) > tolerance * center.depth ||
                        ColorDifference(points[neighbor.point].m_color, points[p.point].m_color) > 2 * colorSigma) continue;
                    visited[next] = true; queue[tail++] = next;
                }
            }
            // No one-sided extrapolation or fitting through a narrow strip of points.
            if (count < 8) return SurfaceStatus::Missing;
            if (left < 2 || right < 2 || above < 2 || below < 2) return SurfaceStatus::Edge;
            const double inverse = 1.0 / center.depth;
            model = FitPlane(samples, count, 0, 8, inverse);
            if (!model.valid) return SurfaceStatus::Missing;
            if (model.noise > .5 * tolerance * inverse) return SurfaceStatus::Edge;
            const auto normal = [&](const SurfacePlane &p)
            {
                return std::array<double, 3>{ p.x * fx, p.y * fy, p.inverse - p.x * fx * center.u - p.y * fy * center.v };
            };
            const auto compatible = [&](const SurfacePlane &a, const SurfacePlane &b)
            {
                const auto na = normal(a), nb = normal(b);
                double product = 0, aa = 0, bb = 0;
                for (int i = 0; i < 3; ++i) { product += na[i] * nb[i]; aa += na[i] * na[i]; bb += nb[i] * nb[i]; }
                const double denominator = std::sqrt(aa * bb);
                return denominator > 0 && product / denominator >= .8191520443; // 35 degrees, oriented toward the same camera.
            };
            const std::array<SurfacePlane, 4> halves{
                FitPlane(samples, count, 1, 5, inverse), FitPlane(samples, count, 2, 5, inverse),
                FitPlane(samples, count, 3, 5, inverse), FitPlane(samples, count, 4, 5, inverse) };
            int checkedPairs = 0;
            for (int i = 0; i < 4; i += 2)
            {
                if (!halves[i].valid || !halves[i + 1].valid) continue;
                ++checkedPairs;
                const double uncertainty = std::max(.1 * tolerance * inverse, 3 * model.noise);
                if (!compatible(halves[i], halves[i + 1]) || !compatible(model, halves[i]) || !compatible(model, halves[i + 1]) ||
                    std::fabs(halves[i].inverse - halves[i + 1].inverse) > uncertainty ||
                    std::fabs(model.inverse - .5 * (halves[i].inverse + halves[i + 1].inverse)) > uncertainty)
                    return SurfaceStatus::Edge;
            }
            return checkedPairs ? SurfaceStatus::Valid : SurfaceStatus::Missing;
        }
    }

    PlaneSweepCrossViewCorrectionStats CorrectPlaneSweepCrossViewDepths(const ColmapSetup &setup,
        const std::vector<Map> &maps, std::vector<Sweep::PlaneSweepPointCandidate> &points)
    {
        PlaneSweepCrossViewCorrectionStats stats;
        if (!setup.m_planeSweepCrossViewCorrectionEnabled || !setup.m_planeSweepCrossViewConsistencyEnabled || points.empty()) return stats;
        const float tolerance = setup.m_planeSweepCrossViewRelativeDepthTolerance;
        const float pixels = setup.m_planeSweepCrossViewReprojectionTolerancePixels;
        if (!(tolerance > 0) || !std::isfinite(tolerance) || !(pixels > 0) || !std::isfinite(pixels)) return stats;
        const std::size_t minimum = std::size_t(std::max(1, setup.m_planeSweepCrossViewMinimumViews));
        std::vector<std::vector<Camera>> cameras(maps.size());
        std::vector<bool> ranked(maps.size(), false);
        // Only references with candidates need a ranking. Stable camera IDs resolve distance ties.
        for (const auto &point : points)
        {
            const auto r = point.m_depthMapIndex;
            if (r >= maps.size() || !Valid(maps[r]) || ranked[r]) continue;
            ranked[r] = true;
            std::unordered_set<uint32_t> ids;
            const auto referenceId = maps[r].m_camera.m_imageId;
            if (referenceId) ids.insert(referenceId);
            for (std::size_t i = 0; i < maps.size(); ++i)
            {
                if (i == r || !Valid(maps[i])) continue;
                const auto id = maps[i].m_camera.m_imageId;
                if (id && !ids.insert(id).second) continue;
                const float alignment = std::max(0.0f, dot(maps[r].m_camera.m_forward, maps[i].m_camera.m_forward));
                const float score = length(maps[r].m_camera.m_position - maps[i].m_camera.m_position) / (0.25f + alignment);
                if (std::isfinite(score)) cameras[r].push_back({ i, score });
            }
            std::sort(cameras[r].begin(), cameras[r].end(), [&](const Camera &a, const Camera &b)
            {
                if (a.score != b.score) return a.score < b.score;
                if (maps[a.index].m_camera.m_imageId != maps[b.index].m_camera.m_imageId)
                    return maps[a.index].m_camera.m_imageId < maps[b.index].m_camera.m_imageId;
                return a.index < b.index;
            });
            cameras[r].resize(std::min(cameras[r].size(), std::size_t(std::max(1, setup.m_planeSweepCrossViewCount))));
        }
        std::vector<std::vector<std::size_t>> groups(maps.size());
        stats.m_examined = points.size();
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            const auto r = points[i].m_depthMapIndex;
            if (r < maps.size() && Valid(maps[r])) groups[r].push_back(i);
            else ++stats.m_insufficientSupport;
        }
        std::vector<Observation> before, after;
        std::vector<float> depths;
        for (std::size_t r = 0; r < maps.size(); ++r)
        {
            if (groups[r].empty()) continue;
            if (groups[r].size() >= DuplicateCell)
            { stats.m_insufficientSupport += groups[r].size(); continue; }
            const auto &map = maps[r];
            // Only one organized reference grid is resident at a time. Neighbor samples are
            // frozen before any correction in this reference, avoiding scan-order feedback.
            std::vector<uint32_t> grid(std::size_t(map.m_width) * map.m_height, NoCell);
            std::vector<SurfacePoint> snapshot(groups[r].size());
            for (std::size_t local = 0; local < groups[r].size(); ++local)
            {
                auto &sample = snapshot[local]; sample.point = groups[r][local];
                const auto &point = points[sample.point];
                if (!Sweep::ProjectPoint(map.m_camera, map.m_intrinsic, point.m_position, sample.imageX, sample.imageY, sample.depth) ||
                    !std::isfinite(sample.depth) || !(sample.depth > 0)) { sample.depth = 0; continue; }
                const auto ray = (point.m_position - map.m_camera.m_position) * (1 / sample.depth);
                sample.u = dot(ray, map.m_camera.m_right); sample.v = dot(ray, map.m_camera.m_up);
                if (!std::isfinite(sample.u) || !std::isfinite(sample.v)) { sample.depth = 0; continue; }
                const float gx = setup.m_planeSweepCoarseToFineEnabled ? sample.imageX * map.m_width / map.m_imageWidth - .5f :
                    (sample.imageX - .5f) * map.m_width / map.m_imageWidth;
                const float gy = setup.m_planeSweepCoarseToFineEnabled ? sample.imageY * map.m_height / map.m_imageHeight - .5f :
                    (sample.imageY - .5f) * map.m_height / map.m_imageHeight;
                if (!std::isfinite(gx) || !std::isfinite(gy) || gx < -.5f || gy < -.5f || gx > map.m_width || gy > map.m_height) continue;
                sample.x = int(std::lround(gx)); sample.y = int(std::lround(gy));
                if (sample.x < 0 || sample.y < 0 || sample.x >= int(map.m_width) || sample.y >= int(map.m_height)) continue;
                const float dx = (ImageCoordinate(sample.x, map.m_imageWidth, map.m_width, setup.m_planeSweepCoarseToFineEnabled) + .5f - sample.imageX) * map.m_width / map.m_imageWidth;
                const float dy = (ImageCoordinate(sample.y, map.m_imageHeight, map.m_height, setup.m_planeSweepCoarseToFineEnabled) + .5f - sample.imageY) * map.m_height / map.m_imageHeight;
                if (std::fabs(dx) > .1f || std::fabs(dy) > .1f) continue;
                sample.valid = true;
                auto &cell = grid[std::size_t(sample.y) * map.m_width + sample.x];
                cell = cell == NoCell ? uint32_t(local) : DuplicateCell;
            }
            for (uint32_t local = 0; local < snapshot.size(); ++local)
            {
                const auto &sample = snapshot[local];
                auto &point = points[sample.point];
                const float z = sample.depth, x = sample.imageX, y = sample.imageY;
                if (!(z > 0)) { ++stats.m_insufficientSupport; continue; }
                const auto originalPosition = point.m_position;
                const auto gather = [&](const Float3 &position, float depth, std::vector<Observation> &out, bool interpolate)
                {
                    out.clear();
                    for (const auto &camera : cameras[r])
                    {
                        Observation o; o.camera = camera.index;
                        if (Observe(setup, map, maps[camera.index], position, x, y, depth, o, interpolate)) out.push_back(o);
                    }
                };
                gather(originalPosition, z, before, true);
                if (before.size() < minimum) { ++stats.m_insufficientSupport; continue; }
                SurfacePlane surface;
                const auto surfaceStatus = LocalSurface(setup, map, points, snapshot, grid, local, surface);
                const bool regularize = surfaceStatus == SurfaceStatus::Valid;
                if (regularize) ++stats.m_surfaceSupported;
                else if (surfaceStatus == SurfaceStatus::Edge) ++stats.m_surfaceEdges;
                else ++stats.m_surfaceMissing;

                depths.clear();
                for (const auto &o : before) if (o.usable) depths.push_back(o.depth);
                std::sort(depths.begin(), depths.end());
                bool split = false;
                for (std::size_t i = 2; i + 1 < depths.size(); ++i)
                    if (depths[i] - depths[i - 1] > .25f * tolerance * z) split = true;
                if (split) { ++stats.m_ambiguous; continue; }

                const float epsilon = 16 * std::numeric_limits<float>::epsilon() * z;
                const auto consensusMinimum = regularize ? std::size_t(2) : std::max(std::size_t(2), minimum);
                bool consensus = false;
                float consensusDepth = z;
                if (depths.size() >= consensusMinimum)
                {
                    const auto estimate = EstimatePlaneSweepDepthConsensus(z, before, consensusMinimum);
                    consensus = estimate.valid;
                    if (consensus) consensusDepth = estimate.depth;
                }
                if (!regularize && !consensus)
                { ++stats.m_insufficientSupport; continue; }

                double oldScore = 0;
                for (const auto &o : before) oldScore += o.score;
                oldScore /= before.size();
                // Surface coherence is an explicit objective, not just an individual point's
                // agreement with noisy maps. Hard view/pixel/depth limits still apply to every move.
                const double inverseSigma = regularize ? std::max(surface.noise, .1 * tolerance * surface.inverse) : 1;
                const double surfaceWeight = regularize ? .5 * std::min(1.0, surface.effective / 12) : 0;
                const auto objective = [&](float depth, double score)
                {
                    const double anchor = (double(depth) - z) / (tolerance * z);
                    const double residual = regularize ? (1.0 / depth - surface.inverse) / inverseSigma : 0;
                    return score + .25 * anchor * anchor + surfaceWeight * residual * residual;
                };
                const double oldObjective = objective(z, oldScore);
                double bestObjective = oldObjective, bestScore = oldScore;
                float bestDepth = z;
                std::array<float, 4> targets{};
                int targetCount = 0;
                if (regularize)
                {
                    const float planeDepth = float(1 / surface.inverse);
                    targets[targetCount++] = z + .7f * (planeDepth - z);
                    targets[targetCount++] = z + .35f * (planeDepth - z);
                }
                if (consensus) targets[targetCount++] = z + .5f * (consensusDepth - z);
                if (regularize && consensus)
                    targets[targetCount++] = z + .7f * (float(1 / (.7 * surface.inverse + .3 / consensusDepth)) - z);
                float previousDepth = -1;
                for (int candidate = 0; candidate < targetCount; ++candidate)
                {
                    const float proposed = z + std::clamp(targets[candidate] - z, -.5f * tolerance * z, .5f * tolerance * z);
                    if (!(proposed > 0) || !std::isfinite(proposed) || std::fabs(proposed - z) <= epsilon ||
                        std::fabs(proposed - previousDepth) <= epsilon) continue;
                    previousDepth = proposed;
                    // A surface-mode move must actually approach the fitted surface; a better
                    // raw view score alone must not disturb already coherent local geometry.
                    if (regularize && std::fabs(1.0 / proposed - surface.inverse) >= std::fabs(1.0 / z - surface.inverse)) continue;
                    const auto position = map.m_camera.m_position + (originalPosition - map.m_camera.m_position) * (proposed / z);
                    gather(position, proposed, after, false);
                    double score = 0;
                    bool lostSupport = after.size() < minimum;
                    for (const auto &old : before)
                    {
                        const auto match = std::find_if(after.begin(), after.end(), [&](const Observation &o) { return o.camera == old.camera; });
                        if (match == after.end()) { lostSupport = true; break; }
                        score += match->score;
                    }
                    if (lostSupport) { ++stats.m_proposalsRejected; continue; }
                    score /= before.size();
                    // A bounded geometric-score tradeoff is permitted only with a safe local
                    // surface model. Never loosen the underlying per-camera acceptance limits.
                    if (regularize && score > oldScore + .1) { ++stats.m_proposalsRejected; continue; }
                    const double energy = objective(proposed, score);
                    if (!std::isfinite(energy) || energy + 1e-6 >= bestObjective) continue;
                    bestDepth = proposed; bestScore = score; bestObjective = energy;
                }
                if (bestDepth == z) { ++stats.m_unchanged; continue; }
                point.m_position = map.m_camera.m_position + (originalPosition - map.m_camera.m_position) * (bestDepth / z);
                const double movement = std::fabs(bestDepth - z) / z;
                ++stats.m_corrected; stats.m_relativeMovementSum += movement;
                stats.m_maxRelativeMovement = std::max(stats.m_maxRelativeMovement, movement);
                stats.m_scoreBeforeSum += oldScore; stats.m_scoreAfterSum += bestScore;
                stats.m_objectiveBeforeSum += oldObjective; stats.m_objectiveAfterSum += bestObjective;
                if (regularize)
                {
                    ++stats.m_regularized;
                    stats.m_surfaceErrorBeforeSum += std::fabs(1.0 / z - surface.inverse) / surface.inverse;
                    stats.m_surfaceErrorAfterSum += std::fabs(1.0 / bestDepth - surface.inverse) / surface.inverse;
                }
                else ++stats.m_consensusOnly;
            }
        }
        return stats;
    }
}
