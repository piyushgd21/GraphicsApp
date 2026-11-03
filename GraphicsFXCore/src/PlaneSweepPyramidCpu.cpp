#include "PlaneSweepPyramid.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace graphicsFX
{
    SweepUtility::PlaneSweepDepthMap RunPlaneSweepPyramidCpu(const SweepUtility::PlaneSweepImage &reference,
        const std::vector<SweepUtility::PlaneSweepImage> &sources, const ColmapSetup &setup,
        const SweepUtility::PlaneSweepDepthRange &range, std::vector<PlaneSweepPyramidStats> *statistics)
    {
        const bool slanted = setup.m_planeSweepMatchingMode == PlaneSweepMatchingMode::PatchMatchSlanted;
        if (slanted) ValidatePlaneSweepSlantedSetup(setup);
        if (!(range.m_nearDepth > 0 && range.m_farDepth > range.m_nearDepth) || setup.m_planeSweepPlaneCount < 2)
            throw std::invalid_argument("Invalid pyramid depth range or plane count.");
        const auto start = std::chrono::steady_clock::now();
        const auto pyramid = BuildPlaneSweepPyramid(reference, sources, setup);
        // Keep sweep seeds provisional until weighted slanted matching evaluates this level.
        ColmapSetup searchSetup = setup;
        if (slanted)
        {
            searchSetup.m_planeSweepConfidenceEnabled = false;
            searchSetup.m_planeSweepTextureRejectionEnabled = false;
            searchSetup.m_planeSweepCostThreshold = 10.0f;
        }
        const float nearInv = 1.0f / range.m_nearDepth, farInv = 1.0f / range.m_farDepth;
        const float baseSpacing = (nearInv - farInv) / float(setup.m_planeSweepPlaneCount - 1);
        const float maximum = std::numeric_limits<float>::max();
        const auto confidenceThreshold = [&](float spacing)
        // Local samples are closer together; scale the gap threshold by their
        // squared spacing ratio instead of applying the coarse uniqueness gap unchanged.
        { float ratio = std::min(1.0f, spacing / baseSpacing); return searchSetup.m_planeSweepConfidenceMinimumDifference * ratio * ratio; };
        std::vector<PlaneSweepPyramidState> parents;
        std::vector<Float4> parentPlanes;
        uint32_t parentWidth = 0, parentHeight = 0;
        std::vector<PlaneSweepPyramidStats> stats;
        for (size_t l = 0; l < pyramid.size(); ++l)
        {
            const auto &level = pyramid[l];
            const uint32_t width = level.m_reference.m_texture.m_width, height = level.m_reference.m_texture.m_height;
            const bool finalLevel = l + 1 == pyramid.size();
            PlaneSweepPyramidStats counters; counters.m_width = width; counters.m_height = height;
            std::vector<PlaneSweepPyramidState> output(size_t(width) * height, { 0, maximum, 0, baseSpacing });
            for (uint32_t y = 0; y < height; ++y)
                for (uint32_t x = 0; x < width; ++x)
                {
                    Float4 color;
                    if (!SweepUtility::SamplePlaneSweepPixel(level.m_reference, float(x), float(y), color) || color.w <= setup.m_planeSweepAlphaThreshold) continue;
                    float deviation;
                    if (searchSetup.m_planeSweepTextureRejectionEnabled &&
                        (!SweepUtility::ComputeTextureStandardDeviation(level.m_reference, float(x), float(y), setup.m_planeSweepPatchRadius,
                            setup.m_planeSweepAlphaThreshold, deviation) || deviation < setup.m_planeSweepTextureMinimumStdDev))
                    { ++counters.m_textureRejected; continue; }
                    PlaneSweepPyramidState best{ 0, maximum, maximum, baseSpacing };
                    float secondInverse = -1;
                    auto evaluate = [&](float inverse)
                    {
                        ++counters.m_evaluations;
                        float cost;
                        if (!SweepUtility::ComputePlaneCost(level.m_reference, level.m_sources, float(x), float(y), 1.0f / inverse,
                            setup.m_planeSweepPatchRadius, std::max(1, std::min(int(sources.size()), setup.m_planeSweepMinValidSourceViews)),
                            setup.m_planeSweepAlphaThreshold, setup.m_planeSweepMinimumOpaquePatchRatio,
                            setup.m_planeSweepPatchCostMethod, setup.m_planeSweepADCensusWeight, cost, true)) return;
                        cost = std::floor(cost * 1e6f + 0.5f) / 1e6f;
                        const float epsilon = std::max(1e-8f, baseSpacing * 1e-4f);
                        if (best[0] > 0 && std::fabs(inverse - best[0]) <= epsilon) return;
                        if (cost < best[1]) { best[2] = best[1]; secondInverse = best[0]; best[0] = inverse; best[1] = cost; }
                        else if (cost < best[2] && std::fabs(inverse - secondInverse) > epsilon)
                        { best[2] = cost; secondInverse = inverse; }
                    };
                    auto search = [&](float center, float radius, bool global)
                    {
                        const int count = global ? setup.m_planeSweepPlaneCount : setup.m_planeSweepLocalSamples;
                        const float spacing = global ? baseSpacing : 2.0f * radius / float(count - 1);
                        float last = -1;
                        for (int i = 0; i < count; ++i)
                        {
                            const float t = float(i) / float(count - 1);
                            const float candidate = global ? nearInv * (1.0f - t) + farInv * t :
                                std::clamp(center + float(i - count / 2) * spacing, farInv, nearInv);
                            if (candidate == last) continue;
                            last = candidate;
                            const float previous = best[0]; evaluate(candidate);
                            if (best[0] != previous) best[3] = spacing;
                        }
                        if (best[0] > 0) best[3] = std::min(best[3], spacing);
                        const float epsilon = std::max(1e-8f, spacing * 1e-3f);
                        return best[0] > 0 && best[0] > farInv + epsilon && best[0] < nearInv - epsilon &&
                            (best[0] <= center - radius + epsilon || best[0] >= center + radius - epsilon);
                    };
                    bool global = parents.empty(), fallback = false;
                    PlaneSweepPyramidState parent{};
                    if (!global)
                    {
                        const uint32_t px = std::min(uint32_t((float(x) + 0.5f) * float(parentWidth) / float(width)), parentWidth - 1);
                        const uint32_t py = std::min(uint32_t((float(y) + 0.5f) * float(parentHeight) / float(height)), parentHeight - 1);
                        parent = parents[size_t(py) * parentWidth + px];
                        global = parent[0] <= 0 || parent[1] > searchSetup.m_planeSweepCostThreshold ||
                            (searchSetup.m_planeSweepConfidenceEnabled && parent[2] < confidenceThreshold(parent[3]));
                        for (int dy = -1; dy <= 1; ++dy)
                            for (int dx = -1; dx <= 1; ++dx)
                            {
                                const int qx = int(px) + dx, qy = int(py) + dy;
                                if (qx < 0 || qy < 0 || qx >= int(parentWidth) || qy >= int(parentHeight)) continue;
                                const float other = parents[size_t(qy) * parentWidth + size_t(qx)][0];
                                if (parent[0] > 0 && other > 0 && std::fabs(1.0f / other - 1.0f / parent[0]) >
                                    4 * parent[3] + std::max(1e-8f, baseSpacing * 1e-3f)) global = true;
                            }
                    }
                    const auto unreliable = [&](bool boundary)
                    { return best[0] <= 0 || best[1] > searchSetup.m_planeSweepCostThreshold || boundary ||
                        (searchSetup.m_planeSweepConfidenceEnabled && best[2] - best[1] < confidenceThreshold(best[3])); };
                    if (global)
                    {
                        fallback = !parents.empty(); search(0, 0, true);
                    }
                    else
                    {
                        const float center = 1.0f / parent[0], radius = 2 * parent[3];
                        if (unreliable(search(center, radius, false)))
                        {
                            ++counters.m_widened;
                            if (unreliable(search(center, radius * 4, false))) { fallback = true; search(0, 0, true); }
                        }
                    }
                    if (fallback)
                    {
                        ++counters.m_globalFallbacks;
                        if (best[0] > 0) search(best[0], 2 * baseSpacing, false);
                    }
                    if (best[0] > 0)
                    {
                        auto &pixel = output[size_t(y) * width + x];
                        pixel = { 1 / best[0], best[1], best[2] < maximum ? best[2] - best[1] : 0, best[3] };
                        const bool accepted = best[1] <= searchSetup.m_planeSweepCostThreshold &&
                            (!searchSetup.m_planeSweepConfidenceEnabled || pixel[2] >= confidenceThreshold(pixel[3]));
                        if (accepted) ++counters.m_accepted;
                        else
                        {
                            if (best[1] <= searchSetup.m_planeSweepCostThreshold) ++counters.m_confidenceRejected;
                            if (finalLevel) pixel[0] = 0;
                        }
                    }
                }
            if (slanted)
                parentPlanes = RefinePlaneSweepSlantedLevelCpu(level, setup, range, output, parentPlanes, parentWidth, parentHeight, finalLevel, counters);
            if (setup.m_planeSweepSavePyramidDepth)
            {
                ColmapSetup saved = setup; saved.m_planeSweepSaveDepth = true;
                saved.m_planeSweepOutputFolder += "/Pyramid/Level" + std::to_string(l);
                auto map = MakePlaneSweepPyramidDepthMap(reference, width, height, output);
                SweepUtility::SaveDepthOutput(saved, reference.m_camera, map.m_depthValues, width, height);
            }
            stats.push_back(counters); parents = std::move(output); parentWidth = width; parentHeight = height;
        }
        stats.back().m_milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        if (statistics) *statistics = std::move(stats);
        return MakePlaneSweepPyramidDepthMap(reference, parentWidth, parentHeight, parents);
    }
}
