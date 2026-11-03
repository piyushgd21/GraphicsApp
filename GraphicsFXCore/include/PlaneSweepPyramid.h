#pragma once

#include "MultiViewUtility.h"
#include <array>

namespace graphicsFX
{
    using SweepUtility = MultiViewUtility::MultiViewPlaneSweepUtility;

    struct PlaneSweepPyramidLevel
    {
        SweepUtility::PlaneSweepImage m_reference;
        std::vector<SweepUtility::PlaneSweepImage> m_sources;
    };

    struct PlaneSweepPyramidStats
    {
        uint32_t m_width{ 0 }, m_height{ 0 };
        uint64_t m_evaluations{ 0 }, m_widened{ 0 }, m_globalFallbacks{ 0 }, m_accepted{ 0 };
        uint64_t m_textureRejected{ 0 }, m_confidenceRejected{ 0 };
        double m_milliseconds{ 0.0 };
    };

    // RGBA32F-compatible working state: provisional depth, cost, cost gap, inverse-depth spacing.
    using PlaneSweepPyramidState = std::array<float, 4>;
    struct PlaneSweepSlantedDiagnostics
    {
        std::vector<Float4> m_planes; //Winning k.xyz and weighted cost; before final rejection.
        std::vector<float> m_costs; //Final per-pixel costs, including rejected pixels.
    };
    static_assert(sizeof(PlaneSweepPyramidState) == 16, "Pyramid state must match GLSL vec4.");

    std::vector<PlaneSweepPyramidLevel> BuildPlaneSweepPyramid(
        const SweepUtility::PlaneSweepImage &reference,
        const std::vector<SweepUtility::PlaneSweepImage> &sources, const ColmapSetup &setup);

    SweepUtility::PlaneSweepDepthMap MakePlaneSweepPyramidDepthMap(
        const SweepUtility::PlaneSweepImage &reference, uint32_t width, uint32_t height,
        const std::vector<PlaneSweepPyramidState> &state);

    // CPU reference uses the same candidate schedule as the GPU, for small parity checks.
    SweepUtility::PlaneSweepDepthMap RunPlaneSweepPyramidCpu(
        const SweepUtility::PlaneSweepImage &reference,
        const std::vector<SweepUtility::PlaneSweepImage> &sources, const ColmapSetup &setup,
        const SweepUtility::PlaneSweepDepthRange &range,
        std::vector<PlaneSweepPyramidStats> *statistics = nullptr);

    // Refines one level and returns world-space planes for initialization at the next resolution.
    std::vector<Float4> RefinePlaneSweepSlantedLevelCpu(
        const PlaneSweepPyramidLevel &level, const ColmapSetup &setup,
        const SweepUtility::PlaneSweepDepthRange &range, std::vector<PlaneSweepPyramidState> &state,
        const std::vector<Float4> &parentPlanes, uint32_t parentWidth, uint32_t parentHeight,
        bool finalLevel, PlaneSweepPyramidStats &statistics);

    // Explicit optional matching mode, with tilted-plane refinement at every pyramid level.
    void ValidatePlaneSweepSlantedSetup(const ColmapSetup &setup);
    SweepUtility::PlaneSweepDepthMap RunPlaneSweepSlantedCpu(
        const SweepUtility::PlaneSweepImage &reference,
        const std::vector<SweepUtility::PlaneSweepImage> &sources, const ColmapSetup &setup,
        const SweepUtility::PlaneSweepDepthRange &range,
        std::vector<PlaneSweepPyramidStats> *statistics = nullptr);
}
