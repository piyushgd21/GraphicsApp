#pragma once

#include "MultiViewUtility.h"

namespace graphicsFX
{
    struct PlaneSweepDepthObservation
    {
        float depth{}, score{};
        bool usable{};
        std::size_t camera{};
        uint32_t pixel{ UINT32_MAX };
    };

    struct PlaneSweepDepthConsensus
    {
        float depth{}, center{}, scale{};
        bool valid{};
    };

    // The same camera-Z median/MAD estimate is used by standalone correction and fusion.
    PlaneSweepDepthConsensus EstimatePlaneSweepDepthConsensus(float referenceDepth,
        const std::vector<PlaneSweepDepthObservation> &observations, std::size_t minimum);

    bool ValidPlaneSweepDepthMap(const MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepDepthMap &map);

    std::vector<std::vector<std::size_t>> PlaneSweepComparisonViews(const ColmapSetup &setup,
        const std::vector<MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepDepthMap> &maps);

    // Returns the best discrete sample satisfying the existing forward/return depth and pixel limits.
    bool FindPlaneSweepDepthObservation(const ColmapSetup &setup,
        const MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepDepthMap &reference,
        const MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepDepthMap &source,
        const Float3 &point, float referenceX, float referenceY, float referenceDepth,
        PlaneSweepDepthObservation &observation, bool selectBest = true);

    // Refine the depth estimate at the continuous projection only when its entire footprint is valid and coherent.
    // The discrete correspondence pixel and its score remain unchanged for ownership and hard revalidation.
    bool InterpolatePlaneSweepDepthObservation(const ColmapSetup &setup,
        const MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepDepthMap &reference,
        const MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepDepthMap &source,
        const Float3 &point, float referenceX, float referenceY, float referenceDepth,
        PlaneSweepDepthObservation &observation);

    struct PlaneSweepGeometricFusionStats
    {
        std::size_t merged{}, moved{}, unmerged{}, ambiguous{};
    };

    // Input candidates must already pass cross-view validation. Maps stay immutable.
    std::vector<SceneObjectPointCloudPoint>
        FusePlaneSweepGeometricPoints(const ColmapSetup &setup,
            const std::vector<MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepDepthMap> &maps,
            const std::vector<MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepPointCandidate> &points,
            PlaneSweepGeometricFusionStats &statistics);
}
