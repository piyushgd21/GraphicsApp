#pragma once

#include "MultiViewUtility.h"
#include <functional>

namespace graphicsFX
{
    struct PlaneSweepColorSelectionStats
    {
        std::size_t m_boundaryPoints{ 0 }, m_recoloredPoints{ 0 };
        std::size_t m_insufficientObservations{ 0 }, m_imagesUnavailable{ 0 };
    };

    // The loader receives a depth-map index. Images are loaded one at a time, not retained for all cameras.
    // Disabled selection is an exact no-op. Only RGB of existing candidates can change; alpha and geometry remain intact.
    PlaneSweepColorSelectionStats SelectPlaneSweepBoundaryColors(
        const ColmapSetup &setup,
        const std::vector<MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepDepthMap> &maps,
        std::vector<MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepPointCandidate> &points,
        const std::function<bool(std::size_t, MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepImage &)> &loadImage);
}
