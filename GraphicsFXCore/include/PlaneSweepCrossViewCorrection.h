#pragma once

#include "MultiViewUtility.h"

namespace graphicsFX
{
    struct PlaneSweepCrossViewCorrectionStats
    {
        std::size_t m_examined{ 0 }, m_corrected{ 0 }, m_insufficientSupport{ 0 }, m_ambiguous{ 0 }, m_unchanged{ 0 };
        std::size_t m_surfaceSupported{ 0 }, m_surfaceMissing{ 0 }, m_surfaceEdges{ 0 };
        std::size_t m_regularized{ 0 }, m_consensusOnly{ 0 }, m_proposalsRejected{ 0 };
        double m_relativeMovementSum{ 0 }, m_maxRelativeMovement{ 0 }, m_scoreBeforeSum{ 0 }, m_scoreAfterSum{ 0 };
        double m_objectiveBeforeSum{ 0 }, m_objectiveAfterSum{ 0 };
        double m_surfaceErrorBeforeSum{ 0 }, m_surfaceErrorAfterSum{ 0 };
        double MeanRelativeMovement() const { return m_corrected ? m_relativeMovementSum / m_corrected : 0; }
        double MeanScoreBefore() const { return m_corrected ? m_scoreBeforeSum / m_corrected : 0; }
        double MeanScoreAfter() const { return m_corrected ? m_scoreAfterSum / m_corrected : 0; }
        double MeanObjectiveBefore() const { return m_corrected ? m_objectiveBeforeSum / m_corrected : 0; }
        double MeanObjectiveAfter() const { return m_corrected ? m_objectiveAfterSum / m_corrected : 0; }
        double MeanSurfaceErrorBefore() const { return m_regularized ? m_surfaceErrorBeforeSum / m_regularized : 0; }
        double MeanSurfaceErrorAfter() const { return m_regularized ? m_surfaceErrorAfterSum / m_regularized : 0; }
    };

    // Post-filter surface regularization and cross-view correction of existing candidates only.
    // Map/neighbor snapshots are immutable; count, ordering and RGBA are preserved.
    // A disabled correction or consistency switch is an exact no-op. Coordinates remain in COLMAP world units.
    PlaneSweepCrossViewCorrectionStats CorrectPlaneSweepCrossViewDepths(
        const ColmapSetup &setup,
        const std::vector<MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepDepthMap> &maps,
        std::vector<MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepPointCandidate> &points);
}
