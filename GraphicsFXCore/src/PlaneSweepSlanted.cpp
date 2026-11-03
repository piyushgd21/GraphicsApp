#include "PlaneSweepPyramid.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace graphicsFX
{
    void ValidatePlaneSweepSlantedSetup(const ColmapSetup &setup)
    {
        if (setup.m_planeSweepPatchRadius < 1 || setup.m_planeSweepPatchRadius > 16 ||
            setup.m_planeSweepPatchMatchIterations < 1 || setup.m_planeSweepPatchMatchIterations > 12 ||
            !std::isfinite(setup.m_planeSweepPatchMatchColorSigma) || setup.m_planeSweepPatchMatchColorSigma <= 0 ||
            !std::isfinite(setup.m_planeSweepPatchMatchSpatialSigma) || setup.m_planeSweepPatchMatchSpatialSigma <= 0)
            throw std::invalid_argument("PatchMatchSlanted requires radius 1..16, iterations 1..12 and positive finite support sigmas.");
        if (setup.m_planeSweepPatchCostMethod != PatchCostMethod::PatchCostMethodMeanAbsoluteDifference &&
            setup.m_planeSweepPatchCostMethod != PatchCostMethod::PatchCostMethodNormalizedCrossCorrelation &&
            setup.m_planeSweepPatchCostMethod != PatchCostMethod::PatchCostMethodZeroMeanNormalizedCrossCorrelation)
            throw std::invalid_argument("PatchMatchSlanted supports MeanAbsoluteDifference, NormalizedCrossCorrelation and ZeroMeanNormalizedCrossCorrelation.");
    }

    namespace
    {
        using vec3 = Float3;
        using vec4 = Float4;
        using uint = uint32_t;
        struct PsSample { vec3 color{}; float alpha{}; bool valid{}; };

        struct SlantedMatcher
        {
            const PlaneSweepPyramidLevel &images;
            const ColmapSetup &setup;
            const std::vector<PlaneSweepPyramidState> &seed;
            const SweepUtility::PlaneSweepDepthRange &range;
            const std::vector<vec4> &parentPlanes;
            uint32_t parentWidth, parentHeight;
            bool finalLevel;
            const std::vector<vec4> *planes{};
            uint64_t evaluations{};
            void psCountEvaluation() { ++evaluations; }

            static vec3 psV3(float x, float y, float z) { return { x, y, z }; }
            static vec4 psV4(float x, float y, float z, float w) { return { x, y, z, w }; }
            static float psAbs(float v) { return std::fabs(v); }
            static float psExp(float v) { return std::exp(v); }
            static double psSqrt(double v) { return std::sqrt(v); }
            static float psMin(float a, float b) { return std::min(a, b); }
            static float psMax(float a, float b) { return std::max(a, b); }
            static float psClamp(float a, float b, float c) { return std::clamp(a, b, c); }
            static float psFloor(float v) { return std::floor(v); }
            static bool psFinite(float v) { return std::isfinite(v); }
            int psWidth() const { return int(images.m_reference.m_texture.m_width); }
            int psHeight() const { return int(images.m_reference.m_texture.m_height); }
            bool psFinalLevel() const { return finalLevel; }
            int psRadius() const { return setup.m_planeSweepPatchRadius; }
            bool psTiltEnabled() const { return setup.m_planeSweepPatchMatchTiltEnabled; }
            float psColorSigma() const { return setup.m_planeSweepPatchMatchColorSigma; }
            float psSpatialSigma() const { return setup.m_planeSweepPatchMatchSpatialSigma; }
            float psAlphaThreshold() const { return setup.m_planeSweepAlphaThreshold; }
            float psMinimumRatio() const { return setup.m_planeSweepMinimumOpaquePatchRatio; }
            float psFarInverse() const { return 1.0f / range.m_farDepth; }
            float psNearInverse() const { return 1.0f / range.m_nearDepth; }
            float psSpacing() const { return (psNearInverse() - psFarInverse()) / float(setup.m_planeSweepPlaneCount - 1); }
            int psSourceCount() const { return int(images.m_sources.size()); }
            int psMinimumViews() const { return std::max(1, std::min(psSourceCount(), setup.m_planeSweepMinValidSourceViews)); }
            int psMethod() const { return int(setup.m_planeSweepPatchCostMethod); }
            bool psTextureEnabled() const { return setup.m_planeSweepTextureRejectionEnabled; }
            float psTextureMinimum() const { return setup.m_planeSweepTextureMinimumStdDev; }
            bool psConfidenceEnabled() const { return setup.m_planeSweepConfidenceEnabled; }
            float psConfidenceMinimum() const { return setup.m_planeSweepConfidenceMinimumDifference; }
            float psCostThreshold() const { return setup.m_planeSweepCostThreshold; }
            vec3 psForward() const { return images.m_reference.m_camera.m_forward; }
            vec3 psRight() const { return images.m_reference.m_camera.m_right; }
            vec3 psUp() const { return images.m_reference.m_camera.m_up; }
            vec3 psRay(int x, int y) const
            {
                return SweepUtility::UnprojectPixel(images.m_reference.m_camera, images.m_reference.m_intrinsic,
                    float(x), float(y), 1.0f) - images.m_reference.m_camera.m_position;
            }
            PsSample psReference(int x, int y) const
            {
                Float4 c{};
                bool valid = x >= 0 && y >= 0 && x < psWidth() && y < psHeight() &&
                    SweepUtility::SamplePlaneSweepPixel(images.m_reference, float(x), float(y), c);
                return { { c.x, c.y, c.z }, c.w, valid };
            }
            PsSample psSource(int source, int x, int y, vec3 k) const
            {
                vec3 ray = psRay(x, y);
                float inverse = dot(k, ray);
                if (!psFinite(inverse) || inverse <= 0) return {};
                vec3 point = images.m_reference.m_camera.m_position + ray * (1.0f / inverse);
                float sx{}, sy{}, sz{};
                const auto &image = images.m_sources[size_t(source)];
                if (!SweepUtility::ProjectPoint(image.m_camera, image.m_intrinsic, point, sx, sy, sz)) return {};
                sx -= 0.5f; sy -= 0.5f;
                if (sx >= -1e-4f && sx <= float(image.m_texture.m_width - 1) + 1e-4f) sx = std::clamp(sx, 0.0f, float(image.m_texture.m_width - 1));
                if (sy >= -1e-4f && sy <= float(image.m_texture.m_height - 1) + 1e-4f) sy = std::clamp(sy, 0.0f, float(image.m_texture.m_height - 1));
                Float4 c{};
                bool valid = SweepUtility::SamplePlaneSweepPixel(image, sx, sy, c);
                return { { c.x, c.y, c.z }, c.w, valid };
            }
            float psSeed(int x, int y) const
            {
                return x >= 0 && y >= 0 && x < psWidth() && y < psHeight() ? seed[size_t(y) * size_t(psWidth()) + size_t(x)][0] : 0.0f;
            }
            // Map pixel centers to the nearest parent without blending across depth boundaries.
            vec4 psParentPlane(int x, int y)
            {
                if (parentPlanes.empty() || !parentWidth || !parentHeight) return psInvalid();
                const uint32_t px = std::min(uint32_t((float(x) + 0.5f) * float(parentWidth) / float(psWidth())), parentWidth - 1);
                const uint32_t py = std::min(uint32_t((float(y) + 0.5f) * float(parentHeight) / float(psHeight())), parentHeight - 1);
                return parentPlanes[size_t(py) * parentWidth + px];
            }
            vec4 psReadPlane(int x, int y)
            {
                return x >= 0 && y >= 0 && x < psWidth() && y < psHeight() ? (*planes)[size_t(y) * size_t(psWidth()) + size_t(x)] : psInvalid();
            }
#define PS_ACCUM double
#include "../../Shaders/planeSweepSlanted.inl"
#undef PS_ACCUM
        };
    }

    // Refines one level and returns world-space planes for initialization at the next resolution.
    std::vector<Float4> RefinePlaneSweepSlantedLevelCpu(
        const PlaneSweepPyramidLevel &level, const ColmapSetup &setup,
        const SweepUtility::PlaneSweepDepthRange &range, std::vector<PlaneSweepPyramidState> &state,
        const std::vector<Float4> &parentPlanes, uint32_t parentWidth, uint32_t parentHeight,
        bool finalLevel, PlaneSweepPyramidStats &statistics)
    {
        SlantedMatcher matcher{ level, setup, state, range, parentPlanes, parentWidth, parentHeight, finalLevel };
        const size_t width = size_t(matcher.psWidth());
        std::vector<vec4> planes(state.size()), next(planes.size());
        for (int y = 0; y < matcher.psHeight(); ++y)
            for (int x = 0; x < matcher.psWidth(); ++x) planes[size_t(y) * width + size_t(x)] = matcher.psInitialize(x, y);
        for (int iteration = 0; iteration < setup.m_planeSweepPatchMatchIterations; ++iteration)
        {
            matcher.planes = &planes;
            for (int y = 0; y < matcher.psHeight(); ++y)
                for (int x = 0; x < matcher.psWidth(); ++x) next[size_t(y) * width + size_t(x)] = matcher.psRefine(x, y, iteration);
            planes.swap(next);
        }
        matcher.planes = &planes;
        statistics.m_accepted = statistics.m_confidenceRejected = statistics.m_textureRejected = 0;
        for (int y = 0; y < matcher.psHeight(); ++y)
            for (int x = 0; x < matcher.psWidth(); ++x)
            {
                const size_t index = size_t(y) * width + size_t(x);
                const auto result = matcher.psFinalize(x, y);
                const int status = matcher.psStatus(x, y, result);
                const auto output = matcher.psLevelState(x, y, result);
                state[index] = { output.x, output.y, output.z, output.w };
                if (status == 3) ++statistics.m_accepted;
                else if (status == 2) ++statistics.m_confidenceRejected;
                else if (status == 5) ++statistics.m_textureRejected;
            }
        statistics.m_evaluations += matcher.evaluations;
        return planes;
    }

    // Runs the shared pyramid driver in slanted mode, including per-level refinement.
    SweepUtility::PlaneSweepDepthMap RunPlaneSweepSlantedCpu(const SweepUtility::PlaneSweepImage &reference,
        const std::vector<SweepUtility::PlaneSweepImage> &sources, const ColmapSetup &setup,
        const SweepUtility::PlaneSweepDepthRange &range, std::vector<PlaneSweepPyramidStats> *statistics)
    {
        ColmapSetup slantedSetup = setup;
        slantedSetup.m_planeSweepMatchingMode = PlaneSweepMatchingMode::PatchMatchSlanted;
        return RunPlaneSweepPyramidCpu(reference, sources, slantedSetup, range, statistics);
    }
}
