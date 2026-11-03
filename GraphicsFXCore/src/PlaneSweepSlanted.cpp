#include "PlaneSweepPyramid.h"

#include <algorithm>
#include <chrono>
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
            const SweepUtility::PlaneSweepDepthMap &seed;
            const SweepUtility::PlaneSweepDepthRange &range;
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
            int psWidth() const { return int(seed.m_width); }
            int psHeight() const { return int(seed.m_height); }
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
                return x >= 0 && y >= 0 && x < psWidth() && y < psHeight() ? seed.m_depthValues[size_t(y) * seed.m_width + size_t(x)] : 0.0f;
            }
            vec4 psReadPlane(int x, int y)
            {
                return x >= 0 && y >= 0 && x < psWidth() && y < psHeight() ? (*planes)[size_t(y) * seed.m_width + size_t(x)] : psInvalid();
            }
#define PS_ACCUM double
#include "../../Shaders/planeSweepSlanted.inl"
#undef PS_ACCUM
        };
    }

    SweepUtility::PlaneSweepDepthMap RunPlaneSweepSlantedCpu(const SweepUtility::PlaneSweepImage &reference,
        const std::vector<SweepUtility::PlaneSweepImage> &sources, const ColmapSetup &setup,
        const SweepUtility::PlaneSweepDepthRange &range, std::vector<PlaneSweepPyramidStats> *statistics)
    {
        ValidatePlaneSweepSlantedSetup(setup);
        const auto start = std::chrono::steady_clock::now();
        // Retain provisional estimates until the weighted plane costs have been evaluated.
        ColmapSetup seedSetup = setup;
        seedSetup.m_planeSweepMatchingMode = PlaneSweepMatchingMode::PlaneSweep;
        seedSetup.m_planeSweepConfidenceEnabled = false;
        seedSetup.m_planeSweepTextureRejectionEnabled = false;
        seedSetup.m_planeSweepCostThreshold = 10.0f;
        std::vector<PlaneSweepPyramidStats> stats;
        auto seed = RunPlaneSweepPyramidCpu(reference, sources, seedSetup, range, &stats);
        const auto pyramid = BuildPlaneSweepPyramid(reference, sources, setup);
        SlantedMatcher matcher{ pyramid.back(), setup, seed, range };
        std::vector<vec4> planes(seed.m_depthValues.size()), next(planes.size());
        for (int y = 0; y < matcher.psHeight(); ++y)
            for (int x = 0; x < matcher.psWidth(); ++x) planes[size_t(y) * seed.m_width + size_t(x)] = matcher.psInitialize(x, y);
        for (int iteration = 0; iteration < setup.m_planeSweepPatchMatchIterations; ++iteration)
        {
            matcher.planes = &planes;
            for (int y = 0; y < matcher.psHeight(); ++y)
                for (int x = 0; x < matcher.psWidth(); ++x) next[size_t(y) * seed.m_width + size_t(x)] = matcher.psRefine(x, y, iteration);
            planes.swap(next);
        }
        matcher.planes = &planes;
        stats.back().m_accepted = stats.back().m_confidenceRejected = stats.back().m_textureRejected = 0;
        for (int y = 0; y < matcher.psHeight(); ++y)
            for (int x = 0; x < matcher.psWidth(); ++x)
            {
                const size_t index = size_t(y) * seed.m_width + size_t(x);
                const auto result = matcher.psFinalize(x, y);
                seed.m_depthValues[index] = result.x;
                const int status = matcher.psStatus(x, y, result);
                if (status == 3) ++stats.back().m_accepted;
                else if (status == 2) ++stats.back().m_confidenceRejected;
                else if (status == 5) ++stats.back().m_textureRejected;
            }
        stats.back().m_evaluations += matcher.evaluations;
        stats.back().m_milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        if (statistics) *statistics = std::move(stats);
        return seed;
    }
}
