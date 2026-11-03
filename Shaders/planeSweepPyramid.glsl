#define GRAPHICSAPP_PLANE_SWEEP_PYRAMID 1
#include "planeSweepMatching.glsl"

layout(set = 0, binding = 8, std430) writeonly buffer StateOutput { vec4 values[]; } stateOutput;
layout(set = 0, binding = 9, std430) readonly buffer ParentState { vec4 values[]; } parentState;
// Per-pixel counters avoid atomics and permit deterministic CPU/GPU inspection.
layout(set = 0, binding = 10, std430) buffer Counters { uvec4 values[]; } counters;
layout(push_constant) uniform Level {
    uint parentWidth; uint parentHeight; uint finalLevel; uint localSamples;
    uint pass; uint iterations; float colorSigma; float spatialSigma;
    float costThreshold; float confidenceMinimum; float textureMinimum; uint flags;
} level;

float baseSpacing() { return (paramsBuffer.value.inverseNear - paramsBuffer.value.inverseFar) / float(paramsBuffer.value.planeCount - 1); }
#ifdef GRAPHICSAPP_PLANE_SWEEP_SLANTED
#include "planeSweepSlanted.glsl"
#endif
float confidenceThreshold(float spacing)
{
    float ratio = min(1.0, spacing / baseSpacing());
    return paramsBuffer.value.confidenceMinimumDifference * ratio * ratio;
}

void evaluate(float inverseDepth, vec2 pixel, inout vec4 best, inout float secondInverse, inout uint evaluations)
{
    ++evaluations;
    float cost; uint opaqueSamples, sampleCount;
    if (!ComputePlaneCostCpuEquivalent(pixel.x, pixel.y, 1.0 / inverseDepth, cost, opaqueSamples, sampleCount)) return;
    // Canonicalize costs so sub-micro floating-point noise cannot break ties
    // differently on the CPU and GPU (especially for discrete Rank/Census costs).
    cost = floor(cost * 1e6 + 0.5) / 1e6;
    float epsilon = max(1e-8, baseSpacing() * 1e-4);
    if (best.x > 0.0 && abs(inverseDepth - best.x) <= epsilon) return;
    if (cost < best.y)
    {
        best.z = best.y; secondInverse = best.x;
        best.x = inverseDepth; best.y = cost;
    }
    else if (cost < best.z && abs(inverseDepth - secondInverse) > epsilon)
    { best.z = cost; secondInverse = inverseDepth; }
}

void search(float center, float radius, bool globalSearch, vec2 pixel,
    inout vec4 best, inout float secondInverse, inout uint evaluations, out bool boundary)
{
    float nearInv = paramsBuffer.value.inverseNear, farInv = paramsBuffer.value.inverseFar;
    uint count = globalSearch ? uint(paramsBuffer.value.planeCount) : level.localSamples;
    precise float spacing = globalSearch ? baseSpacing() : 2.0 * radius / float(count - 1u);
    float last = -1.0;
    for (uint i = 0u; i < count; ++i)
    {
        precise float candidate = globalSearch ? nearInv * (1.0 - float(i) / float(count - 1u)) + farInv * (float(i) / float(count - 1u))
            : clamp(center + (float(i) - float(count / 2u)) * spacing, farInv, nearInv);
        if (candidate == last) continue;
        last = candidate;
        float previous = best.x;
        evaluate(candidate, pixel, best, secondInverse, evaluations);
        if (best.x != previous) best.w = spacing;
    }
    if (best.x > 0.0) best.w = min(best.w, spacing);
    float epsilon = max(1e-8, spacing * 1e-3);
    boundary = best.x > 0.0 && best.x > farInv + epsilon && best.x < nearInv - epsilon &&
        (best.x <= center - radius + epsilon || best.x >= center + radius - epsilon);
}

void main()
{
    uvec2 pixel = gl_GlobalInvocationID.xy;
    if (pixel.x >= paramsBuffer.value.scaledWidth || pixel.y >= paramsBuffer.value.scaledHeight) return;
#ifdef GRAPHICSAPP_PLANE_SWEEP_SLANTED
    if (level.pass != 0u) { runSlanted(pixel); return; }
#endif
    uint index = pixel.y * paramsBuffer.value.scaledWidth + pixel.x;
    vec2 xy = vec2(pixel);
    vec4 color;
    uint status = kStatusReferenceInvalid, evaluations = 0u, widened = 0u, fallback = 0u;
    vec4 result = vec4(0.0, kFloatMax, 0.0, baseSpacing());
    if (SamplePlaneSweepPixelCpuEquivalent(referenceImage, referenceMask, xy, sourceMetadata.values[16] != 0u,
        sourceMetadata.values[17] != 0u, color) && color.a > paramsBuffer.value.alphaThreshold)
    {
        status = kStatusNoValidPlane;
        float deviation; uint samples;
        if (paramsBuffer.value.textureRejectionEnabled != 0u &&
            (!ComputeTextureStandardDeviationCpuEquivalent(xy.x, xy.y, paramsBuffer.value.patchRadius,
                paramsBuffer.value.alphaThreshold, sourceMetadata.values[16] != 0u, sourceMetadata.values[17] != 0u,
                0u, deviation, samples) || deviation < paramsBuffer.value.textureMinimumStdDev)) status = kStatusTextureRejected;
        if (status == kStatusNoValidPlane)
        {
            vec4 best = vec4(0.0, kFloatMax, kFloatMax, baseSpacing()); float secondInverse = -1.0;
            bool globalSearch = level.parentWidth == 0u, boundary = false;
            vec4 parent = vec4(0.0); uvec2 parentPixel = uvec2(0u);
            if (!globalSearch)
            {
                parentPixel = min(uvec2((vec2(pixel) + 0.5) * vec2(level.parentWidth, level.parentHeight) /
                    vec2(paramsBuffer.value.scaledWidth, paramsBuffer.value.scaledHeight)), uvec2(level.parentWidth - 1u, level.parentHeight - 1u));
                parent = parentState.values[parentPixel.y * level.parentWidth + parentPixel.x];
                globalSearch = parent.x <= 0.0 || parent.y > paramsBuffer.value.costThreshold ||
                    (paramsBuffer.value.confidenceEnabled != 0u && parent.z < confidenceThreshold(parent.w));
                // A depth discontinuity cannot safely inherit one narrow interval.
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        ivec2 q = ivec2(parentPixel) + ivec2(dx, dy);
                        if (q.x < 0 || q.y < 0 || q.x >= int(level.parentWidth) || q.y >= int(level.parentHeight)) continue;
                        float other = parentState.values[uint(q.y) * level.parentWidth + uint(q.x)].x;
                        if (parent.x > 0.0 && other > 0.0 && abs(1.0 / other - 1.0 / parent.x) >
                            4.0 * parent.w + max(1e-8, baseSpacing() * 1e-3)) globalSearch = true;
                    }
            }
            // One search call site keeps driver compilation and register pressure small.
            // Modes: narrow local -> wide local -> global recovery -> recovery refinement.
            int mode = globalSearch ? 2 : 0;
            fallback = globalSearch && level.parentWidth != 0u ? 1u : 0u;
            [[dont_unroll]] for (int pass = 0; pass < 4; ++pass)
            {
                float center = mode == 3 ? best.x : (parent.x > 0.0 ? 1.0 / parent.x : 0.0);
                float radius = mode == 3 ? 2.0 * baseSpacing() : (mode == 1 ? 8.0 : 2.0) * parent.w;
                search(center, radius, mode == 2, xy, best, secondInverse, evaluations, boundary);
                bool unreliable = best.x <= 0.0 || best.y > paramsBuffer.value.costThreshold || boundary ||
                    (paramsBuffer.value.confidenceEnabled != 0u && best.z - best.y < confidenceThreshold(best.w));
                if (mode == 0 && unreliable) { widened = 1u; mode = 1; }
                else if (mode == 1 && unreliable) { fallback = 1u; mode = 2; }
                else if (mode == 2 && level.parentWidth != 0u && best.x > 0.0) mode = 3;
                else break;
            }
            if (best.x > 0.0)
            {
                result = vec4(1.0 / best.x, best.y, best.z < kFloatMax ? best.z - best.y : 0.0, best.w);
                status = best.y > paramsBuffer.value.costThreshold ? kStatusCostRejected :
                    (paramsBuffer.value.confidenceEnabled != 0u && result.z < confidenceThreshold(result.w)
                        ? kStatusConfidenceRejected : kStatusAccepted);
                if (level.finalLevel != 0u && status != kStatusAccepted) result.x = 0.0;
            }
        }
    }
    stateOutput.values[index] = result;
    counters.values[index] = uvec4(evaluations, widened, fallback, status);
}
