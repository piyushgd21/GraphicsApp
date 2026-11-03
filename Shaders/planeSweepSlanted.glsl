// Adapter for the shared, non-learning tilted-plane matcher.
layout(set = 0, binding = 11, std430) readonly buffer SlantedInput { vec4 values[]; } slantedInput;
layout(set = 0, binding = 12, std430) writeonly buffer SlantedOutput { vec4 values[]; } slantedOutput;
struct PsSample { vec3 color; float alpha; bool valid; };
uint psEvaluationCount = 0u;
void psCountEvaluation() { ++psEvaluationCount; }
vec3 psV3(float x, float y, float z) { return vec3(x, y, z); }
vec4 psV4(float x, float y, float z, float w) { return vec4(x, y, z, w); }
#define psAbs abs
#define psExp exp
#define psSqrt sqrt
#define psMin min
#define psMax max
#define psClamp clamp
#define psFloor floor
bool psFinite(float v) { return IsFiniteFloat(v); }
int psWidth() { return int(paramsBuffer.value.scaledWidth); }
int psHeight() { return int(paramsBuffer.value.scaledHeight); }
bool psFinalLevel() { return level.finalLevel != 0u; }
int psRadius() { return paramsBuffer.value.patchRadius; }
bool psTiltEnabled() { return (level.flags & 4u) != 0u; }
float psColorSigma() { return level.colorSigma; }
float psSpatialSigma() { return level.spatialSigma; }
float psAlphaThreshold() { return paramsBuffer.value.alphaThreshold; }
float psMinimumRatio() { return paramsBuffer.value.minimumOpaquePatchRatio; }
float psFarInverse() { return paramsBuffer.value.inverseFar; }
float psNearInverse() { return paramsBuffer.value.inverseNear; }
float psSpacing() { return baseSpacing(); }
int psSourceCount() { return int(paramsBuffer.value.sourceCount); }
int psMinimumViews() { return paramsBuffer.value.minValidSourceViews; }
int psMethod() { return int(paramsBuffer.value.patchCostMethod); }
bool psTextureEnabled() { return (level.flags & 1u) != 0u; }
float psTextureMinimum() { return level.textureMinimum; }
bool psConfidenceEnabled() { return (level.flags & 2u) != 0u; }
float psConfidenceMinimum() { return level.confidenceMinimum; }
float psCostThreshold() { return level.costThreshold; }
vec3 psForward() { return poses.values[0].forward.xyz; }
vec3 psRight() { return poses.values[0].right.xyz; }
vec3 psUp() { return poses.values[0].up.xyz; }
vec3 psRay(int x, int y)
{
    return UnprojectPixelCpuEquivalent(poses.values[0], intrinsics.values[0], float(x), float(y), 1.0) - poses.values[0].position.xyz;
}
PsSample psReference(int x, int y)
{
    vec4 c = vec4(0.0);
    bool valid = x >= 0 && y >= 0 && x < psWidth() && y < psHeight();
    if (valid) valid = SampleBilinearCpuEquivalent(referenceImage, vec2(x, y), c);
    return PsSample(c.rgb, c.a, valid);
}
PsSample psSource(int source, int x, int y, vec3 k)
{
    vec3 ray = psRay(x, y);
    float inverse = dot(k, ray);
    if (!psFinite(inverse) || inverse <= 0.0) return PsSample(vec3(0.0), 0.0, false);
    vec3 point = poses.values[0].position.xyz + ray * (1.0 / inverse);
    float sx, sy, sz;
    if (!ProjectPointCpuEquivalent(poses.values[source + 1], intrinsics.values[source + 1], point, sx, sy, sz))
        return PsSample(vec3(0.0), 0.0, false);
    vec4 c = vec4(0.0);
    bool valid = SampleSource(uint(source), vec2(sx, sy), c);
    return PsSample(c.rgb, c.a, valid);
}
float psSeed(int x, int y)
{
    return x >= 0 && y >= 0 && x < psWidth() && y < psHeight() ? slantedInput.values[y * psWidth() + x].x : 0.0;
}
// Binding 9 holds parent depth state during the sweep and parent planes during refinement.
vec4 psParentPlane(int x, int y)
{
    if (level.parentWidth == 0u || level.parentHeight == 0u) return vec4(0.0, 0.0, 0.0, kFloatMax);
    uvec2 parentPixel = min(uvec2((vec2(x, y) + 0.5) * vec2(level.parentWidth, level.parentHeight) /
        vec2(psWidth(), psHeight())), uvec2(level.parentWidth - 1u, level.parentHeight - 1u));
    return parentState.values[parentPixel.y * level.parentWidth + parentPixel.x];
}
vec4 psReadPlane(int x, int y)
{
    return x >= 0 && y >= 0 && x < psWidth() && y < psHeight() ? slantedInput.values[y * psWidth() + x] : vec4(0.0, 0.0, 0.0, kFloatMax);
}
#ifdef GRAPHICSAPP_PLANE_SWEEP_FP64
#define PS_ACCUM double
#else
#define PS_ACCUM float
#endif
#include "planeSweepSlanted.inl"
#undef PS_ACCUM

void runSlanted(uvec2 pixel)
{
    int x = int(pixel.x), y = int(pixel.y);
    uint index = pixel.y * paramsBuffer.value.scaledWidth + pixel.x;
    if (level.pass == 1u) slantedOutput.values[index] = psInitialize(x, y);
    else if (level.pass <= level.iterations + 1u)
        slantedOutput.values[index] = psRefine(x, y, int(level.pass - 2u));
    else
    {
        vec4 result = psFinalize(x, y);
        counters.values[index].w = uint(psStatus(x, y, result));
        stateOutput.values[index] = psLevelState(x, y, result);
    }
    counters.values[index].x += psEvaluationCount;
}
