// Shared C++/GLSL algorithm. The two adapters supply camera, image and buffer access.
// A plane k satisfies dot(k, X - referenceCameraCenter) = 1.
// With a camera-Z unit ray r, its intersection depth is 1 / dot(k, r).
vec3 psVector(vec4 v) { return psV3(v.x, v.y, v.z); }
vec4 psInvalid() { return psV4(0.0f, 0.0f, 0.0f, 3.402823466e+38f); }
float psColorDistance(vec3 a, vec3 b)
{
    return psAbs(a.x - b.x) + psAbs(a.y - b.y) + psAbs(a.z - b.z);
}
float psIntensity(vec3 c) { return (c.x * 0.299f + c.y * 0.587f) + c.z * 0.114f; }
float psWeight(vec3 center, vec3 color, int dx, int dy)
{
    return psExp(-psColorDistance(center, color) / psColorSigma() -
        float(dx * dx + dy * dy) / (2.0f * psSpatialSigma() * psSpatialSigma()));
}
bool psUsable(PsSample s) { return s.valid && s.alpha > psAlphaThreshold(); }
bool psGeometryValid(int x, int y, vec3 k)
{
    float inverse = dot(k, psRay(x, y));
    return psFinite(k.x) && psFinite(k.y) && psFinite(k.z) &&
        inverse >= psFarInverse() && inverse <= psNearInverse() &&
        dot(k, psForward()) > 0.0f &&
        dot(k, k) <= 17.0f * dot(k, psForward()) * dot(k, psForward());
}
struct PsSupport { float weight; float variance; bool valid; };
PsSupport psSupport(int x, int y)
{
    PsSupport result;
    result.weight = 0.0f; result.variance = 0.0f; result.valid = false;
    PsSample center = psReference(x, y);
    if (!psUsable(center)) return result;
    float referenceWeight = 0.0f, referenceWeight2 = 0.0f;
    float luminance = 0.0f, luminance2 = 0.0f;
    int radius = psRadius();
    for (int dy = -radius; dy <= radius; ++dy)
        for (int dx = -radius; dx <= radius; ++dx)
        {
            PsSample r = psReference(x + dx, y + dy);
            if (!psUsable(r)) continue;
            float w = psWeight(center.color, r.color, dx, dy);
            float value = psIntensity(r.color);
            referenceWeight += w; referenceWeight2 += w * w;
            luminance += w * value; luminance2 += w * value * value;
        }
    result.weight = referenceWeight;
    if (referenceWeight <= 0.0f || referenceWeight * referenceWeight < 3.0f * referenceWeight2) return result;
    float mean = luminance / referenceWeight;
    result.variance = psMax(0.0f, luminance2 / referenceWeight - mean * mean);
    result.valid = true;
    return result;
}
// Reference-guided weights stay identical for all plane hypotheses at a pixel.
float psCost(int x, int y, vec3 k)
{
    psCountEvaluation();
    const float invalidCost = 3.402823466e+38f;
    if (!psGeometryValid(x, y, k)) return invalidCost;
    PsSample center = psReference(x, y);
    PsSupport support = psSupport(x, y);
    if (!support.valid || (psTextureEnabled() && support.variance < psTextureMinimum() * psTextureMinimum())) return invalidCost;
    int radius = psRadius();
    float total = 0.0f;
    int views = 0;
    for (int source = 0; source < psSourceCount(); ++source)
    {
        float sumWeight = 0.0f, sumWeight2 = 0.0f, difference = 0.0f;
        PS_ACCUM rs = PS_ACCUM(0), ss = PS_ACCUM(0), rr = PS_ACCUM(0), st = PS_ACCUM(0), rt = PS_ACCUM(0);
        PS_ACCUM weightedCount = PS_ACCUM(0);
        for (int dy = -radius; dy <= radius; ++dy)
            for (int dx = -radius; dx <= radius; ++dx)
            {
                PsSample r = psReference(x + dx, y + dy);
                if (!psUsable(r)) continue;
                PsSample s = psSource(source, x + dx, y + dy, k);
                if (!psUsable(s)) continue;
                float w = psWeight(center.color, r.color, dx, dy);
                sumWeight += w; sumWeight2 += w * w;
                difference += w * psColorDistance(r.color, s.color) / 3.0f;
                PS_ACCUM rw = PS_ACCUM(w), rv = PS_ACCUM(psIntensity(r.color)), sv = PS_ACCUM(psIntensity(s.color));
                weightedCount += rw;
                rs += rw * rv; ss += rw * sv;
                rr += rw * rv * rv; st += rw * sv * sv; rt += rw * rv * sv;
            }
        if (sumWeight <= 0.0f || sumWeight < psMinimumRatio() * support.weight ||
            sumWeight * sumWeight < 3.0f * sumWeight2) continue;
        float cost = difference / sumWeight;
        if (psMethod() != 0)
        {
            if (psMethod() == 2)
            {
                rr -= rs * rs / weightedCount;
                st -= ss * ss / weightedCount;
                rt -= rs * ss / weightedCount;
            }
            if (rr <= PS_ACCUM(1.0e-12) || st <= PS_ACCUM(1.0e-12)) continue;
            float correlation = float(rt / psSqrt(rr * st));
            correlation = psClamp(correlation, psMethod() == 2 ? -1.0f : 0.0f, 1.0f);
            cost = (1.0f - correlation) * (psMethod() == 2 ? 0.5f : 1.0f);
        }
        if (!psFinite(cost)) continue;
        total += cost; ++views;
    }
    if (views < psMinimumViews()) return invalidCost;
    // Match the pyramid's stable cost comparison precision.
    return psFloor((total / float(views)) * 1.0e6f + 0.5f) / 1.0e6f;
}
vec4 psConsider(int x, int y, vec4 best, vec3 k)
{
    float cost = psCost(x, y, k);
    if (cost < best.w) return psV4(k.x, k.y, k.z, cost);
    return best;
}
vec4 psInitialize(int x, int y)
{
    vec4 best = psInvalid();
    // Camera pose is unchanged by resizing, so k needs no image-scale conversion.
    // Re-score the inherited plane against the current level instead of inheriting its cost.
    vec4 parent = psParentPlane(x, y);
    if (parent.w < 3.402823466e+38f) best = psConsider(x, y, best, psVector(parent));
    float depth = psSeed(x, y);
    if (depth <= 0.0f) return best;
    vec3 k = psForward() * (1.0f / depth);
    best = psConsider(x, y, best, k);
    if (!psTiltEnabled()) return best;
    vec3 p = psRay(x, y) * depth;
    PsSample center = psReference(x, y);
    // Test four local geometric plane fits, rather than averaging through a boundary.
    for (int direction = 0; direction < 4; ++direction)
    {
        int dx = direction < 2 ? 1 : -1, dy = direction % 2 == 0 ? 1 : -1;
        float zx = psSeed(x + dx, y), zy = psSeed(x, y + dy);
        PsSample sx = psReference(x + dx, y), sy = psReference(x, y + dy);
        if (zx <= 0.0f || zy <= 0.0f || !psUsable(sx) || !psUsable(sy) ||
            psColorDistance(center.color, sx.color) > 3.0f * psColorSigma() ||
            psColorDistance(center.color, sy.color) > 3.0f * psColorSigma()) continue;
        vec3 n = cross(psRay(x + dx, y) * zx - p, psRay(x, y + dy) * zy - p);
        float denominator = dot(n, p);
        if (psAbs(denominator) > 1.0e-10f) best = psConsider(x, y, best, n * (1.0f / denominator));
    }
    return best;
}
uint psHash(uint v)
{
    v = (v ^ (v >> 16u)) * 0x7feb352du;
    v = (v ^ (v >> 15u)) * 0x846ca68bu;
    return v ^ (v >> 16u);
}
float psRandom(uint v) { return float(psHash(v) & 0xffffu) / 32767.5f - 1.0f; }
vec3 psShift(vec3 k, vec3 ray, float inverseChange, float slopeX, float slopeY)
{
    vec3 delta = psRight() * slopeX + psUp() * slopeY;
    return k + delta + psForward() * (inverseChange - dot(delta, ray));
}
vec4 psRefine(int x, int y, int iteration)
{
    vec4 best = psReadPlane(x, y);
    // Synchronous neighbors: no read/write races and the same schedule on CPU and GPU.
    for (int direction = 0; direction < 4; ++direction)
    {
        int dx = direction == 0 ? -1 : (direction == 1 ? 1 : 0);
        int dy = direction == 2 ? -1 : (direction == 3 ? 1 : 0);
        vec4 other = psReadPlane(x + dx, y + dy);
        if (other.w < 3.402823466e+38f) best = psConsider(x, y, best, psVector(other));
    }
    if (best.w >= 3.402823466e+38f) return best;
    vec3 ray = psRay(x, y);
    float reduction = psExp(-0.69314718056f * float(iteration));
    float depthStep = psSpacing() * reduction;
    float slopeStep = psTiltEnabled() ? dot(psVector(best), ray) * 0.5f * reduction : 0.0f;
    uint randomSeed = uint(y * psWidth() + x) ^ (uint(iteration + 1) * 0x9e3779b9u);
    for (int sampleIndex = 0; sampleIndex < 6; ++sampleIndex)
    {
        uint v = randomSeed + uint(sampleIndex) * 3u;
        vec3 candidate = psShift(psVector(best), ray, psRandom(v) * depthStep,
            psRandom(v + 1u) * slopeStep, psRandom(v + 2u) * slopeStep);
        best = psConsider(x, y, best, candidate);
        depthStep *= 0.5f; slopeStep *= 0.5f;
    }
    // Deterministic depth-only probes avoid coupling every depth change to a normal change.
    best = psConsider(x, y, best, psShift(psVector(best), ray, psSpacing() * reduction * 0.25f, 0.0f, 0.0f));
    best = psConsider(x, y, best, psShift(psVector(best), ray, -psSpacing() * reduction * 0.25f, 0.0f, 0.0f));
    return best;
}
// Confidence compares distinct center depths, rather than alternative normals at the same depth.
vec4 psFinalize(int x, int y)
{
    vec4 best = psReadPlane(x, y);
    if (best.w >= 3.402823466e+38f || best.w > psCostThreshold()) return psV4(0.0f, best.w, 0.0f, psSpacing());
    float gap = 0.0f;
    if (psConfidenceEnabled())
    {
        vec3 ray = psRay(x, y);
        float step = psSpacing() * 0.125f;
        float lo = psCost(x, y, psShift(psVector(best), ray, -step, 0.0f, 0.0f));
        float hi = psCost(x, y, psShift(psVector(best), ray, step, 0.0f, 0.0f));
        float second = psMin(lo, hi);
        if (second >= 3.402823466e+38f) return psV4(0.0f, best.w, 0.0f, psSpacing());
        gap = psMax(0.0f, second - best.w);
        if (gap < psConfidenceMinimum() * 0.015625f) return psV4(0.0f, best.w, gap, psSpacing());
    }
    return psV4(1.0f / dot(psVector(best), psRay(x, y)), best.w, gap, psSpacing());
}
int psStatus(int x, int y, vec4 result)
{
    if (!psUsable(psReference(x, y))) return 4;
    PsSupport support = psSupport(x, y);
    if (!support.valid || (psTextureEnabled() && support.variance < psTextureMinimum() * psTextureMinimum())) return 5;
    if (result.x > 0.0f) return 3;
    if (psReadPlane(x, y).w <= psCostThreshold() && psConfidenceEnabled()) return 2;
    return 1;
}
// Preserve provisional geometry between levels; final output still applies all rejection rules.
vec4 psLevelState(int x, int y, vec4 result)
{
    vec4 best = psReadPlane(x, y);
    if (!psFinalLevel() && best.w < 3.402823466e+38f)
        result.x = 1.0f / dot(psVector(best), psRay(x, y));
    return result;
}
