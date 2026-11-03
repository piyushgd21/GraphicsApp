#ifdef GRAPHICSAPP_PLANE_SWEEP_FP64
#extension GL_ARB_gpu_shader_fp64 : require
#endif

#ifdef GRAPHICSAPP_PLANE_SWEEP_PYRAMID
#define PS_PRECISE precise
#else
#define PS_PRECISE
#endif

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

struct GpuCameraIntrinsics { int width; int height; float fx; float fy; float cx; float cy; float k1; float k2; float p1; float p2; };
struct GpuCameraPose { vec4 position; vec4 forward; vec4 up; vec4 right; };
struct PlaneSweepGpuParams { float inverseNear; float inverseFar; int planeCount; int patchRadius; int minValidSourceViews; float alphaThreshold; float minimumOpaquePatchRatio; float costThreshold; float confidenceMinimumDifference; float textureMinimumStdDev; uint sourceCount; uint textureRejectionEnabled; uint confidenceEnabled; uint pointStep; uint scaledWidth; uint scaledHeight; uint patchCostMethod; float adCensusWeight; };

layout(set = 0, binding = 0) uniform sampler2D referenceImage;
layout(set = 0, binding = 1) uniform sampler2D referenceMask;
layout(set = 0, binding = 2) uniform sampler2D sourceImages[8];
layout(set = 0, binding = 3) uniform sampler2D sourceMasks[8];
layout(set = 0, binding = 4, std430) readonly buffer IntrinsicsBuffer { GpuCameraIntrinsics values[]; } intrinsics;
layout(set = 0, binding = 5, std430) readonly buffer PosesBuffer { GpuCameraPose values[]; } poses;
layout(set = 0, binding = 6, std430) readonly buffer ParamsBuffer { PlaneSweepGpuParams value; } paramsBuffer;
layout(set = 0, binding = 7, std430) readonly buffer SourceMetadata { uint values[]; } sourceMetadata;
#ifndef GRAPHICSAPP_PLANE_SWEEP_PYRAMID
layout(set = 0, binding = 8, std430) writeonly buffer DepthOutput { float values[]; } depthOutput;
layout(set = 0, binding = 9, std430) writeonly buffer MappingOutput { uint values[]; } mappingOutput;
layout(set = 0, binding = 10, std430) writeonly buffer StatusOutput { uint values[]; } statusOutput;
layout(set = 0, binding = 11, std430) writeonly buffer TextureStdDevOutput { uint values[]; } textureStdDevOutput;
layout(set = 0, binding = 12, std430) writeonly buffer TextureSampleCountOutput { uint values[]; } textureSampleCountOutput;
layout(set = 0, binding = 13, std430) writeonly buffer MinimumOpaqueSamplesOutput { uint values[]; } minimumOpaqueSamplesOutput;
layout(set = 0, binding = 14, std430) writeonly buffer PlaneCostSampleCountOutput { uint values[]; } planeCostSampleCountOutput;
#endif
#if defined(GRAPHICSAPP_ENABLE_PHASE_2PB_GPU_TRACE) && !defined(GRAPHICSAPP_PLANE_SWEEP_PYRAMID)
layout(set = 0, binding = 15, std430) writeonly buffer TraceOutput { uint values[]; } traceOutput;
#endif

const float kFloatMax = 3.402823466e+38;
const uint kStatusNoValidPlane = 0u;
const uint kStatusCostRejected = 1u;
const uint kStatusConfidenceRejected = 2u;
const uint kStatusAccepted = 3u;
const uint kStatusReferenceInvalid = 4u;
const uint kStatusTextureRejected = 5u;

bool IsFiniteFloat(float value) { return !isnan(value) && !isinf(value); }

//Matches the numeric values of the C++ PatchCostMethod enum.
const uint kPatchCostMeanAbsoluteDifference = 0u;
const uint kPatchCostNormalizedCrossCorrelation = 1u;
const uint kPatchCostZeroMeanNormalizedCrossCorrelation = 2u;
const uint kPatchCostRankTransform = 3u;
const uint kPatchCostCensusTransform = 4u;
const uint kPatchCostADCensus = 5u;

//Accumulates measurements for absolute difference, Rank, and Census costs.
struct PlaneSweepPatchStatistics
{
    int sampleCount;
    float absoluteDifference;
    float rankDifference;
    int censusSampleCount;
    int censusDifferenceCount;
#ifdef GRAPHICSAPP_PLANE_SWEEP_FP64
    double referenceSum;
    double sourceSum;
    double referenceSquaredSum;
    double sourceSquaredSum;
    double productSum;
#endif
};

//Converts normalized RGB values into grayscale intensity.
float ComputePatchGrayscale(vec4 color)
{
    precise float red = color.r * 0.299;
    precise float green = color.g * 0.587;
    precise float blue = color.b * 0.114;
    precise float redGreen = red + green;
    return redGreen + blue;
}

//Matches the CPU tolerance for interpolation rounding.
bool IsPatchIntensityDarker(float intensity, float centerIntensity)
{
    return intensity < centerIntensity - 1.0e-6;
}

//Converts accumulated measurements into a supported GPU patch cost.
bool ComputePatchCost(
    PlaneSweepPatchStatistics statistics,
    uint method,
    float adCensusWeight,
    out float cost)
{
    cost = 0.0;
    if (statistics.sampleCount <= 0)
    {
        return false;
    }

    switch (method)
    {
        case kPatchCostMeanAbsoluteDifference:
            cost = statistics.absoluteDifference / float(statistics.sampleCount);
            break;

#ifdef GRAPHICSAPP_PLANE_SWEEP_FP64
        case kPatchCostNormalizedCrossCorrelation:
        case kPatchCostZeroMeanNormalizedCrossCorrelation:
        {
            precise double numerator = statistics.productSum;
            precise double referenceEnergy = statistics.referenceSquaredSum;
            precise double sourceEnergy = statistics.sourceSquaredSum;
            bool zeroMean =
                method == kPatchCostZeroMeanNormalizedCrossCorrelation;

            if (zeroMean)
            {
                double count = double(statistics.sampleCount);
                numerator -= statistics.referenceSum * statistics.sourceSum / count;
                referenceEnergy -=
                    statistics.referenceSum * statistics.referenceSum / count;
                sourceEnergy -=
                    statistics.sourceSum * statistics.sourceSum / count;
            }

            if (referenceEnergy <= 1.0e-12LF ||
                sourceEnergy <= 1.0e-12LF)
            {
                return false;
            }

            double correlation = clamp(
                numerator / sqrt(referenceEnergy * sourceEnergy),
                zeroMean ? -1.0LF : 0.0LF, 1.0LF);
            cost = float(
                (1.0LF - correlation) * (zeroMean ? 0.5LF : 1.0LF));
            break;
        }
#endif

        case kPatchCostRankTransform:
            cost = statistics.rankDifference / float(statistics.sampleCount);
            break;

        case kPatchCostCensusTransform:
        case kPatchCostADCensus:
        {
            float weight = method == kPatchCostCensusTransform
                ? 1.0 : clamp(adCensusWeight, 0.0, 1.0);
            if (weight > 0.0 && statistics.censusSampleCount <= 0)
            {
                return false;
            }

            float censusCost = statistics.censusSampleCount > 0
                ? float(statistics.censusDifferenceCount) /
                    float(statistics.censusSampleCount)
                : 0.0;
            float absoluteCost =
                statistics.absoluteDifference / float(statistics.sampleCount);
            cost = (1.0 - weight) * absoluteCost + weight * censusCost;
            break;
        }

        default:
            return false;
    }

    return IsFiniteFloat(cost);
}

bool SampleBilinearCpuEquivalent(sampler2D image, vec2 pixel, out vec4 color)
{
	ivec2 size = textureSize(image, 0);
#ifdef GRAPHICSAPP_PLANE_SWEEP_PYRAMID
    if (pixel.x >= -1e-4 && pixel.x <= float(size.x - 1) + 1e-4) pixel.x = clamp(pixel.x, 0.0, float(size.x - 1));
    if (pixel.y >= -1e-4 && pixel.y <= float(size.y - 1) + 1e-4) pixel.y = clamp(pixel.y, 0.0, float(size.y - 1));
#endif
	if (size.x <= 0 || size.y <= 0 || pixel.x < 0.0 || pixel.y < 0.0 || pixel.x > float(size.x - 1) || pixel.y > float(size.y - 1)) { color = vec4(0.0); return false; }
	int x0 = int(floor(pixel.x)); int y0 = int(floor(pixel.y)); int x1 = min(size.x - 1, x0 + 1); int y1 = min(size.y - 1, y0 + 1);
	float tx = pixel.x - float(x0); float ty = pixel.y - float(y0);
	vec4 c00 = texelFetch(image, ivec2(x0, y0), 0); vec4 c10 = texelFetch(image, ivec2(x1, y0), 0); vec4 c01 = texelFetch(image, ivec2(x0, y1), 0); vec4 c11 = texelFetch(image, ivec2(x1, y1), 0);
	PS_PRECISE vec4 top = c00 * (1.0 - tx) + c10 * tx;
    PS_PRECISE vec4 bottom = c01 * (1.0 - tx) + c11 * tx;
    PS_PRECISE vec4 sampled = top * (1.0 - ty) + bottom * ty; color = sampled; return true;
}

ivec2 SourceImageSize(uint sourceIndex)
{
	switch (sourceIndex)
	{
		case 0u: return textureSize(sourceImages[0], 0);
		case 1u: return textureSize(sourceImages[1], 0);
		case 2u: return textureSize(sourceImages[2], 0);
		case 3u: return textureSize(sourceImages[3], 0);
		case 4u: return textureSize(sourceImages[4], 0);
		case 5u: return textureSize(sourceImages[5], 0);
		case 6u: return textureSize(sourceImages[6], 0);
		case 7u: return textureSize(sourceImages[7], 0);
		default: return ivec2(0);
	}
}

bool SamplePlaneSweepPixelCpuEquivalent(sampler2D image, sampler2D mask, vec2 pixel, bool hasMask, bool maskUsesAlpha, out vec4 color)
{
	if (!SampleBilinearCpuEquivalent(image, pixel, color)) return false;
	if (!hasMask) return true;
	ivec2 imageSize = textureSize(image, 0); ivec2 maskSize = textureSize(mask, 0);
	float imageWidth = float(max(1, imageSize.x - 1)); float imageHeight = float(max(1, imageSize.y - 1));
	float maskWidth = float(max(1, maskSize.x - 1)); float maskHeight = float(max(1, maskSize.y - 1));
	vec2 maskPixel = vec2(pixel.x / imageWidth * maskWidth, pixel.y / imageHeight * maskHeight); vec4 maskColor;
	if (!SampleBilinearCpuEquivalent(mask, maskPixel, maskColor)) return false;
	color.a = maskUsesAlpha ? maskColor.a : maskColor.r * 0.299 + maskColor.g * 0.587 + maskColor.b * 0.114; return true;
}

bool ComputeTextureStandardDeviationCpuEquivalent(float x, float y, int patchRadius, float alphaThreshold, bool referenceHasMask, bool referenceMaskUsesAlpha, uint diagnosticIndex, out float standardDeviation, out uint acceptedSampleCount)
{
	precise float luminanceSum = 0.0; precise float luminanceSquaredSum = 0.0; int sampleCount = 0; uint diagnosticSampleCount = 0u;
	uint patchWidth = uint(patchRadius * 2 + 1);
	uint diagnosticStride = max(100u, 10u + patchWidth * patchWidth * 10u);
	for (int patchY = -patchRadius; patchY <= patchRadius; ++patchY)
	{
		for (int patchX = -patchRadius; patchX <= patchRadius; ++patchX)
		{
			vec4 color;
			if (!SamplePlaneSweepPixelCpuEquivalent(referenceImage, referenceMask, vec2(x + float(patchX), y + float(patchY)), referenceHasMask, referenceMaskUsesAlpha, color)) continue;
			if (color.a <= alphaThreshold) continue;
			precise float redLuminance = color.r * 0.299;
			precise float greenLuminance = color.g * 0.587;
			precise float blueLuminance = color.b * 0.114;
			precise float luminanceRedGreen = redLuminance + greenLuminance;
			precise float luminance = luminanceRedGreen + blueLuminance;
			precise float squaredLuminance = luminance * luminance;
			luminanceSum = luminanceSum + luminance;
			luminanceSquaredSum = luminanceSquaredSum + squaredLuminance;
#ifndef GRAPHICSAPP_PLANE_SWEEP_PYRAMID
			uint diagnosticBase = diagnosticIndex * diagnosticStride + 10u + diagnosticSampleCount * 10u;
			textureStdDevOutput.values[diagnosticBase] = floatBitsToUint(x + float(patchX));
			textureStdDevOutput.values[diagnosticBase + 1u] = floatBitsToUint(y + float(patchY));
			textureStdDevOutput.values[diagnosticBase + 2u] = floatBitsToUint(color.r);
			textureStdDevOutput.values[diagnosticBase + 3u] = floatBitsToUint(color.g);
			textureStdDevOutput.values[diagnosticBase + 4u] = floatBitsToUint(color.b);
			textureStdDevOutput.values[diagnosticBase + 5u] = floatBitsToUint(color.a);
			textureStdDevOutput.values[diagnosticBase + 6u] = floatBitsToUint(luminance);
			textureStdDevOutput.values[diagnosticBase + 7u] = floatBitsToUint(squaredLuminance);
			textureStdDevOutput.values[diagnosticBase + 8u] = floatBitsToUint(luminanceSum);
			textureStdDevOutput.values[diagnosticBase + 9u] = floatBitsToUint(luminanceSquaredSum);
			++diagnosticSampleCount;
#endif
			++sampleCount;
		}
	}
	acceptedSampleCount = uint(max(sampleCount, 0));
	if (sampleCount == 0) { standardDeviation = 0.0; return false; }
	precise float sampleCountFloat = float(sampleCount);
	precise float mean = luminanceSum / sampleCountFloat;
	precise float averageSquared = luminanceSquaredSum / sampleCountFloat;
	precise float meanSquared = mean * mean;
	precise float variance = averageSquared - meanSquared;
	precise float clampedVariance = max(0.0, variance);
	standardDeviation = sqrt(clampedVariance);
#ifndef GRAPHICSAPP_PLANE_SWEEP_PYRAMID
	uint diagnosticBase = diagnosticIndex * diagnosticStride;
	textureStdDevOutput.values[diagnosticBase] = uint(sampleCount);
	textureStdDevOutput.values[diagnosticBase + 1u] = floatBitsToUint(luminanceSum);
	textureStdDevOutput.values[diagnosticBase + 2u] = floatBitsToUint(luminanceSquaredSum);
	textureStdDevOutput.values[diagnosticBase + 3u] = floatBitsToUint(float(sampleCount));
	textureStdDevOutput.values[diagnosticBase + 4u] = floatBitsToUint(mean);
	textureStdDevOutput.values[diagnosticBase + 5u] = floatBitsToUint(meanSquared);
	textureStdDevOutput.values[diagnosticBase + 6u] = floatBitsToUint(luminanceSquaredSum / float(sampleCount));
	textureStdDevOutput.values[diagnosticBase + 7u] = floatBitsToUint(variance);
	textureStdDevOutput.values[diagnosticBase + 8u] = floatBitsToUint(clampedVariance);
	textureStdDevOutput.values[diagnosticBase + 9u] = floatBitsToUint(standardDeviation);
#endif
	return IsFiniteFloat(standardDeviation);
}

bool SampleSource(uint sourceIndex, vec2 pixel, out vec4 color)
{
	bool hasMask = sourceMetadata.values[sourceIndex * 2u] != 0u; bool maskUsesAlpha = sourceMetadata.values[sourceIndex * 2u + 1u] != 0u;
	switch (sourceIndex)
	{
		case 0u: return SamplePlaneSweepPixelCpuEquivalent(sourceImages[0], sourceMasks[0], pixel, hasMask, maskUsesAlpha, color);
		case 1u: return SamplePlaneSweepPixelCpuEquivalent(sourceImages[1], sourceMasks[1], pixel, hasMask, maskUsesAlpha, color);
		case 2u: return SamplePlaneSweepPixelCpuEquivalent(sourceImages[2], sourceMasks[2], pixel, hasMask, maskUsesAlpha, color);
		case 3u: return SamplePlaneSweepPixelCpuEquivalent(sourceImages[3], sourceMasks[3], pixel, hasMask, maskUsesAlpha, color);
		case 4u: return SamplePlaneSweepPixelCpuEquivalent(sourceImages[4], sourceMasks[4], pixel, hasMask, maskUsesAlpha, color);
		case 5u: return SamplePlaneSweepPixelCpuEquivalent(sourceImages[5], sourceMasks[5], pixel, hasMask, maskUsesAlpha, color);
		case 6u: return SamplePlaneSweepPixelCpuEquivalent(sourceImages[6], sourceMasks[6], pixel, hasMask, maskUsesAlpha, color);
		case 7u: return SamplePlaneSweepPixelCpuEquivalent(sourceImages[7], sourceMasks[7], pixel, hasMask, maskUsesAlpha, color);
		default: color = vec4(0.0); return false;
	}
}

//Computes local rank using the existing reference/source mask sampling.
bool ComputeLocalRank(
    bool isReference,
    uint sourceIndex,
    vec2 pixel,
    float centerIntensity,
    int patchRadius,
    float alphaThreshold,
    int minimumOpaqueSamples,
    out float rank)
{
    rank = 0.0;
    int sampleCount = 1;
    int darkerCount = 0;

    for (int offsetY = -patchRadius; offsetY <= patchRadius; ++offsetY)
    {
        for (int offsetX = -patchRadius; offsetX <= patchRadius; ++offsetX)
        {
            if (offsetX == 0 && offsetY == 0)
            {
                continue;
            }

            vec2 samplePixel = pixel + vec2(float(offsetX), float(offsetY));
            vec4 color;
            bool validSample;
            if (isReference)
            {
                validSample = SamplePlaneSweepPixelCpuEquivalent(
                    referenceImage, referenceMask, samplePixel,
                    sourceMetadata.values[16] != 0u,
                    sourceMetadata.values[17] != 0u, color);
            }
            else
            {
                validSample = SampleSource(sourceIndex, samplePixel, color);
            }

            if (!validSample || color.a <= alphaThreshold)
            {
                continue;
            }

            darkerCount += IsPatchIntensityDarker(
                ComputePatchGrayscale(color), centerIntensity) ? 1 : 0;
            ++sampleCount;
        }
    }

    if (sampleCount < minimumOpaqueSamples || sampleCount <= 1)
    {
        return false;
    }

    rank = float(darkerCount) / float(sampleCount - 1);
    return true;
}

vec3 UnprojectPixelCpuEquivalent(GpuCameraPose camera, GpuCameraIntrinsics intrinsic, float x, float y, float depth)
{
	float targetX = (x + 0.5 - intrinsic.cx) / intrinsic.fx; float targetY = (y + 0.5 - intrinsic.cy) / intrinsic.fy; float localX = targetX; float localY = targetY;
	for (int iteration = 0; iteration < 5; ++iteration)
	{
		float radiusSquared = localX * localX + localY * localY; float radial = 1.0 + intrinsic.k1 * radiusSquared + intrinsic.k2 * radiusSquared * radiusSquared;
		float distortedX = localX * radial + 2.0 * intrinsic.p1 * localX * localY + intrinsic.p2 * (radiusSquared + 2.0 * localX * localX);
		float distortedY = localY * radial + intrinsic.p1 * (radiusSquared + 2.0 * localY * localY) + 2.0 * intrinsic.p2 * localX * localY;
		localX += targetX - distortedX; localY += targetY - distortedY;
	}
	PS_PRECISE vec3 point = camera.position.xyz + camera.right.xyz * (localX * depth) - camera.up.xyz * (localY * depth) + camera.forward.xyz * depth;
    return point;
}

bool ProjectPointCpuEquivalent(GpuCameraPose camera, GpuCameraIntrinsics intrinsic, vec3 point, out float x, out float y, out float depth)
{
	precise vec3 offset = point - camera.position.xyz; precise float depthXY = offset.x * camera.forward.x + offset.y * camera.forward.y; depth = depthXY + offset.z * camera.forward.z;
	if (depth <= 0.0001) { x = 0.0; y = 0.0; return false; }
	precise float localXNumeratorXY = offset.x * camera.right.x + offset.y * camera.right.y; precise float localX = (localXNumeratorXY + offset.z * camera.right.z) / depth;
	precise float localYNumeratorXY = offset.x * camera.up.x + offset.y * camera.up.y; precise float localYNumerator = localYNumeratorXY + offset.z * camera.up.z; precise float localY = -localYNumerator / depth;
	precise float radiusSquared = localX * localX + localY * localY; precise float radial = 1.0 + intrinsic.k1 * radiusSquared + intrinsic.k2 * radiusSquared * radiusSquared;
	precise float distortedX = localX * radial + 2.0 * intrinsic.p1 * localX * localY + intrinsic.p2 * (radiusSquared + 2.0 * localX * localX);
	precise float distortedY = localY * radial + intrinsic.p1 * (radiusSquared + 2.0 * localY * localY) + 2.0 * intrinsic.p2 * localX * localY;
	x = intrinsic.fx * distortedX + intrinsic.cx; y = intrinsic.fy * distortedY + intrinsic.cy;
#ifdef GRAPHICSAPP_PLANE_SWEEP_PYRAMID
    x -= 0.5; y -= 0.5; // Pixel indices, matching the +0.5 in unprojection.
    // Roundoff at an image edge must not change patch validity between devices.
    if (x >= -1e-4 && x <= float(intrinsic.width - 1) + 1e-4) x = clamp(x, 0.0, float(intrinsic.width - 1));
    if (y >= -1e-4 && y <= float(intrinsic.height - 1) + 1e-4) y = clamp(y, 0.0, float(intrinsic.height - 1));
#endif
    return IsFiniteFloat(x) && IsFiniteFloat(y) && IsFiniteFloat(depth);
}

bool ComputePlaneCostCpuEquivalent(float referenceX, float referenceY, float depth, out float cost, out uint minimumOpaqueSamples, out uint acceptedSampleCount)
{
	int patchWidth = paramsBuffer.value.patchRadius * 2 + 1;
	int patchArea = patchWidth * patchWidth;
	minimumOpaqueSamples = uint(max(1, int(ceil(float(patchArea) * paramsBuffer.value.minimumOpaquePatchRatio))));
	acceptedSampleCount = 0u;
	float totalCost = 0.0; int validSourceViews = 0;
	uint method = paramsBuffer.value.patchCostMethod;
	bool usesCorrelation =
	    method == kPatchCostNormalizedCrossCorrelation ||
	    method == kPatchCostZeroMeanNormalizedCrossCorrelation;
	bool usesRank = method == kPatchCostRankTransform;
	bool usesCensus =
	    method == kPatchCostCensusTransform ||
	    (method == kPatchCostADCensus &&
	     paramsBuffer.value.adCensusWeight > 0.0);

	float referenceCenterIntensity = 0.0;
	if (usesCensus)
	{
	    vec4 referenceCenterColor;
	    if (!SamplePlaneSweepPixelCpuEquivalent(
	            referenceImage, referenceMask,
	            vec2(referenceX, referenceY),
	            sourceMetadata.values[16] != 0u,
	            sourceMetadata.values[17] != 0u,
	            referenceCenterColor) ||
	        referenceCenterColor.a <= paramsBuffer.value.alphaThreshold)
	    {
	        cost = 0.0;
	        return false;
	    }

	    referenceCenterIntensity = ComputePatchGrayscale(referenceCenterColor);
	}

	for (uint sourceIndex = 0u; sourceIndex < paramsBuffer.value.sourceCount; ++sourceIndex)
	{
		float sourceCost = 0.0; int sampleCount = 0;
#ifdef GRAPHICSAPP_PLANE_SWEEP_FP64
		PlaneSweepPatchStatistics statistics =
		    PlaneSweepPatchStatistics(
		        0, 0.0, 0.0, 0, 0,
		        0.0LF, 0.0LF, 0.0LF, 0.0LF, 0.0LF);
#else
		PlaneSweepPatchStatistics statistics =
		    PlaneSweepPatchStatistics(0, 0.0, 0.0, 0, 0);
#endif
		float sourceCenterIntensity = 0.0;
		bool sourceCenterValid = true;

		if (usesCensus)
		{
		    vec3 centerPoint = UnprojectPixelCpuEquivalent(
		        poses.values[0], intrinsics.values[0],
		        referenceX, referenceY, depth);
		    float centerX;
		    float centerY;
		    float centerDepth;
		    vec4 centerColor;

		    sourceCenterValid =
		        ProjectPointCpuEquivalent(
		            poses.values[sourceIndex + 1u],
		            intrinsics.values[sourceIndex + 1u],
		            centerPoint, centerX, centerY, centerDepth) &&
		        SampleSource(sourceIndex, vec2(centerX, centerY), centerColor) &&
		        centerColor.a > paramsBuffer.value.alphaThreshold;

		    if (sourceCenterValid)
		    {
		        sourceCenterIntensity = ComputePatchGrayscale(centerColor);
		    }
		}

		for (int patchY = -paramsBuffer.value.patchRadius; sourceCenterValid && patchY <= paramsBuffer.value.patchRadius; ++patchY)
		{
			for (int patchX = -paramsBuffer.value.patchRadius; patchX <= paramsBuffer.value.patchRadius; ++patchX)
			{
				float patchReferenceX = referenceX + float(patchX); float patchReferenceY = referenceY + float(patchY); vec4 patchReferenceColor;
				if (!SamplePlaneSweepPixelCpuEquivalent(referenceImage, referenceMask, vec2(patchReferenceX, patchReferenceY), sourceMetadata.values[16] != 0u, sourceMetadata.values[17] != 0u, patchReferenceColor)) continue;
				if (patchReferenceColor.a <= paramsBuffer.value.alphaThreshold) continue;
				vec3 point = UnprojectPixelCpuEquivalent(poses.values[0], intrinsics.values[0], patchReferenceX, patchReferenceY, depth);
				float sourceX; float sourceY; float sourceDepth;
				if (!ProjectPointCpuEquivalent(poses.values[sourceIndex + 1u], intrinsics.values[sourceIndex + 1u], point, sourceX, sourceY, sourceDepth)) continue;
				ivec2 sourceSize = SourceImageSize(sourceIndex);
				if (!(sourceX >= 0.0 && sourceY >= 0.0 && sourceX < float(sourceSize.x) && sourceY < float(sourceSize.y))) continue;
				vec4 sourceColor;
				if (!SampleSource(sourceIndex, vec2(sourceX, sourceY), sourceColor) || sourceColor.a <= paramsBuffer.value.alphaThreshold) continue;
				float referenceIntensity = usesCorrelation || usesRank || usesCensus
				    ? ComputePatchGrayscale(patchReferenceColor) : 0.0;
				float sourceIntensity = usesCorrelation || usesRank || usesCensus
				    ? ComputePatchGrayscale(sourceColor) : 0.0;

				if (usesRank)
				{
				    float referenceRank;
				    float sourceRank;
				    if (!ComputeLocalRank(
				            true, 0u, vec2(patchReferenceX, patchReferenceY),
				            referenceIntensity, paramsBuffer.value.patchRadius,
				            paramsBuffer.value.alphaThreshold,
				            int(minimumOpaqueSamples), referenceRank) ||
				        !ComputeLocalRank(
				            false, sourceIndex, vec2(sourceX, sourceY),
				            sourceIntensity, paramsBuffer.value.patchRadius,
				            paramsBuffer.value.alphaThreshold,
				            int(minimumOpaqueSamples), sourceRank))
				    {
				        continue;
				    }

				    statistics.rankDifference += abs(referenceRank - sourceRank);
				}

#ifdef GRAPHICSAPP_PLANE_SWEEP_FP64
				if (usesCorrelation)
				{
				    double referenceValue = double(referenceIntensity);
				    double sourceValue = double(sourceIntensity);
				    precise double referenceSquared = referenceValue * referenceValue;
				    precise double sourceSquared = sourceValue * sourceValue;
				    precise double product = referenceValue * sourceValue;

				    statistics.referenceSum += referenceValue;
				    statistics.sourceSum += sourceValue;
				    statistics.referenceSquaredSum += referenceSquared;
				    statistics.sourceSquaredSum += sourceSquared;
				    statistics.productSum += product;
				}
#endif

				if (usesCensus && (patchX != 0 || patchY != 0))
				{
				    bool referenceBit =
				        IsPatchIntensityDarker(referenceIntensity, referenceCenterIntensity);
				    bool sourceBit =
				        IsPatchIntensityDarker(sourceIntensity, sourceCenterIntensity);
				    statistics.censusDifferenceCount += referenceBit != sourceBit ? 1 : 0;
				    ++statistics.censusSampleCount;
				}

				float delta = (abs(patchReferenceColor.r - sourceColor.r) + abs(patchReferenceColor.g - sourceColor.g) + abs(patchReferenceColor.b - sourceColor.b)) / 3.0;
				++sampleCount;
				statistics.absoluteDifference += delta;
				++statistics.sampleCount;
			}
		}
		if (sourceIndex == 0u) acceptedSampleCount = uint(sampleCount);
		if (sampleCount >= int(minimumOpaqueSamples) &&
		    ComputePatchCost(
		        statistics, method,
		        paramsBuffer.value.adCensusWeight, sourceCost))
		{
		    totalCost += sourceCost;
		    ++validSourceViews;
		}
	}
	if (validSourceViews < paramsBuffer.value.minValidSourceViews) { cost = 0.0; return false; }
	cost = totalCost / float(validSourceViews); return IsFiniteFloat(cost);
}

