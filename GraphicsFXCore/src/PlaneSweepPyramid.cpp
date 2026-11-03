#include "PlaneSweepPyramid.h"
#include "TextureUtility.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace graphicsFX
{
    namespace
    {
        SweepUtility::PlaneSweepImage ResizeImage(const SweepUtility::PlaneSweepImage &source,
            uint32_t width, uint32_t height, const ColmapSetup &setup)
        {
            const auto validTexture = [](const TextureData &texture)
            { return texture.m_width && texture.m_height && texture.m_channelCount == 4 &&
                texture.m_pixels.size() == size_t(texture.m_width) * texture.m_height * 4; };
            if (!width || !height || !source.m_texture.m_width || !source.m_texture.m_height ||
                !source.m_intrinsic.m_width || !source.m_intrinsic.m_height || !validTexture(source.m_texture) ||
                (source.m_hasMask && !validTexture(source.m_maskTexture)))
                throw std::invalid_argument("Invalid pyramid image dimensions.");
            SweepUtility::PlaneSweepImage result;
            result.m_camera = source.m_camera;
            result.m_cameraIndex = source.m_cameraIndex;
            result.m_intrinsic = source.m_intrinsic;
            const double sx = double(width) / source.m_intrinsic.m_width;
            const double sy = double(height) / source.m_intrinsic.m_height;
            result.m_intrinsic.m_width = width; result.m_intrinsic.m_height = height;
            result.m_intrinsic.m_fx *= sx; result.m_intrinsic.m_cx *= sx;
            result.m_intrinsic.m_fy *= sy; result.m_intrinsic.m_cy *= sy;
            // Area filtering is the anti-aliasing low-pass filter. Masked colors never bleed
            // into valid colors; coverage is retained in alpha, with conservative rejection.
            auto &texture = result.m_texture;
            texture.m_width = width; texture.m_height = height;
            texture.m_pixels.resize(size_t(width) * height * 4);
            const double rx = double(source.m_texture.m_width) / width;
            const double ry = double(source.m_texture.m_height) / height;
            const bool directMask = !source.m_hasMask || (source.m_maskTexture.m_width == source.m_texture.m_width &&
                source.m_maskTexture.m_height == source.m_texture.m_height);
            for (uint32_t y = 0; y < height; ++y)
                for (uint32_t x = 0; x < width; ++x)
                {
                    const double left = x * rx, right = (x + 1) * rx;
                    const double top = y * ry, bottom = (y + 1) * ry;
                    double weight = 0, rgb[3]{};
                    for (int iy = int(std::floor(top)); iy < int(std::ceil(bottom)); ++iy)
                        for (int ix = int(std::floor(left)); ix < int(std::ceil(right)); ++ix)
                        {
                            const double area = std::max(0.0, std::min(right, double(ix + 1)) - std::max(left, double(ix))) *
                                std::max(0.0, std::min(bottom, double(iy + 1)) - std::max(top, double(iy)));
                            Float4 color;
                            if (directMask && ix >= 0 && iy >= 0 && ix < int(source.m_texture.m_width) && iy < int(source.m_texture.m_height))
                            {
                                const size_t input = (size_t(iy) * source.m_texture.m_width + size_t(ix)) * 4;
                                const auto &bytes = source.m_texture.m_pixels;
                                color = { bytes[input] / 255.0f, bytes[input + 1] / 255.0f, bytes[input + 2] / 255.0f, bytes[input + 3] / 255.0f };
                                if (source.m_hasMask)
                                {
                                    const auto &mask = source.m_maskTexture.m_pixels;
                                    color.w = source.m_maskUsesAlpha ? mask[input + 3] / 255.0f :
                                        (mask[input] / 255.0f * 0.299f + mask[input + 1] / 255.0f * 0.587f + mask[input + 2] / 255.0f * 0.114f);
                                }
                            }
                            else if (!SweepUtility::SamplePlaneSweepPixel(source, float(ix), float(iy), color)) continue;
                            if (color.w <= setup.m_planeSweepAlphaThreshold) continue;
                            const double w = area * color.w;
                            weight += w; rgb[0] += color.x * w; rgb[1] += color.y * w; rgb[2] += color.z * w;
                        }
                    const size_t i = (size_t(y) * width + x) * 4;
                    const double coverage = std::clamp(weight / (rx * ry), 0.0, 1.0);
                    if (weight > 0 && coverage >= setup.m_planeSweepMinimumOpaquePatchRatio)
                    {
                        for (int c = 0; c < 3; ++c)
                            texture.m_pixels[i + c] = uint8_t(std::lround(std::clamp(rgb[c] / weight, 0.0, 1.0) * 255));
                        texture.m_pixels[i + 3] = uint8_t(std::lround(coverage * 255));
                    }
                }
            return result;
        }
    }

    std::vector<PlaneSweepPyramidLevel> BuildPlaneSweepPyramid(
        const SweepUtility::PlaneSweepImage &reference,
        const std::vector<SweepUtility::PlaneSweepImage> &sources, const ColmapSetup &setup)
    {
        if (sources.empty() || setup.m_planeSweepPyramidLevels < 1 || setup.m_planeSweepPyramidLevels > 6 ||
            setup.m_planeSweepImageScale <= 0 || setup.m_planeSweepImageScale > 1 ||
            setup.m_planeSweepLocalSamples < 3 || setup.m_planeSweepLocalSamples > 33 ||
            setup.m_planeSweepLocalSamples % 2 == 0)
            throw std::invalid_argument("Invalid coarse-to-fine configuration.");
        std::vector<PlaneSweepPyramidLevel> result;
        auto resized = [&](const SweepUtility::PlaneSweepImage &image, float scale)
        {
            return ResizeImage(image,
                std::max(1u, uint32_t(std::lround(image.m_texture.m_width * scale))),
                std::max(1u, uint32_t(std::lround(image.m_texture.m_height * scale))), setup);
        };
        PlaneSweepPyramidLevel fine;
        fine.m_reference = resized(reference, setup.m_planeSweepImageScale);
        for (const auto &source : sources) fine.m_sources.push_back(resized(source, setup.m_planeSweepImageScale));
        result.push_back(std::move(fine));
        for (int i = 1; i < setup.m_planeSweepPyramidLevels; ++i)
        {
            const auto &previous = result.back();
            if (previous.m_reference.m_texture.m_width == 1 && previous.m_reference.m_texture.m_height == 1) break;
            PlaneSweepPyramidLevel coarse;
            coarse.m_reference = resized(previous.m_reference, 0.5f);
            for (const auto &source : previous.m_sources) coarse.m_sources.push_back(resized(source, 0.5f));
            result.push_back(std::move(coarse));
        }
        std::reverse(result.begin(), result.end());
        return result;
    }

    SweepUtility::PlaneSweepDepthMap MakePlaneSweepPyramidDepthMap(
        const SweepUtility::PlaneSweepImage &reference, uint32_t width, uint32_t height,
        const std::vector<PlaneSweepPyramidState> &state)
    {
        SweepUtility::PlaneSweepDepthMap result;
        result.m_camera = reference.m_camera; result.m_intrinsic = reference.m_intrinsic;
        result.m_imageWidth = reference.m_texture.m_width; result.m_imageHeight = reference.m_texture.m_height;
        result.m_width = width; result.m_height = height;
        result.m_depthValues.reserve(state.size());
        for (const auto &pixel : state) result.m_depthValues.push_back(pixel[0]);
        return result;
    }
}
