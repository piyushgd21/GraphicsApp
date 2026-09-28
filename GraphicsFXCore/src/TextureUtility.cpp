///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	TextureUtility.cpp
//	Description	:
//		Implements texture loading and saving helpers using Windows WIC.
//
//
///////////////////////////////////////////////////////////////////////

#include "TextureUtility.h"

#include "global.h"
#include "PathUtility.h"
#include "StringUtility.h"

#include <Windows.h>
#include <wincodec.h>
#include <wrl/client.h>

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace graphicsFX
{
    //Defines private helpers for texture loading and saving.
    class TextureFileUtility final
    {

        public:

            TextureFileUtility(void) = delete;

            //Initializes COM while texture loading or saving is active.
            class ComInitializer
            {

                public:

                    //Initializes COM for WIC work.
                    ComInitializer(void)
                    {
                        const HRESULT result = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
                        if (result == RPC_E_CHANGED_MODE)
                        {
                            return;
                        }

                        if (FAILED(result))
                        {
                            throw std::runtime_error("Failed to initialize COM for WIC.");
                        }

                        m_shouldUninitialize = true;
                    }

                    //Releases COM initialization when owned.
                    ~ComInitializer(void)
                    {
                        if (m_shouldUninitialize)
                        {
                            CoUninitialize();
                        }
                    }

                    ComInitializer(const ComInitializer &) = delete;
                    ComInitializer &operator=(const ComInitializer &) = delete;

                private:

                    bool m_shouldUninitialize{ false }; //Stores whether this object owns COM cleanup.
            };

            //Throws a runtime error when an HRESULT fails.
            static void CheckResult(HRESULT result, const char *messagePtr);

            //Returns texture path candidates.
            static std::vector<std::filesystem::path> TexturePathCandidates(const std::string &fileName);

            //Returns the first existing texture path.
            static std::filesystem::path FindTexturePath(const std::string &fileName);

            //Creates a WIC imaging factory.
            static Microsoft::WRL::ComPtr<IWICImagingFactory> CreateFactory(void);

            //Returns the WIC container id for a texture file type.
            static const GUID &ContainerForType(TextureFileType fileType);

            //Returns the WIC pixel format for a texture file type.
            static const WICPixelFormatGUID &PixelFormatForType(TextureFileType fileType);

            //Returns the bytes per pixel for a texture file type.
            static uint32_t BytesPerPixelForType(TextureFileType fileType);

            //Checks whether a texture has valid RGBA data.
            static void ValidateTexture(const TextureData &texture);

            //Builds output pixels in the requested WIC format.
            static std::vector<uint8_t> ConvertPixelsForSave(const TextureData &texture, TextureFileType fileType);
    };

        //Throws a runtime error when an HRESULT fails.
        void TextureFileUtility::CheckResult(HRESULT result, const char *messagePtr)
        {
            if (FAILED(result))
            {
                throw std::runtime_error(messagePtr);
            }
        }

        //Returns texture path candidates.
        std::vector<std::filesystem::path> TextureFileUtility::TexturePathCandidates(const std::string &fileName)
        {
            std::vector<std::filesystem::path> candidates;
            candidates.push_back(PathUtility::TexturePath(fileName));
            return candidates;
        }

        //Returns the first existing texture path.
        std::filesystem::path TextureFileUtility::FindTexturePath(const std::string &fileName)
        {
            std::error_code errorCode;
            for (const auto &candidate : TexturePathCandidates(fileName))
            {
                if (std::filesystem::exists(candidate, errorCode))
                {
                    return candidate;
                }
            }

            return std::filesystem::path(fileName);
        }

        //Creates a WIC imaging factory.
        Microsoft::WRL::ComPtr<IWICImagingFactory> TextureFileUtility::CreateFactory(void)
        {
            Microsoft::WRL::ComPtr<IWICImagingFactory> factoryPtr;
            CheckResult(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&factoryPtr)), "Failed to create WIC imaging factory.");
            return factoryPtr;
        }

        //Returns the WIC container id for a texture file type.
        const GUID &TextureFileUtility::ContainerForType(TextureFileType fileType)
        {
            switch (fileType)
            {
                case TextureFileType::Bmp: return GUID_ContainerFormatBmp;
                case TextureFileType::Jpg: return GUID_ContainerFormatJpeg;
                case TextureFileType::Png:
                default: return GUID_ContainerFormatPng;
            }
        }

        //Returns the WIC pixel format for a texture file type.
        const WICPixelFormatGUID &TextureFileUtility::PixelFormatForType(TextureFileType fileType)
        {
            switch (fileType)
            {
                case TextureFileType::Bmp: return GUID_WICPixelFormat32bppBGRA;
                case TextureFileType::Jpg: return GUID_WICPixelFormat24bppBGR;
                case TextureFileType::Png:
                default: return GUID_WICPixelFormat32bppBGRA;
            }
        }

        //Returns the bytes per pixel for a texture file type.
        uint32_t TextureFileUtility::BytesPerPixelForType(TextureFileType fileType)
        {
            if (fileType == TextureFileType::Jpg)
            {
                return 3U;
            }

            return 4U;
        }

        //Checks whether a texture has valid RGBA data.
        void TextureFileUtility::ValidateTexture(const TextureData &texture)
        {
            if (texture.m_width == 0 || texture.m_height == 0 || texture.m_channelCount != 4U)
            {
                throw std::runtime_error("Texture must contain RGBA pixels.");
            }

            const std::size_t width = static_cast<std::size_t>(texture.m_width);
            const std::size_t height = static_cast<std::size_t>(texture.m_height);
            if (width > std::numeric_limits<std::size_t>::max() / height / 4U)
            {
                throw std::runtime_error("Texture is too large.");
            }

            const std::size_t expectedSize = width * height * 4U;
            if (texture.m_pixels.size() != expectedSize)
            {
                throw std::runtime_error("Texture pixel data size is invalid.");
            }
        }

        //Builds output pixels in the requested WIC format.
        std::vector<uint8_t> TextureFileUtility::ConvertPixelsForSave(const TextureData &texture, TextureFileType fileType)
        {
            ValidateTexture(texture);

            const uint32_t bytesPerPixel = BytesPerPixelForType(fileType);
            const std::size_t pixelCount = static_cast<std::size_t>(texture.m_width) * static_cast<std::size_t>(texture.m_height);
            std::vector<uint8_t> converted(pixelCount * bytesPerPixel);

            for (std::size_t i = 0; i < pixelCount; ++i)
            {
                const uint8_t red = texture.m_pixels[(i * 4U) + 0U];
                const uint8_t green = texture.m_pixels[(i * 4U) + 1U];
                const uint8_t blue = texture.m_pixels[(i * 4U) + 2U];
                const uint8_t alpha = texture.m_pixels[(i * 4U) + 3U];

                converted[(i * bytesPerPixel) + 0U] = blue;
                converted[(i * bytesPerPixel) + 1U] = green;
                converted[(i * bytesPerPixel) + 2U] = red;

                if (bytesPerPixel == 4U)
                {
                    converted[(i * bytesPerPixel) + 3U] = alpha;
                }
            }

            return converted;
        }
    //Loads a texture from disk as RGBA pixels.
    bool LoadTexture(const std::string &fileName, TextureData &texture)
    {
        try
        {
            TextureFileUtility::ComInitializer comInitializer;
            const std::wstring wideFileName = TextureFileUtility::FindTexturePath(fileName).wstring();
            const auto factoryPtr = TextureFileUtility::CreateFactory();

            Microsoft::WRL::ComPtr<IWICBitmapDecoder> decoderPtr;
            TextureFileUtility::CheckResult(factoryPtr->CreateDecoderFromFilename(wideFileName.c_str(), nullptr, GENERIC_READ, WICDecodeMetadataCacheOnLoad, &decoderPtr), "Failed to create WIC image decoder.");

            Microsoft::WRL::ComPtr<IWICBitmapFrameDecode> framePtr;
            TextureFileUtility::CheckResult(decoderPtr->GetFrame(0, &framePtr), "Failed to read WIC image frame.");

            Microsoft::WRL::ComPtr<IWICFormatConverter> converterPtr;
            TextureFileUtility::CheckResult(factoryPtr->CreateFormatConverter(&converterPtr), "Failed to create WIC format converter.");
            TextureFileUtility::CheckResult(
                converterPtr->Initialize(framePtr.Get(), GUID_WICPixelFormat32bppRGBA, WICBitmapDitherTypeNone, nullptr, 0.0, WICBitmapPaletteTypeCustom),
                "Failed to convert WIC image to RGBA."
            );

            UINT width = 0;
            UINT height = 0;
            TextureFileUtility::CheckResult(converterPtr->GetSize(&width, &height), "Failed to get WIC image size.");

            if (width == 0 || height == 0)
            {
                throw std::runtime_error("Texture dimensions are invalid.");
            }

            if (static_cast<std::size_t>(width) > std::numeric_limits<std::size_t>::max() / static_cast<std::size_t>(height) / 4U)
            {
                throw std::runtime_error("Texture is too large.");
            }

            const UINT stride = width * 4U;
            const UINT byteCount = stride * height;
            std::vector<uint8_t> pixels(byteCount);

            TextureFileUtility::CheckResult(converterPtr->CopyPixels(nullptr, stride, byteCount, pixels.data()), "Failed to copy WIC image pixels.");

            texture.m_width = width;
            texture.m_height = height;
            texture.m_channelCount = 4U;
            texture.m_pixels = std::move(pixels);
            return true;
        }
        catch (...)
        {
            texture = {};
            return false;
        }
    }

    //Saves RGBA pixels to disk.
    bool SaveTexture(const std::string &fileName, const TextureData &texture, TextureFileType fileType)
    {
        try
        {
            TextureFileUtility::ComInitializer comInitializer;
            const std::wstring wideFileName = StringUtility::ToWideOrThrow(fileName, "Failed to convert file name for WIC.");
            const auto factoryPtr = TextureFileUtility::CreateFactory();
            std::vector<uint8_t> convertedPixels = TextureFileUtility::ConvertPixelsForSave(texture, fileType);
            WICPixelFormatGUID pixelFormat = TextureFileUtility::PixelFormatForType(fileType);

            Microsoft::WRL::ComPtr<IWICStream> streamPtr;
            TextureFileUtility::CheckResult(factoryPtr->CreateStream(&streamPtr), "Failed to create WIC stream.");
            TextureFileUtility::CheckResult(streamPtr->InitializeFromFilename(wideFileName.c_str(), GENERIC_WRITE), "Failed to open WIC output file.");

            Microsoft::WRL::ComPtr<IWICBitmapEncoder> encoderPtr;
            TextureFileUtility::CheckResult(factoryPtr->CreateEncoder(TextureFileUtility::ContainerForType(fileType), nullptr, &encoderPtr), "Failed to create WIC encoder.");
            TextureFileUtility::CheckResult(encoderPtr->Initialize(streamPtr.Get(), WICBitmapEncoderNoCache), "Failed to initialize WIC encoder.");

            Microsoft::WRL::ComPtr<IWICBitmapFrameEncode> framePtr;
            Microsoft::WRL::ComPtr<IPropertyBag2> propertyBagPtr;
            TextureFileUtility::CheckResult(encoderPtr->CreateNewFrame(&framePtr, &propertyBagPtr), "Failed to create WIC image frame.");
            TextureFileUtility::CheckResult(framePtr->Initialize(propertyBagPtr.Get()), "Failed to initialize WIC image frame.");
            TextureFileUtility::CheckResult(framePtr->SetSize(texture.m_width, texture.m_height), "Failed to set WIC image size.");
            TextureFileUtility::CheckResult(framePtr->SetPixelFormat(&pixelFormat), "Failed to set WIC pixel format.");

            if (!InlineIsEqualGUID(pixelFormat, TextureFileUtility::PixelFormatForType(fileType)))
            {
                throw std::runtime_error("WIC encoder did not accept the requested pixel format.");
            }

            const uint32_t bytesPerPixel = TextureFileUtility::BytesPerPixelForType(fileType);
            const UINT stride = texture.m_width * bytesPerPixel;
            const UINT byteCount = stride * texture.m_height;
            TextureFileUtility::CheckResult(framePtr->WritePixels(texture.m_height, stride, byteCount, convertedPixels.data()), "Failed to write WIC image pixels.");

            TextureFileUtility::CheckResult(framePtr->Commit(), "Failed to commit WIC image frame.");
            TextureFileUtility::CheckResult(encoderPtr->Commit(), "Failed to commit WIC encoder.");
            return true;
        }
        catch (...)
        {
            return false;
        }
    }
}

