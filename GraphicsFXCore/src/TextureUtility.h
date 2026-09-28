///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	TextureUtility.h
//	Description	:
//		Defines texture loading and saving helpers using Windows WIC.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace graphicsFX
{
    //Defines supported texture file types.
    enum class TextureFileType
    {
        Png, Bmp, Jpg
    };

    //Stores decoded texture pixel data.
    struct TextureData
    {
        uint32_t m_width{ 0 }; //Stores the texture width.
        uint32_t m_height{ 0 }; //Stores the texture height.
        uint32_t m_channelCount{ 4 }; //Stores the number of color channels.
        std::vector<uint8_t> m_pixels; //Stores RGBA pixel bytes.
    };

    //Loads a texture from disk as RGBA pixels.
    bool LoadTexture(const std::string &fileName, TextureData &texture);

    //Saves RGBA pixels to disk.
    bool SaveTexture(const std::string &fileName, const TextureData &texture, TextureFileType fileType = TextureFileType::Png);
}

