///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	GaussianSplatUtility.cpp
//	Description	:
//		Implements Gaussian splat PLY loading helpers.
//
///////////////////////////////////////////////////////////////////////

#include "GaussianSplatUtility.h"

#include "PathUtility.h"
#include "StringUtility.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace graphicsFX
{
    //Copies a Gaussian splat.
    GaussianSplatData::GaussianSplatData(const GaussianSplatData &other) :
        m_position(other.m_position),
        m_color(other.m_color),
        m_scale(other.m_scale),
        m_rotation(other.m_rotation),
        m_optionalDataPtr(other.m_optionalDataPtr ? std::make_unique<GaussianSplatOptionalData>(*other.m_optionalDataPtr) : nullptr)
    {
    }

    //Copies a Gaussian splat.
    GaussianSplatData &GaussianSplatData::operator=(const GaussianSplatData &other)
    {
        if (this == &other)
        {
            return *this;
        }

        m_position = other.m_position;
        m_color = other.m_color;
        m_scale = other.m_scale;
        m_rotation = other.m_rotation;
        m_optionalDataPtr = other.m_optionalDataPtr ? std::make_unique<GaussianSplatOptionalData>(*other.m_optionalDataPtr) : nullptr;
        return *this;
    }

    //Loads Gaussian splat PLY data from a folder.
    GaussianSplatReadResult GaussianSplatUtility::GaussianSplatFileUtility::LoadFolder(const GaussianSplatSetup &setup)
    {
        GaussianSplatReadResult result;
        if (!setup.m_enabled)
        {
            return result;
        }

        std::string errorMessage;
        const std::vector<std::filesystem::path> files = FindFiles(setup, errorMessage);
        if (files.empty())
        {
            result.m_errorMessage = errorMessage.empty() ? "No Gaussian splat PLY files were found." : errorMessage;
            return result;
        }

        const std::size_t maxSplats = setup.m_maxSplats <= 0 ? std::numeric_limits<std::size_t>::max() : static_cast<std::size_t>(setup.m_maxSplats);
        const bool loadFirst = LoadModeIsFirst(setup.m_loadMode);

        for (const std::filesystem::path &filePath : files)
        {
            const std::size_t remainingSplats = maxSplats - result.m_splats.size();
            GaussianSplatReadResult fileResult = PLYFileUtility::ReadPlyFile(filePath, setup, remainingSplats);
            result.m_errorMessage = fileResult.m_errorMessage;

            result.m_splats.insert(result.m_splats.end(), std::make_move_iterator(fileResult.m_splats.begin()), std::make_move_iterator(fileResult.m_splats.end()));
            result.m_fileNames.insert(result.m_fileNames.end(), std::make_move_iterator(fileResult.m_fileNames.begin()), std::make_move_iterator(fileResult.m_fileNames.end()));
            result.m_filesRead += fileResult.m_filesRead;

            if (loadFirst || result.m_splats.size() >= maxSplats)
            {
                break;
            }
        }

        return result;
    }

    //Loads Gaussian splat PLY data from one file.
    GaussianSplatReadResult GaussianSplatUtility::GaussianSplatFileUtility::LoadFile(const std::filesystem::path &filePath, const GaussianSplatSetup &setup)
    {
        const std::size_t maxSplats = setup.m_maxSplats <= 0 ? std::numeric_limits<std::size_t>::max() : static_cast<std::size_t>(setup.m_maxSplats);
        return PLYFileUtility::ReadPlyFile(filePath, setup, maxSplats);
    }

    //Returns files that match the splat setup.
    std::vector<std::filesystem::path> GaussianSplatUtility::GaussianSplatFileUtility::FindFiles(const GaussianSplatSetup &setup, std::string &errorMessage)
    {
        std::vector<std::filesystem::path> files;
        if (setup.m_folder.empty())
        {
            errorMessage = "Gaussian splat folder is empty.";
            return files;
        }

        const std::filesystem::path folderPath = PathUtility::ResolvePath(setup.m_folder);
        std::error_code errorCode;
        if (!std::filesystem::exists(folderPath, errorCode) || !std::filesystem::is_directory(folderPath, errorCode))
        {
            errorMessage = "Gaussian splat folder was not found: " + folderPath.string();
            return files;
        }

        std::filesystem::directory_iterator iterator(folderPath, errorCode);
        const std::filesystem::directory_iterator endIterator;
        while (!errorCode && iterator != endIterator)
        {
            const std::filesystem::directory_entry &entry = *iterator;
            if (entry.is_regular_file(errorCode) && FileMatchesPattern(entry.path(), setup.m_filePattern))
            {
                files.push_back(entry.path());
            }

            iterator.increment(errorCode);
        }

        std::sort(files.begin(), files.end());
        return files;
    }

    //Returns whether a file matches a setup pattern.
    bool GaussianSplatUtility::GaussianSplatFileUtility::FileMatchesPattern(const std::filesystem::path &filePath, const std::string &filePattern)
    {
        const std::string pattern = StringUtility::Uppercase(StringUtility::Trim(filePattern));
        const std::string fileName = StringUtility::Uppercase(filePath.filename().string());
        const std::string extension = StringUtility::Uppercase(filePath.extension().string());

        if (pattern.empty() || pattern == "*.PLY")
        {
            return extension == ".PLY";
        }

        if (pattern == "*")
        {
            return true;
        }

        if (pattern.size() > 2U && pattern[0] == '*' && pattern[1] == '.')
        {
            return extension == pattern.substr(1U);
        }

        if (pattern.find('*') == std::string::npos)
        {
            return fileName == pattern;
        }

        return extension == ".PLY";
    }

    //Returns whether setup requests only one PLY file.
    bool GaussianSplatUtility::GaussianSplatFileUtility::LoadModeIsFirst(const std::string &loadMode)
    {
        const std::string mode = StringUtility::Uppercase(StringUtility::Trim(loadMode));
        return mode != "ALL" && mode != "EVERYTHING" && mode != "MULTIPLE";
    }

    //Reads one PLY file.
    GaussianSplatReadResult GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ReadPlyFile(const std::filesystem::path &filePath, const GaussianSplatSetup &setup, std::size_t maxSplats)
    {
        GaussianSplatReadResult result;
        if (maxSplats == 0U)
        {
            return result;
        }

        std::ifstream file(filePath, std::ios::binary);
        if (!file.is_open())
        {
            result.m_errorMessage = "Gaussian splat PLY could not be opened: " + filePath.string();
            return result;
        }

        PlyHeader header;
        if (!ReadHeader(file, header, result.m_errorMessage))
        {
            result.m_errorMessage = result.m_errorMessage + " File: " + filePath.string();
            return result;
        }

        if (header.m_binaryLittleEndian && TryReadBinaryBlock(filePath, file, header, setup, maxSplats, result))
        {
            return result;
        }

        std::unordered_map<std::string, std::size_t> propertyIndices;
        for (std::size_t propertyIndex = 0U; propertyIndex < header.m_vertexProperties.size(); ++propertyIndex)
        {
            propertyIndices[StringUtility::Uppercase(header.m_vertexProperties[propertyIndex].m_name)] = propertyIndex;
        }

        const std::size_t splatStep = static_cast<std::size_t>(std::max(1, setup.m_displayEveryNthSplat));
        std::vector<double> values(header.m_vertexProperties.size(), 0.0);
        result.m_splats.reserve(std::min(maxSplats, header.m_vertexCount / splatStep + 1U));

        for (std::size_t vertexIndex = 0U; vertexIndex < header.m_vertexCount && result.m_splats.size() < maxSplats; ++vertexIndex)
        {
            const bool readOk = header.m_ascii ? ReadAsciiVertex(file, header, values) : [&]() -> bool
            {
                for (std::size_t propertyIndex = 0U; propertyIndex < header.m_vertexProperties.size(); ++propertyIndex)
                {
                    if (!ReadBinaryScalar(file, header.m_vertexProperties[propertyIndex].m_type, values[propertyIndex]))
                    {
                        return false;
                    }
                }

                return true;
            }();

            if (!readOk)
            {
                result.m_errorMessage = "Gaussian splat PLY ended before all vertices were read.";
                break;
            }

            if (vertexIndex % splatStep != 0U)
            {
                continue;
            }

            GaussianSplatData splat;
            if (CreateSplat(propertyIndices, values, setup, splat))
            {
                result.m_splats.push_back(std::move(splat));
            }
        }

        if (!result.m_splats.empty())
        {
            result.m_filesRead = 1U;
            result.m_fileNames.push_back(filePath.string());
        }

        return result;
    }

    //Reads a PLY header.
    bool GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ReadHeader(std::ifstream &file, PlyHeader &header, std::string &errorMessage)
    {

        //
		//Typical Gaussian splat PLY header:
        //
        //      ply
        //      format binary_little_endian 1.0
        //      element vertex 119553
        //      property float x
        //      property float y
        //      property float z
        //      property float nx
        //      property float ny
        //      property float nz
        //      property float f_dc_0
        //      property float f_dc_1
        //      property float f_dc_2
        //      property float f_rest_0
        //      property float f_rest_1
        //      property float f_rest_2
        //      property float f_rest_3
        //      property float f_rest_4
        //      property float f_rest_5
        //      property float f_rest_6
        //      property float f_rest_7
        //      property float f_rest_8
        //      property float f_rest_9
        //      property float f_rest_10
        //      property float f_rest_11
        //      property float f_rest_12
        //      property float f_rest_13
        //      property float f_rest_14
        //      property float f_rest_15
        //      property float f_rest_16
        //      property float f_rest_17
        //      property float f_rest_18
        //      property float f_rest_19
        //      property float f_rest_20
        //      property float f_rest_21
        //      property float f_rest_22
        //      property float f_rest_23
        //      property float f_rest_24
        //      property float f_rest_25
        //      property float f_rest_26
        //      property float f_rest_27
        //      property float f_rest_28
        //      property float f_rest_29
        //      property float f_rest_30
        //      property float f_rest_31
        //      property float f_rest_32
        //      property float f_rest_33
        //      property float f_rest_34
        //      property float f_rest_35
        //      property float f_rest_36
        //      property float f_rest_37
        //      property float f_rest_38
        //      property float f_rest_39
        //      property float f_rest_40
        //      property float f_rest_41
        //      property float f_rest_42
        //      property float f_rest_43
        //      property float f_rest_44
        //      property float opacity
        //      property float scale_0
        //      property float scale_1
        //      property float scale_2
        //      property float rot_0
        //      property float rot_1
        //      property float rot_2
        //      property float rot_3
        //      end_header
        //

        std::string line;
        if (!std::getline(file, line) || StringUtility::Trim(line) != "ply")
        {
            errorMessage = "Gaussian splat file is not a PLY file.";
            return false;
        }

        std::string currentElement;
        bool foundEndHeader = false;
        while (std::getline(file, line))
        {
            const std::string trimmedLine = StringUtility::Trim(line);
            if (trimmedLine == "end_header")
            {
                foundEndHeader = true;
                break;
            }

            std::istringstream stream(trimmedLine);
            std::string token;
            stream >> token;

            if (token == "format")
            {
                std::string formatName;
                stream >> formatName;
                header.m_ascii = formatName == "ascii";
                header.m_binaryLittleEndian = formatName == "binary_little_endian";
                if (!header.m_ascii && !header.m_binaryLittleEndian)
                {
                    errorMessage = "Only ASCII and binary_little_endian Gaussian splat PLY files are supported.";
                    return false;
                }
            }
            else if (token == "element")
            {
                currentElement.clear();
                std::size_t elementCount = 0U;
                stream >> currentElement >> elementCount;
                if (currentElement == "vertex")
                {
                    header.m_vertexCount = elementCount;
                }
            }
            else if (token == "property" && currentElement == "vertex")
            {
                std::string typeName;
                std::string propertyName;
                stream >> typeName;
                if (typeName == "list")
                {
                    errorMessage = "List properties are not supported in Gaussian splat vertices.";
                    return false;
                }

                stream >> propertyName;
                const PlyScalarType propertyType = ParseScalarType(typeName);
                if (propertyType == PlyScalarType::Unknown || propertyName.empty())
                {
                    errorMessage = "Unsupported Gaussian splat PLY vertex property.";
                    return false;
                }

                const std::size_t propertySize = ScalarTypeSize(propertyType);
                if (propertySize == 0U)
                {
                    errorMessage = "Unsupported Gaussian splat PLY vertex property.";
                    return false;
                }

                header.m_vertexProperties.push_back(PlyProperty{ propertyName, propertyType, header.m_vertexStrideBytes, propertySize });
                header.m_vertexStrideBytes += propertySize;
            }
        }

        if (!foundEndHeader)
        {
            errorMessage = "Gaussian splat PLY header is missing end_header.";
            return false;
        }

        if (header.m_vertexCount == 0U || header.m_vertexProperties.empty())
        {
            errorMessage = "Gaussian splat PLY has no vertex data.";
            return false;
        }

        if (!header.m_ascii && !header.m_binaryLittleEndian)
        {
            errorMessage = "Gaussian splat PLY has no supported format.";
            return false;
        }

        return true;
    }

    //Parses a PLY scalar property type.
    GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::PlyScalarType GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ParseScalarType(const std::string &text)
    {
        const std::string typeName = StringUtility::Uppercase(text);
        if (typeName == "CHAR" || typeName == "INT8")
        {
            return PlyScalarType::Int8;
        }

        if (typeName == "UCHAR" || typeName == "UINT8")
        {
            return PlyScalarType::UInt8;
        }

        if (typeName == "SHORT" || typeName == "INT16")
        {
            return PlyScalarType::Int16;
        }

        if (typeName == "USHORT" || typeName == "UINT16")
        {
            return PlyScalarType::UInt16;
        }

        if (typeName == "INT" || typeName == "INT32")
        {
            return PlyScalarType::Int32;
        }

        if (typeName == "UINT" || typeName == "UINT32")
        {
            return PlyScalarType::UInt32;
        }

        if (typeName == "FLOAT" || typeName == "FLOAT32")
        {
            return PlyScalarType::Float32;
        }

        if (typeName == "DOUBLE" || typeName == "FLOAT64")
        {
            return PlyScalarType::Float64;
        }

        return PlyScalarType::Unknown;
    }

    //Parses a double value.
    bool GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ParseDouble(const std::string &text, double &result)
    {
        try
        {
            std::size_t used = 0U;
            const double parsedValue = std::stod(text, &used);
            if (used != text.size())
            {
                return false;
            }

            result = parsedValue;
            return true;
        }
        catch (const std::exception &)
        {
            return false;
        }
    }

    //Reads one ASCII vertex.
    bool GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ReadAsciiVertex(std::ifstream &file, const PlyHeader &header, std::vector<double> &values)
    {
        std::string line;
        if (!std::getline(file, line))
        {
            return false;
        }

        std::istringstream stream(line);
        for (std::size_t propertyIndex = 0U; propertyIndex < header.m_vertexProperties.size(); ++propertyIndex)
        {
            std::string token;
            if (!(stream >> token) || !ParseDouble(token, values[propertyIndex]))
            {
                return false;
            }
        }

        return true;
    }

    //Reads one binary scalar value.
    bool GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ReadBinaryScalar(std::ifstream &file, PlyScalarType type, double &value)
    {
        switch (type)
        {
            case PlyScalarType::Int8:
                return ReadBinaryValue<int8_t>(file, value);
            case PlyScalarType::UInt8:
                return ReadBinaryValue<uint8_t>(file, value);
            case PlyScalarType::Int16:
                return ReadBinaryValue<int16_t>(file, value);
            case PlyScalarType::UInt16:
                return ReadBinaryValue<uint16_t>(file, value);
            case PlyScalarType::Int32:
                return ReadBinaryValue<int32_t>(file, value);
            case PlyScalarType::UInt32:
                return ReadBinaryValue<uint32_t>(file, value);
            case PlyScalarType::Float32:
                return ReadBinaryValue<float>(file, value);
            case PlyScalarType::Float64:
                return ReadBinaryValue<double>(file, value);
            default:
                return false;
        }
    }

    //Reads one binary typed value.
    template<typename Type>
    bool GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ReadBinaryValue(std::ifstream &file, double &value)
    {
        Type typedValue{};
        file.read(reinterpret_cast<char *>(&typedValue), static_cast<std::streamsize>(sizeof(Type)));
        if (!file.good())
        {
            return false;
        }

        value = static_cast<double>(typedValue);
        return true;
    }

    //Reads binary vertex data with one contiguous file read.
    bool GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::TryReadBinaryBlock(
        const std::filesystem::path &filePath,
        std::ifstream &file,
        const PlyHeader &header,
        const GaussianSplatSetup &setup,
        std::size_t maxSplats,
        GaussianSplatReadResult &result
    )
    {
        if (header.m_vertexStrideBytes == 0U || header.m_vertexProperties.empty())
        {
            return false;
        }

        const PlyFastLayout layout = CreateFastLayout(header);
        if (!PropertyPresent(layout.m_x) || !PropertyPresent(layout.m_y) || !PropertyPresent(layout.m_z))
        {
            return false;
        }

        if (header.m_vertexCount > std::numeric_limits<std::size_t>::max() / header.m_vertexStrideBytes)
        {
            result.m_errorMessage = "Gaussian splat PLY binary vertex data is too large: " + filePath.string();
            return true;
        }

        const std::size_t blockSize = header.m_vertexCount * header.m_vertexStrideBytes;
        const std::uintmax_t blockSizeWide = static_cast<std::uintmax_t>(blockSize);
        const std::uintmax_t maxReadSize = static_cast<std::uintmax_t>(std::numeric_limits<std::streamsize>::max());
        if (blockSizeWide > maxReadSize)
        {
            result.m_errorMessage = "Gaussian splat PLY binary vertex data is too large for one read: " + filePath.string();
            return true;
        }

        std::vector<unsigned char> vertexBytes(blockSize);
        file.read(reinterpret_cast<char *>(vertexBytes.data()), static_cast<std::streamsize>(blockSize));
        if (!file.good())
        {
            result.m_errorMessage = "Gaussian splat PLY ended before all binary vertices were read. File: " + filePath.string();
            return true;
        }

        const std::size_t splatStep = static_cast<std::size_t>(std::max(1, setup.m_displayEveryNthSplat));
        result.m_splats.reserve(std::min(maxSplats, header.m_vertexCount / splatStep + 1U));

        for (std::size_t vertexIndex = 0U; vertexIndex < header.m_vertexCount && result.m_splats.size() < maxSplats; ++vertexIndex)
        {
            if (vertexIndex % splatStep != 0U)
            {
                continue;
            }

            const unsigned char *vertexPtr = vertexBytes.data() + (vertexIndex * header.m_vertexStrideBytes);
            GaussianSplatData splat;
            if (CreateSplatFromBinaryVertex(layout, header.m_vertexProperties, vertexPtr, setup, splat))
            {
                result.m_splats.push_back(std::move(splat));
            }
        }

        if (!result.m_splats.empty())
        {
            result.m_filesRead = 1U;
            result.m_fileNames.push_back(filePath.string());
        }

        return true;
    }

    //Creates a direct binary property lookup layout.
    GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::PlyFastLayout GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::CreateFastLayout(const PlyHeader &header)
    {
        PlyFastLayout layout;
        for (std::size_t propertyIndex = 0U; propertyIndex < header.m_vertexProperties.size(); ++propertyIndex)
        {
            const std::string propertyName = StringUtility::Uppercase(header.m_vertexProperties[propertyIndex].m_name);
            if (propertyName == "X")
            {
                layout.m_x = propertyIndex;
            }
            else if (propertyName == "Y")
            {
                layout.m_y = propertyIndex;
            }
            else if (propertyName == "Z")
            {
                layout.m_z = propertyIndex;
            }
            else if (propertyName == "NX")
            {
                layout.m_normalX = propertyIndex;
            }
            else if (propertyName == "NY")
            {
                layout.m_normalY = propertyIndex;
            }
            else if (propertyName == "NZ")
            {
                layout.m_normalZ = propertyIndex;
            }
            else if (propertyName == "RED")
            {
                layout.m_red = propertyIndex;
            }
            else if (propertyName == "GREEN")
            {
                layout.m_green = propertyIndex;
            }
            else if (propertyName == "BLUE")
            {
                layout.m_blue = propertyIndex;
            }
            else if (propertyName == "F_DC_0")
            {
                layout.m_fdc0 = propertyIndex;
            }
            else if (propertyName == "F_DC_1")
            {
                layout.m_fdc1 = propertyIndex;
            }
            else if (propertyName == "F_DC_2")
            {
                layout.m_fdc2 = propertyIndex;
            }
            else if (propertyName == "ALPHA")
            {
                layout.m_alpha = propertyIndex;
            }
            else if (propertyName == "OPACITY")
            {
                layout.m_opacity = propertyIndex;
            }
            else if (propertyName == "SCALE_0")
            {
                layout.m_scale0 = propertyIndex;
            }
            else if (propertyName == "SCALE_1")
            {
                layout.m_scale1 = propertyIndex;
            }
            else if (propertyName == "SCALE_2")
            {
                layout.m_scale2 = propertyIndex;
            }
            else if (propertyName == "ROT_0")
            {
                layout.m_rotation0 = propertyIndex;
            }
            else if (propertyName == "ROT_1")
            {
                layout.m_rotation1 = propertyIndex;
            }
            else if (propertyName == "ROT_2")
            {
                layout.m_rotation2 = propertyIndex;
            }
            else if (propertyName == "ROT_3")
            {
                layout.m_rotation3 = propertyIndex;
            }
            else
            {
                AddSphericalHarmonicIndex(layout, propertyName, propertyIndex);
            }
        }

        layout.m_usesFdcColor = PropertyPresent(layout.m_fdc0) && PropertyPresent(layout.m_fdc1) && PropertyPresent(layout.m_fdc2);
        layout.m_usesRgbColor = PropertyPresent(layout.m_red) && PropertyPresent(layout.m_green) && PropertyPresent(layout.m_blue);
        layout.m_usesOpacity = PropertyPresent(layout.m_opacity);
        layout.m_usesAlpha = PropertyPresent(layout.m_alpha);
        layout.m_hasNormal = PropertyPresent(layout.m_normalX) && PropertyPresent(layout.m_normalY) && PropertyPresent(layout.m_normalZ);
        layout.m_hasScale = PropertyPresent(layout.m_scale0) && PropertyPresent(layout.m_scale1) && PropertyPresent(layout.m_scale2);
        layout.m_hasRotation = PropertyPresent(layout.m_rotation0) && PropertyPresent(layout.m_rotation1) && PropertyPresent(layout.m_rotation2) && PropertyPresent(layout.m_rotation3);
        return layout;
    }

    //Adds a spherical harmonic property index to the direct layout.
    void GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::AddSphericalHarmonicIndex(PlyFastLayout &layout, const std::string &propertyName, std::size_t propertyIndex)
    {
        std::size_t coefficientIndex = 0U;
        if (!ParsePropertySuffixIndex(propertyName, "F_REST_", coefficientIndex))
        {
            return;
        }

        if (layout.m_sphericalHarmonicIndices.size() <= coefficientIndex)
        {
            layout.m_sphericalHarmonicIndices.resize(coefficientIndex + 1U, s_invalidPropertyIndex);
        }

        layout.m_sphericalHarmonicIndices[coefficientIndex] = propertyIndex;
    }

    //Parses a property name suffix index.
    bool GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ParsePropertySuffixIndex(const std::string &propertyName, const std::string &prefix, std::size_t &suffixIndex)
    {
        if (propertyName.rfind(prefix, 0U) != 0U || propertyName.size() <= prefix.size())
        {
            return false;
        }

        try
        {
            std::size_t used = 0U;
            const std::string suffixText = propertyName.substr(prefix.size());
            const unsigned long parsedValue = std::stoul(suffixText, &used);
            if (used != suffixText.size())
            {
                return false;
            }

            suffixIndex = static_cast<std::size_t>(parsedValue);
            return true;
        }
        catch (const std::exception &)
        {
            return false;
        }
    }

    //Returns whether a direct property index is valid.
    bool GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::PropertyPresent(std::size_t propertyIndex)
    {
        return propertyIndex != s_invalidPropertyIndex;
    }

    //Reads one scalar value from a binary vertex block.
    bool GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ReadScalarValue(
        const std::vector<PlyProperty> &properties,
        std::size_t propertyIndex,
        const unsigned char *vertexPtr,
        double &value
    )
    {
        if (!PropertyPresent(propertyIndex) || propertyIndex >= properties.size() || vertexPtr == nullptr)
        {
            return false;
        }

        const PlyProperty &property = properties[propertyIndex];
        const unsigned char *dataPtr = vertexPtr + property.m_offsetBytes;
        switch (property.m_type)
        {
            case PlyScalarType::Int8:
                value = ScalarValueFromBytes<int8_t>(dataPtr);
                return true;
            case PlyScalarType::UInt8:
                value = ScalarValueFromBytes<uint8_t>(dataPtr);
                return true;
            case PlyScalarType::Int16:
                value = ScalarValueFromBytes<int16_t>(dataPtr);
                return true;
            case PlyScalarType::UInt16:
                value = ScalarValueFromBytes<uint16_t>(dataPtr);
                return true;
            case PlyScalarType::Int32:
                value = ScalarValueFromBytes<int32_t>(dataPtr);
                return true;
            case PlyScalarType::UInt32:
                value = ScalarValueFromBytes<uint32_t>(dataPtr);
                return true;
            case PlyScalarType::Float32:
                value = ScalarValueFromBytes<float>(dataPtr);
                return true;
            case PlyScalarType::Float64:
                value = ScalarValueFromBytes<double>(dataPtr);
                return true;
            default:
                return false;
        }
    }

    //Reads one typed scalar value from a binary vertex block.
    template<typename Type>
    double GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ScalarValueFromBytes(const unsigned char *dataPtr)
    {
        Type typedValue{};
        std::memcpy(&typedValue, dataPtr, sizeof(Type));
        return static_cast<double>(typedValue);
    }

    //Creates one Gaussian splat from binary vertex bytes.
    bool GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::CreateSplatFromBinaryVertex(
        const PlyFastLayout &layout,
        const std::vector<PlyProperty> &properties,
        const unsigned char *vertexPtr,
        const GaussianSplatSetup &setup,
        GaussianSplatData &splat
    )
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!ReadScalarValue(properties, layout.m_x, vertexPtr, x) || !ReadScalarValue(properties, layout.m_y, vertexPtr, y) || !ReadScalarValue(properties, layout.m_z, vertexPtr, z))
        {
            return false;
        }

        splat.m_position = ConvertPosition({ static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) }, setup);

        double red = 1.0;
        double green = 1.0;
        double blue = 1.0;
        if (layout.m_usesFdcColor)
        {
            double fdc0 = 0.0;
            double fdc1 = 0.0;
            double fdc2 = 0.0;
            if (
                ReadScalarValue(properties, layout.m_fdc0, vertexPtr, fdc0) &&
                ReadScalarValue(properties, layout.m_fdc1, vertexPtr, fdc1) &&
                ReadScalarValue(properties, layout.m_fdc2, vertexPtr, fdc2)
            )
            {
                constexpr double shCoefficient0 = 0.28209479177387814;
                red = 0.5 + (shCoefficient0 * fdc0);
                green = 0.5 + (shCoefficient0 * fdc1);
                blue = 0.5 + (shCoefficient0 * fdc2);
            }
        }
        else if (layout.m_usesRgbColor &&
            ReadScalarValue(properties, layout.m_red, vertexPtr, red) &&
            ReadScalarValue(properties, layout.m_green, vertexPtr, green) &&
            ReadScalarValue(properties, layout.m_blue, vertexPtr, blue)
        )
        {
            red /= 255.0;
            green /= 255.0;
            blue /= 255.0;
        }

        double opacityValue = 1.0;
        float alpha = 1.0f;
        if (layout.m_usesOpacity && ReadScalarValue(properties, layout.m_opacity, vertexPtr, opacityValue))
        {
            //Returns a sigmoid value.Raw value is logit (-inf to +inf) to (0, 1). Optimizer needs unconstrained value
            alpha = Sigmoid(opacityValue);
        }
        else if (layout.m_usesAlpha && ReadScalarValue(properties, layout.m_alpha, vertexPtr, opacityValue))
        {
            alpha = opacityValue > 1.0 ? ClampFloat(opacityValue / 255.0, 0.0f, 1.0f) : ClampFloat(opacityValue, 0.0f, 1.0f);
        }

        splat.m_color = {
            ClampFloat(red, 0.0f, 1.0f), ClampFloat(green, 0.0f, 1.0f), ClampFloat(blue, 0.0f, 1.0f), ClampFloat(static_cast<double>(alpha) * setup.m_opacityScale, 0.0f, 1.0f)
        };

        if (layout.m_hasScale)
        {
            double scale0 = 0.0;
            double scale1 = 0.0;
            double scale2 = 0.0;
            if (ReadScalarValue(properties, layout.m_scale0, vertexPtr, scale0) && ReadScalarValue(properties, layout.m_scale1, vertexPtr, scale1) &&
                ReadScalarValue(properties, layout.m_scale2, vertexPtr, scale2))
            {
                splat.m_scale = {
                    ScaleFromLog(scale0, setup.m_splatScale), ScaleFromLog(scale1, setup.m_splatScale), ScaleFromLog(scale2, setup.m_splatScale)
                };
            }
        }
        else
        {
            splat.m_scale = { setup.m_splatScale, setup.m_splatScale, setup.m_splatScale };
        }

        if (layout.m_hasRotation)
        {
            double rotation0 = 1.0;
            double rotation1 = 0.0;
            double rotation2 = 0.0;
            double rotation3 = 0.0;
            if (ReadScalarValue(properties, layout.m_rotation0, vertexPtr, rotation0) && ReadScalarValue(properties, layout.m_rotation1, vertexPtr, rotation1) &&
                ReadScalarValue(properties, layout.m_rotation2, vertexPtr, rotation2) && ReadScalarValue(properties, layout.m_rotation3, vertexPtr, rotation3))
            {
                splat.m_rotation = NormalizeQuaternion({
                    static_cast<float>(rotation1), static_cast<float>(rotation2), static_cast<float>(rotation3), static_cast<float>(rotation0)
                });
            }
        }
        splat.m_rotation = ConvertRotation(splat.m_rotation, setup);
        StoreOptionalDataFromBinaryVertex(layout, properties, vertexPtr, setup, splat);

        return splat.m_color.w > 0.001f;
    }

    //Returns the byte size for a scalar PLY type.
    std::size_t GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ScalarTypeSize(PlyScalarType type)
    {
        switch (type)
        {
            case PlyScalarType::Int8:
                return sizeof(int8_t);
            case PlyScalarType::UInt8:
                return sizeof(uint8_t);
            case PlyScalarType::Int16:
                return sizeof(int16_t);
            case PlyScalarType::UInt16:
                return sizeof(uint16_t);
            case PlyScalarType::Int32:
                return sizeof(int32_t);
            case PlyScalarType::UInt32:
                return sizeof(uint32_t);
            case PlyScalarType::Float32:
                return sizeof(float);
            case PlyScalarType::Float64:
                return sizeof(double);
            default:
                return 0U;
        }
    }

    //Stores optional Gaussian data from binary vertex bytes.
    void GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::StoreOptionalDataFromBinaryVertex(
        const PlyFastLayout &layout,
        const std::vector<PlyProperty> &properties,
        const unsigned char *vertexPtr,
        const GaussianSplatSetup &setup,
        GaussianSplatData &splat
    )
    {
        if (setup.m_storeNormals && layout.m_hasNormal)
        {
            double normalX = 0.0;
            double normalY = 0.0;
            double normalZ = 0.0;
            if (ReadScalarValue(properties, layout.m_normalX, vertexPtr, normalX) && ReadScalarValue(properties, layout.m_normalY, vertexPtr, normalY) &&
                ReadScalarValue(properties, layout.m_normalZ, vertexPtr, normalZ))
            {
                EnsureOptionalData(splat).m_normal = ConvertNormal({ static_cast<float>(normalX), static_cast<float>(normalY), static_cast<float>(normalZ) }, setup);
            }
        }

        if (ShouldStoreSphericalHarmonics(setup) && !layout.m_sphericalHarmonicIndices.empty())
        {
            GaussianSplatOptionalData &optionalData = EnsureOptionalData(splat);
            optionalData.m_sphericalHarmonics.assign(layout.m_sphericalHarmonicIndices.size(), 0.0f);
            for (std::size_t coefficientIndex = 0U; coefficientIndex < layout.m_sphericalHarmonicIndices.size(); ++coefficientIndex)
            {
                double coefficient = 0.0;
                const std::size_t propertyIndex = layout.m_sphericalHarmonicIndices[coefficientIndex];
                if (ReadScalarValue(properties, propertyIndex, vertexPtr, coefficient))
                {
                    optionalData.m_sphericalHarmonics[coefficientIndex] = static_cast<float>(coefficient);
                }
            }
        }
    }

    //Stores optional Gaussian data from named property values.
    void GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::StoreOptionalData(
        const std::unordered_map<std::string, std::size_t> &propertyIndices,
        const std::vector<double> &values,
        const GaussianSplatSetup &setup,
        GaussianSplatData &splat
    )
    {
        if (setup.m_storeNormals)
        {
            double normalX = 0.0;
            double normalY = 0.0;
            double normalZ = 0.0;
            if (PropertyValue(propertyIndices, values, "nx", normalX) && PropertyValue(propertyIndices, values, "ny", normalY) && PropertyValue(propertyIndices, values, "nz", normalZ))
            {
                EnsureOptionalData(splat).m_normal = ConvertNormal({ static_cast<float>(normalX), static_cast<float>(normalY), static_cast<float>(normalZ) }, setup);
            }
        }

        if (ShouldStoreSphericalHarmonics(setup))
        {
            std::vector<float> coefficients;
            for (const std::pair<const std::string, std::size_t> &propertyIndex : propertyIndices)
            {
                std::size_t coefficientIndex = 0U;
                if (propertyIndex.second >= values.size() || !ParsePropertySuffixIndex(propertyIndex.first, "F_REST_", coefficientIndex))
                {
                    continue;
                }

                if (coefficients.size() <= coefficientIndex)
                {
                    coefficients.resize(coefficientIndex + 1U, 0.0f);
                }

                coefficients[coefficientIndex] = static_cast<float>(values[propertyIndex.second]);
            }

            if (!coefficients.empty())
            {
                EnsureOptionalData(splat).m_sphericalHarmonics = coefficients;
            }
        }
    }

    //Returns writable optional Gaussian data.
    GaussianSplatOptionalData &GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::EnsureOptionalData(GaussianSplatData &splat)
    {
        if (!splat.m_optionalDataPtr)
        {
            splat.m_optionalDataPtr = std::make_unique<GaussianSplatOptionalData>();
        }

        return *splat.m_optionalDataPtr;
    }

    //Returns whether setup requests spherical harmonic storage.
    bool GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ShouldStoreSphericalHarmonics(const GaussianSplatSetup &setup)
    {
        const std::string colorMode = StringUtility::Uppercase(StringUtility::Trim(setup.m_colorMode));
        return setup.m_storeSphericalHarmonics || colorMode == "SPHERICALHARMONICS" || colorMode == "SPHERICAL_HARMONICS" || colorMode == "SH";
    }

    //Converts a PLY normal into application coordinates.
    Float3 GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ConvertNormal(const Float3 &normal, const GaussianSplatSetup &setup)
    {
        if (CoordinateModeIsColmapToApp(setup))
        {
            return { normal.x, -normal.y, -normal.z };
        }

        return normal;
    }

    //Returns one named property value.
    bool GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::PropertyValue(
        const std::unordered_map<std::string, std::size_t> &propertyIndices,
        const std::vector<double> &values,
        const std::string &name,
        double &result
    )
    {
        const auto found = propertyIndices.find(StringUtility::Uppercase(name));
        if (found == propertyIndices.end() || found->second >= values.size())
        {
            return false;
        }

        result = values[found->second];
        return true;
    }

    //Creates one Gaussian splat from PLY vertex properties.
    bool GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::CreateSplat(
        const std::unordered_map<std::string, std::size_t> &propertyIndices,
        const std::vector<double> &values,
        const GaussianSplatSetup &setup,
        GaussianSplatData &splat
    )
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!PropertyValue(propertyIndices, values, "x", x) || !PropertyValue(propertyIndices, values, "y", y) || !PropertyValue(propertyIndices, values, "z", z))
        {
            return false;
        }

        splat.m_position = ConvertPosition({ static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) }, setup);

        double red = 1.0;
        double green = 1.0;
        double blue = 1.0;
        double fdc0 = 0.0;
        double fdc1 = 0.0;
        double fdc2 = 0.0;
        if (PropertyValue(propertyIndices, values, "f_dc_0", fdc0) && PropertyValue(propertyIndices, values, "f_dc_1", fdc1) && PropertyValue(propertyIndices, values, "f_dc_2", fdc2))
        {
            constexpr double shCoefficient0 = 0.28209479177387814;
            red = 0.5 + (shCoefficient0 * fdc0);
            green = 0.5 + (shCoefficient0 * fdc1);
            blue = 0.5 + (shCoefficient0 * fdc2);
        }
        else if (PropertyValue(propertyIndices, values, "red", red) && PropertyValue(propertyIndices, values, "green", green) && PropertyValue(propertyIndices, values, "blue", blue))
        {
            red /= 255.0;
            green /= 255.0;
            blue /= 255.0;
        }

        double opacityValue = 1.0;
        float alpha = 1.0f;
        if (PropertyValue(propertyIndices, values, "opacity", opacityValue))
        {
            //Returns a sigmoid value.Raw value is logit (-inf to +inf) to (0, 1). Optimizer needs unconstrained value
            alpha = Sigmoid(opacityValue);
        }
        else if (PropertyValue(propertyIndices, values, "alpha", opacityValue))
        {
            alpha = opacityValue > 1.0 ? ClampFloat(opacityValue / 255.0, 0.0f, 1.0f) : ClampFloat(opacityValue, 0.0f, 1.0f);
        }

        splat.m_color = {
            ClampFloat(red, 0.0f, 1.0f), ClampFloat(green, 0.0f, 1.0f), ClampFloat(blue, 0.0f, 1.0f), ClampFloat(static_cast<double>(alpha) * setup.m_opacityScale, 0.0f, 1.0f)
        };

        double scale0 = 0.0;
        double scale1 = 0.0;
        double scale2 = 0.0;
        if (PropertyValue(propertyIndices, values, "scale_0", scale0) && PropertyValue(propertyIndices, values, "scale_1", scale1) && PropertyValue(propertyIndices, values, "scale_2", scale2))
        {
            splat.m_scale = {
                ScaleFromLog(scale0, setup.m_splatScale), ScaleFromLog(scale1, setup.m_splatScale), ScaleFromLog(scale2, setup.m_splatScale)
            };
        }
        else
        {
            splat.m_scale = { setup.m_splatScale, setup.m_splatScale, setup.m_splatScale };
        }

        double rotation0 = 1.0;
        double rotation1 = 0.0;
        double rotation2 = 0.0;
        double rotation3 = 0.0;
        if (PropertyValue(propertyIndices, values, "rot_0", rotation0) && PropertyValue(propertyIndices, values, "rot_1", rotation1) &&
            PropertyValue(propertyIndices, values, "rot_2", rotation2) && PropertyValue(propertyIndices, values, "rot_3", rotation3))
        {
            splat.m_rotation = NormalizeQuaternion({
                static_cast<float>(rotation1), static_cast<float>(rotation2), static_cast<float>(rotation3), static_cast<float>(rotation0)
            });
        }
        splat.m_rotation = ConvertRotation(splat.m_rotation, setup);
        StoreOptionalData(propertyIndices, values, setup, splat);

        return splat.m_color.w > 0.001f;
    }

    //Converts a PLY position into application coordinates.
    Float3 GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ConvertPosition(const Float3 &position, const GaussianSplatSetup &setup)
    {
        Float3 convertedPosition = position;
        if (CoordinateModeIsColmapToApp(setup))
        {
            convertedPosition = { position.x, -position.y, -position.z };
        }

        return convertedPosition * setup.m_positionScale;
    }

    //Converts a PLY rotation into application coordinates.
    Float4 GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ConvertRotation(const Float4 &rotation, const GaussianSplatSetup &setup)
    {
        if (!CoordinateModeIsColmapToApp(setup))
        {
            return NormalizeQuaternion(rotation);
        }

        return NormalizeQuaternion(MultiplyQuaternion({ 1.0f, 0.0f, 0.0f, 0.0f }, rotation));
    }

    //Returns whether setup requests COLMAP to app conversion.
    bool GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::CoordinateModeIsColmapToApp(const GaussianSplatSetup &setup)
    {
        const std::string mode = StringUtility::Uppercase(StringUtility::Trim(setup.m_coordinateMode));
        return mode == "COLMAPTOAPP" || mode == "COLMAP_TO_APP";
    }

    //Multiplies two quaternions.
    Float4 GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::MultiplyQuaternion(const Float4 &a, const Float4 &b)
    {
        return {
            (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y),
            (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x),
            (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w),
            (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z)
        };
    }

    //Returns a sigmoid value.Raw value is logit (-inf to +inf) to (0, 1). Optimizer needs unconstrained value
    float GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::Sigmoid(double value)
    {
        return ClampFloat(1.0 / (1.0 + std::exp(-value)), 0.0f, 1.0f);
    }

    //Returns a scale value from a log scale property.
    float GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ScaleFromLog(double value, float splatScale)
    {
        const double safeValue = std::clamp(value, -30.0, 30.0);
        return ClampFloat(std::exp(safeValue) * static_cast<double>(splatScale), 0.000001f, 1000000.0f);
    }

    //Returns a normalized quaternion.
    Float4 GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::NormalizeQuaternion(const Float4 &quaternion)
    {
        const float lengthSquared = (quaternion.x * quaternion.x) + (quaternion.y * quaternion.y) + (quaternion.z * quaternion.z) + (quaternion.w * quaternion.w);
        if (lengthSquared <= 0.000001f)
        {
            return { 0.0f, 0.0f, 0.0f, 1.0f };
        }

        const float inverseLength = 1.0f / std::sqrt(lengthSquared);
        return { quaternion.x * inverseLength, quaternion.y * inverseLength, quaternion.z * inverseLength, quaternion.w * inverseLength };
    }

    //Returns a clamped float value.
    float GaussianSplatUtility::GaussianSplatFileUtility::PLYFileUtility::ClampFloat(double value, float minimum, float maximum)
    {
        return static_cast<float>(std::clamp(value, static_cast<double>(minimum), static_cast<double>(maximum)));
    }

    //Loads Gaussian splat PLY data from a folder.
    GaussianSplatReadResult GaussianSplatUtility::LoadFolder(const GaussianSplatSetup &setup)
    {
        return GaussianSplatFileUtility::LoadFolder(setup);
    }

    //Returns files that match the splat setup.
    std::vector<std::filesystem::path> GaussianSplatUtility::FindFiles(const GaussianSplatSetup &setup, std::string &errorMessage)
    {
        return GaussianSplatFileUtility::FindFiles(setup, errorMessage);
    }

    //Loads Gaussian splat PLY data from one file.
    GaussianSplatReadResult GaussianSplatUtility::LoadFile(const std::filesystem::path &filePath, const GaussianSplatSetup &setup)
    {
        return GaussianSplatFileUtility::LoadFile(filePath, setup);
    }
}
