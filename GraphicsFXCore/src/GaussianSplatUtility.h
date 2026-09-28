///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	GaussianSplatUtility.h
//	Description	:
//		Defines Gaussian splat PLY loading helpers.
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "global.h"

#include "MathUtility.h"
#include "SetupUtility.h"

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace graphicsFX
{
    //Stores optional Gaussian splat data that is not needed for default rendering.
    struct GaussianSplatOptionalData
    {
        Float3 m_normal{ 0.0f, 0.0f, 0.0f }; //Stores the loaded splat normal.
        std::vector<float> m_sphericalHarmonics; //Stores loaded higher-order spherical harmonic coefficients.
    };

    //Stores one loaded Gaussian splat.
    struct GaussianSplatData
    {
        Float3 m_position{ 0.0f, 0.0f, 0.0f }; //Stores the splat position.
        Float4 m_color{ 1.0f, 1.0f, 1.0f, 1.0f }; //Stores the splat color and opacity.
        Float3 m_scale{ 1.0f, 1.0f, 1.0f }; //Stores the splat local axis scales.
        Float4 m_rotation{ 0.0f, 0.0f, 0.0f, 1.0f }; //Stores the splat local rotation quaternion.
        std::unique_ptr<GaussianSplatOptionalData> m_optionalDataPtr; //Stores optional normal and spherical harmonic data.

        //Creates a Gaussian splat.
        GaussianSplatData(void) = default;

        //Copies a Gaussian splat.
        GaussianSplatData(const GaussianSplatData &other);

        //Moves a Gaussian splat.
        GaussianSplatData(GaussianSplatData &&other) noexcept = default;

        //Copies a Gaussian splat.
        GaussianSplatData &operator=(const GaussianSplatData &other);

        //Moves a Gaussian splat.
        GaussianSplatData &operator=(GaussianSplatData &&other) noexcept = default;
    };

    //Stores loaded Gaussian splat PLY results.
    struct GaussianSplatReadResult
    {
        std::vector<GaussianSplatData> m_splats; //Stores loaded splat data.
        std::vector<std::string> m_fileNames; //Stores loaded source file names.
        std::string m_errorMessage; //Stores the last non-fatal error message.
        std::size_t m_filesRead{ 0U }; //Stores the number of files read.
    };

    //Defines Gaussian splat PLY loading helpers.
    class GaussianSplatUtility final
    {

        private:

            //Loads Gaussian splat PLY files.
            class GaussianSplatFileUtility final
            {

                public:

                    GaussianSplatFileUtility(void) = delete;

                    //Loads Gaussian splat PLY data from a folder.
                    static GaussianSplatReadResult LoadFolder(const GaussianSplatSetup &setup);

                    //Returns files that match the splat setup.
                    static std::vector<std::filesystem::path> FindFiles(const GaussianSplatSetup &setup, std::string &errorMessage);

                    //Loads Gaussian splat PLY data from one file.
                    static GaussianSplatReadResult LoadFile(const std::filesystem::path &filePath, const GaussianSplatSetup &setup);

                private:

                    //Handles one Gaussian splat PLY file.
                    class PLYFileUtility final
                    {

                        public:

                            PLYFileUtility(void) = delete;

                            //Reads one PLY file.
                            static GaussianSplatReadResult ReadPlyFile(const std::filesystem::path &filePath, const GaussianSplatSetup &setup, std::size_t maxSplats);

                        private:

                            //Stores supported PLY scalar property types.
                            enum class PlyScalarType
                            {
                                Int8, UInt8, Int16, UInt16, Int32, UInt32, Float32, Float64, Unknown
                            };

                            //Stores one PLY vertex property.
                            struct PlyProperty
                            {
                                std::string m_name; //Stores the property name.
                                PlyScalarType m_type{ PlyScalarType::Unknown }; //Stores the scalar property type.
                                std::size_t m_offsetBytes{ 0U }; //Stores the byte offset inside one binary vertex.
                                std::size_t m_sizeBytes{ 0U }; //Stores the scalar byte size.
                            };

                            //Stores parsed PLY header values.
                            struct PlyHeader
                            {
                                bool m_binaryLittleEndian{ false }; //Stores whether vertex data is binary little-endian.
                                bool m_ascii{ false }; //Stores whether vertex data is ASCII.
                                std::size_t m_vertexCount{ 0U }; //Stores the number of vertices.
                                std::size_t m_vertexStrideBytes{ 0U }; //Stores one binary vertex byte stride.
                                std::vector<PlyProperty> m_vertexProperties; //Stores vertex scalar properties.
                            };

                            static constexpr std::size_t s_invalidPropertyIndex{ static_cast<std::size_t>(-1) }; //Stores an invalid property index.

                            //Stores direct property indices for binary fast loading.
                            struct PlyFastLayout
                            {
                                std::size_t m_x{ s_invalidPropertyIndex }; //Stores the x property index.
                                std::size_t m_y{ s_invalidPropertyIndex }; //Stores the y property index.
                                std::size_t m_z{ s_invalidPropertyIndex }; //Stores the z property index.
                                std::size_t m_normalX{ s_invalidPropertyIndex }; //Stores the nx property index.
                                std::size_t m_normalY{ s_invalidPropertyIndex }; //Stores the ny property index.
                                std::size_t m_normalZ{ s_invalidPropertyIndex }; //Stores the nz property index.
                                std::size_t m_red{ s_invalidPropertyIndex }; //Stores the red property index.
                                std::size_t m_green{ s_invalidPropertyIndex }; //Stores the green property index.
                                std::size_t m_blue{ s_invalidPropertyIndex }; //Stores the blue property index.
                                std::size_t m_fdc0{ s_invalidPropertyIndex }; //Stores the first f_dc property index.
                                std::size_t m_fdc1{ s_invalidPropertyIndex }; //Stores the second f_dc property index.
                                std::size_t m_fdc2{ s_invalidPropertyIndex }; //Stores the third f_dc property index.
                                std::size_t m_alpha{ s_invalidPropertyIndex }; //Stores the alpha property index.
                                std::size_t m_opacity{ s_invalidPropertyIndex }; //Stores the opacity property index.
                                std::size_t m_scale0{ s_invalidPropertyIndex }; //Stores the first scale property index.
                                std::size_t m_scale1{ s_invalidPropertyIndex }; //Stores the second scale property index.
                                std::size_t m_scale2{ s_invalidPropertyIndex }; //Stores the third scale property index.
                                std::size_t m_rotation0{ s_invalidPropertyIndex }; //Stores the first rotation property index.
                                std::size_t m_rotation1{ s_invalidPropertyIndex }; //Stores the second rotation property index.
                                std::size_t m_rotation2{ s_invalidPropertyIndex }; //Stores the third rotation property index.
                                std::size_t m_rotation3{ s_invalidPropertyIndex }; //Stores the fourth rotation property index.
                                std::vector<std::size_t> m_sphericalHarmonicIndices; //Stores f_rest property indices in coefficient order.
                                bool m_usesFdcColor{ false }; //Stores whether f_dc color is available.
                                bool m_usesRgbColor{ false }; //Stores whether RGB color is available.
                                bool m_usesOpacity{ false }; //Stores whether opacity is available.
                                bool m_usesAlpha{ false }; //Stores whether alpha is available.
                                bool m_hasNormal{ false }; //Stores whether full normal data is available.
                                bool m_hasScale{ false }; //Stores whether full scale is available.
                                bool m_hasRotation{ false }; //Stores whether full rotation is available.
                            };

                            //Reads a PLY header.
                            static bool ReadHeader(std::ifstream &file, PlyHeader &header, std::string &errorMessage);

                            //Parses a PLY scalar property type.
                            static PlyScalarType ParseScalarType(const std::string &text);

                            //Parses a double value.
                            static bool ParseDouble(const std::string &text, double &result);

                            //Reads one ASCII vertex.
                            static bool ReadAsciiVertex(std::ifstream &file, const PlyHeader &header, std::vector<double> &values);

                            //Reads one binary scalar value.
                            static bool ReadBinaryScalar(std::ifstream &file, PlyScalarType type, double &value);

                            //Reads one binary typed value.
                            template<typename Type>
                            static bool ReadBinaryValue(std::ifstream &file, double &value);

                            //Reads binary vertex data with one contiguous file read.
                            static bool TryReadBinaryBlock(
                                const std::filesystem::path &filePath,
                                std::ifstream &file,
                                const PlyHeader &header,
                                const GaussianSplatSetup &setup,
                                std::size_t maxSplats,
                                GaussianSplatReadResult &result
                            );

                            //Creates a direct binary property lookup layout.
                            static PlyFastLayout CreateFastLayout(const PlyHeader &header);

                            //Adds a spherical harmonic property index to the direct layout.
                            static void AddSphericalHarmonicIndex(PlyFastLayout &layout, const std::string &propertyName, std::size_t propertyIndex);

                            //Parses a property name suffix index.
                            static bool ParsePropertySuffixIndex(const std::string &propertyName, const std::string &prefix, std::size_t &suffixIndex);

                            //Returns whether a direct property index is valid.
                            static bool PropertyPresent(std::size_t propertyIndex);

                            //Reads one scalar value from a binary vertex block.
                            static bool ReadScalarValue(const std::vector<PlyProperty> &properties, std::size_t propertyIndex, const unsigned char *vertexPtr, double &value);

                            //Reads one typed scalar value from a binary vertex block.
                            template<typename Type>
                            static double ScalarValueFromBytes(const unsigned char *dataPtr);

                            //Creates one Gaussian splat from binary vertex bytes.
                            static bool CreateSplatFromBinaryVertex(
                                const PlyFastLayout &layout,
                                const std::vector<PlyProperty> &properties,
                                const unsigned char *vertexPtr,
                                const GaussianSplatSetup &setup,
                                GaussianSplatData &splat
                            );

                            //Returns the byte size for a scalar PLY type.
                            static std::size_t ScalarTypeSize(PlyScalarType type);

                            //Stores optional Gaussian data from binary vertex bytes.
                            static void StoreOptionalDataFromBinaryVertex(
                                const PlyFastLayout &layout,
                                const std::vector<PlyProperty> &properties,
                                const unsigned char *vertexPtr,
                                const GaussianSplatSetup &setup,
                                GaussianSplatData &splat
                            );

                            //Stores optional Gaussian data from named property values.
                            static void StoreOptionalData(
                                const std::unordered_map<std::string, std::size_t> &propertyIndices,
                                const std::vector<double> &values,
                                const GaussianSplatSetup &setup,
                                GaussianSplatData &splat
                            );

                            //Returns writable optional Gaussian data.
                            static GaussianSplatOptionalData &EnsureOptionalData(GaussianSplatData &splat);

                            //Returns whether setup requests spherical harmonic storage.
                            static bool ShouldStoreSphericalHarmonics(const GaussianSplatSetup &setup);

                            //Converts a PLY normal into application coordinates.
                            static Float3 ConvertNormal(const Float3 &normal, const GaussianSplatSetup &setup);

                            //Returns one named property value.
                            static bool PropertyValue(const std::unordered_map<std::string, std::size_t> &propertyIndices, const std::vector<double> &values, const std::string &name, double &result);

                            //Creates one Gaussian splat from PLY vertex properties.
                            static bool CreateSplat(
                                const std::unordered_map<std::string, std::size_t> &propertyIndices,
                                const std::vector<double> &values,
                                const GaussianSplatSetup &setup,
                                GaussianSplatData &splat
                            );

                            //Converts a PLY position into application coordinates.
                            static Float3 ConvertPosition(const Float3 &position, const GaussianSplatSetup &setup);

                            //Converts a PLY rotation into application coordinates.
                            static Float4 ConvertRotation(const Float4 &rotation, const GaussianSplatSetup &setup);

                            //Returns whether setup requests COLMAP to app conversion.
                            static bool CoordinateModeIsColmapToApp(const GaussianSplatSetup &setup);

                            //Multiplies two quaternions.
                            static Float4 MultiplyQuaternion(const Float4 &a, const Float4 &b);

                            //Returns a sigmoid value.Raw value is logit (-inf to +inf) to (0, 1). Optimizer needs unconstrained value
                            static float Sigmoid(double value);

                            //Returns a scale value from a log scale property.
                            static float ScaleFromLog(double value, float splatScale);

                            //Returns a normalized quaternion.
                            static Float4 NormalizeQuaternion(const Float4 &quaternion);

                            //Returns a clamped float value.
                            static float ClampFloat(double value, float minimum, float maximum);
                    };

                    //Returns whether a file matches a setup pattern.
                    static bool FileMatchesPattern(const std::filesystem::path &filePath, const std::string &filePattern);

                    //Returns whether setup requests only one PLY file.
                    static bool LoadModeIsFirst(const std::string &loadMode);
            };

        public:

            GaussianSplatUtility(void) = delete;

            //Loads Gaussian splat PLY data from a folder.
            static GaussianSplatReadResult LoadFolder(const GaussianSplatSetup &setup);

            //Returns files that match the splat setup.
            static std::vector<std::filesystem::path> FindFiles(const GaussianSplatSetup &setup, std::string &errorMessage);

            //Loads Gaussian splat PLY data from one file.
            static GaussianSplatReadResult LoadFile(const std::filesystem::path &filePath, const GaussianSplatSetup &setup);

    };
}
