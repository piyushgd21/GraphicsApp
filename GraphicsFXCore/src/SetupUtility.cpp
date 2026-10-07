///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	SetupUtility.cpp
//	Description	:
//		Implements program setup loading from setup.ini.
//
//
///////////////////////////////////////////////////////////////////////

#include "SetupUtility.h"

#include "global.h"

#include <Windows.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace graphicsFX
{
    //Loads setup files and applies values to setup structures.
    class SetupFileUtility final
    {

        public:

            SetupFileUtility(void) = delete;

            //Loads program setup values.
            static ProgramSetup Load(const std::string &fileName);

        private:

        using SetupMap = std::unordered_map<std::string, std::string>;

            //Returns a trimmed string.
            static std::string Trim(const std::string &text);

            //Returns an uppercase setup name.
            static std::string NormalizeName(const std::string &text);

            //Returns a line with comments removed.
            static std::string RemoveComment(const std::string &line);

            //Returns a section and key lookup name.
            static std::string SetupKey(const std::string &section, const std::string &key);

            //Returns the first existing setup path.
            static std::filesystem::path FindSetupPath(const std::string &fileName);

            //Reads setup key value pairs.
            static SetupMap ReadSetupValues(const std::string &fileName);

            //Parses an integer value.
            static bool ParseInt(const std::string &text, int &result);

            //Parses a float value.
            static bool ParseFloat(const std::string &text, float &result);

            //Parses a boolean value.
            static bool ParseBool(const std::string &text, bool &result);

            //Parses a three component float vector.
            static bool ParseFloat3(const std::string &text, Float3 &result);

            //Parses a four component float vector.
            static bool ParseFloat4(const std::string &text, Float4 &result);

            //Returns the Win32 key code for a setup name.
            static uint32_t KeyCodeFromName(const std::string &text, uint32_t fallback);

            //Returns the mouse button code for a setup name.
            static uint32_t MouseButtonFromName(const std::string &text, uint32_t fallback);

            //Applies an integer setup value.
            static void ApplyInt(const SetupMap &values, const std::string &key, int &target, int minimum, int maximum);

            //Applies a float setup value.
            static void ApplyFloat(const SetupMap &values, const std::string &key, float &target, float minimum, float maximum);

            //Applies a boolean setup value.
            static void ApplyBool(const SetupMap &values, const std::string &key, bool &target);

            //Applies a three component float setup value.
            static void ApplyFloat3(const SetupMap &values, const std::string &key, Float3 &target);

            //Applies a string setup value.
            static void ApplyString(const SetupMap &values, const std::string &key, std::string &target);

            //Applies a four component float setup value.
            static void ApplyFloat4(const SetupMap &values, const std::string &key, Float4 &target, float minimum, float maximum);

            //Applies a keyboard setup value.
            static void ApplyKey(const SetupMap &values, const std::string &key, uint32_t &target);

            //Applies a mouse setup value.
            static void ApplyMouseButton(const SetupMap &values, const std::string &key, uint32_t &target);

            //Applies a plane sweep execution mode setup value.
            static void ApplyPlaneSweepExecutionMode(const SetupMap &values, const std::string &key, PlaneSweepExecutionMode &target);
    };

        //Returns a trimmed string.
        std::string SetupFileUtility::Trim(const std::string &text)
        {
            std::size_t first = 0;
            while (first < text.size() && std::isspace(static_cast<unsigned char>(text[first])) != 0)
            {
                ++first;
            }

            std::size_t last = text.size();
            while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1])) != 0)
            {
                --last;
            }

            return text.substr(first, last - first);
        }

        //Returns an uppercase setup name.
        std::string SetupFileUtility::NormalizeName(const std::string &text)
        {
            std::string result = Trim(text);
            std::transform(result.begin(), result.end(), result.begin(), [](unsigned char character) { return static_cast<char>(std::toupper(character)); });
            return result;
        }

        //Returns a line with comments removed.
        std::string SetupFileUtility::RemoveComment(const std::string &line)
        {
            const std::size_t semicolon = line.find(';');
            const std::size_t hash = line.find('#');
            std::size_t comment = std::string::npos;

            if (semicolon != std::string::npos)
            {
                comment = semicolon;
            }

            if (hash != std::string::npos)
            {
                comment = std::min(comment, hash);
            }

            if (comment == std::string::npos)
            {
                return line;
            }

            return line.substr(0, comment);
        }

        //Returns a section and key lookup name.
        std::string SetupFileUtility::SetupKey(const std::string &section, const std::string &key)
        {
            return NormalizeName(section) + "." + NormalizeName(key);
        }

        //Returns the first existing setup path.
        std::filesystem::path SetupFileUtility::FindSetupPath(const std::string &fileName)
        {
            std::error_code errorCode;
            const std::filesystem::path directPath(fileName);

            if (std::filesystem::exists(directPath, errorCode))
            {
                return directPath;
            }

            std::filesystem::path currentPath = std::filesystem::current_path(errorCode);
            for (int count = 0; count < 6 && !currentPath.empty(); ++count)
            {
                const std::filesystem::path candidatePath = currentPath / fileName;
                if (std::filesystem::exists(candidatePath, errorCode))
                {
                    return candidatePath;
                }

                if (!currentPath.has_parent_path())
                {
                    break;
                }

                const std::filesystem::path parentPath = currentPath.parent_path();
                if (parentPath == currentPath)
                {
                    break;
                }

                currentPath = parentPath;
            }

            return directPath;
        }

        //Reads setup key value pairs.
        SetupFileUtility::SetupMap SetupFileUtility::ReadSetupValues(const std::string &fileName)
        {
            SetupMap values;
            std::ifstream file(FindSetupPath(fileName));

            if (!file.is_open())
            {
                return values;
            }

            std::string section;
            std::string line;
            while (std::getline(file, line))
            {
                const std::string cleanLine = Trim(RemoveComment(line));
                if (cleanLine.empty())
                {
                    continue;
                }

                if (cleanLine.front() == '[' && cleanLine.back() == ']')
                {
                    section = cleanLine.substr(1, cleanLine.size() - 2);
                    continue;
                }

                const std::size_t equals = cleanLine.find('=');
                if (equals == std::string::npos)
                {
                    continue;
                }

                const std::string key = cleanLine.substr(0, equals);
                const std::string value = cleanLine.substr(equals + 1);
                values[SetupKey(section, key)] = Trim(value);
            }

            return values;
        }

        //Parses an integer value.
        bool SetupFileUtility::ParseInt(const std::string &text, int &result)
        {
            try
            {
                const std::string value = Trim(text);
                std::size_t used = 0;
                const int parsedValue = std::stoi(value, &used);
                if (used != value.size())
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

        //Parses a float value.
        bool SetupFileUtility::ParseFloat(const std::string &text, float &result)
        {
            try
            {
                const std::string value = Trim(text);
                std::size_t used = 0;
                const float parsedValue = std::stof(value, &used);
                if (used != value.size())
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

        //Parses a boolean value.
        bool SetupFileUtility::ParseBool(const std::string &text, bool &result)
        {
            const std::string value = NormalizeName(text);

            if (value == "TRUE" || value == "ON" || value == "YES" || value == "1")
            {
                result = true;
                return true;
            }

            if (value == "FALSE" || value == "OFF" || value == "NO" || value == "0")
            {
                result = false;
                return true;
            }

            return false;
        }

        //Parses a three component float vector.
        bool SetupFileUtility::ParseFloat3(const std::string &text, Float3 &result)
        {
            std::array<float, 3> values{};
            std::size_t start = 0;

            for (std::size_t index = 0; index < values.size(); ++index)
            {
                const std::size_t comma = text.find(',', start);
                const bool needsComma = index + 1 < values.size();

                if (needsComma && comma == std::string::npos)
                {
                    return false;
                }

                if (!needsComma && comma != std::string::npos)
                {
                    return false;
                }

                const std::size_t end = needsComma ? comma : text.size();
                if (!ParseFloat(text.substr(start, end - start), values[index]))
                {
                    return false;
                }

                start = end + 1;
            }

            result = { values[0], values[1], values[2] };
            return true;
        }

        //Parses a four component float vector.
        bool SetupFileUtility::ParseFloat4(const std::string &text, Float4 &result)
        {
            Float4 values{ 0.0f, 0.0f, 0.0f, 0.0f };
            float *components[] = {
                &values.x, &values.y, &values.z, &values.w
            };
            constexpr std::size_t componentCount = 4;
            std::size_t start = 0;

            for (std::size_t index = 0; index < componentCount; ++index)
            {
                const std::size_t comma = text.find(',', start);
                const bool needsComma = index + 1 < componentCount;

                if (needsComma && comma == std::string::npos)
                {
                    return false;
                }

                if (!needsComma && comma != std::string::npos)
                {
                    return false;
                }

                const std::size_t end = needsComma ? comma : text.size();
                if (!ParseFloat(text.substr(start, end - start), *components[index]))
                {
                    return false;
                }

                start = end + 1;
            }

            result = values;
            return true;
        }

        //Returns a key code from a setup name.
        uint32_t SetupFileUtility::KeyCodeFromName(const std::string &text, uint32_t fallback)
        {
            const std::string name = NormalizeName(text);

            if (name == "NONE")
            {
                return 0U;
            }

            if (name == "+" || name == "PLUS")
            {
                return VK_OEM_PLUS;
            }

            if (name == "-" || name == "MINUS")
            {
                return VK_OEM_MINUS;
            }

            if (name.size() == 1)
            {
                return static_cast<uint32_t>(static_cast<unsigned char>(name[0]));
            }

            if (name == "SHIFT")
            {
                return VK_SHIFT;
            }

            if (name == "CONTROL" || name == "CTRL")
            {
                return VK_CONTROL;
            }

            if (name == "ALT")
            {
                return VK_MENU;
            }

            if (name == "SPACE")
            {
                return VK_SPACE;
            }

            if (name == "TAB")
            {
                return VK_TAB;
            }

            if (name == "UP")
            {
                return VK_UP;
            }

            if (name == "DOWN")
            {
                return VK_DOWN;
            }

            if (name == "PAGEUP" || name == "PGUP")
            {
                return VK_PRIOR;
            }

            if (name == "PAGEDOWN" || name == "PGDN")
            {
                return VK_NEXT;
            }

            if (name == "LEFT")
            {
                return VK_LEFT;
            }

            if (name == "RIGHT")
            {
                return VK_RIGHT;
            }

            return fallback;
        }

        //Returns a mouse button code from a setup name.
        uint32_t SetupFileUtility::MouseButtonFromName(const std::string &text, uint32_t fallback)
        {
            const std::string name = NormalizeName(text);

            if (name == "NONE")
            {
                return std::numeric_limits<uint32_t>::max();
            }

            if (name == "LEFT")
            {
                return 0U;
            }

            if (name == "RIGHT")
            {
                return 1U;
            }

            if (name == "MIDDLE")
            {
                return 2U;
            }

            if (name == "X1")
            {
                return 3U;
            }

            if (name == "X2")
            {
                return 4U;
            }

            return fallback;
        }

        //Applies an integer setup value.
        void SetupFileUtility::ApplyInt(const SetupMap &values, const std::string &key, int &target, int minimum, int maximum)
        {
            const auto found = values.find(key);
            if (found == values.end())
            {
                return;
            }

            int parsedValue = target;
            if (ParseInt(found->second, parsedValue) && parsedValue >= minimum && parsedValue <= maximum)
            {
                target = parsedValue;
            }
        }

        //Applies a float setup value.
        void SetupFileUtility::ApplyFloat(const SetupMap &values, const std::string &key, float &target, float minimum, float maximum)
        {
            const auto found = values.find(key);
            if (found == values.end())
            {
                return;
            }

            float parsedValue = target;
            if (ParseFloat(found->second, parsedValue) && parsedValue >= minimum && parsedValue <= maximum)
            {
                target = parsedValue;
            }
        }

        //Applies a boolean setup value.
        void SetupFileUtility::ApplyBool(const SetupMap &values, const std::string &key, bool &target)
        {
            const auto found = values.find(key);
            if (found == values.end())
            {
                return;
            }

            bool parsedValue = target;
            if (ParseBool(found->second, parsedValue))
            {
                target = parsedValue;
            }
        }

        //Applies a vector setup value.
        void SetupFileUtility::ApplyFloat3(const SetupMap &values, const std::string &key, Float3 &target)
        {
            const auto found = values.find(key);
            if (found == values.end())
            {
                return;
            }

            Float3 parsedValue = target;
            if (ParseFloat3(found->second, parsedValue))
            {
                target = parsedValue;
            }
        }

        //Applies a string setup value.
        void SetupFileUtility::ApplyString(const SetupMap &values, const std::string &key, std::string &target)
        {
            const auto found = values.find(key);
            if (found == values.end())
            {
                return;
            }

            const std::string parsedValue = Trim(found->second);
            if (!parsedValue.empty())
            {
                target = parsedValue;
            }
        }

        //Applies a color setup value.
        void SetupFileUtility::ApplyFloat4(const SetupMap &values, const std::string &key, Float4 &target, float minimum, float maximum)
        {
            const auto found = values.find(key);
            if (found == values.end())
            {
                return;
            }

            Float4 parsedValue = target;
            if (ParseFloat4(found->second, parsedValue) && parsedValue.x >= minimum && parsedValue.x <= maximum && parsedValue.y >= minimum && parsedValue.y <= maximum &&
                parsedValue.z >= minimum && parsedValue.z <= maximum && parsedValue.w >= minimum && parsedValue.w <= maximum)
            {
                target = parsedValue;
            }
        }

        //Applies a keyboard setup value.
        void SetupFileUtility::ApplyKey(const SetupMap &values, const std::string &key, uint32_t &target)
        {
            const auto found = values.find(key);
            if (found == values.end())
            {
                return;
            }

            target = KeyCodeFromName(found->second, target);
        }

        //Applies a mouse setup value.
        void SetupFileUtility::ApplyMouseButton(const SetupMap &values, const std::string &key, uint32_t &target)
        {
            const auto found = values.find(key);
            if (found == values.end())
            {
                return;
            }

            target = MouseButtonFromName(found->second, target);
        }

        //Applies a plane sweep execution mode setup value.
        void SetupFileUtility::ApplyPlaneSweepExecutionMode(const SetupMap &values, const std::string &key, PlaneSweepExecutionMode &target)
        {
            const auto found = values.find(key);
            if (found == values.end())
            {
                return;
            }

            const std::string normalizedValue = NormalizeName(found->second);
            std::string compactValue;
            compactValue.reserve(normalizedValue.size());
            for (const char character : normalizedValue)
            {
                if (std::isalnum(static_cast<unsigned char>(character)) != 0)
                {
                    compactValue.push_back(character);
                }
            }

            if (compactValue == "CPUONLY")
            {
                target = PlaneSweepExecutionMode::CpuOnly;
                return;
            }

            if (compactValue == "GPU")
            {
                target = PlaneSweepExecutionMode::Gpu;
                return;
            }

            if (compactValue == "CPUGPUCOMPARE")
            {
                target = PlaneSweepExecutionMode::CpuGpuCompare;
                return;
            }

            std::cerr << "Warning: invalid COLMAP.PLANESWEEPEXECUTIONMODE '" << Trim(found->second)
                      << "'; falling back to CPU_ONLY." << std::endl;
            target = PlaneSweepExecutionMode::CpuOnly;
        }
    //Loads program setup values.
    ProgramSetup SetupFileUtility::Load(const std::string &fileName)
    {
        ProgramSetup setup;
        const SetupMap values = ReadSetupValues(fileName);

        ApplyString(values, "WINDOW.TITLE", setup.m_window.m_title);
        ApplyInt(values, "WINDOW.WIDTH", setup.m_window.m_width, 320, 7680);
        ApplyInt(values, "WINDOW.HEIGHT", setup.m_window.m_height, 240, 4320);

        ApplyFloat4(values, "GRAPHICS.CLEARCOLOR", setup.m_graphics.m_clearColor, 0.0f, 1.0f);
        ApplyBool(values, "GRAPHICS.VSYNCENABLED", setup.m_graphics.m_vsyncEnabled);
        ApplyBool(values, "GRAPHICS.ANTIALIASINGENABLED", setup.m_graphics.m_antiAliasingEnabled);
        ApplyInt(values, "GRAPHICS.ANTIALIASINGSAMPLES", setup.m_graphics.m_antiAliasingSamples, 1, 64);
        ApplyFloat(values, "GRAPHICS.GRIDLINEWIDTH", setup.m_graphics.m_gridLineWidth, 1.0f, 64.0f);
        ApplyFloat(values, "GRAPHICS.AXISLINEWIDTH", setup.m_graphics.m_axisLineWidth, 1.0f, 64.0f);
        ApplyFloat(values, "GRAPHICS.WIREFRAMELINEWIDTH", setup.m_graphics.m_wireframeLineWidth, 1.0f, 64.0f);

        ApplyString(values, "PATHS.TEXTUREFOLDER", setup.m_paths.m_textureFolder);
        ApplyString(values, "PATHS.MODELFOLDER", setup.m_paths.m_modelFolder);
        ApplyString(values, "PATHS.SHADERSOURCEFOLDER", setup.m_paths.m_shaderSourceFolder);
        ApplyString(values, "PATHS.SHADEROUTPUTFOLDER", setup.m_paths.m_shaderOutputFolder);
        ApplyString(values, "PATHS.CAPTUREOUTPUTFOLDER", setup.m_paths.m_captureOutputFolder);

        setup.m_shaders.m_sourceFolder = setup.m_paths.m_shaderSourceFolder;
        setup.m_shaders.m_outputFolder = setup.m_paths.m_shaderOutputFolder;
        ApplyBool(values, "SHADERS.COMPILEATSTARTUP", setup.m_shaders.m_compileAtStartup);
        ApplyString(values, "SHADERS.SOURCEFOLDER", setup.m_shaders.m_sourceFolder);
        ApplyString(values, "SHADERS.OUTPUTFOLDER", setup.m_shaders.m_outputFolder);
        ApplyString(values, "SHADERS.COMPILERPATH", setup.m_shaders.m_compilerPath);
        ApplyBool(values, "SHADERS.COMPILEONLYIFCHANGED", setup.m_shaders.m_compileOnlyIfChanged);
        ApplyString(values, "PATHS.SHADERSOURCEFOLDER", setup.m_shaders.m_sourceFolder);
        ApplyString(values, "PATHS.SHADEROUTPUTFOLDER", setup.m_shaders.m_outputFolder);
        setup.m_paths.m_shaderSourceFolder = setup.m_shaders.m_sourceFolder;
        setup.m_paths.m_shaderOutputFolder = setup.m_shaders.m_outputFolder;

        ApplyFloat(values, "CAMERA.FOVDEGREES", setup.m_camera.m_fovDegrees, 1.0f, 170.0f);
        ApplyFloat(values, "CAMERA.NEARPLANE", setup.m_camera.m_nearPlane, 0.001f, 1000.0f);
        ApplyFloat(values, "CAMERA.FARPLANE", setup.m_camera.m_farPlane, 1.0f, 100000.0f);
        ApplyFloat(values, "CAMERA.MOVESPEED", setup.m_camera.m_moveSpeed, 0.001f, 10000.0f);
        ApplyFloat(values, "CAMERA.ROTATESPEED", setup.m_camera.m_rotateSpeed, 0.001f, 1000.0f);
        ApplyFloat(values, "CAMERA.MOUSESENSITIVITY", setup.m_camera.m_mouseSensitivity, 0.000001f, 100.0f);
        ApplyFloat(values, "CAMERA.MOUSEWHEELMOVEAMOUNT", setup.m_camera.m_mouseWheelMoveAmount, 0.0f, 10000.0f);
        ApplyFloat(values, "CAMERA.SMOOTHMOUSEMOVESENSITIVITY", setup.m_camera.m_smoothMouseMoveSensitivity, 0.0f, 1000.0f);
        ApplyFloat(values, "CAMERA.FASTMULTIPLIER", setup.m_camera.m_fastMultiplier, 0.001f, 1000.0f);
        ApplyFloat(values, "CAMERA.SLOWMULTIPLIER", setup.m_camera.m_slowMultiplier, 0.001f, 1000.0f);
        ApplyFloat3(values, "CAMERA.POSITION", setup.m_camera.m_position);
        ApplyFloat3(values, "CAMERA.LOOKAT", setup.m_camera.m_lookAt);
        ApplyFloat3(values, "CAMERA.UP", setup.m_camera.m_up);
        ApplyFloat3(values, "CAMERA.SIDE", setup.m_camera.m_side);

        ApplyFloat3(values, "LIGHTING.POSITION", setup.m_lighting.m_position);
        ApplyFloat3(values, "LIGHTING.DIRECTION", setup.m_lighting.m_direction);
        ApplyFloat(values, "LIGHTING.FOVDEGREES", setup.m_lighting.m_fovDegrees, 1.0f, 170.0f);
        ApplyFloat4(values, "LIGHTING.AMBIENTCOLOR", setup.m_lighting.m_ambientColor, 0.0f, 1.0f);
        ApplyFloat4(values, "LIGHTING.DIFFUSECOLOR", setup.m_lighting.m_diffuseColor, 0.0f, 1.0f);
        ApplyFloat4(values, "LIGHTING.SPECULARCOLOR", setup.m_lighting.m_specularColor, 0.0f, 1.0f);
        ApplyFloat(values, "LIGHTING.SPECULARPOWER", setup.m_lighting.m_specularPower, 1.0f, 512.0f);
        ApplyBool(values, "LIGHTING.SHADOWSENABLED", setup.m_lighting.m_shadowsEnabled);
        ApplyFloat(values, "LIGHTING.SHADOWDISTANCE", setup.m_lighting.m_shadowDistance, 1.0f, 10000.0f);
        ApplyFloat(values, "LIGHTING.SHADOWDEPTHBIAS", setup.m_lighting.m_shadowDepthBias, 0.0f, 1.0f);
        ApplyFloat(values, "LIGHTING.SHADOWNORMALBIAS", setup.m_lighting.m_shadowNormalBias, 0.0f, 1.0f);
        ApplyFloat(values, "LIGHTING.SHADOWSOFTNESS", setup.m_lighting.m_shadowSoftness, 0.0f, 32.0f);

        ApplyBool(values, "SCENE.GRIDENABLED", setup.m_scene.m_gridEnabled);
        ApplyString(values, "SCENE.GRIDMODE", setup.m_scene.m_gridMode);
        ApplyString(values, "SCENE.GRIDTEXTURE", setup.m_scene.m_gridTexture);
        ApplyBool(values, "SCENE.LIGHTMARKERENABLED", setup.m_scene.m_lightMarkerEnabled);
        ApplyFloat(values, "SCENE.LIGHTMARKERSIZE", setup.m_scene.m_lightMarkerSize, 0.01f, 100.0f);
        ApplyFloat4(values, "SCENE.LIGHTMARKERCOLOR", setup.m_scene.m_lightMarkerColor, 0.0f, 1.0f);

        ApplyBool(values, "COLMAP.ENABLED", setup.m_colmap.m_enabled);
        ApplyString(values, "COLMAP.CAMERAINTRINSICFILE", setup.m_colmap.m_cameraIntrinsicFile);
        ApplyString(values, "COLMAP.CAMERAEXTRINSICFILE", setup.m_colmap.m_cameraExtrinsicFile);
        ApplyString(values, "COLMAP.CAMERAFILE", setup.m_colmap.m_cameraExtrinsicFile);
        ApplyString(values, "COLMAP.CAMERAIMAGEFOLDER", setup.m_colmap.m_cameraImageFolder);
        ApplyString(values, "COLMAP.CAMERAMASKFOLDER", setup.m_colmap.m_cameraMaskFolder);
        ApplyString(values, "COLMAP.CAMERADEPTHFOLDER", setup.m_colmap.m_cameraDepthFolder);
        ApplyString(values, "COLMAP.CAMERAPOINTSFILE", setup.m_colmap.m_cameraPointsFile);
        ApplyString(values, "COLMAP.POINTDISPLAYMODE", setup.m_colmap.m_pointDisplayMode);
        ApplyFloat(values, "COLMAP.CAMERASIZE", setup.m_colmap.m_cameraSize, 0.001f, 10000.0f);
        ApplyFloat(values, "COLMAP.POSITIONSCALE", setup.m_colmap.m_positionScale, 0.000001f, 1000000.0f);
        ApplyFloat3(values, "COLMAP.BASETRANSLATION", setup.m_colmap.m_baseTranslation);
        ApplyFloat3(values, "COLMAP.BASEROTATIONDEGREES", setup.m_colmap.m_baseRotationDegrees);
        ApplyFloat(values, "COLMAP.BASESCALE", setup.m_colmap.m_baseScale, 0.000001f, 1000000.0f);
        ApplyInt(values, "COLMAP.DISPLAYEVERYNTHCAMERA", setup.m_colmap.m_displayEveryNthCamera, 1, 1000000);
        ApplyInt(values, "COLMAP.DISPLAYEVERYNTHPOINT", setup.m_colmap.m_displayEveryNthPoint, 1, 1000000);
        ApplyInt(values, "COLMAP.DISPLAYEVERYNTHPIXEL", setup.m_colmap.m_displayEveryNthPixel, 1, 1000000);
        ApplyBool(values, "COLMAP.PLANESWEEPENABLED", setup.m_colmap.m_planeSweepEnabled);
        ApplyPlaneSweepExecutionMode(values, "COLMAP.PLANESWEEPEXECUTIONMODE", setup.m_colmap.m_planeSweepExecutionMode);
        ApplyInt(values, "COLMAP.PLANESWEEPREFERENCESTEP", setup.m_colmap.m_planeSweepReferenceStep, 1, 1000000);
        ApplyInt(values, "COLMAP.PLANESWEEPSOURCECOUNT", setup.m_colmap.m_planeSweepSourceCount, 1, 128);
        ApplyInt(values, "COLMAP.PLANESWEEPPLANECOUNT", setup.m_colmap.m_planeSweepPlaneCount, 2, 4096);
        ApplyFloat(values, "COLMAP.PLANESWEEPIMAGESCALE", setup.m_colmap.m_planeSweepImageScale, 0.01f, 1.0f);
        ApplyInt(values, "COLMAP.PLANESWEEPPATCHRADIUS", setup.m_colmap.m_planeSweepPatchRadius, 0, 16);
        ApplyInt(values, "COLMAP.PLANESWEEPMINVALIDSOURCEVIEWS", setup.m_colmap.m_planeSweepMinValidSourceViews, 1, 128);
        ApplyFloat(values, "COLMAP.PLANESWEEPCOSTTHRESHOLD", setup.m_colmap.m_planeSweepCostThreshold, 0.0f, 10.0f);
        ApplyFloat(values, "COLMAP.PLANESWEEPALPHATHRESHOLD", setup.m_colmap.m_planeSweepAlphaThreshold, 0.0f, 1.0f);
        ApplyFloat(values, "COLMAP.PLANESWEEPMINIMUMOPAQUEPATCHRATIO", setup.m_colmap.m_planeSweepMinimumOpaquePatchRatio, 0.0f, 1.0f);
        ApplyBool(values, "COLMAP.PLANESWEEPCONFIDENCEENABLED", setup.m_colmap.m_planeSweepConfidenceEnabled);
        ApplyFloat(values, "COLMAP.PLANESWEEPCONFIDENCEMINIMUMDIFFERENCE", setup.m_colmap.m_planeSweepConfidenceMinimumDifference, 0.0f, 10.0f);
        ApplyBool(values, "COLMAP.PLANESWEEPTEXTUREREJECTIONENABLED", setup.m_colmap.m_planeSweepTextureRejectionEnabled);
        ApplyFloat(values, "COLMAP.PLANESWEEPTEXTUREMINIMUMSTDDEV", setup.m_colmap.m_planeSweepTextureMinimumStdDev, 0.0f, 1.0f);
        ApplyBool(values, "COLMAP.PLANESWEEPCROSSVIEWCONSISTENCYENABLED", setup.m_colmap.m_planeSweepCrossViewConsistencyEnabled);
        ApplyInt(values, "COLMAP.PLANESWEEPCROSSVIEWCOUNT", setup.m_colmap.m_planeSweepCrossViewCount, 1, 128);
        ApplyInt(values, "COLMAP.PLANESWEEPCROSSVIEWMINIMUMVIEWS", setup.m_colmap.m_planeSweepCrossViewMinimumViews, 1, 128);
        ApplyFloat(values, "COLMAP.PLANESWEEPCROSSVIEWRELATIVEDEPTHTOLERANCE", setup.m_colmap.m_planeSweepCrossViewRelativeDepthTolerance, 0.0f, 10.0f);
        ApplyBool(values, "COLMAP.PLANESWEEPPOINTFUSIONENABLED", setup.m_colmap.m_planeSweepPointFusionEnabled);
        ApplyFloat(values, "COLMAP.PLANESWEEPFUSIONVOXELSIZE", setup.m_colmap.m_planeSweepFusionVoxelSize, 0.000001f, 1000000.0f);
        ApplyInt(values, "COLMAP.PLANESWEEPFUSIONMINIMUMOBSERVATIONS", setup.m_colmap.m_planeSweepFusionMinimumObservations, 1, 1000000);
        ApplyBool(values, "COLMAP.PLANESWEEPOUTLIERFILTERENABLED", setup.m_colmap.m_planeSweepOutlierFilterEnabled);
        ApplyFloat(values, "COLMAP.PLANESWEEPOUTLIERRADIUS", setup.m_colmap.m_planeSweepOutlierRadius, 0.000001f, 1000000.0f);
        ApplyInt(values, "COLMAP.PLANESWEEPOUTLIERMINIMUMNEIGHBORS", setup.m_colmap.m_planeSweepOutlierMinimumNeighbors, 0, 1000000);
        ApplyFloat(values, "COLMAP.PLANESWEEPNEARDEPTH", setup.m_colmap.m_planeSweepNearDepth, 0.000001f, 1000000.0f);
        ApplyFloat(values, "COLMAP.PLANESWEEPFARDEPTH", setup.m_colmap.m_planeSweepFarDepth, 0.000001f, 1000000.0f);
        ApplyString(values, "COLMAP.PLANESWEEPDEPTHRANGEMODE", setup.m_colmap.m_planeSweepDepthRangeMode);
        ApplyInt(values, "COLMAP.PLANESWEEPPOINTSTEP", setup.m_colmap.m_planeSweepPointStep, 1, 1000000);
        ApplyString(values, "COLMAP.PLANESWEEPOUTPUTFOLDER", setup.m_colmap.m_planeSweepOutputFolder);
        ApplyBool(values, "COLMAP.PLANESWEEPSAVEDEPTH", setup.m_colmap.m_planeSweepSaveDepth);
        ApplyBool(values, "COLMAP.PLANESWEEPEXPERIMENTMODE", setup.m_colmap.m_planeSweepExperimentMode);
        ApplyString(values, "COLMAP.DEPTHSCALEMODE", setup.m_colmap.m_depthScaleMode);
        ApplyFloat(values, "COLMAP.DEPTHSCALEVALUEMETRICTOCOLMAP", setup.m_colmap.m_depthScaleValueMetricToColmap, 0.000001f, 1000000.0f);
        ApplyString(values, "COLMAP.DEPTHVALUEMODE", setup.m_colmap.m_depthValueMode);
        ApplyFloat4(values, "COLMAP.CAMERACOLOR", setup.m_colmap.m_cameraColor, 0.0f, 1.0f);
        ApplyBool(values, "COLMAP.SHOWCAMERALABELS", setup.m_colmap.m_showCameraLabels);
        ApplyString(values, "COLMAP.COORDINATEMODE", setup.m_colmap.m_coordinateMode);

        ApplyBool(values, "GAUSSIANSPLAT.ENABLED", setup.m_gaussianSplat.m_enabled);
        ApplyString(values, "GAUSSIANSPLAT.FOLDER", setup.m_gaussianSplat.m_folder);
        ApplyString(values, "GAUSSIANSPLAT.FILEPATTERN", setup.m_gaussianSplat.m_filePattern);
        ApplyString(values, "GAUSSIANSPLAT.LOADMODE", setup.m_gaussianSplat.m_loadMode);
        ApplyString(values, "GAUSSIANSPLAT.COORDINATEMODE", setup.m_gaussianSplat.m_coordinateMode);
        ApplyBool(values, "GAUSSIANSPLAT.STORENORMALS", setup.m_gaussianSplat.m_storeNormals);
        ApplyBool(values, "GAUSSIANSPLAT.STORESPHERICALHARMONICS", setup.m_gaussianSplat.m_storeSphericalHarmonics);
        ApplyString(values, "GAUSSIANSPLAT.COLORMODE", setup.m_gaussianSplat.m_colorMode);
        ApplyFloat3(values, "GAUSSIANSPLAT.BASETRANSLATION", setup.m_gaussianSplat.m_baseTranslation);
        ApplyFloat3(values, "GAUSSIANSPLAT.BASEROTATIONDEGREES", setup.m_gaussianSplat.m_baseRotationDegrees);
        ApplyFloat(values, "GAUSSIANSPLAT.BASESCALE", setup.m_gaussianSplat.m_baseScale, 0.000001f, 1000000.0f);
        ApplyFloat(values, "GAUSSIANSPLAT.POSITIONSCALE", setup.m_gaussianSplat.m_positionScale, 0.000001f, 1000000.0f);
        ApplyInt(values, "GAUSSIANSPLAT.DISPLAYEVERYNTHSPLAT", setup.m_gaussianSplat.m_displayEveryNthSplat, 1, 1000000);
        ApplyInt(values, "GAUSSIANSPLAT.MAXSPLATS", setup.m_gaussianSplat.m_maxSplats, 0, 100000000);
        ApplyFloat(values, "GAUSSIANSPLAT.STREAMFPS", setup.m_gaussianSplat.m_streamFps, 0.001f, 1000.0f);
        ApplyBool(values, "GAUSSIANSPLAT.STREAMLOOP", setup.m_gaussianSplat.m_streamLoop);
        ApplyInt(values, "GAUSSIANSPLAT.STREAMPRELOADCOUNT", setup.m_gaussianSplat.m_streamPreloadCount, 1, 16);
        ApplyInt(values, "GAUSSIANSPLAT.STREAMWORKERTHREADCOUNT", setup.m_gaussianSplat.m_streamWorkerThreadCount, 1, 4);
        ApplyBool(values, "GAUSSIANSPLAT.STREAMWAITFORFRAME", setup.m_gaussianSplat.m_streamWaitForFrame);
        ApplyBool(values, "GAUSSIANSPLAT.STREAMTIMINGLOG", setup.m_gaussianSplat.m_streamTimingLog);
        ApplyFloat(values, "GAUSSIANSPLAT.STREAMTIMINGLOGINTERVALSECONDS", setup.m_gaussianSplat.m_streamTimingLogIntervalSeconds, 0.1f, 60.0f);
        ApplyBool(values, "GAUSSIANSPLAT.STREAMTIMINGLOGEVERYFRAME", setup.m_gaussianSplat.m_streamTimingLogEveryFrame);
        ApplyFloat(values, "GAUSSIANSPLAT.POINTSIZE", setup.m_gaussianSplat.m_pointSize, 0.1f, 256.0f);
        ApplyFloat(values, "GAUSSIANSPLAT.MAXPOINTSIZE", setup.m_gaussianSplat.m_maxPointSize, 1.0f, 8192.0f);
        ApplyFloat(values, "GAUSSIANSPLAT.SPLATSCALE", setup.m_gaussianSplat.m_splatScale, 0.000001f, 1000000.0f);
        ApplyFloat(values, "GAUSSIANSPLAT.OPACITYSCALE", setup.m_gaussianSplat.m_opacityScale, 0.0f, 1000.0f);
        ApplyBool(values, "GAUSSIANSPLAT.SORTENABLED", setup.m_gaussianSplat.m_sortEnabled);
        ApplyBool(values, "GAUSSIANSPLAT.SORTEVERYFRAME", setup.m_gaussianSplat.m_sortEveryFrame);
        ApplyBool(values, "GAUSSIANSPLAT.DEPTHTEST", setup.m_gaussianSplat.m_depthTest);
        ApplyBool(values, "GAUSSIANSPLAT.DEPTHWRITE", setup.m_gaussianSplat.m_depthWrite);
        ApplyBool(values, "GAUSSIANSPLAT.CLICKABLE", setup.m_gaussianSplat.m_clickable);

        ApplyBool(values, "XR.ENABLED", setup.m_xr.m_enabled);
        ApplyBool(values, "XR.REQUIRED", setup.m_xr.m_required);
        ApplyBool(values, "XR.MIRRORTOWINDOW", setup.m_xr.m_mirrorToWindow);
        ApplyString(values, "XR.MIRRORMODE", setup.m_xr.m_mirrorMode);
        ApplyString(values, "XR.REFERENCESPACE", setup.m_xr.m_referenceSpace);
        ApplyFloat(values, "XR.RENDERSCALE", setup.m_xr.m_renderScale, 0.25f, 4.0f);
        ApplyBool(values, "XR.USEDEPTHLAYER", setup.m_xr.m_useDepthLayer);
        ApplyBool(values, "XR.CONTROLLERINPUT", setup.m_xr.m_controllerInput);
        ApplyBool(values, "XR.PASSTHROUGH", setup.m_xr.m_passthroughEnabled);
        ApplyBool(values, "XR.PASSTHROUGHDEPTHTEST", setup.m_xr.m_passthroughDepthTestEnabled);
        ApplyString(values, "XR.PASSTHROUGHDEPTHMODE", setup.m_xr.m_passthroughDepthMode);
        ApplyString(values, "XR.PASSTHROUGHDEPTHDEBUGVIEW", setup.m_xr.m_passthroughDepthDebugView);
        ApplyString(values, "XR.PASSTHROUGHDEPTHVALUEMODE", setup.m_xr.m_passthroughDepthValueMode);
        ApplyString(values, "XR.PASSTHROUGHALPHADEBUG", setup.m_xr.m_passthroughAlphaDebug);
        ApplyBool(values, "XR.PASSTHROUGHDEPTHTEXTURESAMPLING", setup.m_xr.m_passthroughDepthTextureSampling);
        ApplyFloat(values, "XR.PASSTHROUGHDEPTHBIASMETERS", setup.m_xr.m_passthroughDepthBiasMeters, 0.0f, 1.0f);
        ApplyFloat(values, "XR.PASSTHROUGHDEPTHMAXMETERS", setup.m_xr.m_passthroughDepthMaxMeters, 0.1f, 100.0f);
        ApplyBool(values, "XR.PASSTHROUGHDEPTHHANDREMOVAL", setup.m_xr.m_passthroughDepthHandRemoval);
        ApplyBool(values, "XR.PASSTHROUGHDEPTHLOG", setup.m_xr.m_passthroughDepthLog);

        ApplyFloat(values, "UI.HUDTEXTSIZE", setup.m_ui.m_hudTextSize, 0.5f, 8.0f);
        ApplyInt(values, "UI.MAXVERTICES", setup.m_ui.m_maxVertices, 1024, 50000000);
        ApplyInt(values, "UI.MAXINDICES", setup.m_ui.m_maxIndices, 1536, 75000000);
        ApplyFloat4(values, "UI.TEXTCOLOR", setup.m_ui.m_textColor, 0.0f, 1.0f);
        ApplyFloat4(values, "UI.TEXTSHADOWCOLOR", setup.m_ui.m_textShadowColor, 0.0f, 1.0f);
        ApplyFloat4(values, "UI.GLASSPANELCOLOR", setup.m_ui.m_glassPanelColor, 0.0f, 1.0f);
        ApplyFloat4(values, "UI.GLASSPANELTINTCOLOR", setup.m_ui.m_glassPanelTintColor, 0.0f, 1.0f);
        ApplyFloat4(values, "UI.GLASSBORDERCOLOR", setup.m_ui.m_glassBorderColor, 0.0f, 1.0f);
        ApplyFloat4(values, "UI.GLASSACCENTCOLOR", setup.m_ui.m_glassAccentColor, 0.0f, 1.0f);
        ApplyFloat4(values, "UI.GLASSHEADERCOLOR", setup.m_ui.m_glassHeaderColor, 0.0f, 1.0f);
        ApplyFloat4(values, "UI.BUTTONCOLOR", setup.m_ui.m_buttonColor, 0.0f, 1.0f);
        ApplyFloat4(values, "UI.BUTTONHOVERCOLOR", setup.m_ui.m_buttonHoverColor, 0.0f, 1.0f);
        ApplyFloat4(values, "UI.BUTTONPRESSEDCOLOR", setup.m_ui.m_buttonPressedColor, 0.0f, 1.0f);

        ApplyKey(values, "KEYBOARD.MOVEFORWARD", setup.m_keyboard.m_moveForwardKey);
        ApplyKey(values, "KEYBOARD.MOVEBACKWARD", setup.m_keyboard.m_moveBackwardKey);
        ApplyKey(values, "KEYBOARD.MOVELEFT", setup.m_keyboard.m_moveLeftKey);
        ApplyKey(values, "KEYBOARD.MOVERIGHT", setup.m_keyboard.m_moveRightKey);
        ApplyKey(values, "KEYBOARD.MOVEUP", setup.m_keyboard.m_moveUpKey);
        ApplyKey(values, "KEYBOARD.MOVEDOWN", setup.m_keyboard.m_moveDownKey);
        ApplyKey(values, "KEYBOARD.PITCHUP", setup.m_keyboard.m_pitchUpKey);
        ApplyKey(values, "KEYBOARD.PITCHDOWN", setup.m_keyboard.m_pitchDownKey);
        ApplyKey(values, "KEYBOARD.YAWLEFT", setup.m_keyboard.m_yawLeftKey);
        ApplyKey(values, "KEYBOARD.YAWRIGHT", setup.m_keyboard.m_yawRightKey);
        ApplyKey(values, "KEYBOARD.ROLLLEFT", setup.m_keyboard.m_rollLeftKey);
        ApplyKey(values, "KEYBOARD.ROLLRIGHT", setup.m_keyboard.m_rollRightKey);
        ApplyKey(values, "KEYBOARD.FAST", setup.m_keyboard.m_fastKey);
        ApplyKey(values, "KEYBOARD.SLOW", setup.m_keyboard.m_slowKey);
        ApplyKey(values, "KEYBOARD.TRANSFORMTRANSLATE", setup.m_keyboard.m_transformTranslateKey);
        ApplyKey(values, "KEYBOARD.TRANSFORMROTATE", setup.m_keyboard.m_transformRotateKey);
        ApplyKey(values, "KEYBOARD.TRANSFORMSCALE", setup.m_keyboard.m_transformScaleKey);

        ApplyMouseButton(values, "MOUSE.LOOKBUTTON", setup.m_mouse.m_lookButton);
        ApplyMouseButton(values, "MOUSE.SMOOTHMOVEBUTTON", setup.m_mouse.m_smoothMoveButton);

        ApplyInt(
            values,
            "COLMAP.PLANESWEEPDEBUGREFERENCEIMAGEID",
            setup.m_colmap.m_planeSweepDebugReferenceImageId,
            0,
            1000000
        );

        ApplyBool(
            values,
            "COLMAP.PLANESWEEPDEBUGENABLED",
            setup.m_colmap.m_planeSweepDebugEnabled
        );

        ApplyBool(
            values,
            "COLMAP.PLANESWEEPDEBUGCOLMAPANCHORENABLED",
            setup.m_colmap.m_planeSweepDebugColmapAnchorEnabled
        );

        ApplyInt(
            values,
            "COLMAP.PLANESWEEPDEBUGPIXELX",
            setup.m_colmap.m_planeSweepDebugPixelX,
            0,
            100000
        );

        ApplyInt(
            values,
            "COLMAP.PLANESWEEPDEBUGPIXELY",
            setup.m_colmap.m_planeSweepDebugPixelY,
            0,
            100000
        );

        ApplyBool(
            values,
            "COLMAP.PLANESWEEPDEBUGSAVEEVERYPLANE",
            setup.m_colmap.m_planeSweepDebugSaveEveryPlane
        );

        ApplyInt(
            values,
            "COLMAP.PLANESWEEPDEBUGCOLMAPANCHORMAXPIXELDISTANCE",
            setup.m_colmap.m_planeSweepDebugColmapAnchorMaxPixelDistance,
            0,
            10000
        );

        return setup;
    }

    //Loads program setup values.
    ProgramSetup LoadProgramSetup(const std::string &fileName)
    {
        return SetupFileUtility::Load(fileName);
    }
}

