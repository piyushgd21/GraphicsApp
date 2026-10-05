///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	MultiViewUtility.cpp
//	Description	:
//		Implements multi-view camera and point cloud loading helpers.
//
///////////////////////////////////////////////////////////////////////

#include "MultiViewUtility.h"

#include "PlaneSweepGpu.h"
#include "PlaneSweepGpuTypes.h"

#include "PathUtility.h"
#include "StringUtility.h"
#include "TextureUtility.h"
#include "UIUtility.h"

#include <Windows.h>
#include <wincodec.h>
#include <wrl/client.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <chrono>

namespace graphicsFX
{
        // Debug run context used by ComputePlaneCost instrumentation.
        struct DebugRunContext
        {
            bool enabled{ false };
            int refImageId{ -1 };
            int pixelX{ 0 };
            int pixelY{ 0 };
            int currentPlaneIndex{ -1 };
            float currentDepth{ 0.0f };
            int minValidSourceViews{ 0 };
            std::ofstream detailedCsv;
            int lastPlaneValidSources{ 0 };
            bool lastPlaneHasCost{ false };
            float lastPlaneAvgCost{ 0.0f };
        };

        // Global debug context instance used by DebugPixelSweep and ComputePlaneCost.
        static DebugRunContext g_debugContext;

        //Returns a line with comments removed.
        std::string MultiViewUtility::MultiViewCameraUtility::RemoveComment(const std::string &line)
        {
            const std::size_t hash = line.find('#');
            const std::size_t semicolon = line.find(';');
            std::size_t comment = std::string::npos;

            if (hash != std::string::npos)
            {
                comment = hash;
            }

            if (semicolon != std::string::npos)
            {
                comment = std::min(comment, semicolon);
            }

            if (comment == std::string::npos)
            {
                return line;
            }

            return line.substr(0, comment);
        }

        //Debug-only: loads COLMAP points3D.txt and images.txt observations (preserves per-image 2D obs).
        ColmapPointsDebugReadResult MultiViewUtility::LoadColmapPointsWithObservations(const ColmapSetup &setup)
        {
            ColmapPointsDebugReadResult result;
            if (!setup.m_enabled || setup.m_cameraPointsFile.empty() || setup.m_cameraExtrinsicFile.empty())
            {
                return result;
            }

            // First parse points3D.txt to get XYZ and reprojection error.
            const std::filesystem::path pointsPath = PathUtility::ResolvePath(setup.m_cameraPointsFile);
            std::ifstream pointsFile(pointsPath);
            if (!pointsFile.is_open())
            {
                return result;
            }

            std::string line;
            while (std::getline(pointsFile, line))
            {
                const std::vector<std::string> tokens = MultiViewUtility::MultiViewSparsePointCloudUtility::SplitTokens(line);
                if (tokens.size() < 8U)
                {
                    continue;
                }

                // tokens: POINT3D_ID X Y Z R G B ERROR TRACK...
                int64_t pointId = -1;
                try
                {
                    pointId = std::stoll(tokens[0]);
                }
                catch (const std::exception &)
                {
                    continue;
                }

                double x = 0.0, y = 0.0, z = 0.0, err = 0.0;
                if (!MultiViewUtility::MultiViewSparsePointCloudUtility::ParseDouble(tokens[1], x) ||
                    !MultiViewUtility::MultiViewSparsePointCloudUtility::ParseDouble(tokens[2], y) ||
                    !MultiViewUtility::MultiViewSparsePointCloudUtility::ParseDouble(tokens[3], z) ||
                    !MultiViewUtility::MultiViewSparsePointCloudUtility::ParseDouble(tokens[7], err))
                {
                    continue;
                }

                ColmapPoint3DDebug point;
                point.m_pointId = pointId;
                const Float3 colmapPosition{ static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
                point.m_position = MultiViewUtility::MultiViewSparsePointCloudUtility::ConvertCoordinate(colmapPosition, setup.m_coordinateMode) * setup.m_positionScale;
                point.m_error = err;
                result.m_pointsById[pointId] = std::move(point);
            }

            // Next parse images.txt to extract per-image 2D observations (x y POINT3D_ID triples per image)
            const std::filesystem::path imagesPath = PathUtility::ResolvePath(setup.m_cameraExtrinsicFile);
            std::ifstream imagesFile(imagesPath);
            if (!imagesFile.is_open())
            {
                return result;
            }

            bool skipPointObservationLine = false;
            while (std::getline(imagesFile, line))
            {
                if (skipPointObservationLine)
                {
                    skipPointObservationLine = false;
                    // This line contains point observations for the last parsed pose.
                    // But we need to have stored the last pose image id; however we parse pose and then read this line inside the same loop below.
                    continue;
                }

                const std::vector<std::string> tokens = MultiViewUtility::MultiViewCameraUtility::SplitTokens(line);
                if (tokens.empty())
                {
                    continue;
                }

                MultiViewUtility::MultiViewCameraUtility::RawColmapPose pose;
                if (MultiViewUtility::MultiViewCameraUtility::TryParseImagePoseLine(tokens, pose))
                {
                    // read following observation line (if present)
                    std::string obsLine;
                    if (!std::getline(imagesFile, obsLine))
                    {
                        continue;
                    }

                    const std::vector<std::string> obsTokens = MultiViewUtility::MultiViewCameraUtility::SplitTokens(obsLine);
                    // obsTokens repeated triples: x y POINT3D_ID
                    uint32_t point2dIdx = 0U;
                    for (std::size_t i = 0U; i + 2U < obsTokens.size(); i += 3U)
                    {
                        double ox = 0.0, oy = 0.0;
                        int64_t pid = -1;
                        if (!MultiViewUtility::MultiViewSparsePointCloudUtility::ParseDouble(obsTokens[i], ox) ||
                            !MultiViewUtility::MultiViewSparsePointCloudUtility::ParseDouble(obsTokens[i + 1U], oy))
                        {
                            ++point2dIdx;
                            continue;
                        }

                        try
                        {
                            pid = std::stoll(obsTokens[i + 2U]);
                        }
                        catch (const std::exception &)
                        {
                            ++point2dIdx;
                            continue;
                        }

                        if (pid == -1) { ++point2dIdx; continue; }

                        const uint32_t imageId = pose.m_imageId;
                        ColmapPointObservation obs;
                        obs.m_imageId = imageId;
                        obs.m_point2dIdx = point2dIdx;
                        obs.m_x = static_cast<float>(ox);
                        obs.m_y = static_cast<float>(oy);

                        // add to per-image map
                        result.m_imageObservations[imageId].push_back(obs);

                        // add to point record if known
                        const auto it = result.m_pointsById.find(pid);
                        if (it != result.m_pointsById.end())
                        {
                            it->second.m_observations.push_back(obs);
                        }

                        ++point2dIdx;
                    }

                    continue;
                }

                // Try compact pose
                if (MultiViewUtility::MultiViewCameraUtility::TryParseCompactPoseLine(tokens, pose))
                {
                    // read following observation line (if present)
                    std::string obsLine;
                    if (!std::getline(imagesFile, obsLine))
                    {
                        continue;
                    }

                    const std::vector<std::string> obsTokens = MultiViewUtility::MultiViewCameraUtility::SplitTokens(obsLine);
                    uint32_t point2dIdx = 0U;
                    for (std::size_t i = 0U; i + 2U < obsTokens.size(); i += 3U)
                    {
                        double ox = 0.0, oy = 0.0;
                        int64_t pid = -1;
                        if (!MultiViewUtility::MultiViewSparsePointCloudUtility::ParseDouble(obsTokens[i], ox) ||
                            !MultiViewUtility::MultiViewSparsePointCloudUtility::ParseDouble(obsTokens[i + 1U], oy))
                        {
                            ++point2dIdx;
                            continue;
                        }

                        try
                        {
                            pid = std::stoll(obsTokens[i + 2U]);
                        }
                        catch (const std::exception &)
                        {
                            ++point2dIdx;
                            continue;
                        }

                        if (pid == -1) { ++point2dIdx; continue; }

                        const uint32_t imageId = pose.m_imageId;
                        ColmapPointObservation obs;
                        obs.m_imageId = imageId;
                        obs.m_point2dIdx = point2dIdx;
                        obs.m_x = static_cast<float>(ox);
                        obs.m_y = static_cast<float>(oy);

                        result.m_imageObservations[imageId].push_back(obs);
                        const auto it = result.m_pointsById.find(pid);
                        if (it != result.m_pointsById.end())
                        {
                            it->second.m_observations.push_back(obs);
                        }
                        ++point2dIdx;
                    }

                    continue;
                }
            }

            return result;
        }

        //Splits one text line into whitespace tokens.
        std::vector<std::string> MultiViewUtility::MultiViewCameraUtility::SplitTokens(std::string line)
        {
            std::replace(line.begin(), line.end(), ',', ' ');

            std::vector<std::string> tokens;
            std::istringstream stream(RemoveComment(line));
            std::string token;
            while (stream >> token)
            {
                tokens.push_back(token);
            }

            return tokens;
        }

        //Returns whether a token is an integer token.
        bool MultiViewUtility::MultiViewCameraUtility::IsIntegerToken(const std::string &token)
        {
            if (token.empty())
            {
                return false;
            }

            std::size_t start = token.front() == '-' || token.front() == '+' ? 1U : 0U;
            if (start >= token.size())
            {
                return false;
            }

            for (std::size_t index = start; index < token.size(); ++index)
            {
                if (std::isdigit(static_cast<unsigned char>(token[index])) == 0)
                {
                    return false;
                }
            }

            return true;
        }

        //Parses an unsigned integer token.
        bool MultiViewUtility::MultiViewCameraUtility::ParseUnsignedInteger(const std::string &token, uint32_t &result)
        {
            if (token.empty() || token.front() == '-')
            {
                return false;
            }

            try
            {
                std::size_t used = 0;
                const unsigned long parsedValue = std::stoul(token, &used);
                if (used != token.size() || parsedValue > std::numeric_limits<uint32_t>::max())
                {
                    return false;
                }

                result = static_cast<uint32_t>(parsedValue);
                return true;
            }
            catch (const std::exception &)
            {
                return false;
            }
        }

        //Parses a double token.
        bool MultiViewUtility::MultiViewCameraUtility::ParseDouble(const std::string &token, double &result)
        {
            try
            {
                std::size_t used = 0;
                const double parsedValue = std::stod(token, &used);
                if (used != token.size() || !std::isfinite(parsedValue))
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

        //Returns whether a quaternion has a plausible length.
        bool MultiViewUtility::MultiViewCameraUtility::QuaternionLooksValid(const RawColmapPose &pose)
        {
            const double lengthSquared = pose.m_qw * pose.m_qw + pose.m_qx * pose.m_qx + pose.m_qy * pose.m_qy + pose.m_qz * pose.m_qz;
            return lengthSquared > 0.25 && lengthSquared < 4.0;
        }

        //Parses a COLMAP image pose line.
        bool MultiViewUtility::MultiViewCameraUtility::TryParseImagePoseLine(const std::vector<std::string> &tokens, RawColmapPose &pose)
        {
            if (tokens.size() < 9U || !IsIntegerToken(tokens[0]) || !IsIntegerToken(tokens[8]))
            {
                return false;
            }

            RawColmapPose parsedPose;
            if (!ParseUnsignedInteger(tokens[0], parsedPose.m_imageId) || !ParseUnsignedInteger(tokens[8], parsedPose.m_cameraId))
            {
                return false;
            }

            parsedPose.m_name = tokens.size() > 9U ? tokens[9] : tokens[0];
            if (!ParseDouble(tokens[1], parsedPose.m_qw) || !ParseDouble(tokens[2], parsedPose.m_qx) || !ParseDouble(tokens[3], parsedPose.m_qy) || !ParseDouble(tokens[4], parsedPose.m_qz) ||
                !ParseDouble(tokens[5], parsedPose.m_tx) || !ParseDouble(tokens[6], parsedPose.m_ty) || !ParseDouble(tokens[7], parsedPose.m_tz) || !QuaternionLooksValid(parsedPose))
            {
                return false;
            }

            pose = parsedPose;
            return true;
        }

        //Parses a compact quaternion and translation pose line.
        bool MultiViewUtility::MultiViewCameraUtility::TryParseCompactPoseLine(const std::vector<std::string> &tokens, RawColmapPose &pose)
        {
            if (tokens.size() < 7U)
            {
                return false;
            }

            double firstValue = 0.0;
            const bool namedPose = !ParseDouble(tokens[0], firstValue);
            const std::size_t offset = namedPose ? 1U : 0U;
            if (tokens.size() < offset + 7U)
            {
                return false;
            }

            RawColmapPose parsedPose;
            parsedPose.m_imageId = 0U;
            parsedPose.m_cameraId = 0U;
            parsedPose.m_name = namedPose ? tokens[0] : "";
            if (!ParseDouble(tokens[offset + 0U], parsedPose.m_qw) || !ParseDouble(tokens[offset + 1U], parsedPose.m_qx) || !ParseDouble(tokens[offset + 2U], parsedPose.m_qy) ||
                !ParseDouble(tokens[offset + 3U], parsedPose.m_qz) || !ParseDouble(tokens[offset + 4U], parsedPose.m_tx) || !ParseDouble(tokens[offset + 5U], parsedPose.m_ty) ||
                !ParseDouble(tokens[offset + 6U], parsedPose.m_tz) || !QuaternionLooksValid(parsedPose))
            {
                return false;
            }

            pose = parsedPose;
            return true;
        }

        //Interprets known COLMAP intrinsic camera models.
        bool MultiViewUtility::MultiViewCameraUtility::InterpretCameraIntrinsicModel(ColmapCameraIntrinsicData &camera)
        {
            const std::string model = StringUtility::Uppercase(camera.m_model);
            const std::vector<double> &parameters = camera.m_parameters;
            camera.m_model = model;
            camera.m_supportedModel = false;

            if (model == "SIMPLE_PINHOLE")
            {
                if (parameters.size() != 3U)
                {
                    return false;
                }

                camera.m_fx = parameters[0];
                camera.m_fy = parameters[0];
                camera.m_cx = parameters[1];
                camera.m_cy = parameters[2];
                camera.m_supportedModel = true;
                return true;
            }

            if (model == "PINHOLE")
            {
                if (parameters.size() != 4U)
                {
                    return false;
                }

                camera.m_fx = parameters[0];
                camera.m_fy = parameters[1];
                camera.m_cx = parameters[2];
                camera.m_cy = parameters[3];
                camera.m_supportedModel = true;
                return true;
            }

            if (model == "SIMPLE_RADIAL")
            {
                if (parameters.size() != 4U)
                {
                    return false;
                }

                camera.m_fx = parameters[0];
                camera.m_fy = parameters[0];
                camera.m_cx = parameters[1];
                camera.m_cy = parameters[2];
                camera.m_k1 = parameters[3];
                camera.m_supportedModel = true;
                return true;
            }

            if (model == "RADIAL")
            {
                if (parameters.size() != 5U)
                {
                    return false;
                }

                camera.m_fx = parameters[0];
                camera.m_fy = parameters[0];
                camera.m_cx = parameters[1];
                camera.m_cy = parameters[2];
                camera.m_k1 = parameters[3];
                camera.m_k2 = parameters[4];
                camera.m_supportedModel = true;
                return true;
            }

            if (model == "OPENCV")
            {
                if (parameters.size() != 8U)
                {
                    return false;
                }

                camera.m_fx = parameters[0];
                camera.m_fy = parameters[1];
                camera.m_cx = parameters[2];
                camera.m_cy = parameters[3];
                camera.m_k1 = parameters[4];
                camera.m_k2 = parameters[5];
                camera.m_p1 = parameters[6];
                camera.m_p2 = parameters[7];
                camera.m_supportedModel = true;
                return true;
            }

            return true;
        }

        //Parses one COLMAP cameras.txt intrinsic line.
        bool MultiViewUtility::MultiViewCameraUtility::TryParseCameraIntrinsicLine(const std::vector<std::string> &tokens, ColmapCameraIntrinsicData &camera)
        {
            if (tokens.size() < 4U)
            {
                return false;
            }

            ColmapCameraIntrinsicData parsedCamera;
            if (!ParseUnsignedInteger(tokens[0], parsedCamera.m_cameraId) || !ParseUnsignedInteger(tokens[2], parsedCamera.m_width) || !ParseUnsignedInteger(tokens[3], parsedCamera.m_height))
            {
                return false;
            }

            parsedCamera.m_model = tokens[1];
            parsedCamera.m_parameters.reserve(tokens.size() - 4U);
            for (std::size_t index = 4U; index < tokens.size(); ++index)
            {
                double parameter = 0.0;
                if (!ParseDouble(tokens[index], parameter))
                {
                    return false;
                }

                parsedCamera.m_parameters.push_back(parameter);
            }

            if (!InterpretCameraIntrinsicModel(parsedCamera))
            {
                return false;
            }

            camera = std::move(parsedCamera);
            return true;
        }

        //Returns a safe normalized vector.
        //Converts a COLMAP coordinate vector into app coordinates.
        Float3 MultiViewUtility::MultiViewCameraUtility::ConvertCoordinate(const Float3 &value, const std::string &coordinateMode)
        {
            const std::string mode = StringUtility::Uppercase(coordinateMode);
            if (mode == "COLMAPTOAPP" || mode == "COLMAP_TO_APP")
            {
                return { value.x, -value.y, -value.z };
            }

            return value;
        }

        //Returns camera data from one raw COLMAP pose.
        ColmapCameraData MultiViewUtility::MultiViewCameraUtility::ConvertPose(const RawColmapPose &pose, const ColmapSetup &setup, std::size_t index)
        {
            const double length = std::sqrt(pose.m_qw * pose.m_qw + pose.m_qx * pose.m_qx + pose.m_qy * pose.m_qy + pose.m_qz * pose.m_qz);
            const double qw = pose.m_qw / length;
            const double qx = pose.m_qx / length;
            const double qy = pose.m_qy / length;
            const double qz = pose.m_qz / length;

            std::array<std::array<double, 3>, 3> rotation{};
            rotation[0] = {
                1.0 - (2.0 * qy * qy) - (2.0 * qz * qz), (2.0 * qx * qy) - (2.0 * qz * qw), (2.0 * qx * qz) + (2.0 * qy * qw)
            };
            rotation[1] = {
                (2.0 * qx * qy) + (2.0 * qz * qw), 1.0 - (2.0 * qx * qx) - (2.0 * qz * qz), (2.0 * qy * qz) - (2.0 * qx * qw)
            };
            rotation[2] = {
                (2.0 * qx * qz) - (2.0 * qy * qw), (2.0 * qy * qz) + (2.0 * qx * qw), 1.0 - (2.0 * qx * qx) - (2.0 * qy * qy)
            };

            const Float3 position = {
                static_cast<float>(-((rotation[0][0] * pose.m_tx) + (rotation[1][0] * pose.m_ty) + (rotation[2][0] * pose.m_tz))),
                static_cast<float>(-((rotation[0][1] * pose.m_tx) + (rotation[1][1] * pose.m_ty) + (rotation[2][1] * pose.m_tz))),
                static_cast<float>(-((rotation[0][2] * pose.m_tx) + (rotation[1][2] * pose.m_ty) + (rotation[2][2] * pose.m_tz)))
            };
            const Float3 right = {
                static_cast<float>(rotation[0][0]), static_cast<float>(rotation[0][1]), static_cast<float>(rotation[0][2])
            };
            const Float3 up = {
                static_cast<float>(-rotation[1][0]), static_cast<float>(-rotation[1][1]), static_cast<float>(-rotation[1][2])
            };
            const Float3 forward = {
                static_cast<float>(rotation[2][0]), static_cast<float>(rotation[2][1]), static_cast<float>(rotation[2][2])
            };

            ColmapCameraData camera;
            camera.m_imageId = pose.m_imageId;
            camera.m_cameraId = pose.m_cameraId;
            camera.m_name = pose.m_name.empty() ? "Camera_" + std::to_string(index) : pose.m_name;

            const Float3 convertedPosition = ConvertCoordinate(position, setup.m_coordinateMode) * setup.m_positionScale;
            const Float3 convertedForward = ConvertCoordinate(forward, setup.m_coordinateMode);
            const Float3 convertedUp = ConvertCoordinate(up, setup.m_coordinateMode);
            const Float3 convertedRight = ConvertCoordinate(right, setup.m_coordinateMode);

            camera.m_position = convertedPosition;
            camera.m_forward = normalizeOrDefault(convertedForward, { 0.0f, 0.0f, -1.0f });
            camera.m_up = normalizeOrDefault(convertedUp, { 0.0f, 1.0f, 0.0f });
            camera.m_right = normalizeOrDefault(convertedRight, { 1.0f, 0.0f, 0.0f });
            return camera;
        }

        //Loads COLMAP cameras.txt intrinsic records.
        std::vector<ColmapCameraIntrinsicData> MultiViewUtility::MultiViewCameraUtility::LoadIntrinsicData(const std::string &fileName)
        {
            std::vector<ColmapCameraIntrinsicData> intrinsics;
            if (fileName.empty())
            {
                return intrinsics;
            }

            std::ifstream file{ std::filesystem::path(fileName) };
            if (!file.is_open())
            {
                return intrinsics;
            }

            std::string line;
            while (std::getline(file, line))
            {
                const std::vector<std::string> tokens = SplitTokens(line);
                if (tokens.empty())
                {
                    continue;
                }

                ColmapCameraIntrinsicData camera;
                if (TryParseCameraIntrinsicLine(tokens, camera))
                {
                    intrinsics.push_back(std::move(camera));
                }
            }

            return intrinsics;
        }

        //Loads COLMAP images.txt extrinsic pose records.
        std::vector<ColmapCameraData> MultiViewUtility::MultiViewCameraUtility::LoadExtrinsicData(const ColmapSetup &setup)
        {
            std::vector<ColmapCameraData> cameras;
            if (setup.m_cameraExtrinsicFile.empty())
            {
                return cameras;
            }

            std::ifstream file{ std::filesystem::path(setup.m_cameraExtrinsicFile) };
            if (!file.is_open())
            {
                return cameras;
            }

            std::string line;
            std::size_t cameraIndex = 0;
            bool skipPointObservationLine = false;
            while (std::getline(file, line))
            {
                if (skipPointObservationLine)
                {
                    skipPointObservationLine = false;
                    continue;
                }

                const std::vector<std::string> tokens = SplitTokens(line);
                if (tokens.empty())
                {
                    continue;
                }

                RawColmapPose pose;
                if (TryParseImagePoseLine(tokens, pose))
                {
                    cameras.push_back(ConvertPose(pose, setup, cameraIndex));
                    ++cameraIndex;
                    skipPointObservationLine = true;
                    continue;
                }

                if (TryParseCompactPoseLine(tokens, pose))
                {
                    cameras.push_back(ConvertPose(pose, setup, cameraIndex));
                    ++cameraIndex;
                }
            }

            return cameras;
        }


        //Loads COLMAP camera intrinsics and poses from the configured files.
        ColmapCameraReadResult MultiViewUtility::LoadColmapCameras(const ColmapSetup &setup)
        {
            ColmapCameraReadResult result;
            if (!setup.m_enabled)
            {
                return result;
            }

            result.m_intrinsics = MultiViewUtility::MultiViewCameraUtility::LoadIntrinsicData(setup.m_cameraIntrinsicFile);
            result.m_cameras = MultiViewUtility::MultiViewCameraUtility::LoadExtrinsicData(setup);
            return result;
        }

        //Returns a line with comments removed.
        std::string MultiViewUtility::MultiViewSparsePointCloudUtility::RemoveComment(const std::string &line)
        {
            const std::size_t hash = line.find('#');
            if (hash == std::string::npos)
            {
                return line;
            }

            return line.substr(0U, hash);
        }

        //Splits one text line into whitespace tokens.
        std::vector<std::string> MultiViewUtility::MultiViewSparsePointCloudUtility::SplitTokens(const std::string &line)
        {
            std::vector<std::string> tokens;
            std::istringstream stream(RemoveComment(line));
            std::string token;
            while (stream >> token)
            {
                tokens.push_back(token);
            }

            return tokens;
        }

        //Parses an unsigned integer token.
        bool MultiViewUtility::MultiViewSparsePointCloudUtility::ParseUnsignedInteger(const std::string &token, uint32_t &result)
        {
            if (token.empty() || token.front() == '-')
            {
                return false;
            }

            try
            {
                std::size_t used = 0U;
                const unsigned long parsedValue = std::stoul(token, &used);
                if (used != token.size() || parsedValue > std::numeric_limits<uint32_t>::max())
                {
                    return false;
                }

                result = static_cast<uint32_t>(parsedValue);
                return true;
            }
            catch (const std::exception &)
            {
                return false;
            }
        }

        //Parses a double token.
        bool MultiViewUtility::MultiViewSparsePointCloudUtility::ParseDouble(const std::string &token, double &result)
        {
            try
            {
                std::size_t used = 0U;
                const double parsedValue = std::stod(token, &used);
                if (used != token.size() || !std::isfinite(parsedValue))
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

        //Parses an 8-bit color token.
        bool MultiViewUtility::MultiViewSparsePointCloudUtility::ParseColorByte(const std::string &token, uint8_t &result)
        {
            uint32_t value = 0U;
            if (!ParseUnsignedInteger(token, value) || value > 255U)
            {
                return false;
            }

            result = static_cast<uint8_t>(value);
            return true;
        }

        //Converts a COLMAP coordinate vector into app coordinates.
        Float3 MultiViewUtility::MultiViewSparsePointCloudUtility::ConvertCoordinate(const Float3 &value, const std::string &coordinateMode)
        {
            const std::string mode = StringUtility::Uppercase(coordinateMode);
            if (mode == "COLMAPTOAPP" || mode == "COLMAP_TO_APP")
            {
                return { value.x, -value.y, -value.z };
            }

            return value;
        }

        //Builds the image id set for loaded cameras.
        std::unordered_set<uint32_t> MultiViewUtility::MultiViewSparsePointCloudUtility::LoadedImageIds(const ColmapCameraReadResult &colmapData)
        {
            std::unordered_set<uint32_t> imageIds;
            for (const ColmapCameraData &camera : colmapData.m_cameras)
            {
                if (camera.m_imageId != 0U)
                {
                    imageIds.insert(camera.m_imageId);
                }
            }

            return imageIds;
        }

        //Counts point track observations that reference loaded cameras.
        std::size_t MultiViewUtility::MultiViewSparsePointCloudUtility::CountLoadedCameraTrackMatches(const std::vector<std::string> &tokens, const std::unordered_set<uint32_t> &loadedImageIds)
        {
            std::size_t trackMatches = 0U;
            for (std::size_t tokenIndex = 8U; tokenIndex + 1U < tokens.size(); tokenIndex += 2U)
            {
                if (loadedImageIds.empty())
                {
                    ++trackMatches;
                    continue;
                }

                uint32_t imageId = 0U;
                if (ParseUnsignedInteger(tokens[tokenIndex], imageId) && loadedImageIds.find(imageId) != loadedImageIds.end())
                {
                    ++trackMatches;
                }
            }

            return trackMatches;
        }


        //Builds sampled sparse point cloud points from a COLMAP points3D file.
        ColmapSparsePointCloudResult MultiViewUtility::BuildColmapSparsePointCloud(const ColmapCameraReadResult &colmapData, const ColmapSetup &setup)
        {
            ColmapSparsePointCloudResult result;
            if (!setup.m_enabled || setup.m_cameraPointsFile.empty())
            {
                return result;
            }

            const std::filesystem::path pointsPath = PathUtility::ResolvePath(setup.m_cameraPointsFile);
            std::ifstream file(pointsPath);
            if (!file.is_open())
            {
                return result;
            }

            const std::unordered_set<uint32_t> loadedImageIds = MultiViewUtility::MultiViewSparsePointCloudUtility::LoadedImageIds(colmapData);
            const std::size_t pointStep = static_cast<std::size_t>(std::max(1, setup.m_displayEveryNthPoint));

            std::string line;
            while (std::getline(file, line))
            {
                const std::vector<std::string> tokens = MultiViewUtility::MultiViewSparsePointCloudUtility::SplitTokens(line);
                if (tokens.size() < 8U)
                {
                    continue;
                }

                double x = 0.0;
                double y = 0.0;
                double z = 0.0;
                uint8_t red = 0U;
                uint8_t green = 0U;
                uint8_t blue = 0U;
                if (!MultiViewUtility::MultiViewSparsePointCloudUtility::ParseDouble(tokens[1], x) || !MultiViewUtility::MultiViewSparsePointCloudUtility::ParseDouble(tokens[2], y) ||
                    !MultiViewUtility::MultiViewSparsePointCloudUtility::ParseDouble(tokens[3], z) || !MultiViewUtility::MultiViewSparsePointCloudUtility::ParseColorByte(tokens[4], red) ||
                    !MultiViewUtility::MultiViewSparsePointCloudUtility::ParseColorByte(tokens[5], green) || !MultiViewUtility::MultiViewSparsePointCloudUtility::ParseColorByte(tokens[6], blue))
                {
                    ++result.m_pointsSkipped;
                    continue;
                }

                if ((result.m_pointsRead % pointStep) != 0U)
                {
                    ++result.m_pointsRead;
                    ++result.m_pointsSkipped;
                    continue;
                }

                ++result.m_pointsRead;
                result.m_trackMatches += MultiViewUtility::MultiViewSparsePointCloudUtility::CountLoadedCameraTrackMatches(tokens, loadedImageIds);

                const Float3 colmapPosition{
                    static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)
                };
                const Float3 convertedPosition = MultiViewUtility::MultiViewSparsePointCloudUtility::ConvertCoordinate(colmapPosition, setup.m_coordinateMode) * setup.m_positionScale;
                const Float4 color{
                    static_cast<float>(red) / 255.0f, static_cast<float>(green) / 255.0f, static_cast<float>(blue) / 255.0f, 1.0f
                };

                result.m_points.push_back(SceneObjectPointCloudPoint{ convertedPosition, color });
                ++result.m_pointsDisplayed;
            }

            return result;
        }

        //Initializes COM for WIC work.
        MultiViewUtility::MultiViewDepthPointCloudUtility::ComInitializer::ComInitializer(void)
        {
            const HRESULT result = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
            if (result == RPC_E_CHANGED_MODE)
            {
                return;
            }

            if (FAILED(result))
            {
                throw std::runtime_error("Failed to initialize COM for COLMAP depth loading.");
            }

            m_shouldUninitialize = true;
        }

        //Releases COM initialization when owned.
        MultiViewUtility::MultiViewDepthPointCloudUtility::ComInitializer::~ComInitializer(void)
        {
            if (m_shouldUninitialize)
            {
                CoUninitialize();
            }
        }

        //Throws a runtime error when an HRESULT fails.
        void MultiViewUtility::MultiViewDepthPointCloudUtility::CheckResult(HRESULT result, const char *messagePtr)
        {
            if (FAILED(result))
            {
                throw std::runtime_error(messagePtr);
            }
        }

        //Creates a WIC imaging factory.
        Microsoft::WRL::ComPtr<IWICImagingFactory> MultiViewUtility::MultiViewDepthPointCloudUtility::CreateFactory(void)
        {
            Microsoft::WRL::ComPtr<IWICImagingFactory> factoryPtr;
            CheckResult(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&factoryPtr)), "Failed to create WIC factory for COLMAP depth loading.");
            return factoryPtr;
        }

        //Returns a line with comments removed.
        std::string MultiViewUtility::MultiViewDepthPointCloudUtility::RemoveComment(const std::string &line)
        {
            const std::size_t hash = line.find('#');
            const std::size_t semicolon = line.find(';');
            std::size_t comment = std::string::npos;

            if (hash != std::string::npos)
            {
                comment = hash;
            }

            if (semicolon != std::string::npos)
            {
                comment = std::min(comment, semicolon);
            }

            if (comment == std::string::npos)
            {
                return line;
            }

            return line.substr(0U, comment);
        }

        //Splits one text line into whitespace tokens.
        std::vector<std::string> MultiViewUtility::MultiViewDepthPointCloudUtility::SplitTokens(const std::string &line)
        {
            std::vector<std::string> tokens;
            std::istringstream stream(RemoveComment(line));
            std::string token;
            while (stream >> token)
            {
                tokens.push_back(token);
            }

            return tokens;
        }

        //Parses an unsigned integer token.
        bool MultiViewUtility::MultiViewDepthPointCloudUtility::ParseUnsignedInteger(const std::string &token, uint32_t &result)
        {
            if (token.empty() || token.front() == '-')
            {
                return false;
            }

            try
            {
                std::size_t used = 0U;
                const unsigned long parsedValue = std::stoul(token, &used);
                if (used != token.size() || parsedValue > std::numeric_limits<uint32_t>::max())
                {
                    return false;
                }

                result = static_cast<uint32_t>(parsedValue);
                return true;
            }
            catch (const std::exception &)
            {
                return false;
            }
        }

        //Parses a double token.
        bool MultiViewUtility::MultiViewDepthPointCloudUtility::ParseDouble(const std::string &token, double &result)
        {
            try
            {
                std::size_t used = 0U;
                const double parsedValue = std::stod(token, &used);
                if (used != token.size() || !std::isfinite(parsedValue))
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

        //Returns the configured depth scale mode.
        MultiViewUtility::MultiViewDepthPointCloudUtility::DepthScaleMode MultiViewUtility::MultiViewDepthPointCloudUtility::ParseDepthScaleMode(std::string depthScaleMode)
        {
            const std::string mode = StringUtility::Uppercase(std::move(depthScaleMode));
            if (mode == "AUTOGLOBALFROMPOINTS3D" || mode == "AUTO_GLOBAL_FROM_POINTS3D" || mode == "AUTO")
            {
                return DepthScaleMode::AutoGlobalFromPoints3D;
            }

            if (mode == "AUTOPERIMAGEFROMPOINTS3D" || mode == "AUTO_PER_IMAGE_FROM_POINTS3D" || mode == "PERIMAGE" || mode == "PER_IMAGE")
            {
                return DepthScaleMode::AutoPerImageFromPoints3D;
            }

            if (mode == "AUTOHYBRIDFROMPOINTS3D" || mode == "AUTO_HYBRID_FROM_POINTS3D" || mode == "HYBRID")
            {
                return DepthScaleMode::AutoHybridFromPoints3D;
            }

            return DepthScaleMode::Manual;
        }

        //Returns the configured depth value convention.
        MultiViewUtility::MultiViewDepthPointCloudUtility::DepthValueMode MultiViewUtility::MultiViewDepthPointCloudUtility::ParseDepthValueMode(std::string depthValueMode)
        {
            const std::string mode = StringUtility::Uppercase(std::move(depthValueMode));
            if (mode == "RAYLENGTH" || mode == "RAY_LENGTH" || mode == "DISTANCE")
            {
                return DepthValueMode::RayLength;
            }

            return DepthValueMode::CameraZ;
        }

        //Converts a COLMAP coordinate vector into app coordinates.
        Float3 MultiViewUtility::MultiViewDepthPointCloudUtility::ConvertCoordinate(const Float3 &value, const std::string &coordinateMode)
        {
            const std::string mode = StringUtility::Uppercase(coordinateMode);
            if (mode == "COLMAPTOAPP" || mode == "COLMAP_TO_APP")
            {
                return { value.x, -value.y, -value.z };
            }

            return value;
        }

        //Returns a normalized stem with the final number unpadded.
        std::string MultiViewUtility::MultiViewDepthPointCloudUtility::NormalizeNumberedStem(const std::string &stem)
        {
            const std::string upperStem = StringUtility::Uppercase(stem);
            if (upperStem.empty())
            {
                return upperStem;
            }

            std::size_t end = upperStem.size();
            while (end > 0U && std::isdigit(static_cast<unsigned char>(upperStem[end - 1U])) == 0)
            {
                --end;
            }

            if (end == 0U)
            {
                return upperStem;
            }

            std::size_t begin = end;
            while (begin > 0U && std::isdigit(static_cast<unsigned char>(upperStem[begin - 1U])) != 0)
            {
                --begin;
            }

            if (begin == end)
            {
                return upperStem;
            }

            std::string number = upperStem.substr(begin, end - begin);
            const std::size_t firstDigit = number.find_first_not_of('0');
            number = firstDigit == std::string::npos ? "0" : number.substr(firstDigit);
            return upperStem.substr(0U, begin) + number + upperStem.substr(end);
        }

        //Adds one file lookup key.
        void MultiViewUtility::MultiViewDepthPointCloudUtility::AddLookupKey(FileLookup &lookup, const std::string &key, const std::filesystem::path &path)
        {
            if (!key.empty())
            {
                lookup.try_emplace(StringUtility::Uppercase(key), path);
            }
        }

        //Builds a stem lookup for files in a folder.
        MultiViewUtility::MultiViewDepthPointCloudUtility::FileLookup MultiViewUtility::MultiViewDepthPointCloudUtility::BuildFileLookup(
            const std::string &folderName,
            const std::vector<std::string> &extensions
        )
        {
            FileLookup lookup;
            if (folderName.empty())
            {
                return lookup;
            }

            const std::filesystem::path folderPath = PathUtility::ResolvePath(folderName);
            std::error_code errorCode;
            if (!std::filesystem::exists(folderPath, errorCode))
            {
                return lookup;
            }

            std::unordered_set<std::string> extensionSet;
            for (const std::string &extension : extensions)
            {
                extensionSet.insert(StringUtility::Uppercase(extension));
            }

            std::filesystem::directory_iterator iterator(folderPath, errorCode);
            const std::filesystem::directory_iterator endIterator;
            while (!errorCode && iterator != endIterator)
            {
                const std::filesystem::directory_entry entry = *iterator;
                if (entry.is_regular_file(errorCode))
                {
                    const std::string extension = StringUtility::Uppercase(entry.path().extension().string());
                    if (extensionSet.find(extension) != extensionSet.end())
                    {
                        const std::string stem = entry.path().stem().string();
                        AddLookupKey(lookup, stem, entry.path());
                        AddLookupKey(lookup, NormalizeNumberedStem(stem), entry.path());
                    }
                }

                iterator.increment(errorCode);
            }

            return lookup;
        }

        //Finds the image file matching a COLMAP camera record.
        std::filesystem::path MultiViewUtility::MultiViewDepthPointCloudUtility::FindMatchingFile(const FileLookup &lookup, const ColmapCameraData &camera, std::size_t cameraIndex)
        {
            if (lookup.empty())
            {
                return {};
            }

            std::vector<std::string> keys;
            const std::filesystem::path imagePath(camera.m_name);
            const std::string stem = imagePath.stem().string();
            keys.push_back(stem);
            keys.push_back(NormalizeNumberedStem(stem));
            keys.push_back("img_" + std::to_string(cameraIndex));
            keys.push_back("img_" + std::to_string(cameraIndex + 1U));
            keys.push_back("image_" + std::to_string(cameraIndex));
            keys.push_back("image_" + std::to_string(cameraIndex + 1U));

            for (const std::string &key : keys)
            {
                const auto found = lookup.find(StringUtility::Uppercase(key));
                if (found != lookup.end())
                {
                    return found->second;
                }
            }

            return {};
        }

        //Loads a metric float depth image through WIC.
        bool MultiViewUtility::MultiViewDepthPointCloudUtility::LoadFloatDepthImage(const std::filesystem::path &fileName, FloatDepthImage &depthImage)
        {
            try
            {
                if (fileName.empty())
                {
                    return false;
                }

                ComInitializer comInitializer;
                const auto factoryPtr = CreateFactory();

                Microsoft::WRL::ComPtr<IWICBitmapDecoder> decoderPtr;
                CheckResult(
                    factoryPtr->CreateDecoderFromFilename(fileName.wstring().c_str(), nullptr, GENERIC_READ, WICDecodeMetadataCacheOnLoad, &decoderPtr),
                    "Failed to create COLMAP depth decoder."
                );

                Microsoft::WRL::ComPtr<IWICBitmapFrameDecode> framePtr;
                CheckResult(decoderPtr->GetFrame(0U, &framePtr), "Failed to get COLMAP depth frame.");

                UINT width = 0U;
                UINT height = 0U;
                CheckResult(framePtr->GetSize(&width, &height), "Failed to get COLMAP depth size.");
                if (width == 0U || height == 0U)
                {
                    return false;
                }

                WICPixelFormatGUID sourceFormat{};
                CheckResult(framePtr->GetPixelFormat(&sourceFormat), "Failed to get COLMAP depth pixel format.");

                Microsoft::WRL::ComPtr<IWICBitmapSource> sourcePtr;
                if (IsEqualGUID(sourceFormat, GUID_WICPixelFormat32bppGrayFloat))
                {
                    CheckResult(framePtr.As(&sourcePtr), "Failed to use COLMAP depth frame as source.");
                }
                else
                {
                    Microsoft::WRL::ComPtr<IWICFormatConverter> converterPtr;
                    CheckResult(factoryPtr->CreateFormatConverter(&converterPtr), "Failed to create COLMAP depth converter.");

                    BOOL canConvert = FALSE;
                    CheckResult(converterPtr->CanConvert(sourceFormat, GUID_WICPixelFormat32bppGrayFloat, &canConvert), "Failed to query COLMAP depth conversion.");
                    if (!canConvert)
                    {
                        return false;
                    }

                    CheckResult(
                        converterPtr->Initialize(framePtr.Get(), GUID_WICPixelFormat32bppGrayFloat, WICBitmapDitherTypeNone, nullptr, 0.0, WICBitmapPaletteTypeCustom),
                        "Failed to convert COLMAP depth to float."
                    );
                    CheckResult(converterPtr.As(&sourcePtr), "Failed to use converted COLMAP depth source.");
                }

                const std::size_t pixelCount = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
                if (pixelCount > std::numeric_limits<UINT>::max() / sizeof(float))
                {
                    return false;
                }

                depthImage.m_width = width;
                depthImage.m_height = height;
                depthImage.m_values.resize(pixelCount);

                const UINT stride = width * static_cast<UINT>(sizeof(float));
                const UINT bufferSize = static_cast<UINT>(pixelCount * sizeof(float));
                CheckResult(sourcePtr->CopyPixels(nullptr, stride, bufferSize, reinterpret_cast<BYTE *>(depthImage.m_values.data())), "Failed to copy COLMAP depth pixels.");
                return true;
            }
            catch (const std::exception &)
            {
                return false;
            }
        }

        //Returns the intrinsic matching one camera.
        const ColmapCameraIntrinsicData *MultiViewUtility::MultiViewDepthPointCloudUtility::FindIntrinsic(const ColmapCameraReadResult &colmapData, const ColmapCameraData &camera)
        {
            for (const ColmapCameraIntrinsicData &intrinsic : colmapData.m_intrinsics)
            {
                if (intrinsic.m_cameraId == camera.m_cameraId)
                {
                    return &intrinsic;
                }
            }

            if (camera.m_cameraId == 0U && colmapData.m_intrinsics.size() == 1U)
            {
                return &colmapData.m_intrinsics.front();
            }

            return nullptr;
        }

        //Returns one sampled color from an image.
        Float4 MultiViewUtility::MultiViewDepthPointCloudUtility::SampleColor(
            const TextureData &texture,
            uint32_t x,
            uint32_t y,
            uint32_t sourceWidth,
            uint32_t sourceHeight,
            const Float4 &fallbackColor
        )
        {
            if (texture.m_width == 0U || texture.m_height == 0U || texture.m_pixels.empty() || sourceWidth == 0U || sourceHeight == 0U)
            {
                return fallbackColor;
            }

            const uint32_t colorX = std::min(texture.m_width - 1U, (x * texture.m_width) / sourceWidth);
            const uint32_t colorY = std::min(texture.m_height - 1U, (y * texture.m_height) / sourceHeight);
            const std::size_t pixelIndex = (static_cast<std::size_t>(colorY) * texture.m_width + colorX) * 4U;
            if (pixelIndex + 3U >= texture.m_pixels.size())
            {
                return fallbackColor;
            }

            return {
                static_cast<float>(texture.m_pixels[pixelIndex + 0U]) / 255.0f, static_cast<float>(texture.m_pixels[pixelIndex + 1U]) / 255.0f,
                static_cast<float>(texture.m_pixels[pixelIndex + 2U]) / 255.0f, 1.0f
            };
        }

        //Unprojects one depth sample into a world position.
        Float3 MultiViewUtility::MultiViewDepthPointCloudUtility::UnprojectDepthSample(
            const ColmapCameraData &camera,
            const ColmapCameraIntrinsicData &intrinsic,
            uint32_t x,
            uint32_t y,
            float depth,
            DepthValueMode depthValueMode
        )
        {
            const float localX = (static_cast<float>(x) + 0.5f - static_cast<float>(intrinsic.m_cx)) / static_cast<float>(intrinsic.m_fx);
            const float localY = (static_cast<float>(y) + 0.5f - static_cast<float>(intrinsic.m_cy)) / static_cast<float>(intrinsic.m_fy);

            if (depthValueMode == DepthValueMode::RayLength)
            {
                const Float3 rayDirection = normalize((camera.m_right * localX) - (camera.m_up * localY) + camera.m_forward);
                return camera.m_position + (rayDirection * depth);
            }

            return
                camera.m_position + (camera.m_right * (localX * depth)) - (camera.m_up * (localY * depth)) + (camera.m_forward * depth);
        }

        //Loads sparse COLMAP points and image tracks for scale estimation.
        std::vector<MultiViewUtility::MultiViewDepthPointCloudUtility::SparseScalePoint> MultiViewUtility::MultiViewDepthPointCloudUtility::LoadSparseScalePoints(const ColmapSetup &setup)
        {
            std::vector<SparseScalePoint> points;
            if (setup.m_cameraPointsFile.empty())
            {
                return points;
            }

            const std::filesystem::path pointsPath = PathUtility::ResolvePath(setup.m_cameraPointsFile);
            std::ifstream file(pointsPath);
            if (!file.is_open())
            {
                return points;
            }

            const std::size_t pointStep = static_cast<std::size_t>(std::max(1, setup.m_displayEveryNthPoint));
            std::size_t validPointIndex = 0U;

            std::string line;
            while (std::getline(file, line))
            {
                const std::vector<std::string> tokens = SplitTokens(line);
                if (tokens.size() < 8U)
                {
                    continue;
                }

                double x = 0.0;
                double y = 0.0;
                double z = 0.0;
                if (!ParseDouble(tokens[1], x) || !ParseDouble(tokens[2], y) || !ParseDouble(tokens[3], z))
                {
                    continue;
                }

                if ((validPointIndex % pointStep) != 0U)
                {
                    ++validPointIndex;
                    continue;
                }

                ++validPointIndex;

                SparseScalePoint point;
                const Float3 colmapPosition{
                    static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)
                };
                const Float3 convertedPosition = ConvertCoordinate(colmapPosition, setup.m_coordinateMode) * setup.m_positionScale;
                point.m_position = convertedPosition;

                for (std::size_t tokenIndex = 8U; tokenIndex + 1U < tokens.size(); tokenIndex += 2U)
                {
                    uint32_t imageId = 0U;
                    if (ParseUnsignedInteger(tokens[tokenIndex], imageId) && imageId != 0U && std::find(point.m_imageIds.begin(), point.m_imageIds.end(), imageId) == point.m_imageIds.end())
                    {
                        point.m_imageIds.push_back(imageId);
                    }
                }

                if (!point.m_imageIds.empty())
                {
                    points.push_back(std::move(point));
                }
            }

            return points;
        }

        //Builds sparse point lookup by image id.
        std::unordered_map<uint32_t, std::vector<std::size_t>> MultiViewUtility::MultiViewDepthPointCloudUtility::BuildSparsePointLookup(const std::vector<SparseScalePoint> &points)
        {
            std::unordered_map<uint32_t, std::vector<std::size_t>> lookup;
            for (std::size_t pointIndex = 0U; pointIndex < points.size(); ++pointIndex)
            {
                for (uint32_t imageId : points[pointIndex].m_imageIds)
                {
                    lookup[imageId].push_back(pointIndex);
                }
            }

            return lookup;
        }

        //Projects one world point into a camera pixel.
        bool MultiViewUtility::MultiViewDepthPointCloudUtility::ProjectPointToDepthPixel(
            const ColmapCameraData &camera,
            const ColmapCameraIntrinsicData &intrinsic,
            const Float3 &point,
            uint32_t imageWidth,
            uint32_t imageHeight,
            DepthValueMode depthValueMode,
            uint32_t &x,
            uint32_t &y,
            float &colmapDepth
        )
        {
            const Float3 offset = point - camera.m_position;
            const float cameraDepth = dot(offset, camera.m_forward);
            if (cameraDepth <= 0.0001f)
            {
                return false;
            }

            const float projectedX = (dot(offset, camera.m_right) * static_cast<float>(intrinsic.m_fx) / cameraDepth) + static_cast<float>(intrinsic.m_cx);
            const float projectedY = (-dot(offset, camera.m_up) * static_cast<float>(intrinsic.m_fy) / cameraDepth) + static_cast<float>(intrinsic.m_cy);
            if (!std::isfinite(projectedX) || !std::isfinite(projectedY))
            {
                return false;
            }

            const long pixelX = std::lround(projectedX);
            const long pixelY = std::lround(projectedY);
            if (pixelX < 0L || pixelY < 0L || pixelX >= static_cast<long>(imageWidth) || pixelY >= static_cast<long>(imageHeight))
            {
                return false;
            }

            x = static_cast<uint32_t>(pixelX);
            y = static_cast<uint32_t>(pixelY);
            colmapDepth = depthValueMode == DepthValueMode::RayLength ? std::sqrt(dot(offset, offset)) : cameraDepth;
            return std::isfinite(colmapDepth) && colmapDepth > 0.0f;
        }

        //Returns the median value from a non-empty list.
        float MultiViewUtility::MultiViewDepthPointCloudUtility::MedianValue(std::vector<float> &values)
        {
            std::sort(values.begin(), values.end());
            const std::size_t middle = values.size() / 2U;
            if ((values.size() % 2U) != 0U)
            {
                return values[middle];
            }

            return (values[middle - 1U] + values[middle]) * 0.5f;
        }

        //Formats one float for debug output.
        std::string MultiViewUtility::MultiViewDepthPointCloudUtility::FormatFloat(float value)
        {
            std::ostringstream stream;
            stream << value;
            return stream.str();
        }

        //Returns whether a scale mode uses sparse point matches.
        bool MultiViewUtility::MultiViewDepthPointCloudUtility::DepthScaleModeUsesPointMatches(DepthScaleMode depthScaleMode)
        {
            return
                depthScaleMode == DepthScaleMode::AutoGlobalFromPoints3D || depthScaleMode == DepthScaleMode::AutoPerImageFromPoints3D || depthScaleMode == DepthScaleMode::AutoHybridFromPoints3D;
        }

        //Returns the scale lookup key for one camera.
        uint32_t MultiViewUtility::MultiViewDepthPointCloudUtility::GetDepthScaleImageKey(const ColmapCameraData &camera, std::size_t cameraIndex)
        {
            if (camera.m_imageId != 0U)
            {
                return camera.m_imageId;
            }

            const std::size_t fallbackKey = cameraIndex + 1U;
            if (fallbackKey > std::numeric_limits<uint32_t>::max())
            {
                return std::numeric_limits<uint32_t>::max();
            }

            return static_cast<uint32_t>(fallbackKey);
        }

        //Estimates metric depth to COLMAP depth scale lookup.
        MultiViewUtility::MultiViewDepthPointCloudUtility::DepthScaleLookup MultiViewUtility::MultiViewDepthPointCloudUtility::EstimateDepthScaleLookupFromPoints3D(
            const ColmapCameraReadResult &colmapData,
            const ColmapSetup &setup,
            const FileLookup &depthFiles,
            DepthValueMode depthValueMode,
            std::size_t cameraStep
        )
        {
            DepthScaleLookup depthScaleLookup;
            const std::vector<SparseScalePoint> sparsePoints = LoadSparseScalePoints(setup);
            if (sparsePoints.empty())
            {
                return depthScaleLookup;
            }

            const std::unordered_map<uint32_t, std::vector<std::size_t>> sparsePointLookup = BuildSparsePointLookup(sparsePoints);
            std::vector<float> globalScaleSamples;
            for (std::size_t cameraIndex = 0U; cameraIndex < colmapData.m_cameras.size(); cameraIndex += cameraStep)
            {
                const ColmapCameraData &camera = colmapData.m_cameras[cameraIndex];
                const auto pointList = sparsePointLookup.find(camera.m_imageId);
                if (pointList == sparsePointLookup.end())
                {
                    continue;
                }

                const ColmapCameraIntrinsicData *intrinsicPtr = FindIntrinsic(colmapData, camera);
                if (intrinsicPtr == nullptr || !intrinsicPtr->m_supportedModel || intrinsicPtr->m_fx <= 0.0 || intrinsicPtr->m_fy <= 0.0)
                {
                    continue;
                }

                const std::filesystem::path depthPath = FindMatchingFile(depthFiles, camera, cameraIndex);
                FloatDepthImage depthImage;
                if (!LoadFloatDepthImage(depthPath, depthImage))
                {
                    continue;
                }

                std::vector<float> imageScaleSamples;
                for (std::size_t pointIndex : pointList->second)
                {
                    uint32_t x = 0U;
                    uint32_t y = 0U;
                    float colmapDepth = 0.0f;
                    if (!ProjectPointToDepthPixel(camera,
                        *intrinsicPtr,
                        sparsePoints[pointIndex].m_position, depthImage.m_width, depthImage.m_height, depthValueMode, x, y, colmapDepth))
                    {
                        continue;
                    }

                    const std::size_t depthIndex = static_cast<std::size_t>(y) * depthImage.m_width + x;
                    const float metricDepth = depthImage.m_values[depthIndex];
                    if (!std::isfinite(metricDepth) || metricDepth <= 0.0f)
                    {
                        continue;
                    }

                    const float scaleSample = colmapDepth / metricDepth;
                    if (std::isfinite(scaleSample) && scaleSample > 0.000001f && scaleSample < 1000000.0f)
                    {
                        globalScaleSamples.push_back(scaleSample);
                        imageScaleSamples.push_back(scaleSample);
                    }
                }

                DepthScaleEstimate imageEstimate;
                imageEstimate.m_sampleCount = imageScaleSamples.size();
                if (imageScaleSamples.size() >= s_minAutoScaleSamples)
                {
                    imageEstimate.m_scale = MedianValue(imageScaleSamples);
                    imageEstimate.m_estimated = true;
                    depthScaleLookup.m_imageScales[GetDepthScaleImageKey(camera, cameraIndex)] = imageEstimate;
                }
            }

            depthScaleLookup.m_globalSampleCount = globalScaleSamples.size();
            if (globalScaleSamples.size() >= s_minAutoScaleSamples)
            {
                depthScaleLookup.m_globalScale = MedianValue(globalScaleSamples);
                depthScaleLookup.m_globalEstimated = true;
            }

            return depthScaleLookup;
        }

        //Returns the resolved depth scale lookup for unprojection.
        MultiViewUtility::MultiViewDepthPointCloudUtility::DepthScaleLookup MultiViewUtility::MultiViewDepthPointCloudUtility::ResolveDepthScaleLookup(
            const ColmapCameraReadResult &colmapData,
            const ColmapSetup &setup,
            DepthScaleMode depthScaleMode,
            DepthValueMode depthValueMode,
            const FileLookup &depthFiles,
            std::size_t cameraStep
        )
        {
            DepthScaleLookup depthScaleLookup;
            depthScaleLookup.m_globalScale = setup.m_depthScaleValueMetricToColmap;
            if (DepthScaleModeUsesPointMatches(depthScaleMode))
            {
                depthScaleLookup = EstimateDepthScaleLookupFromPoints3D(colmapData, setup, depthFiles, depthValueMode, cameraStep);
                if (depthScaleLookup.m_globalEstimated)
                {
                    UIUtility::Print("Auto COLMAP depth scale = " + FormatFloat(depthScaleLookup.m_globalScale) + " from " + std::to_string(depthScaleLookup.m_globalSampleCount) +
                        " sparse/depth samples, per-image estimates = " + std::to_string(depthScaleLookup.m_imageScales.size()) + ".\n");
                }
                else
                {
                    depthScaleLookup.m_globalScale = setup.m_depthScaleValueMetricToColmap;
                    UIUtility::Print("Auto COLMAP depth scale did not find enough samples. Using manual scale " + FormatFloat(setup.m_depthScaleValueMetricToColmap) + ".\n");
                }
            }

            return depthScaleLookup;
        }

        //Returns the resolved depth scale for one camera.
        float MultiViewUtility::MultiViewDepthPointCloudUtility::ResolveCameraDepthScale(
            const DepthScaleLookup &depthScaleLookup,
            DepthScaleMode depthScaleMode,
            const ColmapCameraData &camera,
            std::size_t cameraIndex,
            float manualScale,
            bool &usedPerImageScale
        )
        {
            usedPerImageScale = false;
            if (depthScaleMode == DepthScaleMode::Manual)
            {
                return manualScale;
            }

            const uint32_t imageKey = GetDepthScaleImageKey(camera, cameraIndex);
            const auto imageScale = depthScaleLookup.m_imageScales.find(imageKey);
            if ((depthScaleMode == DepthScaleMode::AutoPerImageFromPoints3D || depthScaleMode == DepthScaleMode::AutoHybridFromPoints3D) && imageScale != depthScaleLookup.m_imageScales.end() &&
                imageScale->second.m_estimated)
            {
                usedPerImageScale = true;
                return imageScale->second.m_scale;
            }

            if ((depthScaleMode == DepthScaleMode::AutoGlobalFromPoints3D || depthScaleMode == DepthScaleMode::AutoHybridFromPoints3D) && depthScaleLookup.m_globalEstimated)
            {
                return depthScaleLookup.m_globalScale;
            }

            return depthScaleMode == DepthScaleMode::AutoGlobalFromPoints3D ? depthScaleLookup.m_globalScale : manualScale;
        }

        //Returns whether plane sweep can run.
        bool MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepReady(const ColmapCameraReadResult &colmapData, const ColmapSetup &setup)
        {
            return setup.m_enabled && setup.m_planeSweepEnabled && !setup.m_cameraImageFolder.empty() && !colmapData.m_cameras.empty() && !colmapData.m_intrinsics.empty();
        }

        //Builds a lookup from image id to camera index.
        std::unordered_map<uint32_t, std::size_t> MultiViewUtility::MultiViewPlaneSweepUtility::BuildCameraIndexLookup(const ColmapCameraReadResult &colmapData)
        {
            std::unordered_map<uint32_t, std::size_t> lookup;
            for (std::size_t cameraIndex = 0U; cameraIndex < colmapData.m_cameras.size(); ++cameraIndex)
            {
                const uint32_t imageId = colmapData.m_cameras[cameraIndex].m_imageId;
                if (imageId != 0U)
                {
                    lookup.try_emplace(imageId, cameraIndex);
                }
            }

            return lookup;
        }

        //Loads one plane sweep image.
        bool MultiViewUtility::MultiViewPlaneSweepUtility::LoadPlaneSweepImage(
            const ColmapCameraReadResult &colmapData,
            const ColmapSetup &setup,
            const MultiViewDepthPointCloudUtility::FileLookup &imageFiles,
            const MultiViewDepthPointCloudUtility::FileLookup &maskFiles,
            std::size_t cameraIndex,
            PlaneSweepImage &image
        )
        {
            if (cameraIndex >= colmapData.m_cameras.size())
            {
                return false;
            }

            const ColmapCameraData &camera = colmapData.m_cameras[cameraIndex];
            const ColmapCameraIntrinsicData *intrinsicPtr = MultiViewDepthPointCloudUtility::FindIntrinsic(colmapData, camera);
            if (intrinsicPtr == nullptr || !intrinsicPtr->m_supportedModel || intrinsicPtr->m_fx <= 0.0 || intrinsicPtr->m_fy <= 0.0)
            {
                return false;
            }

            const std::filesystem::path imagePath = MultiViewDepthPointCloudUtility::FindMatchingFile(imageFiles, camera, cameraIndex);
            TextureData texture;
            if (imagePath.empty() || !LoadTexture(imagePath.string(), texture) || texture.m_width == 0U || texture.m_height == 0U || texture.m_pixels.empty())
            {
                return false;
            }

            image.m_camera = camera;
            image.m_intrinsic = *intrinsicPtr;
            image.m_texture = std::move(texture);
            const std::filesystem::path maskPath = MultiViewDepthPointCloudUtility::FindMatchingFile(maskFiles, camera, cameraIndex);
            if (!maskPath.empty())
            {
                TextureData maskTexture;
                if (LoadTexture(maskPath.string(), maskTexture) && maskTexture.m_width > 0U && maskTexture.m_height > 0U && !maskTexture.m_pixels.empty())
                {
                    image.m_maskTexture = std::move(maskTexture);
                    image.m_hasMask = true;
                    for (std::size_t alphaIndex = 3U; alphaIndex < image.m_maskTexture.m_pixels.size(); alphaIndex += 4U)
                    {
                        if (image.m_maskTexture.m_pixels[alphaIndex] < 255U)
                        {
                            image.m_maskUsesAlpha = true;
                            break;
                        }
                    }
                }
            }

            image.m_cameraIndex = cameraIndex;
            return HasOpaquePixel(image, setup.m_planeSweepAlphaThreshold);
        }

        //Returns whether a plane sweep image contains an opaque pixel.
        bool MultiViewUtility::MultiViewPlaneSweepUtility::HasOpaquePixel(const PlaneSweepImage &image, float alphaThreshold)
        {
            const TextureData &texture = image.m_hasMask ? image.m_maskTexture : image.m_texture;
            if (texture.m_channelCount < 4U || texture.m_pixels.size() < 4U)
            {
                return false;
            }

            const uint8_t alphaByteThreshold = static_cast<uint8_t>(std::lround(std::clamp(alphaThreshold, 0.0f, 1.0f) * 255.0f));
            for (std::size_t pixelIndex = 0U; pixelIndex + 3U < texture.m_pixels.size(); pixelIndex += 4U)
            {
                uint8_t opacity = texture.m_pixels[pixelIndex + 3U];
                if (image.m_hasMask && !image.m_maskUsesAlpha)
                {
                    const uint32_t luminance = (static_cast<uint32_t>(texture.m_pixels[pixelIndex + 0U]) * 77U) +
                        (static_cast<uint32_t>(texture.m_pixels[pixelIndex + 1U]) * 150U) +
                        (static_cast<uint32_t>(texture.m_pixels[pixelIndex + 2U]) * 29U);
                    opacity = static_cast<uint8_t>(luminance / 256U);
                }

                if (opacity > alphaByteThreshold)
                {
                    return true;
                }
            }

            return false;
        }

        //Selects source camera indices for one reference camera.
        std::vector<std::size_t> MultiViewUtility::MultiViewPlaneSweepUtility::SelectSourceCameraIndices(
            const ColmapCameraReadResult &colmapData,
            const ColmapSetup &setup,
            const std::unordered_map<uint32_t, std::size_t> &cameraIndexLookup,
            const std::unordered_map<uint32_t, std::vector<std::size_t>> &sparsePointLookup,
            const std::vector<MultiViewDepthPointCloudUtility::SparseScalePoint> &sparsePoints,
            std::size_t referenceCameraIndex
        )
        {
            //Counts sparse points shared between the reference camera and each source camera.
            std::unordered_map<std::size_t, std::size_t> sharedPointCounts;
            const ColmapCameraData &referenceCamera = colmapData.m_cameras[referenceCameraIndex];
            const auto referenceSparsePoints = sparsePointLookup.find(referenceCamera.m_imageId);
            if (referenceSparsePoints != sparsePointLookup.end())
            {
                for (std::size_t sparsePointIndex : referenceSparsePoints->second)
                {
                    for (uint32_t imageId : sparsePoints[sparsePointIndex].m_imageIds)
                    {
                        const auto cameraIndex = cameraIndexLookup.find(imageId);
                        if (cameraIndex != cameraIndexLookup.end() && cameraIndex->second != referenceCameraIndex)
                        {
                            ++sharedPointCounts[cameraIndex->second];
                        }
                    }
                }
            }

            //Builds source candidates using camera distance, view alignment, and shared sparse points.
            std::vector<PlaneSweepSourceCandidate> candidates;
            candidates.reserve(colmapData.m_cameras.size());
            for (std::size_t cameraIndex = 0U; cameraIndex < colmapData.m_cameras.size(); ++cameraIndex)
            {
                if (cameraIndex == referenceCameraIndex)
                {
                    continue;
                }

                const ColmapCameraData &camera = colmapData.m_cameras[cameraIndex];
                const Float3 offset = camera.m_position - referenceCamera.m_position;
                const float distance = std::max(length(offset), 0.0001f);
                const float viewAlignment = std::max(0.0f, dot(referenceCamera.m_forward, camera.m_forward));
                const auto sharedPoints = sharedPointCounts.find(cameraIndex);

                PlaneSweepSourceCandidate candidate;
                candidate.m_cameraIndex = cameraIndex;
                candidate.m_sharedPoints = sharedPoints == sharedPointCounts.end() ? 0U : sharedPoints->second;
                candidate.m_score = distance / (0.25f + viewAlignment);
                candidates.push_back(candidate);
            }

            //Ranks cameras by shared sparse points before using geometric proximity.
            std::sort(candidates.begin(), candidates.end(), [](const PlaneSweepSourceCandidate &a, const PlaneSweepSourceCandidate &b)
            {
                if (a.m_sharedPoints != b.m_sharedPoints)
                {
                    return a.m_sharedPoints > b.m_sharedPoints;
                }

                return a.m_score < b.m_score;
            });

            //Returns extra candidates so unavailable source images can be skipped later.
            std::vector<std::size_t> sourceCameraIndices;
            const std::size_t requestedSourceCount = static_cast<std::size_t>(std::max(1, setup.m_planeSweepSourceCount));
            const std::size_t candidateCount = std::min(candidates.size(), requestedSourceCount * 4U);
            sourceCameraIndices.reserve(candidateCount);
            for (std::size_t candidateIndex = 0U; candidateIndex < candidateCount; ++candidateIndex)
            {
                sourceCameraIndices.push_back(candidates[candidateIndex].m_cameraIndex);
            }

            return sourceCameraIndices;
        }

        //Returns a sparse point depth range for one camera. O(k log k). Sorting for getting 5 to 95 percentile.
        MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepDepthRange MultiViewUtility::MultiViewPlaneSweepUtility::EstimateDepthRange(
            const ColmapCameraData &camera,
            const ColmapCameraIntrinsicData &intrinsic,
            const ColmapSetup &setup,
            const std::unordered_map<uint32_t, std::vector<std::size_t>> &sparsePointLookup,
            const std::vector<MultiViewDepthPointCloudUtility::SparseScalePoint> &sparsePoints
        )
        {
            PlaneSweepDepthRange range;
            range.m_nearDepth = std::min(setup.m_planeSweepNearDepth, setup.m_planeSweepFarDepth);
            range.m_farDepth = std::max(setup.m_planeSweepNearDepth, setup.m_planeSweepFarDepth);

            if (!DepthRangeUsesSparsePoints(setup.m_planeSweepDepthRangeMode))
            {
                return range;
            }

            const auto sparsePointList = sparsePointLookup.find(camera.m_imageId);
            if (sparsePointList == sparsePointLookup.end())
            {
                return range;
            }

            std::vector<float> depthValues;
            for (std::size_t sparsePointIndex : sparsePointList->second)
            {
                float x = 0.0f;
                float y = 0.0f;
                float depth = 0.0f;
                if (!ProjectPoint(camera, intrinsic, sparsePoints[sparsePointIndex].m_position, x, y, depth))
                {
                    continue;
                }

                if (x >= 0.0f && y >= 0.0f && x < static_cast<float>(intrinsic.m_width) && y < static_cast<float>(intrinsic.m_height))
                {
                    depthValues.push_back(depth);
                }
            }

            if (depthValues.size() < 12U)
            {
                return range;
            }

            const float sparseNear = PercentileValue(depthValues, 0.05f);
            const float sparseFar = PercentileValue(depthValues, 0.95f);
            const float expansion = std::max((sparseFar - sparseNear) * 0.10f, 0.01f);
            range.m_nearDepth = std::max(0.0001f, sparseNear - expansion);
            range.m_farDepth = std::max(range.m_nearDepth + 0.0001f, sparseFar + expansion);
            range.m_fromSparsePoints = true;
            return range;
        }

        //Unprojects one reference pixel at a camera Z depth.
        Float3 MultiViewUtility::MultiViewPlaneSweepUtility::UnprojectPixel(const ColmapCameraData &camera, const ColmapCameraIntrinsicData &intrinsic, float x, float y, float depth)
        {
            const float targetX = (x + 0.5f - static_cast<float>(intrinsic.m_cx)) / static_cast<float>(intrinsic.m_fx);
            const float targetY = (y + 0.5f - static_cast<float>(intrinsic.m_cy)) / static_cast<float>(intrinsic.m_fy);
            float localX = targetX;
            float localY = targetY;

            for (int iteration = 0; iteration < 5; ++iteration)
            {
                const float radiusSquared = (localX * localX) + (localY * localY);
                const float radial = 1.0f + (static_cast<float>(intrinsic.m_k1) * radiusSquared) + (static_cast<float>(intrinsic.m_k2) * radiusSquared * radiusSquared);
                const float distortedX = (localX * radial) + (2.0f * static_cast<float>(intrinsic.m_p1) * localX * localY) +
                    (static_cast<float>(intrinsic.m_p2) * (radiusSquared + (2.0f * localX * localX)));
                const float distortedY = (localY * radial) + (static_cast<float>(intrinsic.m_p1) * (radiusSquared + (2.0f * localY * localY))) +
                    (2.0f * static_cast<float>(intrinsic.m_p2) * localX * localY);
                localX += targetX - distortedX;
                localY += targetY - distortedY;
            }

            return camera.m_position + (camera.m_right * (localX * depth)) - (camera.m_up * (localY * depth)) + (camera.m_forward * depth);
        }

        //Projects one world point into an image pixel.
        bool MultiViewUtility::MultiViewPlaneSweepUtility::ProjectPoint(
            const ColmapCameraData &camera,
            const ColmapCameraIntrinsicData &intrinsic,
            const Float3 &point,
            float &x,
            float &y,
            float &depth
        )
        {
            const Float3 offset = point - camera.m_position;
            depth = dot(offset, camera.m_forward);
            if (depth <= 0.0001f)
            {
                return false;
            }

            const float localX = dot(offset, camera.m_right) / depth;
            const float localY = -dot(offset, camera.m_up) / depth;
            const float radiusSquared = (localX * localX) + (localY * localY);
            const float radial = 1.0f + (static_cast<float>(intrinsic.m_k1) * radiusSquared) + (static_cast<float>(intrinsic.m_k2) * radiusSquared * radiusSquared);
            const float distortedX = (localX * radial) + (2.0f * static_cast<float>(intrinsic.m_p1) * localX * localY) +
                (static_cast<float>(intrinsic.m_p2) * (radiusSquared + (2.0f * localX * localX)));
            const float distortedY = (localY * radial) + (static_cast<float>(intrinsic.m_p1) * (radiusSquared + (2.0f * localY * localY))) +
                (2.0f * static_cast<float>(intrinsic.m_p2) * localX * localY);

            x = (static_cast<float>(intrinsic.m_fx) * distortedX) + static_cast<float>(intrinsic.m_cx);
            y = (static_cast<float>(intrinsic.m_fy) * distortedY) + static_cast<float>(intrinsic.m_cy);
            return std::isfinite(x) && std::isfinite(y) && std::isfinite(depth);
        }

        //Samples one texture with bilinear filtering.
        bool MultiViewUtility::MultiViewPlaneSweepUtility::SampleBilinear(const TextureData &texture, float x, float y, Float4 &color)
        {
            if (texture.m_width == 0U || texture.m_height == 0U || texture.m_pixels.empty() || x < 0.0f || y < 0.0f || x > static_cast<float>(texture.m_width - 1U) ||
                y > static_cast<float>(texture.m_height - 1U))
            {
                return false;
            }

            const uint32_t x0 = static_cast<uint32_t>(std::floor(x));
            const uint32_t y0 = static_cast<uint32_t>(std::floor(y));
            const uint32_t x1 = std::min(texture.m_width - 1U, x0 + 1U);
            const uint32_t y1 = std::min(texture.m_height - 1U, y0 + 1U);
            const float tx = x - static_cast<float>(x0);
            const float ty = y - static_cast<float>(y0);

            const auto samplePixel = [&texture](uint32_t sampleX, uint32_t sampleY, uint32_t channel) -> float
            {
                const std::size_t index = ((static_cast<std::size_t>(sampleY) * texture.m_width + sampleX) * 4U) + channel;
                return index < texture.m_pixels.size() ? static_cast<float>(texture.m_pixels[index]) / 255.0f : 0.0f;
            };

            const auto sampleChannel = [&](uint32_t channel) -> float
            {
                const float top = (samplePixel(x0, y0, channel) * (1.0f - tx)) + (samplePixel(x1, y0, channel) * tx);
                const float bottom = (samplePixel(x0, y1, channel) * (1.0f - tx)) + (samplePixel(x1, y1, channel) * tx);
                return (top * (1.0f - ty)) + (bottom * ty);
            };

            color = { sampleChannel(0U), sampleChannel(1U), sampleChannel(2U), sampleChannel(3U) };
            return true;
        }

        //Samples one plane sweep image with its optional mask.
        bool MultiViewUtility::MultiViewPlaneSweepUtility::SamplePlaneSweepPixel(const PlaneSweepImage &image, float x, float y, Float4 &color)
        {
            if (!SampleBilinear(image.m_texture, x, y, color))
            {
                return false;
            }

            if (!image.m_hasMask)
            {
                return true;
            }

            const float imageWidth = static_cast<float>(std::max(1U, image.m_texture.m_width - 1));
            const float imageHeight = static_cast<float>(std::max(1U, image.m_texture.m_height - 1));
            const float maskX = (x / imageWidth) * static_cast<float>(std::max(1U, image.m_maskTexture.m_width - 1));
            const float maskY = (y / imageHeight) * static_cast<float>(std::max(1U, image.m_maskTexture.m_height - 1));
            Float4 maskColor;
            if (!SampleBilinear(image.m_maskTexture, maskX, maskY, maskColor))
            {
                return false;
            }

            color.w = image.m_maskUsesAlpha ? maskColor.w : ((maskColor.x * 0.299f) + (maskColor.y * 0.587f) + (maskColor.z * 0.114f));
            return true;
        }

        //Computes the matching cost for one depth plane.
        bool MultiViewUtility::MultiViewPlaneSweepUtility::ComputePlaneCost(
            const PlaneSweepImage &referenceImage,
            const std::vector<PlaneSweepImage> &sourceImages,
            float x,
            float y,
            float depth,
            int patchRadius,
            int minValidSourceViews,
            float alphaThreshold,
            float minimumOpaquePatchRatio,
            float &cost
        )
        {
            float totalCost = 0.0f;
            int validSourceViews = 0;
            const int patchWidth = (patchRadius * 2) + 1;
            const int patchArea = patchWidth * patchWidth;
            const int minimumOpaqueSamples = std::max(1, static_cast<int>(std::ceil(static_cast<float>(patchArea) * minimumOpaquePatchRatio)));

            // Check whether instrumentation is active for this reference pixel
            const bool instrument = g_debugContext.enabled && g_debugContext.refImageId == static_cast<int>(referenceImage.m_camera.m_imageId) &&
                static_cast<int>(std::lround(x)) == g_debugContext.pixelX && static_cast<int>(std::lround(y)) == g_debugContext.pixelY;

            for (const PlaneSweepImage &sourceImage : sourceImages)
            {
                float sourceCost = 0.0f;
                int sampleCount = 0;

                // instrumentation counters
                int patchSamplesConsidered = 0;
                int patchSamplesReferenceOpaque = 0;
                int patchSamplesProjSuccess = 0;
                int patchSamplesProjFailures = 0;
                int patchSamplesProjOutside = 0;
                int patchSamplesSourceMaskRejected = 0;
                int patchSamplesSourceOpaque = 0;
                bool sourceContributed = false;
                float sourceIndividualCost = 0.0f;
                std::string rejectionReason = "";

                for (int patchY = -patchRadius; patchY <= patchRadius; ++patchY)
                {
                    for (int patchX = -patchRadius; patchX <= patchRadius; ++patchX)
                    {
                        const float referenceX = x + static_cast<float>(patchX);
                        const float referenceY = y + static_cast<float>(patchY);

                        Float4 referenceColor;
                        ++patchSamplesConsidered;
                        if (!SamplePlaneSweepPixel(referenceImage, referenceX, referenceY, referenceColor) || referenceColor.w <= alphaThreshold)
                        {
                            continue;
                        }
                        ++patchSamplesReferenceOpaque;

                        const Float3 point = UnprojectPixel(referenceImage.m_camera, referenceImage.m_intrinsic, referenceX, referenceY, depth);
                        float sourceX = 0.0f;
                        float sourceY = 0.0f;
                        float sourceDepth = 0.0f;
                        if (!ProjectPoint(sourceImage.m_camera, sourceImage.m_intrinsic, point, sourceX, sourceY, sourceDepth))
                        {
                            ++patchSamplesProjFailures;
                            continue;
                        }

                        // Check whether the projected coordinate falls inside the source image bounds
                        const bool projInside = sourceX >= 0.0f && sourceY >= 0.0f && sourceX < static_cast<float>(sourceImage.m_texture.m_width) && sourceY < static_cast<float>(sourceImage.m_texture.m_height);
                        if (!projInside)
                        {
                            ++patchSamplesProjOutside;
                            continue;
                        }

                        ++patchSamplesProjSuccess;

                        Float4 sourceColor;
                        if (!SamplePlaneSweepPixel(sourceImage, sourceX, sourceY, sourceColor) || sourceColor.w <= alphaThreshold)
                        {
                            ++patchSamplesSourceMaskRejected;
                            continue;
                        }

                        ++patchSamplesSourceOpaque;

                        const float delta = (std::fabs(referenceColor.x - sourceColor.x) + std::fabs(referenceColor.y - sourceColor.y) + std::fabs(referenceColor.z - sourceColor.z)) / 3.0f;
                        sourceCost += delta;
                        ++sampleCount;
                    }
                }

                if (sampleCount >= minimumOpaqueSamples)
                {
                    totalCost += sourceCost / static_cast<float>(sampleCount);
                    ++validSourceViews;
                    sourceContributed = true;
                    sourceIndividualCost = sourceCost / static_cast<float>(sampleCount);
                }
                else
                {
                    // Determine rejection reason using observed counters
                    if (patchSamplesProjSuccess == 0)
                    {
                        rejectionReason = "projectionFailed";
                    }
                    else
                    {
                        rejectionReason = "insufficientOpaqueSamples";
                    }
                }

                // If instrumentation is enabled, write per-source diagnostics
                if (instrument && g_debugContext.detailedCsv.is_open())
                {
                    const int srcId = static_cast<int>(sourceImage.m_camera.m_imageId);
                    const std::string srcName = sourceImage.m_camera.m_name.empty() ? ("image_" + std::to_string(srcId)) : sourceImage.m_camera.m_name;
                    // compute projected center for summary
                    float centerPX = 0.0f, centerPY = 0.0f, centerPDepth = 0.0f;
                    const bool centerProj = ProjectPoint(sourceImage.m_camera, sourceImage.m_intrinsic, UnprojectPixel(referenceImage.m_camera, referenceImage.m_intrinsic, x, y, depth), centerPX, centerPY, centerPDepth);
                    const bool centerInside = centerProj && centerPX >= 0.0f && centerPY >= 0.0f && centerPX < static_cast<float>(sourceImage.m_texture.m_width) && centerPY < static_cast<float>(sourceImage.m_texture.m_height);
                    // reference patch validity and required counts
                    const int refRequiredOpaque = minimumOpaqueSamples;
                    const bool refPatchValid = patchSamplesReferenceOpaque >= refRequiredOpaque;

                    g_debugContext.detailedCsv << g_debugContext.currentPlaneIndex << ",";
                    g_debugContext.detailedCsv << depth << "," << srcId << "," << srcName << ",";
                    g_debugContext.detailedCsv << centerPX << "," << centerPY << "," << centerPDepth << ",";
                    g_debugContext.detailedCsv << (centerProj ? "1" : "0") << "," << (centerInside ? "1" : "0") << ",";

                    // reference patch diagnostics
                    g_debugContext.detailedCsv << patchSamplesConsidered << "," << patchSamplesReferenceOpaque << "," << refRequiredOpaque << "," << (refPatchValid ? "1" : "0") << ",";

                    // per-source projection and sampling diagnostics
                    // fields: projSuccess, projFailures, projOutside, sourceMaskRejected, sourceOpaque
                    g_debugContext.detailedCsv << patchSamplesProjSuccess << "," << patchSamplesProjFailures << "," << patchSamplesProjOutside << "," << patchSamplesSourceMaskRejected << "," << patchSamplesSourceOpaque << ",";

                    // final contribution and cost
                    g_debugContext.detailedCsv << sampleCount << "," << refRequiredOpaque << "," << (sourceContributed ? "1" : "0") << ",";
                    g_debugContext.detailedCsv << (sourceContributed ? std::to_string(sourceIndividualCost) : std::string()) << ",";

                    // rejection reason (if any)
                    g_debugContext.detailedCsv << (sourceContributed ? std::string() : rejectionReason) << "\n";

                    // Also emit a console-friendly per-source message so you see rejections immediately
                    std::ostringstream ps;
                    ps << "[PlaneSweepDebug] plane=" << g_debugContext.currentPlaneIndex << " depth=" << g_debugContext.currentDepth << "\n";
                    ps << "source=" << srcId << " expectedPatchSamples=" << patchArea << " refRejectedByMask=" << (patchSamplesConsidered - patchSamplesReferenceOpaque) << " projFailures=" << patchSamplesProjFailures << " projOutside=" << patchSamplesProjOutside << " srcMaskRejected=" << patchSamplesSourceMaskRejected << " validSamples=" << sampleCount << " requiredSamples=" << refRequiredOpaque << " contributed=" << (sourceContributed ? "true" : "false") << "\n";
                    ps << "reason=" << (sourceContributed ? "" : rejectionReason) << "\n";
                    UIUtility::Print(ps.str());
                }
            }

            // instrumentation: write per-plane summary
            if (instrument && g_debugContext.detailedCsv.is_open())
            {
                // write a plane summary row with sourceImageId = -1
                g_debugContext.detailedCsv << g_debugContext.currentPlaneIndex << "," << depth << ",-1,SUMMARY,,,,,,," << ",,,,,," << validSourceViews << "," << minValidSourceViews << "," << (validSourceViews >= minValidSourceViews ? "1" : "0") << "," << (validSourceViews >= minValidSourceViews ? std::to_string((totalCost / static_cast<float>(validSourceViews))) : "") << "\n";
                g_debugContext.lastPlaneValidSources = validSourceViews;
            }

            if (validSourceViews < minValidSourceViews)
            {
                g_debugContext.lastPlaneHasCost = false;
                g_debugContext.lastPlaneAvgCost = 0.0f;
                return false;
            }

            cost = totalCost / static_cast<float>(validSourceViews);
            g_debugContext.lastPlaneHasCost = std::isfinite(cost);
            g_debugContext.lastPlaneAvgCost = cost;
            return g_debugContext.lastPlaneHasCost;
        }

        //Saves one plane sweep depth output.
        bool MultiViewUtility::MultiViewPlaneSweepUtility::SaveDepthOutput(
            const ColmapSetup &setup,
            const ColmapCameraData &camera,
            const std::vector<float> &depthValues,
            uint32_t width,
            uint32_t height
        )
        {
            if (!setup.m_planeSweepSaveDepth || depthValues.empty() || width == 0U || height == 0U)
            {
                return false;
            }

            std::error_code errorCode;
            const std::filesystem::path outputFolder = PathUtility::ResolvePath(setup.m_planeSweepOutputFolder);
            std::filesystem::create_directories(outputFolder, errorCode);
            if (errorCode)
            {
                return false;
            }

            const std::filesystem::path cameraPath(camera.m_name);
            const std::string stem = cameraPath.stem().empty() ? "image_" + std::to_string(camera.m_imageId) : cameraPath.stem().string();
            const std::filesystem::path basePath = outputFolder / ("PlaneSweep_" + std::to_string(camera.m_imageId) + "_" + stem);

            std::ofstream depthFile(basePath.string() + ".bin", std::ios::binary);
            if (depthFile.is_open())
            {
                depthFile.write(reinterpret_cast<const char *>(&width), sizeof(width));
                depthFile.write(reinterpret_cast<const char *>(&height), sizeof(height));
                depthFile.write(reinterpret_cast<const char *>(depthValues.data()), static_cast<std::streamsize>(depthValues.size() * sizeof(float)));
            }

            float minimumDepth = std::numeric_limits<float>::max();
            float maximumDepth = 0.0f;
            for (float depth : depthValues)
            {
                if (std::isfinite(depth) && depth > 0.0f)
                {
                    minimumDepth = std::min(minimumDepth, depth);
                    maximumDepth = std::max(maximumDepth, depth);
                }
            }

            if (maximumDepth <= minimumDepth)
            {
                return depthFile.is_open();
            }

            TextureData preview;
            preview.m_width = width;
            preview.m_height = height;
            preview.m_channelCount = 4U;
            preview.m_pixels.resize(static_cast<std::size_t>(width) * height * 4U, 255U);

            const float inverseRange = 1.0f / (maximumDepth - minimumDepth);
            for (std::size_t index = 0U; index < depthValues.size(); ++index)
            {
                const float depth = depthValues[index];
                const uint8_t value = std::isfinite(depth) && depth > 0.0f ? static_cast<uint8_t>(std::clamp((depth - minimumDepth) * inverseRange, 0.0f, 1.0f) * 255.0f) : 0U;
                preview.m_pixels[(index * 4U) + 0U] = value;
                preview.m_pixels[(index * 4U) + 1U] = value;
                preview.m_pixels[(index * 4U) + 2U] = value;
                preview.m_pixels[(index * 4U) + 3U] = 255U;
            }

            const bool previewSaved = SaveTexture(basePath.string() + ".png", preview, TextureFileType::Png);
            return depthFile.is_open() || previewSaved;
        }

        //Debug one reference pixel across all candidate depth planes and write diagnostics.
        void MultiViewUtility::MultiViewPlaneSweepUtility::DebugPixelSweep(
            const ColmapSetup &setup,
            const PlaneSweepImage &referenceImage,
            const std::vector<PlaneSweepImage> &sourceImages,
            const PlaneSweepDepthRange &depthRange
        )
        {
            if (!setup.m_planeSweepDebugEnabled)
            {
                return;
            }

            std::error_code ec;
            const std::filesystem::path outputFolder = PathUtility::ResolvePath(setup.m_planeSweepOutputFolder) / "PlaneSweepDebug";
            std::filesystem::create_directories(outputFolder, ec);
            if (ec)
            {
                UIUtility::Print("PlaneSweepDebug: failed to create output folder " + outputFolder.string() + "\n");
                return;
            }

            const uint32_t refImageId = referenceImage.m_camera.m_imageId;
            const std::string refName = referenceImage.m_camera.m_name.empty() ? ("image_" + std::to_string(refImageId)) : referenceImage.m_camera.m_name;
            const int debugX = setup.m_planeSweepDebugPixelX;
            const int debugY = setup.m_planeSweepDebugPixelY;

            // Sanity print
            std::ostringstream info;
            info << "PlaneSweepDebug for reference=" << refName << "(id=" << refImageId << ") pixel=(" << debugX << "," << debugY << ")\n";
            info << "  ref texture size=" << referenceImage.m_texture.m_width << "x" << referenceImage.m_texture.m_height << "\n";
            info << "  intrinsic size=" << referenceImage.m_intrinsic.m_width << "x" << referenceImage.m_intrinsic.m_height << "\n";
            info << "  fx=" << referenceImage.m_intrinsic.m_fx << " fy=" << referenceImage.m_intrinsic.m_fy << " cx=" << referenceImage.m_intrinsic.m_cx << " cy=" << referenceImage.m_intrinsic.m_cy << "\n";
            info << "  depthRange near=" << depthRange.m_nearDepth << " far=" << depthRange.m_farDepth << " fromSparsePoints=" << (depthRange.m_fromSparsePoints ? "true" : "false") << "\n";
            info << "  selected source image ids:";
            for (const auto &s : sourceImages) info << " " << s.m_camera.m_imageId;
            info << "\n";
            UIUtility::Print(info.str());

            // If configured, attempt to load COLMAP observations for anchor diagnostics (debug-only)
            ColmapPointsDebugReadResult colmapDebugPoints;
            bool haveColmapAnchor = false;
            int64_t anchorPointId = -1;
            Float3 anchorWorldPos{ 0.0f, 0.0f, 0.0f };
            double anchorColmapError = 0.0;
            float anchorObservedX = 0.0f, anchorObservedY = 0.0f;
            float anchorObservedDistance = 0.0f;
            size_t anchorTrackLength = 0;
            float anchorCameraZ = 0.0f;
            int projMatches = 0;
            double reprojMean = 0.0;
            double reprojMax = 0.0;

            // Track best and second-best costs for the debug pixel across planes (declared early so anchor summary can reference them)
            const float s_infty_f = std::numeric_limits<float>::infinity();
            float bestCost = s_infty_f;
            float secondBestCost = s_infty_f;
            float bestDepth = 0.0f;

            // Local plane-sweep parameters used by anchor diagnostics (available before the plane loop)
            // Note: these locals are re-declared later at the main plane loop; keep these only for early anchor diagnostics
            // plane-sweep parameters (declared here for use in the main loop)
            const int patchRadius = std::max(0, setup.m_planeSweepPatchRadius);
            const int minValidSourceViews = std::max(1, std::min(static_cast<int>(sourceImages.size()), setup.m_planeSweepMinValidSourceViews));
            const float alphaThreshold = std::clamp(setup.m_planeSweepAlphaThreshold, 0.0f, 1.0f);
            const float minimumOpaquePatchRatio = std::clamp(setup.m_planeSweepMinimumOpaquePatchRatio, 0.0f, 1.0f);

            // prepare anchor CSV handle (opened only if needed)
            std::ofstream anchorCsv;
            bool anchorCsvOpened = false;
            float anchorColmapCost = 0.0f;
            bool anchorColmapHasCost = false;

            if (setup.m_planeSweepDebugColmapAnchorEnabled)
            {
                colmapDebugPoints = MultiViewUtility::LoadColmapPointsWithObservations(setup);
                const auto it = colmapDebugPoints.m_imageObservations.find(refImageId);
                if (it == colmapDebugPoints.m_imageObservations.end() || it->second.empty())
                {
                    UIUtility::Print("PlaneSweepDebug COLMAP anchor: no COLMAP 2D observations found for reference image " + std::to_string(refImageId) + "\n");
                }

            // After plane sweep, write anchor summary row if requested
            if (haveColmapAnchor && anchorCsvOpened)
            {
                // abs depth difference
                double absDepthDiff = std::numeric_limits<double>::infinity();
                if (bestDepth > 0.0f && anchorCameraZ > 0.0f)
                {
                    absDepthDiff = std::fabs(static_cast<double>(bestDepth) - static_cast<double>(anchorCameraZ));
                }

                // write one summary row containing anchor info and plane-sweep best comparisons
                anchorCsv << anchorPointId << "," << anchorWorldPos.x << "," << anchorWorldPos.y << "," << anchorWorldPos.z << "," << anchorColmapError << "," << anchorObservedX << "," << anchorObservedY << "," << anchorObservedDistance << "," << anchorTrackLength << "," << anchorCameraZ << ",";
                if (bestDepth > 0.0f)
                {
                    anchorCsv << bestDepth << "," << bestCost << "," << secondBestCost << ",";
                }
                else
                {
                    anchorCsv << ",,,";
                }

                if (anchorColmapHasCost)
                {
                    anchorCsv << anchorColmapCost << ",";
                }
                else
                {
                    anchorCsv << ",";
                }

                if (std::isfinite(absDepthDiff))
                {
                    anchorCsv << absDepthDiff << "\n";
                }
                else
                {
                    anchorCsv << "\n";
                }

                anchorCsv.close();

                // Print concise summary to console using available helpers
                {
                    std::ostringstream s;
                    s << "PlaneSweepDebug COLMAP anchor summary: pointId=" << anchorPointId << " colmapXYZ=(" << anchorWorldPos.x << "," << anchorWorldPos.y << "," << anchorWorldPos.z << ") colmapErr=" << static_cast<float>(anchorColmapError) << "\n";
                    UIUtility::Print(s.str());
                }
                {
                    std::ostringstream s;
                    s << "  requestedPixel=(" << debugX << "," << debugY << ") nearestColmapPixel=(" << static_cast<int>(std::lround(anchorObservedX)) << "," << static_cast<int>(std::lround(anchorObservedY)) << ") distance=" << anchorObservedDistance << " trackLen=" << anchorTrackLength << "\n";
                    UIUtility::Print(s.str());
                }
                {
                    std::ostringstream s;
                    s << "  colmapCameraZ=" << anchorCameraZ << " planeSweepBestDepth=" << bestDepth << " bestCost=" << bestCost << " secondBestCost=" << secondBestCost << " absDepthDiff=";
                    if (std::isfinite(absDepthDiff)) s << static_cast<float>(absDepthDiff); else s << "n/a";
                    s << "\n";
                    UIUtility::Print(s.str());
                }
                {
                    std::ostringstream s;
                    s << "  projectionConsistency: obsCount=" << projMatches << " meanPx=" << static_cast<float>(reprojMean) << " maxPx=" << static_cast<float>(reprojMax) << "\n";
                    UIUtility::Print(s.str());
                }
            }
                else
                {
                    const int maxDist = setup.m_planeSweepDebugColmapAnchorMaxPixelDistance;
                    double bestDist = std::numeric_limits<double>::infinity();
                    ColmapPointObservation bestObs;
                    bool found = false;
                    for (const ColmapPointObservation &obs : it->second)
                    {
                        const double dx = static_cast<double>(obs.m_x) - static_cast<double>(debugX);
                        const double dy = static_cast<double>(obs.m_y) - static_cast<double>(debugY);
                        const double d = std::sqrt(dx * dx + dy * dy);
                        if (d < bestDist && d <= static_cast<double>(maxDist))
                        {
                            bestDist = d;
                            bestObs = obs;
                            found = true;
                        }
                    }

                    if (!found)
                    {
                        UIUtility::Print("PlaneSweepDebug COLMAP anchor: no COLMAP point within max pixel distance " + std::to_string(maxDist) + " of requested debug pixel\n");
                    }
                    else
                    {
                        // find point3D record
                        const auto pit = colmapDebugPoints.m_pointsById.find(static_cast<int64_t>(bestObs.m_point2dIdx));
                        // Note: bestObs stores point2d index but not point id; images.txt stores triples x y POINT3D_ID in obsTokens - we stored pid into observation by mapping token third value to m_point2dIdx? Actually m_point2dIdx stores the 2D index; we need to recover point id differently.
                        // The observation stored in m_imageObservations was created from the triple, but we didn't store POINT3D_ID in the observation struct. Recover point id from mapping by searching points that list this observation (slow but acceptable for debug).
                        int64_t matchedPid = -1;
                        for (const auto &pair : colmapDebugPoints.m_pointsById)
                        {
                            for (const ColmapPointObservation &po : pair.second.m_observations)
                            {
                                if (po.m_imageId == bestObs.m_imageId && po.m_point2dIdx == bestObs.m_point2dIdx)
                                {
                                    matchedPid = pair.first;
                                    break;
                                }
                            }
                            if (matchedPid != -1) break;
                        }

                        if (matchedPid == -1)
                        {
                            UIUtility::Print("PlaneSweepDebug COLMAP anchor: matched observation but could not find corresponding 3D point record\n");
                        }
                        else
                        {
                            const ColmapPoint3DDebug &pt = colmapDebugPoints.m_pointsById[matchedPid];
                            haveColmapAnchor = true;
                            anchorPointId = matchedPid;
                            anchorWorldPos = pt.m_position;
                            anchorColmapError = pt.m_error;
                            anchorObservedX = bestObs.m_x;
                            anchorObservedY = bestObs.m_y;
                            anchorObservedDistance = static_cast<float>(bestDist);
                            anchorTrackLength = pt.m_observations.size();

                            // compute camera-space Z in reference camera
                            float projX = 0.0f, projY = 0.0f, projDepth = 0.0f;
                            if (MultiViewUtility::MultiViewPlaneSweepUtility::ProjectPoint(referenceImage.m_camera, referenceImage.m_intrinsic, anchorWorldPos, projX, projY, projDepth))
                            {
                                anchorCameraZ = projDepth;
                                {
                                    std::ostringstream s;
                                    s << "PlaneSweepDebug COLMAP anchor POINT3D_ID=" << anchorPointId << " XYZ=(" << anchorWorldPos.x << "," << anchorWorldPos.y << "," << anchorWorldPos.z << ") error=" << static_cast<float>(anchorColmapError) << " refObserved=(" << anchorObservedX << "," << anchorObservedY << ") distance=" << anchorObservedDistance << " trackLen=" << anchorTrackLength << "\n";
                                    UIUtility::Print(s.str());
                                }
                                {
                                    std::ostringstream s;
                                    s << "PlaneSweepDebug COLMAP anchor camera-Z depth = " << anchorCameraZ << "\n";
                                    UIUtility::Print(s.str());
                                }
                            }
                            else
                            {
                                UIUtility::Print("PlaneSweepDebug COLMAP anchor: failed to project COLMAP XYZ into reference camera\n");
                            }

                            // Projection consistency test across source images
                            projMatches = 0;
                            reprojMean = 0.0;
                            reprojMax = 0.0;
                            const std::filesystem::path anchorCsvPath = outputFolder / ("colmap_anchor_ref" + std::to_string(refImageId) + "_x" + std::to_string(debugX) + "_y" + std::to_string(debugY) + ".csv");
                            anchorCsv.open(anchorCsvPath.string());
                            if (anchorCsv.is_open())
                            {
                                anchorCsvOpened = true;
                                anchorCsv << "POINT3D_ID,worldX,worldY,worldZ,colmapError,refObservedX,refObservedY,refDist,trackLength,colmapCameraZ,planeSweepBestDepth,planeSweepBestCost,planeSweepSecondBestCost,costAtColmapDepth,absDepthDiff" << "\n";
                            }

                            // For each source image, check whether COLMAP has an observation and compare projected position
                            for (const PlaneSweepImage &src : sourceImages)
                            {
                                const uint32_t sid = src.m_camera.m_imageId;
                                // find observation in point observations
                                bool foundObs = false;
                                float colmapX = 0.0f, colmapY = 0.0f;
                                for (const ColmapPointObservation &o : pt.m_observations)
                                {
                                    if (o.m_imageId == sid)
                                    {
                                        foundObs = true;
                                        colmapX = o.m_x;
                                        colmapY = o.m_y;
                                        break;
                                    }
                                }

                                if (foundObs)
                                {
                                    float ourX = 0.0f, ourY = 0.0f, ourDepth = 0.0f;
                                    bool projOk = MultiViewUtility::MultiViewPlaneSweepUtility::ProjectPoint(src.m_camera, src.m_intrinsic, anchorWorldPos, ourX, ourY, ourDepth);
                                    const bool inside = projOk && ourX >= 0.0f && ourY >= 0.0f && ourX < static_cast<float>(src.m_texture.m_width) && ourY < static_cast<float>(src.m_texture.m_height);
                                    double errPx = std::numeric_limits<double>::infinity();
                                    if (projOk)
                                    {
                                        const double dx = static_cast<double>(ourX) - static_cast<double>(colmapX);
                                        const double dy = static_cast<double>(ourY) - static_cast<double>(colmapY);
                                        errPx = std::sqrt(dx * dx + dy * dy);
                                        ++projMatches;
                                        reprojMean += errPx;
                                        reprojMax = std::max(reprojMax, errPx);
                                    }

                                    if (anchorCsv.is_open())
                                    {
                                        anchorCsv << sid << "," << colmapX << "," << colmapY << "," << (projOk ? std::to_string(ourX) : std::string()) << "," << (projOk ? std::to_string(ourY) : std::string()) << "," << (projOk ? std::to_string(errPx) : std::string()) << "," << (projOk ? (inside ? "inside" : "outside") : "projFailed") << "\n";
                                    }
                                }
                            }

                            if (projMatches > 0)
                            {
                                reprojMean /= static_cast<double>(projMatches);
                            }
                            {
                                std::ostringstream s;
                                s << "PlaneSweepDebug COLMAP projection consistency: obsCount=" << projMatches << " meanPx=" << static_cast<float>(reprojMean) << " maxPx=" << static_cast<float>(reprojMax) << "\n";
                                UIUtility::Print(s.str());
                            }

                            // Evaluate matching cost at exact COLMAP depth (reference camera Z)
                            if (anchorCameraZ > 0.0f)
                            {
                                float colmapCost = 0.0f;
                                bool hasCost = MultiViewUtility::MultiViewPlaneSweepUtility::ComputePlaneCost(referenceImage, sourceImages, static_cast<float>(debugX), static_cast<float>(debugY), anchorCameraZ, patchRadius, std::max(1, std::min(static_cast<int>(sourceImages.size()), setup.m_planeSweepMinValidSourceViews)), alphaThreshold, minimumOpaquePatchRatio, colmapCost);
                                {
                                    std::ostringstream s;
                                    s << "PlaneSweepDebug COLMAP depth cost: depth=" << anchorCameraZ << " cost=" << colmapCost << " hasCost=" << (hasCost ? "true" : "false") << "\n";
                                    UIUtility::Print(s.str());
                                }
                                anchorColmapCost = colmapCost;
                                anchorColmapHasCost = hasCost;
                            }
                        }
                    }
                }
            }

            // CSV file
            const std::string csvName = "planesweep_debug_ref" + std::to_string(refImageId) + "_x" + std::to_string(debugX) + "_y" + std::to_string(debugY) + ".csv";
            const std::filesystem::path csvPath = outputFolder / csvName;
            std::ofstream csv(csvPath.string());
            // Detailed CSV for ComputePlaneCost instrumentation
            const std::string detailedName = "planesweep_debug_detailed_ref" + std::to_string(refImageId) + "_x" + std::to_string(debugX) + "_y" + std::to_string(debugY) + ".csv";
            const std::filesystem::path detailedPath = outputFolder / detailedName;
            // initialize global debug context used by ComputePlaneCost instrumentation
            g_debugContext.enabled = true;
            g_debugContext.refImageId = static_cast<int>(refImageId);
            g_debugContext.pixelX = debugX;
            g_debugContext.pixelY = debugY;
            g_debugContext.minValidSourceViews = 0;
            g_debugContext.detailedCsv.open(detailedPath.string());
            if (g_debugContext.detailedCsv.is_open())
            {
                g_debugContext.detailedCsv << "planeIndex,depth,sourceImageId,sourceName,projX,projY,projDepth,projValid,insideImage,patchSamplesConsidered,patchSamplesReferenceOpaque,patchSamplesRefRejected,patchSamplesProjSuccess,patchSamplesProjFailures,patchSamplesProjOutside,patchSamplesSourceMaskRejected,patchSamplesSourceOpaque,validSamples,requiredSamples,contributed,sourceCost,rejectionReason" << "\n";
            }
            if (!csv.is_open())
            {
                UIUtility::Print("PlaneSweepDebug: failed to open " + csvPath.string() + " for writing\n");
                return;
            }

            // Header with sanity info - prefix each metadata line with '# '
            {
                std::istringstream metaStream(info.str());
                std::string metaLine;
                while (std::getline(metaStream, metaLine))
                {
                    if (!metaLine.empty())
                    {
                        csv << "# " << metaLine << "\n";
                    }
                }
            }

            const int planeCount = std::max(2, setup.m_planeSweepPlaneCount);

            const float inverseNear = 1.0f / std::max(depthRange.m_nearDepth, 0.0001f);
            const float inverseFar = 1.0f / std::max(depthRange.m_farDepth, depthRange.m_nearDepth + 0.0001f);

            // CSV column header
            csv << "planeIndex,depth,hasCost,cost,worldX,worldY,worldZ";
            for (std::size_t si = 0; si < sourceImages.size(); ++si)
            {
                csv << ",src" << si << "_projX,src" << si << "_projY,src" << si << "_projDepth,src" << si << "_projValid,src" << si << "_insideImage";
            }
            csv << "\n";

            // Save one reference image crop around the configured debug pixel (once)
            const int refWidth = static_cast<int>(referenceImage.m_texture.m_width);
            const int refHeight = static_cast<int>(referenceImage.m_texture.m_height);
            const int debugXint = debugX;
            const int debugYint = debugY;
            if (debugXint < 0 || debugYint < 0 || debugXint >= refWidth || debugYint >= refHeight)
            {
                UIUtility::Print("PlaneSweepDebug: configured debug pixel (" + std::to_string(debugXint) + "," + std::to_string(debugYint) + ") is outside reference texture " + std::to_string(refWidth) + "x" + std::to_string(refHeight) + ". Aborting debug sweep.\n");
                return;
            }

            // create a small crop around the reference pixel
            const int cropSizeRef = 41;
            const int halfRef = cropSizeRef / 2;
            TextureData refCrop;
            refCrop.m_width = static_cast<uint32_t>(cropSizeRef);
            refCrop.m_height = static_cast<uint32_t>(cropSizeRef);
            refCrop.m_channelCount = 4U;
            refCrop.m_pixels.assign(static_cast<std::size_t>(cropSizeRef * cropSizeRef * 4), 0U);
            for (int yy = 0; yy < cropSizeRef; ++yy)
            {
                for (int xx = 0; xx < cropSizeRef; ++xx)
                {
                    const int srcX = debugXint + (xx - halfRef);
                    const int srcY = debugYint + (yy - halfRef);
                    if (srcX >= 0 && srcY >= 0 && srcX < refWidth && srcY < refHeight)
                    {
                        const std::size_t srcIndex = ((static_cast<std::size_t>(srcY) * referenceImage.m_texture.m_width + static_cast<std::size_t>(srcX)) * 4U);
                        const std::size_t dstIndex = ((static_cast<std::size_t>(yy) * refCrop.m_width + static_cast<std::size_t>(xx)) * 4U);
                        for (int c = 0; c < 4; ++c)
                        {
                            refCrop.m_pixels[dstIndex + c] = referenceImage.m_texture.m_pixels[srcIndex + c];
                        }
                    }
                }
            }
            // draw red crosshair at center
            const uint8_t r = 255U;
            const uint8_t g = 0U;
            const uint8_t b = 0U;
            const uint8_t a = 255U;
            const int cx = halfRef;
            const int cy = halfRef;
            for (int dx = -1; dx <= 1; ++dx)
            {
                const int x = cx + dx;
                if (x >= 0 && x < cropSizeRef)
                {
                    const std::size_t idx = ((static_cast<std::size_t>(cy) * refCrop.m_width + static_cast<std::size_t>(x)) * 4U);
                    refCrop.m_pixels[idx + 0U] = r;
                    refCrop.m_pixels[idx + 1U] = g;
                    refCrop.m_pixels[idx + 2U] = b;
                    refCrop.m_pixels[idx + 3U] = a;
                }
            }
            for (int dy = -1; dy <= 1; ++dy)
            {
                const int y = cy + dy;
                if (y >= 0 && y < cropSizeRef)
                {
                    const std::size_t idx = ((static_cast<std::size_t>(y) * refCrop.m_width + static_cast<std::size_t>(cx)) * 4U);
                    refCrop.m_pixels[idx + 0U] = r;
                    refCrop.m_pixels[idx + 1U] = g;
                    refCrop.m_pixels[idx + 2U] = b;
                    refCrop.m_pixels[idx + 3U] = a;
                }
            }
            const std::filesystem::path refCropPath = outputFolder / ("ref" + std::to_string(refImageId) + "_debug_pixel_crop.png");
            SaveTexture(refCropPath.string(), refCrop, TextureFileType::Png);

            for (int planeIndex = 0; planeIndex < planeCount; ++planeIndex)
            {
                const float t = planeCount <= 1 ? 0.0f : static_cast<float>(planeIndex) / static_cast<float>(planeCount - 1);
                const float inverseDepth = (inverseNear * (1.0f - t)) + (inverseFar * t);
                const float depth = 1.0f / std::max(inverseDepth, 0.000001f);

                float cost = 0.0f;
                // set current plane in global debug context used by ComputePlaneCost
                if (g_debugContext.enabled)
                {
                    g_debugContext.currentPlaneIndex = planeIndex;
                    g_debugContext.currentDepth = depth;
                    g_debugContext.minValidSourceViews = minValidSourceViews;
                }

                bool hasCost = ComputePlaneCost(referenceImage, sourceImages, static_cast<float>(debugX), static_cast<float>(debugY), depth, patchRadius, minValidSourceViews, alphaThreshold, minimumOpaquePatchRatio, cost);

                if (hasCost && std::isfinite(cost))
                {
                    if (cost < bestCost)
                    {
                        secondBestCost = bestCost;
                        bestCost = cost;
                        bestDepth = depth;
                    }
                    else if (cost < secondBestCost)
                    {
                        secondBestCost = cost;
                    }
                }

                // Print concise console summary per plane using global context
                if (g_debugContext.enabled)
                {
                    const int validSources = g_debugContext.lastPlaneValidSources;
                    const int required = g_debugContext.minValidSourceViews;
                    const bool planeHasCost = g_debugContext.lastPlaneHasCost;
                    std::ostringstream ps;
                    ps << "[PlaneSweepDebug] plane=" << planeIndex << " depth=" << depth << " validSources=" << validSources << " required=" << required << " hasCost=" << (planeHasCost ? "true" : "false");
                    UIUtility::Print(ps.str() + "\n");
                }

                const Float3 worldPoint = UnprojectPixel(referenceImage.m_camera, referenceImage.m_intrinsic, static_cast<float>(debugX), static_cast<float>(debugY), depth);

                csv << planeIndex << "," << depth << "," << (hasCost ? "1" : "0") << "," << cost << "," << worldPoint.x << "," << worldPoint.y << "," << worldPoint.z;

                // For each source, project and record
                for (const auto &src : sourceImages)
                {
                    float px = 0.0f;
                    float py = 0.0f;
                    float pdepth = 0.0f;
                    const bool projValid = ProjectPoint(src.m_camera, src.m_intrinsic, worldPoint, px, py, pdepth);
                    const bool insideImage = projValid && px >= 0.0f && py >= 0.0f && px < static_cast<float>(src.m_texture.m_width) && py < static_cast<float>(src.m_texture.m_height);
                    csv << "," << px << "," << py << "," << pdepth << "," << (projValid ? "1" : "0") << "," << (insideImage ? "1" : "0");

                    // Optionally save a small crop around the projected coordinate per-plane/source
                    if (setup.m_planeSweepDebugSaveEveryPlane && projValid && src.m_texture.m_width > 0 && src.m_texture.m_height > 0)
                    {
                        const int cropSize = 41;
                        const int half = cropSize / 2;
                        const int centerX = static_cast<int>(std::lround(px));
                        const int centerY = static_cast<int>(std::lround(py));
                        TextureData crop;
                        crop.m_width = static_cast<uint32_t>(cropSize);
                        crop.m_height = static_cast<uint32_t>(cropSize);
                        crop.m_channelCount = 4U;
                        crop.m_pixels.assign(static_cast<std::size_t>(cropSize * cropSize * 4), 0U);

                        for (int yy = 0; yy < cropSize; ++yy)
                        {
                            for (int xx = 0; xx < cropSize; ++xx)
                            {
                                const int srcX = centerX + (xx - half);
                                const int srcY = centerY + (yy - half);
                                if (srcX >= 0 && srcY >= 0 && srcX < static_cast<int>(src.m_texture.m_width) && srcY < static_cast<int>(src.m_texture.m_height))
                                {
                                    const std::size_t srcIndex = ((static_cast<std::size_t>(srcY) * src.m_texture.m_width + static_cast<std::size_t>(srcX)) * 4U);
                                    const std::size_t dstIndex = ((static_cast<std::size_t>(yy) * crop.m_width + static_cast<std::size_t>(xx)) * 4U);
                                    for (int c = 0; c < 4; ++c)
                                    {
                                        crop.m_pixels[dstIndex + c] = src.m_texture.m_pixels[srcIndex + c];
                                    }
                                }
                            }
                        }

                        // Draw crosshair at center (red) using distinct local names to avoid hiding outer vars
                        const uint8_t rr = 255U;
                        const uint8_t gg = 0U;
                        const uint8_t bb = 0U;
                        const uint8_t aa = 255U;
                        const int local_cx = half;
                        const int local_cy = half;
                        for (int dx = -1; dx <= 1; ++dx)
                        {
                            const int x = local_cx + dx;
                            if (x >= 0 && x < cropSize)
                            {
                                const std::size_t idx = ((static_cast<std::size_t>(local_cy) * crop.m_width + static_cast<std::size_t>(x)) * 4U);
                                crop.m_pixels[idx + 0U] = rr;
                                crop.m_pixels[idx + 1U] = gg;
                                crop.m_pixels[idx + 2U] = bb;
                                crop.m_pixels[idx + 3U] = aa;
                            }
                        }
                        for (int dy = -1; dy <= 1; ++dy)
                        {
                            const int y = local_cy + dy;
                            if (y >= 0 && y < cropSize)
                            {
                                const std::size_t idx = ((static_cast<std::size_t>(y) * crop.m_width + static_cast<std::size_t>(local_cx)) * 4U);
                                crop.m_pixels[idx + 0U] = rr;
                                crop.m_pixels[idx + 1U] = gg;
                                crop.m_pixels[idx + 2U] = bb;
                                crop.m_pixels[idx + 3U] = aa;
                            }
                        }

                        const std::string planeImgName = "ref" + std::to_string(refImageId) + "_plane" + std::to_string(planeIndex) + "_src" + std::to_string(src.m_camera.m_imageId) + ".png";
                        const std::filesystem::path planeImgPath = outputFolder / planeImgName;
                        SaveTexture(planeImgPath.string(), crop, TextureFileType::Png);
                    }
                }

                csv << "\n";
            }

            UIUtility::Print("PlaneSweepDebug: wrote diagnostics to " + csvPath.string() + "\n");
        }

        //Computes grayscale patch texture variation.
        bool MultiViewUtility::MultiViewPlaneSweepUtility::ComputeTextureStandardDeviation(
            const PlaneSweepImage &referenceImage,
            float x,
            float y,
            int patchRadius,
            float alphaThreshold,
            float &standardDeviation
        )
        {
            float luminanceSum = 0.0f;
            float luminanceSquaredSum = 0.0f;
            int sampleCount = 0;

            for (int patchY = -patchRadius; patchY <= patchRadius; ++patchY)
            {
                for (int patchX = -patchRadius; patchX <= patchRadius; ++patchX)
                {
                    Float4 color;
                    if (!SamplePlaneSweepPixel(referenceImage, x + static_cast<float>(patchX), y + static_cast<float>(patchY), color) || color.w <= alphaThreshold)
                    {
                        continue;
                    }

                    const float luminance = (color.x * 0.299f) + (color.y * 0.587f) + (color.z * 0.114f);
                    luminanceSum += luminance;
                    luminanceSquaredSum += luminance * luminance;
                    ++sampleCount;
                }
            }

            if (sampleCount == 0)
            {
                return false;
            }

            const float mean = luminanceSum / static_cast<float>(sampleCount);
            standardDeviation = std::sqrt(std::max(0.0f, (luminanceSquaredSum / static_cast<float>(sampleCount)) - (mean * mean)));
            return std::isfinite(standardDeviation);
        }

        //Builds plane sweep points for one reference image.
        void MultiViewUtility::MultiViewPlaneSweepUtility::BuildReferencePoints(
            const ColmapSetup &setup,
            const PlaneSweepImage &referenceImage,
            const std::vector<PlaneSweepImage> &sourceImages,
            const PlaneSweepDepthRange &depthRange,
            std::size_t depthMapIndex,
            PlaneSweepDepthMap &depthMap,
            std::vector<PlaneSweepPointCandidate> &pointCandidates,
            ColmapPlaneSweepResult &result,
            PlaneSweepExperimentMetrics *metricsPtr
        )
        {
            const uint32_t scaledWidth = std::max(1U, static_cast<uint32_t>(std::lround(static_cast<float>(referenceImage.m_texture.m_width) * setup.m_planeSweepImageScale)));
            const uint32_t scaledHeight = std::max(1U, static_cast<uint32_t>(std::lround(static_cast<float>(referenceImage.m_texture.m_height) * setup.m_planeSweepImageScale)));
            const int pointStep = std::max(1, setup.m_planeSweepPointStep);
            const int planeCount = std::max(2, setup.m_planeSweepPlaneCount);
            const int patchRadius = std::max(0, setup.m_planeSweepPatchRadius);
            const int minValidSourceViews = std::max(1, std::min(static_cast<int>(sourceImages.size()), setup.m_planeSweepMinValidSourceViews));
            const float alphaThreshold = std::clamp(setup.m_planeSweepAlphaThreshold, 0.0f, 1.0f);
            const float minimumOpaquePatchRatio = std::clamp(setup.m_planeSweepMinimumOpaquePatchRatio, 0.0f, 1.0f);
            const float confidenceMinimumDifference = std::max(0.0f, setup.m_planeSweepConfidenceMinimumDifference);
            const float textureMinimumStdDev = std::max(0.0f, setup.m_planeSweepTextureMinimumStdDev);

            depthMap = {};
            depthMap.m_camera = referenceImage.m_camera;
            depthMap.m_intrinsic = referenceImage.m_intrinsic;
            depthMap.m_imageWidth = referenceImage.m_texture.m_width;
            depthMap.m_imageHeight = referenceImage.m_texture.m_height;
            depthMap.m_width = scaledWidth;
            depthMap.m_height = scaledHeight;
            depthMap.m_depthValues.resize(static_cast<std::size_t>(scaledWidth) * scaledHeight, 0.0f);
            const float inverseNear = 1.0f / std::max(depthRange.m_nearDepth, 0.0001f);
            const float inverseFar = 1.0f / std::max(depthRange.m_farDepth, depthRange.m_nearDepth + 0.0001f);

            for (uint32_t scaledY = 0U; scaledY < scaledHeight; scaledY += static_cast<uint32_t>(pointStep))
            {
                for (uint32_t scaledX = 0U; scaledX < scaledWidth; scaledX += static_cast<uint32_t>(pointStep))
                {
                    const float referenceX = static_cast<float>((static_cast<std::size_t>(scaledX) * referenceImage.m_texture.m_width) / scaledWidth);
                    const float referenceY = static_cast<float>((static_cast<std::size_t>(scaledY) * referenceImage.m_texture.m_height) / scaledHeight);
                    Float4 referenceColor;
                    if (!SamplePlaneSweepPixel(referenceImage, referenceX, referenceY, referenceColor) || referenceColor.w <= alphaThreshold)
                    {
                        continue;
                    }

                    if (setup.m_planeSweepTextureRejectionEnabled)
                    {
                        float textureStandardDeviation = 0.0f;
                        if (!ComputeTextureStandardDeviation(referenceImage, referenceX, referenceY, patchRadius, alphaThreshold, textureStandardDeviation) ||
                            textureStandardDeviation < textureMinimumStdDev)
                        {
                            ++result.m_pointsRejectedTexture;
                            continue;
                        }
                    }

                    float bestCost = std::numeric_limits<float>::max();
                    float secondBestCost = std::numeric_limits<float>::max();
                    float bestDepth = 0.0f;

                    for (int planeIndex = 0; planeIndex < planeCount; ++planeIndex)
                    {
                        const float t = planeCount <= 1 ? 0.0f : static_cast<float>(planeIndex) / static_cast<float>(planeCount - 1);
                        const float inverseDepth = (inverseNear * (1.0f - t)) + (inverseFar * t);
                        const float depth = 1.0f / std::max(inverseDepth, 0.000001f);
                        float cost = 0.0f;
                        bool hasCost = false;
                        if (metricsPtr)
                        {
                            const auto pcStart = std::chrono::steady_clock::now();
                            hasCost = ComputePlaneCost(
                                referenceImage,
                                sourceImages,
                                referenceX,
                                referenceY,
                                depth,
                                patchRadius,
                                minValidSourceViews,
                                alphaThreshold,
                                minimumOpaquePatchRatio,
                                cost
                            );
                            const auto pcEnd = std::chrono::steady_clock::now();
                            metricsPtr->m_computePlaneCostMs += std::chrono::duration<double, std::milli>(pcEnd - pcStart).count();
                        }
                        else
                        {
                            hasCost = ComputePlaneCost(
                                referenceImage,
                                sourceImages,
                                referenceX,
                                referenceY,
                                depth,
                                patchRadius,
                                minValidSourceViews,
                                alphaThreshold,
                                minimumOpaquePatchRatio,
                                cost
                            );
                        }
                        if (hasCost && cost < bestCost)
                        {
                            secondBestCost = bestCost;
                            bestCost = cost;
                            bestDepth = depth;
                        }
                        else if (hasCost && std::isfinite(cost) && cost < secondBestCost)
                        {
                            secondBestCost = cost;
                        }
                    }

                    if (bestDepth > 0.0f && bestCost <= setup.m_planeSweepCostThreshold)
                    {
                        if (setup.m_planeSweepConfidenceEnabled && (!std::isfinite(secondBestCost) || (secondBestCost - bestCost) < confidenceMinimumDifference))
                        {
                            ++result.m_pointsRejectedConfidence;
                            continue;
                        }

                        const std::size_t depthIndex = static_cast<std::size_t>(scaledY) * scaledWidth + scaledX;
                        depthMap.m_depthValues[depthIndex] = bestDepth;

                        const Float3 position = UnprojectPixel(referenceImage.m_camera, referenceImage.m_intrinsic, referenceX, referenceY, bestDepth);
                        pointCandidates.push_back(PlaneSweepPointCandidate{ depthMapIndex, position, referenceColor });
                    }
                }
            }

            if (SaveDepthOutput(setup, referenceImage.m_camera, depthMap.m_depthValues, scaledWidth, scaledHeight))
            {
                ++result.m_depthFilesSaved;
            }

            ++result.m_referenceImagesProcessed;
        }

        //Compares two plane sweep voxel keys.
        bool MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepVoxelKey::operator == (const PlaneSweepVoxelKey &other) const
        {
            return m_x == other.m_x && m_y == other.m_y && m_z == other.m_z;
        }

        //Hashes one plane sweep spatial key.
        std::size_t MultiViewUtility::MultiViewPlaneSweepUtility::PlaneSweepVoxelKeyHash::operator () (const PlaneSweepVoxelKey &key) const
        {
            const std::size_t xHash = std::hash<int64_t>{}(key.m_x);
            const std::size_t yHash = std::hash<int64_t>{}(key.m_y);
            const std::size_t zHash = std::hash<int64_t>{}(key.m_z);
            return xHash ^ (yHash << 1U) ^ (zHash << 2U);
        }

        //Rejects points unsupported by other reconstructed depth maps.
        void MultiViewUtility::MultiViewPlaneSweepUtility::ApplyCrossViewConsistency(
            const ColmapSetup &setup,
            const std::vector<PlaneSweepDepthMap> &depthMaps,
            std::vector<PlaneSweepPointCandidate> &pointCandidates,
            ColmapPlaneSweepResult &result
        )
        {
            // Note: metrics timing is collected by the caller for cross-view step if requested.
            if (!setup.m_planeSweepCrossViewConsistencyEnabled || depthMaps.size() < 2U || pointCandidates.empty())
            {
                return;
            }

            const std::size_t comparisonCount = static_cast<std::size_t>(std::max(1, setup.m_planeSweepCrossViewCount));
            const int minimumViews = std::max(1, setup.m_planeSweepCrossViewMinimumViews);
            const float depthTolerance = std::max(0.0f, setup.m_planeSweepCrossViewRelativeDepthTolerance);
            const int sampleRadius = std::max(1, setup.m_planeSweepPointStep);
            std::vector<PlaneSweepPointCandidate> consistentCandidates;
            consistentCandidates.reserve(pointCandidates.size());

            for (const PlaneSweepPointCandidate &pointCandidate : pointCandidates)
            {
                if (pointCandidate.m_depthMapIndex >= depthMaps.size())
                {
                    ++result.m_pointsRejectedCrossView;
                    continue;
                }

                const PlaneSweepDepthMap &referenceDepthMap = depthMaps[pointCandidate.m_depthMapIndex];
                std::vector<PlaneSweepSourceCandidate> comparisonCandidates;
                comparisonCandidates.reserve(depthMaps.size() - 1U);
                for (std::size_t depthMapIndex = 0U; depthMapIndex < depthMaps.size(); ++depthMapIndex)
                {
                    if (depthMapIndex == pointCandidate.m_depthMapIndex)
                    {
                        continue;
                    }

                    const PlaneSweepDepthMap &comparisonDepthMap = depthMaps[depthMapIndex];
                    const Float3 offset = comparisonDepthMap.m_camera.m_position - referenceDepthMap.m_camera.m_position;
                    const float distance = std::sqrt((offset.x * offset.x) + (offset.y * offset.y) + (offset.z * offset.z));
                    const float viewAlignment = std::max(0.0f, dot(referenceDepthMap.m_camera.m_forward, comparisonDepthMap.m_camera.m_forward));
                    comparisonCandidates.push_back(PlaneSweepSourceCandidate{ depthMapIndex, 0U, distance / (0.25f + viewAlignment) });
                }

                std::sort(comparisonCandidates.begin(), comparisonCandidates.end(), [](const PlaneSweepSourceCandidate &a, const PlaneSweepSourceCandidate &b)
                {
                    return a.m_score < b.m_score;
                });

                int validViews = 0;
                int agreeingViews = 0;
                const std::size_t candidateCount = std::min(comparisonCount, comparisonCandidates.size());
                for (std::size_t candidateIndex = 0U; candidateIndex < candidateCount; ++candidateIndex)
                {
                    const PlaneSweepDepthMap &comparisonDepthMap = depthMaps[comparisonCandidates[candidateIndex].m_cameraIndex];
                    float x = 0.0f;
                    float y = 0.0f;
                    float projectedDepth = 0.0f;
                    if (!ProjectPoint(comparisonDepthMap.m_camera, comparisonDepthMap.m_intrinsic, pointCandidate.m_position, x, y, projectedDepth) ||
                        x < 0.0f || y < 0.0f || x >= static_cast<float>(comparisonDepthMap.m_imageWidth) || y >= static_cast<float>(comparisonDepthMap.m_imageHeight))
                    {
                        continue;
                    }

                    const int centerX = static_cast<int>(std::lround((x / static_cast<float>(std::max(1U, comparisonDepthMap.m_imageWidth))) * comparisonDepthMap.m_width));
                    const int centerY = static_cast<int>(std::lround((y / static_cast<float>(std::max(1U, comparisonDepthMap.m_imageHeight))) * comparisonDepthMap.m_height));
                    float smallestRelativeDifference = std::numeric_limits<float>::max();
                    for (int sampleY = centerY - sampleRadius; sampleY <= centerY + sampleRadius; ++sampleY)
                    {
                        for (int sampleX = centerX - sampleRadius; sampleX <= centerX + sampleRadius; ++sampleX)
                        {
                            if (sampleX < 0 || sampleY < 0 || sampleX >= static_cast<int>(comparisonDepthMap.m_width) || sampleY >= static_cast<int>(comparisonDepthMap.m_height))
                            {
                                continue;
                            }

                            const float comparisonDepth = comparisonDepthMap.m_depthValues[(static_cast<std::size_t>(sampleY) * comparisonDepthMap.m_width) + static_cast<uint32_t>(sampleX)];
                            if (comparisonDepth <= 0.0f)
                            {
                                continue;
                            }

                            const float relativeDifference = std::fabs(projectedDepth - comparisonDepth) / std::max(projectedDepth, comparisonDepth);
                            smallestRelativeDifference = std::min(smallestRelativeDifference, relativeDifference);
                        }
                    }

                    if (std::isfinite(smallestRelativeDifference))
                    {
                        ++validViews;
                        if (smallestRelativeDifference <= depthTolerance)
                        {
                            ++agreeingViews;
                        }
                    }
                }

                if (validViews >= minimumViews && agreeingViews >= minimumViews)
                {
                    consistentCandidates.push_back(pointCandidate);
                }
                else
                {
                    ++result.m_pointsRejectedCrossView;
                }
            }

            pointCandidates = std::move(consistentCandidates);
        }

        //Fuses and filters plane sweep point candidates.
        void MultiViewUtility::MultiViewPlaneSweepUtility::BuildFilteredPoints(
            const ColmapSetup &setup,
            const std::vector<PlaneSweepPointCandidate> &pointCandidates,
            ColmapPlaneSweepResult &result
        )
        {
            result.m_pointsBeforeFiltering = pointCandidates.size();
            std::vector<SceneObjectPointCloudPoint> points;

            if (setup.m_planeSweepPointFusionEnabled)
            {
                const float voxelSize = std::max(0.000001f, setup.m_planeSweepFusionVoxelSize);
                const std::size_t minimumObservations = static_cast<std::size_t>(std::max(1, setup.m_planeSweepFusionMinimumObservations));
                std::unordered_map<PlaneSweepVoxelKey, PlaneSweepVoxelData, PlaneSweepVoxelKeyHash> voxels;
                voxels.reserve(pointCandidates.size());
                for (const PlaneSweepPointCandidate &pointCandidate : pointCandidates)
                {
                    const PlaneSweepVoxelKey key{
                        static_cast<int64_t>(std::floor(pointCandidate.m_position.x / voxelSize)),
                        static_cast<int64_t>(std::floor(pointCandidate.m_position.y / voxelSize)),
                        static_cast<int64_t>(std::floor(pointCandidate.m_position.z / voxelSize))
                    };
                    PlaneSweepVoxelData &voxel = voxels[key];
                    voxel.m_positionSum.x += pointCandidate.m_position.x;
                    voxel.m_positionSum.y += pointCandidate.m_position.y;
                    voxel.m_positionSum.z += pointCandidate.m_position.z;
                    voxel.m_colorSum.x += pointCandidate.m_color.x;
                    voxel.m_colorSum.y += pointCandidate.m_color.y;
                    voxel.m_colorSum.z += pointCandidate.m_color.z;
                    voxel.m_colorSum.w += pointCandidate.m_color.w;
                    ++voxel.m_observationCount;
                }

                points.reserve(voxels.size());
                for (const auto &entry : voxels)
                {
                    const PlaneSweepVoxelData &voxel = entry.second;
                    if (voxel.m_observationCount < minimumObservations)
                    {
                        continue;
                    }

                    const float inverseCount = 1.0f / static_cast<float>(voxel.m_observationCount);
                    points.push_back(SceneObjectPointCloudPoint{
                        { voxel.m_positionSum.x * inverseCount, voxel.m_positionSum.y * inverseCount, voxel.m_positionSum.z * inverseCount },
                        { voxel.m_colorSum.x * inverseCount, voxel.m_colorSum.y * inverseCount, voxel.m_colorSum.z * inverseCount, voxel.m_colorSum.w * inverseCount }
                    });
                }
            }
            else
            {
                points.reserve(pointCandidates.size());
                for (const PlaneSweepPointCandidate &pointCandidate : pointCandidates)
                {
                    points.push_back(SceneObjectPointCloudPoint{ pointCandidate.m_position, pointCandidate.m_color });
                }
            }

            result.m_pointsAfterFusion = points.size();
            if (!setup.m_planeSweepOutlierFilterEnabled || points.empty())
            {
                result.m_pointsAfterOutlierFilter = points.size();
                result.m_points = std::move(points);
                return;
            }

            const float radius = std::max(0.000001f, setup.m_planeSweepOutlierRadius);
            const float radiusSquared = radius * radius;
            const int minimumNeighbors = std::max(0, setup.m_planeSweepOutlierMinimumNeighbors);
            std::unordered_map<PlaneSweepVoxelKey, std::vector<std::size_t>, PlaneSweepVoxelKeyHash> pointCells;
            pointCells.reserve(points.size());
            for (std::size_t pointIndex = 0U; pointIndex < points.size(); ++pointIndex)
            {
                const Float3 &position = points[pointIndex].m_position;
                pointCells[PlaneSweepVoxelKey{
                    static_cast<int64_t>(std::floor(position.x / radius)),
                    static_cast<int64_t>(std::floor(position.y / radius)),
                    static_cast<int64_t>(std::floor(position.z / radius))
                }].push_back(pointIndex);
            }

            std::vector<SceneObjectPointCloudPoint> filteredPoints;
            filteredPoints.reserve(points.size());
            for (std::size_t pointIndex = 0U; pointIndex < points.size(); ++pointIndex)
            {
                const Float3 &position = points[pointIndex].m_position;
                const PlaneSweepVoxelKey centerKey{
                    static_cast<int64_t>(std::floor(position.x / radius)),
                    static_cast<int64_t>(std::floor(position.y / radius)),
                    static_cast<int64_t>(std::floor(position.z / radius))
                };
                int neighborCount = 0;
                for (int64_t zOffset = -1; zOffset <= 1; ++zOffset)
                {
                    for (int64_t yOffset = -1; yOffset <= 1; ++yOffset)
                    {
                        for (int64_t xOffset = -1; xOffset <= 1; ++xOffset)
                        {
                            const auto found = pointCells.find(PlaneSweepVoxelKey{ centerKey.m_x + xOffset, centerKey.m_y + yOffset, centerKey.m_z + zOffset });
                            if (found == pointCells.end())
                            {
                                continue;
                            }

                            for (std::size_t nearbyIndex : found->second)
                            {
                                if (nearbyIndex == pointIndex)
                                {
                                    continue;
                                }

                                const Float3 &nearbyPosition = points[nearbyIndex].m_position;
                                const float xDifference = position.x - nearbyPosition.x;
                                const float yDifference = position.y - nearbyPosition.y;
                                const float zDifference = position.z - nearbyPosition.z;
                                if ((xDifference * xDifference) + (yDifference * yDifference) + (zDifference * zDifference) <= radiusSquared)
                                {
                                    ++neighborCount;
                                }
                            }
                        }
                    }
                }

                if (neighborCount >= minimumNeighbors)
                {
                    filteredPoints.push_back(points[pointIndex]);
                }
            }

            result.m_pointsAfterOutlierFilter = filteredPoints.size();
            result.m_points = std::move(filteredPoints);
        }

        //Returns whether sparse points should define the depth range.
        bool MultiViewUtility::MultiViewPlaneSweepUtility::DepthRangeUsesSparsePoints(const std::string &depthRangeMode)
        {
            const std::string mode = StringUtility::Uppercase(StringUtility::Trim(depthRangeMode));
            return mode == "SPARSEPOINTS" || mode == "SPARSE_POINTS" || mode == "POINTS3D";
        }

        //Returns a robust percentile value.
        float MultiViewUtility::MultiViewPlaneSweepUtility::PercentileValue(std::vector<float> &values, float percentile)
        {
            if (values.empty())
            {
                return 0.0f;
            }

            std::sort(values.begin(), values.end());
            const float scaledIndex = std::clamp(percentile, 0.0f, 1.0f) * static_cast<float>(values.size() - 1U);
            return values[static_cast<std::size_t>(std::lround(scaledIndex))];
        }

        //Builds COLMAP points from plane sweep reconstruction.
        ColmapPlaneSweepResult MultiViewUtility::BuildColmapPlaneSweepPointCloud(const ColmapCameraReadResult &colmapData, const ColmapSetup &setup, PlaneSweepExperimentMetrics *metricsPtr, PlaneSweepGpu *planeSweepGpuPtr)
        {
            ColmapPlaneSweepResult result;
            if (planeSweepGpuPtr != nullptr)
            {
                UIUtility::Print("BuildColmapPlaneSweepPointCloud received PlaneSweepGpu handoff.\n");
            }
            static bool multiSourceCostTestAttempted = false;
            if (planeSweepGpuPtr != nullptr && !multiSourceCostTestAttempted)
            {
                multiSourceCostTestAttempted = true;
                using PlaneSweepImage = MultiViewPlaneSweepUtility::PlaneSweepImage;
                PlaneSweepImage referenceImage;
                referenceImage.m_camera.m_position = Float3{0.0f, 0.0f, 0.0f};
                referenceImage.m_camera.m_forward = Float3{0.0f, 0.0f, 1.0f};
                referenceImage.m_camera.m_up = Float3{0.0f, 1.0f, 0.0f};
                referenceImage.m_camera.m_right = Float3{1.0f, 0.0f, 0.0f};
                referenceImage.m_intrinsic.m_width = 1;
                referenceImage.m_intrinsic.m_height = 1;
                referenceImage.m_intrinsic.m_fx = 1.0;
                referenceImage.m_intrinsic.m_fy = 1.0;
                referenceImage.m_intrinsic.m_cx = 0.5;
                referenceImage.m_intrinsic.m_cy = 0.5;
                referenceImage.m_texture.m_width = 1;
                referenceImage.m_texture.m_height = 1;
                referenceImage.m_texture.m_channelCount = 4;
                referenceImage.m_texture.m_pixels = {100, 100, 100, 255};

                const std::array<std::array<uint8_t, 4>, 6> controlledPixels{{
                    {{160, 140, 120, 100}}, {{160, 140, 120, 110}}, {{160, 140, 100, 100}},
                    {{160, 140, 120, 100}}, {{100, 100, 100, 100}}, {{160, 140, 100, 100}}
                }};
                std::array<PlaneSweepImage, 6> scenarioImages{};
                std::vector<std::vector<uint8_t>> scenarioSourcePixels(6, std::vector<uint8_t>(8 * 4, 255));
                for (uint32_t scenario = 0; scenario < scenarioImages.size(); ++scenario)
                {
                    PlaneSweepImage &source = scenarioImages[scenario];
                    source.m_camera.m_position = Float3{1.0f, 0.0f, 0.0f};
                    source.m_camera.m_forward = Float3{0.0f, 0.0f, 1.0f};
                    source.m_camera.m_up = Float3{0.0f, 1.0f, 0.0f};
                    source.m_camera.m_right = Float3{1.0f, 0.0f, 0.0f};
                    source.m_intrinsic.m_width = 8;
                    source.m_intrinsic.m_height = 1;
                    source.m_intrinsic.m_fx = 4.0;
                    source.m_intrinsic.m_fy = 1.0;
                    source.m_intrinsic.m_cx = 4.0;
                    source.m_intrinsic.m_cy = 0.0;
                    source.m_texture.m_width = 8;
                    source.m_texture.m_height = 1;
                    source.m_texture.m_channelCount = 4;
                    source.m_texture.m_pixels.resize(8 * 4);
                    for (uint32_t x = 0; x < 8; ++x)
                    {
                        const size_t offset = static_cast<size_t>(x) * 4;
                        const uint8_t value = x < 4 ? controlledPixels[scenario][x] : 100;
                        const uint8_t alpha = scenario == 4 ? 0 : 255;
                        scenarioSourcePixels[scenario][offset] = value;
                        scenarioSourcePixels[scenario][offset + 1] = value;
                        scenarioSourcePixels[scenario][offset + 2] = value;
                        scenarioSourcePixels[scenario][offset + 3] = alpha;
                        source.m_texture.m_pixels[offset] = value;
                        source.m_texture.m_pixels[offset + 1] = value;
                        source.m_texture.m_pixels[offset + 2] = value;
                        source.m_texture.m_pixels[offset + 3] = alpha;
                    }
                }

                ColmapSetup controlledSetup = setup;
                controlledSetup.m_planeSweepImageScale = 1.0f;
                controlledSetup.m_planeSweepPointStep = 1;
                controlledSetup.m_planeSweepPlaneCount = 4;
                controlledSetup.m_planeSweepPatchRadius = 0;
                controlledSetup.m_planeSweepMinValidSourceViews = 1;
                controlledSetup.m_planeSweepAlphaThreshold = 0.5f;
                controlledSetup.m_planeSweepMinimumOpaquePatchRatio = 1.0f;
                controlledSetup.m_planeSweepTextureRejectionEnabled = false;
                controlledSetup.m_planeSweepTextureMinimumStdDev = 0.0f;
                controlledSetup.m_planeSweepSaveDepth = false;
                const std::array<float, 6> thresholds{{1.0f, 0.03f, 1.0f, 1.0f, 1.0f, 1.0f}};
                const std::array<uint32_t, 6> confidenceEnabled{{0, 0, 1, 1, 0, 0}};
                const std::array<float, 6> confidenceDifferences{{0.0f, 0.0f, 0.01f, 0.05f, 0.0f, 0.0f}};
                const std::array<uint32_t, 6> conceptualStatuses{{3, 1, 2, 3, 0, 3}};
                std::vector<float> scenarioFloats(6 * 8, 0.0f);
                std::vector<uint32_t> scenarioParameters(6 * 4, 0);
                std::vector<float> expectedPlaneDepths(6 * 4, 0.0f), expectedPlaneCosts(6 * 4, 0.0f);
                std::vector<uint32_t> expectedPlaneValidity(6 * 4, 0), expectedStatuses(6, 0);
                std::vector<float> expectedFinalDepths(6, 0.0f), expectedBestDepths(6, 0.0f), expectedBestCosts(6, std::numeric_limits<float>::max()), expectedSecondBestCosts(6, std::numeric_limits<float>::max());
                std::vector<int32_t> expectedBestPlanes(6, -1);
                const float inverseNear = 1.0f / std::max(1.0f, 0.0001f);
                const float inverseFar = 1.0f / std::max(4.0f, 1.0f + 0.0001f);
                bool geometryValid = true;
                for (uint32_t plane = 0; plane < 4; ++plane)
                {
                    const float t = static_cast<float>(plane) / 3.0f;
                    const float inverseDepth = (inverseNear * (1.0f - t)) + (inverseFar * t);
                    const float depth = 1.0f / std::max(inverseDepth, 0.000001f);
                    const Float3 point = MultiViewPlaneSweepUtility::UnprojectPixel(referenceImage.m_camera, referenceImage.m_intrinsic, 0.0f, 0.0f, depth);
                    float sourceX = 0.0f, sourceY = 0.0f, projectedDepth = 0.0f;
                    const bool projected = MultiViewPlaneSweepUtility::ProjectPoint(scenarioImages[0].m_camera, scenarioImages[0].m_intrinsic, point, sourceX, sourceY, projectedDepth);
                    if (!projected || std::fabs(sourceX - static_cast<float>(plane)) > 1e-5f || std::fabs(sourceY) > 1e-5f)
                    {
                        UIUtility::Print("Phase 2N geometry sanity failed: plane=" + std::to_string(plane) + ", depth=" + std::to_string(depth) + ", projected=(" + std::to_string(sourceX) + "," + std::to_string(sourceY) + "). GPU test skipped.\n");
                        geometryValid = false;
                    }
                    for (uint32_t scenario = 0; scenario < 6; ++scenario) expectedPlaneDepths[scenario * 4 + plane] = depth;
                }
                if (!geometryValid) return result;

                bool cpuExpectationsMatch = true;
                for (uint32_t scenario = 0; scenario < 6; ++scenario)
                {
                    controlledSetup.m_planeSweepCostThreshold = thresholds[scenario];
                    controlledSetup.m_planeSweepConfidenceEnabled = confidenceEnabled[scenario] != 0;
                    controlledSetup.m_planeSweepConfidenceMinimumDifference = confidenceDifferences[scenario];
                    std::vector<PlaneSweepImage> activeSources{scenarioImages[scenario]};
                    float bestCost = std::numeric_limits<float>::max();
                    float secondBestCost = std::numeric_limits<float>::max();
                    float bestDepth = 0.0f;
                    int32_t bestPlane = -1;
                    for (uint32_t plane = 0; plane < 4; ++plane)
                    {
                        const size_t diagnosticIndex = static_cast<size_t>(scenario) * 4 + plane;
                        const float depth = expectedPlaneDepths[diagnosticIndex];
                        float cost = 0.0f;
                        const bool hasCost = MultiViewPlaneSweepUtility::ComputePlaneCost(referenceImage, activeSources, 0.0f, 0.0f, depth, 0, 1, 0.5f, 1.0f, cost);
                        expectedPlaneValidity[diagnosticIndex] = hasCost ? 1u : 0u;
                        if (hasCost)
                        {
                            expectedPlaneCosts[diagnosticIndex] = cost;
                            if (cost < bestCost) { secondBestCost = bestCost; bestCost = cost; bestDepth = depth; bestPlane = static_cast<int32_t>(plane); }
                            else if (std::isfinite(cost) && cost < secondBestCost) secondBestCost = cost;
                        }
                        UIUtility::Print("Phase 2N CPU case " + std::to_string(scenario) + " plane " + std::to_string(plane) + ": valid=" + (hasCost ? "true" : "false") + ", depth=" + std::to_string(depth) + ", cost=" + (hasCost ? std::to_string(cost) : std::string("invalid")) + "\n");
                    }
                    expectedBestDepths[scenario] = bestDepth;
                    expectedBestCosts[scenario] = bestCost;
                    expectedSecondBestCosts[scenario] = secondBestCost;
                    expectedBestPlanes[scenario] = bestPlane;
                    uint32_t status = 0;
                    float finalDepth = 0.0f;
                    if (bestDepth > 0.0f)
                    {
                        if (bestCost > thresholds[scenario]) status = 1;
                        else if (controlledSetup.m_planeSweepConfidenceEnabled && (!std::isfinite(secondBestCost) || (secondBestCost - bestCost) < std::max(0.0f, confidenceDifferences[scenario]))) status = 2;
                        else { status = 3; finalDepth = bestDepth; }
                    }
                    MultiViewPlaneSweepUtility::PlaneSweepDepthRange depthRange; depthRange.m_nearDepth = 1.0f; depthRange.m_farDepth = 4.0f; depthRange.m_fromSparsePoints = false;
                    MultiViewPlaneSweepUtility::PlaneSweepDepthMap cpuDepthMap;
                    std::vector<MultiViewPlaneSweepUtility::PlaneSweepPointCandidate> cpuCandidates;
                    ColmapPlaneSweepResult cpuResult{};
                    MultiViewPlaneSweepUtility::BuildReferencePoints(controlledSetup, referenceImage, activeSources, depthRange, 0, cpuDepthMap, cpuCandidates, cpuResult, nullptr);
                    if (cpuDepthMap.m_width != 1 || cpuDepthMap.m_height != 1 || cpuDepthMap.m_depthValues.size() != 1)
                    {
                        UIUtility::Print("Phase 2N CPU depth-map dimensions invalid for case=" + std::to_string(scenario) + ". GPU test skipped.\n"); return result;
                    }
                    const float cpuFinalDepth = cpuDepthMap.m_depthValues[0];
                    const bool cpuAccepted = cpuFinalDepth > 0.0f && cpuCandidates.size() == 1;
                    const bool expectedAccepted = conceptualStatuses[scenario] == 3;
                    const int expectedConfidenceRejects = scenario == 2 ? 1 : 0;
                    if (status != conceptualStatuses[scenario] || cpuAccepted != expectedAccepted || std::fabs(cpuFinalDepth - finalDepth) > 1e-6f || (cpuAccepted && cpuCandidates.size() != 1) || (!cpuAccepted && !cpuCandidates.empty()) || cpuResult.m_pointsRejectedConfidence != expectedConfidenceRejects)
                    {
                        UIUtility::Print("Phase 2N CPU BuildReferencePoints expectation mismatch case=" + std::to_string(scenario) + ", conceptualStatus=" + std::to_string(conceptualStatuses[scenario]) + ", derivedStatus=" + std::to_string(status) + ", depth=" + std::to_string(cpuFinalDepth) + ", candidates=" + std::to_string(cpuCandidates.size()) + ", confidenceRejects=" + std::to_string(cpuResult.m_pointsRejectedConfidence) + ". GPU test skipped.\n"); cpuExpectationsMatch = false;
                    }
                    expectedStatuses[scenario] = cpuAccepted ? 3u : status;
                    expectedFinalDepths[scenario] = cpuFinalDepth;
                    scenarioFloats[scenario * 8] = 0.0f; scenarioFloats[scenario * 8 + 1] = 0.0f; scenarioFloats[scenario * 8 + 2] = inverseNear; scenarioFloats[scenario * 8 + 3] = inverseFar;
                    scenarioFloats[scenario * 8 + 4] = thresholds[scenario]; scenarioFloats[scenario * 8 + 5] = confidenceDifferences[scenario]; scenarioFloats[scenario * 8 + 6] = 0.5f; scenarioFloats[scenario * 8 + 7] = 1.0f;
                    scenarioParameters[scenario * 4] = 4; scenarioParameters[scenario * 4 + 1] = confidenceEnabled[scenario]; scenarioParameters[scenario * 4 + 2] = 0; scenarioParameters[scenario * 4 + 3] = 1;
                    UIUtility::Print("Phase 2N CPU final case " + std::to_string(scenario) + ": status=" + std::to_string(status) + ", BuildReferencePoints depth=" + std::to_string(cpuFinalDepth) + ", candidates=" + std::to_string(cpuCandidates.size()) + "\n");
                }
                if (!cpuExpectationsMatch) return result;
                std::vector<GpuCameraIntrinsics> gpuIntrinsics{MakeGpuCameraIntrinsics(referenceImage.m_intrinsic), MakeGpuCameraIntrinsics(scenarioImages[0].m_intrinsic)};
                std::vector<GpuCameraPose> gpuPoses{MakeGpuCameraPose(referenceImage.m_camera), MakeGpuCameraPose(scenarioImages[0].m_camera)};
                planeSweepGpuPtr->RunDepthPlaneSearchSmokeTest(referenceImage.m_texture.m_pixels, scenarioSourcePixels, gpuIntrinsics, gpuPoses, scenarioFloats, scenarioParameters,
                    expectedPlaneDepths, expectedPlaneValidity, expectedPlaneCosts, expectedStatuses, expectedFinalDepths, expectedBestDepths, expectedBestCosts, expectedSecondBestCosts, expectedBestPlanes);
            }

            //Returns an empty result when plane sweep input is unavailable or disabled.
            if (!MultiViewPlaneSweepUtility::PlaneSweepReady(colmapData, setup))
            {
                return result;
            }

            const std::vector<std::string> imageExtensions{ ".PNG", ".JPG", ".JPEG", ".BMP", ".TIF", ".TIFF" };
            const MultiViewUtility::MultiViewDepthPointCloudUtility::FileLookup imageFiles =
                MultiViewUtility::MultiViewDepthPointCloudUtility::BuildFileLookup(setup.m_cameraImageFolder, imageExtensions);
            const MultiViewUtility::MultiViewDepthPointCloudUtility::FileLookup maskFiles =
                MultiViewUtility::MultiViewDepthPointCloudUtility::BuildFileLookup(setup.m_cameraMaskFolder, imageExtensions);

            if (imageFiles.empty())
            {
                UIUtility::Print("COLMAP plane sweep found no source images in " + setup.m_cameraImageFolder + ".\n");
                return result;
            }

            //Builds a lookup from image id to camera index.
            const std::unordered_map<uint32_t, std::size_t> cameraIndexLookup = MultiViewPlaneSweepUtility::BuildCameraIndexLookup(colmapData);
            //Loads sparse COLMAP points and image tracks for scale estimation.
            const std::vector<MultiViewDepthPointCloudUtility::SparseScalePoint> sparsePoints = MultiViewDepthPointCloudUtility::LoadSparseScalePoints(setup);
            //Builds sparse point lookup by image id.
            const std::unordered_map<uint32_t, std::vector<std::size_t>> sparsePointLookup = MultiViewDepthPointCloudUtility::BuildSparsePointLookup(sparsePoints);

            //Resolves the reference and source camera sampling requirements.
            const std::size_t referenceStep = static_cast<std::size_t>(std::max(1, setup.m_planeSweepReferenceStep));
            const std::size_t requestedSourceCount = static_cast<std::size_t>(std::max(1, setup.m_planeSweepSourceCount));
            const std::size_t requiredSourceCount = std::min(requestedSourceCount, static_cast<std::size_t>(std::max(1, setup.m_planeSweepMinValidSourceViews)));
            std::vector<MultiViewPlaneSweepUtility::PlaneSweepDepthMap> depthMaps;
            std::vector<MultiViewPlaneSweepUtility::PlaneSweepPointCandidate> pointCandidates;

            //Processes each sampled camera as a plane-sweep reference view.
            for (std::size_t referenceCameraIndex = 0U; referenceCameraIndex < colmapData.m_cameras.size(); referenceCameraIndex += referenceStep)
            {
                MultiViewPlaneSweepUtility::PlaneSweepImage referenceImage;
                if (!MultiViewPlaneSweepUtility::LoadPlaneSweepImage(colmapData, setup, imageFiles, maskFiles, referenceCameraIndex, referenceImage))
                {
                    ++result.m_camerasSkipped;
                    continue;
                }

                //Selects the strongest source-camera candidates for the current reference view.
                const std::vector<std::size_t> sourceCameraIndices = MultiViewPlaneSweepUtility::SelectSourceCameraIndices(
                    colmapData,
                    setup,
                    cameraIndexLookup,
                    sparsePointLookup,
                    sparsePoints,
                    referenceCameraIndex
                );

                //Loads the requested number of available source images.
                std::vector<MultiViewPlaneSweepUtility::PlaneSweepImage> sourceImages;
                sourceImages.reserve(std::min(requestedSourceCount, sourceCameraIndices.size()));
                for (std::size_t sourceCameraIndex : sourceCameraIndices)
                {
                    MultiViewPlaneSweepUtility::PlaneSweepImage sourceImage;
                    if (MultiViewPlaneSweepUtility::LoadPlaneSweepImage(colmapData, setup, imageFiles, maskFiles, sourceCameraIndex, sourceImage))
                    {
                        sourceImages.push_back(std::move(sourceImage));
                    }

                    if (sourceImages.size() >= requestedSourceCount)
                    {
                        break;
                    }
                }

                if (sourceImages.size() < requiredSourceCount)
                {
                    ++result.m_camerasSkipped;
                    continue;
                }

                result.m_sourceImagesLoaded += sourceImages.size();
                //Estimates the depth range and reconstructs points for the reference view. O(k log k). Sorting for getting 5 to 95 percentile.
                MultiViewPlaneSweepUtility::PlaneSweepDepthRange depthRange;
                if (metricsPtr)
                {
                    const auto erStart = std::chrono::steady_clock::now();
                    depthRange = MultiViewPlaneSweepUtility::EstimateDepthRange(
                        referenceImage.m_camera,
                        referenceImage.m_intrinsic,
                        setup,
                        sparsePointLookup,
                        sparsePoints
                    );
                    const auto erEnd = std::chrono::steady_clock::now();
                    metricsPtr->m_estimateDepthRangeMs += std::chrono::duration<double, std::milli>(erEnd - erStart).count();
                }
                else
                {
                    depthRange = MultiViewPlaneSweepUtility::EstimateDepthRange(
                        referenceImage.m_camera,
                        referenceImage.m_intrinsic,
                        setup,
                        sparsePointLookup,
                        sparsePoints
                    );
                }

                // If debug enabled and this is the configured reference image, run pixel sweep diagnostics and then return to avoid full reconstruction.
                if (setup.m_planeSweepDebugEnabled && setup.m_planeSweepDebugReferenceImageId == static_cast<int>(referenceImage.m_camera.m_imageId))
                {
                    MultiViewPlaneSweepUtility::DebugPixelSweep(setup, referenceImage, sourceImages, depthRange);
                    // Do not continue with full expensive reconstruction when debugging a single pixel.
                    return result;
                }

                //Builds plane sweep points for one reference image.
                MultiViewPlaneSweepUtility::PlaneSweepDepthMap depthMap;
                const std::size_t depthMapIndex = depthMaps.size();
                if (metricsPtr)
                {
                    const auto brStart = std::chrono::steady_clock::now();
                    MultiViewPlaneSweepUtility::BuildReferencePoints(
                        setup,
                        referenceImage,
                        sourceImages,
                        depthRange,
                        depthMapIndex,
                        depthMap,
                        pointCandidates,
                        result,
                        metricsPtr
                    );
                    const auto brEnd = std::chrono::steady_clock::now();
                    metricsPtr->m_buildReferencePointsMs += std::chrono::duration<double, std::milli>(brEnd - brStart).count();
                }
                else
                {
                    MultiViewPlaneSweepUtility::BuildReferencePoints(
                        setup,
                        referenceImage,
                        sourceImages,
                        depthRange,
                        depthMapIndex,
                        depthMap,
                        pointCandidates,
                        result
                    );
                }

                if (setup.m_planeSweepCrossViewConsistencyEnabled)
                {
                    depthMaps.push_back(std::move(depthMap));
                }
            }

            if (metricsPtr)
            {
                const auto cvStart = std::chrono::steady_clock::now();
                MultiViewPlaneSweepUtility::ApplyCrossViewConsistency(setup, depthMaps, pointCandidates, result);
                const auto cvEnd = std::chrono::steady_clock::now();
                metricsPtr->m_crossViewMs += std::chrono::duration<double, std::milli>(cvEnd - cvStart).count();

                const auto bfStart = std::chrono::steady_clock::now();
                MultiViewPlaneSweepUtility::BuildFilteredPoints(setup, pointCandidates, result);
                const auto bfEnd = std::chrono::steady_clock::now();
                metricsPtr->m_filteringMs += std::chrono::duration<double, std::milli>(bfEnd - bfStart).count();
            }
            else
            {
                MultiViewPlaneSweepUtility::ApplyCrossViewConsistency(setup, depthMaps, pointCandidates, result);
                MultiViewPlaneSweepUtility::BuildFilteredPoints(setup, pointCandidates, result);
            }

            //Reports the plane-sweep reconstruction totals.
            UIUtility::Print("COLMAP plane sweep processed " + std::to_string(result.m_referenceImagesProcessed) + " references, loaded " + std::to_string(result.m_sourceImagesLoaded) +
                " source images, saved " + std::to_string(result.m_depthFilesSaved) + " depth outputs, skipped " + std::to_string(result.m_camerasSkipped) + " cameras. Rejected texture " +
                std::to_string(result.m_pointsRejectedTexture) + ", confidence " + std::to_string(result.m_pointsRejectedConfidence) + ", cross view " +
                std::to_string(result.m_pointsRejectedCrossView) + ". Points raw " + std::to_string(result.m_pointsBeforeFiltering) + ", fused " +
                std::to_string(result.m_pointsAfterFusion) + ", filtered " + std::to_string(result.m_pointsAfterOutlierFilter) + ".\n");
            return result;
        }

        //Builds sampled point cloud points from COLMAP camera depth images.
        ColmapDepthPointCloudResult MultiViewUtility::BuildColmapDepthPointCloud(const ColmapCameraReadResult &colmapData, const ColmapSetup &setup)
        {
            ColmapDepthPointCloudResult result;
            if (!setup.m_enabled || setup.m_cameraDepthFolder.empty() || colmapData.m_cameras.empty())
            {
                return result;
            }

            const std::vector<std::string> depthExtensions{ ".TIF", ".TIFF", ".PNG" };
            const std::vector<std::string> colorExtensions{ ".PNG", ".JPG", ".JPEG", ".BMP", ".TIF", ".TIFF" };
            const MultiViewUtility::MultiViewDepthPointCloudUtility::FileLookup depthFiles =
                MultiViewUtility::MultiViewDepthPointCloudUtility::BuildFileLookup(setup.m_cameraDepthFolder, depthExtensions);
            const MultiViewUtility::MultiViewDepthPointCloudUtility::FileLookup colorFiles =
                MultiViewUtility::MultiViewDepthPointCloudUtility::BuildFileLookup(setup.m_cameraImageFolder, colorExtensions);

            const std::size_t cameraStep = static_cast<std::size_t>(std::max(1, setup.m_displayEveryNthCamera));
            const uint32_t pixelStep = static_cast<uint32_t>(std::max(1, setup.m_displayEveryNthPixel));
            const MultiViewUtility::MultiViewDepthPointCloudUtility::DepthScaleMode depthScaleMode = MultiViewUtility::MultiViewDepthPointCloudUtility::ParseDepthScaleMode(setup.m_depthScaleMode);
            const MultiViewUtility::MultiViewDepthPointCloudUtility::DepthValueMode depthValueMode = MultiViewUtility::MultiViewDepthPointCloudUtility::ParseDepthValueMode(setup.m_depthValueMode);
            const MultiViewUtility::MultiViewDepthPointCloudUtility::DepthScaleLookup depthScaleLookup = MultiViewUtility::MultiViewDepthPointCloudUtility::ResolveDepthScaleLookup(
                colmapData,
                setup,
                depthScaleMode,
                depthValueMode,
                depthFiles,
                cameraStep
            );

            std::size_t perImageDepthScaleCount = 0U;
            std::size_t fallbackDepthScaleCount = 0U;
            for (std::size_t cameraIndex = 0U; cameraIndex < colmapData.m_cameras.size(); cameraIndex += cameraStep)
            {
                const ColmapCameraData &camera = colmapData.m_cameras[cameraIndex];
                const ColmapCameraIntrinsicData *intrinsicPtr = MultiViewUtility::MultiViewDepthPointCloudUtility::FindIntrinsic(colmapData, camera);
                if (intrinsicPtr == nullptr || !intrinsicPtr->m_supportedModel || intrinsicPtr->m_fx <= 0.0 || intrinsicPtr->m_fy <= 0.0)
                {
                    ++result.m_camerasSkipped;
                    continue;
                }

                const std::filesystem::path depthPath = MultiViewUtility::MultiViewDepthPointCloudUtility::FindMatchingFile(depthFiles, camera, cameraIndex);
                MultiViewUtility::MultiViewDepthPointCloudUtility::FloatDepthImage depthImage;
                if (!MultiViewUtility::MultiViewDepthPointCloudUtility::LoadFloatDepthImage(depthPath, depthImage))
                {
                    ++result.m_camerasSkipped;
                    continue;
                }

                ++result.m_depthImagesLoaded;

                bool usedPerImageScale = false;
                const float cameraDepthScale = MultiViewUtility::MultiViewDepthPointCloudUtility::ResolveCameraDepthScale(
                    depthScaleLookup,
                    depthScaleMode,
                    camera,
                    cameraIndex,
                    setup.m_depthScaleValueMetricToColmap,
                    usedPerImageScale
                );
                if (usedPerImageScale)
                {
                    ++perImageDepthScaleCount;
                }
                else if (depthScaleMode == MultiViewUtility::MultiViewDepthPointCloudUtility::DepthScaleMode::AutoPerImageFromPoints3D ||
                    depthScaleMode == MultiViewUtility::MultiViewDepthPointCloudUtility::DepthScaleMode::AutoHybridFromPoints3D)
                {
                    ++fallbackDepthScaleCount;
                }

                UIUtility::Print(std::to_string(cameraIndex) + " depth scale value "  + std::to_string(cameraDepthScale) + "\n");

                TextureData colorImage;
                bool hasColorImage = false;
                const std::filesystem::path colorPath = MultiViewUtility::MultiViewDepthPointCloudUtility::FindMatchingFile(colorFiles, camera, cameraIndex);
                if (!colorPath.empty())
                {
                    hasColorImage = LoadTexture(colorPath.string(), colorImage);
                    if (hasColorImage)
                    {
                        ++result.m_colorImagesLoaded;
                    }
                }

                for (uint32_t y = 0U; y < depthImage.m_height; y += pixelStep)
                {
                    for (uint32_t x = 0U; x < depthImage.m_width; x += pixelStep)
                    {
                        const std::size_t depthIndex = static_cast<std::size_t>(y) * depthImage.m_width + x;
                        const float depth = depthImage.m_values[depthIndex] * cameraDepthScale;
                        if (!std::isfinite(depth) || depth <= 0.0f)
                        {
                            continue;
                        }

                        const Float3 position = MultiViewUtility::MultiViewDepthPointCloudUtility::UnprojectDepthSample(camera, *intrinsicPtr, x, y, depth, depthValueMode);
                        const Float4 color = hasColorImage ?
                            MultiViewUtility::MultiViewDepthPointCloudUtility::SampleColor(colorImage, x, y, depthImage.m_width, depthImage.m_height, setup.m_cameraColor) : setup.m_cameraColor;

                        result.m_points.push_back(SceneObjectPointCloudPoint{ position, color });
                    }
                }
            }

            if (depthScaleMode == MultiViewUtility::MultiViewDepthPointCloudUtility::DepthScaleMode::AutoPerImageFromPoints3D ||
                depthScaleMode == MultiViewUtility::MultiViewDepthPointCloudUtility::DepthScaleMode::AutoHybridFromPoints3D)
            {
                UIUtility::Print("Depth scale cameras using per-image = " + std::to_string(perImageDepthScaleCount) + ", fallback = " + std::to_string(fallbackDepthScaleCount) + ".\n");
            }

            return result;
        }
}
