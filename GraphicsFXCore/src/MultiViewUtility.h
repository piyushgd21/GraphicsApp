///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	MultiViewUtility.h
//	Description	:
//		Defines multi-view camera and point cloud loading helpers.
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "global.h"

#include "MathUtility.h"
#include "SceneObjectPointCloud.h"
#include "SetupUtility.h"
#include "TextureUtility.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct IWICImagingFactory;

namespace Microsoft
{
    namespace WRL
    {
        template <typename T>
        class ComPtr;
    }
}

namespace graphicsFX
{
    class PlaneSweepGpu;

    //Stores one COLMAP camera intrinsic record.
    struct ColmapCameraIntrinsicData
    {
        uint32_t m_cameraId{ 0U }; //Stores the COLMAP camera id.
        std::string m_model; //Stores the COLMAP camera model name.
        uint32_t m_width{ 0U }; //Stores the camera image width.
        uint32_t m_height{ 0U }; //Stores the camera image height.
        std::vector<double> m_parameters; //Stores the model-specific camera parameters.
        bool m_supportedModel{ false }; //Stores whether the model parameters were interpreted.
        double m_fx{ 0.0 }; //Stores the focal length in X.
        double m_fy{ 0.0 }; //Stores the focal length in Y.
        double m_cx{ 0.0 }; //Stores the principal point X coordinate.
        double m_cy{ 0.0 }; //Stores the principal point Y coordinate.
        double m_k1{ 0.0 }; //Stores the first radial distortion term.
        double m_k2{ 0.0 }; //Stores the second radial distortion term.
        double m_p1{ 0.0 }; //Stores the first tangential distortion term.
        double m_p2{ 0.0 }; //Stores the second tangential distortion term.
    };

    //Stores one COLMAP 2D observation for debug anchor experiments.
    struct ColmapPointObservation
    {
        uint32_t m_imageId{ 0U }; // image id where observed
        uint32_t m_point2dIdx{ 0U }; // index of the 2D keypoint in the image (order in images.txt)
        float m_x{ 0.0f }; // observed pixel x
        float m_y{ 0.0f }; // observed pixel y
    };

    //Stores one COLMAP 3D point with attached debug observations.
    struct ColmapPoint3DDebug
    {
        int64_t m_pointId{ -1 }; // POINT3D_ID
        Float3 m_position{ 0.0f, 0.0f, 0.0f }; // XYZ (converted to app coordinates)
        double m_error{ 0.0 }; // COLMAP reprojection error
        std::vector<ColmapPointObservation> m_observations; // all 2D observations
    };

    //Result of a debug-only COLMAP points + observations read.
    struct ColmapPointsDebugReadResult
    {
        // map point id -> point record
        std::unordered_map<int64_t, ColmapPoint3DDebug> m_pointsById;
        // per-image observation list (image id -> list of observations)
        std::unordered_map<uint32_t, std::vector<ColmapPointObservation>> m_imageObservations;
    };

    //Metrics and timings collected during an experimental plane sweep run.
    struct PlaneSweepExperimentMetrics
    {
        std::string m_experimentId;
        std::string m_timestamp;
        std::string m_datasetPath;
        // Timings (milliseconds)
        double m_totalMs{ 0.0 };
        double m_estimateDepthRangeMs{ 0.0 };
        double m_buildReferencePointsMs{ 0.0 };
        double m_computePlaneCostMs{ 0.0 };
        double m_crossViewMs{ 0.0 };
        double m_filteringMs{ 0.0 };
        // Counters (copied from result)
        std::size_t m_referenceImagesProcessed{ 0 };
        std::size_t m_sourceImagesLoaded{ 0 };
        std::size_t m_depthFilesSaved{ 0 };
        std::size_t m_pointsRejectedTexture{ 0 };
        std::size_t m_pointsRejectedConfidence{ 0 };
        std::size_t m_pointsRejectedCrossView{ 0 };
        std::size_t m_pointsBeforeFiltering{ 0 };
        std::size_t m_pointsAfterFusion{ 0 };
        std::size_t m_pointsAfterOutlierFilter{ 0 };
        std::size_t m_finalPointCount{ 0 };
        std::string m_depthOutputFolder;
        std::string m_pointCloudOutputPath;
    };

    //Stores one COLMAP camera pose ready for scene visualization.
    struct ColmapCameraData
    {
        uint32_t m_imageId{ 0U }; //Stores the COLMAP image id.
        uint32_t m_cameraId{ 0U }; //Stores the referenced COLMAP camera id.
        std::string m_name; //Stores the camera or image name.
        Float3 m_position{ 0.0f, 0.0f, 0.0f }; //Stores the camera center position.
        Float3 m_forward{ 0.0f, 0.0f, -1.0f }; //Stores the camera forward direction.
        Float3 m_up{ 0.0f, 1.0f, 0.0f }; //Stores the camera up direction.
        Float3 m_right{ 1.0f, 0.0f, 0.0f }; //Stores the camera right direction.
    };

    //Stores parsed COLMAP camera files.
    struct ColmapCameraReadResult
    {
        std::vector<ColmapCameraIntrinsicData> m_intrinsics; //Stores parsed camera intrinsic records.
        std::vector<ColmapCameraData> m_cameras; //Stores parsed camera pose records.
    };

    //Stores the result of building a COLMAP sparse point cloud.
    struct ColmapSparsePointCloudResult
    {
        std::vector<SceneObjectPointCloudPoint> m_points; //Stores generated point cloud points.
        std::size_t m_pointsRead{ 0U }; //Stores number of parsed points.
        std::size_t m_pointsDisplayed{ 0U }; //Stores number of displayed points.
        std::size_t m_pointsSkipped{ 0U }; //Stores number of skipped points.
        std::size_t m_trackMatches{ 0U }; //Stores number of track observations matching loaded cameras.
    };

    //Stores the result of building a COLMAP depth point cloud.
    struct ColmapDepthPointCloudResult
    {
        std::vector<SceneObjectPointCloudPoint> m_points; //Stores generated point cloud points.
        std::size_t m_depthImagesLoaded{ 0U }; //Stores number of loaded depth images.
        std::size_t m_colorImagesLoaded{ 0U }; //Stores number of loaded color images.
        std::size_t m_camerasSkipped{ 0U }; //Stores number of skipped cameras.
    };

    //Stores the result of building a COLMAP plane sweep point cloud.
    struct ColmapPlaneSweepResult
    {
        std::vector<SceneObjectPointCloudPoint> m_points; //Stores generated point cloud points.
        std::size_t m_referenceImagesProcessed{ 0U }; //Stores processed reference images.
        std::size_t m_sourceImagesLoaded{ 0U }; //Stores loaded source images.
        std::size_t m_depthFilesSaved{ 0U }; //Stores saved depth output files.
        std::size_t m_camerasSkipped{ 0U }; //Stores skipped reference cameras.
        std::size_t m_pointsRejectedTexture{ 0U }; //Stores points rejected for insufficient texture.
        std::size_t m_pointsRejectedConfidence{ 0U }; //Stores points rejected for ambiguous depth cost.
        std::size_t m_pointsRejectedCrossView{ 0U }; //Stores points rejected by depth-map consistency.
        std::size_t m_pointsBeforeFiltering{ 0U }; //Stores points before fusion and outlier filtering.
        std::size_t m_pointsAfterFusion{ 0U }; //Stores points after voxel fusion.
        std::size_t m_pointsAfterOutlierFilter{ 0U }; //Stores points after outlier filtering.
    };

    //Defines multi-view camera and point cloud loading helpers.
    class MultiViewUtility final
    {

        public:

            //Defines helper functions for COLMAP camera loading.
            class MultiViewCameraUtility final
            {

                public:

                    //Stores one raw COLMAP world-to-camera pose.
                    struct RawColmapPose
                    {
                        uint32_t m_imageId{ 0U }; //Stores the COLMAP image id.
                        uint32_t m_cameraId{ 0U }; //Stores the referenced COLMAP camera id.
                        std::string m_name; //Stores the camera or image name.
                        double m_qw{ 1.0 }; //Stores the quaternion W component.
                        double m_qx{ 0.0 }; //Stores the quaternion X component.
                        double m_qy{ 0.0 }; //Stores the quaternion Y component.
                        double m_qz{ 0.0 }; //Stores the quaternion Z component.
                        double m_tx{ 0.0 }; //Stores the translation X component.
                        double m_ty{ 0.0 }; //Stores the translation Y component.
                        double m_tz{ 0.0 }; //Stores the translation Z component.
                    };

                    MultiViewCameraUtility(void) = delete;

                    //Returns a line with comments removed.
                    static std::string RemoveComment(const std::string &line);

                    //Splits one text line into whitespace tokens.
                    static std::vector<std::string> SplitTokens(std::string line);

                    //Returns whether a token is an integer token.
                    static bool IsIntegerToken(const std::string &token);

                    //Parses an unsigned integer token.
                    static bool ParseUnsignedInteger(const std::string &token, uint32_t &result);

                    //Parses a double token.
                    static bool ParseDouble(const std::string &token, double &result);

                    //Returns whether a raw pose has a valid quaternion.
                    static bool QuaternionLooksValid(const RawColmapPose &pose);

                    //Parses a standard COLMAP image pose line.
                    static bool TryParseImagePoseLine(const std::vector<std::string> &tokens, RawColmapPose &pose);

                    //Parses a compact COLMAP pose line.
                    static bool TryParseCompactPoseLine(const std::vector<std::string> &tokens, RawColmapPose &pose);

                    //Interprets one COLMAP camera intrinsic model.
                    static bool InterpretCameraIntrinsicModel(ColmapCameraIntrinsicData &camera);

                    //Parses one COLMAP camera intrinsic line.
                    static bool TryParseCameraIntrinsicLine(const std::vector<std::string> &tokens, ColmapCameraIntrinsicData &camera);

                    //Converts a COLMAP coordinate vector into app coordinates.
                    static Float3 ConvertCoordinate(const Float3 &value, const std::string &coordinateMode);

                    //Converts one raw COLMAP pose to app camera data.
                    static ColmapCameraData ConvertPose(const RawColmapPose &pose, const ColmapSetup &setup, std::size_t cameraIndex);

                    //Loads COLMAP camera intrinsic records.
                    static std::vector<ColmapCameraIntrinsicData> LoadIntrinsicData(const std::string &fileName);

                    //Loads COLMAP camera pose records.
                    static std::vector<ColmapCameraData> LoadExtrinsicData(const ColmapSetup &setup);
            };

            //Defines helper functions for COLMAP sparse point cloud loading.
            class MultiViewSparsePointCloudUtility final
            {

                public:

                    MultiViewSparsePointCloudUtility(void) = delete;

                    //Returns a line with comments removed.
                    static std::string RemoveComment(const std::string &line);

                    //Splits one text line into whitespace tokens.
                    static std::vector<std::string> SplitTokens(const std::string &line);

                    //Parses an unsigned integer token.
                    static bool ParseUnsignedInteger(const std::string &token, uint32_t &result);

                    //Parses a double token.
                    static bool ParseDouble(const std::string &token, double &result);

                    //Parses an 8-bit color token.
                    static bool ParseColorByte(const std::string &token, uint8_t &result);

                    //Converts a COLMAP coordinate vector into app coordinates.
                    static Float3 ConvertCoordinate(const Float3 &value, const std::string &coordinateMode);

                    //Builds the image id set for loaded cameras.
                    static std::unordered_set<uint32_t> LoadedImageIds(const ColmapCameraReadResult &colmapData);

                    //Counts point track observations that reference loaded cameras.
                    static std::size_t CountLoadedCameraTrackMatches(const std::vector<std::string> &tokens, const std::unordered_set<uint32_t> &loadedImageIds);
            };

            //Defines helper functions for COLMAP depth point cloud loading.
            class MultiViewDepthPointCloudUtility final
            {

                public:

                    //Stores one decoded float depth image.
                    struct FloatDepthImage
                    {
                        uint32_t m_width{ 0U }; //Stores image width.
                        uint32_t m_height{ 0U }; //Stores image height.
                        std::vector<float> m_values; //Stores depth values in row-major order.
                    };

                    //Stores one sparse COLMAP point used for scale estimation.
                    struct SparseScalePoint
                    {
                        Float3 m_position{ 0.0f, 0.0f, 0.0f }; //Stores the transformed point position.
                        std::vector<uint32_t> m_imageIds; //Stores image ids that observe the point.
                    };

                    //Stores one auto depth scale estimate.
                    struct DepthScaleEstimate
                    {
                        float m_scale{ 1.0f }; //Stores the estimated scale.
                        std::size_t m_sampleCount{ 0U }; //Stores number of accepted samples.
                        bool m_estimated{ false }; //Stores whether estimation succeeded.
                    };

                    //Stores resolved global and per-image depth scales.
                    struct DepthScaleLookup
                    {
                        float m_globalScale{ 1.0f }; //Stores the global or fallback scale.
                        std::size_t m_globalSampleCount{ 0U }; //Stores samples used for the global scale.
                        bool m_globalEstimated{ false }; //Stores whether the global scale was estimated.
                        std::unordered_map<uint32_t, DepthScaleEstimate> m_imageScales; //Stores scale estimates by COLMAP image id.
                    };

                    //Defines supported depth scale modes.
                    enum class DepthScaleMode
                    {
                        Manual, AutoGlobalFromPoints3D, AutoPerImageFromPoints3D, AutoHybridFromPoints3D
                    };

                    //Defines supported depth value conventions.
                    enum class DepthValueMode
                    {
                        CameraZ, RayLength
                    };

                    using FileLookup = std::unordered_map<std::string, std::filesystem::path>;

                    //Initializes COM while WIC loading is active.
                    class ComInitializer
                    {

                        public:

                            //Initializes COM for WIC work.
                            ComInitializer(void);

                            //Releases COM initialization when owned.
                            ~ComInitializer(void);

                            ComInitializer(const ComInitializer &) = delete;
                            ComInitializer &operator=(const ComInitializer &) = delete;

                        private:

                            bool m_shouldUninitialize{ false }; //Stores whether this object owns COM cleanup.
                    };

                    MultiViewDepthPointCloudUtility(void) = delete;

                    //Throws a runtime error when an HRESULT fails.
                    static void CheckResult(long result, const char *messagePtr);

                    //Creates a WIC imaging factory.
                    static Microsoft::WRL::ComPtr<IWICImagingFactory> CreateFactory(void);

                    //Returns a line with comments removed.
                    static std::string RemoveComment(const std::string &line);

                    //Splits one text line into whitespace tokens.
                    static std::vector<std::string> SplitTokens(const std::string &line);

                    //Parses an unsigned integer token.
                    static bool ParseUnsignedInteger(const std::string &token, uint32_t &result);

                    //Parses a double token.
                    static bool ParseDouble(const std::string &token, double &result);

                    //Returns the configured depth scale mode.
                    static DepthScaleMode ParseDepthScaleMode(std::string depthScaleMode);

                    //Returns the configured depth value mode.
                    static DepthValueMode ParseDepthValueMode(std::string depthValueMode);

                    //Converts a COLMAP coordinate vector into app coordinates.
                    static Float3 ConvertCoordinate(const Float3 &value, const std::string &coordinateMode);

                    //Returns a normalized file stem key.
                    static std::string NormalizeNumberedStem(const std::string &stem);

                    //Adds one lookup key.
                    static void AddLookupKey(FileLookup &lookup, const std::string &key, const std::filesystem::path &path);

                    //Builds a file lookup table.
                    static FileLookup BuildFileLookup(const std::string &folderName, const std::vector<std::string> &extensions);

                    //Finds a depth or color file matching a camera.
                    static std::filesystem::path FindMatchingFile(const FileLookup &lookup, const ColmapCameraData &camera, std::size_t cameraIndex);

                    //Loads one float depth image.
                    static bool LoadFloatDepthImage(const std::filesystem::path &fileName, FloatDepthImage &depthImage);

                    //Finds an intrinsic record for one camera.
                    static const ColmapCameraIntrinsicData *FindIntrinsic(const ColmapCameraReadResult &colmapData, const ColmapCameraData &camera);

                    //Samples a color image.
                    static Float4 SampleColor(const TextureData &colorImage, uint32_t x, uint32_t y, uint32_t sourceWidth, uint32_t sourceHeight, const Float4 &fallbackColor);

                    //Unprojects one depth sample.
                    static Float3 UnprojectDepthSample(const ColmapCameraData &camera, const ColmapCameraIntrinsicData &intrinsic, uint32_t x, uint32_t y, float depth, DepthValueMode depthValueMode);

                    //Loads sparse COLMAP points for auto depth scaling.
                    static std::vector<SparseScalePoint> LoadSparseScalePoints(const ColmapSetup &setup);

                    //Builds sparse point lookup by image id.
                    static std::unordered_map<uint32_t, std::vector<std::size_t>> BuildSparsePointLookup(const std::vector<SparseScalePoint> &points);

                    //Projects one world point into a camera pixel.
                    static bool ProjectPointToDepthPixel(
                        const ColmapCameraData &camera,
                        const ColmapCameraIntrinsicData &intrinsic,
                        const Float3 &point,
                        uint32_t imageWidth,
                        uint32_t imageHeight,
                        DepthValueMode depthValueMode,
                        uint32_t &x,
                        uint32_t &y,
                        float &colmapDepth
                    );

                    //Returns the median value.
                    static float MedianValue(std::vector<float> &values);

                    //Returns a formatted float string.
                    static std::string FormatFloat(float value);

                    //Returns whether a scale mode uses sparse point matches.
                    static bool DepthScaleModeUsesPointMatches(DepthScaleMode depthScaleMode);

                    //Returns the scale lookup key for one camera.
                    static uint32_t GetDepthScaleImageKey(const ColmapCameraData &camera, std::size_t cameraIndex);

                    //Estimates metric depth to COLMAP depth scale lookup.
                    static DepthScaleLookup EstimateDepthScaleLookupFromPoints3D(
                        const ColmapCameraReadResult &colmapData,
                        const ColmapSetup &setup,
                        const FileLookup &depthFiles,
                        DepthValueMode depthValueMode,
                        std::size_t cameraStep
                    );

                    //Returns the resolved depth scale lookup for unprojection.
                    static DepthScaleLookup ResolveDepthScaleLookup(
                        const ColmapCameraReadResult &colmapData,
                        const ColmapSetup &setup,
                        DepthScaleMode depthScaleMode,
                        DepthValueMode depthValueMode,
                        const FileLookup &depthFiles,
                        std::size_t cameraStep
                    );

                    //Returns the resolved depth scale for one camera.
                    static float ResolveCameraDepthScale(
                        const DepthScaleLookup &depthScaleLookup,
                        DepthScaleMode depthScaleMode,
                        const ColmapCameraData &camera,
                        std::size_t cameraIndex,
                        float manualScale,
                        bool &usedPerImageScale
                    );

                    static constexpr std::size_t s_minAutoScaleSamples = 12U; //Stores minimum samples needed for auto scale.
            };

            //Defines helper functions for COLMAP plane sweep reconstruction.
            class MultiViewPlaneSweepUtility final
            {

                public:

                    //Stores one plane sweep source image.
                    struct PlaneSweepImage
                    {
                        ColmapCameraData m_camera; //Stores the image camera pose.
                        ColmapCameraIntrinsicData m_intrinsic; //Stores the image intrinsic record.
                        TextureData m_texture; //Stores the decoded image texture.
                        TextureData m_maskTexture; //Stores the optional decoded mask texture.
                        bool m_hasMask{ false }; //Stores whether a mask texture is available.
                        bool m_maskUsesAlpha{ false }; //Stores whether mask opacity comes from alpha.
                        std::size_t m_cameraIndex{ 0U }; //Stores the source camera index.
                    };

                    //Stores one plane sweep depth range.
                    struct PlaneSweepDepthRange
                    {
                        float m_nearDepth{ 0.1f }; //Stores the near depth.
                        float m_farDepth{ 50.0f }; //Stores the far depth.
                        bool m_fromSparsePoints{ false }; //Stores whether sparse points defined the range.
                    };

                    //Stores one source camera candidate.
                    struct PlaneSweepSourceCandidate
                    {
                        std::size_t m_cameraIndex{ 0U }; //Stores the candidate camera index.
                        std::size_t m_sharedPoints{ 0U }; //Stores number of shared sparse points.
                        float m_score{ 0.0f }; //Stores the source selection score.
                    };

                    //Stores one reconstructed reference depth map.
                    struct PlaneSweepDepthMap
                    {
                        ColmapCameraData m_camera; //Stores the reference camera pose.
                        ColmapCameraIntrinsicData m_intrinsic; //Stores the reference camera intrinsics.
                        uint32_t m_imageWidth{ 0U }; //Stores the source image width.
                        uint32_t m_imageHeight{ 0U }; //Stores the source image height.
                        uint32_t m_width{ 0U }; //Stores the sampled depth map width.
                        uint32_t m_height{ 0U }; //Stores the sampled depth map height.
                        std::vector<float> m_depthValues; //Stores camera Z depth values.
                    };

                    //Stores one plane sweep point before final filtering.
                    struct PlaneSweepPointCandidate
                    {
                        std::size_t m_depthMapIndex{ 0U }; //Stores the originating depth map index.
                        Float3 m_position; //Stores the reconstructed world position.
                        Float4 m_color; //Stores the reference image color.
                    };

                    //Stores one integer coordinate used by spatial filters.
                    struct PlaneSweepVoxelKey
                    {
                        int64_t m_x{ 0 }; //Stores the voxel X coordinate.
                        int64_t m_y{ 0 }; //Stores the voxel Y coordinate.
                        int64_t m_z{ 0 }; //Stores the voxel Z coordinate.

                        bool operator == (const PlaneSweepVoxelKey &other) const;
                    };

                    //Hashes one plane sweep spatial key.
                    struct PlaneSweepVoxelKeyHash
                    {
                        std::size_t operator () (const PlaneSweepVoxelKey &key) const;
                    };

                    //Stores averaged point values for one fusion voxel.
                    struct PlaneSweepVoxelData
                    {
                        Float3 m_positionSum; //Stores summed point positions.
                        Float4 m_colorSum; //Stores summed point colors.
                        std::size_t m_observationCount{ 0U }; //Stores number of fused observations.
                    };

                    MultiViewPlaneSweepUtility(void) = delete;

                    //Returns whether plane sweep can run.
                    static bool PlaneSweepReady(const ColmapCameraReadResult &colmapData, const ColmapSetup &setup);

                    //Builds a lookup from image id to camera index.
                    static std::unordered_map<uint32_t, std::size_t> BuildCameraIndexLookup(const ColmapCameraReadResult &colmapData);

                    //Loads one plane sweep image.
                    static bool LoadPlaneSweepImage(
                        const ColmapCameraReadResult &colmapData,
                        const ColmapSetup &setup,
                        const MultiViewDepthPointCloudUtility::FileLookup &imageFiles,
                        const MultiViewDepthPointCloudUtility::FileLookup &maskFiles,
                        std::size_t cameraIndex,
                        PlaneSweepImage &image
                    );

                    //Returns whether a plane sweep image contains an opaque pixel.
                    static bool HasOpaquePixel(const PlaneSweepImage &image, float alphaThreshold);

                    //Selects source camera indices for one reference camera.
                    static std::vector<std::size_t> SelectSourceCameraIndices(
                        const ColmapCameraReadResult &colmapData,
                        const ColmapSetup &setup,
                        const std::unordered_map<uint32_t, std::size_t> &cameraIndexLookup,
                        const std::unordered_map<uint32_t, std::vector<std::size_t>> &sparsePointLookup,
                        const std::vector<MultiViewDepthPointCloudUtility::SparseScalePoint> &sparsePoints,
                        std::size_t referenceCameraIndex
                    );

                    //Returns a sparse point depth range for one camera.
                    static PlaneSweepDepthRange EstimateDepthRange(
                        const ColmapCameraData &camera,
                        const ColmapCameraIntrinsicData &intrinsic,
                        const ColmapSetup &setup,
                        const std::unordered_map<uint32_t, std::vector<std::size_t>> &sparsePointLookup,
                        const std::vector<MultiViewDepthPointCloudUtility::SparseScalePoint> &sparsePoints
                    );

                    //Unprojects one reference pixel at a camera Z depth.
                    static Float3 UnprojectPixel(const ColmapCameraData &camera, const ColmapCameraIntrinsicData &intrinsic, float x, float y, float depth);

                    //Projects one world point into an image pixel.
                    static bool ProjectPoint(const ColmapCameraData &camera, const ColmapCameraIntrinsicData &intrinsic, const Float3 &point, float &x, float &y, float &depth);

                    //Samples one texture with bilinear filtering.
                    static bool SampleBilinear(const TextureData &texture, float x, float y, Float4 &color);

                    //Samples one plane sweep image with its optional mask.
                    static bool SamplePlaneSweepPixel(const PlaneSweepImage &image, float x, float y, Float4 &color);

                    //Computes grayscale patch texture variation.
                    static bool ComputeTextureStandardDeviation(
                        const PlaneSweepImage &referenceImage,
                        float x,
                        float y,
                        int patchRadius,
                        float alphaThreshold,
                        float &standardDeviation
                    );

                    //Computes the matching cost for one depth plane.
                    static bool ComputePlaneCost(
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
                    );

                    //Debug one reference pixel across all candidate depth planes and write diagnostics.
                    static void DebugPixelSweep(
                        const ColmapSetup &setup,
                        const PlaneSweepImage &referenceImage,
                        const std::vector<PlaneSweepImage> &sourceImages,
                        const PlaneSweepDepthRange &depthRange
                    );

                    //Saves one plane sweep depth output.
                    static bool SaveDepthOutput(const ColmapSetup &setup, const ColmapCameraData &camera, const std::vector<float> &depthValues, uint32_t width, uint32_t height);

                    //Builds plane sweep points for one reference image.
                    static void BuildReferencePoints(
                        const ColmapSetup &setup,
                        const PlaneSweepImage &referenceImage,
                        const std::vector<PlaneSweepImage> &sourceImages,
                        const PlaneSweepDepthRange &depthRange,
                        std::size_t depthMapIndex,
                        PlaneSweepDepthMap &depthMap,
                        std::vector<PlaneSweepPointCandidate> &pointCandidates,
                        ColmapPlaneSweepResult &result,
                        PlaneSweepExperimentMetrics *metricsPtr = nullptr
                    );

                    //Rejects points unsupported by other reconstructed depth maps.
                    static void ApplyCrossViewConsistency(
                        const ColmapSetup &setup,
                        const std::vector<PlaneSweepDepthMap> &depthMaps,
                        std::vector<PlaneSweepPointCandidate> &pointCandidates,
                        ColmapPlaneSweepResult &result
                    );

                    //Fuses and filters plane sweep point candidates.
                    static void BuildFilteredPoints(
                        const ColmapSetup &setup,
                        const std::vector<PlaneSweepPointCandidate> &pointCandidates,
                        ColmapPlaneSweepResult &result
                    );

                    //Returns whether sparse points should define the depth range.
                    static bool DepthRangeUsesSparsePoints(const std::string &depthRangeMode);

                    //Returns a robust percentile value.
                    static float PercentileValue(std::vector<float> &values, float percentile);
            };

        public:

            MultiViewUtility(void) = delete;

            //Loads COLMAP camera intrinsics and poses.
            static ColmapCameraReadResult LoadColmapCameras(const ColmapSetup &setup);

            //Debug-only: Loads COLMAP points3D.txt and images.txt observations.
            static ColmapPointsDebugReadResult LoadColmapPointsWithObservations(const ColmapSetup &setup);

            //Builds sparse COLMAP points from points3D data.
            static ColmapSparsePointCloudResult BuildColmapSparsePointCloud(const ColmapCameraReadResult &colmapData, const ColmapSetup &setup);

            //Builds COLMAP depth points from metric depth images.
            static ColmapDepthPointCloudResult BuildColmapDepthPointCloud(const ColmapCameraReadResult &colmapData, const ColmapSetup &setup);

            //Builds COLMAP points from plane sweep reconstruction.
            static ColmapPlaneSweepResult BuildColmapPlaneSweepPointCloud(
                const ColmapCameraReadResult &colmapData,
                const ColmapSetup &setup,
                PlaneSweepExperimentMetrics *metricsPtr = nullptr,
                PlaneSweepGpu *planeSweepGpuPtr = nullptr
            );

    };
}
