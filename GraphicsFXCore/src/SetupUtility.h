///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	SetupUtility.h
//	Description	:
//		Defines program setup values loaded from setup.ini.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "global.h"

#include "MathUtility.h"

#include <cstdint>
#include <string>

namespace graphicsFX
{
    //Stores window setup settings.
    struct WindowSetup
    {
        std::string m_title{ "UM-IHC Immersive Visualization Research Center - Graphics Application" }; //Stores the window title.
        int m_width{ 1920 }; //Stores the window client width.
        int m_height{ 1080 }; //Stores the window client height.
    };

    //Stores graphics setup settings.
    struct GraphicsSetup
    {
        Float4 m_clearColor{ 0.0f, 0.0f, 0.0f, 1.0f }; //Stores the default clear color.
        bool m_vsyncEnabled{ true }; //Stores whether vertical sync is enabled.
        bool m_antiAliasingEnabled{ true }; //Stores whether anti-aliasing is enabled.
        int m_antiAliasingSamples{ 4 }; //Stores requested anti-aliasing samples.
        float m_gridLineWidth{ 2.0f }; //Stores the grid line width.
        float m_axisLineWidth{ 3.0f }; //Stores the axis line width.
        float m_wireframeLineWidth{ 2.0f }; //Stores the wireframe line width.
    };

    //Stores shader compilation setup settings.
    struct ShaderSetup
    {
        bool m_compileAtStartup{ true }; //Stores whether shaders compile when the program starts.
        std::string m_sourceFolder{ "Shaders" }; //Stores the GLSL source shader folder.
        std::string m_outputFolder{ "Output/Shaders" }; //Stores the compiled shader output folder.
        std::string m_compilerPath; //Stores an optional explicit glslc compiler path.
        bool m_compileOnlyIfChanged{ true }; //Stores whether unchanged shaders are skipped.
    };

    //Stores centralized runtime path settings.
    struct PathSetup
    {
        std::string m_textureFolder{ "Asset" }; //Stores the texture search folder.
        std::string m_modelFolder{ "Asset" }; //Stores the model search folder.
        std::string m_shaderSourceFolder{ "Shaders" }; //Stores the shader source folder.
        std::string m_shaderOutputFolder{ "Output/Shaders" }; //Stores the shader output folder.
        std::string m_captureOutputFolder{ "Output" }; //Stores the frame capture output folder.
    };

    //Stores camera keyboard action mappings.
    struct KeyboardSetup
    {
        uint32_t m_moveForwardKey{ 'W' }; //Stores the move forward key.
        uint32_t m_moveBackwardKey{ 'S' }; //Stores the move backward key.
        uint32_t m_moveLeftKey{ 'A' }; //Stores the move left key.
        uint32_t m_moveRightKey{ 'D' }; //Stores the move right key.
        uint32_t m_moveUpKey{ 'R' }; //Stores the move up key.
        uint32_t m_moveDownKey{ 'F' }; //Stores the move down key.
        uint32_t m_pitchUpKey{ 0x26U }; //Stores the pitch up key.
        uint32_t m_pitchDownKey{ 0x28U }; //Stores the pitch down key.
        uint32_t m_yawLeftKey{ 0x25U }; //Stores the yaw left key.
        uint32_t m_yawRightKey{ 0x27U }; //Stores the yaw right key.
        uint32_t m_rollLeftKey{ 'Q' }; //Stores the roll left key.
        uint32_t m_rollRightKey{ 'E' }; //Stores the roll right key.
        uint32_t m_fastKey{ 0x10U }; //Stores the faster movement key.
        uint32_t m_slowKey{ 0x11U }; //Stores the slower movement key.
        uint32_t m_transformTranslateKey{ 'Z' }; //Stores the transform translate key.
        uint32_t m_transformRotateKey{ 'X' }; //Stores the transform rotate key.
        uint32_t m_transformScaleKey{ 'C' }; //Stores the transform scale key.
    };

    //Stores camera mouse action mappings.
    struct MouseSetup
    {
        uint32_t m_lookButton{ 1U }; //Stores the mouse look button.
        uint32_t m_smoothMoveButton{ 2U }; //Stores the smooth forward/back mouse button.
    };

    //Stores camera setup settings.
    struct CameraSetup
    {
        Float3 m_position{ 0.0f, 0.0f, 5.0f }; //Stores the camera position.
        Float3 m_lookAt{ 0.0f, 0.0f, -1.0f }; //Stores the camera look direction.
        Float3 m_up{ 0.0f, 1.0f, 0.0f }; //Stores the camera up direction.
        Float3 m_side{ 1.0f, 0.0f, 0.0f }; //Stores the camera side direction.
        float m_fovDegrees{ 45.0f }; //Stores the camera field of view.
        float m_nearPlane{ 0.1f }; //Stores the camera near clip plane.
        float m_farPlane{ 100.0f }; //Stores the camera far clip plane.
        float m_moveSpeed{ 4.0f }; //Stores camera movement speed.
        float m_rotateSpeed{ 1.8f }; //Stores camera keyboard rotation speed.
        float m_mouseSensitivity{ 0.004f }; //Stores camera mouse rotation sensitivity.
        float m_mouseWheelMoveAmount{ 1.5f }; //Stores movement distance for one mouse wheel notch.
        float m_smoothMouseMoveSensitivity{ 0.025f }; //Stores smooth mouse drag movement sensitivity.
        float m_fastMultiplier{ 3.0f }; //Stores the fast movement multiplier.
        float m_slowMultiplier{ 0.25f }; //Stores the slow movement multiplier.
    };

    //Stores lighting setup settings.
    struct LightingSetup
    {
        Float3 m_position{ 5.0f, 7.0f, 5.0f }; //Stores the light position.
        Float3 m_direction{ -0.5f, -0.7f, -0.5f }; //Stores the light pointing direction.
        float m_fovDegrees{ 45.0f }; //Stores the light field of view.
        Float4 m_ambientColor{ 0.18f, 0.18f, 0.18f, 1.0f }; //Stores ambient light color.
        Float4 m_diffuseColor{ 0.85f, 0.85f, 0.85f, 1.0f }; //Stores diffuse light color.
        Float4 m_specularColor{ 1.0f, 1.0f, 1.0f, 1.0f }; //Stores specular light color.
        float m_specularPower{ 32.0f }; //Stores specular highlight power.
        bool m_shadowsEnabled{ true }; //Stores whether shadow mapping is enabled.
        float m_shadowDistance{ 70.0f }; //Stores the directional shadow half distance.
        float m_shadowDepthBias{ 0.004f }; //Stores depth bias used when sampling shadows.
        float m_shadowNormalBias{ 0.02f }; //Stores normal-based shadow bias.
        float m_shadowSoftness{ 1.5f }; //Stores shadow PCF sampling softness.
    };

    //Stores scene setup settings.
    struct SceneSetup
    {
        bool m_gridEnabled{ true }; //Stores whether the grid is shown.
        std::string m_gridMode{ "CheckerPlane" }; //Stores whether the grid is line or checker plane.
        std::string m_gridTexture{ "checker_blue.png" }; //Stores the checker grid texture file.
        bool m_lightMarkerEnabled{ true }; //Stores whether the light marker is shown.
        float m_lightMarkerSize{ 0.18f }; //Stores the light marker size.
        Float4 m_lightMarkerColor{ 1.0f, 0.95f, 0.2f, 1.0f }; //Stores the light marker color.
    };

    //Selects how reference and source image patches are compared.
    enum class PatchCostMethod
    {
        PatchCostMethodMeanAbsoluteDifference,
        PatchCostMethodNormalizedCrossCorrelation,
        PatchCostMethodZeroMeanNormalizedCrossCorrelation,
        PatchCostMethodRankTransform,
        PatchCostMethodCensusTransform,
        PatchCostMethodADCensus
    };

    enum class PlaneSweepExecutionMode
    {
        CpuOnly,
        Gpu,
        CpuGpuCompare
    };

    inline const char *PlaneSweepExecutionModeName(PlaneSweepExecutionMode mode)
    {
        switch (mode)
        {
        case PlaneSweepExecutionMode::Gpu: return "GPU";
        case PlaneSweepExecutionMode::CpuGpuCompare: return "CPU_GPU_COMPARE";
        default: return "CPU_ONLY";
        }
    }

    //Stores COLMAP camera visualization settings.
    struct ColmapSetup
    {
        bool m_enabled{ false }; //Stores whether COLMAP camera visualization is enabled.
        std::string m_cameraIntrinsicFile; //Stores the COLMAP camera intrinsics file.
        std::string m_cameraExtrinsicFile; //Stores the COLMAP camera extrinsics file.
        std::string m_cameraImageFolder; //Stores the folder containing COLMAP camera images.
        std::string m_cameraMaskFolder; //Stores the optional folder containing camera masks.
        std::string m_cameraDepthFolder; //Stores the folder containing metric camera depth images.
        std::string m_cameraPointsFile; //Stores the COLMAP points3D correspondence file.
        std::string m_pointDisplayMode{ "Auto" }; //Stores the COLMAP point display mode.
        float m_cameraSize{ 0.25f }; //Stores the displayed camera marker size.
        float m_positionScale{ 1.0f }; //Stores the scale applied to camera positions.
        Float3 m_baseTranslation{ 0.0f, 0.0f, 0.0f }; //Stores the base translation applied to the COLMAP visualization group.
        Float3 m_baseRotationDegrees{ 0.0f, 0.0f, 0.0f }; //Stores the base rotation applied to the COLMAP visualization group.
        float m_baseScale{ 1.0f }; //Stores the base scale applied to the COLMAP visualization group.
        int m_displayEveryNthCamera{ 1 }; //Stores the camera display sampling interval.
        int m_displayEveryNthPoint{ 1 }; //Stores the sparse point cloud sampling interval.
        int m_displayEveryNthPixel{ 50 }; //Stores the depth point cloud pixel sampling interval.
        bool m_planeSweepEnabled{ true }; //Stores whether plane sweep reconstruction is enabled.
        bool m_planeSweepUseHomographyWarping{ false }; //Enables undistorted homography warping for plane sweep.
        PlaneSweepExecutionMode m_planeSweepExecutionMode{ PlaneSweepExecutionMode::CpuOnly }; //Stores the PlaneSweep depth-search execution mode.
        int m_planeSweepReferenceStep{ 10 }; //Stores the plane sweep reference camera sampling interval.
        int m_planeSweepSourceCount{ 4 }; //Stores number of source cameras used per reference camera.
        int m_planeSweepPlaneCount{ 96 }; //Stores number of swept inverse-depth planes.
        float m_planeSweepImageScale{ 0.25f }; //Stores the image scale used for plane sweep.
        int m_planeSweepPatchRadius{ 1 }; //Stores the matching patch radius in pixels.
        PatchCostMethod m_planeSweepPatchCostMethod{ PatchCostMethod::PatchCostMethodMeanAbsoluteDifference }; //Selects the patch matching method.
        float m_planeSweepADCensusWeight{ 0.5f }; //Sets the Census weight in [0, 1] for AD-Census.
        int m_planeSweepMinValidSourceViews{ 2 }; //Stores minimum valid source views per depth sample.
        float m_planeSweepCostThreshold{ 0.18f }; //Stores maximum accepted matching cost.
        float m_planeSweepAlphaThreshold{ 0.01f }; //Stores maximum alpha treated as transparent.
        float m_planeSweepMinimumOpaquePatchRatio{ 0.75f }; //Stores minimum opaque patch sample ratio.
        bool m_planeSweepConfidenceEnabled{ true }; //Stores whether ambiguous depth matches are rejected.
        float m_planeSweepConfidenceMinimumDifference{ 0.02f }; //Stores minimum best and second-best cost difference.
        bool m_planeSweepTextureRejectionEnabled{ true }; //Stores whether textureless reference patches are rejected.
        float m_planeSweepTextureMinimumStdDev{ 0.03f }; //Stores minimum grayscale patch standard deviation.
        bool m_planeSweepCrossViewConsistencyEnabled{ true }; //Stores whether depth maps validate each other.
        int m_planeSweepCrossViewCount{ 4 }; //Stores maximum comparison depth maps per point.
        int m_planeSweepCrossViewMinimumViews{ 2 }; //Stores minimum agreeing comparison depth maps.
        float m_planeSweepCrossViewRelativeDepthTolerance{ 0.05f }; //Stores maximum relative depth difference for agreement.
        bool m_planeSweepPointFusionEnabled{ true }; //Stores whether nearby points are fused into voxels.
        float m_planeSweepFusionVoxelSize{ 0.02f }; //Stores voxel size used to fuse plane sweep points.
        int m_planeSweepFusionMinimumObservations{ 2 }; //Stores minimum observations required for a fused voxel.
        bool m_planeSweepOutlierFilterEnabled{ true }; //Stores whether isolated points are removed.
        float m_planeSweepOutlierRadius{ 0.05f }; //Stores neighbor search radius for outlier filtering.
        int m_planeSweepOutlierMinimumNeighbors{ 3 }; //Stores minimum nearby points needed to keep a point.
        float m_planeSweepNearDepth{ 0.1f }; //Stores fallback near depth.
        float m_planeSweepFarDepth{ 50.0f }; //Stores fallback far depth.
        std::string m_planeSweepDepthRangeMode{ "SparsePoints" }; //Stores plane sweep depth range mode.
        int m_planeSweepPointStep{ 2 }; //Stores plane sweep output pixel sampling step.
        std::string m_planeSweepOutputFolder{ "Output/PlaneSweep" }; //Stores plane sweep output folder.
        bool m_planeSweepSaveDepth{ true }; //Stores whether plane sweep depth files are saved.
        bool m_planeSweepExperimentMode{ false }; //Stores whether plane sweep experiment runner should run at startup.
        std::string m_depthScaleMode{ "Manual" }; //Stores the depth scale mode.
        float m_depthScaleValueMetricToColmap{ 1.0f }; //Stores the metric depth to COLMAP depth scale.
        std::string m_depthValueMode{ "CameraZ" }; //Stores the depth value convention.
        Float4 m_cameraColor{ 0.1f, 0.6f, 1.0f, 1.0f }; //Stores the camera marker color.
        bool m_showCameraLabels{ false }; //Stores whether camera id labels are shown.
        std::string m_coordinateMode{ "ColmapToApp" }; //Stores the coordinate conversion mode.

        bool m_planeSweepDebugEnabled{ false };

        int m_planeSweepDebugReferenceImageId{ 0 };

        int m_planeSweepDebugPixelX{ 0 };
        int m_planeSweepDebugPixelY{ 0 };

        bool m_planeSweepDebugSaveEveryPlane{ true };
        // Debug-only: enable COLMAP anchor point diagnostic experiment when plane sweep debug is enabled
        bool m_planeSweepDebugColmapAnchorEnabled{ false };
        // Debug-only: maximum pixel search radius when finding nearest COLMAP observation to requested debug pixel
        int m_planeSweepDebugColmapAnchorMaxPixelDistance{ 20 };
    };

    //Stores Gaussian splat visualization settings.
    struct GaussianSplatSetup
    {
        bool m_enabled{ false }; //Stores whether Gaussian splat visualization is enabled.
        std::string m_folder; //Stores the folder containing Gaussian splat PLY files.
        std::string m_filePattern{ "*.ply" }; //Stores the Gaussian splat file pattern.
        std::string m_loadMode{ "First" }; //Stores whether to load the first file or all files.
        std::string m_coordinateMode{ "ColmapToApp" }; //Stores the coordinate conversion mode.
        bool m_storeNormals{ false }; //Stores whether loaded splats keep PLY normals.
        bool m_storeSphericalHarmonics{ false }; //Stores whether loaded splats keep higher-order spherical harmonics.
        std::string m_colorMode{ "DCOnly" }; //Stores the Gaussian color mode.
        Float3 m_baseTranslation{ 0.0f, 0.0f, 0.0f }; //Stores the visualization base translation.
        Float3 m_baseRotationDegrees{ 0.0f, 0.0f, 0.0f }; //Stores the visualization base rotation.
        float m_baseScale{ 1.0f }; //Stores the visualization base scale.
        float m_positionScale{ 1.0f }; //Stores the scale applied to splat positions.
        int m_displayEveryNthSplat{ 1 }; //Stores the splat sampling interval.
        int m_maxSplats{ 0 }; //Stores the maximum number of splats to load, or zero for all.
        float m_streamFps{ 30.0f }; //Stores Gaussian stream playback frames per second.
        bool m_streamLoop{ true }; //Stores whether Gaussian stream playback loops after the last file.
        int m_streamPreloadCount{ 2 }; //Stores how many future Gaussian stream frames are decoded in advance.
        int m_streamWorkerThreadCount{ 1 }; //Stores how many background threads decode Gaussian stream frames.
        bool m_streamWaitForFrame{ false }; //Stores whether stream playback waits for an exact decoded frame.
        bool m_streamTimingLog{ false }; //Stores whether Gaussian stream timing logs are enabled.
        float m_streamTimingLogIntervalSeconds{ 1.0f }; //Stores the Gaussian stream timing summary interval.
        bool m_streamTimingLogEveryFrame{ false }; //Stores whether Gaussian stream timing logs print every frame event.
        float m_pointSize{ 3.0f }; //Stores the base point sprite size.
        float m_maxPointSize{ 1024.0f }; //Stores the maximum Gaussian splat point sprite size.
        float m_splatScale{ 1.0f }; //Stores the multiplier applied to PLY splat scale values.
        float m_opacityScale{ 1.0f }; //Stores the multiplier applied to PLY opacity values.
        bool m_sortEnabled{ true }; //Stores whether Gaussian splats are sorted back to front.
        bool m_sortEveryFrame{ true }; //Stores whether Gaussian splats are sorted every frame.
        bool m_depthTest{ true }; //Stores whether Gaussian splats use depth testing.
        bool m_depthWrite{ false }; //Stores whether Gaussian splats write depth.
        bool m_clickable{ false }; //Stores whether the splat object can be selected.
    };

    //Stores XR setup settings.
    struct XRSetup
    {
        bool m_enabled{ false }; //Stores whether OpenXR startup is requested.
        bool m_required{ false }; //Stores whether missing OpenXR support should stop the program.
        bool m_mirrorToWindow{ true }; //Stores whether the PC window should keep displaying frames.
        std::string m_mirrorMode{ "CenterEye" }; //Stores the PC mirror mode for future stereo rendering.
        std::string m_referenceSpace{ "LocalFloor" }; //Stores the preferred OpenXR reference space.
        float m_renderScale{ 1.0f }; //Stores the XR render resolution scale.
        bool m_useDepthLayer{ true }; //Stores whether XR depth composition should be used.
        bool m_controllerInput{ true }; //Stores whether OpenXR controller input should be used.
        bool m_passthroughEnabled{ true }; //Stores whether OpenXR passthrough should be used.
        bool m_passthroughDepthTestEnabled{ true }; //Stores whether passthrough environment depth testing is requested.
        std::string m_passthroughDepthMode{ "OpenXRMeta" }; //Stores the passthrough depth provider mode.
        std::string m_passthroughDepthDebugView{ "Off" }; //Stores the passthrough depth debug view mode.
        std::string m_passthroughDepthValueMode{ "Meters" }; //Stores how environment depth values are interpreted.
        std::string m_passthroughAlphaDebug{ "Off" }; //Stores the XR alpha debug mode.
        bool m_passthroughDepthTextureSampling{ false }; //Stores whether shaders may sample Meta depth textures.
        float m_passthroughDepthBiasMeters{ 0.03f }; //Stores passthrough depth bias in meters.
        float m_passthroughDepthMaxMeters{ 8.0f }; //Stores passthrough depth maximum distance in meters.
        bool m_passthroughDepthHandRemoval{ true }; //Stores whether environment depth removes hands.
        bool m_passthroughDepthLog{ true }; //Stores whether environment depth diagnostic logging is enabled.
    };

    //Stores UI setup settings.
    struct UISetup
    {
        float m_hudTextSize{ 2.5f }; //Stores bottom HUD text size.
        int m_maxVertices{ 8000000 }; //Stores maximum UI vertices per frame.
        int m_maxIndices{ 12000000 }; //Stores maximum UI indices per frame.
        Float4 m_textColor{ 1.0f, 1.0f, 1.0f, 1.0f }; //Stores UI text color.
        Float4 m_textShadowColor{ 0.0f, 0.0f, 0.0f, 0.65f }; //Stores UI text shadow color.
        Float4 m_glassPanelColor{ 0.72f, 0.88f, 1.0f, 0.42f }; //Stores glass panel base color.
        Float4 m_glassPanelTintColor{ 1.0f, 1.0f, 1.0f, 0.26f }; //Stores glass panel inner tint.
        Float4 m_glassBorderColor{ 0.82f, 0.96f, 1.0f, 0.58f }; //Stores glass panel border color.
        Float4 m_glassAccentColor{ 0.10f, 0.88f, 0.84f, 0.92f }; //Stores glass panel accent color.
        Float4 m_glassHeaderColor{ 0.04f, 0.07f, 0.12f, 0.78f }; //Stores glass panel header color.
        Float4 m_buttonColor{ 0.10f, 0.30f, 0.52f, 0.78f }; //Stores button color.
        Float4 m_buttonHoverColor{ 0.09f, 0.48f, 0.74f, 0.90f }; //Stores hovered button color.
        Float4 m_buttonPressedColor{ 0.06f, 0.20f, 0.42f, 0.90f }; //Stores pressed button color.
    };

    //Stores all program setup settings.
    struct ProgramSetup
    {
        WindowSetup m_window; //Stores window setup settings.
        GraphicsSetup m_graphics; //Stores graphics setup settings.
        PathSetup m_paths; //Stores centralized runtime path settings.
        ShaderSetup m_shaders; //Stores shader compilation setup settings.
        KeyboardSetup m_keyboard; //Stores keyboard setup settings.
        MouseSetup m_mouse; //Stores mouse setup settings.
        CameraSetup m_camera; //Stores camera setup settings.
        LightingSetup m_lighting; //Stores lighting setup settings.
        SceneSetup m_scene; //Stores scene setup settings.
        ColmapSetup m_colmap; //Stores COLMAP camera visualization settings.
        GaussianSplatSetup m_gaussianSplat; //Stores Gaussian splat visualization settings.
        XRSetup m_xr; //Stores OpenXR setup settings.
        UISetup m_ui; //Stores UI setup settings.
    };

    //Loads program setup values.
    ProgramSetup LoadProgramSetup(const std::string &fileName);
}

