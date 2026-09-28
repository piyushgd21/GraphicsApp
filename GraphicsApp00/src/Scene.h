///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	Scene.h
//	Description	:
//		Defines scene object ownership and rendering behavior.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "Camera.h"
#include "GaussianSplatStreamLoader.h"
#include "InputUtility.h"
#include "MultiViewUtility.h"
#include "SceneInterface.h"
#include "SceneObjectGaussianSplat.h"
#include "SceneObjectPointCloud.h"
#include "SetupUtility.h"
#include "UIUtility.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

//Defines scene object ownership and rendering behavior.
class Scene final : public graphicsFX::SceneInterface
{

    public:

        //Creates the scene.
        Scene(void) = default;

        //Destroys the scene.
        ~Scene(void) override = default;

		//do not allow copy constructor or assignment operator
        Scene(const Scene &) = delete;
        Scene &operator=(const Scene &) = delete;

        //Initializes scene objects and scene UI.
        void Init(const graphicsFX::SceneObjectContext &context, const graphicsFX::ProgramSetup &programSetup, graphicsFX::UIUtility &uiUtility) override;

        //Updates scene objects.
        void Update(float elapsedSeconds, float deltaSeconds) override;

        //Updates scene input state.
        void UpdateInput(
            const graphicsFX::InputUtility &inputUtility,
            float deltaSeconds,
            const graphicsFX::CameraMovementBasis *movementBasisPtr,
            const graphicsFX::ControllerMovementState *controllerStatePtr
        ) override;

        //Returns the active camera buffer.
        graphicsFX::CameraBuffer CameraBufferInfo(float aspect) const override;

        //Returns whether frame capture was requested.
        bool ConsumeCaptureFrameRequest(void) override;

        //Draws scene UI.
        void ShowUI(graphicsFX::UIUtility &uiUtility, const graphicsFX::UISetup &uiSetup, float framesPerSecond, VkExtent2D swapChainExtent) const override;

        //Returns the default world camera.
        const graphicsFX::Camera &Eye(void) const;

        //Draws scene objects.
        void Show(VkCommandBuffer commandBufferPtrGPU, VkDescriptorSet cameraDescriptorSetPtrGPU) const override;

        //Draws scene objects with an explicit render extent.
        void Show(VkCommandBuffer commandBufferPtrGPU, VkDescriptorSet cameraDescriptorSetPtrGPU, VkExtent2D renderExtent) const override;

        //Draws scene objects into the shadow map.
        void ShowShadow(VkCommandBuffer commandBufferPtrGPU, VkDescriptorSet cameraDescriptorSetPtrGPU, VkExtent2D shadowExtent) const override;

        //Releases scene object resources.
        void Clean(void) override;

        //Releases scene object pipelines.
        void CleanPipelines(void) override;

        //Recreates scene object pipelines.
        void RefreshPipelines(const graphicsFX::SceneObjectContext &context) override;

    private:

        //Stores transform gizmo modes.
        enum class TransformGizmoMode
        {
            Translate, Rotate, Scale
        };

        //Stores transform gizmo axes.
        enum class TransformGizmoAxis
        {
            None, X, Y, Z
        };

        //Defines available COLMAP point display modes.
        enum class ColmapPointDisplayMode
        {
            Auto, Points3D, Depth, PlaneSweep, Off
        };

        //Stores one 3D UI label in the scene.
        struct SceneTextLabelData
        {
            graphicsFX::Float3 m_position{ 0.0f, 0.0f, 0.0f }; //Stores the label world position.
            std::string m_text; //Stores the label text.
            int m_groupIndex{ -1 }; //Stores the owning group index.
        };

        //Stores selected scene object and optional instance index.
        struct SceneSelectionData
        {
            graphicsFX::SceneObject *m_objectPtr{ nullptr }; //Stores the selected scene object pointer.
            int m_instanceIndex{ -1 }; //Stores the selected instance index.
            int m_groupIndex{ -1 }; //Stores the selected group index.
        };

        //Stores Gaussian splat stream timing summary values.
        struct GaussianSplatStreamTimingStats
        {
            std::size_t m_loadedFrames{ 0U }; //Counts decoded frames consumed by the render thread.
            std::size_t m_appliedFrames{ 0U }; //Counts decoded frames applied to the scene object.
            std::size_t m_missedFrames{ 0U }; //Counts stream frame misses because decoded data was not ready.
            std::size_t m_sortCalls{ 0U }; //Counts timed Gaussian sort calls.
            std::size_t m_lastSplatCount{ 0U }; //Stores last applied splat count.
            double m_queueWaitMillisecondsTotal{ 0.0 }; //Stores total loader queue wait time.
            double m_queueWaitMillisecondsMax{ 0.0 }; //Stores maximum loader queue wait time.
            double m_loadMillisecondsTotal{ 0.0 }; //Stores total background load time.
            double m_loadMillisecondsMax{ 0.0 }; //Stores maximum background load time.
            double m_applyMillisecondsTotal{ 0.0 }; //Stores total render-thread apply time.
            double m_applyMillisecondsMax{ 0.0 }; //Stores maximum render-thread apply time.
            double m_sortMillisecondsTotal{ 0.0 }; //Stores total Gaussian sort time.
            double m_sortMillisecondsMax{ 0.0 }; //Stores maximum Gaussian sort time.
            float m_elapsedSeconds{ 0.0f }; //Stores elapsed seconds since last timing log.
        };

        //Adds simple cube objects to the scene.
        void DrawCubes(const graphicsFX::SceneObjectShaderFiles &shaderFiles);

        //Adds simple sphere objects to the scene.
        void DrawSpheres(const graphicsFX::SceneObjectShaderFiles &shaderFiles);

        //Adds textured cube objects to the scene.
        void DrawCubesWithTexture(const graphicsFX::SceneObjectShaderFiles &shaderFiles);

        //Adds textured sphere objects to the scene.
        void DrawSpheresWithTextures(const graphicsFX::SceneObjectShaderFiles &shaderFiles);

        //Adds instanced cube objects to the scene.
        void DrawCubesInstanced(const graphicsFX::SceneObjectShaderFiles &shaderFiles);

        //Adds instanced sphere objects to the scene.
        void DrawSpheresInstanced(const graphicsFX::SceneObjectShaderFiles &shaderFiles);

        //Adds instanced pyramid objects to the scene.
        void DrawPyramidsInstanced(const graphicsFX::SceneObjectShaderFiles &shaderFiles);

        //Adds a sample point cloud object to the scene.
        void DrawPointCloud(const graphicsFX::SceneObjectShaderFiles &shaderFiles);

        //Adds Gaussian splat PLY objects to the scene.
        void DrawGaussianSplats(const graphicsFX::ProgramSetup &programSetup, const graphicsFX::SceneObjectShaderFiles &shaderFiles);

        //Initializes Gaussian splat stream playback.
        void InitGaussianSplatStream(const graphicsFX::ProgramSetup &programSetup, const graphicsFX::SceneObjectShaderFiles &shaderFiles);

        //Updates Gaussian splat stream playback.
        void UpdateGaussianSplatStream(float deltaSeconds);

        //Loads one Gaussian splat stream frame.
        bool LoadGaussianSplatStreamFrame(std::size_t frameIndex);

        //Applies one decoded Gaussian splat stream frame.
        bool ApplyGaussianSplatStreamFrame(std::size_t frameIndex, graphicsFX::GaussianSplatReadResult gaussianData);

        //Requests upcoming Gaussian splat stream frames.
        void RequestGaussianSplatStreamFrames(std::size_t firstFrameIndex);

        //Records one decoded Gaussian stream frame timing sample.
        void RecordGaussianSplatStreamLoadTiming(const graphicsFX::GaussianSplatStreamFrameTiming &timing);

        //Records one render-thread Gaussian stream apply timing sample.
        void RecordGaussianSplatStreamApplyTiming(double applyMilliseconds, std::size_t splatCount);

        //Records one Gaussian stream frame miss.
        void RecordGaussianSplatStreamMiss(void);

        //Records one Gaussian sort timing sample.
        void RecordGaussianSplatStreamSortTiming(double sortMilliseconds);

        //Prints Gaussian stream timing when configured.
        void PrintGaussianSplatStreamTiming(float deltaSeconds);

        //Prints the current Gaussian stream timing summary.
        void PrintGaussianSplatStreamTimingSummary(void);

        //Clears Gaussian stream timing summary values.
        void ResetGaussianSplatStreamTiming(void);

        //Creates a Gaussian splat scene object.
        std::unique_ptr<graphicsFX::SceneObjectGaussianSplat> CreateGaussianSplatObject(
            std::vector<graphicsFX::GaussianSplatData> splats,
            const graphicsFX::GaussianSplatSetup &setup,
            const graphicsFX::SceneObjectShaderFiles &shaderFiles,
            const std::string &name
        ) const;

        //Adds the directional light marker to the scene.
        void DrawLightMarker(const graphicsFX::ProgramSetup &programSetup, const graphicsFX::SceneObjectShaderFiles &shaderFiles);

        //Adds COLMAP camera and point cloud objects to the scene.
        void DrawColmap(
            const graphicsFX::ProgramSetup &programSetup,
            const graphicsFX::SceneObjectShaderFiles &cameraShaderFiles,
            const graphicsFX::SceneObjectShaderFiles &pointCloudShaderFiles,
            const graphicsFX::ColmapCameraReadResult &colmapData
        );

        //Adds one COLMAP point cloud object to the scene.
        void AddColmapPointCloudObject(std::vector<graphicsFX::SceneObjectPointCloudPoint> points, const graphicsFX::SceneObjectShaderFiles &shaderFiles, float pointSize);

        //Adds the grid object to the scene.
        void DrawGrid(const graphicsFX::ProgramSetup &programSetup, const graphicsFX::SceneObjectShaderFiles &lineShaderFiles, const graphicsFX::SceneObjectShaderFiles &planeShaderFiles);

        //Adds the axis object to the scene.
        void DrawAxis(const graphicsFX::ProgramSetup &programSetup, const graphicsFX::SceneObjectShaderFiles &shaderFiles);

        //Draws the lower-right HUD axis widget.
        void DrawHudAxis(graphicsFX::UIUtility &uiUtility, const graphicsFX::UISetup &uiSetup, VkExtent2D swapChainExtent) const;

        //Prints the camera location and camera axes.
        void PrintCameraAndAxisLine(void) const;

        //Handles transform gizmo input.
        void UpdateTransformGizmoInput(const graphicsFX::InputUtility &inputUtility);

        //Selects an object near the current mouse position.
        void SelectObjectAtMouse(const graphicsFX::InputUtility &inputUtility);

        //Shows the selected object debug popup.
        void ShowSelectedObjectPopup(void);

        //Returns the selected object debug popup message.
        std::string SelectedObjectPopupMessage(void) const;

        //Updates the selected object debug popup text.
        void UpdateSelectedObjectPopup(void);

        //Returns whether a scene object is selected.
        bool SelectionActive(void) const;

        //Returns the selected transform position.
        graphicsFX::Float3 SelectedPosition(void) const;

        //Returns the selected transform orientation.
        graphicsFX::Float4x4 SelectedOrientation(void) const;

        //Returns the selected transform scale.
        float SelectedScale(void) const;

        //Returns the selected transform radius.
        float SelectedSelectionRadius(void) const;

        //Returns the selected gizmo display radius.
        float SelectedGizmoRadius(void) const;

        //Sets the selected transform position.
        void SetSelectedPosition(const graphicsFX::Float3 &position);

        //Sets the selected transform orientation.
        void SetSelectedOrientation(const graphicsFX::Float4x4 &orientation);

        //Sets the selected transform scale.
        void SetSelectedScale(float scaleValue);

        //Finds the group containing one scene object.
        int FindSceneObjectGroupIndex(const graphicsFX::SceneObject *sceneObjectPtr) const;

        //Returns the selected group pointer.
        graphicsFX::SceneObjectGroup *SelectedGroup(void);

        //Returns the selected group pointer.
        const graphicsFX::SceneObjectGroup *SelectedGroup(void) const;

        //Returns the object projected screen position.
        bool ProjectObjectToScreen(const graphicsFX::SceneObject &sceneObject, graphicsFX::Float2 &screenPosition, float &depth) const;

        //Returns the world point projected screen position.
        bool ProjectWorldPointToScreen(const graphicsFX::Float3 &position, graphicsFX::Float2 &screenPosition, float &depth) const;

        //Returns the transform axis direction.
        graphicsFX::Float3 TransformAxisDirection(TransformGizmoAxis axis) const;

        //Returns the active gizmo axis under the mouse.
        TransformGizmoAxis HitTestTransformGizmo(const graphicsFX::InputUtility &inputUtility) const;

        //Draws the transform gizmo.
        void DrawTransformGizmo(graphicsFX::UIUtility &uiUtility, const graphicsFX::UISetup &uiSetup, VkExtent2D swapChainExtent) const;

        //Initializes GPU resources for all scene objects.
        void InitSceneObjects(const graphicsFX::SceneObjectContext &context);

        //Creates one instanced scene object data record.
        graphicsFX::SceneObjectInstanceData CreateInstanceData(const graphicsFX::Float3 &position, float size, const graphicsFX::Float4 &color, const graphicsFX::Float4x4 &orientation) const;

        //Returns the configured COLMAP point display mode.
        static ColmapPointDisplayMode ParseColmapPointDisplayMode(std::string pointDisplayMode);

        //Advances one deterministic random state.
        static uint32_t NextRandomState(uint32_t &state);

        //Creates one deterministic random rotation angle.
        static float RandomAngle(uint32_t &state);

        //Creates one deterministic random orientation matrix.
        static graphicsFX::Float4x4 RandomOrientation(uint32_t seed);

        //Returns whether setup requests a line grid.
        static bool GridModeIsLine(std::string gridMode);

        //Returns an example object position lifted above the grid.
        static graphicsFX::Float3 LiftExamplePosition(const graphicsFX::Float3 &position);

        //Returns whether a ray intersects a sphere.
        static bool RaySphereHit(const graphicsFX::Float3 &origin, const graphicsFX::Float3 &direction, const graphicsFX::Float3 &center, float radius, float &hitDistance);

        //Returns the squared distance between two screen points.
        static float ScreenDistanceSquared(const graphicsFX::Float2 &a, const graphicsFX::Float2 &b);

        //Returns a normalized screen direction.
        static graphicsFX::Float2 NormalizeScreenDirection(const graphicsFX::Float2 &value);

        //Returns the display color for one axis.
        static graphicsFX::Float4 AxisColor(uint32_t axisIndex);

        //Returns a camera pyramid orientation matrix.
        static graphicsFX::Float4x4 CameraPyramidOrientation(const graphicsFX::ColmapCameraData &camera);

        //Returns the stable COLMAP visualization local center.
        static bool ColmapStableLocalCenter(const graphicsFX::ColmapCameraReadResult &colmapData, const graphicsFX::ColmapSetup &colmapSetup, graphicsFX::Float3 &center);

        //Returns the COLMAP visualization base rotation matrix.
        static graphicsFX::Float4x4 ColmapBaseRotationMatrix(const graphicsFX::ColmapSetup &colmapSetup);

        //Returns whether setup requests Gaussian splat streaming.
        static bool GaussianLoadModeIsStream(std::string loadMode);

        //Returns the Gaussian splat visualization base rotation matrix.
        static graphicsFX::Float4x4 GaussianSplatBaseRotationMatrix(const graphicsFX::GaussianSplatSetup &gaussianSplatSetup);

        //Returns an average timing value.
        static double AverageTiming(double totalMilliseconds, std::size_t sampleCount);

        static const uint32_t s_frameCaptureKey; //Stores the main keyboard slash key.
        static const uint32_t s_frameCaptureNumpadKey; //Stores the numpad slash key.
        static const uint32_t s_wireframeKey; //Stores the main keyboard period key.
        static const uint32_t s_wireframeNumpadKey; //Stores the numpad period key.
        static const uint32_t s_leftMouseButton; //Stores the left mouse button index.

        graphicsFX::Camera m_eye; //Stores the default world camera.
        graphicsFX::LightingSetup m_lightingSetup; //Stores scene lighting settings.
        bool m_captureFrameRequested{ false }; //Stores whether frame capture was requested.
        bool m_wireframeSupported{ false }; //Stores whether wireframe mode is supported.
        bool m_wireframeEnabled{ false }; //Stores whether wireframe rendering is enabled.
        bool m_sceneObjectsInitialized{ false }; //Stores whether scene objects have GPU resources.
        graphicsFX::UIUtility *m_uiUtilityPtr{ nullptr }; //Stores the scene UI utility pointer.
        SceneSelectionData m_selection; //Stores the selected scene object.
        uint32_t m_transformTranslateKey{ 'Z' }; //Stores the transform translate key.
        uint32_t m_transformRotateKey{ 'X' }; //Stores the transform rotate key.
        uint32_t m_transformScaleKey{ 'C' }; //Stores the transform scale key.
        TransformGizmoMode m_transformGizmoMode{ TransformGizmoMode::Translate }; //Stores active transform gizmo mode.
        TransformGizmoAxis m_activeTransformAxis{ TransformGizmoAxis::None }; //Stores the active transform axis.
        graphicsFX::Float2 m_transformStartMouse{ 0.0f, 0.0f }; //Stores drag start mouse position.
        graphicsFX::Float3 m_transformStartPosition{ 0.0f, 0.0f, 0.0f }; //Stores drag start object position.
        graphicsFX::Float4x4 m_transformStartOrientation{ graphicsFX::identity() }; //Stores drag start object orientation.
        float m_transformStartScale{ 1.0f }; //Stores drag start object scale.
        mutable VkExtent2D m_lastScreenExtent{}; //Stores the last known screen extent.

        std::vector<std::unique_ptr<graphicsFX::SceneObject>> m_sceneObjectPtrs; //Stores all scene object pointers.
        std::vector<SceneTextLabelData> m_sceneTextLabels; //Stores scene text labels projected through UI.
        std::vector<graphicsFX::SceneObjectGroup> m_sceneObjectGroups; //Stores selectable scene object groups.
        graphicsFX::SceneObject *m_colmapCameraObjectPtr{ nullptr }; //Stores the COLMAP camera object pointer.
        graphicsFX::SceneObject *m_colmapPointCloudObjectPtr{ nullptr }; //Stores the COLMAP point cloud object pointer.
        graphicsFX::SceneObjectGaussianSplat *m_gaussianSplatObjectPtr{ nullptr }; //Stores the Gaussian splat object pointer.
        graphicsFX::SceneObjectContext m_sceneObjectContext; //Stores the current scene object context.
        graphicsFX::GaussianSplatSetup m_gaussianSplatSetup; //Stores Gaussian splat setup for streaming.
        graphicsFX::SceneObjectShaderFiles m_gaussianSplatShaderFiles; //Stores Gaussian splat shader files for streaming.
        graphicsFX::GaussianSplatStreamLoader m_gaussianSplatStreamLoader; //Stores the asynchronous Gaussian stream loader.
        std::vector<std::filesystem::path> m_gaussianSplatStreamFiles; //Stores ordered Gaussian stream PLY files.
        std::size_t m_gaussianSplatStreamFrameIndex{ 0U }; //Stores the active Gaussian stream frame index.
        float m_gaussianSplatStreamAccumulator{ 0.0f }; //Stores elapsed stream playback time.
        GaussianSplatStreamTimingStats m_gaussianSplatStreamTimingStats; //Stores Gaussian stream timing summary values.
        bool m_gaussianSplatStreamEnabled{ false }; //Stores whether Gaussian stream playback is active.
        bool m_gaussianSplatStreamStopped{ false }; //Stores whether Gaussian stream playback stopped at the last frame.
    };
