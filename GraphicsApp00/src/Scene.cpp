///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	Scene.cpp
//	Description	:
//		Implements scene object ownership and rendering behavior.
//
//
///////////////////////////////////////////////////////////////////////

#include "Scene.h"

#include "GaussianSplatUtility.h"
#include "InputUtility.h"
#include "MultiViewUtility.h"
#include "PlaneSweepGpu.h"
#include "SceneObjectAxis.h"
#include "SceneObjectBox.h"
#include "SceneObjectGaussianSplat.h"
#include "SceneObjectGrid.h"
#include "SceneObjectGridPlane.h"
#include "SceneObjectPointCloud.h"
#include "SceneObjectPyramid.h"
#include "SceneObjectSphere.h"
#include "StringUtility.h"
#include "UIUtility.h"

#include <Windows.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace graphicsFX;

const uint32_t Scene::s_frameCaptureKey = VK_OEM_2; //Stores the main keyboard slash key.
const uint32_t Scene::s_frameCaptureNumpadKey = VK_DIVIDE; //Stores the numpad slash key.
const uint32_t Scene::s_wireframeKey = VK_OEM_PERIOD; //Stores the main keyboard period key.
const uint32_t Scene::s_wireframeNumpadKey = VK_DECIMAL; //Stores the numpad period key.
const uint32_t Scene::s_leftMouseButton = 0U; //Stores the left mouse button index.

//Initializes scene objects and scene UI.
void Scene::Init(const SceneObjectContext &context, const ProgramSetup &programSetup, UIUtility &uiUtility)
{
    const SceneObjectShaderFiles defaultShaders{ "shaders/default.vert", "shaders/default.frag" };
    const SceneObjectShaderFiles litShaders{ "shaders/defaultLit.vert", "shaders/defaultLit.frag" };
    const SceneObjectShaderFiles defaultInstancedShaders{ "shaders/defaultInstanced.vert", "shaders/default.frag" };
    const SceneObjectShaderFiles litInstancedShaders{ "shaders/defaultLitInstanced.vert", "shaders/defaultLit.frag" };
    const SceneObjectShaderFiles pointCloudShaders{ "shaders/pointCloud.vert", "shaders/pointCloud.frag" };
    const SceneObjectShaderFiles gaussianSplatShaders{ "shaders/gaussianSplat.vert", "shaders/gaussianSplat.frag" };

    Clean();

    m_sceneObjectContext = context;
    m_uiUtilityPtr = &uiUtility;
    m_eye.Init(programSetup.m_camera, programSetup.m_keyboard, programSetup.m_mouse);
    m_lightingSetup = programSetup.m_lighting;
    m_wireframeSupported = context.m_wireframeEnabled;
    m_wireframeEnabled = false;
    m_transformTranslateKey = programSetup.m_keyboard.m_transformTranslateKey;
    m_transformRotateKey = programSetup.m_keyboard.m_transformRotateKey;
    m_transformScaleKey = programSetup.m_keyboard.m_transformScaleKey;
    m_lastScreenExtent = context.m_swapChainExtent;

	//Temporarly disabled welcome popup
    //uiUtility.ShowPopup(
    //    "Info",
    //    "Welcome to UM-IHC Visualization Application",
    //    { "Ok", "Close" },
    //    [this](int buttonIndex)
    //    {
    //        UIUtility::Print("Button pressed " + std::to_string(buttonIndex) + "\n");
    //    });


    //uiUtility.ShowPopup(
    //    "Settings",
    //    "Adjust values.",
    //    {
    //        "Apply",
    //        "Cancel"
    //    },
    //    {
    //        UIUtility::PopupSlider{ "Speed", 0.0f, 10.0f, 4.0f },
    //        UIUtility::PopupSlider{ "Brightness", 0.0f, 1.0f, 0.5f }
    //    },
    //    [](int buttonIndex, const std::vector<UIUtility::PopupSlider> &sliders)
    //    {
    //        std::string sliderText = "Button pressed " + std::to_string(buttonIndex);
    //        for (const UIUtility::PopupSlider &slider : sliders)
    //        {
    //            sliderText += " | " + slider.m_label + " = " + std::to_string(slider.m_value);
    //        }
    //
    //        UIUtility::Print(sliderText + "\n");
    //    },
    //    [](int sliderIndex, float value)
    //    {
    //        UIUtility::Print("Slider changed " + std::to_string(sliderIndex) + " = " + std::to_string(value) + "\n");
    //    });

    //examples
	bool showBasicExamples = false;

    if (showBasicExamples)
    {
        DrawCubes(defaultShaders);
        DrawSpheres(defaultShaders);
        DrawCubesWithTexture(litShaders);
        DrawSpheresWithTextures(litShaders);
        DrawCubesInstanced(defaultInstancedShaders);
        DrawSpheresInstanced(litInstancedShaders);
        DrawPyramidsInstanced(litInstancedShaders);
        DrawPointCloud(pointCloudShaders);
    }

	//setup objects
    DrawLightMarker(programSetup, litShaders);
    DrawGrid(programSetup, defaultShaders, litShaders);

    //colmap objects
    const ColmapCameraReadResult colmapData = MultiViewUtility::LoadColmapCameras(programSetup.m_colmap);
    DrawColmap(programSetup, defaultInstancedShaders, pointCloudShaders, colmapData);

    std::vector<SceneObject *> colmapObjectPtrs;
    if (m_colmapCameraObjectPtr != nullptr)
    {
        colmapObjectPtrs.push_back(m_colmapCameraObjectPtr);
    }

    if (m_colmapPointCloudObjectPtr != nullptr)
    {
        colmapObjectPtrs.push_back(m_colmapPointCloudObjectPtr);
    }

    Float3 colmapLocalCenter{ 0.0f, 0.0f, 0.0f };
    const bool hasColmapLocalCenter = ColmapStableLocalCenter(colmapData, programSetup.m_colmap, colmapLocalCenter);

    SceneObjectGroup colmapGroup;
    if (colmapGroup.Init("ColmapData", colmapObjectPtrs, programSetup.m_colmap.m_baseTranslation, ColmapBaseRotationMatrix(programSetup.m_colmap), programSetup.m_colmap.m_baseScale,
        hasColmapLocalCenter ? &colmapLocalCenter : nullptr))
    {
        const int groupIndex = static_cast<int>(m_sceneObjectGroups.size());
        const Float3 localCenter = colmapGroup.m_localCenter;
        for (SceneTextLabelData &labelData : m_sceneTextLabels)
        {
            labelData.m_position = labelData.m_position - localCenter;
            labelData.m_groupIndex = groupIndex;
        }

        colmapGroup.Apply();
        m_sceneObjectGroups.push_back(std::move(colmapGroup));
    }

    if (GaussianLoadModeIsStream(programSetup.m_gaussianSplat.m_loadMode))
    {
        InitGaussianSplatStream(programSetup, gaussianSplatShaders);
    }
    else
    {
        DrawGaussianSplats(programSetup, gaussianSplatShaders);
    }

    InitSceneObjects(context);
    m_sceneObjectsInitialized = true;
}

//Updates scene objects.
void Scene::Update(float elapsedSeconds, float deltaSeconds)
{
    UpdateGaussianSplatStream(deltaSeconds);

    for (const auto &sceneObjectPtr : m_sceneObjectPtrs)
    {
        sceneObjectPtr->Update(elapsedSeconds, deltaSeconds);
    }

    if (m_gaussianSplatObjectPtr != nullptr)
    {
        if (m_gaussianSplatStreamEnabled && m_gaussianSplatSetup.m_streamTimingLog)
        {
            const auto sortStartTime = std::chrono::steady_clock::now();
            m_gaussianSplatObjectPtr->UpdateSort(m_eye.Position());
            const auto sortEndTime = std::chrono::steady_clock::now();
            const double sortMilliseconds = std::chrono::duration<double, std::milli>(sortEndTime - sortStartTime).count();
            RecordGaussianSplatStreamSortTiming(sortMilliseconds);
        }
        else
        {
            m_gaussianSplatObjectPtr->UpdateSort(m_eye.Position());
        }
    }

    PrintGaussianSplatStreamTiming(deltaSeconds);
}

//Updates scene input state.
void Scene::UpdateInput(const InputUtility &inputUtility, float deltaSeconds, const CameraMovementBasis *movementBasisPtr, const ControllerMovementState *controllerStatePtr)
{
    m_eye.UpdateInput(inputUtility, deltaSeconds, movementBasisPtr, controllerStatePtr);

    if (inputUtility.KeyPressed(s_frameCaptureKey) || inputUtility.KeyPressed(s_frameCaptureNumpadKey))
    {
        m_captureFrameRequested = true;
        PrintCameraAndAxisLine();
    }

    if (inputUtility.KeyPressed(s_wireframeKey) || inputUtility.KeyPressed(s_wireframeNumpadKey))
    {
        if (m_wireframeSupported)
        {
            m_wireframeEnabled = !m_wireframeEnabled;
            UIUtility::Print(m_wireframeEnabled ? "Wireframe mode on.\n" : "Wireframe mode off.\n");
        }
        else
        {
            UIUtility::Print("Wireframe mode is not supported by this Vulkan device.\n");
        }
    }

    UpdateTransformGizmoInput(inputUtility);
}

//Returns the active camera buffer.
CameraBuffer Scene::CameraBufferInfo(float aspect) const
{
    //Returns a safe normalized light direction.
    auto NormalizeDirection = [](const Float3 &direction) -> Float3
    {
        const float lengthSquared = dot(direction, direction);
        if (lengthSquared <= 0.000001f)
        {
            return { -0.5f, -0.7f, -0.5f };
        }

        return direction * (1.0f / std::sqrt(lengthSquared));
    };

    const Float3 lightDirection = NormalizeDirection(m_lightingSetup.m_direction);
    const float shadowDistance = std::max(1.0f, m_lightingSetup.m_shadowDistance);
    const Float3 shadowCenter{ 0.0f, 0.0f, 0.0f };
    const Float3 lightPosition = m_lightingSetup.m_position;
    const Float3 worldUp{ 0.0f, 1.0f, 0.0f };
    const Float3 lightUp = std::abs(dot(lightDirection, worldUp)) > 0.95f ? Float3{ 1.0f, 0.0f, 0.0f } : worldUp;
    const Float4x4 lightView = lookAt(lightPosition, shadowCenter, lightUp);
    const Float4x4 lightProjection = orthographic(-shadowDistance, shadowDistance, -shadowDistance, shadowDistance, 0.1f, shadowDistance * 3.0f);

    CameraBuffer camera{};
    camera.m_view = m_eye.ViewMatrix();
    camera.m_projection = m_eye.ProjectionMatrix(aspect);
    camera.m_cameraPosition = { m_eye.Position().x, m_eye.Position().y, m_eye.Position().z, 1.0f };
    camera.m_lightDirection = { lightDirection.x, lightDirection.y, lightDirection.z, 0.0f };
    camera.m_ambientColor = m_lightingSetup.m_ambientColor;
    camera.m_diffuseColor = m_lightingSetup.m_diffuseColor;
    camera.m_specularColor = m_lightingSetup.m_specularColor;
    camera.m_lightSettings = { m_lightingSetup.m_specularPower, 0.0f, 0.0f, 0.0f };
    camera.m_lightViewProjection = multiply(lightProjection, lightView);
    camera.m_shadowSettings = {
        m_lightingSetup.m_shadowsEnabled ? 1.0f : 0.0f, m_lightingSetup.m_shadowDepthBias, m_lightingSetup.m_shadowNormalBias, m_lightingSetup.m_shadowSoftness
    };
    return camera;
}

//Returns whether frame capture was requested.
bool Scene::ConsumeCaptureFrameRequest(void)
{
    const bool captureFrameRequested = m_captureFrameRequested;
    m_captureFrameRequested = false;
    return captureFrameRequested;
}

//Draws scene UI.
void Scene::ShowUI(UIUtility &uiUtility, const UISetup &uiSetup, float framesPerSecond, VkExtent2D swapChainExtent) const
{
    m_lastScreenExtent = swapChainExtent;

    const std::string hudLine = "FPS: " + StringUtility::FormatFloat(framesPerSecond, 1) + " | Pos: " + StringUtility::FormatFloat3(m_eye.Position()) +
        " | LookAt: " + StringUtility::FormatFloat3(m_eye.Forward()) + " | Up: " + StringUtility::FormatFloat3(m_eye.Up()) + " | Right: " + StringUtility::FormatFloat3(m_eye.Right())
        ;

    const float hudX = 14.0f;
    const float hudTextSize = uiSetup.m_hudTextSize;
    const float hudY = std::max(4.0f, static_cast<float>(swapChainExtent.height) - (hudTextSize * 24.0f));
    uiUtility.DrawText(hudX + 1.0f, hudY + 1.0f, hudLine, uiSetup.m_textShadowColor, hudTextSize);
    uiUtility.DrawText(hudX, hudY, hudLine, uiSetup.m_textColor, hudTextSize);

    const float aspect = static_cast<float>(swapChainExtent.width) / std::max(1.0f, static_cast<float>(swapChainExtent.height));
    const CameraBuffer cameraBuffer = CameraBufferInfo(aspect);
    const float labelTextSize = std::max(1.35f, uiSetup.m_hudTextSize * 0.72f);
    for (const SceneTextLabelData &labelData : m_sceneTextLabels)
    {
        Float3 labelPosition = labelData.m_position;
        if (labelData.m_groupIndex >= 0 && static_cast<std::size_t>(labelData.m_groupIndex) < m_sceneObjectGroups.size())
        {
            labelPosition = m_sceneObjectGroups[static_cast<std::size_t>(labelData.m_groupIndex)].LocalToWorld(labelData.m_position);
        }

        uiUtility.DrawText3D(labelData.m_text, labelPosition, cameraBuffer.m_view, cameraBuffer.m_projection, { 9.0f, -9.0f }, uiSetup.m_textShadowColor, labelTextSize);
        uiUtility.DrawText3D(labelData.m_text, labelPosition, cameraBuffer.m_view, cameraBuffer.m_projection, { 8.0f, -8.0f }, uiSetup.m_textColor, labelTextSize);
    }
    DrawHudAxis(uiUtility, uiSetup, swapChainExtent);
    DrawTransformGizmo(uiUtility, uiSetup, swapChainExtent);
}

//Returns the default world camera.
const Camera &Scene::Eye(void) const
{
    return m_eye;
}

//Draws scene objects.
void Scene::Show(VkCommandBuffer commandBufferPtrGPU, VkDescriptorSet cameraDescriptorSetPtrGPU) const
{
    for (const auto &sceneObjectPtr : m_sceneObjectPtrs)
    {
        sceneObjectPtr->Show(commandBufferPtrGPU, cameraDescriptorSetPtrGPU, m_wireframeEnabled);
    }
}

//Draws scene objects with an explicit render extent.
void Scene::Show(VkCommandBuffer commandBufferPtrGPU, VkDescriptorSet cameraDescriptorSetPtrGPU, VkExtent2D renderExtent) const
{
    for (const auto &sceneObjectPtr : m_sceneObjectPtrs)
    {
        sceneObjectPtr->Show(commandBufferPtrGPU, cameraDescriptorSetPtrGPU, renderExtent, m_wireframeEnabled);
    }
}

//Draws scene objects into the shadow map.
void Scene::ShowShadow(VkCommandBuffer commandBufferPtrGPU, VkDescriptorSet cameraDescriptorSetPtrGPU, VkExtent2D shadowExtent) const
{
    for (const auto &sceneObjectPtr : m_sceneObjectPtrs)
    {
        sceneObjectPtr->ShowShadow(commandBufferPtrGPU, cameraDescriptorSetPtrGPU, shadowExtent);
    }
}

//Releases scene object resources.
void Scene::Clean(void)
{
    m_gaussianSplatStreamLoader.Stop();

    for (const auto &sceneObjectPtr : m_sceneObjectPtrs)
    {
        sceneObjectPtr->Clean();
    }

    m_sceneObjectPtrs.clear();
    m_sceneTextLabels.clear();
    m_sceneObjectGroups.clear();
    m_colmapCameraObjectPtr = nullptr;
    m_colmapPointCloudObjectPtr = nullptr;
    m_gaussianSplatObjectPtr = nullptr;
    m_sceneObjectsInitialized = false;
    m_sceneObjectContext = {};
    m_gaussianSplatSetup = {};
    m_gaussianSplatShaderFiles = {};
    m_gaussianSplatStreamFiles.clear();
    m_gaussianSplatStreamFrameIndex = 0U;
    m_gaussianSplatStreamAccumulator = 0.0f;
    m_gaussianSplatStreamEnabled = false;
    m_gaussianSplatStreamStopped = false;
    m_uiUtilityPtr = nullptr;
    m_selection = {};
    m_activeTransformAxis = TransformGizmoAxis::None;
}

//Releases scene object pipelines.
void Scene::CleanPipelines(void)
{
    for (const auto &sceneObjectPtr : m_sceneObjectPtrs)
    {
        sceneObjectPtr->CleanPipeline();
    }
}

//Recreates scene object pipelines.
void Scene::RefreshPipelines(const SceneObjectContext &context)
{
    m_sceneObjectContext = context;
    m_wireframeSupported = context.m_wireframeEnabled;
    if (!m_wireframeSupported)
    {
        m_wireframeEnabled = false;
    }

    for (const auto &sceneObjectPtr : m_sceneObjectPtrs)
    {
        sceneObjectPtr->RefreshPipeline(context);
    }
}
//Adds simple cube objects to the scene.
void Scene::DrawCubes(const SceneObjectShaderFiles &shaderFiles)
{
    //Stores default transform and color settings for one scene object.
    struct SceneObjectData
    {
        Float3 m_position; //Stores the object position.
        Float3 m_boxSize; //Stores the box size.
        Float4 m_color; //Stores the object color.
    };

    SceneObjectRenderOptions renderOptions;
    renderOptions.m_selectionMode = SceneObjectSelectionMode::PerObject;

    const std::array<SceneObjectData, 1> cubeDataList = {
        SceneObjectData{ { -1.25f, 0.625f, -0.8f }, { 1.35f, 1.25f, 1.05f }, { 1.0f, 1.0f, 1.0f, 1.0f } }
    };

    for (const SceneObjectData &cube : cubeDataList)
    {
        auto cubeObjectPtr = std::make_unique<SceneObjectBox>();
        cubeObjectPtr->SetName("Cube");
        cubeObjectPtr->Init(LiftExamplePosition(cube.m_position), cube.m_boxSize, identity(), cube.m_color, shaderFiles, "", renderOptions);
        m_sceneObjectPtrs.push_back(std::move(cubeObjectPtr));
    }
}

//Adds simple sphere objects to the scene.
void Scene::DrawSpheres(const SceneObjectShaderFiles &shaderFiles)
{
    //Stores default transform and color settings for one scene object.
    struct SceneObjectData
    {
        Float3 m_position; //Stores the object position.
        float m_size; //Stores the object size.
        Float4 m_color; //Stores the object color.
    };

    SceneObjectRenderOptions renderOptions;
    renderOptions.m_selectionMode = SceneObjectSelectionMode::PerObject;

    const std::array<SceneObjectData, 1> sphereDataList = {
        SceneObjectData{ { 1.25f, 0.35f, -0.8f }, 0.7f, { 1.0f, 0.3f, 0.25f, 1.0f } }
    };

    for (const SceneObjectData &sphere : sphereDataList)
    {
        auto sphereObjectPtr = std::make_unique<SceneObjectSphere>();
        sphereObjectPtr->SetName("Sphere");
        sphereObjectPtr->Init(LiftExamplePosition(sphere.m_position), sphere.m_size * 0.5f, 16U, 24U, sphere.m_color, shaderFiles, "", "", true, renderOptions);
        m_sceneObjectPtrs.push_back(std::move(sphereObjectPtr));
    }
}

//Adds textured cube objects to the scene.
void Scene::DrawCubesWithTexture(const SceneObjectShaderFiles &shaderFiles)
{
    //Stores default transform, color, and texture settings for one cube.
    struct TexturedCubeData
    {
        Float3 m_position; //Stores the object position.
        Float3 m_boxSize; //Stores the box size.
        Float4 m_color; //Stores the object color.
        std::string m_textureFileName; //Stores the texture file name.
    };

    SceneObjectRenderOptions renderOptions;
    renderOptions.m_selectionMode = SceneObjectSelectionMode::PerObject;

    const std::array<TexturedCubeData, 4> texturedCubeDataList = {
        TexturedCubeData{ { -4.5f, 0.425f, -2.4f }, { 0.85f, 0.85f, 0.85f }, { 1.0f, 1.0f, 1.0f, 1.0f }, "checker_red.png" },
        TexturedCubeData{ { -1.5f, 0.425f, -2.4f }, { 0.95f, 0.75f, 0.85f }, { 1.0f, 1.0f, 1.0f, 1.0f }, "checker_blue.png" },
        TexturedCubeData{ {  1.5f, 0.425f, -2.4f }, { 0.8f, 0.95f, 0.85f }, { 1.0f, 1.0f, 1.0f, 1.0f }, "stripes_green.png" },
        TexturedCubeData{ {  4.5f, 0.425f, -2.4f }, { 0.85f, 0.8f, 1.0f }, { 1.0f, 1.0f, 1.0f, 1.0f }, "grid_yellow.png" }, };

    uint32_t cubeIndex = 0;
    for (const TexturedCubeData &cube : texturedCubeDataList)
    {
        auto cubeObjectPtr = std::make_unique<SceneObjectBox>();
        cubeObjectPtr->SetName("CubeWithTexture" + std::to_string(cubeIndex));
        cubeObjectPtr->Init(LiftExamplePosition(cube.m_position), cube.m_boxSize, RandomOrientation(1000U + cubeIndex), cube.m_color, shaderFiles, cube.m_textureFileName, renderOptions);
        m_sceneObjectPtrs.push_back(std::move(cubeObjectPtr));
        ++cubeIndex;
    }
}

//Adds textured sphere objects to the scene.
void Scene::DrawSpheresWithTextures(const SceneObjectShaderFiles &shaderFiles)
{
    //Stores default transform, color, texture, and normal settings for one sphere.
    struct TexturedSphereData
    {
        Float3 m_position; //Stores the object position.
        float m_radius; //Stores the sphere radius.
        Float4 m_color; //Stores the object color.
        std::string m_textureFileName; //Stores the diffuse texture file name.
        std::string m_normalMapFileName; //Stores the normal map file name.
    };

    SceneObjectRenderOptions renderOptions;
    renderOptions.m_selectionMode = SceneObjectSelectionMode::PerObject;

    const std::array<TexturedSphereData, 4> texturedSphereDataList = {
        TexturedSphereData{ { -4.5f, 0.55f, -4.0f }, 0.55f, { 1.0f, 1.0f, 1.0f, 1.0f }, "checker_red.png", "normal_soft.png" },
        TexturedSphereData{ { -1.5f, 0.55f, -4.0f }, 0.55f, { 1.0f, 1.0f, 1.0f, 1.0f }, "checker_blue.png", "normal_tiles.png" },
        TexturedSphereData{ {  1.5f, 0.55f, -4.0f }, 0.55f, { 1.0f, 1.0f, 1.0f, 1.0f }, "stripes_green.png", "normal_ridges.png" },
        TexturedSphereData{ {  4.5f, 0.55f, -4.0f }, 0.55f, { 1.0f, 1.0f, 1.0f, 1.0f }, "grid_yellow.png", "normal_waves.png" }, };

    uint32_t sphereIndex = 0;
    for (const TexturedSphereData &sphere : texturedSphereDataList)
    {
        auto sphereObjectPtr = std::make_unique<SceneObjectSphere>();
        sphereObjectPtr->SetName("SphereWithTexture" + std::to_string(sphereIndex));
        sphereObjectPtr->Init(
            LiftExamplePosition(sphere.m_position),
            sphere.m_radius,
            24U,
            32U,
            sphere.m_color,
            shaderFiles,
            sphere.m_textureFileName,
            sphere.m_normalMapFileName,
            true,
            renderOptions
        );
        sphereObjectPtr->SetOrientation(RandomOrientation(2000U + sphereIndex));
        m_sceneObjectPtrs.push_back(std::move(sphereObjectPtr));
        ++sphereIndex;
    }
}

//Adds instanced cube objects to the scene.
void Scene::DrawCubesInstanced(const SceneObjectShaderFiles &shaderFiles)
{
    //Stores default transform and color settings for one scene object.
    struct SceneObjectData
    {
        Float3 m_position; //Stores the object position.
        float m_size; //Stores the object size.
        Float4 m_color; //Stores the object color.
    };

    const std::array<SceneObjectData, 4> cubeDataList = {
        SceneObjectData{ { -3.6f, 0.425f, -5.6f }, 0.85f, { 1.0f, 0.55f, 0.55f, 1.0f } }, SceneObjectData{ { -1.2f, 0.425f, -5.6f }, 0.85f, { 0.55f, 0.9f, 1.0f, 1.0f } },
        SceneObjectData{ {  1.2f, 0.325f, -5.6f }, 0.65f, { 0.8f, 1.0f, 0.6f, 1.0f } }, SceneObjectData{ {  3.6f, 0.325f, -5.6f }, 0.65f, { 1.0f, 0.75f, 1.0f, 1.0f } }, };

    std::vector<SceneObjectInstanceData> cubeInstanceData;
    cubeInstanceData.reserve(cubeDataList.size());
    uint32_t cubeIndex = 0;
    for (const SceneObjectData &cube : cubeDataList)
    {
        cubeInstanceData.push_back(CreateInstanceData(LiftExamplePosition(cube.m_position), cube.m_size, cube.m_color, RandomOrientation(3000U + cubeIndex)));
        ++cubeIndex;
    }

    auto instancedCubeObjectPtr = std::make_unique<SceneObjectBox>();
    instancedCubeObjectPtr->SetName("CubesInstanced");
    SceneObjectRenderOptions renderOptions;
    renderOptions.m_selectionMode = SceneObjectSelectionMode::PerInstance;
    instancedCubeObjectPtr->Init(std::move(cubeInstanceData), { 1.0f, 1.0f, 1.0f }, shaderFiles, "", renderOptions);
    m_sceneObjectPtrs.push_back(std::move(instancedCubeObjectPtr));
}

//Adds instanced sphere objects to the scene.
void Scene::DrawSpheresInstanced(const SceneObjectShaderFiles &shaderFiles)
{
    //Stores default transform and color settings for one scene object.
    struct SceneObjectData
    {
        Float3 m_position; //Stores the object position.
        float m_size; //Stores the object size.
        Float4 m_color; //Stores the object color.
    };

    const std::array<SceneObjectData, 3> sphereDataList = {
        SceneObjectData{ { -2.4f, 0.31f, -7.2f }, 0.62f, { 0.2f, 0.75f, 1.0f, 1.0f } }, SceneObjectData{ {  0.0f, 0.31f, -7.2f }, 0.62f, { 0.35f, 0.95f, 0.35f, 1.0f } },
        SceneObjectData{ {  2.4f, 0.35f, -7.2f }, 0.7f, { 1.0f, 0.65f, 0.15f, 1.0f } }, };

    std::vector<SceneObjectInstanceData> sphereInstanceData;
    sphereInstanceData.reserve(sphereDataList.size());
    uint32_t sphereIndex = 0;
    for (const SceneObjectData &sphere : sphereDataList)
    {
        sphereInstanceData.push_back(CreateInstanceData(LiftExamplePosition(sphere.m_position), sphere.m_size * 0.5f, sphere.m_color, RandomOrientation(4000U + sphereIndex)));
        ++sphereIndex;
    }

    auto instancedSphereObjectPtr = std::make_unique<SceneObjectSphere>();
    instancedSphereObjectPtr->SetName("SphereInstanced");
    SceneObjectRenderOptions renderOptions;
    renderOptions.m_selectionMode = SceneObjectSelectionMode::PerInstance;
    instancedSphereObjectPtr->Init(std::move(sphereInstanceData), 16U, 24U, shaderFiles, "", "", true, renderOptions);
    m_sceneObjectPtrs.push_back(std::move(instancedSphereObjectPtr));
}

//Adds instanced pyramid objects to the scene.
void Scene::DrawPyramidsInstanced(const SceneObjectShaderFiles &shaderFiles)
{
    //Stores default transform and color settings for one scene object.
    struct SceneObjectData
    {
        Float3 m_position; //Stores the object position.
        float m_size; //Stores the object size.
        Float4 m_color; //Stores the object color.
    };

    const std::array<SceneObjectData, 5> pyramidDataList = {
        SceneObjectData{ { -4.0f, 0.28f, -8.8f }, 0.8f, { 1.0f, 0.35f, 0.25f, 1.0f } }, SceneObjectData{ { -2.0f, 0.28f, -8.8f }, 0.8f, { 0.25f, 0.75f, 1.0f, 1.0f } },
        SceneObjectData{ {  0.0f, 0.28f, -8.8f }, 0.8f, { 0.45f, 1.0f, 0.45f, 1.0f } }, SceneObjectData{ {  2.0f, 0.28f, -8.8f }, 0.8f, { 1.0f, 0.85f, 0.25f, 1.0f } },
        SceneObjectData{ {  4.0f, 0.28f, -8.8f }, 0.8f, { 0.95f, 0.45f, 1.0f, 1.0f } }, };

    std::vector<SceneObjectInstanceData> pyramidInstanceData;
    pyramidInstanceData.reserve(pyramidDataList.size());
    uint32_t pyramidIndex = 0;
    for (const SceneObjectData &pyramid : pyramidDataList)
    {
        pyramidInstanceData.push_back(CreateInstanceData(LiftExamplePosition(pyramid.m_position), pyramid.m_size, pyramid.m_color, RandomOrientation(5000U + pyramidIndex)));
        ++pyramidIndex;
    }

    auto instancedPyramidObjectPtr = std::make_unique<SceneObjectPyramid>();
    instancedPyramidObjectPtr->SetName("PyramidInstanced");
    SceneObjectRenderOptions renderOptions;
    renderOptions.m_selectionMode = SceneObjectSelectionMode::PerInstance;
    instancedPyramidObjectPtr->Init(std::move(pyramidInstanceData), { 0.0f, 0.0f, -1.0f }, 0.7f, 1.0f, shaderFiles, renderOptions);
    m_sceneObjectPtrs.push_back(std::move(instancedPyramidObjectPtr));
}

//Adds a sample point cloud object to the scene.
void Scene::DrawPointCloud(const SceneObjectShaderFiles &shaderFiles)
{
    std::vector<SceneObjectPointCloudPoint> points;
    points.reserve(81);

    for (int zIndex = -4; zIndex <= 4; ++zIndex)
    {
        for (int xIndex = -4; xIndex <= 4; ++xIndex)
        {
            const float x = static_cast<float>(xIndex) * 0.35f;
            const float z = static_cast<float>(zIndex) * 0.35f;
            const float y = 0.18f * std::sin(x * 2.0f) * std::cos(z * 2.0f);
            const float red = 0.35f + (static_cast<float>(xIndex + 4) / 8.0f) * 0.55f;
            const float blue = 0.35f + (static_cast<float>(zIndex + 4) / 8.0f) * 0.55f;

            points.push_back(SceneObjectPointCloudPoint{ { x, y, z }, { red, 0.85f, blue, 1.0f } });
        }
    }

    SceneObjectRenderOptions renderOptions;
    renderOptions.m_castShadow = false;
    renderOptions.m_receiveShadow = false;
    renderOptions.m_clickable = false;

    auto pointCloudObjectPtr = std::make_unique<SceneObjectPointCloud>();
    pointCloudObjectPtr->SetName("PointCloudSample");
    pointCloudObjectPtr->Init(std::move(points), { -2.0f, 1.1f, -10.2f }, identity(), 1.0f, 6.0f, { 1.0f, 1.0f, 1.0f, 1.0f }, shaderFiles, renderOptions);
    m_sceneObjectPtrs.push_back(std::move(pointCloudObjectPtr));
}

//Adds Gaussian splat PLY objects to the scene.
void Scene::DrawGaussianSplats(const ProgramSetup &programSetup, const SceneObjectShaderFiles &shaderFiles)
{
    if (!programSetup.m_gaussianSplat.m_enabled)
    {
        return;
    }

    GaussianSplatReadResult gaussianData = GaussianSplatUtility::LoadFolder(programSetup.m_gaussianSplat);
    if (!gaussianData.m_errorMessage.empty())
    {
        UIUtility::Print(gaussianData.m_errorMessage + "\n");
    }

    if (gaussianData.m_splats.empty())
    {
        UIUtility::Print("Loaded 0 Gaussian splats.\n");
        return;
    }
    const std::size_t splatCount = gaussianData.m_splats.size();

    auto gaussianObjectPtr = CreateGaussianSplatObject(std::move(gaussianData.m_splats), programSetup.m_gaussianSplat, shaderFiles, "GaussianSplats");
    m_gaussianSplatObjectPtr = gaussianObjectPtr.get();
    m_sceneObjectPtrs.push_back(std::move(gaussianObjectPtr));

    UIUtility::Print("Loaded " + std::to_string(splatCount) + " Gaussian splats from " + std::to_string(gaussianData.m_filesRead) + " file(s).\n");
}

//Initializes Gaussian splat stream playback.
void Scene::InitGaussianSplatStream(const ProgramSetup &programSetup, const SceneObjectShaderFiles &shaderFiles)
{
    if (!programSetup.m_gaussianSplat.m_enabled)
    {
        return;
    }

    m_gaussianSplatSetup = programSetup.m_gaussianSplat;
    m_gaussianSplatShaderFiles = shaderFiles;
    m_gaussianSplatStreamAccumulator = 0.0f;
    m_gaussianSplatStreamFrameIndex = 0U;
    m_gaussianSplatStreamStopped = false;
    ResetGaussianSplatStreamTiming();

    std::string errorMessage;
    m_gaussianSplatStreamFiles = GaussianSplatUtility::FindFiles(m_gaussianSplatSetup, errorMessage);
    if (m_gaussianSplatStreamFiles.empty())
    {
        UIUtility::Print((errorMessage.empty() ? "No Gaussian splat stream PLY files were found." : errorMessage) + "\n");
        return;
    }

    const float streamFps = std::max(0.001f, m_gaussianSplatSetup.m_streamFps);
    UIUtility::Print(
        "Gaussian stream found " + std::to_string(m_gaussianSplatStreamFiles.size()) + " file(s) at " + StringUtility::FormatFloat(streamFps, 1) + " FPS. " +
        (m_gaussianSplatSetup.m_streamLoop ? "Loop enabled.\n" : "Stop at end enabled.\n")
    );
    if (m_gaussianSplatSetup.m_streamTimingLog)
    {
        UIUtility::Print("Gaussian stream timing log enabled.\n");
    }

    m_gaussianSplatStreamEnabled = LoadGaussianSplatStreamFrame(0U);
    if (m_gaussianSplatStreamEnabled)
    {
        UIUtility::Print("Gaussian stream loaded first frame: " + m_gaussianSplatStreamFiles.front().filename().string() + "\n");
        m_gaussianSplatStreamLoader.Start(m_gaussianSplatStreamFiles, m_gaussianSplatSetup, m_gaussianSplatSetup.m_streamPreloadCount);
        RequestGaussianSplatStreamFrames(1U);
    }
}

//Updates Gaussian splat stream playback.
void Scene::UpdateGaussianSplatStream(float deltaSeconds)
{
    if (!m_gaussianSplatStreamEnabled || m_gaussianSplatStreamStopped || m_gaussianSplatStreamFiles.size() <= 1U)
    {
        return;
    }

    const float streamFps = std::max(0.001f, m_gaussianSplatSetup.m_streamFps);
    const float frameDuration = 1.0f / streamFps;
    m_gaussianSplatStreamAccumulator += std::max(0.0f, deltaSeconds);
    if (m_gaussianSplatStreamAccumulator < frameDuration)
    {
        return;
    }

    std::size_t nextFrameIndex = m_gaussianSplatStreamFrameIndex + 1U;
    if (nextFrameIndex >= m_gaussianSplatStreamFiles.size())
    {
        if (!m_gaussianSplatSetup.m_streamLoop)
        {
            m_gaussianSplatStreamStopped = true;
            UIUtility::Print("Gaussian stream stopped at last frame.\n");
            return;
        }

        nextFrameIndex = 0U;
        UIUtility::Print("Gaussian stream looped to first frame.\n");
    }

    RequestGaussianSplatStreamFrames(nextFrameIndex);

    GaussianSplatReadResult gaussianData;
    GaussianSplatStreamFrameTiming timing;
    if (!m_gaussianSplatStreamLoader.TryConsumeFrame(nextFrameIndex, gaussianData, &timing))
    {
        RecordGaussianSplatStreamMiss();
        m_gaussianSplatStreamAccumulator = m_gaussianSplatSetup.m_streamWaitForFrame ? frameDuration : std::fmod(m_gaussianSplatStreamAccumulator, frameDuration);
        return;
    }

    RecordGaussianSplatStreamLoadTiming(timing);

    const std::size_t splatCount = gaussianData.m_splats.size();
    const auto applyStartTime = std::chrono::steady_clock::now();
    const bool frameApplied = ApplyGaussianSplatStreamFrame(nextFrameIndex, std::move(gaussianData));
    const auto applyEndTime = std::chrono::steady_clock::now();
    const double applyMilliseconds = std::chrono::duration<double, std::milli>(applyEndTime - applyStartTime).count();
    RecordGaussianSplatStreamApplyTiming(applyMilliseconds, splatCount);

    if (!frameApplied)
    {
        m_gaussianSplatStreamStopped = true;
        UIUtility::Print("Gaussian stream stopped because frame " + std::to_string(nextFrameIndex + 1U) + " could not be loaded.\n");
        return;
    }

    m_gaussianSplatStreamAccumulator = std::fmod(m_gaussianSplatStreamAccumulator, frameDuration);
    RequestGaussianSplatStreamFrames(m_gaussianSplatStreamFrameIndex + 1U);
}

//Loads one Gaussian splat stream frame.
bool Scene::LoadGaussianSplatStreamFrame(std::size_t frameIndex)
{
    if (frameIndex >= m_gaussianSplatStreamFiles.size())
    {
        return false;
    }

    const auto loadStartTime = std::chrono::steady_clock::now();
    GaussianSplatReadResult gaussianData = GaussianSplatUtility::LoadFile(m_gaussianSplatStreamFiles[frameIndex], m_gaussianSplatSetup);
    const auto loadEndTime = std::chrono::steady_clock::now();
    const double loadMilliseconds = std::chrono::duration<double, std::milli>(loadEndTime - loadStartTime).count();
    if (!gaussianData.m_errorMessage.empty())
    {
        UIUtility::Print(gaussianData.m_errorMessage + "\n");
    }

    GaussianSplatStreamFrameTiming timing;
    timing.m_frameIndex = frameIndex;
    timing.m_splatCount = gaussianData.m_splats.size();
    timing.m_loadMilliseconds = loadMilliseconds;
    timing.m_fileName = m_gaussianSplatStreamFiles[frameIndex].filename().string();
    RecordGaussianSplatStreamLoadTiming(timing);

    const std::size_t splatCount = gaussianData.m_splats.size();
    const auto applyStartTime = std::chrono::steady_clock::now();
    const bool frameApplied = ApplyGaussianSplatStreamFrame(frameIndex, std::move(gaussianData));
    const auto applyEndTime = std::chrono::steady_clock::now();
    const double applyMilliseconds = std::chrono::duration<double, std::milli>(applyEndTime - applyStartTime).count();
    RecordGaussianSplatStreamApplyTiming(applyMilliseconds, splatCount);
    return frameApplied;
}

//Applies one decoded Gaussian splat stream frame.
bool Scene::ApplyGaussianSplatStreamFrame(std::size_t frameIndex, GaussianSplatReadResult gaussianData)
{
    if (gaussianData.m_splats.empty())
    {
        UIUtility::Print("Gaussian stream frame has 0 splats: " + m_gaussianSplatStreamFiles[frameIndex].string() + "\n");
        return false;
    }

    if (m_gaussianSplatObjectPtr == nullptr)
    {
        auto gaussianObjectPtr = CreateGaussianSplatObject(std::move(gaussianData.m_splats), m_gaussianSplatSetup, m_gaussianSplatShaderFiles, "GaussianSplatStream");
        m_gaussianSplatObjectPtr = gaussianObjectPtr.get();

        if (m_sceneObjectsInitialized)
        {
            static_cast<SceneObject *>(gaussianObjectPtr.get())->Init(m_sceneObjectContext);
        }

        m_sceneObjectPtrs.push_back(std::move(gaussianObjectPtr));
    }
    else
    {
        m_gaussianSplatObjectPtr->UpdateSplats(std::move(gaussianData.m_splats));
    }

    m_gaussianSplatStreamFrameIndex = frameIndex;
    return true;
}

//Requests upcoming Gaussian splat stream frames.
void Scene::RequestGaussianSplatStreamFrames(std::size_t firstFrameIndex)
{
    if (!m_gaussianSplatStreamEnabled || m_gaussianSplatStreamFiles.empty())
    {
        return;
    }

    std::size_t frameIndex = firstFrameIndex;
    const int preloadCount = std::max(1, m_gaussianSplatSetup.m_streamPreloadCount);
    for (int preloadIndex = 0; preloadIndex < preloadCount; ++preloadIndex)
    {
        if (frameIndex >= m_gaussianSplatStreamFiles.size())
        {
            if (!m_gaussianSplatSetup.m_streamLoop)
            {
                break;
            }

            frameIndex = 0U;
        }

        m_gaussianSplatStreamLoader.RequestFrame(frameIndex);
        ++frameIndex;
    }
}

//Records one decoded Gaussian stream frame timing sample.
void Scene::RecordGaussianSplatStreamLoadTiming(const GaussianSplatStreamFrameTiming &timing)
{
    if (!m_gaussianSplatSetup.m_streamTimingLog)
    {
        return;
    }

    ++m_gaussianSplatStreamTimingStats.m_loadedFrames;
    m_gaussianSplatStreamTimingStats.m_lastSplatCount = timing.m_splatCount;
    m_gaussianSplatStreamTimingStats.m_queueWaitMillisecondsTotal += timing.m_queueWaitMilliseconds;
    m_gaussianSplatStreamTimingStats.m_loadMillisecondsTotal += timing.m_loadMilliseconds;
    m_gaussianSplatStreamTimingStats.m_queueWaitMillisecondsMax = std::max(m_gaussianSplatStreamTimingStats.m_queueWaitMillisecondsMax, timing.m_queueWaitMilliseconds);
    m_gaussianSplatStreamTimingStats.m_loadMillisecondsMax = std::max(m_gaussianSplatStreamTimingStats.m_loadMillisecondsMax, timing.m_loadMilliseconds);

    if (m_gaussianSplatSetup.m_streamTimingLogEveryFrame)
    {
        UIUtility::Print(
            "Gaussian stream timing load frame " + std::to_string(timing.m_frameIndex + 1U) +
            " splats " + std::to_string(timing.m_splatCount) +
            " queue " + StringUtility::FormatFloat(static_cast<float>(timing.m_queueWaitMilliseconds), 2) + " ms" +
            " load " + StringUtility::FormatFloat(static_cast<float>(timing.m_loadMilliseconds), 2) + " ms " + timing.m_fileName + "\n"
        );
    }
}

//Records one render-thread Gaussian stream apply timing sample.
void Scene::RecordGaussianSplatStreamApplyTiming(double applyMilliseconds, std::size_t splatCount)
{
    if (!m_gaussianSplatSetup.m_streamTimingLog)
    {
        return;
    }

    ++m_gaussianSplatStreamTimingStats.m_appliedFrames;
    m_gaussianSplatStreamTimingStats.m_lastSplatCount = splatCount;
    m_gaussianSplatStreamTimingStats.m_applyMillisecondsTotal += applyMilliseconds;
    m_gaussianSplatStreamTimingStats.m_applyMillisecondsMax = std::max(m_gaussianSplatStreamTimingStats.m_applyMillisecondsMax, applyMilliseconds);

    if (m_gaussianSplatSetup.m_streamTimingLogEveryFrame)
    {
        UIUtility::Print(
            "Gaussian stream timing apply splats " + std::to_string(splatCount) +
            " apply " + StringUtility::FormatFloat(static_cast<float>(applyMilliseconds), 2) + " ms\n"
        );
    }
}

//Records one Gaussian stream frame miss.
void Scene::RecordGaussianSplatStreamMiss(void)
{
    if (!m_gaussianSplatSetup.m_streamTimingLog)
    {
        return;
    }

    ++m_gaussianSplatStreamTimingStats.m_missedFrames;
    if (m_gaussianSplatSetup.m_streamTimingLogEveryFrame)
    {
        UIUtility::Print("Gaussian stream timing miss: decoded frame was not ready.\n");
    }
}

//Records one Gaussian sort timing sample.
void Scene::RecordGaussianSplatStreamSortTiming(double sortMilliseconds)
{
    if (!m_gaussianSplatSetup.m_streamTimingLog)
    {
        return;
    }

    ++m_gaussianSplatStreamTimingStats.m_sortCalls;
    m_gaussianSplatStreamTimingStats.m_sortMillisecondsTotal += sortMilliseconds;
    m_gaussianSplatStreamTimingStats.m_sortMillisecondsMax = std::max(m_gaussianSplatStreamTimingStats.m_sortMillisecondsMax, sortMilliseconds);
}

//Prints Gaussian stream timing when configured.
void Scene::PrintGaussianSplatStreamTiming(float deltaSeconds)
{
    if (!m_gaussianSplatSetup.m_streamTimingLog || !m_gaussianSplatStreamEnabled)
    {
        return;
    }

    m_gaussianSplatStreamTimingStats.m_elapsedSeconds += std::max(0.0f, deltaSeconds);
    const float logInterval = std::max(0.1f, m_gaussianSplatSetup.m_streamTimingLogIntervalSeconds);
    if (m_gaussianSplatStreamTimingStats.m_elapsedSeconds < logInterval)
    {
        return;
    }

    PrintGaussianSplatStreamTimingSummary();
    ResetGaussianSplatStreamTiming();
}

//Prints the current Gaussian stream timing summary.
void Scene::PrintGaussianSplatStreamTimingSummary(void)
{
    const GaussianSplatStreamTimingStats &stats = m_gaussianSplatStreamTimingStats;
    if (stats.m_loadedFrames == 0U && stats.m_appliedFrames == 0U && stats.m_missedFrames == 0U && stats.m_sortCalls == 0U)
    {
        return;
    }

    UIUtility::Print(
        "Gaussian stream timing summary: loaded " + std::to_string(stats.m_loadedFrames) +
        " applied " + std::to_string(stats.m_appliedFrames) +
        " missed " + std::to_string(stats.m_missedFrames) +
        " splats " + std::to_string(stats.m_lastSplatCount) +
        " load avg/max " + StringUtility::FormatFloat(static_cast<float>(AverageTiming(stats.m_loadMillisecondsTotal, stats.m_loadedFrames)), 2) +
        "/" + StringUtility::FormatFloat(static_cast<float>(stats.m_loadMillisecondsMax), 2) + " ms" +
        " queue avg/max " + StringUtility::FormatFloat(static_cast<float>(AverageTiming(stats.m_queueWaitMillisecondsTotal, stats.m_loadedFrames)), 2) +
        "/" + StringUtility::FormatFloat(static_cast<float>(stats.m_queueWaitMillisecondsMax), 2) + " ms" +
        " apply avg/max " + StringUtility::FormatFloat(static_cast<float>(AverageTiming(stats.m_applyMillisecondsTotal, stats.m_appliedFrames)), 2) +
        "/" + StringUtility::FormatFloat(static_cast<float>(stats.m_applyMillisecondsMax), 2) + " ms" +
        " sort avg/max " + StringUtility::FormatFloat(static_cast<float>(AverageTiming(stats.m_sortMillisecondsTotal, stats.m_sortCalls)), 2) +
        "/" + StringUtility::FormatFloat(static_cast<float>(stats.m_sortMillisecondsMax), 2) + " ms\n"
    );
}

//Clears Gaussian stream timing summary values.
void Scene::ResetGaussianSplatStreamTiming(void)
{
    m_gaussianSplatStreamTimingStats = GaussianSplatStreamTimingStats{};
}

//Creates a Gaussian splat scene object.
std::unique_ptr<SceneObjectGaussianSplat> Scene::CreateGaussianSplatObject(
    std::vector<GaussianSplatData> splats,
    const GaussianSplatSetup &setup,
    const SceneObjectShaderFiles &shaderFiles,
    const std::string &name
) const
{
    SceneObjectRenderOptions renderOptions;
    renderOptions.m_clickable = setup.m_clickable;
    renderOptions.m_selectionMode = setup.m_clickable ? SceneObjectSelectionMode::PerObject : SceneObjectSelectionMode::None;

    auto gaussianObjectPtr = std::make_unique<SceneObjectGaussianSplat>();
    gaussianObjectPtr->SetName(name);
    gaussianObjectPtr->Init(
        std::move(splats),
        setup.m_baseTranslation,
        GaussianSplatBaseRotationMatrix(setup),
        setup.m_baseScale,
        setup.m_pointSize,
        setup.m_maxPointSize,
        setup.m_sortEnabled,
        setup.m_sortEveryFrame,
        setup.m_depthTest,
        setup.m_depthWrite,
        shaderFiles,
        renderOptions
    );

    return gaussianObjectPtr;
}

//Adds the directional light marker to the scene.
void Scene::DrawLightMarker(const ProgramSetup &programSetup, const SceneObjectShaderFiles &shaderFiles)
{
    if (!programSetup.m_scene.m_lightMarkerEnabled)
    {
        return;
    }

    auto lightMarkerObjectPtr = std::make_unique<SceneObjectPyramid>();
    lightMarkerObjectPtr->SetName("LightMarker");
    const float lightMarkerHeight = programSetup.m_scene.m_lightMarkerSize;
    const float lightMarkerFovRadians = std::clamp(programSetup.m_lighting.m_fovDegrees, 1.0f, 170.0f) * Pi / 180.0f;
    const float lightMarkerBaseSize = 2.0f * lightMarkerHeight * std::tan(lightMarkerFovRadians * 0.5f);

    SceneObjectRenderOptions renderOptions;
    renderOptions.m_castShadow = false;
    renderOptions.m_receiveShadow = false;
    renderOptions.m_wireframe = true;
    renderOptions.m_clickable = false;
    renderOptions.m_wireframeLineWidth = 2.0f;


    lightMarkerObjectPtr->Init(
        programSetup.m_lighting.m_position,
        programSetup.m_lighting.m_direction,
        lightMarkerBaseSize,
        lightMarkerHeight,
        programSetup.m_scene.m_lightMarkerColor,
        shaderFiles,
        renderOptions
    );
    m_sceneObjectPtrs.push_back(std::move(lightMarkerObjectPtr));
}

//Adds COLMAP camera and point cloud objects to the scene.
void Scene::DrawColmap(const ProgramSetup &programSetup, const SceneObjectShaderFiles &cameraShaderFiles, const SceneObjectShaderFiles &pointCloudShaderFiles, const ColmapCameraReadResult &colmapData)
{
    const std::vector<ColmapCameraData> &cameras = colmapData.m_cameras;
    if (cameras.empty())
    {
        //Prints camera load status when COLMAP is enabled.
        if (programSetup.m_colmap.m_enabled)
        {
            UIUtility::Print("Loaded " + std::to_string(colmapData.m_intrinsics.size()) + " COLMAP camera intrinsics and 0 COLMAP image poses.\n");
        }
    }
    else
    {
        //Builds display instances for COLMAP camera markers.
        std::vector<SceneObjectInstanceData> cameraInstanceData;
        cameraInstanceData.reserve(cameras.size());

        const int displayStep = std::max(1, programSetup.m_colmap.m_displayEveryNthCamera);
        for (std::size_t cameraIndex = 0; cameraIndex < cameras.size(); cameraIndex += static_cast<std::size_t>(displayStep))
        {
            const ColmapCameraData &camera = cameras[cameraIndex];
            cameraInstanceData.push_back(CreateInstanceData(camera.m_position, programSetup.m_colmap.m_cameraSize, programSetup.m_colmap.m_cameraColor, CameraPyramidOrientation(camera)));

            if (programSetup.m_colmap.m_showCameraLabels)
            {
                //Adds one text label near the displayed camera marker.
                SceneTextLabelData labelData;
                labelData.m_position = camera.m_position + (camera.m_right * (programSetup.m_colmap.m_cameraSize * 0.55f)) + (camera.m_up * (programSetup.m_colmap.m_cameraSize * 0.30f));
                labelData.m_text = std::to_string(camera.m_imageId);
                m_sceneTextLabels.push_back(labelData);
            }
        }

        if (!cameraInstanceData.empty())
        {
            //Creates one instanced pyramid object for COLMAP cameras.
            SceneObjectRenderOptions renderOptions;
            renderOptions.m_castShadow = false;
            renderOptions.m_receiveShadow = false;
            renderOptions.m_wireframe = true;
            renderOptions.m_selectionMode = SceneObjectSelectionMode::PerObject;
            renderOptions.m_wireframeLineWidth = 2.0f;

            auto colmapCameraObjectPtr = std::make_unique<SceneObjectPyramid>();
            colmapCameraObjectPtr->SetName("ColmapCamera");
            m_colmapCameraObjectPtr = colmapCameraObjectPtr.get();
            colmapCameraObjectPtr->Init(std::move(cameraInstanceData), { 0.0f, 0.0f, -1.0f }, 0.3f, 0.3f, cameraShaderFiles, renderOptions);
            m_sceneObjectPtrs.push_back(std::move(colmapCameraObjectPtr));
        }

        UIUtility::Print("Loaded " + std::to_string(colmapData.m_intrinsics.size()) + " COLMAP camera intrinsics and " + std::to_string(cameras.size()) + " COLMAP image poses.\n");
    }

    //Selects how COLMAP points should be displayed.
    const ColmapPointDisplayMode mode = ParseColmapPointDisplayMode(programSetup.m_colmap.m_pointDisplayMode);
    bool sparsePointCloudLoaded = false;
    if (mode == ColmapPointDisplayMode::Points3D || mode == ColmapPointDisplayMode::Auto)
    {
        //Builds sparse COLMAP points from points3D data.
        ColmapSparsePointCloudResult pointCloudResult = MultiViewUtility::BuildColmapSparsePointCloud(colmapData, programSetup.m_colmap);
        if (!pointCloudResult.m_points.empty())
        {
            sparsePointCloudLoaded = true;
            AddColmapPointCloudObject(std::move(pointCloudResult.m_points), pointCloudShaderFiles, 3.0f);

            UIUtility::Print("Loaded " + std::to_string(pointCloudResult.m_pointsDisplayed) + " COLMAP sparse points from points3D with " + std::to_string(pointCloudResult.m_trackMatches) +
                " loaded camera track observations.\n");
        }
    }

    if (mode == ColmapPointDisplayMode::Depth || (mode == ColmapPointDisplayMode::Auto && !sparsePointCloudLoaded))
    {
        //Builds COLMAP points by unprojecting metric depth images.
        ColmapDepthPointCloudResult pointCloudResult = MultiViewUtility::BuildColmapDepthPointCloud(colmapData, programSetup.m_colmap);
        if (!pointCloudResult.m_points.empty())
        {
            const std::size_t pointCount = pointCloudResult.m_points.size();
            AddColmapPointCloudObject(std::move(pointCloudResult.m_points), pointCloudShaderFiles, 3.0f);

            UIUtility::Print("Loaded " + std::to_string(pointCount) + " COLMAP depth points from " + std::to_string(pointCloudResult.m_depthImagesLoaded) + " depth images and " +
                std::to_string(pointCloudResult.m_colorImagesLoaded) + " color images.\n");
        }
    }

    if (mode == ColmapPointDisplayMode::PlaneSweep)
    {
        std::chrono::time_point timeStart = std::chrono::steady_clock::now();
        PlaneSweepGpu planeSweepGpu;
        planeSweepGpu.Initialize(
            m_sceneObjectContext.m_devicePtrGPU,
            m_sceneObjectContext.m_physicalDevicePtrGPU,
            m_sceneObjectContext.m_graphicsQueuePtrGPU,
            m_sceneObjectContext.m_commandPoolPtrGPU
        );
        UIUtility::Print("PlaneSweepGpu initialized for PlaneSweep reconstruction handoff.\n");

        // If experiment mode is enabled, run a one-factor-at-a-time sweep across chosen parameters.
        if (programSetup.m_colmap.m_planeSweepExperimentMode)
        {
            // Baseline setup copy
            const ColmapSetup baselineSetup = programSetup.m_colmap;

            // Prepare experiments directory base
            const std::string baseOut = baselineSetup.m_planeSweepOutputFolder.empty() ? std::string("Output/PlaneSweepExperiments") : (baselineSetup.m_planeSweepOutputFolder + "/Experiments");
            std::filesystem::create_directories(baseOut);

            // Helper to run one experiment configuration
            auto runOne = [&](const ColmapSetup &cfg, const std::string &runName)
            {
                PlaneSweepExperimentMetrics metrics;
                const auto t0 = std::chrono::steady_clock::now();
                ColmapPlaneSweepResult result = MultiViewUtility::BuildColmapPlaneSweepPointCloud(colmapData, cfg, &metrics, &planeSweepGpu);
                const auto t1 = std::chrono::steady_clock::now();
                metrics.m_totalMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
                metrics.m_referenceImagesProcessed = result.m_referenceImagesProcessed;
                metrics.m_sourceImagesLoaded = result.m_sourceImagesLoaded;
                metrics.m_depthFilesSaved = result.m_depthFilesSaved;
                metrics.m_pointsRejectedTexture = result.m_pointsRejectedTexture;
                metrics.m_pointsRejectedConfidence = result.m_pointsRejectedConfidence;
                metrics.m_pointsRejectedCrossView = result.m_pointsRejectedCrossView;
                metrics.m_pointsBeforeFiltering = result.m_pointsBeforeFiltering;
                metrics.m_pointsAfterFusion = result.m_pointsAfterFusion;
                metrics.m_pointsAfterOutlierFilter = result.m_pointsAfterOutlierFilter;
                metrics.m_finalPointCount = result.m_points.size();

                // Per-run unique paths
                const std::string ts = std::to_string(static_cast<long long>(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count()));
                const std::string runFolder = baseOut + "/" + runName + "_" + ts;
                std::filesystem::create_directories(runFolder);

                // Move/copy depth outputs if generated in a different folder: ensure outputs were written to cfg.m_planeSweepOutputFolder.
                // For safety, write metrics into runFolder.
                const std::string csvPath = baseOut + "/results.csv";
                const bool csvExists = std::filesystem::exists(csvPath);
                std::ofstream csv(csvPath, std::ios::app);
                if (csv)
                {
                    if (!csvExists)
                    {
                        csv << "run,group,value,totalMs,estimateDepthRangeMs,buildReferencePointsMs,computePlaneCostMs,crossViewMs,filteringMs,referenceImagesProcessed,sourceImagesLoaded,depthFilesSaved,pointsRejectedTexture,pointsRejectedConfidence,pointsRejectedCrossView,pointsBeforeFiltering,pointsAfterFusion,pointsAfterOutlierFilter,finalPointCount\n";
                    }
                    csv << runName << "," << "," << metrics.m_totalMs << "," << metrics.m_estimateDepthRangeMs << "," << metrics.m_buildReferencePointsMs << "," << metrics.m_computePlaneCostMs << "," << metrics.m_crossViewMs << "," << metrics.m_filteringMs << "," << metrics.m_referenceImagesProcessed << "," << metrics.m_sourceImagesLoaded << "," << metrics.m_depthFilesSaved << "," << metrics.m_pointsRejectedTexture << "," << metrics.m_pointsRejectedConfidence << "," << metrics.m_pointsRejectedCrossView << "," << metrics.m_pointsBeforeFiltering << "," << metrics.m_pointsAfterFusion << "," << metrics.m_pointsAfterOutlierFilter << "," << metrics.m_finalPointCount << "\n";
                    csv.close();
                }

                // Write per-run JSON
                const std::string jsonPath = runFolder + "/metrics.json";
                std::ofstream json(jsonPath);
                if (json)
                {
                    json << "{\n";
                    json << "  \"run\": \"" << runName << "\",\n";
                    json << "  \"totalMs\": " << metrics.m_totalMs << ",\n";
                    json << "  \"estimateDepthRangeMs\": " << metrics.m_estimateDepthRangeMs << ",\n";
                    json << "  \"buildReferencePointsMs\": " << metrics.m_buildReferencePointsMs << ",\n";
                    json << "  \"computePlaneCostMs\": " << metrics.m_computePlaneCostMs << ",\n";
                    json << "  \"crossViewMs\": " << metrics.m_crossViewMs << ",\n";
                    json << "  \"filteringMs\": " << metrics.m_filteringMs << ",\n";
                    json << "  \"referenceImagesProcessed\": " << metrics.m_referenceImagesProcessed << ",\n";
                    json << "  \"sourceImagesLoaded\": " << metrics.m_sourceImagesLoaded << ",\n";
                    json << "  \"depthFilesSaved\": " << metrics.m_depthFilesSaved << ",\n";
                    json << "  \"pointsRejectedTexture\": " << metrics.m_pointsRejectedTexture << ",\n";
                    json << "  \"pointsRejectedConfidence\": " << metrics.m_pointsRejectedConfidence << ",\n";
                    json << "  \"pointsRejectedCrossView\": " << metrics.m_pointsRejectedCrossView << ",\n";
                    json << "  \"pointsBeforeFiltering\": " << metrics.m_pointsBeforeFiltering << ",\n";
                    json << "  \"pointsAfterFusion\": " << metrics.m_pointsAfterFusion << ",\n";
                    json << "  \"pointsAfterOutlierFilter\": " << metrics.m_pointsAfterOutlierFilter << ",\n";
                    json << "  \"finalPointCount\": " << metrics.m_finalPointCount << "\n";
                    json << "}\n";
                    json.close();
                }

                UIUtility::Print("Experiment " + runName + " finished. Results in " + runFolder + "\n");
            };

            // Baseline run
            runOne(baselineSetup, std::string("baseline"));

            // Define groups and values (one-factor-at-a-time)
            // Plane count
            {
                ColmapSetup cfg = baselineSetup;
                int base = std::max(2, baselineSetup.m_planeSweepPlaneCount);
                std::vector<int> vals = { std::max(2, base/2), base, std::min(1024, base*2) };
                for (int v : vals)
                {
                    if (v == base) continue;
                    cfg.m_planeSweepPlaneCount = v;
                    cfg.m_planeSweepOutputFolder = baseOut + "/PlaneCount_" + std::to_string(v);
                    runOne(cfg, std::string("PlaneCount_") + std::to_string(v));
                }
            }

            // Image scale
            {
                ColmapSetup cfg = baselineSetup;
                double base = baselineSetup.m_planeSweepImageScale;
                std::vector<double> vals = { std::max(0.05, base/2.0), base, std::min(1.0, base*2.0) };
                for (double v : vals)
                {
                    if (v == base) continue;
                    cfg.m_planeSweepImageScale = static_cast<float>(v);
                    cfg.m_planeSweepOutputFolder = baseOut + "/ImageScale_" + std::to_string(static_cast<int>(v*100));
                    runOne(cfg, std::string("ImageScale_") + std::to_string(static_cast<int>(v*100)));
                }
            }

            // Patch radius
            {
                ColmapSetup cfg = baselineSetup;
                int base = baselineSetup.m_planeSweepPatchRadius;
                std::vector<int> vals = { std::max(0, base-2), base, base+2 };
                for (int v : vals)
                {
                    if (v == base) continue;
                    cfg.m_planeSweepPatchRadius = v;
                    cfg.m_planeSweepOutputFolder = baseOut + "/PatchRadius_" + std::to_string(v);
                    runOne(cfg, std::string("PatchRadius_") + std::to_string(v));
                }
            }

            // Source count
            {
                ColmapSetup cfg = baselineSetup;
                int base = baselineSetup.m_planeSweepSourceCount;
                std::vector<int> vals = { std::max(1, base-1), base, base+1 };
                for (int v : vals)
                {
                    if (v == base) continue;
                    cfg.m_planeSweepSourceCount = v;
                    cfg.m_planeSweepOutputFolder = baseOut + "/SourceCount_" + std::to_string(v);
                    runOne(cfg, std::string("SourceCount_") + std::to_string(v));
                }
            }

            // Cost threshold
            {
                ColmapSetup cfg = baselineSetup;
                double base = baselineSetup.m_planeSweepCostThreshold;
                std::vector<double> vals = { std::max(0.0, base*0.5), base, base*1.5 };
                for (double v : vals)
                {
                    if (v == base) continue;
                    cfg.m_planeSweepCostThreshold = static_cast<float>(v);
                    cfg.m_planeSweepOutputFolder = baseOut + "/CostThreshold_" + std::to_string(static_cast<int>(v*100));
                    runOne(cfg, std::string("CostThreshold_") + std::to_string(static_cast<int>(v*100)));
                }
            }

            // Point step
            {
                ColmapSetup cfg = baselineSetup;
                int base = baselineSetup.m_planeSweepPointStep;
                std::vector<int> vals = { std::max(1, base/2), base, std::min(8, base*2) };
                for (int v : vals)
                {
                    if (v == base) continue;
                    cfg.m_planeSweepPointStep = v;
                    cfg.m_planeSweepOutputFolder = baseOut + "/PointStep_" + std::to_string(v);
                    runOne(cfg, std::string("PointStep_") + std::to_string(v));
                }
            }

            // Toggle cross-view consistency
            {
                ColmapSetup cfg = baselineSetup;
                cfg.m_planeSweepCrossViewConsistencyEnabled = !baselineSetup.m_planeSweepCrossViewConsistencyEnabled;
                cfg.m_planeSweepOutputFolder = baseOut + "/CrossView_" + (cfg.m_planeSweepCrossViewConsistencyEnabled ? std::string("On") : std::string("Off"));
                runOne(cfg, std::string("CrossView_") + (cfg.m_planeSweepCrossViewConsistencyEnabled ? "On" : "Off"));
            }

            UIUtility::Print("All experiments complete. Exiting.\n");
            exit(0);
        }

        //Builds COLMAP points with CPU plane sweep reconstruction.
        PlaneSweepExperimentMetrics metrics;
        ColmapPlaneSweepResult pointCloudResult = MultiViewUtility::BuildColmapPlaneSweepPointCloud(colmapData, programSetup.m_colmap, nullptr, &planeSweepGpu);
        if (!pointCloudResult.m_points.empty())
        {
            const std::size_t pointCount = pointCloudResult.m_points.size();
            AddColmapPointCloudObject(std::move(pointCloudResult.m_points), pointCloudShaderFiles, 3.0f);

            UIUtility::Print("Loaded " + std::to_string(pointCount) + " COLMAP plane sweep points from " + std::to_string(pointCloudResult.m_referenceImagesProcessed) + " reference images.\n");
        }

        std::chrono::time_point timeEnd = std::chrono::steady_clock::now();
        double elapsedTimeInSeconds = std::chrono::duration<double>(timeEnd - timeStart).count();

        UIUtility::Print("Plane sweep reconstruction took " + std::to_string(elapsedTimeInSeconds) + "s.\n");

        // If experiment mode was enabled, write metrics and exit after running experiments.
        if (programSetup.m_colmap.m_planeSweepExperimentMode)
        {
            // Populate basic counters into metrics and write a simple JSON/CSV summary.
            metrics.m_totalMs = elapsedTimeInSeconds * 1000.0;
            metrics.m_referenceImagesProcessed = pointCloudResult.m_referenceImagesProcessed;
            metrics.m_sourceImagesLoaded = pointCloudResult.m_sourceImagesLoaded;
            metrics.m_depthFilesSaved = pointCloudResult.m_depthFilesSaved;
            metrics.m_pointsRejectedTexture = pointCloudResult.m_pointsRejectedTexture;
            metrics.m_pointsRejectedConfidence = pointCloudResult.m_pointsRejectedConfidence;
            metrics.m_pointsRejectedCrossView = pointCloudResult.m_pointsRejectedCrossView;
            metrics.m_pointsBeforeFiltering = pointCloudResult.m_pointsBeforeFiltering;
            metrics.m_pointsAfterFusion = pointCloudResult.m_pointsAfterFusion;
            metrics.m_pointsAfterOutlierFilter = pointCloudResult.m_pointsAfterOutlierFilter;
            metrics.m_finalPointCount = pointCloudResult.m_points.size();

            const std::string outFolder = programSetup.m_colmap.m_planeSweepOutputFolder.empty() ? "Output/PlaneSweepExperiments" : programSetup.m_colmap.m_planeSweepOutputFolder + "/Experiments";
            std::filesystem::create_directories(outFolder);
            const std::string csvPath = outFolder + "/results.csv";
            std::ofstream csv(csvPath, std::ios::app);
            if (csv)
            {
                csv << "totalMs,estimateDepthRangeMs,buildReferencePointsMs,computePlaneCostMs,crossViewMs,filteringMs,referenceImagesProcessed,sourceImagesLoaded,depthFilesSaved,pointsRejectedTexture,pointsRejectedConfidence,pointsRejectedCrossView,pointsBeforeFiltering,pointsAfterFusion,pointsAfterOutlierFilter,finalPointCount\n";
                csv << metrics.m_totalMs << "," << metrics.m_estimateDepthRangeMs << "," << metrics.m_buildReferencePointsMs << "," << metrics.m_computePlaneCostMs << "," << metrics.m_crossViewMs << "," << metrics.m_filteringMs << "," << metrics.m_referenceImagesProcessed << "," << metrics.m_sourceImagesLoaded << "," << metrics.m_depthFilesSaved << "," << metrics.m_pointsRejectedTexture << "," << metrics.m_pointsRejectedConfidence << "," << metrics.m_pointsRejectedCrossView << "," << metrics.m_pointsBeforeFiltering << "," << metrics.m_pointsAfterFusion << "," << metrics.m_pointsAfterOutlierFilter << "," << metrics.m_finalPointCount << "\n";
                csv.close();
            }

            // Optionally write per-run JSON.
            const std::string jsonPath = outFolder + "/metrics.json";
            std::ofstream json(jsonPath);
            if (json)
            {
                json << "{\n";
                json << "  \"totalMs\": " << metrics.m_totalMs << ",\n";
                json << "  \"estimateDepthRangeMs\": " << metrics.m_estimateDepthRangeMs << ",\n";
                json << "  \"buildReferencePointsMs\": " << metrics.m_buildReferencePointsMs << ",\n";
                json << "  \"computePlaneCostMs\": " << metrics.m_computePlaneCostMs << ",\n";
                json << "  \"crossViewMs\": " << metrics.m_crossViewMs << ",\n";
                json << "  \"filteringMs\": " << metrics.m_filteringMs << ",\n";
                json << "  \"referenceImagesProcessed\": " << metrics.m_referenceImagesProcessed << ",\n";
                json << "  \"sourceImagesLoaded\": " << metrics.m_sourceImagesLoaded << ",\n";
                json << "  \"depthFilesSaved\": " << metrics.m_depthFilesSaved << ",\n";
                json << "  \"pointsRejectedTexture\": " << metrics.m_pointsRejectedTexture << ",\n";
                json << "  \"pointsRejectedConfidence\": " << metrics.m_pointsRejectedConfidence << ",\n";
                json << "  \"pointsRejectedCrossView\": " << metrics.m_pointsRejectedCrossView << ",\n";
                json << "  \"pointsBeforeFiltering\": " << metrics.m_pointsBeforeFiltering << ",\n";
                json << "  \"pointsAfterFusion\": " << metrics.m_pointsAfterFusion << ",\n";
                json << "  \"pointsAfterOutlierFilter\": " << metrics.m_pointsAfterOutlierFilter << ",\n";
                json << "  \"finalPointCount\": " << metrics.m_finalPointCount << "\n";
                json << "}\n";
                json.close();
            }

            // Exit the application after experiments complete.
            UIUtility::Print("Experiment mode complete. Exiting application.\n");
            exit(0);
        }
    }
}

//Adds one COLMAP point cloud object to the scene.
void Scene::AddColmapPointCloudObject(std::vector<SceneObjectPointCloudPoint> points, const SceneObjectShaderFiles &shaderFiles, float pointSize)
{
    SceneObjectRenderOptions renderOptions;
    renderOptions.m_castShadow = false;
    renderOptions.m_receiveShadow = false;
    renderOptions.m_clickable = true;
    renderOptions.m_selectionMode = SceneObjectSelectionMode::PerObject;

    auto pointCloudObjectPtr = std::make_unique<SceneObjectPointCloud>();
    pointCloudObjectPtr->SetName("ColmapPointCloud");
    m_colmapPointCloudObjectPtr = pointCloudObjectPtr.get();
    pointCloudObjectPtr->Init(std::move(points), { 0.0f, 0.0f, 0.0f }, identity(), 1.0f, pointSize, { 1.0f, 1.0f, 1.0f, 1.0f }, shaderFiles, renderOptions);
    m_sceneObjectPtrs.push_back(std::move(pointCloudObjectPtr));
}

//Adds the grid object to the scene.
void Scene::DrawGrid(const ProgramSetup &programSetup, const SceneObjectShaderFiles &lineShaderFiles, const SceneObjectShaderFiles &planeShaderFiles)
{
    if (!programSetup.m_scene.m_gridEnabled)
    {
        return;
    }

    if (GridModeIsLine(programSetup.m_scene.m_gridMode))
    {
        auto gridObjectPtr = std::make_unique<SceneObjectGrid>();
        gridObjectPtr->SetName("GridLine");
        gridObjectPtr->Init(-50.0f, 50.0f, 10.0f, programSetup.m_graphics.m_gridLineWidth, lineShaderFiles);
        gridObjectPtr->SetClickable(false);
        m_sceneObjectPtrs.push_back(std::move(gridObjectPtr));
        return;
    }

    auto gridObjectPtr = std::make_unique<SceneObjectGridPlane>();
    gridObjectPtr->SetName("GridPlane");
    gridObjectPtr->Init(-50.0f, 50.0f, 10.0f, programSetup.m_scene.m_gridTexture, planeShaderFiles);
    gridObjectPtr->SetClickable(false);
    m_sceneObjectPtrs.push_back(std::move(gridObjectPtr));
}

//Adds the axis object to the scene.
void Scene::DrawAxis(const ProgramSetup &programSetup, const SceneObjectShaderFiles &shaderFiles)
{
    auto axisObjectPtr = std::make_unique<SceneObjectAxis>();
    axisObjectPtr->SetName("WorldAxis");
    axisObjectPtr->Init(-50.0f, 50.0f, programSetup.m_graphics.m_axisLineWidth, shaderFiles);
    axisObjectPtr->SetClickable(false);
    m_sceneObjectPtrs.push_back(std::move(axisObjectPtr));
}

//Draws the lower-right HUD axis widget.
void Scene::DrawHudAxis(UIUtility &uiUtility, const UISetup &uiSetup, VkExtent2D swapChainExtent) const
{
    const float screenWidth = static_cast<float>(swapChainExtent.width);
    const float screenHeight = static_cast<float>(swapChainExtent.height);
    if (screenWidth <= 1.0f || screenHeight <= 1.0f)
    {
        return;
    }

    const float axisLength = 46.0f;
    const float lineThickness = 3.0f;
    const float labelSize = std::max(1.35f, uiSetup.m_hudTextSize * 0.85f);
    const float rightMargin = 80.0f;
    const float bottomReserved = std::max(58.0f, uiSetup.m_hudTextSize * 28.0f);
    const Float2 origin{
        std::max(axisLength + rightMargin, screenWidth - axisLength - rightMargin), std::max(axisLength + rightMargin, screenHeight - axisLength - bottomReserved)
    };

    auto DrawAxisLine = [&](const Float3 &axis, const Float4 &color, const std::string &label)
    {
        const Float2 projectedAxis{
            dot(axis, m_eye.Right()), -dot(axis, m_eye.Up())
        };
        const float projectedLength = std::sqrt((projectedAxis.x * projectedAxis.x) + (projectedAxis.y * projectedAxis.y));
        Float2 direction{};
        float drawLength = axisLength;

        if (projectedLength > 0.0001f)
        {
            direction = { projectedAxis.x / projectedLength, projectedAxis.y / projectedLength };
            drawLength *= std::max(0.35f, projectedLength);
        }
        else
        {
            const float depth = dot(axis, m_eye.Forward());
            if (depth < 0.0f)
            {
                direction = { -0.80f, 0.60f };
            }
            else
            {
                direction = { 0.80f, -0.60f };
            }

            drawLength *= 0.45f;
        }

        const float endX = origin.x + (direction.x * drawLength);
        const float endY = origin.y + (direction.y * drawLength);
        const float labelX = endX + (direction.x * 7.0f) - 4.0f;
        const float labelY = endY + (direction.y * 7.0f) - 7.0f;

        uiUtility.DrawLine(origin.x, origin.y, endX, endY, color, lineThickness);
        uiUtility.DrawText(labelX + 1.0f, labelY + 1.0f, label, uiSetup.m_textShadowColor, labelSize);
        uiUtility.DrawText(labelX, labelY, label, color, labelSize);
    };

    DrawAxisLine({ 0.0f, 0.0f, 1.0f }, { 0.15f, 0.35f, 1.0f, 1.0f }, "Z");
    DrawAxisLine({ 0.0f, 1.0f, 0.0f }, { 0.1f, 0.75f, 0.2f, 1.0f }, "Y");
    DrawAxisLine({ 1.0f, 0.0f, 0.0f }, { 1.0f, 0.15f, 0.1f, 1.0f }, "X");

    uiUtility.DrawLine(origin.x - 4.0f, origin.y, origin.x + 4.0f, origin.y, uiSetup.m_textColor, 2.0f);
    uiUtility.DrawLine(origin.x, origin.y - 4.0f, origin.x, origin.y + 4.0f, uiSetup.m_textColor, 2.0f);
}

//Prints the camera location and camera axes.
void Scene::PrintCameraAndAxisLine(void) const
{
    UIUtility::Print("Camera Position: " + StringUtility::FormatFloat3(m_eye.Position()) + " | LookAt: " + StringUtility::FormatFloat3(m_eye.Forward()) +
        " | Up: " + StringUtility::FormatFloat3(m_eye.Up()) + " | Right: " + StringUtility::FormatFloat3(m_eye.Right()) + "\n");
}

//Handles transform gizmo input.
void Scene::UpdateTransformGizmoInput(const InputUtility &inputUtility)
{
    if (inputUtility.KeyPressed(m_transformTranslateKey))
    {
        m_transformGizmoMode = TransformGizmoMode::Translate;
        UIUtility::Print("Transform gizmo mode: Translate.\n");
    }

    if (inputUtility.KeyPressed(m_transformRotateKey))
    {
        m_transformGizmoMode = TransformGizmoMode::Rotate;
        UIUtility::Print("Transform gizmo mode: Rotate.\n");
    }

    if (inputUtility.KeyPressed(m_transformScaleKey))
    {
        m_transformGizmoMode = TransformGizmoMode::Scale;
        UIUtility::Print("Transform gizmo mode: Scale.\n");
    }

    if (inputUtility.MouseButtonDoubleClicked(s_leftMouseButton))
    {
        SelectObjectAtMouse(inputUtility);
        return;
    }

    if (!SelectionActive())
    {
        return;
    }

    if (m_activeTransformAxis == TransformGizmoAxis::None && inputUtility.MouseButtonPressed(s_leftMouseButton))
    {
        const TransformGizmoAxis axis = HitTestTransformGizmo(inputUtility);
        if (axis != TransformGizmoAxis::None)
        {
            m_activeTransformAxis = axis;
            m_transformStartMouse = {
                static_cast<float>(inputUtility.MouseX()), static_cast<float>(inputUtility.MouseY())
            };
            m_transformStartPosition = SelectedPosition();
            m_transformStartOrientation = SelectedOrientation();
            m_transformStartScale = SelectedScale();
        }
    }

    if (m_activeTransformAxis == TransformGizmoAxis::None)
    {
        return;
    }

    if (!inputUtility.MouseButtonDown(s_leftMouseButton))
    {
        m_activeTransformAxis = TransformGizmoAxis::None;
        return;
    }

    Float2 objectScreen{};
    float objectDepth = 1.0f;
    const Float3 selectedPosition = SelectedPosition();
    if (!ProjectWorldPointToScreen(selectedPosition, objectScreen, objectDepth))
    {
        return;
    }
    static_cast<void>(objectDepth);

    const Float3 axisDirection = TransformAxisDirection(m_activeTransformAxis);
    const Float3 axisPoint = selectedPosition + (axisDirection * SelectedGizmoRadius());
    const CameraBuffer cameraBuffer = CameraBufferInfo(static_cast<float>(std::max(1U, m_lastScreenExtent.width)) / static_cast<float>(std::max(1U, m_lastScreenExtent.height)));
    const Float4 clipAxis = transform(cameraBuffer.m_projection, transform(cameraBuffer.m_view, { axisPoint.x, axisPoint.y, axisPoint.z, 1.0f }));
    Float2 axisScreenDirection{ 1.0f, 0.0f };
    if (std::isfinite(clipAxis.w) && clipAxis.w > 0.0001f)
    {
        const float screenWidth = static_cast<float>(m_lastScreenExtent.width);
        const float screenHeight = static_cast<float>(m_lastScreenExtent.height);
        const Float2 axisScreen{
            (((clipAxis.x / clipAxis.w) * 0.5f) + 0.5f) * screenWidth, (((clipAxis.y / clipAxis.w) * 0.5f) + 0.5f) * screenHeight
        };
        axisScreenDirection = NormalizeScreenDirection({ axisScreen.x - objectScreen.x, axisScreen.y - objectScreen.y });
    }

    const Float2 currentMouse{
        static_cast<float>(inputUtility.MouseX()), static_cast<float>(inputUtility.MouseY())
    };
    const Float2 mouseDelta{
        currentMouse.x - m_transformStartMouse.x, currentMouse.y - m_transformStartMouse.y
    };
    const float projectedDrag = (mouseDelta.x * axisScreenDirection.x) + (mouseDelta.y * axisScreenDirection.y);
    const float distanceFromCamera = std::sqrt(dot(selectedPosition - m_eye.Position(), selectedPosition - m_eye.Position()));

    if (m_transformGizmoMode == TransformGizmoMode::Translate)
    {
        const float amount = projectedDrag * std::max(0.01f, distanceFromCamera * 0.0025f);
        SetSelectedPosition(m_transformStartPosition + (axisDirection * amount));
        return;
    }

    if (m_transformGizmoMode == TransformGizmoMode::Rotate)
    {
        const float amount = projectedDrag * 0.01f;
        Float4x4 rotation = identity();
        if (m_activeTransformAxis == TransformGizmoAxis::X)
        {
            rotation = rotationX(amount);
        }
        else if (m_activeTransformAxis == TransformGizmoAxis::Y)
        {
            rotation = rotationY(amount);
        }
        else if (m_activeTransformAxis == TransformGizmoAxis::Z)
        {
            rotation = rotationZ(amount);
        }

        SetSelectedOrientation(multiply(rotation, m_transformStartOrientation));
        return;
    }

    const float scaleAmount = std::max(0.05f, 1.0f + (projectedDrag * 0.01f));
    SetSelectedScale(m_transformStartScale * scaleAmount);
}

//Selects an object near the current mouse position.
void Scene::SelectObjectAtMouse(const InputUtility &inputUtility)
{
    if (m_lastScreenExtent.width == 0U || m_lastScreenExtent.height == 0U)
    {
        return;
    }

    const float screenWidth = static_cast<float>(std::max(1U, m_lastScreenExtent.width));
    const float screenHeight = static_cast<float>(std::max(1U, m_lastScreenExtent.height));
    const float aspect = screenWidth / screenHeight;
    const CameraBuffer cameraBuffer = CameraBufferInfo(aspect);
    const float ndcX = ((static_cast<float>(inputUtility.MouseX()) / screenWidth) * 2.0f) - 1.0f;
    const float ndcY = ((static_cast<float>(inputUtility.MouseY()) / screenHeight) * 2.0f) - 1.0f;
    const float tanHalfFovX = std::fabs(cameraBuffer.m_projection.m[0]) > 0.0001f ? 1.0f / cameraBuffer.m_projection.m[0] : 1.0f;
    const float tanHalfFovY = std::fabs(cameraBuffer.m_projection.m[5]) > 0.0001f ? -1.0f / cameraBuffer.m_projection.m[5] : 1.0f;
    const Float3 rayOrigin = m_eye.Position();
    const Float3 rayDirection = normalizeOrDefault(m_eye.Forward() + (m_eye.Right() * (ndcX * tanHalfFovX)) + (m_eye.Up() * (-ndcY * tanHalfFovY)), m_eye.Forward());
    SceneSelectionData bestSelection;
    float bestScore = std::numeric_limits<float>::max();

    auto TryPickPosition = [&](SceneObject &sceneObject, int instanceIndex, const Float3 &position, float selectionRadius)
    {
        float hitDistance = 0.0f;
        const float pickRadius = std::max(0.1f, selectionRadius * 1.12f);
        if (!RaySphereHit(rayOrigin, rayDirection, position, pickRadius, hitDistance))
        {
            return;
        }

        if (hitDistance < bestScore)
        {
            bestScore = hitDistance;
            bestSelection.m_objectPtr = &sceneObject;
            bestSelection.m_instanceIndex = instanceIndex;
            bestSelection.m_groupIndex = FindSceneObjectGroupIndex(&sceneObject);
        }
    };

    for (const auto &sceneObjectPtr : m_sceneObjectPtrs)
    {
        if (!sceneObjectPtr->Clickable())
        {
            continue;
        }

        const SceneObjectSelectionMode selectionMode = sceneObjectPtr->SelectionMode();
        if (selectionMode == SceneObjectSelectionMode::PerInstance && sceneObjectPtr->InstancingEnabled())
        {
            for (uint32_t instanceIndex = 0; instanceIndex < sceneObjectPtr->InstanceCount(); ++instanceIndex)
            {
                TryPickPosition( *sceneObjectPtr, static_cast<int>(instanceIndex), sceneObjectPtr->InstanceSelectionCenter(instanceIndex), sceneObjectPtr->InstanceSelectionRadius(instanceIndex));
            }
            continue;
        }

        if (selectionMode == SceneObjectSelectionMode::PerObject && sceneObjectPtr->InstancingEnabled())
        {
            for (uint32_t instanceIndex = 0; instanceIndex < sceneObjectPtr->InstanceCount(); ++instanceIndex)
            {
                TryPickPosition( *sceneObjectPtr, -1, sceneObjectPtr->InstanceSelectionCenter(instanceIndex), sceneObjectPtr->InstanceSelectionRadius(instanceIndex));
            }
            continue;
        }

        TryPickPosition(*sceneObjectPtr, -1, sceneObjectPtr->SelectionCenter(), sceneObjectPtr->SelectionRadius());
    }

    m_selection = bestSelection;
    m_activeTransformAxis = TransformGizmoAxis::None;

    if (SelectionActive())
    {
        const SceneObjectGroup *groupPtr = SelectedGroup();
        const std::string selectionName = groupPtr != nullptr ? groupPtr->m_name : m_selection.m_objectPtr->Name();
        UIUtility::Print("Scene object selected: " + selectionName + ". Use configured transform keys and drag gizmo handles.\n");
        ShowSelectedObjectPopup();
    }
    else
    {
        if (m_uiUtilityPtr != nullptr)
        {
            m_uiUtilityPtr->HidePopup();
        }

        UIUtility::Print("No clickable scene object selected.\n");
    }
}

//Shows the selected object debug popup.
void Scene::ShowSelectedObjectPopup(void)
{
    if (!SelectionActive())
    {
        return;
    }

    const std::string message = SelectedObjectPopupMessage();

    if (m_uiUtilityPtr != nullptr)
    {
        m_uiUtilityPtr->ShowPopup("Info", message, {}, UIUtility::PopupCallback{});
    }

    UIUtility::Print(message + "\n");
}

//Returns the selected object debug popup message.
std::string Scene::SelectedObjectPopupMessage(void) const
{
    if (!SelectionActive())
    {
        return "";
    }

    const SceneObjectGroup *groupPtr = SelectedGroup();
    if (groupPtr != nullptr)
    {
        return groupPtr->PopupMessage();
    }

    const SceneObject &selectedObject = *m_selection.m_objectPtr;
    const Float3 rotationDegrees = rotationDegreesFromMatrix(SelectedOrientation());
    return
        "Name: " + selectedObject.Name() + "\n" + (m_selection.m_instanceIndex >= 0 ? "Instance ID: " + std::to_string(m_selection.m_instanceIndex) + "\n" : "") +
        (selectedObject.InstancingEnabled() ? "Instance Count: " + std::to_string(selectedObject.InstanceCount()) + "\n" : "") + "Position: " + StringUtility::FormatFloat3(SelectedPosition()) + "\n" +
        "Rotation XYZ Degrees: " + StringUtility::FormatFloat3(rotationDegrees) + "\n" + "Scale: " + StringUtility::FormatFloat(SelectedScale());
}

//Updates the selected object debug popup text.
void Scene::UpdateSelectedObjectPopup(void)
{
    if (m_uiUtilityPtr == nullptr || !m_uiUtilityPtr->PopupVisible() || !SelectionActive())
    {
        return;
    }

    m_uiUtilityPtr->UpdatePopupText("Info", SelectedObjectPopupMessage());
}

//Returns whether a scene object is selected.
bool Scene::SelectionActive(void) const
{
    return SelectedGroup() != nullptr || m_selection.m_objectPtr != nullptr;
}

//Returns the selected transform position.
Float3 Scene::SelectedPosition(void) const
{
    if (!SelectionActive())
    {
        return { 0.0f, 0.0f, 0.0f };
    }

    const SceneObjectGroup *groupPtr = SelectedGroup();
    if (groupPtr != nullptr)
    {
        return groupPtr->m_position;
    }

    if (m_selection.m_instanceIndex >= 0)
    {
        return m_selection.m_objectPtr->InstancePosition(static_cast<uint32_t>(m_selection.m_instanceIndex));
    }

    return m_selection.m_objectPtr->Position();
}

//Returns the selected transform orientation.
Float4x4 Scene::SelectedOrientation(void) const
{
    if (!SelectionActive())
    {
        return identity();
    }

    const SceneObjectGroup *groupPtr = SelectedGroup();
    if (groupPtr != nullptr)
    {
        return groupPtr->m_orientation;
    }

    if (m_selection.m_instanceIndex >= 0)
    {
        return m_selection.m_objectPtr->InstanceOrientation(static_cast<uint32_t>(m_selection.m_instanceIndex));
    }

    return m_selection.m_objectPtr->Orientation();
}

//Returns the selected transform scale.
float Scene::SelectedScale(void) const
{
    if (!SelectionActive())
    {
        return 1.0f;
    }

    const SceneObjectGroup *groupPtr = SelectedGroup();
    if (groupPtr != nullptr)
    {
        return groupPtr->m_scale;
    }

    if (m_selection.m_instanceIndex >= 0)
    {
        return m_selection.m_objectPtr->InstanceScale(static_cast<uint32_t>(m_selection.m_instanceIndex));
    }

    return m_selection.m_objectPtr->Scale();
}

//Returns the selected transform radius.
float Scene::SelectedSelectionRadius(void) const
{
    if (!SelectionActive())
    {
        return 1.0f;
    }

    const SceneObjectGroup *groupPtr = SelectedGroup();
    if (groupPtr != nullptr)
    {
        return std::max(0.1f, groupPtr->m_selectionRadius * std::max(0.001f, groupPtr->m_scale));
    }

    if (m_selection.m_instanceIndex >= 0)
    {
        return m_selection.m_objectPtr->InstanceSelectionRadius(static_cast<uint32_t>(m_selection.m_instanceIndex));
    }

    return m_selection.m_objectPtr->SelectionRadius();
}

//Returns the selected gizmo display radius.
float Scene::SelectedGizmoRadius(void) const
{
    const float selectionRadius = SelectedSelectionRadius();
    const float distanceFromCamera = std::sqrt(dot(SelectedPosition() - m_eye.Position(), SelectedPosition() - m_eye.Position()));
    const float cameraScaledRadius = std::max(0.6f, distanceFromCamera * 0.08f);

    if (SelectedGroup() != nullptr)
    {
        return std::clamp(cameraScaledRadius, 0.8f, 3.0f);
    }

    return std::clamp(selectionRadius, 0.5f, cameraScaledRadius * 1.5f);
}

//Sets the selected transform position.
void Scene::SetSelectedPosition(const Float3 &position)
{
    if (!SelectionActive())
    {
        return;
    }

    SceneObjectGroup *groupPtr = SelectedGroup();
    if (groupPtr != nullptr)
    {
        groupPtr->m_position = position;
        groupPtr->Apply();
        UpdateSelectedObjectPopup();
        return;
    }

    if (m_selection.m_instanceIndex >= 0)
    {
        m_selection.m_objectPtr->SetInstanceTransform(static_cast<uint32_t>(m_selection.m_instanceIndex), position, SelectedOrientation(), SelectedScale());
        UpdateSelectedObjectPopup();
        return;
    }

    m_selection.m_objectPtr->SetPosition(position);
    UpdateSelectedObjectPopup();
}

//Sets the selected transform orientation.
void Scene::SetSelectedOrientation(const Float4x4 &orientation)
{
    if (!SelectionActive())
    {
        return;
    }

    SceneObjectGroup *groupPtr = SelectedGroup();
    if (groupPtr != nullptr)
    {
        groupPtr->m_orientation = orientation;
        groupPtr->Apply();
        UpdateSelectedObjectPopup();
        return;
    }

    if (m_selection.m_instanceIndex >= 0)
    {
        m_selection.m_objectPtr->SetInstanceTransform(static_cast<uint32_t>(m_selection.m_instanceIndex), SelectedPosition(), orientation, SelectedScale());
        UpdateSelectedObjectPopup();
        return;
    }

    m_selection.m_objectPtr->SetOrientation(orientation);
    UpdateSelectedObjectPopup();
}

//Sets the selected transform scale.
void Scene::SetSelectedScale(float scaleValue)
{
    if (!SelectionActive())
    {
        return;
    }

    SceneObjectGroup *groupPtr = SelectedGroup();
    if (groupPtr != nullptr)
    {
        groupPtr->m_scale = std::max(0.001f, scaleValue);
        groupPtr->Apply();
        UpdateSelectedObjectPopup();
        return;
    }

    if (m_selection.m_instanceIndex >= 0)
    {
        m_selection.m_objectPtr->SetInstanceTransform(static_cast<uint32_t>(m_selection.m_instanceIndex), SelectedPosition(), SelectedOrientation(), scaleValue);
        UpdateSelectedObjectPopup();
        return;
    }

    m_selection.m_objectPtr->SetScale(scaleValue);
    UpdateSelectedObjectPopup();
}

//Finds the group containing one scene object.
int Scene::FindSceneObjectGroupIndex(const SceneObject *sceneObjectPtr) const
{
    if (sceneObjectPtr == nullptr)
    {
        return -1;
    }

    for (std::size_t groupIndex = 0U; groupIndex < m_sceneObjectGroups.size(); ++groupIndex)
    {
        const SceneObjectGroup &group = m_sceneObjectGroups[groupIndex];
        for (const SceneObjectGroupChild &childData : group.m_children)
        {
            if (childData.m_objectPtr == sceneObjectPtr)
            {
                return static_cast<int>(groupIndex);
            }
        }
    }

    return -1;
}

//Returns the selected group pointer.
SceneObjectGroup *Scene::SelectedGroup(void)
{
    if (m_selection.m_groupIndex < 0 || static_cast<std::size_t>(m_selection.m_groupIndex) >= m_sceneObjectGroups.size())
    {
        return nullptr;
    }

    return &m_sceneObjectGroups[static_cast<std::size_t>(m_selection.m_groupIndex)];
}

//Returns the selected group pointer.
const SceneObjectGroup *Scene::SelectedGroup(void) const
{
    if (m_selection.m_groupIndex < 0 || static_cast<std::size_t>(m_selection.m_groupIndex) >= m_sceneObjectGroups.size())
    {
        return nullptr;
    }

    return &m_sceneObjectGroups[static_cast<std::size_t>(m_selection.m_groupIndex)];
}

//Returns the object projected screen position.
bool Scene::ProjectObjectToScreen(const SceneObject &sceneObject, Float2 &screenPosition, float &depth) const
{
    return ProjectWorldPointToScreen(sceneObject.SelectionCenter(), screenPosition, depth);
}

//Returns the world point projected screen position.
bool Scene::ProjectWorldPointToScreen(const Float3 &position, Float2 &screenPosition, float &depth) const
{
    if (m_lastScreenExtent.width == 0U || m_lastScreenExtent.height == 0U)
    {
        return false;
    }

    const float aspect = static_cast<float>(m_lastScreenExtent.width) / static_cast<float>(std::max(1U, m_lastScreenExtent.height));
    const CameraBuffer cameraBuffer = CameraBufferInfo(aspect);
    const Float4 clipPoint = transform(cameraBuffer.m_projection, transform(cameraBuffer.m_view, { position.x, position.y, position.z, 1.0f }));
    if (!std::isfinite(clipPoint.w) || clipPoint.w <= 0.0001f)
    {
        return false;
    }

    const float ndcX = clipPoint.x / clipPoint.w;
    const float ndcY = clipPoint.y / clipPoint.w;
    const float ndcZ = clipPoint.z / clipPoint.w;
    if (!std::isfinite(ndcX) || !std::isfinite(ndcY) || !std::isfinite(ndcZ) || ndcX < -1.1f || ndcX > 1.1f || ndcY < -1.1f || ndcY > 1.1f || ndcZ < 0.0f || ndcZ > 1.0f)
    {
        return false;
    }

    screenPosition = {
        ((ndcX * 0.5f) + 0.5f) * static_cast<float>(m_lastScreenExtent.width), ((ndcY * 0.5f) + 0.5f) * static_cast<float>(m_lastScreenExtent.height)
    };
    depth = std::max(0.001f, clipPoint.w);
    return true;
}

//Returns the transform axis direction.
Float3 Scene::TransformAxisDirection(TransformGizmoAxis axis) const
{
    if (axis == TransformGizmoAxis::X)
    {
        return { 1.0f, 0.0f, 0.0f };
    }

    if (axis == TransformGizmoAxis::Y)
    {
        return { 0.0f, 1.0f, 0.0f };
    }

    if (axis == TransformGizmoAxis::Z)
    {
        return { 0.0f, 0.0f, 1.0f };
    }

    return { 0.0f, 0.0f, 0.0f };
}

//Returns the active gizmo axis under the mouse.
Scene::TransformGizmoAxis Scene::HitTestTransformGizmo(const InputUtility &inputUtility) const
{
    if (!SelectionActive())
    {
        return TransformGizmoAxis::None;
    }

    Float2 centerScreen{};
    float depth = 1.0f;
    const Float3 selectedPosition = SelectedPosition();
    if (!ProjectWorldPointToScreen(selectedPosition, centerScreen, depth))
    {
        return TransformGizmoAxis::None;
    }
    static_cast<void>(depth);

    const Float2 mousePosition{
        static_cast<float>(inputUtility.MouseX()), static_cast<float>(inputUtility.MouseY())
    };
    const std::array<TransformGizmoAxis, 3> axes{
        TransformGizmoAxis::X, TransformGizmoAxis::Y, TransformGizmoAxis::Z
    };

    if (m_transformGizmoMode == TransformGizmoMode::Rotate)
    {
        const float distance = std::sqrt(ScreenDistanceSquared(mousePosition, centerScreen));
        for (std::size_t axisIndex = 0U; axisIndex < axes.size(); ++axisIndex)
        {
            const float radius = 44.0f + (static_cast<float>(axisIndex) * 14.0f);
            if (std::fabs(distance - radius) <= 7.0f)
            {
                return axes[axisIndex];
            }
        }

        return TransformGizmoAxis::None;
    }

    for (std::size_t axisIndex = 0U; axisIndex < axes.size(); ++axisIndex)
    {
        const Float3 axisDirection = TransformAxisDirection(axes[axisIndex]);
        const Float3 axisPoint = selectedPosition + (axisDirection * SelectedGizmoRadius() * 1.6f);
        Float2 axisScreen{};
        float axisDepth = 1.0f;
        if (!ProjectWorldPointToScreen(axisPoint, axisScreen, axisDepth))
        {
            continue;
        }
        static_cast<void>(axisDepth);

        if (ScreenDistanceSquared(mousePosition, axisScreen) <= 16.0f * 16.0f)
        {
            return axes[axisIndex];
        }
    }

    return TransformGizmoAxis::None;
}

//Draws the transform gizmo.
void Scene::DrawTransformGizmo(UIUtility &uiUtility, const UISetup &uiSetup, VkExtent2D swapChainExtent) const
{
    static_cast<void>(swapChainExtent);

    if (!SelectionActive())
    {
        return;
    }

    Float2 centerScreen{};
    float depth = 1.0f;
    const Float3 selectedPosition = SelectedPosition();
    if (!ProjectWorldPointToScreen(selectedPosition, centerScreen, depth))
    {
        return;
    }
    static_cast<void>(depth);

    const std::array<TransformGizmoAxis, 3> axes{
        TransformGizmoAxis::X, TransformGizmoAxis::Y, TransformGizmoAxis::Z
    };
    const std::array<std::string, 3> labels{ "X", "Y", "Z" };
    const std::string modeText = m_transformGizmoMode == TransformGizmoMode::Translate ? "Translate" : (m_transformGizmoMode == TransformGizmoMode::Rotate ? "Rotate" : "Scale");

    uiUtility.DrawText(centerScreen.x + 13.0f, centerScreen.y - 38.0f, modeText, uiSetup.m_textShadowColor, 1.65f);
    uiUtility.DrawText(centerScreen.x + 12.0f, centerScreen.y - 39.0f, modeText, uiSetup.m_textColor, 1.65f);

    if (m_transformGizmoMode == TransformGizmoMode::Rotate)
    {
        constexpr int segmentCount = 48;
        for (std::size_t axisIndex = 0U; axisIndex < axes.size(); ++axisIndex)
        {
            const float radius = 44.0f + (static_cast<float>(axisIndex) * 14.0f);
            const Float4 color = AxisColor(static_cast<uint32_t>(axisIndex));
            Float2 previousPoint{
                centerScreen.x + radius, centerScreen.y
            };

            for (int segmentIndex = 1; segmentIndex <= segmentCount; ++segmentIndex)
            {
                const float angle = (static_cast<float>(segmentIndex) / static_cast<float>(segmentCount)) * Pi * 2.0f;
                const Float2 currentPoint{
                    centerScreen.x + std::cos(angle) * radius, centerScreen.y + std::sin(angle) * radius
                };
                uiUtility.DrawLine(previousPoint.x, previousPoint.y, currentPoint.x, currentPoint.y, color, 2.0f);
                previousPoint = currentPoint;
            }
        }

        return;
    }

    for (std::size_t axisIndex = 0U; axisIndex < axes.size(); ++axisIndex)
    {
        const Float3 axisDirection = TransformAxisDirection(axes[axisIndex]);
        const Float3 axisPoint = selectedPosition + (axisDirection * SelectedGizmoRadius() * 1.6f);

        Float2 axisScreen{};
        float axisDepth = 1.0f;
        if (!ProjectWorldPointToScreen(axisPoint, axisScreen, axisDepth))
        {
            continue;
        }
        static_cast<void>(axisDepth);

        const Float4 color = AxisColor(static_cast<uint32_t>(axisIndex));
        uiUtility.DrawLine(centerScreen.x, centerScreen.y, axisScreen.x, axisScreen.y, color, 3.0f);

        if (m_transformGizmoMode == TransformGizmoMode::Scale)
        {
            uiUtility.DrawLine(axisScreen.x - 8.0f, axisScreen.y - 8.0f, axisScreen.x + 8.0f, axisScreen.y - 8.0f, color, 3.0f);
            uiUtility.DrawLine(axisScreen.x + 8.0f, axisScreen.y - 8.0f, axisScreen.x + 8.0f, axisScreen.y + 8.0f, color, 3.0f);
            uiUtility.DrawLine(axisScreen.x + 8.0f, axisScreen.y + 8.0f, axisScreen.x - 8.0f, axisScreen.y + 8.0f, color, 3.0f);
            uiUtility.DrawLine(axisScreen.x - 8.0f, axisScreen.y + 8.0f, axisScreen.x - 8.0f, axisScreen.y - 8.0f, color, 3.0f);
        }
        else
        {
            uiUtility.DrawLine(axisScreen.x - 7.0f, axisScreen.y, axisScreen.x + 7.0f, axisScreen.y, color, 3.0f);
            uiUtility.DrawLine(axisScreen.x, axisScreen.y - 7.0f, axisScreen.x, axisScreen.y + 7.0f, color, 3.0f);
        }

        uiUtility.DrawText(axisScreen.x + 7.0f, axisScreen.y + 7.0f, labels[axisIndex], uiSetup.m_textShadowColor, 1.7f);
        uiUtility.DrawText(axisScreen.x + 6.0f, axisScreen.y + 6.0f, labels[axisIndex], color, 1.7f);
    }
}

//Initializes GPU resources for all scene objects.
void Scene::InitSceneObjects(const SceneObjectContext &context)
{
    for (const auto &sceneObjectPtr : m_sceneObjectPtrs)
    {
        sceneObjectPtr->Init(context);
    }
}

//Creates one instanced scene object data record.
SceneObjectInstanceData Scene::CreateInstanceData(const Float3 &position, float size, const Float4 &color, const Float4x4 &orientation) const
{
    SceneObjectInstanceData instanceData;
    instanceData.m_model = multiply(translation(position.x, position.y, position.z), multiply(orientation, scale(size)));
    instanceData.m_color = color;
    return instanceData;
}

//Returns the configured COLMAP point display mode.
Scene::ColmapPointDisplayMode Scene::ParseColmapPointDisplayMode(std::string pointDisplayMode)
{
    const std::string mode = StringUtility::Uppercase(std::move(pointDisplayMode));
    if (mode == "POINTS3D" || mode == "POINTS_3D" || mode == "SPARSE")
    {
        return ColmapPointDisplayMode::Points3D;
    }

    if (mode == "DEPTH" || mode == "UNPROJECT" || mode == "UNPROJECTED")
    {
        return ColmapPointDisplayMode::Depth;
    }

    if (mode == "PLANESWEEP" || mode == "PLANE_SWEEP" || mode == "SWEEP")
    {
        return ColmapPointDisplayMode::PlaneSweep;
    }

    if (mode == "OFF" || mode == "NONE" || mode == "DISABLED")
    {
        return ColmapPointDisplayMode::Off;
    }

    return ColmapPointDisplayMode::Auto;
}

//Advances one deterministic random state.
uint32_t Scene::NextRandomState(uint32_t &state)
{
    state = (state * 1664525U) + 1013904223U;
    return state;
}

//Creates one deterministic random rotation angle.
float Scene::RandomAngle(uint32_t &state)
{
    constexpr float maxRandomValue = 16777215.0f;
    const uint32_t randomValue = NextRandomState(state) & 0x00FFFFFFU;
    return (static_cast<float>(randomValue) / maxRandomValue) * Pi * 2.0f;
}

//Creates one deterministic random orientation matrix.
Float4x4 Scene::RandomOrientation(uint32_t seed)
{
    uint32_t state = seed;
    const Float4x4 rotateX = rotationX(RandomAngle(state));
    const Float4x4 rotateY = rotationY(RandomAngle(state));
    const Float4x4 rotateZ = rotationZ(RandomAngle(state));

    return multiply(rotateZ, multiply(rotateY, rotateX));
}

//Returns whether setup requests a line grid.
bool Scene::GridModeIsLine(std::string gridMode)
{
    const std::string mode = StringUtility::Uppercase(std::move(gridMode));
    return mode == "LINE" || mode == "LINES" || mode == "LINEGRID";
}

//Returns an example object position lifted above the grid.
Float3 Scene::LiftExamplePosition(const Float3 &position)
{
    return { position.x, position.y + 0.5f, position.z };
}

//Returns whether a ray intersects a sphere.
bool Scene::RaySphereHit(const Float3 &origin, const Float3 &direction, const Float3 &center, float radius, float &hitDistance)
{
    const float safeRadius = std::max(0.001f, radius);
    const Float3 offset = center - origin;
    const float centerDistance = dot(offset, direction);
    const float offsetLengthSquared = dot(offset, offset);
    const float closestDistanceSquared = offsetLengthSquared - (centerDistance * centerDistance);
    const float radiusSquared = safeRadius * safeRadius;

    if (closestDistanceSquared > radiusSquared)
    {
        return false;
    }

    const float halfDistance = std::sqrt(std::max(0.0f, radiusSquared - closestDistanceSquared));
    hitDistance = centerDistance - halfDistance;
    if (hitDistance < 0.0f)
    {
        hitDistance = centerDistance + halfDistance;
    }

    return std::isfinite(hitDistance) && hitDistance >= 0.0f;
}

//Returns the squared distance between two screen points.
float Scene::ScreenDistanceSquared(const Float2 &a, const Float2 &b)
{
    const float deltaX = a.x - b.x;
    const float deltaY = a.y - b.y;
    return (deltaX * deltaX) + (deltaY * deltaY);
}

//Returns a normalized screen direction.
Float2 Scene::NormalizeScreenDirection(const Float2 &value)
{
    const float vectorLength = std::sqrt((value.x * value.x) + (value.y * value.y));
    if (vectorLength <= 0.0001f)
    {
        return { 1.0f, 0.0f };
    }

    return { value.x / vectorLength, value.y / vectorLength };
}

//Returns the display color for one axis.
Float4 Scene::AxisColor(uint32_t axisIndex)
{
    if (axisIndex == 0U)
    {
        return { 1.0f, 0.16f, 0.12f, 1.0f };
    }

    if (axisIndex == 1U)
    {
        return { 0.12f, 0.78f, 0.22f, 1.0f };
    }

    return { 0.18f, 0.38f, 1.0f, 1.0f };
}

//Returns a camera pyramid orientation matrix.
Float4x4 Scene::CameraPyramidOrientation(const ColmapCameraData &camera)
{
    const Float3 forward = normalizeOrDefault(camera.m_forward, { 0.0f, 0.0f, -1.0f });
    const Float3 right = normalizeOrDefault(camera.m_right, normalizeOrDefault(cross(forward, camera.m_up), { 1.0f, 0.0f, 0.0f }));
    const Float3 up = normalizeOrDefault(cross(right, forward), { 0.0f, 1.0f, 0.0f });

    Float4x4 orientation = identity();
    orientation.m[0] = right.x;
    orientation.m[1] = right.y;
    orientation.m[2] = right.z;
    orientation.m[4] = up.x;
    orientation.m[5] = up.y;
    orientation.m[6] = up.z;
    orientation.m[8] = -forward.x;
    orientation.m[9] = -forward.y;
    orientation.m[10] = -forward.z;
    return orientation;
}

//Returns the stable COLMAP visualization local center.
bool Scene::ColmapStableLocalCenter(const ColmapCameraReadResult &colmapData, const ColmapSetup &colmapSetup, Float3 &center)
{
    bool hasBounds = false;
    Float3 boundsMinimum{ 0.0f, 0.0f, 0.0f };
    Float3 boundsMaximum{ 0.0f, 0.0f, 0.0f };
    const float cameraRadius = std::max(0.0f, colmapSetup.m_cameraSize);

    for (const ColmapCameraData &camera : colmapData.m_cameras)
    {
        const Float3 minimum{
            camera.m_position.x - cameraRadius, camera.m_position.y - cameraRadius, camera.m_position.z - cameraRadius
        };
        const Float3 maximum{
            camera.m_position.x + cameraRadius, camera.m_position.y + cameraRadius, camera.m_position.z + cameraRadius
        };

        if (!hasBounds)
        {
            boundsMinimum = minimum;
            boundsMaximum = maximum;
            hasBounds = true;
            continue;
        }

        boundsMinimum.x = std::min(boundsMinimum.x, minimum.x);
        boundsMinimum.y = std::min(boundsMinimum.y, minimum.y);
        boundsMinimum.z = std::min(boundsMinimum.z, minimum.z);
        boundsMaximum.x = std::max(boundsMaximum.x, maximum.x);
        boundsMaximum.y = std::max(boundsMaximum.y, maximum.y);
        boundsMaximum.z = std::max(boundsMaximum.z, maximum.z);
    }

    if (!hasBounds)
    {
        return false;
    }

    center = (boundsMinimum + boundsMaximum) * 0.5f;
    return true;
}

//Returns the COLMAP visualization base rotation matrix.
Float4x4 Scene::ColmapBaseRotationMatrix(const ColmapSetup &colmapSetup)
{
    constexpr float degreesToRadians = Pi / 180.0f;
    const Float4x4 rotateX = rotationX(colmapSetup.m_baseRotationDegrees.x * degreesToRadians);
    const Float4x4 rotateY = rotationY(colmapSetup.m_baseRotationDegrees.y * degreesToRadians);
    const Float4x4 rotateZ = rotationZ(colmapSetup.m_baseRotationDegrees.z * degreesToRadians);
    return multiply(rotateZ, multiply(rotateY, rotateX));
}

//Returns whether setup requests Gaussian splat streaming.
bool Scene::GaussianLoadModeIsStream(std::string loadMode)
{
    const std::string mode = StringUtility::Uppercase(StringUtility::Trim(loadMode));
    return mode == "STREAM" || mode == "STREAMING";
}

//Returns the Gaussian splat visualization base rotation matrix.
Float4x4 Scene::GaussianSplatBaseRotationMatrix(const GaussianSplatSetup &gaussianSplatSetup)
{
    constexpr float degreesToRadians = Pi / 180.0f;
    const Float4x4 rotateX = rotationX(gaussianSplatSetup.m_baseRotationDegrees.x * degreesToRadians);
    const Float4x4 rotateY = rotationY(gaussianSplatSetup.m_baseRotationDegrees.y * degreesToRadians);
    const Float4x4 rotateZ = rotationZ(gaussianSplatSetup.m_baseRotationDegrees.z * degreesToRadians);
    return multiply(rotateZ, multiply(rotateY, rotateX));
}

//Returns an average timing value.
double Scene::AverageTiming(double totalMilliseconds, std::size_t sampleCount)
{
    return sampleCount == 0U ? 0.0 : totalMilliseconds / static_cast<double>(sampleCount);
}
