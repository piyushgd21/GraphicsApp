///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	SceneInterface.h
//	Description	:
//		Defines the interface used by EngineCore to run an application scene.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "global.h"

#include "RenderTypes.h"
#include "SceneObject.h"
#include "SetupUtility.h"

#include <vulkan/vulkan.h>

namespace graphicsFX
{
    class InputUtility;
    class UIUtility;

    //Defines the scene callbacks required by EngineCore.
    class SceneInterface
    {

        public:

            //Destroys the scene interface.
            virtual ~SceneInterface(void) = default;

            //Initializes scene objects and scene UI.
            virtual void Init(const SceneObjectContext &context, const ProgramSetup &programSetup, UIUtility &uiUtility) = 0;

            //Updates scene objects.
            virtual void Update(float elapsedSeconds, float deltaSeconds) = 0;

            //Updates scene input state.
            virtual void UpdateInput(const InputUtility &inputUtility, float deltaSeconds, const CameraMovementBasis *movementBasisPtr, const ControllerMovementState *controllerStatePtr) = 0;

            //Returns the active camera buffer.
            virtual CameraBuffer CameraBufferInfo(float aspect) const = 0;

            //Returns whether frame capture was requested.
            virtual bool ConsumeCaptureFrameRequest(void) = 0;

            //Draws scene UI.
            virtual void ShowUI(UIUtility &uiUtility, const UISetup &uiSetup, float framesPerSecond, VkExtent2D swapChainExtent) const = 0;

            //Draws scene objects.
            virtual void Show(VkCommandBuffer commandBufferPtrGPU, VkDescriptorSet cameraDescriptorSetPtrGPU) const = 0;

            //Draws scene objects with an explicit render extent.
            virtual void Show(VkCommandBuffer commandBufferPtrGPU, VkDescriptorSet cameraDescriptorSetPtrGPU, VkExtent2D renderExtent) const = 0;

            //Draws scene objects into the shadow map.
            virtual void ShowShadow(VkCommandBuffer commandBufferPtrGPU, VkDescriptorSet cameraDescriptorSetPtrGPU, VkExtent2D shadowExtent) const = 0;

            //Releases scene object resources.
            virtual void Clean(void) = 0;

            //Releases scene object pipelines.
            virtual void CleanPipelines(void) = 0;

            //Recreates scene object pipelines.
            virtual void RefreshPipelines(const SceneObjectContext &context) = 0;
    };
}

