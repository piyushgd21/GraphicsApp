///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	Camera.h
//	Description	:
//		Defines camera movement and orientation behavior.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "RenderTypes.h"
#include "SetupUtility.h"

namespace graphicsFX
{
    class InputUtility;

    //Defines camera movement and orientation behavior.
    class Camera
    {

        public:

            //Creates the camera.
            Camera(void) = default;

            //Destroys the camera.
            ~Camera(void) = default;

            Camera(const Camera &) = delete;
            Camera &operator=(const Camera &) = delete;

            //Initializes the camera.
            void Init(const CameraSetup &cameraSetup, const KeyboardSetup &keyboardSetup, const MouseSetup &mouseSetup);

            //Updates the camera from input.
            void UpdateInput(const InputUtility &inputUtility, float deltaSeconds);

            //Updates the camera from input using a movement basis.
            void UpdateInput(const InputUtility &inputUtility, float deltaSeconds, const CameraMovementBasis *movementBasisPtr);

            //Updates the camera from input and controller state.
            void UpdateInput(const InputUtility &inputUtility, float deltaSeconds, const CameraMovementBasis *movementBasisPtr, const ControllerMovementState *controllerStatePtr);

            //Returns the view matrix.
            Float4x4 ViewMatrix(void) const;

            //Returns the projection matrix.
            Float4x4 ProjectionMatrix(float aspect) const;

            //Returns the camera position.
            Float3 Position(void) const;

            //Returns the forward direction.
            Float3 Forward(void) const;

            //Returns the right direction.
            Float3 Right(void) const;

            //Returns the up direction.
            Float3 Up(void) const;

            //Moves the camera.
            void Move(const Float3 &amount);

            //Rotates the camera pitch.
            void Pitch(float radians);

            //Rotates the camera yaw.
            void Yaw(float radians);

            //Rotates the camera roll.
            void Roll(float radians);

        private:

            //Returns whether a mapped key is down.
            static bool MappedKeyDown(const InputUtility &inputUtility, uint32_t keyCode);

            //Returns a controller axis after dead-zone filtering.
            static float ControllerAxis(float value);

            static constexpr float s_pitchLimit = Pi * 0.49f; //Stores the maximum pitch angle.
            static constexpr float s_mouseWheelDelta = 120.0f; //Stores Win32 mouse wheel units per notch.
            static constexpr float s_controllerDeadZone = 0.15f; //Stores controller stick dead zone.

            Float3 m_position{ 0.0f, 0.0f, 5.0f }; //Stores camera position.
            Float3 m_worldUp{ 0.0f, 1.0f, 0.0f }; //Stores camera world up direction.
            KeyboardSetup m_keyboardSetup; //Stores camera keyboard mappings.
            MouseSetup m_mouseSetup; //Stores camera mouse mappings.
            float m_pitch{ 0.0f }; //Stores pitch rotation.
            float m_yaw{ 0.0f }; //Stores yaw rotation.
            float m_roll{ 0.0f }; //Stores roll rotation.
            float m_fovDegrees{ 45.0f }; //Stores field of view in degrees.
            float m_nearPlane{ 0.1f }; //Stores near clip plane distance.
            float m_farPlane{ 100.0f }; //Stores far clip plane distance.
            float m_moveSpeed{ 4.0f }; //Stores movement speed.
            float m_rotateSpeed{ 1.8f }; //Stores keyboard rotation speed.
            float m_mouseSensitivity{ 0.004f }; //Stores mouse rotation sensitivity.
            float m_mouseWheelMoveAmount{ 0.75f }; //Stores mouse wheel movement per notch.
            float m_smoothMouseMoveSensitivity{ 0.025f }; //Stores smooth mouse drag movement sensitivity.
            float m_fastMultiplier{ 3.0f }; //Stores fast movement multiplier.
            float m_slowMultiplier{ 0.25f }; //Stores slow movement multiplier.
    };
}

