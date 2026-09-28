///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	Camera.cpp
//	Description	:
//		Implements camera movement and orientation behavior.
//
//
///////////////////////////////////////////////////////////////////////

#include "Camera.h"

#include "InputUtility.h"

#include <algorithm>
#include <cmath>

namespace graphicsFX
{
    //Initializes the camera.
    void Camera::Init(const CameraSetup &cameraSetup, const KeyboardSetup &keyboardSetup, const MouseSetup &mouseSetup)
    {
        m_keyboardSetup = keyboardSetup;
        m_mouseSetup = mouseSetup;
        m_fovDegrees = cameraSetup.m_fovDegrees;
        m_nearPlane = cameraSetup.m_nearPlane;
        m_farPlane = std::max(cameraSetup.m_farPlane, cameraSetup.m_nearPlane + 0.001f);
        m_moveSpeed = cameraSetup.m_moveSpeed;
        m_rotateSpeed = cameraSetup.m_rotateSpeed;
        m_mouseSensitivity = cameraSetup.m_mouseSensitivity;
        m_mouseWheelMoveAmount = cameraSetup.m_mouseWheelMoveAmount;
        m_smoothMouseMoveSensitivity = cameraSetup.m_smoothMouseMoveSensitivity;
        m_fastMultiplier = cameraSetup.m_fastMultiplier;
        m_slowMultiplier = cameraSetup.m_slowMultiplier;
        
        m_position = cameraSetup.m_position;
        m_worldUp = normalizeOrDefault(cameraSetup.m_up, { 0.0f, 1.0f, 0.0f });

        //const Float3 forward = normalizeOrDefault(cameraSetup.m_lookAt, { 0.0f, 0.0f, -1.0f });
        //m_pitch = std::asin(std::clamp(forward.y, -1.0f, 1.0f));
        //m_yaw = std::atan2(forward.x, -forward.z);
        //
        //m_roll = 0.0f;
        //const Float3 baseRight = normalizeOrDefault(cross(forward, m_worldUp), { 1.0f, 0.0f, 0.0f });
        //const Float3 baseUp = normalizeOrDefault(cross(baseRight, forward), m_worldUp);
        //const Float3 side = normalizeOrDefault(cameraSetup.m_side, baseRight);
        //m_roll = std::atan2(dot(side, baseUp), dot(side, baseRight));


        const Float3 forward = normalizeOrDefault(cameraSetup.m_lookAt, { 0.0f, 0.0f, -1.0f });
        Float3 baseRight = normalizeOrDefault(cameraSetup.m_side, cross(forward, m_worldUp));
        Float3 baseUp = normalizeOrDefault(cameraSetup.m_up, cross(baseRight, forward));
        
        //Re-orthonormalize
        baseRight = normalize(baseRight - forward * dot(baseRight, forward));
        baseUp = normalize(cross(baseRight, forward));


        m_pitch = std::asin(std::clamp(forward.y, -1.0f, 1.0f));
        m_yaw = std::atan2(forward.x, -forward.z);
        m_roll = std::atan2(dot(baseRight, baseUp), dot(baseRight, baseRight));

    }

    //Updates the camera from input.
    void Camera::UpdateInput(const InputUtility &inputUtility, float deltaSeconds)
    {
        UpdateInput(inputUtility, deltaSeconds, nullptr);
    }

    //Updates the camera from input using a movement basis.
    void Camera::UpdateInput(const InputUtility &inputUtility, float deltaSeconds, const CameraMovementBasis *movementBasisPtr)
    {
        UpdateInput(inputUtility, deltaSeconds, movementBasisPtr, nullptr);
    }

    //Updates the camera from input and controller state.
    void Camera::UpdateInput(const InputUtility &inputUtility, float deltaSeconds, const CameraMovementBasis *movementBasisPtr, const ControllerMovementState *controllerStatePtr)
    {
        float moveSpeed = m_moveSpeed;
        if (MappedKeyDown(inputUtility, m_keyboardSetup.m_fastKey))
        {
            moveSpeed *= m_fastMultiplier;
        }

        if (MappedKeyDown(inputUtility, m_keyboardSetup.m_slowKey))
        {
            moveSpeed *= m_slowMultiplier;
        }

        const float moveAmount = moveSpeed * deltaSeconds;
        Float3 movement{ 0.0f, 0.0f, 0.0f };
        const Float3 forwardDirection = movementBasisPtr != nullptr ? normalizeOrDefault(movementBasisPtr->m_forward, Forward()) : Forward();
        const Float3 rightDirection = movementBasisPtr != nullptr ? normalizeOrDefault(movementBasisPtr->m_right, Right()) : Right();
        const Float3 upDirection = movementBasisPtr != nullptr ? normalizeOrDefault(movementBasisPtr->m_up, Up()) : Up();

        if (MappedKeyDown(inputUtility, m_keyboardSetup.m_moveForwardKey))
        {
            movement += forwardDirection * moveAmount;
        }

        if (MappedKeyDown(inputUtility, m_keyboardSetup.m_moveBackwardKey))
        {
            movement += forwardDirection * -moveAmount;
        }

        if (MappedKeyDown(inputUtility, m_keyboardSetup.m_moveLeftKey))
        {
            movement += rightDirection * -moveAmount;
        }

        if (MappedKeyDown(inputUtility, m_keyboardSetup.m_moveRightKey))
        {
            movement += rightDirection * moveAmount;
        }

        if (MappedKeyDown(inputUtility, m_keyboardSetup.m_moveUpKey))
        {
            movement += upDirection * moveAmount;
        }

        if (MappedKeyDown(inputUtility, m_keyboardSetup.m_moveDownKey))
        {
            movement += upDirection * -moveAmount;
        }

        if (controllerStatePtr != nullptr && controllerStatePtr->m_active)
        {
            const float controllerMoveX = ControllerAxis(controllerStatePtr->m_move.x);
            const float controllerMoveY = ControllerAxis(controllerStatePtr->m_move.y);
            movement += rightDirection * (controllerMoveX * moveAmount);
            movement += forwardDirection * (controllerMoveY * moveAmount);

            if (controllerStatePtr->m_moveUp)
            {
                movement += upDirection * moveAmount;
            }

            if (controllerStatePtr->m_moveDown)
            {
                movement += upDirection * -moveAmount;
            }
        }

        Move(movement);

        const int mouseWheelDelta = inputUtility.MouseWheelDelta();
        if (mouseWheelDelta != 0)
        {
            const float mouseWheelSteps = static_cast<float>(mouseWheelDelta) / s_mouseWheelDelta;
            Move(forwardDirection * (mouseWheelSteps * m_mouseWheelMoveAmount));
        }

        if (inputUtility.MouseButtonDown(m_mouseSetup.m_smoothMoveButton))
        {
            const float smoothMoveAmount = static_cast<float>(-inputUtility.MouseDeltaY()) * m_smoothMouseMoveSensitivity;
            Move(forwardDirection * smoothMoveAmount);
        }

        const float rotateAmount = m_rotateSpeed * deltaSeconds;
        if (MappedKeyDown(inputUtility, m_keyboardSetup.m_pitchUpKey))
        {
            Pitch(rotateAmount);
        }

        if (MappedKeyDown(inputUtility, m_keyboardSetup.m_pitchDownKey))
        {
            Pitch(-rotateAmount);
        }

        if (MappedKeyDown(inputUtility, m_keyboardSetup.m_yawLeftKey))
        {
            Yaw(-rotateAmount);
        }

        if (MappedKeyDown(inputUtility, m_keyboardSetup.m_yawRightKey))
        {
            Yaw(rotateAmount);
        }

        if (controllerStatePtr != nullptr && controllerStatePtr->m_active)
        {
            const float controllerTurnX = ControllerAxis(controllerStatePtr->m_turn.x);
            const float controllerTurnY = ControllerAxis(controllerStatePtr->m_turn.y);
            Yaw(controllerTurnX * rotateAmount);
            Pitch(controllerTurnY * rotateAmount);
        }

        if (MappedKeyDown(inputUtility, m_keyboardSetup.m_rollLeftKey))
        {
            Roll(-rotateAmount);
        }

        if (MappedKeyDown(inputUtility, m_keyboardSetup.m_rollRightKey))
        {
            Roll(rotateAmount);
        }

        if (inputUtility.MouseButtonDown(m_mouseSetup.m_lookButton))
        {
            Yaw(static_cast<float>(inputUtility.MouseDeltaX()) * m_mouseSensitivity);
            Pitch(static_cast<float>(-inputUtility.MouseDeltaY()) * m_mouseSensitivity);
        }
    }

    //Returns the view matrix.
    Float4x4 Camera::ViewMatrix(void) const
    {
        return lookAt(m_position, m_position + Forward(), Up());
    }

    //Returns the projection matrix.
    Float4x4 Camera::ProjectionMatrix(float aspect) const
    {
        return perspective(m_fovDegrees * Pi / 180.0f, aspect, m_nearPlane, m_farPlane);
    }

    //Returns the camera position.
    Float3 Camera::Position(void) const
    {
        return m_position;
    }

    //Returns the forward direction.
    Float3 Camera::Forward(void) const
    {
        const float cosPitch = std::cos(m_pitch);
        return normalize({ cosPitch * std::sin(m_yaw), std::sin(m_pitch), -cosPitch * std::cos(m_yaw) });
    }

    //Returns the right direction.
    Float3 Camera::Right(void) const
    {
        const Float3 forward = Forward();
        const Float3 baseRight = normalizeOrDefault(cross(forward, m_worldUp), { 1.0f, 0.0f, 0.0f });
        const Float3 baseUp = normalizeOrDefault(cross(baseRight, forward), m_worldUp);
        const float cosRoll = std::cos(m_roll);
        const float sinRoll = std::sin(m_roll);

        return normalize((baseRight * cosRoll) + (baseUp * sinRoll));
    }

    //Returns the up direction.
    Float3 Camera::Up(void) const
    {
        return normalize(cross(Right(), Forward()));
    }

    //Moves the camera.
    void Camera::Move(const Float3 &amount)
    {
        m_position += amount;
    }

    //Rotates the camera pitch.
    void Camera::Pitch(float radians)
    {
        m_pitch = std::clamp(m_pitch + radians, -s_pitchLimit, s_pitchLimit);
    }

    //Rotates the camera yaw.
    void Camera::Yaw(float radians)
    {
        m_yaw += radians;
    }

    //Rotates the camera roll.
    void Camera::Roll(float radians)
    {
        m_roll += radians;
    }

    //Returns whether a mapped key is down.
    bool Camera::MappedKeyDown(const InputUtility &inputUtility, uint32_t keyCode)
    {
        return keyCode != 0U && inputUtility.KeyDown(keyCode);
    }

    //Returns a controller axis after dead-zone filtering.
    float Camera::ControllerAxis(float value)
    {
        return std::abs(value) < s_controllerDeadZone ? 0.0f : std::clamp(value, -1.0f, 1.0f);
    }
}

