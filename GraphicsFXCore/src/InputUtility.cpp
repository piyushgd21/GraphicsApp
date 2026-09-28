///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	InputUtility.cpp
//	Description	:
//		Implements keyboard and mouse input state handling.
//
//
///////////////////////////////////////////////////////////////////////

#include "InputUtility.h"

#include <Windowsx.h>

namespace graphicsFX
{
    //Starts a new input frame.
    void InputUtility::BeginFrame(void)
    {
        m_previousKeyDowns = m_keyDowns;
        m_previousMouseButtonDowns = m_mouseButtonDowns;
        m_mouseDeltaX = 0;
        m_mouseDeltaY = 0;
        m_mouseWheelDelta = 0;
        m_mouseButtonDoubleClicks = {};
    }

    //Handles one Win32 input message.
    void InputUtility::HandleMessage(UINT message, WPARAM wParam, LPARAM lParam)
    {
        switch (message)
        {
        case WM_KEYDOWN:
        case WM_SYSKEYDOWN: SetKey(static_cast<uint32_t>(wParam), true);
            break;

        case WM_KEYUP:
        case WM_SYSKEYUP: SetKey(static_cast<uint32_t>(wParam), false);
            break;

        case WM_MOUSEMOVE: UpdateMousePosition(lParam);
            break;

        case WM_LBUTTONDOWN: UpdateMousePosition(lParam);
            SetMouseButton(s_mouseLeftButton, true);
            break;

        case WM_LBUTTONDBLCLK: UpdateMousePosition(lParam);
            SetMouseButton(s_mouseLeftButton, true);
            m_mouseButtonDoubleClicks[s_mouseLeftButton] = true;
            break;

        case WM_LBUTTONUP: UpdateMousePosition(lParam);
            SetMouseButton(s_mouseLeftButton, false);
            break;

        case WM_RBUTTONDOWN: UpdateMousePosition(lParam);
            SetMouseButton(s_mouseRightButton, true);
            break;

        case WM_RBUTTONDBLCLK: UpdateMousePosition(lParam);
            SetMouseButton(s_mouseRightButton, true);
            m_mouseButtonDoubleClicks[s_mouseRightButton] = true;
            break;

        case WM_RBUTTONUP: UpdateMousePosition(lParam);
            SetMouseButton(s_mouseRightButton, false);
            break;

        case WM_MBUTTONDOWN: UpdateMousePosition(lParam);
            SetMouseButton(s_mouseMiddleButton, true);
            break;

        case WM_MBUTTONDBLCLK: UpdateMousePosition(lParam);
            SetMouseButton(s_mouseMiddleButton, true);
            m_mouseButtonDoubleClicks[s_mouseMiddleButton] = true;
            break;

        case WM_MBUTTONUP: UpdateMousePosition(lParam);
            SetMouseButton(s_mouseMiddleButton, false);
            break;

        case WM_XBUTTONDOWN: UpdateMousePosition(lParam);
            SetMouseButton(XButtonFromWParam(wParam), true);
            break;

        case WM_XBUTTONDBLCLK:
        {
            UpdateMousePosition(lParam);
            const uint32_t button = XButtonFromWParam(wParam);
            SetMouseButton(button, true);
            if (button < m_mouseButtonDoubleClicks.size())
            {
                m_mouseButtonDoubleClicks[button] = true;
            }
            break;
        }

        case WM_XBUTTONUP: UpdateMousePosition(lParam);
            SetMouseButton(XButtonFromWParam(wParam), false);
            break;

        case WM_MOUSEWHEEL: m_mouseWheelDelta += static_cast<int>(GET_WHEEL_DELTA_WPARAM(wParam));
            break;

        default: break;
        }
    }

    //Returns whether a key is currently down.
    bool InputUtility::KeyDown(uint32_t keyCode) const
    {
        return keyCode < m_keyDowns.size() && m_keyDowns[keyCode];
    }

    //Returns whether a key was pressed this frame.
    bool InputUtility::KeyPressed(uint32_t keyCode) const
    {
        return keyCode < m_keyDowns.size() && m_keyDowns[keyCode] && !m_previousKeyDowns[keyCode];
    }

    //Returns whether a key was released this frame.
    bool InputUtility::KeyReleased(uint32_t keyCode) const
    {
        return keyCode < m_keyDowns.size() && !m_keyDowns[keyCode] && m_previousKeyDowns[keyCode];
    }

    //Returns whether a mouse button is currently down.
    bool InputUtility::MouseButtonDown(uint32_t button) const
    {
        return button < m_mouseButtonDowns.size() && m_mouseButtonDowns[button];
    }

    //Returns whether a mouse button was pressed this frame.
    bool InputUtility::MouseButtonPressed(uint32_t button) const
    {
        return button < m_mouseButtonDowns.size() && m_mouseButtonDowns[button] && !m_previousMouseButtonDowns[button];
    }

    //Returns whether a mouse button was released this frame.
    bool InputUtility::MouseButtonReleased(uint32_t button) const
    {
        return button < m_mouseButtonDowns.size() && !m_mouseButtonDowns[button] && m_previousMouseButtonDowns[button];
    }

    //Returns whether a mouse button was double-clicked this frame.
    bool InputUtility::MouseButtonDoubleClicked(uint32_t button) const
    {
        return button < m_mouseButtonDoubleClicks.size() && m_mouseButtonDoubleClicks[button];
    }

    //Sets one key state.
    void InputUtility::SetKey(uint32_t keyCode, bool isDown)
    {
        if (keyCode < m_keyDowns.size())
        {
            m_keyDowns[keyCode] = isDown;
        }
    }

    //Sets one mouse button state.
    void InputUtility::SetMouseButton(uint32_t button, bool isDown)
    {
        if (button < m_mouseButtonDowns.size())
        {
            m_mouseButtonDowns[button] = isDown;
        }
    }

    //Updates mouse position and movement.
    void InputUtility::UpdateMousePosition(LPARAM lParam)
    {
        const int newMouseX = GET_X_LPARAM(lParam);
        const int newMouseY = GET_Y_LPARAM(lParam);

        if (m_mousePositionValid)
        {
            m_mouseDeltaX += newMouseX - m_mouseX;
            m_mouseDeltaY += newMouseY - m_mouseY;
        }
        else
        {
            m_mousePositionValid = true;
        }

        m_mouseX = newMouseX;
        m_mouseY = newMouseY;
    }

    //Returns the mouse button index from an X button message.
    uint32_t InputUtility::XButtonFromWParam(WPARAM wParam) const
    {
        if (GET_XBUTTON_WPARAM(wParam) == XBUTTON1)
        {
            return s_mouseX1Button;
        }

        if (GET_XBUTTON_WPARAM(wParam) == XBUTTON2)
        {
            return s_mouseX2Button;
        }

        return static_cast<uint32_t>(s_mouseButtonCount);
    }
}

