///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	InputUtility.h
//	Description	:
//		Defines keyboard and mouse input state handling.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "global.h"

#include <Windows.h>

#include <array>
#include <cstddef>
#include <cstdint>

namespace graphicsFX
{
    //Defines keyboard and mouse input state handling.
    class InputUtility
    {

        public:

            //Creates the input utility.
            InputUtility(void) = default;

            //Destroys the input utility.
            ~InputUtility(void) = default;

            InputUtility(const InputUtility &) = delete;
            InputUtility &operator=(const InputUtility &) = delete;

            //Starts a new input frame.
            void BeginFrame(void);

            //Handles one Win32 input message.
            void HandleMessage(UINT message, WPARAM wParam, LPARAM lParam);

            //Returns whether a key is currently down.
            bool KeyDown(uint32_t keyCode) const;

            //Returns whether a key was pressed this frame.
            bool KeyPressed(uint32_t keyCode) const;

            //Returns whether a key was released this frame.
            bool KeyReleased(uint32_t keyCode) const;

            //Returns whether a mouse button is currently down.
            bool MouseButtonDown(uint32_t button) const;

            //Returns whether a mouse button was pressed this frame.
            bool MouseButtonPressed(uint32_t button) const;

            //Returns whether a mouse button was released this frame.
            bool MouseButtonReleased(uint32_t button) const;

            //Returns whether a mouse button was double-clicked this frame.
            bool MouseButtonDoubleClicked(uint32_t button) const;

            //Returns the mouse X position.
            int MouseX(void) const { return m_mouseX; }

            //Returns the mouse Y position.
            int MouseY(void) const { return m_mouseY; }

            //Returns the mouse X movement this frame.
            int MouseDeltaX(void) const { return m_mouseDeltaX; }

            //Returns the mouse Y movement this frame.
            int MouseDeltaY(void) const { return m_mouseDeltaY; }

            //Returns the mouse wheel movement this frame.
            int MouseWheelDelta(void) const { return m_mouseWheelDelta; }

        private:

            //Sets one key state.
            void SetKey(uint32_t keyCode, bool isDown);

            //Sets one mouse button state.
            void SetMouseButton(uint32_t button, bool isDown);

            //Updates mouse position and movement.
            void UpdateMousePosition(LPARAM lParam);

            //Returns the mouse button index from an X button message.
            uint32_t XButtonFromWParam(WPARAM wParam) const;

            static constexpr std::size_t s_keyCount = 256; //Stores the number of tracked keys.
            static constexpr std::size_t s_mouseButtonCount = 5; //Stores the number of tracked mouse buttons.
            static constexpr uint32_t s_mouseLeftButton = 0U; //Stores the left mouse button index.
            static constexpr uint32_t s_mouseRightButton = 1U; //Stores the right mouse button index.
            static constexpr uint32_t s_mouseMiddleButton = 2U; //Stores the middle mouse button index.
            static constexpr uint32_t s_mouseX1Button = 3U; //Stores the X1 mouse button index.
            static constexpr uint32_t s_mouseX2Button = 4U; //Stores the X2 mouse button index.

            std::array<bool, s_keyCount> m_keyDowns{}; //Stores current key states.
            std::array<bool, s_keyCount> m_previousKeyDowns{}; //Stores previous key states.
            std::array<bool, s_mouseButtonCount> m_mouseButtonDowns{}; //Stores current mouse button states.
            std::array<bool, s_mouseButtonCount> m_previousMouseButtonDowns{}; //Stores previous mouse button states.
            std::array<bool, s_mouseButtonCount> m_mouseButtonDoubleClicks{}; //Stores mouse double-click states.
            int m_mouseX{ 0 }; //Stores the mouse X position.
            int m_mouseY{ 0 }; //Stores the mouse Y position.
            int m_mouseDeltaX{ 0 }; //Stores mouse X movement this frame.
            int m_mouseDeltaY{ 0 }; //Stores mouse Y movement this frame.
            int m_mouseWheelDelta{ 0 }; //Stores mouse wheel movement this frame.
            bool m_mousePositionValid{ false }; //Stores whether mouse position is initialized.
    };
}

