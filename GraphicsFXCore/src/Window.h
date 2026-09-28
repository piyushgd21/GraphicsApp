///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	Window.h
//	Description	:
//		Defines the Win32 window wrapper.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "global.h"

#include <Windows.h>

#include <cstdint>
#include <string>

namespace graphicsFX
{
    class InputUtility;

    //Defines the Win32 window wrapper.
    class Window
    {

        public:

            //Creates the Win32 application window.
            Window(HINSTANCE instance, int width, int height, const std::wstring &title);

            //Destroys the Win32 application window.
            ~Window();

            Window(const Window &) = delete;
            Window &operator=(const Window &) = delete;

            //Processes pending window messages.
            void PollEvents(void);

            //Waits for a window message.
            void WaitForEvents(void);

            //Sets the input utility that receives Win32 messages.
            void SetInputUtility(InputUtility *inputUtilityPtr) { m_inputUtilityPtr = inputUtilityPtr; }

            //Returns whether the window should close.
            bool ShouldClose(void) const { return m_shouldClose; }

            //Returns whether the framebuffer changed size.
            bool WasResized(void) const { return m_framebufferResized; }

            //Clears the resize flag.
            void ResetResizedFlag(void) { m_framebufferResized = false; }

            //Returns the Win32 window handle.
            HWND Hwnd(void) const { return m_hwndPtr; }

            //Returns the Win32 application instance.
            HINSTANCE Instance(void) const { return m_instancePtr; }

            //Returns the current window width.
            uint32_t Width(void) const { return static_cast<uint32_t>(m_width); }

            //Returns the current window height.
            uint32_t Height(void) const { return static_cast<uint32_t>(m_height); }

        private:

            //Routes Win32 messages to the window object.
            static LRESULT CALLBACK WindowProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);

            //Handles a Win32 message for this window.
            LRESULT HandleMessage(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);

            HINSTANCE m_instancePtr{}; //Stores the Win32 application instance.
            HWND m_hwndPtr{}; //Stores the Win32 window handle.
            InputUtility *m_inputUtilityPtr{ nullptr }; //Stores the input utility pointer.
            std::wstring m_className; //Stores the registered window class name.
            int m_width{}; //Stores the current window width.
            int m_height{}; //Stores the current window height.
            bool m_shouldClose{ false }; //Stores whether the window should close.
            bool m_framebufferResized{ false }; //Stores whether the framebuffer resized.
    };
}

