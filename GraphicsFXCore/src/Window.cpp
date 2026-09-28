///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	Window.cpp
//	Description	:
//		Implements the Win32 window wrapper.
//
//
///////////////////////////////////////////////////////////////////////

#include "Window.h"

#include "InputUtility.h"

#include <stdexcept>

namespace graphicsFX
{
    //Creates the Win32 application window.
    Window::Window(HINSTANCE instance, int width, int height, const std::wstring &title) : m_instancePtr(instance), m_className(L"GraphicsAppWindowClass"), m_width(width), m_height(height)
    {
        WNDCLASSEXW wc{};
        wc.cbSize = sizeof(WNDCLASSEXW);
        wc.style = CS_HREDRAW | CS_VREDRAW | CS_DBLCLKS;
        wc.lpfnWndProc = Window::WindowProc;
        wc.hInstance = m_instancePtr;
        wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
        wc.lpszClassName = m_className.c_str();

        if (!RegisterClassExW(&wc))
        {
            throw std::runtime_error("Failed to register Win32 window class.");
        }

        RECT rect{ 0, 0, m_width, m_height };
        AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW, FALSE);

        m_hwndPtr = CreateWindowExW(
            0,
            m_className.c_str(),
            title.c_str(),
            WS_OVERLAPPEDWINDOW,
            CW_USEDEFAULT,
            CW_USEDEFAULT,
            rect.right - rect.left,
            rect.bottom - rect.top,
            nullptr,
            nullptr,
            m_instancePtr,
            this
        );

        if (!m_hwndPtr)
        {
            UnregisterClassW(m_className.c_str(), m_instancePtr);
            throw std::runtime_error("Failed to create Win32 window.");
        }

        ShowWindow(m_hwndPtr, SW_SHOW);
    }

    //Destroys the Win32 application window.
    Window::~Window()
    {
        if (m_hwndPtr)
        {
            DestroyWindow(m_hwndPtr);
            m_hwndPtr = nullptr;
        }

        if (!m_className.empty())
        {
            UnregisterClassW(m_className.c_str(), m_instancePtr);
        }
    }

    //Processes pending window messages.
    void Window::PollEvents(void)
    {
        MSG message{};
        while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE))
        {
            if (message.message == WM_QUIT)
            {
                m_shouldClose = true;
            }

            TranslateMessage(&message);
            DispatchMessageW(&message);
        }
    }

    //Waits for a window message.
    void Window::WaitForEvents(void)
    {
        WaitMessage();
        PollEvents();
    }

    //Routes Win32 messages to the window object.
    LRESULT CALLBACK Window::WindowProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
    {
        Window *window = nullptr;

        if (message == WM_NCCREATE)
        {
            const auto *createStruct = reinterpret_cast<CREATESTRUCTW *>(lParam);
            window = static_cast<Window *>(createStruct->lpCreateParams);
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(window));
        }
        else
        {
            window = reinterpret_cast<Window *>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        }

        if (window)
        {
            return window->HandleMessage(hwnd, message, wParam, lParam);
        }

        return DefWindowProcW(hwnd, message, wParam, lParam);
    }

    //Handles a Win32 message for this window.
    LRESULT Window::HandleMessage(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
    {
        if (m_inputUtilityPtr != nullptr)
        {
            m_inputUtilityPtr->HandleMessage(message, wParam, lParam);
        }

        switch (message)
        {
        case WM_KEYDOWN:
        case WM_SYSKEYDOWN: if (wParam == VK_ESCAPE)
            {
                m_shouldClose = true;
                DestroyWindow(hwnd);
                return 0;
            }
            break;

        case WM_CLOSE: m_shouldClose = true;
            DestroyWindow(hwnd);
            return 0;

        case WM_DESTROY: m_shouldClose = true;
            PostQuitMessage(0);
            return 0;

        case WM_SIZE: m_width = LOWORD(lParam);
            m_height = HIWORD(lParam);
            m_framebufferResized = true;
            return 0;

        case WM_NCDESTROY: m_hwndPtr = nullptr;
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, 0);
            break;
        }

        return DefWindowProcW(hwnd, message, wParam, lParam);
    }
}

