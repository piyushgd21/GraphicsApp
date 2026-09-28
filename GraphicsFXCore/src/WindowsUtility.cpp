///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	WindowsUtility.cpp
//	Description	:
//		Implements shared Windows helper behavior.
//
///////////////////////////////////////////////////////////////////////

#if defined(_DEBUG)
#define _CRTDBG_MAP_ALLOC
#endif

#include "WindowsUtility.h"

#include <Windows.h>

#include <cstdio>

#if defined(_DEBUG)
#include <crtdbg.h>
#endif

namespace graphicsFX
{
#if defined(_DEBUG)
    static FILE *s_memoryLeakReportFilePtr = nullptr; //Stores the memory leak report file pointer.
    static _CrtMemState s_startMemoryState{}; //Stores the startup CRT memory state.
#endif

    //Writes an optional exception report file.
    void WindowsUtility::WriteExceptionReport(const char *messagePtr)
    {
        if (messagePtr == nullptr)
        {
            return;
        }

        char reportFileName[MAX_PATH]{};
        const DWORD reportFileNameLength = GetEnvironmentVariableA("GRAPHICSAPP_EXCEPTION_REPORT", reportFileName, static_cast<DWORD>(sizeof(reportFileName)));

        if (reportFileNameLength == 0 || reportFileNameLength >= static_cast<DWORD>(sizeof(reportFileName)))
        {
            return;
        }

        FILE *exceptionReportFilePtr = nullptr;
        if (fopen_s(&exceptionReportFilePtr, reportFileName, "w") == 0 && exceptionReportFilePtr != nullptr)
        {
            fputs(messagePtr, exceptionReportFilePtr);
            fclose(exceptionReportFilePtr);
        }
    }

    //Shows a Windows error message box.
    void WindowsUtility::ShowErrorMessage(const char *titlePtr, const char *messagePtr)
    {
        const char *safeTitlePtr = titlePtr != nullptr ? titlePtr : "GraphicsApp Error";
        const char *safeMessagePtr = messagePtr != nullptr ? messagePtr : "";
        MessageBoxA(nullptr, safeMessagePtr, safeTitlePtr, MB_ICONERROR | MB_OK);
    }

#if defined(_DEBUG)

    //Enables CRT debug allocation tracking.
    void WindowsUtility::EnableMemoryLeakCheck(void)
    {
        const int flags = _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG);
        _CrtSetDbgFlag(flags | _CRTDBG_ALLOC_MEM_DF);

        char reportFileName[MAX_PATH]{};
        const DWORD reportFileNameLength = GetEnvironmentVariableA("GRAPHICSAPP_MEMORY_REPORT", reportFileName, static_cast<DWORD>(sizeof(reportFileName)));

        if (reportFileNameLength > 0 && reportFileNameLength < static_cast<DWORD>(sizeof(reportFileName)) && fopen_s(&s_memoryLeakReportFilePtr, reportFileName, "w") == 0 &&
            s_memoryLeakReportFilePtr != nullptr)
        {
            _CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_FILE | _CRTDBG_MODE_DEBUG);
            _CrtSetReportFile(_CRT_WARN, s_memoryLeakReportFilePtr);
        }

        _CrtMemCheckpoint(&s_startMemoryState);
    }

    //Returns whether CRT memory is leak-free.
    bool WindowsUtility::CheckMemoryLeaks(void)
    {
        _CrtMemState endMemoryState{};
        _CrtMemState differenceMemoryState{};
        _CrtMemCheckpoint(&endMemoryState);

        if (_CrtMemDifference(&differenceMemoryState, &s_startMemoryState, &endMemoryState) == FALSE)
        {
            return true;
        }

        _CrtMemDumpStatistics(&differenceMemoryState);
        _CrtMemDumpAllObjectsSince(&s_startMemoryState);
        return false;
    }

    //Closes the CRT memory leak report file.
    void WindowsUtility::CloseMemoryLeakReport(void)
    {
        if (s_memoryLeakReportFilePtr != nullptr)
        {
            fclose(s_memoryLeakReportFilePtr);
            s_memoryLeakReportFilePtr = nullptr;
        }
    }

#else

    //Enables CRT debug allocation tracking.
    void WindowsUtility::EnableMemoryLeakCheck(void)
    {
    }

    //Returns whether CRT memory is leak-free.
    bool WindowsUtility::CheckMemoryLeaks(void)
    {
        return true;
    }

    //Closes the CRT memory leak report file.
    void WindowsUtility::CloseMemoryLeakReport(void)
    {
    }

#endif
}
