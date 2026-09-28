///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	WinMain.cpp
//	Description	:
//		Defines the Windows entry point for the graphics application.
//
//
///////////////////////////////////////////////////////////////////////

#include "EngineCore.h"
#include "Scene.h"
#include "WindowsUtility.h"

#include <Windows.h>

#include <cstdlib>
#include <exception>

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int)
{
    graphicsFX::WindowsUtility::EnableMemoryLeakCheck();

    int exitCode = EXIT_SUCCESS;

    try
    {
        {
            Scene scene;
            graphicsFX::EngineCore app(instance, scene, "GraphicsApp00/setup/setup.ini");
            app.Run();
        }
    }
    catch (const std::exception &ex)
    {
        graphicsFX::WindowsUtility::WriteExceptionReport(ex.what());
        graphicsFX::WindowsUtility::ShowErrorMessage("GraphicsApp Error", ex.what());
        exitCode = EXIT_FAILURE;
    }

    if (!graphicsFX::WindowsUtility::CheckMemoryLeaks())
    {
        exitCode = EXIT_FAILURE;
    }

    graphicsFX::WindowsUtility::CloseMemoryLeakReport();

    return exitCode;
}

