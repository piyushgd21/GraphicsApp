///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	WindowsUtility.h
//	Description	:
//		Defines shared Windows helper behavior.
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "global.h"

namespace graphicsFX
{
    //Defines shared Windows helper behavior.
    class WindowsUtility final
    {

        public:

            WindowsUtility(void) = delete;

            //Writes an optional exception report file.
            static void WriteExceptionReport(const char *messagePtr);

            //Shows a Windows error message box.
            static void ShowErrorMessage(const char *titlePtr, const char *messagePtr);

            //Enables CRT debug allocation tracking.
            static void EnableMemoryLeakCheck(void);

            //Returns whether CRT memory is leak-free.
            static bool CheckMemoryLeaks(void);

            //Closes the CRT memory leak report file.
            static void CloseMemoryLeakReport(void);

    };
}
