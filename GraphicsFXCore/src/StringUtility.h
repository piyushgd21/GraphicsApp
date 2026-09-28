///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	StringUtility.h
//	Description	:
//		Defines shared string formatting and conversion helpers.
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "global.h"

#include "MathUtility.h"

#include <string>

namespace graphicsFX
{
    //Defines shared string formatting and conversion helpers.
    class StringUtility final
    {

        public:

            StringUtility(void) = delete;

            //Returns a trimmed string.
            static std::string Trim(const std::string &text);

            //Returns an uppercase copy of a string.
            static std::string Uppercase(std::string text);

            //Converts a UTF-8 string to a Windows wide string.
            static std::wstring ToWide(const std::string &text, const std::wstring &fallback);

            //Converts a UTF-8 string to a Windows wide string or throws.
            static std::wstring ToWideOrThrow(const std::string &text, const char *messagePtr);

            //Returns a formatted float string.
            static std::string FormatFloat(float value, int precision = 2);

            //Returns a formatted three component float string.
            static std::string FormatFloat3(const Float3 &value, int precision = 2);

    };
}
