///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	StringUtility.cpp
//	Description	:
//		Implements shared string formatting and conversion helpers.
//
///////////////////////////////////////////////////////////////////////

#include "StringUtility.h"

#include <Windows.h>

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace graphicsFX
{
    //Returns a trimmed string.
    std::string StringUtility::Trim(const std::string &text)
    {
        std::size_t first = 0U;
        while (first < text.size() && std::isspace(static_cast<unsigned char>(text[first])) != 0)
        {
            ++first;
        }

        std::size_t last = text.size();
        while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1U])) != 0)
        {
            --last;
        }

        return text.substr(first, last - first);
    }

    //Returns an uppercase copy of a string.
    std::string StringUtility::Uppercase(std::string text)
    {
        std::transform(text.begin(), text.end(), text.begin(), [](unsigned char character) { return static_cast<char>(std::toupper(character)); });
        return text;
    }

    //Converts a UTF-8 string to a Windows wide string.
    std::wstring StringUtility::ToWide(const std::string &text, const std::wstring &fallback)
    {
        if (text.empty())
        {
            return {};
        }

        if (text.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
        {
            return fallback;
        }

        const int textLength = static_cast<int>(text.size());
        int requiredLength = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.c_str(), textLength, nullptr, 0);
        UINT codePage = CP_UTF8;
        DWORD flags = MB_ERR_INVALID_CHARS;

        if (requiredLength == 0)
        {
            codePage = CP_ACP;
            flags = 0;
            requiredLength = MultiByteToWideChar(codePage, flags, text.c_str(), textLength, nullptr, 0);
        }

        if (requiredLength == 0)
        {
            return fallback;
        }

        std::wstring result(static_cast<std::size_t>(requiredLength), L'\0');
        const int convertedLength = MultiByteToWideChar(codePage, flags, text.c_str(), textLength, result.data(), requiredLength);

        if (convertedLength == 0)
        {
            return fallback;
        }

        return result;
    }

    //Converts a UTF-8 string to a Windows wide string or throws.
    std::wstring StringUtility::ToWideOrThrow(const std::string &text, const char *messagePtr)
    {
        if (text.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
        {
            throw std::runtime_error(messagePtr);
        }

        const std::wstring result = ToWide(text, {});
        if (!text.empty() && result.empty())
        {
            throw std::runtime_error(messagePtr);
        }

        return result;
    }

    //Returns a formatted float string.
    std::string StringUtility::FormatFloat(float value, int precision)
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return stream.str();
    }

    //Returns a formatted three component float string.
    std::string StringUtility::FormatFloat3(const Float3 &value, int precision)
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision);
        stream << "(" << value.x << ", " << value.y << ", " << value.z << ")";
        return stream.str();
    }
}
