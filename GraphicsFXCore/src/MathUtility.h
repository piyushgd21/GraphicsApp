///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	MathUtility.h
//	Description	:
//		Defines the math utility types and helper functions used by the renderer.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include <algorithm>
#include <cmath>

namespace graphicsFX
{
    constexpr float Pi = 3.14159265358979323846f;

    //Stores a two component float vector.
    struct Float2
    {
        float x; //Stores the X coordinate.
        float y; //Stores the Y coordinate.
    };

    //Stores a three component float vector.
    struct Float3
    {
        float x; //Stores the X coordinate.
        float y; //Stores the Y coordinate.
        float z; //Stores the Z coordinate.
    };

    //Stores a four component float vector.
    struct Float4
    {
        float x; //Stores the X coordinate.
        float y; //Stores the Y coordinate.
        float z; //Stores the Z coordinate.
        float w; //Stores the W coordinate.
    };

    //Adds two vectors.
    inline Float3 operator+(const Float3 &a, const Float3 &b)
    {
        return { a.x + b.x, a.y + b.y, a.z + b.z };
    }

    //Subtracts two vectors.
    inline Float3 operator-(const Float3 &a, const Float3 &b)
    {
        return { a.x - b.x, a.y - b.y, a.z - b.z };
    }

    //Multiplies a vector by a scalar.
    inline Float3 operator*(const Float3 &v, float amount)
    {
        return { v.x * amount, v.y * amount, v.z * amount };
    }

    //Adds one vector into another.
    inline Float3 &operator+=(Float3 &a, const Float3 &b)
    {
        a.x += b.x;
        a.y += b.y;
        a.z += b.z;
        return a;
    }

    //Calculates the vector dot product.
    inline float dot(const Float3 &a, const Float3 &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    //Calculates the vector cross product.
    inline Float3 cross(const Float3 &a, const Float3 &b)
    {
        return {
            a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x
        };
    }

    //Returns a normalized vector.
    inline Float3 normalize(const Float3 &v)
    {
        const float length = std::sqrt(dot(v, v));
        return { v.x / length, v.y / length, v.z / length };
    }

    //Returns the length of a vector.
    inline float length(const Float3 &value)
    {
        return std::sqrt(dot(value, value));
    }

    //Returns a normalized vector or fallback.
    inline Float3 normalizeOrDefault(const Float3 &value, const Float3 &fallback)
    {
        constexpr float minimumLength = 0.0001f;
        if (length(value) <= minimumLength)
        {
            return fallback;
        }

        return normalize(value);
    }

    //Stores a 3 by 3 float matrix.
    struct Float3x3
    {
        float m[9]{}; //Stores matrix values in column-major order.
    };

    //Stores a 4 by 4 float matrix.
    struct Float4x4
    {
        float m[16]{}; //Stores matrix values in column-major order.
    };

    //Creates an identity matrix.
    inline Float4x4 identity(void)
    {
        Float4x4 result{};
        result.m[0] = 1.0f;
        result.m[5] = 1.0f;
        result.m[10] = 1.0f;
        result.m[15] = 1.0f;
        return result;
    }

    //Multiplies two matrices.
    inline Float4x4 multiply(const Float4x4 &a, const Float4x4 &b)
    {
        Float4x4 result{};

        for (int col = 0; col < 4; ++col)
        {
            for (int row = 0; row < 4; ++row)
            {
                for (int k = 0; k < 4; ++k)
                {
                    result.m[col * 4 + row] += a.m[k * 4 + row] * b.m[col * 4 + k];
                }
            }
        }

        return result;
    }

    //Transforms a four component vector by a matrix.
    inline Float4 transform(const Float4x4 &matrix, const Float4 &value)
    {
        return {
            (matrix.m[0] * value.x) + (matrix.m[4] * value.y) + (matrix.m[8] * value.z) + (matrix.m[12] * value.w),
            (matrix.m[1] * value.x) + (matrix.m[5] * value.y) + (matrix.m[9] * value.z) + (matrix.m[13] * value.w),
            (matrix.m[2] * value.x) + (matrix.m[6] * value.y) + (matrix.m[10] * value.z) + (matrix.m[14] * value.w),
            (matrix.m[3] * value.x) + (matrix.m[7] * value.y) + (matrix.m[11] * value.z) + (matrix.m[15] * value.w)
        };
    }

    //Creates a translation matrix.
    inline Float4x4 translation(float x, float y, float z)
    {
        Float4x4 result = identity();
        result.m[12] = x;
        result.m[13] = y;
        result.m[14] = z;
        return result;
    }

    //Creates an X rotation matrix.
    inline Float4x4 rotationX(float radians)
    {
        Float4x4 result = identity();
        const float c = std::cos(radians);
        const float s = std::sin(radians);

        result.m[5] = c;
        result.m[6] = s;
        result.m[9] = -s;
        result.m[10] = c;
        return result;
    }

    //Creates a Y rotation matrix.
    inline Float4x4 rotationY(float radians)
    {
        Float4x4 result = identity();
        const float c = std::cos(radians);
        const float s = std::sin(radians);

        result.m[0] = c;
        result.m[2] = -s;
        result.m[8] = s;
        result.m[10] = c;
        return result;
    }

    //Creates a Z rotation matrix.
    inline Float4x4 rotationZ(float radians)
    {
        Float4x4 result = identity();
        const float c = std::cos(radians);
        const float s = std::sin(radians);

        result.m[0] = c;
        result.m[1] = s;
        result.m[4] = -s;
        result.m[5] = c;
        return result;
    }

    //Creates a uniform scale matrix.
    inline Float4x4 scale(float amount)
    {
        Float4x4 result = identity();
        result.m[0] = amount;
        result.m[5] = amount;
        result.m[10] = amount;
        return result;
    }

    //Creates a perspective projection matrix.
    inline Float4x4 perspective(float fovRadians, float aspect, float nearPlane, float farPlane)
    {
        const float f = 1.0f / std::tan(fovRadians / 2.0f);

        Float4x4 result{};
        result.m[0] = f / aspect;
        result.m[5] = -f;
        result.m[10] = farPlane / (nearPlane - farPlane);
        result.m[11] = -1.0f;
        result.m[14] = (farPlane * nearPlane) / (nearPlane - farPlane);
        return result;
    }

    //Creates an orthographic projection matrix.
    inline Float4x4 orthographic(float left, float right, float bottom, float top, float nearPlane, float farPlane)
    {
        Float4x4 result = identity();
        result.m[0] = 2.0f / (right - left);
        result.m[5] = -2.0f / (top - bottom);
        result.m[10] = 1.0f / (nearPlane - farPlane);
        result.m[12] = -(right + left) / (right - left);
        result.m[13] = (top + bottom) / (top - bottom);
        result.m[14] = nearPlane / (nearPlane - farPlane);
        return result;
    }

    //Creates a view matrix.
    inline Float4x4 lookAt(const Float3 &eye, const Float3 &center, const Float3 &up)
    {
        const Float3 f = normalize(center - eye);
        const Float3 s = normalize(cross(f, up));
        const Float3 u = cross(s, f);

        Float4x4 result = identity();
        result.m[0] = s.x;
        result.m[4] = s.y;
        result.m[8] = s.z;

        result.m[1] = u.x;
        result.m[5] = u.y;
        result.m[9] = u.z;

        result.m[2] = -f.x;
        result.m[6] = -f.y;
        result.m[10] = -f.z;

        result.m[12] = -dot(s, eye);
        result.m[13] = -dot(u, eye);
        result.m[14] = dot(f, eye);
        return result;
    }

    //Returns orientation matrix rotation as Euler degrees.
    inline Float3 rotationDegreesFromMatrix(const Float4x4 &orientation)
    {
        constexpr float radiansToDegrees = 180.0f / Pi;
        const float yRadians = std::asin(std::clamp(-orientation.m[2], -1.0f, 1.0f));
        const float cosY = std::cos(yRadians);
        float xRadians = 0.0f;
        float zRadians = 0.0f;

        if (std::fabs(cosY) > 0.0001f)
        {
            xRadians = std::atan2(orientation.m[6], orientation.m[10]);
            zRadians = std::atan2(orientation.m[1], orientation.m[0]);
        }
        else
        {
            xRadians = std::atan2(-orientation.m[9], orientation.m[5]);
            zRadians = 0.0f;
        }

        return {
            xRadians * radiansToDegrees, yRadians * radiansToDegrees, zRadians * radiansToDegrees
        };
    }
}

