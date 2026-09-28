///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	gaussianSplat.vert
//	Description	:
//		Defines the Gaussian splat vertex shader.
//
///////////////////////////////////////////////////////////////////////

#version 450

layout(set = 0, binding = 0) uniform CameraBuffer
{
    mat4 view;
    mat4 projection;
    vec4 cameraPosition;
    vec4 lightDirection;
    vec4 ambientColor;
    vec4 diffuseColor;
    vec4 specularColor;
    vec4 lightSettings;
    mat4 lightViewProjection;
    vec4 shadowSettings;
    mat4 xrEnvironmentDepthView;
    mat4 xrEnvironmentDepthViewProjection;
    vec4 xrDepthSettings;
    vec4 xrDepthExtent;
    vec4 xrDepthModeSettings;
} camera;

layout(push_constant) uniform PushConstants
{
    mat4 model;
    vec4 color;
    vec4 objectSettings;
} pc;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;
layout(location = 3) in vec3 inScale;
layout(location = 4) in vec4 inRotation;

layout(location = 0) out vec4 fragColor;
layout(location = 1) flat out vec4 fragConic;

mat3 QuaternionToMatrix(vec4 quaternion)
{
    vec4 q = normalize(quaternion);
    float x = q.x;
    float y = q.y;
    float z = q.z;
    float w = q.w;

    return mat3(
        1.0 - (2.0 * y * y) - (2.0 * z * z), (2.0 * x * y) + (2.0 * w * z), (2.0 * x * z) - (2.0 * w * y),
        (2.0 * x * y) - (2.0 * w * z), 1.0 - (2.0 * x * x) - (2.0 * z * z), (2.0 * y * z) + (2.0 * w * x),
        (2.0 * x * z) + (2.0 * w * y), (2.0 * y * z) - (2.0 * w * x), 1.0 - (2.0 * x * x) - (2.0 * y * y)
    );
}

mat2 ProjectCovarianceToScreen(vec3 viewPosition, mat3 viewCovariance, float screenWidth, float screenHeight)
{
    float zDistance = max(-viewPosition.z, 0.0001);
    float zDistanceSquared = zDistance * zDistance;
    float xScale = camera.projection[0][0] * screenWidth * 0.5;
    float yScale = camera.projection[1][1] * screenHeight * 0.5;

    float j00 = xScale / zDistance;
    float j02 = xScale * viewPosition.x / zDistanceSquared;
    float j11 = yScale / zDistance;
    float j12 = yScale * viewPosition.y / zDistanceSquared;

    float c00 = viewCovariance[0][0];
    float c01 = viewCovariance[0][1];
    float c02 = viewCovariance[0][2];
    float c11 = viewCovariance[1][1];
    float c12 = viewCovariance[1][2];
    float c22 = viewCovariance[2][2];

    mat2 result;
    result[0][0] = (j00 * j00 * c00) + (2.0 * j00 * j02 * c02) + (j02 * j02 * c22);
    result[0][1] = (j00 * j11 * c01) + (j00 * j12 * c02) + (j02 * j11 * c12) + (j02 * j12 * c22);
    result[1][0] = result[0][1];
    result[1][1] = (j11 * j11 * c11) + (2.0 * j11 * j12 * c12) + (j12 * j12 * c22);

    result[0][0] += 0.3;
    result[1][1] += 0.3;
    return result;
}

void main()
{
    vec4 worldPosition = pc.model * vec4(inPosition, 1.0);
    vec4 viewPosition = camera.view * worldPosition;

    if (viewPosition.z > -0.05)
    {
        gl_Position = camera.projection * viewPosition;
        gl_PointSize = 1.0;
        fragColor = vec4(0.0);
        fragConic = vec4(1.0, 0.0, 1.0, 1.0);
        return;
    }

    vec3 safeScale = max(abs(inScale), vec3(0.000001));
    mat3 localRotation = QuaternionToMatrix(inRotation);
    mat3 localCovariance = localRotation * mat3(
        safeScale.x * safeScale.x, 0.0, 0.0,
        0.0, safeScale.y * safeScale.y, 0.0,
        0.0, 0.0, safeScale.z * safeScale.z
    ) * transpose(localRotation);

    mat3 modelViewTransform = mat3(camera.view) * mat3(pc.model);
    mat3 viewCovariance = modelViewTransform * localCovariance * transpose(modelViewTransform);
    mat2 screenCovariance = ProjectCovarianceToScreen(viewPosition.xyz, viewCovariance, max(pc.objectSettings.z, 1.0), max(pc.objectSettings.w, 1.0));

    float a = screenCovariance[0][0];
    float b = screenCovariance[0][1];
    float c = screenCovariance[1][1];
    float middle = 0.5 * (a + c);
    float radius = sqrt(max(0.0, (0.25 * (a - c) * (a - c)) + (b * b)));
    float maximumEigenvalue = max(middle + radius, 0.25);
    float unclampedPointSize = max(pc.objectSettings.y, 8.0 * sqrt(maximumEigenvalue));
    float maximumPointSize = max(pc.objectSettings.x, pc.objectSettings.y);
    float pointSize = clamp(unclampedPointSize, 1.0, maximumPointSize);
    float clampScale = min(pointSize / max(unclampedPointSize, 0.000001), 1.0);
    screenCovariance *= clampScale * clampScale;

    a = screenCovariance[0][0];
    b = screenCovariance[0][1];
    c = screenCovariance[1][1];

    float determinant = max((a * c) - (b * b), 0.000001);
    mat2 inverseCovariance = mat2(c, -b, -b, a) / determinant;

    gl_Position = camera.projection * viewPosition;
    gl_PointSize = pointSize;
    fragColor = inColor * pc.color;
    fragConic = vec4(inverseCovariance[0][0], inverseCovariance[0][1], inverseCovariance[1][1], pointSize);
}
