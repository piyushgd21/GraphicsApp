///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	default.frag
//	Description	:	
//		Defines the default fragment shader.
//
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

layout(set = 0, binding = 2) uniform sampler2D xrEnvironmentDepth;
layout(set = 1, binding = 0) uniform sampler2D objectTexture;

layout(location = 0) in vec4 fragColor;
layout(location = 1) in vec2 fragTexCoord;
layout(location = 2) in vec3 fragWorldPosition;
layout(location = 0) out vec4 outColor;

float LinearEnvironmentDepth(float depthValue)
{
    float nearZ = max(camera.xrDepthExtent.z, 0.0001);
    float farZ = max(camera.xrDepthExtent.w, nearZ + 0.0001);

    if (camera.xrDepthModeSettings.x > 1.5)
    {
        float normalizedValue = clamp(depthValue, 0.0001, 0.9999);
        return (nearZ * farZ) / (farZ - normalizedValue * (farZ - nearZ));
    }

    if (camera.xrDepthModeSettings.x > 0.5)
    {
        return mix(nearZ, farZ, clamp(depthValue, 0.0, 1.0));
    }

    return depthValue;
}

float SampleEnvironmentDepth(vec3 worldPosition, out float sceneMeters, out bool valid)
{
    sceneMeters = 0.0;
    valid = false;

    if (camera.xrDepthSettings.x < 0.5)
    {
        return 0.0;
    }

    vec4 depthViewPosition = camera.xrEnvironmentDepthView * vec4(worldPosition, 1.0);
    sceneMeters = max(-depthViewPosition.z, 0.0);
    if (sceneMeters <= 0.0 || sceneMeters > camera.xrDepthSettings.z)
    {
        return 0.0;
    }

    vec4 depthClipPosition = camera.xrEnvironmentDepthViewProjection * vec4(worldPosition, 1.0);
    if (depthClipPosition.w <= 0.0)
    {
        return 0.0;
    }

    vec3 depthNdc = depthClipPosition.xyz / depthClipPosition.w;
    vec2 depthUv = depthNdc.xy * 0.5 + 0.5;
    if (depthUv.x < 0.0 || depthUv.x > 1.0 || depthUv.y < 0.0 || depthUv.y > 1.0)
    {
        return 0.0;
    }

    float environmentMeters = LinearEnvironmentDepth(texture(xrEnvironmentDepth, depthUv).r);
    if (environmentMeters <= 0.0 || environmentMeters > camera.xrDepthSettings.z)
    {
        return 0.0;
    }

    valid = true;
    return environmentMeters;
}

bool PassthroughOccludes(vec3 worldPosition)
{
    float sceneMeters = 0.0;
    bool valid = false;
    float environmentMeters = SampleEnvironmentDepth(worldPosition, sceneMeters, valid);
    if (!valid)
    {
        return false;
    }

    return environmentMeters + camera.xrDepthSettings.y < sceneMeters;
}

vec4 XRDepthDebugColor(vec3 worldPosition)
{
    float sceneMeters = 0.0;
    bool valid = false;
    float environmentMeters = SampleEnvironmentDepth(worldPosition, sceneMeters, valid);
    if (!valid)
    {
        return vec4(1.0, 0.0, 0.0, 1.0);
    }

    if (camera.xrDepthSettings.w > 1.5)
    {
        bool occluded = environmentMeters + camera.xrDepthSettings.y < sceneMeters;
        return occluded ? vec4(1.0, 0.15, 0.05, 1.0) : vec4(0.05, 0.85, 0.25, 1.0);
    }

    float normalizedDepth = clamp(environmentMeters / camera.xrDepthSettings.z, 0.0, 1.0);
    return vec4(vec3(normalizedDepth), 1.0);
}

void main()
{
    if (camera.xrDepthModeSettings.y > 0.5 && camera.xrDepthModeSettings.y < 1.5)
    {
        discard;
    }

    if (camera.xrDepthSettings.w > 0.5)
    {
        outColor = XRDepthDebugColor(fragWorldPosition);
        return;
    }

    if (camera.xrDepthModeSettings.y < 1.5 && PassthroughOccludes(fragWorldPosition))
    {
        discard;
    }

    outColor = texture(objectTexture, fragTexCoord) * fragColor;
}

