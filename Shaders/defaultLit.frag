///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	defaultLit.frag
//	Description	:	
//		Defines the default lit fragment shader.
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

layout(set = 0, binding = 1) uniform sampler2DShadow shadowMap;
layout(set = 0, binding = 2) uniform sampler2D xrEnvironmentDepth;
layout(set = 1, binding = 0) uniform sampler2D diffuseTexture;
layout(set = 1, binding = 1) uniform sampler2D normalTexture;

layout(push_constant) uniform PushConstants
{
    mat4 model;
    vec4 color;
    vec4 objectSettings;
} pc;

layout(location = 0) in vec4 fragColor;
layout(location = 1) in vec2 fragTexCoord;
layout(location = 2) in vec3 fragWorldPosition;
layout(location = 3) in vec3 fragNormal;
layout(location = 4) in vec4 fragTangent;
layout(location = 5) in vec4 fragLightPosition;

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

float ShadowAmount(vec4 lightPosition, vec3 normal, vec3 surfaceToLightDirection)
{
    if (camera.shadowSettings.x < 0.5 || pc.objectSettings.x < 0.5)
    {
        return 1.0;
    }

    vec3 projectedPosition = lightPosition.xyz / lightPosition.w;
    vec2 shadowCoord = projectedPosition.xy * 0.5 + 0.5;
    float currentDepth = projectedPosition.z;

    if (shadowCoord.x < 0.0 ||
        shadowCoord.x > 1.0 ||
        shadowCoord.y < 0.0 ||
        shadowCoord.y > 1.0 ||
        currentDepth < 0.0 ||
        currentDepth > 1.0)
    {
        return 1.0;
    }

    float normalBias = camera.shadowSettings.z * (1.0 - max(dot(normal, surfaceToLightDirection), 0.0));
    float depthBias = camera.shadowSettings.y + normalBias;
    vec2 texelSize = 1.0 / vec2(textureSize(shadowMap, 0));
    float softness = max(camera.shadowSettings.w, 0.0);
    float shadow = 0.0;

    for (int y = -1; y <= 1; ++y)
    {
        for (int x = -1; x <= 1; ++x)
        {
            vec2 offset = vec2(float(x), float(y)) * texelSize * softness;
            shadow += texture(shadowMap, vec3(shadowCoord + offset, currentDepth - depthBias));
        }
    }

    return shadow / 9.0;
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

    vec4 textureColor = texture(diffuseTexture, fragTexCoord);
    vec4 baseColor = textureColor * fragColor;

    vec3 normal = normalize(fragNormal);
    vec3 tangent = normalize(fragTangent.xyz - normal * dot(normal, fragTangent.xyz));
    vec3 bitangent = normalize(cross(normal, tangent) * fragTangent.w);
    mat3 tbn = mat3(tangent, bitangent, normal);

    vec3 normalSample = texture(normalTexture, fragTexCoord).xyz * 2.0 - 1.0;
    vec3 mappedNormal = normalize(tbn * normalSample);

    vec3 surfaceToLightDirection = normalize(-camera.lightDirection.xyz);
    vec3 viewDirection = normalize(camera.cameraPosition.xyz - fragWorldPosition);
    float diffuseAmount = max(dot(mappedNormal, surfaceToLightDirection), 0.0);
    float shadowAmount = ShadowAmount(fragLightPosition, mappedNormal, surfaceToLightDirection);

    vec3 reflectDirection = reflect(-surfaceToLightDirection, mappedNormal);
    float specularAmount = 0.0;
    if (diffuseAmount > 0.0)
    {
        specularAmount = pow(max(dot(viewDirection, reflectDirection), 0.0), camera.lightSettings.x);
    }

    vec3 litColor =
        baseColor.rgb * camera.ambientColor.rgb +
        shadowAmount * (
            baseColor.rgb * camera.diffuseColor.rgb * diffuseAmount +
            camera.specularColor.rgb * specularAmount);

    outColor = vec4(litColor, baseColor.a);
}

