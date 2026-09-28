///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	default.vert
//	Description	:	
//		Defines the default vertex shader.
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

layout(push_constant) uniform PushConstants
{
    mat4 model;
    vec4 color;
    vec4 objectSettings;
} pc;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;
layout(location = 2) in vec2 inTexCoord;

layout(location = 0) out vec4 fragColor;
layout(location = 1) out vec2 fragTexCoord;
layout(location = 2) out vec3 fragWorldPosition;

void main()
{
    vec4 worldPosition = pc.model * vec4(inPosition, 1.0);

    gl_Position = camera.projection * camera.view * worldPosition;
    fragColor = inColor * pc.color;
    fragTexCoord = inTexCoord;
    fragWorldPosition = worldPosition.xyz;
}

