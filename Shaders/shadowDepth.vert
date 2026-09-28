///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	shadowDepth.vert
//	Description	:	
//		Defines the non-instanced shadow depth vertex shader.
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
} camera;

layout(push_constant) uniform PushConstants
{
    mat4 model;
    vec4 color;
    vec4 objectSettings;
} pc;

layout(location = 0) in vec3 inPosition;

void main()
{
    gl_Position = camera.lightViewProjection * pc.model * vec4(inPosition, 1.0);
}

