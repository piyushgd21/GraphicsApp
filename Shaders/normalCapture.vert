///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	normalCapture.vert
//	Description	:	
//		Defines the normal capture vertex shader.
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
layout(location = 3) in vec3 inNormal;
layout(location = 4) in vec4 inTangent;

layout(location = 0) out vec2 fragTexCoord;
layout(location = 1) out vec3 fragNormal;
layout(location = 2) out vec4 fragTangent;

void main()
{
    vec4 worldPosition = pc.model * vec4(inPosition, 1.0);
    mat3 tangentMatrix = mat3(pc.model);
    mat3 normalMatrix = transpose(inverse(tangentMatrix));

    gl_Position = camera.projection * camera.view * worldPosition;
    fragTexCoord = inTexCoord;
    fragNormal = normalize(normalMatrix * inNormal);
    fragTangent = vec4(normalize(tangentMatrix * inTangent.xyz), inTangent.w);
}

