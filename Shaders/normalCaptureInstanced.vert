///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	normalCaptureInstanced.vert
//	Description	:	
//		Defines the normal capture instanced vertex shader.
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
layout(location = 5) in mat4 inInstanceModel;
layout(location = 9) in vec4 inInstanceColor;

layout(location = 0) out vec2 fragTexCoord;
layout(location = 1) out vec3 fragNormal;
layout(location = 2) out vec4 fragTangent;

void main()
{
    mat4 model = pc.model * inInstanceModel;
    vec4 worldPosition = model * vec4(inPosition, 1.0);
    mat3 tangentMatrix = mat3(model);
    mat3 normalMatrix = transpose(inverse(tangentMatrix));

    gl_Position = camera.projection * camera.view * worldPosition;
    fragTexCoord = inTexCoord;
    fragNormal = normalize(normalMatrix * inNormal);
    fragTangent = vec4(normalize(tangentMatrix * inTangent.xyz), inTangent.w);
}

