///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	normalCapture.frag
//	Description	:	
//		Defines the normal capture fragment shader.
//
//
///////////////////////////////////////////////////////////////////////

#version 450

layout(set = 1, binding = 1) uniform sampler2D normalTexture;

layout(location = 0) in vec2 fragTexCoord;
layout(location = 1) in vec3 fragNormal;
layout(location = 2) in vec4 fragTangent;

layout(location = 0) out vec4 outColor;

void main()
{
    vec3 normal = normalize(fragNormal);
    vec3 tangent = normalize(fragTangent.xyz - normal * dot(normal, fragTangent.xyz));
    vec3 bitangent = normalize(cross(normal, tangent) * fragTangent.w);
    mat3 tbn = mat3(tangent, bitangent, normal);

    vec3 normalSample = texture(normalTexture, fragTexCoord).xyz * 2.0 - 1.0;
    vec3 mappedNormal = normalize(tbn * normalSample);
    vec3 encodedNormal = mappedNormal * 0.5 + 0.5;

    outColor = vec4(encodedNormal, 1.0);
}

