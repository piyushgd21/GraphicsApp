///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	xrDepthDebug.frag
//	Description	:	
//		Defines the XR environment depth debug fragment shader.
//
//
///////////////////////////////////////////////////////////////////////

#version 450

layout(set = 0, binding = 0) uniform sampler2D xrEnvironmentDepth;

layout(push_constant) uniform DebugConstants
{
    vec4 settings;
} pc;

layout(location = 0) in vec2 fragTexCoord;
layout(location = 0) out vec4 outColor;

float LinearEnvironmentDepth(float depthValue)
{
    float nearZ = max(pc.settings.z, 0.0001);
    float farZ = max(pc.settings.w, nearZ + 0.0001);

    if (pc.settings.y > 1.5)
    {
        float normalizedValue = clamp(depthValue, 0.0001, 0.9999);
        return (nearZ * farZ) / (farZ - normalizedValue * (farZ - nearZ));
    }

    if (pc.settings.y > 0.5)
    {
        return mix(nearZ, farZ, clamp(depthValue, 0.0, 1.0));
    }

    return depthValue;
}

void main()
{
    float rawDepth = texture(xrEnvironmentDepth, fragTexCoord).r;
    float environmentMeters = LinearEnvironmentDepth(rawDepth);
    float normalizedDepth = clamp(environmentMeters / max(pc.settings.x, 0.0001), 0.0, 1.0);

    if (environmentMeters <= 0.0)
    {
        outColor = vec4(1.0, 0.0, 0.0, 1.0);
        return;
    }

    outColor = vec4(vec3(normalizedDepth), 1.0);
}

