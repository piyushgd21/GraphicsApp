///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	pointCloud.vert
//	Description	:	
//		Defines the point cloud vertex shader.
//
///////////////////////////////////////////////////////////////////////

#version 450

// Set by the renderer when the color attachment performs linear-to-sRGB encoding.
layout(constant_id = 0) const bool colorAttachmentIsSrgb = false;

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

layout(location = 0) out vec4 fragColor;

vec3 srgbToLinear(vec3 color)
{
    return mix(
        pow((color + vec3(0.055)) / 1.055, vec3(2.4)),
        color / 12.92,
        lessThanEqual(color, vec3(0.04045))
    );
}

void main()
{
    vec4 worldPosition = pc.model * vec4(inPosition, 1.0);

    gl_Position = camera.projection * camera.view * worldPosition;
    gl_PointSize = max(pc.objectSettings.y, 1.0);
    // Image-derived point RGB is sRGB encoded; alpha and object tint keep their existing meaning.
    vec3 pointColor = colorAttachmentIsSrgb ? srgbToLinear(inColor.rgb) : inColor.rgb;
    fragColor = vec4(pointColor, inColor.a) * pc.color;
}
