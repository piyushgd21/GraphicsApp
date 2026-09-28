///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	ui.vert
//	Description	:	
//		Defines the UI vertex shader.
//
///////////////////////////////////////////////////////////////////////

#version 450

layout(push_constant) uniform UIShaderConstants
{
    vec2 screenSize;
} ui;

layout(location = 0) in vec2 inPosition;
layout(location = 1) in vec4 inColor;

layout(location = 0) out vec4 fragColor;

void main()
{
    vec2 normalizedPosition = vec2(
        (inPosition.x / ui.screenSize.x) * 2.0 - 1.0,
        (inPosition.y / ui.screenSize.y) * 2.0 - 1.0);

    gl_Position = vec4(normalizedPosition, 0.0, 1.0);
    fragColor = inColor;
}
