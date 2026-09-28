///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	gaussianSplat.frag
//	Description	:
//		Defines the Gaussian splat fragment shader.
//
///////////////////////////////////////////////////////////////////////

#version 450

layout(location = 0) in vec4 fragColor;
layout(location = 1) flat in vec4 fragConic;

layout(location = 0) out vec4 outColor;

void main()
{
    vec2 deltaPixel = ((gl_PointCoord * 2.0) - 1.0) * (fragConic.w * 0.5);
    float exponent = -0.5 * ((fragConic.x * deltaPixel.x * deltaPixel.x) + (2.0 * fragConic.y * deltaPixel.x * deltaPixel.y) + (fragConic.z * deltaPixel.y * deltaPixel.y));

    if (exponent < -12.5)
    {
        discard;
    }

    float alpha = fragColor.a * exp(exponent);
    if (alpha <= 0.003)
    {
        discard;
    }

    outColor = vec4(fragColor.rgb, alpha);
}
