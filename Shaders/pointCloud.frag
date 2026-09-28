///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	pointCloud.frag
//	Description	:	
//		Defines the point cloud fragment shader.
//
///////////////////////////////////////////////////////////////////////

#version 450

layout(location = 0) in vec4 fragColor;

layout(location = 0) out vec4 outColor;

void main()
{
    vec2 centeredPoint = (gl_PointCoord * 2.0) - 1.0;
    if (dot(centeredPoint, centeredPoint) > 1.0)
    {
        discard;
    }

    outColor = fragColor;
}
