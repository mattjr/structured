#version 120
#extension GL_EXT_gpu_shader4 : enable

uniform sampler2DArray theTexture;
//uniform sampler2D theTexture;

void main()
{

	vec3 coord = gl_TexCoord[1].xyz;
//	coord.z=2;	
	
       	vec4 color=  texture2DArray(theTexture,coord);
        gl_FragColor = color;
} 
 
