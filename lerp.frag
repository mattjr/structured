#version 120
#extension GL_EXT_gpu_shader4 : enable

uniform sampler2DArray theTexture;
//uniform sampler2D theTexture;

void main()
{
        vec3 coord = vec3(0.0);
//      float mult;
//      mult = 65500.0/maxVal;
	coord.xy = gl_TexCoord[0].xy;
	coord.z=0;	
       // vec3 color = vec3(texture2DArray(theTexture,vec3(0.4,0.,0)));
       // coord.xy = gl_TexCoord[0].xy;
       	vec4 color=  texture2DArray(theTexture,gl_TexCoord[0].xyz);
        gl_FragColor = color;//*0.5;
//vec4(color, 1.0);


} 
