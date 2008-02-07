#extension GL_ARB_texture_rectangle : enable
#extension GL_EXT_gpu_shader4 : enable
uniform sampler2DRect planes;
varying vec3 vpos;

flat varying ivec2 planeCoord;



void main()
{

  gl_TexCoord[0] = gl_MultiTexCoord0;

  planeCoord = ivec2(gl_MultiTexCoord1.xy);
  gl_Position = ftransform();
  vpos=gl_Vertex.xyz;
}
