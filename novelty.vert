#extension GL_ARB_texture_rectangle : enable

uniform sampler2DRect planes;
varying vec3 vpos;
varying vec2 planeCoord;



void main()
{

  gl_TexCoord[0] = gl_MultiTexCoord0;

  planeCoord = gl_MultiTexCoord1.xy;
  gl_Position = ftransform();
  vpos=gl_Vertex.xyz;
}
