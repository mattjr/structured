
varying vec3 vpos;
varying vec4 planeData;



void main()
{

  gl_TexCoord[0] = gl_MultiTexCoord0;

  planeData = gl_MultiTexCoord1;
  gl_Position = ftransform();
  vpos=gl_Vertex.xyz;
}
