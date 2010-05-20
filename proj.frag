#version 120
varying vec4 projCord;
uniform mat4 teMat;

uniform sampler2D colorMap;
void main()
{
  vec2 or1= gl_TexCoord[0].st;
  vec2 new1=projCord.st;
  //vec4 shadowCoordinateWdivide = projCord / projCord.w ;
  //shadowCoordinateWdivide.z += 0.0015;
 vec4 color = texture2D(colorMap,projCord.st);
 vec4 orig =texture2D( colorMap, gl_TexCoord[0].st)+vec4(0,0,0.4,1);

 gl_FragColor=color;
}
