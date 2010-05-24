#version 120
varying vec4 projCord;
uniform mat4 teMat;
vec4 jet_colormap(float val_un)
{
  float val=clamp(val_un,0.0,1.0);
  vec4 rgba;
  rgba.r = min(4.0 * val - 1.5,-4.0 * val + 4.5) ;
  rgba.g = min(4.0 * val - 0.5,-4.0 * val + 3.5) ;
  rgba.b = min(4.0 * val + 0.5,-4.0 * val + 2.5) ;
  rgba.a=0.0;
return rgba;
}

uniform sampler2D colorMap;
void main()
{
  vec2 or1= gl_TexCoord[0].st;
  vec2 new1=projCord.st;
  //vec4 shadowCoordinateWdivide = projCord / projCord.w ;
  //shadowCoordinateWdivide.z += 0.0015;
    vec4 color;
    if(projCord.s > 1.0 || projCord.s < 0.0)
    color=vec4(1.0,0,0,1);
    else
        color=vec4(0.0,0,0,0);

  color = texture2D(colorMap,projCord.st);
 //vec4 orig =texture2D( colorMap, gl_TexCoord[0].st)+vec4(0,0,0.4,1);

 gl_FragColor=color;
}
