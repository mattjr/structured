#extension GL_ARB_texture_rectangle : enable

uniform sampler2D rtex;
uniform sampler2D infoT;
vec3 jet_colormap(float val)
{
  vec3 rgb;
  rgb.x = min(4.0 * val - 1.5,-4.0 * val + 4.5) ;
  rgb.y = min(4.0 * val - 0.5,-4.0 * val + 3.5) ;
  rgb.z = min(4.0 * val + 0.5,-4.0 * val + 2.5) ;
  return rgb;
}

uniform int shaderOut;
uniform vec3 weights;

void main()
{
  

  vec4 infoC= texture2D(infoT,gl_TexCoord[0].xy);
  vec4 src= texture2D(rtex,gl_TexCoord[0].xy);
  vec4 color;
  float nov=infoC.x;
  if(shaderOut == 1)
    color= src;
  else if(shaderOut ==3)
    color=vec4(jet_colormap(nov),1);
  else if(shaderOut ==2){
    if(nov > weights.z)
      color=mix(vec4(0.0,1.0,0.0,1.0),src,0.6);
    else
      color=src;
  }
    
  gl_FragColor = color;

} 
 
