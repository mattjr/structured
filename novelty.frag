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


void main()
{
  
  vec4 infoC= texture2D(infoT,gl_TexCoord[0].xy);
  vec4 src= texture2D(rtex,gl_TexCoord[0].xy);
 
  gl_FragColor = vec4(jet_colormap(infoC.x),1);

} 
 
