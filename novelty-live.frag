#extension GL_ARB_texture_rectangle : enable

uniform sampler2D rtex;
uniform sampler2DRect hist;
uniform sampler2D infoT;
uniform int shaderOut;
vec3 jet_colormap(float val)
{
  vec3 rgb;
  rgb.x = min(4.0 * val - 1.5,-4.0 * val + 4.5) ;
  rgb.y = min(4.0 * val - 0.5,-4.0 * val + 3.5) ;
  rgb.z = min(4.0 * val + 0.5,-4.0 * val + 2.5) ;
  return rgb;
}
vec3 colorband_rgb_hsv(vec3 rgb)
{
  vec3 color = rgb;
  float maxn = max(max(rgb.r, rgb.g), rgb.b);
  float minn = min(min(rgb.r, rgb.g), rgb.b);
  float delta;

  color.b = maxn;

  if(maxn != 0.0)
    color.g = (maxn - minn) / maxn;
  else
    color.g = 0.0;

  if(color.g == 0.0)
    color.b = -1.0;

  else{
    delta =  maxn - minn;
    if(rgb.r == maxn)
      color.r = (rgb.g - rgb.b) / delta;
    else if(rgb.g == maxn)
      color.r = 2.0 + (rgb.b - rgb.r) / delta;
    else if(rgb.b == maxn)
      color.r = 4.0 + (rgb.r - rgb.g) / delta;

    color.r = color.r * 60.0;

    if(color.r < 0.0)
      color.r = color.r + 360.0;
  }

  return(color);
}


void main()
{
   float binsize=16.0;
  
  
  // gg2=texture2DRect(hist,vec2(0,i)).x;
 
  vec4 src= texture2D(rtex,gl_TexCoord[0].xy);
  vec4 hsv;
  hsv.xyz=colorband_rgb_hsv(src.xyz);
  //  texture2DRect(hist,vec2(0,i)).x;
  hsv.w=src.w;
  vec4 div=vec4(360,1,1,1);
  vec4 bin=min(floor((hsv/div)*(binsize)),binsize-1.0);
  float idx=bin.y *(16*16) + (bin.z * 16) + bin.w;
  float i=min(floor((idx/(16.0*16*16))*64.0),64-1.0);
  float j=mod(idx,63.0);
  float nov=texture2D(infoT,gl_TexCoord[0].xy).x;
  float val=texture2DRect(hist,vec2(i,j)).x;
 if(shaderOut == 0)
   gl_FragColor = vec4(src.xyz,1); //vec4(jet_colormap(val),1);
else  if(shaderOut == 1)
    gl_FragColor = vec4(jet_colormap(val),1);
else  if(shaderOut == 2)
    gl_FragColor = vec4(jet_colormap(src.w),1);
else  if(shaderOut == 3)
  gl_FragColor = vec4(jet_colormap(nov),1);
} 
 
