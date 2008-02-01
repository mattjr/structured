#extension GL_ARB_texture_rectangle : enable
#extension GL_EXT_gpu_shader4 : enable
uniform sampler2D infoT;
flat varying ivec2 planeCoord;
varying vec3 vpos;
uniform sampler2D rtex;
uniform sampler2DRect planes;
uniform int shaderOut;
uniform vec3 weights;


vec3 jet_colormap(float val)
{
  vec3 rgb;
  rgb.x = min(4.0 * val - 1.5,-4.0 * val + 4.5) ;
  rgb.y = min(4.0 * val - 0.5,-4.0 * val + 3.5) ;
  rgb.z = min(4.0 * val + 0.5,-4.0 * val + 2.5) ;
  return rgb;
}

float planeDist(vec3 p,vec4 plane){
  
  vec3 u=plane.xyz;
  float d0=plane.w;
  
  float val=(u[1]*p.x+u[0]*p.y+(-u[2])*p.z+d0);
  return min(abs(val)*weights.x,1.0);


}


void main()
{
  float dist=0.0;
  if(planeCoord.x >= 0 && planeCoord.y >= 0){
    vec4 plane=texelFetch2DRect(planes,planeCoord);
    dist = planeDist(vpos,plane);
  }
   float nov=texture2D(infoT,gl_TexCoord[0].xy).x;
   vec4 src= texture2D(rtex,gl_TexCoord[0].xy);

   vec4 color;
   float total=mix(dist,nov,weights.y);

   if(shaderOut == 1)
    color= src;
   else if(shaderOut ==2){
    if(total > weights.z)
      color=mix(vec4(0.0,1.0,0.0,1.0),src,0.6);
    else
      color=src;
   }else if(shaderOut ==3)
     color=vec4(jet_colormap(nov),1);
   else if(shaderOut ==4)
     color=vec4(jet_colormap(dist),1);
   else if(shaderOut ==0)
     color=vec4(jet_colormap(total),1);

   gl_FragColor =color;

} 
 
