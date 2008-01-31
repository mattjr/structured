#extension GL_ARB_texture_rectangle : enable
#extension GL_EXT_gpu_shader4 : enable
flat varying ivec2 planeCoord;
varying vec3 vpos;
uniform sampler2D rtex;
uniform sampler2DRect planes;
vec3 jet_colormap(float val)
{
  vec3 rgb;
  rgb.x = min(4.0 * val - 1.5,-4.0 * val + 4.5) ;
  rgb.y = min(4.0 * val - 0.5,-4.0 * val + 3.5) ;
  rgb.z = min(4.0 * val + 0.5,-4.0 * val + 2.5) ;
  return rgb;
}

vec4 planeDist(vec3 p,vec4 plane){
  
  vec3 u=plane.xyz;
  float d0=plane.w;
  
  float val=(u[1]*p.x+u[0]*p.y+(-u[2])*p.z+d0);
  //val=gl_Vertex.z;
  return  vec4(jet_colormap(abs(val)),1.0);


}


void main()
{
  // vec4 tmp2=texelFetch2DRect(planes,ivec2(0,0));
   vec4 plane=texelFetch2DRect(planes,planeCoord);
  vec4 dist= planeDist(vpos,plane);
  // tmp.xy=vec2(planeCoord);
  vec4 src= texture2D(rtex,gl_TexCoord[0].xy);
 
  gl_FragColor =dist;//texture2DRect(planes,gl_TexCoord[0].xy);

} 
 
