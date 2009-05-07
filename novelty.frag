

varying vec4 planeData;
varying vec3 vpos;
uniform sampler2D rtex;
uniform int shaderOut;
uniform vec3 weights;

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

float planeDist(vec3 p,vec4 plane,float height_weight){
  
  vec3 u=plane.xyz;
  float d0=plane.w;
  
  float val=(u[0]*p.x+u[1]*p.y+u[2]*p.z+d0);
  return min(abs(val)*height_weight,1.0);


}


void main()
{

 
  vec3 actual_weights;
  if(weights.x == 0.0 && weights.y == 0.0 && weights.z == 0.0)
    actual_weights=vec3(1.1, 0.66, 0.26);
  else
    actual_weights=weights;

  float dist=0.0;
  if(planeData != vec4(0,0,0,0) ){
    dist = planeDist(vpos,planeData,actual_weights.x);
  }
   
   vec4 src= texture2D(rtex,gl_TexCoord[0].xy);
   float nov=src.w;
   vec4 color;
   float total=mix(dist,nov,actual_weights.y);

   if(shaderOut == 0)
     //    color= vec4(src.xyz,1);
        color=jet_colormap(planeData.w+58.3079+1.0);
   else if(shaderOut ==1){
    if(nov > actual_weights.z)
      color=mix(vec4(0.0,1.0,0.0,1.0),src,0.6);
    else
      color=src;
   }
   else if(shaderOut ==2){
    if(total > actual_weights.z)
      color=mix(vec4(1.0,0.0,0.0,1.0),src,0.6);
    else
      color=src;
   }else if(shaderOut ==3)
     color=jet_colormap(nov);
   else if(shaderOut ==4)
     color=jet_colormap(dist*0.75);
   else if(shaderOut ==5)
     color=jet_colormap(total);


   gl_FragColor =color;

} 
 
