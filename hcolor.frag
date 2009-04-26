#version 120

varying vec4 normal;
uniform int shaderOut;
uniform vec3 zrange;
vec4 HSV_to_RGB (vec4 hsv){
  vec4 color;
  float f,p,q,t;
  float h,s,v;
  float r=0,g=0,b=0;
  float i;
  if (hsv[1] == 0){
    if (hsv[2] != 0){
      color.x = hsv[2];
    }
  }
  else{
    h = hsv.x * 360.0;
    s = hsv.y;
    v = hsv.z;
    if (h == 360.0) {
      h=0;
    }
    h /=60;
    i = floor (h);
    f = h-i;
    p = v * (1.0 - s);
    q = v * (1.0 - (s * f));
    t = v * (1.0 - (s * (1.0 -f)));
    if (i == 0){
      r = v;
      g = t;
      b = p;
    }
    else if (i == 1){
      r = q;
      g = v;
      b = p;
    }
    else if (i == 2){
      r = p;
      g = v;
      b = t;
    }
    else if (i == 3) {
      r = p;
      g = q;
      b = v;
    }
    else if (i == 4) {
      r = t;
      g = p;
      b = v;
    }
    else if (i == 5) {
      r = v;
      g = p;
      b = q;
    }
    color.r = r;
    color.g = g;
    color.b = b;
    color.w = hsv.w;
  }
  return color;
}

vec4 rainbowColorMap(float hue) {
  return HSV_to_RGB(vec4(hue, 1.0f, 1.0f,1.0));
}


vec4 jetColorMap(float val) {
  val= clamp(val,0.0,1.0);
  
  vec4 jet;
        jet.x = min(4.0f * val - 1.5f,-4.0f * val + 4.5f) ;
        jet.y = min(4.0f * val - 0.5f,-4.0f * val + 3.5f) ;
        jet.z = min(4.0f * val + 0.5f,-4.0f * val + 2.5f) ;


        jet.x = clamp(jet.x, 0.0f, 1.0f);
        jet.y = clamp(jet.y, 0.0f, 1.0f);
        jet.z = clamp(jet.z, 0.0f, 1.0f);
	jet.w = 1.0;
        return jet;
}

vec4 coldColorMap(float value)
{
  float max3=1.0/3.0;
  vec4 rgb;
  rgb[3]=1.0;
  if(value<max3)
    {rgb[0]=0;rgb[1]=0;rgb[2]=(value/max3);}
  else if(value<2*max3)
    {rgb[0]=0;rgb[1]=((value-max3)/max3);rgb[2]=1.0;}
  else if(value<1.0)
    {rgb[0]=(value-2*max3)/max3;rgb[1]=1.0;rgb[2]=1.0;}
  else {rgb[0]=rgb[1]=rgb[2]=1.0;}

return rgb;
}

vec4 hotColorMap(float value)
{
  float max3=1.0/3.0;
  vec4 rgb;
  rgb[3]=1.0;
  if(value<max3)
    {rgb[0]=0;rgb[1]=0;rgb[0]=(value/max3);}
  else if(value<2*max3)
    {rgb[0]=1.0;rgb[1]=((value-max3)/max3);rgb[2]=0.0;}
  else if(value<1.0)
    {rgb[0]=1.0; rgb[1]=1.0; rgb[2]=(value-2*max3)/max3;}
  else {rgb[0]=rgb[1]=rgb[2]=1.0;}

return rgb;
}
vec4 positiveColorMap(float value)
{
  vec4 rgb;

  rgb[0]=0.7529;rgb[1]=0;rgb[2]=0;
  rgb[0]+=(0.2470*value);
  rgb[1]+=(1.0*value);
  if(value>0.5)
  rgb[2]+=(2*(value-0.5));
  return rgb;
}

/*

void main()
{   
    vec3 n = normalize(normal.xyz);
    float height = normal.w;
    float nDotL = max(0.0, dot(n, gl_LightSource[0].position.xyz));
        
    vec4 ambient = gl_FrontLightProduct[0].ambient;
    vec4 diffuse = gl_FrontLightProduct[0].diffuse * nDotL;
    vec4 color = gl_FrontLightModelProduct.sceneColor + ambient + diffuse;   

    float range= zrange.y-zrange.x;
    float val =(height-zrange.x)/range;
    
    vec4 jet=jetColorMap(val);
    vec4 ran=rainbowColorMap(val);
    gl_FragColor =  color* ran;
}
*/
 /*vec4 class_color( classid){
  if(classid == -1)
    return vec4(0.0,0.0,0.0,0.0);
  else if(classid == 0)
    return vec4(0.0,0.0,0.0,0.0);
  else if(classid == 1)
    return vec4(1.0,0.0,0.0,0.0);
  else if(classid == 2)
    return vec4(1.0,1.0,0.0,0.0);
 else if(classid == 3)
    return vec4(0.0,0.5,5.0,0.0);
 else if(classid == 4)
    return vec4(0.5,0.5,0.0,0.0);
 else if(classid == 5)
    return vec4(1.0,0.5,0.0,0.0);

    }*/
 varying vec3 L;
 varying vec3 E;
 varying vec3 H;
varying vec4 vC;
uniform bool untex;
uniform float classid;
uniform sampler2D colorMap;
void main()
{

  vec4 aux= vec4(0.0,vC.y,0.0,1.0);
  vec4 auxratio = vec4(0.0,0.5,0.0,1.0);
  vec4 class_color =rainbowColorMap(classid);
  if(shaderOut == 0 && !untex){
    
    gl_FragColor = texture2D( colorMap, gl_TexCoord[0].st);
    // gl_FragColor =  (((vec4(1.0,1.0,1.0,0.0)-auxratio)*texture2D( colorMap, gl_TexCoord[0].st)) + (auxratio * aux));
  }else if(shaderOut == 1 && !untex){
   if(classid == 0.0)
      gl_FragColor =vec4(0,0,0,0);
    else
    gl_FragColor = class_color;
  
  }
  else if(shaderOut == 2 && !untex){
 
    float alpha=0.3;
    if(classid ==0.0)
      gl_FragColor =texture2D( colorMap, gl_TexCoord[0].st);
    else
      gl_FragColor =((1-alpha)*texture2D( colorMap, gl_TexCoord[0].st))+(alpha*class_color);
  }else{
     vec3 NNormal = normalize(normal.xyz);
     vec3 Light  = normalize(vec3(1,  2.5,  -1));
//normalize(vec3(0,0,-1 ));
     vec4 specular_val=vec4( 0.18, 0.18, 0.18, 0.18 );
    
     float mat_shininess = 64.0f ;
     vec4 ambient_val = vec4(0.92, 0.92, 0.92, 0.95 );
      vec4 diffuse_val =vec4( 0.8, 0.8, 0.8, 0.85 );
		

// normalize(L);
     vec3 Eye    = normalize(E);
     vec3 Half   = normalize(E + Light);
     float Kd = max(dot(NNormal, Light), 0.0);
     float Ks = pow(max(dot(Half, NNormal), 0.0),
                mat_shininess);
     float Ka = 1.0;
     
     vec4 diffuse  = Kd * diffuse_val;
     vec4 specular = Ks * specular_val;
     vec4 ambient  = Ka * vec4(0.35,0.35,0.35,1.0) ;//gl_FrontLightProduct[0].ambient;
     float height = normal.w;;
     float range= zrange.y-zrange.x;
     float val =(height-zrange.x)/range;
    
    vec4 jet=jetColorMap(val);
    vec4 ran=rainbowColorMap(val);
    vec4 shadow = vec4(1.0-vC.x,1.0-vC.x,1.0-vC.x,1.0);
    vec4 auxC = hotColorMap(vC.y);//coldColorMap(vC.y);
    //if(shaderOut == 1)
    //  gl_FragColor = ran* shadow* (ambient + diffuse + specular);
    // else
    gl_FragColor = jet   * (ambient + diffuse + specular);
    
  }
}
