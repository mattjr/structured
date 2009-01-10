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

 varying vec3 L;
 varying vec3 E;
 varying vec3 H;

void main()
{
     vec3 NNormal = normalize(normal.xyz);
     vec3 Light  = normalize(vec3(1,  2.5,  -1));
//normalize(vec3(0,0,-1 ));

// normalize(L);
     vec3 Eye    = normalize(E);
     vec3 Half   = normalize(E + Light);
     float Kd = max(dot(NNormal, Light), 0.0);
     float Ks = pow(max(dot(Half, NNormal), 0.0),
                 gl_FrontMaterial.shininess);
     float Ka = 1.0;
     
     vec4 diffuse  = Kd * gl_FrontLightProduct[0].diffuse;
     vec4 specular = Ks * gl_FrontLightProduct[0].specular;
     vec4 ambient  = Ka * vec4(0.35,0.35,0.35,1.0) ;//gl_FrontLightProduct[0].ambient;
     float height = normal.w;;
     float range= zrange.y-zrange.x;
     float val =(height-zrange.x)/range;
    
    vec4 jet=jetColorMap(val);
    vec4 ran=rainbowColorMap(val);
    gl_FragColor = ran* (ambient + diffuse + specular);
}
