uniform int shaderOut;
varying vec4 normal;
uniform vec3 zrange;
varying vec3 L;
varying vec3 E;
varying vec3 H;
varying vec3 VaryingTexCoord[4];
varying float ao;

uniform sampler2D theTexture;
vec4 jetColorMap(float val) {
  val= clamp(val,0.0,1.0);
  
  vec4 jet;
        jet.x = min(4.0 * val - 1.5,-4.0 * val + 4.5) ;
        jet.y = min(4.0 * val - 0.5,-4.0 * val + 3.5) ;
        jet.z = min(4.0 * val + 0.5,-4.0 * val + 2.5) ;


        jet.x = clamp(jet.x, 0.0, 1.0);
        jet.y = clamp(jet.y, 0.0, 1.0);
        jet.z = clamp(jet.z, 0.0, 1.0);
	jet.w = 1.0;
        return jet;
}
vec4 HSV_to_RGB (vec4 hsv){
  vec4 color;
  float f,p,q,t;
  float h,s,v;
  float r=0.0,g=0.0,b=0.0;
  float i;
  if (hsv[1] == 0.0){
    if (hsv[2] != 0.0){
      color.x = hsv[2];
    }
  }
  else{
    h = hsv.x * 360.0;
    s = hsv.y;
    v = hsv.z;
    if (h == 360.0) {
      h=0.0;
    }
    h /=60.0;
    i = floor (h);
    f = h-i;
    p = v * (1.0 - s);
    q = v * (1.0 - (s * f));
    t = v * (1.0 - (s * (1.0 -f)));
    if (i == 0.0){
      r = v;
      g = t;
      b = p;
    }
    else if (i == 1.0){
      r = q;
      g = v;
      b = p;
    }
    else if (i == 2.0){
      r = p;
      g = v;
      b = t;
    }
    else if (i == 3.0) {
      r = p;
      g = q;
      b = v;
    }
    else if (i == 4.0) {
      r = t;
      g = p;
      b = v;
    }
    else if (i == 5.0) {
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
void main()
{
  vec4 color;

  if(shaderOut == 0)
    color=texture2D(theTexture,VaryingTexCoord[0].xy);
  else{
  vec3 NNormal = normalize(normal.xyz);
  vec3 Light  = normalize(vec3(1,  2.5,  -1));
  vec4 specular_val=vec4( 0.18, 0.18, 0.18, 0.18 );

  float mat_shininess = 64.0f ;
  vec4 ambient_val = vec4(0.92, 0.92, 0.92, 0.95 );
  vec4 diffuse_val =vec4( 0.8, 0.8, 0.8, 0.85 );
  vec3 Eye    = normalize(E);
  vec3 Half   = normalize(E + Light);
  float Kd = max(dot(NNormal, Light), 0.0);
  float Ks = pow(max(dot(Half, NNormal), 0.0),
                 mat_shininess);
  float Ka = 1.0;

  vec4 diffuse  = Kd * diffuse_val;
  vec4 specular = Ks * specular_val;
  vec4 ambient  = Ka * vec4(0.35,0.35,0.35,1.0) ;
  float height = normal.w;;
  float range=0.0;
  float val=0.0;

  range= zrangeHi-zrangeLow;
  val =(height-zrangeLow)/range;
  vec4 jet=rainbowColorMap(val);
  vec4 aoV=vec4(ao,ao,ao,1.0);
  color = jet * aoV * (ambient + diffuse + specular);

  
  gl_FragColor = color;
} 
 
