#version 110
uniform int shaderOut;
varying vec4 normal;
uniform vec3 zrange;
varying vec3 L;
varying vec3 E;
varying vec3 H;
varying vec3 VaryingTexCoord[4];

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

void main()
{
  vec4 color;

  if(shaderOut == 0)
    color=texture2D(theTexture,VaryingTexCoord[0].xy);
  else{
    vec3 NNormal = normalize(normal.xyz);
    vec3 Light  = normalize(vec3(1,  2.5,  -1));
    vec4 specular_val=vec4( 0.18, 0.18, 0.18, 0.18 );
    
    float mat_shininess = 64.0;
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
    float range= zrange.y-zrange.x;
    float val =(height-zrange.x)/range;
    vec4 jet=jetColorMap(1.0-val);
    color = jet * (ambient + diffuse + specular);
  }
  
  gl_FragColor = color;
} 
 
