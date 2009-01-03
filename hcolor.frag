#version 120

varying vec4 normal;
uniform int shaderOut;
uniform vec3 zrange;

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

    gl_FragColor =  color* jet;
}
