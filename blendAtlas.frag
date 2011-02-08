#version 110
uniform vec3 weights;
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


vec4 freq3Blend(vec3 Cb){
  vec3 mipmapL = vec3(0,2,4);
  float rmax=0.70710678;
  vec3 WSum =vec3(0.0,0.0,0.0);
    
  vec4 outP;
  vec4 outComp[3];
  outComp[0]=vec4(0.0,0.0,0.0,0.0);
  outComp[1]=vec4(0.0,0.0,0.0,0.0);
  outComp[2]=vec4(0.0,0.0,0.0,0.0);

  bool validPix=false;
  for(int j=0; j < 3; j++){
    for(int i=0;i<4; i++){
      //If no valid texture at this pixel don't blend it
      if(VaryingTexCoord[i].z < 0.0 || VaryingTexCoord[i].x < 0.0 || VaryingTexCoord[i].x > 1.0 ||VaryingTexCoord[i].y <0.0||VaryingTexCoord[i].y >1.0)
            continue;
      validPix=true;

      float r=VaryingTexCoord[i].z;
      float W=exp(-r*10.0*16.0*Cb[j]);


      if(j == 0)
	outComp[j]+=((texture2D(theTexture,VaryingTexCoord[i].xy)-texture2DLod(theTexture,VaryingTexCoord[i].xy,mipmapL[1]))*W);
      else if (j==1)
	outComp[j]+=((texture2DLod(theTexture,VaryingTexCoord[i].xy,mipmapL[1])-texture2DLod(theTexture,VaryingTexCoord[i].xy,mipmapL[2]))*W);
      else if(j==2)
	outComp[j]+=(texture2DLod(theTexture,VaryingTexCoord[i].xy,mipmapL[2])*W);

      WSum[j]+=W;
    }
  }
  //No valid texture at all put color
  if(!validPix)
    return gl_Color;
  outComp[0]=outComp[0]/WSum[0];
  outComp[1]=outComp[1]/WSum[1];
  outComp[2]=outComp[2]/WSum[2];

  outP = outComp[0]+outComp[1]+outComp[2];
  return outP;

}

vec4 avgA(){
  vec4 c[4];
vec3 v[4];
v[0]=VaryingTexCoord[0].xyz;
v[1]=VaryingTexCoord[1].xyz;
v[2]=VaryingTexCoord[2].xyz;
v[3]=VaryingTexCoord[3].xyz;

  c[0]=texture2D(theTexture,VaryingTexCoord[0].xy);
  c[1]=texture2D(theTexture,VaryingTexCoord[1].xy);
  c[2]=texture2D(theTexture,VaryingTexCoord[2].xy);
  c[3]=texture2D(theTexture,VaryingTexCoord[3].xy);

  float blendA = 0.5;
  vec4 t1 = mix(c[0],c[1],blendA);
  vec4 t2 = mix(c[1],c[2],blendA);
	
  vec4 avg= mix(t1,t2,blendA);	
   
  return avg;
}

void main()
{
  vec4 color;
  vec3 usedWeights;
  usedWeights=weights;

  if(usedWeights.x==0.0 && usedWeights.y==0.0 &&usedWeights.z==0.0)
    usedWeights=vec3(0.710000,0.650000,0.070000);
 
  if(shaderOut == 0)
    color= freq3Blend(usedWeights);
  else if(shaderOut ==1)
    color=texture2D(theTexture,VaryingTexCoord[0].xy);
  else if(shaderOut ==2){
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
  else
    color=texture2D(theTexture,VaryingTexCoord[0].xy);
  
  gl_FragColor = color;
} 
 
