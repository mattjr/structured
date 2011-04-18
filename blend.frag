uniform vec3 weights;
uniform int shaderOut;
varying float height;


varying vec3 VaryingTexCoord[4];
vec4 texture( void );
void lighting( inout vec4 color );
uniform sampler2DArray theTexture;
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



vec4 avgC(){
   vec4 c[4];
   vec3 v[4];
   v[0]=VaryingTexCoord[0].xyz;
   v[1]=VaryingTexCoord[1].xyz;
   v[2]=VaryingTexCoord[2].xyz;
   v[3]=VaryingTexCoord[3].xyz;

  c[0]=texture2DArray(theTexture,VaryingTexCoord[0].xyz);
  c[1]=texture2DArray(theTexture,VaryingTexCoord[1].xyz);
  c[2]=texture2DArray(theTexture,VaryingTexCoord[2].xyz);
  c[3]=texture2DArray(theTexture,VaryingTexCoord[3].xyz);

  float blendA = 0.5;
  vec4 t1 = mix(c[0],c[1],blendA);
  vec4 t2 = mix(c[1],c[2],blendA);
	
  vec4 avg= mix(t1,t2,blendA);	

 
   
  return avg;
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
      float r = length(VaryingTexCoord[i].xy - vec2(0.5,0.5));
      float W=exp(-r*10.0*16.0*Cb[j]);


      if(j == 0)
	outComp[j]+=((texture2DArray(theTexture,VaryingTexCoord[i].xyz)-texture2DArrayLod(theTexture,VaryingTexCoord[i].xyz,mipmapL[1]))*W);
      else if (j==1)
	outComp[j]+=((texture2DArrayLod(theTexture,VaryingTexCoord[i].xyz,mipmapL[1])-texture2DArrayLod(theTexture,VaryingTexCoord[i].xyz,mipmapL[2]))*W);
      else if(j==2)
	outComp[j]+=(texture2DArrayLod(theTexture,VaryingTexCoord[i].xyz,mipmapL[2])*W);

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



void main()
{
  vec4 color=vec4(1.0,0,0,1.0);
  vec3 usedWeights;
  usedWeights=weights;

  if(usedWeights.x==0.0 && usedWeights.y==0.0 &&usedWeights.z==0.0)
    usedWeights=vec3(0.710000,0.650000,0.070000);

 if(shaderOut == 0)
    color= freq3Blend(usedWeights);
  else if(shaderOut ==1)
    color=texture2DArray(theTexture,VaryingTexCoord[0].xyz);
  else if(shaderOut ==3){
    color = texture();
    lighting( color );
  }
  else
    color=texture2DArray(theTexture,VaryingTexCoord[0].xyz);

  gl_FragColor = color;
} 
varying vec3 Normal;
varying vec3 Position; // not used for directional lighting
void lighting( inout vec4 color )
{
    vec3 n = normalize( Normal );
    float NdotL = dot( n, normalize(gl_LightSource[0].position.xyz) );
    NdotL = max( 0.0, NdotL );
    float NdotHV = dot( n, gl_LightSource[0].halfVector.xyz );
    NdotHV = max( 0.0, NdotHV );
    color *= gl_FrontLightModelProduct.sceneColor +
             gl_FrontLightProduct[0].ambient +
             gl_FrontLightProduct[0].diffuse * NdotL;

   if ( NdotL * NdotHV > 0.0 )
        color += gl_FrontLightProduct[0].specular *
                 pow( NdotHV, gl_FrontMaterial.shininess );
}

vec4 texture( void )
{

    float range= zrangeHi-zrangeLow;
    float val =(height-zrangeLow)/range;
    return rainbowColorMap(val);
}
