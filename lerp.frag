#version 120
#extension GL_EXT_gpu_shader4 : enable
uniform vec3 weights;
uniform int shaderOut;
uniform sampler2DArray theTexture;

vec4 grayV(vec4 total){
  return vec4(max(max(total.x,total.y),total.z),max(max(total.x,total.y),total.z),max(max(total.x,total.y),total.z),1.0);
}
vec4 err(){
  vec4 c[4];
  
  c[0]=texture2DArray(theTexture,gl_TexCoord[1].xyz);
  c[1]=texture2DArray(theTexture,gl_TexCoord[2].xyz);
  c[2]=texture2DArray(theTexture,gl_TexCoord[3].xyz);
  c[3]=texture2DArray(theTexture,gl_TexCoord[4].xyz);
  
  
  float blendA = 0.5;
  vec4 t1 = mix(c[0],c[1],blendA);
  vec4 t2 = mix(c[1],c[2],blendA);
      
  vec4 avg= mix(t1,t2,blendA);	
  vec4 v0=pow(c[0]-avg,vec4(2,2,2,2));
  vec4 v1=pow(c[1]-avg,vec4(2,2,2,2));
  vec4 v2=pow(c[2]-avg,vec4(2,2,2,2));
  vec4 v3=pow(c[3]-avg,vec4(2,2,2,2));
  vec4 total=sqrt((v0+v1+v2+v3)/4);
  vec4 std= vec4(max(max(total.x,total.y),total.z),max(max(total.x,total.y),total.z),max(max(total.x,total.y),total.z),1.0);
   
  return log2(std+1);
}

vec4 avgC(){
  vec4 c[4];

  c[0]=texture2DArray(theTexture,gl_TexCoord[1].xyz);
  c[1]=texture2DArray(theTexture,gl_TexCoord[2].xyz);
  c[2]=texture2DArray(theTexture,gl_TexCoord[3].xyz);
  c[3]=texture2DArray(theTexture,gl_TexCoord[4].xyz);

  float blendA = 0.5;
  vec4 t1 = mix(c[0],c[1],blendA);
  vec4 t2 = mix(c[1],c[2],blendA);
	
  vec4 avg= mix(t1,t2,blendA);	

 
   
  return avg;
}


vec4 distI(){

  float len = length(gl_TexCoord[1].xy - vec2(0.5));
  //return vec4(len,0.0,0.0,1.0);
  return grayV(texture2DArray(theTexture,gl_TexCoord[1].xyz)-texture2DArrayLod(theTexture,gl_TexCoord[1].xyz,3));
}

vec4 freq3Blend(){
  vec3 Cb=weights;
  vec3 mipmapL = vec3(0,2,4);
  float rmax=0.70710678;
  vec3 WSum =vec3(0.0,0.0,0.0);
    
  vec4 outP;
  vec4 outComp[3];
   
  for(int j=0; j < 3; j++){
    for(int i=1;i<5; i++){
      float r = length(gl_TexCoord[i].xy - vec2(0.5,0.5));
      // float num=exp(-((r/rmax)-Cb[j])/Cb[j]);
      float num=exp(-5*((r-Cb[j])/(rmax*Cb[j])));
      float W=num/(1+num);

      if(j == 0)
	outComp[j]+=((texture2DArray(theTexture,gl_TexCoord[i].xyz)-texture2DArrayLod(theTexture,gl_TexCoord[i].xyz,mipmapL[1]))*W);
      else if (j==1)
	outComp[j]+=((texture2DArrayLod(theTexture,gl_TexCoord[i].xyz,mipmapL[1])-texture2DArrayLod(theTexture,gl_TexCoord[i].xyz,mipmapL[2]))*W);
      else if(j==2)
	outComp[j]+=(texture2DArrayLod(theTexture,gl_TexCoord[i].xyz,mipmapL[2])*W);

      WSum[j]+=W;
    }
  }
  outComp[0]=outComp[0]/WSum[0];
  outComp[1]=outComp[1]/WSum[1];
  outComp[2]=outComp[2]/WSum[2];

  outP = outComp[0]+outComp[1]+outComp[2];
  return outP;

}

vec4 freqBlend(){
  float hiCb=weights.x;
  float mipmapL=4;
  float lowCb=weights.y;
  float rmax=0.70710678;
  float lowWSum=0.0;
  float hiWSum=0.0;
  vec4 outP;
  vec4 outHi;
  vec4 outLow;
  
  for(int i=1;i<5; i++){

    float r = length(gl_TexCoord[i].xy - vec2(0.5,0.5));
    float numHi=exp(-((r/rmax)-hiCb)/hiCb);
    float hiW=numHi/(1+numHi);

    float numLow=exp(-((r/rmax)-lowCb)/lowCb);
    float lowW=numLow/(1+numLow);

    vec4 hiF=texture2DArray(theTexture,gl_TexCoord[i].xyz)-texture2DArrayLod(theTexture,gl_TexCoord[i].xyz,mipmapL);
    vec4 loF=texture2DArrayLod(theTexture,gl_TexCoord[i].xyz,mipmapL);
    outHi+=(hiF*hiW);
    outLow+= (loF*lowW);
    hiWSum+=hiW;
    lowWSum+=lowW;

  }
  outP = (outHi/hiWSum + outLow/lowWSum);
  return outP;

}
vec4 pass(){
  return  texture2DArray(theTexture,gl_TexCoord[1].xyz);
}
void main()
{
  vec4 color;

  if(shaderOut == 1)
    color= freqBlend();
  else if(shaderOut ==2)
    color=avgC();
  else if(shaderOut ==3)
    color=err();
  else if(shaderOut ==4)
    color=freq3Blend();	
  else
    color=pass();

  gl_FragColor = color;
} 
 
