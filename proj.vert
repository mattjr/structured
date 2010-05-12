#version 120

uniform float tilingFactor;
uniform vec4 fc;
varying vec4 projCord;


varying vec4 normal;
varying vec3  L, E, H;
varying vec4 vC;

void main()
{

 vec4  localpos=  gl_Vertex*gl_TextureMatrix[0];
 vec2 n=vec2(localpos.x/localpos.z,localpos.y/localpos.z);
 vec2 pixelcoord=vec2((fc.x*n.x)+fc.z,(fc.y*n.y)+fc.w);
 float multU=512.0/1360.0;
 float multV=512.0/1024.0;
 pixelcoord*=vec2(multU,multV);
 vec2 texcoord= pixelcoord/vec2(512.0,512.0);
 projCord = vec4(texcoord.x,1-texcoord.y,0,0);
  normal.xyz = normalize( /*gl_NormalMatrix * */gl_Normal);
    normal.w = gl_Vertex.z;

    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    vec4 eyePosition = gl_ModelViewMatrix * gl_Vertex;
    vec4 eyeLightPos = gl_LightSource[0].position;

    L = normalize(eyeLightPos.xyz - eyePosition.xyz);
    E = -normalize(eyePosition.xyz);
    H = normalize(L + E);
    vC=gl_Color;
    gl_TexCoord[0] = gl_MultiTexCoord0;
 
}
