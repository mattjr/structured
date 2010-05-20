

uniform float tilingFactor;
uniform vec4 fc;
varying vec4 projCord;

uniform mat4 teMat;
varying vec4 normal;
varying vec3  L, E, H;
varying vec4 vC;
vec3 preMult(mat4 _mat,vec3 v){
 float d = 1.0/(_mat[0][3]*v.x+_mat[1][3]*v.y+_mat[2][3]*v.z+_mat[3][3]) ;
    return vec3( (_mat[0][0]*v.x + _mat[1][0]*v.y + _mat[2][0]*v.z + _mat[3][0])*d,
        (_mat[0][1]*v.x + _mat[1][1]*v.y + _mat[2][1]*v.z + _mat[3][1])*d,
        (_mat[0][2]*v.x + _mat[1][2]*v.y + _mat[2][2]*v.z + _mat[3][2])*d);
}
void main()
{
/*
 vec4  localpos=  gl_Vertex*gl_TextureMatrix[0];
 vec2 n=vec2(localpos.x/localpos.z,localpos.y/localpos.z);
 vec2 pixelcoord=vec2((fc.x*n.x)+fc.z,(fc.y*n.y)+fc.w);
 float multU=512.0/1360.0;
 float multV=512.0/1024.0;
 pixelcoord*=vec2(multU,multV);
 vec2 texcoord= pixelcoord/vec2(512.0,512.0);
 projCord = vec4(texcoord.x,1-texcoord.y,0,0);*/


vec3 p=preMult(teMat,gl_Vertex.xyz);//=gl_TextureMatrix[0];
projCord.xyz=p;


 //projCord=gl_Vertex*gl_TextureMatrix[0];

//vec3 texcoord=gl_Vertex*gl_TextureMatrix[0];
  normal.xyz = normalize( /*gl_NormalMatrix * */gl_Normal);
    normal.w = gl_Vertex.z;

    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
   vec4 eyePosition = gl_ModelViewMatrix * gl_Vertex;
   vec4 eyeLightPos = gl_LightSource[0].position;

   L = normalize(eyeLightPos.xyz - eyePosition.xyz);
   E = -normalize(eyePosition.xyz);
   H = normalize(L + E);
   vC=gl_Color;
   //gl_TexCoord[0] = gl_MultiTexCoord0;
 
}
