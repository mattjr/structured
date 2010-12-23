#version 120


uniform float tilingFactor;
uniform vec4 fc_cc;
uniform vec4 kc1234;
uniform float kc5;

varying vec4 projCord;
uniform mat4 projMatrices[128];
uniform mat4 teMat;
varying vec4 normal;
varying vec3  L, E, H;
varying vec4 vC;
attribute vec4 osg_ProjCoord;
vec3 preMult(mat4 _mat,vec3 v){
 float d = 1.0/(_mat[0][3]*v.x+_mat[1][3]*v.y+_mat[2][3]*v.z+_mat[3][3]) ;
    return vec3( (_mat[0][0]*v.x + _mat[1][0]*v.y + _mat[2][0]*v.z + _mat[3][0])*d,
        (_mat[0][1]*v.x + _mat[1][1]*v.y + _mat[2][1]*v.z + _mat[3][1])*d,
        (_mat[0][2]*v.x + _mat[1][2]*v.y + _mat[2][2]*v.z + _mat[3][2])*d);
}

vec3 postMult( mat4 _mat,vec3 v )	
{
    float d = 1.0f/(_mat[3][0]*v.x+_mat[3][1]*v.y+_mat[3][2]*v.z+_mat[3][3]) ;
    return vec3( (_mat[0][0]*v.x + _mat[0][1]*v.y + _mat[0][2]*v.z + _mat[0][3])*d,
        (_mat[1][0]*v.x + _mat[1][1]*v.y + _mat[1][2]*v.z + _mat[1][3])*d,
        (_mat[2][0]*v.x + _mat[2][1]*v.y + _mat[2][2]*v.z + _mat[2][3])*d) ;
}

void main()
{
 int idx=int(osg_ProjCoord.x);
 mat4 m=projMatrices[idx];
 vec3 localpos=postMult(m,gl_Vertex.xyz);
vec3 globalP=gl_Vertex.xyz;
// vec4  localpos=  gl_Vertex*gl_TextureMatrix[0];
 vec2 n_udis=vec2(localpos.x/localpos.z,localpos.y/localpos.z);
 float r_2 = pow(n_udis.x,2) + pow(n_udis.y,2);
float delta_x = 2*kc1234.z*n_udis.x*n_udis.y + kc1234.w*(r_2 + 2*pow(n_udis.x,2));
 float delta_y = kc1234.z*(r_2 + 2*pow(n_udis.y,2)) + 2*kc1234.w*n_udis.x*n_udis.y;
 float k_radial = 1 + kc1234.x*r_2 + kc1234.y*pow(r_2,2) + kc5*pow(r_2,3);

vec2 n_dis=(n_udis*vec2(k_radial,k_radial))+vec2(delta_x,delta_y);
 vec2 pixelcoord=vec2((fc_cc.x*n_dis.x)+fc_cc.z,(fc_cc.y*n_dis.y)+fc_cc.w);
 float multU=512.0/1360.0;
 float multV=512.0/1024.0;
 pixelcoord*=vec2(multU,multV);
 vec2 texcoord= pixelcoord/vec2(512.0,512.0);
 projCord = vec4(texcoord.x,1-texcoord.y,idx,0);
/*

vec3 p=preMult(projMatrices[idx],gl_Vertex.xyz);
/* gl_TextureMatrix[0];
 projCord.xyz=clamp(p,0.0,1.0);
projCord.xyz=p+vec3(0.0,0.0,0);
*/
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
   vC=osg_ProjCoord;
   gl_TexCoord[0] = gl_MultiTexCoord0;
 
}
