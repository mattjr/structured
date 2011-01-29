#version 120


uniform float tilingFactor;
uniform vec4 fc_cc;
uniform vec4 kc1234;
uniform float kc5;

varying vec4 projCord;
/*uniform mat4 projMatrices[64];
uniform mat4 texMatrices[64];
*/
uniform mat4 teMat;
varying vec4 normal;
varying vec3  L, E, H;
varying vec4 vC;
attribute vec4 osg_ProjCoord;
//attribute vec2 osg_texCoord1 ;



void main()
{
 
projCord.x=gl_MultiTexCoord0.x;
projCord.y=gl_MultiTexCoord0.y;
projCord.z=osg_ProjCoord.x;
projCord.w=osg_ProjCoord.w;


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
