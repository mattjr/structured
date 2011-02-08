#version 110

uniform float tilingFactor;

varying vec4 normal;
varying vec3  L, E, H;
varying vec3 VaryingTexCoord[4];
attribute vec3 osg_texCoord1;
attribute vec3 osg_texCoord2;
attribute vec3 osg_texCoord3;
attribute vec3 osg_texCoord4;
void main()
{
  normal.xyz = normalize( /*gl_NormalMatrix * */gl_Normal);
    normal.w = gl_Vertex.z;

    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    vec4 eyePosition = gl_ModelViewMatrix * gl_Vertex;
    vec4 eyeLightPos = gl_LightSource[0].position;

    L = normalize(eyeLightPos.xyz - eyePosition.xyz);
    E = -normalize(eyePosition.xyz);
    H = normalize(L + E);
    VaryingTexCoord[0] = osg_texCoord1;
     VaryingTexCoord[1] = osg_texCoord2;
     VaryingTexCoord[2] = osg_texCoord3;
     VaryingTexCoord[3] = osg_texCoord4;
}
