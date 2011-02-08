#version 110

uniform float tilingFactor;

varying vec4 normal;
varying vec3  L, E, H;
varying vec3 VaryingTexCoord[4];
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
    VaryingTexCoord[0] = gl_MultiTexCoord0.xyz;
     VaryingTexCoord[1] = gl_MultiTexCoord1.xyz;
     VaryingTexCoord[2] = gl_MultiTexCoord2.xyz;
     VaryingTexCoord[3] = gl_MultiTexCoord3.xyz;
     //gl_TexCoord[0] = gl_MultiTexCoord4;

}
