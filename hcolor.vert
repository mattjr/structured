#version 120

uniform float tilingFactor;

varying vec4 normal;
varying vec3  L, E, H;
varying vec4 vC;

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
    vC=gl_Color;
}
