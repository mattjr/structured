#version 120

uniform float tilingFactor;

varying vec4 normal;
varying vec3  L, E, H;

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
    gl_TexCoord[0] = gl_MultiTexCoord0;
     gl_TexCoord[1] = gl_MultiTexCoord1;
     gl_TexCoord[2] = gl_MultiTexCoord2;
     gl_TexCoord[3] = gl_MultiTexCoord3;
     gl_TexCoord[4] = gl_MultiTexCoord4;

}
