#version 120

uniform float tilingFactor;

varying vec4 normal;



void main()
{
    normal.xyz = normalize(gl_NormalMatrix * gl_Normal);
    normal.w = gl_Vertex.z;

    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    gl_TexCoord[0] = gl_MultiTexCoord0 * tilingFactor;
}
