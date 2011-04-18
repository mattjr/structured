#version 110


varying vec3 VaryingTexCoord[4];
attribute vec3 osg_texCoord1;
attribute vec3 osg_texCoord2;
attribute vec3 osg_texCoord3;
attribute vec3 osg_texCoord4;
varying float height;

vec4 texture( in vec3 position, in vec3 normal );
void lighting( in vec3 position, in vec3 normal );

void main()
{
    height=gl_Vertex.z;
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    vec4 position4 = gl_ModelViewMatrix * gl_Vertex;
    vec3 position = position4.xyz / position4.w;
    vec3 normal = normalize( gl_NormalMatrix * gl_Normal );

    VaryingTexCoord[0] = osg_texCoord1;
    VaryingTexCoord[1] = osg_texCoord2;
    VaryingTexCoord[2] = osg_texCoord3;
    VaryingTexCoord[3] = osg_texCoord4;
    lighting( position, normal );

}
varying vec3 Normal;
varying vec3 Position; // not used for directional lighting
void lighting( in vec3 position, in vec3 normal )
{
    Normal = normal;
    Position = position;
}


