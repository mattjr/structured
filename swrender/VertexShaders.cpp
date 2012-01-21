#include "VertexShaders.h"
mat4x VertexShaderBlending::modelview_matrix;
mat4x VertexShaderBlending::modelviewprojection_matrix;
int VertexShaderBlending::triIdx;

TextureMipMap *VertexShaderBlending::texture = 0;

mat4x VertexShaderBlendingDistPass::modelview_matrix;
mat4x VertexShaderBlendingDistPass::modelviewprojection_matrix;

mat4x VertexShader::modelview_matrix;
mat4x VertexShader::modelviewprojection_matrix;
Texture *VertexShader::texture = 0;
