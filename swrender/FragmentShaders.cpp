#include "FragmentShaders.h"
Texture *FragmentShader::texture = 0;
TextureMipMap *FragmentShaderBlendingMain::texture = 0;
TextureMipMap *FragmentShaderBlendingDistPass::texture = 0;

int FragmentShaderBlendingDistPass::idx =-1;
int FragmentShaderBlendingMain::triIdx;
int FragmentShaderBlendingDistPass::triIdx;

const int FragmentShaderBlendingMain::mipmapL[]= {0,2,4};
int FragmentShaderBlendingMain::idx;

Rect gRect;
REGION *regOutput;
REGION *regRange;


int doubleTouchCount=0;

dcm_t doublecountmap;
