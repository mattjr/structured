#include "FragmentShaders.h"
Texture *FragmentShader::texture = 0;
TextureMipMap *FragmentShaderBlendingMain::texture = 0;
TextureMipMap *FragmentShaderBlendingDistPass::texture = 0;

int FragmentShaderBlendingDistPass::idx =-1;
int FragmentShaderBlendingDistPass::triIdx;
int FragmentShaderBlendingMain::triIdx;

const int FragmentShaderBlendingMain::mipmapL[]= {0,2,4};
int FragmentShaderBlendingMain::idx;
bool FragmentShaderVarMain::writeOut;
FILE *FragmentShaderVarMain::f_arr[3];
IplImage *FragmentShaderVarMain::outputImage;
 std::vector<osg::Vec2> FragmentShaderCollectTC:: tc;
Rect gRect;
REGION *regOutput;
REGION *regRange;


int doubleTouchCount=0;

dcm_t doublecountmap;
