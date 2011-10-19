/*
Copyright (c) 2007, Markus Trenkwalder

All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

* Neither the name of the library's copyright owner nor the names of its 
contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <time.h>
#include <string>
#include <iostream>
#include <osg/TriangleIndexFunctor>
#include <osg/Timer>
// requires software renderer source
#include "renderer/geometry_processor.h"
#include "renderer/rasterizer_subdivaffine.h"
#include "renderer/span.h"
#include "democommon.h"
#include "fixedpoint/fixed_func.h"
#include <osg/io_utils>
#include <osg/ComputeBoundsVisitor>
#include "vertexData.h"
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osgUtil/Optimizer>
#include <opencv/cv.h>
#include <osgDB/FileNameUtils>
#include <opencv/highgui.h>
#include "mipmap.h"
#include "render_utils.h"
#include "../MemUtils.h"
#include "../GLImaging.h"
// the software renderer stuff is located in the namespace "swr" so include 
// that here
using namespace swr;
using namespace std;
#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>
#include <vips/vips>
#include <osgDB/FileUtils>
#include <vips/vips.h>
static osg::Vec3 debugV;
bool USE_BGR=false;
struct InputVertex {
    vec3x vertex;
};

static IMAGE *outputImage;
static IMAGE *rangeImage;
static REGION *regOutput;
static REGION *regRange;
static Rect gRect;
/* Generate function --- just black out the region.*/
static int white_gen( REGION *reg, void *seq, void *a, void *b )
{
    im_region_paint( reg,&reg->valid ,255);

    return( 0 );
}

/**
 * im_black:
 * @out: output #IMAGE
 * @x: output width
 * @y: output height
 * @bands: number of output bands
 *
 * Make a black unsigned char image of a specified size.
 *
 * See also: im_make_xy(), im_text(), im_gaussnoise().
 *
 * Returns: 0 on success, -1 on error
 */
int
        im_white( IMAGE *out, int x, int y, int bands )
{
    if( x <= 0 || y <= 0 || bands <= 0 ) {
        im_error( "im_white", "%s", "bad parameter"  );
        return( -1 );
    }

    if( im_poutcheck( out ) )
        return( -1 );
    im_initdesc( out,
                 x, y, bands,
                 IM_BBITS_BYTE, IM_BANDFMT_UCHAR, IM_CODING_NONE,
                 bands == 1 ? IM_TYPE_B_W : IM_TYPE_MULTIBAND,
                 1.0, 1.0, 0, 0 );
    if( im_demand_hint( out, IM_ANY, NULL ) )
        return( -1 );

    if( im_generate( out, NULL, white_gen, NULL, NULL, NULL ) )
        return( -1 );

    return( 0 );
}
// this is the vertex shader which is executed for each individual vertex that
// needs to ne processed.
struct VertexShader {

    // this specifies that this shader is only going to use 1 vertex attribute
    // array. There you be used up to Renderer::MAX_ATTRIBUTES arrays.
    static const unsigned attribute_count = 1;

    // this specifies the number of varyings the shader will output. This is
    // for instance used when clipping.
    static const unsigned varying_count = 2;
    static mat4x modelview_matrix;
    static mat4x modelviewprojection_matrix;
    // this static function is called for each vertex to be processed.
    // "in" is an array of void* pointers with the location of the individial
    // vertex attributes. The "out" structure has to be written to.
    static void shade(const GeometryProcessor::VertexInput in, GeometryProcessor::VertexOutput &out)
    {
        // cast the first attribute array to the input vertex type
        const Vertex &v = *static_cast<const Vertex*>(in[0]);

        // x, y, z and w are the components that must be written by the vertex
        // shader. They all have to be specified in 16.16 fixed point format.

        vec4x tvertex = modelviewprojection_matrix * vec4x(v.x,v.y,v.z, fixed16_t(1));

        //        const mat4x &m = modelview_matrix;

        out.x = tvertex.x.intValue;
        out.y = tvertex.y.intValue;
        out.z = fixed16_t(1.0).intValue;//tvertex.z.intValue;
        out.w = 1 << 16;//tvertex.w.intValue;
        // bring the texture coordinates into the appropriate range for the rasterizer.
        // this mean it has to be converted to fixed point and premultiplied with the width, height of the
        // texture minus 1. Doing this in the vertex shader saves us from doing this in the fragment shader
        // which makes things faster.
        out.varyings[0] = static_cast<int>(v.tx * texture->w_minus1 * (1 << 16));
        out.varyings[1] = static_cast<int>(v.ty * texture->h_minus1 * (1 << 16));

    }
    static Texture *texture;

};
mat4x VertexShader::modelview_matrix;
mat4x VertexShader::modelviewprojection_matrix;
Texture *VertexShader::texture = 0;


// this is the fragment shader
struct FragmentShader : public SpanDrawer32BitColorAndDepth<FragmentShader> {
    // varying_count = 3 tells the rasterizer that it only needs to interpolate
    // three varying values (the r, g and b in this context).
    static const unsigned varying_count = 2;

    // we don't need to interpolate z in this example
    static const bool interpolate_z = false;

    // per triangle callback. This could for instance be used to select the
    // mipmap level of detail. We don't need it but it still needs to be defined
    // for everything to work.
    static void begin_triangle(
            const IRasterizer::Vertex& v1,
            const IRasterizer::Vertex& v2,
            const IRasterizer::Vertex& v3,
            int area2)
    {}

    // the fragment shader is called for each pixel and has read/write access to
    // the destination color and depth buffers.
    static void single_fragment(const IRasterizer::FragmentData &fd, unsigned int &color, unsigned int &depth)
    {
        // sample the texture and write the color information
        unsigned int c =texture->sample_nearest(fd.varyings[0], fd.varyings[1]);
        // always write the color;
        color = c;
    }

    // this is called by the span drawing function to get the location of the color buffer
    static void* color_pointer(int x, int y)
    {

        gRect.left=x;
        gRect.top=y;

        if(im_prepare(regOutput,&gRect)){
            fprintf(stderr,"Prepare fail\n");
            exit(-1);
        }

        return IM_REGION_ADDR( regOutput, x, y );
    }

    // this is called by the span drawing function to get the location of the depth buffer
    static void* depth_pointer(int x, int y)
    {
        // We don't use a depth buffer
        return 0;
    }

    static Texture *texture;
};


// this is the vertex shader which is executed for each individual vertex that
// needs to ne processed.
struct VertexShaderBlending {

    // this specifies that this shader is only going to use 1 vertex attribute
    // array. There you be used up to Renderer::MAX_ATTRIBUTES arrays.
    static const unsigned attribute_count = 1;

    // this specifies the number of varyings the shader will output. This is
    // for instance used when clipping.
    static const unsigned varying_count = 2;
    static mat4x modelview_matrix;
    static mat4x modelviewprojection_matrix;
    // this static function is called for each vertex to be processed.
    // "in" is an array of void* pointers with the location of the individial
    // vertex attributes. The "out" structure has to be written to.
    static void shade(const GeometryProcessor::VertexInput in, GeometryProcessor::VertexOutput &out)
    {
        // cast the first attribute array to the input vertex type
        const Vertex &v = *static_cast<const Vertex*>(in[0]);

        // x, y, z and w are the components that must be written by the vertex
        // shader. They all have to be specified in 16.16 fixed point format.

        vec4x tvertex = modelviewprojection_matrix * vec4x(v.x,v.y,v.z, fixed16_t(1));

        //const mat4x &m = modelview_matrix;

        out.x = tvertex.x.intValue;
        out.y = tvertex.y.intValue;
        out.z = fixed16_t(1.0).intValue;//tvertex.z.intValue;
        out.w = 1 << 16;//tvertex.w.intValue;
        // bring the texture coordinates into the appropriate range for the rasterizer.
        // this mean it has to be converted to fixed point and premultiplied with the width, height of the
        // texture minus 1. Doing this in the vertex shader saves us from doing this in the fragment shader
        // which makes things faster.
        out.varyings[0] = static_cast<int>(v.tx * texture->w_minus1 * (1 << 16));
        out.varyings[1] = static_cast<int>(v.ty * texture->h_minus1 * (1 << 16));


    }
    static int triIdx;

    static TextureMipMap *texture;

};
mat4x VertexShaderBlending::modelview_matrix;
mat4x VertexShaderBlending::modelviewprojection_matrix;
int VertexShaderBlending::triIdx;

TextureMipMap *VertexShaderBlending::texture = 0;


// this is the vertex shader which is executed for each individual vertex that
// needs to ne processed.
struct VertexShaderBlendingDistPass {

    // this specifies that this shader is only going to use 1 vertex attribute
    // array. There you be used up to Renderer::MAX_ATTRIBUTES arrays.
    static const unsigned attribute_count = 1;

    // this specifies the number of varyings the shader will output. This is
    // for instance used when clipping.
    static const unsigned varying_count = 2;
    static mat4x modelview_matrix;
    static mat4x modelviewprojection_matrix;
    // this static function is called for each vertex to be processed.
    // "in" is an array of void* pointers with the location of the individial
    // vertex attributes. The "out" structure has to be written to.
    static void shade(const GeometryProcessor::VertexInput in, GeometryProcessor::VertexOutput &out)
    {
        // cast the first attribute array to the input vertex type
        const Vertex &v = *static_cast<const Vertex*>(in[0]);

        // x, y, z and w are the components that must be written by the vertex
        // shader. They all have to be specified in 16.16 fixed point format.
        /*  out.x = static_cast<int>((v.x * (1 << 16)));
        out.y = static_cast<int>((v.y * (1 << 16)));
        out.z = static_cast<int>((v.z * (1 << 16)));
        out.w = 1 << 16;
*/
        vec4x tvertex = modelviewprojection_matrix * vec4x(v.x,v.y,v.z, fixed16_t(1));
        // cout << fixedpoint::fix2float<16>(tvertex.x.intValue) << " " <<         fixedpoint::fix2float<16>(tvertex.y.intValue) << " "<<        fixedpoint::fix2float<16>(tvertex.z.intValue)<<endl;

        //  cout << v.id << " "<<v.tx << " " << v.ty<<endl;
        // const vec3x &v = i.vertex;
        //const mat4x &m = modelview_matrix;
        /*  fixed16_t d = -(v.x * m.elem[2][0] + v.y * m.elem[2][1] +
                        v.z * m.elem[2][2] + m.elem[2][3]);

        const fixed16_t start = 2;
        const fixed16_t end = 6;

        if (d < start) d = 1;
        else if (d > end) d = 0;
        else {
            d = X(1) - (d-start)/(end-start);
        }
*/
        out.x = tvertex.x.intValue;
        out.y = tvertex.y.intValue;
        out.z = fixed16_t(1.0).intValue;//tvertex.z.intValue;
        out.w = 1 << 16;//tvertex.w.intValue;
        // bring the texture coordinates into the appropriate range for the rasterizer.
        // this mean it has to be converted to fixed point and premultiplied with the width, height of the
        // texture minus 1. Doing this in the vertex shader saves us from doing this in the fragment shader
        // which makes things faster.
        out.varyings[0] = static_cast<int>(v.tx * (1 << 16));
        out.varyings[1] = static_cast<int>(v.ty * (1 << 16));
    }
    static int triIdx;

    static TextureMipMap *texture;

};
static int doubleTouchCount=0;
mat4x VertexShaderBlendingDistPass::modelview_matrix;
mat4x VertexShaderBlendingDistPass::modelviewprojection_matrix;
typedef std::map<std::pair<int,int>,int > dcm_t;
dcm_t doublecountmap;


// this is the fragment shader
struct FragmentShaderBlendingDistPass : public SpanDrawer32BitColorAndDepthSetDouble<FragmentShaderBlendingDistPass> {
    // varying_count = 3 tells the rasterizer that it only needs to interpolate
    // three varying values (the r, g and b in this context).
    static const unsigned varying_count = 3;
    static const float rmax=0.70710678;

    // we don't need to interpolate z in this example
    static const bool interpolate_z = false;
    static int idx;
    // per triangle callback. This could for instance be used to select the
    // mipmap level of detail. We don't need it but it still needs to be defined
    // for everything to work.
    static void begin_triangle(
            const IRasterizer::Vertex& v1,
            const IRasterizer::Vertex& v2,
            const IRasterizer::Vertex& v3,
            int area2)
    {}

    // the fragment shader is called for each pixel and has read/write access to
    // the destination color and depth buffers.
    static void single_fragment(const IRasterizer::FragmentData &fd, int ix,int iy,unsigned int &color, unsigned int &depth)
    {
        // float x=fix2float<16>(fd.varyings[0]);// >> 16;
        // float y=fix2float<16>(fd.varyings[1]);// >> 16;
        int x=fd.varyings[0] >> 16;
        int y=fd.varyings[1] >> 16;

        // cout << "rig: "<<x*1359 << " " <<y*511;
        unsigned char dist=255; //No value sentinal
        bool valid=false;
        if(x>=0 && y>=0){

            float distx=(x-(texture->w_minus1/2.0))/(float)texture->w_minus1;
            float disty=(y-(texture->h_minus1/2.0))/(float)texture->h_minus1;
            // printf("x: %f y%f\n",x/(double)texture->w_minus1,y/(double)texture->h_minus1);
            float r1 = sqrtf( distx*distx + disty*disty);
            dist=clamp((int)((r1/rmax)*255.0),0,255);
            valid=true;

        }
        unsigned char  oldd[4];

        unsigned int d=color;
        oldd[0]= d & 0xFF;
        oldd[1] = (d >> 8) & 0xFF;
        oldd[2] = (d >> 16) & 0xFF;
        oldd[3]= (d >> 24) & 0xFF;
        if(valid){
            if( oldd[idx]!=255){
                // printf("Sentinal touch two %d %d\n",idx,triIdx);
                if(!doublecountmap.count(std::make_pair<int,int>(ix,iy))){
                    std::set<int> a;
                    doublecountmap[std::make_pair<int,int>(ix,iy)]=triIdx;
                    doubleTouchCount++;
                }
            }else{
                oldd[idx]=dist;
            }
        }
        color= oldd[0] | oldd[1] << 8 | oldd[2] << 16 | oldd[3] << 24;

    }

    // this is called by the span drawing function to get the location of the color buffer
    static void* color_pointer(int x, int y)
    {

        gRect.left=x;
        gRect.top=y;

        if(im_prepare(regRange,&gRect)){
            fprintf(stderr,"Prepare fail range rect\n");
            assert(0);
            exit(-1);
        }

        return IM_REGION_ADDR( regRange, x, y );

    }

    // this is called by the span drawing function to get the location of the depth buffer
    static void* depth_pointer(int x, int y)
    {
        return 0;
    }
    static TextureMipMap *texture;
    static int triIdx;

};
int FragmentShaderBlendingDistPass::triIdx;
// this is the fragment shader
struct FragmentShaderBlendingMain : public SpanDrawer32BitColorAndDepthSetDouble<FragmentShaderBlendingMain> {
    // varying_count = 3 tells the rasterizer that it only needs to interpolate
    // three varying values (the r, g and b in this context).
    static const unsigned varying_count = 3;
    static const int mipmapL[];
    static const float rmax=0.70710678;
    static int idx;
    // we don't need to interpolate z in this example
    static const bool interpolate_z = false;

    // per triangle callback. This could for instance be used to select the
    // mipmap level of detail. We don't need it but it still needs to be defined
    // for everything to work.
    static void begin_triangle(
            const IRasterizer::Vertex& v1,
            const IRasterizer::Vertex& v2,
            const IRasterizer::Vertex& v3,
            int area2)
            //   {printf("T1 %f %f\nT2 %f %f\nT3 %f %f\n",fix2float<16>(v1.x),fix2float<16>(v1.y),fix2float<16>(v2.x),fix2float<16>(v2.y),fix2float<16>(v3.x)
            //  ,fix2float<16>(v3.y));}
    {}

    // the fragment shader is called for each pixel and has read/write access to
    // the destination color and depth buffers.
    // static void single_fragment(const IRasterizer::FragmentData &fd, unsigned int &color, unsigned int &depth)
    static void single_fragment(const IRasterizer::FragmentData &fd, int ix,int iy,unsigned int &color, unsigned int &depth)

    {
        if(doublecountmap.count(std::make_pair<int,int>(ix,iy)) && doublecountmap[std::make_pair<int,int>(ix,iy)] != triIdx)
            return;
        float Cb[]={0.710000,0.650000,0.070000};
        int x=fd.varyings[0] >> 16;
        int y=fd.varyings[1] >> 16;


        if(x <0 || y <0|| x>(int)texture->w_minus1 || y>(int)texture->h_minus1){
            printf("Fail \n");
            return;
        }
        unsigned char dist_curr=reinterpret_cast<unsigned char *>(&depth)[idx];
        //Need to handle invalid dist
        float local_r=(dist_curr/255.0)*rmax;

        osg::Vec3d outComp[3];
        for(int j=0; j < 3; j++){
            float W=exp(-local_r*10.0*16.0*Cb[j]);
            // sample the texture and write the color information
            if(j == 0){
                outComp[j]=((texture->sample_nearest(fd.varyings[0], fd.varyings[1],mipmapL[0])-texture->sample_nearest(fd.varyings[0], fd.varyings[1],mipmapL[1]))*W);
            }else if (j==1){
                outComp[j]=((texture->sample_nearest(fd.varyings[0], fd.varyings[1],mipmapL[1])-texture->sample_nearest(fd.varyings[0], fd.varyings[1],mipmapL[2]))*W);
            }else if(j==2){
                outComp[j]=(texture->sample_nearest(fd.varyings[0], fd.varyings[1],mipmapL[2])*W);
            }
            // always write the color;
        }
        osg::Vec3 WSum(0.0,0.0,0.0);
        osg::Vec4 r4(0.0,0.0,0.0,0.0);
        for(int j=0; j < 3; j++){
            for(int i=0;i<4; i++){
                unsigned int d=depth;
                unsigned char oldd[4];

                oldd[0]= d & 0xFF;
                oldd[1] = (d >> 8) & 0xFF;
                oldd[2] = (d >> 16) & 0xFF;
                oldd[3]= (d >> 24) & 0xFF;
                unsigned char dist=oldd[i];
                if(dist ==255)
                    continue;
                //Need to handle invalid dist
                float local_r=(dist/255.0)*rmax;
                r4[i]=local_r;
                float W=exp(-local_r*10.0*16.0*Cb[j]);
                WSum[j]+=W;
            }
        }
        outComp[0]=outComp[0]/WSum[0];
        outComp[1]=outComp[1]/WSum[1];
        outComp[2]=outComp[2]/WSum[2];

        osg::Vec3 outP = outComp[0]+outComp[1]+outComp[2];

        int r,g,b;
        unsigned int c=color;
        int oldblue,oldgreen,oldred;
        if(USE_BGR){
            oldblue = c & 0xFF;
            oldred = (c >> 16) & 0xFF;
        }else{
            oldred = c & 0xFF;
            oldblue = (c >> 16) & 0xFF;
        }
        oldgreen = (c >> 8) & 0xFF;

        r=(int)(outP.x()*255.0);
        g=(int)(outP.y()*255.0);
        b=(int)(outP.z()*255.0);
        if(USE_BGR)
            c= clamp(b+oldblue,0,255) | clamp(g+oldgreen,0,255) << 8 | clamp(r+oldred,0,255) << 16 | 255 << 24;
        else
            c=  clamp(r+oldred,0,255) | clamp(g+oldgreen,0,255) << 8 | clamp(b+oldblue,0,255) << 16 | 255 << 24;

        color = c;


    }

    // this is called by the span drawing function to get the location of the color buffer
    static void* color_pointer(int x, int y)
    {
        gRect.left=x;
        gRect.top=y;

        if(im_prepare(regOutput,&gRect)){
            fprintf(stderr,"Prepare fail color\n");
            assert(0);
            exit(-1);
        }

        return IM_REGION_ADDR( regOutput, x, y );
    }

    // this is called by the span drawing function to get the location of the depth buffer
    static void* depth_pointer(int x, int y)
    {
        gRect.left=x;
        gRect.top=y;

        if(im_prepare(regRange,&gRect)){
            fprintf(stderr,"Prepare fail range blend pass\n");
            assert(0);
            exit(-1);
        }

        return IM_REGION_ADDR( regRange, x, y );
    }

    static TextureMipMap *texture;
    static int triIdx;

};
const int FragmentShaderBlendingMain::mipmapL[]= {0,2,4};
int FragmentShaderBlendingMain::idx;

void readFile(string fname,map<int,imgData> &imageList){

    std::ifstream m_fin(fname.c_str());
    if(!osgDB::fileExists(fname.c_str())||!m_fin.good() ){
        fprintf(stderr,"Can't load %s\n",fname.c_str());
        exit(-1);
    }
    while(!m_fin.eof()){
        imgData cam;
        double low[3], high[3];
        if(m_fin >> cam.id >> cam.filename >> low[0] >> low[1] >> low[2] >> high[0] >> high[1] >> high[2]
           >> cam.m(0,0) >>cam.m(0,1)>>cam.m(0,2) >>cam.m(0,3)
           >> cam.m(1,0) >>cam.m(1,1)>>cam.m(1,2) >>cam.m(1,3)
           >> cam.m(2,0) >>cam.m(2,1)>>cam.m(2,2) >>cam.m(2,3)
           >> cam.m(3,0) >>cam.m(3,1)>>cam.m(3,2) >>cam.m(3,3)){
            cam.bbox.expandBy(low[0],low[1],low[2]);
            cam.bbox.expandBy(high[0],high[1],high[2]);
            imageList[cam.id]=cam;
        }
    }
}

Texture *FragmentShader::texture = 0;
TextureMipMap *FragmentShaderBlendingMain::texture = 0;
TextureMipMap *FragmentShaderBlendingDistPass::texture = 0;

int FragmentShaderBlendingDistPass::idx =-1;
int FragmentShaderBlendingMain::triIdx;
static Vertex tmp_vertices[3];

inline bool process_tri(plyA::tri_t &tri,osg::Vec3Array *verts, std::vector<osg::ref_ptr<osg::Vec3Array> >   &texCoord, bool blending)
{
    int pos=tri.pos;


    VertexShaderBlending::triIdx=tri.tri_idx;
    FragmentShaderBlendingMain::triIdx=tri.tri_idx;
    FragmentShaderBlendingDistPass::idx =pos;
    FragmentShaderBlendingDistPass::triIdx=tri.tri_idx;
    if(blending)
        FragmentShaderBlendingMain::idx=pos;
    osg::Vec3 v1=verts->at(tri.idx[0]);
    osg::Vec3 v2=verts->at(tri.idx[1]);
    osg::Vec3 v3=verts->at(tri.idx[2]);
    osg::Vec3 tc1,tc2,tc3;

    tc1=texCoord[pos]->at(tri.idx[0]);
    tc2=texCoord[pos]->at(tri.idx[1]);;

    tc3=texCoord[pos]->at(tri.idx[2]);;



    if(tc1.x() <0 || tc1.y() <0 || tc2.x() <0 || tc2.y() <0 || tc3.x() <0 || tc3.y() <0){
        //printf("Fail %f %f %f %f %f %f\n",tc1.x(),tc1.y(),tc2.x(),tc2.y(),tc3.x(),tc3.y());
        return false;
    }
    tmp_vertices[0].x=v1.x();
    tmp_vertices[0].y=v1.y();
    tmp_vertices[0].tx=tc1.x();
    tmp_vertices[0].ty=1.0-tc1.y();
    tmp_vertices[0].id=tc1.z();

    tmp_vertices[1].x=v2.x();
    tmp_vertices[1].y=v2.y();
    tmp_vertices[1].tx=tc2.x();
    tmp_vertices[1].ty=1.0-tc2.y();
    tmp_vertices[1].id=tc2.z();

    tmp_vertices[2].x=v3.x();
    tmp_vertices[2].y=v3.y();
    tmp_vertices[2].tx=tc3.x();
    tmp_vertices[2].ty=1.0-tc3.y();
    tmp_vertices[2].id=tc3.z();
    return true;
}
int main(int ac, char *av[]) {

    osg::Timer_t start;
    osg::ref_ptr<osg::Node> model;//= osgDB::readNodeFile(av[1]);
    plyA::VertexData vertexData;
    osg::ArgumentParser arguments(&ac,av);
    int sizeX,sizeY; sizeX=sizeY=1024;
    outputImage=NULL;
    rangeImage=NULL;
    arguments.read("--size",sizeX,sizeY);
    bool useDisk=arguments.read("--outofcore");
    string imageName,depthName;
    unsigned int _tileRows;
    unsigned int _tileColumns;
    int row;
    int col;
    char tmp[1024];
    if(!arguments.read("--image",row,col,_tileRows,_tileColumns)){
        fprintf(stderr,"Fail to get image params\n");
        return -1;
    }
    sprintf(tmp,"mesh-diced/image_r%04d_c%04d_rs%04d_cs%04d",row,col,_tileRows,_tileColumns);

    imageName=string(tmp)+".v";
    depthName=string(tmp)+"-tmp_dist.v";
    double lat=0,lon=0;
    if(!arguments.read("-lat",lat)|| !arguments.read("-lon",lon)){
        fprintf(stderr,"Can't get lat long\n");
        return -1;
    }
    bool blending = arguments.read("--blend");
    if(blending)
        printf("Blending Enabled\n");
    outputImage=im_open("tmp","p");
    if(im_black(outputImage,sizeX,sizeY,4)){
        fprintf(stderr,"Can't create color image\n");
        return -1;
    }
    gRect.width=128;
    gRect.height=1;

    if(useDisk){
        IMAGE *diskFile=im_open(imageName.c_str(),"w");
        im_copy(outputImage,diskFile);
        im_close(outputImage);
        outputImage=diskFile;
    }

    if(blending){
        rangeImage=im_open("tmp_range","p");
        if(im_white(rangeImage,sizeX,sizeY,4)){
            fprintf(stderr,"Can't create range image\n");
            return -1;
        }
        if(useDisk){

            IMAGE *diskFileRange=im_open(depthName.c_str(),"w");
            im_copy(rangeImage,diskFileRange);
            im_close(rangeImage);
            rangeImage=diskFileRange;
        }
    }





    double vm, rss;
    process_mem_usage(vm, rss);
    cout << "1st VM: " << get_size_string(vm) << "; RSS: " << get_size_string(rss) << endl;

    if(  im_rwcheck(outputImage) ){
        fprintf(stderr,"can't open\n");
        return -1;
    }

    regOutput = im_region_create(outputImage);
    if(regOutput == NULL){
        fprintf(stderr,"Can't init color image region\n");
        exit(-1);
    }

    if(blending){
        if(  im_rwcheck(rangeImage) ){
            fprintf(stderr,"can't open\n");
            return -1;
        }


        regRange  = im_region_create(rangeImage);
        if(regRange == NULL){
            fprintf(stderr,"Can't init range image region\n");
            exit(-1);
        }
    }
    process_mem_usage(vm, rss);
    cout << "snd VM: " << get_size_string(vm) << "; RSS: " << get_size_string(rss) << endl;

    map<int,imgData> imageList;
    model= vertexData.readPlyFile(av[1]);
    readFile(av[2],imageList);

    if(model.valid()){
        osg::Geode *geode= dynamic_cast<osg::Geode*>(model.get());
        if(!geode)
            geode=model->asGroup()->getChild(0)->asGeode();
        else{
            //printf("fail\n");
        }
        if(geode && geode->getNumDrawables()){
            // printf("valid\n");
        }else{
            printf("Fail2\n");
            exit(-1);

        }
        float rx, ry, rz;
        osg::Matrix inverseM=osg::Matrix::identity();

        if(arguments.read("--invrot",rx,ry,rz)){
            inverseM =osg::Matrix::rotate(
                    osg::DegreesToRadians( rx ), osg::Vec3( 1, 0, 0 ),
                    osg::DegreesToRadians( ry ), osg::Vec3( 0, 1, 0 ),
                    osg::DegreesToRadians( rz ), osg::Vec3( 0, 0, 1 ) );
        }
        osg::Matrix rotM=osg::Matrix::inverse(inverseM);

        osg::Vec3Array *verts=vertexData._vertices;

        osg::BoundingBox totalbb;

        for(int i=0; i <(int)verts->size(); i++){
            verts->at(i)=verts->at(i)*rotM;
            totalbb.expandBy(verts->at(i));
        }

        /*osg::Vec3d eye(totalbb.center()+osg::Vec3(0,0,3.5*totalbb.radius()));
        double xrange=totalbb.xMax()-totalbb.xMin();
        double yrange=totalbb.yMax()-totalbb.yMin();
        double largerSide=std::max(xrange,yrange);*/
        /*        osg::Matrixd matrix;
        matrix.makeTranslate( eye );

        osg::Matrixd view=osg::Matrix::inverse(matrix);
*/
        osg::Matrixd view,proj;
        mat4x viewA,projA;

        std::fstream _file("view.mat",std::ios::binary|std::ios::in);
        for(int i=0; i<4; i++)
            for(int j=0; j<4; j++){
            _file.read(reinterpret_cast<char*>(&(view(i,j))),sizeof(double));
            viewA.elem[j][i]=fixed16_t(view(i,j));
        }
        for(int i=0; i<4; i++)
            for(int j=0; j<4; j++){
            _file.read(reinterpret_cast<char*>(&(proj(i,j))),sizeof(double));
            projA.elem[j][i]=fixed16_t(proj(i,j));

        }
        _file.close();

        //        printf("AAAA %d %d %d %d\n",row,col,_tileRows,_tileColumns);

        osg::Matrix offsetMatrix=osg::Matrix::scale((double)_tileColumns,(double) _tileRows, 1.0)*osg::Matrix::translate((double)_tileColumns-1-2*col, (double)_tileRows-1-2*row, 0.0);


        mat4x offsetMatrixA=translation_matrix<fixed16_t>((double)_tileColumns-1-2*col, (double)_tileRows-1-2*row, 0.0)*scaling_matrix<fixed16_t>((double)_tileColumns,(double) _tileRows, 1.0);

        //mat4x viewA2=fast_inverse<fixed16_t>(translation_matrix<fixed16_t>(eye.x(),eye.y(),eye.z()));
        //osg::Matrixd proj= osg::Matrixd::ortho2D(-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0));

        //mat4x projA2=ortho_matrix<fixed16_t>(-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0),totalbb.zMin(),totalbb.zMax());
        osg::Matrixd viewprojmat=view*(proj*offsetMatrix);
        mat4x viewprojmatA=(offsetMatrixA*projA)*viewA;
        /*  cout << view <<endl;
        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                cout << fix2float<16>(viewprojmatA.elem[i][j].intValue) << " ";
            }
            cout <<endl;
        }
        cout <<endl;

        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                cout << fix2float<16>(viewA.elem[i][j].intValue) << " ";
            }
            cout <<endl;
        }
        cout <<endl;

        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                cout << fix2float<16>(viewA2.elem[i][j].intValue) << " ";
            }
            cout <<endl;
        }
        cout <<endl;*/
        if(!blending)
            VertexShader::modelviewprojection_matrix=viewprojmatA;
        else{
            VertexShaderBlendingDistPass::modelviewprojection_matrix=viewprojmatA;
            VertexShaderBlending::modelviewprojection_matrix=viewprojmatA;
        }

        start = osg::Timer::instance()->tick();



        // the indices we need for rendering
        unsigned indices[] = {0, 1, 2};

        // create a rasterizer class that will be used to rasterize primitives
        RasterizerSubdivAffine r;
        // create a geometry processor class used to feed vertex data.
        GeometryProcessor g(&r);
        // it is necessary to set the viewport
        g.viewport(0, 0, outputImage->Xsize, outputImage->Ysize);
        // set the cull mode (CW is already the default mode)
        g.cull_mode(GeometryProcessor::CULL_CW);

        // it is also necessary to set the clipping rectangle
        r.clip_rect(0, 0, outputImage->Xsize, outputImage->Ysize);

        // set the vertex and fragment shaders


        // specify where out data lies in memory
        g.vertex_attrib_pointer(0, sizeof(Vertex), tmp_vertices);

        if(!vertexData._texCoord.size()){
            fprintf(stderr,"No tex coords\n");
            exit(-1);
        }


        if(!blending){
            g.vertex_shader<VertexShader>();
            r.fragment_shader<FragmentShader>();
        }
        else{
            g.vertex_shader<VertexShaderBlending>();
            r.fragment_shader<FragmentShaderBlendingDistPass>();
            TextureMipMap *sizeI=NULL;
            map<int,vector<plyA::tri_t> >::iterator itr=vertexData._img2tri.begin();
            while((!sizeI || sizeI->surface==NULL) && itr!=vertexData._img2tri.end()){
                string tmp=string(string(av[3])+string("/")+imageList[itr->first].filename);

                sizeI=new TextureMipMap(tmp);
                itr++;
            }
            VertexShaderBlending::texture =sizeI;
            FragmentShaderBlendingDistPass::texture =sizeI;

            for( map<int,vector<plyA::tri_t> >::iterator itr=vertexData._img2tri.begin(); itr!=vertexData._img2tri.end(); itr++){
                for(int t=0; t< (int)itr->second.size(); t++){
                    if(process_tri(itr->second[t],verts,vertexData._texCoord,blending))
                        g.draw_triangles(3, indices);
                }
            }
            printf("Double touch count %d\n",doubleTouchCount);

            g.vertex_shader<VertexShaderBlending>();
            r.fragment_shader<FragmentShaderBlendingMain>();
            if(sizeI){
                delete sizeI;
            }
        }
        for( map<int,vector<plyA::tri_t> >::iterator itr=vertexData._img2tri.begin(); itr!=vertexData._img2tri.end(); itr++){
            string tmp=string(string(av[3])+string("/")+imageList[itr->first].filename);

            Texture *texture=NULL;
            TextureMipMap *textureMipMap=NULL;

            if(!blending){
                texture = new Texture(tmp);
                FragmentShader::texture = texture;
                VertexShader::texture = texture;
                if(!texture->surface)
                    continue;
            }else{
                textureMipMap = new TextureMipMap(tmp,itr->first);
                FragmentShaderBlendingMain::texture = textureMipMap;
                VertexShaderBlending::texture =textureMipMap;
                if(!textureMipMap->surface)
                    continue;

            }
            for(int t=0; t< (int)itr->second.size(); t++){
                if(!blending && itr->second[t].pos !=0 )
                    continue;
                if(process_tri(itr->second[t],verts,vertexData._texCoord,blending))
                    g.draw_triangles(3, indices);
            }
            if(!blending){
                delete texture;

            }
            else{
                delete textureMipMap;
            }

        }

        double elapsed=osg::Timer::instance()->delta_s(start,osg::Timer::instance()->tick());
        std::cout << "\n"<<format_elapsed(elapsed) << std::endl;
        double vm, rss;
        process_mem_usage(vm, rss);
        cout << "VM: " << get_size_string(vm) << "; RSS: " << get_size_string(rss) << endl;
        im_region_free(regOutput);

        if(blending){
            im_region_free(regRange);
            im_close(rangeImage);

            if(useDisk){
                if( remove( depthName.c_str() ) != 0 )
                    perror( "Error deleting file" );
                else
                    puts( "File successfully deleted" );
            }
        }
        process_mem_usage(vm, rss);
        cout << "VM: " << get_size_string(vm) << "; RSS: " << get_size_string(rss) << endl;

        start=osg::Timer::instance()->tick();

        if( im_vips2tiff(outputImage,(osgDB::getNameLessExtension(imageName)+"-tmp.tif:packbits,tile:256x256").c_str())){
            fprintf(stderr,"Failed to write\n");
            cerr << im_error_buffer()<<endl;
            exit(-1);
        }
        elapsed=osg::Timer::instance()->delta_s(start,osg::Timer::instance()->tick());
        std::cout << "\n"<<format_elapsed(elapsed) << std::endl;
        process_mem_usage(vm, rss);
        cout << "VM: " << get_size_string(vm) << "; RSS: " << get_size_string(rss) << endl;
        im_close(outputImage);

        if(applyGeoTags(osgDB::getNameLessExtension(imageName)+".tif",osg::Vec2(lat,lon),view,(proj*offsetMatrix),sizeX,sizeY)){
            if( remove((osgDB::getNameLessExtension(imageName)+"-tmp.tif").c_str() ) != 0 )
                perror( "Error deleting file" );
            else
                puts( "File successfully deleted" );
        }


        if(useDisk){
            if( remove( imageName.c_str() ) != 0 )
                perror( "Error deleting file" );
            else
                puts( "File successfully deleted" );
        }

    }

    return 0;
}
