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
#include "../MemUtils.h"
#include "render_utils.h"
#include <iterator>
#include <numeric>

// the software renderer stuff is located in the namespace "swr" so include
// that here
using namespace swr;
using namespace std;
static bool writeOut=false;
#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>
inline bool isZero(float f) { return fabs(f) < 1E-6; }
inline bool isZero(double f) { return fabs(f) < 1E-10; }
inline bool eq(double a, double b) { return isZero(fabsf(a - b));}
inline bool eq(float a, float b) { return isZero(fabsf(a - b));}
 osg::Vec3 jetColorMap(const float& val) {
        assert(val <= 1.0f && val >= 0.0f );

        //truncated triangles where sides have slope 4
        osg::Vec3 jet;

        jet[0] = min(4.0f * val - 1.5f,-4.0f * val + 4.5f) ;
        jet[1] = min(4.0f * val - 0.5f,-4.0f * val + 3.5f) ;
        jet[2] = min(4.0f * val + 0.5f,-4.0f * val + 2.5f) ;


        jet[0] = clamp(jet[0], 0.0f, 1.0f);
        jet[1] = clamp(jet[1], 0.0f, 1.0f);
        jet[2] = clamp(jet[2], 0.0f, 1.0f);

        return jet;
}
static void rgb2hsv(const osg::Vec3 &rgb, float *h, float *s, float *v)
{
    float r, g, b;
    float max, min;

    r = rgb[0];
    g = rgb[1];
    b = rgb[2];

    max = std::max(r, std::max(g, b));
    min = std::min(r, std::min(g, b));

    if (eq(max, min)){
        *h = 0;
    }else if (eq(max, r) && g >= b){
        *h = 60.f / 360.f * ((g - b) / (max - min));
    }else if (eq(max, r)){ // g < b is implicit
        *h = (60.f / 360.f * ((g - b) / (max - min))) + 1;
    }else if (eq(max, g)){
        *h = (60.f / 360.f * ((b - r) / (max - min))) + (120.f / 360.f);
    }else{ // eq(max, b)
        *h = (60.f / 360.f * ((r - g) / (max - min))) + (240.f / 360.f);
    }

    if (isZero(max)){
        *s = 0.f;
    }else{
        *s = 1 - (min / max);
    }

    *v = max;
}
osg::Vec3d rgb2xyz( osg::Vec3 c ) {
    /*osg::Vec3d tmp;
    tmp.x() = ( c.x() > 0.04045 ) ? pow( ( c.x() + 0.055 ) / 1.055, 2.4 ) : c.x() / 12.92;
    tmp.y() = ( c.y() > 0.04045 ) ? pow( ( c.y() + 0.055 ) / 1.055, 2.4 ) : c.y() / 12.92,
    tmp.z() = ( c.z() > 0.04045 ) ? pow( ( c.z() + 0.055 ) / 1.055, 2.4 ) : c.z() / 12.92;*/

    // The following performs an inverse "gamma correction" specified by the sRGB
    // color space.  sRGB is defined by a cononical definition of a display
    // monitor and has been standardized by the International Electrotechnical
    // Commission (IEC 61966-2-1).  However, it is a non-linear color space, which
    // means that it will invalidate the color computations done by OpenGL.
    // Furthermore, most OS or display drivers will do some gamma correction on
    // there own, so displaying these colors directly usually results in overly
    // bright images.  Thus, the non-linear RGB values are inappropriate for VTK,
    // so the code is commented out.  If someone needs sRGB values in the future,
    // there should be a separate set of color conversion functions for that.
    //   if ( r > 0.04045 ) r = pow(( r + 0.055 ) / 1.055, 2.4);
    //   else               r = r / 12.92;
    //   if ( g > 0.04045 ) g = pow(( g + 0.055 ) / 1.055, 2.4);
    //   else               g = g / 12.92;
    //   if ( b > 0.04045 ) b = pow(( b + 0.055 ) / 1.055, 2.4);
    //   else               b = b / 12.92;
    osg::Vec3 xyz;
    //Observer. = 2 deg, Illuminant = D65
    xyz[0] = c[0] * 0.4124 + c[1] * 0.3576 + c[2] * 0.1805;
    xyz[1] = c[0] * 0.2126 + c[1] * 0.7152 + c[2] * 0.0722;
    xyz[2] = c[0] * 0.0193 + c[1] * 0.1192 + c[2] * 0.9505;
    return xyz;
}

osg::Vec3 xyz2lab( osg::Vec3 c ) {
    const double ref_X = 0.9505;
    const double ref_Y = 1.000;
    const double ref_Z = 1.089;
    double var_X = c[0] / ref_X;  //ref_X = 0.9505  Observer= 2 deg, Illuminant= D65
    double var_Y = c[1] / ref_Y;  //ref_Y = 1.000
    double var_Z = c[2] / ref_Z;  //ref_Z = 1.089

    if ( var_X > 0.008856 ) var_X = pow(var_X, 1.0/3.0);
    else                    var_X = ( 7.787 * var_X ) + ( 16.0 / 116.0 );
    if ( var_Y > 0.008856 ) var_Y = pow(var_Y, 1.0/3.0);
    else                    var_Y = ( 7.787 * var_Y ) + ( 16.0 / 116.0 );
    if ( var_Z > 0.008856 ) var_Z = pow(var_Z, 1.0/3.0);
    else                    var_Z = ( 7.787 * var_Z ) + ( 16.0 / 116.0 );
    osg::Vec3 Lab;
    Lab[0] = ( 116 * var_Y ) - 16;
    Lab[1] = 500 * ( var_X - var_Y );
    Lab[2] = 200 * ( var_Y - var_Z );
    return Lab;
}

osg::Vec3 rgb2lab(osg::Vec3 c) {
    osg::Vec3 lab = xyz2lab( rgb2xyz( c ) );
    return osg::Vec3( lab.x() / 100.0, 0.5 + 0.5 * ( lab.y() / 127.0 ), 0.5 + 0.5 * ( lab.z() / 127.0 ));
}


struct InputVertex {
    vec3x vertex;
};

static IplImage *outputImage;
FILE *rfp,*gfp,*bfp;

//static IplImage *varImage;

// this is the vertex shader which is executed for each individual vertex that
// needs to ne processed.
struct VertexShaderBlending {

    // this specifies that this shader is only going to use 1 vertex attribute
    // array. There you be used up to Renderer::MAX_ATTRIBUTES arrays.
    static const unsigned attribute_count = 1;

    // this specifies the number of varyings the shader will output. This is
    // for instance used when clipping.
    static const unsigned varying_count = 12;
    static mat4x modelview_matrix;
    static mat4x modelviewprojection_matrix;
    // this static function is called for each vertex to be processed.
    // "in" is an array of void* pointers with the location of the individial
    // vertex attributes. The "out" structure has to be written to.
    static void shade(const GeometryProcessor::VertexInput in, GeometryProcessor::VertexOutput &out)
    {
        // cast the first attribute array to the input vertex type
        const VertexBlend &v = *static_cast<const VertexBlend*>(in[0]);

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
        for(int i=0; i <4; i++){
            out.varyings[i*2+0] = static_cast<int>(v.tx[i] * (1 << 16));
            out.varyings[i*2+1] = static_cast<int>(v.ty[i] * (1 << 16));
        }
        for(int i=0; i <4; i++){
            out.varyings[i+9]= static_cast<int>(v.id[i]/1.0 * (1 << 16));
            // printf("B:%f \n",fix2float<16>(out.varyings[i+9]));
        }

    }

    static TextureMipMap *texture;

};
mat4x VertexShaderBlending::modelview_matrix;
mat4x VertexShaderBlending::modelviewprojection_matrix;

TextureMipMap *VertexShaderBlending::texture = 0;

double standard_dev( std::vector<double> &v ) {
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();

    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / v.size() - mean * mean);
    return stdev;
}


// this is the fragment shader
struct FragmentShaderBlendingMain : public SpanDrawer32BitColorAndDepth<FragmentShaderBlendingMain> {
    // varying_count = 3 tells the rasterizer that it only needs to interpolate
    // three varying values (the r, g and b in this context).
    static const unsigned varying_count = 12;
    static const int mipmapL[];
    static const float rmax=0.70710678;
    static int idx;
    // we don't need to interpolate z in this example
    static const bool interpolate_z = false;
    static map<int,TextureMipMap*> texturepool;

    // per triangle callback. This could for instance be used to select the
    // mipmap level of detail. We don't need it but it still needs to be defined
    // for everything to work.
    static void begin_triangle(
        const IRasterizer::Vertex& v1,
        const IRasterizer::Vertex& v2,
        const IRasterizer::Vertex& v3,
        int area2)
    /*{printf("T1 %f %f\nT2 %f %f\nT3 %f %f\n",fix2float<16>(v1.x),fix2float<16>(v1.y),fix2float<16>(v2.x),fix2float<16>(v2.y),fix2float<16>(v3.x)
    ,fix2float<16>(v3.y));}*/
    {}

    // the fragment shader is called for each pixel and has read/write access to
    // the destination color and depth buffers.
    static void single_fragment(const IRasterizer::FragmentData &fd, unsigned int &color, unsigned int &depth)
    {

        osg::Vec4f x;// >> 16;
        osg::Vec4f y;
        int id[4];
        for(int i=0;i <4; i++){
            x[i]=fix2float<16>(fd.varyings[i*2+0]);
            y[i]=fix2float<16>(fd.varyings[i*2+1]);// >> 16;
            x[i]=clamp(x[i],0.0,1.0);
            y[i]=clamp(y[i],0.0,1.0);
        }
        for(int i=0;i <4; i++){
            id[i]=(int)fix2float<16>(fd.varyings[9+i])*1.0;

        }



        std::vector<double> outComp[3];
        osg::Vec3 WSum(0.0,0.0,0.0);

        for(int i=0;i<4; i++){

            if(id[i]<=0 || !texturepool.count(id[i]))
                continue;
            TextureMipMap *texturec=texturepool[id[i]];
            if(!texturec){
                printf("Can't load %d",id[i]);
                continue;
            }

            //cout<<"id "<< id[i] << "\n";
            if(x[i]<0 || y[i]<0)
                continue;


            //  printf("%d W:%f %f\n",W,r1[i],dist);
            int cx,cy;
            cx=static_cast<int>((x[i]*texturec->w_minus1) * (1 << 16));
            cy=static_cast<int>((y[i]*texturec->h_minus1) * (1 << 16));
            int mipmapL=6;
            osg::Vec3 val=texturec->sample_nearest(cx,cy,mipmapL);
            //float h,s,v;
            // rgb2hsv(val,&h,&s,&v);
            osg::Vec3 lab=rgb2lab(val);
            //cout << "RGB " << val<<endl;
           // cout <<"LAB " << lab<<endl;
            for(int j=0; j <3; j++)
                outComp[j].push_back(lab[0]);



        }

        osg::Vec3d std_dev_pixels;
        osg::Vec3 outP;
        FILE *f_arr[]={rfp,gfp,bfp};

        for(int j=0; j <3; j++){
            std_dev_pixels[j]=standard_dev(outComp[j]);
            if(writeOut)
                fwrite(&(std_dev_pixels[j]),sizeof(double),1,f_arr[j]);
            //scale from 0.5 max std to full 0 to 1.0 range
         //   outP[j]=std_dev_pixels[j];
        }
       // printf("%f\n",std_dev_pixels[0]);
        std_dev_pixels[0]=clamp((std_dev_pixels[0]/0.1f),0.0f,1.0f);
        outP=jetColorMap(std_dev_pixels[0]);
        //cout <<std_dev_pixels<<endl;
        //cout << "outP "<<outP<<endl;
        int cx,cy;
        TextureMipMap *texturec= texturepool[id[0]];
        if(texturec)
        {
            cx=static_cast<int>((x[0]*texturec->w_minus1) * (1 << 16));
            cy=static_cast<int>((y[0]*texturec->h_minus1) * (1 << 16));
            //  outP=outComp[2];
            //      outP=texturec->sample_nearest(cx,cy,6);
        }
        //cout << "FFF" <<outP<<endl;
        unsigned char r,g,b;
        unsigned int c=0;
        int oldblue = c & 0xFF;
        int oldgreen = (c >> 8) & 0xFF;
        int oldred = (c >> 16) & 0xFF;
        //outP=texture->sample_nearest(fd.varyings[0], fd.varyings[1],2)-texture->sample_nearest(fd.varyings[0], fd.varyings[1],4);
        r=clamp((int)(outP.x()*255.0),0,255);
        g=clamp((int)(outP.y()*255.0),0,255);
        b=clamp((int)(outP.z()*255.0),0,255);
        //  cout << x << " "<< y<<endl;
        //   cout << (int)r << " " << (int)g << " " <<(int)b << endl;
        //        cout << oldred << " " << oldgreen << " " <<oldblue << endl;

        c= (b+oldblue) | (g+oldgreen) << 8 | (r+oldred) << 16 | 255 << 24;
        // c= 0xff | 0xff << 8 | 0xff << 16 | 255 << 24;

        // if(idx ==0)
        color = c;

    }

    // this is called by the span drawing function to get the location of the color buffer
    static void* color_pointer(int x, int y)
    {
        IplImage *screen = outputImage;///SDL_GetVideoSurface();
        //return static_cast<unsigned short*>(screen->pixels) + x + y * screen->w;
        return &(CV_IMAGE_ELEM(screen,unsigned char,y,x*screen->nChannels));
    }

    // this is called by the span drawing function to get the location of the depth buffer
    static void* depth_pointer(int x, int y)
    {
        //IplImage *screen = varImage;///SDL_GetVideoSurface();
        //return static_cast<unsigned short*>(screen->pixels) + x + y * screen->w;
        return NULL;//&(CV_IMAGE_ELEM(screen,unsigned char,y,x*screen->nChannels));
    }
    // static TextureMipMap *texture;
};
const int FragmentShaderBlendingMain::mipmapL[]= {0,2,4};
int FragmentShaderBlendingMain::idx;
//int FragmentShaderBlendingMain::ids[4];

map<int,TextureMipMap*> FragmentShaderBlendingMain::texturepool;

void readFile(string fname,map<int,imgData> &imageList){
    std::ifstream m_fin(fname.c_str());

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

//TextureMipMap *FragmentShaderBlendingMain::texture = 0;

int main(int ac, char *av[]) {

    osg::Timer_t start;
    osg::ref_ptr<osg::Node> model;//= osgDB::readNodeFile(av[1]);
    ply::VertexData vertexData;
    osg::ArgumentParser arguments(&ac,av);
    int sizeX,sizeY; sizeX=sizeY=1024;

    arguments.read("--size",sizeX,sizeY);

    unsigned int _tileRows;
    unsigned int _tileColumns;
    int row;
    int col;
    char tmp[1024];
    if(!arguments.read("--image",row,col,_tileRows,_tileColumns)){
        fprintf(stderr,"Fail to get image params\n");
        return -1;
    }


    sprintf(tmp,"mesh-diced/var_r%04d_c%04d_rs%04d_cs%04d",row,col,_tileRows,_tileColumns);

    string imageName=string(tmp)+".v";
    std::string matfile;
    double lat=0,lon=0;
    if(!arguments.read("-lat",lat)|| !arguments.read("-lon",lon)){
        fprintf(stderr,"Can't get lat long\n");
        return -1;
    }
    if(!arguments.read("--mat",matfile)){
        fprintf(stderr,"Fail mat file\n");
        return -1;
    }
    mat4x  viewProjReadA ;
    osg::Matrixd viewProjRead;
    std::fstream _file(matfile.c_str(),std::ios::binary|std::ios::in);
    if(!_file.good()){
        fprintf(stderr,"Can't load %s\n",matfile.c_str());
        exit(-1);
    }
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++){
            _file.read(reinterpret_cast<char*>(&(viewProjRead(i,j))),sizeof(double));
            viewProjReadA.elem[j][i]=fixed16_t(viewProjRead(i,j));
        }

    sprintf(tmp,"mesh-diced/image_r%04d_c%04d_rs%04d_cs%04d",row,col,_tileRows,_tileColumns);
    outputImage=cvCreateImage(cvSize(sizeX,sizeY),IPL_DEPTH_8U,4);
    //varImage=cvCreateImage(cvSize(sizeX,sizeY),IPL_DEPTH_32F,1);
    // cvZero(varImage);
    if(writeOut){
        rfp=fopen("rstd.raw","w");
        gfp=fopen("gstd.raw","w");
        bfp=fopen("bstd.raw","w");
    }

    cvZero(outputImage);
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


        osg::BoundingBox totalbb;

        osg::Vec3Array *verts=vertexData._vertices;
        for(int i=0; i <(int)verts->size(); i++){
            verts->at(i)=verts->at(i)*rotM;
            totalbb.expandBy(verts->at(i));
        }

        /* osg::ComputeBoundsVisitor cbbv(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
       model->accept(cbbv);
       osg::BoundingBox totalbb = cbbv.getBoundingBox();*/

        osg::Vec3d eye(totalbb.center()+osg::Vec3(0,0,3.5*totalbb.radius()));
        double xrange=totalbb.xMax()-totalbb.xMin();
        double yrange=totalbb.yMax()-totalbb.yMin();
        double largerSide=std::max(xrange,yrange);
        osg::Matrixd matrix;
        matrix.makeTranslate( eye );

        osg::Matrixd view=osg::Matrix::inverse(matrix);
        //cout << view << endl;

        mat4x viewA=fast_inverse<fixed16_t>(translation_matrix<fixed16_t>(eye.x(),eye.y(),eye.z()));
        osg::Matrixd proj= osg::Matrixd::ortho2D(-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0));

        mat4x projA=ortho_matrix<fixed16_t>(-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0),totalbb.zMin(),totalbb.zMax());
        osg::Matrixd viewprojmat=view*proj;
        //cout <<viewprojmat <<endl;

        //if(!blending)
        //  VertexShader::modelviewprojection_matrix=projA*viewA;
        //else{
        //VertexShaderBlendingDistPass::modelviewprojection_matrix=projA*viewA;
        VertexShaderBlending::modelviewprojection_matrix=viewProjReadA;//projA*viewA;
        //}
        /*       for(int k=0; k<4; k++){
      cout << endl;
      for(int j=0;j<4;j++)
      cout << fixedpoint::fix2float<16>(VertexShader::modelviewprojection_matrix.elem[k][j].intValue) << " ";
      }
      cout <<endl;*/
        //exit(-1);
        /* osg::Drawable *drawable = geode->getDrawable(0);
       osg::TriangleIndexFunctor<TriangleIndexVisitor> tif;
       drawable->accept(tif);

    */
        //  printf("Size %d",(int)tif.indices_double_counted.size());
        /* osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
    //geom->setUseDisplayList(false);
    osg::Vec3Array *verts=static_cast<const osg::Vec3Array*>(geom->getVertexArray());
    int origSize=tif.new_list.size();*/
        //  verts->resize(origSize+tif.indices_double_counted.size());
        start = osg::Timer::instance()->tick();

        // initialize SDL without error handling an all
        //  SDL_Init(SDL_INIT_VIDEO);
        //SDL_Surface *screen = SDL_SetVideoMode(1280, 1024, 16, 0);

        // the four vertices of the textured quad together with the texture coordinates

        VertexBlend tmp_vertices[3];
        // load the texture file
        //texture = new Texture(load_surface_r5g5a1b5("texture.png"));
        map<int,TextureMipMap*> pool;
        for(map<int,imgData>::iterator itr= imageList.begin(); itr!= imageList.end(); itr++){
            string tmp=string(string(av[3])+string("/")+itr->second.filename);
            pool[itr->second.id]=new TextureMipMap(tmp.c_str());//load_surface_r5g5a1b5(tmp.c_str());
            if(pool[itr->second.id]->surface == NULL){
                printf("Can't load %s\n",tmp.c_str());
                pool.erase(itr->second.id);
            }
        }
        // texturepool = new TexturePool(surface);
        // make the shaders know which texture to use
        //	VertexShader::texture = texture;
        FragmentShaderBlendingMain::texturepool = pool;

        // the indices we need for rendering
        unsigned indices[] = {0, 1, 2};

        // create a rasterizer class that will be used to rasterize primitives
        swr::RasterizerSubdivAffine r;
        // create a geometry processor class used to feed vertex data.
        GeometryProcessor g(&r);
        // it is necessary to set the viewport
        g.viewport(0, 0, outputImage->width, outputImage->height);
        // set the cull mode (CW is already the default mode)
        g.cull_mode(GeometryProcessor::CULL_CW);

        // it is also necessary to set the clipping rectangle
        r.clip_rect(0, 0, outputImage->width, outputImage->height);

        // set the vertex and fragment shaders


        // specify where out data lies in memory
        g.vertex_attrib_pointer(0, sizeof(VertexBlend), tmp_vertices);


        int last_count=0;
        // draw the triangle
        //  VertexShader::modelviewprojection_matrix=identity4<fixed16_t>();
        /* map<int,vector<ply::tri_t> > img2tri;

       for(long int i=0; i<origSize-2; i+=3){
       ply::tri_t tri;
       tri.idx[0]=tif.new_list[i];
       tri.idx[1]=tif.new_list[i+1];
       tri.idx[2]=tif.new_list[i+2];
       int id1=(int)vertexData._texIds->at(tri.idx[0])[0];
       int id2=(int)vertexData._texIds->at(tri.idx[1])[0];
       int id3=(int)vertexData._texIds->at(tri.idx[2])[0];
       img2tri[id1].push_back(tri);
       }
       printf("%d %d\n",img2tri.size(),vertexData._img2tri.size());
       for( map<int,vector<ply::tri_t> >::iterator itr=vertexData._img2tri.begin(); itr!=vertexData._img2tri.end(); itr++)
       printf("%d ",itr->second.size());
       printf("\n");
       for( map<int,vector<ply::tri_t> >::iterator itr=img2tri.begin(); itr!=img2tri.end(); itr++)
       printf("%d ",itr->second.size());
       printf("\n");
    */
        int cnt=0;
        int origSize=verts->size();

        /* if(!blending){
       g.vertex_shader<VertexShader>();
       r.fragment_shader<FragmentShader>();
       }
       else*/{
            /*  g.vertex_shader<VertexShaderBlendingDistPass>();
   r.fragment_shader<FragmentShaderBlendingDistPass>();
   for( map<int,vector<ply::tri_t> >::iterator itr=vertexData._img2tri.begin(); itr!=vertexData._img2tri.end(); itr++){
   for(int t=0; t< itr->second.size(); t++){
   int pos=itr->second[t].pos;
   FragmentShaderBlendingDistPass::idx =pos;
   osg::Vec3 v1=verts->at(itr->second[t].idx[0]);
   osg::Vec3 v2=verts->at(itr->second[t].idx[1]);
   osg::Vec3 v3=verts->at(itr->second[t].idx[2]);
   osg::Vec3 tc1,tc2,tc3;
   if(vertexData._texCoord.size()){

   tc1=vertexData._texCoord[pos]->at(itr->second[t].idx[0]);
   tc2=vertexData._texCoord[pos]->at(itr->second[t].idx[1]);;

   tc3=vertexData._texCoord[pos]->at(itr->second[t].idx[2]);;


   }
   //  cout << tc1 <<endl;
   if(tc1.x() <0 || tc1.y() <0)
   continue;

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
   g.draw_triangles(3, indices);
   if(cnt++ % 300 == 0){


   printf("\rA %04d/%04d",(int)cnt,origSize);
   last_count=cnt;
   fflush(stdout);

   }
   }
   }*/
            g.vertex_shader<VertexShaderBlending>();
            r.fragment_shader<FragmentShaderBlendingMain>();
        }

        //for(long int i=0; i<origSize-2; i+=3){
        //for( map<int,vector<ply::tri_t> >::iterator itr=vertexData._img2tri.begin(); itr!=vertexData._img2tri.end(); itr++){
        //   string tmp=string(string(av[3])+string("/")+imageList[itr->first].filename);

        /*  IplImage *image=cvLoadImage(tmp.c_str(),1);
 if(!image){
 printf("Can't load %s\n",tmp.c_str());
 continue;
 }
 Texture *texture;
 TextureMipMap *textureMipMap;
    */
        /* if(!blending){
       texture = new Texture(image);
       FragmentShader::texture = texture;
       VertexShader::texture = texture;
       }else*/
        for(int i=0; i<(int)vertexData._triangles->size()-2; i+=3){

            //textureMipMap = new TextureMipMap(image);
            //FragmentShaderBlendingMain::texture = textureMipMap;
            VertexShaderBlending::texture =pool.begin()->second;


            /*for(int t=0; t< itr->second.size(); t++){
 int pos=itr->second[t].pos;
 if(!blending && pos !=0 )
 continue;
 if(blending)
 FragmentShaderBlendingMain::idx=pos;*/
            // printf("Tri %d %d %d\n",vertexData._triangles->at(i),vertexData._triangles->at(i+1),vertexData._triangles->at(i+2));
            //if(vertexData._triangles->at(i) != 0 && vertexData._triangles->at(i+1) != 1 && vertexData._triangles->at(i+2)!= 2)
            //  continue;
            osg::Vec3 v1=verts->at(vertexData._triangles->at(i));
            osg::Vec3 v2=verts->at(vertexData._triangles->at(i+1));
            osg::Vec3 v3=verts->at(vertexData._triangles->at(i+2));
            //cout << v1 << " "<<v2 << " "<<v3 <<endl;
            osg::Vec3 tc1[4],tc2[4],tc3[4];
            if(vertexData._texCoord.size()){

                for(int a=0;a<4; a++){
                    tc1[a]=vertexData._texCoord[a]->at(vertexData._triangles->at(i));
                    tc1[a].z()=vertexData._texIds->at(vertexData._triangles->at(i))[a];
                    tc2[a]=vertexData._texCoord[a]->at(vertexData._triangles->at(i+1));;
                    tc2[a].z()=vertexData._texIds->at(vertexData._triangles->at(i))[a];

                    tc3[a]=vertexData._texCoord[a]->at(vertexData._triangles->at(i+2));;
                    tc3[a].z()=vertexData._texIds->at(vertexData._triangles->at(i))[a];
                    //cout <<"Zig "<<tc1[a] << " "<< tc2[a]<< " "<<tc3[a]<<endl;

                }

            }
            for(int a=0;a<4; a++){

                //  cout << tc1 <<endl;
                if(tc1[a].x() <0 || tc1[a].y() <0 || tc2[a].x() <0 || tc2[a].y() <0 || tc3[a].x() <0 || tc3[a].y() <0)
                    continue;
                tmp_vertices[0].x=v1.x();
                tmp_vertices[0].y=v1.y();
                tmp_vertices[0].tx[a]=tc1[a].x();
                tmp_vertices[0].ty[a]=1.0-tc1[a].y();
                tmp_vertices[0].id[a]=tc1[a].z();

                tmp_vertices[1].x=v2.x();
                tmp_vertices[1].y=v2.y();
                tmp_vertices[1].tx[a]=tc2[a].x();
                tmp_vertices[1].ty[a]=1.0-tc2[a].y();
                tmp_vertices[1].id[a]=tc2[a].z();

                tmp_vertices[2].x=v3.x();
                tmp_vertices[2].y=v3.y();
                tmp_vertices[2].tx[a]=tc3[a].x();
                tmp_vertices[2].ty[a]=1.0-tc3[a].y();
                tmp_vertices[2].id[a]=tc3[a].z();
            }
            g.draw_triangles(3, indices);
            if(cnt++ % 300 == 0){


                printf("\r%04d/%04d",(int)cnt,origSize);
                last_count=cnt;
                fflush(stdout);

            }

            //   SDL_FillRect( SDL_GetVideoSurface(), NULL, 0 );

        }
        /* if(!blending){
       cvReleaseImage(&image);
       delete texture;
       }
       else*/{
            //delete textureMipMap;
        }

        // }
        double elapsed=osg::Timer::instance()->delta_s(start,osg::Timer::instance()->tick());
        std::cout << "\n"<<format_elapsed(elapsed) << std::endl;
        double vm, rss;
        process_mem_usage(vm, rss);
        cout << "VM: " << get_size_string(vm) << "; RSS: " << get_size_string(rss) << endl;

        //  SDL_Flip(SDL_GetVideoSurface());
        cvSaveImage((osgDB::getNameLessExtension(imageName)+"-tmp.png").c_str(),outputImage);
        if(writeOut){fclose(rfp);
            fclose(gfp);
            fclose(bfp);
        }
        // show everything on screen


        if(applyGeoTags(osgDB::getNameLessExtension(imageName)+".tif",osg::Vec2(lat,lon),viewProjRead,sizeX,sizeY,"png")){
            /* if( remove((osgDB::getNameLessExtension(imageName)+"-tmp.tif").c_str() ) != 0 )
            perror( "Error deleting file" );
        else
            puts( "File successfully deleted" );*/
        }

    }


    // free texture memory

    // quit SDL
    return 0;
}
