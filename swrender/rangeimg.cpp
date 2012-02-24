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
#include "ar.h"
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
#include "Clipper.h"
#include <opencv/cv.h>
#include "../GLImaging.h"
#include <opencv/highgui.h>
#include "mipmap.h"
#include "../MemUtils.h"
#include "render_utils.h"
// the software renderer stuff is located in the namespace "swr" so include
// that here
using namespace swr;
using namespace std;
#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>
#include "calibFile.h"

struct InputVertex {
    vec3x vertex;
};

static IplImage *outputImage;
static IplImage *rangeImage;

osg::Vec3 reprojectPt(const osg::Matrixf &mat,const osg::Vec3 &v,const CameraCalib &_calib){

    osg::Vec3 cam_frame=mat*v;
   // cout << "beat" <<cam_frame<<endl;

    osg::Vec3 und_n;
    und_n.x()=cam_frame.x()/cam_frame.z();
    und_n.y()=cam_frame.y()/cam_frame.z();
  //  cout << "ff" <<und_n<<endl;
    double r_2 = pow(und_n.x(),2) + pow(und_n.y(),2);
    double k_radial = 1 + _calib.kc1*r_2 + _calib.kc2*pow(r_2,2) + _calib.kc5*pow(r_2,3);
    double delta_x = 2*_calib.kc3*und_n.x()*und_n.y() + _calib.kc4*(r_2 + 2*pow(und_n.x(),2));
    double delta_y = _calib.kc3*(r_2 + 2*pow(und_n.y(),2)) + 2*_calib.kc4*und_n.x()*und_n.y();
    osg::Matrix m1=osg::Matrix::scale(_calib.fcx,_calib.fcy,1);
    osg::Matrix m2=osg::Matrix::translate(_calib.ccx,_calib.ccy,0);
    osg::Matrix k1=osg::Matrix::scale(k_radial,k_radial,1);
    osg::Matrix delta=osg::Matrix::translate(delta_x,delta_y,0);
    osg::Vec3 norm_c=k1*und_n*delta;
    osg::Vec3 pixel_c=(m1*norm_c*m2);
  //  cout << "ff2" <<norm_c<<endl;

    return osg::Vec3(pixel_c.x(),pixel_c.y(),cam_frame.z());
}
// this is the vertex shader which is executed for each individual vertex that
// needs to ne processed.
struct VertexShaderBlending {

    // this specifies that this shader is only going to use 1 vertex attribute
    // array. There you be used up to Renderer::MAX_ATTRIBUTES arrays.
    static const unsigned attribute_count = 1;

    // this specifies the number of varyings the shader will output. This is
    // for instance used when clipping.
    static const unsigned varying_count = 1;
    static mat4x modelview_matrix;
    static mat4x modelviewprojection_matrix;
    static osg::Matrix viewproj;
    static osg::Matrix view;
    static CameraCalib _calib;

    static osg::Matrix scaledevice;
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
        //   vec4x tvertex = modelviewprojection_matrix * vec4x(v.x,v.y,v.z, fixed16_t(1));
        osg::Vec3 pt(v.x,v.y,v.z);
        double zrange= (view*pt).length();
       /* osg::Vec3 tmpv= viewproj * pt;
        tmpv.x()/=tmpv.z();
        tmpv.y()/=tmpv.z();*/
       // cout <<"tmp1 " <<tmpv<<endl;

        osg::Vec3 tmpv2=  reprojectPt(view,pt,_calib);
       // cout <<tmpv2<<endl;
      //cout <<"tmp2 " <<tmpv2<<endl;
        osg::Vec3 tvertex=  scaledevice*tmpv2;
       // cout << tvertex<<endl;
        //   cout << "V:" << tvertex<<endl;
        //cout <<"V:"<<v.x<<" "<<v.y<<" "<<v.z<<endl;
        //cout << "T:"<<fixedpoint::fix2float<16>(tvertex.x.intValue) << " " <<         fixedpoint::fix2float<16>(tvertex.y.intValue) << " "<<        fixedpoint::fix2float<16>(tvertex.z.intValue)<<endl;
        //cout << tvertex.x << " " << tvertex.y<< " " <<tcer
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

        out.x =fixed16_t(tvertex.x()).intValue;//tvertex.x.intValue;
        out.y = fixed16_t(tvertex.y()).intValue;//tvertex.y.intValue;
        out.z = fixed16_t(1.0).intValue;//fixedpoint::float2fix<16>(tvertex.z());//tvertex.z.intValue;
        out.w = 1 << 16;//tvertex.w.intValue;
        // bring the texture coordinates into the appropriate range for the rasterizer.
        // this mean it has to be converted to fixed point and premultiplied with the width, height of the
        // texture minus 1. Doing this in the vertex shader saves us from doing this in the fragment shader
        // which makes things faster.
      //  printf("Zragne %f\n",zrange);
        out.varyings[0] = fixed16_t(zrange).intValue;

    }


};
mat4x VertexShaderBlending::modelview_matrix;
mat4x VertexShaderBlending::modelviewprojection_matrix;

osg::Matrix VertexShaderBlending::scaledevice;
osg::Matrix VertexShaderBlending::viewproj;
osg::Matrix VertexShaderBlending::view;

CameraCalib VertexShaderBlending::_calib;

// this is the fragment shader
struct FragmentShaderBlendingMain : public SpanDrawer32BitColorAndDepth<FragmentShaderBlendingMain> {
    // varying_count = 3 tells the rasterizer that it only needs to interpolate
    // three varying values (the r, g and b in this context).
    static const unsigned varying_count = 1;
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
    {/*printf("T1 %f %f\nT2 %f %f\nT3 %f %f\n",fix2float<16>(v1.x),fix2float<16>(v1.y),fix2float<16>(v2.x),fix2float<16>(v2.y),fix2float<16>(v3.x)
                    ,fix2float<16>(v3.y));**/}//{}

    // the fragment shader is called for each pixel and has read/write access to
    // the destination color and depth buffers.
    static void single_fragment(const IRasterizer::FragmentData &fd, unsigned int &color, unsigned int &depth)
    {
        //printf("Ge\n");

        float r=fixedpoint::fix2float<16>(fd.varyings[0]);
        if(r < 0.0)
            return;
        unsigned char ri=(int)clamp((float)round((r/5.0)*255.0),(float)0.0,(float)255.0);
        // r=-1;
        int tmp=color;
        float tmpr=-1;
        memcpy((void*)&tmpr,&tmp,sizeof(float));
        if(tmpr > r || tmpr == -1.0){
         //   printf("%f\n",r);
            memcpy((void*)&tmp,&r,sizeof(float));
            color=tmp;
            depth = 0 | 0 << 8 | (ri) << 16 | 255 << 24;

        }
    }

    // this is called by the span drawing function to get the location of the color buffer
    static void* color_pointer(int x, int y)
    {
        IplImage *screen = outputImage;///SDL_GetVideoSurface();
        //return static_cast<unsigned short*>(screen->pixels) + x + y * screen->w;
        return &(CV_IMAGE_ELEM(screen,float,y,x*screen->nChannels));
    }

    // this is called by the span drawing function to get the location of the depth buffer
    static void* depth_pointer(int x, int y)
    {
        IplImage *screen = rangeImage;///SDL_GetVideoSurface();
        //return static_cast<unsigned short*>(screen->pixels) + x + y * screen->w;
        return &(CV_IMAGE_ELEM(screen,unsigned char,y,x*screen->nChannels));
    }
    // static TextureMipMap *texture;
};
int FragmentShaderBlendingMain::idx;


void readFile(string fname,map<int,imgData> &imageList){
    std::ifstream m_fin(fname.c_str());

    while(!m_fin.eof()){
        imgData cam;
        double low[3], high[3];
        string tmp;
        double time;
        if(m_fin >> cam.id >>cam.filename >> low[0] >> low[1] >> low[2] >> high[0] >> high[1] >> high[2]
           >> cam.m(0,0) >>cam.m(0,1)>>cam.m(0,2) >>cam.m(0,3)
           >> cam.m(1,0) >>cam.m(1,1)>>cam.m(1,2) >>cam.m(1,3)
           >> cam.m(2,0) >>cam.m(2,1)>>cam.m(2,2) >>cam.m(2,3)
           >> cam.m(3,0) >>cam.m(3,1)>>cam.m(3,2) >>cam.m(3,3)){
            cam.bbox.expandBy(low[0],low[1],low[2]);
            cam.bbox.expandBy(high[0],high[1],high[2]);
            imageList[cam.id]=cam;
        }else{
            fprintf(stderr,"Failed to read line\n");
        }
    }
}

static void PFMWrite( float *pFloatImage, const char *pFilename, int width, int height )
{
    FILE *fp;
    fp = fopen( pFilename, "wb" );
    fprintf( fp, "Pf\n%d %d\n-1.000000\n", width, height );
    int i;
    for( i = height-1; i >= 0; i-- )
    {
        float *pRow = &pFloatImage[ width * i];
//        for(int j=0; j< width; j++)
  //          printf("%f\n",pRow[j]);
        fwrite( pRow, width * sizeof( float ), 1, fp );
    }
    fclose( fp );
}

int main(int ac, char *av[]) {

    osg::Timer_t start;
    //osg::ref_ptr<osg::Node> model;//= osgDB::readNodeFile(av[1]);
    osg::ArgumentParser arguments(&ac,av);
    int sizeX,sizeY; sizeX=sizeY=1024;
    int num_threads=    num_threads=OpenThreads::GetNumberOfProcessors();

    arguments.read("-t",num_threads);
  //  printf("Using %d threads\n",num_threads);
    arguments.read("--size",sizeX,sizeY);

    unsigned int _tileRows;
    unsigned int _tileColumns;
    int row;
    int col;
    char tmp[1024];
    /* if(!arguments.read("--image",row,col,_tileRows,_tileColumns)){
        fprintf(stderr,"Fail to get image params\n");
        return -1;
    }
    std::string matfile;
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
*/
    //sprintf(tmp,"mesh-diced/image_r%04d_c%04d_rs%04d_cs%04d",row,col,_tileRows,_tileColumns);
    outputImage=cvCreateImage(cvSize(sizeX,sizeY),IPL_DEPTH_32F,1);
    rangeImage=cvCreateImage(cvSize(sizeX,sizeY),IPL_DEPTH_8U,4);
    cvZero(rangeImage);
    cvSet(outputImage,cvScalar(-1.0));
    string ddir="depthimg/";
    if(!osgDB::fileExists(ddir))
        osgDB::makeDirectory(ddir);
    map<int,imgData> imageList;
    //model= vertexData.readPlyFile(av[1]);

    osgDB::Registry::instance()->setBuildKdTreesHint(osgDB::ReaderWriter::Options::BUILD_KDTREES);
    osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(av[1]);
    if(!model.valid()){
        fprintf(stderr,"Failed to load mesh-diced/totalrot.ive can't split bailing!\n");
        exit(-1);
    }
    osg::ref_ptr<KdTreeBbox> kdbb=setupKdTree(model);

    readFile(av[2],imageList);
   // bool blending = arguments.read("--blend");
    string calibfile;
    arguments.read("-calib",calibfile);
    StereoCalib stereo_calib(calibfile);
    mat4x projA;
    const CameraCalib &calib = stereo_calib.camera_calibs[0];
    double scale=1.0;


    double cparam2[16] = {calib.fcx/scale, 0.000000, calib.ccx/scale, 0.0000 ,
                          0,     calib.fcy/scale, calib.ccy/scale ,   0,
                          0,            0,              1,             0,
                          0,            0,              0,             1};


    double cparam3[16] = {2.0/calib.width, 0.000000, 0.0, -1.0000 ,
                          0,     2.0/calib.height, 0.0 ,   -1.00,
                          0,            0,              1,             0,
                          0,            0,              0,             1};

    double intrinsic[16];


    osg::Matrix proj(cparam2);
    osg::Matrix scaledevice(cparam3);

    for(int i=0; i<4; i++){
        for(int j=0; j<4; j++){
            projA.elem[j][i]=fixed16_t(proj(i,j));
        }
    }
    cout <<proj<<endl;


    if(model.valid()){
     /*   osg::Geode *geode= dynamic_cast<osg::Geode*>(model.get());
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

        }*/
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

     /*   osg::Vec3Array *verts=vertexData._vertices;
        for(int i=0; i <(int)verts->size(); i++){
            verts->at(i)=verts->at(i)*rotM;
            totalbb.expandBy(verts->at(i));
        }
*/
        /* osg::ComputeBoundsVisitor cbbv(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
        model->accept(cbbv);
        osg::BoundingBox totalbb = cbbv.getBoundingBox();*/

        osg::Vec3d eye(totalbb.center()+osg::Vec3(0,0,3.5*totalbb.radius()));
        double xrange=totalbb.xMax()-totalbb.xMin();
        double yrange=totalbb.yMax()-totalbb.yMin();
        double largerSide=std::max(xrange,yrange);
        osg::Matrixd matrix;
        matrix.makeTranslate( eye );

        /*        osg::Matrixd view=osg::Matrix::inverse(matrix);
        //cout << view << endl;

        mat4x viewA=fast_inverse<fixed16_t>(translation_matrix<fixed16_t>(eye.x(),eye.y(),eye.z()));
        osg::Matrixd proj= osg::Matrixd::ortho2D(-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0));

        mat4x projA=ortho_matrix<fixed16_t>(-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0),totalbb.zMin(),totalbb.zMax());*/
        // osg::Matrixd viewprojmat=view*proj;

        start = osg::Timer::instance()->tick();


        Vertex tmp_vertices[3];


        // the indices we need for rendering
        unsigned indices[] = {0, 1, 2};

        // create a rasterizer class that will be used to rasterize primitives
        RasterizerSubdivAffine r;
        // create a geometry processor class used to feed vertex data.
        GeometryProcessor g(&r);
        // it is necessary to set the viewport
        g.viewport(0, 0, outputImage->width, outputImage->height);
        // set the cull mode (CW is already the default mode)
        g.cull_mode(GeometryProcessor::CULL_NONE);

        // it is also necessary to set the clipping rectangle
        r.clip_rect(0, 0, outputImage->width, outputImage->height);

        // set the vertex and fragment shaders


        // specify where out data lies in memory
        g.vertex_attrib_pointer(0, sizeof(Vertex), tmp_vertices);


        int last_count=0;
        int cnt=0;
        //int origSize=verts->size();



        g.vertex_shader<VertexShaderBlending>();
        r.fragment_shader<FragmentShaderBlendingMain>();

        VertexShaderBlending::_calib=stereo_calib.camera_calibs[0];
        osg::Timer_t startTick = osg::Timer::instance()->tick();
        formatBar("Img",startTick,0,imageList.size());

        for( map<int,imgData >::iterator itr=imageList.begin(); itr!=imageList.end(); itr++){
          //  printf("\r%04d/%04d",(int)++cnt,imageList.size());
           // fflush(stdout);
          //  if(itr->second.filename != "PR_20090613_001426_259_LC16.png")
            //    continue;
            mat4x viewA;
            osg::Matrix view;

            for(int i=0; i<4; i++){
                for(int j=0; j<4; j++){
                    viewA.elem[j][i]=fixed16_t(itr->second.m(j,i));
                    view=itr->second.m;
                }
            }
            mat4x viewprojmatA=/*(projA)**/viewA;
            VertexShaderBlending::view=(view);

            VertexShaderBlending::viewproj=(proj*view);
            VertexShaderBlending::scaledevice=scaledevice;
            VertexShaderBlending::modelviewprojection_matrix=viewprojmatA;//projA*viewA;

            osg::BoundingBox bbox;
            bbox.expandBy((itr->second.bbox._min)*rotM);
            bbox.expandBy((itr->second.bbox._max)*rotM);
           // cout << bbox._min <<endl;
           // cout << bbox._max <<endl;
            double expandmult=bbox.radius();
            bbox.expandBy(osg::Vec3(bbox._min.x()-expandmult,
                                    bbox._min.y()-expandmult,
                                    bbox._min.z()-expandmult));


            bbox.expandBy(osg::Vec3(bbox._max.x()+expandmult,
                                    bbox._max.y()+expandmult,
                                    bbox._max.z()+expandmult));


            /* cout << bbox._min <<endl;
            cout << bbox._max <<endl;*/
            geom_elems_dst dstGeom(0);
            osg::ref_ptr<osg::Node> root=kdbb->intersect(bbox,dstGeom,IntersectKdTreeBbox::DUP);


            for(int i=0; i<(int)dstGeom.faces->size()-2; i+=3){



                const osg::Vec3 &v1=dstGeom.vertices->at(dstGeom.faces->at(i));
                const osg::Vec3 &v2=dstGeom.vertices->at(dstGeom.faces->at(i+1));
                const osg::Vec3 &v3=dstGeom.vertices->at(dstGeom.faces->at(i+2));
                osg::Vec3 v1p=  reprojectPt(view,v1,stereo_calib.camera_calibs[0]);
                double margin=0.2;
                if(v1p.x() < 0.0-stereo_calib.camera_calibs[0].width*margin || v1p.x() > stereo_calib.camera_calibs[0].width*(1.0+margin) || v1p.y() < 0.0-stereo_calib.camera_calibs[0].height*margin || v1p.y() > stereo_calib.camera_calibs[0].height*(1.0+margin) )
                    continue;

                osg::Vec3 v2p=  reprojectPt(view,v2,stereo_calib.camera_calibs[0]);

                if(v2p.x() < 0.0-stereo_calib.camera_calibs[0].width*margin || v2p.x() > stereo_calib.camera_calibs[0].width*(1.0+margin) || v2p.y() < 0.0-stereo_calib.camera_calibs[0].height*margin || v2p.y() > stereo_calib.camera_calibs[0].height*(1.0+margin) )
                    continue;

                osg::Vec3 v3p=  reprojectPt(view,v3,stereo_calib.camera_calibs[0]);

                if(v3p.x() < 0.0-stereo_calib.camera_calibs[0].width*margin || v3p.x() > stereo_calib.camera_calibs[0].width*(1.0+margin) || v3p.y() < 0.0-stereo_calib.camera_calibs[0].height*margin || v3p.y() > stereo_calib.camera_calibs[0].height*(1.0+margin) )
                    continue;


                //cout <<"cam:"<<v1<<" "<<bbox._min<<" "<<bbox.contains(v1)<<endl;

                if(!bbox.contains(v1) || !bbox.contains(v2) || !bbox.contains(v3))
                    continue;
                /*cout <<view<<endl;
                cout <<"v1:" << v1<<endl;
                osg::Vec3 pt=view*v1;
                osg::Vec4 pt2(pt.x(),pt.y(),pt.z(),1.0);
                cout <<"cam frame:"<<pt<<endl;
                osg::Vec4 a=proj*pt2;
                a.x()/=a.z();
                a.y()/=a.z();

                cout <<"proj:"<<a<<endl;
                //  cout <<proj<<endl;
                //cout <<scaless<<endl;
                osg::Vec3 a2(a.x(),a.y(),a.z());
                cout <<"ss: "<<scaledevice*(a2)<<endl;

*/
                /* osg::Vec3 t=*v1;
                t.x()/=t.z();
                t.y()/=t.z();
                cout <<"comp "<<t<<endl;
*/
                tmp_vertices[0].x=v1.x();
                tmp_vertices[0].y=v1.y();
                tmp_vertices[0].z=v1.z();

                tmp_vertices[1].x=v2.x();
                tmp_vertices[1].y=v2.y();
                tmp_vertices[1].z=v2.z();

                tmp_vertices[2].x=v3.x();
                tmp_vertices[2].y=v3.y();
                tmp_vertices[2].z=v3.z();

               //  float area = 0.5*((v1 - v2)^(v3 - v2)).length();
            //     printf("%f\n",area);
               // if((v2[0]- -16.7324009)<0.0001 && (v2[1] - -0.0329999998 ) < 0.001 && (v2[2]- 26.6951008) < 0.001)
                g.draw_triangles(3, indices);
            }
            char tmp[1024];
            sprintf(tmp,"%s/%s.pfm",ddir.c_str(),itr->second.filename.c_str());
            PFMWrite((float*)outputImage->imageData,tmp,outputImage->width,outputImage->height);
            cvSet(outputImage,cvScalar(-1.0));
            sprintf(tmp,"%s/%s.png",ddir.c_str(),itr->second.filename.c_str());
            cvSaveImage(tmp,rangeImage);
            cvZero(rangeImage);
            formatBar("Img",startTick,++cnt,imageList.size());

        }
        formatBar("Img",startTick,imageList.size(),imageList.size());

        double elapsed=osg::Timer::instance()->delta_s(start,osg::Timer::instance()->tick());
        std::cout << "\n"<<format_elapsed(elapsed) << std::endl;
        double vm, rss;
        process_mem_usage(vm, rss);
        cout << "VM: " << get_size_string(vm) << "; RSS: " << get_size_string(rss) << endl;

        //  SDL_Flip(SDL_GetVideoSurface());

        // show everything on screen



    }


    // free texture memory

    // quit SDL
    return 0;
}
