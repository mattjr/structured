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
#include "VertexShaders.h"
#include "FragmentShaders.h"
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


struct InputVertex {
    vec3x vertex;
};

static IplImage *outputImage;
//FILE *totalfp;

FILE *rfp,*gfp,*bfp;



map<int,TextureMipMap*> FragmentShaderVarMain::texturepool;

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
    writeOut=arguments.read("--write");
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

    outputImage=cvCreateImage(cvSize(sizeX,sizeY),IPL_DEPTH_8U,4);
    FragmentShaderVarMain::outputImage =outputImage;
    FragmentShaderVarMain::writeOut=writeOut;
    FragmentShaderVarMain::f_arr[0]=NULL;
    FragmentShaderVarMain::f_arr[1]=NULL;
    FragmentShaderVarMain::f_arr[2]=NULL;

    //varImage=cvCreateImage(cvSize(sizeX,sizeY),IPL_DEPTH_32F,1);
    // cvZero(varImage);

    if(writeOut){
        sprintf(tmp,"mosaic/var_r%04d_c%04d_rs%04d_cs%04d.r.raw",row,col,_tileRows,_tileColumns);

        rfp=fopen(tmp,"w");
        if(!rfp){
            fprintf(stderr,"can't open %s\n",tmp);
            exit(-1);
        }
        sprintf(tmp,"mosaic/var_r%04d_c%04d_rs%04d_cs%04d.g.raw",row,col,_tileRows,_tileColumns);

        gfp=fopen(tmp,"w");
        if(!gfp){
            fprintf(stderr,"can't open %s\n",tmp);
            exit(-1);
        }
        sprintf(tmp,"mosaic/var_r%04d_c%04d_rs%04d_cs%04d.b.raw",row,col,_tileRows,_tileColumns);

        bfp=fopen(tmp,"w");
        if(!bfp){
            fprintf(stderr,"can't open %s\n",tmp);
            exit(-1);
        }
        FragmentShaderVarMain::f_arr[0]=rfp;
        FragmentShaderVarMain::f_arr[1]=gfp;
        FragmentShaderVarMain::f_arr[2]=bfp;

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

        //mat4x viewA=fast_inverse<fixed16_t>(translation_matrix<fixed16_t>(eye.x(),eye.y(),eye.z()));
        osg::Matrixd proj= osg::Matrixd::ortho2D(-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0));

      //  mat4x projA=ortho_matrix<fixed16_t>(-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0),totalbb.zMin(),totalbb.zMax());
        osg::Matrixd viewprojmat=view*proj;
        //cout <<viewprojmat <<endl;

        //if(!blending)
        //  VertexShader::modelviewprojection_matrix=projA*viewA;
        //else{
        //VertexShaderVarDistPass::modelviewprojection_matrix=projA*viewA;
        VertexShaderVar::modelviewprojection_matrix=viewProjReadA;//projA*viewA;

        start = osg::Timer::instance()->tick();


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
        FragmentShaderVarMain::texturepool = pool;

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

        int cnt=0;
        int origSize=verts->size();



            g.vertex_shader<VertexShaderVar>();
            r.fragment_shader<FragmentShaderVarMain>();



        for(int i=0; i<(int)vertexData._triangles->size()-2; i+=3){


            VertexShaderBlending::texture =pool.begin()->second;



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


        }


        double elapsed=osg::Timer::instance()->delta_s(start,osg::Timer::instance()->tick());
        std::cout << "\n"<<format_elapsed(elapsed) << std::endl;
        double vm, rss;
        process_mem_usage(vm, rss);
        cout << "VM: " << get_size_string(vm) << "; RSS: " << get_size_string(rss) << endl;

        //  SDL_Flip(SDL_GetVideoSurface());
        IplImage *tmp = cvCreateImage(cvGetSize(outputImage), IPL_DEPTH_8U, 3);
        cvCvtColor(outputImage, tmp, CV_RGBA2RGB);
        cvReleaseImage(&outputImage);
        cvSaveImage((osgDB::getNameLessExtension(imageName)+"-tmp.ppm").c_str(),tmp);
        cvReleaseImage(&tmp);
        if(writeOut){
            fclose(rfp);
            fclose(gfp);
            fclose(bfp);

        }
        // show everything on screen


        if(applyGeoTags(osgDB::getNameLessExtension(imageName)+".tif",osg::Vec2(lat,lon),viewProjRead,sizeX,sizeY,"ppm")){
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
