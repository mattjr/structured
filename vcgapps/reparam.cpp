/****************************************************************************
 * VCGLib                                                            o o     *
 * Visual and Computer Graphics Library                            o     o   *
 *                                                                _   O  _   *
 * Copyright(C) 2004                                                \/)\/    *
 * Visual Computing Lab                                            /\/|      *
 * ISTI - Italian National Research Council                           |      *
 *                                                                    \      *
 * All rights reserved.                                                      *
 *                                                                           *
 * This program is free software; you can redistribute it and/or modify      *
 * it under the terms of the GNU General Public License as published by      *
 * the Free Software Foundation; either version 2 of the License, or         *
 * (at your option) any later version.                                       *
 *                                                                           *
 * This program is distributed in the hope that it will be useful,           *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
 * GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
 * for more details.                                                         *
 *                                                                           *
 ****************************************************************************/
/****************************************************************************
  History

$Log: not supported by cvs2svn $
Revision 1.2  2009-01-07 22:44:33  m.roberson
ao calcultor

Revision 1.1  2009-01-07 22:03:13  m.roberson
shadevis

Revision 1.11  2006/01/10 13:20:42  cignoni
Changed ply::PlyMask to io::Mask

Revision 1.10  2005/11/12 06:48:47   cignoni
Version 1.0
Added management of point set, correct bug in printing on the screen,

Revision 1.9  2005/01/03 13:59:54  cignoni
Resolved min/max macro conflict

Revision 1.8  2004/09/30 00:57:42  ponchio
Reference to temporary fixed and indented.

Revision 1.7  2004/09/28 09:46:51  cignoni
Added MapFalseColor

Revision 1.6  2004/09/16 14:08:35  ponchio
gamma is a math function.

Revision 1.5  2004/09/10 14:02:20  cignoni
Added Cone directions

Revision 1.4  2004/09/09 22:34:38  cignoni
Integrated lost modifications...

Revision 1.3  2004/09/09 14:35:54  ponchio
Various changes for gcc compatibility

Revision 1.2  2004/07/11 22:13:30  cignoni
Added GPL comments


****************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <time.h>
// this define is mandatory to avoid the conflicts due to the silly definition of
// min and max macros in windows.h (included by glut...)
#define NOMINMAX
#ifdef USEGL

#include <GL/glew.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <wrap/gl/space.h>
#include <wrap/gui/trackball.h>

#endif


#include <iostream>

#include <osgDB/FileNameUtils>
#include <wrap/callback.h>
#include <vcg/math/base.h>
/*#include <vcg/simplex/vertex/with/vcvn.h>
#include <vcg/simplex/vertex/with/vcvn.h>
#include <vcg/simplex/face/with/fn.h>
#include <vcg/space/index/grid_static_ptr.h>
#include <vcg/complex/trimesh/base.h>
#include<wrap/io_trimesh/export_ply.h>
#include<wrap/io_trimesh/import_ply.h>
#include<vcg/complex/trimesh/update/normal.h>
#include<vcg/complex/trimesh/update/bounding.h>
#include<vcg/complex/trimesh/update/color.h>
*/
// stuff to define the mesh
#include <vcg/simplex/vertex/base.h>
#include <vcg/simplex/face/base.h>
#include <vcg/simplex/edge/base.h>


#include <vcg/math/quadric.h>
#include <vcg/complex/algorithms/clean.h>

// io
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export_ply.h>
#include <wrap/io_trimesh/export_obj.h>
// *include the algorithms for updating: */
#include <vcg/complex/algorithms/update/topology.h>	/* topology */
#include <vcg/complex/algorithms/update/bounding.h>	/* bounding box */
#include <vcg/complex/algorithms/update/normal.h>		/* normal */
#include <time.h>
#include <string>
#include <iostream>
#include <osg/TriangleIndexFunctor>
#include <osg/Timer>
// requires software renderer source
#include "../swrender/renderer/geometry_processor.h"
#include "../swrender/renderer/rasterizer_subdivaffine.h"
#include "../swrender/renderer/span.h"
#include "../swrender/democommon.h"
#include "../swrender/fixedpoint/fixed_func.h"
#include <osg/io_utils>
#include <osg/ComputeBoundsVisitor>
#include "vertexData.h"
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osgUtil/Optimizer>
#include <opencv/cv.h>
#include <osgDB/FileNameUtils>
#include <opencv/highgui.h>
#include "../swrender/mipmap.h"
#include "../swrender/render_utils.h"
#include "../MemUtils.h"
#include "../GLImaging.h"
#include "../swrender/VertexShaders.h"
#include "../swrender/FragmentShaders.h"
#include "../swrender/Raster.h"
#include "../swrender/VipsSampler.h"
#include "GenParam.h"
#include "PLYWriterNodeVisitor.h"

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
bool removeDups(std::string basename,osg::Vec3Array *verts,osg::DrawElementsUInt * triangles);

static osg::Vec3 debugV;
bool USE_BGR=false;
struct InputVertex {
    vec3x vertex;
};



void write_header(std::ostream& _fout,int total_face_count,bool color){
    _fout <<"ply\n";
    _fout <<"format binary_little_endian 1.0\n";
    //_fout <<"comment PLY exporter written by Paul Adams\n";
    _fout <<"element vertex "<<total_face_count <<std::endl;
    _fout <<"property float x\n";
    _fout <<"property float y\n";
    _fout <<"property float z\n";
    if(color){
        _fout <<"property uchar red\n";
        _fout <<"property uchar green\n";
        _fout <<"property uchar blue\n";
    }
    _fout <<"element face " <<total_face_count/3<<std::endl;
    _fout <<"property list uchar int vertex_indices\n";
    _fout <<"property list uchar float texcoord\n";
    _fout <<"property int texnumber\n";
    _fout <<"property float quality\n";
    _fout <<"end_header\n";
}
void write_all(std::ostream& _fout,osg::DrawElementsUInt *tri,osg::Vec3Array *verts,osg::Vec4Array *colors,const std::vector<int> &imageId,osg::Vec2Array *tcarr,const int mosaic,bool flip,osg::Vec2Array *valid){
    int cnt=0;
    for(int i=0; i< (int)tri->size()-2; i+=3){
        if(valid != NULL && valid->at(i/3).x() == -999.0)
            continue;
        for(int j=0; j<3; j++){
            osg::Vec3 v=verts->at(tri->at(i+j));
            float vf[3];
            vf[0]=v[0];
            vf[1]=v[1];
            vf[2]=v[2];
            _fout.write((char *)vf,3*sizeof(float));
            if(colors != NULL && i+j <(int)colors->size() ){
                unsigned char col[3];
                osg::Vec4 c=colors->at(i+j);
                // cout <<c<<endl;
                col[0]=c[0]*255.0;
                col[1]=c[1]*255.0;
                col[2]=c[2]*255.0;
                _fout.write((char *)col,3*sizeof(unsigned char));

            }
        }

    }
    int iout[3];
    unsigned char c12=4*3;

    unsigned char c3=3;
    unsigned char ctex=3*2;
    float fout[6];
    float cfout[4];
    cfout[0]=cfout[1]=cfout[2]=cfout[3]=0;
    for(int i=0; i<(int) tri->size()-2; i+=3){
        if(valid != NULL && valid->at(i/3).x() == -999.0)
            continue;
        _fout.write((char *)&c3,sizeof(char));

        if(flip){
            iout[0]=cnt+0;
            iout[1]=cnt+1;
            iout[2]=cnt+2;
        }else{
            iout[0]=cnt+2;
            iout[1]=cnt+1;
            iout[2]=cnt+0;

        }
        _fout.write((char*)iout,sizeof(int)*3);


        _fout.write((char *)&ctex,sizeof(char));
        for(int j=0; j<3; j++){
            osg::Vec2 tc=tcarr->at(tri->at(i+j));
            fout[(j*2)+0]=tc.x();
            fout[(j*2)+1]=tc.y();
        }
        _fout.write((char*)fout,sizeof(float)*ctex);
        iout[0]= ((i/3) < imageId.size()) ? imageId.at(i/3) : 1;
        _fout.write((char*)iout,sizeof(int));

        fout[0]=(float)mosaic;
        _fout.write((char*)fout,sizeof(float));

        cnt+=3;

    }


}






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



int main(int ac, char *av[]) {
    string path=string(av[0]);
    unsigned int loc=path.rfind("/");

    string basepath= loc == string::npos ? "./" : path.substr(0,loc+1);
    basepath= osgDB::getRealPath (basepath);
    basepath+="/../../";
    osg::Timer_t start;
    osg::ref_ptr<osg::Node> model;//= osgDB::readNodeFile(av[1]);
    ply::VertexData vertexData;
    osg::ArgumentParser arguments(&ac,av);

    double scaleTex=1.0;
    bool pyramid=arguments.read("-pyr");
    int jpegQuality=95;
    arguments.read("--jpeg-quality",jpegQuality);
    bool useDisk=arguments.read("--outofcore");
    arguments.read("--scale",scaleTex);

    osg::Vec2 vtSize(-1,-1);
    arguments.read("--vt",vtSize.x(),vtSize.y());

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
    int mosaic=-1;
    if(!arguments.read("--mosaicid",mosaic)){
        fprintf(stderr,"Fail to get mosaic id\n");
        return -1;
    }

    /*std::string matfile;
    if(!arguments.read("--mat",matfile)){
        fprintf(stderr,"Fail mat file\n");
        return -1;
    }
    mat4x  viewProjReadA ;
    mat4x  viewProjRemapped ;
    /* osg::Vec2 positions[4];
    osg::Vec3 normals[4];
    osg::Vec2 texcoords[4];

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
    mat4x *viewProjMats[2]={&viewProjRemapped,&viewProjReadA};*/
    sprintf(tmp,"%s/image_r%04d_c%04d_rs%04d_cs%04d",diced_img_dir,row,col,_tileRows,_tileColumns);

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

    map<int,imgData> imageList;
    model= vertexData.readPlyFile(av[1],false,NULL,DUP,true);
    readFile(av[2],imageList);
    osg::Vec3Array *dupfreeVerts= new osg::Vec3Array;
    osg::DrawElementsUInt *dupfreeTri= new osg::DrawElementsUInt;

    removeDups(av[1],dupfreeVerts,dupfreeTri);
    cout << "Size " << dupfreeVerts->size()<< " "<<dupfreeTri->size()<<endl;
    if(!model.valid()){
        fprintf(stderr,"Failed to load model\n");
    }
    osg::Timer_t t=osg::Timer::instance()->tick();
    osg::Vec3Array *newVerts=OGFreparam(dupfreeVerts,dupfreeTri);
    if(newVerts->size() != vertexData._vertices->size()){
        fprintf(stderr,"Failed to get same number of verts %d %d!!!!\n",newVerts->size(), vertexData._vertices->size());
        exit(-1);
    }

    std::vector<osg::Vec3Array *>   texCoord;
    texCoord.push_back(vertexData._texCoord[0]);
    texCoord.push_back(vertexData._texCoord[1]);
    texCoord.push_back(vertexData._texCoord[2]);
    texCoord.push_back(vertexData._texCoord[3]);
    osg::DrawElementsUInt *tri=vertexData._triangles.get();
    /*  for (int i = 0 ; i < (int)tri->size()-2 ; i+=3) {
            for(int j=0;j<3;j++)
            for(int k=0;k<3;k++)

             assert(vertexData._vertices->at(tri->at(i+j))[k] == newVerts->at(tri->at(i+j))[k]);

        }*/
    int sizeX,sizeY;
    if(!arguments.read("--size",sizeX,sizeY)){
        osg::Vec2 srcsize;
        if(!arguments.read("--srcsize",srcsize.x(),srcsize.y())){
            fprintf(stderr,"need to have a src image size\n");
            exit(-1);
        }
        int sizeImage=  calcOptimalImageSize(srcsize,newVerts,tri,texCoord,scaleTex,(int)vtSize.x(),(int)vtSize.y());
        sizeX=sizeY=sizeImage;
    }
#if VIPS_MINOR_VERSION > 24

    vips_init(av[0]);
#endif
    osg::Vec2 texSize(sizeX,sizeY);



    double vm, rss;
    process_mem_usage(vm, rss);
    cout << "1st VM: " << get_size_string(vm) << "; RSS: " << get_size_string(rss) << endl;

    double s_d=osg::Timer::instance()->delta_s(t,osg::Timer::instance()->tick());
    printf("To reparam %f\n",s_d);
    if(!newVerts){
        fprintf(stderr,"Can't reorder bailing\n");
        exit(-1);
    }
    //doMeshReorder(av[1]);

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
    osg::Geometry *geom=geode->getDrawable(0)->asGeometry();
    if(!geom){
        printf("No embedded geom\n");
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

    osg::Vec3Array *verts=newVerts;//vertexData._vertices;
    assert(verts->size() == vertexData._vertices->size());
    osg::BoundingBox totalbb;

    for(int i=0; i <(int)verts->size(); i++){
        verts->at(i)=verts->at(i)*rotM;
        totalbb.expandBy(verts->at(i));
    }
    cout << totalbb._min << " " <<totalbb._max<<endl;


    osg::Matrixd view,proj;
    osg::Vec3d eye(0.5,0.5,0);//totalbb.center()+osg::Vec3(0,0,3.5*totalbb.radius()));
    double xrange=1.0;//totalbb.xMax()-totalbb.xMin();
    double yrange=1.0;//totalbb.yMax()-totalbb.yMin();
    double largerSide=1.0;
    osg::Matrixd matrix;
    matrix.makeTranslate( eye );
    view=osg::Matrix::inverse(matrix);
    proj= osg::Matrixd::ortho2D(-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0));
    osg::Matrix viewproj=view*proj;
    osg::Matrix bottomLeftToTopLeft= (osg::Matrix::scale(1,-1,1)*osg::Matrix::translate(0,sizeY,0));

    osg::Matrix toTex=viewproj*( osg::Matrix::translate(1.0,1.0,1.0)*osg::Matrix::scale(0.5*sizeX,0.5*sizeY,0.5f))*bottomLeftToTopLeft;

    // cout <<viewProjRead<<endl;
    osg::Vec2Array *newTCArr=new osg::Vec2Array;
    for(int i=0; i <(int)verts->size(); i++){
        osg::Vec2 tc=calcCoordReprojSimple(verts->at(i),rotM,toTex,texSize);
        //     cout << "v: " << verts->at(i)<< " :" << tc<<endl;

        newTCArr->push_back(osg::Vec2(1.0-tc[1],1.0-tc[0]));
    }
    geom->setTexCoordArray(0,newTCArr);
    // set up the texture state.

  /*  for(int i=0; i<4; i++)
        for(int j=0; j<4; j++){
            viewProjRemapped.elem[j][i]=fixed16_t(viewproj(i,j));
        }
*/

    for(int i=0; i< (int)vertexData._triangles->size(); i++){
        osg::Vec2  tc2=osg::Vec2(newVerts->at(vertexData._triangles->at(i))[0],
                                 newVerts->at(vertexData._triangles->at(i))[1]);
        //    cout <<tc << " "<<tc2<<endl;
        newTCArr->at(vertexData._triangles->at(i))=tc2;
    }







    {

        std::vector<int>imageId;
        if(vertexData._texIds.valid()){
            for(int i=0; i< (int)tri->size()-2 && i < (int)vertexData._texIds->size(); i+=3){
                imageId.push_back((int)vertexData._texIds->at(i)[0]);
            }
        }else{
            fprintf(stderr,"Failed to load texIDS\n");
            exit(-1);
        }
        std::vector<bool> *marginFace=NULL;
        if(vertexData._qualArray.valid() && vertexData._qualArray->size()){
            marginFace= new std::vector<bool>( vertexData._vertices->size(),false);
            if( vertexData._vertices->size() !=  vertexData._qualArray->size()*3){
                fprintf(stderr,"%d should be 3 time %d\n", vertexData._vertices->size(),  vertexData._qualArray->size());
                exit(-1);
            }
            for(int i=0; i< (int)vertexData._triangles->size(); i+=3)
                for(int j=0;j<3; j++)
                    marginFace->at(vertexData._triangles->at(i+j))= (vertexData._qualArray->at(i/3).y() == -999);
        }

        sprintf(tmp,"%s/param-%s",diced_dir,osgDB::getSimpleFileName(av[1]).c_str());
        std::ofstream pf(tmp);
        PLYWriterNodeVisitor nv(pf,vertexData._texIds,&vertexData._texCoord,"",marginFace,vertexData._colors);
        model->asGeode()->getDrawable(0)->asGeometry()->setVertexArray(newVerts);
        model->accept(nv);
        if(marginFace)
            delete marginFace;
      //  write_header(pf,vertexData._triangles->size(),false);

       // write_all(pf,vertexData._triangles,newVerts,NULL,imageId,newTCArr,mosaic,true,NULL);
        pf.close();

        char tmp[1024];
        sprintf(tmp,"%s/remap-%s",diced_dir,osgDB::getSimpleFileName(av[1]).c_str());
        std::ofstream f(tmp);
        bool color = vertexData._colors.valid() ? (vertexData._colors->size() >0) : false;

        if(vertexData._qualArray->size()*3 != vertexData._triangles->size()){
            fprintf(stderr,"Fail size not equal %d %d\n",vertexData._qualArray->size()*3 , vertexData._triangles->size());
            exit(-1);
        }

        int facecount=0;
        for(int i=0; i< (int)vertexData._triangles->size()-2; i+=3){
            if(vertexData._qualArray->at(i/3).x() == -999.0)
                continue;
            facecount+=3;
        }
        printf("fc %d %d\n ",facecount,vertexData._triangles->size());
        write_header(f,facecount,color);

        osg::Vec4Array *colorArr=color ? vertexData._colors.get() : NULL;
        osg::Vec2Array *validArr=(vertexData._qualArray.valid() && vertexData._qualArray->size()>0) ? vertexData._qualArray : NULL;
        if(!colorArr || !validArr){
            fprintf(stderr,"Can't load ColorArr is null %d or validArr is null %d\n",(colorArr==NULL),(validArr == NULL));
            exit(-1);
        }
        write_all(f,vertexData._triangles,vertexData._vertices,colorArr,imageId,newTCArr,mosaic,true,validArr);

        f.close();
    }







    //printf("\r %02d%%: %d/%d\n",(int)(100.0*(count/(float)total_tri_count)),count,total_tri_count);
    fflush(stdout);
    double elapsed=osg::Timer::instance()->delta_s(start,osg::Timer::instance()->tick());
    std::cout << "\n"<<format_elapsed(elapsed) << std::endl;
    process_mem_usage(vm, rss);
    cout << "VM: " << get_size_string(vm) << "; RSS: " << get_size_string(rss) << endl;






    return 0;
}



using namespace vcg;
using namespace std;


#define AMesh CMeshO

/*
// Vertex, Face, Mesh and Grid definitions.
class MyEdge;
class AFace;
class AVertex   : public VertexVCVN< float ,MyEdge,AFace > {};
class AFace     : public FaceFN< AVertex,MyEdge,AFace > {};
class AMesh     : public tri::TriMesh< vector<AVertex>, vector<AFace> > {};*/
#include "meshmodel.h"

#include "sw-visshader.h"
///////// Global ////////

int SampleNum=64;
int WindowRes=800;
unsigned int TexInd=0;
bool SwapFlag=false;
bool CullFlag=false;
bool ClosedFlag=false;
Point3f ConeDir(0,1,0);
float ConeAngleRad = math::ToRad(180.0f);

float lopass=0,hipass=1,gamma_correction=1;
float diff=.8;
float ambi=.2;
double MM[16];
double MP[16];
int VP[4];

bool LightFlag=true;
bool ColorFlag=true;
bool FalseColorFlag=false;
bool ShowDirFlag=false;
int imgcnt=0;

Color4b BaseColor=Color4b::White;
#ifdef USEGL

Trackball QV;
Trackball QL;
Trackball *Q=&QV;
#endif
int ScreenH,ScreenW;
float ViewAngle=33;
vector<Point3f> ViewVector;



AMesh m;
VertexVisShader<AMesh> Vis(m);

string OutNameMsh;

bool reorderVertsForTex(void);



bool removeDups(std::string basename,osg::Vec3Array *verts,osg::DrawElementsUInt * triangles)
{




    if(!(basename.substr(basename.length()-4)==".ply"))	{
        printf("Error: Unknown file extension %s\n",basename.c_str());
        return NULL;
    }

    // loading original mesh
    int ret=tri::io::ImporterPLY<AMesh>::Open(m,basename.c_str());
    if(ret) {printf("Error unable to open mesh %s : '%s' \n",basename.c_str(),tri::io::ImporterPLY<AMesh>::ErrorMsg(ret));exit(-1);}
    if(m.fn == 0){
        printf("No faces empty mesh \n");
        return false;
        //return 1;
    }

    /*  if(SwapFlag){
            printf("Flipping normal\n");
            tri::Clean<CMeshO>::FlipMesh(m);

        }*/
    double CCPerc=0.05;
    tri::UpdateNormals<AMesh>::PerVertexNormalized(m);
    tri::UpdateBounding<AMesh>::Box(m);
    //  tri::UpdateColor<AMesh>::VertexConstant(m,Color4b::White);
    int dup= tri::Clean<AMesh>::RemoveDuplicateVertex(m);


    /*
    tri::UpdateTopology<CMeshO>::FaceFace(m);
    tri::UpdateFlags<CMeshO>::FaceBorderFromFF(m);
    cout <<" NONmanifold edge "<<tri::Clean<AMesh>:: CountNonManifoldEdgeFF(m)<<endl;
    cout <<" Nonmanifold vertex "<<tri::Clean<AMesh>:: CountNonManifoldVertexFF(m,true)<<endl;
    tri::UpdateSelection<CMeshO>::FaceFromVertexLoose(m);
    CMeshO::FaceIterator   fi;
    CMeshO::VertexIterator vi;

    for(fi=m.face.begin();fi!=m.face.end();++fi)
        if(!(*fi).IsD() && (*fi).IsS() )
            tri::Allocator<CMeshO>::DeleteFace(m,*fi);

    for(AMesh::VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi){
        if(vi->IsS()){
            tri::Allocator<CMeshO>::DeleteVertex(m,*vi);

        }
    }
    tri::UpdateTopology<CMeshO>::FaceFace(m);
    tri::UpdateFlags<CMeshO>::FaceBorderFromFF(m);
    cout <<" NONmanifold edge "<<tri::Clean<AMesh>:: CountNonManifoldEdgeFF(m)<<endl;
    cout <<" Nonmanifold vertex "<<tri::Clean<AMesh>:: CountNonManifoldVertexFF(m,true)<<endl;*/



    /*
 int unref2= tri::Clean<AMesh>::RemoveNonManifoldFace(m);
    int unref= tri::Clean<AMesh>::RemoveNonManifoldVertex(m);
    int dup2= tri::Clean<AMesh>::RemoveDegenerateFace(m);
    tri::UpdateTopology<CMeshO>::FaceFace(m);

    cout <<" important "<<tri::Clean<AMesh>:: CountNonManifoldEdgeFF(m) <<" "<<tri::Clean<AMesh>:: CountNonManifoldVertexFF(m,true)<< endl;
    tri::UpdateSelection<CMeshO>::FaceFromVertexLoose(m);

    for(fi=m.face.begin();fi!=m.face.end();++fi)
        if(!(*fi).IsD() && (*fi).IsS() )
            tri::Allocator<CMeshO>::DeleteFace(m,*fi);



    tri::UpdateSelection<CMeshO>::ClearVertex(m);

    printf("Removed Non man %d %d\n",unref2,unref);
    unref= tri::Clean<AMesh>::RemoveUnreferencedVertex(m);
    tri::UpdateTopology<CMeshO>::FaceFace(m);

    cout <<" final "<<tri::Clean<AMesh>:: CountNonManifoldEdgeFF(m) <<" "<<tri::Clean<AMesh>:: CountNonManifoldVertexFF(m,true)<< endl;

    float minCC= CCPerc*m.bbox.Diag();
    printf("Cleaning Min CC %.1f m\n",minCC);
    std::pair<int,int> delInfo= tri::Clean<AMesh>::RemoveSmallConnectedComponentsDiameter(m,minCC);

    printf("fff %d %d\n",m.fn,m.vn);*/
    /*double CCPerc=0.2;

    float minCC= CCPerc*m.bbox.Diag();
    printf("Cleaning Min CC %.1f m\n",minCC);
    std::pair<int,int> delInfo= tri::Clean<AMesh>::RemoveSmallConnectedComponentsDiameter(m,minCC);*/
    /*  float minCC= CCPerc*m.bbox.Diag();
    printf("Cleaning Min CC %.1f m\n",minCC);
    std::pair<int,int> delInfo= tri::Clean<AMesh>::RemoveSmallConnectedComponentsDiameter(m,minCC);
    cout <<delInfo.first<<"/"<<delInfo.second<<endl;*/
    vcg::SimpleTempData<AMesh::VertContainer,int> indices(m.vert);

    int j=0;
    for(AMesh::VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi){
        if(!vi->IsD()){
            verts->push_back(osg::Vec3((*vi).P()[0],(*vi).P()[1],(*vi).P()[2]));
            indices[vi] = j++;
        }
    }
    printf("fff %d %d\n",verts->size(),j);

    for(AMesh::FaceIterator fi=m.face.begin();fi!=m.face.end();++fi){
        if(fi->IsD()){
            for(int k=0;k<3;++k)
                triangles->push_back(-1);
        }else{
            for(int k=0;k<3;++k)
                triangles->push_back(indices[(*fi).cV(k)]);
        }
    }


    // vcg::tri::io::PlyInfo pi;

    //  vcg::tri::io::ExporterOBJ<AMesh>::Save(m,"nodup.obj",pi.mask);

    //tri::io::ExporterPLY<AMesh>::Save(m,OutNameMsh.c_str(),false);
    //     exit(0);

    // glutMainLoop();
    return true;
}
