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
#include "VertexShaders.h"
#include "FragmentShaders.h"
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
static REGION *regRange;
static REGION *regOutput;
 dcm_t doublecountmapInst;
 int dTCount;
bool USE_BGR=false;
struct InputVertex {
    vec3x vertex;
};

static IMAGE *outputImage;
static IMAGE *rangeImage;
/* Generate function --- just black out the region.*/
static int white_gen( REGION *reg, void *seq, void *a, void *b )
{
  PEL *q = (PEL *) IM_REGION_ADDR( reg, reg->valid.left, reg->valid.top );
  int wd = IM_REGION_SIZEOF_LINE( reg );
  int ls = IM_REGION_LSKIP( reg );
  int y;
  
  for( y = 0; y < reg->valid.height; y++, q += ls )
    memset( (char *) q, 255, wd );
  
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
        if(valid && valid->at(i/3).x() == -999.0)
            continue;
        for(int j=0; j<3; j++){
            osg::Vec3 v=verts->at(tri->at(i+j));
            float vf[3];
            vf[0]=v[0];
            vf[1]=v[1];
            vf[2]=v[2];
            _fout.write((char *)vf,3*sizeof(float));
            if(colors && i+j <(int)colors->size() ){
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
        if(valid && valid->at(i/3).x() == -999.0)
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
        iout[0]=imageId.at(i/3);
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


static Vertex tmp_vertices[3];

inline bool process_tri(ply::tri_t &tri,osg::Vec3Array *verts, std::vector<osg::ref_ptr<osg::Vec3Array> >   &texCoord, bool blending)
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
    string path=string(av[0]);
    unsigned int loc=path.rfind("/");
#if VIPS_MINOR_VERSION > 24

    vips_init(av[0]);
#endif
    string basepath= loc == string::npos ? "./" : path.substr(0,loc+1);
    basepath= osgDB::getRealPath (basepath);
    osg::Timer_t start;
    osg::ref_ptr<osg::Node> model;//= osgDB::readNodeFile(av[1]);
    ply::VertexData vertexData;
    osg::ArgumentParser arguments(&ac,av);
    int sizeX,sizeY; sizeX=sizeY=1024;
    outputImage=NULL;
    rangeImage=NULL;
    arguments.read("--size",sizeX,sizeY);
    int jpegQuality=95;
    arguments.read("--jpeg-quality",jpegQuality);
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
    sprintf(tmp,"%s/image_r%04d_c%04d_rs%04d_cs%04d",diced_img_dir,row,col,_tileRows,_tileColumns);

    imageName=string(tmp)+".v";
    depthName=string(tmp)+"-tmp_dist.v";
    double lat=0,lon=0;
    dTCount=0;
    if(!arguments.read("-lat",lat)|| !arguments.read("-lon",lon)){
        fprintf(stderr,"Can't get lat long\n");
        return -1;
    }

    int mosaic=-1;
    if(!arguments.read("--mosaicid",mosaic)){
        fprintf(stderr,"Fail to get mosaic id\n");
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

     /*   osg::Matrix offsetMatrix=osg::Matrix::scale((double)_tileColumns,(double) _tileRows, 1.0)*osg::Matrix::translate((double)_tileColumns-1-2*col, (double)_tileRows-1-2*row, 0.0);


        mat4x offsetMatrixA=translation_matrix<fixed16_t>((double)_tileColumns-1-2*col, (double)_tileRows-1-2*row, 0.0)*scaling_matrix<fixed16_t>((double)_tileColumns,(double) _tileRows, 1.0);

        //mat4x viewA2=fast_inverse<fixed16_t>(translation_matrix<fixed16_t>(eye.x(),eye.y(),eye.z()));
        //osg::Matrixd proj= osg::Matrixd::ortho2D(-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0));

        //mat4x projA2=ortho_matrix<fixed16_t>(-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0),totalbb.zMin(),totalbb.zMax());
        osg::Matrixd viewprojmat=view*(proj*offsetMatrix);
        mat4x viewprojmatA=(offsetMatrixA*projA)*viewA;
        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                cout << fix2float<16>(viewprojmatA.elem[i][j].intValue) << " ";
            }
            cout <<endl;
        }
        cout <<endl;

        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                cout << fix2float<16>(viewProjReadA.elem[i][j].intValue) << " ";
            }
            cout <<endl;
        }
        cout <<endl;
        cout <<"OSG\n";
        cout <<viewprojmat<< endl;

        cout <<viewProjRead<< endl;*/
        
/*
        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                cout << fix2float<16>(viewA2.elem[i][j].intValue) << " ";
            }
            cout <<endl;
        }
        cout <<endl;*/
        osg::Vec2Array *newTCArr=new osg::Vec2Array;

        {
        //    osg::Matrix viewproj=view*proj;
            osg::Matrix bottomLeftToTopLeft;//= (osg::Matrix::scale(1,-1,1)*osg::Matrix::translate(0,sizeY,0));

            osg::Matrix toTex=viewProjRead*( osg::Matrix::translate(1.0,1.0,1.0)*osg::Matrix::scale(0.5*sizeX,0.5*sizeY,0.5f))*bottomLeftToTopLeft;
            osg::Vec2 texSize(sizeX-1,sizeY-1);

            for(int i=0; i <(int)verts->size(); i++){
                vec4x tvertex = (scaling_matrix<fixed16_t>(0.5*texSize.x(),0.5*texSize.y(),0.5)*translation_matrix<fixed16_t>(1.0f,1.0f,1.0f)*viewProjReadA ) * vec4x(verts->at(i).x(),verts->at(i).y(),verts->at(i).z(), fixed16_t(1));
   // cout << " "<<fixedpoint::fix2float<16>(tvertex.x.intValue) << " " <<         fixedpoint::fix2float<16>(tvertex.y.intValue) << " ";
                osg::Vec2 tc=osg::Vec2(fixedpoint::fix2float<16>(tvertex.x.intValue)/(double)texSize.x(),
                                       fixedpoint::fix2float<16>(tvertex.y.intValue)/(double)texSize.y());
                                                                              //calcCoordReprojSimple(verts->at(i),rotM,toTex,texSize);
                //     cout << "v: " << verts->at(i)<< " :" << tc<<endl;
 //cout << " "<< tc<<endl;
                newTCArr->push_back(osg::Vec2(tc[0],tc[1]));
            }

            std::vector<int>imageId;
            osg::DrawElementsUInt *tri=vertexData._triangles.get();

            for(int i=0; i< (int)tri->size()-2; i+=3){
                imageId.push_back((int)vertexData._texIds->at(i)[0]);
            }

            char tmp[1024];
            sprintf(tmp,"%s/flat-%s",diced_dir,osgDB::getSimpleFileName(av[1]).c_str());
            std::ofstream f(tmp);
            bool color = vertexData._colors.valid() ? (vertexData._colors->size() >0) : false;

         /*   if(vertexData._qualArray->size()*3 != vertexData._triangles->size()){
                fprintf(stderr,"Fail size not equal %d %d\n",vertexData._qualArray->size()*3 , vertexData._triangles->size());
                exit(-1);
            }*/

            int facecount=0;
           /* for(int i=0; i< (int)vertexData._triangles->size()-2; i+=3){
                if(vertexData._qualArray->at(i/3).x() == -999.0)
                    continue;
                facecount+=3;
            }*/
            facecount=vertexData._triangles->size();
            printf("fc %d %d\n ",facecount,vertexData._triangles->size());
            write_header(f,facecount,color);


            write_all(f,vertexData._triangles,vertexData._vertices,vertexData._colors,imageId,newTCArr,mosaic,true,/*vertexData._qualArray*/NULL);
            f.close();
        }

        if(!blending)
            VertexShader::modelviewprojection_matrix=viewProjReadA;
        else{
            VertexShaderBlendingDistPass::modelviewprojection_matrix=viewProjReadA;
            VertexShaderBlending::modelviewprojection_matrix=viewProjReadA;
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
        g.cull_mode(GeometryProcessor::CULL_NONE);

        // it is also necessary to set the clipping rectangle
        r.clip_rect(0, 0, outputImage->Xsize, outputImage->Ysize);

        // set the vertex and fragment shaders


        // specify where out data lies in memory
        g.vertex_attrib_pointer(0, sizeof(Vertex), tmp_vertices);

        if(!vertexData._texCoord.size()){
            fprintf(stderr,"No tex coords\n");
            exit(-1);
        }

        if(0){
            g.vertex_shader<VertexShaderBlending>();
            r.fragment_shader<FragmentShaderCollectTC>();
            VertexShaderBlending::modelviewprojection_matrix=viewProjReadA;



            for( map<int,vector<ply::tri_t> >::iterator itr=vertexData._img2tri.begin(); itr!=vertexData._img2tri.end(); itr++){
                for(int t=0; t< (int)itr->second.size(); t++){
                    if(process_tri(itr->second[t],verts,vertexData._texCoord,blending)){
                        FragmentShaderCollectTC::tc.clear();
                        g.draw_points(3, indices);
                        osg::Vec2 &tc1=newTCArr->at(itr->second[t].idx[0]);
                        osg::Vec2 &tc2=newTCArr->at(itr->second[t].idx[1]);
                        osg::Vec2 &tc3=newTCArr->at(itr->second[t].idx[2]);
                        if(FragmentShaderCollectTC::tc.size() ==3){
                            for(int l=0; l<3; l++){
                                osg::Vec2 tc=osg::Vec2(((FragmentShaderCollectTC::tc[l].x())/(double)(outputImage->Xsize)),
                                                       1.0-((FragmentShaderCollectTC::tc[l].y())/(double)(outputImage->Ysize)));
                                if(l ==0 )
                                    cout << "co: "<<tc <<  " " <<tc1<< " " << tc2<< " "<<tc3<<endl;
                                //       cout << FragmentShaderCollectTC::tc[l]<<endl;
                             //   osg::Vec2  tc2=osg::Vec2(newVerts->at(itr->second[t].idx[l])[0],
                                             //            newVerts->at(itr->second[t].idx[l])[1]);
                                 //  cout <<tc << " "<<tc2<<endl;
                                newTCArr->at(itr->second[t].idx[l])=tc2;  //tc;
                                //cout << newTCArr->at(itr->second[t].idx[l]) << tc1<<endl;
                            }
                            //  cout <<endl;
                            //  cout <<   FragmentShaderCollectTC::tc[0] << " "<<(int)round(tc1[0]*(outputImage->Xsize-1))-1 << " "<< (int)round(tc1[1]*outputImage->Xsize)-1<<endl;
                            // cout <<   FragmentShaderCollectTC::tc[1] << " "<< (int)round(tc2[0]*(outputImage->Xsize-1))-1 << " "<< (int)round(tc2[1]*outputImage->Xsize)-1<<endl;
                            // cout <<   FragmentShaderCollectTC::tc[2] <<  " "<<(int)round(tc3[0]*(outputImage->Xsize-1))-1 << " "<< (int)round(tc3[1]*outputImage->Xsize)-1<<endl;

                        }
                    }
                }
            }


        }

        if(!blending){
            g.vertex_shader<VertexShader>();
            r.fragment_shader<FragmentShader>();
        }
        else{
            g.vertex_shader<VertexShaderBlending>();
            r.fragment_shader<FragmentShaderBlendingDistPass>();
            TextureMipMap *sizeI=NULL;
            map<int,vector<ply::tri_t> >::iterator itr=vertexData._img2tri.begin();
            while((!sizeI || sizeI->surface==NULL) && itr!=vertexData._img2tri.end()){
                string tmp=string(string(av[3])+string("/")+imageList[itr->first].filename);

                sizeI=new TextureMipMap(tmp);
                itr++;
            }
            VertexShaderBlending::texture =sizeI;
            FragmentShaderBlendingDistPass::texture =sizeI;
            FragmentShaderBlendingDistPass::regRange=regRange;
            FragmentShaderBlendingDistPass::doublecountmapPtr =&(doublecountmapInst);
            FragmentShaderBlendingDistPass::doubleTouchCountPtr =&(dTCount);

            for( map<int,vector<ply::tri_t> >::iterator itr=vertexData._img2tri.begin(); itr!=vertexData._img2tri.end(); itr++){
                for(int t=0; t< (int)itr->second.size(); t++){
                    if(process_tri(itr->second[t],verts,vertexData._texCoord,blending))
                        g.draw_triangles(3, indices);
                }
            }
            printf("Double touch count %d\n",dTCount);

            g.vertex_shader<VertexShaderBlending>();
            r.fragment_shader<FragmentShaderBlendingMain>();
            if(sizeI){
                delete sizeI;
            }
        }
        unsigned int total_tri_count=0,count=0;
        for( map<int,vector<ply::tri_t> >::iterator itr=vertexData._img2tri.begin(); itr!=vertexData._img2tri.end(); itr++)
            total_tri_count+=itr->second.size();

        for( map<int,vector<ply::tri_t> >::iterator itr=vertexData._img2tri.begin(); itr!=vertexData._img2tri.end(); itr++){
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
                FragmentShaderBlendingMain::regOutput=regOutput;
                FragmentShaderBlendingMain::regRange=regRange;
                FragmentShaderBlendingMain::doublecountmapPtr =&(doublecountmapInst);

                if(!textureMipMap->surface)
                    continue;

            }
            for(int t=0; t< (int)itr->second.size(); t++){
                if(count % 300 == 0){
                    printf("\r %02d%%: %d/%d",(int)(100.0*(count/(float)total_tri_count)),count,total_tri_count);
                    fflush(stdout);
                }
                count++;

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
        printf("\r %02d%%: %d/%d\n",(int)(100.0*(count/(float)total_tri_count)),count,total_tri_count);
        fflush(stdout);
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
//(osgDB::getNameLessExtension(imageName)+"-tmp.tif:packbits,tile:256x256").c_str()
        IMAGE *tmpI=im_open("tmp","p");
        im_extract_bands(outputImage,tmpI,0,3);
        if( im_vips2ppm(tmpI,(osgDB::getNameLessExtension(imageName)+"-tmp.ppm").c_str())){
            fprintf(stderr,"Failed to write\n");
            cerr << im_error_buffer()<<endl;
            im_close(tmpI);
            exit(-1);
        }
        im_close(tmpI);
       /* int levels=(int)ceil(log( max( sizeX, sizeY ))/log(2.0) );
        if(!genPyramid(osgDB::getNameLessExtension(imageName)+".tif",levels,"ppm")){
            fprintf(stderr,"FAil to gen pyramid\n");
            exit(-1);
        }
*/
        IMAGE *tmpI2=im_open("tmp2","p");
        im_extract_bands(outputImage,tmpI2,3,1);
        if( im_vips2ppm(tmpI2,(osgDB::getNameLessExtension(imageName)+"-tmp-mask.pgm").c_str())){
            fprintf(stderr,"Failed to write\n");
            cerr << im_error_buffer()<<endl;
            im_close(tmpI2);
            exit(-1);
        }
        im_close(tmpI2);
        elapsed=osg::Timer::instance()->delta_s(start,osg::Timer::instance()->tick());
        std::cout << "\n"<<format_elapsed(elapsed) << std::endl;
        process_mem_usage(vm, rss);
        cout << "VM: " << get_size_string(vm) << "; RSS: " << get_size_string(rss) << endl;
        im_close(outputImage);

        if(applyGeoTags(osgDB::getNameLessExtension(imageName)+".tif",osg::Vec2(lat,lon),viewProjRead,sizeX,sizeY,basepath.c_str(),"ppm",jpegQuality)){
           /* if( remove((osgDB::getNameLessExtension(imageName)+"-tmp.tif").c_str() ) != 0 )
                perror( "Error deleting file" );
            else
                puts( "File successfully deleted" );*/
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
