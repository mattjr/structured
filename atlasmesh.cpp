//
// structured - Tools for the Generation and Visualization of Large-scale
// Three-dimensional Reconstructions from Image Data. This software includes
// source code from other projects, which is subject to different licensing,
// see COPYING for details. If this project is used for research see COPYING
// for making the appropriate citations.
// Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
//
// This file is part of structured.
//
// structured is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// structured is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with structured.  If not, see <http://www.gnu.org/licenses/>.
//

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/io_utils>
#include <string>
#include "BuildAtlas.h"
#include "vertexData.h"
#include <vips/vips.h>
#include <vips/version.h>
using namespace std;
void write_header(std::ostream& _fout,int total_face_count,bool color,bool aux,bool tex){
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
    if(aux){
                _fout <<"property float confidence\n";
    }
    if(tex){
       _fout <<"property float texture_u\n";
        _fout <<"property float texture_v\n";
    }
    _fout <<"element face " <<total_face_count/3<<std::endl;
    _fout <<"property list uchar int vertex_indices\n";


    _fout <<"end_header\n";
}
void write_all(std::ostream& _fout,osg::DrawElementsUInt *tri,osg::Vec3Array *verts,osg::Vec4Array *colors,osg::Vec3Array *tex,vector<float>&aux,bool flip){
    int cnt=0;
    for(int i=0; i< (int)tri->size()-2; i+=3){
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
            if(aux.size() && i+j<(int)aux.size()){
                float a;
                a=aux.at(i+j);
                _fout.write((char *)&a,1*sizeof(float));

            }
            if(tex && i+j <(int)tex->size() ){
                float t[2];
                osg::Vec3 tc=tex->at(i+j);
               // cout <<c<<endl;
                t[0]=tc[0];
                t[1]=tc[1];
                _fout.write((char *)t,2*sizeof(float));

            }
        }
    }
    int iout[3];
    unsigned char c=3;
    for(int i=0; i<(int) tri->size()-2; i+=3){
        _fout.write((char *)&c,sizeof(char));

        if(flip){
            iout[0]=i;
            iout[1]=i+1;
            iout[2]=i+2;
        }else{
            iout[0]=i+2;
            iout[1]=i+1;
            iout[2]=i+0;

        }
        _fout.write((char*)iout,sizeof(int)*3);
        cnt++;

    }
   // printf("%d\n",cnt);


}
int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    string matname;
    if(!arguments.read("-mat",matname)){
        fprintf(stderr, "need mat\n");
        exit(-1);
    }
#if VIPS_MINOR_VERSION > 24
    vips_init(argv[0]);
#endif
    double scaleFactor=1.0;
    arguments.read("-scale",scaleFactor);
    string basedir="mesh";
    arguments.read("-dir",basedir);
    std::string outfile="temp.obj";
    arguments.read("-outfile",outfile);
    bool flip=arguments.read("-F");
    int POTAtlasSize;
    if(!arguments.read("-potsize",POTAtlasSize)){
        fprintf(stderr, "need atlasSize\n");
        exit(-1);
    }
    string mosaic_cells_fname;
    if(!arguments.read("-cells",mosaic_cells_fname)){
        fprintf(stderr, "need mat\n");
        exit(-1);
    }
    int totalX,totalY;
    std::vector<mosaic_cell> mosaic_cells;

    loadMosaicCells(mosaic_cells_fname,totalX,totalY,mosaic_cells);

    osg::Matrix viewProj;
    readMatrixToScreen(matname,viewProj);
    ply::VertexDataMosaic vertexData;
    osg::Node *model;
    for(int pos=1;pos<arguments.argc();++pos)
        {
            if (!arguments.isOption(pos))
            {
                //cout << arguments[pos] <<endl;
                model= vertexData.readPlyFile( arguments[pos]);
               // cout << "ver "<<vertexData._vertices->size()<<endl;
              //  cout << "tri "<<vertexData._triangles->size()<<endl;

            }
        }


    osg::Geode *geode=model->asGeode();

    osg::Drawable *drawable=geode->getDrawable(0);
    osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
    osg::Vec3Array *verts=static_cast< osg::Vec3Array*>(geom->getVertexArray());
    osg::Vec4Array *colors=static_cast< osg::Vec4Array*>(geom->getColorArray());
    osg::Vec4Array *texCoordsStored=static_cast< osg::Vec4Array*>(geom->getTexCoordArray(0));

    osg::DrawElementsUInt* primitiveSet = dynamic_cast<osg::DrawElementsUInt*>(geom->getPrimitiveSet(0));
    //int offset=newVerts->size();
    osg::ref_ptr<VipsAtlasBuilder >tf_atlas=createVTAtlas( viewProj, totalX, totalY,POTAtlasSize,
                 mosaic_cells,
               false,scaleFactor,basedir);
    vector<float>aux(vertexData._texCoord[0]->size(),-1.0f);

    for(int i=0; i< (int)primitiveSet->getNumIndices()-2; i+=3){
        for(int j=0; j <3; j++){
            int mosaic_id=( int)vertexData._texIds->at(i/3)[1];
            int aux_data=( int)vertexData._texIds->at(i/3)[0];

            aux.at(i+j)=(float)aux_data;
            osg::Vec3f &tc =    vertexData._texCoord[0]->at(i+j);

            if(mosaic_id >= 0 && mosaic_id< (int)tf_atlas->atlasSourceMatrix.size()){
                if(tf_atlas->atlasSourceMatrix[mosaic_id] != NULL){
                    //     printf("%d\n", atlas->atlasMatrix[mosaic_id]);
                   //  cout << ((vips::VImage*)tf_atlas->atlasSourceMatrix[mosaic_id])->filename() <<endl;
                    // cout<< "orig: "<< tc<<endl;
                    // cout <<"offset "<<atlas->offsetMats[mosaic_id]<<endl;
                    osg::Matrix matrix2;
                    matrix2=dynamic_cast<VipsAtlasBuilder*>(tf_atlas.get())->getTextureMatrix((vips::VImage*)tf_atlas->atlasSourceMatrix[mosaic_id]);


                    const osg::Matrix &matrix =tf_atlas->offsetMats[mosaic_id];//atlas->atlasMatrix[mosaic_id]);

                    // cout << matrix << matrix2<<endl;
                    osg::Vec2 tc2;
                    tc2.set(tc[0]*matrix(0,0) + tc[1]*matrix(1,0) + matrix(3,0),
                            tc[0]*matrix(0,1) + tc[1]*matrix(1,1) + matrix(3,1));

                    //  cout << "shift: "<<tc2<<endl;

                    tc.set(tc2[0]*matrix2(0,0) + tc2[1]*matrix2(1,0) + matrix2(3,0),
                           tc2[0]*matrix2(0,1) + tc2[1]*matrix2(1,1) + matrix2(3,1),0);
                    //   cout << "Final:"<<tc<<endl;
                }else{
                    tc.set(-1,-1,-1);
                    printf("Failed atlas matrix map %d 0x%lx %f %f\n",mosaic_id,(long unsigned int)tf_atlas->atlasSourceMatrix[mosaic_id],tc.x(),tc.y() );
                    printf("Num of map %d\n",(int)tf_atlas->atlasSourceMatrix.size());
                    for(int k=0; k< (int)tf_atlas->atlasSourceMatrix.size(); k++){
                        printf("0x%lx\n",(long unsigned int)tf_atlas->atlasSourceMatrix[k]);
                    }
                    //    std::map< MyDataSet::range<std::pair<double,double > > ,int>::iterator itr;
                    //   for(itr=dynamic_cast<MyDataSet*>(_dataSet)->cell_coordinate_map.begin(); itr!=dynamic_cast<MyDataSet*>(_dataSet)->cell_coordinate_map.end(); itr++)
                    //     cout << "["<<itr->first.min().first<< "," << itr->first.min().second<<" - "<< itr->first.max().first<< "," << itr->first.max().second<<"] : " << itr->second<<"\n";


                }
            }else{
                printf("Failed mosaic_id %d\n",mosaic_id);
                tc.set(-1,-1,-1);

            }
        }
    }
  //  cout << vertexData._colors->size()<<endl;
  //  cout << ((osg::Vec3Array*)model->asGeode()->getDrawable(0)->asGeometry()->getVertexArray())->size()<<endl;
   // cout << ((osg::Vec4Array*)model->asGeode()->getDrawable(0)->asGeometry()->getColorArray())->size()<<endl;
 //   cout << "tex "<<vertexData._texCoord[0]->size()<<endl;
  //  cout << "id "<<vertexData._texIds->size()*3<<endl;
    model->asGeode()->getDrawable(0)->asGeometry()->setTexCoordArray(0,vertexData._texCoord[0]);

  //  cout << model->asGeode()->getDrawable(0)->asGeometry()->getPrimitiveSet(0)->getNumIndices()<<endl;
    std::ofstream f(outfile.c_str());
    bool color = vertexData._colors.valid() ? (vertexData._colors->size() >0) : false;
    bool tex = vertexData._texCoord[0].valid() ? (vertexData._texCoord[0]->size() >0) : false;
    bool auxUsed=aux.size()>0 &&aux.size() == vertexData._texCoord[0]->size();
    write_header(f,vertexData._triangles->size(),color,auxUsed,tex);
    write_all(f,vertexData._triangles,vertexData._vertices,vertexData._colors,vertexData._texCoord[0],aux,flip);
    //PLYWriterNodeVisitor nv(f);
    //root->accept(nv);
    f.close();;
   // osgDB::writeNodeFile(*model,outfile.c_str());



}

