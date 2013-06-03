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
#include "Clipper.h"
#include <osgUtil/SmoothingVisitor>
#include "PLYWriterNodeVisitor.h"
#include <osgUtil/Optimizer>
#include <osgUtil/DelaunayTriangulator>
#include <osg/Point>
#include <iostream>
#include <osg/ComputeBoundsVisitor>
#include "vertexData.h"
#include <vector>
void addDups(osg::Geode *geode);
using namespace std;
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
    _fout <<"property float quality\n";

    _fout <<"end_header\n";
}
void write_all(std::ostream& _fout,osg::DrawElementsUInt *tri,osg::Vec3Array *verts,osg::Vec4Array *colors,std::vector<bool> *_marginFace,bool flip){
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
        if(!((*_marginFace)[iout[0]] == (*_marginFace)[iout[1]] && (*_marginFace)[iout[1]] == (*_marginFace)[iout[2]])){
            printf("MArgin not the saem %d %d %d %d %d %d\n",iout[0],iout[1],iout[2],(*_marginFace)[iout[0]] ,(*_marginFace)[iout[1]] ,(*_marginFace)[iout[2]] );
            exit(-1);
        }
        float qual =(*_marginFace)[iout[0]] ? -999.0 : 1.0;
        _fout.write((char*)&qual,sizeof(float));

        cnt++;

    }
    printf("%d\n",cnt);


}

int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);
    string outdump;

    // set up the usage document, in case we need to print out how to use this program.
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName() +" is the example which demonstrates Depth Peeling");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" filename");
    string outfilename="out.ply";
    arguments.read("--outfile",outfilename);
    bool flip=arguments.read("-F");

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }
    /*  if(arguments.argc() < (2+6)  ){
        fprintf(stderr,"Must pass two meshes and bbox arg must be base dir\n");
        arguments.getApplicationUsage()->write(std::cerr,osg::ApplicationUsage::COMMAND_LINE_OPTION);
        exit(-1);
    }*/

    osg::Vec3 minV,maxV;
    osg::Vec3 minVMargin,maxVMargin;

    /* minV.x() = atof(arguments[2]);
    minV.y() = atof(arguments[3]);
    minV.z() = atof(arguments[4]);

    maxV.x() = atof(arguments[5]);
    maxV.y() = atof(arguments[6]);
    maxV.z() = atof(arguments[7]);*/
    if(!arguments.read("--bbox-margin",minVMargin.x(),minVMargin.y(),minVMargin.z(),maxVMargin.x() ,maxVMargin.y() ,maxVMargin.z() )){
        fprintf(stderr,"Must pass two meshes and bbox arg must be base dir\n");
        arguments.getApplicationUsage()->write(std::cerr,osg::ApplicationUsage::COMMAND_LINE_OPTION);
        exit(-1);
    }
    if(!arguments.read("--bbox",minV.x(),minV.y(),minV.z(),maxV.x() ,maxV.y() ,maxV.z() )){
        fprintf(stderr,"Must pass two meshes and bbox margin and bbox \n");
        arguments.getApplicationUsage()->write(std::cerr,osg::ApplicationUsage::COMMAND_LINE_OPTION);
        exit(-1);
    }


    ply::VertexData vertexData;
    osg::Node *root=NULL;
    osg::BoundingBox bb(minV,maxV);
    osg::BoundingBox bbox_margin(minVMargin,maxVMargin);
    std::cout <<"Full " <<bb._min << " " << bb._max << endl;
    std::cout << "Margin " <<bbox_margin._min << " " << bbox_margin._max << endl;

    for(int pos=1;pos<arguments.argc();++pos)
    {
        if (!arguments.isOption(pos))
        {
            // not an option so assume string is a filename.
            string fname= arguments[pos];
            cout <<"Loading:"<< fname <<endl;
            root= vertexData.readPlyFile(fname.c_str(),false,&bbox_margin, DUP);


        }
    }
    string empt=outfilename+".empty";

    if(!vertexData._vertices.valid() || vertexData._vertices->size() == 0){
        fprintf(stderr,"No valid data in bbox returning %d\n",vertexData._vertices.valid() ?  vertexData._vertices->size() : -1 );
        FILE *fp=fopen(empt.c_str(),"w");
        fprintf(fp,"0\n");
        fclose(fp);
        return 0;
    }
    if(osgDB::fileExists(empt)){
        if( remove( empt.c_str() ) != 0 ){
            perror( "Error deleting empty file" );
            exit(-1);
        }
        else
            puts( "empty File successfully deleted" );
    }


    vector<bool> marginFaces(vertexData._triangles->size());
    for(int i=0; i< (int)vertexData._triangles->size()-2; i+=3){
        osg::Vec3 &v0=vertexData._vertices->at(vertexData._triangles->at(i));
        osg::Vec3 &v1=vertexData._vertices->at(vertexData._triangles->at(i+1));
        osg::Vec3 &v2=vertexData._vertices->at(vertexData._triangles->at(i+2));
        bool isMarginFace= ((!bb.contains(v0) && !bb.contains(v1) && !bb.contains(v2) ));
       // if(isMarginFace )
         //   cout << "Yes\n";
        for(int t=0; t<3; t++){
            marginFaces.at(i+t)=isMarginFace;
        }
    }

        if(osgDB::getFileExtension(outfilename) == "ply"){

        // osgUtil::SmoothingVisitor sv;
        //root->accept(sv);
        std::ofstream f(outfilename.c_str());
        bool color = vertexData._colors.valid() ? (vertexData._colors->size() >0) : false;
        write_header(f,vertexData._triangles->size(),color);
        write_all(f,vertexData._triangles,vertexData._vertices,vertexData._colors,&marginFaces,flip);
        //PLYWriterNodeVisitor nv(f);
        //root->accept(nv);
        f.close();;
    }else{

        osgUtil::SmoothingVisitor sv;
        root->accept(sv);
        osgDB::writeNodeFile(*root,outfilename);
    }

}

