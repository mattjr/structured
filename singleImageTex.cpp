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
#include "TexturedSource.h"
#include "TexturingQuery.h"
#include "Extents.h"
#include "PLYWriterNodeVisitor.h"
using namespace std;
/*int write_all(std::string basename,osg::DrawElementsUInt *tri,osg::Vec3Array *verts,osg::Vec3Array *norms,osg::Vec3Array *tex){
    +    int num_models=0;
    +    int  facesLeft=tri->size();
    +    int triStart=0;
    +    int triEnd=0;
    +    int maxVerts= (0xffff-1);
    +    while(triEnd<facesLeft){
    +        printf("Tri end %d %d\n",triEnd,facesLeft);
    +        char tmp[1024];
    +        sprintf(tmp,"%s-%04d.obj",basename.c_str(),num_models);
    +        std::ofstream _fout(tmp);
    +        std::set<int> seenVert;
    +        std::vector<int> backmap;
    +        std::map<int,int> frontmap;

    +      //  std::vector<int> seenVert;
    +
    +        int vertIdx=0;
    +        triEnd=triStart;
    +        for(int i=triStart; i< (int)tri->size()-2 && seenVert.size() < (maxVerts-2); i+=3,triEnd+=3){
    +            for(int j=0; j<3; j++){
    +                if(seenVert.count(tri->at(i+j)) == 0){
    +                    seenVert.insert(tri->at(i+j));
    +                    frontmap.insert(std::make_pair<int,int>(tri->at(i+j),backmap.size()));
    +
    +                    backmap.push_back(tri->at(i+j));
    +
    +               }
    +            }
    +        }
    +    //    printf("seenVert %d\n",(int)seenVert.size());
    +
    +        std::vector<int>::iterator itr;
    +        for(itr= backmap.begin(); itr!=backmap.end(); itr++){
    +            osg::Vec3 v=verts->at(*itr);
    +            _fout<< "v "<<v<<endl;
    +        }
    +
    +        for(itr= backmap.begin(); itr!=backmap.end(); itr++){
    +            osg::Vec3 vn=norms->at(*itr);
    +            _fout<< "vn "<<vn<<endl;
    +        }
    +        for(itr= backmap.begin(); itr!=backmap.end(); itr++){
    +            osg::Vec2 vt(tex->at(*itr)[0],tex->at(*itr)[1]);
    +            _fout<< "vt "<<vt<<endl;
    +        }
    +            for(int i=triStart; i< triEnd-2; i+=3){
    +                _fout << "f ";
    +                for(int j=0; j<3; j++){
    +                    int id=frontmap[tri->at(i+j)]+1;
    +                    _fout<<id<<"/"<<id<<"/"<<id<<" ";
    +                }
    +                _fout<<endl;
    +
    +            }

    +        triStart=triEnd;
    +        num_models++;
    +       // printf("Tri end %d %d\n",triEnd,facesLeft);
    +
    +    }
    +    return num_models;
    +}
*/
int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    // set up the usage document, in case we need to print out how to use this program.
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName() +" is the example which demonstrates Depth Peeling");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" filename");


    string outfilename;
    if(!arguments.read("--outfile",outfilename)){
        fprintf(stderr,"Need outfile name\n");
        return -1;
    }
    string extraOutput;
    arguments.read("--extra",extraOutput);

    bool debug=false;
    osg::Vec2 size;
    arguments.read("--size",size.x(),size.y());
    std::string mf=argv[1];
    osg::Matrixd viewProj,rotMat;
    readMatrixToScreen("viewproj.mat",viewProj);
    readMatrix("rot.mat",rotMat);


    osg::Matrix bottomLeftToTopLeft;//= (osg::Matrix::scale(1,-1,1)*osg::Matrix::translate(0,size.y(),0));

    osg::Matrix toTex=viewProj*( osg::Matrix::translate(1.0,1.0,1.0)*osg::Matrix::scale(0.5*size.x(),0.5*size.y(),0.5f))*bottomLeftToTopLeft;
    std::vector<osg::ref_ptr<osg::Vec3Array> > coordsArray;
    osg::Node* model = osgDB::readNodeFile(mf);
    if (model)
    {


        osgUtil::Optimizer::MergeGeodesVisitor visitor;

        model->accept(visitor);
        osgUtil::Optimizer::MergeGeometryVisitor mgv;
        mgv.setTargetMaximumNumberOfVertices(INT_MAX);
        model->accept(mgv);

        GeometryCollector gc(NULL, osgUtil::Optimizer::DEFAULT_OPTIMIZATIONS);
        model->accept(gc);
        GeometryCollector::GeometryList geomList = gc.getGeometryList();

        if(geomList.size() != 1){
             osg::notify(osg::ALWAYS)  << "Number of collected geometies " << geomList.size() << "problem "<<endl;
            //   OSG_FATAL << "Number of collected geometies " << geomList.size() << "problem "<<endl;
            assert(0);

        }
        osg::Vec3Array *verts=static_cast< osg::Vec3Array*>((*geomList.begin())->getVertexArray());
        if(!verts){
            fprintf(stderr,"Fail to convert verts\n");
            exit(-1);
        }
        osg::Vec3Array *coords=new osg::Vec3Array(verts->size());
        coordsArray.push_back(coords);
        (*geomList.begin())->setTexCoordArray(0,coords);
#pragma omp for
        for(int j=0; j< (int)verts->size(); j++){
            osg::Vec4 pt(verts->at(j)[0],verts->at(j)[1],verts->at(j)[2],1.0);
            osg::Vec4 rotpt=pt*rotMat;
            osg::Vec4 proj=rotpt*toTex;
            proj.x() /= proj.w();
            proj.y() /= proj.w();
            proj.x() /= size.x();;
            proj.y() /= size.y();
            coords->at(j)=osg::Vec3(proj.x(),proj.y(),0);
        }



        if(debug){

            // set up the texture state.
            osg::Texture2D* texture = new osg::Texture2D;
            texture->setDataVariance(osg::Object::DYNAMIC); // protect from being optimized away as static state.
            texture->setImage(osgDB::readImageFile("tmp.jpg"));

            osg::StateSet* stateset = (*geomList.begin())->getOrCreateStateSet();
            stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
            osgDB::writeNodeFile(*model,osgDB::getNameLessExtension(outfilename).append(".ive"));

        }else{
            if(osgDB::getFileExtension(outfilename) == "ply"){
                std::ofstream f(outfilename.c_str());

                PLYWriterNodeVisitor nv(f,NULL,&(coordsArray));
                model->accept(nv);
                if(extraOutput.length()){
                    osgDB::writeNodeFile(*model,extraOutput);
                }
            }else{
                osgDB::writeNodeFile(*model,outfilename);
            }

        }


    }else
        cerr << "Failed to open "<<mf <<endl;

}
