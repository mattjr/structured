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


int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);
    string outdump;

    // set up the usage document, in case we need to print out how to use this program.
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName() +" is the example which demonstrates Depth Peeling");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" filename");
    string outfilename="out.pcd";
    arguments.read("--outfile",outfilename);

    bool flip=arguments.read("-F");

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }


    ply::VertexData vertexData;
    osg::Node *root=NULL;

    FILE *fp2 =fopen(argv[1],"r");

    /*for(int pos=1;pos<arguments.argc();++pos)
    {
        if (!arguments.isOption(pos))
        {
            // not an option so assume string is a filename.
            string fname= arguments[pos];*/
    string inp;
    if(arguments.read("-input",inp)){
        root= vertexData.readPlyFile(inp.c_str(),false,NULL, DUP);

    }else{
        while(!feof(fp2)){
            char tmp[1024];
            fscanf(fp2,"%s\n",tmp);
            cout <<"Loading:"<< tmp <<endl;
            root= vertexData.readPlyFile(tmp,false,NULL, DUP);



        }
        fclose(fp2);;

    }
    osg::Vec3Array *norms= new osg::Vec3Array;
    vertexData.calcNorms(norms);
    FILE *fp=fopen(outfilename.c_str(),"w");
    fprintf(fp,"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\n");
    fprintf(fp,"FIELDS x y z normal_x normal_y normal_z\n");
    fprintf(fp,"SIZE 4 4 4 4 4 4\n");
    fprintf(fp,"TYPE F F F F F F\n");
    fprintf(fp,"COUNT 1 1 1 1 1 1\n");
    fprintf(fp,"WIDTH %d\n",norms->size());
    fprintf(fp,"HEIGHT 1\n");
    fprintf(fp,"VIEWPOINT 0 0 0 1 0 0 0\n");
    fprintf(fp,"POINTS %d\n",norms->size());
    fprintf(fp,"DATA binary\n");


    if(norms->size() != vertexData._vertices->size()){
        fprintf(stderr,"Failed to generate normals\n");
    }
    float d[6];
    for(int i=0; i< (int)vertexData._triangles->size()-2; i+=3){

        for(int j=0; j<3; j++){
            osg::Vec3 &v0=vertexData._vertices->at(vertexData._triangles->at(i+j));
            osg::Vec3 &n0=norms->at(vertexData._triangles->at(i+j));
            d[0]=v0[0];
            d[1]=v0[1];
            d[2]=v0[2];
            d[3]=n0[0];
            d[4]=n0[1];
            d[5]=n0[2];
            fwrite(d,6,sizeof(float),fp);

        }
    }

    fclose(fp);

}


