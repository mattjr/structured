#include "imageNode.h"
#include <osg/ArgumentParser>
#include <iostream>
#include <osgDB/ReadFile>
/* The main entry */
int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );
    osg::Node *scene=osgDB::readNodeFile( argv[1] );
    std::vector<osg::Matrixd> views;
    std::vector<osg::Matrixd> projs;
    std::vector<std::string> fnames;
    std::ifstream file(argv[2]);
    int i=0;
    char tmp[1024];
    while(!file.eof()){
        osg::Matrixd view,proj;
        readCameraMatrix(file,view,proj);
        projs.push_back(proj);
        views.push_back(view);
        sprintf(tmp,"%d.png",i++);
        fnames.push_back(tmp);
    }
    osg::Vec4 sizes(512,512,512,512);

    renderAll(scene,views,projs,fnames,sizes);

}
