#include "PosterPrinter.h"
#include <osg/ArgumentParser>
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/io_utils>
#include <osg/Texture2D>
#include <osg/TriangleIndexFunctor>
#include <osg/TriangleFunctor>


int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);
    osg::Node* model = osgDB::readNodeFile(argv[1]);
    //osg::Geode *newGeode= convertModel(model->asGroup());
    //osgDB::writeNodeFile(*newGeode,"test.ive");


}
