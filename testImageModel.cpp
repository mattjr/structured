#include "PosterPrinter.h"
#include <osg/ArgumentParser>
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);
    osg::ref_ptr<osg::Image> image;
    osg::Matrix trans(   osg::Matrix::rotate(osg::inDegrees(-90.0f),
                                             1.0f,0.0f,0.0f)*
                         osg::Matrix::rotate(osg::inDegrees(-90.0f),0.0f,
                                             1.0f,0.0f));


    osg::ref_ptr<osg::MatrixTransform> positioned1 = new osg::MatrixTransform(trans);
    osg::Node* model = osgDB::readNodeFile(argv[1]);
  //  positioned1->addChild(model);
    render(model,image);
    osgDB::Registry::instance()->writeImage( *image,"ass.png",NULL);


}
