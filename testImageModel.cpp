#include "PosterPrinter.h"
#include <osg/ArgumentParser>
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/io_utils>
#include <osg/Texture2D>
#include <osg/TriangleIndexFunctor>
#include <osg/TriangleFunctor>
#include <osg/ComputeBoundsVisitor>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osg/Texture3D>
#include <iostream>
#include "GLImaging.h"
#include <osgGA/TrackballManipulator>

int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);
    if (true) {
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits();

        if (viewer.getCamera()->getGraphicsContext() && viewer.getCamera()->getGraphicsContext()->getTraits()) {
            //use viewer settings for window size
            osg::ref_ptr<const osg::GraphicsContext::Traits> src_traits = viewer.getCamera()->getGraphicsContext()->getTraits();
            traits->screenNum = src_traits->screenNum;
            traits->displayNum = src_traits->displayNum;
            traits->hostName = src_traits->hostName;
            traits->width = src_traits->width;
            traits->height = src_traits->height;
            traits->red = src_traits->red;
            traits->green = src_traits->green;
            traits->blue = src_traits->blue;
            traits->alpha = src_traits->alpha;
            traits->depth = src_traits->depth;
            traits->pbuffer = true;
        } else {
            //viewer would use fullscreen size (unknown here) pbuffer will use 4096 x4096 (or best avaiable)
            traits->width = 1 << 12;
            traits->height = 1 << 12;
            traits->pbuffer = true;
        }
        osg::ref_ptr<osg::GraphicsContext> pbuffer = osg::GraphicsContext::createGraphicsContext(traits.get());
        if (pbuffer.valid())
        {
            osg::notify(osg::NOTICE)<<"Pixel buffer has been created successfully."<<std::endl;
            osg::ref_ptr<osg::Camera> camera = new osg::Camera(*viewer.getCamera());
            camera->setGraphicsContext(pbuffer.get());
            camera->setViewport(new osg::Viewport(0,0,traits->width,traits->height));
            GLenum buffer = pbuffer->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
            camera->setDrawBuffer(buffer);
            camera->setReadBuffer(buffer);
            viewer.setCamera(camera.get());
        }
        else
        {
            osg::notify(osg::NOTICE)<<"Pixel buffer has not been created successfully."<<std::endl;
        }
    }

    osg::Node* model = osgDB::readNodeFiles(arguments);
    osg::ref_ptr<osg::PixelBufferObject> pboWriteBuffer=new osg::PixelBufferObject;
    osg::ref_ptr<OptFormatTexturesVisitor> pbotv=new OptFormatTexturesVisitor();
    model->accept(*pbotv.get());
    pbotv->opt();
    //osg::Geode *newGeode= convertModel(model->asGroup());
    //osgDB::writeNodeFile(*newGeode,"test.ive");
    osgGA::TrackballManipulator *tb=new osgGA::TrackballManipulator() ;
#if 0
    osg::ComputeBoundsVisitor cbbv(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
    model->traverse(cbbv);
    osg::BoundingBox totalbb = cbbv.getBoundingBox();
    std::cout << totalbb._max << "\n";
    osg::BoundingSphere bs=model->getBound();
    osg::Vec3d eye(bs.center()+osg::Vec3(0,0,3.5*bs.radius()));

    osg::Matrixd matrix;
    matrix.makeTranslate( eye );
    //osg::Matrix trans(osg::Matrix::rotate(osg::inDegrees(180.0f),0.0f,0.0f,1.0f));
    // matrix.preMult(trans);
    //  osg::Matrixd     view = osg::Matrixd::lookAt(bs.center(), osg::Vec3(0.0f,0.0f,0.0f), osg::Vec3(0.0,0.0,1.0));
    //  view.makeLookAt(bs.center()-osg::Vec3(0.0,0,2.0f*bs.radius()),bs.center(),osg::Vec3(1.0f,0.0f,0.0f));
    // osg::Matrixd     view ;
    // double dist = 3.5f * bs.radius();
    //  view.makeLookAt(bs.center() + osg::Vec3d(0.0,-dist,0.0f),
    //                         bs.center(),
    //                         osg::Vec3d(0.0f,0.0f,1.0f));

    osg::Matrix view=osg::Matrix::inverse(matrix);
    // osg::Vec3 centeredMin,centeredMax;

    // centeredMin=(totalbb._min-totalbb.center());
    // centeredMax=(totalbb._max-totalbb.center());
    // osg::Matrixd proj= osg::Matrixd::ortho2D(centeredMin[0],centeredMax[0],centeredMin[1],centeredMax[1]);
    osg::Matrixd proj= osg::Matrixd::ortho2D(-bs.radius(),bs.radius(),-bs.radius(),bs.radius());//centeredMin[1],centeredMax[1],centeredMin[0],centeredMax[0]);
#endif
    viewer.setCameraManipulator(tb);
    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    // add the thread model handler
    //viewer.addEventHandler(new osgViewer::ThreadingHandler);

    // add the window size toggle handler
    //viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    viewer.setSceneData(model);
    //viewer.getCamera()->setProjectionMatrix(proj);
    //viewer.getCamera()->setViewMatrix(view);
    return viewer.run();

    osg::BoundingSphere bs=model->getBound();
    std::cout << "Bounding " << bs.center()<<"\n";
    /*osg::ComputeBoundsVisitor cbbv(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
model->traverse(cbbv);
osg::BoundingBox totalbb = cbbv.getBoundingBox();
std::cout << totalbb._min << " "<<totalbb._max<<std::endl;
double dist = 3.5f * bs.radius();
viewer.realize();
osg::Vec3d eye(bs.center()+osg::Vec3(0,0,3.5*bs.radius()));

osg::Matrixd matrix;
matrix.makeTranslate( eye );
osg::Matrixd mat=osg::Matrixd::ortho2D(-bs.radius(),bs.radius(),-bs.radius(),bs.radius());
//osg::Matrixd view=osg::Matrixd::lookAt(bs.center(), bs.center() +osg::Vec3d(0.0,0.0f,dist), osg::Vec3(1.0,0.0,0.0));

//viewer.getCamera()->setProjectionMatrixAsOrtho2D(-bs.radius(),bs.radius(),-bs.radius(),bs.radius());
//viewer.getCamera()->setViewMatrixAsLookAt(bs.center(), bs.center() +osg::Vec3d(0.0,0.0f,dist), osg::Vec3(1.0,0.0,0.0));
viewer.getCamera()->setViewMatrix(osg::Matrix::inverse(matrix));
viewer.getCamera()->setProjectionMatrix(mat);
std::cout << osg::Matrix::inverse(matrix) << " "<< mat<<std::endl;

while(!viewer.done()){
viewer.frame();

}*/
    //while(!viewer.done()){
    osg::Timer_t beginRender = osg::Timer::instance()->tick();

    viewer.frame();
    viewer.advance();
    viewer.updateTraversal();
    viewer.renderingTraversals();
    osg::Timer_t timeEndRender = osg::Timer::instance()->tick();
    double renderTime = osg::Timer::instance()->delta_s(beginRender, timeEndRender);
    printf("Timing Rendering %f\n",renderTime);
    //viewer.frame();
    //std::cout << "Distance: "<< tb->getDistance()<<std::endl;

    //}
    //viewer.run();
}
