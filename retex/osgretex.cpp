/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield 
 *
 * This application is open source and may be redistributed and/or modified   
 * freely and without restriction, both in commericial and non commericial applications,
 * as long as this copyright notice is maintained.
 * 
 * This application is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>


#include <iostream>

#include "SnapshotCallback.h"
#include "Path.h"

using namespace std;

SnapImageDrawCallback* snapImageDrawCallback;

void argConvGLcpara2( double cparam[3][4], int width, int height, double gnear, double gfar, double m[16] );

int main(int argc, char** argv)
{
  string p;
  bool aa;
  // use an ArgumentParser object to manage the program arguments.
  osg::ArgumentParser arguments(&argc,argv);
  arguments.read("-pathfile",p);
  aa=arguments.read("-aa");
  if(aa){
    printf("AntiAliasing enabled\n");
    osg::DisplaySettings * ds = osg::DisplaySettings::instance();
    ds->setNumMultiSamples(16); 
  }
  
  osgViewer::Viewer viewer(arguments);
  // left window + left slave camera
  
  // report any errors if they have occurred when parsing the program arguments.
  if (arguments.errors()){
    arguments.writeErrorMessages(std::cout);
    return 1;
  }
  
  if (arguments.argc()<=1){
    arguments.getApplicationUsage()->write(std::cout,osg::ApplicationUsage::COMMAND_LINE_OPTION);
    return 1;
  }
  
  osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
  traits->x =  0;
  traits->y =  0;
  traits->width = 680;
  traits->height = 512;
  traits->windowDecoration = true;
  traits->doubleBuffer = true;
  traits->sharedContext = 0;
  
  osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
  
  osg::ref_ptr<osg::Camera> camera = viewer.getCamera();
  
  
  camera->setGraphicsContext(gc.get());
  camera->setViewport(new osg::Viewport(0,0, traits->width, traits->height));
  GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
  camera->setDrawBuffer(buffer);
  camera->setReadBuffer(buffer);
  
  double cparam[3][4] = {{1714.139241/2.0, 0.000000, -687.347717/2.0, 0.0000 },
			 {0,	 1713.360969/2.0, -499.609809/2.0 ,	0},
			 {0,		0,		-1,		0}};
  double intrinsic[16];
  
  argConvGLcpara2( cparam, 680, 512, 0.1, 2000, intrinsic );
  camera->setProjectionMatrix(osg::Matrix(intrinsic));
  camera->setClearColor(osg::Vec4(0.0,0.0,0.0,0.0));
  PathLoader *pLoad = new PathLoader();

 
   std::vector<osg::Matrixd> cameraPath=  pLoad->createPath(p.c_str());


 
    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
    
    // add the thread model handler
    viewer.addEventHandler(new osgViewer::ThreadingHandler);

    // add the window size toggle handler
    
        
    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    // add the help handler
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    // add the record camera path handler
    viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);

    // add the LOD Scale handler
    viewer.addEventHandler(new osgViewer::LODScaleHandler);

    
  
    
    

    // load the data
    osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFiles(arguments);
    if (!loadedModel) 
    {
        std::cout << arguments.getApplicationName() <<": No data loaded" << std::endl;
        return 1;
    }

    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }

  

    osgDB::DatabasePager *dp=viewer.getScene()->getDatabasePager();
    snapImageDrawCallback = new SnapImageDrawCallback(cameraPath,dp,&viewer,loadedModel.get()); 
    
    viewer.getCamera()->setPostDrawCallback(snapImageDrawCallback);
    // optimize the scene graph, remove redundant nodes and state etc.
    osgUtil::Optimizer optimizer;
    optimizer.optimize(loadedModel.get());

    viewer.setSceneData( loadedModel.get() );

    viewer.realize();
    snapImageDrawCallback->setSnapImageOnNextFrame(1);
    return viewer.run();

}
