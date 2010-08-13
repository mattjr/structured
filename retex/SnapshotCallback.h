#ifndef SNAPSHOTCALLBACK
#define SNAPSHOTCALLBACK
#include <osgUtil/Optimizer>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/WriteFile> 
#include <osgDB/Registry>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/UFOManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <iostream>
#include <osg/Material>
#include <osg/Geode>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Projection>
#include <osg/PolygonOffset>
#include <osg/MatrixTransform>
#include <osg/CameraNode>
#include <osg/FrontFace>
#include <osgGA/TrackballManipulator>
#include <osgText/Text>
#include <osg/Image>
#include <osg/io_utils>

//#include <osgGA/MatrixManipulator>
#include "adt_dataio.hpp"
#include "adt_cvutils.hpp"
#include "Path.h"
class ChangeMetricPagedLODSubgraphsVistor : public osg::NodeVisitor
{
public:
    ChangeMetricPagedLODSubgraphsVistor():
        osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
    {
    }
    
    virtual void apply(osg::PagedLOD& plod)
    {

      plods.push_back(&plod);
      
      traverse(plod);
    } 
 
  void changeToPixel(){

    for(unsigned int i=0; i< plods.size(); i++){
  
     
      oldranges.push_back(plods[i]->getRangeList());
      oldrangemode.push_back(plods[i]->getRangeMode());

      plods[i]->setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
      osg::LOD::MinMaxPair lod0(0,0);
      osg::LOD::MinMaxPair lod1(0,0);
      osg::LOD::MinMaxPair lod2(1,1e7);
      osg::LOD::RangeList pixelranges;
      pixelranges.push_back(lod0);
      pixelranges.push_back(lod1);
      pixelranges.push_back(lod2);

      plods[i]->setRangeList(pixelranges);
    }
  }
  
  void revert(){
       for(unsigned int i=0; i< plods.size(); i++){
	 plods[i]->setRangeMode(oldrangemode[i]);
	 plods[i]->setRangeList(oldranges[i]);
       }
  }
  std::vector<osg::PagedLOD *> plods;
  std::vector<osg::LOD::RangeList> oldranges;
  std::vector<osg::LOD::RangeMode> oldrangemode;
};
      

class SnapImageDrawCallback : public osg::CameraNode::DrawCallback
 {
 public:

   SnapImageDrawCallback(std::vector<osg::Matrixd> passedM,std::vector<bbox> bboxes,osgDB::DatabasePager *dp=NULL,osgViewer::Viewer *viewer=NULL,osg::Node *root=NULL);
 
   void setSnapImageOnNextFrame(int resolution);
   bool getSnapImageOnNextFrame() const;
   virtual void operator () (const osg::CameraNode& camera) const;
    void pasteTile() const;
   bool getClosestZValue(int x,int y,float &z) const;
 protected:
   // mutable  osg::ref_ptr<osg::Image> snapBuffer,tileBuffer;
  mutable std::string _filename;
   mutable  bool        _snapImageOnNextFrame;
   int totalCols,totalRows;
   int _resolution;
  mutable int tileRow,tileCol;
  mutable int takeSnapTile;
   mutable int premove;
   mutable IplImage *depthImg,*depthFrameBuffer,*snapBuffer,*tileBuffer;
  void myGluPerspective(GLdouble fovy, GLdouble aspect, GLdouble zNear, GLdouble zFar ,osg::Camera* camera) const;
   mutable double fov,aspectRatio,zNear,zFar;
   mutable bool firstFrame;
   mutable osg::Camera* self;
   enum {MOVE,UPDATE,SAVE};
   mutable int snapcount;
   osgDB::DatabasePager *_dp;
   osgViewer::Viewer *_viewer;
   double _expDelay;
   FILE *depthFP;
   

mutable   bool use_png;
   mutable bool imgAllocated;
   mutable osg::Node *_root;
   mutable bool getDepth,coarseDepthMethod;
   mutable GLfloat depthRange[2];
   mutable ChangeMetricPagedLODSubgraphsVistor cmplsv;
   mutable IplImage *texImage;
   mutable osgViewer::WindowSizeHandler *_wsh;
   mutable bool _fullscreen;
   mutable bool _batch;
   mutable std::vector<osg::Matrixd> _passedM;
   mutable int _index;
   mutable osg::Matrix mod_view;
   mutable FILE *bboxfp;
   mutable std::vector<bbox> _bboxes;
};


#endif
