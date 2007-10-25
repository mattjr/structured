#ifndef OBJ_H
#define OBJ_H

#include <string>
#include <vector>
#include <map>
#include <istream>

#include <osg/ref_ptr>
#include <osg/Referenced>
#include <osg/Vec2>
#include <osg/Vec3>
#include <osg/Vec4>

#include <osgDB/ReaderWriter>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/Material>
#include <osg/Texture2D>
#include <osg/Texture3D>
#include <osg/TexGen>
#include <osg/TexMat>

#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osg/Notify>
#include <osg/Node>
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osg/Version>
#include "GraphicsWindowCarbon.h"
#include "PixelBufferCarbon.h"
#include "auv_mesh.hpp"
#include "auv_stereo_geometry.hpp"
#define USE_LIB3DS
#ifdef USE_LIB3DS
#include <lib3ds/file.h>
#include <lib3ds/mesh.h>
#include <lib3ds/material.h>
#include <lib3ds/light.h>
#include <lib3ds/camera.h>
#include <lib3ds/node.h>
#include <lib3ds/matrix.h>
#include <lib3ds/vector.h>
#endif
using namespace std;
using namespace libsnapper;
#if ((OSG_VERSION_MAJOR==2))
#include <osgViewer/Viewer>


class MyGraphicsContext {
    public:
        MyGraphicsContext()
        {
            osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
            traits->x = 0;
            traits->y = 0;
            traits->width = 1;
            traits->height = 1;
            traits->windowDecoration = false;
            traits->doubleBuffer = false;
            traits->sharedContext = 0;
	    traits->pbuffer = true;
#ifdef __APPLE__
	    _gw= new osgViewer::GraphicsWindowCarbon(traits.get()); 

#else
            _gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	  
#endif


            if (!_gc)
            {
                osg::notify(osg::NOTICE)<<"Failed to create pbuffer, failing back to normal graphics window."<<std::endl;
                
                traits->pbuffer = false;
                _gc = osg::GraphicsContext::createGraphicsContext(traits.get());
            }

            if (_gc.valid()) 
            
            
            {
                _gc->realize();
                _gc->makeCurrent();
                //std::cout<<"Realized window"<<std::endl;
            }else{
	      printf("Can't realize window\n");
	      exit(0);
	    }
        }
        
        bool valid() const { return _gc.valid() && _gc->isRealized(); }
        
    private:
        osg::ref_ptr<osg::GraphicsContext> _gc;

#ifdef __APPLE__
  osg::ref_ptr<osgViewer::GraphicsWindowCarbon> _gw;
#endif

};

#else

#include <osgProducer/Viewer>
class MyGraphicsContext {
    public:
        MyGraphicsContext()
        {
            rs = new Producer::RenderSurface;
            rs->setWindowRectangle(0,0,1,1);
            rs->useBorder(false);
            rs->useConfigEventThread(false);
            rs->realize();
            std::cout<<"Realized window"<<std::endl;
        }

        virtual ~MyGraphicsContext()
        {
        }
        
    private:
        Producer::ref_ptr<Producer::RenderSurface> rs;
};

#endif

class OSGExporter 
{
public:
  OSGExporter(string prefixdir="mesh/",bool tex_saved=true,bool compress_tex=false,int tex_size=512,int num_threads=1): prefixdir(prefixdir),tex_saved(tex_saved),compress_tex(compress_tex),tex_size(tex_size),num_threads(num_threads) {state=NULL;
    context=NULL;
    context=new MyGraphicsContext();
    internalFormatMode=osg::Texture::USE_IMAGE_DATA_FORMAT;
    if(compress_tex){
      printf("Compressing Textures\n");
#ifndef __APPLE__
      internalFormatMode=osg::Texture::USE_S3TC_DXT5_COMPRESSION; 
#else
      internalFormatMode=osg::Texture::USE_S3TC_DXT1_COMPRESSION; 
#endif
    }
  }
  osg::Image *LoadResizeSave(string filename,string outname,bool save,IplImage *cvImg);
  int convertModelOSG(GtsSurface *s,std::map<int,string> textures,std::string fileNameOut) ;

protected:
  osg::ref_ptr<osg::State> state;

  void compress(osg::Texture2D* texture2D);
  osg::Texture::InternalFormatMode internalFormatMode;
    MyGraphicsContext *context;
  bool ive_out;
  //osg::Geometry* convertGtsSurfListToGeometry(GtsSurface *s) const;  
  osg::Geode* convertGtsSurfListToGeometry(GtsSurface *s, std::map<int,string> textures) ;  
  bool Export3DS(GtsSurface *s,const char *c3DSFile,map<int,string> material_names);
  string prefixdir;
  bool tex_saved;
  bool compress_tex;
 int tex_size;
  int num_threads;
  /*
    inline osg::Vec3 transformVertex(const osg::Vec3& vec, const bool rotate) const ;
    inline osg::Vec3 transformNormal(const osg::Vec3& vec, const bool rotate) const ;
    
    bool _fixBlackMaterials;
  */
};

// collect all the data relavent to a particular osg::Geometry being created.
struct GeometryCollection
{
    GeometryCollection():
        _numPrimitives(0),
        _numPrimitivesWithTexCoords(0),
        _numPoints(0),
        _texturesActive(false),
        _vertices(osg::Vec3Array::iterator()),
        _texcoords(osg::Vec2Array::iterator()),
        _coordCount(0),
        _geom(0) {}
  
  int                         _numPrimitives;
  int                         _numPrimitivesWithTexCoords;
  int                         _numPoints;
  bool                        _texturesActive;
  osg::Vec3Array::iterator    _vertices;
  osg::Vec4Array::iterator    _colors;
  osg::Vec2Array::iterator    _texcoords;
  int                         _coordCount;
  osg::Geometry*              _geom;
};


typedef std::map<int,GeometryCollection> MaterialToGeometryCollectionMap;
typedef std::map<int,string> MaterialToIDMap;
void gen_mesh_tex_coord(GtsSurface *s ,Camera_Calib *calib, std::map<int,GtsMatrix *> back_trans,GNode *bboxTree,int tex_size,int num_threads);
#endif
