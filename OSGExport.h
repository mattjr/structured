#ifndef OBJ_H
#define OBJ_H

#include <string>
#include <vector>
#include <map>
#include <istream>
#include <boost/thread/once.hpp>
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
#include <boost/thread/mutex.hpp>
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
IplImage *doCvResize(osg::Image *img,int size);
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


class CompressTexturesVisitor : public osg::NodeVisitor
{
public:

  CompressTexturesVisitor(osg::Texture::InternalFormatMode internalFormatMode,int tex_size):
        osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
        _internalFormatMode(internalFormatMode),_tex_size(tex_size) {}

    virtual void apply(osg::Node& node)
    {
        if (node.getStateSet()) apply(*node.getStateSet());
        traverse(node);
    }
    
    virtual void apply(osg::Geode& node)
    {
        if (node.getStateSet()) apply(*node.getStateSet());
        
        for(unsigned int i=0;i<node.getNumDrawables();++i)
        {
            osg::Drawable* drawable = node.getDrawable(i);
            if (drawable && drawable->getStateSet()) apply(*drawable->getStateSet());
        }
        
        traverse(node);
    }
    
    virtual void apply(osg::StateSet& stateset)
    {
        // search for the existance of any texture object attributes
        for(unsigned int i=0;i<stateset.getTextureAttributeList().size();++i)
        {
            osg::Texture* texture = dynamic_cast<osg::Texture*>(stateset.getTextureAttribute(i,osg::StateAttribute::TEXTURE));
            if (texture)
            {
                _textureSet.insert(texture);
            }
        }
    }
    
    void compress()
    {
        MyGraphicsContext context;
        if (!context.valid())
        {
            osg::notify(osg::NOTICE)<<"Error: Unable to create graphis context - cannot run compression"<<std::endl;
            return;
        }

        osg::ref_ptr<osg::State> state = new osg::State;

        for(TextureSet::iterator itr=_textureSet.begin();
            itr!=_textureSet.end();
            ++itr)
        {
            osg::Texture* texture = const_cast<osg::Texture*>(itr->get());
            
            osg::Texture2D* texture2D = dynamic_cast<osg::Texture2D*>(texture);
            osg::Texture3D* texture3D = dynamic_cast<osg::Texture3D*>(texture);


            osg::ref_ptr<osg::Image> image = texture2D ? texture2D->getImage() : (texture3D ? texture3D->getImage() : 0);
            if (image.valid() && 
                (image->getPixelFormat()==GL_RGB || image->getPixelFormat()==GL_RGBA) &&
                (image->s()>=32 && image->t()>=32))
            {
	      IplImage *cvptr=NULL;
	      if(_tex_size)
		cvptr= doCvResize(image.get(),_tex_size);
	      
	      texture->setInternalFormatMode(_internalFormatMode);
	      
	      // need to disable the unref after apply, other the image could go out of scope.
	      bool unrefImageDataAfterApply = texture->getUnRefImageDataAfterApply();
	      texture->setUnRefImageDataAfterApply(false);
              
	      // get OpenGL driver to create texture from image.
	      texture->apply(*state);
	      
                // restore the original setting
                texture->setUnRefImageDataAfterApply(unrefImageDataAfterApply);

                image->readImageFromCurrentTexture(0,true);

                texture->setInternalFormatMode(osg::Texture::USE_IMAGE_DATA_FORMAT);
		if(_tex_size){
		  if(cvptr)
		    cvReleaseImage(&cvptr);
		}
            }
        }
    }
    
    typedef std::set< osg::ref_ptr<osg::Texture> > TextureSet;
    TextureSet                          _textureSet;
    osg::Texture::InternalFormatMode    _internalFormatMode;
    int _tex_size;
};

class OSGExporter 
{
public:
  OSGExporter(string prefixdir="mesh/",bool tex_saved=true,bool compress_tex=false,int num_threads=1): prefixdir(prefixdir),tex_saved(tex_saved),compress_tex(compress_tex),num_threads(num_threads) {state=NULL;
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
  osg::Image *LoadResizeSave(string filename,string outname,bool save,int tex_size);
  osg::Node *convertModelOSG(GtsSurface *s,std::map<int,string> textures,char *out_name,int tex_size) ;
~OSGExporter();
 std::map<string,IplImage *> tex_image_cache;

protected:
  osg::ref_ptr<osg::State> state;
  
  void compress(osg::Texture2D* texture2D);
  osg::Texture::InternalFormatMode internalFormatMode;
    MyGraphicsContext *context;
  bool ive_out;
  //osg::Geometry* convertGtsSurfListToGeometry(GtsSurface *s) const;  
  osg::Geode* convertGtsSurfListToGeometry(GtsSurface *s, std::map<int,string> textures,int tex_size) ;  
  bool Export3DS(GtsSurface *s,const char *c3DSFile,map<int,string> material_names,int tex_size);
  string prefixdir;
  bool tex_saved;
  bool compress_tex;

  int num_threads;
  
  vector<osg::ref_ptr<osg::Texture2D> > osg_tex_ptrs;
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
enum {IVE_OUT,OSG_OUT,THREEDS_OUT};



typedef std::map<int,GeometryCollection> MaterialToGeometryCollectionMap;
typedef std::map<int,string> MaterialToIDMap;
void gen_mesh_tex_coord(GtsSurface *s ,Camera_Calib *calib, std::map<int,GtsMatrix *> back_trans,GNode *bboxTree,int tex_size,int num_threads);
osg::Image* Convert_OpenCV_TO_OSG_IMAGE(IplImage* cvImg,bool flip=true);
#endif
