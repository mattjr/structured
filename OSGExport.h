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
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include "auv_clipped_texture_atlas.hpp"
using namespace std;
using namespace libsnapper;
using namespace libpolyp;
bool FileExists(string strFilename);
IplImage *doCvResize(osg::Image *img,int size);
#if ((OSG_VERSION_MAJOR==2))
#include <osgViewer/Viewer>
typedef  boost::function<bool( int ,int)> VerboseMeshFunc;

class MyGraphicsContext {
    public:
  MyGraphicsContext(); 
        bool valid() const { return _gc.valid() && _gc->isRealized(); }
        
        osg::GraphicsContext *getContext( void )
        {
           return _gc.get();
        }

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
extern MyGraphicsContext *mgc;
#if 0
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
#endif
class OSGExporter 
{
public:
  OSGExporter(string prefixdir="mesh/",bool tex_saved=true,bool compress_tex=false,int num_threads=1,int verbose=0,bool hardware_compress=true): prefixdir(prefixdir),tex_saved(tex_saved),compress_tex(compress_tex),num_threads(num_threads),verbose(verbose),_hardware_compress(hardware_compress) {state=NULL;
    do_atlas=false;
    context=NULL;
    internalFormatMode=osg::Texture::USE_IMAGE_DATA_FORMAT;
    if(compress_tex && _hardware_compress){
      
      if(verbose)
	printf("Compressing Textures\n");
      if(!mgc)
	mgc=new MyGraphicsContext();
      context=mgc;
      
      if(!context){
	printf("Can't use OPENGL without valid context");
	exit(0);
      }


      internalFormatMode=osg::Texture::USE_S3TC_DXT1_COMPRESSION; 

    }
  }
  osg::Image *LoadResizeSave(string filename,string outname,bool save,int tex_size);
  osg::ref_ptr< osg::Group>convertModelOSG(GtsSurface *s,std::map<int,string> textures,char *out_name,int tex_size,VerboseMeshFunc vmcallback=NULL,float *zrange=NULL) ;
~OSGExporter();
 std::map<string,IplImage *> tex_image_cache;
  std::map<string,osg::Image *> compressed_img_cache;
  osg::Image *getCachedCompressedImage(string name,int size);
  osg::ref_ptr<osg::Image>cacheCompressedImage(IplImage *img,string name,int tex_size);
protected:
  osg::ref_ptr<osg::State> state;
 
  void compress(osg::Texture2D* texture2D);
  osg::Texture::InternalFormatMode internalFormatMode;
    MyGraphicsContext *context;
  bool ive_out;
  
  osg::ref_ptr<osg::Group> convertGtsSurfListToGeometry(GtsSurface *s, std::map<int,string> textures,ClippingMap *cm,int tex_size,VerboseMeshFunc vmcallback=NULL,float *zrange=NULL) ;  
  bool Export3DS(GtsSurface *s,const char *c3DSFile,map<int,string> material_names,int tex_size,VerboseMeshFunc vmcallback=NULL);
  string prefixdir;
  bool tex_saved;
  bool compress_tex;
  bool do_atlas;
  int num_threads;
  int verbose;
  bool _hardware_compress;
  vector<osg::ref_ptr<osg::Texture2D> > osg_tex_ptrs;
  
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
  bool                        _colorsActive;
  osg::Vec2Array::iterator    _texcoords;
  int                         _coordCount;
  osg::Geometry*              _geom;
  osg::BoundingBox            _texLimits;

};
enum {IVE_OUT,OSG_OUT,THREEDS_OUT};



typedef std::map<int,GeometryCollection> MaterialToGeometryCollectionMap;
typedef std::map<int,string> MaterialToIDMap;
void gen_mesh_tex_coord(GtsSurface *s ,Camera_Calib *calib, std::map<int,GtsMatrix *> back_trans,GNode *bboxTree,int tex_size,int num_threads,int verbose=0);
osg::Image* Convert_OpenCV_TO_OSG_IMAGE(IplImage* cvImg,bool flip=true,bool compress=false);

void genPagedLod(vector< osg::ref_ptr <osg::Group> > nodes,vector < vector< vector<string > > > lodnames);
osg::Node *create_paged_lod(osg::Node * model,vector<string> lod_file_names);
class CheckVisitor : public osg::NodeVisitor
{
public:
    CheckVisitor():
        osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
    {
    }
    
    virtual void apply(osg::PagedLOD& plod)
    {
        std::cout<<"PagedLOD "<<plod.getName()<<"  numRanges = "<< plod.getNumRanges()<<"  numFiles = "<<plod.getNumFileNames()<<std::endl;
        for(unsigned int i=0;i<plod.getNumFileNames();++i)
        {
            std::cout<<"  files = '"<<plod.getFileName(i)<<"'"<<std::endl;
        }
    }    
};
#endif
