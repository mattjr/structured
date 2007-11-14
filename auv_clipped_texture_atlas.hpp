#ifndef AUV_CLIPPED_TEXTURE_ATLAS
#define AUV_CLIPPED_TEXTURE_ATLAS

#include <osgUtil/Optimizer>
#include <sstream>
#include <osg/StateSet>
#include <osg/Material>
#include <osg/Texture2D>
#include <osg/Texture3D>
#include <osg/TexGen>
#include <osg/TexMat>
#include <iostream>
#include <osgDB/FileNameUtils>

#include <osg/io_utils>
using namespace osgUtil;
using namespace std;
typedef std::map<std::string,osg::BoundingBox> ClippingMap;

class ClippedTextureAtlasBuilder : public osgUtil::Optimizer::TextureAtlasBuilder {
  
public:
 
  class ClippedAtlas;
  class ClippedSource : public Source {
 

   
  public:
int sclipped,tclipped;
    ClippedSource(const osg::Image* image,osg::BoundingBox* texLimit): Source(image),_texLimit(texLimit){
      //_texLimit->set(0.0,0.0,0.0,1.0,1.0,1.0);
      
      //  _texLimit->xMin()=0.0;
      // _texLimit->xMax()=1.0;
      if(_texLimit){
      sclipped=(int)ceil(_image->s() *(_texLimit->xMax()- _texLimit->xMin() ));
      tclipped=(int)ceil(_image->t() *(_texLimit->yMax()- _texLimit->yMin() ));
      }else{
	sclipped=_image->s();
	tclipped=_image->t();
      }
 }
    ClippedSource(const osg::Texture2D* texture,osg::BoundingBox* texLimit):Source(texture),_texLimit(texLimit){
      //_texLimit->set(0.0,0.0,0.0,1.0,1.0,0.0);
      
      //_texLimit->xMin()=0.0;
      //_texLimit->xMax()=1.0;

      if(_texLimit)    {
	sclipped=(int)ceil(_image->s() *(_texLimit->xMax()- _texLimit->xMin() ));
      tclipped=(int)ceil(_image->t() *(_texLimit->yMax()- _texLimit->yMin() ));
      }else{
	sclipped=_image->s();
	tclipped=_image->t();
      }
}

    osg::Matrix computeTextureMatrix() const
    {
      if (!_atlas) return osg::Matrix();
      if (!_image) return osg::Matrix();
      if (!(_atlas->_image)) return osg::Matrix();
  
 
       osg::Matrix m=osg::Matrix::scale(float(sclipped)/float(_atlas->_image->s()), float(tclipped)/float(_atlas->_image->t()), 1.0)*
	osg::Matrix::translate(float(_x)/float(_atlas->_image->s()), float(_y)/float(_atlas->_image->t()), 0.0);
	    
      return m;
    }
    ClippedAtlas *_atlas;
    osg::BoundingBox *_texLimit;
  };
    osg::Image* getImageAtlas(unsigned int i);
            osg::Texture2D* getTextureAtlas(unsigned int i);
            osg::Matrix getTextureMatrix(unsigned int i);
  osg::Vec3d getTextureMin(const osg::Texture2D* texture);
            osg::Image* getImageAtlas(const osg::Image* image);
            osg::Texture2D* getTextureAtlas(const osg::Image* image);
            osg::Matrix getTextureMatrix(const osg::Image* image);
            
            osg::Image* getImageAtlas(const osg::Texture2D* textue);
            osg::Texture2D* getTextureAtlas(const osg::Texture2D* texture);
            osg::Matrix getTextureMatrix(const osg::Texture2D* texture);
            


  void addSource(const osg::Image* image){
    osg::BoundingBox *texLimit=NULL;
    if (!getSource(image)){
      if(_clipmap)
	texLimit=&((*_clipmap)[osgDB::getSimpleFileName(image->getFileName())]);
      _sourceList.push_back(new ClippedSource(image,texLimit));
    }
  }
  
  void addSource(const osg::Texture2D* texture){
    osg::BoundingBox *texLimit=NULL;
    const osg::Image *img=NULL;
    if (!getSource(texture)){
      img=texture->getImage();
      if(img){
	if(_clipmap){
	  texLimit=&((*_clipmap)[osgDB::getSimpleFileName(img->getFileName())]);
	}
      }
	_sourceList.push_back(new ClippedSource(texture,texLimit));
    }	
  
  }
  
  typedef std::vector< osg::ref_ptr<ClippedSource> > SourceList;  
  typedef std::vector< osg::ref_ptr<ClippedAtlas> > AtlasList;
  AtlasList _atlasList;  
  SourceList _sourceList;
public:
  void buildAtlas();
  void setClipMap(ClippingMap *clipmap){
    _clipmap=clipmap;
  }
  ClippedSource* getSource(const osg::Image* image);
  ClippedSource* getSource(const osg::Texture2D* texture);
  
  class ClippedAtlas: public osgUtil::Optimizer::TextureAtlasBuilder::Atlas{
  public:
    typedef  std::vector< osg::BoundingBox > TexBoundsList;
    typedef std::vector< osg::ref_ptr<ClippedAtlas> > AtlasList;
    typedef std::vector< osg::ref_ptr<ClippedSource> > SourceList;   
    SourceList _sourceList; 
    ClippedAtlas(unsigned int width, unsigned height, unsigned margin):Atlas(width,height,margin){} 
    bool addSource(ClippedSource* source);
    void copySources();
bool    doesSourceFit(ClippedSource* source);
  private :
 
    AtlasList _atlasList;      
  };
  ClippingMap *_clipmap;
};
class ClippedTextureAtlasVisitor:public osgUtil::Optimizer::TextureAtlasVisitor{
public:
  ClippedTextureAtlasVisitor(osgUtil::Optimizer *opt) : osgUtil::Optimizer::TextureAtlasVisitor(opt){

  }
  void optimize();
  ClippedTextureAtlasBuilder& getTextureAtlasBuilder() { return _builder; }
  ClippedTextureAtlasBuilder _builder;

};


#endif
