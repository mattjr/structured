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
using namespace osgUtil;


class ClippedTextureAtlasBuilder : public osgUtil::Optimizer::TextureAtlasBuilder {
  
public:
  class ClippedAtlas;
  class ClippedSource : public Source {
    int minX,maxX;
    int minY,maxY;
  public:
    ClippedSource(const osg::Image* image): Source(image){}
    ClippedSource(const osg::Texture2D* texture):Source(texture){}
    osg::Matrix computeTextureMatrix() const
    {printf("Clip ctm\n");
      if (!_atlas) return osg::Matrix();
      if (!_image) return osg::Matrix();
      if (!(_atlas->_image)) return osg::Matrix();
      
      return osg::Matrix::scale(float(_image->s())/float(_atlas->_image->s()), float(_image->t())/float(_atlas->_image->t()), 1.0)*
	osg::Matrix::translate(float(_x)/float(_atlas->_image->s()), float(_y)/float(_atlas->_image->t()), 0.0);
    }
   
  };
  
  void addSource(const osg::Image* image){
    if (!getSource(image)) _sourceList.push_back(new ClippedSource(image));
    }
  
    void addSource(const osg::Texture2D* texture){
    if (!getSource(texture)) _sourceList.push_back(new ClippedSource(texture));
    }

  typedef std::vector< osg::ref_ptr<ClippedSource> > SourceList;  
  typedef std::vector< osg::ref_ptr<ClippedAtlas> > AtlasList;
  AtlasList _atlasList;  
  SourceList _sourceList;
public:
  void buildAtlas();



class ClippedAtlas: public osgUtil::Optimizer::TextureAtlasBuilder::Atlas{
public:
  typedef  std::vector< osg::BoundingBox > TexBoundsList;
  typedef std::vector< osg::ref_ptr<ClippedAtlas> > AtlasList;
  typedef std::vector< osg::ref_ptr<ClippedSource> > SourceList;   
  SourceList _sourceList; 
ClippedAtlas(unsigned int width, unsigned height, unsigned margin):Atlas(width,height,margin){} 

  void copySources();
private :
 
  
  AtlasList _atlasList;  


};
};
class ClippedTextureAtlasVisitor :public osgUtil::Optimizer::TextureAtlasVisitor{
public:
  ClippedTextureAtlasVisitor(osgUtil::Optimizer *opt) : osgUtil::Optimizer::TextureAtlasVisitor(opt){

  }
  void optimize();
  ClippedTextureAtlasBuilder& getTextureAtlasBuilder() { return _builder; }
  ClippedTextureAtlasBuilder _builder;

};


#endif
