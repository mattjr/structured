#ifndef TEXPYRATLAS_H
#define TEXPYRATLAS_H
#include <osgUtil/Optimizer>
#include <string>
#include <osg/State>
class TexPyrAtlas
{
public:
    TexPyrAtlas(std::string imgdir);
    void loadSources(std::vector<std::string> imageList);
    osg::ref_ptr<osg::Image> getImage(int index,int sizeIndex);

protected:
     osgUtil::Optimizer::TextureAtlasBuilder _tb;
     std::vector<int> _downsampleSizes;
   osg::State *_state;
   std::vector<osg::ref_ptr<osg::Image> > _images;
   bool
   resizeImage(const osg::Image* input,
                           unsigned int out_s, unsigned int out_t,
                           osg::ref_ptr<osg::Image>& output,
                           unsigned int mipmapLevel );
   std::string _imgdir;
};

#endif // TEXPYRATLAS_H
