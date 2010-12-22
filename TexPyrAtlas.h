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

   /**
          * Resizes an image using nearest-neighbor resampling. Returns a new image, leaving
          * the input image unaltered.
          *
          * Note. If the output parameter is NULL, this method will allocate a new image and
          * resize into that new image. If the output parameter is non-NULL, this method will
          * assume that the output image is already allocated to the proper size, and will
          * do a resize+copy into that image. In the latter case, it is your responsibility
          * to make sure the output image is allocated to the proper size and with the proper
          * pixel configuration (it must match that of the input image).
          *
          * If the output parameter is non-NULL, then the mipmapLevel is also considered.
          * This lets you resize directly into a particular mipmap level of the output image.
          */
   bool
   resizeImage(const osg::Image* input,
                           unsigned int out_s, unsigned int out_t,
                           osg::ref_ptr<osg::Image>& output,
                           unsigned int mipmapLevel=0 );
   std::string _imgdir;
};

#endif // TEXPYRATLAS_H
