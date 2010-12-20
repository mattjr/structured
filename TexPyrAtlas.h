#ifndef TEXPYRATLAS_H
#define TEXPYRATLAS_H
#include <osgUtil/Optimizer>
#include <string>
#include <osg/State>
class TexPyrAtlas
{
public:
    TexPyrAtlas();
    void loadSources(std::vector<std::string> imageList);
protected:
     osgUtil::Optimizer::TextureAtlasBuilder _tb;
     std::vector<int> _downsampleSizes;
   osg::State *_state;

};

#endif // TEXPYRATLAS_H
