#include "TexPyrAtlas.h"
#include <osgDB/ReadFile>
#include <vpb/TextureUtils>
TexPyrAtlas::TexPyrAtlas()
{
    _tb.setMaximumAtlasSize(4096,4096);
    _downsampleSizes.clear();
    _downsampleSizes.push_back(512);
    _downsampleSizes.push_back(256);
    _downsampleSizes.push_back(32);
    _state = new osg::State;


}

void TexPyrAtlas::loadSources(std::vector<std::string> imageList){
    for(int i=0; i< imageList.size(); i++){
        osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
        osg::ref_ptr<osg::Image> image=osgDB::readImageFile(imageList[i]);
        texture->setImage(image);
        bool resizePowerOfTwo=true;
        vpb::generateMipMap(*_state,*texture,resizePowerOfTwo,vpb::BuildOptions::NVTT);
    }

}
