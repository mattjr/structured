#ifndef TightFit_H
#define TightFit_H
#include <osgUtil/Optimizer>
#include <string>
#include <osg/State>
#include <osg/Referenced>
#include <vips/vips>
class TightFitAtlasBuilder : public osgUtil::Optimizer::TextureAtlasBuilder , public osg::Referenced
{
public:

    TightFitAtlasBuilder(int mosaic_cell_num);


    // void addSource(vips::VImage &img);
    void buildAtlas(void);
    osg::ref_ptr<osg::Image> getImage(int index,int sizeIndex);

    unsigned int getNumAtlases(){return _atlasList.size();}
    osg::Image *getAtlasByNumber(unsigned int i){
        if(i < _atlasList.size())
            return _atlasList[i]->_image;
        return NULL;
    }
    int getMaxNumImagesPerAtlas(int tilewidth,int tileheight)

    {

        double area=((double)tilewidth + 2*_margin)*((double)tileheight + 2*_margin);
        if(area==0.0)
            return 0;
        return (int)floor((_maximumAtlasWidth*_maximumAtlasHeight)/area);
    }
    std::vector<osg::Image *> atlasSourceMatrix;
    std::vector<osg::Matrix> offsetMats;
};




#endif // TightFit_H
