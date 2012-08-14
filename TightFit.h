#ifndef TightFit_H
#define TightFit_H
#include <osgUtil/Optimizer>
#include <string>
#include <osg/State>
#include <osg/Referenced>
#include <vips/vips>
#include <stdio.h>
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
    std::vector<void *> atlasSourceMatrix;
    std::vector<osg::Matrix> offsetMats;
};


class VipsAtlasBuilder : public TightFitAtlasBuilder
{
public:

    VipsAtlasBuilder(int mosaic_cell_num,int VTtileSize,int VToverlap,bool dryRun=false);
    int _VTtileSize,_VToverlap;
    osg::Matrix getTextureMatrix(vips::VImage *);
    int getAtlasHeight(){return _maximumAtlasHeight;}
    int getAtlasWidth(){return _maximumAtlasWidth;}
    bool _dryRun;
    void addSource(vips::VImage *img);
    void buildAtlas(void);
    void completeRow(unsigned int indexAtlas);
class VAtlas;
     class VSource : public Source{
     public:
         VSource( vips::VImage * image): Source((const osg::Image*)NULL),
             _image(image) ,_atlas(NULL){width=_image->Xsize(); height=_image->Ysize();}
      vips::VImage *_image;
      VAtlas *_atlas;
      int width,height;
      bool suitableForAtlas(int maximumAtlasWidth, int maximumAtlasHeight);

     };
     typedef std::vector< osg::ref_ptr<VSource> > VSourceList;
     VSource* getSource(const vips::VImage *image);

     class VAtlas: public Atlas{
     public:
         VAtlas(int width, int height, int margin):Atlas(width,height,0),_image(NULL),_indexFirstOfRow(0){if(margin>0){fprintf(stderr,"Cannot have margin in this implmentation being reset to 0 margin!\n");}}

         vips::VImage *_image;
         VSourceList _sourceList;
         bool addSource(VSource* source);
       enum FitsIn
       {
	 DOES_NOT_FIT_IN_ANY_ROW,
	 FITS_IN_CURRENT_ROW,
	 IN_NEXT_ROW
       };	 
       FitsIn doesSourceFit(VSource* source);

         void copySources(void);

         int _indexFirstOfRow;

     };
     VSourceList _sourceList;
     typedef std::vector< osg::ref_ptr<VAtlas> > VAtlasList;
     VAtlasList _atlasList;

     struct CompareSrc
     {
         bool operator()(osg::ref_ptr<VSource> src1, osg::ref_ptr<VSource> src2) const
         {
             return src1->_image->Xsize() > src2->_image->Ysize();
         }
     };
};



#endif // TightFit_H
