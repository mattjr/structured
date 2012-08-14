#include "TightFit.h"
#include <stdio.h>
#include <algorithm>
TightFitAtlasBuilder::TightFitAtlasBuilder(int mosaic_cell_num){
        atlasSourceMatrix.resize(mosaic_cell_num,NULL);
        offsetMats.resize(mosaic_cell_num);

}
void TightFitAtlasBuilder::buildAtlas()
{
    if(_sourceList.size() == 1)
       {
           Source * source = _sourceList.front();
           osg::ref_ptr<Atlas> atlas = new Atlas(_maximumAtlasWidth,_maximumAtlasHeight,_margin);
           if (!source->_atlas) atlas->addSource(source);

           std::stringstream ostr;
           ostr<<"atlas_"<<_atlasList.size()<<".rgb";
           atlas->_image->setFileName(ostr.str());
           _atlasList.push_back(atlas);
           atlas->clampToNearestPowerOfTwoSize();
           atlas->copySources();

       }else
    osgUtil::Optimizer::TextureAtlasBuilder::buildAtlas();

}
VipsAtlasBuilder::VSource* VipsAtlasBuilder::getSource(const vips::VImage *image)
{
    for(VSourceList::iterator itr = _sourceList.begin();
        itr != _sourceList.end();
        ++itr)
    {
        if ((*itr)->_image == image) return itr->get();
    }
    return 0;
}
osg::Matrix VipsAtlasBuilder::getTextureMatrix(vips::VImage *img)
{


    VSource* source =   getSource(img);
    if(!source || !source->_atlas || !source->_image || !source->_atlas->_image){
        fprintf(stderr,"Failed to get source 0x%lx 0x%lx 0x%lx 0x%lx %d %d\n",(long int)source,(long int)source->_atlas, (long int)source->_image ,
                (long int)source->_atlas->_image,source->_atlas->_width,source->_atlas->_height);
        return osg::Matrix();
    }
    typedef osg::Matrix::value_type Float;
    return osg::Matrix::scale(Float(source->width)/Float(source->_atlas->_width), Float(source->height)/Float(source->_atlas->_height), 1.0)*
           osg::Matrix::translate(Float(source->_x)/Float(source->_atlas->_width), Float(source->_y)/Float(source->_atlas->_height), 0.0);

}
void VipsAtlasBuilder::buildAtlas()
{
    if(_sourceList.size() ==0)
        return;
    long int totalSide=0;
    int largestTile=0;
    for(int i=0; i <(int)_sourceList.size(); i++){
        if(_sourceList[i]->_image){
           int maxSide=std::max(_sourceList[i]->_image->Xsize(),_sourceList[i]->_image->Ysize());
           if(maxSide > largestTile)
               largestTile=maxSide;
           totalSide+=(maxSide*maxSide);
        }
    }

    int sqrtSize=    ceil(sqrt(totalSide));
   // printf("Un adjusted size %d\n",sqrtSize);
    int ajustedGLImageSize=(int)sqrtSize+((sqrtSize/_VTtileSize)*2*_VToverlap);

    printf("Preajusted Atlas Size %d\n",ajustedGLImageSize);

    totalSide=osg::Image::computeNearestPowerOfTwo(ajustedGLImageSize,1.0);
    //printf("POT %ld\n",totalSide);
    totalSide=std:: max(totalSide,(long int)2*_VTtileSize);
    totalSide=(int)totalSide-((totalSide/_VTtileSize)*2*_VToverlap);

    _maximumAtlasWidth=totalSide;
    _maximumAtlasHeight=totalSide;

    if(_sourceList.size() == 1)
       {
           VSource * source = _sourceList.front();
           osg::ref_ptr<VAtlas> atlas = new VAtlas(_maximumAtlasWidth,_maximumAtlasHeight,_margin);
           if (!source->_atlas) atlas->addSource(source);
          // atlas->_height=_maximumAtlasHeight;
           atlas->_width=_maximumAtlasWidth;
           std::stringstream ostr;
           ostr<<"atlas_"<<_atlasList.size()<<".rgb";
           //atlas->_image->setFileName(ostr.str());
           _atlasList.push_back(atlas);
           atlas->copySources(_dryRun);

       }else{

        std::sort(_sourceList.begin(), _sourceList.end(), CompareSrc());        // Sort using the height of images
    _atlasList.clear();
    for(VSourceList::iterator sitr = _sourceList.begin();
        sitr != _sourceList.end();
        ++sitr)
    {
        VSource * source = sitr->get();
        if (!source->_atlas && source->suitableForAtlas(_maximumAtlasWidth,_maximumAtlasHeight))
        {
            bool addedSourceToAtlas = false;
            for(VAtlasList::iterator aitr = _atlasList.begin();
                aitr != _atlasList.end() && !addedSourceToAtlas;
                ++aitr)
            {
               // printf("%d 0x%lx\n",(int)_atlasList.size(),(long int)(*aitr)->_image);

           //     if((*aitr)->_image )
                    /*||
                    ((*aitr)->_image->BandFmt() == (*sitr)->_image->BandFmt() &&
                    (*aitr)->_image->Bands() == (*sitr)->_image->Bands()))*/
                {
//                    osg::notify(osg::INFO)<<"checking source "<<source->_image->filename()<<" to see it it'll fit in atlas "<<aitr->get()<<std::endl;
                    VAtlas::FitsIn fitsIn = (*aitr)->doesSourceFit(source);
                    if (fitsIn == VAtlas::FITS_IN_CURRENT_ROW)
                    {
                                         osg::notify(osg::INFO)<<"fist current row "<<source->_image->filename()<<" to see it it'll fit in atlas "<<aitr->get()<<std::endl;

                        addedSourceToAtlas = true;
                        (*aitr)->addSource(source); // Add in the currentRow.
                    }
                    else if(fitsIn == VAtlas::IN_NEXT_ROW)
                    {                                         osg::notify(osg::INFO)<<"fist next row "<<source->_image->filename()<<" to see it it'll fit in atlas "<<aitr->get()<<std::endl;

                        completeRow(aitr - _atlasList.begin()); //Fill Empty spaces.
                        addedSourceToAtlas = true;
                        (*aitr)->addSource(source); // Add the source in the new row.
                    }
                    else
                    {                                         osg::notify(osg::INFO)<<"doesn't fit  "<<source->_image->filename()<<" to see it it'll fit in atlas "<<aitr->get()<<std::endl;

                        completeRow(aitr - _atlasList.begin()); //Fill Empty spaces before creating a new atlas.
                    }
                }
            }

            if (!addedSourceToAtlas)
            {
                osg::notify(osg::INFO)<<"creating new Atlas for "<<source->_image->filename()<<std::endl;

                osg::ref_ptr<VAtlas> atlas = new VAtlas(_maximumAtlasWidth,_maximumAtlasHeight,_margin);
              //  atlas->_height=_maximumAtlasHeight;
                atlas->_width=_maximumAtlasWidth;

                _atlasList.push_back(atlas);
                if (!source->_atlas) atlas->addSource(source);
            }
        }
    }

    // build the atlas which are suitable for use, and discard the rest.
    VAtlasList activeAtlasList;
    for(VAtlasList::iterator aitr = _atlasList.begin();
        aitr != _atlasList.end();
        ++aitr)
    {
        osg::ref_ptr<VAtlas> atlas = *aitr;

        if (atlas->_sourceList.size()==1)
        {
            // no point building an atlas with only one entry
            // so disconnect the source.
            Source * source = atlas->_sourceList[0].get();
            source->_atlas = 0;
            atlas->_sourceList.clear();
        }

        if (!(atlas->_sourceList.empty()))
        {
            std::stringstream ostr;
            ostr<<"atlas_"<<activeAtlasList.size()<<".rgb";
            //atlas->_image->setFileName(ostr.str());
            activeAtlasList.push_back(atlas);
            atlas->copySources(_dryRun);
        }
    }
    // keep only the active atlas'
    _atlasList.swap(activeAtlasList);
    }
}
void VipsAtlasBuilder::addSource(vips::VImage *img){
    _sourceList.push_back(new VSource(img));
    _sourceList.back()->ref();
}
VipsAtlasBuilder::VipsAtlasBuilder(int count,int VTtileSize,int VToverlap,bool dryRun):TightFitAtlasBuilder(count),_VTtileSize(VTtileSize),_VToverlap(VToverlap),_dryRun(dryRun){

}
void VipsAtlasBuilder::completeRow(unsigned int indexAtlas)
{
    VAtlasList::iterator aitr = _atlasList.begin() + indexAtlas;
    //SourceList::iterator sitr = _sourceList.begin() + indexSource;
    VAtlas * atlas = aitr->get();
    if(atlas->_indexFirstOfRow < atlas->_sourceList.size())
    {
        //Try to fill the row with smaller images.
        int x_max = atlas->_width  - _margin;
        int y_max = atlas->_height - _margin;
        //int x_max = atlas->_maximumAtlasWidth  - _margin;
        //int y_max = atlas->_maximumAtlasHeight - _margin;

        // Fill last Row
        for(VSourceList::iterator sitr3 = _sourceList.begin(); sitr3 != _sourceList.end(); ++sitr3)
        {
            int x_min = atlas->_x + _margin;
            int y_min = atlas->_y + _margin;
            if (y_min >= y_max || x_min >= x_max) continue;

            VSource * source = sitr3->get();
            if (source->_atlas /*|| atlas->_image->BandFmt() != source->_image->BandFmt() ||
                atlas->_image->Bands() != source->_image->Bands()*/)
            {
                continue;
            }

            int image_s = source->_image->Xsize();
            int image_t = source->_image->Ysize();
            if (x_min + image_s <= x_max && y_min + image_t <= y_max)        // Test if the image can fit in the empty space.
            {
                source->_x = x_min;
                source->_y = y_min;
                //assert(source->_x + source->_image->Xsize()+_margin <= atlas->_maximumAtlasWidth );        // "+_margin" and not "+2*_margin" because _x already takes the margin into account
                //assert(source->_y + source->_image->Ysize()+_margin <= atlas->_maximumAtlasHeight);
                //assert(source->_x >= _margin);
                //assert(source->_y >= _margin);
                atlas->_x += image_s + 2*_margin;
                //assert(atlas->_x <= atlas->_maximumAtlasWidth);
                source->_atlas = atlas;
                atlas->_sourceList.push_back(source);
            }
        }

        // Fill the last column
        VSourceList srcListTmp;
        for(VSourceList::iterator sitr4 = atlas->_sourceList.begin() + atlas->_indexFirstOfRow;
            sitr4 != atlas->_sourceList.end(); ++sitr4)
        {
            VSource * srcAdded = sitr4->get();
            int y_min = srcAdded->_y + srcAdded->_image->Ysize() + 2 * _margin;
            int x_min = srcAdded->_x;
            int x_max = x_min + srcAdded->_image->Xsize();        // Hides upper block's x_max
            if (y_min >= y_max || x_min >= x_max) continue;

            VSource * maxWidthSource = NULL;
            for(VSourceList::iterator sitr2 = _sourceList.begin(); sitr2 != _sourceList.end(); ++sitr2)
            {
                VSource * source = sitr2->get();
                if (source->_atlas /*|| atlas->_image->BandFmt() != source->_image->BandFmt() ||
                    atlas->_image->Bands() != source->_image->Bands()*/)
                {
                    continue;
                }
                int image_s = source->_image->Xsize();
                int image_t = source->_image->Ysize();
                if(x_min + image_s <= x_max && y_min + image_t <= y_max)        // Test if the image can fit in the empty space.
                {
                    if (maxWidthSource == NULL || maxWidthSource->_image->Xsize() < source->_image->Xsize())
                    {
                        maxWidthSource = source; //Keep the maximum width for source.
                    }
                }
            }
            if (maxWidthSource)
            {
                // Add the source with the max width to the atlas
                maxWidthSource->_x = x_min;
                maxWidthSource->_y = y_min;
                maxWidthSource->_atlas = atlas;
                srcListTmp.push_back(maxWidthSource); //Store the mawWidth source in the temporary vector.
            }
        }
        for(VSourceList::iterator itTmp = srcListTmp.begin(); itTmp != srcListTmp.end(); ++itTmp)
        {
            //Add the sources to the general list (wasn't possible in the loop using the iterator on the same list)
            atlas->_sourceList.push_back(*itTmp);
        }
        atlas->_indexFirstOfRow = atlas->_sourceList.size();
    }
}
void VipsAtlasBuilder::VAtlas::copySources(bool dryRun)
{
   /* GLenum pixelFormat = _image->getPixelFormat();
    GLenum dataType = _image->getDataType();
    GLenum packing = _image->getPacking();
    OSG_INFO<<"Allocated to "<<_width<<","<<_height<<std::endl;
    _image->allocateImage(_width,_height,1,
                          pixelFormat, dataType,
                          packing);
*/
    _image = new vips::VImage(vips::VImage::black(_maximumAtlasWidth,_maximumAtlasHeight,3));//initdesc(_maximumAtlasWidth,_maximumAtlasHeight,3,vips::VImage::FMTUCHAR,vips::VImage::NOCODING,vips::VImage::sRGB,1.0,1.0,0,0);

    //printf("Creating atlas image of %dx%d\n",_maximumAtlasWidth,_maximumAtlasHeight);
    osg::notify(osg::INFO)<<"Atlas::copySources() "<<std::endl;
    _height=_maximumAtlasHeight;
    _width=_maximumAtlasWidth;
    for(VSourceList::iterator itr = _sourceList.begin();
        itr !=_sourceList.end();
        ++itr)
    {
       VipsAtlasBuilder::VSource* source = itr->get();
        VAtlas* atlas = source->_atlas;

        if (atlas == this)
        {
            osg::notify(osg::INFO)<<"Copying image"<<" to "<<source->_x<<" ,"<<source->_y<<std::endl;
            osg::notify(osg::INFO)<<"        image size "<<source->_image->Ysize()<<","<<source->_image->Xsize()<<std::endl;

            vips::VImage * sourceImage = source->_image;
            vips::VImage* atlasImage = atlas->_image;
            //assert(sourceImage->getPacking() == atlasImage->getPacking()); //Test if packings are equals.
            /*unsigned int rowSize = sourceImage->getRowSizeInBytes();
            unsigned int pixelSizeInBits = sourceImage->getPixelSizeInBits();
            unsigned int pixelSizeInBytes = pixelSizeInBits/8;
            unsigned int marginSizeInBytes = pixelSizeInBytes*_margin;
*/
            //assert(atlas->_width  == static_cast<int>(atlasImage->s()));
            //assert(atlas->_height == static_cast<int>(atlasImage->t()));
            //assert(source->_x + static_cast<int>(source->_image->s())+_margin <= static_cast<int>(atlas->_image->s()));        // "+_margin" and not "+2*_margin" because _x already takes the margin into account
            //assert(source->_y + static_cast<int>(source->_image->t())+_margin <= static_cast<int>(atlas->_image->t()));
            //assert(source->_x >= _margin);
            //assert(source->_y >= _margin);
            int x = source->_x;
            int y = source->_y;
            if(!dryRun)
                atlasImage->insertplace(*sourceImage,x,_height-y-sourceImage->Ysize());
         /*   int t;
            for(t=0; t<sourceImage->t(); ++t, ++y)
            {
                unsigned char* destPtr = atlasImage->data(x, y);
                const unsigned char* sourcePtr = sourceImage->data(0, t);
                for(unsigned int i=0; i<rowSize; i++)
                {
                    *(destPtr++) = *(sourcePtr++);
                }
            }

            // copy top row margin
            y = source->_y + sourceImage->t();
            int m;
            for(m=0; m<_margin; ++m, ++y)
            {
                unsigned char* destPtr = atlasImage->data(x, y);
                const unsigned char* sourcePtr = sourceImage->data(0, sourceImage->t()-1);
                for(unsigned int i=0; i<rowSize; i++)
                {
                    *(destPtr++) = *(sourcePtr++);
                }

            }



            // copy bottom row margin
            y = source->_y-1;
            for(m=0; m<_margin; ++m, --y)
            {
                unsigned char* destPtr = atlasImage->data(x, y);
                const unsigned char* sourcePtr = sourceImage->data(0, 0);
                for(unsigned int i=0; i<rowSize; i++)
                {
                    *(destPtr++) = *(sourcePtr++);
                }

            }

            // copy left column margin
            y = source->_y;
            for(t=0; t<sourceImage->t(); ++t, ++y)
            {
                x = source->_x-1;
                for(m=0; m<_margin; ++m, --x)
                {
                    unsigned char* destPtr = atlasImage->data(x, y);
                    const unsigned char* sourcePtr = sourceImage->data(0, t);
                    for(unsigned int i=0; i<pixelSizeInBytes; i++)
                    {
                        *(destPtr++) = *(sourcePtr++);
                    }
                }
            }

            // copy right column margin
            y = source->_y;
            for(t=0; t<sourceImage->t(); ++t, ++y)
            {
                x = source->_x + sourceImage->s();
                for(m=0; m<_margin; ++m, ++x)
                {
                    unsigned char* destPtr = atlasImage->data(x, y);
                    const unsigned char* sourcePtr = sourceImage->data(sourceImage->s()-1, t);
                    for(unsigned int i=0; i<pixelSizeInBytes; i++)
                    {
                        *(destPtr++) = *(sourcePtr++);
                    }
                }
            }

            // copy top left corner margin
            y = source->_y + sourceImage->t();
            for(m=0; m<_margin; ++m, ++y)
            {
                unsigned char* destPtr = atlasImage->data(source->_x - _margin, y);
                unsigned char* sourcePtr = atlasImage->data(source->_x - _margin, y-1); // copy from row below
                for(unsigned int i=0; i<marginSizeInBytes; i++)
                {
                    *(destPtr++) = *(sourcePtr++);
                }
            }

            // copy top right corner margin
            y = source->_y + sourceImage->t();
            for(m=0; m<_margin; ++m, ++y)
            {
                unsigned char* destPtr = atlasImage->data(source->_x + sourceImage->s(), y);
                unsigned char* sourcePtr = atlasImage->data(source->_x + sourceImage->s(), y-1); // copy from row below
                for(unsigned int i=0; i<marginSizeInBytes; i++)
                {
                    *(destPtr++) = *(sourcePtr++);
                }
            }

            // copy bottom left corner margin
            y = source->_y - 1;
            for(m=0; m<_margin; ++m, --y)
            {
                unsigned char* destPtr = atlasImage->data(source->_x - _margin, y);
                unsigned char* sourcePtr = atlasImage->data(source->_x - _margin, y+1); // copy from row below
                for(unsigned int i=0; i<marginSizeInBytes; i++)
                {
                    *(destPtr++) = *(sourcePtr++);
                }
            }

            // copy bottom right corner margin
            y = source->_y - 1;
            for(m=0; m<_margin; ++m, --y)
            {
                unsigned char* destPtr = atlasImage->data(source->_x + sourceImage->s(), y);
                unsigned char* sourcePtr = atlasImage->data(source->_x + sourceImage->s(), y+1); // copy from row below
                for(unsigned int i=0; i<marginSizeInBytes; i++)
                {
                    *(destPtr++) = *(sourcePtr++);
                }
            }
*/
        }
    }
}
bool VipsAtlasBuilder::VSource::suitableForAtlas(int maximumAtlasWidth, int maximumAtlasHeight)
{
    if (!_image) return false;

    // size too big?
    if (_image->Xsize()*2 > maximumAtlasWidth) return false;
    if (_image->Ysize()*2 > maximumAtlasHeight) return false;

    return true;
}

bool VipsAtlasBuilder::VAtlas::addSource(VSource* source)
{
    // double check source is compatible
   /* if (!doesSourceFit(source))
    {
        OSG_INFO<<"source "<<source->_image->getFileName()<<" does not fit in atlas "<<this<<std::endl;
        return false;
    }*/
    vips::VImage* sourceImage = source->_image;

    if (!_image)
    {
        _image=new vips::VImage;

    }



    // now work out where to fit it, first try current row.
    if ((_x + sourceImage->Xsize() + 2*_margin) <= _maximumAtlasWidth)
    {
        // yes it fits, so add the source to the atlas's list of sources it contains
        _sourceList.push_back(source);

        osg::notify(osg::INFO)<<"current row insertion, source "<<source->_image->filename()<<" "<<_x<<","<<_y<<" fits in row of atlas "<<this<<std::endl;

        // set up the source so it knows where it is in the atlas
        source->_x = _x + _margin;
        source->_y = _y + _margin;
        source->_atlas = this;

        // move the atlas' cursor along to the right
        _x += sourceImage->Xsize() + 2*_margin;

        if (_x > _width) _width = _x;

        int localTop = _y + sourceImage->Ysize() + 2*_margin;
        if ( localTop > _height) _height = localTop;

        return true;
    }

    // does the source fit in the new row up?
    if ((_height + sourceImage->Ysize() + 2*_margin) <= _maximumAtlasHeight)
    {
        // now row so first need to reset the atlas cursor
        _x = 0;
        _y = _height;

        // yes it fits, so add the source to the atlas' list of sources it contains
        _sourceList.push_back(source);

        osg::notify(osg::INFO)<<"next row insertion, source "<<source->_image->filename()<<" "<<_x<<","<<_y<<" fits in row of atlas "<<this<<std::endl;

        // set up the source so it knows where it is in the atlas
        source->_x = _x + _margin;
        source->_y = _y + _margin;
        source->_atlas = this;

        // move the atlas' cursor along to the right
        _x += sourceImage->Xsize() + 2*_margin;

        if (_x > _width) _width = _x;

        _height = _y + sourceImage->Ysize() + 2*_margin;

         osg::notify(osg::INFO)<<"source "<<source->_image->filename()<<" "<<_x<<","<<_y<<" fits in row of atlas "<<this<<std::endl;

        return true;
    }

    osg::notify(osg::INFO)<<"source "<<source->_image->filename()<<" does not fit in atlas "<<this<<std::endl;

    // shouldn't get here, unless doesSourceFit isn't working...
    return false;
}
VipsAtlasBuilder::VAtlas::FitsIn VipsAtlasBuilder::VAtlas::doesSourceFit(VSource* source)
{
    // does the source have a valid image?
     vips::VImage *sourceImage = source->_image;
    if (!sourceImage) {
        return DOES_NOT_FIT_IN_ANY_ROW;
     osg::notify(osg::INFO)<<"Null Image."<<std::endl;
    }
    // does pixel format match?
   /* if (_image.valid())
    {
        if (_image->getPixelFormat() != sourceImage->getPixelFormat()) return DOES_NOT_FIT_IN_ANY_ROW;
        if (_image->getDataType() != sourceImage->getDataType()) return DOES_NOT_FIT_IN_ANY_ROW;
    }
*/


    if (sourceImage->Xsize() + 2*_margin > _maximumAtlasWidth)
    {   osg::notify(osg::NOTICE)<<"// image too big for Atlas width."<<std::endl;
        // image too big for Atlas
        return DOES_NOT_FIT_IN_ANY_ROW;
    }

    if (sourceImage->Ysize() + 2*_margin > _maximumAtlasHeight)
    {            osg::notify(osg::NOTICE)<<"// image too big for Atlas."<<std::endl;

        // image too big for Atlas
        return DOES_NOT_FIT_IN_ANY_ROW;
    }

    if ((_y + sourceImage->Ysize() + 2*_margin) > _maximumAtlasHeight)
    {         osg::notify(osg::NOTICE)<<"image doesn't have up space in height axis."<<std::endl;

        // image doesn't have up space in height axis.
        return DOES_NOT_FIT_IN_ANY_ROW;
    }

    // does the source fit in the current row?
    if ((_x + sourceImage->Xsize() + 2*_margin) <= _maximumAtlasWidth)
    {
        // yes it fits :-)
         osg::notify(osg::INFO)<<"Fits in current row"<<std::endl;
        return FITS_IN_CURRENT_ROW;
    }

    // does the source fit in the new row up?
    if ((_height + sourceImage->Ysize() + 2*_margin) <= _maximumAtlasHeight)
    {
        // yes it fits :-)
         osg::notify(osg::INFO)<<"Fits in next row"<<std::endl;
        return IN_NEXT_ROW;
    }

    // no space for the texture
    return DOES_NOT_FIT_IN_ANY_ROW;
}

