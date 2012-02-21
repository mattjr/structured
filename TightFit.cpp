#include "TightFit.h"
#include <stdio.h>

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
        /*std::sort(_sourceList.begin(), _sourceList.end(), CompareSrc());        // Sort using the height of images
    _atlasList.clear();
    for(SourceList::iterator sitr = _sourceList.begin();
        sitr != _sourceList.end();
        ++sitr)
    {
        Source * source = sitr->get();
        if (!source->_atlas && source->suitableForAtlas(_maximumAtlasWidth,_maximumAtlasHeight,_margin))
        {
            bool addedSourceToAtlas = false;
            for(AtlasList::iterator aitr = _atlasList.begin();
                aitr != _atlasList.end() && !addedSourceToAtlas;
                ++aitr)
            {
                if(!(*aitr)->_image ||
                    ((*aitr)->_image->getPixelFormat() == (*sitr)->_image->getPixelFormat() &&
                    (*aitr)->_image->getPacking() == (*sitr)->_image->getPacking()))
                {
                    OSG_INFO<<"checking source "<<source->_image->getFileName()<<" to see it it'll fit in atlas "<<aitr->get()<<std::endl;
                    Optimizer::TextureAtlasBuilder::Atlas::FitsIn fitsIn = (*aitr)->doesSourceFit(source);
                    if (fitsIn == Optimizer::TextureAtlasBuilder::Atlas::FITS_IN_CURRENT_ROW)
                    {
                        addedSourceToAtlas = true;
                        (*aitr)->addSource(source); // Add in the currentRow.
                    }
                    else if(fitsIn == Optimizer::TextureAtlasBuilder::Atlas::IN_NEXT_ROW)
                    {
                        completeRow(aitr - _atlasList.begin()); //Fill Empty spaces.
                        addedSourceToAtlas = true;
                        (*aitr)->addSource(source); // Add the source in the new row.
                    }
                    else
                    {
                        completeRow(aitr - _atlasList.begin()); //Fill Empty spaces before creating a new atlas.
                    }
                }
            }

            if (!addedSourceToAtlas)
            {
                OSG_INFO<<"creating new Atlas for "<<source->_image->getFileName()<<std::endl;

                osg::ref_ptr<Atlas> atlas = new Atlas(_maximumAtlasWidth,_maximumAtlasHeight,_margin);
                _atlasList.push_back(atlas);
                if (!source->_atlas) atlas->addSource(source);
            }
        }
    }

    // build the atlas which are suitable for use, and discard the rest.
    AtlasList activeAtlasList;
    for(AtlasList::iterator aitr = _atlasList.begin();
        aitr != _atlasList.end();
        ++aitr)
    {
        osg::ref_ptr<Atlas> atlas = *aitr;

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
            atlas->_image->setFileName(ostr.str());
            activeAtlasList.push_back(atlas);
            atlas->clampToNearestPowerOfTwoSize();
            atlas->copySources();
        }
    }
    // keep only the active atlas'
    _atlasList.swap(activeAtlasList);
*/
}
