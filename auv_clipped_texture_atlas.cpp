#include "auv_clipped_texture_atlas.hpp"

void ClippedTextureAtlasBuilder::buildAtlas(){
    // assign the source to the atlas
    _atlasList.clear();
    for(SourceList::iterator sitr = _sourceList.begin();
        sitr != _sourceList.end();
        ++sitr)
      {
        ClippedSource* source = sitr->get();
        if (source->suitableForAtlas(_maximumAtlasWidth,_maximumAtlasHeight,_margin))
	  {
            bool addedSourceToAtlas = false;
            for(AtlasList::iterator aitr = _atlasList.begin();
                aitr != _atlasList.end() && !addedSourceToAtlas;
                ++aitr)
	      {
                osg::notify(osg::INFO)<<"checking source "<<source->_image->getFileName()<<" to see it it'll fit in atlas "<<aitr->get()<<std::endl;
                if ((*aitr)->doesSourceFit(source))
		  {
                    addedSourceToAtlas = true;
                    (*aitr)->addSource(source);
                }
                else
		  {
                    osg::notify(osg::INFO)<<"source "<<source->_image->getFileName()<<" does not fit in atlas "<<aitr->get()<<std::endl;
		  }
	      }
	    
            if (!addedSourceToAtlas)
	      {
                osg::notify(osg::INFO)<<"creating new Atlas for "<<source->_image->getFileName()<<std::endl;
		
                osg::ref_ptr<ClippedAtlas> atlas = new ClippedAtlas(_maximumAtlasWidth,_maximumAtlasHeight,_margin);
                _atlasList.push_back(atlas.get());
                
                atlas->addSource(source);
	      }
	  }
      }
    
    // build the atlas which are suitable for use, and discard the rest.
    AtlasList activeAtlasList;
    for(AtlasList::iterator aitr = _atlasList.begin();
        aitr != _atlasList.end();
        ++aitr)
      {
        ClippedAtlas* atlas = aitr->get();
	
        if (atlas->_sourceList.size()==1)
        {
            // no point building an atlas with only one entry
            // so disconnect the source.
            Source* source = atlas->_sourceList[0].get();
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

}


void ClippedTextureAtlasBuilder::ClippedAtlas::copySources(){
  
  osg::notify(osg::INFO)<<"Allocated to "<<_width<<","<<_height<<std::endl;
  _image->allocateImage(_width,_height,1,
			_image->getPixelFormat(),_image->getDataType(),
			_image->getPacking());
  
  {
    // clear memory
    unsigned int size = _image->getTotalSizeInBytes();
    unsigned char* str = _image->data();
    for(unsigned int i=0; i<size; ++i) *(str++) = 0;
  }        
  
  osg::notify(osg::INFO)<<"Atlas::copySources() "<<std::endl;
  for(SourceList::iterator itr = _sourceList.begin();
      itr !=_sourceList.end();
      ++itr)
    {
        Source* source = itr->get();
        Atlas* atlas = source->_atlas;

        if (atlas)
        {
            osg::notify(osg::INFO)<<"Copying image "<<source->_image->getFileName()<<" to "<<source->_x<<" ,"<<source->_y<<std::endl;
            osg::notify(osg::INFO)<<"        image size "<<source->_image->s()<<","<<source->_image->t()<<std::endl;

            const osg::Image* sourceImage = source->_image.get();
            osg::Image* atlasImage = atlas->_image.get();

            unsigned int rowSize = sourceImage->getRowSizeInBytes();
            unsigned int pixelSizeInBits = sourceImage->getPixelSizeInBits();
            unsigned int pixelSizeInBytes = pixelSizeInBits/8;
            unsigned int marginSizeInBytes = pixelSizeInBytes*_margin;

            unsigned int x = source->_x;
            unsigned int y = source->_y;

            int t;
	    int tmin=0;
	    int tmax=sourceImage->t();
	    unsigned int sminBytes=0;
	    unsigned int smaxBytes=pixelSizeInBytes*sourceImage->s();
	    printf("Clipped Min %d %d Max %d %d\n",tmin,tmax,sminBytes,smaxBytes);
            for(t=tmin; t<tmax; ++t, ++y)
            {
                unsigned char* destPtr = atlasImage->data(x, y);
                const unsigned char* sourcePtr = sourceImage->data(0, t);
                for(unsigned int i=sminBytes; i<smaxBytes; i++)
                {
                    *(destPtr++) = *(sourcePtr++);
                }
            }
	}
    }

#if 0
    osg::notify(osg::NOTICE)<<"Writing atlas image "<<_image->getFileName()<<std::endl;
    osgDB::writeImageFile(*_image,_image->getFileName() + ".png");
#endif
}

void ClippedTextureAtlasVisitor::optimize()
{

    _builder.reset();
    
    if (_textures.size()<2)
    {
        // nothing to optimize
        return;
    }

    Textures texturesThatRepeat;
    Textures texturesThatRepeatAndAreOutOfRange;

    StateSetMap::iterator sitr;
    for(sitr = _statesetMap.begin();
        sitr != _statesetMap.end();
        ++sitr)
    {
        osg::StateSet* stateset = sitr->first;
        Drawables& drawables = sitr->second;

        osg::StateSet::TextureAttributeList& tal = stateset->getTextureAttributeList();
        for(unsigned int unit=0; unit<tal.size(); ++unit)
        {
            osg::Texture2D* texture = dynamic_cast<osg::Texture2D*>(stateset->getTextureAttribute(unit,osg::StateAttribute::TEXTURE));
            if (texture)
            {
                bool s_repeat = texture->getWrap(osg::Texture2D::WRAP_S)==osg::Texture2D::REPEAT ||
                                texture->getWrap(osg::Texture2D::WRAP_S)==osg::Texture2D::MIRROR;

                bool t_repeat = texture->getWrap(osg::Texture2D::WRAP_T)==osg::Texture2D::REPEAT ||
                                texture->getWrap(osg::Texture2D::WRAP_T)==osg::Texture2D::MIRROR;
                               
                if (s_repeat || t_repeat)
                {
                    texturesThatRepeat.insert(texture);
                
                    bool s_outOfRange = false;
                    bool t_outOfRange = false;
  
                    float s_min = -0.001;
                    float s_max = 1.001;

                    float t_min = -0.001;
                    float t_max = 1.001;
                    
                    for(Drawables::iterator ditr = drawables.begin();
                        ditr != drawables.end();
                        ++ditr)
                    {
                        osg::Geometry* geom = (*ditr)->asGeometry();
                        osg::Vec2Array* texcoords = geom ? dynamic_cast<osg::Vec2Array*>(geom->getTexCoordArray(unit)) : 0;
                        if (texcoords && !texcoords->empty())
                        {
                            for(osg::Vec2Array::iterator titr = texcoords->begin();
                                titr != texcoords->end() /*&& !s_outOfRange && !t_outOfRange*/;
                                ++titr)
                            {
                                osg::Vec2 tc = *titr;
                                if (tc[0]<s_min) { s_min = tc[0]; s_outOfRange = true; }
                                if (tc[0]>s_max) { s_max = tc[0]; s_outOfRange = true; }

                                if (tc[1]<t_min) { t_min = tc[1]; t_outOfRange = true; }
                                if (tc[1]>t_max) { t_max = tc[1]; t_outOfRange = true; }
                            }
                        }
                        else
                        {
                            // if no texcoords then texgen must be being used, therefore must assume that texture is truely repeating
                            s_outOfRange = true;
                            t_outOfRange = true;
                        }                        
                    }

                    if (s_outOfRange || t_outOfRange) 
                    {                    
                        texturesThatRepeatAndAreOutOfRange.insert(texture);
                    }

                }
            }
        }
    }

    // now change any texture that repeat but all texcoords to them 
    // are in 0 to 1 range than converting the to CLAMP mode, to allow them
    // to be used in an atlas.
    Textures::iterator titr;
    for(titr = texturesThatRepeat.begin();
        titr != texturesThatRepeat.end();
        ++titr)
    {
        osg::Texture2D* texture = *titr;
        if (texturesThatRepeatAndAreOutOfRange.count(texture)==0)
        {
            // safe to convert into CLAMP wrap mode.
            osg::notify(osg::INFO)<<"Changing wrap mode to CLAMP"<<std::endl;
            texture->setWrap(osg::Texture2D::WRAP_S, osg::Texture::CLAMP);
            texture->setWrap(osg::Texture2D::WRAP_T, osg::Texture::CLAMP);
        }
    }

    // add the textures as sources for the TextureAtlasBuilder
    for(titr = _textures.begin();
        titr != _textures.end();
        ++titr)
    {
        osg::Texture2D* texture = *titr;

        bool s_repeat = texture->getWrap(osg::Texture2D::WRAP_S)==osg::Texture2D::REPEAT ||
                        texture->getWrap(osg::Texture2D::WRAP_S)==osg::Texture2D::MIRROR;

        bool t_repeat = texture->getWrap(osg::Texture2D::WRAP_T)==osg::Texture2D::REPEAT ||
                        texture->getWrap(osg::Texture2D::WRAP_T)==osg::Texture2D::MIRROR;

        if (!s_repeat && !t_repeat)
        {
            _builder.addSource(*titr);
        }
    }
    
    // build the atlas'
    _builder.buildAtlas();


    typedef std::set<osg::StateSet*> StateSetSet;
    typedef std::map<osg::Drawable*, StateSetSet> DrawableStateSetMap;
    DrawableStateSetMap dssm;
    for(sitr = _statesetMap.begin();
        sitr != _statesetMap.end();
        ++sitr)
    {
        Drawables& drawables = sitr->second;
        for(Drawables::iterator ditr = drawables.begin();
            ditr != drawables.end();
            ++ditr)
        {
            dssm[(*ditr)->asGeometry()].insert(sitr->first);
        }
    }
    
    Drawables drawablesThatHaveMultipleTexturesOnOneUnit;
    for(DrawableStateSetMap::iterator ditr = dssm.begin();
        ditr != dssm.end();
        ++ditr)
    {
        osg::Drawable* drawable = ditr->first;
        StateSetSet& ssm = ditr->second;
        if (ssm.size()>1)
        {
            typedef std::map<unsigned int, Textures> UnitTextureMap;
            UnitTextureMap unitTextureMap;
            for(StateSetSet::iterator ssm_itr = ssm.begin();
                ssm_itr != ssm.end();
                ++ssm_itr)
            {
                osg::StateSet* ss = *ssm_itr;
                unsigned int numTextureUnits = ss->getTextureAttributeList().size();
                for(unsigned int unit=0; unit<numTextureUnits; ++unit)
                {
                    osg::Texture2D* texture = dynamic_cast<osg::Texture2D*>(ss->getTextureAttribute(unit, osg::StateAttribute::TEXTURE));
                    if (texture) unitTextureMap[unit].insert(texture);
                }
            }
            bool drawablesHasMultiTextureOnOneUnit = false;
            for(UnitTextureMap::iterator utm_itr = unitTextureMap.begin();
                utm_itr != unitTextureMap.end() && !drawablesHasMultiTextureOnOneUnit;
                ++utm_itr)
            {
                if (utm_itr->second.size()>1)
                {
                    drawablesHasMultiTextureOnOneUnit = true;
                }
            }
            if (drawablesHasMultiTextureOnOneUnit)
            {
                drawablesThatHaveMultipleTexturesOnOneUnit.insert(drawable);
            }

        }
    }
        
    // remap the textures in the StateSet's 
    for(sitr = _statesetMap.begin();
        sitr != _statesetMap.end();
        ++sitr)
    {
        osg::StateSet* stateset = sitr->first;
        osg::StateSet::TextureAttributeList& tal = stateset->getTextureAttributeList();
        for(unsigned int unit=0; unit<tal.size(); ++unit)
        {
            osg::Texture2D* texture = dynamic_cast<osg::Texture2D*>(stateset->getTextureAttribute(unit,osg::StateAttribute::TEXTURE));
            if (texture)
            {
                bool s_repeat = texture->getWrap(osg::Texture2D::WRAP_S)==osg::Texture2D::REPEAT ||
                                texture->getWrap(osg::Texture2D::WRAP_S)==osg::Texture2D::MIRROR;

                bool t_repeat = texture->getWrap(osg::Texture2D::WRAP_T)==osg::Texture2D::REPEAT ||
                                texture->getWrap(osg::Texture2D::WRAP_T)==osg::Texture2D::MIRROR;

                osg::Texture2D* newTexture = _builder.getTextureAtlas(texture);
                if (newTexture && newTexture!=texture)
                {
                    if (s_repeat || t_repeat)
                    {
                        osg::notify(osg::NOTICE)<<"Warning!!! shouldn't get here"<<std::endl;
                    }

                    stateset->setTextureAttribute(unit, newTexture);
                    
                    Drawables& drawables = sitr->second;
                    
                    osg::Matrix matrix = _builder.getTextureMatrix(texture);
                    
                    // first check to see if all drawables are ok for applying texturematrix to.
                    bool canTexMatBeFlattenedToAllDrawables = true;
                    for(Drawables::iterator ditr = drawables.begin();
                        ditr != drawables.end() && canTexMatBeFlattenedToAllDrawables;
                        ++ditr)
                    {
                        osg::Geometry* geom = (*ditr)->asGeometry();
                        osg::Vec2Array* texcoords = geom ? dynamic_cast<osg::Vec2Array*>(geom->getTexCoordArray(unit)) : 0;

                        if (!texcoords) 
                        {
                            canTexMatBeFlattenedToAllDrawables = false;
                        }
                        
                        if (drawablesThatHaveMultipleTexturesOnOneUnit.count(*ditr)!=0)
                        {
                            canTexMatBeFlattenedToAllDrawables = false;
                        }
                    }

                    if (canTexMatBeFlattenedToAllDrawables)
                    {
                        // osg::notify(osg::NOTICE)<<"All drawables can be flattened "<<drawables.size()<<std::endl;
                        for(Drawables::iterator ditr = drawables.begin();
                            ditr != drawables.end();
                            ++ditr)
                        {
                            osg::Geometry* geom = (*ditr)->asGeometry();
                            osg::Vec2Array* texcoords = geom ? dynamic_cast<osg::Vec2Array*>(geom->getTexCoordArray(unit)) : 0;
                            if (texcoords)
                            {
                                for(osg::Vec2Array::iterator titr = texcoords->begin();
                                    titr != texcoords->end();
                                    ++titr)
                                {
                                    osg::Vec2 tc = *titr;
                                    (*titr).set(tc[0]*matrix(0,0) + tc[1]*matrix(1,0) + matrix(3,0),
                                              tc[0]*matrix(0,1) + tc[1]*matrix(1,1) + matrix(3,1));
                                }
                            }
                            else
                            {
                                osg::notify(osg::NOTICE)<<"Error, Optimizer::TextureAtlasVisitor::optimize() shouldn't ever get here..."<<std::endl;
                            }                        
                        }
                    }
                    else
                    {
                        // osg::notify(osg::NOTICE)<<"Applying TexMat "<<drawables.size()<<std::endl;
                        stateset->setTextureAttribute(unit, new osg::TexMat(matrix));
                    }
                }
            }
        }

    }
}
