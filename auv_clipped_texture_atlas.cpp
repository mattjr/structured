#include "auv_clipped_texture_atlas.hpp"
#include <osgDB/WriteFile>
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
	  printf("One Source\n");
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
  
  osg::notify(osg::INFO)<<"ClippedAtlas::copySources() "<<std::endl;
  for(SourceList::iterator itr = _sourceList.begin();
      itr !=_sourceList.end();
      ++itr)
    {
        ClippedSource* source = itr->get();
        ClippedAtlas* atlas = source->_atlas;

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

#if 1
    osg::notify(osg::NOTICE)<<"Writing atlas image "<<_image->getFileName()<<std::endl;
    osgDB::writeImageFile(*_image,_image->getFileName() + ".png");
#endif
}

void ClippedTextureAtlasVisitor::optimize()
{

    _builder.reset();
    
    if (_textures.size()<2)
    {
      printf("No textures\n");
        // nothing to optimize
        return;
    }
printf("D o ittextures\n");
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
                        osg::notify(osg::NOTICE)<<"All drawables can be flattened "<<drawables.size()<<std::endl;
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
                         osg::notify(osg::NOTICE)<<"Applying TexMat "<<drawables.size()<<std::endl;
                        stateset->setTextureAttribute(unit, new osg::TexMat(matrix));
                    }
                }
            }
        }

    }
}
bool ClippedTextureAtlasBuilder::ClippedAtlas::addSource(ClippedSource* source)
{
    // double check source is compatible
    if (!doesSourceFit(source))
    {
        osg::notify(osg::INFO)<<"source "<<source->_image->getFileName()<<" does not fit in atlas "<<this<<std::endl;
        return false;
    }
    
    const osg::Image* sourceImage = source->_image.get();
    const osg::Texture2D* sourceTexture = source->_texture.get();

    if (!_image)
    {
        // need to create an image of the same pixel format to store the atlas in
        _image = new osg::Image;
        _image->setPixelFormat(sourceImage->getPixelFormat());
        _image->setDataType(sourceImage->getDataType());
    }
    
    if (!_texture && sourceTexture)
    {
        _texture = new osg::Texture2D(_image.get());

        _texture->setWrap(osg::Texture2D::WRAP_S, sourceTexture->getWrap(osg::Texture2D::WRAP_S));
        _texture->setWrap(osg::Texture2D::WRAP_T, sourceTexture->getWrap(osg::Texture2D::WRAP_T));
        
        _texture->setBorderColor(sourceTexture->getBorderColor());
        _texture->setBorderWidth(0);
            
        _texture->setFilter(osg::Texture2D::MIN_FILTER, sourceTexture->getFilter(osg::Texture2D::MIN_FILTER));
        _texture->setFilter(osg::Texture2D::MAG_FILTER, sourceTexture->getFilter(osg::Texture2D::MAG_FILTER));

        _texture->setMaxAnisotropy(sourceTexture->getMaxAnisotropy());

        _texture->setInternalFormat(sourceTexture->getInternalFormat());

        _texture->setShadowCompareFunc(sourceTexture->getShadowCompareFunc());
        _texture->setShadowTextureMode(sourceTexture->getShadowTextureMode());
        _texture->setShadowAmbient(sourceTexture->getShadowAmbient());

    }

    // now work out where to fit it, first try current row.
    if ((_x + sourceImage->s() + 2*_margin) <= _maximumAtlasWidth)
    {
        // yes it fits, so add the source to the atlas's list of sources it contains
        _sourceList.push_back(source);

        osg::notify(osg::INFO)<<"current row insertion, source "<<source->_image->getFileName()<<" "<<_x<<","<<_y<<" fits in row of atlas "<<this<<std::endl;

        // set up the source so it knows where it is in the atlas
        source->_x = _x + _margin;
        source->_y = _y + _margin;
        source->_atlas = this;
        
        // move the atlas' cursor along to the right
        _x += sourceImage->s() + 2*_margin;
        
        if (_x > _width) _width = _x;
        
        unsigned int localTop = _y + sourceImage->t() + 2*_margin;
        if ( localTop > _height) _height = localTop;

        return true;
    }

    // does the source fit in the new row up?
    if ((_height + sourceImage->t() + 2*_margin) <= _maximumAtlasHeight)
    {
        // now row so first need to reset the atlas cursor
        _x = 0;
        _y = _height;

        // yes it fits, so add the source to the atlas' list of sources it contains
        _sourceList.push_back(source);

        osg::notify(osg::INFO)<<"next row insertion, source "<<source->_image->getFileName()<<" "<<_x<<","<<_y<<" fits in row of atlas "<<this<<std::endl;

        // set up the source so it knows where it is in the atlas
        source->_x = _x + _margin;
        source->_y = _y + _margin;
        source->_atlas = this;
        
        // move the atlas' cursor along to the right
        _x += sourceImage->s() + 2*_margin;

        if (_x > _width) _width = _x;

        _height = _y + sourceImage->t() + 2*_margin;

        osg::notify(osg::INFO)<<"source "<<source->_image->getFileName()<<" "<<_x<<","<<_y<<" fits in row of atlas "<<this<<std::endl;

        return true;
    }

    osg::notify(osg::INFO)<<"source "<<source->_image->getFileName()<<" does not fit in atlas "<<this<<std::endl;

    // shouldn't get here, unless doesSourceFit isn't working...
    return false;
}

osg::Image* ClippedTextureAtlasBuilder::getImageAtlas(unsigned int i)
{
    ClippedSource* source = _sourceList[i].get();
    ClippedAtlas* atlas = source ? source->_atlas : 0;
    return atlas ? atlas->_image.get() : 0;
}

osg::Texture2D* ClippedTextureAtlasBuilder::getTextureAtlas(unsigned int i)
{
    ClippedSource* source = _sourceList[i].get();
    ClippedAtlas* atlas = source ? source->_atlas : 0;
    return atlas ? atlas->_texture.get() : 0;
}

osg::Matrix ClippedTextureAtlasBuilder::getTextureMatrix(unsigned int i)
{
    ClippedSource* source = _sourceList[i].get();
    return source ? source->computeTextureMatrix() : osg::Matrix();
}

osg::Image* ClippedTextureAtlasBuilder::getImageAtlas(const osg::Image* image)
{
    ClippedSource* source = getSource(image);
    ClippedAtlas* atlas = source ? source->_atlas : 0;
    return atlas ? atlas->_image.get() : 0;
}

osg::Texture2D* ClippedTextureAtlasBuilder::getTextureAtlas(const osg::Image* image)
{
    ClippedSource* source = getSource(image);
    ClippedAtlas* atlas = source ? source->_atlas : 0;
    return atlas ? atlas->_texture.get() : 0;
}

osg::Matrix ClippedTextureAtlasBuilder::getTextureMatrix(const osg::Image* image)
{
    ClippedSource* source = getSource(image);
    return source ? source->computeTextureMatrix() : osg::Matrix();
}

osg::Image* ClippedTextureAtlasBuilder::getImageAtlas(const osg::Texture2D* texture)
{
    ClippedSource* source = getSource(texture);
    ClippedAtlas* atlas = source ? source->_atlas : 0;
    return atlas ? atlas->_image.get() : 0;
}

osg::Texture2D* ClippedTextureAtlasBuilder::getTextureAtlas(const osg::Texture2D* texture)
{
    ClippedSource* source = getSource(texture);
    ClippedAtlas* atlas = source ? source->_atlas : 0;
    return atlas ? atlas->_texture.get() : 0;
}

osg::Matrix ClippedTextureAtlasBuilder::getTextureMatrix(const osg::Texture2D* texture)
{
    ClippedSource* source = getSource(texture);
    return source ? source->computeTextureMatrix() : osg::Matrix();
}

ClippedTextureAtlasBuilder::ClippedSource* ClippedTextureAtlasBuilder::getSource(const osg::Image* image)
{
    for(SourceList::iterator itr = _sourceList.begin();
        itr != _sourceList.end();
        ++itr)
    {
        if ((*itr)->_image == image) return itr->get();
    }
    return 0;
}

ClippedTextureAtlasBuilder::ClippedSource* ClippedTextureAtlasBuilder::getSource(const osg::Texture2D* texture)
{
    for(SourceList::iterator itr = _sourceList.begin();
        itr != _sourceList.end();
        ++itr)
    {
        if ((*itr)->_texture == texture) return itr->get();
    }
    return 0;
}
