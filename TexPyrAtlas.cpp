#include "TexPyrAtlas.h"
#include <osgDB/ReadFile>
#include <string.h>
#include <assert.h>
using namespace std;
TexPyrAtlas::TexPyrAtlas(texcache_t imgdir):_imgdir(imgdir)
{
    setMaximumAtlasSize(8192,8192);
    setMargin(5);
    _downsampleSizes.clear();
    _downsampleSizes.push_back(512);
    _downsampleSizes.push_back(128);
    _downsampleSizes.push_back(32);
    _state = new osg::State;
    _useTextureArray=false;
    _useAtlas=true;
    _useStub=true;
    srand(getpid());
}


void TexPyrAtlas::computeImageNumberToAtlasMap(void){
    for(int i=0; i < (int)_atlasList.size(); i++){
        for(int j=0; j< (int)_atlasList[i]->_sourceList.size(); j++)
            if(_sourceToId.count(_atlasList[i]->_sourceList[j]))
                _idToAtlas[_sourceToId[_atlasList[i]->_sourceList[j]]]=i;
    }
}

int TexPyrAtlas::getAtlasId(id_type id){
    if(id < 0 )
        return id;

    if(!_useAtlas)
        return id;
    if(_idToAtlas.count(id))
        return _idToAtlas[id];
    return -1;
}
osg::Matrix TexPyrAtlas::getTextureMatrixByID(id_type id){
    if(_idToSource.count(id))
        return computeTextureMatrixFreedImage(_idToSource[id]);     //   return _idToSource[id]->computeTextureMatrix(); This was using images we had freed

    return osg::Matrix::identity();
}
osg::Matrix TexPyrAtlas::computeTextureMatrixFreedImage(Source *s)
{
    if (!s->_atlas) return osg::Matrix();
    double image_t,image_s;
    if(!_sourceToSize.count(s))return osg::Matrix();
    image_s=_sourceToSize[s][0];
    image_t=_sourceToSize[s][1];

    if (!(s->_atlas->_image)) return osg::Matrix();

    return osg::Matrix::scale(float(image_s)/float(s->_atlas->_image->s()), float(image_t)/float(s->_atlas->_image->t()), 1.0)*
            osg::Matrix::translate(float(s->_x)/float(s->_atlas->_image->s()), float(s->_y)/float(s->_atlas->_image->t()), 0.0);
}
void TexPyrAtlas::addSources(std::vector<std::pair<id_type ,std::string> > imageList){
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_imageListMutex);

    for(int i=0; i< (int)imageList.size(); i++){

        if(_totalImageList.count(imageList[i].first) == 0 )
            _totalImageList.insert(imageList[i]);
    }
}
osg::ref_ptr<osg::Image> TexPyrAtlas::getImage(int index,int sizeIndex){
    osg::ref_ptr<osg::Image> img;
    if(!_useAtlas){
        if(index >= 0 && index < (int)_images.size() && _images[index].valid()){
            resizeImage(_images[index].get(),_downsampleSizes[sizeIndex],_downsampleSizes[sizeIndex],img);
        }

        return img;
    }else{
        return getAtlasByNumber(index);
    }
    /*  if(index >= 0 && index < (int)_images.size() && _images[index].valid())
        return _tb.getImageAtlas(_images[index]);
    else
        return osg::ref_ptr<osg::Image> ();*/
}
osg::ref_ptr<osg::Image> TexPyrAtlas::getImageFullorStub(string fname,int size){
    if(_useStub){
        osg::ref_ptr<osg::Image> img= new osg::Image();
        img->setFileName(fname);
        img->setImage(size,size,3,GL_RGB, GL_RGB, GL_UNSIGNED_BYTE,NULL,osg::Image::NO_DELETE);
        img->setWriteHint(osg::Image::EXTERNAL_FILE);
        return img;
    }
    return osgDB::readImageFile(fname);
}
string getUUID(void){
    char strUuid[1024];

    sprintf(strUuid, "%x%x-%x-%x-%x-%x%x%x",
            rand(), rand(),                 // Generates a 64-bit Hex number
            rand(),                         // Generates a 32-bit Hex number
            ((rand() & 0x0fff) | 0x4000),   // Generates a 32-bit Hex number of the form 4xxx (4 indicates the UUID version)
            rand() % 0x3fff + 0x8000,       // Generates a 32-bit Hex number in the range [0x8000, 0xbfff]
            rand(), rand(), rand());        // Generates a 96-bit Hex number
    return string(strUuid);
}

void TexPyrAtlas::loadTextureFiles(int size){
    std::vector<osg::ref_ptr<osg::Image> > loc_images;
    if( _totalImageList.size() == 0)
        return;
    string closestDir;
    if(!getClosestDir(closestDir,size)){
        cerr << "Can't find any dir close to size " << size<<endl;
        exit(-1);
    }
    std::map<id_type,string>::const_iterator end = _totalImageList.end();
    for (std::map<id_type,string>::const_iterator it = _totalImageList.begin(); it != end; ++it){
        string fname=closestDir+"/"+it->second;
        osg::ref_ptr<osg::Image> img=getImageFullorStub(fname,size);
        assert(img.valid());
        osg::ref_ptr<osg::Image> tmp=NULL;
        if(img->s() == size && img->t() == size)
            tmp=img;
        else
            resizeImage(img,size,size,tmp);
        loc_images.push_back(tmp);
        if(loc_images.back().valid()){
            if(!_useAtlas) {
                _allIDs[it->first]=_images.size();
                _images.push_back(loc_images.back());
                if(!_useStub)
                    _images.back()->setFileName(getUUID());

            }else{
                //texture->setImage(_images[i]);
                /*   texture->setTextureSize(_downsampleSizes[0],_downsampleSizes[0]);
                bool resizePowerOfTwo=true;
                osg::NotifySeverity saved_ns=osg::getNotifyLevel();
                // osg::setNotifyLevel(osg::FATAL);
                vpb::generateMipMap(*_state,*texture,resizePowerOfTwo,vpb::BuildOptions::GL_DRIVER);
                // osg::setNotifyLevel(saved_ns);*/
                if (!getSource(loc_images.back())) {
                    Source *s=new Source(loc_images.back());
                    _sourceList.push_back(s);
                    _sourceToId[s]=it->first;
                    _idToSource[it->first]=s;
                    _sourceToSize[s]=osg::Vec2(loc_images.back()->s(),loc_images.back()->t());
                }
            }
        }else{
            OSG_ALWAYS << it->second << " not found or couldn't be loaded"<<std::endl;
        }
    }


    if(_useAtlas){
        buildAtlas();
        computeImageNumberToAtlasMap();
        for(int i=0; i < (int)getNumAtlases(); i++){
                _images.push_back(getAtlasByNumber(i));
                _images.back()->setFileName(getUUID());
            }
        //Free ref created by source new above which is leaks after copy from sources
       // for(int i=0; i < (int)loc_images.size(); i++){
        //    loc_images[i]->unref();
        //}

    }
       /* for(int i=0; i < loc_images.size(); i++){
            OSG_ALWAYS << loc_images[i]->referenceCount() << endl;
        }*/
}
osg::ref_ptr<osg::Texture> TexPyrAtlas::getTexture(int index,int sizeIndex){

    if(index >= 0 && index < (int)_images.size() && _images[index].valid())
        return getTextureAtlas(_images[index]);
    else
        return osg::ref_ptr<osg::Texture> ();
}

bool
        TexPyrAtlas::resizeImage(const osg::Image* input,
                                 unsigned int out_s, unsigned int out_t,
                                 osg::ref_ptr<osg::Image>& output,
                                 unsigned int mipmapLevel )
{
    if ( !input && out_s == 0 && out_t == 0 )
        return false;

    GLenum pf = input->getPixelFormat();

    //if ( pf != GL_RGBA && pf != GL_RGB && pf != GL_LUMINANCE && pf != GL_RED && pf != GL_LUMINANCE_ALPHA )
    //{
    //    OE_WARN << LC << "resizeImage: unsupported pixel format " << std::hex << pf << std::endl;
    //    return 0L;
    //}

    unsigned int in_s = input->s();
    unsigned int in_t = input->t();

    if ( !output.valid() )
    {
        output = new osg::Image();
        output->allocateImage( out_s, out_t, 1, pf, input->getDataType(), input->getPacking() );
        assert(output.valid() && output->data());
    }
    output->setInternalTextureFormat( input->getInternalTextureFormat() );

    if ( in_s == out_s && in_t == out_t && mipmapLevel == 0 )
    {
        memcpy( output->getMipmapData(mipmapLevel), input->data(), input->getTotalSizeInBytes() );
    }
    else
    {
        //float s_ratio = (float)in_s/(float)out_s;
        //float t_ratio = (float)in_t/(float)out_t;
        unsigned int pixel_size_bytes = input->getRowSizeInBytes() / in_s;

        unsigned char* dataOffset = output->getMipmapData(mipmapLevel);
        unsigned int   dataRowSizeBytes = output->getRowSizeInBytes() >> mipmapLevel;

        for( unsigned int output_row=0; output_row < out_t; output_row++ )
        {
            // get an appropriate input row
            float output_row_ratio = (float)output_row/(float)out_t;
            int input_row = (unsigned int)( output_row_ratio * (float)in_t );
            if ( input_row >= input->t() ) input_row = in_t-1;
            else if ( input_row < 0 ) input_row = 0;

            for(unsigned int output_col = 0; output_col < out_s; output_col++ )
            {
                float output_col_ratio = (float)output_col/(float)out_s;
                int input_col = (unsigned int)( output_col_ratio * (float)in_s );
                if ( input_col >= (int)in_s ) input_col = in_s-1;
                else if ( input_row < 0 ) input_row = 0;

                unsigned char* outaddr =
                        dataOffset +
                        (output_col*output->getPixelSizeInBits())/8+output_row*dataRowSizeBytes;

                memcpy(
                        outaddr,
                        input->data( input_col, input_row ),
                        pixel_size_bytes );
            }
        }
    }

    return true;
}


void TexPyrAtlas::buildAtlas()
{
    //printf("Number of sources %d\n",(int)_sourceList.size());
    // assign the source to the atlas
    _atlasList.clear();
    for(SourceList::iterator sitr = _sourceList.begin();
    sitr != _sourceList.end();
    ++sitr)
    {
        Source* source = sitr->get();
        if (source->suitableForAtlas(_maximumAtlasWidth,_maximumAtlasHeight,_margin))
        {
            bool addedSourceToAtlas = false;
            for(AtlasList::iterator aitr = _atlasList.begin();
            aitr != _atlasList.end() && !addedSourceToAtlas;
            ++aitr)
            {
                OSG_INFO<<"checking source "<<source->_image->getFileName()<<" to see it it'll fit in atlas "<<aitr->get()<<std::endl;
                if ((*aitr)->doesSourceFit(source))
                {
                    addedSourceToAtlas = true;
                    (*aitr)->addSource(source);
                }
                else
                {
                    OSG_INFO<<"source "<<source->_image->getFileName()<<" does not fit in atlas "<<aitr->get()<<std::endl;
                }
            }

            if (!addedSourceToAtlas)
            {
                OSG_INFO<<"creating new Atlas for "<<source->_image->getFileName()<<std::endl;

                osg::ref_ptr<Atlas> atlas = new Atlas(_maximumAtlasWidth,_maximumAtlasHeight,_margin);
                if(_atlasList.size() && _useTextureArray){
                    atlas->_width=_maximumAtlasWidth;
                    atlas->_height=_maximumAtlasHeight;
                }
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
        Atlas* atlas = aitr->get();

        /*   if (atlas->_sourceList.size()==1)
        {
            // no point building an atlas with only one entry
            // so disconnect the source.
            Source* source = atlas->_sourceList[0].get();
            source->_atlas = 0;
            atlas->_sourceList.clear();
        }*/

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

