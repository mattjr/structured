#include "TexPyrAtlas.h"
#include <osgDB/ReadFile>
#include <vpb/TextureUtils>
TexPyrAtlas::TexPyrAtlas(std::string imgdir):_imgdir(imgdir)
{
    _tb.setMaximumAtlasSize(4096,4096);
    _downsampleSizes.clear();
    _downsampleSizes.push_back(512);
    _downsampleSizes.push_back(256);
    _downsampleSizes.push_back(32);
    _state = new osg::State;


}

void TexPyrAtlas::loadSources(std::vector<std::string> imageList){
    _images.resize(imageList.size());
    for(int i=0; i< (int)imageList.size(); i++){
        osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
        _images[i]=osgDB::readImageFile(_imgdir+"/"+imageList[i]);
        if(_images[i].valid()){
            texture->setImage(_images[i]);
            bool resizePowerOfTwo=true;
            osg::NotifySeverity saved_ns=osg::getNotifyLevel();
            osg::setNotifyLevel(osg::FATAL);
            vpb::generateMipMap(*_state,*texture,resizePowerOfTwo,vpb::BuildOptions::GL_DRIVER);
            osg::setNotifyLevel(saved_ns);
        }else{
            OSG_ALWAYS << imageList[i] << " not found or couldn't be loaded"<<std::endl;
        }
    }

}
osg::ref_ptr<osg::Image> TexPyrAtlas::getImage(int index,int sizeIndex){
    osg::ref_ptr<osg::Image> img;

    if(index >= 0 && index < (int)_images.size() && _images[index].valid())
        resizeImage(_images[index].get(),_downsampleSizes[sizeIndex],_downsampleSizes[sizeIndex],img);
    return img;
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
