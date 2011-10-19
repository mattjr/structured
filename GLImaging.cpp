#include "GLImaging.h"
#include "Semaphore.h"
#include <osg/Texture3D>
#include <stddef.h>
#include "CPUDetect.h"
#include <sys/stat.h>

void optImage(osg::ref_ptr<osg::Image> &image){
    //int pitch=image->getRowSizeInBytes();
    unsigned char *dataBGRA=new unsigned char[image->s()*image->t()*4];
    unsigned char *dataRGBA=new unsigned char[image->s()*image->t()*4];
    RGB2RGBA(image->s(),image->t(),image->data(),dataRGBA);
    /*#if _M_SSE >= 0x301
    cpu_inf
            // Uses SSSE3 intrinsics to optimize RGBA -> BGRA swizzle:
            if (cpu_info.bSSSE3) {
        ConvertRGBA_BGRA_SSSE3((unsigned int *)dataBGRA,pitch,(unsigned int *)dataRGBA,image->s(),image->t(),pitch/4);
    } else
#endif
    // Uses SSE2 intrinsics to optimize RGBA -> BGRA swizzle:
    {
        ConvertRGBA_BGRA_SSE2((unsigned int *)dataBGRA,pitch,(unsigned int *)dataRGBA,image->s(),image->t(),pitch/4);

    }
    */
    RGBA2BGRA(image->s(),image->t(),(uint32_t*)dataRGBA,(uint32_t*)dataBGRA);

    delete dataRGBA;
    image->setImage(image->s(),image->t(),1,GL_RGB,GL_BGRA,GL_UNSIGNED_BYTE,dataBGRA,osg::Image::USE_NEW_DELETE,1);
    //image->setPixelBufferObject(new osg::PixelBufferObject(image)); SLOWER
}
void OptFormatTexturesVisitor::opt()
{

    for(TextureSet::iterator itr=_textureSet.begin();
        itr!=_textureSet.end();
        ++itr)
    {
        osg::Texture* texture = const_cast<osg::Texture*>(itr->get());

        osg::Texture2D* texture2D = dynamic_cast<osg::Texture2D*>(texture);
        osg::Texture3D* texture3D = dynamic_cast<osg::Texture3D*>(texture);

        osg::ref_ptr<osg::Image> image = texture2D ? texture2D->getImage() : (texture3D ? texture3D->getImage() : 0);
        if (image.valid()){

            if((image->getPixelFormat()==GL_RGB || image->getPixelFormat()==GL_RGBA) &&
                    (image->s()>=32 && image->t()>=32))
            {
                optImage(image);

            }
            _imageSizeMB+=(image->getImageSizeInBytes())/1024.0/1024.0;

        }
    }
}

void
RGB2RGBA(unsigned int w, unsigned int h,
         unsigned char *src, unsigned char *dst) {
    for (unsigned int i=w*h; i; i--) {
        memmove(dst, src, 3) ;
        dst += 3 ;
        src += 3 ;
        *dst++ = 255 ;
    }
}

void
RGBA2BGRA(unsigned int w, unsigned int h,
          uint32_t *src, uint32_t *dst) {
    for (unsigned int i=0; i<w*h; i++) {
#if __BYTE_ORDER == __BIG_ENDIAN
        dst[i] = ((src[i] & 0xFF00) << 16) | (src[i] & 0xFF00FF) | ((src[i] >> 16) & 0xFF00);
#else
        dst[i] = ((src[i] >> 16) & 0xFF) | (src[i] & 0xFF00FF00) | ((src[i] & 0xFF) << 16);
#endif
    }
}

#if 0
void ConvertRGBA_BGRA_SSE2(u32 *dst, const int dstPitch, u32 *pIn, const int width, const int height, const int pitch)
{
    // Converts RGBA to BGRA:
    // TODO: this would be totally unnecessary if we just change the TextureDecoder_RGBA to decode
    // to BGRA instead.
    for (int y = 0; y < height; y++, pIn += pitch)
    {
        u8 *pIn8 = (u8 *)pIn;
        u8 *pBits = (u8 *)((u8*)dst + (y * dstPitch));

        // Batch up loads/stores into 16 byte chunks to use SSE2 efficiently:
        int sse2blocks = (width * 4) / 16;
        int sse2remainder = (width * 4) & 15;

        // Do conversions in batches of 16 bytes:
        if (sse2blocks > 0)
        {
            // Generate a constant of all FF bytes:
            const __m128i allFFs128 = _mm_cmpeq_epi32(_mm_setzero_si128(), _mm_setzero_si128());
            __m128i *src128 = (__m128i *)pIn8;
            __m128i *dst128 = (__m128i *)pBits;

            // Increment by 16 bytes at a time:
            for (int i = 0; i < sse2blocks; ++i, ++dst128, ++src128)
            {
                // Load up 4 colors simultaneously:
                __m128i rgba = _mm_loadu_si128(src128);
                // Swap the R and B components:
                // Isolate the B component and shift it left 16 bits:
                // ABGR
                const __m128i bMask = _mm_srli_epi32(allFFs128, 24);
                const __m128i bNew = _mm_slli_epi32(_mm_and_si128(rgba, bMask), 16);
                // Isolate the R component and shift it right 16 bits:
                const __m128i rMask = _mm_slli_epi32(bMask, 16);
                const __m128i rNew = _mm_srli_epi32(_mm_and_si128(rgba, rMask), 16);
                // Now mask off the old R and B components from the rgba data to get 0g0a:
                const __m128i _g_a = _mm_or_si128(
                            _mm_and_si128(
                                rgba,
                                _mm_or_si128(
                                    _mm_slli_epi32(bMask, 8),
                                    _mm_slli_epi32(rMask, 8)
                                    )
                                ),
                            _mm_or_si128(rNew, bNew)
                            );
                // Finally, OR up all the individual components to get BGRA:
                const __m128i bgra = _mm_or_si128(_g_a, _mm_or_si128(rNew, bNew));
                _mm_storeu_si128(dst128, bgra);
            }
        }

        // Take the remainder colors at the end of the row that weren't able to
        // be included into the last 16 byte chunk:
        if (sse2remainder > 0)
        {
            for (int x = (sse2blocks * 16); x < (width * 4); x += 4)
            {
                pBits[x + 0] = pIn8[x + 2];
                pBits[x + 1] = pIn8[x + 1];
                pBits[x + 2] = pIn8[x + 0];
                pBits[x + 3] = pIn8[x + 3];
            }
        }
    }

    // Memory fence to make sure the stores are good:
    _mm_mfence();
}

void ConvertRGBA_BGRA_SSSE3(u32 *dst, const int dstPitch, u32 *pIn, const int width, const int height, const int pitch)
{
    __m128i mask = _mm_set_epi8(15, 12, 13, 14, 11, 8, 9, 10, 7, 4, 5, 6, 3, 0, 1, 2);
    for (int y = 0; y < height; y++, pIn += pitch)
    {
        u8 *pIn8 = (u8 *)pIn;
        u8 *pBits = (u8 *)((u8*)dst + (y * dstPitch));

        // Batch up loads/stores into 16 byte chunks to use SSE2 efficiently:
        int ssse3blocks = (width * 4) / 16;
        int ssse3remainder = (width * 4) & 15;

        // Do conversions in batches of 16 bytes:
        if (ssse3blocks > 0)
        {
            __m128i *src128 = (__m128i *)pIn8;
            __m128i *dst128 = (__m128i *)pBits;

            // Increment by 16 bytes at a time:
            for (int i = 0; i < ssse3blocks; ++i, ++dst128, ++src128)
            {
                _mm_storeu_si128(dst128, _mm_shuffle_epi8(_mm_loadu_si128(src128), mask));
            }
        }

        // Take the remainder colors at the end of the row that weren't able to
        // be included into the last 16 byte chunk:
        if (ssse3remainder > 0)
        {
            for (int x = (ssse3blocks * 16); x < (width * 4); x += 4)
            {
                pBits[x + 0] = pIn8[x + 2];
                pBits[x + 1] = pIn8[x + 1];
                pBits[x + 2] = pIn8[x + 0];
                pBits[x + 3] = pIn8[x + 3];
            }
        }
    }

    // Memory fence to make sure the stores are good:
    _mm_mfence();
}
#endif

void WindowCaptureCallback::ContextData::updateTimings(osg::Timer_t tick_start,
                                                       osg::Timer_t tick_afterReadPixels,
                                                       osg::Timer_t tick_afterMemCpy,
                                                       unsigned int dataSize)
{
    if (!_reportTimingFrequency) return;

    double timeForReadPixels = osg::Timer::instance()->delta_s(tick_start, tick_afterReadPixels);
    double timeForFullCopy = osg::Timer::instance()->delta_s(tick_start, tick_afterMemCpy);
    double timeForMemCpy = osg::Timer::instance()->delta_s(tick_afterReadPixels, tick_afterMemCpy);

    _timeForReadPixels += timeForReadPixels;
    _timeForFullCopy += timeForFullCopy;
    _timeForMemCpy += timeForMemCpy;

    ++_numTimeValuesRecorded;

    if (_numTimeValuesRecorded==_reportTimingFrequency)
    {
        timeForReadPixels = _timeForReadPixels/double(_numTimeValuesRecorded);
        timeForFullCopy = _timeForFullCopy/double(_numTimeValuesRecorded);
        timeForMemCpy = _timeForMemCpy/double(_numTimeValuesRecorded);

        double averageFrameTime =  osg::Timer::instance()->delta_s(_previousFrameTick, tick_afterMemCpy)/double(_numTimeValuesRecorded);
        double fps = 1.0/averageFrameTime;
        _previousFrameTick = tick_afterMemCpy;

        _timeForReadPixels = 0.0;
        _timeForFullCopy = 0.0;
        _timeForMemCpy = 0.0;

        _numTimeValuesRecorded = 0;

        double numMPixels = double(_width * _height) / 1000000.0;
        double numMb = double(dataSize) / (1024*1024);

        int prec = osg::notify(osg::INFO).precision(5);

        if (timeForMemCpy==0.0)
        {
            osg::notify(osg::INFO)<<"fps = "<<fps<<", full frame copy = "<<timeForFullCopy*1000.0f<<"ms rate = "<<numMPixels / timeForFullCopy<<" Mpixel/sec, copy speed = "<<numMb / timeForFullCopy<<" Mb/sec"<<std::endl;
        }
        else
        {
            osg::notify(osg::INFO)<<"fps = "<<fps<<", full frame copy = "<<timeForFullCopy*1000.0f<<"ms rate = "<<numMPixels / timeForFullCopy<<" Mpixel/sec, "<<numMb / timeForFullCopy<< " Mb/sec "<<
                                     "time for memcpy = "<<timeForMemCpy*1000.0<<"ms  memcpy speed = "<<numMb / timeForMemCpy<<" Mb/sec"<<std::endl;
        }
        osg::notify(osg::INFO).precision(prec);

    }

}
int gpuUsage(int gpu,int &mem){
    char cmd[1024];
    // system("nvidia-smi -a | grep Mb >> debugLog.txt");
    sprintf(cmd,"nvidia-smi -q --gpu=%d | grep Mb| grep Free| cut -c34-38 ",gpu);
    FILE *lsofFile_p = popen(cmd, "r");

    if (!lsofFile_p) { return -1; }
    char buffer[1024];char *lp=fgets(buffer, sizeof(buffer), lsofFile_p); pclose(lsofFile_p);
    if(lp==NULL){
        fprintf(stderr,"error on read\n");
    }
    mem=atoi(buffer);

    return 0;
}

void WindowCaptureCallback::ContextData::readPixels()
{
    // std::cout<<"readPixels("<<_fileName<<" image "<<_currentImageIndex<<" "<<_currentPboIndex<<std::endl;

    unsigned int nextImageIndex = (_currentImageIndex+1)%_imageBuffer.size();
    unsigned int nextPboIndex = _pboBuffer.empty() ? 0 : (_currentPboIndex+1)%_pboBuffer.size();

    int width=0, height=0;
    getSize(_gc, width, height);
    if (width!=_width || _height!=height)
    {
        std::cout<<"   Window resized "<<width<<", "<<height<<std::endl;
        _width = width;
        _height = height;
    }

    osg::Image* image = _imageBuffer[_currentImageIndex].get();

    osg::Timer_t tick_start = osg::Timer::instance()->tick();

#if 1
    image->readPixels(0,0,_width,_height,
                      _pixelFormat,_type);
#endif

    osg::Timer_t tick_afterReadPixels = osg::Timer::instance()->tick();

    updateTimings(tick_start, tick_afterReadPixels, tick_afterReadPixels, image->getTotalSizeInBytes());

    if (!_fileName.empty())
    {
        // osgDB::writeImageFile(*image, _fileName);
    }

    _currentImageIndex = nextImageIndex;
    _currentPboIndex = nextPboIndex;
}

void WindowCaptureCallback::ContextData::singlePBO(osg::GLBufferObject::Extensions* ext)
{
    // std::cout<<"singelPBO(  "<<_fileName<<" image "<<_currentImageIndex<<" "<<_currentPboIndex<<std::endl;

    unsigned int nextImageIndex = (_currentImageIndex+1)%_imageBuffer.size();

    int width=0, height=0;
    getSize(_gc, width, height);
    if (width!=_width || _height!=height)
    {
        std::cout<<"   Window resized "<<width<<", "<<height<<std::endl;
        _width = width;
        _height = height;
    }

    GLuint& pbo = _pboBuffer[0];

    osg::Image* image = _imageBuffer[_currentImageIndex].get();
    if (image->s() != _width ||
            image->t() != _height)
    {
        osg::notify(osg::INFO)<<"Allocating image "<<std::endl;
        image->allocateImage(_width, _height, 1, GL_RGBA, _type);

        if (pbo!=0)
        {
            osg::notify(osg::INFO)<<"deleting pbo "<<pbo<<std::endl;
            ext->glDeleteBuffers (1, &pbo);
            pbo = 0;
        }
    }


    if (pbo==0)
    {
        ext->glGenBuffers(1, &pbo);
        ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, pbo);
        ext->glBufferData(GL_PIXEL_PACK_BUFFER_ARB, image->getTotalSizeInBytes(), 0, GL_STREAM_READ);

        osg::notify(osg::INFO)<<"Generating pbo "<<pbo<<std::endl;
    }
    else
    {
        ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, pbo);
    }

    osg::Timer_t tick_start = osg::Timer::instance()->tick();

#if 1
    glReadPixels(0, 0, _width, _height, _pixelFormat, _type, 0);
#endif

    osg::Timer_t tick_afterReadPixels = osg::Timer::instance()->tick();

    GLubyte* src = (GLubyte*)ext->glMapBuffer(GL_PIXEL_PACK_BUFFER_ARB,
                                              GL_READ_ONLY_ARB);
    if(src)
    {

        //memcpy(image->data(), src, image->getTotalSizeInBytes());
        //int pitch=image->getRowSizeInBytes();//4*_width;
        image->setPixelFormat(GL_RGBA);
        //uint32_t *p = (uint32_t *)src;
        //uint32_t *dst = (uint32_t *)image->data();
        RGBA2BGRA(image->s(),image->t(),(uint32_t*)src,(uint32_t*)image->data());



#if 0
#if _M_SSE >= 0x301
        cpu_inf
                // Uses SSSE3 intrinsics to optimize RGBA -> BGRA swizzle:
                if (cpu_info.bSSSE3) {
            ConvertRGBA_BGRA_SSSE3((unsigned int *)image->data(),pitch,(unsigned int *)src,_width,_height,pitch/4);
        } else
#endif
        // Uses SSE2 intrinsics to optimize RGBA -> BGRA swizzle:
        {
            ConvertRGBA_BGRA_SSE2((unsigned int *)image->data(),pitch,(unsigned int *)src,_width,_height,pitch/4);

        }
#endif

        ext->glUnmapBuffer(GL_PIXEL_PACK_BUFFER_ARB);
    }

    ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);

    osg::Timer_t tick_afterMemCpy = osg::Timer::instance()->tick();

    updateTimings(tick_start, tick_afterReadPixels, tick_afterMemCpy, image->getTotalSizeInBytes());

    if (!_fileName.empty())
    {
        // osgDB::writeImageFile(*image, _fileName);
    }


    _currentImageIndex = nextImageIndex;
}

void WindowCaptureCallback::ContextData::multiPBO(osg::GLBufferObject::Extensions* ext)
{
    // std::cout<<"multiPBO(  "<<_fileName<<" image "<<_currentImageIndex<<" "<<_currentPboIndex<<std::endl;
    unsigned int nextImageIndex = (_currentImageIndex+1)%_imageBuffer.size();
    unsigned int nextPboIndex = (_currentPboIndex+1)%_pboBuffer.size();

    int width=0, height=0;
    getSize(_gc, width, height);
    if (width!=_width || _height!=height)
    {
        std::cout<<"   Window resized "<<width<<", "<<height<<std::endl;
        _width = width;
        _height = height;
    }

    GLuint& copy_pbo = _pboBuffer[_currentPboIndex];
    GLuint& read_pbo = _pboBuffer[nextPboIndex];

    osg::Image* image = _imageBuffer[_currentImageIndex].get();
    if (image->s() != _width ||
            image->t() != _height)
    {
        osg::notify(osg::INFO)<<"Allocating image "<<std::endl;
        image->allocateImage(_width, _height, 1, _pixelFormat, _type);

        if (read_pbo!=0)
        {
            osg::notify(osg::INFO)<<"deleting pbo "<<read_pbo<<std::endl;
            ext->glDeleteBuffers (1, &read_pbo);
            read_pbo = 0;
        }

        if (copy_pbo!=0)
        {
            osg::notify(osg::INFO)<<"deleting pbo "<<copy_pbo<<std::endl;
            ext->glDeleteBuffers (1, &copy_pbo);
            copy_pbo = 0;
        }
    }


    bool doCopy = copy_pbo!=0;
    if (copy_pbo==0)
    {
        ext->glGenBuffers(1, &copy_pbo);
        ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, copy_pbo);
        ext->glBufferData(GL_PIXEL_PACK_BUFFER_ARB, image->getTotalSizeInBytes(), 0, GL_STREAM_READ);

        osg::notify(osg::INFO)<<"Generating pbo "<<read_pbo<<std::endl;
    }

    if (read_pbo==0)
    {
        ext->glGenBuffers(1, &read_pbo);
        ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, read_pbo);
        ext->glBufferData(GL_PIXEL_PACK_BUFFER_ARB, image->getTotalSizeInBytes(), 0, GL_STREAM_READ);

        osg::notify(osg::INFO)<<"Generating pbo "<<read_pbo<<std::endl;
    }
    else
    {
        ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, read_pbo);
    }

    osg::Timer_t tick_start = osg::Timer::instance()->tick();

#if 1
    glReadPixels(0, 0, _width, _height, _pixelFormat, _type, 0);
#endif

    osg::Timer_t tick_afterReadPixels = osg::Timer::instance()->tick();

    if (doCopy)
    {

        ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, copy_pbo);

        GLubyte* src = (GLubyte*)ext->glMapBuffer(GL_PIXEL_PACK_BUFFER_ARB,
                                                  GL_READ_ONLY_ARB);
        if(src)
        {
            memcpy(image->data(), src, image->getTotalSizeInBytes());
            ext->glUnmapBuffer(GL_PIXEL_PACK_BUFFER_ARB);
        }

        if (!_fileName.empty())
        {
            // osgDB::writeImageFile(*image, _fileName);
        }
    }

    ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);

    osg::Timer_t tick_afterMemCpy = osg::Timer::instance()->tick();

    updateTimings(tick_start, tick_afterReadPixels, tick_afterMemCpy, image->getTotalSizeInBytes());

    _currentImageIndex = nextImageIndex;
    _currentPboIndex = nextPboIndex;
}

void addCallbackToViewer(osgViewer::ViewerBase& viewer, WindowCaptureCallback* callback)
{

    if (callback->getFramePosition()==WindowCaptureCallback::START_FRAME)
    {
        osgViewer::ViewerBase::Windows windows;
        viewer.getWindows(windows);
        for(osgViewer::ViewerBase::Windows::iterator itr = windows.begin();
            itr != windows.end();
            ++itr)
        {
            osgViewer::GraphicsWindow* window = *itr;
            osg::GraphicsContext::Cameras& cameras = window->getCameras();
            osg::Camera* firstCamera = 0;
            for(osg::GraphicsContext::Cameras::iterator cam_itr = cameras.begin();
                cam_itr != cameras.end();
                ++cam_itr)
            {
                if (firstCamera)
                {
                    if ((*cam_itr)->getRenderOrder() < firstCamera->getRenderOrder())
                    {
                        firstCamera = (*cam_itr);
                    }
                    if ((*cam_itr)->getRenderOrder() == firstCamera->getRenderOrder() &&
                            (*cam_itr)->getRenderOrderNum() < firstCamera->getRenderOrderNum())
                    {
                        firstCamera = (*cam_itr);
                    }
                }
                else
                {
                    firstCamera = *cam_itr;
                }
            }

            if (firstCamera)
            {
                osg::notify(osg::INFO)<<"First camera "<<firstCamera<<std::endl;

                firstCamera->setInitialDrawCallback(callback);
            }
            else
            {
                osg::notify(osg::INFO)<<"No camera found"<<std::endl;
            }
        }
    }
    else
    {
        osgViewer::ViewerBase::Windows windows;
        viewer.getWindows(windows);
        for(osgViewer::ViewerBase::Windows::iterator itr = windows.begin();
            itr != windows.end();
            ++itr)
        {
            osgViewer::GraphicsWindow* window = *itr;
            osg::GraphicsContext::Cameras& cameras = window->getCameras();
            osg::Camera* lastCamera = 0;
            for(osg::GraphicsContext::Cameras::iterator cam_itr = cameras.begin();
                cam_itr != cameras.end();
                ++cam_itr)
            {
                if (lastCamera)
                {
                    if ((*cam_itr)->getRenderOrder() > lastCamera->getRenderOrder())
                    {
                        lastCamera = (*cam_itr);
                    }
                    if ((*cam_itr)->getRenderOrder() == lastCamera->getRenderOrder() &&
                            (*cam_itr)->getRenderOrderNum() >= lastCamera->getRenderOrderNum())
                    {
                        lastCamera = (*cam_itr);
                    }
                }
                else
                {
                    lastCamera = *cam_itr;
                }
            }

            if (lastCamera)
            {
                osg::notify(osg::INFO)<<"Last camera "<<lastCamera<<std::endl;

                lastCamera->setFinalDrawCallback(callback);
            }
            else
            {
                osg::notify(osg::INFO)<<"No camera found"<<std::endl;
            }
        }
    }
}


void formatBar(string name,osg::Timer_t startTick,unsigned int count,unsigned int totalCount){
    osg::Timer_t tick = osg::Timer::instance()->tick();
    double currentTime = osg::Timer::instance()->delta_s(startTick, tick);
    struct winsize w;
    ioctl(0, TIOCGWINSZ, &w);
    int term_width=w.ws_col;
    std::stringstream tmp;
    tmp<<totalCount;
    int countlength=tmp.str().size();
    std::stringstream time;
    double percentage=(count/(double)totalCount);

    if(count == 0){
        time <<"ETA: --:--:--";
    }else{
        if(count == totalCount){
            time << "Time: ";
        } else{
            time <<"ETA:  ";
            currentTime= currentTime*totalCount/count - currentTime;
        }
        double hours=floor(currentTime/60.0/60.0);
        double mins=floor((currentTime-(hours*60.0*60.0))/60.0);
        double secs=floor(currentTime-(hours*60.0*60.0)-(mins*60.0));

        time   <<  setfill('0') << setw(2) <<(int)(hours) <<":"<<setfill('0') << setw(2) <<(int)(mins)<< ":"<<setfill('0') << setw(2) <<secs;
    }

    std::stringstream bar;

    bar << name << " " <<  setw(3)<<(int)(round(100.0*percentage))<<"% " <<setfill('0')<<setw(countlength+1)<< count <<"/"<<setfill('0')<<setw(countlength+1)<<totalCount;
    bar <<" |";
    int length=term_width-bar.str().size()-time.str().size()-2;
    for(int i=0; i< length; i++){
        if(i < length*percentage)
            bar<<"#";
        else
            bar <<" ";
    }
    bar << "| "<<time.str();
    // if(count==totalCount)
    //   bar<<"\n";
    printf("\r%s",bar.str().c_str());
    fflush(stdout);
}

/*
void applyGeoTags(osg::Vec2 geoOrigin,osg::Matrix viewMatrix,osg::Matrix projMatrix,int width,int height){

    osg::Matrix modWindow =( osg::Matrix::translate(1.0,1.0,1.0)*osg::Matrix::scale(0.5*width,0.5*height,0.5f));
    osg::Matrix bottomLeftToTopLeft= (osg::Matrix::scale(1,-1,1)*osg::Matrix::translate(0,height,0));
    osg::Matrix worldtoScreen=viewMatrix * projMatrix * modWindow*bottomLeftToTopLeft;
    osg::Matrix screenToWorld=osg::Matrix::inverse(worldtoScreen);
    osg::Vec3 tl(0,0,0);
    osg::Vec3 bl(0,height,0);
    osg::Vec3 tr(width,0,0);
    osg::Vec3 br(width,height,0);
    char szProj4[4096];
    sprintf( szProj4,
            "\"+proj=tmerc +lat_0=%.24f +lon_0=%.24f +k=%.12f +x_0=%.12f +y_0=%.12f +datum=WGS84 +ellps=WGS84 +units=m +no_defs\"",geoOrigin.x(),geoOrigin.y(),1.0,0.0,0.0);

    osg::Vec3 tlGlobal=tl*screenToWorld;
    osg::Vec3 blGlobal=bl*screenToWorld;
    osg::Vec3 trGlobal=tr*screenToWorld;
    osg::Vec3 brGlobal=br*screenToWorld;
    osg::Vec3 diff=(brGlobal-tlGlobal);
    osg::Vec2 scalePix( diff.x()/width, diff.y()/height);
    cout << "brGlobal " << brGlobal<< " tlGlobal " << tlGlobal << endl;
    cout << "trGlobal " << trGlobal<< " " << blGlobal << endl;

    std::ofstream worldfile("subtile.tfw");
    worldfile << std::setprecision(12) << scalePix.x() << std::endl<< 0 <<std::endl<< 0 << std::endl<<scalePix.y() << std::endl<<tlGlobal.x()<<std::endl<<tlGlobal.y()<<std::endl;
    std::cout << tlGlobal << " " << blGlobal <<" " <<trGlobal<< " " << brGlobal <<"\n";
    worldfile.close();
    char gdal_param[4096];
    sprintf(gdal_param," -of GTiff -co \"TILED=YES\" -a_ullr %.12f %.12f %.12f %.12f -a_srs %s",tlGlobal.x(),tlGlobal.y(),brGlobal.x(),brGlobal.y(),szProj4);

    FILE *fp=fopen("add_geo.sh","w");
    if(!fp)
        std::cerr << "Failed!\n";
    fprintf(fp,"#!/bin/bash\n");
    //fprintf(fp,"vips im_vips2tiff subtile.v tex.tif:none:tile:256x256\n");


    fprintf(fp,"#gdal_translate %s subtile.tif geo_tif.tif\n",gdal_param);
    fprintf(fp,"#geotifcp -e subtile.tfw subtile.tif subtile.tif\n");

    fchmod(fileno(fp),0777);
    fclose (fp);
    //gdalwarp  -t_srs '+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs' geo_tif2.tif utm.tif
    int res= system("./add_geo.sh");
    if(res != 0)
        printf("Failed on run\n");
}*/

int imageNodeGL(osg::Node *node,unsigned int _tileRows,unsigned int _tileColumns,unsigned int width,unsigned int height,int row,int col,
                const osg::Matrixd &view,const osg::Matrixd &proj,bool untex,bool depth,std::string ext)
{
    //int pid=getpid();
    if(node == NULL)
        return -1;
    //Convert to a fast upload format currently seems to be GL_RGB, GL_BGRA for nvidia cards 280-580 gtx
    //Driver specific worth checking on new cards
    osg::ref_ptr<OptFormatTexturesVisitor> oftv=new OptFormatTexturesVisitor();
    node->accept(*oftv.get());
    oftv->opt();
    double sizeImages=oftv->getImageSizeMB();
    printf("Size of textures %.2f MB\n",sizeImages);
    key_t semkey;
    osg::ref_ptr<osg::Image> depthImage;
    if(depth){
        depthImage = new osg::Image();
        depthImage->allocateImage(width, height, 1,GL_DEPTH_COMPONENT, GL_FLOAT );

    }
#define KEY (1492)

    semkey=KEY;//ftok("/tmp/glimagesem",'a');

    osg::ref_ptr<osg::Image> tmpImg1,tmpImg2;

    android::Semaphore sem;
    if(!sem.attach(semkey))
    {
        fprintf(stderr,"Semaphore Failure FATAL ERROR\n");
        exit(-1);
    }

    {
        osgViewer::Viewer viewer;

        viewer.setThreadingModel( osgViewer::Viewer::SingleThreaded );
        GLenum readBuffer = GL_BACK;
        WindowCaptureCallback::FramePosition position = WindowCaptureCallback::END_FRAME;
        WindowCaptureCallback::Mode mode = WindowCaptureCallback::SINGLE_PBO;


        osg::ref_ptr<osg::GraphicsContext> pbuffer;
        {
            osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
            traits->x = 0;
            traits->y = 0;
            traits->width = width;
            traits->height = height;
            traits->red = 8;
            traits->green = 8;
            traits->blue = 8;
            traits->alpha = 8;
            traits->windowDecoration = false;
            traits->pbuffer = true;
            traits->doubleBuffer = true;
            traits->sharedContext = 0;

            pbuffer = osg::GraphicsContext::createGraphicsContext(traits.get());
            // std::cout << "Buffer obj "<< pbuffer->getState()->getMaxBufferObjectPoolSize() << " tex "<<  pbuffer->getState()->getMaxBufferObjectPoolSize() <<std::endl;
            if (pbuffer.valid())
            {
                //   osg::notify(osg::INFO)<<"Pixel buffer has been created successfully."<<std::endl;
            }
            else
            {
                osg::notify(osg::INFO)<<"Pixel buffer has not been created successfully."<<std::endl;
            }

        }



        osg::ref_ptr<WindowCaptureCallback> wcc=new WindowCaptureCallback(mode, position, readBuffer);
        osg::ref_ptr<osg::Camera> camera;
        /* NOT FASTER
    osg::ref_ptr<osg::PixelBufferObject> pboWriteBuffer=new osg::PixelBufferObject;
        osg::ref_ptr<PBOTexturesVisitor> pbotv=new PBOTexturesVisitor(pboWriteBuffer.get());
        node->accept(*pbotv.get());
        pbotv->addPBO();
*/
        if (pbuffer.valid())
        {camera = new osg::Camera;
            camera->setGraphicsContext(pbuffer.get());
            camera->setViewport(new osg::Viewport(0,0,width,height));
            GLenum buffer = pbuffer->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
            camera->setDrawBuffer(buffer);
            camera->setReadBuffer(buffer);
            camera->setFinalDrawCallback(wcc);
            camera->setClearColor(osg::Vec4(0.0,0.0,0.0,0.0));

            if (/*pbufferOnly*/1)
            {
                viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd());

                viewer.realize();
            }
            else
            {
                viewer.realize();

                viewer.stopThreading();

                pbuffer->realize();

                viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd());

                viewer.startThreading();
            }
        }else{
            std::cerr<<"Can't Init Pbuffer\n"<<std::endl;
            return -1;
        }
        /*else
        {
            viewer.realize();

            addCallbackToViewer(viewer, wcc);
        }*/


        // load the data
        osg::Matrix offsetMatrix=osg::Matrix::scale((double)_tileColumns,(double) _tileRows, 1.0)*osg::Matrix::translate((double)_tileColumns-1-2*col, (double)_tileRows-1-2*row, 0.0);


        osg::Timer_t beginRender = osg::Timer::instance()->tick();
        {  //std::cout <<"row: "<<row << " col: "<<col<<" tc: "<<_tileColumns << " "<<_tileRows<<" "<<"\n"<<view << "\n"<<proj <<"\n"<<offsetMatrix<<endl;


            // sem.create(semkey,1,false);

            sem.acquire();
            viewer.setSceneData( node );
            viewer.getCamera()->setProjectionMatrix(proj*offsetMatrix);
            viewer.getCamera()->setViewMatrix(view);
            viewer.frame();
            viewer.advance();
            viewer.updateTraversal();
            viewer.renderingTraversals();
            osg::Timer_t timeEndRender = osg::Timer::instance()->tick();
            double renderTime = osg::Timer::instance()->delta_s(beginRender, timeEndRender);
            printf("Render %.1f\n",renderTime);
            //int mem;
            //gpuUsage(1,mem);
            /*    char aa[1024];sprintf(aa,"render-time-%d.txt",pid);
            FILE *fp=fopen(aa,"w");
            fprintf(fp,"%f s %.2f MB\n",renderTime,sizeImages);
            fclose(fp);
*/
            // osg::Image *img=(wcc->getContextData(pbuffer)->_imageBuffer[wcc->getContextData(pbuffer)->_currentImageIndex]);
            tmpImg1 =  dynamic_cast<osg::Image*>(wcc->getContextData(pbuffer)->_imageBuffer[wcc->getContextData(pbuffer)->_currentImageIndex]->clone(osg::CopyOp::DEEP_COPY_ALL));

            if(untex){
                node->getOrCreateStateSet()->addUniform(new osg::Uniform("shaderOut",3));
                viewer.setSceneData( node );
                viewer.frame();
                viewer.advance();
                viewer.updateTraversal();
                viewer.renderingTraversals();
                osg::Timer_t timeEndRender2 = osg::Timer::instance()->tick();
                double renderTime2 = osg::Timer::instance()->delta_s(timeEndRender, timeEndRender2);
                printf("Render %.1f\n",renderTime2);
                //gpuUsage(1,mem);
                tmpImg2 =  dynamic_cast<osg::Image*>(wcc->getContextData(pbuffer)->_imageBuffer[wcc->getContextData(pbuffer)->_currentImageIndex]->clone(osg::CopyOp::DEEP_COPY_ALL));
            }
            if(depth){
                node->getOrCreateStateSet()->addUniform(new osg::Uniform("shaderOut",3));
                viewer.getCamera()->detach(osg::Camera::DEPTH_BUFFER);
                camera->attach(osg::Camera::DEPTH_BUFFER, depthImage.get(),0,0);
                camera->setComputeNearFarMode(osg::Camera::DO_NOT_COMPUTE_NEAR_FAR);
                viewer.setSceneData( node );

                viewer.frame();
                viewer.advance();
                viewer.updateTraversal();
                viewer.renderingTraversals();

            }

        }
    }

    sem.release();
    char tmp[1024];

    sprintf(tmp,"mesh-diced/image_r%04d_c%04d_rs%04d_cs%04d.%s",row,col,_tileRows,_tileColumns,ext.c_str());
    if(tmpImg1.valid()){
        osgDB::writeImageFile(*tmpImg1,tmp);
    }else
        fprintf(stderr,"Failed to copy image into RAM\n");
    if(untex){

        sprintf(tmp,"mesh-diced/image_r%04d_c%04d_rs%04d_cs%04d_untex.%s",row,col,_tileRows,_tileColumns,ext.c_str());
        if(tmpImg2.valid()){
            osgDB::writeImageFile(*tmpImg2,tmp);
        }else
            fprintf(stderr,"Failed to copy image into RAM\n");
    }
    if(depth){
        sprintf(tmp,"mesh-diced/image_r%04d_c%04d_rs%04d_cs%04d_depth.%s",row,col,_tileRows,_tileColumns,ext.c_str());
        if(depthImage.valid()){
            osgDB::writeImageFile(*depthImage,tmp);
        }else
            fprintf(stderr,"Failed to copy image into RAM\n");
    }
    return 0;
}
void compress(osg::State& state, osg::Texture& texture, osg::Texture::InternalFormatMode compressedFormat, bool generateMipMap, bool resizeToPowerOfTwo)
{


    texture.setInternalFormatMode(compressedFormat);

    // force the mip mapping off temporay if we intend the graphics hardware to do the mipmapping.
    osg::Texture::FilterMode filterMin = texture.getFilter(osg::Texture::MIN_FILTER);
    if (!generateMipMap)
    {
        osg::notify(osg::INFO )<<"   switching off MIP_MAPPING for compile\n";
        texture.setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR);
    }

    // make sure the OSG doesn't rescale images if it doesn't need to.
    texture.setResizeNonPowerOfTwoHint(resizeToPowerOfTwo);

    // get OpenGL driver to create texture from image.
    texture.apply(state);

    texture.getImage(0)->readImageFromCurrentTexture(0,true);

    // restore the mip mapping mode.
    if (!generateMipMap)
    {
        texture.setFilter(osg::Texture::MIN_FILTER,filterMin);
    }
    texture.dirtyTextureObject();
    texture.setInternalFormatMode(osg::Texture::USE_IMAGE_DATA_FORMAT);



}
void generateMipMap(osg::State& state, osg::Texture& texture, bool resizeToPowerOfTwo)
{



    // make sure the OSG doesn't rescale images if it doesn't need to.
    texture.setResizeNonPowerOfTwoHint(resizeToPowerOfTwo);

    // get OpenGL driver to create texture from image.
    texture.apply(state);

    texture.getImage(0)->readImageFromCurrentTexture(0,true);

    texture.setInternalFormatMode(osg::Texture::USE_IMAGE_DATA_FORMAT);

    texture.dirtyTextureObject();


}
