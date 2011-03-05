/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
 *
 * This application is open source and may be redistributed and/or modified
 * freely and without restriction, both in commericial and non commericial applications,
 * as long as this copyright notice is maintained.
 *
 * This application is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osgUtil/Optimizer>
#include <osg/CoordinateSystemNode>

#include <osg/Switch>
#include <osgText/Text>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/ComputeBoundsVisitor>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <iostream>
#include <sstream>
#include <osg/GLObjects>
#include <string.h>
//#if _M_SSE >= 0x401
#include <smmintrin.h>
#include <emmintrin.h>
//#elif _M_SSE >= 0x301 && !(defined __GNUC__ && !defined __SSSE3__)
#include <tmmintrin.h>
//#endif
#include <vips/vips.h>
#include <osg/io_utils>
#define u32 unsigned int
#define u16 unsigned short
#define u8 unsigned char
#define s8 signed char
#define s16 signed short
#define s32 signed int
/* scalar types  */
typedef int8_t          cl_char;
typedef uint8_t         cl_uchar;
typedef int16_t         cl_short    __attribute__((aligned(2)));
typedef uint16_t        cl_ushort   __attribute__((aligned(2)));
typedef int32_t         cl_int      __attribute__((aligned(4)));
typedef uint32_t        cl_uint     __attribute__((aligned(4)));
typedef int64_t         cl_long     __attribute__((aligned(8)));
typedef uint64_t        cl_ulong    __attribute__((aligned(8)));

typedef uint16_t        cl_half     __attribute__((aligned(2)));
typedef float           cl_float    __attribute__((aligned(4)));
typedef double          cl_double   __attribute__((aligned(8)));

#if defined( __SSE2__ )
#if defined( __MINGW64__ )
#include <intrin.h>
#else
#include <emmintrin.h>
#endif
#if defined( __GNUC__ )
typedef cl_uchar    __cl_uchar16    __attribute__((vector_size(16)));
typedef cl_char     __cl_char16     __attribute__((vector_size(16)));
typedef cl_ushort   __cl_ushort8    __attribute__((vector_size(16)));
typedef cl_short    __cl_short8     __attribute__((vector_size(16)));
typedef cl_uint     __cl_uint4      __attribute__((vector_size(16)));
typedef cl_int      __cl_int4       __attribute__((vector_size(16)));
typedef cl_ulong    __cl_ulong2     __attribute__((vector_size(16)));
typedef cl_long     __cl_long2      __attribute__((vector_size(16)));
typedef cl_double   __cl_double2    __attribute__((vector_size(16)));
#else
typedef __m128i __cl_uchar16;
typedef __m128i __cl_char16;
typedef __m128i __cl_ushort8;
typedef __m128i __cl_short8;
typedef __m128i __cl_uint4;
typedef __m128i __cl_int4;
typedef __m128i __cl_ulong2;
typedef __m128i __cl_long2;
typedef __m128d __cl_double2;
#endif
#define __CL_UCHAR16__  1
#define __CL_CHAR16__   1
#define __CL_USHORT8__  1
#define __CL_SHORT8__   1
#define __CL_INT4__     1
#define __CL_UINT4__    1
#define __CL_ULONG2__   1
#define __CL_LONG2__    1
#define __CL_DOUBLE2__  1
#endif


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
class WindowCaptureCallback : public osg::Camera::DrawCallback
{
public:

    enum Mode
    {
        READ_PIXELS,
        SINGLE_PBO,
        DOUBLE_PBO,
        TRIPLE_PBO
    };

    enum FramePosition
    {
        START_FRAME,
        END_FRAME
    };

    struct ContextData : public osg::Referenced
    {

        ContextData(osg::GraphicsContext* gc, Mode mode, GLenum readBuffer, const std::string& name):
                _gc(gc),
                _mode(mode),
                _readBuffer(readBuffer),
                _fileName(name),
                _pixelFormat(GL_BGRA),
                _type(GL_UNSIGNED_BYTE),
                _width(0),
                _height(0),
                _currentImageIndex(0),
                _currentPboIndex(0),
                _reportTimingFrequency(100),
                _numTimeValuesRecorded(0),
                _timeForReadPixels(0.0),
                _timeForFullCopy(0.0),
                _timeForMemCpy(0.0)

        {
            _previousFrameTick = osg::Timer::instance()->tick();

            if (gc->getTraits())
            {
                if (gc->getTraits()->alpha)
                {
                    osg::notify(osg::NOTICE)<<"Select GL_BGRA read back format"<<std::endl;
                    _pixelFormat = GL_BGRA;
                }
                else
                {
                    osg::notify(osg::NOTICE)<<"Select GL_BGR read back format"<<std::endl;
                    _pixelFormat = GL_BGR;
                }
            }

            getSize(gc, _width, _height);

            std::cout<<"Window size "<<_width<<", "<<_height<<std::endl;

            // single buffered image
            _imageBuffer.push_back(new osg::Image);

            // double buffer PBO.
            switch(_mode)
            {
            case(READ_PIXELS):
                osg::notify(osg::NOTICE)<<"Reading window usig glReadPixels, with out PixelBufferObject."<<std::endl;
                break;
            case(SINGLE_PBO):
                osg::notify(osg::NOTICE)<<"Reading window usig glReadPixels, with a single PixelBufferObject."<<std::endl;
                _pboBuffer.push_back(0);
                break;
            case(DOUBLE_PBO):
                osg::notify(osg::NOTICE)<<"Reading window usig glReadPixels, with a double buffer PixelBufferObject."<<std::endl;
                _pboBuffer.push_back(0);
                _pboBuffer.push_back(0);
                break;
            case(TRIPLE_PBO):
                osg::notify(osg::NOTICE)<<"Reading window usig glReadPixels, with a triple buffer PixelBufferObject."<<std::endl;
                _pboBuffer.push_back(0);
                _pboBuffer.push_back(0);
                _pboBuffer.push_back(0);
                break;
            default:
                break;
            }
        }

        void getSize(osg::GraphicsContext* gc, int& width, int& height)
        {
            if (gc->getTraits())
            {
                width = gc->getTraits()->width;
                height = gc->getTraits()->height;
            }
        }

        void updateTimings(osg::Timer_t tick_start,
                           osg::Timer_t tick_afterReadPixels,
                           osg::Timer_t tick_afterMemCpy,
                           unsigned int dataSize);

        void read()
        {
            osg::GLBufferObject::Extensions* ext = osg::GLBufferObject::getExtensions(_gc->getState()->getContextID(),true);

            if (ext->isPBOSupported() && !_pboBuffer.empty())
            {
                if (_pboBuffer.size()==1)
                {
                    singlePBO(ext);
                }
                else
                {
                    multiPBO(ext);
                }
            }
            else
            {
                readPixels();
            }
        }

        void readPixels();

        void singlePBO(osg::GLBufferObject::Extensions* ext);

        void multiPBO(osg::GLBufferObject::Extensions* ext);

        typedef std::vector< osg::ref_ptr<osg::Image> >             ImageBuffer;
        typedef std::vector< GLuint > PBOBuffer;

        osg::GraphicsContext*   _gc;
        Mode                    _mode;
        GLenum                  _readBuffer;
        std::string             _fileName;

        GLenum                  _pixelFormat;
        GLenum                  _type;
        int                     _width;
        int                     _height;

        unsigned int            _currentImageIndex;
        ImageBuffer             _imageBuffer;

        unsigned int            _currentPboIndex;
        PBOBuffer               _pboBuffer;

        unsigned int            _reportTimingFrequency;
        unsigned int            _numTimeValuesRecorded;
        double                  _timeForReadPixels;
        double                  _timeForFullCopy;
        double                  _timeForMemCpy;
        osg::Timer_t            _previousFrameTick;
    };

    WindowCaptureCallback(Mode mode, FramePosition position, GLenum readBuffer):
            _mode(mode),
            _position(position),
            _readBuffer(readBuffer)
    {
    }

    FramePosition getFramePosition() const { return _position; }

    ContextData* createContextData(osg::GraphicsContext* gc) const
    {
        std::stringstream filename;
        filename << "test_"<<_contextDataMap.size()<<".jpg";
        return new ContextData(gc, _mode, _readBuffer, filename.str());
    }

    ContextData* getContextData(osg::GraphicsContext* gc) const
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
        osg::ref_ptr<ContextData>& data = _contextDataMap[gc];
        if (!data) data = createContextData(gc);

        return data.get();
    }

    virtual void operator () (osg::RenderInfo& renderInfo) const
    {
        glReadBuffer(_readBuffer);

        osg::GraphicsContext* gc = renderInfo.getState()->getGraphicsContext();
        osg::ref_ptr<ContextData> cd = getContextData(gc);
        cd->read();
    }

    typedef std::map<osg::GraphicsContext*, osg::ref_ptr<ContextData> > ContextDataMap;

    Mode                        _mode;
    FramePosition               _position;
    GLenum                      _readBuffer;
    mutable OpenThreads::Mutex  _mutex;
    mutable ContextDataMap      _contextDataMap;


};

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

        int prec = osg::notify(osg::NOTICE).precision(5);

        if (timeForMemCpy==0.0)
        {
            osg::notify(osg::NOTICE)<<"fps = "<<fps<<", full frame copy = "<<timeForFullCopy*1000.0f<<"ms rate = "<<numMPixels / timeForFullCopy<<" Mpixel/sec, copy speed = "<<numMb / timeForFullCopy<<" Mb/sec"<<std::endl;
        }
        else
        {
            osg::notify(osg::NOTICE)<<"fps = "<<fps<<", full frame copy = "<<timeForFullCopy*1000.0f<<"ms rate = "<<numMPixels / timeForFullCopy<<" Mpixel/sec, "<<numMb / timeForFullCopy<< " Mb/sec "<<
                    "time for memcpy = "<<timeForMemCpy*1000.0<<"ms  memcpy speed = "<<numMb / timeForMemCpy<<" Mb/sec"<<std::endl;
        }
        osg::notify(osg::NOTICE).precision(prec);

    }

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
        osg::notify(osg::NOTICE)<<"Allocating image "<<std::endl;
        image->allocateImage(_width, _height, 1, GL_RGBA, _type);

        if (pbo!=0)
        {
            osg::notify(osg::NOTICE)<<"deleting pbo "<<pbo<<std::endl;
            ext->glDeleteBuffers (1, &pbo);
            pbo = 0;
        }
    }


    if (pbo==0)
    {
        ext->glGenBuffers(1, &pbo);
        ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, pbo);
        ext->glBufferData(GL_PIXEL_PACK_BUFFER_ARB, image->getTotalSizeInBytes(), 0, GL_STREAM_READ);

        osg::notify(osg::NOTICE)<<"Generating pbo "<<pbo<<std::endl;
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
        int pitch=image->getRowSizeInBytes();//4*_width;
        image->setPixelFormat(GL_RGBA);
        ConvertRGBA_BGRA_SSSE3((unsigned int *)image->data(),pitch,(unsigned int *)src,_width,_height,pitch/4);
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
        osg::notify(osg::NOTICE)<<"Allocating image "<<std::endl;
        image->allocateImage(_width, _height, 1, _pixelFormat, _type);

        if (read_pbo!=0)
        {
            osg::notify(osg::NOTICE)<<"deleting pbo "<<read_pbo<<std::endl;
            ext->glDeleteBuffers (1, &read_pbo);
            read_pbo = 0;
        }

        if (copy_pbo!=0)
        {
            osg::notify(osg::NOTICE)<<"deleting pbo "<<copy_pbo<<std::endl;
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

        osg::notify(osg::NOTICE)<<"Generating pbo "<<read_pbo<<std::endl;
    }

    if (read_pbo==0)
    {
        ext->glGenBuffers(1, &read_pbo);
        ext->glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, read_pbo);
        ext->glBufferData(GL_PIXEL_PACK_BUFFER_ARB, image->getTotalSizeInBytes(), 0, GL_STREAM_READ);

        osg::notify(osg::NOTICE)<<"Generating pbo "<<read_pbo<<std::endl;
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
                osg::notify(osg::NOTICE)<<"First camera "<<firstCamera<<std::endl;

                firstCamera->setInitialDrawCallback(callback);
            }
            else
            {
                osg::notify(osg::NOTICE)<<"No camera found"<<std::endl;
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
                osg::notify(osg::NOTICE)<<"Last camera "<<lastCamera<<std::endl;

                lastCamera->setFinalDrawCallback(callback);
            }
            else
            {
                osg::notify(osg::NOTICE)<<"No camera found"<<std::endl;
            }
        }
    }
}
typedef struct _picture_cell{
    int row;
    int col;
    std::vector<int> idx;

    std::vector<std::string> names;
}picture_cell;
struct cell_compare {
    bool operator ()(picture_cell const& a, picture_cell const& b) const {
        if (a.idx.size() < b.idx.size()) return true;
        if (a.idx.size()  > b.idx.size() ) return false;

        for(int i=0; i < a.idx.size(); i++){
            if (a.idx[i] < b.idx[i]) return true;
            if (a.idx[i] > b.idx[i]) return false;

        }

        return false;
    }
};


bool cell_equal(picture_cell const& a, picture_cell const& b)  {
    if (a.idx.size() != b.idx.size()) return false;

    for(int i=0; i < a.idx.size(); i++){
        if (a.idx[i] != b.idx[i]) return false;

    }

    return true;
}
int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] filename ...");

    unsigned int width=1280;
    unsigned int height=1024;
    arguments.read("--pbuffer-only",width,height);

    //  osg::BoundingSphere bs;
    //osg::BoundingBox bb ;
    osg::BoundingBox totalbb;
    std::string _tileBasename="/home/mattjr/data/d100/real_root_L0_X0_Y0/real";
    int currentLevel=2;
    int   xdel[] = { 0,-1, 1, 0,  0,1,-1,1,-1};
    int   ydel[] = { 0, 0, 0,-1,  1,1,-1,-1,1};
    //  for(int modelX=0; modelX<4; modelX++){
    //    for(int modelY=0; modelY<4; modelY++){
    std::vector<std::pair<osg::BoundingBox,std::string> > bboxes;
    FILE *fp=fopen(argv[1],"r");
    if(!fp){
        fprintf(stderr,"Can't open %s\n",argv[1]);
        exit(-1);
    }
    int cnt=0;
    while(!feof(fp)){
        char fname[1024];
        float minx,maxx,miny,maxy;
        int res=fscanf(fp,"%f %f %f %f %s\n",&minx,&maxx,&miny,&maxy,fname);
        if(res != 5){
            fprintf(stderr,"Bad parse\n");
            continue;
        }
        if(cnt==0){
            totalbb=osg::BoundingBox(minx,miny,46,maxx,maxy,49);

        }else{
            bboxes.push_back(std::make_pair<osg::BoundingBox,std::string>(osg::BoundingBox(minx,miny,0.0,maxx,maxy,1.0),fname));
            std::cout << " bbox " << bboxes.back().first._min<< " " << bboxes.back().first._max << std::endl;

        }
        cnt++;

    }
    std::cout << "total bbox " << totalbb._min<< " " << totalbb._max << std::endl;

    osg::Vec3d eye(totalbb.center()+osg::Vec3(0,0,3.5*totalbb.radius()));

    osg::Matrixd matrix;
    matrix.makeTranslate( eye );
    osg::Matrixd view=osg::Matrix::inverse(matrix);
    osg::Vec3 centeredMin,centeredMax;
    // double margin=0.05*totalbb.radius();
    // bb._min-=osg::Vec3(margin,margin,margin);
    // bb._max+=osg::Vec3(margin,margin,margin);


    centeredMin=(totalbb._min-totalbb.center());
    centeredMax=(totalbb._max-totalbb.center());
    osg::Matrixd proj= osg::Matrixd::ortho2D(centeredMin[0],centeredMax[0],centeredMin[1],centeredMax[1]);

    std::stringstream os2;
    os2<< "view.mat";

    std::fstream _file(os2.str().c_str(),std::ios::binary|std::ios::out);
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            _file.write(reinterpret_cast<char*>(&(view(i,j))),sizeof(double));
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            _file.write(reinterpret_cast<char*>(&(proj(i,j))),sizeof(double));
    _file.close();

    int _tileColumns=4;
    int _tileRows=8;
    osg::Vec3 deltaV=totalbb._max-totalbb._min;
    deltaV.x()/= _tileColumns;
    deltaV.y()/= _tileRows;
    IMAGE *raw;
    std::ostringstream os;
    os <<"subtile.tif";
    if( !(raw = im_open(  os.str().c_str(), "w" )) )
        return( -1 );


    /* Make sure we can write PIO-style.
 */
    if( im_poutcheck( raw ) )
        return( -1 );

    /* Process and save as VIPS.
 */
    if( im_demand_hint( raw, IM_SMALLTILE, NULL ))
        fprintf(stderr,"Freakout");
    im_initdesc(raw,width*_tileColumns,height*_tileRows,4,IM_BBITS_BYTE,IM_BANDFMT_UCHAR,IM_CODING_NONE,IM_TYPE_sRGB,1.0,1.0,0,0);
    if( im_setupout( raw ) ){
        fprintf(stderr,"Fail!\n");
    }
    //  osg::ref_ptr<osg::Node> loadedModel =osgDB::readNodeFile("/home/mattjr/data/d100/mesh-diced/total.ply");
    std::vector<picture_cell> cells;
    for(int row=0; row< _tileRows; row++){
        for(int col=0; col<_tileColumns; col++){

            bool centerValid=false;

            osg::BoundingBox thisCellBbox;
            thisCellBbox._min=totalbb._min+osg::Vec3(col*deltaV.x(),row*deltaV.y(),0.0);
            thisCellBbox._max=totalbb._min+osg::Vec3((col+1)*deltaV.x(),(row+1)*deltaV.y(),1.0);
            thisCellBbox._min[2]=0;
            thisCellBbox._max[2]=1;
            //  std::cout << "this bbox " << thisCellBbox._min<< " " << thisCellBbox._max << std::endl;
            osg::Timer_t tick_start = osg::Timer::instance()->tick();
            //for(int i=0; i < 9; i++){
            //   int offsetX=xdel[i];
            // int offsetY=ydel[i];
            //    os << _tileBasename << "_L"<<currentLevel<<"_X"<<modelX+offsetX<<"_Y"<<modelY+offsetY<<"_subtile.ive";
            //files.push_back(os.str());
            char tmp[1024];
            //sprintf(tmp,"%08d",i);
            //os << _tileBasename << "_L"<<currentLevel<<"_X"<<modelX+offsetX<<"_Y"<<modelY+offsetY<<"_subtile.ive";
            picture_cell cell;
            cell.row=row;
            cell.col=col;
            std::cout << " == " << thisCellBbox._min<< " " << thisCellBbox._max << std::endl;

            for(int i=0; i< bboxes.size(); i++){
                // std::cout << " A " << bboxes[i].first._min<< " " << bboxes[i].first._max;
                double eps=0.5;
                if(osg::Vec3(bboxes[i].first._min -thisCellBbox._min).length()  < eps && osg::Vec3(bboxes[i].first._max - thisCellBbox._max).length() < eps)//if(bboxes[i].first.intersects(thisCellBbox) )
                {
                    std::ostringstream os;

                    os<< argv[2] <<"/"<<bboxes[i].second;
                    std::cout << os.str() <<std::endl;
                    //std::cout << "row " << row << " col "<<col <<" i: " <<osgDB::getNameLessAllExtensions(osgDB::getSimpleFileName(os.str()))<<"\n";
                    //if(osgDB::fileExists(os.str())){
                    cell.names.push_back(osgDB::getNameLessExtension(os.str())+".ive");
                    cell.idx.push_back(i);

                    //loadedModel->addChild(node);
                    //}
                }


            }
            std::sort(cell.idx.begin(),cell.idx.end());

            cells.push_back(cell);

        }
    }
    int mult_count=0;
    std::sort(cells.begin(),cells.end(),cell_compare());
    for(int i=0; i < cells.size(); i++){
        if(cells[i].names.size() > 1)
            mult_count++;
        for(int j=0; j < cells[i].names.size(); j++)
            std::cout << cells[i].idx[j] << " ";
        std::cout << "\n\n  ";
    }
    printf("Mult count %d\n",mult_count);
    osg::ref_ptr<osg::Group> loadedModel;

    for(int i=0; i < cells.size(); i++)
    //int i=3;
    {
        osgViewer::Viewer viewer(arguments);

        unsigned int helpType = 0;
        if ((helpType = arguments.readHelpType()))
        {
            arguments.getApplicationUsage()->write(std::cout, helpType);
            return 1;
        }

        // report any errors if they have occurred when parsing the program arguments.
        if (arguments.errors())
        {
            arguments.writeErrorMessages(std::cout);
            return 1;
        }

        if (arguments.argc()<=1)
        {
            arguments.getApplicationUsage()->write(std::cout,osg::ApplicationUsage::COMMAND_LINE_OPTION);
            return 1;
        }

        GLenum readBuffer = GL_BACK;
        WindowCaptureCallback::FramePosition position = WindowCaptureCallback::END_FRAME;
        WindowCaptureCallback::Mode mode = WindowCaptureCallback::SINGLE_PBO;

        while (arguments.read("--start-frame")) { position = WindowCaptureCallback::START_FRAME; readBuffer = GL_FRONT; }
        while (arguments.read("--end-frame")) position = WindowCaptureCallback::END_FRAME;

        while (arguments.read("--front")) readBuffer = GL_FRONT;
        while (arguments.read("--back")) readBuffer = GL_BACK;

        while (arguments.read("--no-pbo")) mode = WindowCaptureCallback::READ_PIXELS;
        while (arguments.read("--single-pbo")) mode = WindowCaptureCallback::SINGLE_PBO;
        while (arguments.read("--double-pbo")) mode = WindowCaptureCallback::DOUBLE_PBO;
        while (arguments.read("--triple-pbo")) mode = WindowCaptureCallback::TRIPLE_PBO;
     osg::Texture::TextureObjectManager* tom;


        bool pbufferOnly = true;
        osg::ref_ptr<osg::GraphicsContext> pbuffer;
    //    if (arguments.read("--pbuffer",width,height) ||
      //      (pbufferOnly = arguments.read("--pbuffer-only",width,height)))
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
            std::cout << "Buffer obj "<< pbuffer->getState()->getMaxBufferObjectPoolSize() << " tex "<<  pbuffer->getState()->getMaxBufferObjectPoolSize() <<std::endl;
            tom= osg::Texture::getTextureObjectManager(pbuffer->getState()->getContextID()).get();
            if (pbuffer.valid())
            {
                osg::notify(osg::NOTICE)<<"Pixel buffer has been created successfully."<<std::endl;
            }
            else
            {
                osg::notify(osg::NOTICE)<<"Pixel buffer has not been created successfully."<<std::endl;
            }

        }



        WindowCaptureCallback *wcc=new WindowCaptureCallback(mode, position, readBuffer);
        osg::ref_ptr<osg::Camera> camera;

        if (pbuffer.valid())
        {camera = new osg::Camera;
            camera->setGraphicsContext(pbuffer.get());
            camera->setViewport(new osg::Viewport(0,0,width,height));
            GLenum buffer = pbuffer->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
            camera->setDrawBuffer(buffer);
            camera->setReadBuffer(buffer);
            camera->setFinalDrawCallback(wcc);

            if (pbufferOnly)
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
        }
        else
        {
            viewer.realize();

            addCallbackToViewer(viewer, wcc);
        }
        viewer.setReleaseContextAtEndOfFrameHint(true);
        // load the data
        osg::Matrix offsetMatrix=   osg::Matrix::scale(_tileColumns, _tileRows, 1.0) *osg::Matrix::translate(_tileColumns-1-2*cells[i].col, _tileRows-1-2*cells[i].row, 0.0);
        printf("\r%03d/%03d",i,cells.size());
        fflush(stdout);
        osg::ref_ptr<osg::Node> node=osgDB::readNodeFile(cells[i].names[0]);
        bool load=false;
        if(i == 0)
            load=true;
        else
            if(!cell_equal(cells[i],cells[i-1]))
                load=true;

        if(cells[i].idx.size() ==0){
            fprintf(stderr,"No data\n");
            load=false;
        }
        //if(cells[i].idx.size() > 3)
        //  load=false;

       // if(load){
        //    loadedModel =0;
           // loadedModel=new osg::Group;
           // for(int j=0; j < cells[i].names.size(); j++){

            //    osg::ref_ptr<osg::Node> node= osgDB::readNodeFile(cells[i].names[j]);
              //  std::cout << "\nLoading: "<<osgDB::getSimpleFileName(cells[i].names[j]) << "\n";
               // loadedModel->addChild(node);
                //std::cout << "Loaded\n";

          //  }
       // }


        if (node.valid())// && loadedModel->getNumChildren() > 0)
        {

            // optimize the scene graph, remove redundant nodes and state etc.
            //osgUtil::Optimizer optimizer;
            //optimizer.optimize(node.get());
            osg::Timer_t tick_afterReadPixels = osg::Timer::instance()->tick();

            //double timeForReadPixels = osg::Timer::instance()->delta_s(tick_start, tick_afterReadPixels);
            //printf("Time for read %.2f\n",timeForReadPixels);
            viewer.setSceneData( node );




            std::ostringstream os2;

            // os << _tileBasename << "_L"<<currentLevel<<"_X"<<modelX<<"_Y"<<modelY<<"_subtile.tif";
            // os2 << _tileBasename << "_L"<<currentLevel<<"_X"<<modelX<<"_Y"<<modelY<<"_subtile.txt";
            // char tmp[1024];
            //zsprintf(tmp,"%08d",i);
            //os<< "/home/mattjr/data/dall_6/mesh-diced/clipped-diced-"<<tmp<<"-lod3.tif";
            /*os2<< "/home/mattjr/data/dall_6/mesh-diced/clipped-diced-"<<tmp<<"-lod3.mat";

                std::fstream _file(os2.str().c_str(),std::ios::binary|std::ios::out);
                for(int i=0; i<4; i++)
                    for(int j=0; j<4; j++)
                        _file.write(reinterpret_cast<char*>(&(view(i,j))),sizeof(double));
                for(int i=0; i<4; i++)
                    for(int j=0; j<4; j++)
                        _file.write(reinterpret_cast<char*>(&(proj(i,j))),sizeof(double));
                _file.close();
                */

            IMAGE *im;
            if( !(im = im_open( "temp", "t" )) )
                fprintf(stderr,"Freakout");

            im_initdesc(im,width,height,4,IM_BBITS_BYTE,IM_BANDFMT_UCHAR,IM_CODING_NONE,IM_TYPE_sRGB,1.0,1.0,0,0);


            if( im_setupout( im ) ){
                fprintf(stderr,"Fail!\n");
            }
            if(im_incheck(im)){
                fprintf(stderr,"Fail!\n");
            }


            char tmpn[255];


            viewer.getCamera()->setProjectionMatrix(proj*offsetMatrix);
            viewer.getCamera()->setViewMatrix(view);
            //     std::cout << proj << " " <<view <<std::endl;
            //  for(int t=0; t<2; t++){
            viewer.frame();
            viewer.advance();
            viewer.updateTraversal();
            viewer.renderingTraversals();
            //  }
            osg::Image *img=(wcc->getContextData(pbuffer)->_imageBuffer[wcc->getContextData(pbuffer)->_currentImageIndex]);
            //         for(int i = 0; i < img->t(); ++i) {
            //         im_writeline(i,im, (PEL *)img->data(0,img->t() - i - 1));
            //     }
            /*  char *ptr=im->data;
            unsigned char *src=img->data();
            for(int i=0; i<(int)img->getImageSizeInBytes(); i+=4,ptr+=3){
                *(ptr+0)= *(src+i+0);
                *(ptr+1)= *(src+i+1);
                *(ptr+2)= *(src+i+2);

            }
*/
            if(_tileColumns ==1 && _tileRows ==1 ){
                memcpy(raw->data,img->data(),img->getImageSizeInBytes());

            }else{
                memcpy(im->data,img->data(),img->getImageSizeInBytes());
                // sprintf(tmpn,"%d_%d.tif",row,col);
                /*       IMAGE *out;
        out=im_open( tmpn, "w" );
    if( im_copy( im, out) )
        printf("Fail!\n");
    im_close(out);*/
                if(im_insertplace(raw,im,width*cells[i].col,height*cells[i].row))
                    printf("Fail!\n");
            }

            //   return(-1);
            //osgDB::writeImageFile(*img,tmpn);
            im_close(im);

        }else{
            std::cout << "Prob shouldn't get here\n";
        }
      /*  node->releaseGLObjects();
        tom->deleteAllTextureObjects();
        tom->flushAllDeletedTextureObjects();
        pbuffer->releaseGLObjects(0);
        viewer.releaseGLObjects(0);
        osg::deleteAllGLObjects(pbuffer->getState()->getContextID());
        osgViewer::ViewerBase::Contexts contexts;

        viewer.getContexts(contexts);

        // clear out all the previously assigned operations
        for(osgViewer::ViewerBase::Contexts::iterator citr = contexts.begin();
            citr != contexts.end();
            ++citr)
        {
            //(*citr)->close();

            osg::deleteAllGLObjects(   (*citr)->getState()->getContextID());
         //   (*citr)->getState// close();
        }
        viewer.setSceneData(NULL);
*/
        ///osg::deleteAllGLObjects(viewer.getCamera()->->getContextID());

    }
    im_close(raw);

}

// }


