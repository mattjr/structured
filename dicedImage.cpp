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
#include <vips/vips>
#include <iomanip>
using namespace std;
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
void applyGeoTags(osg::Vec2 geoOrigin,osg::Matrix viewMatrix,osg::Matrix projMatrix,int width,int height);
void applyGeoTags2(const char* filename,osg::Matrix viewMatrix,osg::Matrix projMatrix,osg::Matrix win,osg::Vec2 geoOrigin,int width,int height);
#include <geotiff.h>
#include <xtiffio.h>
#include <GeographicConversions/ufRedfearn.h>
#include <limits>
#include <adt_write_gtiff.hpp>
#include <auv_map_projection.hpp>
#include <adt_file_utils.hpp>
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
                    osg::notify(osg::INFO)<<"Select GL_BGRA read back format"<<std::endl;
                    _pixelFormat = GL_BGRA;
                }
                else
                {
                    osg::notify(osg::INFO)<<"Select GL_BGR read back format"<<std::endl;
                    _pixelFormat = GL_BGR;
                }
            }

            getSize(gc, _width, _height);

            osg::notify(osg::INFO)<<"Window size "<<_width<<", "<<_height<<std::endl;

            // single buffered image
            _imageBuffer.push_back(new osg::Image);

            // double buffer PBO.
            switch(_mode)
            {
            case(READ_PIXELS):
                osg::notify(osg::INFO)<<"Reading window usig glReadPixels, with out PixelBufferObject."<<std::endl;
                break;
            case(SINGLE_PBO):
                osg::notify(osg::INFO)<<"Reading window usig glReadPixels, with a single PixelBufferObject."<<std::endl;
                _pboBuffer.push_back(0);
                break;
            case(DOUBLE_PBO):
                osg::notify(osg::INFO)<<"Reading window usig glReadPixels, with a double buffer PixelBufferObject."<<std::endl;
                _pboBuffer.push_back(0);
                _pboBuffer.push_back(0);
                break;
            case(TRIPLE_PBO):
                osg::notify(osg::INFO)<<"Reading window usig glReadPixels, with a triple buffer PixelBufferObject."<<std::endl;
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

typedef struct _picture_cell{
    int row;
    int col;
    osg::BoundingBox bbox;
    std::string name;
}picture_cell;
int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] filename ...");
    bool pbufferOnly = !arguments.read("--show");

    unsigned int width=1280;
    unsigned int height=1024;
    arguments.read("--pbuffer-only",width,height);
    int _tileColumns;
    int _tileRows;

    osg::BoundingBox totalbb;

    double lat=0,lon=0;
    arguments.read("-lat",lat);
    arguments.read("-lon",lon);

    std::vector<picture_cell> cells;
    FILE *fp=fopen(argv[1],"r");
    if(!fp){
        fprintf(stderr,"Can't open %s\n",argv[1]);
        exit(-1);
    }
    int cnt=0;
    while(!feof(fp)){
        char fname[1024];
        float minx,maxx,miny,maxy,minz,maxz;
        int row,col;
        int res=fscanf(fp,"%f %f %f %f %f %f %d %d %s\n",&minx,&maxx,&miny,&maxy,&minz,&maxz,&row,&col,fname);
        if(res != 9){
            fprintf(stderr,"Bad parse\n");
            exit(-1);
        }
        if(cnt==0){
            //totalbb=osg::BoundingBox(minx,miny,minz,maxx,maxy,maxz);
            totalbb=osg::BoundingBox(miny,minx,-maxz,maxy,maxx,-minz);
            _tileColumns=col;
            _tileRows=row;

        }else{
            picture_cell cell;
            //cell.bbox=osg::BoundingBox(minx,miny,minz,maxx,maxy,maxz);
           cell.bbox= osg::BoundingBox(miny,minx,-maxz,maxy,maxx,-minz);
            cell.col=col;
            cell.row=row;
            if(std::string(fname) != "null")
             cell.name=std::string(argv[2])+"/"+std::string(fname);
            else
                 cell.name=std::string(fname);
            cells.push_back(cell);
           // std::cout << " bbox " << bboxes.back().first._min<< " " << bboxes.back().first._max << std::endl;

        }
        cnt++;

    }


    osg::BoundingSphere bs(totalbb);
double dist=3.5*bs.radius();
std::cout << "Bounding " << totalbb._min << " "<<totalbb._max<<"\n";

  //  osg::Vec3d eye(bs.center()+osg::Vec3(0,0,dist));
   // osg::Matrixd matrix;
   // matrix.makeTranslate( eye );
   // osg::Matrixd view=osg::Matrix::identity();//inverse(matrix);
   // view =osg::Matrixd::lookAt(bs.center()+ osg::Vec3d(0.0,-dist,0.0f), bs.center(), osg::Vec3(0.0,0.0,1.0));


//osg::Matrixd view=osg::Matrixd::lookAt(bs.center(), bs.center() +osg::Vec3d(0.0,0.0f,dist), osg::Vec3(1.0,0.0,0.0));

  //  osg::Matrixd proj= osg::Matrixd::ortho2D(-bs.radius(),bs.radius(),-bs.radius(),bs.radius());




//proj.postMult(trans);
     osg::Vec3 centeredMin,centeredMax;
    centeredMin=(totalbb._min-totalbb.center());
        centeredMax=(totalbb._max-totalbb.center());
   osg::Matrixd proj= osg::Matrixd::ortho2D(-bs.radius(),bs.radius(),-bs.radius(),bs.radius());//osg::Matrixd::ortho2D(centeredMin[0],centeredMax[0],centeredMin[1],centeredMax[1]);
   osg::Vec3d eye(bs.center()+osg::Vec3(0,0,3.5*bs.radius()));
   osg::Matrixd matrix;
      matrix.makeTranslate( eye );
       osg::Matrixd view=osg::Matrix::inverse(matrix);
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


    osg::Vec3 deltaV=totalbb._max-totalbb._min;
    deltaV.x()/= _tileColumns;
    deltaV.y()/= _tileRows;
    std::ostringstream os;
    os <<"subtile.v";//<<":deflate";
    vips::VImage raw;

    raw.initdesc(width*_tileColumns,height*_tileRows,3,vips::VImage::FMTUCHAR,vips::VImage::NOCODING,vips::VImage::sRGB,1.0,1.0,0,0);


    osg::Matrix win;


    for(int i=0; i < (int)cells.size(); i++)
    {
        if(cells[i].name == "null")
            continue;
        osgViewer::Viewer viewer(arguments);



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
            //std::cout << "Buffer obj "<< pbuffer->getState()->getMaxBufferObjectPoolSize() << " tex "<<  pbuffer->getState()->getMaxBufferObjectPoolSize() <<std::endl;
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

        // load the data
        osg::Matrix offsetMatrix=osg::Matrix::scale(_tileColumns, _tileRows, 1.0) *osg::Matrix::translate(_tileColumns-1-2*cells[i].col, _tileRows-1-2*cells[i].row, 0.0);
        printf("\r%03d/%03d",i,(int)cells.size());
        fflush(stdout);
     //   if()
        osg::ref_ptr<osg::Node> node=osgDB::readNodeFile(/*cells[i].name*/string("mesh-diced/tex-clipped-diced-r_0007_c_0000-lod3.ive")+".0,180,-90.rot");//");

        if (node.valid() && cells[i].row ==_tileRows-1 )
        {
            viewer.setSceneData( node );
            viewer.getCamera()->setProjectionMatrix(proj*offsetMatrix);
            viewer.getCamera()->setViewMatrix(view);
            //for(int p=0; p<; p++){
        viewer.frame();
            viewer.advance();
            viewer.updateTraversal();
            viewer.renderingTraversals();

            win= osg::Matrix::translate(1.0,1.0,1.0)*osg::Matrix::scale(0.5*raw.Xsize(),0.5*raw.Ysize(),0.5f)*osg::Matrix::translate(0,0,0.0f);//viewer.getCamera()->getViewport()->computeWindowMatrix();
            osg::Image *img=(wcc->getContextData(pbuffer)->_imageBuffer[wcc->getContextData(pbuffer)->_currentImageIndex]);
           // img->flipHorizontal();
        //  img->flipVertical();
       //    osgDB::writeImageFile(*img,cells[i].name+".png");

            vips::VImage tmp(img->data(),img->s(),img->t(),4,vips::VImage::FMTUCHAR);
            raw.insertplace(tmp.flipver().extract_bands(0,3),width*cells[i].col,height*(_tileRows-cells[i].row-1));

        }else{
            std::cout << "Invalid " << cells[i].name << "\n";
        }


    }
    raw.write("subtile.v");
    applyGeoTags(osg::Vec2(lat,lon),view,proj,raw.Xsize(),raw.Ysize());
  //  applyGeoTags2("/home/mattjr/data/d100/out.tif",view,proj,win,osg::Vec2(lat,lon),raw.Xsize(),raw.Ysize());

}


void applyGeoTags(osg::Vec2 geoOrigin,osg::Matrix viewMatrix,osg::Matrix projMatrix,int width,int height){
    osg::Matrix trans;/*(
            osg::Matrix::rotate(osg::inDegrees(-90.0f),
                                1.0f,0.0f,0.0f)*
            osg::Matrix::rotate(osg::inDegrees(-90.0f),0.0f,
                                1.0f,0.0f));
*/
    osg::Matrix modWindow =( osg::Matrix::translate(1.0,1.0,1.0)*osg::Matrix::scale(0.5*width,0.5*height,0.5f));
    osg::Matrix bottomLeftToTopLeft= (osg::Matrix::scale(1,-1,1)*osg::Matrix::translate(0,height,0));
    osg::Matrix worldtoScreen=trans*viewMatrix * projMatrix * modWindow*bottomLeftToTopLeft;
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

    std::ofstream worldfile("geo_tif.tfw");
    worldfile << std::setprecision(12) << scalePix.x() << std::endl<< 0 <<std::endl<< 0 << std::endl<<scalePix.y() << std::endl<<tlGlobal.x()<<std::endl<<tlGlobal.y()<<std::endl;
    std::cout << tlGlobal << " " << blGlobal <<" " <<trGlobal<< " " << brGlobal <<"\n";
    char gdal_param[4096];
    sprintf(gdal_param," -of GTiff -co \"TILED=YES\" -a_ullr %.12f %.12f %.12f %.12f -a_srs %s",tlGlobal.y(),brGlobal.x(),brGlobal.y(),tlGlobal.x(),szProj4);
    //sprintf(gdal_param," -of GTiff -co \"TILED=YES\" -a_ullr %.12f %.12f %.12f %.12f -a_srs %s",tlGlobal.x(),tlGlobal.y(),brGlobal.x(),brGlobal.y(),szProj4);

    std::ofstream gdalcommand("add_geo.sh");
    if(!gdalcommand.good())
        std::cerr << "Failed!\n";
    gdalcommand << "#!/bin/bash\n";
    gdalcommand << "vips im_vips2tiff subtile.v out.tif:none:tile:256x256\n";

    //gdalcommand << "geotifcp -e geo_tif.tfw -4 " << szProj4 << " out.tif geo_tif1.tif\n";

    gdalcommand << "gdal_translate " << gdal_param << " out.tif geo_tif2.tif\n";
    sprintf(gdal_param," -of GTiff -co \"TILED=YES\" -a_ullr %.12f %.12f %.12f %.12f -a_srs %s",tlGlobal.x(),trGlobal.y(),blGlobal.x(),blGlobal.y(),szProj4);
    gdalcommand << "#gdal_translate " << gdal_param << " out.tif geo_tif.tif\n";
    system("sh ./add_geo.sh");

}


void applyGeoTags2(const char* filename,osg::Matrix viewMatrix,osg::Matrix projMatrix,osg::Matrix win,osg::Vec2 geoOrigin,int width,int height){
    gtiffspace::geo_info pos;
    libplankton::Local_WGS84_TM_Projection *map_projection = new libplankton::Local_WGS84_TM_Projection( geoOrigin.x(),
                                                                                                         geoOrigin.y());//glong

    osg::Matrix trans(
            osg::Matrix::rotate(osg::inDegrees(-90.0f),
                                1.0f,0.0f,0.0f)*
            osg::Matrix::rotate(osg::inDegrees(-90.0f),0.0f,
                                1.0f,0.0f));
    int _tileColumns=1;
    int _tileRows=1;


    osg::Matrix modWindow=osg::Matrix( osg::Matrix::scale(_tileColumns, _tileRows, 1.0) *win);
    modWindow(3,0)*=_tileColumns;
    modWindow(3,1)*=_tileRows;
    osg::Matrix bottomLeftToTopLeft= (osg::Matrix::scale(1,-1,1)*osg::Matrix::translate(0,height,0));
    //std::cout << bottomLeftToTopLeft<<std::endl;
    //std::cout <<modWindow<<std::endl;
    osg::Matrix worldtoScreen=trans*viewMatrix * projMatrix * modWindow*bottomLeftToTopLeft;
    osg::Matrix screenToWorld=osg::Matrix::inverse(worldtoScreen);
    osg::Vec3 tl(0,0,0);
    osg::Vec3 bl(0,height,0);
    osg::Vec3 tr(width,0,0);
    osg::Vec3 br(width,height,0);

    osg::Vec3 tlGlobal=tl*screenToWorld;
    cout << "TL Global" << tlGlobal<<endl;
    double latTL,longTL;
    map_projection->calc_geo_coords(tlGlobal.x(),tlGlobal.y(),latTL,longTL);


    osg::Vec3 blGlobal=bl*screenToWorld;
    cout << "bl Global" << blGlobal<<endl;
    double latbl,longbl;
    map_projection->calc_geo_coords(blGlobal.x(),blGlobal.y(),latbl,longbl);

    osg::Vec3 trGlobal=tr*screenToWorld;
    cout << "tr Global" << trGlobal<<endl;
    double lattr,longtr;
    map_projection->calc_geo_coords(trGlobal.x(),trGlobal.y(),lattr,longtr);

    osg::Vec3 brGlobal=br*screenToWorld;
    //cout << "br Global" << brGlobal<<endl;
    double latbr,longbr;
    map_projection->calc_geo_coords(brGlobal.x(),brGlobal.y(),latbr,longbr);

    //osg::Vec3 test(1956.17,3937.951,47.6057);
    //double testLat,testLong;
    //map_projection->calc_geo_coords(test.x(),test.y(),testLat,testLong);
    // std::cout << "LAt LONG " <<testLat << " "<< testLong<<"\n";

    /* std::cout << "good " <<test*worldtoScreen<<"\n";
    std::cout << "LAt LONG " <<testLat << " "<< testLong<<"\n";
    cout << "brGlobal " << brGlobal<< " " << tlGlobal << endl;*/
    osg::Vec3 diff=(brGlobal-tlGlobal);
    //cout << "Diff " << diff<<endl;
    osg::Vec2 scalePix( fabs(diff.x()/width), fabs(diff.y()/height));
    cout << "Pix Scale "<<scalePix<<endl;
    UF::GeographicConversions::Redfearn gpsConversion("WGS84","UTM");
    string zone;
    double easting_ctr, northing_ctr;

    double gridConvergence, pointScale;
    if(geoOrigin.x() != 0 || geoOrigin.y() !=0){
        gpsConversion.GetGridCoordinates(latTL, longTL
                                         , zone, easting_ctr, northing_ctr, gridConvergence, pointScale);
        pos.UTMZone=zone;
        gtiffspace::tie_point tie;
        tie.pt[0]=tl.x();
        tie.pt[1]=tl.y();
        tie.pt[2]=0;
        tie.pt[3]=easting_ctr;
        tie.pt[4]=northing_ctr;
        tie.pt[5]=0;
        pos.tie_points.push_back(tie);

    }
    /*std::cout << "world " <<(test*worldtoScreen)<<"Grab Iron\n";
    osg::Matrix transScreen;
    Transpose(transScreen,screenToWorld);
      pos.transform[0] = transScreen(0,0);
    pos.transform[1] = transScreen(0,1);
    pos.transform[2] = 0;
    pos.transform[3] = transScreen(0,3);
    pos.transform[4] = transScreen(1,0);
    pos.transform[5] = transScreen(1,1);
    pos.transform[6] = 0;
    pos.transform[7] = transScreen(1,3);
    pos.transform[8] = 0;
    pos.transform[9] = 0;
    pos.transform[10] = 0;
    pos.transform[11] = 0;
    pos.transform[12] = 0;
    pos.transform[13] = 0;
    pos.transform[14] = 0;
    pos.transform[15] = 1;
    pos.UTMZone = "51S";*/


    /*printf("%f %f %f %f\n",pos.transform[0],pos.transform[1],pos.transform[2],pos.transform[3]);
    printf("%f %f %f %f\n",pos.transform[4],pos.transform[5],pos.transform[6],pos.transform[7]);
    std::cout<<"Screen to World" << screenToWorld << std::endl;
    std::cout<<"Trans Scren" << (transScreen*test) << std::endl;
    std::cout<<"View" << transScreen << std::endl;
    std::cout<<"CAm" << camera->getViewport()->computeWindowMatrix() << std::endl;*/

    pos.pixscale[0]=scalePix.x();
    pos.pixscale[1]=scalePix.y();
    pos.pixscale[2]=0;
    gtiffspace::append_gtiff_tags(filename,pos);
    // std::cout<<"Screen to World" << screenToWorlds << std::endl;

}
