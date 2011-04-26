#ifndef GLIMAGING_H
#define GLIMAGING_H
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
#include <stdio.h>
#include <sys/ioctl.h>

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
class OptFormatTexturesVisitor : public osg::NodeVisitor
{
public:

    OptFormatTexturesVisitor():
            osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),_imageSizeMB(0.0)
           {}

    virtual void apply(osg::Node& node)
    {
        if (node.getStateSet()) apply(*node.getStateSet());
        traverse(node);
    }

    virtual void apply(osg::Geode& node)
    {
        if (node.getStateSet()) apply(*node.getStateSet());

        for(unsigned int i=0;i<node.getNumDrawables();++i)
        {
            osg::Drawable* drawable = node.getDrawable(i);
            if (drawable && drawable->getStateSet()) apply(*drawable->getStateSet());
        }

        traverse(node);
    }

    virtual void apply(osg::StateSet& stateset)
    {
        // search for the existence of any texture object attributes
        for(unsigned int i=0;i<stateset.getTextureAttributeList().size();++i)
        {
            osg::Texture* texture = dynamic_cast<osg::Texture*>(stateset.getTextureAttribute(i,osg::StateAttribute::TEXTURE));
            if (texture)
            {
                _textureSet.insert(texture);
            }
        }
    }


    void opt(void);

    double getImageSizeMB(){return _imageSizeMB;}
    typedef std::set< osg::ref_ptr<osg::Texture> > TextureSet;
    TextureSet                          _textureSet;
    double _imageSizeMB;

};
int gpuUsage(int gpu,int &mem);
void applyGeoTags(osg::Vec2 geoOrigin,osg::Matrix viewMatrix,osg::Matrix projMatrix,int width,int height);
void addCallbackToViewer(osgViewer::ViewerBase& viewer, WindowCaptureCallback* callback);
void formatBar(string name,osg::Timer_t startTick,unsigned int count,unsigned int totalCount);
 void ConvertRGBA_BGRA_SSSE3(u32 *dst, const int dstPitch, u32 *pIn, const int width, const int height, const int pitch);
 void RGB2RGBA(unsigned int w, unsigned int h,
                 unsigned char *src, unsigned char *dst);
int imageNodeGL(osg::Node *node,unsigned int _tileRows,unsigned int _tileColumns,unsigned int width,unsigned int height,int row,int col,
                const osg::Matrixd &view,const osg::Matrixd &proj,bool untex,std::string ext);
typedef struct _picture_cell{
    int row;
    int col;
    osg::BoundingBox bbox;
    std::string name;
}picture_cell;
#endif // GLIMAGING_H
