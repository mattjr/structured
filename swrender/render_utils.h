#ifndef RENDER_UTILS_H
#define RENDER_UTILS_H
#include <string>
#include <opencv/highgui.h>
#include <stdio.h>
#include <opencv/cv.h>
#include <fstream>
#include <osg/BoundingBox>
#include <osg/Matrix>
#include <map>
#include <set>
#include <vips/vips.h>
#include <numeric>

#include "mipmap.h"
// our vertex structure which will be used to store our triangle data
struct Vertex {
    float x, y,z;
    float tx, ty;
    int id;
};
struct VertexBlend {
    float x, y,z;
    float tx[4], ty[4];
    float id[4];
};
typedef struct _imgData{
    osg::BoundingBox bbox;
    osg::Matrix m;
    std::string filename;
    int id;
}imgData;
int log2i(uint32_t value) ;
int log2i(uint64_t value) ;
void fast_mipmap_downsize( const IplImage *source, IplImage *dest );
extern bool USE_BGR;


class TriangleIndexVisitor
{
public:
    TriangleIndexVisitor(){newVertCounter=0;}
    std::set<int> indices_counted;
    std::map<int,int> indices_double_counted;
    std::vector<int> new_list;
    int newVertCounter;

    void operator()(const int v1, const int v2, const int v3)
    {
        new_list.push_back(v1);
        new_list.push_back(v2);
        new_list.push_back(v3);

        /*   // toss the computed indices into the indices array
        if(indices_counted.count(v1) == 0)
            indices_counted.insert( v1 );
        else{
            indices_double_counted[v1]=newVertCounter++;
        }
        if(indices_counted.count(v2) == 0)
            indices_counted.insert( v2 );
        else
            indices_double_counted[v2]=newVertCounter++;
        if(indices_counted.count(v3) == 0)
            indices_counted.insert( v3 );
        else
            indices_double_counted[v3]=newVertCounter++;*/

    }
};
inline int clamp(int x, int a, int b){    return x < a ? a : (x > b ? b : x);}
inline float clamp(float x, float a, float b){    return x < a ? a : (x > b ? b : x);}
#if 0

// our texture class
struct Texture {
    IplImage *surface;

    unsigned w_minus1, h_minus1;

    // create a texture from an SDL_Surface
    Texture(std::string filename)
    {
        surface = cvLoadImage(filename.c_str(),1);

        if(!surface){
            fprintf(stderr,"Can't Load %s\n",filename.c_str());
        }else{
            w_minus1 = surface->width - 1;
            h_minus1 = surface->height - 1;
        }
    }

    ~Texture()
    {
        if(surface)
            cvReleaseImage(&surface);

    }



    // samples the texture using the given texture coordinate.
    // the integer texture coordinate is given in the upper 16 bits of the x and y variables.
    // it is NOT in the range [0.0, 1.0] but rather in the range of [0, width - 1] or [0, height - 1].
    // texture coordinates outside this range are wrapped.
    unsigned int sample_nearest(int x, int y) const
    {
        x >>= 16;
        y >>= 16;
        // x &= w_minus1;
        // y &= h_minus1;
        x=clamp(x,0,w_minus1);
        y=clamp(y,0,h_minus1);
        unsigned char r,g,b;
        unsigned int sample=0;
        r=CV_IMAGE_ELEM(surface,unsigned char,y,(surface->nChannels*x)+2);
        g=CV_IMAGE_ELEM(surface,unsigned char,y,(surface->nChannels*x)+1);
        b=CV_IMAGE_ELEM(surface,unsigned char,y,(surface->nChannels*x)+0);
        if(USE_BGR)
            sample= b | g << 8 | r << 16 | 255 << 24;
        else
            sample= r | g << 8 | b << 16 | 255 << 24;


        return sample;
    }
};

class  TextureMipMap: public Texture {
public:

    TextureMipMap(std::string filename,int id=-1) : Texture(filename),id(id){
        if(!filename.size() || !surface){
            fprintf(stderr,"Can't load %s\n",filename.c_str());
            return;
        }
        numLevels = 1 + log2i((uint32_t) std::max(surface->width, surface->height));
        texture_pyr.resize(numLevels);
        texture_pyr[0]=surface;
        for(int i=1; i <numLevels; i++){
            texture_pyr[i]=cvCreateImage(cvSize(texture_pyr[i-1]->width/2,texture_pyr[i-1]->height/2),IPL_DEPTH_8U,3);
            fast_mipmap_downsize(texture_pyr[i-1],texture_pyr[i]);
        }
    }
    unsigned int sample_nearest(int x, int y) const{
        int res;

        osg::Vec3 c=sample_nearest(x,y,0);
        res= (unsigned char)(c[0]*255) | (unsigned char)(c[1]*255) << 8 | (unsigned char)(c[2]*255) << 16 | 255 << 24;
        return res;
    }
    ~TextureMipMap()
    {
        for(int i=1; i< (int) texture_pyr.size(); i++){
            cvReleaseImage(&texture_pyr[i]);
        }
    }

    osg::Vec3 sample_nearest(float x, float y,int l) const
    {
        //  x >>= 16;
        //y >>= 16;
        // x &= w_minus1;
        // y &= h_minus1;
        x=clamp(x*w_minus1,0.0,(float)w_minus1);
        y=clamp(y*h_minus1,0.0,(float)h_minus1);
        unsigned char r,g,b;
        osg::Vec3 sample(0,0,0);
        if(l > numLevels)
            return sample;
        for(int i=0; i<l; i++){
            x/=2.0;
            y/=2.0;
        }
        IplImage *img=texture_pyr[l];
        int ix=(int)round(x);
        int iy=(int)round(y);

        r=CV_IMAGE_ELEM(img,unsigned char,iy,(img->nChannels*ix)+2);
        g=CV_IMAGE_ELEM(img,unsigned char,iy,(img->nChannels*ix)+1);
        b=CV_IMAGE_ELEM(img,unsigned char,iy,(img->nChannels*ix)+0);
        //sample= b | g << 8 | r << 16 | 255 << 24;
        sample=osg::Vec3(r/255.0,g/255.0,b/255.0);
        return sample;
    }
    std::vector<IplImage *>texture_pyr;
    int numLevels;
    int id;
};
#else
// our texture class
struct Texture {
    IplImage *surface;
    MIPMap *mipmapTex;

    unsigned w_minus1, h_minus1;

    // create a texture from an SDL_Surface
    Texture(std::string filename,const MIPMap::EFilterType &filt=MIPMap::ENone)
    {
        surface = cvLoadImage(filename.c_str(),1);

        if(!surface){
            fprintf(stderr,"Can't Load %s\n",filename.c_str());
        }else{
            w_minus1 = surface->width - 1;
            h_minus1 = surface->height - 1;
            mipmapTex = MIPMap::fromBitmap(surface,filt,MIPMap::EClamp);

        }
    }

    ~Texture()
    {
       // if(surface)
         //   cvReleaseImage(&surface);
        delete mipmapTex;
//Frees surface

    }



    // samples the texture using the given texture coordinate.
    // the integer texture coordinate is given in the upper 16 bits of the x and y variables.
    // it is NOT in the range [0.0, 1.0] but rather in the range of [0, width - 1] or [0, height - 1].
    // texture coordinates outside this range are wrapped.
    unsigned int sample_nearest(float x, float y) const
    {
        osg::Vec3 tmp;
        Spectrum c =mipmapTex->triangle(0,x,y);
        c.toLinearRGB(tmp[0],tmp[1],tmp[2]);
        unsigned char r,g,b;
        r = (unsigned char)(tmp[0]*255);
        g=(unsigned char)(tmp[1]*255);
        b=(unsigned char)(tmp[2]*255);
        unsigned int sample=0;

        if(USE_BGR)
            sample= b | g << 8 | r << 16 | 255 << 24;
        else
            sample= r | g << 8 | b << 16 | 255 << 24;


        return sample;
    }
    osg::Vec3 sample_nearest_vec(float x, float y) const{
        osg::Vec3 tmp;
        Spectrum c =mipmapTex->triangle(0,x,y);
        c.toLinearRGB(tmp[0],tmp[1],tmp[2]);

         return tmp;
     }
};

class  TextureMipMap: public Texture {
public:

    TextureMipMap(std::string filename,int id=-1) : Texture(filename,MIPMap::EEWA),id(id){
        if(!filename.size() || !surface){
            fprintf(stderr,"Can't load %s\n",filename.c_str());
            return;
        }
        numLevels = mipmapTex->getLevels();

    }
    unsigned int sample_nearest(float x, float y) const{
        int res;

        osg::Vec3 c=sample_nearest(x,y,0);
        res= (unsigned char)(c[0]*255) | (unsigned char)(c[1]*255) << 8 | (unsigned char)(c[2]*255) << 16 | 255 << 24;
        return res;
    }

   osg::Vec3 sample_nearest_vec(float x, float y) const{
        int res;

        osg::Vec3 c=sample_nearest(x,y,0);

        return c;
    }
    ~TextureMipMap()
    {
    }

    osg::Vec3 sample_nearest(float x, float y,int l) const
    {
        osg::Vec3 sample;
        Spectrum c =mipmapTex->triangle(l,x,y);
        //sample= b | g << 8 | r << 16 | 255 << 24;
        c.toLinearRGB(sample[0],sample[1],sample[2]);
        return sample;
    }
    int numLevels;
    int id;
};


#endif
std::string format_elapsed(double d);
osg::Vec3 rgb2lab(osg::Vec3 c) ;
osg::Vec3 jetColorMap(const float& val) ;
inline bool isZero(float f) { return fabs(f) < 1E-6; }
inline bool isZero(double f) { return fabs(f) < 1E-10; }
inline bool eq(double a, double b) { return isZero(fabsf(a - b));}
inline bool eq(float a, float b) { return isZero(fabsf(a - b));}

double standard_dev( std::vector<double> &v );

#endif // RENDER_UTILS_H
