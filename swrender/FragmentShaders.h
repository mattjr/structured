#ifndef FRAGMENTSHADERS_H
#define FRAGMENTSHADERS_H
#include "renderer/span.h"
#include "renderer/geometry_processor.h"
#include "renderer/fixed_func.h"
#include "renderer/util.h"
#include "democommon.h"
#include "renderer/util.h"
#include "render_utils.h"
#include <osg/Vec4>
// this is the fragment shader
extern Rect gRect;
extern REGION *regOutput;
extern REGION *regRange;
typedef std::map<std::pair<int,int>,int > dcm_t;

extern  dcm_t doublecountmap;
extern int doubleTouchCount;

struct FragmentShader : public swr::SpanDrawer32BitColorAndDepth<FragmentShader> {
    // varying_count = 3 tells the rasterizer that it only needs to interpolate
    // three varying values (the r, g and b in this context).
    static const unsigned varying_count = 2;

    // we don't need to interpolate z in this example
    static const bool interpolate_z = false;

    // per triangle callback. This could for instance be used to select the
    // mipmap level of detail. We don't need it but it still needs to be defined
    // for everything to work.
    static void begin_triangle(
        const swr::IRasterizer::Vertex& v1,
        const swr::IRasterizer::Vertex& v2,
        const swr::IRasterizer::Vertex& v3,
        int area2)
    {}

    // the fragment shader is called for each pixel and has read/write access to
    // the destination color and depth buffers.
    static void single_fragment(const swr::IRasterizer::FragmentData &fd, unsigned int &color, unsigned int &depth)
    {
        float x=fix2float<16>(fd.varyings[0]);// >> 16;
        float y=fix2float<16>(fd.varyings[1]);
        // sample the texture and write the color information
        unsigned int c =texture->sample_nearest(x,y);
        // always write the color;
        color = c;
    }

    // this is called by the span drawing function to get the location of the color buffer
    static void* color_pointer(int x, int y)
    {

        gRect.left=x;
        gRect.top=y;

        if(im_prepare(regOutput,&gRect)){
            fprintf(stderr,"Prepare fail\n");
            exit(-1);
        }

        return IM_REGION_ADDR( regOutput, x, y );
    }

    // this is called by the span drawing function to get the location of the depth buffer
    static void* depth_pointer(int x, int y)
    {
        // We don't use a depth buffer
        return 0;
    }

    static Texture *texture;



};
// this is the fragment shader
struct FragmentShaderBlendingDistPass : public swr::SpanDrawer32BitColorAndDepthSetDouble<FragmentShaderBlendingDistPass> {
    // varying_count = 3 tells the rasterizer that it only needs to interpolate
    // three varying values (the r, g and b in this context).
    static const unsigned varying_count = 3;
    static const float rmax=0.70710678;

    // we don't need to interpolate z in this example
    static const bool interpolate_z = false;
    static int idx;
    // per triangle callback. This could for instance be used to select the
    // mipmap level of detail. We don't need it but it still needs to be defined
    // for everything to work.
    static void begin_triangle(
        const swr::IRasterizer::Vertex& v1,
        const swr::IRasterizer::Vertex& v2,
        const swr::IRasterizer::Vertex& v3,
        int area2)
    {}

    // the fragment shader is called for each pixel and has read/write access to
    // the destination color and depth buffers.
    static void single_fragment(const swr::IRasterizer::FragmentData &fd, int ix,int iy,unsigned int &color, unsigned int &depth)
    {
        // float x=fix2float<16>(fd.varyings[0]);// >> 16;
        // float y=fix2float<16>(fd.varyings[1]);// >> 16;
        //    float x=fd.varyings[0];
        //  float y=fd.varyings[1];
        float x=fix2float<16>(fd.varyings[0]);// >> 16;
        float y=fix2float<16>(fd.varyings[1]);
        // cout << "rig: "<<x*1359 << " " <<y*511;
        unsigned char dist=255; //No value sentinal
        bool valid=false;
        if(x>=0 && y>=0){

            float distx=(x-0.5);//(texture->w_minus1/2.0))/(float)texture->w_minus1;
            float disty=(y-0.5);//(texture->h_minus1/2.0))/(float)texture->h_minus1;
            // printf("x: %f y%f\n",x/(double)texture->w_minus1,y/(double)texture->h_minus1);
            float r1 = sqrtf( distx*distx + disty*disty);
            dist=clamp((int)((r1/rmax)*255.0),0,255);
            valid=true;

        }
        unsigned char  oldd[4];

        unsigned int d=color;
        oldd[0]= d & 0xFF;
        oldd[1] = (d >> 8) & 0xFF;
        oldd[2] = (d >> 16) & 0xFF;
        oldd[3]= (d >> 24) & 0xFF;
        if(valid){
            if( oldd[idx]!=255){
                // printf("Sentinal touch two %d %d\n",idx,triIdx);
                if(!doublecountmap.count(std::make_pair<int,int>(ix,iy))){
                    std::set<int> a;
                    doublecountmap[std::make_pair<int,int>(ix,iy)]=triIdx;
                    doubleTouchCount++;
                }
            }else{
                oldd[idx]=dist;
            }
        }
        color= oldd[0] | oldd[1] << 8 | oldd[2] << 16 | oldd[3] << 24;

    }

    // this is called by the span drawing function to get the location of the color buffer
    static void* color_pointer(int x, int y)
    {

        gRect.left=x;
        gRect.top=y;

        if(im_prepare(regRange,&gRect)){
            fprintf(stderr,"Prepare fail range rect\n");
            assert(0);
            exit(-1);
        }

        return IM_REGION_ADDR( regRange, x, y );

    }

    // this is called by the span drawing function to get the location of the depth buffer
    static void* depth_pointer(int x, int y)
    {
        return 0;
    }
    static TextureMipMap *texture;
    static int triIdx;

};

// this is the fragment shader
struct FragmentShaderBlendingMain : public swr::SpanDrawer32BitColorAndDepthSetDouble<FragmentShaderBlendingMain> {
    // varying_count = 3 tells the rasterizer that it only needs to interpolate
    // three varying values (the r, g and b in this context).
    static const unsigned varying_count = 3;
    static const int mipmapL[];
    static const float rmax=0.70710678;
    static int idx;
    // we don't need to interpolate z in this example
    static const bool interpolate_z = false;

    // per triangle callback. This could for instance be used to select the
    // mipmap level of detail. We don't need it but it still needs to be defined
    // for everything to work.
    static void begin_triangle(
        const swr::IRasterizer::Vertex& v1,
        const swr::IRasterizer::Vertex& v2,
        const swr::IRasterizer::Vertex& v3,
        int area2)
    //   {printf("T1 %f %f\nT2 %f %f\nT3 %f %f\n",fix2float<16>(v1.x),fix2float<16>(v1.y),fix2float<16>(v2.x),fix2float<16>(v2.y),fix2float<16>(v3.x)
    //  ,fix2float<16>(v3.y));}
    {}

    // the fragment shader is called for each pixel and has read/write access to
    // the destination color and depth buffers.
    // static void single_fragment(const swr::IRasterizer::FragmentData &fd, unsigned int &color, unsigned int &depth)
    static void single_fragment(const swr::IRasterizer::FragmentData &fd, int ix,int iy,unsigned int &color, unsigned int &depth)

    {
        if(doublecountmap.count(std::make_pair<int,int>(ix,iy)) && doublecountmap[std::make_pair<int,int>(ix,iy)] != triIdx)
            return;
        float Cb[]={0.710000,0.650000,0.070000};
        //int x=fd.varyings[0] >> 16;
        //int y=fd.varyings[1] >> 16;
        float x=fix2float<16>(fd.varyings[0]);
        float y=fix2float<16>(fd.varyings[1]);

        if(x <0 || y <0|| x>(int)texture->w_minus1 || y>(int)texture->h_minus1){
            printf("Fail \n");
            return;
        }
        unsigned char dist_curr=reinterpret_cast<unsigned char *>(&depth)[idx];
        //Need to handle invalid dist
        float local_r=(dist_curr/255.0)*rmax;

        osg::Vec3d outComp[3];
        for(int j=0; j < 3; j++){
            float W=exp(-local_r*10.0*16.0*Cb[j]);
            // sample the texture and write the color information
            if(j == 0){
                outComp[j]=((texture->sample_nearest(x, y,mipmapL[0])-texture->sample_nearest(x, y,mipmapL[1]))*W);
            }else if (j==1){
                outComp[j]=((texture->sample_nearest(x, y,mipmapL[1])-texture->sample_nearest(x, y,mipmapL[2]))*W);
            }else if(j==2){
                outComp[j]=(texture->sample_nearest(x, y,mipmapL[2])*W);
            }
            // always write the color;
        }
        osg::Vec3 WSum(0.0,0.0,0.0);
        osg::Vec4 r4(0.0,0.0,0.0,0.0);
        for(int j=0; j < 3; j++){
            for(int i=0;i<4; i++){
                unsigned int d=depth;
                unsigned char oldd[4];

                oldd[0]= d & 0xFF;
                oldd[1] = (d >> 8) & 0xFF;
                oldd[2] = (d >> 16) & 0xFF;
                oldd[3]= (d >> 24) & 0xFF;
                unsigned char dist=oldd[i];
                if(dist ==255)
                    continue;
                //Need to handle invalid dist
                float local_r=(dist/255.0)*rmax;
                r4[i]=local_r;
                float W=exp(-local_r*10.0*16.0*Cb[j]);
                WSum[j]+=W;
            }
        }
        outComp[0]=outComp[0]/WSum[0];
        outComp[1]=outComp[1]/WSum[1];
        outComp[2]=outComp[2]/WSum[2];

        osg::Vec3 outP = outComp[0]+outComp[1]+outComp[2];

        int r,g,b;
        unsigned int c=color;
        int oldblue,oldgreen,oldred;
        if(USE_BGR){
            oldblue = c & 0xFF;
            oldred = (c >> 16) & 0xFF;
        }else{
            oldred = c & 0xFF;
            oldblue = (c >> 16) & 0xFF;
        }
        oldgreen = (c >> 8) & 0xFF;

        r=(int)(outP.x()*255.0);
        g=(int)(outP.y()*255.0);
        b=(int)(outP.z()*255.0);
        if(USE_BGR)
            c= clamp(b+oldblue,0,255) | clamp(g+oldgreen,0,255) << 8 | clamp(r+oldred,0,255) << 16 | 255 << 24;
        else
            c=  clamp(r+oldred,0,255) | clamp(g+oldgreen,0,255) << 8 | clamp(b+oldblue,0,255) << 16 | 255 << 24;

        color = c;


    }

    // this is called by the span drawing function to get the location of the color buffer
    static void* color_pointer(int x, int y)
    {
        gRect.left=x;
        gRect.top=y;

        if(im_prepare(regOutput,&gRect)){
            fprintf(stderr,"Prepare fail color\n");
            assert(0);
            exit(-1);
        }

        return IM_REGION_ADDR( regOutput, x, y );
    }

    // this is called by the span drawing function to get the location of the depth buffer
    static void* depth_pointer(int x, int y)
    {
        gRect.left=x;
        gRect.top=y;

        if(im_prepare(regRange,&gRect)){
            fprintf(stderr,"Prepare fail range blend pass\n");
            assert(0);
            exit(-1);
        }

        return IM_REGION_ADDR( regRange, x, y );
    }

    static TextureMipMap *texture;
    static int triIdx;

};

// this is the fragment shader
struct FragmentShaderVarMain : public swr::SpanDrawer32BitColorAndDepthSetDouble<FragmentShaderVarMain> {
    // varying_count = 3 tells the rasterizer that it only needs to interpolate
    // three varying values (the r, g and b in this context).
    static const unsigned varying_count = 12;
    static const int mipmapL[];
    static const float rmax=0.70710678;
    // we don't need to interpolate z in this example
    static const bool interpolate_z = false;
    static std::map<int,TextureMipMap*> texturepool;
    static bool writeOut;
    static FILE *f_arr[3];
    static IplImage *outputImage;

    // per triangle callback. This could for instance be used to select the
    // mipmap level of detail. We don't need it but it still needs to be defined
    // for everything to work.
    static void begin_triangle(
        const swr::IRasterizer::Vertex& v1,
        const swr::IRasterizer::Vertex& v2,
        const swr::IRasterizer::Vertex& v3,
        int area2)
    /*{printf("T1 %f %f\nT2 %f %f\nT3 %f %f\n",fix2float<16>(v1.x),fix2float<16>(v1.y),fix2float<16>(v2.x),fix2float<16>(v2.y),fix2float<16>(v3.x)
        ,fix2float<16>(v3.y));}*/
    {}

    // the fragment shader is called for each pixel and has read/write access to
    // the destination color and depth buffers.
    static void single_fragment(const swr::IRasterizer::FragmentData &fd, int ix,int iy,unsigned int &color, unsigned int &depth)
    {

        osg::Vec4f x;// >> 16;
        osg::Vec4f y;
        int id[4];
        for(int i=0;i <4; i++){
            x[i]=fix2float<16>(fd.varyings[i*2+0]);
            y[i]=fix2float<16>(fd.varyings[i*2+1]);// >> 16;
            x[i]=clamp(x[i],0.0,1.0);
            y[i]=clamp(y[i],0.0,1.0);
        }
        for(int i=0;i <4; i++){
            id[i]=(int)fix2float<16>(fd.varyings[9+i])*1.0;

        }



        std::vector<double> outComp[3];
        std::vector<osg::Vec3> colors;
        std::vector<double> grays;

        osg::Vec3 WSum(0.0,0.0,0.0);

        for(int i=0;i<4; i++){

            if(id[i]<=0 || !texturepool.count(id[i]))
                continue;
            TextureMipMap *texturec=texturepool[id[i]];
            if(!texturec){
                printf("Can't load %d",id[i]);
                continue;
            }

            //cout<<"id "<< id[i] << "\n";
            if(x[i]<0 || y[i]<0)
                continue;
            int mipmapL=6;
            osg::Vec3 val=texturec->sample_nearest(x[i],y[i],mipmapL);
            double grey=0.3*val[0]+0.59*val[1]+0.11*val[2];

            colors.push_back(val);
            grays.push_back(grey);
            //osg::Vec3 lab=rgb2lab(val);
            for(int j=0; j<3; j++)
                outComp[j].push_back(val[j]);



        }

        double std_dev_pixels[3];
        osg::Vec3 outP;

        for(int j=0; j <3; j++){
            std_dev_pixels[j]=standard_dev(outComp[j]);
            if(writeOut)
                fwrite(&(std_dev_pixels[j]),sizeof(double),1,f_arr[j]);
            //scale from 0.5 max std to full 0 to 1.0 range
       //       outP[j]=std_dev_pixels[j]/0.15;
        }
        // this is the fragment shader
        // printf("%f\n",std_dev_pixels[0]);
        double std_dev_gray=standard_dev(grays);
        //printf("%f\n",std_dev_gray);
        std_dev_gray=clamp((std_dev_gray/0.05f),0.0f,1.0f);
        outP=jetColorMap(std_dev_gray);

        unsigned char r,g,b;
        r=clamp((int)(outP.x()*255.0),0,255);
        g=clamp((int)(outP.y()*255.0),0,255);
        b=clamp((int)(outP.z()*255.0),0,255);

        color= (b) | (g) << 8 | (r) << 16 | 255 << 24;

    }

    // this is called by the span drawing function to get the location of the color buffer
    static void* color_pointer(int x, int y)
    {
        IplImage *screen = outputImage;///SDL_GetVideoSurface();
        //return static_cast<unsigned short*>(screen->pixels) + x + y * screen->w;
        return &(CV_IMAGE_ELEM(screen,unsigned char,y,x*screen->nChannels));
    }

    // this is called by the span drawing function to get the location of the depth buffer
    static void* depth_pointer(int x, int y)
    {
        //IplImage *screen = varImage;///SDL_GetVideoSurface();
        //return static_cast<unsigned short*>(screen->pixels) + x + y * screen->w;
        return NULL;//&(CV_IMAGE_ELEM(screen,unsigned char,y,x*screen->nChannels));
    }
    // static TextureMipMap *texture;
};
#endif // FRAGMENTSHADERS_H
