/*
Copyright (c) 2007, Markus Trenkwalder

All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

* Neither the name of the library's copyright owner nor the names of its 
contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "SDL.h"
#include "SDL_image.h"
#include <time.h>
#include <string>
#include <iostream>
#include <osg/TriangleIndexFunctor>
#include <osg/Timer>
// requires software renderer source
#include "renderer/geometry_processor.h"
#include "renderer/rasterizer_subdivaffine.h"
#include "renderer/span.h"
#include "democommon.h"
#include "fixedpoint/fixed_func.h"
#include <osg/io_utils>
#include <osg/ComputeBoundsVisitor>
#include "vertexData.h"
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osgUtil/Optimizer>
#include <opencv/cv.h>
#include <opencv/highgui.h>
// the software renderer stuff is located in the namespace "swr" so include 
// that here
using namespace swr;
using namespace std;
// our vertex structure which will be used to store our triangle data
struct Vertex {
    float x, y,z;
    float tx, ty;
    int id;
};
typedef struct _imgData{
    osg::BoundingBox bbox;
    osg::Matrix m;
    string filename;
    int id;
}imgData;

struct InputVertex {
    vec3x vertex;
};
class TriangleIndexVisitor
{
public:
    TriangleIndexVisitor(){newVertCounter=0;}
    set<int> indices_counted;
    map<int,int> indices_double_counted;
    vector<int> new_list;
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
// our texture class
struct Texture {
    SDL_Surface *surface;

    unsigned w_log2, h_log2;
    unsigned w_minus1, h_minus1;

    // create a texture from an SDL_Surface
    Texture(SDL_Surface *s)
    {
        surface = s;

        w_log2 = log2_of_pot(s->w);
        h_log2 = log2_of_pot(s->h);

        w_minus1 = s->w - 1;
        h_minus1 = s->h - 1;
    }

    ~Texture()
    {
        SDL_FreeSurface(surface);
    }

    // returns log2 of a number which is a power if two
    unsigned log2_of_pot(unsigned v) const
    {
        unsigned r = 0;
        while (!(v & 1)) {
            v >>= 1;
            ++r;
        }struct InputVertex {
        vec3x vertex;
    };
        return r;
    }

    // samples the texture using the given texture coordinate.
    // the integer texture coordinate is given in the upper 16 bits of the x and y variables.
    // it is NOT in the range [0.0, 1.0] but rather in the range of [0, width - 1] or [0, height - 1].
    // texture coordinates outside this range are wrapped.
    unsigned short sample_nearest(int x, int y) const
    {
        x >>= 16;
        y >>= 16;
        x &= w_minus1;
        y &= h_minus1;
        return *(static_cast<unsigned short*>(surface->pixels) + ((y << w_log2) + x));
    }
};
std::string format_elapsed(double d);


static IplImage *outputImage;
struct TexturePool {
    map<int,IplImage*> surface;

    unsigned w_log2, h_log2;
    unsigned w_minus1, h_minus1;

    // create a texture from an IplImage
    TexturePool(map<int,IplImage*> &surface): surface(surface)
    {
        //  surface = s;

        w_log2 = log2_of_pot(surface.begin()->second->width);
        h_log2 = log2_of_pot(surface.begin()->second->height);

        w_minus1 = surface.begin()->second->width - 1;
        h_minus1 = surface.begin()->second->height - 1;
    }

    ~TexturePool()
    {
        //SDL_FreeSurface(surface);
    }

    // returns log2 of a number which is a power if two
    unsigned log2_of_pot(unsigned v) const
    {
        unsigned r = 0;
        while (!(v & 1)) {
            v >>= 1;
            ++r;
        }struct InputVertex {
        vec3x vertex;
    };
        return r;
    }

    // samples the texture using the given texture coordinate.
    // the integer texture coordinate is given in the upper 16 bits of the x and y variables.
    // it is NOT in the range [0.0, 1.0] but rather in the range of [0, width - 1] or [0, height - 1].
    // texture coordinates outside this range are wrapped.
    unsigned int sample_nearest(int x, int y,int id)
    {
        x >>= 16;
        y >>= 16;
        x &= w_minus1;
        y &= h_minus1;

        if(surface.count(id) == 0 ){
            return 0;
        }else{
            if(surface[id] == NULL)
                return 0;
        }
        unsigned char r,g,b;
        unsigned int sample=0;
        r=CV_IMAGE_ELEM(surface[id],unsigned char,y,x*surface[id]->nChannels+2);
        g=CV_IMAGE_ELEM(surface[id],unsigned char,y,x*surface[id]->nChannels+1);
        b=CV_IMAGE_ELEM(surface[id],unsigned char,y,x*surface[id]->nChannels+0);
        sample= b | g << 8 | r << 16 | 255 << 24;

        return sample;//*(static_cast<unsigned short*>(surface[id]->pixels) + ((y << w_log2) + x));
    }
};

// loads an image file using SDL_image and converts it to a r5g5a1b5 format.
// this format can be directly copied to the screen and one can also use the 
// embedded alpha bit for an alpha test.
SDL_Surface* load_surface_r5g5a1b5(const char *filename)
{
    const Uint32 rmask = 0xF800;
    const Uint32 gmask = 0x7C0;
    const Uint32 bmask = 0x1F;
    const Uint32 amask = 0x20;

    SDL_Surface *result = 0;
    SDL_Surface *img = 0;
    SDL_Surface *dummy = 0;

    img = IMG_Load(filename); if (!img) goto end;
    dummy = SDL_CreateRGBSurface(0, 0, 0, 16, rmask, gmask, bmask, amask); if (!dummy) goto end;
    result = SDL_ConvertSurface(img, dummy->format, 0);

    end:
    if (img) SDL_FreeSurface(img);
    if (dummy) SDL_FreeSurface(dummy);

    return result;
}

// this is the vertex shader which is executed for each individual vertex that
// needs to ne processed.
struct VertexShader {

    // this specifies that this shader is only going to use 1 vertex attribute
    // array. There you be used up to Renderer::MAX_ATTRIBUTES arrays.
    static const unsigned attribute_count = 1;

    // this specifies the number of varyings the shader will output. This is
    // for instance used when clipping.
    static const unsigned varying_count = 3;
    static mat4x modelview_matrix;
    static mat4x modelviewprojection_matrix;
    // this static function is called for each vertex to be processed.
    // "in" is an array of void* pointers with the location of the individial
    // vertex attributes. The "out" structure has to be written to.
    static void shade(const GeometryProcessor::VertexInput in, GeometryProcessor::VertexOutput &out)
    {
        // cast the first attribute array to the input vertex type
        const Vertex &v = *static_cast<const Vertex*>(in[0]);

        // x, y, z and w are the components that must be written by the vertex
        // shader. They all have to be specified in 16.16 fixed point format.
        /*  out.x = static_cast<int>((v.x * (1 << 16)));
        out.y = static_cast<int>((v.y * (1 << 16)));
        out.z = static_cast<int>((v.z * (1 << 16)));
        out.w = 1 << 16;
*/
        vec4x tvertex = modelviewprojection_matrix * vec4x(v.x,v.y,v.z, X(1));
        // cout << fixedpoint::fix2float<16>(tvertex.x.intValue) << " " <<         fixedpoint::fix2float<16>(tvertex.y.intValue) << " "<<        fixedpoint::fix2float<16>(tvertex.z.intValue)<<endl;

        //  cout << v.id << " "<<v.tx << " " << v.ty<<endl;
        // const vec3x &v = i.vertex;
        const mat4x &m = modelview_matrix;
        /*  fixed16_t d = -(v.x * m.elem[2][0] + v.y * m.elem[2][1] +
                        v.z * m.elem[2][2] + m.elem[2][3]);

        const fixed16_t start = 2;
        const fixed16_t end = 6;

        if (d < start) d = 1;
        else if (d > end) d = 0;
        else {
            d = X(1) - (d-start)/(end-start);
        }
*/
        out.x = tvertex.x.intValue;
        out.y = tvertex.y.intValue;
        out.z = X(1.0).intValue;//tvertex.z.intValue;
        out.w = 1 << 16;//tvertex.w.intValue;
        // bring the texture coordinates into the appropriate range for the rasterizer.
        // this mean it has to be converted to fixed point and premultiplied with the width, height of the
        // texture minus 1. Doing this in the vertex shader saves us from doing this in the fragment shader
        // which makes things faster.
        out.varyings[0] = static_cast<int>(v.tx * /*texture->*/w_minus1 * (1 << 16));
        out.varyings[1] = static_cast<int>(v.ty * /*texture->*/h_minus1 * (1 << 16));
         out.varyings[2] = static_cast<int>(v.id);

    }

    //  static Texture *texture;
    static int w_minus1;
    static int h_minus1;
};
mat4x VertexShader::modelview_matrix;
mat4x VertexShader::modelviewprojection_matrix;
//Texture *VertexShader::texture = 0;
int VertexShader::w_minus1=511;
int VertexShader::h_minus1=511;

// this is the fragment shader
struct FragmentShader : public SpanDrawer32BitColorAndDepth<FragmentShader> {
    // varying_count = 3 tells the rasterizer that it only needs to interpolate
    // three varying values (the r, g and b in this context).
    static const unsigned varying_count = 3;

    // we don't need to interpolate z in this example
    static const bool interpolate_z = false;
    static map<int,imgData> imageList;

    // per triangle callback. This could for instance be used to select the
    // mipmap level of detail. We don't need it but it still needs to be defined
    // for everything to work.
    static void begin_triangle(
            const IRasterizer::Vertex& v1,
            const IRasterizer::Vertex& v2,
            const IRasterizer::Vertex& v3,
            int area2)
    {}

    // the fragment shader is called for each pixel and has read/write access to
    // the destination color and depth buffers.
    static void single_fragment(const IRasterizer::FragmentData &fd, unsigned int &color, unsigned int &depth)
    {
        // sample the texture and write the color information
        unsigned int c = texturepool->sample_nearest(fd.varyings[0], fd.varyings[1],fd.varyings[2]);

#	ifdef ALPHA_TEST
        // do an alpha test and only write color if the test passed
        if (c & (1 << 5)) {
            // write the color
            color = c;
        }
#	else
        // always write the color;
        color = c;
#	endif
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
        // We don't use a depth buffer
        return 0;
    }

    static TexturePool *texturepool;
};
map<int,imgData> FragmentShader::imageList;

void readFile(string fname,map<int,imgData> &imageList){
    std::ifstream m_fin(fname.c_str());

    while(!m_fin.eof()){
        imgData cam;
        double low[3], high[3];
        m_fin >> cam.id >> cam.filename >> low[0] >> low[1] >> low[2] >> high[0] >> high[1] >> high[2]
                >> cam.m(0,0) >>cam.m(0,1)>>cam.m(0,2) >>cam.m(0,3)
                >> cam.m(1,0) >>cam.m(1,1)>>cam.m(1,2) >>cam.m(1,3)
                >> cam.m(2,0) >>cam.m(2,1)>>cam.m(2,2) >>cam.m(2,3)
                >> cam.m(3,0) >>cam.m(3,1)>>cam.m(3,2) >>cam.m(3,3);
        cam.bbox.expandBy(low[0],low[1],low[2]);
        cam.bbox.expandBy(high[0],high[1],high[2]);
        imageList[cam.id]=cam;

    }
}

TexturePool *FragmentShader::texturepool = 0;

int main(int ac, char *av[]) {
    SDL_Event e;
    TexturePool *texturepool;
    osg::Timer_t start;
    osg::ref_ptr<osg::Node> model;//= osgDB::readNodeFile(av[1]);
    ply::VertexData vertexData;
    osg::ArgumentParser arguments(&ac,av);
    int sizeX,sizeY; sizeX=sizeY=1024;

    arguments.read("--size",sizeX,sizeY);
    outputImage=cvCreateImage(cvSize(sizeX,sizeY),IPL_DEPTH_8U,4);

    model= vertexData.readPlyFile(av[1]);
    readFile(av[2],FragmentShader::imageList);
    if(model.valid()){
        osg::Geode *geode= dynamic_cast<osg::Geode*>(model.get());
        if(!geode)
            geode=model->asGroup()->getChild(0)->asGeode();
        else{
            printf("fail\n");
        }
        if(geode && geode->getNumDrawables()){
            printf("valid\n");
        }else{
            printf("Fail2\n");
            exit(-1);

        }
        float rx, ry, rz;
        osg::Matrix inverseM=osg::Matrix::identity();

        if(arguments.read("--invrot",rx,ry,rz)){
            inverseM =osg::Matrix::rotate(
                    osg::DegreesToRadians( rx ), osg::Vec3( 1, 0, 0 ),
                    osg::DegreesToRadians( ry ), osg::Vec3( 0, 1, 0 ),
                    osg::DegreesToRadians( rz ), osg::Vec3( 0, 0, 1 ) );
        }
        osg::Matrix rotM=osg::Matrix::inverse(inverseM);

        osg::ref_ptr<osg::MatrixTransform>xform = new osg::MatrixTransform;
        xform->setDataVariance( osg::Object::STATIC );
        xform->setMatrix(rotM);
        osgUtil::Optimizer::FlattenStaticTransformsVisitor fstv(NULL);
        xform->addChild(model);
        xform->accept(fstv);
        fstv.removeTransforms(xform);



        osg::BoundingBox totalbb;
        for(int i=0; i <vertexData._vertices->size(); i++){
            totalbb.expandBy(vertexData._vertices->at(i));
        }

        osg::Vec3d eye(totalbb.center()+osg::Vec3(0,0,3.5*totalbb.radius()));
        double xrange=totalbb.xMax()-totalbb.xMin();
        double yrange=totalbb.yMax()-totalbb.yMin();
        double largerSide=std::max(xrange,yrange);
        osg::Matrixd matrix;
        matrix.makeTranslate( eye );

        osg::Matrixd view=osg::Matrix::inverse(matrix);
        cout << view << endl;

        mat4x viewA=fast_inverse<fixed16_t>(translation_matrix<fixed16_t>(eye.x(),eye.y(),eye.z()));
        osg::Matrixd proj= osg::Matrixd::ortho2D(-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0));

        mat4x projA=ortho_matrix<fixed16_t>(-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0),totalbb.zMin(),totalbb.zMax());
        osg::Matrixd viewprojmat=view*proj;
        cout <<viewprojmat <<endl;


        VertexShader::modelviewprojection_matrix=projA*viewA;
        for(int k=0; k<4; k++){
            cout << endl;
            for(int j=0;j<4;j++)
                cout << fixedpoint::fix2float<16>(VertexShader::modelviewprojection_matrix.elem[k][j].intValue) << " ";
        }
        cout <<endl;
        //exit(-1);
        osg::Drawable *drawable = geode->getDrawable(0);
        osg::TriangleIndexFunctor<TriangleIndexVisitor> tif;
        drawable->accept(tif);


        //  printf("Size %d",(int)tif.indices_double_counted.size());
        osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
        //geom->setUseDisplayList(false);
        osg::Vec3Array *verts=static_cast<const osg::Vec3Array*>(geom->getVertexArray());
        int origSize=tif.new_list.size();
        //  verts->resize(origSize+tif.indices_double_counted.size());
        start = osg::Timer::instance()->tick();

	// initialize SDL without error handling an all
      //  SDL_Init(SDL_INIT_VIDEO);
        //SDL_Surface *screen = SDL_SetVideoMode(1280, 1024, 16, 0);

	// the four vertices of the textured quad together with the texture coordinates
	Vertex vertices[] = {
            {-0.5f,  0.5f, 0.0f, 0.0f},
            {-0.5f, -0.5f, 0.0f, 1.0f},
            { 0.5f, -0.5f, 1.0f, 1.0f},
            { 1.0f,  1.0f, 1.0f, 0.0f}
	};
        Vertex tmp_vertices[3];
	// load the texture file
        //texture = new Texture(load_surface_r5g5a1b5("texture.png"));
        map<int,IplImage*> surface;
        for(map<int,imgData>::iterator itr= FragmentShader::imageList.begin(); itr!= FragmentShader::imageList.end(); itr++){
            string tmp=string(string(av[3])+string("/")+itr->second.filename);
            surface[itr->second.id]=cvLoadImage(tmp.c_str(),-1);//load_surface_r5g5a1b5(tmp.c_str());
            if(surface[itr->second.id] == NULL){
                printf("Can't load %s\n",tmp.c_str());
            }
        }
        texturepool = new TexturePool(surface);
	// make the shaders know which texture to use
        //	VertexShader::texture = texture;
        FragmentShader::texturepool = texturepool;

	// the indices we need for rendering
        unsigned indices[] = {0, 1, 2};

	// create a rasterizer class that will be used to rasterize primitives
	RasterizerSubdivAffine r;
	// create a geometry processor class used to feed vertex data.
	GeometryProcessor g(&r);
	// it is necessary to set the viewport
        g.viewport(0, 0, outputImage->width, outputImage->height);
	// set the cull mode (CW is already the default mode)
	g.cull_mode(GeometryProcessor::CULL_CW);

	// it is also necessary to set the clipping rectangle
        r.clip_rect(0, 0, outputImage->width, outputImage->height);

	// set the vertex and fragment shaders
	g.vertex_shader<VertexShader>();
	r.fragment_shader<FragmentShader>();

	// specify where out data lies in memory
        g.vertex_attrib_pointer(0, sizeof(Vertex), tmp_vertices);


        int last_count=0;
	// draw the triangle
        //  VertexShader::modelviewprojection_matrix=identity4<fixed16_t>();

        for(long int i=0; i<origSize-2; i+=3){

            osg::Vec3 v1=verts->at(tif.new_list[i]);
            osg::Vec3 v2=verts->at(tif.new_list[i+1]);
            osg::Vec3 v3=verts->at(tif.new_list[i+2]);
             osg::Vec3 tc1,tc2,tc3;
            if(vertexData._texCoord.size()){

                 tc1=vertexData._texCoord[0]->at(tif.new_list[i]);
                 tc2=vertexData._texCoord[0]->at(tif.new_list[i+1]);;

                 tc3=vertexData._texCoord[0]->at(tif.new_list[i+2]);;

                /*tc1.z()=vertexData._texIds->at(tif.new_list[i])[0];
                tc2.z()=vertexData._texIds->at(tif.new_list[i]+1)[0];
                tc3.z()=vertexData._texIds->at(tif.new_list[i]+2)[0];*/
            }
            //  cout << "A " <<v1*viewprojmat << endl;
            /*osg::Vec3 minV;
            minV.x()=min(min(v1.x(),v2.x()),v3.x());
            minV.y()=min(min(v1.y(),v2.y()),v3.y());
            minV.z()=min(min(v1.z(),v2.z()),v3.z());


            v1-=minV;
            v2-=minV;

            v3-=minV;
            double minL=min(v1.length(),min(v2.length(),v3.length()));
            v1/=(minL);
            v2/=(minL);
            v3/=(minL);
*/

            //cout << v1 << " "<<v2<< " "<< v3 <<endl;
            tmp_vertices[0].x=v1.x();
            tmp_vertices[0].y=v1.y();
            tmp_vertices[0].tx=tc1.x();
            tmp_vertices[0].ty=1.0-tc1.y();
            tmp_vertices[0].id=tc1.z();

            tmp_vertices[1].x=v2.x();
            tmp_vertices[1].y=v2.y();
            tmp_vertices[1].tx=tc2.x();
            tmp_vertices[1].ty=1.0-tc2.y();
            tmp_vertices[1].id=tc2.z();

            tmp_vertices[2].x=v3.x();
            tmp_vertices[2].y=v3.y();
            tmp_vertices[2].tx=tc3.x();
            tmp_vertices[2].ty=1.0-tc3.y();
            tmp_vertices[2].id=tc3.z();
            g.draw_triangles(3, indices);
            if(i % 300 == 0){


                printf("\r%04d/%04d",(int)i,origSize);
                last_count=i;
                fflush(stdout);

            }

            //   SDL_FillRect( SDL_GetVideoSurface(), NULL, 0 );



        }
        double elapsed=osg::Timer::instance()->delta_s(start,osg::Timer::instance()->tick());
        std::cout << "\n"<<format_elapsed(elapsed) << std::endl;


      //  SDL_Flip(SDL_GetVideoSurface());
        cvSaveImage("out.png",outputImage);

	// show everything on screen



    }


    // free texture memory
    delete texturepool;

    // quit SDL
    return 0;
}
