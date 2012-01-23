#ifndef VERTEXSHADERS_H
#define VERTEXSHADERS_H
#include "renderer/span.h"
#include "renderer/geometry_processor.h"
#include "renderer/fixed_func.h"
#include "renderer/util.h"
#include "democommon.h"
#include "renderer/util.h"
#include "render_utils.h"
// this is the vertex shader which is executed for each individual vertex that
// needs to ne processed.
struct VertexShader {

    // this specifies that this shader is only going to use 1 vertex attribute
    // array. There you be used up to Renderer::MAX_ATTRIBUTES arrays.
    static const unsigned attribute_count = 1;

    // this specifies the number of varyings the shader will output. This is
    // for instance used when clipping.
    static const unsigned varying_count = 2;
    static mat4x modelview_matrix;
    static mat4x modelviewprojection_matrix;
    // this static function is called for each vertex to be processed.
    // "in" is an array of void* pointers with the location of the individial
    // vertex attributes. The "out" structure has to be written to.
    static void shade(const swr::GeometryProcessor::VertexInput in, swr::GeometryProcessor::VertexOutput &out)
    {
        // cast the first attribute array to the input vertex type
        const Vertex &v = *static_cast<const Vertex*>(in[0]);

        // x, y, z and w are the components that must be written by the vertex
        // shader. They all have to be specified in 16.16 fixed point format.

        vec4x tvertex = modelviewprojection_matrix * vec4x(v.x,v.y,v.z, fixed16_t(1));

        //        const mat4x &m = modelview_matrix;
      //  std::cout << fixedpoint::fix2float<16>(tvertex.x.intValue) << " " <<         fixedpoint::fix2float<16>(tvertex.y.intValue) << " "<<        fixedpoint::fix2float<16>(tvertex.z.intValue)<<endl;

        out.x = tvertex.x.intValue;
        out.y = tvertex.y.intValue;
        out.z = fixed16_t(1.0).intValue;//tvertex.z.intValue;
        out.w = 1 << 16;//tvertex.w.intValue;
        // bring the texture coordinates into the appropriate range for the rasterizer.
        // this mean it has to be converted to fixed point and premultiplied with the width, height of the
        // texture minus 1. Doing this in the vertex shader saves us from doing this in the fragment shader
        // which makes things faster. NOT ANYMORE


        out.varyings[0] = float2fix<16>(v.tx);//static_cast<int>(v.tx * (1 << 16));
        out.varyings[1] = float2fix<16>(v.ty);//static_cast<int>(v.ty * (1 << 16));

    }
    static Texture *texture;

};

// this is the vertex shader which is executed for each individual vertex that
// needs to ne processed.
struct VertexShaderBlending {

    // this specifies that this shader is only going to use 1 vertex attribute
    // array. There you be used up to Renderer::MAX_ATTRIBUTES arrays.
    static const unsigned attribute_count = 1;

    // this specifies the number of varyings the shader will output. This is
    // for instance used when clipping.
    static const unsigned varying_count = 2;
    static mat4x modelview_matrix;
    static mat4x modelviewprojection_matrix;
    // this static function is called for each vertex to be processed.
    // "in" is an array of void* pointers with the location of the individial
    // vertex attributes. The "out" structure has to be written to.
    static void shade(const swr::GeometryProcessor::VertexInput in, swr::GeometryProcessor::VertexOutput &out)
    {
        // cast the first attribute array to the input vertex type
        const Vertex &v = *static_cast<const Vertex*>(in[0]);

        // x, y, z and w are the components that must be written by the vertex
        // shader. They all have to be specified in 16.16 fixed point format.

        vec4x tvertex = modelviewprojection_matrix * vec4x(v.x,v.y,v.z, fixed16_t(1));

        //const mat4x &m = modelview_matrix;
        //cout << fixedpoint::fix2float<16>(tvertex.x.intValue) << " " <<         fixedpoint::fix2float<16>(tvertex.y.intValue) << " "<<        fixedpoint::fix2float<16>(tvertex.z.intValue)<<endl;

        out.x = tvertex.x.intValue;
        out.y = tvertex.y.intValue;
        out.z = fixed16_t(1.0).intValue;//tvertex.z.intValue;
        out.w = 1 << 16;//tvertex.w.intValue;
        // bring the texture coordinates into the appropriate range for the rasterizer.
        // this mean it has to be converted to fixed point and premultiplied with the width, height of the
        // texture minus 1. Doing this in the vertex shader saves us from doing this in the fragment shader
        // which makes things faster.NOT ANYMORE

        out.varyings[0] = float2fix<16>(v.tx);//static_cast<int>(v.tx * (1 << 16));
        out.varyings[1] = float2fix<16>(v.ty);//static_cast<int>(v.ty * (1 << 16));


    }
    static int triIdx;

    static TextureMipMap *texture;

};

// this is the vertex shader which is executed for each individual vertex that
// needs to ne processed.
struct VertexShaderBlendingDistPass {

    // this specifies that this shader is only going to use 1 vertex attribute
    // array. There you be used up to Renderer::MAX_ATTRIBUTES arrays.
    static const unsigned attribute_count = 1;

    // this specifies the number of varyings the shader will output. This is
    // for instance used when clipping.
    static const unsigned varying_count = 2;
    static mat4x modelview_matrix;
    static mat4x modelviewprojection_matrix;
    // this static function is called for each vertex to be processed.
    // "in" is an array of void* pointers with the location of the individial
    // vertex attributes. The "out" structure has to be written to.
    static void shade(const swr::GeometryProcessor::VertexInput in, swr::GeometryProcessor::VertexOutput &out)
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
        vec4x tvertex = modelviewprojection_matrix * vec4x(v.x,v.y,v.z, fixed16_t(1));
        // cout << fixedpoint::fix2float<16>(tvertex.x.intValue) << " " <<         fixedpoint::fix2float<16>(tvertex.y.intValue) << " "<<        fixedpoint::fix2float<16>(tvertex.z.intValue)<<endl;

        //  cout << v.id << " "<<v.tx << " " << v.ty<<endl;
        // const vec3x &v = i.vertex;
        //const mat4x &m = modelview_matrix;
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
        out.z = fixed16_t(1.0).intValue;//tvertex.z.intValue;
        out.w = 1 << 16;//tvertex.w.intValue;
        // bring the texture coordinates into the appropriate range for the rasterizer.
        // this mean it has to be converted to fixed point and premultiplied with the width, height of the
        // texture minus 1. Doing this in the vertex shader saves us from doing this in the fragment shader
        // which makes things faster.NOT ANYMORE
        out.varyings[0] = float2fix<16>(v.tx);//static_cast<int>(v.tx * (1 << 16));
        out.varyings[1] = float2fix<16>(v.ty);//static_cast<int>(v.ty * (1 << 16));
    }
    static int triIdx;

    static TextureMipMap *texture;

};
struct VertexShaderVar {

    // this specifies that this shader is only going to use 1 vertex attribute
    // array. There you be used up to Renderer::MAX_ATTRIBUTES arrays.
    static const unsigned attribute_count = 1;

    // this specifies the number of varyings the shader will output. This is
    // for instance used when clipping.
    static const unsigned varying_count = 12;
    static mat4x modelviewprojection_matrix;
    // this static function is called for each vertex to be processed.
    // "in" is an array of void* pointers with the location of the individial
    // vertex attributes. The "out" structure has to be written to.
    static void shade(const swr::GeometryProcessor::VertexInput in, swr::GeometryProcessor::VertexOutput &out)
    {
        // cast the first attribute array to the input vertex type
        const VertexBlend &v = *static_cast<const VertexBlend*>(in[0]);

        // x, y, z and w are the components that must be written by the vertex
        // shader. They all have to be specified in 16.16 fixed point format.

        vec4x tvertex = modelviewprojection_matrix * vec4x(v.x,v.y,v.z, fixed16_t(1));

        out.x = tvertex.x.intValue;
        out.y = tvertex.y.intValue;
        out.z = fixed16_t(1.0).intValue;//tvertex.z.intValue;
        out.w = 1 << 16;//tvertex.w.intValue;

        for(int i=0; i <4; i++){
            out.varyings[i*2+0] = static_cast<int>(v.tx[i] * (1 << 16));
            out.varyings[i*2+1] = static_cast<int>(v.ty[i] * (1 << 16));
        }
        for(int i=0; i <4; i++){
            out.varyings[i+9]= static_cast<int>(v.id[i]/1.0 * (1 << 16));
        }

    }


};
#endif // VERTEXSHADERS_H
