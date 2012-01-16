/*
    Fusion2X - OpenGL ES-CL 1.0 Implementation
    Copyright (C) 2008 Markus Trenkwalder

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

    Markus Trenkwalder
    trenki2@gmx.net
*/

#ifndef GENERIC_FRAGMENT_SHADER_H_
#define GENERIC_FRAGMENT_SHADER_H_

#if defined (_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

#include "renderer/span.h"
#include "renderer/util.h"
#include "GL/gl.h"
#include "fixedpoint/fixed_class.h"
typedef struct F2X_RenderSurface {
        unsigned format;
        unsigned width;
        unsigned height;
        unsigned pitch;
        void *data;
} F2X_RenderSurface;
struct GenericFragmentShader : public swr::GenericSpanDrawer<GenericFragmentShader> {
        static const unsigned varying_count = 0;
	static const bool interpolate_z = true;

	static void begin_triangle(
		const swr::IRasterizer::Vertex& v1,
		const swr::IRasterizer::Vertex& v2,
		const swr::IRasterizer::Vertex& v3,
		int area2)
        {}

	static void single_fragment(
		int x,
		int y,
		const swr::IRasterizer::FragmentData &fd)
	{
		// depth test moved before color computations
                if (s_depth_buffer.data) {
                        unsigned short *depth_buffer =
                                static_cast<unsigned short*>(s_depth_buffer.data) + x + y * s_depth_buffer.pitch / 2;

			// only shift by 15 to take the most significant bit.
			// the highest bit is just the sign and should always be positive
                        //const unsigned short depth = fd.z >> 15;

                        const unsigned short depthbuf = *depth_buffer;
                        float f = fixedpoint::fix2float<16>(fd.z);
                        f = f * 0.5 + 0.5;

                        f=(f *(1.0-1e-3))+1e-3;
                        float depth = f;
                        float *dbuf =
                                depthZ +(s_depth_buffer.width*(s_depth_buffer.width-y-1)+x);
                        unsigned char *color =
                                                static_cast<unsigned char*>(s_color_buffer.data) +(s_color_buffer.width*(s_color_buffer.width-y-1)+x);


                        switch (depth_test_func) {
			case GL_NEVER: return;
			case GL_LESS:
                                if (!(depth < *dbuf)) return;
				break;
			case GL_EQUAL: 
                                if (!(depth == *dbuf)) return;
				break;
			case GL_LEQUAL: 
                                if (!(depth <= *dbuf)) return;
				break;
			case GL_GREATER: 
                                if (!(depth > *dbuf)) return;
				break;
			case GL_NOTEQUAL: 
                                if (!(depth != *dbuf)) return;
				break;
			case GL_GEQUAL: 
                                if (!(depth >= *dbuf)) return;
				break;
			case GL_ALWAYS: 
				break;
			}

			// handle write mask
                        //if (ctx->gl_state.framebuf_ctrl.depth_writemask)
                            //    *depth_buffer = depth;
                        //unsigned char *color =
                          //                      static_cast<unsigned char*>(s_color_buffer.data) +(s_color_buffer.width*(s_color_buffer.width-y-1)+x);

                        *color=flatcolor;
                        *dbuf=f;
		}

		// skip color computations when there is no color buffer
                //return;
                //if (!ctx->color_buffer.data) return;
		

	}
        static F2X_RenderSurface s_color_buffer;
        static F2X_RenderSurface s_depth_buffer;
        static GLint depth_test_func;
        static float *depthZ;
        static unsigned char flatcolor;

};

#endif
