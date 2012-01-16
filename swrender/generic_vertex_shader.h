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

#ifndef GENERIC_VERTEX_SHADER_H_
#define GENERIC_VERTEX_SHADER_H_

#if defined (_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

#include "renderer/span.h"
#include "renderer/geometry_processor.h"
#include "renderer/fixed_func.h"
#include "renderer/util.h"
#include "democommon.h"
#include "renderer/util.h"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <GL/glu.h>
#include <vcg/math/matrix44.h>

void printmatrix2(mat4x mat);
void printmatrix3(vcg::Matrix44d mat);
struct GenericVertexShader {
	static const unsigned attribute_count = 4;
        static const unsigned varying_count = 0;

	static void shade(const swr::GeometryProcessor::VertexInput in, swr::GeometryProcessor::VertexOutput &out)
	{
		// fetch vertex
                vec4x v = *static_cast<const vec4x*>(in[0]);
                double tx,ty,tz;

                        gluProject(fixedpoint::fix2float<16>(v.x.intValue),fixedpoint::fix2float<16>(v.y.intValue),
                                   fixedpoint::fix2float<16>(v.z.intValue),
                                   MM,MP,(GLint*)VP,
                                &tx,&ty,&tz);
		// transform and write position
                  //      printmatrix2(modelview_projection_matrix);
                    //    printmatrix3(modelview_projection_matrix_d);
                        vcg::Point4d v2(fixedpoint::fix2float<16>(v.x.intValue),fixedpoint::fix2float<16>(v.y.intValue),
                                        fixedpoint::fix2float<16>(v.z.intValue),1.0);
              // std::cout <<"A:"<< fixedpoint::fix2float<16>(v.x.intValue) << " " <<         fixedpoint::fix2float<16>(v.y.intValue) << " "<<        fixedpoint::fix2float<16>(v.z.intValue)<< " "<<fixedpoint::fix2float<16>(v.w.intValue)<<std::endl;
                v = modelview_projection_matrix * v;
                //std::cout << fixedpoint::fix2float<16>(v.x.intValue) << " " <<         fixedpoint::fix2float<16>(v.y.intValue) << " "<<        fixedpoint::fix2float<16>(v.z.intValue)<<" "<<        fixedpoint::fix2float<16>(v.w.intValue)<<std::endl;
                 v2 = modelview_projection_matrix_d *v2;
                // std::cout << v2.X()<<" " << v2.Y()<< " " << v2.Z()<<" "<<v2.W()<<std::endl;
                //std::cout << fixedpoint::fix2float<16>(v.z.intValue) << "\n";
                //std::cout << "Correct "<<tz <<std::endl;
		out.x = v.x.intValue;
		out.y = v.y.intValue;
		out.z = v.z.intValue;
		out.w = v.w.intValue;

                out.x = float2fix<16>(v2.X());
                out.y = float2fix<16>(v2.Y());
                out.z = float2fix<16>(v2.Z());
                out.w = float2fix<16>(v2.W());
	}

	static inline vec3x direction(const vec4x &from, const vec4x &to)
	{
		if (to.w == 0 && from.w != 0) return normalize(vec3x(to));
		else if (from.w == 0 && to.w != 0) return normalize(-vec3x(from));
		else {
			vec3x pp1 = vec3x(from);
			vec3x pp2 = vec3x(to);
			if (from.w != 1 && from.w != 0) pp1 /= from.w;
			if (to.w != 1 && to.w != 0) pp2 /= to.w;
			return normalize(pp2 - pp1);
		}
	}

	static inline fixed16_t lit(const vec3x &n, const vec3x &l)
	{
		return std::max(dot(n, l), fixed16_t(0));
	}

	static inline vec4x clamp(const vec4x &in)
	{
		vec4x r;
		r.x = std::min(std::max(in.x, fixed16_t(0)), fixed16_t(1));
		r.y = std::min(std::max(in.y, fixed16_t(0)), fixed16_t(1));
		r.z = std::min(std::max(in.z, fixed16_t(0)), fixed16_t(1));
		r.w = std::min(std::max(in.w, fixed16_t(0)), fixed16_t(1));
		return r;
	}

        static mat4x modelview_projection_matrix;
        static vcg::Matrix44d modelview_projection_matrix_d;

};

#endif
