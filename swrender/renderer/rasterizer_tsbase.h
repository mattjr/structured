/*
Copyright (c) 2007, 2008 Markus Trenkwalder

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

#ifndef RASTERIZERTSBASE_77D1E6A1_8DE1_4e2e_9F99_F1F2F04873B9
#define RASTERIZERTSBASE_77D1E6A1_8DE1_4e2e_9F99_F1F2F04873B9

#if defined (_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

#include "irasterizer.h"
#include "util.h"

#include <algorithm>

namespace swr {

	namespace detail {
		inline int abs(int x) 
		{ 
			return x >= 0 ? x : -x; 
		}
	}

	// Base rasterizer implementation for all rasterizers where the shader
	// can be set via a template function. Here only line and point functions
	// will be implemented. The triangle drawing function is implemented by
	// specialized shaders.
	class RasterizerTemplateShaderBase : public IRasterizer {
	protected:
		struct { int x0, y0, x1, y1; } clip_rect_;
		struct { int offset; int mask; } interlace_;
		unsigned varying_count_;

	public:
		RasterizerTemplateShaderBase()
		{
			clip_rect(0, 0, 0, 0); 
			interlace(0, 0);
		} 

		// upper left is (0,0)
		void clip_rect(int x, int y, int w, int h)
		{
			clip_rect_.x0 = std::max(0, x);
			clip_rect_.y0 = std::max(0, y);
			clip_rect_.x1 = std::max(0, x + w);
			clip_rect_.y1 = std::max(0, y + h);
		}

		unsigned varying_count()
		{ return varying_count_; }

		// set the fragment shader
		template <typename FragSpan>
		void fragment_shader()
		{
			varying_count_ = FragSpan::varying_count;
			line_func_ = &RasterizerTemplateShaderBase::line_template<FragSpan>;
			point_func_ = &RasterizerTemplateShaderBase::point_template<FragSpan>;
		}

		void draw_line(const Vertex &v1, const Vertex &v2)
		{
			if (line_func_)
				(this->*line_func_)(v1, v2);
		}

		void draw_point(const Vertex &v1)
		{
			if (point_func_)
				(this->*point_func_)(v1);
		}

		// Controls interlacing. Uses the following formula to determine if a 
		// row should be drawn:
		// 
		// bool drawit = ((y + offset) & mask) == 0
		//
		// Thus to draw every line you can pass (0,0). To draw every line with 
		// even y you pass (0,1). For odd lines you pass (1,1).
		//
		// This should give enough control. More control could only be achived 
		// by doing (y + offset) % some_value == 0 which requires an expensive
		// mod.
		void interlace(int offset, int mask) 
		{
			interlace_.offset = offset;
			interlace_.mask = mask;
		}

	protected:
		inline bool clip_test(int x, int y)
		{
			return (x >= clip_rect_.x0 && x < clip_rect_.x1 &&
				x >= clip_rect_.y0 && y < clip_rect_.y1);
		}

		inline bool ilace_drawit(int y)
		{
			return ((y + interlace_.offset) & interlace_.mask) == 0;
		}

	private:
		void (RasterizerTemplateShaderBase::*line_func_)(const Vertex &v1, const Vertex &v2);
		void (RasterizerTemplateShaderBase::*point_func_)(const Vertex &v1);

	private:
		template <typename FragSpan>
		void line_template(const Vertex &v1, const Vertex &v2)
		{
			using namespace detail;

			int x0 = v1.x >> 4;
			int y0 = v1.y >> 4;

			int x1 = v2.x >> 4;
			int y1 = v2.y >> 4;

			int dx = x1 - x0;
			int dy = y1 - y0;

			int adx = detail::abs(dx);
			int ady = detail::abs(dy);

			if (dx == 0 && dy == 0) 
				return;

			FragmentData fragment_data;

			struct step_info {
				int step;
				int remainder;
				int error_term;

				void init(int start, int end, int delta)
				{
					floor_divmod(end-start, delta, &step, &remainder);
					error_term = 0;
				}

				int do_step(int absdelta) 
				{
					int r = step;
					error_term += remainder;
					if (error_term >= absdelta) {
						error_term -= absdelta;
						r++;
					}
					return r;
				}
			} step_z, step_v[MAX_VARYING];

			if (adx > ady) {
				int xstep = dx > 0 ? 1 : -1;
				//int ystep_extra = dy > 0 ? 1 : -1;
				int ystep, remainder, error_term = 0;
				floor_divmod(dy, adx, &ystep, &remainder);

				if (FragSpan::interpolate_z) {
					fragment_data.z = v1.z;
					step_z.init(v1.z, v2.z, adx);
				}
				for (unsigned i = 0; i < FragSpan::varying_count; ++i) {
					fragment_data.varyings[i] = v1.varyings[i];
					step_v[i].init(v1.varyings[i], v2.varyings[i], adx);
				}

				int x = x0;
				int y = y0;
				while (x != x1) {
					x += xstep;
					y += ystep;
					error_term += remainder;
					if (error_term >= adx) {
						error_term -= adx;
						y++;
					}

					if (FragSpan::interpolate_z)
						fragment_data.z += step_z.do_step(adx);

					for (unsigned i = 0; i < FragSpan::varying_count; ++i)
						fragment_data.varyings[i] += step_v[i].do_step(adx);

					if (ilace_drawit(y) && clip_test(x, y))
						FragSpan::affine_span(x, y, fragment_data, fragment_data, 1);
				}
			}
			else {
				int ystep = dy > 0 ? 1 : -1;
				//int xstep_extra = dx > 0 ? 1 : -1;
				int xstep, remainder, error_term = 0;
				floor_divmod(dx, ady, &xstep, &remainder);

				if (FragSpan::interpolate_z) {
					fragment_data.z = v1.z;
					step_z.init(v1.z, v2.z, ady);
				}
				for (unsigned i = 0; i < FragSpan::varying_count; ++i) {
					fragment_data.varyings[i] = v1.varyings[i];
					step_v[i].init(v1.varyings[i], v2.varyings[i], ady);
				}

				int x = x0;
				int y = y0;
				while (y != y1) {
					y += ystep;
					x += xstep;
					error_term += remainder;
					if (error_term >= ady) {
						error_term -= ady;
						x++;
					}

					if (FragSpan::interpolate_z)
						fragment_data.z += step_z.do_step(adx);

					for (unsigned i = 0; i < FragSpan::varying_count; ++i)
						fragment_data.varyings[i] += step_v[i].do_step(adx);

					if (ilace_drawit(y) && clip_test(x, y))
						FragSpan::affine_span(x, y, fragment_data, fragment_data, 1);
				}
			}
		}

		template <typename FragSpan>
		void point_template(const Vertex &v1)
		{
			FragmentData fd;

			int x = v1.x >> 4;
			int y = v1.y >> 4;

			if (!clip_test(x, y) || !ilace_drawit(y)) 
				return;

			if (FragSpan::interpolate_z)
				fd.z = v1.z;

			for (unsigned i = 0; i < FragSpan::varying_count; ++i)
				fd.varyings[i] = v1.varyings[i];

			FragSpan::affine_span(x, y, fd, fd, 1);
		}
	};
}

#endif
