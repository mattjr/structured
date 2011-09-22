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

#ifndef RASTERIZER_EE4F8987_BDA2_434d_A01F_BB3446E535C3
#define RASTERIZER_EE4F8987_BDA2_434d_A01F_BB3446E535C3

#if defined (_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

#include "fixed_func.h"
#include "util.h"
#include "rasterizer_tsbase.h"

#include "stepmacros.h"

#include <cmath>
#include <algorithm>

namespace swr {

class RasterizerSubdivAffine : public RasterizerTemplateShaderBase {
public:
	// constructor
	RasterizerSubdivAffine() : 
		triangle_func_(0),
		perspective_correction_(true)
	{
		perspective_threshold(0, 0);
	}

public:
	// main interface

	void perspective_correction(bool enable)
	{ perspective_correction_ = enable;	}

	void perspective_threshold(int w, int h)
	{
		perspective_threshold_.w = w;
		perspective_threshold_.h = h;
	}

	// set the fragment shader
	template <typename FragSpan>
	void fragment_shader()
	{
		RasterizerTemplateShaderBase::fragment_shader<FragSpan>();
		triangle_func_ = &RasterizerSubdivAffine::triangle_template<FragSpan>;
	}

	void draw_triangle(const Vertex &v1, const Vertex &v2, const Vertex &v3)
	{
		if (triangle_func_)
			(this->*triangle_func_)(v1, v2, v3);
	}

private:
	void (RasterizerSubdivAffine::*triangle_func_)(const Vertex &v1, const Vertex &v2, 
		const Vertex &v3);

	bool perspective_correction_;
	
	struct {
		int w;
		int h;
	} perspective_threshold_;

	// The triangle must be counter clockwise in screen space in order to be
	// drawn.
	template <typename FragSpan>
	void triangle_template(const Vertex &v1, const Vertex &v2, const Vertex &v3)
	{
		using namespace detail;

		// Early bounds test. Skip triangle if outside clip rect. 
		// This is intended for the special case when the clip_rect is smaller
		// than the whole screen as is the case in dual cpu rendering, when each 
		// cpu renders only part of the screen.
		int minx = std::min(std::min(v1.x, v2.x), v3.x) >> 4;
		int miny = std::min(std::min(v1.y, v2.y), v3.y) >> 4;
		int maxx = std::max(std::max(v1.x, v2.x), v3.x) >> 4;
		int maxy = std::max(std::max(v1.y, v2.y), v3.y) >> 4;

		if (minx >= clip_rect_.x1 || miny >= clip_rect_.y1 || 
			maxx < clip_rect_.x0 || maxy < clip_rect_.y0)
			return;

		// Deltas
		const int DX12 = v1.x - v2.x;
		const int DX31 = v3.x - v1.x;

		const int DY12 = v1.y - v2.y;
		const int DY31 = v3.y - v1.y;

		// this is actually twice the area
		const int area = DX12 * DY31 - DX31 * DY12;

		if (area <= 0xf)
			return;

		// Execute per triangle function. This can be used to compute the mipmap
		// level per primitive. To support mipmap per pixel at least one varying 
		// attributes would need to be written by this function but this isn't 
		// possible for now as the vertices are declared const.
		FragSpan::begin_triangle(v1, v2, v3, area);
	
		// inv_area in 8.24
		const int inv_area = invert(area);

		// sort vertices in y and determine on which side the middle vertex lies
		const Vertex *top, *middle, *bottom;
		bool middle_is_left;

		if (v1.y < v2.y) {
			if (v1.y < v3.y) {
				top = &v1;
				if (v2.y < v3.y) {
					middle = &v2;
					bottom = &v3;
					middle_is_left = true;
				} 
				else {
					middle = &v3;
					bottom = &v2;
					middle_is_left = false;
				}
			} 
			else {
				top = &v3;
				middle = &v1;
				bottom = &v2;
				middle_is_left = true;
			}			
		}
		else {
			if (v2.y < v3.y) {
				top = &v2;
				if (v1.y < v3.y) {
					middle = &v1;
					bottom = &v3;
					middle_is_left = false;
				} 
				else {
					middle = &v3;
					bottom = &v1;
					middle_is_left = true;
				}
			}
			else {
				top = &v3;
				middle = &v2;
				bottom = &v1;
				middle_is_left = false;
			}
		}

		if (perspective_correction_ && (
			(maxx - minx) > perspective_threshold_.w || 
			(maxy - miny) > perspective_threshold_.h ))
		{
			// computes the gradients of the varyings to be used for stepping
			struct Gradients {
				FragmentDataPerspective dx;
				FragmentDataPerspective dy;

				Gradients(const Vertex &v1, const Vertex &v2, const Vertex &v3,
					int DX12, int DY12, int DX31, int DY31, int inv_area)
				{
					if (FragSpan::interpolate_z)
						compute_gradients(DX12, DY12, DX31, DY31, 
							inv_area, v1.z, v2.z, v3.z, dx.fd.z, dy.fd.z);

					if (FragSpan::varying_count) {
						int invw1 = invert(v1.w);
						int invw2 = invert(v2.w);
						int invw3 = invert(v3.w);

						compute_gradients(DX12, DY12, DX31, DY31, 
							inv_area, invw1, invw2, invw3, dx.oow, dy.oow);

						for (unsigned i = 0; i < FragSpan::varying_count; ++i) {
							int var1 = fixmul<16>(v1.varyings[i], invw1);
							int var2 = fixmul<16>(v2.varyings[i], invw2);
							int var3 = fixmul<16>(v3.varyings[i], invw3);

							compute_gradients(DX12, DY12, DX31, DY31, 
								inv_area, var1, var2, var3, 
								dx.fd.varyings[i], dy.fd.varyings[i]);
						}
					}
				}
			};

			// Edge structure used to walk the edge of the triangle and fill the
			// scanlines
			struct Edge {
				int x, x_step, numerator, denominator, error_term; // DDA info for x
				int y, height;

				FragmentDataPerspective fragment_data;
				FragmentDataPerspective fragment_step;

				const Gradients& grad;		

				Edge(const Gradients &grad_, const Vertex* top, const Vertex *bottom)
					: grad(grad_)
				{
					y = ceil28_4(top->y);
					int yend = ceil28_4(bottom->y);
					height = yend - y;

					if (height) {
						int dn = bottom->y - top->y;
						int dm = bottom->x - top->x;

						int initial_numerator = dm*16*y - dm*top->y + 
							dn*top->x - 1 + dn*16;
						floor_divmod(initial_numerator, dn*16, &x, &error_term);
						floor_divmod(dm*16, dn*16, &x_step, &numerator);
						denominator = dn*16;

						int y_prestep = y*16 - top->y;
						int x_prestep = x*16 - top->x;
						
						#define PRESTEP(VAR) \
							(((y_prestep * grad.dy.VAR) >> 4) + \
							((x_prestep * grad.dx.VAR) >> 4))
							
						#define STEP(VAR) \
							(x_step * grad.dx.VAR + grad.dy.VAR)

						if (FragSpan::interpolate_z) {
							fragment_data.fd.z = top->z + PRESTEP(fd.z);
							fragment_step.fd.z = STEP(fd.z);
						}

						if (FragSpan::varying_count) {
							int invw = invert(top->w);

							fragment_data.oow = invw + PRESTEP(oow);
							fragment_step.oow = STEP(oow);

							for (unsigned i = 0; i < FragSpan::varying_count; ++i) {
								fragment_data.fd.varyings[i] = 
									fixmul<16>(top->varyings[i], invw) + PRESTEP(fd.varyings[i]);
								fragment_step.fd.varyings[i] = STEP(fd.varyings[i]);
							}
						}

						#undef STEP
						#undef PRESTEP
					}
				}

				void step(bool step_varyings)
				{
					x += x_step; y++; height--;

					if (step_varyings) 
						FRAGMENTDATA_PERSPECTIVE_APPLY(FragSpan, fragment_data, +=, fragment_step);

					error_term += numerator;
					if (error_term >= denominator) {
						x++;
						if (step_varyings) 
							FRAGMENTDATA_PERSPECTIVE_APPLY(FragSpan, fragment_data, +=, grad.dx);
						error_term -= denominator;
					}
				}
			};

			Gradients grad(v1, v2, v3, DX12, DY12, DX31, DY31, inv_area);
			Edge top_middle(grad, top, middle);
			Edge top_bottom(grad, top, bottom);
			Edge middle_bottom(grad, middle, bottom);

			Edge *left, *right;
			if (middle_is_left) {
				left = &top_middle;
				right = &top_bottom;
			}
			else {
				left = &top_bottom;
				right = &top_middle;
			}

			struct Scanline {
				static void draw(const Edge *left, const Edge *right, int cl, int cr)
				{
					int y = left->y;
					int l = left->x;
					int r = std::min(right->x, cr);
					const Gradients &grad = left->grad;

					if (r - l <= 0) return;

					// suppress possible GCC warnings by doing copy construction on fdp.
					// should not harm performance.
					FragmentDataPerspective fdp = FragmentDataPerspective();
					FRAGMENTDATA_PERSPECTIVE_APPLY(FragSpan, fdp, =, left->fragment_data);

					// skip pixels left of the clipping rectangle
					if (l < cl) {
						int d = cl - l;
						FRAGMENTDATA_PERSPECTIVE_APPLY(FragSpan, fdp, += d *, grad.dx);
						l = cl;
					}

					FragSpan::perspective_span(l, y, fdp, grad.dx, r - l);
				}
			};

			int height = middle_is_left ? left->height : right->height;

			// draw top triangle
			while (height) {
				int y = left->y;
				if (ilace_drawit(y) && y >= clip_rect_.y0 && y < clip_rect_.y1) 
					Scanline::draw(left, right, clip_rect_.x0, clip_rect_.x1);
				left->step(true);
				right->step(false);
				height--;
			}

			if (middle_is_left) {
				left = &middle_bottom;
				height = left->height;
			}
 			else {
				right = &middle_bottom;
				height = right->height;
			}

			// draw bottom triangle
			while (height) {
				int y = left->y;
				if (ilace_drawit(y) && y >= clip_rect_.y0 && y < clip_rect_.y1) 
					Scanline::draw(left, right, clip_rect_.x0, clip_rect_.x1);
				left->step(true);
				right->step(false);
				height--;
			}
		}
		// affine interpolation and rendering
		else {
			// computes the gradients of the varyings to be used for stepping
			struct Gradients {
				FragmentData dx;
				FragmentData dy;

				Gradients(const Vertex &v1, const Vertex &v2, const Vertex &v3,
					int DX12, int DY12, int DX31, int DY31, int inv_area)
				{
					if (FragSpan::interpolate_z)
						compute_gradients(DX12, DY12, DX31, DY31, 
							inv_area, v1.z, v2.z, v3.z, dx.z, dy.z);

					for (unsigned i = 0; i < FragSpan::varying_count; ++i)
						compute_gradients(DX12, DY12, DX31, DY31, 
						inv_area, v1.varyings[i], v2.varyings[i], v3.varyings[i], 
						dx.varyings[i], dy.varyings[i]);
				}
			};

			// Edge structure used to walk the edge of the triangle and fill the
			// scanlines
			struct Edge {
				int x, x_step, numerator, denominator, error_term; // DDA info for x
				int y, height;

				FragmentData fragment_data;
				FragmentData fragment_step;

				const Gradients& grad;		

				Edge(const Gradients &grad_, const Vertex* top, const Vertex *bottom)
					: grad(grad_)
				{
					y = ceil28_4(top->y);
					int yend = ceil28_4(bottom->y);
					height = yend - y;

					if (height) {
						int dn = bottom->y - top->y;
						int dm = bottom->x - top->x;

						int initial_numerator = dm*16*y - dm*top->y + 
							dn*top->x - 1 + dn*16;
						floor_divmod(initial_numerator, dn*16, &x, &error_term);
						floor_divmod(dm*16, dn*16, &x_step, &numerator);
						denominator = dn*16;

						int y_prestep = y*16 - top->y;
						int x_prestep = x*16 - top->x;

						#define PRESTEP(VAR) \
							(((y_prestep * grad.dy.VAR) >> 4) + \
							((x_prestep * grad.dx.VAR) >> 4))
							
						#define STEP(VAR) \
							(x_step * grad.dx.VAR + grad.dy.VAR)

						if (FragSpan::interpolate_z) {
							fragment_data.z = top->z + PRESTEP(z);
							fragment_step.z = STEP(z);
						}

						for (unsigned i = 0; i < FragSpan::varying_count; ++i) {
							fragment_data.varyings[i] = top->varyings[i] + PRESTEP(varyings[i]);
							fragment_step.varyings[i] = STEP(varyings[i]);
						}

						#undef PRESTEP
						#undef STEP
					}
				}

				void step(bool step_varyings)
				{
					x += x_step; y++; height--;

					if (step_varyings)
						FRAGMENTDATA_APPLY(FragSpan, fragment_data, +=, fragment_step);

					error_term += numerator;
					if (error_term >= denominator) {
						x++;
						if (step_varyings)
							FRAGMENTDATA_APPLY(FragSpan, fragment_data, +=, grad.dx);
						error_term -= denominator;
					}
				}
			};

			Gradients grad(v1, v2, v3, DX12, DY12, DX31, DY31, inv_area);
			Edge top_middle(grad, top, middle);
			Edge top_bottom(grad, top, bottom);
			Edge middle_bottom(grad, middle, bottom);

			Edge *left, *right;
			if (middle_is_left) {
				left = &top_middle;
				right = &top_bottom;
			}
			else {
				left = &top_bottom;
				right = &top_middle;
			}

			struct Scanline {
				static void draw(const Edge *left, const Edge *right, int cl, int cr)
				{
					int y = left->y;
					int l = left->x;
					int r = std::min(right->x, cr);
					const Gradients &grad = left->grad;

					if (r - l <= 0) return;

					// suppress possible GCC warnings by doing copy construction on fd.
					// should not harm performance.
					FragmentData fd = FragmentData();
					FRAGMENTDATA_APPLY(FragSpan, fd, =, left->fragment_data);

					// skip pixels left of the clipping rectangle
					if (l < cl) {
						int d = cl - l;
						FRAGMENTDATA_APPLY(FragSpan, fd, += d *, grad.dx);
						l = cl;
					}

					// draw the scanline up until the right side
					FragSpan::affine_span(l, y, fd, grad.dx, r - l);
				}
			};

			int height = middle_is_left ? left->height : right->height;

			// draw top triangle
			while (height) {
				int y = left->y;
				if (ilace_drawit(y) && y >= clip_rect_.y0 && y < clip_rect_.y1) 
					Scanline::draw(left, right, clip_rect_.x0, clip_rect_.x1);
				left->step(true);
				right->step(false);
				height--;
			}

			if (middle_is_left) {
				left = &middle_bottom;
				height = left->height;
			}
			else {
				right = &middle_bottom;
				height = right->height;
			}

			// draw bottom triangle
			while (height) {
				int y = left->y;
				if (ilace_drawit(y) && y >= clip_rect_.y0 && y < clip_rect_.y1) 
					Scanline::draw(left, right, clip_rect_.x0, clip_rect_.x1);
				left->step(true);
				right->step(false);
				height--;
			}
		}
	}
};
} // end namespace swr

#include "stepmacros_undef.h"

#endif
