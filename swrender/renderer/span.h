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


#ifndef SPAN_5C49406D_561D_4dd1_85B0_9EBA56E58CB2
#define SPAN_5C49406D_561D_4dd1_85B0_9EBA56E58CB2

#ifdef _MSC_VER
#pragma once
#endif

#include "irasterizer.h"
#include "duffsdevice.h"
#include "fixed_func.h"
#include <map>
#include <set>
#include <stdio.h>
#include "stepmacros.h"

namespace swr {

	template <typename FragmentShader>
	struct SpanDrawerBase {
		static const int AFFINE_LENGTH = 24;

		static IRasterizer::FragmentData compute_step_al(
			const IRasterizer::FragmentData &fdl, 
			const IRasterizer::FragmentData &fdr)
		{
			IRasterizer::FragmentData r;

			if (FragmentShader::interpolate_z)
				r.z = (fdr.z - fdl.z) / AFFINE_LENGTH;

			DUFFS_DEVICE8(
				int i = 0,
				r.varyings[i] = (fdr.varyings[i] - fdl.varyings[i]) / AFFINE_LENGTH; ++i,
				FragmentShader::varying_count,
				/**/)

			return r;
		}

		static IRasterizer::FragmentData compute_step(
			const IRasterizer::FragmentData &fdl, 
			const IRasterizer::FragmentData &fdr,
			int inv_delta)
		{
			using namespace detail;
			
			IRasterizer::FragmentData r;

			if (FragmentShader::interpolate_z)
				r.z = fixmul<16>(fdr.z - fdl.z, inv_delta);

			DUFFS_DEVICE8(
				int i = 0,
				r.varyings[i] = fixmul<16>(fdr.varyings[i] - fdl.varyings[i], inv_delta); ++i,
				FragmentShader::varying_count,
				/**/)

			return r;
		}

		static IRasterizer::FragmentData fd_from_fds(const IRasterizer::FragmentDataPerspective &fd)
		{
			using namespace detail;
			
			IRasterizer::FragmentData r = IRasterizer::FragmentData();

			if (FragmentShader::interpolate_z)
				r.z = fd.fd.z;

			if (FragmentShader::varying_count) {
				int w = detail::invert(fd.oow);

				DUFFS_DEVICE8(
					int i = 0,
					r.varyings[i] = fixmul<16>(fd.fd.varyings[i], w); ++i,
					FragmentShader::varying_count,
					/**/)
			}

			return r;
		}

		static void perspective_span(
			int x, 
			int y, 
			const IRasterizer::FragmentDataPerspective &fd_in, 
			const IRasterizer::FragmentDataPerspective &step, 
			unsigned n) 
		{
			using namespace detail;

			IRasterizer::FragmentDataPerspective fds[2];
			FRAGMENTDATA_PERSPECTIVE_APPLY(FragmentShader, fds[0], = , fd_in);

			IRasterizer::FragmentData fd[2];
			fd[0] = fd_from_fds(fds[0]);

			while (AFFINE_LENGTH <= static_cast<int>(n)) {
				FRAGMENTDATA_PERSPECTIVE_APPLY(FragmentShader, fds[1], = , fds[0])
				FRAGMENTDATA_PERSPECTIVE_APPLY(FragmentShader, fds[1], += AFFINE_LENGTH *, step)

				fd[1] = fd_from_fds(fds[1]);

				const IRasterizer::FragmentData step = compute_step_al(fd[0], fd[1]);

				FragmentShader::affine_span(x, y, fd[0], step, AFFINE_LENGTH);
				x += AFFINE_LENGTH; n -= AFFINE_LENGTH;

				FRAGMENTDATA_PERSPECTIVE_APPLY(FragmentShader, fds[0], =, fds[1])
				FRAGMENTDATA_APPLY(FragmentShader, fd[0], =, fd[1])
			}

			if (n) {
				const int inv_n = detail::invert(n << 16);

				FRAGMENTDATA_PERSPECTIVE_APPLY(FragmentShader, fds[1], = , fds[0])
				FRAGMENTDATA_PERSPECTIVE_APPLY(FragmentShader, fds[1], += n *, step)

				fd[1] = fd_from_fds(fds[1]);

				const IRasterizer::FragmentData step = compute_step(fd[0], fd[1], inv_n);

				FragmentShader::affine_span(x, y, fd[0], step, n);
			}
		}
	};

	template <typename FragmentShader>
	struct GenericSpanDrawer : public SpanDrawerBase<FragmentShader> {
		static void affine_span(
			int x, 
			int y, 
			IRasterizer::FragmentData fd, 
			const IRasterizer::FragmentData &step, 
			unsigned n)
		{
			DUFFS_DEVICE8(
				/**/,
				FragmentShader::single_fragment(x++, y, fd);
				FRAGMENTDATA_APPLY(FragmentShader, fd, +=, step),
				n,
				/**/)
		}
	};

	template <typename FragmentShader>
	struct SpanDrawer16BitColorAndDepth : public SpanDrawerBase<FragmentShader> {
		static void affine_span(
			int x, 
			int y, 
			IRasterizer::FragmentData fd, 
			const IRasterizer::FragmentData &step, 
			unsigned n)
		{
			unsigned short *color16_pointer = static_cast<unsigned short*>(FragmentShader::color_pointer(x, y));
			unsigned short *depth16_pointer = static_cast<unsigned short*>(FragmentShader::depth_pointer(x, y));

			// using duffs device for loop unrolling can improve performance
			// and seems useful even when using -funroll-loops in GCC.
			// In a few tests the above was 5% faster than a normal while loop with -funroll-loops.

			DUFFS_DEVICE16(
				/**/,
				{
					FragmentShader::single_fragment(fd, *color16_pointer, *depth16_pointer);
					FRAGMENTDATA_APPLY(FragmentShader, fd, += , step);

					color16_pointer++;
					depth16_pointer++;
				},
				n,
				/**/)
		}
	};


        template <typename FragmentShader>
        struct SpanDrawer32BitColorAndDepth : public SpanDrawerBase<FragmentShader> {
                static void affine_span(
                        int x,
                        int y,
                        IRasterizer::FragmentData fd,
                        const IRasterizer::FragmentData &step,
                        unsigned n)
                {
                        unsigned int *color32_pointer = static_cast<unsigned int*>(FragmentShader::color_pointer(x, y));
                        unsigned int *depth32_pointer = static_cast<unsigned int*>(FragmentShader::depth_pointer(x, y));

                        // using duffs device for loop unrolling can improve performance
                        // and seems useful even when using -funroll-loops in GCC.
                        // In a few tests the above was 5% faster than a normal while loop with -funroll-loops.

                        DUFFS_DEVICE32(
                                /**/,
                                {
                          //  if(x==183 && y==289)
                                        FragmentShader::single_fragment(fd, *color32_pointer, *depth32_pointer);
                                        FRAGMENTDATA_APPLY(FragmentShader, fd, += , step);

                                        color32_pointer++;
                                        depth32_pointer++;
                                },
                                n,
                                /**/)
                }
        };



        template <typename FragmentShader>
        struct SpanDrawer32BitColorAndDepthSetDouble : public SpanDrawerBase<FragmentShader> {
                static void affine_span(
                        int x,
                        int y,
                        IRasterizer::FragmentData fd,
                        const IRasterizer::FragmentData &step,
                        unsigned n)
                {
                        unsigned int *color32_pointer = static_cast<unsigned int*>(FragmentShader::color_pointer(x, y));
                        unsigned int *depth32_pointer = static_cast<unsigned int*>(FragmentShader::depth_pointer(x, y));
                        // using duffs device for loop unrolling can improve performance
                        // and seems useful even when using -funroll-loops in GCC.
                        // In a few tests the above was 5% faster than a normal while loop with -funroll-loops.
                        int tmpx=x;
                        int tmpy=y;
                        DUFFS_DEVICE32(
                                /**/,
                                {

                                        FragmentShader::single_fragment(fd, tmpx,tmpy,*color32_pointer, *depth32_pointer);
                                        FRAGMENTDATA_APPLY(FragmentShader, fd, += , step);

                                        color32_pointer++;
                                        depth32_pointer++;
                                        tmpx++;

                                },
                                n,
                                /**/)
                }
        };
/*        std::pair<int,int> p=std::make_pair<int,int>(x,y);
        if(doublecountmap.count(p)){
            if(doublecountmap[p]==FragmentShader::tri_idx)
                    FragmentShader::single_fragment(fd, *color32_pointer, *depth32_pointer);*/

}

#include "stepmacros_undef.h"

#endif
