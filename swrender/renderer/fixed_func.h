/*
Copyright (c) 2007, 2008 Markus Trenkwalder

Portions taken from the Vicent 3D rendering library
Copyright (c) 2004, David Blythe, Hans Martin Will

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

#ifndef FIXEDP_FUNC_06240279_53A2_4d2b_AB11_F664FD7D4371
#define FIXEDP_FUNC_06240279_53A2_4d2b_AB11_F664FD7D4371

#if defined (_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

#include "mystdint.h"

namespace swr {
	namespace detail {
		// The template argument p in all of the following functions refers to the 
		// fixed point precision (e.g. p = 8 gives 24.8 fixed point functions).

		// Perform a fixed point multiplication without a 64-bit intermediate result.
		// This is fast but beware of overflow!
		template <int p> 
		inline int32_t fixmulf(int32_t a, int32_t b)
		{
			return (a * b) >> p;
		}

		// Perform a fixed point multiplication using a 64-bit intermediate result to
		// prevent overflow problems.
		template <int p>
		inline int32_t fixmul(int32_t a, int32_t b)
		{
			return (int32_t)(((int64_t)a * b) >> p);
		}

		// Fixed point division
		template <int p>
		inline int fixdiv(int32_t a, int32_t b)
		{
			#if 0
			return (int32_t)((((int64_t)a) << p) / b);
			#else	
			// The following produces the same results as the above but gcc 4.0.3 
			// generates fewer instructions (at least on the ARM processor).
			union {
				int64_t a;
				struct {
					int32_t l;
					int32_t h;
				} i;
			} x;

			x.i.l = a << p;
			x.i.h = a >> (sizeof(int32_t) * 8 - p);
			return (int32_t)(x.a / b);
			#endif
		}

		// Conversion from and to float

		template <int p>
		inline float fix2float(int32_t f)
		{
			return (float)f / (1 << p);
		}

		template <int p>
		inline int32_t float2fix(float f)
		{
			return (int32_t)(f * (1 << p));
		}

		// q is the precision of the input
		// output has 32-q bits of fraction
		// this function was adapted from the fixed point library in the Vincent 
		// library (C) Hans Martin Will
		template <int q>
		inline int fixinv(int32_t a)
		{
			int32_t x;

			if (a == 0) 
				return 0x7fffffff;

			bool sign = false;
			
			
			if (a < 0) {
				sign = true;
				a = -a;
			}
			
			int xa = a;
			int32_t exp = 31;
			if (xa & 0xffff0000) { 
				exp -= 16; 
				xa >>= 16; 
			}
			if (xa & 0xff00) { 
				exp -= 8; 
				xa >>= 8; 
			}	
			if (xa & 0xf0) { 
				exp -= 4; 
				xa >>= 4; 
			}
			if (xa & 0xc) { 
				exp -= 2; 
				xa >>= 2; 
			}	
			if (xa & 0x2) { 
				exp -= 1; 
			}

			static const uint16_t rcp_tab[] = { 
				0x8000, 0x71c7, 0x6666, 0x5d17, 0x5555, 0x4ec4, 0x4924, 0x4444
			};

			x = ((int32_t)rcp_tab[(a>>(28-exp))&0x7]) << 2;
			exp -= 16;

			if (exp <= 0)
				x >>= -exp;
			else
				x <<= exp;

			/* two iterations of newton-raphson  x = x(2-ax) */
			x = fixmul<(32-q)>(x,((2<<(32-q)) - fixmul<q>(a,x)));
			x = fixmul<(32-q)>(x,((2<<(32-q)) - fixmul<q>(a,x)));

			if (sign)
				return -x;
			else
				return x;
		}
	} // end namespace detail
} // end namespace swr

#endif
