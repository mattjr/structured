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

#ifndef UTIL_95F21CB9_7ABC_486c_A190_70703C3AA2FC
#define UTIL_95F21CB9_7ABC_486c_A190_70703C3AA2FC

#if defined (_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

#include "mystdint.h"

#include <cassert>

namespace swr {
	namespace detail {
		inline void compute_gradients(
			int DX12, int DY12, // deltas in x and y to the other vertices
			int DX31, int DY31,
			int inv_area, // inverse of area in 8.24
			int z0, int z1, int z2, // values at the vertices for which to 
			                        // compute gradients
			int &dx, int &dy) // result will be stored here
		{
			const int DZ12 = z0 - z1;
			const int DZ31 = z2 - z0;

			const int64_t a = (int64_t)DY12 * DZ31 - (int64_t)DZ12 * DY31;
			const int64_t b = (int64_t)DZ12 * DX31 - (int64_t)DX12 * DZ31;

			dx = (int32_t)((a * -inv_area) >> 28);
			dy = (int32_t)((b * -inv_area) >> 28);
		}

		// unsigned integer division
		inline void divmodui(unsigned num, unsigned den, int *q, int *r)
		{
			unsigned bit = 1;
			unsigned res = 0;

			while (den < num && bit && !(den & (1L<<31)))
			{
				den <<=1;
				bit <<=1;
			}
			while (bit)
			{
				if (num >= den)
				{
					num -= den;
					res |= bit;
				}
				bit >>=1;
				den >>=1;
			}
			*q = res;
			*r = num;
		}

		// signed integer division
		inline void divmodsi(int numerator, int denominator, int *quotient, int *remainder)
		{
			bool sign = false;

			if (numerator < 0) {
				numerator = -numerator;
				sign = true;
			}

			if (denominator < 0) {
				denominator = -denominator;
				sign = !sign;
			}

			divmodui(numerator, denominator, quotient, remainder);
			if (sign) *quotient = -*quotient;
		}

		inline void floor_divmod(int numerator, int denominator, int *quotient, 
			int *remainder)
		{
			int q, r;

			assert(denominator > 0);

			if (numerator >= 0) {
				divmodui(numerator, denominator, &q, &r);
			} else {
				divmodui(-numerator, denominator, &q, &r); 
				q = -q;

				if (r != 0) {
					q--;
					r = denominator - r;
				}
			}

			*quotient = q;
			*remainder = r;
		}

		inline int ceil28_4(int value) {
		#if 0
			int result;
			int numerator = value - 1 + 16;
			if (numerator >= 0) {
				result = numerator >> 4;
			} else {
				// deal with negative numerators correctly
				result = -((-numerator) >> 4);
				result -= ((-numerator) & 15) ? 1 : 0;
			}
			return result;
		#else
			return (value + 15) >> 4;
		#endif
		}

		// this can be used to invert a fixed point number of any precision.
		// if input is a 28.4 number output is 4.28.
		inline int invert(int value)
		{
			// tells how many bits of precision are lost. the minimum value is 2.
			// larger values make the function faster but less accurate.
			static const unsigned bit_loss = 2;
			
			if ((value >> bit_loss) == 0) return 0xffffffff;

		#ifndef ARM940
			return (1 << (32 - bit_loss)) / (value >> bit_loss);
		#else
			int q, r;
			divmodsi(1 << (32 - bit_loss), value >> bit_loss, &q, &r);
			return q;
		#endif
		}
	}
}

#endif
