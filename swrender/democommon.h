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

#ifndef DEMOCOMMON_H_552F7375_1BA4_4f12_91F4_51950EF31347
#define DEMOCOMMON_H_552F7375_1BA4_4f12_91F4_51950EF31347

#include "vector_math.h"
#include "fixedpoint/fixed_class.h"

using namespace vmath;
using namespace fixedpoint;

typedef fixed_point<16> fixed16_t;

typedef vec2<fixed16_t> vec2x;
typedef vec3<fixed16_t> vec3x;
typedef vec4<fixed16_t> vec4x;
typedef mat4<fixed16_t> mat4x;

#define X(a) fixed16_t(a)

#include <algorithm>

#ifdef GP2X
#include "flush_uppermem_cache.h"
#else
static inline void flush_uppermem_cache(void *start_address, void *end_address, int flags){}
#endif

/*inline void fade_effect(SDL_Surface *s, bool ltr, float t)
{
	for (int y = 0; y < 240; y += 16) {
		for (int x = 0; x < 320; x += 16) {

			float sz;
			if (ltr)
				sz = ((1.0f - (x/304.0f)) -1.0f + t * 2.0f);
			else
				sz = ((x/304.0f) -1.0f + t * 2.0f);

			int size = std::max(0, std::min(16, (int)(sz * 16 + 0.5f)));

			SDL_Rect r = {x + 8 - size / 2, y + 8 - size / 2, size, size};
			SDL_FillRect(s, &r, 0);
		}
	}
}
*/
#endif
