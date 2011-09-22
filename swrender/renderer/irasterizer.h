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

#ifndef IRASTERIZER_8B18E9A9_B88A_4f69_9E6B_99A9EFCDE545
#define IRASTERIZER_8B18E9A9_B88A_4f69_9E6B_99A9EFCDE545

#if defined (_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

namespace swr {

	class IRasterizer {
	public:
		virtual ~IRasterizer() {};

                static const int MAX_VARYING = 12;

		// Type definitions
		struct Vertex {
			int x, y; // in 28.4 fixed point
			int z; // range from 0 to 0x7fffffff
			int w; // in 16.16 fixed point
			int varyings[MAX_VARYING];
		};

		// This is the data necessary for each fragment. It is defined here
		// as probably all rasterizers will need this.
		struct FragmentData {
			int z;
			int varyings[MAX_VARYING];
		};

		// Use for perspective spans. Defined here for convenience
		struct FragmentDataPerspective {
			int oow;
			FragmentData fd;
		};

		virtual void clip_rect(int x, int y, int w, int h) = 0;
		virtual void draw_triangle(const Vertex &v1, const Vertex &v2, const Vertex &v3) = 0;
		virtual void draw_line(const Vertex &v1, const Vertex &v2) = 0;
		virtual void draw_point(const Vertex &v1) = 0;
	};
}

#endif
