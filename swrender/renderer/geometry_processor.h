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

#ifndef GEOMETRYPROCESSOR_H_35BBBF6B_5A83_41c4_B56A_726F99C116CB
#define GEOMETRYPROCESSOR_H_35BBBF6B_5A83_41c4_B56A_726F99C116CB

#if defined (_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

#include "irasterizer.h"
#include "vertex_processor.h"

#include <cstddef>

namespace swr {

	namespace detail {
		template <typename T, int SIZE>
		class static_vector {
			T data_[SIZE];
			size_t size_;
		public:
			static_vector() : size_(0) {}
			static_vector(size_t s, const T& i) : size_(0)
			{ resize(s, i);	}

			size_t size() const 
			{ return size_;	}
			
			void resize(size_t size) 
			{ size_ = size; }

			void resize(size_t size, const T& i)
			{ while (size_ < size) data_[size_++] = i; }

			T& back() 
			{ return data_[size_ - 1]; }

			const T& back() const 
			{ const_cast<static_vector>(this)->back(); }

			void push_back(const T& a)
			{ data_[size_++] = a;}

			void clear()
			{ size_ = 0; }

			T& operator[] (size_t i)
			{ return data_[i]; }

			const T& operator[] (size_t i) const
			{ return const_cast<static_vector>(this)[i]; }
		};
	}

class GeometryProcessor : private VertexProcessor<IRasterizer::Vertex, GeometryProcessor> {
	typedef VertexProcessor<IRasterizer::Vertex, GeometryProcessor> Base;
	friend class VertexProcessor<IRasterizer::Vertex, GeometryProcessor>;
public:
	enum CullMode {
		CULL_NONE, 
		CULL_CCW,
		CULL_CW
	};

	// make these inherited types and constants public
	typedef Base::VertexInput VertexInput;
	typedef Base::VertexOutput VertexOutput;

	static const int MAX_ATTRIBUTES = Base::MAX_ATTRIBUTES;

public:
	GeometryProcessor(IRasterizer *r);

	void rasterizer(IRasterizer *r)
	{ rasterizer_ = r; }

	// public interface

	// upper left is (0,0)
	void viewport(int x, int y, int w, int h);

	// the depth range to use. Normally from 0 to a value less than MAX_INT
	void depth_range(int n, int f);

	void vertex_attrib_pointer(int n, int stride, const void* buffer);

	// count gives the number of indices
	void draw_triangles(unsigned count, unsigned *indices);
	void draw_lines(unsigned count, unsigned *indices);
	void draw_points(unsigned count, unsigned *indices);

	void cull_mode(CullMode m);

	template <typename VertexShader>
	void vertex_shader()
	{
		Base::vertex_shader<VertexShader>();
		varying_count_ = &varying_count_template<VertexShader>;
	}

private:
	template <typename VertexShader>
	static unsigned varying_count_template()
	{
		return VertexShader::varying_count;
	}

private:
	void add_interp_vertex(int t, int out, int in);

	void pdiv_and_vt();

	void clip_triangles();
	void process_triangles();

	void clip_lines();
	void process_lines();

	void clip_points();
	void process_points();

private:
	// interface inherited from the vertex processor
	void process_begin();
	VertexOutput* acquire_output_location();
	bool push_vertex_index(unsigned i);
	void process_end();

private:
	// maximum number of triangles to accumulate before going on with
	// clipping and triangle setup
	static const unsigned MAX_TRIANGLES = 32;
	static const unsigned MAX_LINES = 32;
	static const unsigned MAX_POINTS = 32;

	static const unsigned MAX_VERTICES_INDICES =
		MAX_TRIANGLES * 3 + 
		MAX_TRIANGLES * 12; // Clipping generates additional vertices

	enum DrawMode {
		DM_TRIANGLES,
		DM_LINES,
		DM_POINTS
	};

	DrawMode draw_mode_;

	detail::static_vector<VertexOutput, MAX_VERTICES_INDICES> vertices_;
	detail::static_vector<unsigned, MAX_VERTICES_INDICES> indices_;

	struct {
		int ox, oy; // origin x and y
		int px, py; // width and height divided by 2
	} viewport_;

	struct {
		int fmndiv2, npfdiv2; // (f - n)/2 | (n + f)/2
	} depth_range_;

	CullMode cull_mode_;

	// internal function pointer; returns the number of varyings to interpolate.
	unsigned (*varying_count_)();

	IRasterizer *rasterizer_;
};

} // end namespace swr

#endif
