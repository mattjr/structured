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

#ifndef VERTEX_PROCESSOR_B6F50388_413D_4fe7_837A_E8EADB125DE1
#define VERTEX_PROCESSOR_B6F50388_413D_4fe7_837A_E8EADB125DE1

#if defined (_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

#include <cassert>

namespace swr {

// This class processes vertices and outputs vertices of the type VertexType
// which can be specified as a template parameter. Internally it has a vertex
// cache to detect duplicate vertices. For these vertices the stored result
// will be used insted of calling the vertex shader again.
template <typename VertexType, typename Derived>
class VertexProcessor {
public:
	static const unsigned MAX_ATTRIBUTES = 8;
	typedef VertexType VertexOutput;
	typedef const void *VertexInput[MAX_ATTRIBUTES];

	VertexProcessor() : process_func_(0) {}

	// Set the vertex shader
	template <typename VertexShader>
	void vertex_shader()
	{
		process_func_ = &VertexProcessor<VertexType, Derived>::
			template process_template<VertexShader>;
	}
	
	// Specify the attribute arrays
	void vertex_attrib_pointer(unsigned n, unsigned stride, const void* buffer)
	{
		assert(n < MAX_ATTRIBUTES);
		attributes_[n].stride = stride;
		attributes_[n].buffer = buffer;
	}
	
	// Processes a list of vertices
	void process(unsigned count, unsigned *indices)
	{
		if (process_func_)
			(this->*process_func_)(count, indices);
	}

private:
	struct Attribute {
		unsigned stride;
		const void* buffer;
	} attributes_[MAX_ATTRIBUTES];
	
	void (VertexProcessor<VertexType, Derived>::*process_func_)(unsigned, unsigned*);

	template <typename VertexShader>
	void process_template(unsigned count, unsigned* indices)
	{
		assert(VertexShader::attribute_count <= MAX_ATTRIBUTES);

		static const int VERTEX_CACHE_SIZE = 16;

		struct PostTransformCache {
			unsigned index_in, index_out;
			PostTransformCache():index_in(-1) {}
		} vcache[VERTEX_CACHE_SIZE];

		unsigned vertex_index = 0;

		VertexInput in;

		static_cast<Derived*>(this)->process_begin();
		while (count--) {
			unsigned index = *indices++;

			for (unsigned i = 0; i < VertexShader::attribute_count; ++i) {
				in[i] = static_cast<const char*>(attributes_[i].buffer) + 
					index * attributes_[i].stride;
			}

			unsigned cache_index = index & VERTEX_CACHE_SIZE - 1;
			if (vcache[cache_index].index_in != index) {
				VertexOutput& out = 
					*static_cast<Derived*>(this)->acquire_output_location();
				VertexShader::shade(in, out);
				vcache[cache_index].index_in = index;
				vcache[cache_index].index_out = vertex_index++;
			}

			bool flush_cache = static_cast<Derived*>(this)->push_vertex_index(
				vcache[cache_index].index_out);
			if (flush_cache) {
				vertex_index = 0;
				for (int i = 0; i < VERTEX_CACHE_SIZE; ++i)
					vcache[i].index_in = -1;
			}
		}
		static_cast<Derived*>(this)->process_end();
	}

#if 0
	// interface for derived classes. This code is here for documentation 
	// purposes

	// This method is called every time the vertex processos need a new 
	// location to store the vertex shader output
	typename VertexOutput* acquire_output_location();
	
	// For every processed vertex we push a new index. This starts at 0 and 
	// counts upwards. If we run out of memory and need to start anew this
	// should return true to flush the vertex cache and start counting from 0
	// again.
	bool push_vertex_index(int i);
	
	// called when processing begins
	void process_begin();
	
	// called when processing ends
	void process_end();
#endif
};
} // end namespace swr;

#endif
