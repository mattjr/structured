/*
Copyright (c) 2007, Markus Trenkwalder

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

#include "SDL.h"
#include "SDL_image.h"

// requires software renderer source
#include "renderer/geometry_processor.h"
#include "renderer/rasterizer_subdivaffine.h"
#include "renderer/span.h"

// the software renderer stuff is located in the namespace "swr" so include 
// that here
using namespace swr;

// our vertex structure which will be used to store our triangle data
struct Vertex {
	float x, y;
	float tx, ty;
};

// our texture class
struct Texture {
	SDL_Surface *surface;

	unsigned w_log2, h_log2;
	unsigned w_minus1, h_minus1;
 
	// create a texture from an SDL_Surface	
	Texture(SDL_Surface *s) 
	{
		surface = s;

		w_log2 = log2_of_pot(s->w);
		h_log2 = log2_of_pot(s->h);

		w_minus1 = s->w - 1;
		h_minus1 = s->h - 1;
	}

	~Texture() 
	{
		SDL_FreeSurface(surface); 
	}

	// returns log2 of a number which is a power if two
	unsigned log2_of_pot(unsigned v) const
	{
		unsigned r = 0;
		while (!(v & 1)) {
			v >>= 1;
			++r;
		}
		return r;
	}

	// samples the texture using the given texture coordinate.
	// the integer texture coordinate is given in the upper 16 bits of the x and y variables.
	// it is NOT in the range [0.0, 1.0] but rather in the range of [0, width - 1] or [0, height - 1].
	// texture coordinates outside this range are wrapped.
	unsigned short sample_nearest(int x, int y) const
	{
		x >>= 16;
		y >>= 16;
		x &= w_minus1;
		y &= h_minus1;
		return *(static_cast<unsigned short*>(surface->pixels) + ((y << w_log2) + x));
	}
};

// loads an image file using SDL_image and converts it to a r5g5a1b5 format.
// this format can be directly copied to the screen and one can also use the 
// embedded alpha bit for an alpha test.
SDL_Surface* load_surface_r5g5a1b5(const char *filename)
{
	const Uint32 rmask = 0xF800;
	const Uint32 gmask = 0x7C0;
	const Uint32 bmask = 0x1F;
	const Uint32 amask = 0x20;

	SDL_Surface *result = 0;
	SDL_Surface *img = 0;
	SDL_Surface *dummy = 0;

	img = IMG_Load(filename); if (!img) goto end;
	dummy = SDL_CreateRGBSurface(0, 0, 0, 16, rmask, gmask, bmask, amask); if (!dummy) goto end;
	result = SDL_ConvertSurface(img, dummy->format, 0);

end:
	if (img) SDL_FreeSurface(img);
	if (dummy) SDL_FreeSurface(dummy);

	return result;
}

// this is the vertex shader which is executed for each individual vertex that
// needs to ne processed.
struct VertexShader {

	// this specifies that this shader is only going to use 1 vertex attribute 
	// array. There you be used up to Renderer::MAX_ATTRIBUTES arrays.
	static const unsigned attribute_count = 1;

	// this specifies the number of varyings the shader will output. This is
	// for instance used when clipping.
	static const unsigned varying_count = 2;

	// this static function is called for each vertex to be processed.
	// "in" is an array of void* pointers with the location of the individial 
	// vertex attributes. The "out" structure has to be written to.
	static void shade(const GeometryProcessor::VertexInput in, GeometryProcessor::VertexOutput &out)
	{
		// cast the first attribute array to the input vertex type
		const Vertex &v = *static_cast<const Vertex*>(in[0]);

		// x, y, z and w are the components that must be written by the vertex
		// shader. They all have to be specified in 16.16 fixed point format.
		out.x = static_cast<int>((v.x * (1 << 16)));
		out.y = static_cast<int>((v.y * (1 << 16)));
		out.z = 0;
		out.w = 1 << 16;

		// bring the texture coordinates into the appropriate range for the rasterizer.
		// this mean it has to be converted to fixed point and premultiplied with the width, height of the 
		// texture minus 1. Doing this in the vertex shader saves us from doing this in the fragment shader 
		// which makes things faster.
		out.varyings[0] = static_cast<int>(v.tx * texture->w_minus1 * (1 << 16));
		out.varyings[1] = static_cast<int>(v.ty * texture->h_minus1 * (1 << 16));
	}

	static Texture *texture;
};

Texture *VertexShader::texture = 0;

// this is the fragment shader
struct FragmentShader : public SpanDrawer16BitColorAndDepth<FragmentShader> {
	// varying_count = 3 tells the rasterizer that it only needs to interpolate
	// three varying values (the r, g and b in this context).
	static const unsigned varying_count = 2;

	// we don't need to interpolate z in this example
	static const bool interpolate_z = false;

	// per triangle callback. This could for instance be used to select the 
	// mipmap level of detail. We don't need it but it still needs to be defined
	// for everything to work.
	static void begin_triangle(
		const IRasterizer::Vertex& v1,
		const IRasterizer::Vertex& v2,
		const IRasterizer::Vertex& v3,
		int area2)
	{}

	// the fragment shader is called for each pixel and has read/write access to 
	// the destination color and depth buffers.
	static void single_fragment(const IRasterizer::FragmentData &fd, unsigned short &color, unsigned short &depth)
	{
		// sample the texture and write the color information
		unsigned short c = texture->sample_nearest(fd.varyings[0], fd.varyings[1]);

#	ifdef ALPHA_TEST
		// do an alpha test and only write color if the test passed
		if (c & (1 << 5)) {
			// write the color
			color = c; 
		}
#	else
		// always write the color;
		color = c;
#	endif
	}

	// this is called by the span drawing function to get the location of the color buffer
	static void* color_pointer(int x, int y)
	{
		SDL_Surface *screen = SDL_GetVideoSurface();
		return static_cast<unsigned short*>(screen->pixels) + x + y * screen->w;
	}

	// this is called by the span drawing function to get the location of the depth buffer
	static void* depth_pointer(int x, int y)
	{
		// We don't use a depth buffer
		return 0;
	}

	static Texture *texture;
};

Texture *FragmentShader::texture = 0;

int main(int ac, char *av[]) {
	// initialize SDL without error handling an all
	SDL_Init(SDL_INIT_VIDEO);
	SDL_Surface *screen = SDL_SetVideoMode(640, 480, 16, 0);

	// the four vertices of the textured quad together with the texture coordinates
	Vertex vertices[] = {
		{-1.0f,  1.0f, 0.0f, 0.0f},
		{-1.0f, -1.0f, 0.0f, 1.0f},
		{ 1.0f, -1.0f, 1.0f, 1.0f},
		{ 1.0f,  1.0f, 1.0f, 0.0f}
	};

	// load the texture file
	Texture *texture = new Texture(load_surface_r5g5a1b5("texture.png"));

	// make the shaders know which texture to use
	VertexShader::texture = texture;
	FragmentShader::texture = texture;

	// the indices we need for rendering
	unsigned indices[] = {0, 1, 2, 0, 2, 3};

	// create a rasterizer class that will be used to rasterize primitives
	RasterizerSubdivAffine r;
	// create a geometry processor class used to feed vertex data.
	GeometryProcessor g(&r);
	// it is necessary to set the viewport
	g.viewport(0, 0, screen->w, screen->h);
	// set the cull mode (CW is already the default mode)
	g.cull_mode(GeometryProcessor::CULL_CW);

	// it is also necessary to set the clipping rectangle
	r.clip_rect(0, 0, screen->w, screen->h);

	// set the vertex and fragment shaders
	g.vertex_shader<VertexShader>();
	r.fragment_shader<FragmentShader>();

	// specify where out data lies in memory
	g.vertex_attrib_pointer(0, sizeof(Vertex), vertices);

	// draw the triangle
	g.draw_triangles(6, indices);

	// show everything on screen
	SDL_Flip(SDL_GetVideoSurface());

	// wait for the user closing the application
	SDL_Event e;
	while (SDL_WaitEvent(&e) && e.type != SDL_QUIT);

	// free texture memory
	delete texture;

	// quit SDL
	SDL_Quit();
	return 0;
}
