// File: drawtext.frag
// Author: Mike Weiblen 2005-10-05
// Copyright (C) 2005  3Dlabs Inc. Ltd.
// All rights reserved.
// 
// An OpenGL Shading Language fragment shader to render numeric values as
// text, providing visibility into GLSL shaders.
// Inspired by Jeff Boody's printnum shader.
// Requires the verasansmono.png glyph image.
// Designed to run with fixed-function vertex processing, so no GLSL
// vertex shader is required.
//
///////////////////////////////////////////////////////////////////////////
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
//     Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
// 
//     Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
// 
//     Neither the name of 3Dlabs Inc. Ltd. nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

///////////////////////////////////////////////////////////////////////////
// Convert geometry texcoord to cell index; i.e.: a character's row/column

uniform vec2 CellSpan;    // number of cells across texture

ivec2 getCellIndex( vec2 geomTexCoord )
{
    return ivec2( geomTexCoord * CellSpan );
}

///////////////////////////////////////////////////////////////////////////
// Return the texel of an ASCII character's glyph at the given geometry
// texture coordinate.

uniform sampler2D GlyphTexSampler; // contains the verasansmono.png image

vec4 readGlyphTexture( vec2 geomTexCoord, int ascii )
{
    // glyph scaling constants for the verasansmono.png image.
    const vec2 glyScale = vec2( 1.0/16.0, (300.0/384.0)/6.0 );

    vec2 cellTexCoord = fract( geomTexCoord * CellSpan );
    if( (ascii < 32) || (ascii > 127) ) ascii = 127;
    float glyNum = float(ascii - 32) * glyScale.x;
    vec2 glyTexOrigin = vec2( fract(glyNum), glyScale.y * floor(glyNum) );
    vec2 glyTexCoord = cellTexCoord * glyScale + glyTexOrigin;
    return texture2D( GlyphTexSampler, glyTexCoord );
}

///////////////////////////////////////////////////////////////////////////
// Return a single ASCII character from the decimal expansion of a float.
// (change "base" to render as binary, decimal, or hexadecimal)

int Float2Ascii( float val, int pos )
{
    const float base = 10.;

    if( (pos == 0) && (val < 0.) ) return 45;   // minus sign '-'

    // at which cell position is the radix point?
    int radixPos = 0;
    if( val < 0. ) radixPos++;                  // room for minus sign
    float x = abs(val);
    do {
        x = floor( x / base );
        radixPos++;
    } while( x > 0. );
    if( pos == radixPos ) return 46;            // radix point '.'

    float exp = float(pos - radixPos);
    if( pos < radixPos ) exp++;
    int digit = int( mod( abs(val) * pow( base, exp ), base ) );
    //if( digit > 9 ) digit += 39;              // for hexadecimal
    return digit + 48;                          // add ascii zero '0'
}

///////////////////////////////////////////////////////////////////////////
// Utility functions which, when given a cell position, return a single
// ASCII character to be rendered in that cell.
///////////////////////////////////////////////////////////////////////////

// Display each vec4 component as a decimal ASCII string, analogous to
// printf( "%f", data[cell.y] )

int getFloatCharacter( ivec2 cell, float data[4] )
{
   return Float2Ascii( data[cell.y], cell.x );
}

///////////////////////////////////////////////////////////////////////////
// Display each vec4 component as a single ASCII character, analogous to
// printf( "%4c", data[0], data[1], data[2], data[3] )

int getWordCharacter( ivec2 cell, vec4 data )
{
    return int( data[cell.x] );
}

///////////////////////////////////////////////////////////////////////////
// The main() function configures what and how to display the desired data.
// Below are some simple examples, use as you wish.
///////////////////////////////////////////////////////////////////////////

// render the vec4 "Word" as 4 ascii characters

#if 0 //[

// drawtext.frag external declarations (if necessary)
ivec2 getCellIndex( vec2 geomTexCoord );
vec4 readGlyphTexture( vec2 geomTexCoord, int ascii );
int getWordCharacter( ivec2 cell, vec4 data );

uniform vec4 Word;

void main(void)
{
    vec2 geomTexCoord = gl_TexCoord[0].xy;
    ivec2 cellIndex = getCellIndex( geomTexCoord );
    int ascii = getWordCharacter( cellIndex, Word );
    vec4 texel = readGlyphTexture( geomTexCoord, ascii );
    gl_FragColor = gl_Color * texel;
}

#endif //]

///////////////////////////////////////////////////////////////////////////

#if 1 //[

// render a vec4 as 4 lines of printf-like numeric strings.

// drawtext.frag external declarations (if necessary)
ivec2 getCellIndex( vec2 geomTexCoord );
vec4 readGlyphTexture( vec2 geomTexCoord, int ascii );
//int getFloatCharacter( ivec2 cell, vec4 data );
#extension GL_ARB_texture_rectangle : enable
uniform mat4 osg_ViewMatrix;
uniform int osg_FrameNumber;
uniform float osg_FrameTime;
uniform float osg_DeltaFrameTime;
uniform sampler2DRect hist;

void main(void)
{
  
  float t=0;
  float t2=0;
  /*
  for(int i=0; i<64; i++)
    for(int j=0; j<64; j++){

      vec4 histV=texture2DRect(hist,vec2(i,j));
      if(histV.x > 0){
	t=(float)i;
	t2=(float)j;
      }
      }*/
    float data[4];
    vec4 ass=texture2DRect(hist,vec2(2,0));
float fft=128.0*texture2DRect(hist,vec2(2,0)).x;
  data[0]=texture2DRect(hist,vec2(2,0)).x;
  data[1]=texture2DRect(hist,vec2(3,0)).x;
  data[2]=texture2DRect(hist,vec2(4,0)).x;
  data[3]=0.1;

    // some examples of interesting data to render...
  //data[0] =  gl_ModelViewMatrix[3];
    //data = osg_ViewMatrix[3];
    //data = gl_LightSource[0].position;
    //data = gl_LightSource[0].diffuse;
    //data = vec4( osg_FrameTime, osg_DeltaFrameTime, float(osg_FrameNumber), 0.0 );

    vec2 geomTexCoord = gl_TexCoord[0].xy;
    ivec2 cellIndex = getCellIndex( geomTexCoord );
    int ascii = getFloatCharacter( cellIndex, data );
    vec4 texel = readGlyphTexture( geomTexCoord, ascii );
    gl_FragColor = gl_Color * texel;
    //   gl_FragColor = vec4(1.0,0,0,0);
}

#endif //]

// vim: set sw=4 ts=8 et ic ai:
