
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sstream>
#include "resample.hpp"


inline float max( float a, float b )
{
  return a > b ? a : b;
}

inline float min( float a, float b )
{
  return a < b ? a : b;
}


void downsampleArray( const float *in,int nx ,int ny, float *out,int nnx,int nny  , bool flipx )
{
  const float inRows = (float)ny;
  const float inCols = (float)nx;

  const int outRows = nny;
  const int outCols = nnx;

  const float dx = (float)inCols / (float)outCols;
  const float dy = (float)inRows / (float)outRows;

  const float filterSize = 0.5;

  float sx, sy;
  int x, y;

  for( y = 0, sy = dy/2-0.5f; y < outRows; y++, sy += dy )
    for( x = 0, sx = dx/2-0.5f; x < outCols; x++, sx += dx ) {

      float pixVal = 0;
      float w = 0;
      for( float ix = max( 0, ceilf( sx-dx*filterSize ) ); ix <= min( floorf( sx+dx*filterSize ), inCols-1 ); ix++ )
        for( float iy = max( 0, ceilf( sy-dx*filterSize ) ); iy <= min( floorf( sy+dx*filterSize), inRows-1 ); iy++ ) {
          pixVal += in[ (int)iy*(int)inCols + (int)ix];//(*in)( (int)ix, (int)iy );
          w += 1;
        }
      //(*out)(x,y) = pixVal/w;
      if(flipx)
	out[x*outRows + (outRows-1-y)] = pixVal/w;
      else
 	out[y*outCols + x] = pixVal/w;
    }
}

