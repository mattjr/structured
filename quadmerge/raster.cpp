#include "raster.hpp"
float kill_threshold_squared = 2500.0f; // 50*50 units


//----------------------------------------------------------------------------
// CLAMP TO RASTER INT THAT IS INSIDE TRIANGLE BIASED TOWARDS MIN (INCLUSIVE)
//----------------------------------------------------------------------------

static inline int Ceil(const float x)
{
  int ceilx = (int)x;
  if (x>ceilx) ceilx++;
  return ceilx;
}

static inline int Floor(const float x)
{
  int floorx = (int)x;
  if (x<floorx) {
//    fprintf(stderr,"FLOOR: decrement happens: %f %d\n", x, floorx);
    floorx--;
  }
  return floorx;
}

/*
  Rasterize a triangle, handling all the boundary cases.
  We know that ay<by<cy, with ties broken on x coordinate 
    (horizontal lines are thought of as having positive slope epsilon)
  We shift the triangle up and right to resolve points on edges.
    (makes sure northing & easting are in a triangle).
  Thus, y range is floor(ay)+1 to <floor(cy)+1; 
  and x range is ceil(first x) to < ceil(xlimit)

  We consider scanlines in two cases:
    < b: from floor(ay)+1 to <=by (or <floor(by)+1), then
    >=b: from max(floor(ay)+1,floor(by)+1) to <=cy (or <floor(cy)+1).

  For scanline iy, we find the first x and limit x coordinates.
  Both are rounded down (toward the left), 
    so the first point is included if on the boundary 
    and the last is not if on the boundary.
  There are two subcases (since we know that the orientation det != 0): 
    if det > 0, then the triangle is to the right of middle point b
       <=b case: first x determined by ab, bc; xlimit by ac;
       > b case: first x determined by ac; xlimit by ab, bc;
   if det < 0, then the triangle is to the left of middle point b
       <=b case: first x determined by ac; xlimit by ab, bc;
       > b case: first x determined by ab, bc; xlimit by ac;

  In each case, if the line is ab, ix = Ceil(ax + (ax-bx)/(ay-by)*(iy-ay)) 
 
  That determines the pairs (ix,iy) at which to interpolate.

  The linear interpolant satisfies my favorite determinant in homog coords:
    Det[b; a; c; (1 ix iy Z)] = 0.
  Subtract first row to go to cartesian coords: 
    Det[a-b; c-b; (ix-bx iy-by Z-bz)] = 0;
  Expand by minors in last row:
    Z = bz - (ix-bx)*Dx/det + (iy-by)*Dy/det, 
  where det = the orientation Det[ax-bx ay-by; cx-bx cy-by];
    Dx  = Det[ay-by az-bz; cy-by cz-bz]; 
    Dy  = Det[ax-bx az-bz; cx-bx cz-bz]; 
*/
 void raster_triangle(const float* a, const float* b, const float* c)
{
  const float* t;

  // SORT VERTICES BY Y VALUES, breaking ties by x
#define SWAP(a,b,t) t=a,a=b,b=t
#define LEXGREATER(a,b) (a[1]>b[1] || (a[1] == b[1] && a[0]>b[0]))

  // enforce ay<=by<=cy
  if (LEXGREATER(a,c)) SWAP(a,c,t);
  if (LEXGREATER(a,b)) SWAP(a,b,t);
  else if (LEXGREATER(b,c)) SWAP(b,c,t);
      
  int iy = Floor(a[1])+1;      // start just above lowest point = a
  if (iy > c[1])
  {
#ifdef COLLECT_STATISTICS
    count_early_exits++;
#endif
    //   return; // triangles has no rasters
  }
    // CALCULATE EDGE VALUE DIFFERENCES IN X and Y DIRECTION
  float ABx=b[0]-a[0], ABy=b[1]-a[1];
  float BCx=c[0]-b[0], BCy=c[1]-b[1];
  
  // Move so b is origin. 
  float Ax=a[0]-b[0], Ay=a[1]-b[1];
  float Cx=c[0]-b[0], Cy=c[1]-b[1];
  float ACx=c[0]-a[0], ACy=c[1]-a[1];
 
  double det = Ax*Cy - Ay*Cx; // orientation determinant: + if A->C is ccw around b. 0 for degenerate line triangle;
  
  if (det == 0 || Ax*Ax + Ay*Ay > kill_threshold_squared || ACx*ACx + ACy*ACy > kill_threshold_squared || Cx*Cx + Cy*Cy > kill_threshold_squared)
  {
#ifdef COLLECT_STATISTICS
    killed_triangles++;
#endif
    //    return;  // triangle is too badly shaped
  }

  double ACxy = ACx/ACy;  // know ACy>0
  double Az = a[2]-b[2], Cz = c[2]-b[2], ACz = c[2]-a[2];
  double Dx = (Az*Cy-Ay*Cz)/det;  // linear interp: Z = (x-b0)*Dx+(y-b1)*Dy+b2
  double Dy = (Ax*Cz-Az*Cx)/det; 
  int ix, Xlimit; 

  if (iy <= b[1]) // do a_y to b_y range only if iy<=b_y
  {          
    double xy = Ax/Ay;        // NB: here we know Ay = a_y-b_y < 0.

    for (; iy <= b[1] ; iy++) {        // do scanline iy

      int abx = Ceil(b[0] + xy*(iy-b[1]));  // ix and Xlimit 
      int acx = Ceil(a[0]+ACxy*(iy-a[1]));  //   for 2 subcases:
      if (det>0) {ix = abx; Xlimit = acx;}  // -triangle right of b
      else {ix = acx; Xlimit = abx;};    // -triangle left of b

      if (ix < Xlimit)
      {
        double Z = b[2] + Dy*(iy-b[1]) + Dx*(ix-b[0]); // starting Z
        for (; ix < Xlimit; ix++) {
	  printf("%d %d %f\n",iy,ix,Z);
          //srwriter->write_raster(iy, ix, (float) Z);
          Z += Dx;
        }
      }
    } 
  }

  if (iy <= c[1]) // do b_y to c_y range only if b_y<c_y
  {
    double xy = Cx/Cy;          // NB: know Cy = c_y-b_y > 0.

    for (; iy <= c[1]; iy++) {    // do scanline iy

      int cbx = Ceil(b[0] + xy*(iy-b[1]));  // ix and Xlimit 
      int cax = Ceil(c[0]+ACxy*(iy-c[1]));  //   for 2 subcases:
      if (det>0) {ix = cbx; Xlimit = cax;}  // -triangle right of b
      else {ix = cax; Xlimit = cbx;};    // -triangle left of b

      if (ix<=Xlimit)
      {
        double Z = b[2] + Dy*(iy-b[1]) + Dx*(ix-b[0]); // starting Z
        for (; ix < Xlimit; ix++)
        {
	  printf("%d %d %f\n",iy,ix,Z);
	  //  srwriter->write_raster(iy, ix, (float) Z);
          Z += Dx;
        }
      }
    }
  }
}
