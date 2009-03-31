#include "raster.hpp"
#include "uquadtree.hpp"
#include <float.h>
float kill_threshold_squared = 2500.0f; // 50*50 units
#include "nn/delaunay.h"
#include <sys/time.h>
#include "fileio.hpp"
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
}*/

 void write_mesh(point_nn *pin, int nin,const char *fn,bool ascii){
   if(nin < 3 ){
     fprintf(stderr,"Less than three interpolated points\n");
     return;
   }
   point_nn* pin_clean=new point_nn[nin];
   int nin_clean=0;
   for(int i=0; i < nin; i++){
     if(!std::isnan(pin[i].z ))
       pin_clean[nin_clean++]=pin[i];
   }
   if(nin_clean == 0)
     return;
   delaunay* d = delaunay_build(nin_clean, pin_clean, 0, NULL, 0, NULL);
   FILE *fp=fopen(fn,"w");
   if(!fp){
     fprintf(stderr,"Cannont open %s\n",fn);
     return;
   }
   ply_header(fp,d->ntriangles,d->npoints,ascii);
   float buf[3];
   for(int i=0; i<d->npoints; i++){
     buf[0]=d->points[i].x;
     buf[1]=d->points[i].y;
     buf[2]=d->points[i].z;
     if(ascii)
       fprintf(fp,"%f %f %f\n",buf[0],buf[1],buf[2]);
     else 
       fwrite((char*)&buf,3,sizeof(float),fp);
   }
   unsigned char numt=3;
   int ibuf[3];
   for(int i=0; i<d->ntriangles; i++){
     if(!ascii)
       fwrite((char*)&numt,1,sizeof(unsigned char),fp);
     ibuf[2]=d->triangles[i].vids[0];
     ibuf[1]=d->triangles[i].vids[1];
     ibuf[0]=d->triangles[i].vids[2];
     if(ascii)
       fprintf(fp,"3 %d %d %d\n",ibuf[0],ibuf[1],ibuf[2]);
     else
       fwrite((char*)&ibuf,3,sizeof(int),fp);
   }
   delete pin_clean;

}

void interpolate_grid(TriMesh *mesh,const mesh_input &mesh_data, point_nn *&pout,int &nout,int &nx,int &ny,float &cx,float &cy,double &res,int &level,bool extrap){
  point_nn* pin = NULL;
 
  pout = NULL;
  
  nout = 0;
  //struct timeval tv0, tv1, tv2;
  //struct timezone tz;
  int nv = mesh->vertices.size();
  pin = (point_nn *)malloc(nv * sizeof(point_nn));

    char fname[255];
  std::string::size_type slash = mesh_data.name.rfind('/');
  if (slash == std::string::npos)
    slash = 0;
  else
    slash++;

  sprintf(fname,"tmp2/%s",mesh_data.name.substr(slash).c_str());
  FILE *fp=fopen(fname,"w");

  for (int i = 0; i < nv; i++){
    point_nn* p = &pin[i];
    
    p->x = mesh->vertices[i][0];
    p->y = mesh->vertices[i][1];
    p->z = mesh->vertices[i][2];

    fprintf(fp,"%f %f %f\n",p->x,p->y,p->z);
  }
  fclose(fp);
  int nin=nv;
  points_thinlin(&nin,&pin,0.1);
  double actual_res;
  ge.get_closest_res_level(mesh_data.res,level,actual_res);
  printf("Target Res %f Actual Res %f Level %d\n",mesh_data.res,actual_res,level);
  // actual_res=0.001;
  cx=mesh_data.envelope.center().x;
  cy=mesh_data.envelope.center().y;

  nx=(int)floor(mesh_data.envelope.width()/actual_res);
  ny=(int)floor(mesh_data.envelope.height()/actual_res);
   printf("%f %f %d %d\n",nx,ny,cx,cy);
  // std::cout << mesh_data.envelope<<std::endl;
   double wmin =0;// -1;
  //  if(extrap)
  //  wmin=-DBL_MAX;
  points_generate(mesh_data.envelope.minx(),mesh_data.envelope.maxx(),mesh_data.envelope.miny(),mesh_data.envelope.maxy(),nx,ny,&nout, &pout);
  //  nnpi_interpolate_points(nin, pin, wmin, nout, pout);
          lpi_interpolate_points(nin, pin, nout, pout);

  free(pin);
}


void interpolate_grid(float *xyzdata,const mesh_input &mesh_data, point_nn *&pout,int &nout,int &nx,int &ny,float &cx,float &cy,double &res,int &level){
  point_nn* pin = NULL;
 
  pout = NULL;
  
  nout = 0;
  //struct timeval tv0, tv1, tv2;
  //struct timezone tz;
  int nv = mesh_data.count;
  pin = (point_nn *)malloc(nv * sizeof(point_nn));
  int cnt=0;
  for (int i = 0; i < nv; i++){
    point_nn* p = &pin[i];
    
    p->x = xyzdata[cnt];
    p->y = xyzdata[cnt+1];
    p->z = xyzdata[cnt+2];
    cnt +=3;
    
  }
  int nin=nv;
  points_thinlin(&nin,&pin,0.1);
  double actual_res;
  ge.get_closest_res_level(mesh_data.res,level,actual_res);
  //  printf("Target Res %f Actual Res %f Level %d\n",mesh_data.res,actual_res,level);
  cx=mesh_data.envelope.center().x;
  cy=mesh_data.envelope.center().y;

  nx=(int)floor(mesh_data.envelope.width()/actual_res);
  ny=(int)floor(mesh_data.envelope.height()/actual_res);
  double wmin = 0;//-DBL_MAX;
  points_generate(mesh_data.envelope.minx(),mesh_data.envelope.maxx(),mesh_data.envelope.miny(),mesh_data.envelope.maxy(),nx,ny,&nout, &pout);
  nnpi_interpolate_points(nin, pin, wmin, nout, pout);
  free(pin);
}
