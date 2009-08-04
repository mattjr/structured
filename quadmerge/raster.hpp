#ifndef RASTER_HPP
#define RASTER_HPP
#include <stdio.h>
#include <vector>
#include <string>
#include "envelope.hpp"
typedef struct {
    double x;
    double y;
    double z;
} point_nn;

typedef struct _mesh_input{
  std::string name;
  float res;
  int count;
  int index;
  int interp;
  mapnik::Envelope<double> envelope;
}mesh_input;
extern double min_cell_size;
extern bool natural_neigbor;
extern bool nearest_neigbor;
extern std::vector<point_nn> fixed_grid_data;
void interpolate_grid(float *xyzdata,int numin,float *&dataout,int &nout,const mesh_input &mesh_data,int &nx,int &ny,float &cx,float &cy,int level,double actual_res);
void raster_triangle(const float* a, const float* b, const float* c);



#endif
