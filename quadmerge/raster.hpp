#include <stdio.h>
#include "TriMesh.h"
#include <string>
#include "envelope.hpp"
#ifdef __cplusplus
 extern "C" {
 #endif 
#include "nn/nn.h"
#ifdef __cplusplus
 };
#endif

typedef struct _mesh_input{
  std::string name;
  float res;
  mapnik::Envelope<double> envelope;
}mesh_input;

void raster_triangle(const float* a, const float* b, const float* c);
void interpolate_grid(TriMesh *mesh,const mesh_input &mesh_data, point_nn *&pout,int &nout,int &nx, int &ny,float &cx,float &cy,double &res,int &level);
void write_mesh(point_nn *pin, int nin,const char *fn);
