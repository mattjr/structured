#include <stdio.h>
#include "TriMesh.h"
#include <string>
#include "envelope.hpp"
typedef struct _mesh_input{
  std::string name;
  float res;
  mapnik::Envelope<double> envelope;
}mesh_input;

void raster_triangle(const float* a, const float* b, const float* c);
void interpolate_grid(TriMesh *mesh,const mesh_input &mesh_data);
