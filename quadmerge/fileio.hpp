#include "netcdf.h"
#include <string>
#include <iostream>
#include <string.h>
#include "envelope.hpp"
#include "raster.hpp"
#include <GeographicConversions/ufRedfearn.h>
#include <gmt.h>

bool bound_grd( mesh_input &m,double &zmin, double &zmax,double local_easting,double local_northing);

extern UF::GeographicConversions::Redfearn gpsConversion;
bool read_grd_data(const char *name,short &nx,short &ny,float *&data);
void ply_header(FILE *fp,int num_tris,int num_verts,bool ascii=false,bool color=false);
