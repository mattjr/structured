#include "netcdf.h"
#include <string>
#include <iostream>
#include <string.h>
#include "envelope.hpp"
#include "raster.hpp"
#include <GeographicConversions/ufRedfearn.h>
#include <gmt.h>

void bound_grd( mesh_input &m,double &zmin, double &zmax);
extern double local_easting, local_northing;
extern double gridConvergence, pointScale;
extern std::string zone;
extern UF::GeographicConversions::Redfearn gpsConversion;
bool read_grd_data(const char *name,short &nx,short &ny,float *&data);
