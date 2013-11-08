//////////////////////////////////////////////////
// Copyright (c) INRIA (France) 2011, 2012, 2013
// 
// This file is part of inria-mvs. You can redistribute it and/or
// modify it under the terms of the GNU General Public License.
// 
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
// 
// Author: Jean-Paul CHIEZE <jean-paul.chieze@inria.fr>
// 
//////////////////////////////////////////////////

#include "delaunay.h"
#include <math.h>
#include <ctype.h>
#ifdef __USE_POSIX
#include <time.h>
#endif

void usage(char *prog, bool fatal = true) {
  std::cout << "Usage : " << prog << " input.cgal " << std::endl;
  std::cout << "    Repartition of number of intersections" << std::endl;
  std::cout << "    Each line gives the nb of tetrahedrons more intersections than given after '>'." << std::endl;
  exit(1);
}

int main(int argc,char **argv)
{
#ifdef __USE_POSIX
  struct timespec tstart, tend;
#else
  time_t tstart, tend;
#endif
  if(argc != 2 || *(argv[1]) == '-') usage(argv[0]);
  Delaunay T;
  PointColor pcolors;
  std::vector<Point> normals;
#ifdef __USE_POSIX
  clock_gettime(CLOCK_REALTIME,&tstart);
#else
  tstart = time(NULL);
#endif
  CGAL::Bbox_3 bb;
  int nbcams;
  int *cams_index;
  std::map<int, VisiblePatches*> image_patches;
  TPoint bad_cameras;
  float dx,dy,dz;
  float edge_mean, tetra_coefs[2];
  try {
    read_cgal_data(argv[1],T,pcolors,normals,bb,&nbcams,&cams_index,image_patches,bad_cameras,CG_BADCAMS,&edge_mean,tetra_coefs);
#ifdef __USE_POSIX
    clock_gettime(CLOCK_REALTIME,&tend);
#else
    tend = time(NULL);
#endif
    std::cout << "Read took " << delta_t(&tstart,&tend) << "s" << std::endl;
    dx = bb.xmax() - bb.xmin();
    dy = bb.ymax() - bb.ymin();
    dz = bb.zmax() - bb.zmin();
    // marquage des cameras utiles(V)
    for(int icam = 0;icam < nbcams;icam++) {
      pcolors[cams_index[icam]] = CGAL::Color(0,255,0);
    }
  }
  catch(char const *_e) {
  std::cout << "ERROR " << _e << std::endl;
    return 1;
    
  }
  long nbcells = std::distance(T.finite_cells_begin(),T.finite_cells_end());
  std::cout << "BOX : " << bb << ", NBCELLS: " << nbcells << std::endl;
  std::cout << "Average edge length : " << sqrt(edge_mean) << ", tetras coefs " << tetra_coefs[0] << " " << tetra_coefs[1] << std::endl;
  long hist[9] = {0,0,0,0,0,0,0,0,0}; // > 0 1 2 3 4 5 6 10 100
  int limits[9] = {0,1,2,3,4,5,6,10,100};
  long nbzeros = 0;
  for (Delaunay::Finite_cells_iterator cit = T.finite_cells_begin(); cit != T.finite_cells_end(); ++cit) {
    Cell_handle c = cit;
    int info = c->info() >> 1;
    if (info == 0) nbzeros++;
    else {
      for(int i = 0;i <= 9;i++)
	if (info > limits[i]) hist[i] += 1;
    }
  }
  long nb = nbcells - nbzeros;
  std::cout << nbzeros << " tetras non atteints, " << nb << " atteints" << std::endl;
  std::cout << "Histogramme :" << std::endl;
  for(int i = 0;i < 9;i++) {
    float v = (float)hist[i] / (float)nb;
    std::cout << ">" << limits[i] << " " << hist[i] << " [" << v << "]" << std::endl;
  }
  std::cout << std::endl;
    //Segment s = Segment(Point(bb.xmin(), bb.ymin(),bb.zmin()),Point(bb.xmax(), bb.ymax(),bb.zmax()));
    //  box_lgr = s.squared_length();
 return 0;
}
