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

void usage(char *prog, bool fatal = true) {
  std::cout << "Usage : " << prog << " facets.ply " << std::endl;
  std::cout << "    Counts points being vertices of facets" << std::endl;
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
  TPoint points;
  PointColor pcolors;
  std::vector<Point> normals;
  std::vector<Face> faces;
#ifdef __USE_POSIX
  clock_gettime(CLOCK_REALTIME,&tstart);
#else
  tstart = time(NULL);
#endif
  CGAL::Bbox_3 bb;
  try {
    bb = read_ply(argv[1],points,normals,pcolors,faces);
#ifdef __USE_POSIX
    clock_gettime(CLOCK_REALTIME,&tend);
#else
    tend = time(NULL);
#endif
    std::cout << "Read took " << delta_t(&tstart,&tend) << "s" << std::endl;
  }
  catch(char const *_e) {
  std::cout << "ERROR " << _e << std::endl;
    return 1;
    
  }
  int npts = points.size();
  std::vector<bool> validpts(npts);
  validpts.assign(npts,false);
  for(std::vector<Face>::iterator it = faces.begin();it != faces.end();it++) {
    validpts[it->first] = true;
    validpts[it->second] = true;
    validpts[it->third] = true;
  }
  int nbgood = 0;
  for(int i = 0;i < npts;i++)
    if (validpts[i]) nbgood++;
  std::cout << "** " << nbgood << " used points / " << npts << std::endl;
  return 0;
}
