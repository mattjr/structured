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
#include <time.h>
#include <qapplication.h>

#include "qviewer.h"

static double box_lgr(0.);
static int min_intersect = 2;  // le bit 0 du nb d'intersections est reservé 
// pour marquer les tetras incidents aux points vus, donc min_intersect doit etre multiple de 2
static unsigned long nb_test_facets = 0;
static int nb_tetra0(0);  // nb tetraedres lies aux cameras
static int nb_test0(0), nb_rayons(0);   // nb de rayons testes

void usage(char *prog) {
  std::cout << "Usage : " << prog << " file.cgal facets.ply [-b x1 y1 x2 y2]" << std::endl << std::endl;
  std::cout << "    View the results of PMVS and triangulation in an OpenGL window ." << std::endl;
  std::cout << std::endl << "\t -b : Only draw rays to points in the XY box." << std::endl;
  exit(1);
}

void draw_points(TPoint &points,PointColor &pcolors) {
    int i = 0;
    std::cout << "DRAW POINTS\n";
    for(TPoint::iterator it = points.begin();it != points.end();it++,i++) {
      int j = i;
    }
} 

int main(int argc,char **argv)
{
  QApplication app(argc, argv);
  app.setApplicationName("PCA demo");

#ifdef __USE_POSIX
  struct timespec tstart, tend;
#else
  time_t tstart, tend;
#endif
  if(argc < 3 || argc > 8) usage(argv[0]);

  int nbcams(0), nbpts(0);
  TPoint points;
  PointColor pcolors;
  std::vector<Point> normals;
  CGAL::Bbox_3 bb1, bb2;
  int *cams_index;
  std::vector<Face> faces;
  std::map<int, VisiblePatches*> image_patches;
  TPoint bad_cameras;
  float edge_mean, tetra_coefs[2];
  float xybox[4];
  CGAL::Bbox_3 bbox, limit_bbox;
  bool in_bbox = mygetopt("-b",OPT_FLOAT,3,argc,argv,xybox,4);
  if(in_bbox) {
    float xmin(xybox[0]), xmax(xybox[2]),
      ymin(xybox[1]), ymax(xybox[3]);
    if(xmin > xmax) {
      xmin = xmax;
      xmax = xybox[0];
    }
    if(ymin > ymax) {
      ymin = ymax;
      ymax = xybox[1];
    }
     
    limit_bbox = CGAL::Bbox_3(xmin,ymin,0,xmax,ymax,0);
  }
#ifdef __USE_POSIX
  clock_gettime(CLOCK_REALTIME,&tstart);
#else
  tstart = time(NULL);
#endif
  try {
    bb1 = read_ply(argv[2],points,normals,pcolors,faces);
    std::cout << "PLY BOX " << bb1 << std::endl;
    nbpts = points.size();
    read_cgal_xdata(argv[1],&nbcams,&cams_index,bb2,image_patches,bad_cameras,CG_PATCHES | CG_BADCAMS , &edge_mean, tetra_coefs);
    double dx = bb2.xmax() - bb2.xmin();
    double dy = bb2.ymax() - bb2.ymin();
    double dz = bb2.zmax() - bb2.zmin();
    Segment s = Segment(Point(bb2.xmin(), bb2.ymin(),bb2.zmin()),Point(bb2.xmax(), bb2.ymax(),bb2.zmax()));
    box_lgr = s.squared_length();
    std::cout << "CGAL BOX " << bb2 << std::endl;
    if(normals.size() < points.size()) normals.clear();
  }
  catch(char const *_e) {
    std::cout << "ERROR " << _e << std::endl;
    return 1;
    
  }
#ifdef __USE_POSIX
  clock_gettime(CLOCK_REALTIME,&tend);
#else
  tend = time(NULL);
#endif
  std::cout << "Read took " << delta_t(&tstart,&tend) << "s" << std::endl;
  std::cout << nbcams << " cameras, " << nbpts << " points, diag: " << box_lgr 
	    << std::endl;

  Viewer viewer(nbcams,cams_index,points,pcolors,faces,image_patches,bb1,in_bbox,limit_bbox);
  viewer.setWindowTitle(argv[2]);
  viewer.show();
  app.exec();
  return 0;
}
