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
//#include <CGAL/Qt/resources.h>

#include "qviewer.h"

static double box_lgr(0.);
static int min_intersect = 2;  // le bit 0 du nb d'intersections est reservé 
// pour marquer les tetras incidents aux points vus, donc min_intersect doit etre multiple de 2
static unsigned long nb_test_facets = 0;
static int nb_tetra0(0);  // nb tetraedres lies aux cameras
static int nb_test0(0), nb_rayons(0);   // nb de rayons testes

void usage(char *prog) {
  std::cout << "Usage : " << prog << " cameras.ply <model-basename>[-b x1 y1 x2 y2] " << std::endl;
  std::cout << "    Display PMVS points and cameras, and draw rays to visible points." << 
    std::endl << "    <model-basename> is the basename of files .ply and .patch built by PMVS2." << std::endl;
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
  std::map<int, VisiblePatches*> image_patches;
  std::vector<Face> faces;
  CGAL::Bbox_3 bb, limit_bbox;
  float xybox[4];
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
    read_ply(argv[1],points,normals,pcolors);
    nbcams = points.size();
    if(normals.size() == 0)
      normals.resize(nbcams,Point(1.,1.,1.));
    char tmp[512];
    snprintf(tmp,512,"%s.ply",argv[2]);
    bool no_points = false;
    std::ifstream ifstr;
    ifstr.open(tmp);
    if(! ifstr.good())
      no_points = true;
    else {
      ifstr.close();
      std::cout << "READ PLY\n";
      bb = read_ply(tmp,points,normals,pcolors);
      double dx = bb.xmax() - bb.xmin();
      double dy = bb.ymax() - bb.ymin();
      double dz = bb.zmax() - bb.zmin();
      Segment s = Segment(Point(bb.xmin(), bb.ymin(),bb.zmin()),Point(bb.xmax(), bb.ymax(),bb.zmax()));
      box_lgr = s.squared_length();
      if(normals.size() < points.size()) normals.clear();
    }
    snprintf(tmp,512,"%s.patch",argv[2]);
    read_patches(tmp,nbcams,nbcams,points,image_patches,no_points);
    nbpts = points.size() - nbcams;
    if(no_points) {
      normals.clear();
      pcolors.resize(points.size(),CGAL::Color(128,128,128));
    }
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
  std::cout << "MAP " << image_patches.size() << std::endl;
  // marquage des cameras utiles(V) et inutiles (R)
  for(int icam = 0;icam < nbcams;icam++) {
    std::map<int, VisiblePatches*>::iterator it = image_patches.find(icam);
    if (it != image_patches.end()) 
      pcolors[icam] = CGAL::Color(0,255,0);
    else
      pcolors[icam] = CGAL::Color(255,0,0);
  }
  int cam_index[2 * nbcams];
  for(int i = 0;i<nbcams;i++) {
    cam_index[i] = i;
    cam_index[i + nbcams] = i;
  }  
  Viewer viewer(nbcams,cam_index,points,pcolors,faces,image_patches,bb,in_bbox,limit_bbox);
  viewer.setWindowTitle("simpleViewer");
  viewer.show();
  app.exec();
  return 0;
}
