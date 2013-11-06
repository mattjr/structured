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

#ifndef _DELAUNAY_H
#define _DELAUNAY_H

#include "triangdefs.h"
#include <time.h>

/** \file delaunay.h
 **/

/** 
   @brief Store statistics.
 **/

class TrStats {
 public:
 TrStats() : nb_tested_facets(0),nb_tot_intersect(0),nb_intv(0),nb_int_edge(0),
    nb_test_facets(0), nb_tetra0(0),
    nb_tested_lv0(0), nb_rayons(0) {}
  void sum(TrStats *stats2);
  unsigned long nb_tested_facets; /// nb of tested facets
  unsigned long nb_tot_intersect; /// nb of intersections found
  unsigned long nb_intv;  /// nb of intersections on vertex
  unsigned long nb_int_edge;  /// nb of intersections on edge
  unsigned long nb_test_facets; /// nb of facets candidate for ray intersection 
  int nb_tetra0; /// nb of tetraedrons with a camera vertex
  unsigned long nb_tested_lv0; /// nb of intersected facets of tetrahedrons with a camera as vertex
  unsigned long nb_rayons; /// nb of tested rays
};

/// params for facets extraction
class TrParams {
  #define CLEAN_LGR 1
  #define CLEAN_SURF 2
  #define CLEAN_TETRA 4
 public:
 TrParams() : do_clean(0), lgr_threshold(-1.),surf_threshold(-1.),
    cosinus_thresh(0.),moy_normal(false),smooth_coef(0.),smooth_nb_iter(1),
    nb_intersect(2), extract_type(XTR_DEFAULT), extract_mode(0)
      {}
  int do_clean;  /// OR of various cleaning methods 
  double lgr_threshold; /// inf limit for edge square lgr
  double surf_threshold; /// inf limit for facet surf
  double average_volume;  /// volume of a regular tetra with edges = average edge
  float cosinus_thresh;  /// cosine max value for normal angles
  bool moy_normal;   /// use average vertices normal instead of individual normals
  float smooth_coef;  /// coef for smothness
  int smooth_nb_iter;  /// nb of iteration in smoothing process
  int nb_intersect;  /// minimum nb of intersection
  ExtractType extract_type;  /// extraction type : maxflow, threshold, weigthed threshold
  int extract_mode; ///  surface facet extraction method : 0 = from removed cells, 1 = from kept cells 
                    /// +2 : all 4 facets of tetrahedrons (for tests)
};

#define CG_PATCHES 1
#define CG_BADCAMS 2

bool mygetopt(const char *opt,OP_TYPE op_type,int i,int argc, char **argv,void *res, int nbargs = 1);

// function to draw various element in debug mode
void draw_all(CGAL::Geomview_stream &gv,Delaunay &Tr);
void draw_seg(CGAL::Geomview_stream &gv,Segment &seg);
void prompt(std::string s);
void draw_tetra(Delaunay &Tr,std::vector<Cell_handle> &cells,CGAL::Geomview_stream &gv);
void draw_line_tetra(Delaunay &Tr,std::vector<Cell_handle> &cells,CGAL::Geomview_stream &gv);
void draw_facets(Delaunay &Tr,std::vector<Facet> facets,PointColor pcolors,CGAL::Geomview_stream &gv);
void draw_facets(Delaunay &Tr,std::vector<Facet> &facets,CGAL::Geomview_stream &gv);
void draw_cells_edges(Delaunay &Tr,std::vector<Cell_handle> &cells,CGAL::Geomview_stream &gv);
void draw_segs(CGAL::Geomview_stream &gv,std::vector<Segment> &segs);
//

void delaunay_extract(const char *outply,Delaunay &T,std::vector<Point> &normals,PointColor &pcolors,int nbcams,int *cams_index,TPoint &bad_cameras,TrParams &params,CGAL::Geomview_stream &gv,char *comment = NULL,bool flip_face=false);
CGAL::Bbox_3 read_ply(const char *filename,TPoint &points,std::vector<Point> &normals,PointColor &colors) throw(const char *) ;
CGAL::Bbox_3 read_ply(const char *filename,TPoint &points,std::vector<Point> &normals,PointColor &colors,std::vector<Face> &faces, char **coment = NULL) throw(const char *) ;

void read_patches(const char *filename,int firstpoint,int nbcams,TPoint &points, std::map<int, VisiblePatches *> &image_patches, bool read_points = false) throw(const char *);
void read_cgal_data(char *file,Delaunay &T,PointColor &pcolors,std::vector<Point> &normals,CGAL::Bbox_3 &bb,int *nbcams,int **cams_index,std::map<int, VisiblePatches*> &image_patches,TPoint &bad_cameras,int data_mode,float *edge_mean,float*tetra_coefs) throw(const char *);

void read_cgal_xdata(char *file,int *nbcams,int **cams_index,CGAL::Bbox_3 &bb,std::map<int, VisiblePatches*> &image_patches,TPoint &bad_cameras,int data_mode,float *edge_mean,float*tetra_coefs) throw(const char *);

std::vector<Facet> *intersect(int *err,int *nb_tested,Segment &seg,Intersect &in_inter,Intersect &out_inter,std::vector<Facet> *facets,Delaunay &Tr,std::vector<Cell_handle> &marked_cells,TrStats *stats,CGAL::Geomview_stream &gv) throw(const char *);

Vector *smooth(Delaunay &T,std::vector<Point> &normals,std::vector<Facet> &facets,float lambda,int nbiter,int nbcams,int *cams_index);

#ifdef __USE_POSIX
float delta_t(struct timespec *t1,struct timespec *t2);
#else
// no clock_gettime on MacOS
float delta_t(time_t *t1,time_t *t2);
#endif  // __USE_POSIX

float mean_edge(Delaunay &T);
int add_cells(Delaunay &T,float edge_min,float edge_max,TPoint &points,PointColor &pcolors,std::vector<Point> &normals);

extern bool gv_on;
extern int debug_stop, debug_vis;
extern int facet_vertex_order[];
extern char file_version[];
#endif // _DELAUNAY_H
