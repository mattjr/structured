//
// structured - Tools for the Generation and Visualization of Large-scale
// Three-dimensional Reconstructions from Image Data. This software includes
// source code from other projects, which is subject to different licensing,
// see COPYING for details. If this project is used for research see COPYING
// for making the appropriate citations.
// Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
//
// This file is part of structured.
//
// structured is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// structured is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with structured.  If not, see <http://www.gnu.org/licenses/>.
//
#ifndef MESH_PROC_HPP
#define MESH_PROC_HPP

#include <vector>
#include <glib.h>
#include <gts.h>

#include "TriMesh.h"
#include <osg/Vec3>
#include <cv.h>
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define FACE_TRIANGLE   1
#define FACE_STRIP      2
#define FACE_FAN        3
#define NUM_TEX_BLEND_COORDS 4
typedef struct _T_Face         T_Face;
typedef struct _T_FaceClass    T_FaceClass;

struct _T_Face {
   GtsFace f;
   /* add extra data here */
   //size_t type;
  int material;

  int materialB[NUM_TEX_BLEND_COORDS];
  /*size_t iTerrain;
   size_t object;
   size_t iObject;*/
  //double normal[3];
};

struct _T_FaceClass {
   GtsFaceClass parent_class;
   /* add extra methods here */
};

#define T_FACE(obj)            GTS_OBJECT_CAST (obj,\
					           T_Face,\
					           t_face_class ())
#define T_FACE_CLASS(klass)    GTS_OBJECT_CLASS_CAST (klass,\
						         T_FaceClass,\
						         t_face_class())
#define IS_T_FACE(obj)         (gts_object_is_from_class (obj,\
						   t_face_class ()))
     
T_FaceClass * t_face_class(void);
  T_Face *      t_face_new(T_FaceClass * klass, GtsEdge * e1, GtsEdge * e2, GtsEdge * e3, size_t m=-1);
  //void             T_face_compute_normal( T_Face * f );

typedef struct _TVertex         TVertex;

struct _TVertex {
  /*< private >*/
  GtsVertex parent;
  gfloat u,v;
  gint id;
  int plane;
  gint idB[NUM_TEX_BLEND_COORDS];
  gfloat uB[NUM_TEX_BLEND_COORDS],vB[NUM_TEX_BLEND_COORDS];
  gfloat r,g,b;
  /*  gfloat confidence;
  gfloat r,g,b;
  gfloat ex,ey,ez;*/
  /*< public >*/
  /* add extra data here (if public) */
};


#define T_VERTEX(obj)            GTS_OBJECT_CAST (obj,\
					         TVertex,\
					         t_vertex_class ())
#define T_VERTEX_CLASS(klass)    GTS_OBJECT_CLASS_CAST (klass,\
						 TVertexClass,\
						 t_vertex_class())
#define IS_T_VERTEX(obj)         (gts_object_is_from_class (obj,\
						 t_vertex_class ()))

GtsVertexClass * t_vertex_class  (void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
namespace mesh_proc
{
  void edge_len_thresh(TriMesh *mesh,double thresh);
  void edge_len_thresh_percent(TriMesh *mesh,double thresh);
  extern guint nmax,nold;
  extern GTimer * timer, *total_timer;	       
  \
  gboolean tex_add_verbose ( guint number, guint total, int reject);
  void jet_color_map(const float& val,float &r,float &g, float &b);
  void clean_surf_pts(GtsSurface *surface,double edgemax);
  gboolean stop_number_verbose (gdouble cost, guint number, guint * min);
  void fill_gts_matrix(double *cam_pose,GtsMatrix *&trans);
  T_Face *copy_face (T_Face * t, GtsSurface * s);
  void cc_size_filter(TriMesh *in,int size);
  void remove_duplicate_vertices(TriMesh *mesh,double eps=1e-5);
bool  find_closest_pt_idx( TriMesh *mesh, const point &p,
			  int &ind,double _maxdist);
TriMesh* get_dense_grid(IplImage *disp,std::vector<osg::Vec3> points);
void auv_write_ply(GtsSurface *s,FILE *f, bool write_conf=false,
            const char *comment=NULL,bool bin=true);
GtsSurface *auv_mesh_pts(GPtrArray *vertices,double zedgemax=DBL_MAX,int coarsenNumber=0);

//!Clamp a value to between low and hi return it
//!
//! \param x value to be clamped
//! \param low min value
//! \param hi max value
//!
  float clamp(float x, float low,float hi);

}

#endif // !MESH_PROC_HPP
