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
#include <qapplication.h>
#include <QGLViewer/qglviewer.h>
#include <QtOpenGL/qgl.h>
//#include "qmap.h"
# include <QKeyEvent>
#include <iostream>
#include <cmath>
#include <vector>
//#include "triangdefs.h"
void read_data(char *file,CGAL::Bbox_3 &bb,TPoint &points,std::vector<Point> &normals,PointColor &colors,std::vector<Face> &faces)  throw(const char *);
void read_data_in_box(char *file,CGAL::Bbox_3 &cbbox,CGAL::Bbox_3 &bb,TPoint &points,std::vector<Point> &normals,PointColor &colors,std::vector<Face> &faces)  throw(const char *);

class Viewer : public QGLViewer {

public:
  Viewer(char *name1,char *name2,bool in_bbox,float *xybox,TPoint &points1,PointColor &pcolors1,std::vector<Face> &faces1,TPoint &points2,PointColor &pcolors2,std::vector<Face> &faces2,CGAL::Bbox_3 bbox);
  // overload several QGLViewer virtual functions
  ~Viewer() {}
  void draw();
  void switch_data();
  void init();
  virtual void keyPressEvent(QKeyEvent *e);
  //  void initializeGL();
  virtual QString helpString() const;
  void postSelection(const QPoint &point);
  int m_point_size;
  bool m_data1;
private:
  bool m_draw_faces;
  char *m_names[2];
  bool m_in_bbox;
  float *m_xybox;
  TPoint m_points, m_points1, m_points2;
  PointColor m_pcolors, m_pcolors1, m_pcolors2;
  std::vector<Face> m_faces, m_faces1, m_faces2;
  CGAL::Bbox_3 m_bb;
}; // end class Viewer

