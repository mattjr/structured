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

#ifndef VIEWER_H
#define VIEWER_H

#include <qapplication.h>
#include <QGLViewer/qglviewer.h>
#include <QtOpenGL/qgl.h>
//#include "qmap.h"
# include <QKeyEvent>
#include <iostream>
#include <cmath>
#include "triangdefs.h"

// forward declarations

class Viewer : public QGLViewer {

  typedef enum {RAY_SEGS,RAY_POINTS,RAY_NONE} RAY_MODE;

public:
  Viewer(int nbcams,int *cam_index,TPoint &points,PointColor &pcolors,std::vector<Face> &faces,std::map<int, VisiblePatches*> &image_patches,CGAL::Bbox_3 bbox,bool in_bbox,CGAL::Bbox_3 limit_bbox);
  ~Viewer() { delete[] m_cam_nums;}
  // overload several QGLViewer virtual functions
  void draw();
  void init();
  virtual void keyPressEvent(QKeyEvent *e);
  //  void initializeGL();
  virtual QString helpString() const;
  void draw_cam();
  void postSelection(const QPoint &point);
  void initlight();
  int m_point_size;
private:
  void drawPoints(bool withname);
  void set_cam(int incr,int ilimit,int istart);
  bool m_with_data;
  bool m_draw_points, m_draw_faces;
  bool m_transparency;
  RAY_MODE m_draw_rays;
  TPoint m_points;
  PointColor m_pcolors;
  std::vector<int> m_segs;
  std::vector<Face> m_faces;
  int m_icam;
  int m_nbcams;
  int *m_cam_index; // indices of cams in points, followed by orig cam nums
  int *m_cam_nums; // orig cam nums
  int m_prev_cam;
  bool m_in_bbox;
  CGAL::Bbox_3 m_limit_bbox;
  CGAL::Color m_prev_color;
  CGAL::Bbox_3 m_bb;
  std::map<int, VisiblePatches*> m_image_patches;
  std::vector<int> m_icams;
  
}; // end class Viewer

#endif // VIEWER_H
