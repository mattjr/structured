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

#include "qviewer.h"
#include "triangdefs.h"

typedef qglviewer::Vec Vec;

static char *helpstr = "<h2>Display points or facets and camera rays</h2><br/><br/>\n\
<b>Esc</b> to exit.<br/>\n\
<b>+</b>/<b>-</b>: increment / decrement size of points<br/>\n\
<b>F</b>: switch between points and facets display.<br/>\n\
<b>Right</b> / <b>Left</b> arrow: increment / decrement num of current camera.<br/>\n\
<b>S</b>: toggle drawing of rays from current camera to visible points<br/>\n\
<b>C</b>: switch between noraml display and display of visible points for current camera.<br/>\n\
<b>T</b>: Toggle faces transparency.<br/>\n\
<b>Shift+MouseLeft</b> (selection) : print X Y Z coords of point under mouse<br>\n\
";
Viewer::Viewer(int nbcams,int *cam_index,TPoint &points,PointColor &pcolors,std::vector<Face> &faces,std::map<int, VisiblePatches*> &image_patches,CGAL::Bbox_3 bbox,bool in_bbox,CGAL::Bbox_3 limit_bbox)
  : m_draw_rays(RAY_NONE), m_icam(-1), m_point_size(2), m_nbcams(nbcams),m_points(points),m_pcolors(pcolors),
    m_faces(faces), m_image_patches (image_patches),m_bb(bbox), m_cam_index(cam_index), m_prev_cam(-1),
    m_in_bbox(in_bbox),m_limit_bbox(limit_bbox)
{
  m_cam_nums = new int[nbcams];
  int k = nbcams;
  for (int i = 0;i < nbcams;i++,k++) {
    int j = m_cam_index[k];
    m_cam_nums[i] = j;
  }
}

void Viewer::init()
{
  m_with_data = false;
  m_draw_rays = RAY_NONE;
  m_draw_faces = false;
  m_draw_points = true;
  m_transparency = false;
  //  setBackgroundColor(::Qt::black)
  //setBackgroundColor(::QColor(0,255,240));
  std::cout << "INIT\n";
  this->camera()->setSceneBoundingBox(Vec(m_bb.xmin(),m_bb.ymin(),m_bb.zmin()),Vec(m_bb.xmax(),m_bb.ymax(),m_bb.zmax()));
  this->showEntireScene();
  this->initlight();
  //  glLightfv(GL_LIGHT1, GL_SPECULAR,  light_specular);
  //glLightfv(GL_LIGHT1, GL_DIFFUSE,  light_diffuse);
  glEnable(GL_DEPTH_TEST);
  //glEnable(GL_CULL_FACE);
  set_cam(1,m_nbcams,0);
  setKeyDescription(Qt::Key_F,"Switch between points and facets display");
  setKeyDescription(Qt::Key_S,"Toggle drawing of rays from current camera to visible points");
  setKeyDescription(Qt::Key_C,"Switch between noraml display and display of visible points for current camera");
  setKeyDescription(Qt::Key_Plus,"Increment size of points");
  setKeyDescription(Qt::Key_Minus,"Decrement size of points");
  setKeyDescription(Qt::Key_Right,"Increment  num of current camera");
  setKeyDescription(Qt::Key_Left,"Decrement num of current camera");
  help();
}

void Viewer::drawPoints(bool withname) {
  if (! m_draw_points || (m_draw_rays == RAY_POINTS)) return;
  int listId = ::glGenLists(1);
  ::glNewList(listId,GL_COMPILE );
  ::glPointSize(m_point_size);
  ::glBegin(GL_POINTS);
  int i = 0;
  for(TPoint::iterator it = m_points.begin();it != m_points.end();it++,i++) {
    Point p = it->first;
    glColor3ub(m_pcolors[i].red(),m_pcolors[i].green(),m_pcolors[i].blue());
    glVertex3f(p.x(),p.y(),p.z());
  }
  ::glEnd();
  ::glEndList();
  ::glCallList(listId);
  ::glDeleteLists(listId,1);
}

void Viewer::draw()
{
  if(! m_with_data) return;
  //::glClearColor(1.0f,1.0f,1.0f,0.0f);
  ::glClearColor(0.,0.,0.,0.);
  int listId = ::glGenLists(1);
  drawPoints(false);
  if( m_draw_rays != RAY_NONE) {
    // draw segs
    ::glNewList(listId,GL_COMPILE );
    if (m_draw_rays == RAY_SEGS) {
      ::glLineWidth(1);
      glColor3ub(255,0,0);
      ::glBegin(GL_LINES);
      std::vector<int>::iterator it = m_segs.begin();
      Point p1 = m_points[*it++].first;
      for(;it != m_segs.end();it++) {
	Point p2 = m_points[*it].first;
	glVertex3f(p1.x(),p1.y(),p1.z());
	glVertex3f(p2.x(),p2.y(),p2.z());
      }
      ::glEnd();
      ::glEndList();
      ::glCallList(listId);
    } else {
      ::glPointSize(m_point_size);
      ::glBegin(GL_POINTS);
      for(std::vector<int>::iterator it = m_segs.begin();it != m_segs.end();it++) {
	int i = *it;
	Point p = m_points[i].first;
	glColor3ub(m_pcolors[i].red(),m_pcolors[i].green(),m_pcolors[i].blue());
	glVertex3f(p.x(),p.y(),p.z());
      }
      ::glEnd();
      ::glEndList();
      ::glCallList(listId);

    }
    ::glDeleteLists(listId,1);
  }
  if (m_draw_faces) {
    ::glNewList(listId,GL_COMPILE );
    ::glPointSize(2);
    ::glBegin(GL_TRIANGLES);
    for(std::vector<Face>::iterator it = m_faces.begin();it != m_faces.end();it++) {
      int pts[] = {it->first,it->second,it->third};
      for(int i = 0;i < 3;i++) {
	int j = pts[i];
	Point p = m_points[j].first;
	glColor4ub(m_pcolors[j].red(),m_pcolors[j].green(),m_pcolors[j].blue(),150);
	glVertex3f(p.x(),p.y(),p.z());
      }
    }
    ::glEnd();
    ::glEndList();
    ::glCallList(listId);
    ::glDeleteLists(listId,1);
  }

  // title
  ::glColor3ub(255,255,255);
  char buf[80];
  snprintf(buf,80,"Camera %d",m_cam_nums[m_icam]);
  drawText(20, 20,buf);

}

void Viewer::draw_cam()
{
  int numcam = m_cam_nums[m_icam];
  std::map<int, VisiblePatches*>::iterator it = m_image_patches.find(m_icam);
  if (it == m_image_patches.end()) return;
  VisiblePatches *pvpatches;
  pvpatches = it->second;
  std::cout << "START CAM " << numcam << " ( " << m_icam << "), " << pvpatches->size() << " PTS" << std::endl;
  if(pvpatches->size() == 0) return;
  m_segs.clear();
  Point pt1 = m_points[m_cam_index[m_icam]].first;
  m_segs.push_back(m_cam_index[m_icam]);
  for(VisiblePatches::iterator it = pvpatches->begin();it != pvpatches->end();it++) {

    if(m_in_bbox) {
      Point p = m_points[*it].first;
      if(p.x() < m_limit_bbox.xmin() || p.x() > m_limit_bbox.xmax()) continue;
      if(p.y() < m_limit_bbox.ymin() || p.y() > m_limit_bbox.ymax()) continue;
    }
    m_segs.push_back(*it);
  }
  m_with_data = true;
}
void Viewer::initlight()
{
  glDisable(GL_LIGHTING);
  //  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
  // enable semi-transparent culling planes
  ::glDisable(GL_BLEND);
  ::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  return;
  //glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,1);
  glEnable(GL_LIGHT1);
  //  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0);
  //GLfloat specular_color[4] = { 0,0,0, 1.0 };
  //glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,  specular_color);
  // Set Smooth Shading
  ::glShadeModel(GL_SMOOTH);

  // depth buffer setup 
  ::glClearDepth(1.0f);
  ::glEnable(GL_DEPTH_TEST);
  ::glDepthFunc(GL_LEQUAL);
  //  ::glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);


  // anti-aliasing, i.e. reduce jaggedness (if the OpenGL driver permits that)
  ::glEnable(GL_POINT_SMOOTH);
  ::glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
  ::glEnable(GL_LINE_SMOOTH);
  ::glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

  //return;
  float light_ambient[] = {0.5,0.5,0.5,1};
  float light_specular[] = {1.0, 1.0, 1.0, 1.0};
  float light_diffuse[] = {3.0, 3.0, 1.0, 1.0};
  float light_pos[] = {0.,0.,10.,0.};
  float light_pos2[] = {0.,0.,-10.,0.};
  //  glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 0.5);
  //glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, 1.);
  glLightfv(GL_LIGHT1, GL_POSITION, light_pos);
  glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
  glEnable(GL_LIGHT2);
  glLightfv(GL_LIGHT2, GL_POSITION, light_pos2);
}

void Viewer::set_cam(int incr, int ilimit,int istart) {
  if(m_prev_cam >= 0) {
    m_pcolors[m_prev_cam] = m_prev_color;
  }
  while (1) {
    m_icam += incr;
    if (m_icam == ilimit) m_icam = istart;
    int numcam = m_cam_nums[m_icam];
    std::map<int, VisiblePatches*>::iterator it = m_image_patches.find(m_icam);
    if (it == m_image_patches.end()) continue;
    break;
  }
  m_prev_cam = m_cam_index[m_icam];
  m_prev_color = m_pcolors[m_prev_cam];
  m_pcolors[m_prev_cam] = CGAL::Color(255,255,0);
  draw_cam();
}

void Viewer::keyPressEvent(QKeyEvent *e)
{
  // Get event modifiers key
  if (e->key()==Qt::Key_Plus) {
    m_point_size++;
    updateGL();
    return;
  }
  if (e->key()==Qt::Key_Minus) {
    m_point_size--;
    if (m_point_size < 1) m_point_size = 1;
    updateGL();
    return;
  }
  if ((e->key()==Qt::Key_Right) || (e->key()==Qt::Key_Left)) {
    int incr(1), ilimit(m_nbcams), istart(0);
    
    if (e->key()==Qt::Key_Left) {
      incr = -1;
      ilimit = -1;
      istart = m_nbcams - 1;
    }
    set_cam(incr,ilimit,istart);
    updateGL();
    return;
  }
  if (e->key()==Qt::Key_F) {
    m_draw_faces = !m_draw_faces;
    m_draw_points = !m_draw_faces;
    updateGL();
    return;
  }
  if (e->key()==Qt::Key_T) {
    m_transparency = !m_transparency;
    if (m_transparency)
      ::glEnable(GL_BLEND);
    else
      ::glDisable(GL_BLEND);
    updateGL();
    return;
  }
  if (e->key()==Qt::Key_S) {
    if(m_draw_rays == RAY_SEGS)
      m_draw_rays = RAY_NONE;
    else 
      m_draw_rays = RAY_SEGS;
    updateGL();
    return;
  }
  if (e->key()==Qt::Key_C) {
    if(m_draw_rays == RAY_POINTS)
      m_draw_rays = RAY_NONE;
    else 
      m_draw_rays = RAY_POINTS;
    updateGL();
    return;
  }
  QGLViewer::keyPressEvent(e);
}

void Viewer::postSelection(const QPoint &point) {
  bool found;
  qglviewer::Vec selpoint = camera()->pointUnderPixel(point, found);
  if(! found) return;
  // find nearest point
  float eps = 0.002;
  float x0, x1, y0, y1, z0, z1;
  x0 = selpoint.x - eps;x1 = selpoint.x + eps;
  y0 = selpoint.y - eps;y1 = selpoint.y + eps;
  z0 = selpoint.z - eps;z1 = selpoint.z + eps;
  std::cout << "XYZ= " << selpoint.x << " " << selpoint.y << " " << selpoint.z << std::endl;
  for(int i = 0;i < m_points.size();i++) {
    Point p = m_points[i].first;
    if (p.x() < x0 || p.x() > x1) continue;
    if (p.y() < y0 || p.y() > y1) continue;
    if (p.z() < z0 || p.z() > z1) continue;
    std::cout << "POINT " << i << " X Y Z " << p.x() << " " << p.y() << " " << p.z() << std::endl;
  }

}

QString Viewer::helpString() const
{
  QString text(helpstr);
  return text;
}
