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

#include "cmpcgal.h"

// compare 2 sets of points
void usage(char *prog) {
  std::cout << "Usage : " << prog << " {points1.ply points2.ply | data1.cgal data2.cgal} [-b x1 y1 x2 y2" << std::endl;
  std::cout << "    Alternatively display 2 datasets for comparison," <<
    std::endl << "    either facets (.ply) or delaunay (.cgal) files." << std::endl;
  std::cout << "\n\t-b : display points inside xy box" << std::endl;
  exit(1);
}
typedef qglviewer::Vec Vec;

static char *helpstr = "<h2>Alternatively display two triangulations.</h2><br/><br/>\n\
<b>Esc</b>:  EXIT.<br/>\n\
<b>+</b>/<b>-</b>: increment / decrement size of points<br/>\n\
<b>F</b>: switch between points and facets display.<br/>\n\
<b>F</b>: re-read data files.<br/>\n\
<b>Right_arrow</b>: switch to other data set.<br/>\n\
<b>Shift+MouseLeft</b> (selection) : print X,Y coords of point under mouse<br>\n\
";

CGAL::Bbox_3 get_data(char *name1,char *name2,bool in_bbox,float *xybox,TPoint &points1,TPoint &points2,PointColor &pcolors1,PointColor &pcolors2,std::vector<Face> &faces1,std::vector<Face> &faces2) {
  CGAL::Bbox_3 bbox, limit_bbox;
  float dx, dy, dz;
  std::vector<Point> normals1, normals2;
  points1.clear();
  points2.clear();
  faces1.clear();
  faces2.clear();
  pcolors1.clear();
  pcolors2.clear();
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
  bool one_file = (strcmp(name1,name2) == 0) ? true : false;
  if(in_bbox) {
    read_data_in_box(name1,limit_bbox,bbox,points1,normals1,pcolors1,faces1);
    if (! one_file) read_data_in_box(name2,limit_bbox,bbox,points2,normals2,pcolors2,faces2);
  } else {
    read_data(name1,bbox,points1,normals1,pcolors1,faces1);
    if (! one_file) read_data(name2,bbox,points2,normals2,pcolors2,faces2);
  }
  dx = (bbox.xmax() - bbox.xmin()) / 2.;
  dy = (bbox.ymax() - bbox.ymin()) / 2.;
  dz = (bbox.zmax() - bbox.zmin()) / 2.;
#if 0
  for (TPoint::iterator it = points1.begin();it != points1.end();it++) {
    Point p = it->first;
    Point p2 = Point(p.x() - dx,p.y() - dy,p.z() -dz);
    it->first = p2;
  }
  if (! one_file) {
    for (TPoint::iterator it = points2.begin();it != points2.end();it++) {
      Point p = it->first;
      Point p2 = Point(p.x() - dx,p.y() - dy,p.z() -dz);
      it->first = p2;
    }
  }
#endif  
  CGAL::Bbox_3 bb = CGAL::Bbox_3(bbox.xmin() - dx,bbox.ymin() - dy,bbox.zmin() - dz,
				 bbox.xmax() - dx,bbox.ymax() - dy,bbox.zmax() - dz);
  std::cout << "BOX " << bbox << std::endl;
  std::cout << "DX " << bbox.xmax() - bbox.xmin() << ", DY " << bbox.ymax() - bbox.ymin() << ", DZ " << bbox.zmax() - bbox.zmin() << std::endl;
  return bbox;
}

Viewer::Viewer(char *name1,char *name2,bool in_bbox,float *xybox,TPoint &points1,PointColor &pcolors1,std::vector<Face> &faces1,TPoint &points2,PointColor &pcolors2,std::vector<Face> &faces2,CGAL::Bbox_3 bbox)
  : m_point_size(2), m_points1(points1),m_pcolors1(pcolors1), m_faces1(faces1),
    m_points2(points2),m_pcolors2(pcolors2), m_faces2(faces2), m_bb(bbox),
    m_in_bbox(in_bbox),m_xybox(xybox)
{
  m_data1 = true;
  m_draw_faces = false;
  m_points = m_points1;
  m_pcolors = m_pcolors1;
  m_faces = m_faces1;
  m_names[0] = name1;
  m_names[1] = name2;
}

void Viewer::init()
{
  m_draw_faces = false;
  this->camera()->setSceneBoundingBox(Vec(m_bb.xmin(),m_bb.ymin(),m_bb.zmin()),Vec(m_bb.xmax(),m_bb.ymax(),m_bb.zmax()));
  this->showEntireScene();
  glDisable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
  //glEnable(GL_CULL_FACE);
  setWindowTitle(m_names[0]);
  setKeyDescription(Qt::Key_F,"Switch between points and facets display");
  setKeyDescription(Qt::Key_R,"Re-read data files");
  setKeyDescription(Qt::Key_Plus,"Increment size of points");
  setKeyDescription(Qt::Key_Minus,"Decrement size of points");
  setKeyDescription(Qt::Key_Right,"Switch to other data file");
  help();
}
void Viewer::draw() {
  ::glClearColor(0.,0.,0.,0.);
  //  setBackgroundColor(::QColor(0,200,200));
  int listId = ::glGenLists(1);
  if(m_draw_faces && m_faces.size() == 0) m_draw_faces = false;
  if (m_draw_faces) {
    ::glNewList(listId,GL_COMPILE );
    ::glPointSize(2);
    ::glBegin(GL_TRIANGLES);
    for(std::vector<Face>::iterator it = m_faces.begin();it != m_faces.end();it++) {
      int pts[] = {it->first,it->second,it->third};
      for(int i = 0;i < 3;i++) {
	int j = pts[i];
	Point p = m_points[j].first;
	glColor3ub(m_pcolors[j].red(),m_pcolors[j].green(),m_pcolors[j].blue());
	glVertex3f(p.x(),p.y(),p.z());
      }
    }
    ::glEnd();
    ::glEndList();
    ::glCallList(listId);
    ::glDeleteLists(listId,1);
  } else {
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

}

void Viewer::switch_data() {
  if (m_data1) {
    m_points = m_points1;
    m_pcolors = m_pcolors1;
    m_faces = m_faces1;
    setWindowTitle(m_names[0]);
  } else {
    m_points = m_points2;
    m_pcolors = m_pcolors2;
    m_faces = m_faces2;
    setWindowTitle(m_names[1]);
  }
  if (m_faces.size() == 0) m_draw_faces = false;
}

void Viewer::keyPressEvent(QKeyEvent *e)
{
  // Get event modifiers key
  if (e->key()==Qt::Key_R) {
    m_bb = get_data(m_names[0],m_names[1],m_in_bbox,m_xybox,m_points1,m_points2,m_pcolors1,m_pcolors2,m_faces1,m_faces2);
    switch_data();
    updateGL();
    return;
  }
  if (e->key()==Qt::Key_F) {
    m_draw_faces = ! m_draw_faces;
    updateGL();
    return;
  }
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
  if (e->key()==Qt::Key_Right) {
    if(m_points2.size() == 0) return;
    m_data1 = !m_data1;
    switch_data();
    updateGL();
    return;
  }
  QGLViewer::keyPressEvent(e);
}
void Viewer::postSelection(const QPoint &point) {
  bool found;
  qglviewer::Vec selpoint = camera()->pointUnderPixel(point, found);
  if(found)
    std::cout << "XY= " << selpoint.x << " " << selpoint.y << std::endl;

}

QString Viewer::helpString() const
{
  QString text(helpstr);
  return text;
}

void read_data(char *file,CGAL::Bbox_3 &bb,TPoint &points,std::vector<Point> &normals,PointColor &colors,std::vector<Face> &faces)  throw(const char *){
  int n = strlen(file);
  float edge_mean, tetra_coefs[2];
  if (n <= 5) 
    throw("filename length too short");
  bool ply = false;
  char *pc = file + n - 4;
  char *comment = NULL;
  if(strcmp(pc,".ply") == 0) ply = true;
  if(ply) {
    bb = read_ply(file,points,normals,colors,faces,&comment);
    if(comment != NULL)
      printf("%s : %s\n",file,comment);
  } else {
    Delaunay T;
    int nbcams;
    int *cams_index;
    std::map<int, VisiblePatches*> image_patches;
    TPoint bad_cameras;
    read_cgal_data(file,T,colors,normals,bb,&nbcams,&cams_index,image_patches,bad_cameras,0,&edge_mean,tetra_coefs);
    for(Delaunay::Finite_vertices_iterator itv = T.finite_vertices_begin(); itv != T.finite_vertices_end(); itv++) {
      std::pair<Point,int> p = std::make_pair(itv->point(),itv->info());
      points.push_back(p);
    }

  }
}
void read_data_in_box(char *file,CGAL::Bbox_3 &cbbox,CGAL::Bbox_3 &bb,TPoint &points,std::vector<Point> &normals,PointColor &colors,std::vector<Face> &faces)  throw(const char *){
  TPoint points1;
  PointColor pcolors1;
  std::vector<Point> normals1;
  std::vector<Face> faces1;
  read_data(file,bb,points1,normals1,pcolors1,faces1);
  int j = 0;
  for(int i = 0;i < points1.size();i++) {
    Point p = points1[i].first;
    points1[i].second = -1;
    if(p.x() < cbbox.xmin() || p.x() > cbbox.xmax()) continue;
    if(p.y() < cbbox.ymin() || p.y() > cbbox.ymax()) continue;
    points.push_back(points1[i]);
    colors.push_back(pcolors1[i]);
    normals.push_back(normals1[i]);
    points1[i].second = j++;
  }
  if(faces1.size() == 0) return;
  for(std::vector<Face>::iterator it = faces1.begin();it != faces1.end();it++) {
    Face f = *it;
    if((points1[f.first].second < 0) || (points1[f.second].second < 0) || (points1[f.third].second < 0)) continue;
    faces.push_back(Face(points1[f.first].second,points1[f.second].second,points1[f.third].second));
  }
}
int main(int argc,char **argv)
{
  QApplication app(argc, argv);
  app.setApplicationName("PLY CMP");
  if(argc < 3 || argc > 8) usage(argv[0]);
  TPoint points1, points2;
  PointColor pcolors1, pcolors2;
  std::vector<Point> normals1, normals2;
  std::vector<Face> faces1, faces2;
  CGAL::Bbox_3 bbox, limit_bbox;
  float dx, dy, dz;
  float xybox[4];
  bool in_bbox = mygetopt("-b",OPT_FLOAT,3,argc,argv,xybox,4);
  CGAL::Bbox_3 bb;
  try {
    bb = get_data(argv[1],argv[2],in_bbox,xybox,points1,points2,pcolors1,pcolors2,faces1,faces2);
  }
  
  catch(char const *_e) {
    std::cout << "ERROR " << _e << std::endl;
    return 1;
  }

  Viewer viewer(argv[1],argv[2], in_bbox,xybox,points1,pcolors1,faces1,points2,pcolors2,faces2,bb);
  viewer.setWindowTitle("simpleViewer");
  viewer.show();
  app.exec();
  return 0;
}
