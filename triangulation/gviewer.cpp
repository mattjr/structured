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

void draw_all(CGAL::Geomview_stream &gv,Delaunay &Tr) {
  if(! gv_on) return;
  gv.clear();
  gv.set_line_width(2);
  gv.set_wired(true);
  gv << CGAL::BLACK;
#ifndef TESTS
  gv << Tr;
#endif
}
void draw_seg(CGAL::Geomview_stream &gv,Segment &seg) 
{
  if(! gv_on) return;
  gv << CGAL::RED;
  gv << seg;
}
void draw_segs(CGAL::Geomview_stream &gv,std::vector<Segment> &segs) {
  gv << CGAL::RED;
  for(std::vector<Segment>::iterator it = segs.begin();it != segs.end();it++) {
    gv << *it;
  }
}

void draw_tetra(Delaunay &Tr,std::vector<Cell_handle> &cells,CGAL::Geomview_stream &gv) {
  if(! gv_on) return;
  CGAL::Color colors[] = {CGAL::BLUE,CGAL::GREEN,CGAL::YELLOW,CGAL::DEEPBLUE,
			  CGAL::PURPLE,CGAL::VIOLET,CGAL::ORANGE,CGAL::RED};
  int k = 0;
  for (std::vector<Cell_handle>::iterator it = cells.begin();it != cells.end();it++,k++) {
    Cell_handle c = *it;
    gv << colors[k % 8];
    Tetrahedron t = Tr.tetrahedron(c);
    //    Tetrahedron t(c->vertex(0)->point(),c->vertex(1)->point(),
    //		  c->vertex(2)->point(),c->vertex(3)->point());
    gv << t;
  }
}
void draw_line_tetra(Delaunay &Tr,std::vector<Cell_handle> &cells,CGAL::Geomview_stream &gv) {
  if(! gv_on) return;
  CGAL::Color colors[] = {CGAL::BLUE,CGAL::GREEN,CGAL::YELLOW,CGAL::DEEPBLUE,
			  CGAL::PURPLE,CGAL::VIOLET,CGAL::ORANGE,CGAL::RED};
  int k = 0;
  for (std::vector<Cell_handle>::iterator it = cells.begin();it != cells.end();it++,k++) {
    Cell_handle c = *it;
    gv << colors[k % 8];
    Segment segs[4];
    for(int i = 0;i < 4;i++) {
      for(int j = i + 1;j < 4;j++) {
	Segment s = Segment(c->vertex(i)->point(),c->vertex(j)->point());
	gv << s;
      }
    }
  }
}
void draw_facets(Delaunay &Tr,std::vector<Facet> &facets,CGAL::Geomview_stream &gv) {
  if(! gv_on) return;
  CGAL::Color colors[] = {CGAL::BLUE,CGAL::GREEN,CGAL::YELLOW,CGAL::DEEPBLUE,
			  CGAL::PURPLE,CGAL::VIOLET,CGAL::ORANGE,CGAL::RED};
  int k = 0;
  for (std::vector<Facet>::iterator it = facets.begin();it != facets.end();it++,k++) {
    gv << colors[k % 8];
    Triangle t = Tr.triangle(*it);
    gv << t;
  }
}

void draw_facets(Delaunay &Tr,std::vector<Facet> &facets,PointColor pcolors,CGAL::Geomview_stream &gv) {
  if(! gv_on) return;
  CGAL::Color colors[] = {CGAL::BLUE,CGAL::GREEN,CGAL::YELLOW,CGAL::DEEPBLUE,
			  CGAL::PURPLE,CGAL::VIOLET,CGAL::ORANGE,CGAL::RED};
  if(pcolors.size() == 0) 
    return draw_facets(Tr,facets,gv);
  // draw with color interpolation
  for (std::vector<Facet>::iterator it = facets.begin();it != facets.end();it++) {
    Facet f = *it;
    Cell_handle c = f.first;
    int j = f.second;
    CGAL::Color vcolors[3];
    int k = 0;
    for(int i = 0;i < 4;i++) 
      if(i != j)
	vcolors[k++] = pcolors[c->vertex(i)->info()];
    int r = (vcolors[0].red() + vcolors[1].red() + vcolors[2].red()) / 3;
    int g = (vcolors[0].green() + vcolors[1].green() + vcolors[2].green()) / 3;
    int b = (vcolors[0].blue() + vcolors[1].blue() + vcolors[2].blue()) / 3;
    gv << CGAL::Color(r,g,b);
    //    std::cout << "RGB " << r << " " << g << " " << b <<std::endl;
    Triangle t = Tr.triangle(f);
    gv << t;
  }
}

void draw_cells_edges(Delaunay &Tr,std::vector<Cell_handle> &cells,CGAL::Geomview_stream &gv) {
  if(! gv_on) return;
  CGAL::Color colors[] = {CGAL::BLUE,CGAL::GREEN,CGAL::YELLOW};
  int k = 0;
  for (std::vector<Cell_handle>::iterator it = cells.begin();it != cells.end();it++,k++) {
    Cell_handle c = *it;
    gv << colors[k % 3];
    for(int i = 0;i<4;i++)
      for(int j = i+1;j < 4;j++) {
	Segment e = Tr.segment(c,i,j);
	gv << e;
      }
  }
}
