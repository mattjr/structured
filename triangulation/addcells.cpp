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

/** \file addcells.cpp
    \brief Add the barycenter of "large" tetrahedrons to the triangulation.
    
 **/

/**
   \brief Compute the average edge length in a delaunay triangulation.
 **/
float mean_edge(Delaunay &T) {
  double stot = 0.;
  int n = 0;
  for(Delaunay::Finite_edges_iterator it = T.finite_edges_begin();it != T.finite_edges_end();it++,n++) {
    Cell_handle c = it->first;
    int i = it->second;
    int j = it->third;
    Segment s(c->vertex(i)->point(),c->vertex(j)->point());
    stot += s.squared_length();
  }
  return (float)(stot / n);
}

/**
   \brief Add the barycenter of "large" tetrahedrons to the set of points and update the triangulation.
   \param[in,out] T : the delaunay data
   \param edge_min : consider tetrahedrons that have at least one edge longer than \b edge_min
   \param edge_max : but ignore tetrahedrons that have at least one edge longer than \b edge_max
   \param[in,out] points : the vector of points. New points will be appended.
   \param[in,out] pcolors : the vector of colors. Colors of new points will be estimated and appended.
   \param[in,out] normals : the vector of normals. Normals of new points will be estimated and appended.
 **/
int add_cells(Delaunay &T,float edge_min,float edge_max,TPoint &points,PointColor &pcolors,std::vector<Point> &normals) {
  int nbadd = 0;
  // reset cells info
  for (Delaunay::Finite_cells_iterator cit = T.finite_cells_begin(); cit != T.finite_cells_end(); ++cit) {
    cit->info() = 0;
  }
  std::vector<Cell_handle> acells;
  for(Delaunay::Edge_iterator it = T.edges_begin();it != T.edges_end();it++) {
    Cell_handle c = it->first;
    int i = it->second;
    int j = it->third;
    Segment s(c->vertex(i)->point(),c->vertex(j)->point());
    float sl = s.squared_length();
    if ((sl >= edge_min) && (sl <= edge_max)) {
      Delaunay::Cell_circulator ifc0 = T. incident_cells(*it);
      Delaunay::Cell_circulator ifc = ifc0;
      do {
	c = ifc++;
	if (T.is_infinite(c) || (c->info() != 0)) continue;
	bool valid_size = true;
	for(int i = 0;i < 4;i++) {
	  for(int j = i + 1;j < 4;j++) {
	    Segment s = Segment(c->vertex(i)->point(),c->vertex(j)->point());
	    if (s.squared_length() > edge_max) {// this tetra is too big
	      valid_size = false;
	      break;
	    }
	  }
	}
	c->info() = 1;
	if (valid_size) {
	  nbadd++;
	  acells.push_back(c);
	}
      } while (ifc != ifc0);
    }
  }
  Point pn = Point(0,0,0);
  bool has_normals = (normals.size() == 0) ? false : true;
  int i0 = points.size();
  for(std::vector<Cell_handle>::iterator it = acells.begin();it != acells.end();it++) {
    int j;
    Cell_handle c = *it;
    Vector n = Vector(0.,0.,0.);
    float r(0.), g(0.), b(0.);
    CGAL::Color col;
    for(int i = 0;i < 4;i++) {
      j = c->vertex(i)->info();
      
      if (has_normals) pn = normals[j];
      Vector vn = Vector(pn.x(),pn.y(),pn.z());
      n = n + vn;
      col = pcolors[j];
      r += col.red();
      g += col.green();
      b += col.blue();
    }
    n = n / 4.;
    col = CGAL::Color((int)((r / 4.) + .5),(int)((g / 4.) + .5),(int)((b / 4.) + .5));
    Tetrahedron t = T.tetrahedron(c);
    j = points.size();
    Point p = CGAL::centroid(t);
    points.push_back(std::make_pair(p,j));
    if (has_normals) normals.push_back(Point(n[0],n[1],n[2]));
    pcolors.push_back(col);
  }
  TPoint::iterator itp = points.begin() + i0;
  T.insert(itp,points.end());
  return nbadd;
}

