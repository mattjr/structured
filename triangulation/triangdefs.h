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

#ifndef _TRIANGDEFS_H
#define _TRIANGDEFS_H

#include <CGAL/basic.h>
#include <CGAL/Object.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Triangulation_cell_base_with_info_3.h>
#include <CGAL/intersections.h>
#include <CGAL/circulator.h>

#include <CGAL/IO/Triangulation_geomview_ostream_3.h>
#include <iostream>
#include <fstream>
#include <iterator>
#include <unistd.h>

/** \mainpage pmvs-triangulation
 * There are essentially  two main programs : delaunay and triangclean.
 *
 * They are based on the CGAL library (Computational Geometry Algorithms Library, http://www.cgal.org)
    \section d_sec delaunay
    it builds the delaunay triangulation and does the ray tracing.
    Depends on:
      delaunay.h triangdefs.h delaunay.cpp config.cpp delaunay_io.cpp addcells.cpp intersect.cpp gviewer.cpp 
    \section t_sec triangclean
    It analyzes ray tracing information and extracts surface facets.
    Depends on :
      delaunay.h triangdefs.h triangclean.cpp config.cpp extract.cpp smooth.cpp gviewer.cpp delaunay_io.cpp
 **/

/** \file triangdefs.h
 @brief Definitions of types.
 **/

#define DEF_LGR_COEF 0.14
#define DEF_SURF_COEF 0.01
#define MAX_INTERSECT 2 * 100000

typedef enum {OPT_INT,OPT_FLOAT,OPT_STRING,OPT_NOARGS} OP_TYPE;

typedef enum {XTR_DEFAULT, XTR_STD, XTR_STD_VOL, XTR_MAXFLOW} ExtractType;

typedef CGAL::Exact_predicates_inexact_constructions_kernel         K;
typedef CGAL::Triangulation_vertex_base_with_info_3<int, K>    Vb;
typedef K::Vector_3 Vector;

typedef CGAL::Triangulation_cell_base_with_info_3<int, K>    Cb;
typedef CGAL::Triangulation_data_structure_3<Vb,Cb>                    Tds;
//typedef CGAL::Delaunay_triangulation_3<K, Tds,CGAL::Fast_location>            Delaunay;
typedef CGAL::Delaunay_triangulation_3<K, Tds>            Delaunay;
typedef Delaunay::Point                                             Point;
typedef Delaunay::Edge                                             Edge;
typedef Delaunay::Cell_handle Cell_handle;
typedef Delaunay::Vertex_handle Vertex_handle;
typedef Delaunay::Locate_type Locate_type;
typedef Delaunay::Segment Segment;
typedef Delaunay::Triangle Triangle;
typedef Delaunay::Tetrahedron Tetrahedron;


typedef std::pair<Cell_handle,int> Facet;
typedef CGAL::Triple<int,int,int> Face;

typedef  std::vector<Facet>::iterator                  I;
typedef CGAL::Circulator_from_iterator< I >  Circulator;

typedef std::vector<int> VisiblePatches;
typedef std::vector<CGAL::Color> PointColor;
typedef std::vector<std::pair<Point,int> > TPoint;
typedef enum { INT_FACET, INT_EDGE, INT_VERTEX} InterType;

/// information about an intersectection between a ray and a facet
typedef struct {
  InterType int_type;  /// type of intersection (inside facet, on edge ...)
  Vertex_handle v1;  /// vertex for vertex intersection, 1st edge vertex for edge intersection
  Vertex_handle v2;  /// 2nd edge vertex for edge intersection
  int intersected_facet; /// index of the facet in the vector of candidate facets
} Intersect;

#endif // _TRIANGDEFS_H
