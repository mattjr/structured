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

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>

#include <CGAL/IO/Triangulation_geomview_ostream_3.h>
#include <iostream>
#include <fstream>
#include <iterator>
#include <unistd.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel         K;
typedef CGAL::Triangulation_vertex_base_with_info_3<unsigned, K>    Vb;
typedef CGAL::Triangulation_data_structure_3<Vb>                    Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds>                      Delaunay;
typedef Delaunay::Point                                             Point;

// visu geomview de demo/Triangulation_3_Geomview_demos/Triangulation_3_demo.cpp
int main()
{
  std::vector< std::pair<Point,unsigned> > points;
  points.push_back( std::make_pair(Point(0,0,0),0) );
  points.push_back( std::make_pair(Point(1,0,0),1) );
  points.push_back( std::make_pair(Point(0,1,0),2) );
  points.push_back( std::make_pair(Point(0,0,1),3) );
  points.push_back( std::make_pair(Point(2,2,2),4) );
  points.push_back( std::make_pair(Point(-1,0,1),5) );

  
  Delaunay T( points.begin(),points.end() );

  CGAL_assertion( T.number_of_vertices() == 6 );

  // check that the info was correctly set.
  Delaunay::Finite_vertices_iterator vit;
  for (vit = T.finite_vertices_begin(); vit != T.finite_vertices_end(); ++vit)
    if( points[ vit->info() ].first != vit->point() ){
      std::cerr << "Error different info" << std::endl;
      exit(EXIT_FAILURE);
    }
  std::cout << "OK" << std::endl;
  std::cout << "N " << std::distance(T.finite_vertices_begin(),T.finite_vertices_end())<< std::endl;
  std::ofstream oFileT("output",std::ios::out);
  oFileT << T;
  oFileT.close();
  CGAL::Geomview_stream gv(CGAL::Bbox_3(0,0,0, 2, 2, 2));
  gv.set_bg_color(CGAL::Color(0, 200, 200));
  gv.clear();
  gv.set_wired(false);
  gv << T;
  char ch;
  std::cin >> ch;
  return 0;
}
