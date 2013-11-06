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

void read_data(const char *filename,std::vector< std::pair<Point,unsigned> > &points) throw(const char *){
  std::ifstream ifstr;
  ifstr.open(filename);
  if(! ifstr.good())
    throw("read_data: cannot open file");
 
  float x, y, z, nx, ny, nz;
  int i = 0;
  while(1) {
    ifstr >> x >> y >> z >> nx >>ny >>nz;
    points.push_back( std::make_pair(Point(x,y,z),i));
    i++;
    if (ifstr.eof())
      break;
  }
  ifstr.close();
}
void read_ply(const char *filename,std::vector< std::pair<Point,unsigned> > &points) throw(const char *){
  std::ifstream ifstr;
  ifstr.open(filename);
  if(! ifstr.good())
    throw("read_data: cannot open file");
  char tmp[256];
  std::string name, str1;
  float x, y, z, nx, ny, nz;
  int r,g,b;
  int nbpts = 0;
  int nbflt = 0, nbint = 0;
  ifstr >> name;
  if(name != "ply") throw("read_ply: bad ply header\n");
  ifstr >> name >> str1;
  if(name != "format" || str1 != "ascii") throw("read_ply: incorrect ply format\n");
  ifstr >> str1;
  while(1) { // read options
    ifstr >> name;
    if (name == "end_header") break;
    ifstr >> str1;
    if(name == "element") {
      int itmp;
      ifstr >> itmp;
      if(str1 == "vertex")
	nbpts = itmp;
      else
	if(itmp != 0) throw("read_ply: unexpected non nul faces nb\n");
    } else if(name == "property") {
      std::string str2;
      ifstr >> str2;
      if(str1 == "float") nbflt++;
      else if (str1 == "uchar") nbint++;
      else if(str1 == "list") {
	ifstr.getline(tmp, 256);
	continue;
      }
      else throw("read_ply: unexpected property type\n");
    } else {
      std::cout << "ERR " << name << std::endl;
      throw("read_ply: unexpected keyword\n");
    }
    if (ifstr.eof()) throw("read_ply: unexpected EOF\n");
  }
  if((nbflt != 3 && nbflt != 6) || (nbint != 0 && nbint != 3)) 
    throw("read_ply: unexpected data structure\n");
  int j = points.size();
  for(int i = 0;i < nbpts;i++) {
    if (ifstr.eof()) {
      std::cout << "I = " << i << std::endl;
      throw("read_ply: points reading unexpected EOF\n");
    }
    ifstr >> x >> y >> z;
    if(nbflt > 3)
      ifstr >> nx >>ny >>nz;
    if(nbint > 0)
      ifstr >> r >> g >> b;
    points.push_back( std::make_pair(Point(x,y,z),j++));
  }
  ifstr.close();
}

void usage(char *prog) {
  std::cout << "Usage : " << prog << " cameras.ply points.ply" << std::endl;
  exit(1);
}

// visu geomview de demo/Triangulation_3_Geomview_demos/Triangulation_3_demo.cpp
int main(int argc,char **argv)
{
  if(argc != 3) usage(argv[0]);
  std::vector< std::pair<Point,unsigned> > points;
  try {
    read_ply(argv[1],points);
    int nbcams = points.size();
    read_ply(argv[2],points);
    int nbpts = points.size() - nbcams;
    std::cout << nbcams << " cameras, " << nbpts << " points" << std::endl;
  }
  catch(char const *_e) {
    std::cout << "ERROR " << _e << std::endl;
    return 1;
    
  }
  
  Delaunay T( points.begin(),points.end() );

  //  CGAL_assertion( T.number_of_vertices() == 6 );

  // check that the info was correctly set.
  Delaunay::Finite_vertices_iterator vit;
  for (vit = T.finite_vertices_begin(); vit != T.finite_vertices_end(); ++vit)
    if( points[ vit->info() ].first != vit->point() ){
      std::cerr << "Error different info" << std::endl;
      exit(EXIT_FAILURE);
    }
  std::cout << "OK" << std::endl;
  std::cout << "N " << std::distance(T.finite_vertices_begin(),T.finite_vertices_end())<< std::endl;
#if 0
  std::ofstream oFileT("output",std::ios::out);
  oFileT << T;
  oFileT.close();
#endif
  CGAL::Geomview_stream gv(CGAL::Bbox_3(0,0,0, 2, 2, 2));
  gv.set_bg_color(CGAL::Color(0, 200, 200));
  gv.clear();
  gv.set_wired(false);
  gv << T;
  char ch;
  std::cin >> ch;
  return 0;
}
