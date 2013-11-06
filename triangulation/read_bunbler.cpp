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
#include <stdlib.h>
/** \file read_bundler.cpp
    @brief functions to read data from bundler files
 **/

CGAL::Bbox_3 read_bundler(const char *filename,TPoint &points,PointColor &colors,int nbcams0,std::map<int, VisiblePatches *> &image_patches) throw(const char *){
  std::ifstream ifstr;
  ifstr.open(filename);
  if(! ifstr.good())
    throw("read_bundler: cannot open file");
  char tmp[256];
  ifstr.getline(tmp, 256);
  if(strncmp("# Bundle",tmp,8) != 0)
    throw("read_bundler: not a bundler file");
  int nbcams, nbpts;
  ifstr >> nbcams >> nbpts;
  ifstr.getline(tmp, 256);  // rm the previous newline
  // skip cameras
  if (nbcams != nbcams0)
    throw("read_bundler: nb of bundler cameras != nb of camera points\n");
  for(int i =0;i < 5 * nbcams;i++) {
    if (ifstr.eof()) throw("read_bundler: unexpected EOF\n");
    ifstr.getline(tmp, 256);
  }
  // read points
  float xmin(1.e20), ymin(1.e20), zmin(1.e20), xmax(-1.e20), ymax(-1.e20), zmax(-1.e20);
  for(int i =0;i < nbpts;i++) {
    float x, y, z;
    int r, g, b;
    int nbview;
    if (ifstr.eof()) throw("read_bundler: unexpected EOF\n");
    ifstr >> x >> y >> z;
    ifstr >> r >> g >> b;
    colors.push_back(CGAL::Color(r,g,b));
    if(x < xmin) xmin = x;
    if(y < ymin) ymin = y;
    if(z < zmin) zmin = z;
    if(x > xmax) xmax = x;
    if(y > ymax) ymax = y;
    if(z > zmax) zmax = z;
    points.push_back( std::make_pair(Point(x,y,z),i));
    ifstr >> nbview;
    int icam, key;
    for (int j = 0;j < nbview;j++) {
      float xx;
      ifstr >> icam >> key >> xx >> xx;
      if (icam >= nbcams) 
	throw("read_bundler: bad cam number");
      if(image_patches.count(icam) == 0)
	image_patches[icam] = new VisiblePatches();
      image_patches[icam]->push_back(i);
    }
  }
  ifstr.close();
  return CGAL::Bbox_3(xmin,ymin,zmin,xmax,ymax,zmax);
}
