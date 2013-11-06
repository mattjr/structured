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
/** \file delaunay_io.cpp
    @brief functions to read data from ply or cgal files
 **/

/**
   Read the data part of a binary ply file
 **/
CGAL::Bbox_3 ply_binary_data(std::ifstream &ifstr,TPoint &points,std::vector<Point> &normals,PointColor &colors,std::vector<Face> &faces,int nbpts,int nbfaces,int nbflt, int nbint) throw(const char *){
  float x, y, z, nx, ny, nz;
  double xmin(1.e20), ymin(1.e20), zmin(1.e20), xmax(-1.e20), ymax(-1.e20), zmax(-1.e20);
  int r,g,b;
  float ftmp[6];
  int itmp[4];
  unsigned char colbuf[4];
  int j = points.size();
  // skip end of header line
  ifstr.getline((char *)ftmp, 4);
  for(int i = 0;i < nbpts;i++) {
    ifstr.read((char *)ftmp,nbflt * sizeof(float));
    x = ftmp[0];y = ftmp[1]; z = ftmp[2];
    if(x < xmin) xmin = x;
    if(y < ymin) ymin = y;
    if(z < zmin) zmin = z;
    if(x > xmax) xmax = x;
    if(y > ymax) ymax = y;
    if(z > zmax) zmax = z;
    points.push_back( std::make_pair(Point(x,y,z),j++));
    if(nbflt > 3)
      normals.push_back(Point(ftmp[3],ftmp[4],ftmp[5]));
    if(nbint > 0) {
      ifstr.read((char *)colbuf,nbint * sizeof(unsigned char));
      colors.push_back(CGAL::Color(colbuf[0],colbuf[1],colbuf[2]));
    }
  }
  if(nbfaces > 0) {
    for(int i = 0;i < nbfaces;i++) {
      ifstr.read((char *)colbuf,sizeof(char));
      ifstr.read((char *)itmp,3 * sizeof(int));
      if(colbuf[0] != 3)
	throw ("PLY : bad facet size \n");
      faces.push_back(Face(itmp[0],itmp[1],itmp[2]));
    }
  }
  return CGAL::Bbox_3(xmin,ymin,zmin,xmax,ymax,zmax);
}

CGAL::Bbox_3 ply_ascii_data(std::ifstream &ifstr,TPoint &points,std::vector<Point> &normals,PointColor &colors,std::vector<Face> &faces,int nbpts,int nbfaces,int nbflt, int nbint) throw(const char *){
  float x, y, z, nx, ny, nz;
  double xmin(1.e20), ymin(1.e20), zmin(1.e20), xmax(-1.e20), ymax(-1.e20), zmax(-1.e20);
  int r,g,b,alpha;
  int j = points.size();
  if(nbfaces > 0) throw("faces not yet supported in ascii mode\n");
  for(int i = 0;i < nbpts;i++) {
    if (ifstr.eof()) {
      std::cout << "I = " << i << std::endl;
      throw("read_ply: points reading unexpected EOF\n");
    }
    ifstr >> x >> y >> z;
    if(x < xmin) xmin = x;
    if(y < ymin) ymin = y;
    if(z < zmin) zmin = z;
    if(x > xmax) xmax = x;
    if(y > ymax) ymax = y;
    if(z > zmax) zmax = z;
    if(nbflt > 3) {
      ifstr >> nx >>ny >>nz;
      normals.push_back(Point(nx,ny,nz));
    }
    if(nbint > 0) {
      ifstr >> r >> g >> b;
      if(nbint == 4)
	ifstr >> alpha;
      colors.push_back(CGAL::Color(r,g,b));
    } else
      colors.push_back(CGAL::Color(128,128,128));
    points.push_back( std::make_pair(Point(x,y,z),j++));
  }
  return CGAL::Bbox_3(xmin,ymin,zmin,xmax,ymax,zmax);
}

CGAL::Bbox_3 read_all_ply(const char *filename,TPoint &points,std::vector<Point> &normals,PointColor &colors,std::vector<Face> &faces,char **comment,bool with_faces) throw(const char *){
  std::ifstream ifstr;
  ifstr.open(filename);
  if(! ifstr.good()) {
    std::cerr << "File " << filename << " ";
    throw("read_data: cannot open file");
  }
  char tmp[1024];
  std::string name, str1;
  int nbpts(0), nbfaces(0);
  int nbflt = 0, nbint = 0;
  ifstr >> name;
  if(name != "ply") throw("read_ply: bad ply header\n");
  ifstr >> name >> str1;
  bool ascii_mode = true;
  if(name != "format")  throw("read_ply: incorrect ply format\n");
  if (str1 != "ascii") {
    if(str1 == "binary_little_endian") ascii_mode = false;
    else
      throw("read_ply: incorrect ply format\n");
  }
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
      else if (str1 == "face")
	nbfaces = itmp;
      else throw("read_ply: unexpected element");
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
    } else if(name == "comment") {
      ifstr.getline(tmp, 1024);
      tmp[1023] = 0;
      if(comment != NULL) {
	char *p = new char[strlen(tmp)];
	strcpy(p,tmp);
	*comment = p;
      }
      continue;
    } else {
      std::cout << "ERR " << name << std::endl;
      throw("read_ply: unexpected keyword\n");
    }
    if (ifstr.eof()) throw("read_ply: unexpected EOF\n");
  }
  if(!with_faces && nbfaces != 0) throw("read_ply: unexpected non nul faces nb\n");

  if((nbflt != 3 && nbflt != 6) || (nbint != 0 && nbint != 3 && nbint != 4)) 
    throw("read_ply: unexpected data structure\n");
  CGAL::Bbox_3 bb;
  if(ascii_mode)
    bb = ply_ascii_data(ifstr,points,normals,colors,faces,nbpts,nbfaces,nbflt,nbint);
  else
    bb = ply_binary_data(ifstr,points,normals,colors,faces,nbpts,nbfaces,nbflt,nbint);
  ifstr.close();
  return bb;
}
/**
   @brief read a ply file containing only points, optionnaly with colors and normals 
 **/
CGAL::Bbox_3 read_ply(const char *filename,TPoint &points,std::vector<Point> &normals,PointColor &colors) throw(const char *){
  std::vector<Face> faces;
  return read_all_ply(filename,points,normals,colors,faces,NULL,false);
}
/**
   @brief read a ply file containing points and facets.
**/
CGAL::Bbox_3 read_ply(const char *filename,TPoint &points,std::vector<Point> &normals,PointColor &colors,std::vector<Face> &faces,char **comment) throw(const char *){
  return read_all_ply(filename,points,normals,colors,faces,comment,true);
}


/* path file :
1st line = PATCHES
2nd : nb of patches
9 lignes per patch :
  - PATHCES
  - points homogeneous coords : x y z 1
  - normal in homogeneous coords : nx ny nz 0
  - m_ncc m_dscale m_ascale
  - nb of associated images
  - list of associated images indices
  - nb of images where the patch is visible
  - list of images or empty line
  - empty line
 */

void read_patches(const char *filename,int firstpoint,int nbcams,TPoint &points, std::map<int, VisiblePatches *> &image_patches, bool read_points) throw(const char *){
  std::ifstream ifstr;
  ifstr.open(filename);
  if(! ifstr.good())
    throw("read_data: cannot open file");
  char tmp[256];
  std::string name;
  int nbpatches;
  int n;
  float x, y, z, w;
  ifstr >> name;
  if(name != "PATCHES") throw("read_patches: bad header\n");
  ifstr >> nbpatches;
  int j = firstpoint;
  int nbvis = 0;
  for(int k = 0;k < nbpatches;k++) {
    if (ifstr.eof()) throw("read_patches: unexpected EOF\n");
    ifstr >> name;
    if(name != "PATCHS") throw("read_patches: bad patch header\n");
    ifstr >> x >> y >> z >> w;
    if (read_points) points.push_back( std::make_pair(Point(x,y,z),j));
    ifstr >> x >> y >> z >> w;  // normal
    ifstr >> x >> y >> z;
    ifstr >> n; // nb of associated images
    if(n == 0) throw("read_patches: nb of images is not > 0\n");
    nbvis++;
    for(int i = 0; i < n; i++) {
      int v;
      ifstr >> v;
      if (v >= nbcams) continue;  // for tests
      if(image_patches.count(v) == 0)
	image_patches[v] = new VisiblePatches();
      image_patches[v]->push_back(j);
    }
    ifstr >> n;  // nb of visibility images
    ifstr.ignore(256,'\n');
    ifstr.ignore(256,'\n');

    ifstr.ignore(256,'\n'); // empty line
    j++;
  }
  ifstr.close();
  //  std::cout << nbvis << " / " << nbpatches << " visibles" << std::endl;
}
static int cmp(const void *i1, const void *i2) { return *(int *)i1 > *(int *)i2;}

void get_patches(std::ifstream &iFileT,int nbcams,std::map<int, VisiblePatches*> &image_patches,TPoint &bad_cameras,int data_mode)  throw(const char *){
  if((data_mode & (CG_PATCHES | CG_BADCAMS)) == 0) return;
  for(int i = 0;i < nbcams;i++) {
    int tmp[2];
    iFileT.read((char *)tmp,sizeof(tmp));
    int n = tmp[1];
    int icam = tmp[0];
    if(data_mode & CG_PATCHES) {
      image_patches[icam] = new VisiblePatches();
      int *pi0 = new int[n];
      int *pi = pi0;
      iFileT.read((char *)pi,n * sizeof(int));
      for(int j = 0;j < n;j++)
	image_patches[icam]->push_back(*pi++);
      delete[] pi0;
    } else {
      iFileT.seekg(n * sizeof(int),std::ios_base::cur);
    }
  }
  if(! (data_mode & CG_BADCAMS)) return;
  int nb_badcams;
  iFileT.read((char *)&nb_badcams,sizeof(int));
  float *pf0 = new float[3 * nb_badcams];
  float *pf = pf0;
  int *pi0 = new int[nb_badcams];
  int *pi = pi0;
  iFileT.read((char *)pf0,3 * nb_badcams * sizeof(float));
  iFileT.read((char *)pi0,nb_badcams * sizeof(int));
  for(int i = 0;i < nb_badcams;i++, pf += 3, pi++) {
    bad_cameras.push_back(std::make_pair(Point(*pf,*(pf+1),*(pf+2)),*pi));
  }
  delete[] pf0;
  delete[] pi0;
}

void read_cgal_data(char *file,Delaunay &T,PointColor &pcolors,std::vector<Point> &normals,CGAL::Bbox_3 &bb,int *nbcams,int **cams_index,std::map<int, VisiblePatches*> &image_patches,TPoint &bad_cameras,int data_mode,float *edge_mean,float*tetra_coefs) throw(const char *){
  std::ifstream iFileT;
  iFileT.open(file);
  if(! iFileT.good()) {
    std::cerr << "File " << file << " ";
    throw("read_cgal_data: cannot open file");
  }
  CGAL::set_binary_mode(iFileT);
  char tmp[4];
  iFileT.read(tmp,4);
  if(strncmp(tmp,file_version,4) != 0)
    throw ("Incompatible version for CGAL file\n");
  // delaunay
  iFileT >> T;
  Delaunay::Finite_vertices_iterator it;
  int i = 0;
  // bbox + tetra_coefs
  float box[6];
  iFileT.read((char *)box,sizeof(box));
  bb = CGAL::Bbox_3(box[0],box[1],box[2],box[3],box[4],box[5]);
  float ftmp[3];
  iFileT.read((char *)ftmp,3 * sizeof(float));
  *edge_mean = ftmp[0];
  tetra_coefs[0] = ftmp[1];
  tetra_coefs[1] = ftmp[2];
  // nb valid cameras
  iFileT.read((char *)nbcams,sizeof(int));
  // colors of points
  for(it = T.finite_vertices_begin();it != T.finite_vertices_end();it++,i++) {
    unsigned char tmp[3];
    iFileT.read((char *)tmp,sizeof(tmp));
    it->info() = i;
    pcolors.push_back(CGAL::Color(tmp[0],tmp[1],tmp[2]));
  }
  bool with_normals;
  // normals of points if available
  iFileT.read((char *)&with_normals,sizeof(with_normals));
  if(with_normals) {
    for(it = T.finite_vertices_begin();it != T.finite_vertices_end();it++) {
      float tmp[3];
      iFileT.read((char *)tmp,sizeof(tmp));
      normals.push_back(Point(tmp[0],tmp[1],tmp[2]));
    }
  }
  // cameras indexes
  int *cams = new int[*nbcams * 2];
  iFileT.read((char *)cams,*nbcams * 2 * sizeof(int));
  Delaunay::Finite_cells_iterator cit;
  for (cit = T.finite_cells_begin(); cit != T.finite_cells_end(); ++cit) {
    iFileT.read((char *)&i,sizeof(i));
    cit->info() = i;
  }
  get_patches(iFileT,*nbcams,image_patches,bad_cameras,data_mode);
  iFileT.close();
  // return sorted list oy cam indexes
  
  *cams_index = cams;
  qsort(cams,*nbcams,sizeof(int),cmp);
}


void read_cgal_xdata(char *file,int *nbcams,int **cams_index,CGAL::Bbox_3 &bb,std::map<int, VisiblePatches*> &image_patches,TPoint &bad_cameras,int data_mode,float *edge_mean,float*tetra_coefs)  throw(const char *){
  std::ifstream iFileT;
  iFileT.open(file);
  if(! iFileT.good()) {
    std::cerr << "File " << file << " ";
    throw("read_cgal_xdata: cannot open file");
  }
  CGAL::set_binary_mode(iFileT);
  char tmp[4];
  iFileT.read(tmp,4);
  if(strncmp(tmp,file_version,4) != 0)
    throw ("Incompatible version for CGAL file\n");
  Delaunay T;
  iFileT >> T;
  float box[6];
  iFileT.read((char *)box,sizeof(box));
  bb = CGAL::Bbox_3(box[0],box[1],box[2],box[3],box[4],box[5]);
  float ftmp[3];
  iFileT.read((char *)ftmp,3 * sizeof(float));
  *edge_mean = ftmp[0];
  tetra_coefs[0] = ftmp[1];
  tetra_coefs[1] = ftmp[2];
  iFileT.read((char *)nbcams,sizeof(int));
  int nbv = std::distance(T.finite_vertices_begin(),T.finite_vertices_end());
  // colors
  iFileT.seekg(nbv * 3,std::ios_base::cur);
  // normals
  bool with_normals;
  iFileT.read((char *)&with_normals,sizeof(with_normals));
  if(with_normals)
    iFileT.seekg(nbv * 3 * sizeof(float),std::ios_base::cur);
  // cams index
  int *cams = new int[2 * *nbcams];
  iFileT.read((char *)cams,*nbcams * 2 * sizeof(int));
  *cams_index = cams;
  int nbcells = std::distance(T.finite_cells_begin(),T.finite_cells_end());
  iFileT.seekg(nbcells * sizeof(int),std::ios_base::cur);
  get_patches(iFileT,*nbcams,image_patches,bad_cameras,data_mode);
}

