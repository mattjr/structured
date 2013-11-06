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
#include <map>
#include <time.h>
#include "graph.h"
static int nb_rm_added(0),nb_rm_normals(0);

typedef Graph<int,int,int> GraphType;

/** \file extract.cpp
    \brief Functions for surface facets extraction.
 **/

/**
   \brief Find tetrahedrons to keep and remove with a cost function minimization
   \param[in,out] T : delaunay triangulation. Removed and valid cells are flagged in field info
   \param params.nb_intersect : number of intersection threshold
 **/
void prepare_extract_mxf(Delaunay &T,TrParams &params,CGAL::Geomview_stream &gv) {
  Delaunay::Finite_cells_iterator cit;
  int nb_inter = params.nb_intersect;
  int nbp(0), nbrm(0);
  int nbpos(0), nbneg(0);
  int nbcells = std::distance(T.finite_cells_begin(),T.finite_cells_end());
  GraphType *g = new GraphType(nbcells,nbcells);
  int *infos = new int[nbcells];
  int i = 0;
  // we need to find nb intersections for neighbours;
  for (cit = T.finite_cells_begin(); cit != T.finite_cells_end(); ++cit,i++) {
    Cell_handle c = cit;
    infos[i] = c->info();
    c->info() = i;
  }
  for (int i=0;i<nbcells;i++)
    g -> add_node();
  for (cit = T.finite_cells_begin(); cit != T.finite_cells_end(); ++cit) {
    int cout1, cout2;
    int k = cit->info();
    if(infos[k] > 0) nbp++;
    int n = infos[k] - nb_inter;
    // cout1 = 0 si n < 0 ??
    cout1 = n;
    cout2 = 0;
    g -> add_tweights( k, cout1,cout2);
    //    std::cout << "XX " << k << " " << cout1 << " " << cout2 << std::endl;
    int i0 = cit->info();
    for(int j = 0;j < 4;j++) {
      Cell_handle c1 = cit->neighbor(j);
      if(T.is_infinite(c1)) continue;
      int i1 = c1->info();
#if 1
      if(i1 > i0){
	int v0 = infos[i0];
	int v1 = infos[i1];
#if 0
	v0 = (v0 == v1) ? 1 : ((v0 > v1) ? v0 - v1 : v1 - v0);
	g -> add_edge(i0,i1,v0,v0);
#else
	v0 = (v0 <= nb_inter) ? 0 : v0 - nb_inter;
	v1 = (v1 <= nb_inter) ? 0 : v1 - nb_inter;
	g -> add_edge(i0,i1,v0,v1);
	//	v0 = (v0 == v1) ? 1. : ((v0 < v1) ? v1 - v0 : v0 -v1);
	//v0 = 1. / v0;
#endif
	// add_edge(node1,node2,cost1_2,cost2_1)
	// std::cout << "XX " << i0 << " - " << i1 << " " << v0 << std::endl;
      }
#endif
    }
  }
  //  std::cout << nbpos << " POS " << nbneg << " NEG" << std::endl;
  int flow = g -> maxflow();
  int k = 0;
  for (cit = T.finite_cells_begin(); cit != T.finite_cells_end(); ++cit,k++) {
    int k = cit->info();
    if (g->what_segment(k) == GraphType::SINK) { // cell to keep
      cit->info() = infos[k] | 1;  // mark as valid
    } else {
      nbrm++;
      cit->info() = infos[k] & ~1; // mark as rm
    }
  }
  delete[] infos;
  std::cout << "Flow " << flow << ", " << nbcells << " CELLS, " << nbp << " intersected, " << nbrm << " removed." << std::endl;
  delete g;
  return;
}
/**
   \brief Find which tetrahedrons to remove or keep, based on nb of ray intersections
   \param[in,out] T : delaunay triangulation. Removed and valid cells are flagged in field info
   \param params.nb_intersect : number of intersection threshold
   \param params.average_volume : increase the threshold for cells having a larger volume.
 **/
void prepare_extract_std(Delaunay &T,TrParams &params,
			 CGAL::Geomview_stream &gv) {
  Delaunay::Finite_cells_iterator cit;
  int nbp(0), nbrm(0);
  int min_intersect = params.nb_intersect;
  int nbcells = std::distance(T.finite_cells_begin(),T.finite_cells_end());
  //  Tds tds = T.tds();
  int nb_rm1 = 0;
  //  draw_all(gv,T);
  double min_vol;
  bool check_vol = false;
  if(params.average_volume > 0.) {
     min_vol = params.average_volume * 1.1;
     check_vol = true;
  }
  int k = 0;
  for (cit = T.finite_cells_begin(); cit != T.finite_cells_end(); ++cit, k++) {
    Cell_handle c = cit;
    int info = c->info();
    //    std::cout << "INFO " << info << std::endl;
    if(info > 0) {
      nbp++;
      if(info >= min_intersect) {
	if((info < MAX_INTERSECT) && check_vol) {
	  Tetrahedron t = T.tetrahedron(c);
	  double v = t.volume();
	  int vcoef = (int) (v / min_vol);
	  //	  std::cout  << "VOL " << v  << " coef " << vcoef << std::endl;
	  if(vcoef > 1) {
	    int new_min = vcoef * min_intersect;
	    if(vcoef > (MAX_INTERSECT / min_intersect))
	      new_min = MAX_INTERSECT;
	    if(info < new_min) continue;
	  }
	}
	nbrm++;
	c->info() &= ~1; // mark as rm
	continue;
      } 
    }
    c->info() |= 1; // mark as valid
  }

  std::cout << nbcells << " CELLS, " << nbp << " intersected, " << nbrm << " removed." << std::endl;
}

/**
   \brief try to remove rough elements, that is tetrahedrons with
   only one face adjacent to a non infinite tetrahedron.
**/
int clean1(Delaunay &T) {
  int nb_rm1 = 0;
  std::cout << "CLEAN\n";
  for (Delaunay::Finite_cells_iterator it = T.finite_cells_begin();it != T.finite_cells_end();it++) {
    Cell_handle c = it;
    if(! c->info() & 1) continue;
    int nb_ext = 0;
    for(int i = 0;i < 4; i++) {
      Facet f1 = T.mirror_facet(Facet(c,i));
      if (T.is_infinite(f1.first) || (! ((f1.first)->info() & 1)))
	nb_ext++;
    }
    if(nb_ext == 3) { // si les 4 faces sont exterieures on ferait un trou
      c->info() &= ~1; // mark as rm
      nb_rm1++;
    }
  }
  return nb_rm1;
}
/**
\brief Check if a facet is too wide and/or has a too long edge
  \param T : the delaunay data.
  \param f : the facet to test.
  \param[in,out] nb_rmlg : nb of faces removed by edge length check; incremented .
  \param[in,out] nb_rmsurf : nb of faces removed by surface check; incremented .
  \param params.do_clean : define what checking to do
  \param params.lgr_threshold : threshold for length check.
  \param params.surf_threshold : threshold for surface check.
**/
bool clean2(Delaunay &T,Facet &f, int *nb_rmlg, int *nb_rmsurf,TrParams &params) {
  Triangle t = T.triangle(f);
  if (params.do_clean & CLEAN_SURF) {
    double surf;
    if (t.squared_area() >= params.surf_threshold) {
      (*nb_rmsurf)++;
      return true;
    }
  }
  if (params.do_clean & CLEAN_LGR) {
    int ipts[] = {0,1,2,3,0};
    for(int i = 0;i <3; i++) {
      Segment s = Segment(t[ipts[i]],t[ipts[i+1]]);
      double lgr = s.squared_length();
      if(lgr > params.lgr_threshold) {
	(*nb_rmlg)++;
	return true;
      }
    }
  }
  return false;
}
// recherche par dichotomie
static bool is_cam(Vertex_handle v,int nbcams,int *cams) {
  int iv = v->info();
  int j1(0), j2(nbcams - 1);
  if (iv < cams[j1] || iv > cams[j2]) return false;
  while (j1 <= j2) {
    int j = (j1 + j2) / 2;
    if (iv == cams[j]) return true;
    if(iv < cams[j])
      j2 = j - 1;
    else
      j1 = j + 1;
  }
  return false;
}

static bool acceptable_normals(Delaunay &T,Facet f,std::vector<Point> &normals,TrParams &params) {
  if(normals.size() == 0) return true;
  Cell_handle c = f.first;
  int i = f.second;
  int j = 4 * i;
  Point pts[3];
  Vector ns[3];
  for(int k = 0;k < 3;k++) {
    Vertex_handle v = c->vertex(facet_vertex_order[j++]);
    pts[k] = v->point();
    Point p = normals[v->info()];
    ns[k] = Vector(p.x(),p.y(),p.z());
  }
  Vector normal = -CGAL::unit_normal(pts[0],pts[1],pts[2]);
  bool reject;
  if(params.moy_normal) 
    reject =  ((ns[0] + ns[1] + ns[2]) * normal  < 3. * params.cosinus_thresh);
  else {
    reject =  ((normal * ns[0] < params.cosinus_thresh) || (normal * ns[1] < params.cosinus_thresh) || (normal * ns[2] < params.cosinus_thresh));
  }
  if(reject)
    {
      nb_rm_normals++;
      //     std::cout << cosinus_thresh << ", " << normal * ns[0] << ", " << normal * ns[1] << ", " << normal * ns[2] << std::endl;
      return false;
    }
  return true;
}

/**
   \brief build the list of surface facets
   \param normals: the vector of PMVS normals
   \param nbcams : number of cameras
   \param cams_index : indexes of cameras coords in PMVS points table
   \param params.extract_mode : 0/1 = use removed/valid cells to find facets
       +2 = retrieve all facets of corresponding tetrahedrons (for test purpose)
   \param params.do_clean : type of desired cleaning
**/

void prepare_facets(Delaunay &T,std::vector<Facet> &facets,std::vector<Point> &normals,int nbcams,int *cams_index,TrParams &params,CGAL::Geomview_stream &gv) {
  int nb_rm1 = 0, nb_rmlg = 0, nb_rmsurf= 0;
  int mode = params.extract_mode;

  if(params.do_clean & CLEAN_TETRA) {
    // remove cells that have 3 faces on the border
    nb_rm1 = clean1(T);
  }

  if ((mode & 1) == 0) { // selection based on removed cells
    // infinite cells must be considered as removed ones
    for (Delaunay::All_cells_iterator itc = T.all_cells_begin();itc != T.all_cells_end();itc++) {
      bool is_inf = false;
      if(T.is_infinite(itc)) is_inf = true;
      else if(itc->info() & 1) continue;
      // removed tetra : check which adjacent facet to keep
      std::vector<Facet> rm_facets;
      std::vector<Cell_handle> rm_cells;
      if(debug_vis > 3) {
	rm_cells.push_back(itc);
	draw_all(gv,T);
	draw_tetra(T,rm_cells,gv);
	prompt("ADDED FACETS");
      }
      // ifd mode == 2 ignore tetra on cameras
      if(mode == 2 && ! is_inf) {
	bool valid = true;
	for(int i = 0;i < 4;i++) {
	  if(is_cam(itc->vertex(i),nbcams,cams_index)) {
	    valid = false;
	    break;
	  }
	}
	if(! valid) continue;
      }
      for(int i = 0;i < 4;i++) {
	Facet f1 = Facet(itc,i);
	if(mode == 2 && ! is_inf) { // all faces of object tetras
	  //if(itc->info() >= MAX_INTERSECT) continue;
	  facets.push_back(f1);
	  continue;
	}
	Facet f = T.mirror_facet(f1);
	if(T.is_infinite(f.first)) {
	
#if 0
	  int j = 4 * i;
	  bool candidate = true;
	  for(int k = 0;k < 3;k++) {
	    if( is_cam(c->vertex(facet_vertex_order[j++]),nbcams,cams_index)) {
	      candidate = false;
	      break;
	    }
	  }
	  if(candidate) {
	    if ((params.do_clean & (CLEAN_LGR | CLEAN_SURF)) && clean2(T,f1,&nb_rmlg,&nb_rmsurf,params)) continue;
	    if(! acceptable_normals(T,f,normals,params)) continue;
	    nb_rm_added++;
	    facets.push_back(f1);

	    rm_facets.push_back(f1);
	  }
#endif
	  continue;
	}
	// removed cell adjacent to a finite cell
	if((f.first)->info() & 1) { // mirror facet belongs to a valid cell
	  // rm "too long" facets
	  if (params.do_clean & (CLEAN_LGR | CLEAN_SURF)) {
	    if(clean2(T,f,&nb_rmlg,&nb_rmsurf,params)) continue;
	  }
	  if(! acceptable_normals(T,f,normals,params)) continue;
	  facets.push_back(f1);
	}
      }
      if(debug_vis > 3) {
	if(rm_facets.size() > 0) {
	  draw_all(gv,T);
	  draw_facets(T,rm_facets,gv);
	}
	prompt("NEXT");
      }

    }
  } else {  // selection based on kept tetras
    for (Delaunay::Finite_cells_iterator it = T.finite_cells_begin();it != T.finite_cells_end();it++) {
      if(! (it->info() & 1)) continue;
      Cell_handle c = it;
      for(int i = 0;i < 4;i++) {
	Facet f = Facet(c,i);
	if(mode == 3) {
	  facets.push_back(f);
	  continue;
	}
	Facet f1 = T.mirror_facet(f);
	if ( T.is_infinite(f1.first) || (! ((f1.first)->info() & 1))) {

	// rm facets trop "longues"
	  if (params.do_clean & (CLEAN_SURF | CLEAN_LGR)) {
	    if(clean2(T,f,&nb_rmlg,&nb_rmsurf,params)) continue;
	  }
	  if(! acceptable_normals(T,f,normals,params)) continue;
	  facets.push_back(f);
	}
      }
    }
  }
  if(params.do_clean > 0)
    std::cout << "CLEAN removed " << nb_rm1 << " '-ct' + " << nb_rmlg << " '-lg' + " << nb_rmsurf << " '-s'." << std::endl;
  std::cout << "Removed by test on normals : " << nb_rm_normals << ", cos max limit " << params.cosinus_thresh << std::endl;
  int nbfacets = facets.size();
  if(debug_vis > 0 && nbfacets < 500) {
    draw_all(gv,T);
    draw_facets(T,facets,gv);
    prompt("XX");
  }
  
}
/** 
    \brief Save results (points and facets) in an ascii ply file.
           For test only (data is partial)
 */
void save_ply(const char *file,Delaunay &T,std::vector<Facet> &facets,Vector *points,std::vector<Point> &normals,PointColor &pcolors,int nbcams,int *cams_index,TPoint &bad_cameras,TrParams &params,char *comment) throw(const char *){
  //  Delaunay nt;
  //Tds tds = nt.tds();
  //tds.clear();
  std::cout << "SAVE PLY\n";
  bool with_color = (pcolors.size() > 0) ? true : false;
  bool with_normals = (normals.size() > 0) ? true : false;
  int nb_badcams = bad_cameras.size();
  int nbvertices = std::distance(T.finite_vertices_begin(),T.finite_vertices_end());
  int nbpts = nbvertices + nb_badcams;
  std::ofstream oFileT;
  int nbfacets = facets.size();
  oFileT.open(file);
  if(! oFileT.good())
    throw("save_ply: cannot create file");
  oFileT << "ply\nformat ascii 1.0\n";
  if(comment != NULL) oFileT << "comment " << comment << std::endl;
  oFileT << "element vertex " << nbpts << "\n" 
	 << "property float x\nproperty float y\nproperty float z\n";
  if(with_normals)
    oFileT << "property float nx\nproperty float ny\nproperty float nz\n";
  if(with_color) 
    oFileT << "property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar alpha\n";

  oFileT << "element face " << nbfacets << "\nproperty list uchar int vertex_indices\nend_header\n";
  
  std::map< Vertex_handle, std::size_t > V;
  Delaunay::Finite_vertices_iterator itv;
  int i = 0;
  for(itv = T.finite_vertices_begin(); itv != T.finite_vertices_end(); itv++, i++) {
    Vertex_handle v = itv;
    V[v] = i;
    if(points == NULL)
      oFileT << v->point();
    else {
      Vector vp = points[v->info()];
      oFileT << vp;
    }
    if(with_normals) {
      oFileT << " " << normals[itv->info()];
    }
    if(with_color) {
      CGAL::Color col = pcolors[itv->info()];
      oFileT << " " << (int) col.red() << " " <<  (int) col.green() << " " << (int) col.blue() << " 255";
    }
    oFileT << std::endl;
  }
  // ajout des "bad cams"
  Point normal(1.,1.,1.);
  for(int i = 0;i < nb_badcams;i++) {
    Point p = bad_cameras[i].first;
    oFileT << p << " 1. 1. 1. 255 0 0 255" << std::endl;
  }
  // facets
  int lst_pos[3] = {2,1,3};
  int lst_neg[3] = {1,2,3};
  int *lst;
  int mode = params.extract_mode & 1;
  for (std::vector<Facet>::iterator it = facets.begin();it != facets.end();it++) {
    int vertices[3];
    int k = 0;
    Cell_handle c = it->first;
    int j = it->second;
    // print facets correctly oriented
    if ((j & 1) == mode) lst = lst_pos;
    else lst = lst_neg;
    for(int i = 0;i < 3;i++) {
      int ij = (j + lst[i]) & 3;
      Vertex_handle v = c->vertex(ij);
      vertices[k++] = V[v];
    }
    oFileT << "3 " << vertices[0] << " " << vertices[1] << " " << vertices[2] << std::endl;
  }
  oFileT.close();
  std::cout << "DATA SAVED: " << nbpts << " vertices, " << nbfacets << " facets." <<  std::endl;
}

/**
   \brief Save vertices and extracted faces in a binary ply file
   \param file : destination file
   \param T : delaunay data
   \param points : optional vector of points. If non nul use it for points coords instead of delaunay vertices.
   \param normals : vector of PMVS normals
   \param pcolors : vector of PMVS point colors
   \param nbcams : number of cameras
   \param cams_index : indices of camera coords in points vector
   \param bad_cameras : vector of the vertices of bad cameras.
   \param params.extract_mode : use to print facet vertice in the correct order
   \param comment : comment to put in the header (the arguments of the command line)
 **/
void save_ply_binary(const char *file,Delaunay &T,std::vector<Facet> &facets,Vector *points,std::vector<Point> &normals,PointColor &pcolors,int nbcams,int *cams_index,TPoint &bad_cameras,TrParams &params,char *comment,  bool flip_face=false) throw(const char *){
  //  Delaunay nt;
  //Tds tds = nt.tds();
  //tds.clear();
  std::cout << "SAVE PLY BIN" << std::endl;
  std::cout <<"Flipping: " << flip_face <<std::endl;
  bool with_color = (pcolors.size() > 0) ? true : false;
  bool with_normals = (normals.size() > 0) ? true : false;
  int nb_badcams = bad_cameras.size();
  int nbvertices = std::distance(T.finite_vertices_begin(),T.finite_vertices_end());
  int nbpts = nbvertices + nb_badcams;
  std::ofstream oFileT;
  int nbfacets = facets.size();
  oFileT.open(file);
  if(! oFileT.good())
    throw("save_ply: cannot create file");
  oFileT << "ply\nformat binary_little_endian 1.0\n";
  if(comment != NULL) oFileT << "comment " << comment << std::endl;
  oFileT << "element vertex " << nbpts << "\n" 
	 << "property float x\nproperty float y\nproperty float z\n";
  if(with_normals)
    oFileT << "property float nx\nproperty float ny\nproperty float nz\n";
  if(with_color) 
    oFileT << "property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar alpha\n";

  oFileT << "element face " << nbfacets << "\nproperty list uchar int vertex_indices\nend_header\n";
  
  std::map< Vertex_handle, std::size_t > V;
  Delaunay::Finite_vertices_iterator itv;
  int i = 0;
  float ptbuf[3];
  unsigned char  colbuf[4] = {0,0,0,255};
  for(itv = T.finite_vertices_begin(); itv != T.finite_vertices_end(); itv++, i++) {
    Vertex_handle v = itv;
    V[v] = i;
    Point p;
    if(points == NULL) {
      p = v->point();
      ptbuf[0] = (float)p.x();
      ptbuf[1] = (float)p.y();
      ptbuf[2] = (float)p.z();
    } else {
      Vector vp = points[v->info()];
      ptbuf[0] = (float)vp[0];
      ptbuf[1] = (float)vp[1];
      ptbuf[2] = (float)vp[2];
    }
    oFileT.write((char *)ptbuf,sizeof(ptbuf));
    if(with_normals) {
      p = normals[itv->info()];
      ptbuf[0] = (float)p.x();
      ptbuf[1] = (float)p.y();
      ptbuf[2] = (float)p.z();
      oFileT.write((char *)ptbuf,sizeof(ptbuf));
    }
    if(with_color) {
      CGAL::Color col = pcolors[itv->info()];
      colbuf[0] = (int) col.red();
      colbuf[1] = (int) col.green();
      colbuf[2] = (int) col.blue();
      oFileT.write((char *)colbuf,sizeof(colbuf));
    }
  }
  // ajout des "bad cams"
  float ftmp[6] = {0.,0.,0.,1.,1.,1.};
  ptbuf[3] = 1.;ptbuf[4] = 1.;ptbuf[5] = 1.;
  colbuf[0] = 255;colbuf[1] = 0;colbuf[2] = 0;
  for(int i = 0;i < nb_badcams;i++) {
    Point p = bad_cameras[i].first;
    ftmp[0] = (float)p.x();
    ftmp[1] = (float)p.y();
    ftmp[2] = (float)p.z();
    oFileT.write((char *)ftmp,sizeof(ftmp));
    oFileT.write((char *)colbuf,sizeof(colbuf));
  }
  // facets

  int lst_pos[3] = {2,1,3};
  int lst_neg[3] = {1,2,3};
  int *lst;
  int mode = params.extract_mode & 1;
  for (std::vector<Facet>::iterator it = facets.begin();it != facets.end();it++) {
    int vertices[3];
    int k = 0;
    colbuf[0] = 3;
    Cell_handle c = it->first;
    int j = it->second;
    // print facets correctly oriented
    if ((j & 1) == mode) lst = lst_pos;
    else lst = lst_neg;
    if(flip_face){
        for(int i = 2;i>=0;i--) {
        int ij = (j + lst[i]) & 3;
        Vertex_handle v = c->vertex(ij);
        vertices[k++] = V[v];
        }
    }else{
        for(int i = 0;i<3;i++) {
        int ij = (j + lst[i]) & 3;
        Vertex_handle v = c->vertex(ij);
        vertices[k++] = V[v];
        }

    }
	
    oFileT.write((char *)colbuf,1);
    oFileT.write((char *)vertices,sizeof(vertices));
  }
  oFileT.close();
  std::cout << "DATA SAVED " << nbpts << " vertices, " << nbfacets << " facets (" <<  nb_rm_added << " from rm tetras)" << std::endl;
}

/**
   \brief Extract surface facets from the delaunay triangulation
   \param outply : name of output ply file.
   \param T : delaunay data
   \param normals : vector of PMVS normals
   \param pcolors : vector of PMVS point colors
   \param nbcams : number of cameras
   \param cams_index : indices of camera coords in points vector
   \param bad_cameras : vector of the vertices of bad cameras.
   \param params.extract_type : call prepare_extract_mxf or prepare_extract_std
   \param params.smooth_coef : if >0., run the smooth program on vertices.
   \param comment : comment to put in the header (the arguments of the command line)
  \param 
 **/
void delaunay_extract(const char *outply,Delaunay &T,std::vector<Point> &normals,PointColor &pcolors,int nbcams,int *cams_index,TPoint &bad_cameras,TrParams &params,CGAL::Geomview_stream &gv,char *comment,bool flip_face) {
#ifdef __USE_POSIX
  struct timespec tstart, tend;
  clock_gettime(CLOCK_REALTIME,&tstart);
#else
  time_t tstart, tend;
  tstart = time(NULL);
#endif

  int nbcells = std::distance(T.finite_cells_begin(),T.finite_cells_end());
  // find tetrahedrons to remove and to keep
  if (params.extract_type == XTR_MAXFLOW)
    prepare_extract_mxf(T,params,gv);
  else
    prepare_extract_std(T,params,gv);
#ifdef __USE_POSIX
  clock_gettime(CLOCK_REALTIME,&tend);
#else
  tend = time(NULL);
#endif
  std::cout << "Extraction took " << delta_t(&tstart,&tend) << "s" << std::endl;

  // find facets of the surface
  PointColor nocol(0);
  std::vector<Facet> facets;
#ifdef __USE_POSIX
  clock_gettime(CLOCK_REALTIME,&tstart);
#else
  tstart = time(NULL);
#endif
  prepare_facets(T,facets,normals, nbcams,cams_index,params,gv);
#ifdef __USE_POSIX
  clock_gettime(CLOCK_REALTIME,&tend);
#else
  tend = time(NULL);
#endif
  std::cout << "Facets selection took " << delta_t(&tstart,&tend) << "s" << std::endl;
  // smooth points if validated
  Vector *points = NULL;
  if(params.smooth_coef > 0.) {
#ifdef __USE_POSIX
    clock_gettime(CLOCK_REALTIME,&tstart);
#else
    tstart = time(NULL);
#endif
    points = smooth(T,normals,facets,params.smooth_coef,params.smooth_nb_iter,nbcams,cams_index);
#ifdef __USE_POSIX
    clock_gettime(CLOCK_REALTIME,&tend);
#else
    tend = time(NULL);
#endif
    std::cout << "SMOOTH took " << delta_t(&tstart,&tend) << "s" << std::endl;
  }
  // write result in binary
  save_ply_binary(outply,T,facets,points,normals,pcolors,nbcams,cams_index,bad_cameras,params,comment,flip_face);
  //save_ply(outply,T,facets,points,normals,pcolors,nbcams,cams_index,bad_cameras,params,comment);
#ifdef __USE_POSIX
  clock_gettime(CLOCK_REALTIME,&tend);
#else
  tend = time(NULL);
#endif
  std::cout << "Saving ply took " << delta_t(&tstart,&tend) << "s" << std::endl;
  //save_ply("newtr.ply",T,keepcells,rm_map,normals,nocol,gv);
  if(debug_vis > 0) {
    gv.clear();
    std::vector<Facet> keepfacets;
    for (Delaunay::Finite_cells_iterator it = T.finite_cells_begin();it != T.finite_cells_end();it++) { 
      if(it->info() & 1)
	for(int i = 0;i < 4;i++) 
	  keepfacets.push_back(Facet(it,i));
    }
    
#if 1
    std::vector<Triangle> triangles;
    for(std::vector<Facet>::iterator it = keepfacets.begin();it != keepfacets.end();it++)
      triangles.push_back(T.triangle(*it));
    gv.draw_triangles(triangles.begin(),triangles.end());
#else
    draw_facets(T,keepfacets,pcolors,gv);
#endif
    prompt("END\n");
  }
}
