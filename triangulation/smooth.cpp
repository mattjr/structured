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

/** \file smooth.cpp
    \brief "Smoothing" of verices coordinates.
 **/

/**
   \brief Does one iteration of smoothing
 **/
void smooth1(Delaunay &T,std::vector<Point> &normals,std::map<Facet,bool> keep_facets,Vector *in_pts,Vector *out_pts, float lambda,std::map<int,bool> &cams_map,bool same_weight) {

  int nb_orphans = 0;

  for(Delaunay::Finite_vertices_iterator itv = T.finite_vertices_begin(); itv != T.finite_vertices_end(); itv++) {
    Vertex_handle vp = itv;
    int jp = vp->info();
    Point nx = vp->point();
    Vector p = *(in_pts + jp);
    nx = normals[jp];
    Vector np = Vector(nx.x(),nx.y(),nx.z());
    if(cams_map.find(jp) != cams_map.end()) {
      out_pts[jp] = in_pts[jp];
      continue;
    }
    std::vector<Facet> ifacets;
    T.finite_incident_facets(vp,std::back_inserter(ifacets));
    std::map<Vertex_handle,float> vneighbours; // liste des voisins appartenant à un tetra valide
    int nbwq = 0;
    float sumw = 0.;
    for (std::vector<Facet>::iterator it = ifacets.begin();it != ifacets.end();it++) {
      Cell_handle c = it->first;
      if(keep_facets.find(*it) == keep_facets.end()) continue;
      int j = 4 * it->second;
      for(int i = 0;i<3;i++) {
	Vertex_handle v = c->vertex(facet_vertex_order[j++]);
	if(vp == v || vneighbours.find(v) != vneighbours.end()) continue;
	if(cams_map.find(v->info()) != cams_map.end()) continue;
	float w;
	if (same_weight) {
	  w = 1.;
	  nbwq++;
	} else {
	  Point nx = normals[v->info()];
	  Vector nq = Vector(nx.x(),nx.y(),nx.z());
	  w = np * nq;
	  if (w < 1.e-50) w = 0.;
	  else {
	    // uncomment for k=2
	    //w *= w;
	    nbwq++;
	  
#if 0	  
	  // lower weight of far points ? 
	  Vector q = *(in_pts + v->info());
	  
	  Vector r = p - q;
	  double coefd = fabs(r[0]) + fabs(r[1]) + fabs(r[2]);
	  if(coefd < 1.e-20)
	    std::cout << "PB " << p << " / " << q << " : " << r << " , " << coefd << std::endl;
	  else
	    w = w / coefd;
#endif
	  }
	}
	sumw += w;
	vneighbours[v] = w;
      }
    }
    if(nbwq == 0) {
      nb_orphans++;
      out_pts[jp] = p;
      /*
      std::cout << "WARNING : no neighbours (" << p << ")" << std::endl;
      out_pts[jp] = in_pts[jp];
      */
      continue;
    }
    Vector sumwq = Vector(0.,0.,0.);
    for(std::map<Vertex_handle,float>::iterator itm = vneighbours.begin();itm != vneighbours.end();itm++) {
      float w = itm->second;
      if(w == 0.) continue;
      Vertex_handle v = itm->first;
      Vector q = *(in_pts + v->info());
      sumwq = sumwq + q * w;
      
    }
    float mu = 1. / sumw;
    Vector r = mu * sumwq;
    Vector newp = lambda * p + (1 - lambda) * r;
    out_pts[jp] = newp;
  }
  if(nb_orphans > 0)
    std::cout << nb_orphans << " points without valid neighbours" << std::endl;
}
/**
   \brief Replace vertices coordinates by a function of their neighbours
   
    \f$p = \lambda p + (1 - \lambda) \sum_{q \in N_p} w_q q\f$ 

       \f$N_p\f$
       is the set of neighbours of p, \f$\lambda\f$ is a positive scalar \f$< 1\f$,
       weights \f$w_q\f$ sums to 1 and are a function of the unit normals \f$n_p n_q\f$.

    For example \f$w_q = \mu[n_p.n_q]_+^k\f$, where \f$x_+ = x\f$ if \f$x \geq 0\f$ and 0 otherwise,
    and \f$\mu\f$ is choosen so that \f$\sum_a\in N_pw_q = 1\f$.

   \param T : delaunay data
   \param normals : vector of vertices PMVS normals.
   \param facets : vector of facets on the surface. The neighbourhood is restricted to cells containing te facets.
   \param lambda : the \f$\lambda\f$ coefficient.
   \param nbiter : number of iterations. If negative, 
   \param nbcams : number of cameras
   \param cams_index : indexes of cameras coords in PMVS points table.
   
 **/
Vector *smooth(Delaunay &T,std::vector<Point> &normals,std::vector<Facet> &facets,float lambda,int nbiter,int nbcams,int *cams_index) {
  Vector *ppoints[2];
  std::map<int,bool> cams_map;
  for(int i = 0;i < nbcams;i++) 
    cams_map[cams_index[i]] = true;
  std::map<Facet,bool> keep_facets;
  for(std::vector<Facet>::iterator it = facets.begin();it != facets.end();it++)
    keep_facets[*it] = true;
  int nbpoints = std::distance(T.finite_vertices_begin(),T.finite_vertices_end());
  ppoints[0] = new Vector[nbpoints];
  Vector *pts = ppoints[0];
  for(Delaunay::Finite_vertices_iterator itv = T.finite_vertices_begin(); itv != T.finite_vertices_end(); itv++) {
    Vertex_handle v = itv;
    Point p = v->point();
    *(pts++) = Vector(p.x(),p.y(),p.z());
  }
  ppoints[1] = new Vector[nbpoints];
  int i1(0), i2(1);
  bool same_weight = false;
  if(nbiter < 0) {
    nbiter = -nbiter;
    same_weight = true;
  }
  for (int iter = 0;iter < nbiter;iter++) {
    std::cout << "SMOOTH " << iter << std::endl;
    smooth1(T,normals,keep_facets,ppoints[i1],ppoints[i2],lambda,cams_map,same_weight);
    i1 ^= 1;
    i2 ^= 1;
  }
  delete[] ppoints[i2];
  return ppoints[i1];
}
