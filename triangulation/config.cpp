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
/** \file config.cpp
 @brief Configuration parameters
**/
char file_version[] = "GG05"; /// used  to tag .cgal files 
int debug_stop = 0 ; // 1 2 3 4 pour les endroits ou s'arreter
int debug_vis = 0; // 1 2 3 4 pour la visu (option = stop + 10 * vis

bool gv_on = false;
int facet_vertex_order[] = {2,1,3,2,2,3,0,2,0,3,1,0,0,1,2,0}; // vertices of facet i :
// j = 4 * i, vertices = facet_vertex_order[j,j+1,j+2] negative orientation

#ifdef __USE_POSIX
float delta_t(struct timespec *t1,struct timespec *t2) {
  float sec = (float)(t2->tv_sec - t1->tv_sec);
  float ms = (float)(t2->tv_nsec - t1->tv_nsec) / 1000000.; 
  float t = (sec * 1000. + ms) / 1000.;
  return t;
}
#else
// no clock_gettime on MacOS
float delta_t(time_t *t1,time_t *t2) {
  return (float)(*t2 - *t1);
}
#endif  // __USE_POSIX

void prompt(std::string s) {
  std::cout << s;
  //return;
  char ch;
  std::cin >> ch;
}
/**
 * @brief Find presence and arguments of a program option
 * @param opt The option string (for example -i)
 * @param op_type The argument(s) type : OPT_INT,OPT_FLOAT,OPT_STRING,OPT_NOARGS
 * @param i Index of argv entry to compare with opt
 * @param argc Nb of elements in argv
 * @param argv string list of program arguments
 * @param res Pointer to store the value or values associated to option
 * @param nbargs Number of args of option (default 1, ignored for OPT_NOARGS)
 * @return bool Whether option was found or not.
 **/
bool mygetopt(const char *opt,OP_TYPE op_type,int i,int argc, char **argv,void *res, int nbargs) {
  int n = 1;
  if(nbargs < 0 || nbargs > 10) return false;
  if(op_type == OPT_NOARGS) nbargs = 0;
  n += nbargs;
  if(i > (argc - n)) return false;
  if(strcmp(opt,argv[i]) != 0) return false;
  switch(op_type) {
  case OPT_INT:
    {
      int *ires = (int *)res;
      for(int j = 0;j < nbargs;j++)
	*(ires+j) = atoi(argv[i + j + 1]);
      return true;
    }
  case OPT_FLOAT:
    {
      float *fres = (float *)res;
      for(int j = 0;j < nbargs;j++)
	*(fres+j) = atof(argv[i+1+j]);
      return true;
      
    }
  case OPT_STRING:
    {
      char **sres = (char **)res;
      for(int j = 0;j < nbargs;j++)
	*(sres+j) = argv[i+1+j];
      return true;
    }
  case OPT_NOARGS:
    return true;
  }
}

/**
   @brief Sum of 2 statistics objects (to gather statistics from multiple threads)
 **/
void TrStats::sum(TrStats *stats2) {
  nb_tested_facets += stats2->nb_tested_facets;
  nb_tot_intersect += stats2->nb_tot_intersect;
  nb_intv += stats2->nb_intv;
  nb_int_edge += stats2->nb_int_edge;
  nb_test_facets += stats2->nb_test_facets;
  nb_tetra0 += stats2->nb_tetra0;
  nb_tested_lv0 += stats2->nb_tested_lv0;
  nb_rayons += stats2->nb_rayons;
  return;
}
