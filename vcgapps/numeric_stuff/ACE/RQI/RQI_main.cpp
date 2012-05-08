/****************************************************************
 *																*
 *	     Rayleigh Quotient Iteration refinement					*
 *       --------------------------------------					*
 *																*
 *     Based on the code of Chaco by Bruce Hendrickson			*
 *     and Robert Leland at Sandia National Laboratories        *				
 ****************************************************************/


#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include <iostream>
#include "../defs.h"
#include "../structs.h"
#include "rqi.h"


// We don't normalize the matrix, since the matrix is accessed via the 
// function 'aprod()' that does this normalization on-the-fly

void refine_generalized_eigs_RQI(
    struct vtx_data ** graph, double * masses, int nvtxs, int ndims, 
    double ** evecs, double rqi_tol, bool IsWeightedGraph
) {

    int i,j;

//    std::cerr << "refine_generalized_eigs_RQI, ndims="  << ndims << std::endl ;

    // make vector of masses sqrt's
    double * masses_sqrt = (double *) malloc((unsigned) (nvtxs + 1) * sizeof(double));
    for (i=1; i<=nvtxs; i++) {
        masses_sqrt[i]=sqrt(masses[i]);
    }
		
    x2y(evecs, ndims, nvtxs, masses_sqrt);

    // for accelerating running time, do not use weights when they are uniform
    float * tmp_ewgts;
    if (!IsWeightedGraph) {
        tmp_ewgts = graph[1]->ewgts;
        graph[1]->ewgts = NULL;
    }

    /* orthogonalization against masses_sqrt, can be skipped since this constraint
       is preserved by the coarsening: */
    for (i = 1; i <= ndims; i++)
        orthogvec(evecs[i], 1, nvtxs, masses_sqrt);
	
    /* Allocate space that will be needed in RQI. */
    double * r1 = (double *) malloc((unsigned) 7 * (nvtxs + 1) * sizeof(double));
    double * r2 = &r1[nvtxs + 1];
    double * v = &r1[2 * (nvtxs + 1)];
    double * w = &r1[3 * (nvtxs + 1)];
    double * x = &r1[4 * (nvtxs + 1)];
    double * y = &r1[5 * (nvtxs + 1)];
    double *work = &r1[6 * (nvtxs + 1)];

	
    double initshift = 0;
    struct orthlink * orthlist = NULL;
    struct orthlink *newlink;	  
    double evalest;		/* eigenvalue estimate returned by RQI */
    double alpha;
    double vec_norm;

    for (i = 1; true ; i++) {  // Yehuda: Originally, here was i < ndims
        normalize(evecs[i], 1, nvtxs);
        rqi(
            graph, evecs, i, nvtxs, r1, r2, v, w, x, y, work,
            rqi_tol, initshift, &evalest, masses_sqrt, orthlist
        ) ;


//        std::cerr << "i= " << i << " ndmis=" << ndims << std::endl ;
        if (i==ndims) // Yehuda: I added this
            break;

        // Now orthogonalize higher evecs against this one. 
        vec_norm = dot(evecs[i], 1, nvtxs, evecs[i]);
        for (j = i + 1; j <= ndims; j++) {
            alpha = -dot(evecs[j], 1, nvtxs, evecs[i]) / vec_norm;
            scadd(evecs[j], 1, nvtxs, alpha, evecs[i]);
        }

        // Now prepare for next pass through loop. 
        initshift = evalest;
        newlink = makeorthlnk();
        newlink->vec = evecs[i];
        newlink->pntr = orthlist;
        orthlist = newlink;
    }
//    std::cerr << "RQI: exited loop1" << std::endl ;
	
    y2x(evecs, ndims, nvtxs, masses_sqrt); // Yehuda: I added this line

	
    /* Free the space allocated for RQI. */
    while (orthlist != NULL) {
        newlink = orthlist->pntr;
        free( orthlist);
        orthlist = newlink;
    }
//    std::cerr << "RQI: exited orthlist loop" << std::endl ;
    free(r1);
    if (masses_sqrt != NULL)
        free(masses_sqrt);

    if (!IsWeightedGraph) {
        graph[1]->ewgts = tmp_ewgts;
    }
}

