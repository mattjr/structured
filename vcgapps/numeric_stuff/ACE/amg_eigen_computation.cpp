#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include <time.h>
#include "defs.h"
#include "structs.h"
#include "weighted_coarsening.h"
#include "read_params.h"
#include "power_iteration.h"
#include "utils.h"
#include "coarsen_amg.h"
#include "coarsen_match.h"
#include "RQI/rqi.h"
#include <iostream>


/********************************************

  AMG computation of lowest eigenvectors
  of the Laplacian.

  Coarsen until nvtxs <= vmax, compute and 
  uncoarsen.

*********************************************/

void   amg_eigen_computation(
    struct vtx_data **graph,	/* array of vtx data for graph */
    int       nvtxs,		/* number of vertices in graph */
    int       nedges,		/* number of edges in graph - needed only for debug info*/
    double  **evecs,		/* eigenvectors returned */
    int       ndims,		/* number of eigenvectors to calculate */
    int       vmax,			/* largest subgraph to stop coarsening */
    int       step,			/* current step number */
    double *masses,          /* 'masses' of the fine nodes */
    int       nstep		/* number of coarsenings between PI steps */
)
{
    extern int COARSEN_VWGTS;	/* use vertex weights while coarsening? */
    extern int COARSEN_EWGTS;	/* use edge weights while coarsening? */
    extern unsigned long power_iteration_time, rqi_time;
    struct vtx_data **cgraph;	/* array of vtx data for coarsened graph */
    struct vtx_data **A;		/* interpolation matrix */
    double   **cevecs;	/* eigenvectors for coarse graph */
    int      *v2cv;		/* mapping from vertices to coarse vtxs */
    int       cnvtxs;		/* number of vertices in coarsened graph */
    int       cnedges;		/* number of edges in coarsened graph */
    int       i;		/* loop counters */
    double *cmasses;		/* masses of coarse nodes */
	
    typedef double * double_ptr;

    extern CoarseningMethod  coarseningMethod;

    unsigned long tmp_time;
    extern unsigned long  coarsening_time;
    			

    if (showStats) {
        fprintf(fpStatsFile,"<Entering coarsen AMG, step=%d, nvtxs=%d, nedges=%d>\n",
            step, nvtxs, nedges);
    }
    printf("<Entering coarsen AMG, step=%d, nvtxs=%d, nedges=%d>\n",
        step, nvtxs, nedges);

    
    /* Is problem small enough to solve? */
    if (nvtxs <= vmax) {
        double old_power_iteration_tol=power_iteration_tol;
        if (power_iteration_tol<=0)
            power_iteration_tol=1e-8;
        // To add stability, we compute here more eigenvectors,
        // so we can choose from them the 'ndims' largest
        /*double ** tmp_evecs = new double_ptr[2*ndims+1];
          for (i=1; i<=2*ndims; i++) 
          tmp_evecs[i]= new double[nvtxs+1];*/

        //refine_generalized_eigs(graph, masses, nvtxs, 2*ndims, tmp_evecs, 1-power_iteration_tol,true); 
        refine_generalized_eigs(graph, masses, nvtxs, ndims, evecs, 1-power_iteration_tol,true); 

        /*for (i=1; i<=ndims; i++) 
          for (j=1; j<=nvtxs; j++)
          evecs[i][j]= tmp_evecs[i][j];

          for (i=1; i<=2*ndims; i++) 
          delete [] tmp_evecs[i];
          delete [] tmp_evecs;*/
        power_iteration_tol=old_power_iteration_tol;
        return;
    }
	
    /* Otherwise I have to coarsen. */

    tmp_time=clock();

    if (coarseningMethod==contraction) {
        if (step==0 && showStats) 
            fprintf(fpStatsFile,"Interpolation method: Contraction\n");
        coarsen_match(graph, nvtxs, nedges, &cgraph, &cnvtxs, &cnedges, &v2cv, step>0 || IsWeightedGraph, masses, &cmasses);
    }
	
    else {
        if (step==0 && showStats) 
            fprintf(fpStatsFile,"Interpolation method: Weighted\n");
		
        construct_weighted_interpolation(graph, &A, nvtxs, &cnvtxs, minConnectivity,
            connectivityGrowth, nvtxs ,numPasses);
	
        //compute_coarse_matrix(graph, &cgraph, A, nvtxs, cnvtxs, &cnedges);
        compute_coarse_matrix_2steps(graph, &cgraph, A, nvtxs, cnvtxs, nedges, &cnedges);
		
        cmasses = new double[cnvtxs+1];
        compute_masses(A, masses, cmasses, nvtxs, cnvtxs); 
    }
    coarsening_time+=clock()-tmp_time;

	

    /* Create space for coarse evecs. */
    cevecs = new double_ptr[ndims+1];
    for (i = 1; i <= ndims; i++) {
        cevecs[i] = new double[cnvtxs + 1];
    }

		
   
    /* Now recurse on coarse subgraph. */
    amg_eigen_computation(cgraph, cnvtxs, cnedges, cevecs, ndims, vmax,  step + 1, cmasses, nstep);
    
    delete [] cmasses;

    tmp_time = clock();

    if (coarseningMethod==contraction) {
        interpolate_match(evecs, cevecs, ndims, graph, nvtxs, v2cv);
        delete [] v2cv;
        v2cv=NULL;
    }
	
    else {
        for (i = 1; i <= ndims; i++) {
            interpolate_amg(A, evecs[i], cevecs[i], nvtxs, cnvtxs,graph);
        }
        free_graph(A);
        A=NULL;
    }

    // Power iteration refinements:

    tmp_time=clock();
    if (!(step % nstep) && power_iteration_tol>0)
        refine_generalized_eigs(graph, masses, nvtxs, ndims, evecs, 1-power_iteration_tol); 
    power_iteration_time+=clock()-tmp_time;


    // Rayleigh Quotient Iteration refinement (Optional):
    // (Do RQI each nstep stages)
    if (!(step % nstep) && rqi_tol>0) {
        if (showStats) 
            fprintf(fpStatsFile,"Entering RQI\n");
        tmp_time=clock();
		
//        std::cerr << "refine_generalized_eigs_RQI - tol=" << rqi_tol << std::endl ;
        refine_generalized_eigs_RQI(
            graph, masses, nvtxs, ndims, 
            evecs, rqi_tol, IsWeightedGraph || step!=0
        ) ;
		
        rqi_time+=clock()-tmp_time;
        if (showStats) 
            fprintf(fpStatsFile,"Ending RQI\n");
		

    } // end "if (step%nstep)"

	
 
    if (showStats) {
        fprintf(fpStatsFile," Leaving coarsen AMG, step=%d\n\n", step);
    }
    printf(" Leaving coarsen AMG, step=%d\n", step);


    /* Free the space that was allocated. */
    
    for (i = ndims; i > 0; i--)
        delete [] cevecs[i];
    if (coarseningMethod==contraction) 
        free_graph(cgraph);
    else
        free_graph_ex(cgraph, cnvtxs);

    //if (step==0)
    //	avg_rqi_steps = (double)total_rqi_steps /(recursion_depth*ndims);
}
