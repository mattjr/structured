#ifndef _COARSEN_AMG_H_
#define _COARSEN_AMG_H_

#include "defs.h"
#include "structs.h"
#include "read_params.h"



/*
This function gets an n*n fine matrix (L) and produces
an n*m interpolation matrix (A), using "weighted_interpolation"

*/

int construct_weighted_interpolation(
		struct vtx_data ** matrix, // the fine matrix
		struct vtx_data *** interpolation_matrix,
		int n,
		int *coarse_size, // number of coarse var's (what's called 'm')
		double min_connectivity, // each fine variable should be connected to
								 // the coarse variables with at least 'min_connectivity'
		double connectivity_growth, // in next passes increase the min_connectivity by 
									// this amount
		int max_nodes,  // max number of coarse var's (one kind of stopping condition)
		int max_passes  // max number of passes (another kind of stopping condition)
		);

/***************************************************************
 Construct interpolation matrix using the edge-contraction.
 In practice, for this kind of interpolation it would be much 
 faster to use the functions in the file "coarsen_match", which
 do not build the interpolation matrix explicitly
 ***************************************************************/


int construct_interpolation_matching(
		struct vtx_data ** matrix, // the fine matrix
		struct vtx_data *** interpolation_matrix,
		int n,
		int *coarse_size // number of coarse var's (what's called 'm')
		);

/*************************************************************************
A third kind of interpolation not mentioned in the paper.

The idea is to first make edge-contraction, and to lower the energy
using gauss-seidel sweeps. The result is a kind of weighted-interpolation
that does not have any representatives, but *all* nodes get their val's
from several coarse nodes.

AT THIS STAGE WE DO NOR RECOMMEND USING THIS METHOD!!!

**************************************************************************/

#define SELF_LOOP_WEIGHT 1.0
int construct_interpolation_matching_gs(
		struct vtx_data ** matrix, // the fine matrix
		struct vtx_data *** interpolation_matrix,
		int n,
		int *coarse_size, // number of coarse var's (what's called 'm')
		int matrix_edges
		);


/*
This function gets an n*n fine matrix (L) and an 
n*m interpolation matrix (A), 
and computes an m*m coarse matrix: A^T * L * A.
Space for the coarse matrix is allocated inside the function.

Notice that matrix dimensions start with 1 (not 0)

Another function that does the same job probably faster
is "compute_coarse_matrix_2steps" which appears below

*/

//#define MIN_WEIGHT 0
#define MIN_WEIGHT 0.001
int compute_coarse_matrix(
		struct vtx_data ** fine_matrix,
		struct vtx_data *** coarse_matrix,
		struct vtx_data ** interpolation_matrix,
		int n,
		int m,
		int * num_coarse_edges // not including self loops
		) ;

/* 
This function gets an n*n fine matrix (L) and an 
n*m interpolation matrix (A), 
and computes an m*m coarse matrix: A^T * L * A.
Space for the coarse matrix is allocated inside the function.

The computation is done in two steps: first compute A^T * L
and then (A^T * L) * A

This found to be faster in some cases than the above
 'compute_coarse_matrix'


Notice that matrix dimensions start with 1 (not 0)

*/

//#define MIN_WEIGHT 0.01
#define MIN_WEIGHT 0.001
int compute_coarse_matrix_2steps(
		struct vtx_data ** fine_matrix,
		struct vtx_data *** coarse_matrix,
		struct vtx_data ** interpolation_matrix,
		int n,
		int m,
		int num_fine_edges,
		int * num_coarse_edges // not including self loops
		) ;

// Compute x = Ay
// space for x is allocated outside the function
void interpolate_amg(
				struct vtx_data ** A, // interpolation matrix
				double * x, // n components vector
				double * y, // m components vector
				int n,
				int m,
				struct vtx_data ** graph // needed only for gs refinement
				);

// compute the 'mass' of each coarse node, which is the sum of its 
// respected column in the interpolation matrix - A, weighted with 
// masses of the fine nodes.
// the vector coarse_masses should be allocated for at least m+1 entries outside
// this function.
void compute_masses(struct vtx_data ** A, double * fine_masses, double * coarse_masses, int n, int m);


#endif
