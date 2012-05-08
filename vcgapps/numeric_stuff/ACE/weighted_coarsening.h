#ifndef _WEIGHTED_COARSENING_H_
#define _WEIGHTED_COARSENING_H_

#include "defs.h"
#include "structs.h"

void choose_representatives(
		struct vtx_data ** matrix, // the fine matrix
		int n,
		int * fine2coarse, // if v is a representative then fine2coarse[v] is its new name, otherwise
						// fine2coarse[v]==0
		double * degrees, // amount of connectivity of each fine node to rep's
		int *num_representatives, // number of coarse var's (what's called 'm')
		double min_connectivity, // each fine variable should be connected to
								 // the coarse variables with at least 'min_connectivity'
		double connectivity_growth, // in next passes increase the min_connectivity by 
									// this amount
		int max_nodes,  // max number of coarse var's (one kind of stopping condition)
		int max_passes  // max number of passes (another kind of stopping condition)
		);

#endif
