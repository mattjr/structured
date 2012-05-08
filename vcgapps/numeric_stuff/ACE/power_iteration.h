#ifndef _POWER_ITERATION_
#define _POWER_ITERATION_

#include "defs.h"

void refine_generalized_eigs(
    struct vtx_data ** graph, double * masses,
    int n, int dim, double ** eigs, double tol, bool initialize=false
) ;

#endif
	

