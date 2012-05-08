#include	<stdio.h>
#include	<stdlib.h>
#include	<time.h>
#undef min
#undef max
#include "defs.h"
#include "structs.h"
#include "read_params.h"
#include "utils.h"
#include "amg_eigen_computation.h"
#include "graph_generator.h"

unsigned long coarsening_time=0, power_iteration_time=0, rqi_time=0;
int total_negative_edges=0, total_coarse_edges=0;
double max_neg_percentage=0;

int IsWeightedGraph = TRUE;
