#ifndef _READ_PARAMS_H
#define _READ_PARAMS_H 


typedef enum{unknown,grid,folded_grid,partial_grid,partial_folded_grid,
			 circle,tree,torus,cylinder,sierpinski,hypercube,star} GraphType;
typedef enum{weighted, contraction, non_amg} CoarseningMethod;


extern CoarseningMethod  coarseningMethod;
extern int numPasses;
extern double minConnectivity;
extern double connectivityGrowth;
extern int showStats;
extern FILE * fpStatsFile;
extern double power_iteration_tol;
extern double rqi_tol;
extern int relative_tolerance;
extern int IsWeightedGraph;
extern int levels_between_refinement;
extern int MAKE_VWGTS;
	



#define MAX_LEN 100
int read_parameters(char * filename, GraphType *graphType, int *graphSize1, int *graphSize2,
				char * graphFilename, char * coordinatesFilename, int *useGdrawLayoutFormat,
				int * ndims, int ** pdims2show, int *minSizeToCoarsen);

#endif 
