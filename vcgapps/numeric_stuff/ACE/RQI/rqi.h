#ifndef _RQI_H_
#define _RQI_H_
#include "orthog.h"
#include "utils_rqi.h"

int      rqi(
struct vtx_data **A,		/* matrix/graph being analyzed */
double  **yvecs,		/* eigenvectors to be refined */
int       index,		/* index of vector in yvecs to be refined */
int       n,			/* number of rows/columns in matrix */
double   *r1, double *r2, double *v, double *w, double *x, double *y, double *work,	/* work space for symmlq */
double    tol,			/* error tolerance in eigenpair */
double    initshift,		/* initial shift */
double   *evalest,		/* returned eigenvalue */
double   *vwsqrt,		/* square roots of vertex weights */
struct orthlink *orthlist	/* lower evecs to orthogonalize against */
//int       cube_or_mesh,		/* 0 => hypercube, d => d-dimensional mesh */
//int       nsets		/* number of sets to divide into */
//short    *assignment;		/* set number of each vtx (length n+1) */
//int      *active;		/* space for nvtxs integers */
//int       mediantype;		/* which partitioning strategy to use */
//double   *goal;			/* desired set sizes */
//int       vwgt_max;		/* largest vertex weight */
//int       ndims;		/* dimensionality of partition */
);


void      y2x(
				double  **xvecs,		/* pointer to list of x-vectors */
				int       ndims,		/* number of divisions to make (# xvecs) */
				int       nmyvtxs,		/* number of vertices I own (lenght xvecs) */
				double   *wsqrt		/* sqrt of vertex weights */
			);


void      x2y(
				double  **yvecs,		/* pointer to list of y-vectors */
				int       ndims,		/* number of divisions to make (# yvecs) */
				int       nmyvtxs,		/* number of vertices I own (lenght yvecs) */
				double   *wsqrt 		/* sqrt of vertex weights */
			);

void refine_generalized_eigs_RQI(struct vtx_data ** graph, double * masses, int nvtxs, int ndims, 
							     double ** evecs, double rqi_tol, bool IsWeightedGraph);

#endif
