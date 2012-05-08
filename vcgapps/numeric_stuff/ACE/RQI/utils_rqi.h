#ifndef _UTILS_RQI_H_
#define _UTILS_RQI_H_

#include	"../defs.h"
#include	"../structs.h"
#include <stdio.h>



/* Returns scalar product of two double n-vectors. */
double    dot(double   *vec1,int beg, int end, double  *vec2);

/* Returns 2-norm of a double n-vector over range. */
double    norm(double *vec, int beg, int end);

/* Sparse linked A(matrix) times x(vector), double precision. */
void      splarax(
					double   *result,		/* result of matrix vector multiplication */
					struct    vtx_data **mat,	/* graph data structure */
					int       n,			/* number of rows/columns in matrix */
					double   *vec,			/* vector being multiplied by matrix */
					double   *vwsqrt,		/* square roots of vertex weights */
					double   *work		/* work vector from 1-n */
				);

/* Scaled add - fills double vec1 with vec1 + alpha*vec2 over range*/
void      scadd(double   *vec1, int  beg, int end, double fac, double   *vec2);


/* Scale - fills vec1 with alpha*vec2 over range, double version */
void      vecscale(double   *vec1, int beg, int end, double alpha, double *vec2);

/* Print a double precision number with filtering format to file. */
void      doubleout_file(
						FILE     *outfile,              /* output file if not NULL */
						double    number,		/* argument to print */
						int       mode 			/* currently just one */
						);

/* Normalizes a double n-vector over range. */
double    normalize(double *vec, int beg, int end);

#endif
