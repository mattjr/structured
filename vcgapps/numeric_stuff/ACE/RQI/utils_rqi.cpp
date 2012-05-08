
/****************************************************************
 * This file is based on the code of Chaco by Bruce Hendrickson *
 * and Robert Leland at Sandia National Laboratories            *				
 ****************************************************************/


#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include	"../defs.h"
#include	"../structs.h"



/* Returns scalar product of two double n-vectors. */

/*double    dot(double   *vec1,int beg, int end, double  *vec2)
{
    int       i;
    double    sum;

    sum = 0.0;
    vec1 = vec1 + beg;
    vec2 = vec2 + beg;
    for (i = end - beg + 1; i; i--) {
	sum += (*vec1++) * (*vec2++);
    }
    return (sum);
}*/

/* Returns 2-norm of a double n-vector over range. */
/*double    norm(double *vec, int beg, int end)
{
 
    return (sqrt(dot(vec, beg, end, vec)));
}*/






/* Scaled add - fills double vec1 with vec1 + alpha*vec2 over range*/
/*void      scadd(double   *vec1, int  beg, int end, double fac, double   *vec2)
{
    int       i;

    vec1 = vec1 + beg;
    vec2 = vec2 + beg;
    for (i = end - beg + 1; i; i--) {
	(*vec1++) += fac * (*vec2++);
    }
}*/



/* Scale - fills vec1 with alpha*vec2 over range, double version */
/*void      vecscale(double   *vec1, int beg, int end, double alpha, double *vec2)
{
    int       i;

    vec1 += beg;
    vec2 += beg;
    for (i = end - beg + 1; i; i--) {
	(*vec1++) = alpha * (*vec2++);
    }
}*/


/* Print a double precision number with filtering format to file. */
void      doubleout_file(
						FILE     *outfile,              /* output file if not NULL */
						double    number,		/* argument to print */
						int       mode 			/* currently just one */
						)
{
    
    if (outfile == NULL) return;

    if (mode == 1) {
	if (fabs(number) < 100) {
	    //fprintf(outfile,"  %19.16f", number);
		fprintf(outfile,"  %e", number);
	}
	else {
	    fprintf(outfile,"  %19g", number);
	}
    }
}



/* Sparse linked A(matrix) times x(vector), double precision. */
void      splarax(
					double   *result,		/* result of matrix vector multiplication */
					struct    vtx_data **mat,	/* graph data structure */
					int       n,			/* number of rows/columns in matrix */
					double   *vec,			/* vector being multiplied by matrix */
					double   *vwsqrt,		/* square roots of vertex weights */
					double   *work		/* work vector from 1-n */
				)
{
    //extern int PERTURB;		/* perturb matrix? */
    //extern int NPERTURB;	/* if so, number of edges to perturb */
    //extern double PERTURB_MAX;	/* maximum value of perturbation */
    struct vtx_data *mat_i;	/* an entry in "mat" */
    double    sum;		/* sums inner product of matrix-row & vector */
    int      *colpntr;		/* loops through indices of nonzeros in a row */
    float    *wgtpntr;		/* loops through values of nonzeros */
    int       i, j;		/* loop counters */
    double   *wrkpntr;		/* loops through indices of work vector */
    double   *vwsqpntr;		/* loops through indices of vwsqrt */
    double   *vecpntr;		/* loops through indices of vec */
    double   *respntr;		/* loops through indices of result */
    int       last_edge;	/* last edge in edge list */
    void      perturb();

    if (vwsqrt == NULL) {		/* No vertex weights */
		if (mat[1]->ewgts == NULL) {	/* No edge weights */
			respntr = result;
			for (i = 1; i <= n; i++) {
				mat_i = mat[i];
				colpntr = mat_i->edges;
				last_edge = mat_i->nedges - 1;
				sum = last_edge * vec[*colpntr++];
				for (j = last_edge; j; j--) {
					sum -= vec[*colpntr++];
				}
				*(++respntr) = sum;
			}
		}
		else {				/* Edge weights */
			respntr = result;
			for (i = 1; i <= n; i++) {
				mat_i = mat[i];
				colpntr = mat_i->edges;
				wgtpntr = mat_i->ewgts;
				sum = 0.0;
				for (j = mat_i->nedges; j; j--) {
					sum -= *wgtpntr++ * vec[*colpntr++];
				}
				*(++respntr) = sum;	/* -sum if want -Ax */
			}
		}
		//if (PERTURB && NPERTURB > 0 && PERTURB_MAX > 0.0)
		//    perturb(result, vec);
    }
    else {				/* Vertex weights */
	if (vwsqrt != NULL) {
	    wrkpntr = work;
	    vecpntr = vec;
	    vwsqpntr = vwsqrt;
	    for (i = n; i; i--) {
			*(++wrkpntr) = *(++vecpntr) / *(++vwsqpntr);
	    }
	}

	if (mat[1]->ewgts == NULL) {	/* No edge weights. */
	    respntr = result;
	    for (i = 1; i <= n; i++) {
		mat_i = mat[i];
		colpntr = mat_i->edges;
		last_edge = mat_i->nedges - 1;
		sum = (last_edge) * work[*colpntr++];
		for (j = last_edge; j; j--) {
		    sum -= work[*colpntr++];
		}
		*(++respntr) = sum;
	    }
	}
	else {				/* Edge weights. */
	    respntr = result;
	    for (i = 1; i <= n; i++) {
		mat_i = mat[i];
		colpntr = mat_i->edges;
		wgtpntr = mat_i->ewgts;
		sum = 0.0;
		for (j = mat_i->nedges; j; j--) {
		    sum -= *wgtpntr++ * work[*colpntr++];
		}
		*(++respntr) = sum;	/* -sum if want -Ax */
	    }
	}
	//if (PERTURB && NPERTURB > 0 && PERTURB_MAX > 0.0)
	//    perturb(result, work);

	if (vwsqrt != NULL) {
	    respntr = result;
	    vwsqpntr = vwsqrt;
	    for (i = n; i; i--) {
		*(++respntr) /= *(++vwsqpntr);
	    }
	}
    }
}

double    norm(double *vec, int beg, int end);
/* Normalizes a double n-vector over range. */
double    normalize(double *vec, int beg, int end)
{
    int       i;
    double    scale;
    
    scale = norm(vec, beg, end);
    vec = vec + beg;
    for (i = end - beg + 1; i; i--) {
	*vec = *vec / scale;
	vec++;
    }
    return (scale);
}
