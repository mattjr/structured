
/****************************************************************
 * This file is based on the code of Chaco by Bruce Hendrickson *
 * and Robert Leland at Sandia National Laboratories            *				
 ****************************************************************/


#include <malloc.h>
#include "orthog.h"
#include "../structs.h"
#include "utils_rqi.h"

/* Orthogonalize a double vector to all one's */
void      orthog1(double   *x, int  beg, int end)
{
    int       i;
    double   *pntr;
    double    sum;
    int       len;

    len = end - beg + 1;
    sum = 0.0;
    pntr = x + beg;
    for (i = len; i; i--) {
	sum += *pntr++;
    }
    sum /= len;
    pntr = x + beg;
    for (i = len; i; i--) {
	*pntr++ -= sum;

    }
}


void      orthogvec(
							double   *vec1,		     /* vector to be orthogonalized */
							int       beg, int end,	 /* start and stop range for vector */
							double   *vec2			/* vector to be orthogonalized against */
							)
{
    double    alpha;
    
    alpha = -dot(vec1, beg, end, vec2) / dot(vec2, beg, end, vec2);
    scadd(vec1, beg, end, alpha, vec2);
}



void   orthogonalize(
							double   *vec,			/* vector to be orthogonalized */
							int       n,			/* length of the columns of orth */
							struct orthlink *orthlist	/* set of vectors to orthogonalize against */
							)
{
    struct orthlink *curlnk;
    
    curlnk = orthlist;
    while (curlnk != NULL) {
	orthogvec(vec, 1, n, curlnk->vec);
	curlnk = curlnk->pntr;
    }
}


/* Allocate space for new orthlink, double version. */
struct orthlink *makeorthlnk()
{
    struct orthlink *newlnk;
    
    newlnk = (struct orthlink *) malloc(sizeof(struct orthlink));
    return (newlnk);
}
