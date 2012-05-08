#ifndef _ORTHOG_H_
#define _ORTHOG_H_

struct orthlink {
	int depth;		/* bottom of list is 0, previous is 1 etc */
	int index;		/* position in list of ritz vals (i index) */
	double ritzval;		/* good ritz value */		
	double betaji;		/* residual bound on good ritz pair */ 
	double tau;		/* from orthogonality recursion */
	double prevtau;		/* from orthogonality recursion */
	double *vec;		/* vector to orthogonalize against */
	struct orthlink *pntr;	/* pointer to next link */
};


/* Orthogonalize a double vector to all one's */
void      orthog1(double   *x, int  beg, int end);
void      orthogvec(
							double   *vec1,		     /* vector to be orthogonalized */
							int       beg, int end,	 /* start and stop range for vector */
							double   *vec2			/* vector to be orthogonalized against */
							);
void   orthogonalize(
							double   *vec,			/* vector to be orthogonalized */
							int       n,			/* length of the columns of orth */
							struct orthlink *orthlist	/* set of vectors to orthogonalize against */
							);

struct orthlink *makeorthlnk();

#endif
