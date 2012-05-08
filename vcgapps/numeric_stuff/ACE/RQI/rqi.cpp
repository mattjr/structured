/* This software was developed by Bruce Hendrickson and Robert Leland   *
 * at Sandia National Laboratories under US Department of Energy        *
 * contract DE-AC04-76DP00789 and is copyrighted by Sandia Corporation. */

#include <iostream>
#include	<stdio.h>
#include	<math.h>
#include	"../defs.h"
#include	"../structs.h"
#include    "../read_params.h"
#include "utils_rqi.h"
#include "f2c.h"


int symmlq_(
    integer *n,
    doublereal *b, doublereal *r1, doublereal *r2, doublereal *v, 
    doublereal *w, doublereal *x, doublereal *y, doublereal *work,
    logical *checka, logical *goodb, logical *precon,
    doublereal *shift,
    integer *nout, integer *itnlim,
    doublereal *rtol,
    integer *istop, integer *itn,
    doublereal *anorm, doublereal  *acond, doublereal  *rnorm, doublereal *ynorm,
    doublereal *a, doublereal *vwsqrt, doublereal *orthlist, doublereal *macheps,
    doublereal *normxlim,
    integer *itnmin);

/* Perform Rayleigh Quotient Iteration */

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
)
{
    int       rqisteps;		/* # rqi rqisteps */
    double    res;		/* convergence quant for rqi */
    double    last_res;		/* res on previous rqi step */
    double    macheps;		/* machine precision calculated by symmlq */
    double    normxlim;		/* a stopping criteria for symmlq */
    double    normx;		/* norm of the solution vector */
    int       symmlqitns;	/* # symmlq itns */
    int       inv_it_steps;	/* intial steps of inverse iteration */
    int      itnmin;		/* symmlq input */
    double    shift, rtol;	/* symmlq input */
    int      precon, goodb, nout;	/* symmlq input */
    int      checka, intlim;	/* symmlq input */
    double    anorm, acond;	/* symmlq output */
    double    rnorm, ynorm;	/* symmlq output */
    int      istop, itn;	/* symmlq output */
    int      long_n;		/* copy of n for passing to symmlq */
    int       warning;		/* warning on possible misconvergence */
    double    factor;		/* ratio between previous res and new tol */
    double    minfactor;	/* minimum acceptable value of factor */
    int       converged;	/* has process converged yet? */
    double   *u;		/* name of vector being refined */
//  short    *old_assignment;	/* previous assignment vector */
//    short    *assgn_pntr;	/* pntr to assignment vector */
//    short    *old_assgn_pntr;	/* pntr to previous assignment vector */
//    int       assigndiff;	/* discrepancies between old and new assignment */
//    int       assigntol;	/* tolerance on convergence of assignment vector */
    int       first;		/* is this the first RQI step? */
//  int       i;		/* loop index */

    double original_tol = tol;  // New change
    
	    
    /* Initialize RQI loop */
    u = yvecs[index];
    splarax(y, A, n, u, vwsqrt, r1); // y=A*u
    shift = dot(u, 1, n, y); // shift =  (u,y)
    scadd(y, 1, n, -shift, u); // y=A*u-(u,A*u)*u   u*A*u is an estimation of the eigen value
    res = norm(y, 1, n);	/* eigen-residual */
    rqisteps = 0;		/* a counter */
    symmlqitns = 0;		/* a counter */

    /* Set invariant symmlq parameters */
    precon = FALSE;		/* FALSE until we figure out a good way */
    goodb = TRUE;		/* should be TRUE for this application */
    nout = 0;			/* set to 0 for no Symmlq output; 6 for lots */
    checka = FALSE;		/* if don't know by now, too bad */
    intlim = n;			/* set to enforce a maximum number of Symmlq itns */
    itnmin = 0;			/* set to enforce a minimum number of Symmlq itns */
    long_n = n;			/* type change for alint */

    
    /* Perform RQI */
    inv_it_steps = 2;
    warning = FALSE;
    factor = 10;
    minfactor = factor / 2;
    first = TRUE;
    if (relative_tolerance)
        tol = original_tol*shift*sqrt(double(n)); // New change
	
    if (res < tol)
	converged = TRUE;
    else
	converged = FALSE;
		

    while (!converged) {

	if (res / tol < 1.2) {
	    factor = max(factor / 2, minfactor);
	}
	rtol = res / factor;
	
	/* exit Symmlq if iterate is this large */
	normxlim = 1.0 / rtol;
	
 
	if (rqisteps < inv_it_steps) {
	    shift = initshift;
	}

	symmlq_(&long_n, &u[1], &r1[1], &r2[1], &v[1], &w[1], &x[1], &y[1],
            work, &checka, &goodb, &precon, &shift, &nout,
            &intlim, &rtol, &istop, &itn, &anorm, &acond,
            &rnorm, &ynorm, (double *) A, vwsqrt, (double *) orthlist,
            &macheps, &normxlim, &itnmin);
	symmlqitns += itn;
	normx = norm(x, 1, n);
	vecscale(u, 1, n, 1.0 / normx, x); // u=x/||x||
	splarax(y, A, n, u, vwsqrt, r1); // y=A*u
	shift = dot(u, 1, n, y); // estimate of eigenvalue: u_T*A*u
	scadd(y, 1, n, -shift, u); // y=A*u-shift*u
	last_res = res;
	res = norm(y, 1, n);
	if (res > last_res) {
	    warning = TRUE;
	}
	rqisteps++;

	if (relative_tolerance)
            tol = original_tol*::fabs(shift)*sqrt(double(n)); // New change

//        if(!(rqisteps & 1023)) {
//            std::cerr << "RQI: res = " << res << " tol = " << tol << std::endl ;
//        }

	if ((res < tol) || rqisteps > 100000)
	    converged = TRUE;

    }
    *evalest = shift;

    


    if (showStats) {
        if (warning) {
            fprintf(fpStatsFile,"WARNING: Residual convergence not monotonic; RQI may have misconverged.\n");
        }
        fprintf(fpStatsFile,"Eval ");
        doubleout_file(fpStatsFile,*evalest, 1);
        fprintf(fpStatsFile,"   RQI steps %d,  Symmlq iterations %d,  tol=%e\n", 
            rqisteps, symmlqitns,tol);
    }


    return rqisteps;
}
