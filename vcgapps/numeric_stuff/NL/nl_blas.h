/*
 *  OpenNL: Numerical Library
 *  Copyright (C) 2004 Bruno Levy
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  If you modify this software, you should include a notice giving the
 *  name of the person performing the modification, the date of modification,
 *  and the reason for such modification.
 *
 *  Contact: Bruno Levy
 *
 *     levy@loria.fr
 *
 *     ISA Project
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs. 
 */

#include <numeric_stuff/NL/nl_private.h>

#ifndef __NL_BLAS__
#define __NL_BLAS__

#ifndef NL_FORTRAN_WRAP
#define NL_FORTRAN_WRAP(x) f2c_##x
#endif


/***********************************************************************************/
/* C wrappers for BLAS routines */

/* x <- a*x */
void dscal( int n, double alpha, double *x, int incx ) ;

/* y <- x */
void dcopy( 
    int n, double *x, int incx, double *y, int incy 
) ;

/* y <- a*x+y */
void daxpy( 
    int n, double alpha, double *x, int incx, double *y,
    int incy 
) ;

/* returns x^T*y */
double ddot( 
    int n, double *x, int incx, double *y, int incy 
) ;

/** returns |x|_2 */
double dnrm2( int n, double *x, int incx ) ;

typedef enum { NoTranspose=0, Transpose=1, ConjugateTranspose=2 } MatrixTranspose ;
typedef enum { UpperTriangle=0, LowerTriangle=1 } MatrixTriangle ;
typedef enum { UnitTriangular=0, NotUnitTriangular=1 } MatrixUnitTriangular ;

/** x <- A^{-1}*x,  x <- A^{-T}*x */
void dtpsv( 
    MatrixTriangle uplo, MatrixTranspose trans,
    MatrixUnitTriangular diag, int n, double *AP,
    double *x, int incx 
) ;

/** y <- alpha*A*x + beta*y,  y <- alpha*A^T*x + beta*y,   A-(m,n) */
void dgemv( 
    MatrixTranspose trans, int m, int n, double alpha,
    double *A, int ldA, double *x, int incx,
    double beta, double *y, int incy 
) ;

#endif
