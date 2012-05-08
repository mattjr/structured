/*
 *  OGF/Graphite: Geometry and Graphics Programming Library + Utilities
 *  Copyright (C) 2000 Bruno Levy
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

#include <OGF/math/common/common.h>
#include <OGF/basic/types/types.h>
#include <math.h>

#ifndef __OGF_MATH_NUMERIC_BLAS__
#define __OGF_MATH_NUMERIC_BLAS__

static inline bool has_nan(double* p, int n) {
    for(int i=0; i<n; i++) {
        if(::OGF::Numeric::is_nan(p[i])) {
            return true ;
        }
    }
    return false ;
}

// BLAS level 1 wrappers
namespace BLAS {

    /** generates a plane rotation */
    void MATH_API drotg( double *a, double *b, double *c, double *s ) ;
    
    /** applies a plane rotation */
    void MATH_API drot( 
        int n, double *x, int incx, double *y, int incy, double c,
        double s 
    ) ;

    /** x <-> y */
    void MATH_API dswap( int n, double *x, int incx, double *y, int incy ) ;

    /** x <- a*x */
    void MATH_API dscal( int n, double alpha, double *x, int incx ) ;

    /** y <- x */
    void MATH_API dcopy( 
        int n, const double *x, int incx, double *y, int incy 
    ) ;

    /** y <- a*x+y */
    void MATH_API daxpy( 
        int n, double alpha, const double *x, int incx, double *y,
        int incy 
    ) ;

    /** returns x^T*y */
    double MATH_API ddot( 
        int n, const double *x, int incx, const double *y, int incy 
    ) ;

    /** returns |x|_2 */
    double MATH_API dnrm2( int n, const double *x, int incx ) ;

    /** returns |x|_1 */
    double MATH_API dasum( int n, const double *x, int incx ) ;

    /** returns the first k such that |x_k| = max|x_i| */
    int MATH_API idamax( int n, const double *x, int incx ) ;
}

// BLAS level 2 wrappers
namespace BLAS {
    enum MatrixTranspose { NoTranspose=0, Transpose=1, ConjugateTranspose=2 } ;
    enum MatrixTriangle { UpperTriangle=0, LowerTriangle=1 } ;
    enum MatrixUnitTriangular { UnitTriangular=0, NotUnitTriangular=1 } ;

    /** y <- alpha*A*x + beta*y,  y <- alpha*A^T*x + beta*y,   A-(m,n) */
    void MATH_API dgemv( 
        MatrixTranspose trans, int m, int n, double alpha,
        const double *A, int ldA, const double *x, int incx,
        double beta, double *y, int incy 
    ) ;

    /** y <- alpha*A*x + beta*y,  y <- alpha*A^T*x + beta*y,   A-(m,n) */
    void MATH_API dgbmv( 
        MatrixTranspose trans, int m, int n, int kl, int ku, double alpha,
        const double *A, int ldA, const double *x, int incx, double beta,
        double *y, int incy 
    ) ;

    /** y <- alpha*A*x + beta*y */
    void MATH_API dsymv( 
        MatrixTriangle uplo, int n, double alpha, const double *A, int ldA,
        const double *x, int incx, double beta, double *y, int incy 
    ) ;

    /** y <- alpha*A*x + beta*y */
    void MATH_API dsbmv( 
        MatrixTriangle uplo, int n, int k, double alpha, double *A,
        int ldA, const double *x, int incx, double beta, double *y,
        int incy 
    ) ;

    /** y <- alpha*A*x + beta*y */
    void MATH_API dspmv( 
        MatrixTriangle uplo, int n, double alpha, const double *AP,
        const double *x, int incx, double beta, double *y, int incy 
    ) ;

    /** x <- A*x,  x <- A^T*x */
    void MATH_API dtrmv( 
        MatrixTriangle uplo, MatrixTranspose trans,
        MatrixUnitTriangular diag, int n, const double *A, int ldA,
        double *x, int incx 
    ) ;

    /** x <- A*x,  x <- A^T*x */
    void MATH_API dtbmv( 
        MatrixTriangle uplo, MatrixTranspose trans,
        MatrixUnitTriangular diag, int n, int k, const double *A, int ldA,
        double *x, int incx 
    ) ;

    /** x <- A*x,  x <- A^T*x */
    void MATH_API dtpmv( 
        MatrixTriangle uplo, MatrixTranspose trans,
        MatrixUnitTriangular diag, int n, const double *AP,
        double *x, int incx 
    ) ;

    /** x <- A^{-1}*x,  x <- A^{-T}*x */
    void MATH_API dtrsv( 
        MatrixTriangle uplo, MatrixTranspose trans,
        MatrixUnitTriangular diag, int n, const double *A, int ldA,
        double *x, int incx 
    ) ;

    /** x <- A^{-1}*x,  x <- A^{-T}*x */
    void MATH_API dtbsv( 
        MatrixTriangle uplo, MatrixTranspose trans,
        MatrixUnitTriangular diag, int n, int k, const double *A, int ldA,
        double *x, int incx 
    ) ;

    /** x <- A^{-1}*x,  x <- A^{-T}*x */
    void MATH_API dtpsv( 
        MatrixTriangle uplo, MatrixTranspose trans,
        MatrixUnitTriangular diag, int n, const double *AP,
        double *x, int incx 
    ) ;

    /** A <- alpha*x*y^T + A,   A-(m,n) */
    void MATH_API dger( 
        int m, int n, double alpha, const double *x, int incx,
        const double *y, int incy, double *A, int ldA 
    ) ;

    /** A <- alpha*x*x^T + A */
    void MATH_API dsyr( 
        MatrixTriangle uplo, int n, double alpha, const double *x,
        int incx, double *A, int ldA 
    ) ;

    /** A <- alpha*x*x^T + A */
    void MATH_API dspr( 
        MatrixTriangle uplo, int n, double alpha, const double *x,
        int incx, double *AP 
    ) ;

    /** A <- alpha*x*y^T + alpha*y*x^T + A */
    void MATH_API dsyr2( 
        MatrixTriangle uplo, int n, double alpha, const double *x,
        int incx, const double *y, int incy, double *A, int ldA 
    ) ;

    /** A <- alpha*x*y^T + alpha*y*x^T + A */
    void MATH_API dspr2( 
        MatrixTriangle uplo, int n, double alpha, const double *x,
        int incx, const double *y, int incy, double *AP 
    ) ;

}

#endif 
