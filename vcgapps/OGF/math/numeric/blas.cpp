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

#include <OGF/math/numeric/blas.h>

#define OGF_FORTRAN(x) f2c_##x


// BLAS level 1 prototypes
extern "C" {
    void OGF_FORTRAN(drotg)( double *a, double *b, double *c, double *s ) ;
    void OGF_FORTRAN(drot)( 
        int *n, double *x, int *incx, double *y, int *incy,
        double *c, double *s 
    ) ;
    void OGF_FORTRAN(dswap)( 
        const int *n, double *x, const int *incx, double *y, const int *incy 
    ) ;
    void OGF_FORTRAN(dscal)( 
        const int *n, const double *alpha, double *x, const int *incx 
    ) ;
    void OGF_FORTRAN(dcopy)( 
        const int *n, const double *x, const int *incx, double *y,
        const int *incy 
    ) ;
    void OGF_FORTRAN(daxpy)( 
        const int *n, const double *alpha, const double *x,
        const int *incx, double *y, const int *incy 
    ) ;
    double OGF_FORTRAN(ddot)( 
        const int *n, const double *x, const int *incx, const double *y,
        const int *incy 
    ) ;
    double OGF_FORTRAN(dnrm2)( const int *n, const double *x, const int *incx ) ;
    double OGF_FORTRAN(dasum)( const int *n, const double *x, const int *incx ) ;
    int OGF_FORTRAN(idamax)( const int *n, const double *x, const int *incx ) ;
}

// BLAS level 1 wrappers
namespace BLAS {

    /** generates a plane rotation */
    void drotg( double *a, double *b, double *c, double *s ) {
        OGF_FORTRAN(drotg)(a,b,c,s);
    }

    
    /** applies a plane rotation */
    void drot( 
        int n, double *x, int incx, double *y, int incy, double c,
        double s 
    ) {
        OGF_FORTRAN(drot)(&n,x,&incx,y,&incy,&c,&s);
    }


    /** x <-> y */
    void dswap( int n, double *x, int incx, double *y, int incy ) {
        OGF_FORTRAN(dswap)(&n,x,&incx,y,&incy);
    }


    /** x <- a*x */
    void dscal( int n, double alpha, double *x, int incx ) {
        OGF_FORTRAN(dscal)(&n,&alpha,x,&incx);
    }

    /** y <- x */
    void dcopy( 
        int n, const double *x, int incx, double *y, int incy 
    ) {
        OGF_FORTRAN(dcopy)(&n,x,&incx,y,&incy);
    }


    /** y <- a*x+y */
    void daxpy( 
        int n, double alpha, const double *x, int incx, double *y,
        int incy 
    ) {
        OGF_FORTRAN(daxpy)(&n,&alpha,x,&incx,y,&incy);
    }

    /** returns x^T*y */
    double ddot( 
        int n, const double *x, int incx, const double *y, int incy 
    ) {
        return OGF_FORTRAN(ddot)(&n,x,&incx,y,&incy);
    }

    /** returns |x|_2 */
    double dnrm2( int n, const double *x, int incx ) {
        double d=0.;
        while ( n-- ) {
            d+=(*x)*(*x); 
            x+=incx ;
        }
        return sqrt(d);
//        return dnrm2_(&n,x,&incx);
    }

    /** returns |x|_1 */
    double dasum( int n, const double *x, int incx ) {
        return OGF_FORTRAN(dasum)(&n,x,&incx);
    }

    /** returns the first k such that |x_k| = max|x_i| */
    int idamax( int n, const double *x, int incx ) {
        return OGF_FORTRAN(idamax)(&n,x,&incx);
    }
}

// BLAS level 2 prototypes
extern "C" {

    void OGF_FORTRAN(dgemv)( 
        const char *trans, const int *m, const int *n,
        const double *alpha, const double *A, const int *ldA,
        const double *x, const int *incx,
        const double *beta, double *y, const int *incy 
    );

    void OGF_FORTRAN(dgbmv)( 
        const char *trans, const int *m, const int *n, const int *kl,
        const int *ku, const double *alpha, const double *A,
        const int *ldA, const double *x, const int *incx,
        const double *beta, double *y, const int *incy 
    );

    void OGF_FORTRAN(dsymv)( 
        const char *uplo, const int *n, const double *alpha,
        const double *A, const int *ldA, const double *x, const int *incx,
        const double *beta, double *y, const int *incy 
    );

    void OGF_FORTRAN(dsbmv)( 
        const char *uplo, const int *n, const int *k, const double *alpha,
        const double *A, const int *ldA, const double *x, const int *incx,
        const double *beta, double *y, const int *incy 
    ) ;

    void OGF_FORTRAN(dspmv)( 
        const char *uplo, const int *n, const double *alpha,
        const double *AP, const double *x, const int *incx,
        const double *beta, double *y, const int *incy 
    ) ;

    void OGF_FORTRAN(dtrmv)( 
        const char *uplo, const char *trans, const char *diag,
        const int *n, const double *A, const int *ldA,
        double *x, const int *incx  
    ) ;

    void OGF_FORTRAN(dtbmv)( 
        const char *uplo, const char *trans, const char *diag,
        const int *n, const int *k, const double *A, const int *ldA,
        double *x, const int *incx 
    ) ;

    void OGF_FORTRAN(dtpmv)( 
        const char *uplo, const char *trans, const char *diag,
        const int *n, const double *AP, double *x, const int *incx 
    ) ;

    void OGF_FORTRAN(dtrsv)( 
        const char *uplo, const char *trans, const char *diag,
        const int *n, const double *A, const int *ldA,
        double *x, const int *incx 
    ) ;

    void OGF_FORTRAN(dtbsv)( 
        const char *uplo, const char *trans, const char *diag,
        const int *n, const int *k, const double *A, const int *ldA,
        double *x, const int *incx 
    ) ;

    void OGF_FORTRAN(dtpsv)( 
        const char *uplo, const char *trans, const char *diag,
        const int *n, const double *AP, double *x, const int *incx 
    ) ;

    void OGF_FORTRAN(dger)( 
        const int *m, const int *n, const double *alpha, const double *x,
        const int *incx, const double *y, const int *incy, double *A,
        const int *ldA 
    ) ;

    void OGF_FORTRAN(dsyr)( 
        const char *uplo, const int *n, const double *alpha,
        const double *x, const int *incx, double *A, const int *ldA 
    ) ;

    void OGF_FORTRAN(dspr)( 
        const char *uplo, const int *n, const double *alpha,
        const double *x, const int *incx, double *AP 
    ) ;

    void OGF_FORTRAN(dsyr2)( 
        const char *uplo, const int *n, const double *alpha,
        const double *x, const int *incx, const double *y,
        const int *incy, double *A, const int *ldA 
    ) ;

    void OGF_FORTRAN(dspr2)( 
        const char *uplo, const int *n, const double *alpha,
        const double *x, const int *incx, const double *y,
        const int *incy, double *AP 
    ) ;
}

// BLAS level 2 wrappers
namespace BLAS {

    /** y <- alpha*A*x + beta*y,  y <- alpha*A^T*x + beta*y,   A-(m,n) */
    void dgemv( 
        MatrixTranspose trans, int m, int n, double alpha,
        const double *A, int ldA, const double *x, int incx,
        double beta, double *y, int incy 
    ) {
        const char *T[3] = { "N", "T", 0 };
        OGF_FORTRAN(dgemv)(T[(int)trans],&m,&n,&alpha,A,&ldA,x,&incx,&beta,y,&incy);
    }


    /** y <- alpha*A*x + beta*y,  y <- alpha*A^T*x + beta*y,   A-(m,n) */
    void dgbmv( 
        MatrixTranspose trans, int m, int n, int kl, int ku, double alpha,
        const double *A, int ldA, const double *x, int incx, double beta,
        double *y, int incy 
    ) {
        const char *T[3] = { "N", "T" };
        OGF_FORTRAN(dgbmv)(
            T[(int)trans],&m,&n,&kl,&ku,&alpha,A,&ldA,x,&incx,&beta,y,&incy
        ) ;
    }

    /** y <- alpha*A*x + beta*y */
    void dsymv( 
        MatrixTriangle uplo, int n, double alpha, const double *A, int ldA,
        const double *x, int incx, double beta, double *y, int incy 
    ) {
        const char *UL[2] = { "U", "L" };
        OGF_FORTRAN(dsymv)(UL[(int)uplo],&n,&alpha,A,&ldA,x,&incx,&beta,y,&incy);
    }


    /** y <- alpha*A*x + beta*y */
    void dsbmv( 
        MatrixTriangle uplo, int n, int k, double alpha, double *A,
        int ldA, const double *x, int incx, double beta, double *y,
        int incy 
    ) {
        const char *UL[2] = { "U", "L" };
        OGF_FORTRAN(dsbmv)(UL[(int)uplo],&n,&k,&alpha,A,&ldA,x,&incx,&beta,y,&incy);
    }


    /** y <- alpha*A*x + beta*y */
    void dspmv( 
        MatrixTriangle uplo, int n, double alpha, const double *AP,
        const double *x, int incx, double beta, double *y, int incy 
    ) {
        const char *UL[2] = { "U", "L" };
        OGF_FORTRAN(dspmv)(UL[(int)uplo],&n,&alpha,AP,x,&incx,&beta,y,&incy);
    }

    /** x <- A*x,  x <- A^T*x */
    void dtrmv( 
        MatrixTriangle uplo, MatrixTranspose trans,
        MatrixUnitTriangular diag, int n, const double *A, int ldA,
        double *x, int incx 
    ) {
        const char *UL[2] = { "U", "L" };
        const char *T[3]  = { "N", "T", 0 };
        const char *D[2]  = { "U", "N" };
        OGF_FORTRAN(dtrmv)(UL[(int)uplo],T[(int)trans],D[(int)diag],&n,A,&ldA,x,&incx);
    }


    /** x <- A*x,  x <- A^T*x */
    void dtbmv( 
        MatrixTriangle uplo, MatrixTranspose trans,
        MatrixUnitTriangular diag, int n, int k, const double *A, int ldA,
        double *x, int incx 
    ) {
        const char *UL[2] = { "U", "L" };
        const char *T[3]  = { "N", "T", 0 };
        const char *D[2]  = { "U", "N" };
        OGF_FORTRAN(dtbmv)(UL[(int)uplo],T[(int)trans],D[(int)diag],&n,&k,A,&ldA,x,&incx);
    }


    /** x <- A*x,  x <- A^T*x */
    void dtpmv( 
        MatrixTriangle uplo, MatrixTranspose trans,
        MatrixUnitTriangular diag, int n, const double *AP,
        double *x, int incx 
    ) {
        const char *UL[2] = { "U", "L" };
        const char *T[3]  = { "N", "T", 0 };
        const char *D[2]  = { "U", "N" };
        OGF_FORTRAN(dtpmv)(UL[(int)uplo],T[(int)trans],D[(int)diag],&n,AP,x,&incx);
    }

    /** x <- A^{-1}*x,  x <- A^{-T}*x */
    void dtrsv( 
        MatrixTriangle uplo, MatrixTranspose trans,
        MatrixUnitTriangular diag, int n, const double *A, int ldA,
        double *x, int incx 
    ) {
        const char *UL[2] = { "U", "L" };
        const char *T[3]  = { "N", "T", 0 };
        const char *D[2]  = { "U", "N" };
        OGF_FORTRAN(dtrsv)(UL[(int)uplo],T[(int)trans],D[(int)diag],&n,A,&ldA,x,&incx);
    }


    /** x <- A^{-1}*x,  x <- A^{-T}*x */
    void dtbsv( 
        MatrixTriangle uplo, MatrixTranspose trans,
        MatrixUnitTriangular diag, int n, int k, const double *A, int ldA,
        double *x, int incx 
    ) {
        const char *UL[2] = { "U", "L" };
        const char *T[3]  = { "N", "T", 0 };
        const char *D[2]  = { "U", "N" };
        OGF_FORTRAN(dtbsv)(UL[(int)uplo],T[(int)trans],D[(int)diag],&n,&k,A,&ldA,x,&incx);
    }


    /** x <- A^{-1}*x,  x <- A^{-T}*x */
    void dtpsv( 
        MatrixTriangle uplo, MatrixTranspose trans,
        MatrixUnitTriangular diag, int n, const double *AP,
        double *x, int incx 
    ) {
        const char *UL[2] = { "U", "L" };
        const char *T[3]  = { "N", "T", 0 };
        const char *D[2]  = { "U", "N" };
        OGF_FORTRAN(dtpsv)(UL[(int)uplo],T[(int)trans],D[(int)diag],&n,AP,x,&incx);
    }

    /** A <- alpha*x*y^T + A,   A-(m,n) */
    void dger( 
        int m, int n, double alpha, const double *x, int incx,
        const double *y, int incy, double *A, int ldA 
    ) {
        OGF_FORTRAN(dger)(&m,&n,&alpha,x,&incx,y,&incy,A,&ldA);
    }


    /** A <- alpha*x*x^T + A */
    void dsyr( 
        MatrixTriangle uplo, int n, double alpha, const double *x,
        int incx, double *A, int ldA 
    ) {
        const char *UL[2] = { "U", "L" };
        OGF_FORTRAN(dsyr)(UL[(int)uplo],&n,&alpha,x,&incx,A,&ldA);
    }

    /** A <- alpha*x*x^T + A */
    void dspr( 
        MatrixTriangle uplo, int n, double alpha, const double *x,
        int incx, double *AP 
    ) {
        const char *UL[2] = { "U", "L" };
        OGF_FORTRAN(dspr)(UL[(int)uplo],&n,&alpha,x,&incx,AP);
    }


    /** A <- alpha*x*y^T + alpha*y*x^T + A */
    void dsyr2( 
        MatrixTriangle uplo, int n, double alpha, const double *x,
        int incx, const double *y, int incy, double *A, int ldA 
    ) {
        const char *UL[2] = { "U", "L" };
        OGF_FORTRAN(dsyr2)(UL[(int)uplo],&n,&alpha,x,&incx,y,&incy,A,&ldA);
    }


    /** A <- alpha*x*y^T + alpha*y*x^T + A */
    void dspr2( 
        MatrixTriangle uplo, int n, double alpha, const double *x,
        int incx, const double *y, int incy, double *AP 
    ) {
        const char *UL[2] = { "U", "L" };
        OGF_FORTRAN(dspr2)(UL[(int)uplo],&n,&alpha,x,&incx,y,&incy,AP);
    }
}

