/* ========================================================================== */
/* === Include/cholmod_blas.h =============================================== */
/* ========================================================================== */

/* -----------------------------------------------------------------------------
 * CHOLMOD/Include/cholmod_blas.h.  Version 1.0.
 * Copyright (C) 2005, Univ. of Florida.  Author: Timothy A. Davis
 * CHOLMOD/Include/cholmod_blas.h is licensed under Version 2.1 of the GNU
 * Lesser General Public License.  See lesser.txt for a text of the license.
 * CHOLMOD is also available under other licenses; contact authors for details.
 * http://www.cise.ufl.edu/research/sparse
 * -------------------------------------------------------------------------- */

/* This does not need to be included in the user's program. */

#ifndef CHOLMOD_BLAS_H
#define CHOLMOD_BLAS_H

/* ========================================================================== */
/* === Architecture ========================================================= */
/* ========================================================================== */

#define CHOLMOD_ARCHITECTURE "Graphite NumericStuff Library"

#define BLAS_DTRSV f2c_dtrsv
#define BLAS_DGEMV f2c_dgemv
#define BLAS_DTRSM f2c_dtrsm
#define BLAS_DGEMM f2c_dgemm
#define BLAS_DSYRK f2c_dsyrk
#define LAPACK_DPOTRF dpotrf_
#define BLAS_ZTRSV f2c_ztrsv
#define BLAS_ZGEMV f2c_zgemv
#define BLAS_ZTRSM f2c_ztrsm
#define BLAS_ZGEMM f2c_zgemm
#define BLAS_ZHERK f2c_zherk
#define LAPACK_ZPOTRF zpotrf_

/* ========================================================================== */
/* === BLAS and LAPACK integer arguments ==================================== */
/* ========================================================================== */

/* CHOLMOD can be compiled with -D'LONGBLAS=long' for the Sun Performance
 * Library, or -D'LONGBLAS=long long' for SGI's SCSL BLAS.  This defines the
 * integer used in the BLAS for the cholmod_l_* routines.
 *
 * The "int" version of CHOLMOD always uses the "int" version of the BLAS.
 */

#if defined (LONGBLAS) && defined (LONG)
#define BLAS_INT LONGBLAS
#else
#define BLAS_INT int
#endif

/* If the BLAS integer is smaller than the basic CHOLMOD integer, then we need
 * to check for integer overflow when converting from one to the other.  If
 * any integer overflows, the externally-defined blas_ok variable is set to
 * FALSE.  blas_ok should be set to TRUE before calling any BLAS_* macro.
 */

#define CHECK_BLAS_INT (sizeof (BLAS_INT) < sizeof (Int))
#define EQ(K,k) (((BLAS_INT) K) == ((Int) k))

/* ========================================================================== */
/* === BLAS and LAPACK prototypes and macros ================================ */
/* ========================================================================== */

void BLAS_DGEMV (char *trans, BLAS_INT *m, BLAS_INT *n, double *alpha,
	double *A, BLAS_INT *lda, double *X, BLAS_INT *incx, double *beta,
	double *Y, BLAS_INT *incy) ;

#define BLAS_dgemv(trans,m,n,alpha,A,lda,X,incx,beta,Y,incy) \
{ \
    BLAS_INT M = m, N = n, LDA = lda, INCX = incx, INCY = incy ; \
    if (CHECK_BLAS_INT) \
    { \
	blas_ok &= EQ (M,m) && EQ (N,n) && EQ (LDA,lda) && EQ (INCX,incx) \
		&& EQ (INCY,incy) ; \
    } \
    if (blas_ok) \
    { \
	BLAS_DGEMV (trans, &M, &N, alpha, A, &LDA, X, &INCX, beta, Y, &INCY) ; \
    } \
}

void BLAS_ZGEMV (char *trans, BLAS_INT *m, BLAS_INT *n, double *alpha,
	double *A, BLAS_INT *lda, double *X, BLAS_INT *incx, double *beta,
	double *Y, BLAS_INT *incy) ;

#define BLAS_zgemv(trans,m,n,alpha,A,lda,X,incx,beta,Y,incy) \
{ \
    BLAS_INT M = m, N = n, LDA = lda, INCX = incx, INCY = incy ; \
    if (CHECK_BLAS_INT) \
    { \
	blas_ok &= EQ (M,m) && EQ (N,n) && EQ (LDA,lda) && EQ (INCX,incx) \
		&& EQ (INCY,incy) ; \
    } \
    if (blas_ok) \
    { \
	BLAS_ZGEMV (trans, &M, &N, alpha, A, &LDA, X, &INCX, beta, Y, &INCY) ; \
    } \
}

void BLAS_DTRSV (char *uplo, char *trans, char *diag, BLAS_INT *n, double *A,
	BLAS_INT *lda, double *X, BLAS_INT *incx) ;

#define BLAS_dtrsv(uplo,trans,diag,n,A,lda,X,incx) \
{ \
    BLAS_INT N = n, LDA = lda, INCX = incx ; \
    if (CHECK_BLAS_INT) \
    { \
	blas_ok &= EQ (N,n) && EQ (LDA,lda) && EQ (INCX,incx) ; \
    } \
    if (blas_ok) \
    { \
	BLAS_DTRSV (uplo, trans, diag, &N, A, &LDA, X, &INCX) ; \
    } \
}

void BLAS_ZTRSV (char *uplo, char *trans, char *diag, BLAS_INT *n, double *A,
	BLAS_INT *lda, double *X, BLAS_INT *incx) ;

#define BLAS_ztrsv(uplo,trans,diag,n,A,lda,X,incx) \
{ \
    BLAS_INT N = n, LDA = lda, INCX = incx ; \
    if (CHECK_BLAS_INT) \
    { \
	blas_ok &= EQ (N,n) && EQ (LDA,lda) && EQ (INCX,incx) ; \
    } \
    if (blas_ok) \
    { \
	BLAS_ZTRSV (uplo, trans, diag, &N, A, &LDA, X, &INCX) ; \
    } \
}

void BLAS_DTRSM (char *side, char *uplo, char *transa, char *diag, BLAS_INT *m,
	BLAS_INT *n, double *alpha, double *A, BLAS_INT *lda, double *B,
	BLAS_INT *ldb) ;

#define BLAS_dtrsm(side,uplo,transa,diag,m,n,alpha,A,lda,B,ldb) \
{ \
    BLAS_INT M = m, N = n, LDA = lda, LDB = ldb ; \
    if (CHECK_BLAS_INT) \
    { \
	blas_ok &= EQ (M,m) && EQ (N,n) && EQ (LDA,lda) && EQ (LDB,ldb) ; \
    } \
    if (blas_ok) \
    { \
	BLAS_DTRSM (side, uplo, transa, diag, &M, &N, alpha, A, &LDA, B, &LDB);\
    } \
}

void BLAS_ZTRSM (char *side, char *uplo, char *transa, char *diag, BLAS_INT *m,
	BLAS_INT *n, double *alpha, double *A, BLAS_INT *lda, double *B,
	BLAS_INT *ldb) ;

#define BLAS_ztrsm(side,uplo,transa,diag,m,n,alpha,A,lda,B,ldb) \
{ \
    BLAS_INT M = m, N = n, LDA = lda, LDB = ldb ; \
    if (CHECK_BLAS_INT) \
    { \
	blas_ok &= EQ (M,m) && EQ (N,n) && EQ (LDA,lda) && EQ (LDB,ldb) ; \
    } \
    if (blas_ok) \
    { \
	BLAS_ZTRSM (side, uplo, transa, diag, &M, &N, alpha, A, &LDA, B, &LDB);\
    } \
}

void BLAS_DGEMM (char *transa, char *transb, BLAS_INT *m, BLAS_INT *n,
	BLAS_INT *k, double *alpha, double *A, BLAS_INT *lda, double *B,
	BLAS_INT *ldb, double *beta, double *C, BLAS_INT *ldc) ;

#define BLAS_dgemm(transa,transb,m,n,k,alpha,A,lda,B,ldb,beta,C,ldc) \
{ \
    BLAS_INT M = m, N = n, K = k, LDA = lda, LDB = ldb, LDC = ldc ; \
    if (CHECK_BLAS_INT) \
    { \
	blas_ok &= EQ (M,m) && EQ (N,n) && EQ (K,k) && EQ (LDA,lda) \
		&& EQ (LDB,ldb) && EQ (LDC,ldc) ; \
    } \
    if (blas_ok) \
    { \
	BLAS_DGEMM (transa, transb, &M, &N, &K, alpha, A, &LDA, B, &LDB, beta, \
	    C, &LDC) ; \
    } \
}

void BLAS_ZGEMM (char *transa, char *transb, BLAS_INT *m, BLAS_INT *n,
	BLAS_INT *k, double *alpha, double *A, BLAS_INT *lda, double *B,
	BLAS_INT *ldb, double *beta, double *C, BLAS_INT *ldc) ;

#define BLAS_zgemm(transa,transb,m,n,k,alpha,A,lda,B,ldb,beta,C,ldc) \
{ \
    BLAS_INT M = m, N = n, K = k, LDA = lda, LDB = ldb, LDC = ldc ; \
    if (CHECK_BLAS_INT) \
    { \
	blas_ok &= EQ (M,m) && EQ (N,n) && EQ (K,k) && EQ (LDA,lda) \
		&& EQ (LDB,ldb) && EQ (LDC,ldc) ; \
    } \
    if (blas_ok) \
    { \
	BLAS_ZGEMM (transa, transb, &M, &N, &K, alpha, A, &LDA, B, &LDB, beta, \
	    C, &LDC) ; \
    } \
}

void BLAS_DSYRK (char *uplo, char *trans, BLAS_INT *n, BLAS_INT *k,
	double *alpha, double *A, BLAS_INT *lda, double *beta, double *C,
	BLAS_INT *ldc) ;

#define BLAS_dsyrk(uplo,trans,n,k,alpha,A,lda,beta,C,ldc) \
{ \
    BLAS_INT N = n, K = k, LDA = lda, LDC = ldc ; \
    if (CHECK_BLAS_INT) \
    { \
	blas_ok &= EQ (N,n) && EQ (K,k) && EQ (LDA,lda) && EQ (LDC,ldc) ; \
    } \
    if (blas_ok) \
    { \
	BLAS_DSYRK (uplo, trans, &N, &K, alpha, A, &LDA, beta, C, &LDC) ; \
    } \
} \

void BLAS_ZHERK (char *uplo, char *trans, BLAS_INT *n, BLAS_INT *k,
	double *alpha, double *A, BLAS_INT *lda, double *beta, double *C,
	BLAS_INT *ldc) ;

#define BLAS_zherk(uplo,trans,n,k,alpha,A,lda,beta,C,ldc) \
{ \
    BLAS_INT N = n, K = k, LDA = lda, LDC = ldc ; \
    if (CHECK_BLAS_INT) \
    { \
	blas_ok &= EQ (N,n) && EQ (K,k) && EQ (LDA,lda) && EQ (LDC,ldc) ; \
    } \
    if (blas_ok) \
    { \
	BLAS_ZHERK (uplo, trans, &N, &K, alpha, A, &LDA, beta, C, &LDC) ; \
    } \
} \

void LAPACK_DPOTRF (char *uplo, BLAS_INT *n, double *A, BLAS_INT *lda,
	BLAS_INT *info) ;

#define LAPACK_dpotrf(uplo,n,A,lda,info) \
{ \
    BLAS_INT N = n, LDA = lda, INFO = 1 ; \
    if (CHECK_BLAS_INT) \
    { \
	blas_ok &= EQ (N,n) && EQ (LDA,lda) ; \
    } \
    if (blas_ok) \
    { \
	LAPACK_DPOTRF (uplo, &N, A, &LDA, &INFO) ; \
    } \
    info = INFO ; \
}

void LAPACK_ZPOTRF (char *uplo, BLAS_INT *n, double *A, BLAS_INT *lda,
	BLAS_INT *info) ;

#define LAPACK_zpotrf(uplo,n,A,lda,info) \
{ \
    BLAS_INT N = n, LDA = lda, INFO = 1 ; \
    if (CHECK_BLAS_INT) \
    { \
	blas_ok &= EQ (N,n) && EQ (LDA,lda) ; \
    } \
    if (blas_ok) \
    { \
	LAPACK_ZPOTRF (uplo, &N, A, &LDA, &INFO) ; \
    } \
    info = INFO ; \
}

/* ========================================================================== */
/* === BLAS and LAPACK macros =============================================== */
/* ========================================================================== */

#endif
