/*************************************************************************************
 * TAUCS                                                                             
 * Author: Sivan Toledo                                                              
 *************************************************************************************/
#include <assert.h>

#define TAUCS_CORE_CILK

#ifndef TAUCS_CORE_GENERAL
#include <numeric_stuff/TAUCS/taucs.h>
#include <numeric_stuff/TAUCS/taucs_dense.h>
#endif

#ifdef TAUCS_CILK
#pragma lang -C
#endif

/* Not core general because different implementation for different datatypes */
#ifndef TAUCS_CORE_GENERAL

/*************************************************************************************
 *************************************************************************************
 * COMPILE-TIME PARAMETERS
 *************************************************************************************
 *************************************************************************************/

/* 
 * gemm parameters
 * TAUCS_THRESHOLD_GEMM_SMALL: If the matrices are smaller then this size we use a
 *                             naive gemm function.
 * TAUCS_THRESHOLD_GEMM_BLAS:  If the matrices are smaller then this size and not too
 *                             small (previous parameter) we use the BLAS's function
 */
#define TAUCS_THRESHOLD_GEMM_SMALL    20
#define TAUCS_THRESHOLD_GEMM_BLAS     180

/* 
 * trsm parameters
 * TAUCS_THRESHOLD_TRSM_SMALL: If the matrices are smaller then this size we use a
 *                             naive trsm function.
 * TAUCS_THRESHOLD_TRSM_BLAS:  If the matrices are smaller then this size and not too
 *                             small (previous parameter) we use the BLAS's function
 */
#define TAUCS_THRESHOLD_TRSM_SMALL    20 
#define TAUCS_THRESHOLD_TRSM_BLAS     180

/* 
 * laswp parameters
 * TAUCS_THRESHOLD_LASWP:  If the matrices are smaller then this size and not too
 *                         small (previous parameter) we use the LAPACK's function
 */
#define TAUCS_THRESHOLD_LASWP_LAPACK  40

/* 
 * getrf parameters
 * TAUCS_THRESHOLD_GETRF_SMALL: If the matrices are smaller then this size we use a
 *                              naive getrf function.
 * TAUCS_THRESHOLD_GETRF_BLAS:  If the matrices are smaller then this size and not too
 *                              small (previous parameter) we use the LAPACK's function
 */
#define TAUCS_THRESHOLD_GETRF_SMALL   20
#define TAUCS_THRESHOLD_GETRF_LAPACK  80


/*
 * TODO: This is sure for the C interface. Not sure about 
 *       the fotran interface. Check if this indeed true...
 * The sun performance library has a "bug": it uses 0-based indices in ipiv
 * in GETRF. This is documented in their manuals.
 */
#ifdef TAUCS_SUNPERF_LAPACK
#define _TAUCS_LAPACK_ZERO_IND 0
#else
#define _TAUCS_LAPACK_ZERO_IND 1
#endif

/*************************************************************************************
 *************************************************************************************
 * SEQUENTIAL FUNCTIONS
 * These merely calls the BLAS and LAPACK with right set of parameters
 *************************************************************************************
 ************************************************************************************/

/*************************************************************************************
 * Function: S_UnitLowerRightTriSolve
 *
 * Description: Solves the unit-lower right triangular system. Sequential version.
 *
 *************************************************************************************/
void taucs_dtl(S_UnitLowerRightTriSolve)(int m, int n, 
					 taucs_datatype *L, int ld_l, taucs_datatype *B, int ld_b)
{
  taucs_trsm("Right", "Lower", "Transpose", "Unit", &m, &n, &taucs_one_const, 
	     L, &ld_l, B, &ld_b);
}

/*************************************************************************************
 * Function: S_UnitLowerLeftTriSolve
 *
 * Description: Solves the unit-lower left triangular system. Sequential version.
 *
 *************************************************************************************/
void taucs_dtl(S_UnitLowerLeftTriSolve)(int m, int n, 
					taucs_datatype *L, int ld_l, taucs_datatype *B, int ld_b)
{
  taucs_trsm("Left", "Lower", "No Transpose", "Unit", &m, &n, &taucs_one_const, 
	     L, &ld_l, B, &ld_b);
}

/*************************************************************************************
 * Function: S_UpperLeftTriSolve
 *
 * Description: Solves the upper left triangular system. Sequential version.
 *
 *************************************************************************************/
void taucs_dtl(S_UpperLeftTriSolve)(int m, int n, 
				    taucs_datatype *L, int ld_l, taucs_datatype *B, int ld_b)
{
  taucs_trsm("Left", "Upper", "No Transpose", "Non Unit", &m, &n, &taucs_one_const,
	     L, &ld_l, B, &ld_b);
}

/*************************************************************************************
 * Function: S_CeqMABT
 *
 * Description: Makes C = -A * B'. A is mxk. B is nxk. Their load is given too.
 *              A load for C is passed too. Sequential version.
 *
 *************************************************************************************/
void taucs_dtl(S_CeqMABT)(int m, int n, int k, 
			  taucs_datatype *A, int ld_a, 
			  taucs_datatype *B, int ld_b, 
			  taucs_datatype *C, int ld_c)
{
  taucs_gemm("No Operation", "Transpose", &m, &n, &k, 
	     &taucs_minusone_const, A, &ld_a, B, &ld_b,
	     &taucs_zero_const, C, &ld_c);
}

/*************************************************************************************
 * Function: S_CaddMABT
 *
 * Description: Makes C -= A * B'. A is mxk. B is nxk. Their load is given too.
 *              A load for C is passed too. Sequential version.
 *
 *************************************************************************************/
void taucs_dtl(S_CaddMABT)(int m, int n, int k, 
			   taucs_datatype *A, int ld_a, 
			   taucs_datatype *B, int ld_b, 
			   taucs_datatype *C, int ld_c)
{
  taucs_gemm("No Operation", "Transpose", &m, &n, &k, 
	     &taucs_minusone_const, A, &ld_a, B, &ld_b,
	     &taucs_one_const, C, &ld_c);
}

/*************************************************************************************
 * Function: S_CaddMAB
 *
 * Description: Makes C -= A * B. A is mxk. B is nxk. Their load is given too.
 *              A load for C is passed too. Sequential version.
 *
 *************************************************************************************/
void taucs_dtl(S_CaddMAB)(int m, int n, int k, 
			  taucs_datatype *A, int ld_a, 
			  taucs_datatype *B, int ld_b, 
			  taucs_datatype *C, int ld_c)
{
  taucs_gemm("No Operation", "No Operation", &m, &n, &k, 
	     &taucs_minusone_const, A, &ld_a, B, &ld_b,
	     &taucs_one_const, C, &ld_c);
}

/*************************************************************************************
 * Function: S_CaddMATB
 *
 * Description: Makes C -= A' * B. A is kxm. B is nxk. Their load is given too.
 *              A load for C is passed too. Sequential version.
 *
 *************************************************************************************/
void taucs_dtl(S_CaddMATB)(int m, int n, int k, 
			   taucs_datatype *A, int ld_a, 
			   taucs_datatype *B, int ld_b, 
			   taucs_datatype *C, int ld_c)
{
  taucs_gemm("Transpose", "No Operation", &m, &n, &k, 
	     &taucs_minusone_const, A, &ld_a, B, &ld_b,
	     &taucs_one_const, C, &ld_c);
}

/*************************************************************************************
 * LAPACK or ATLAS for LAPACK
 *************************************************************************************/

/*************************************************************************************
 * Function: S_LU
 *
 * Description: LU factorizes A which is mxn with load lda. rows gives rows number.
 *              On output it is the new row-ordering. This is priority LUing with
 *              priorities given in prty. At each step we choose the row with the 
 *              lowest priority with abs value bigger the the max abs value.
 *              A workspace of m elements is requested.
 *
 *************************************************************************************/
void taucs_dtl(S_LU)(taucs_datatype *A, int m, int n, int lda, taucs_double thresh, int *prty, int *rows, int *workspace)
{
  int i, info;

  taucs_getrf(&m, &n, A, &lda, workspace, &info);

  /* Reform the list of rows */
  for(i = 0; i < n; i++)
  {
    int temp = rows[i];
    rows[i] = rows[workspace[i] - _TAUCS_LAPACK_ZERO_IND];
    rows[workspace[i] - _TAUCS_LAPACK_ZERO_IND] = temp;
  }
}

/*************************************************************************************
 *************************************************************************************
 * API FUNCTIONS WITH NO PARALLELIZATION
 * This functions are for the case we do not force parallelization.
 * The simply call the sequential functions. Hopefully the optimizer will know how
 * to kill redundent code.
 *************************************************************************************
 ************************************************************************************/
#if !defined(TAUCS_CILK) || !defined(TAUCS_FORCE_PARALLEL)

void taucs_dtl(C_LU)(taucs_datatype *A, int m, int n, int lda, taucs_double thresh, int *prty, int *rows, int *workspace)
{
  taucs_dtl(S_LU)(A, m, n, lda, thresh, prty, rows, workspace);
}

void taucs_dtl(C_CeqMABT)(int m, int n, int k, 
			  taucs_datatype *A, int ld_a, 
			  taucs_datatype *B, int ld_b, 
			  taucs_datatype *C, int ld_c)
{
  taucs_dtl(S_CeqMABT)(m, n, k, A, ld_a, B, ld_b, C, ld_c);
}

void taucs_dtl(C_CaddMABT)(int m, int n, int k, 
			   taucs_datatype *A, int ld_a, 
			   taucs_datatype *B, int ld_b, 
			   taucs_datatype *C, int ld_c)
{
  taucs_dtl(S_CaddMABT)(m, n, k, A, ld_a, B, ld_b, C, ld_c);
}

void taucs_dtl(C_CaddMAB)(int m, int n, int k, 
			  taucs_datatype *A, int ld_a, 
				    taucs_datatype *B, int ld_b, 
			  taucs_datatype *C, int ld_c)
{
  taucs_dtl(S_CaddMAB)(m, n, k, A, ld_a, B, ld_b, C, ld_c);
}

void taucs_dtl(C_CaddMATB)(int m, int n, int k, 
			   taucs_datatype *A, int ld_a, 
			   taucs_datatype *B, int ld_b, 
			   taucs_datatype *C, int ld_c)
{
  taucs_dtl(S_CaddMATB)(m, n, k, A, ld_a, B, ld_b, C, ld_c);
}


void taucs_dtl(C_UnitLowerRightTriSolve)(int m, int n, 
					 taucs_datatype *L, int ld_l, taucs_datatype *B, int ld_b)
{
  taucs_dtl(S_UnitLowerRightTriSolve)(m, n, L, ld_l, B, ld_b);
}

void taucs_dtl(C_UnitLowerLeftTriSolve)(int m, int n, 
					taucs_datatype *L, int ld_l, taucs_datatype *B, int ld_b)
{
  taucs_dtl(S_UnitLowerLeftTriSolve)(m, n, L, ld_l, B, ld_b);
}

void taucs_dtl(C_UpperLeftTriSolve)(int m, int n, 
				    taucs_datatype *L, int ld_l, taucs_datatype *B, int ld_b)
{
  taucs_dtl(S_UpperLeftTriSolve)(m, n, L, ld_l, B, ld_b);
}

#endif

/*************************************************************************************
 * Function: S_SwapLines
 *
 * Description: Swap lines for A when spaws are given in ipiv's indexes k1:k2-1.
 *              k1 and k2 in are C indexes (zero based). Sequential version.
 *
 *************************************************************************************/
static void S_SwapLines(taucs_datatype *A, int n, int lda, int *ipiv, int k1, int k2)
{
  int i, j, t;
  taucs_datatype v;

  for (j = 0; j < n; j++)
    for (i = k1; i < k2; i++)
    {
      t = ipiv[i] - _TAUCS_LAPACK_ZERO_IND;
      v = A[j * lda + i];
      A[j * lda + i] = A[j * lda + t];
      A[j * lda + t] = v;
    }
}

/*************************************************************************************
 * Function: SwapLines
 *
 * Description: Parallelized version of SwapLines.
 *
 *************************************************************************************/
cilk void taucs_dtl(SwapLines)(taucs_datatype *A, int n, int lda, int *ipiv, int k1, int k2)
{
  int nhalf1, nhalf2;

  /* Small matrices - use LAPACK */
  if (n <= TAUCS_THRESHOLD_LASWP_LAPACK)
  {
    S_SwapLines(A, n, lda, ipiv, k1, k2);
    return;
  }

  /* Break matrix into two and run in parallel */
  nhalf1 = n / 2;
  nhalf2 = n - nhalf1;

  spawn taucs_dtl(SwapLines)(A, nhalf1, lda, ipiv, k1, k2);
  spawn taucs_dtl(SwapLines)(A + nhalf1 * lda, nhalf2, lda, ipiv, k1, k2);
  sync;
}


/*************************************************************************************
 *************************************************************************************
 * API FUNCTIONS WITH PARALLELIZATION
 * Here we make the BLAS and LAPACK parallel even if they are not
 *************************************************************************************
 ************************************************************************************/
#if defined(TAUCS_CILK) && defined(TAUCS_FORCE_PARALLEL)


/*************************************************************************************
 * Function: S_CaddMABT_small
 *
 * Description: Sequential version of CaddMABT for very small matrices. Trivial code.
 *
 *************************************************************************************/
static void S_CaddMABT_small(int m, int n, int k, 
			     taucs_datatype *A, int ld_a, 
			     taucs_datatype *B, int ld_b, 
			     taucs_datatype *C, int ld_c)
{
  int j, i, l;
  taucs_datatype *Cj, *Ail, *Bjl;
  taucs_datatype Cij;

  Cj = C;
  for (j = 0; j < n; j++) 
  {
    for (i = 0; i < m; i++) 
    {
      Cij = *Cj;
      Ail = A + i;
      Bjl = B + j;
      for (l = 0; l < k; l++) 
      {
	Cij = taucs_sub(Cij, taucs_mul((*Ail), (*Bjl)));
	Ail += ld_a;
	Bjl += ld_b;
      }
      *Cj = Cij;
      Cj++;
    }
    Cj = Cj + ld_c - m;     /* now Cj is at the top of column j + 1 */
  }
}

/*************************************************************************************
 * Function: S_CaddMAB_small
 *
 * Description: Sequential version of CaddMAB for very small matrices. Trivial code.
 *
 *************************************************************************************/
static void S_CaddMAB_small(int m, int n, int k, 
			    taucs_datatype *A, int ld_a, 
			    taucs_datatype *B, int ld_b, 
			    taucs_datatype *C, int ld_c)
{
  int j, i, l;
  taucs_datatype *Cj, *Ail, *Blj;
  taucs_datatype Cij;

  Cj = C;
  for (j = 0; j < n; j++) 
  {
    for (i = 0; i < m; i++) 
    {
      Cij = *Cj;
      Ail = A + i;
      Blj = B + j * ld_b;
      for (l = 0; l < k; l++) 
      {
	Cij = taucs_sub(Cij, taucs_mul((*Ail), (*Blj)));
	Ail += ld_a;
	Blj++;
      }
      *Cj = Cij;
      Cj++;
    }
    Cj = Cj + ld_c - m;     /* now Cj is at the top of column j + 1 */
  }
}

/*************************************************************************************
 * Function: S_CaddMATB_small
 *
 * Description: Sequential version of CaddMATB for very small matrices. Trivial code.
 *
 *************************************************************************************/
static void S_CaddMATB_small(int m, int n, int k, 
			    taucs_datatype *A, int ld_a, 
			    taucs_datatype *B, int ld_b, 
			    taucs_datatype *C, int ld_c)
{
  int j, i, l;
  taucs_datatype *Cj, *Ali, *Blj;
  taucs_datatype Cij;

  Cj = C;
  for (j = 0; j < n; j++) 
  {
    for (i = 0; i < m; i++) 
    {
      Cij = *Cj;
      Ali = A + i * ld_a;
      Blj = B + j * ld_b;
      for (l = 0; l < k; l++) 
      {
	Cij = taucs_sub(Cij, taucs_mul((*Ali), (*Blj)));
	Ali++;
	Blj++;
      }
      *Cj = Cij;
      Cj++;
    }
    Cj = Cj + ld_c - m;     /* now Cj is at the top of column j + 1 */
  }
}

/*************************************************************************************
 * Function: C_CaddMABT
 *
 * Description: Parallelized version of CaddMABT.
 *
 *************************************************************************************/
cilk void taucs_dtl(C_CaddMABT)(int m, int n, int k, 
				taucs_datatype *A, int ld_a, 
				taucs_datatype *B, int ld_b, 
				taucs_datatype *C, int ld_c)
{
  /* Very small matrices - use simple code */
  if (n <= TAUCS_THRESHOLD_GEMM_SMALL && k <= TAUCS_THRESHOLD_GEMM_SMALL) 
  {
    S_CaddMABT_small(m, n, k, A, ld_a, B, ld_b, C, ld_c);
    return;
  }

  /* Small matrices - use BLAS */
  if (n <= TAUCS_THRESHOLD_GEMM_BLAS && k <= TAUCS_THRESHOLD_GEMM_BLAS) 
  {
    taucs_dtl(S_CaddMABT)(m, n, k, A, ld_a, B, ld_b, C, ld_c);
    
    return;
  }

  if (k >= n && k >= m) 
  {
    int khalf1 = k / 2;
    int khalf2 = k - khalf1;

    spawn taucs_dtl(C_CaddMABT)(m, n, khalf1, A, ld_a, B, ld_b, C, ld_c);
    sync;

    spawn taucs_dtl(C_CaddMABT)(m, n, khalf2, A + khalf1 * ld_a, ld_a, B + khalf1 * ld_b, ld_b, C, ld_c);
    sync;

    return;
  } 

  if (n >= k && n >= m) 
  {
    int nhalf1 = n / 2;
    int nhalf2 = n - nhalf1;
    
    spawn taucs_dtl(C_CaddMABT)(m, nhalf1, k, A, ld_a, B, ld_b, C, ld_c);
    spawn taucs_dtl(C_CaddMABT)(m, nhalf2, k, A, ld_a, B + nhalf1, ld_b, C + nhalf1 * ld_c, ld_c);
    sync;

    return;
  }

  /* The following condition must be true so we put 1 */
  if (1 /* m >= k && m >= n*/) 
  {
    int mhalf1 = m / 2;
    int mhalf2 = m - mhalf1;

    spawn taucs_dtl(C_CaddMABT)(mhalf1, n, k, A, ld_a, B, ld_b, C, ld_c);
    spawn taucs_dtl(C_CaddMABT)(mhalf2, n, k, A + mhalf1, ld_a, B, ld_b, C + mhalf1, ld_c);
    sync;

    return;
  }

  assert(0);
}

/*************************************************************************************
 * Function: C_CaddMAB
 *
 * Description: Parallelized version of CaddMAB.
 *
 *************************************************************************************/
cilk void taucs_dtl(C_CaddMAB)(int m, int n, int k, 
			     taucs_datatype *A, int ld_a, 
			     taucs_datatype *B, int ld_b, 
			     taucs_datatype *C, int ld_c)
{
  /* Very small matrices - use simple code */
  if (n <= TAUCS_THRESHOLD_GEMM_SMALL && k <= TAUCS_THRESHOLD_GEMM_SMALL) 
  {
    S_CaddMAB_small(m,n,k, A, ld_a, B, ld_b, C, ld_c);
    return;
  }

  /* Small matrices - use BLAS */
  if (n <= TAUCS_THRESHOLD_GEMM_BLAS && k <= TAUCS_THRESHOLD_GEMM_BLAS) 
  {
    taucs_dtl(S_CaddMAB)(m, n, k, A, ld_a, B, ld_b, C, ld_c);
    
    return;
  }

  if (k >= n && k >= m) 
  {
    int khalf1 = k / 2;
    int khalf2 = k - khalf1;

    spawn taucs_dtl(C_CaddMAB)(m, n, khalf1, A, ld_a, B, ld_b, C, ld_c);
    sync;

    spawn taucs_dtl(C_CaddMAB)(m, n, khalf2, A + khalf1 * ld_a, ld_a, B + khalf1, ld_b, C, ld_c);
    sync;

    return;
  } 

  if (n >= k && n >= m) 
  {
    int nhalf1 = n / 2;
    int nhalf2 = n - nhalf1;
    
    spawn taucs_dtl(C_CaddMAB)(m, nhalf1, k, A, ld_a, B, ld_b, C, ld_c);
    spawn taucs_dtl(C_CaddMAB)(m, nhalf2, k, A, ld_a, B + nhalf1 * ld_b, ld_b, C + nhalf1 * ld_c, ld_c);
    sync;

    return;
  }

  /* The following condition must be true so we put 1 */
  if (1 /* m >= k && m >= n*/) 
  {
    int mhalf1 = m / 2;
    int mhalf2 = m - mhalf1;

    spawn taucs_dtl(C_CaddMAB)(mhalf1, n, k, A, ld_a, B, ld_b, C, ld_c);
    spawn taucs_dtl(C_CaddMAB)(mhalf2, n, k, A + mhalf1, ld_a, B, ld_b, C + mhalf1, ld_c);
    sync;

    return;
  }

  assert(0);
}

/*************************************************************************************
 * Function: CaddMATB
 *
 * Description: Parallelized version of CaddMATB.
 *
 *************************************************************************************/
cilk void taucs_dtl(C_CaddMATB)(int m, int n, int k, 
				taucs_datatype *A, int ld_a, 
				taucs_datatype *B, int ld_b, 
				taucs_datatype *C, int ld_c)
{
  /* Very small matrices - use simple code */
  if (n <= TAUCS_THRESHOLD_GEMM_SMALL && k <= TAUCS_THRESHOLD_GEMM_SMALL) 
  {
    S_CaddMATB_small(m, n, k, A, ld_a, B, ld_b, C, ld_c);
    return;
  }

  /* Small matrices - use BLAS */
  if (n <= TAUCS_THRESHOLD_GEMM_BLAS && k <= TAUCS_THRESHOLD_GEMM_BLAS) 
  {
    taucs_dtl(S_CaddMATB)(m, n, k, A, ld_a, B, ld_b, C, ld_c);
    
    return;
  }

  if (k >= n && k >= m) 
  {
    int khalf1 = k / 2;
    int khalf2 = k - khalf1;

    spawn taucs_dtl(C_CaddMATB)(m, n, khalf1, A, ld_a, B, ld_b, C, ld_c);
    sync;

    spawn taucs_dtl(C_CaddMATB)(m, n, khalf2, A + khalf1, ld_a, B + khalf1, ld_b, C, ld_c);
    sync;

    return;
  } 

  if (n >= k && n >= m) 
  {
    int nhalf1 = n / 2;
    int nhalf2 = n - nhalf1;
    
    spawn taucs_dtl(C_CaddMATB)(m, nhalf1, k, A, ld_a, B, ld_b, C, ld_c);
    spawn taucs_dtl(C_CaddMATB)(m, nhalf2, k, A, ld_a, B + nhalf1 * ld_b, ld_b, C + nhalf1 * ld_c, ld_c);
    sync;

    return;
  }

  /* The following condition must be true so we put 1 */
  if (1 /* m >= k && m >= n*/) 
  {
    int mhalf1 = m / 2;
    int mhalf2 = m - mhalf1;

    spawn taucs_dtl(C_CaddMATB)(mhalf1, n, k, A, ld_a, B, ld_b, C, ld_c);
    spawn taucs_dtl(C_CaddMATB)(mhalf2, n, k, A + mhalf1 * ld_a, ld_a, B, ld_b, C + mhalf1, ld_c);
    sync;

    return;
  }

  assert(0);
}

/*************************************************************************************
 * Function: C_CeqMABT
 *
 * Description: Parallelized version of CeqMABT.
 *
 *************************************************************************************/
cilk void taucs_dtl(C_CeqMABT)(int m, int n, int k, 
			     taucs_datatype *A, int ld_a, 
			     taucs_datatype *B, int ld_b, 
			     taucs_datatype *C, int ld_c)
{
  memset(C, 0, ld_c * n * sizeof(taucs_datatype));
  spawn taucs_dtl(C_CaddMABT)(m, n, k, A, ld_a, B, ld_b, C, ld_c);
  sync;
}

/* UnitLowerRightTriSolve, UnitLowerLeftTriSolve and UpperLeftTriSolve */

/*************************************************************************************
 * Function: UnitLowerLeftTriSolve_small
 *
 * Description: Sequential version of UnitLowerLeftTriSolve for very small matrices. 
 *              Trivial code.
 *
 *************************************************************************************/
static void UnitLowerLeftTriSolve_small(int m, int n, 
					taucs_datatype *L, int ld_l, taucs_datatype *B, int ld_b)
{
  int j, i, k;
  taucs_datatype *Bi, *Bkj, *Lik;
  taucs_datatype Bij;

  Bi = B;
  for (i = 0; i < m; i++) 
  {
    for (j = 0; j < n; j++) 
    {
      Bij = *(Bi + j * ld_b);
      Bkj = B + j * ld_b;
      Lik = L + i;
      for (k = 0; k < i; k++) 
      {
	Bij = taucs_sub(Bij, taucs_mul((*Bkj), (*Lik))); 
	Bkj++;
	Lik += ld_l;
      }
      *(Bi + j * ld_b) = Bij;
    }
    Bi = Bi++;
  }
}

/*************************************************************************************
 * Function: UnitLowerRightTriSolve_small
 *
 * Description: Sequential version of UnitLowerRightTriSolve for very small matrices. 
 *              Trivial code.
 *
 *************************************************************************************/
static void UnitLowerRightTriSolve_small(int m, int n, 
					 taucs_datatype *L, int ld_l, taucs_datatype *B, int ld_b)
{
  int j, i, k;
  taucs_datatype *Bi, *Bik, *Ljk;
  taucs_datatype Bij;

  Bi = B;
  for (i = 0; i < m; i++) 
  {
    for (j = 0; j< n; j++) 
    {
      Bij = *(Bi + j * ld_b);
      Bik = Bi;
      Ljk = L + j;
      for (k = 0; k < j; k++) 
      {
	Bij = taucs_sub(Bij, taucs_mul((*Bik), (*Ljk))); 
	Bik += ld_b;
	Ljk += ld_l;
      }
      *(Bi + j * ld_b) = Bij;
    }
    Bi = Bi + 1;
  }
}

/*************************************************************************************
 * Function: UpperLeftTriSolve_small
 *
 * Description: Sequential version of UpperrLeftTriSolve for very small matrices. 
 *              Trivial code.
 *
 *************************************************************************************/
static void UpperLeftTriSolve_small(int m, int n, 
				    taucs_datatype *U, int ld_u, taucs_datatype *B, int ld_b)
{
  int j, i, k;
  taucs_datatype *Bi, *Bkj, *Uik;
  taucs_datatype Bij;

  Bi = B + m - 1;
  for (i = m - 1; i >= 0; i--) 
  {
    for (j = 0; j < n; j++) 
    {
      Bij = *(Bi + j * ld_b);
      Bkj = B + j * ld_b + m - 1;
      Uik = U + i + (m - 1) * ld_u;
      for (k = m - 1; k > i; k--) 
      {
	Bij = taucs_sub(Bij, taucs_mul((*Bkj), (*Uik))); 
	Bkj--;
	Uik -= ld_u;
      }
      Bij = taucs_div(Bij, (*Uik));
      *(Bi + j * ld_b) = Bij;
    }
    Bi = Bi--;
  }
}

/*************************************************************************************
 * Function: C_UnitLowerLeftTriSolve
 *
 * Description: Parallelized version of UnitLowerLeftTriSolve
 *
 *************************************************************************************/
cilk void taucs_dtl(C_UnitLowerLeftTriSolve)(int m, int n, 
					   taucs_datatype *L, int ld_l, taucs_datatype *B, int ld_b)
{
  /* Very small matrices - use simple code */
  if (m <= TAUCS_THRESHOLD_TRSM_SMALL && n <= TAUCS_THRESHOLD_TRSM_SMALL) 
  {
    UnitLowerLeftTriSolve_small(m, n, L, ld_l, B, ld_b);
    return;
  }

  /* Small matrices - use BLAS */
  if (m <= TAUCS_THRESHOLD_TRSM_BLAS && n <= TAUCS_THRESHOLD_TRSM_BLAS) 
  {
    taucs_dtl(S_UnitLowerLeftTriSolve)(m, n, L, ld_l, B, ld_b);
    
    return;
  }

  /* Case A: n >= m */
  if (n >= m) 
  {
    int nhalf1 = n / 2;
    int nhalf2 = n - nhalf1;

    spawn taucs_dtl(C_UnitLowerLeftTriSolve)(m, nhalf1, L, ld_l, B, ld_b);
    spawn taucs_dtl(C_UnitLowerLeftTriSolve)(m, nhalf2, L, ld_l, B + nhalf1 * ld_b, ld_b);
    sync;
    return;
  } 
  else
    /* Case B - m > n */
  {
    int mhalf1 = m / 2;
    int mhalf2 = m - mhalf1;
    
    spawn taucs_dtl(C_UnitLowerLeftTriSolve)(mhalf1, n, L, ld_l, B, ld_b);
    sync;

    spawn taucs_dtl(C_CaddMAB)(mhalf2, n, mhalf1, L + mhalf1, ld_l, B, ld_b, B + mhalf1, ld_b);
    sync;

    spawn taucs_dtl(C_UnitLowerLeftTriSolve)(mhalf2, n, L + mhalf1 * (ld_l + 1), ld_l, B + mhalf1, ld_b);
    sync;

    return;
  }
}

/*************************************************************************************
 * Function: C_UnitLowerRightTriSolve
 *
 * Description: Parallelized version of UnitLowerRightTriSolve.
 *
 *************************************************************************************/
cilk void taucs_dtl(C_UnitLowerRightTriSolve)(int m, int n, 
					      taucs_datatype *L, int ld_l, taucs_datatype *B, int ld_b)
{
  /* Very small matrices - use simple code */
  if (m <= TAUCS_THRESHOLD_TRSM_SMALL && n <= TAUCS_THRESHOLD_TRSM_SMALL) 
  {
    UnitLowerRightTriSolve_small(m, n, L, ld_l, B, ld_b);
    return;
  }

  /* Small matrices - use BLAS */
  if (m <= TAUCS_THRESHOLD_TRSM_BLAS && n <= TAUCS_THRESHOLD_TRSM_BLAS) 
  {
    taucs_dtl(S_UnitLowerRightTriSolve)(m, n, L, ld_l, B, ld_b);
    
    return;
  }

  /* Case A: m >= n */
  if (m >= n) 
  {
    int mhalf1 = m / 2;
    int mhalf2 = m - mhalf1;

    spawn taucs_dtl(C_UnitLowerRightTriSolve)(mhalf1, n, L, ld_l, B, ld_b);
    spawn taucs_dtl(C_UnitLowerRightTriSolve)(mhalf2, n, L, ld_l, B + mhalf1, ld_b);
    sync;
    return;
  } 
  else
    /* Case B - m < n */
  {
    int nhalf1 = n / 2;
    int nhalf2 = n - nhalf1;
    
    spawn taucs_dtl(C_UnitLowerRightTriSolve)(m, nhalf1, L, ld_l, B, ld_b);
    sync;


    spawn taucs_dtl(C_CaddMABT)(m, nhalf2, nhalf1, B, ld_b, L + nhalf1, ld_l, B + nhalf1 * ld_b, ld_b);
    sync;

    spawn taucs_dtl(C_UnitLowerRightTriSolve)(m, nhalf2, L + nhalf1 * (ld_l + 1), ld_l, B + nhalf1 * ld_b, ld_b);
    sync;

    return;
  }
}

/*************************************************************************************
 * Function: C_UpperLeftTriSolve
 *
 * Description: Parallelized version of UpperLeftTriSolve
 *
 *************************************************************************************/
cilk void taucs_dtl(C_UpperLeftTriSolve)(int m, int n, 
				       taucs_datatype *U, int ld_u, taucs_datatype *B, int ld_b)
{
  /* Very small matrices - use simple code */ 
  if (m <= TAUCS_THRESHOLD_TRSM_SMALL && n <= TAUCS_THRESHOLD_TRSM_SMALL) 
  {
    UpperLeftTriSolve_small(m, n, U, ld_u, B, ld_b);
    return;
  }

  /* Small matrices - use BLAS */
  if (m <= TAUCS_THRESHOLD_TRSM_BLAS && n <= TAUCS_THRESHOLD_TRSM_BLAS) 
  {
    taucs_dtl(S_UpperLeftTriSolve)(m, n, U, ld_u, B, ld_b);
    
    return;
  }

  /* Case A: n >= m */
  if (n >= m) 
  {
    int nhalf1 = n / 2;
    int nhalf2 = n - nhalf1;

    spawn taucs_dtl(C_UnitLowerLeftTriSolve)(m, nhalf1, U, ld_u, B, ld_b);
    spawn taucs_dtl(C_UnitLowerLeftTriSolve)(m, nhalf2, U, ld_u, B + nhalf1 * ld_b, ld_b);
    sync;
    return;
  } 
  else
    /* Case B - m > n */
  {
    int mhalf1 = m / 2;
    int mhalf2 = m - mhalf1;
    
    spawn taucs_dtl(C_UpperLeftTriSolve)(mhalf2, n, U + mhalf1 * (ld_u + 1), ld_u, B + mhalf1, ld_b);
    sync;

    spawn taucs_dtl(C_CaddMAB)(mhalf1, n, mhalf2, U + mhalf1 * ld_u, ld_u, B + mhalf1, ld_b, B, ld_b);
    sync;

    spawn taucs_dtl(C_UpperLeftTriSolve)(mhalf1, n, U, ld_u, B, ld_b);
    sync;

    return;
  }
}


/*************************************************************************************
 * Function: LU_ind_small
 *
 * Description: Sequential version of LU_ind for very small matrices. 
 *              Trivial code.
 *
 *************************************************************************************/
static int LU_ind_small(taucs_datatype *A, int m, int n, int lda, taucs_double thresh, int *prty, int *ipiv)
{
  int c, r, k, ind;
  double abs_val;
  taucs_datatype val;
  taucs_datatype *Amax, *Arc, *Ac, *Ark;

  /* Factorize each column */
  for (c = 0; c < n; c++)
  {
    /* Find pivot */
    abs_val = 0;
    Arc = A + c * (lda + 1);
    for(r = c; r < m; r++)
    {
      taucs_double new_abs_val = taucs_abs(*Arc);
      if (new_abs_val > abs_val)
      {
	ind = r;
	abs_val = new_abs_val;
      }

      Arc++;
    }

    /* Maybe max values is 0 */
    if (abs_val == 0)
      return c;


    /* Mark pivot and switch lines */
    ipiv[c] = ind + 1;
    Ac = A + c;
    Amax = A + ind;
    for(k = 0; k < n; k++)
    {
      val = *Ac;
      *Ac = *Amax;
      *Amax = val;

      Ac += lda;
      Amax += lda;
    }

    /* Scale  and do schur update at the same time*/
    val = A[c * lda + c];
    Arc = A + c * (lda + 1) + 1;
    for(r = c + 1; r < m; r++)
    {
      *Arc = taucs_div(*Arc, val);

      /* This is the shur update part */
      Ark = Arc + lda;
      for (k = c + 1; k < n; k++)
      {
	*Ark = taucs_sub(*Ark,
			 taucs_mul(*Arc, *(Ark + c - r))
			 );
	Ark += lda;
      }

      Arc++;
    }
  }

  return 0;
}

/*************************************************************************************
 * Function: LU_ind
 *
 * Description: Parallelized version of LU without the reordering of the rows number
 *              (i.e. we pass vector of changes).
 *
 *************************************************************************************/
cilk static void do_next(taucs_datatype *A, int mrest, int next_bsize, int bsize, int scol, int ecol, int lda, int *ipiv)
{
  int info;

  spawn taucs_dtl(SwapLines)(A + ecol * lda, next_bsize, lda, ipiv, scol, scol + bsize); 
  sync;

  spawn taucs_dtl(C_UnitLowerLeftTriSolve)(bsize, next_bsize, A + scol * lda + scol, lda, A + ecol * lda + scol, lda);
  sync;

  spawn taucs_dtl(C_CaddMAB)(mrest, next_bsize, bsize,
			     A + scol * lda + ecol, lda, 
			     A + ecol * lda + scol, lda, 
			     A + ecol * lda + ecol, lda);
  sync;

	      
  /* Now do the next getrf */
  taucs_getrf(&mrest, &next_bsize, A + ecol  * lda + ecol, &lda, ipiv + ecol, &info);
}


cilk static void do_rest(taucs_datatype *A, int mrest, int nrest, int p, int bsize, int scol, int ecol, int lda, int *ipiv)
{
  int next_nrest = min(nrest - p * TAUCS_THRESHOLD_GETRF_LAPACK, TAUCS_THRESHOLD_GETRF_LAPACK);
  int offset = p * TAUCS_THRESHOLD_GETRF_LAPACK;

  spawn taucs_dtl(SwapLines)(A + (ecol + offset) * lda, next_nrest, lda, ipiv, scol, scol + bsize); 
  sync;

  spawn taucs_dtl(C_UnitLowerLeftTriSolve)(bsize, next_nrest, A + scol * lda + scol, lda, A + (ecol + offset) * lda + scol, lda);
  sync;

  spawn taucs_dtl(C_CaddMAB)(mrest, next_nrest, bsize, 
			     A + scol * lda + ecol, lda, 
			     A + (ecol + offset) * lda + scol, lda, 
			     A + (ecol + offset) * lda + ecol, lda);
  sync;
}

cilk static int LU_ind(taucs_datatype *A, int m, int n, int lda, taucs_double thresh, int *prty, int *ipiv)
{
  int scol, ecol, bsize, nrest, mn, mrest, info, i;

  mn = min(m, n);

  /* To start the loop we factorize the first block */
  bsize = min(TAUCS_THRESHOLD_GETRF_LAPACK, n);
  taucs_getrf(&m, &bsize, A, &lda, ipiv, &info);

  /* Starting from after the first block: update the rest and factor next block */
  for(scol = 0; scol < mn; scol += TAUCS_THRESHOLD_GETRF_LAPACK)
    {

      ecol = scol + bsize;
      nrest = n - ecol;

      /* Apply pivots to start and rest */
      for (i = scol; i < ecol; i++)
	ipiv[i] += scol;
      spawn taucs_dtl(SwapLines)(A, scol, lda, ipiv, scol, scol + bsize);

      /* Create update and factor next block */
      if (nrest > 0)
	{
	  mrest = m - ecol;

	  if (mrest > 0)
	    {

	      int next_bsize = min(min(TAUCS_THRESHOLD_GETRF_LAPACK, nrest), mrest);

	      /* Split updating the next block and the rest so we can pipeline */
	      spawn do_next(A, mrest, next_bsize, bsize, scol, ecol, lda, ipiv);
	      for (i = 1; nrest - i * TAUCS_THRESHOLD_GETRF_LAPACK > 0; i++)
		spawn do_rest(A, mrest, nrest, i, bsize, scol, ecol, lda, ipiv);
	      bsize = next_bsize;
	    }
	  else
	    {
	      /* Apply pivots to U part */
	      spawn taucs_dtl(SwapLines)(A + ecol * lda, nrest, lda, ipiv, scol, scol + bsize); 
	      sync;
	      spawn taucs_dtl(C_UnitLowerLeftTriSolve)(bsize, nrest, A + scol * lda + scol, lda, A + ecol * lda + scol, lda);
	    }
	}

      /* Here is the sync for all operations in this iterations */
      sync;
	  
    }

  return 0;
}
/*************************************************************************************
 * Function: LU
 *
 * Description: Parallelized version of LU.
 *
 *************************************************************************************/
cilk void taucs_dtl(C_LU)(taucs_datatype *A, int m, int n, int lda, taucs_double thresh, int *prty, int *rows, int *workspace)
{
  int i, t;

  t = spawn LU_ind(A, m, n, lda, thresh, prty, workspace);
  sync;

  /* Reform the list of rows */
  for(i = 0; i < n; i++)
  {
    int temp = rows[i];
    rows[i] = rows[workspace[i] - _TAUCS_LAPACK_ZERO_IND];
    rows[workspace[i] - _TAUCS_LAPACK_ZERO_IND] = temp;
  }
}

#endif

#endif


