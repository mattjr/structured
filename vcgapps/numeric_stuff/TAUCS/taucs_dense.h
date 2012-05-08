/*************************************************************************************
 * TAUCS                                                                             
 * Author: Sivan Toledo                                                              
 *************************************************************************************/
#ifndef _TAUCS_DENSE_H
#define _TAUCS_DENSE_H

#ifdef TAUCS_CILK
#pragma lang -C
#endif

/*************************************************************************************
 * Calling method for dense subroutines
 * The calling method goal is to determine whether we use cilk when calling this 
 * dense routines.
 *************************************************************************************/

/* TODO: Rid of temprorary */
#ifdef TAUCS_CILK
#define TAUCS_FORCE_PARALLEL
#endif

/* Cilk conventions for anyone using BLAS or LAPACK */
#if defined(TAUCS_CILK) && defined(TAUCS_FORCE_PARALLEL)
#define TAUCS_DENSE_CILK      cilk
#define TAUCS_DENSE_SPAWN     spawn
#define TAUCS_DENSE_SYNC      sync
#else
#define TAUCS_DENSE_CILK      
#define TAUCS_DENSE_SPAWN     
#define TAUCS_DENSE_SYNC      
#endif

/*************************************************************************************
 * NON CILKED versions (REGULAR)
 *************************************************************************************/

/*************************************************************************************
 * BLAS abstraction
 *************************************************************************************/

/*************************************************************************************
 * Function: S_CeqMABT
 *
 * Description: Makes C = -A * B'. A is mxk. B is nxk. Their load is given too.
 *              A load for C is passed too. 
 *
 *************************************************************************************/
void taucs_dtl(S_CeqMABT)(int m, int n, int k, 
			  taucs_datatype *A, int ld_a, 
			  taucs_datatype *B, int ld_b, 
			  taucs_datatype *C, int ld_c);

/*************************************************************************************
 * Function: S_CaddMABT
 *
 * Description: Makes C -= A * B'. A is mxk. B is nxk. Their load is given too.
 *              A load for C is passed too.
 *
 *************************************************************************************/
void taucs_dtl(S_CaddMABT)(int m, int n, int k, 
			   taucs_datatype *A, int ld_a, 
			   taucs_datatype *B, int ld_b, 
			   taucs_datatype *C, int ld_c);

/*************************************************************************************
 * Function: S_CaddMAB
 *
 * Description: Makes C -= A * B. A is mxk. B is kxn. Their load is given too.
 *              A load for C is passed too.
 *
 *************************************************************************************/
void taucs_dtl(S_CaddMAB)(int m, int n, int k, 
			  taucs_datatype *A, int ld_a, 
			  taucs_datatype *B, int ld_b, 
			  taucs_datatype *C, int ld_c);

/*************************************************************************************
 * Function: S_CaddMATB
 *
 * Description: Makes C -= A' * B. A is kxm. B is kxn. Their load is given too.
 *              A load for C is passed too.
 *
 *************************************************************************************/
void taucs_dtl(S_CaddMATB)(int m, int n, int k, 
			   taucs_datatype *A, int ld_a, 
			   taucs_datatype *B, int ld_b, 
			   taucs_datatype *C, int ld_c);

/*************************************************************************************
 * Function: S_UnitLowerRightTriSolve
 *
 * Description: Solves the unit-lower right triangular system.
 *
 *************************************************************************************/
void taucs_dtl(S_UnitLowerRightTriSolve)(int m, int n, 
					 taucs_datatype *L, int ld_l, 
					 taucs_datatype *B, int ld_b);


/*************************************************************************************
 * Function: S_UnitLowerLeftTriSolve
 *
 * Description: Solves the unit-lower left triangular system. 
 *
 *************************************************************************************/
void taucs_dtl(S_UnitLowerLeftTriSolve)(int m, int n, 
					taucs_datatype *L, int ld_l, 
					taucs_datatype *B, int ld_b);

/*************************************************************************************
 * Function: S_UnitLowerLeftTriSolve
 *
 * Description: Solves the upper left triangular system. 
 *
 *************************************************************************************/
void taucs_dtl(S_UpperLeftTriSolve)(int m, int n, 
				    taucs_datatype *L, int ld_l, 
				    taucs_datatype *B, int ld_b);

/*************************************************************************************
 * LAPACK abstraction
 *************************************************************************************/

/*************************************************************************************
 * Function: S_LU
 *
 * Description: LU factorizes A which is mxn with load lda. rows gives rows number.
 *              On output it is the new row-ordering. This is priority LUing with
 *              priorities given in prty. At each step we choose the row with the 
 *              lowest priority with abs value bigger the the max abs value.
 *              A workspace of m elements is requested. CILKED.
 *
 *************************************************************************************/
void taucs_dtl(S_LU)(taucs_datatype *A, int m, int n, int lda, 
		     double thresh, int *prty, int *rows, int *workspace);

/*************************************************************************************
 * CILKED versions, if we can...
 *************************************************************************************/

/*************************************************************************************
 * BLAS abstraction
 *************************************************************************************/

/*************************************************************************************
 * Function: C_CeqMABT
 *
 * Description: Makes C = -A * B'. A is mxk. B is nxk. Their load is given too.
 *              A load for C is passed too. CILKED.
 *
 *************************************************************************************/
TAUCS_DENSE_CILK void taucs_dtl(C_CeqMABT)(int m, int n, int k, 
					  taucs_datatype *A, int ld_a, 
					  taucs_datatype *B, int ld_b, 
					  taucs_datatype *C, int ld_c);

/*************************************************************************************
 * Function: C_CaddMABT
 *
 * Description: Makes C -= A * B'. A is mxk. B is nxk. Their load is given too.
 *              A load for C is passed too. CILKED
 *
 *************************************************************************************/
TAUCS_DENSE_CILK void taucs_dtl(C_CaddMABT)(int m, int n, int k, 
					    taucs_datatype *A, int ld_a, 
					    taucs_datatype *B, int ld_b, 
					    taucs_datatype *C, int ld_c);

/*************************************************************************************
 * Function: C_CaddMAB
 *
 * Description: Makes C -= A * B. A is mxk. B is kxn. Their load is given too.
 *              A load for C is passed too. CILKED.
 *
 *************************************************************************************/
TAUCS_DENSE_CILK void taucs_dtl(C_CaddMAB)(int m, int n, int k, 
					   taucs_datatype *A, int ld_a, 
					   taucs_datatype *B, int ld_b, 
					   taucs_datatype *C, int ld_c);

/*************************************************************************************
 * Function: C_CaddMATB
 *
 * Description: Makes C -= A' * B. A is kxm. B is kxn. Their load is given too.
 *              A load for C is passed too. CILKED.
 *
 *************************************************************************************/
TAUCS_DENSE_CILK void taucs_dtl(C_CaddMATB)(int m, int n, int k, 
					    taucs_datatype *A, int ld_a, 
					    taucs_datatype *B, int ld_b, 
					    taucs_datatype *C, int ld_c);

/*************************************************************************************
 * Function: C_UnitLowerRightTriSolve
 *
 * Description: Solves the unit-lower right triangular system.CILKED.
 *
 *************************************************************************************/
TAUCS_DENSE_CILK void taucs_dtl(C_UnitLowerRightTriSolve)(int m, int n, 
							  taucs_datatype *L, int ld_l, 
							  taucs_datatype *B, int ld_b);


/*************************************************************************************
 * Function: C_UnitLowerLeftTriSolve
 *
 * Description: Solves the unit-lower left triangular system. CILKED.
 *
 *************************************************************************************/
TAUCS_DENSE_CILK void taucs_dtl(C_UnitLowerLeftTriSolve)(int m, int n, 
							 taucs_datatype *L, int ld_l, 
							 taucs_datatype *B, int ld_b);

/*************************************************************************************
 * Function: C_UnitLowerLeftTriSolve
 *
 * Description: Solves the upper left triangular system. CILKED
 *
 *************************************************************************************/
TAUCS_DENSE_CILK void taucs_dtl(C_UpperLeftTriSolve)(int m, int n, 
						     taucs_datatype *L, int ld_l, 
						     taucs_datatype *B, int ld_b);

/*************************************************************************************
 * LAPACK abstraction
 *************************************************************************************/

/*************************************************************************************
 * Function: C_LU
 *
 * Description: LU factorizes A which is mxn with load lda. rows gives rows number.
 *              On output it is the new row-ordering. This is priority LUing with
 *              priorities given in prty. At each step we choose the row with the 
 *              lowest priority with abs value bigger the the max abs value.
 *              A workspace of m elements is requested. CILKED.
 *
 *************************************************************************************/
TAUCS_DENSE_CILK void taucs_dtl(C_LU)(taucs_datatype *A, int m, int n, int lda, 
				      double thresh, int *prty, int *rows, int *workspace);

/*************************************************************************************
 * Function: SwapLines
 *
 * Description: Swaps lines
 *
 *************************************************************************************/
cilk void taucs_dtl(SwapLines)(taucs_datatype *A, int n, int lda, int *ipiv, int k1, int k2);

#endif

