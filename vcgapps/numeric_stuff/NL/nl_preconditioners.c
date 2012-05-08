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

#include <numeric_stuff/NL/nl_preconditioners.h>
#include <numeric_stuff/NL/nl_blas.h>
#include <numeric_stuff/NL/nl_matrix.h>
#include <numeric_stuff/NL/nl_context.h>

/************************************************************************/
/* preconditioners */

/* Utilities for preconditioners */

void nlMultDiagonal(NLdouble* xy, NLdouble omega) {
    NLuint N = nlCurrentContext->n ;
    NLuint i ;
    NLdouble* diag = nlCurrentContext->M.diag ;
    for(i=0; i<N; i++) {
        xy[i] *= (diag[i] / omega) ;
    }
}

void nlMultDiagonalInverse(NLdouble* xy, NLdouble omega) {
    NLuint N = nlCurrentContext->n ;
    NLuint i ;
    NLdouble* diag = nlCurrentContext->M.diag ;
    for(i=0; i<N; i++) {
        xy[i] *= (omega / diag[i]) ;
    }
}

void nlMultLowerInverse(NLdouble* x, NLdouble* y, double omega) {
    NLSparseMatrix* A = &(nlCurrentContext->M) ;
    NLuint n       = A->n ;
    NLdouble* diag = A->diag ;
    NLuint i ;
    NLuint ij ;
    NLRowColumn*  Ri = NULL ;
    NLCoeff* c = NULL ;
    NLdouble S ;

    nl_assert(A->storage & NL_MATRIX_STORE_SYMMETRIC) ;
    nl_assert(A->storage & NL_MATRIX_STORE_ROWS) ;

    for(i=0; i<n; i++) {
        S = 0 ;
        Ri = &(A->row[i]) ;
        for(ij=0; ij < Ri->size; ij++) {
            c = &(Ri->coeff[ij]) ;
            nl_parano_assert(c->index <= i) ; 
            if(c->index != i) {
                S += c->value * y[c->index] ; 
            }
        }
        y[i] = (x[i] - S) * omega / diag[i] ;
    }
}

void nlMultUpperInverse(NLdouble* x, NLdouble* y, NLdouble omega) {
    NLSparseMatrix* A = &(nlCurrentContext->M) ;
    NLuint n       = A->n ;
    NLdouble* diag = A->diag ;
    NLint i ;
    NLuint ij ;
    NLRowColumn*  Ci = NULL ;
    NLCoeff* c = NULL ;
    NLdouble S ;

    nl_assert(A->storage & NL_MATRIX_STORE_SYMMETRIC) ;
    nl_assert(A->storage & NL_MATRIX_STORE_COLUMNS) ;

    for(i=n-1; i>=0; i--) {
        S = 0 ;
        Ci = &(A->column[i]) ;
        for(ij=0; ij < Ci->size; ij++) {
            c = &(Ci->coeff[ij]) ;
            nl_parano_assert(c->index >= i) ; 
            if(c->index != i) {
                S += c->value * y[c->index] ; 
            }
        }
        y[i] = (x[i] - S) * omega / diag[i] ;
    }
}


void nlPreconditioner_Jacobi(NLdouble* x, NLdouble* y) {
    NLuint N = nlCurrentContext->n ;
    dcopy(N, x, 1, y, 1) ;
    nlMultDiagonalInverse(y, 1.0) ;
}

void nlPreconditioner_SSOR(NLdouble* x, NLdouble* y) {
    NLdouble omega = nlCurrentContext->omega ;
    static double* work = NULL ;
    static int work_size = 0 ;
    NLuint n = nlCurrentContext->n ;
    if(n != work_size) {
        work = NL_RENEW_ARRAY(NLdouble, work, n) ;
        work_size = n ;
    }
    
    nlMultLowerInverse(x, work, omega) ;
    nlMultDiagonal(work, omega) ;
    nlMultUpperInverse(work, y, omega) ;

    dscal(n, 2.0 - omega, y, 1) ;
}

