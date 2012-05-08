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

#ifndef __NL_MATRIX__
#define __NL_MATRIX__

/************************************************************************************/
/* Dynamic arrays for sparse row/columns */

typedef struct {
    NLuint   index ;
    NLdouble value ;
} NLCoeff ;

typedef struct {
    NLuint size ;
    NLuint capacity ;
    NLCoeff* coeff ;
} NLRowColumn ;

void nlRowColumnConstruct(NLRowColumn* c) ;
void nlRowColumnDestroy(NLRowColumn* c) ;
void nlRowColumnGrow(NLRowColumn* c) ;
void nlRowColumnAdd(NLRowColumn* c, NLint index, NLdouble value) ;
void nlRowColumnAppend(NLRowColumn* c, NLint index, NLdouble value) ;
void nlRowColumnZero(NLRowColumn* c) ;
void nlRowColumnClear(NLRowColumn* c) ;
void nlRowColumnSort(NLRowColumn* c) ;

/************************************************************************************/
/* SparseMatrix data structure */

typedef struct {
    NLuint m ;
    NLuint n ;
    NLuint diag_size ;
    NLenum storage ;
    NLRowColumn* row ;
    NLRowColumn* column ;
    NLdouble*      diag ;
} NLSparseMatrix ;


void nlSparseMatrixConstruct(
    NLSparseMatrix* M, NLuint m, NLuint n, NLenum storage
) ;

void nlSparseMatrixDestroy(NLSparseMatrix* M) ;

void nlSparseMatrixAdd(
    NLSparseMatrix* M, NLuint i, NLuint j, NLdouble value
) ;

void nlSparseMatrixZero( NLSparseMatrix* M) ;
void nlSparseMatrixClear( NLSparseMatrix* M) ;

/* returns the number of stored non-zero coefficients */
NLuint nlSparseMatrixNNZ( NLSparseMatrix* M) ;

/* sorts the rows and/or columns (according to storage) in increasing j/i */
void nlSparseMatrixSort( NLSparseMatrix* M) ;

/************************************************************************************/
/* SparseMatrix x Vector routine */
/* y <- A x                      */

void nlSparseMatrixMult(NLSparseMatrix* A, NLdouble* x, NLdouble* y) ;

#endif
