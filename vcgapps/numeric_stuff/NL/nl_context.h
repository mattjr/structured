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

#ifndef __NL_CONTEXT__
#define __NL_CONTEXT__

#include <numeric_stuff/NL/nl_private.h>
#include <numeric_stuff/NL/nl_matrix.h>

/************************************************************************************/
/* NLContext data structure */

typedef void(*NLMatrixFunc)(double* x, double* y) ;
typedef NLboolean(*NLSolverFunc)() ;

typedef struct {
    NLdouble  value ;
    NLboolean locked ;
    NLuint    index ;
} NLVariable ;

#define NL_STATE_INITIAL                               0
#define NL_STATE_SYSTEM                             1
#define NL_STATE_MATRIX                              2
#define NL_STATE_ROW                                   3
#define NL_STATE_MATRIX_CONSTRUCTED  4
#define NL_STATE_SYSTEM_CONSTRUCTED 5
#define NL_STATE_SOLVED                             6

typedef struct {
    NLenum           state ;
    NLVariable*    variable ;
    NLuint            n ;
    NLenum         matrix_store ;
    NLSparseMatrix M ;
    NLRowColumn    af ;
    NLRowColumn    al ;
    NLRowColumn    xl ;
    NLdouble*        x ;
    NLdouble*        b ;
    NLdouble         right_hand_side ;
    NLdouble         row_scaling ;
    NLenum           solver ;
    NLenum           preconditioner ;
    NLuint           nb_variables ;
    NLuint           current_row ;
    NLboolean        least_squares ;
    NLboolean        symmetric ;
    NLuint           max_iterations ;
    NLuint           inner_iterations ;
    NLdouble         threshold ;
    NLdouble         omega ;
    NLboolean        normalize_rows ;
    NLboolean        alloc_M ;
    NLboolean        alloc_af ;
    NLboolean        alloc_al ;
    NLboolean        alloc_xl ;
    NLboolean        alloc_variable ;
    NLboolean        alloc_x ;
    NLboolean        alloc_b ;
    NLuint           used_iterations ;
    NLdouble         error ;
    NLdouble         elapsed_time ;
    NLMatrixFunc   matrix_vector_prod ;
    NLMatrixFunc   precond_vector_prod ;
    NLSolverFunc  solver_func ;
} NLContextStruct ;

extern NLContextStruct* nlCurrentContext ;

void nlCheckState(NLenum state) ;
void nlTransition(NLenum from_state, NLenum to_state) ;

void nlMatrixVectorProd_default(NLdouble* x, NLdouble* y) ;
NLboolean nlDefaultSolver() ;

#endif
