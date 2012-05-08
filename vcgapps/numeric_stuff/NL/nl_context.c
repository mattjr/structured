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

#include <numeric_stuff/NL/nl_context.h>
#include <numeric_stuff/NL/nl_iterative_solvers.h>
#include <numeric_stuff/NL/nl_preconditioners.h>
#include <numeric_stuff/NL/nl_superlu.h>
#include <numeric_stuff/NL/nl_taucs.h>

NLContextStruct* nlCurrentContext = NULL ;

void nlMatrixVectorProd_default(NLdouble* x, NLdouble* y) {
    nlSparseMatrixMult(&(nlCurrentContext->M), x, y) ;
}

NLContext nlNewContext() {
    NLContextStruct* result  = NL_NEW(NLContextStruct) ;
    result->state            = NL_STATE_INITIAL ;
    result->solver           = NL_BICGSTAB ;
    result->max_iterations   = 100 ;
    result->threshold        = 1e-6 ;
    result->omega            = 1.5 ;
    result->row_scaling      = 1.0 ;
    result->right_hand_side  = 0.0 ;
    result->inner_iterations = 5 ;
    result->matrix_vector_prod = nlMatrixVectorProd_default ;
    result->solver_func = nlDefaultSolver ;
    nlMakeCurrent(result) ;
    return result ;
}

void nlDeleteContext(NLContext context_in) {
    NLContextStruct* context = (NLContextStruct*)(context_in) ;
    if(nlCurrentContext == context) {
        nlCurrentContext = NULL ;
    }
    if(context->alloc_M) {
        nlSparseMatrixDestroy(&context->M) ;
    }
    if(context->alloc_af) {
        nlRowColumnDestroy(&context->af) ;
    }
    if(context->alloc_al) {
        nlRowColumnDestroy(&context->al) ;
    }
    if(context->alloc_xl) {
        nlRowColumnDestroy(&context->xl) ;
    }
    if(context->alloc_variable) {
        NL_DELETE_ARRAY(context->variable) ;
    }
    if(context->alloc_x) {
        NL_DELETE_ARRAY(context->x) ;
    }
    if(context->alloc_b) {
        NL_DELETE_ARRAY(context->b) ;
    }

#ifdef NL_PARANOID
    NL_CLEAR(context) ;
#endif
    NL_DELETE(context) ;
}

void nlMakeCurrent(NLContext context) {
    nlCurrentContext = (NLContextStruct*)(context) ;
}

NLContext nlGetCurrent() {
    return nlCurrentContext ;
}

/************************************************************************/
/* Finite state automaton   */

void nlCheckState(NLenum state) {
    nl_assert(nlCurrentContext->state == state) ;
}

void nlTransition(NLenum from_state, NLenum to_state) {
    nlCheckState(from_state) ;
    nlCurrentContext->state = to_state ;
}

/************************************************************************/
/* Default solver */

static void nlSetupPreconditioner() {
    switch(nlCurrentContext->preconditioner) {
    case NL_PRECOND_NONE:
        nlCurrentContext->precond_vector_prod = NULL ;
        break ;
    case NL_PRECOND_JACOBI:
        nlCurrentContext->precond_vector_prod = nlPreconditioner_Jacobi ;
        break ;
    case NL_PRECOND_SSOR:
        nlCurrentContext->precond_vector_prod = nlPreconditioner_SSOR ;
        break ;
    default:
        nl_assert_not_reached ;
        break ;
    }
    /* Check compatibility between solver and preconditioner */
    if(
        nlCurrentContext->solver == NL_BICGSTAB && 
        nlCurrentContext->preconditioner == NL_PRECOND_SSOR
    ) {
        nlWarning(
            "nlSolve", 
            "cannot use SSOR preconditioner with non-symmetric matrix, switching to Jacobi"
        ) ;
        nlCurrentContext->preconditioner = NL_PRECOND_JACOBI ;        
        nlCurrentContext->precond_vector_prod = nlPreconditioner_Jacobi ;
    }
    if(
        nlCurrentContext->solver == NL_GMRES && 
        nlCurrentContext->preconditioner != NL_PRECOND_NONE
    ) {
        nlWarning("nlSolve", "Preconditioner not implemented yet for GMRES") ;
        nlCurrentContext->preconditioner = NL_PRECOND_NONE ;        
        nlCurrentContext->precond_vector_prod = NULL ;
    }
    if(
        nlCurrentContext->solver == NL_SUPERLU_EXT && 
        nlCurrentContext->preconditioner != NL_PRECOND_NONE
    ) {
        nlWarning("nlSolve", "Preconditioner not implemented yet for SUPERLU") ;
        nlCurrentContext->preconditioner = NL_PRECOND_NONE ;        
        nlCurrentContext->precond_vector_prod = NULL ;
    }
    if(
        nlCurrentContext->solver == NL_PERM_SUPERLU_EXT && 
        nlCurrentContext->preconditioner != NL_PRECOND_NONE
    ) {
        nlWarning("nlSolve", "Preconditioner not implemented yet for PERMSUPERLU") ;
        nlCurrentContext->preconditioner = NL_PRECOND_NONE ;        
        nlCurrentContext->precond_vector_prod = NULL ;
    }
    if(
        nlCurrentContext->solver == NL_SYMMETRIC_SUPERLU_EXT && 
        nlCurrentContext->preconditioner != NL_PRECOND_NONE
    ) {
        nlWarning("nlSolve", "Preconditioner not implemented yet for PERMSUPERLU") ;
        nlCurrentContext->preconditioner = NL_PRECOND_NONE ;        
        nlCurrentContext->precond_vector_prod = NULL ;
    }
    if(
        nlCurrentContext->solver == NL_TAUCS_EXT &&
        nlCurrentContext->preconditioner != NL_PRECOND_NONE
    ) {
        nlWarning("nlSolve", "Preconditioner not implemented yet for TAUCS") ;
        nlCurrentContext->preconditioner = NL_PRECOND_NONE ;        
        nlCurrentContext->precond_vector_prod = NULL ;
    }
}

NLboolean nlDefaultSolver() {
    NLboolean result = NL_TRUE ;
    nlSetupPreconditioner() ;
    switch(nlCurrentContext->solver) {
    case NL_CG: {
        if(nlCurrentContext->preconditioner == NL_PRECOND_NONE) {
            nlCurrentContext->used_iterations = nlSolve_CG() ;
        } else {
            nlCurrentContext->used_iterations = nlSolve_CG_precond() ;
        }
    } break ;
    case NL_BICGSTAB: {
        if(nlCurrentContext->preconditioner == NL_PRECOND_NONE) {
            nlCurrentContext->used_iterations = nlSolve_BICGSTAB() ;
        } else {
            nlCurrentContext->used_iterations = nlSolve_BICGSTAB_precond() ;
        }
    } break ;
    case NL_GMRES: {
        nlCurrentContext->used_iterations = nlSolve_GMRES() ;
    } break ;
#ifdef NL_USE_SUPERLU
    case NL_SUPERLU_EXT: 
    case NL_PERM_SUPERLU_EXT: 
    case NL_SYMMETRIC_SUPERLU_EXT: {
        result = nlSolve_SUPERLU() ;
    } break ;
#else
    case NL_SUPERLU_EXT: 
    case NL_PERM_SUPERLU_EXT: 
    case NL_SYMMETRIC_SUPERLU_EXT: {
        nl_assert_not_reached ;
    } break ;
#endif
#ifdef NL_USE_TAUCS
    case NL_TAUCS_EXT: {
        result = nlSolve_TAUCS() ;
    } break ;
#else
    case NL_TAUCS_EXT: {
        nl_assert_not_reached ;
    } break ;
#endif       
    default:
        nl_assert_not_reached ;
    }
    return result ;
}
