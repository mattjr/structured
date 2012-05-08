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

#include <numeric_stuff/NL/nl_taucs.h>
#include <numeric_stuff/NL/nl_context.h>
#include <stdio.h>
#include <string.h>

/************************************************************************/
/* TAUCS wrapper */

#ifdef NL_USE_TAUCS

#include <numeric_stuff/TAUCS/taucs.h>

static char* nl_taucs_options[255] ;
static char* nl_taucs_options_file_name = NULL ;

static char* nl_taucs_default_options[] =  { 
    "taucs.factor.LLT=true", NULL 
} ;

void set_nl_taucs_options_file_name(char* file_name) {
    nl_taucs_options_file_name = file_name ;
}

static char** get_nl_taucs_options() {
    FILE* f = NULL;
    char line[255] ;
    int cur_option = 0 ;
    if(nl_taucs_options_file_name == NULL) {
        return nl_taucs_default_options ;
    }
    f = fopen(nl_taucs_options_file_name, "r")  ;
    if(f == NULL) {
        nlError("get_nl_taucs_options", "could not open options file") ;
        return nl_taucs_default_options ;
    }
    do {
        fscanf(f, "%s\n", line) ;
        nl_taucs_options[cur_option] = strdup(line) ;
        printf("taucs option: %s\n", line) ;
        cur_option++ ;
        nl_assert(cur_option < 255) ;
    } while(!feof(f)) ;
    nl_taucs_options[cur_option] = NULL ;
    return nl_taucs_options ;
}


NLboolean nlSolve_TAUCS() {

    /* OpenNL Context */
    NLSparseMatrix* M  = &(nlCurrentContext->M) ;
    NLdouble* b          = nlCurrentContext->b ;
    NLdouble* x          = nlCurrentContext->x ;

    /* Compressed Row Storage matrix representation */
    NLuint    n      = nlCurrentContext->n ;
    NLuint    nnz    = nlSparseMatrixNNZ(M) ; /* Number of Non-Zero coeffs */

    /* Temporary variables */
    NLRowColumn* Cj = NULL ;
    NLuint       ii,j,count ;

    /* TAUCS matrix */
    int taucs_type = TAUCS_DOUBLE ;
    int taucs_storage = nlCurrentContext->symmetric ? (TAUCS_SYMMETRIC | TAUCS_LOWER) : 0 ;
    taucs_ccs_matrix* taucs_M = taucs_ccs_create(n, n, nnz, taucs_type | taucs_storage) ;

    /* TAUCS options */
    char** taucs_options = get_nl_taucs_options() ;

    NLboolean taucs_result = NL_TRUE ;

    /* Sanity checks */
    nl_assert(M->storage & NL_MATRIX_STORE_COLUMNS) ;
    nl_assert(M->m == M->n) ;
    nl_assert(M->m == n);
    
    /*
     * Step 1: convert matrix M into TAUCS compressed column 
     *   representation.
     * -------------------------------------------------------
     */

    count = 0 ;
    for(j=0; j<n; j++) {
        Cj = &(M->column[j]) ;
        taucs_M->colptr[j] = count ;
        for(ii=0; ii<Cj->size; ii++) {
            taucs_M->values.d[count]    = Cj->coeff[ii].value ;
            taucs_M->rowind[count] = Cj->coeff[ii].index ;
            count++ ;
        }
    }
    taucs_M->colptr[j] = nnz ;


    taucs_result = (taucs_linsolve(taucs_M,NULL,1,x,b,taucs_options,NULL) == TAUCS_SUCCESS) ;

    taucs_ccs_free(taucs_M) ;

    return taucs_result ;
}

#else

NLboolean nlSolve_TAUCS() {
    nl_assert_not_reached ;

    return NL_FALSE ;
}

#endif

