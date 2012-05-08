/*
 *  OGF/Graphite: Geometry and Graphics Programming Library + Utilities
 *  Copyright (C) 2000-2005 INRIA - Project ALICE
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
 *  Contact: Bruno Levy - levy@loria.fr
 *
 *     Project ALICE
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs. 
 *
 * As an exception to the GPL, Graphite can be linked with the following (non-GPL) libraries:
 *     Qt, SuperLU, WildMagic and CGAL
 */
 
#include <OGF/math/numeric/system_solver_cholmod.h>
#include <OGF/math/linear_algebra/sparse_matrix.h>
#include <numeric_stuff/CHOLMOD/cholmod.h>

namespace OGF {

    SystemSolverCholmod::SystemSolverCholmod() { 
    }
    
    SystemSolverCholmod::~SystemSolverCholmod() { 
    }        
    
    bool SystemSolverCholmod::solve(const SparseMatrix& A_in, Vector& x_out, const Vector& b_in) {
        ogf_assert(A_in.n() == A_in.m()) ;
        ogf_assert(A_in.has_symmetric_storage()) ;

        // Step 1: initialize CHOLMOD library
        //----------------------------------------------------
        cholmod_common c ;
        cholmod_start(&c) ;

        int N = A_in.n() ;
        int NNZ = A_in.nnz() ;

        // Step 2: translate sparse matrix into cholmod format
        //---------------------------------------------------------------------------
        cholmod_sparse* A = cholmod_allocate_sparse(N, N, NNZ, false, true, -1, CHOLMOD_REAL, &c);
        
        int* colptr = static_cast<int*>(A->p) ;
        int* rowind = static_cast<int*>(A->i) ;
        double* a = static_cast<double*>(A->x) ;
        
        // Convert Graphite Matrix into CHOLMOD Matrix
        int count = 0 ;
        for(int j=0; j<N; j++) {
            const SparseMatrix::Column& Cj = A_in.column(j) ;
            colptr[j] = count ;
            for(int ii=0; ii<Cj.nb_coeffs(); ii++) {
                a[count]    = Cj.coeff(ii).a ;
                rowind[count] = Cj.coeff(ii).index ;
                count++ ;
            }
        }
        colptr[N] = NNZ ;
        

        // Step 2: construct right-hand side
        cholmod_dense* b = cholmod_allocate_dense(N, 1, N, CHOLMOD_REAL, &c) ;
        Memory::copy(b->x, b_in.data(), N * sizeof(double)) ;
        
        // Step 3: factorize
        cholmod_factor* L = cholmod_analyze(A, &c) ;
        cholmod_factorize(A, L, &c) ;
        cholmod_dense* x = cholmod_solve(CHOLMOD_A, L, b, &c) ;
        Memory::copy(x_out.data(), x->x, N * sizeof(double)) ;

        // Step 5: cleanup
        cholmod_free_factor(&L, &c) ;
        cholmod_free_sparse(&A, &c) ;
        cholmod_free_dense(&x, &c) ;
        cholmod_free_dense(&b, &c) ;
        cholmod_finish(&c) ;
		return true;
    }
    
    bool SystemSolverCholmod::needs_rows() const {
        return false ;
    }  
    
    bool SystemSolverCholmod::needs_columns() const {
        return true ;
    } 
    
    bool SystemSolverCholmod::supports_symmetric_storage() const {
        return true ;
    } 
    
}

