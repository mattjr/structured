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
 

#include <OGF/math/numeric/preconditioner.h>
#include <OGF/math/numeric/blas.h>
#include <OGF/math/linear_algebra/sparse_matrix.h>
#include <OGF/basic/types/types.h>

namespace OGF {

//_________________________________________________________

    Preconditioner::Preconditioner(
        const SparseMatrix& A, double omega 
    ) : A_(A), omega_(omega) {
        ogf_assert(A.is_square()) ;
    }
    

    void Preconditioner::mult_lower_inverse(
        const double* x, double* y
    ) const {
        ogf_assert(A_.has_symmetric_storage()) ;
        ogf_assert(A_.rows_are_stored()) ;
        int n = A_.n() ;
        for(int i=0; i<n; i++) {
            double S = 0 ;
            const SparseMatrix::Row& Ri = A_.row(i) ;  
            for(int ij=0; ij < Ri.nb_coeffs(); ij++) {
                const Coeff& c = Ri.coeff(ij) ;
                ogf_debug_assert(c.index < i) ;
                S += c.a * y[c.index] ; 
            }
            y[i] = (x[i] - S) * omega_ / A_.diag(i) ;
        }
    }
    
    void Preconditioner::mult_upper_inverse(
        const double* x, double* y
    ) const {
        ogf_assert(A_.has_symmetric_storage()) ;
        ogf_assert(A_.columns_are_stored()) ;
        int n = A_.n() ;
        for(int i=n-1; i>=0; i--) {
            double S = 0 ;
            const SparseMatrix::Column& Ci = A_.column(i) ;  
            for(int ij=0; ij < Ci.nb_coeffs(); ij++) {
                const Coeff& c = Ci.coeff(ij) ;
                ogf_debug_assert(c.index > i) ;
                S += c.a * y[c.index] ; 
            }
            y[i] = (x[i] - S) * omega_ / A_.diag(i) ;
        }
    }
    
    void Preconditioner::mult_diagonal(double* xy) const {
        int n = A_.n() ;
        for(int i=0; i<n; i++) {
            xy[i] *= ( A_.diag(i) / omega_ ) ;
        }
    }

    void Preconditioner::mult_diagonal_inverse(double* xy) const {
        int n = A_.n() ;
        for(int i=0; i<n; i++) {
            xy[i] *= ( omega_ / A_.diag(i)  ) ;
        }
    }


//______________________________________________

    Jacobi_Preconditioner::Jacobi_Preconditioner(
        const SparseMatrix& A, double omega 
    ) : Preconditioner(A, omega) {
    }

    void mult(const Jacobi_Preconditioner& M, const double* x, double* y) {
        BLAS::dcopy(M.A().n(), x, 1, y, 1) ;
        M.mult_diagonal_inverse(y) ;
    }    

//______________________________________________


    SSOR_Preconditioner::SSOR_Preconditioner(
        const SparseMatrix& A, double omega 
    ) : Preconditioner(A, omega) {
    } 

    void mult(const SSOR_Preconditioner& M, const double* x, double* y) {

        static double* work = nil ;
        static int work_size = 0 ;

        const SparseMatrix& A = M.A() ;
        int n = A.n() ;

        if(work_size != n) {
            delete[] work ;
            work = new double[n] ;
        }
        
        M.mult_lower_inverse(x, work) ;
        M.mult_diagonal(work) ;
        M.mult_upper_inverse(work, y) ;

        BLAS::dscal(n, 2 - M.omega(), y, 1) ;
    }    
//_________________________________________________________

}

