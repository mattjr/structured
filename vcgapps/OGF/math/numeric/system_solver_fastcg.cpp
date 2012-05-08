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

// http://www.research.ibm.com/journal/rd/416/toledo.html
// Pre-ordering: Lin-Kernigham, Cuthill-McKee

#include <OGF/math/numeric/system_solver_fastcg.h>
#include <OGF/math/numeric/conjugate_gradient.h>
#include <OGF/math/numeric/sparse_matrix_crs.h>
#include <OGF/math/numeric/sparse_matrix_conversion.h>
#include <OGF/math/numeric/quick_blas.h>
#include <OGF/math/linear_algebra/sparse_matrix.h>

#include <fstream>

namespace OGF {

    SystemSolver_FASTCG::SystemSolver_FASTCG() { 
    }
    
    SystemSolver_FASTCG::~SystemSolver_FASTCG() { 
    }        

    template <class MatrixType, class VectorType> inline void solve_cg(
        const MatrixType& A, VectorType& x, const VectorType& b, double eps, unsigned int max_iter
    ) {
        QuickBLAS::flops_counter = 0 ;

        unsigned int N = x.size() ;

        VectorType r(N, alignment_for_SSE2) ;
        VectorType d(N, alignment_for_SSE2) ;
        VectorType h(N, alignment_for_SSE2) ;
        VectorType& Ad = h ;

        VectorType diag_inv(N, alignment_for_SSE2) ;
        for(unsigned int i=0; i<N; i++) {
            diag_inv[i] = (i >= A.diag.size() || A.diag[i] == 0.0) ? 1.0 : 1.0 / A.diag[i] ;
        }

        unsigned int its=0;
        double rh, alpha, beta;
        double bnorm2 = QuickBLAS::squared_nrm2(b) ;
        double err = eps * eps * bnorm2 ;
  
        A.mult(x,r) ;
        QuickBLAS::axpy(-1.,b,r);

        // d = PRECOND * r
        QuickBLAS::mul(diag_inv, r, d) ;

        QuickBLAS::copy(d,h);
        rh=QuickBLAS::dot(r,h);
        
        double cur_err ;

        SystemStopwatch watch ;
        ProcessorStopwatch p_watch ;

        while ( (cur_err = QuickBLAS::squared_nrm2(r))>err && its < max_iter) {
            if(!(its & 127)) {
                std::cerr << its << " : " << cur_err << " -- " << err << std::endl ;
            }

            A.mult(d,Ad);
            alpha=rh/QuickBLAS::dot(d,Ad);
            QuickBLAS::axpy(-alpha,d,x);
            QuickBLAS::axpy(-alpha,Ad,r);

            // h = PRECOND * r
            QuickBLAS::mul(diag_inv, r, h) ;

            beta=1./rh; rh=QuickBLAS::dot(r,h); beta*=rh;
            QuickBLAS::scal(beta,d);
            QuickBLAS::axpy(1.,h,d);
            ++its;
        }
        std::cerr << "CG used " << its << " iterations" << std::endl ;
        double seconds = watch.elapsed_user_time() ;
        std::cerr << "CG main loop time: " << seconds << std::endl ;
        std::cerr << "     Processor clock ticks: " << 1e-9 * double(p_watch.elapsed_time()) << " G" << std::endl ;
        if(seconds != 0.0) {
           double mflops = (1e-6 * double(QuickBLAS::flops_counter) / seconds) ;
           std::cerr << "MFlops = " << mflops << std::endl ;
	}
    }


    static void compute_permutation(const SparseMatrix& A, Permutation& permutation) {
        permutation.allocate(A.n()) ;
        bool RCMK = false ;
        if(RCMK) {
            SparseMatrixPatternCRS A_pattern ;
            ::OGF::convert_matrix(A, A_pattern) ;
            A_pattern.compute_RCMK_ordering(permutation) ;
            if(true) {
                std::cerr << "Bandwidth before permutation: " << A_pattern.bandwidth() << std::endl ;
                SparseMatrixCRS<double> A_crs ;
                ::OGF::convert_matrix(A, A_crs, true, permutation) ;            
                std::cerr << "Bandwidth after permutation: " << A_crs.bandwidth() << std::endl ;
            }
        } else {
            permutation.load_identity() ;
        }
    }

    bool SystemSolver_FASTCG::solve(const SparseMatrix& A_in, Vector& x_in, const Vector& b_in) {

        typedef double CoeffType ;
        typedef Array1d<CoeffType> VectorType ;
        typedef SparseMatrixBCRS<CoeffType, 2, 2> MatrixType ;

        unsigned int N0 = A_in.n() ;
        std::cerr << "N0 = " << N0 << std::endl ;
        Permutation permutation; 
        compute_permutation(A_in, permutation) ;

        MatrixType A ;
        ::OGF::convert_matrix(A_in, A, permutation) ;

//      ::OGF::compress_indices(A) ;

        std::cerr << "filling ratio:" << (A.filling_ratio() * 100.0) << "%" << std::endl ;
        if(false) {
            std::cerr << "Saving matrix to disk (matrix.dat)" << std::endl ;
            std::ofstream out("matrix.dat") ;
//            A.print(out) ;
            ::OGF::output_matrix(A, out) ;
        }

        unsigned int N = A.n() ; // Can be greater than N0 due to blocking
        N = QuickBLAS::aligned_size(N, sizeof(CoeffType)) ;

        std::cerr <<"N = " << N << std::endl ;

        int max_iter = (nb_iters_ == 0) ? 5 * N : nb_iters_ ;
        double eps = threshold_ ;

        std::cerr << "nb iters = " << max_iter << "  threshold = " << eps << std::endl ;

        VectorType b(N, alignment_for_SSE2) ;
        VectorType x(N, alignment_for_SSE2) ;

        permutation.invert_permute_vector(b_in, b) ;
        permutation.invert_permute_vector(x_in, x) ;

        solve_cg(A, x, b, eps, max_iter) ;

        permutation.permute_vector(x, x_in) ;

        return true ;
    }

    
    bool SystemSolver_FASTCG::needs_rows() const {
        return true ;
    }  
    
    bool SystemSolver_FASTCG::needs_columns() const {
        return false ;
    } 
    
    bool SystemSolver_FASTCG::supports_symmetric_storage() const {
        return false ;
    } 

}

