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
 

#include <OGF/math/numeric/system_solver_taucs.h>
#include <OGF/math/linear_algebra/sparse_matrix.h>
#include <OGF/basic/os/file_system.h>
#include <sstream>

extern "C" {
#include <numeric_stuff/TAUCS/taucs.h>
}

namespace OGF {

//______________________________________________________________________________

    static taucs_ccs_matrix* graphite_to_taucs(const SparseMatrix& A) {

        ogf_assert(A.columns_are_stored()) ;

        int n = A.n() ;         // dimension
        int nnz = A.nnz() ; // number of non-zero coefficients
        
        // Create TAUCS matrix
        int taucs_type = TAUCS_DOUBLE ;
        int taucs_storage = A.has_symmetric_storage() ? (TAUCS_SYMMETRIC | TAUCS_LOWER) : 0 ;
        taucs_ccs_matrix* taucs_M = taucs_ccs_create(n, n, nnz, taucs_type | taucs_storage) ;

        // Convert Graphite Matrix into TAUCS Matrix
        int count = 0 ;
        for(int j=0; j<n; j++) {
            const SparseMatrix::Column& Cj = A.column(j) ;
            taucs_M->colptr[j] = count ;
            for(int ii=0; ii<Cj.nb_coeffs(); ii++) {
                taucs_M->values.d[count]    = Cj.coeff(ii).a ;
                taucs_M->rowind[count] = Cj.coeff(ii).index ;
                count++ ;
            }
        }
        taucs_M->colptr[n] = nnz ;

        return taucs_M ;
    }


//______________________________________________________________________________

#ifdef WIN32
#define OOC_BASENAME "C:/windows/temp/taucs.swap"
#else
#define OOC_BASENAME "/tmp/taucs.swap"
#endif

    static char* taucs_LLT_ooc_options[] = {
        "taucs.factor.LLT=true",
        "taucs.ooc=true",
        "taucs.ooc.basename=" OOC_BASENAME,
        "taucs.ooc.memory=536870912", // 512 Mb
        NULL
    } ;

    SystemSolverTaucs::SystemSolverTaucs() : options_(nil) { 
    }
    
    SystemSolverTaucs::~SystemSolverTaucs() { 
    }        
    
    static void cleanup_ooc() {
        for(unsigned int i=0; i<10; i++) {
            std::ostringstream o ;
            o << OOC_BASENAME << "." << i ;
            std::string ooc_filename = o.str() ;
            if(FileSystem::is_file(ooc_filename)) {
                FileSystem::delete_file(ooc_filename) ;
            }
        }
    }

    bool SystemSolverTaucs::solve(const SparseMatrix& A, Vector& x_in, const Vector& b_in) {
        cleanup_ooc() ;
        taucs_logfile("stderr") ;

        ogf_assert(A.n() == A.m()) ;

        double* x = x_in.data() ;
        double* b = const_cast<double*>(b_in.data()) ;

        taucs_ccs_matrix* taucs_M = graphite_to_taucs(A) ;
        int taucs_result = taucs_linsolve(taucs_M,NULL,1,x,b,options_,NULL) ;
        bool ok = ( taucs_result == TAUCS_SUCCESS) ;
        taucs_ccs_free(taucs_M) ;

        if(ok) {
            Logger::out("TAUCS") << "solved system" << std::endl ;
        } else {
            Logger::warn("TAUCS") << "failed, error code = " << taucs_result << std::endl ;
        }

        cleanup_ooc() ;
        return ok ;
    }
    
    bool SystemSolverTaucs::needs_rows() const {
        return false ;
    }

    bool SystemSolverTaucs::needs_columns() const {
        return true ;
    }

    //_________________________________________

    SystemSolverTaucs_LLT::SystemSolverTaucs_LLT() {
        set_options(taucs_LLT_ooc_options) ;
    }

    bool SystemSolverTaucs_LLT::supports_symmetric_storage() const {
        return true ;
    }

    //_________________________________________

    SystemSolverTaucs_LU::SystemSolverTaucs_LU() {
    }

    bool SystemSolverTaucs_LU::supports_symmetric_storage() const {
        return false ;
    }

    // TAUCS higher-level interface (taucs_linsolve) does not implement OOC LU yet (unfortunately),
    // therefore we cannot use SystemSolverTaucs::solve().
    bool SystemSolverTaucs_LU::solve(const SparseMatrix& A, Vector& x_in, const Vector& b_in) {


        taucs_logfile("stderr") ;
        ogf_assert(A.n() == A.m()) ;

        // Step 1: convert data into TAUCS representation.
        double* x = x_in.data() ;
        double* b = const_cast<double*>(b_in.data()) ;
        taucs_ccs_matrix* taucs_A = graphite_to_taucs(A) ;

        // Step 2: Compute reordering
        int* perm ; int* invperm ;
        taucs_ccs_order(taucs_A, &perm, &invperm, "colamd");
        
        // Step 3: Create OOC storage.
        cleanup_ooc() ;
        taucs_io_handle* LU = taucs_io_create_multifile(OOC_BASENAME);
        int taucs_result = taucs_ooc_factor_lu(taucs_A, perm, LU, 512.0 * 1048576.0);
        if(taucs_result == TAUCS_SUCCESS) {
            taucs_result = taucs_ooc_solve_lu(LU, x, b);
        }

        bool ok = ( taucs_result == TAUCS_SUCCESS) ;

        taucs_ccs_free(taucs_A) ;
        free(perm) ;
        free(invperm) ;

        if(ok) {
            Logger::out("TAUCS") << "solved system" << std::endl ;
        } else {
            Logger::warn("TAUCS") << "failed, error code = " << taucs_result << std::endl ;
        }

        cleanup_ooc() ;
        return ok ;
        
    }

    //_________________________________________

    SystemSolverTaucs_LDLT::SystemSolverTaucs_LDLT() {
    }

    bool SystemSolverTaucs_LDLT::supports_symmetric_storage() const {
        return false ;
    }

    bool SystemSolverTaucs_LDLT::solve(const SparseMatrix& A, Vector& x_in, const Vector& b_in) {


        taucs_logfile("stderr") ;
        ogf_assert(A.n() == A.m()) ;

        // Step 1: convert data into TAUCS representation.
        double* x = x_in.data() ;
        double* b = const_cast<double*>(b_in.data()) ;
        taucs_ccs_matrix* taucs_A = graphite_to_taucs(A) ;

        // Step 2: Compute reordering
//        int* perm ; int* invperm ;
//        taucs_ccs_order(taucs_A, &perm, &invperm, "colamd");
        
        // Step 3: Create OOC storage.
        cleanup_ooc() ;
        taucs_io_handle* L = taucs_io_create_multifile(OOC_BASENAME);
        int taucs_result = taucs_ooc_factor_ldlt(taucs_A, L, 512.0 * 1048576.0);
        if(taucs_result == TAUCS_SUCCESS) {
            taucs_result = taucs_ooc_solve_ldlt(L, x, b);
        }

        bool ok = ( taucs_result == TAUCS_SUCCESS) ;

        taucs_ccs_free(taucs_A) ;
//        free(perm) ;
//        free(invperm) ;

        if(ok) {
            Logger::out("TAUCS") << "solved system" << std::endl ;
        } else {
            Logger::warn("TAUCS") << "failed, error code = " << taucs_result << std::endl ;
        }

        cleanup_ooc() ;
        return ok ;
        
    }

    //_________________________________________

    class TAUCSMatrix : public AbstractMatrixImpl {
    public:
        TAUCSMatrix() : mode(OGF_TAUCS_NONE), n(0), flags(0), perm(nil), invperm(nil), Px(nil), Py(nil) { }

        virtual ~TAUCSMatrix() { 
	   free(perm) ; free(invperm) ; delete[] Px ; delete[] Py ;
	   switch(mode) {
	     case OGF_TAUCS_LDLT:
	       taucs_supernodal_factor_ldlt_free(L) ;
//	       taucs_ccs_free((taucs_ccs_matrix*)L) ;	       
	       break ;
	     case OGF_TAUCS_OOC_LDLT:
	       taucs_io_close((taucs_io_handle*)L) ;
	       break ;
	   }
	   L = nil ;
	}

        virtual int* permutation() const { return perm ; }

        void allocate(int n_in) {
            ogf_assert(n == 0) ;
            n = n_in ;
            Px = new double[n] ;
            Py = new double[n] ;
        }

        virtual void mult(double* y, const double* x) {
            taucs_vec_permute(n, flags, const_cast<double*>(x), Px, perm) ;
	   
	    switch(mode) {
	     case OGF_TAUCS_LDLT:
	       taucs_supernodal_solve_ldlt(L, Py, Px) ;
//	       taucs_ccs_solve_ldlt(L, Py, Px) ;	       
	       break ;
	     case OGF_TAUCS_OOC_LDLT:
	       taucs_ooc_solve_ldlt(L, Py, Px) ;
	       break ;
	    }

            taucs_vec_ipermute(n, flags, Py, y, perm);
        }

        OGFTaucsMode mode ;
        int n ;
        int flags ;
        int* perm ;
        int* invperm ;
        double* Px ; // Space to store permuted RHS
        double* Py ; // Space to store permuted solution
        void* L ;
    } ;

    bool invert_matrix_TAUCS(const SparseMatrix& A, AbstractMatrix& A_inv, int* perm, OGFTaucsMode mode) {

        std::cerr << "Calling invert_TAUCS_ldlt" << std::endl ;

        taucs_logfile("stderr") ;
        ogf_assert(A.n() == A.m()) ;
        int n = A.n() ;

        std::cerr << "Creating matrix implementation" << std::endl ;

        TAUCSMatrix* impl = new TAUCSMatrix ;
        impl->allocate(n) ;
        A_inv.set_impl(n, impl) ;

        std::cerr << "Converting matrix to TAUCS" << std::endl ;

        // Step 1: convert data into TAUCS representation.
        taucs_ccs_matrix* taucs_A = graphite_to_taucs(A) ;

        std::cerr << "Computing permutation" << std::endl ;
        
        // Step 2: compute ordering / permutation
        if(perm == nil) {
            taucs_ccs_order(taucs_A, &impl->perm, &impl->invperm, "metis");
        } else {
            for(int i=0; i<=n; i++) {
                impl->perm[i] = perm[i] ;
                impl->invperm[ perm[i] ] = i ;
            }
        }

        std::cerr << "Permuting matrix" << std::endl ;
        taucs_ccs_matrix* taucs_PAPT = taucs_ccs_permute_symmetrically(taucs_A,impl->perm,impl->invperm);
        taucs_ccs_free(taucs_A) ;

        impl->flags = taucs_PAPT->flags ;
        impl->mode = mode ;
       
        int taucs_result ; 
       
        switch(mode) {
	 case OGF_TAUCS_LDLT: {
	   void* LdLt = taucs_ccs_factor_ldlt_mf(taucs_PAPT);
	   taucs_result = (impl->L != nil) ? TAUCS_SUCCESS : TAUCS_ERROR ;
	   if(taucs_result == TAUCS_SUCCESS) {
	      impl->L = LdLt ;
//	      impl->L = taucs_supernodal_factor_ldlt_to_ccs(LdLt);
//	      taucs_supernodal_factor_ldlt_free(LdLt) ;
	   }
         } break ;
	 case OGF_TAUCS_OOC_LDLT:
	   cleanup_ooc() ;
	   impl->L = taucs_io_create_multifile(OOC_BASENAME);
	   taucs_result = taucs_ooc_factor_ldlt(taucs_PAPT, (taucs_io_handle*)(impl->L), 512.0 * 1048576.0) ;
	   break ;
	}
       
        bool ok = ( taucs_result == TAUCS_SUCCESS) ;
        if(ok) {
	   std::cerr << "Inverted matrix" << std::endl ;
	} else {
	   std::cerr << "TAUCS did not manage to Invert matrix" << std::endl ;	    
	}
       

        taucs_ccs_free(taucs_PAPT) ;

        return ok ;
    }

}

