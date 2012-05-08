
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
 

// #define OGF_PARANOID


#include <OGF/math/numeric/eigen_solver_arpack.h>
#include <OGF/math/numeric/blas.h>
#include <OGF/basic/debug/progress.h>
#include <OGF/math/types/math_library.h>

#include <algorithm>

#define LOCAL_OGF_FORTRAN(x) x##_ 

// _________________________________________________________________
//
//              ARPACK Fortran routine prototypes
// _________________________________________________________________

typedef int ARint;
typedef int ARlogical;

extern "C" {

// double precision symmetric routines.

    void LOCAL_OGF_FORTRAN(dsaupd)(
        ARint *ido, char *bmat, ARint *n, char *which,
        ARint *nev, double *tol, double *resid,
        ARint *ncv, double *V, ARint *ldv,
        ARint *iparam, ARint *ipntr, double *workd,
        double *workl, ARint *lworkl, ARint *info
    ) ;

    void LOCAL_OGF_FORTRAN(dseupd)(
        ARlogical *rvec, char *HowMny, ARlogical *select,
        double *d, double *Z, ARint *ldz,
        double *sigma, char *bmat, ARint *n,
        char *which, ARint *nev, double *tol,
        double *resid, ARint *ncv, double *V,
        ARint *ldv, ARint *iparam, ARint *ipntr,
        double *workd, double *workl,
        ARint *lworkl, ARint *info
    ) ;

// double precision nonsymmetric routines.
    
    void LOCAL_OGF_FORTRAN(dnaupd)(
        ARint *ido, char *bmat, ARint *n, char *which,
        ARint *nev, double *tol, double *resid,
        ARint *ncv, double *V, ARint *ldv,
        ARint *iparam, ARint *ipntr, double *workd,
        double *workl, ARint *lworkl, ARint *info
    ) ;

    void LOCAL_OGF_FORTRAN(dneupd)(
        ARlogical *rvec, char *HowMny, ARlogical *select,
        double *dr, double *di, double *Z,
        ARint *ldz, double *sigmar,
        double *sigmai, double *workev,
        char *bmat, ARint *n, char *which,
        ARint *nev, double *tol, double *resid,
        ARint *ncv, double *V, ARint *ldv,
        ARint *iparam, ARint *ipntr,
        double *workd, double *workl,
        ARint *lworkl, ARint *info
    ) ;
}

//_____________________________________________________________________________________


//_____________________________________________________________________________________

namespace OGF {

    // to += mul * from
    void add_matrix(SparseMatrix& to, const SparseMatrix& from, double mul = 1) {
        ogf_assert(from.m() == to.m()) ;
        ogf_assert(from.n() == to.n()) ;

        bool symmetrize = (from.has_symmetric_storage() && !to.has_symmetric_storage()) ;
       
        if(from.rows_are_stored()) {
            for(unsigned int i=0; i<(unsigned int)from.m(); i++) {
                const SparseMatrix::Row& Ri = from.row(i) ;
                for(unsigned int jj=0; jj<(unsigned int)Ri.nb_coeffs(); jj++) {
                    unsigned int j = Ri.coeff(jj).index ;
                    double a = Ri.coeff(jj).a ;
                    a *= mul ;
                    to.add(i,j,a) ;
                    if( symmetrize && i != j) {
                        to.add(j,i,a) ;
                    }
                }
            }
        } else {
            ogf_assert(from.columns_are_stored()) ;
            for(unsigned int j=0; j<(unsigned int)from.n(); j++) {
                const SparseMatrix::Column& Cj = from.column(j) ;
                for(unsigned int ii=0; ii<(unsigned int)Cj.nb_coeffs(); ii++) {
                    unsigned int i = Cj.coeff(ii).index ;
                    double a = Cj.coeff(ii).a ;
                    a *= mul ;
                    to.add(i,j,a) ;
                    if( symmetrize && i != j) {
                        to.add(j,i,a) ;
                    }
                }
            }
        }
    }


    class EigenCompare : public std::binary_function<int, int, bool> {
    public:
        EigenCompare(const Vector& v) : vector_(v) { }
        bool operator()(int a, int b) {
            return (::fabs(vector_(a)) < ::fabs(vector_(b))) ;
        }
    private:
        const Vector& vector_ ;
    } ;

    EigenSolver_ARPACK::EigenSolver_ARPACK() : iparam_(11), ipntr_(14) { 
        A_cache_ = nil ;
        B_cache_ = nil ;
        uses_perm_cache_ = false ;
    }
    
    EigenSolver_ARPACK::~EigenSolver_ARPACK() { 
    }        
    
    bool EigenSolver_ARPACK::solve() {

        std::cerr << "SHIFT_INVERT=" << shift_invert_ << std::endl ;

        ogf_assert(A_->m() == A_->n()) ;
        
        n_ = A_->n() ;

        
        bruno_ = bruno_ && shift_invert_ ;
        shift_invert_ = shift_invert_ && !bruno_ ;

        // Note: it would be tempting to set symmetric_
	// when using bruno_ mode and when both A_ and 
	// B_ are symmetric but this does not work since
	// we are looking for the eigenvalues of B^{-1}*A
	// that is not symmetric...
        symmetric_ = A_->is_symmetric()  && (B_ == nil) ;

        std::cerr << "BRUNO=" << bruno_ << " " << "SHIFT_INVERT="  << shift_invert_ << std::endl ;

        if(!setup_OP()) { return false ;  }

        
        int nev = nb_eigens_ ;    // Number of eigen vectors requested
        double factor = 2.5 ;       // Length of Arnoldi factorization relative to number of eigenvalues requested.
                                                // ARPACK doc mentions this rule of thumb: ncv > 2 * nev
        int ncv = int(nev * factor) ;  // Length of Arnoldi factorization

        if(ncv > n_) { ncv = n_ ; }
        if(nev > n_) { nev = n_ ; }
//        if(nev + 2 > ncv) {  ncv = nev + 2 ;   }
        if(nev + 2 > ncv) {  nev = ncv - 2 ;   }        

        int ldv = n_ ;
        
        char* bmat = "I" ;          // Standard problem
        if(B_ != nil && shift_invert_) {
            bmat = "G" ; // Generalized eigenproblem
            // Rem: when not using shift_invert,
            // we use standard ARPACK computational mode and 
            // multiply by B-1 by ourselves
        }

        char* which = nil ;
        
        if(bruno_) {
            if(mode_ == SMALLEST) { which = "LM" ; } else { which = "SM" ; }
        } else {
            if(mode_ == SMALLEST) { which = "SM" ; } else { which = "LM" ; }
        }

        ogf_assert(nev + 2 <= ncv) ;

        double tol = epsilon_ ;
        int ido = 0 ;
//        int info = 0 ; // start with a random initial vector
        int info = 1 ; // start with initial value of resid

        int lworkl ; // Size of work array
        if(symmetric_) {
            lworkl = ncv * (ncv + 8) ;
        } else {
            lworkl = 3*ncv*ncv + 6*ncv ; 
        }

        iparam(1) = 1 ; // Let the library choose the shifts
        iparam(3) = max_iter_ ;
        if(shift_invert_) {
            iparam(7) = 3 ; // shift-invert mode
        } else {
            iparam(7) = 1 ; // normal mode
/*
            if(B_ == nil && !invert_) { // <- this invert denotes my invert mode
                iparam(7) = 2 ; // <- this one is Arpack's invert mode
            }
*/
        }
        
        Vector ax(n_), resid(n_) ;
        Vector workev(3*ncv) ;
        
        workd_.allocate(3*n_) ;

        if(resid_cache_.size() == resid.size()) {
            std::cerr << "  Initializing from previous result" << std::endl ;
            for(unsigned int i=0; i<resid.size(); i++) {
                resid[i] = resid_cache_[i] ;
            }
            info = 1 ; // use initial value of resid
        } else {
            // For Laplacian eigenproblems, this is a reasonable initial value.
            // resid.set_all(1.0) ;
            for(unsigned int i=0; i<resid.size(); i++) {
                resid[i] = double(i) / double(resid.size()) ;
            }
        }

        workl_.allocate(lworkl) ;

        v_.allocate(ldv*ncv) ;        
        if(symmetric_) {
            d_.allocate(2*ncv) ;
        } else {
            d_.allocate(3*ncv) ;
        }
        
        bool converged = false ;
        ProgressLogger progress(max_iter_) ;
        while(!progress.is_canceled() && !converged) {
            if(symmetric_) {
                LOCAL_OGF_FORTRAN(dsaupd)(
                    &ido, bmat, &n_, which, &nev, &tol, resid.data(), &ncv,
                    v_.data(), &ldv, iparam_.data(), ipntr_.data(), 
                    workd_.data(), workl_.data(), &lworkl, &info
                ) ;
            } else {
                LOCAL_OGF_FORTRAN(dnaupd)(
                    &ido, bmat, &n_, which, &nev, &tol, resid.data(), &ncv,
                    v_.data(), &ldv, iparam_.data(), ipntr_.data(), 
                    workd_.data(), workl_.data(), &lworkl, &info
                ) ;
            }

            if(bruno_) {
                converged = bruno_iter(ido) ; 
            } else if(symmetric_) {
                if(B_ == nil) {
                    if(!shift_invert_) {
                        converged = sym_regular(ido) ;
                    } else {
                        converged = sym_shift_invert(ido) ;
                    }
                } else {
                    if(shift_invert_) {
                        converged = sym_shift_invert_gen(ido) ;
                    } else {
                        converged = sym_invert_gen(ido) ;
                    }
                }
            } else {
                if(B_ == nil) {
                    if(!shift_invert_) {
                        converged = nsym_regular(ido) ;
                    } else {
                        converged = nsym_shift_invert(ido) ;
                    }
                } else {
                    if(!shift_invert_) {
                        converged = nsym_regular_gen(ido) ;
                    } else {
                        converged = nsym_with_B_invert(ido) ;
                    }
                }
            }

            // Be less verbose in the case of multiple calls
            // (like in full-spectrum computation using shift-invert mode)
            if(!uses_perm_cache_) {
                progress.next() ;
            }
            std::cerr << "." << std::flush ;
        }
         
        std::cerr << std::endl ;

        if(info < 0) {
            std::cerr << "Error with d[s|n]aupd(): " << info << std::endl ;
            return false ;
        }
        std::cerr << "d[s|n]aupd(): success" << std::endl ;


        Array1d<ARlogical> select(ncv) ;
        select.set_all(true) ;
        ARlogical rvec = compute_eigen_vectors_ ;
        
        char* howmny = "A" ; // Compute All Ritz Vectors
        double sigma  = shift_ ;
        int ierr ;
        
        std::cerr << "calling d[s|n]eupd()..." << std::endl ;
        if(symmetric_) {
            LOCAL_OGF_FORTRAN(dseupd)( 
                &rvec, howmny, select.data(), d_.data(), v_.data(), 
                &ldv, &sigma, bmat, &n_, which, &nev, 
                &tol, resid.data(), &ncv, v_.data(), &ldv, 
                iparam_.data(), ipntr_.data(), workd_.data(), workl_.data(), &lworkl, &ierr 
            ) ;
        } else {
            double sigmar = shift_ ;
            double sigmai = 0.0 ;
            LOCAL_OGF_FORTRAN(dneupd)( 
                &rvec, howmny, select.data(), d_.data(), d_.data() + ncv,
                v_.data(), &ldv, 
                &sigmar, &sigmai, workev.data(), bmat, &n_, which, &nev, &tol, 
                resid.data(), &ncv, v_.data(), &ldv, iparam_.data(), ipntr_.data(), 
                workd_.data(), workl_.data(), &lworkl, &ierr 
            ) ;

            if(ierr != 0) {
                std::cerr << "Error with d[s|n]eupd(): " << ierr << std::endl ;
                return false ;
            } 
            std::cerr << "d[s|n]eupd(): success" << std::endl ;
            std::cerr << "nconv = " << iparam(3) << std::endl ;
        }
        
        // Apply inverse spectral transform to eigenvalues
        if(bruno_) {
            for(unsigned int i=0; i<static_cast<unsigned int>(nev); i++) {
                d_[i] = (::fabs(d_[i]) < 1e-30) ? 1e30 : 1.0 / d_[i] ;
                d_[i] += shift_ ;
            }            
        }

        permut_.allocate(nev) ;
        for(unsigned int i=0; i<static_cast<unsigned int>(nev); i++) {
            permut_(i) = i ;
        }
        std::sort(permut_.data(), permut_.data() + nev, EigenCompare(d_)) ;
        
        if(symmetric_) {
            for(int i=0; i<nev; i++) {
                int j = permut_(i) ;
                std::cerr << i << " " << d_(j) << std::endl ;
            }
        } else {
            for(int i=0; i<nev; i++) {
                int j = permut_(i) ;
                std::cerr << i << " " << "(" 
                          << d_(j) << "," 
                          << d_(j + ncv) << ")" << std::endl ;
            }
        }

        {
            std::cerr << "Caching last eigenvector" << std::endl ;
            double* eigen = v_.data() ; // + permut_[nb_eigens_ - 1] * n_ ;
            if(resid_cache_.size() != n_) {
                resid_cache_.allocate(n_) ;
            }
            for(int i=0; i<nev; i++) {
                resid_cache_[i] = eigen[i] ;
            }
        }
        
        return true ;
    }

    double* EigenSolver_ARPACK::get_eigen_vector(int index) {
        ogf_assert(index >= 0 && (unsigned int)index < permut_.size()) ;
        index = permut_(index) ;
        ogf_assert(index >= 0 && (unsigned int)index < d_.size()) ;
        int start = index * n_ ;
        return v_.data() + start ;
    }

    void EigenSolver_ARPACK::get_eigen_vector(int index, Vector& v) {
        if(!(index >= 0 && (unsigned int)index < permut_.size())) Logger::out("arpack") << "index=" << index << " permut_.size()=" << permut_.size() << std::endl ;
        ogf_assert(index >= 0 && (unsigned int)index < permut_.size()) ;
        index = permut_(index) ;
        ogf_assert(index >= 0 && (unsigned int)index < d_.size()) ;
        int start = index * n_ ;
        for(int i=0; i<n_; i++) {
            v(i) = v_(start + i) ;
        }
    }

    double EigenSolver_ARPACK::get_eigen_value(int index) {
        ogf_assert(index >= 0 && (unsigned int)index < d_.size()) ;
        return d_(permut_(index)) ;
    }

//_______________________________________________________________________
    
    bool EigenSolver_ARPACK::sym_regular(int ido) {
        if(ido == 1) {  A_mul(1,2) ; return false ;  }
        return true ;
    }
    
    bool EigenSolver_ARPACK::sym_shift_invert(int ido) {
        if(ido == -1 || ido == 1) { OP_mul(1,2) ; return false ; }
        return true ;
    }

    bool EigenSolver_ARPACK::sym_with_B_invert(int ido) {
        if(ido == 1) {
            A_mul(1,0) ;
            OP_mul(0,2) ;
            return false ;
        }
        return true ;
    }

    bool EigenSolver_ARPACK::sym_invert_gen(int ido) {

        if(ido == 1) {
            B_mul(1,0) ;
            OP_mul(0,2) ;
            return false ;
        }
        return true ;

        /*
        if(ido == -1 || ido == 1) {
            A_mul(1,2) ;
            copy_temp(2,1) ;
            OP_mul(1,2) ;
            return false ;
        } else if(ido == 2) {
            B_mul(1,2) ;
            return false ;
        }
        return true ;
        */
    }
    
    bool EigenSolver_ARPACK::sym_shift_invert_gen(int ido) {
        switch(ido) {
        case -1:
            A_mul(1,0) ;
            OP_mul(0,2) ;
            break ;
        case 1:
            OP_mul(3,2) ;
            break ;
        case 2:
            B_mul(1,2) ;
            break ;
        default :
            return true ;
            break ;
        }
        return false ;
    }

    bool EigenSolver_ARPACK::nsym_regular(int ido) {
        if(ido == 1) {
            A_mul(1,2) ;
            return false ;
        }
        return true ;
    }


    bool EigenSolver_ARPACK::nsym_shift_invert(int ido) {
        if(ido == 1 || ido == -1) { OP_mul(1,2) ; return false ;  }
        return true ;
    }

    bool EigenSolver_ARPACK::nsym_regular_gen(int ido) {
        if(ido == 1) {
            A_mul(1,0) ;
            OP_mul(0,2) ;
            return false ;
        }
        return true ;
    }

    bool EigenSolver_ARPACK::nsym_invert_gen(int ido) {
        if(ido == 1) {
            B_mul(1,0) ;
            OP_mul(0,2) ;
            return false ;
        }
        return true ;
    }

    bool EigenSolver_ARPACK::nsym_with_B_invert(int ido) {
        if(ido == 1) {
            A_mul(1,0) ;
            OP_mul(0,2) ;
            return false ;
        }
        return true ;
    }

//_______________________________________________________________________

    bool EigenSolver_ARPACK::bruno_iter(int ido) {
        if(ido == 1) {
            if(B_ == nil) {
                OP_mul(1,2) ; 
            } else {
                B_mul(1,0) ;
                OP_mul(0,2) ;
            }
            return false ;
        }
        return true ;
    }

//_______________________________________________________________________

    bool EigenSolver_ARPACK::setup_OP() {

        if(bruno_) {
            std::cerr << "Bruno Invert mode" << std::endl ;
            bool ok = true ;
            if(shift_ == 0.0 && B_ == nil) {
                A_cache_ = nil ; B_cache_ = nil ;
                std::cerr << "Inverting A matrix with " << direct_solver_ << "..." << std::endl ;
                ok = MathLibrary::instance()->invert_matrix(*A_, OP_, direct_solver_) ;
                std::cerr << "Matrix inverted" << std::endl ;
                if(!ok) {
                    std::cerr << direct_solver_ << " failed" << std::endl ;
                    return false ;
                }
                return true ;
            } 

            // Cache management
            if(A_ != A_cache_ || B_ != B_cache_) {
                uses_perm_cache_ = false ;
                op_permut_cache_.clear() ;
                resid_cache_.clear() ;
                A_cache_ = A_ ; B_cache_ = B_ ;
                std::cerr << "   computing new permutation" << std::endl ;
            } else {
                uses_perm_cache_ = true ;
                std::cerr << "   using cached permutation" << std::endl ;
            }


            // Apply spectral shift to matrix

            SparseMatrix AS(A_->n(), A_->storage(), A_->has_symmetric_storage()) ;
            add_matrix(AS, *A_) ;
                
	   if(shift_ != 0.0) {
                if(B_ == nil) {
                    for(int i=0; i<A_->n(); i++) {
                        AS.add(i,i, -shift_) ;
                    }
                } else {
                    add_matrix(AS, *B_, -shift_) ;
                }
            }

            // Apply spectral inversion

            std::cerr << "Inverting A - sigma B matrix with " << direct_solver_ << "..." << std::endl ;
            if(B_ == nil) { std::cerr << "   (with B = Id)" << std::endl ;  }
            if(shift_ == 0.0) { std::cerr << "   (with sigma = 0)" << std::endl ;  }
	   
	    // Note: we got a problem with the management of permutations, so we
	    // do not use that for the moment (i.e. permut_cache_ is not passed to
   	    // invert_matrix())
            ok = MathLibrary::instance()->invert_matrix(AS, OP_, direct_solver_) ;
            std::cerr << "Matrix inverted" << std::endl ;

            if(!ok) {
                std::cerr << direct_solver_ << " failed" << std::endl ;
                return false ;
            }
            return true ;
        }


        if(shift_invert_) {

            SparseMatrix AS(A_->m(), A_->n(), A_->storage()) ;
            add_matrix(AS, *A_) ;
            
            if(B_ == nil) {
                for(int i=0; i<A_->n(); i++) {
                    AS.add(i,i, -shift_) ;
                }
            } else {
                add_matrix(AS, *B_, -shift_) ;
            }

            std::cerr << "Shift-invert mode" << std::endl ;
            std::cerr << "Inverting A - sigma B matrix with " << direct_solver_ << "..." << std::endl ;

            if(A_ != A_cache_ || B_ != B_cache_) {
                uses_perm_cache_ = false ;
                op_permut_cache_.clear() ;
                resid_cache_.clear() ;
                A_cache_ = A_ ; B_cache_ = B_ ;
                std::cerr << "   computing new permutation" << std::endl ;
            } else {
                uses_perm_cache_ = true ;
                std::cerr << "   using cached permutation" << std::endl ;
            }

            bool ok = MathLibrary::instance()->invert_matrix(AS, OP_, direct_solver_) ;
            std::cerr << "Matrix inverted" << std::endl ;
            if(!ok) {
                std::cerr << direct_solver_ << " failed" << std::endl ;
                return false ;
            }
            return true ;
        }


        if(B_ != nil) {
            std::cerr << "Generalized eigen problem" << std::endl ;
            std::cerr << "Inverting B matrix with " << direct_solver_ << "..." << std::endl ;
            bool ok = MathLibrary::instance()->invert_matrix(*B_, OP_, direct_solver_) ;
            std::cerr << "Matrix inverted" << std::endl ;
            if(!ok) {
                std::cerr << direct_solver_ << " failed" << std::endl ;
                return false ;
            }
            return true ;
        } 
        
        return true ;
    }

}

