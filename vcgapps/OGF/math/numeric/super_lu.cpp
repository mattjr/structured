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

#include <OGF/math/numeric/super_lu.h>
#include <OGF/math/linear_algebra/sparse_matrix.h>
#include <OGF/basic/debug/assert.h>

#include <numeric_stuff/SUPERLU/dsp_defs.h>
#include <numeric_stuff/SUPERLU/util.h>

//   TODO: SuperLU seems to convert everything in compressed column storage,
// update solver so that constructed matrices are already in compressed column
// storage.

namespace OGF {

    // Note: SuperLU is difficult to call, but it is worth it.
    // Here is a driver inspired by A. Sheffer's "cow flattener".
    bool solve_super_lu(
        SparseMatrix& M, const double* b, double* x, 
        Array1d<int>& perm, bool clear_M, bool symmetric
    ) {
        ogf_assert(!M.has_symmetric_storage()) ;
        ogf_assert(M.rows_are_stored()) ;
        ogf_assert(M.m() == M.n()) ;
        int n = M.n() ;

        int nnz = 0 ;
        { for(int i=0; i<n; i++) {
            nnz += M.row(i).nb_coeffs() ;
        }}

        // Allocate storage for sparse matrix and permutation
        int*     xa     = new int[n+1] ;
        double*  rhs    = new double[n] ;
        double*  a      = new double[nnz] ;
        int*     asub   = new int[nnz] ;
        int*     perm_r = new int[n] ;

        // Step 1: convert matrix M into SuperLU compressed column 
        //   representation.
        // -------------------------------------------------------
        {
            int count = 0 ;
            for(int i=0; i<n; i++) {
                const SparseMatrix::Row& r = M.row(i) ;
                xa[i] = count ;
                for(int jj=0; jj<r.nb_coeffs(); jj++) {
                    a[count]    = r.coeff(jj).a ;
                    asub[count] = r.coeff(jj).index ; 
                    count++ ;
                }
            }
            xa[n] = nnz ;
        }

        // Save memory for SuperLU
        if(clear_M) {
            M.clear() ;
        }

        // Rem: symmetric storage does not seem to work with
        //  SuperLU ... (->deactivated in main LinearSolver driver)
        SuperMatrix A;
        dCreate_CompCol_Matrix(
            &A, n, n, nnz, a, asub, xa, 
            SLU_NR,                  // Row_wise, no supernode
            SLU_D,                   // doubles
            SLU_GE                   // general storage
        );

        // Step 2: create vector
        // ---------------------
        SuperMatrix B;
        dCreate_Dense_Matrix(
            &B, n, 1, const_cast<double*>(b), n, 
            SLU_DN, // Fortran-type column-wise storage 
            SLU_D,  // doubles
            SLU_GE  // general
        );
            

        // Step 3: get permutation matrix 
        // ------------------------------
        // com_perm: NATURAL       -> no re-ordering
        //           MMD_ATA       -> re-ordering for A^t.A
        //           MMD_AT_PLUS_A -> re-ordering for A^t+A
        //           COLAMD        -> approximate minimum degree ordering
        if(perm.size() == 0) {
            perm.allocate(n) ;
            if(symmetric) {
                get_perm_c(MMD_AT_PLUS_A, &A, perm.data()) ;
            } else {
                get_perm_c(COLAMD, &A, perm.data()) ;
            }
        }

        // Step 4: call SuperLU main routine
        // ---------------------------------
        superlu_options_t options ;
        SuperLUStat_t     stat ;
        SuperMatrix L;
        SuperMatrix U;
        int info;

        set_default_options(&options) ;
        options.ColPerm = MY_PERMC ;
        if(symmetric) {
            options.SymmetricMode = YES ;
        }
        StatInit(&stat) ;

        dgssv(&options, &A, perm.data(), perm_r, &L, &U, &B, &stat, &info);

        // Step 5: get the solution
        // ------------------------
        // Fortran-type column-wise storage
        DNformat *vals = (DNformat*)B.Store;
        double *rvals = (double*)(vals->nzval);
        if(info == 0) {
            for(int i = 0; i <  n; i++){
                x[i] = rvals[i];
            }
        } else {
            Logger::warn("Solver") << "SuperLU failed with: " << info <<  std::endl ;
            Logger::warn("Solver") << "size= " << n << " non zero= " << nnz <<  std::endl ;
        }

        //  For these two ones, only the "store" structure
        // needs to be deallocated (the arrays have been allocated
        // by us).
        Destroy_SuperMatrix_Store(&A) ;
        Destroy_SuperMatrix_Store(&B) ;

        //   These ones need to be fully deallocated (they have been
        // allocated by SuperLU).
        Destroy_SuperNode_Matrix(&L);
        Destroy_CompCol_Matrix(&U);

        // There are some dynamically allocated arrays in stat
        StatFree(&stat) ;

        delete[] xa     ;
        delete[] rhs    ;
        delete[] a      ;
        delete[] asub   ;
        delete[] perm_r ;

        return (info == 0) ;
    }

    bool solve_super_lu(
        const SparseMatrix& M, const double* b, double* x, bool do_perm, bool symmetric
    ) {
        Array1d<int> perm ;
        if(!do_perm) {
            perm.allocate(M.n()) ;
            for(int i=0; i<M.n(); i++) {
                perm(i) = i ;
            }
        }
        return solve_super_lu(const_cast<SparseMatrix&>(M),b,x,perm, false, symmetric) ;
    }

//____________________________________________________________________________________


    class SuperLUMatrix : public AbstractMatrixImpl {
    public:
        SuperLUMatrix() : n(0), perm_r(nil), perm_c(nil) {
        }
        ~SuperLUMatrix() {
            if(n != 0) {
                Destroy_SuperNode_Matrix(&L);
                Destroy_CompCol_Matrix(&U);
                delete[] perm_r ;
                delete[] perm_c ;
            }
        }

        virtual int* permutation() const {
            return perm_c ;
        }

        virtual void mult(double* y, const double* x) {

            // Create vector
            // ------------------
            SuperMatrix B ;
            dCreate_Dense_Matrix(
                &B, n, 1, y, n, 
                SLU_DN, // Fortran-type column-wise storage 
                SLU_D,  // doubles
                SLU_GE  // general
            );
            

            // copy rhs
            // -------------
            for(int i = 0; i < n; i++){
                y[i] = x[i];
            }


            // Call SuperLU triangular solve
            // -----------------------------
            SuperLUStat_t stat ;
            int info ;
            trans_t trans = NOTRANS ;
            StatInit(&stat) ;
            dgstrs(trans, &L, &U, perm_c, perm_r, &B, &stat, &info);
            

            //  Only the "store" structure needs to be 
            // deallocated (the array has been allocated
            // by client code).
            // ----------------------
            Destroy_SuperMatrix_Store(&B) ;
        }

        void allocate(int n_in) {
            ogf_assert(n == 0) ;
            n = n_in ;
            perm_r = new int[n] ;
            perm_c = new int[n] ;
        }

        int n ;
        SuperMatrix L ;
        SuperMatrix U ;
        int* perm_r ;
        int* perm_c ;
    } ;

    // copy-pasted from dgssv, removed triangular solve
    void superlu_factorize(
        superlu_options_t *options, SuperMatrix *A, int *perm_c, int *perm_r,
        SuperMatrix *L, SuperMatrix *U, SuperLUStat_t *stat, int *info
    ) {
        SuperMatrix *AA = nil ;/* A in SLU_NC format used by the factorization routine.*/
        SuperMatrix AC; /* Matrix postmultiplied by Pc */
        int      lwork = 0, *etree, i;
    
        /* Set default values for some parameters */
        double   drop_tol = 0.;
        int      panel_size;     /* panel size */
        int      relax;          /* no of columns in a relaxed snodes */
        int      permc_spec;
        trans_t  trans = NOTRANS;
        double   *utime;
        double   t;	/* Temporary time */

        /* Test the input parameters ... */
        *info = 0;
        if ( options->Fact != DOFACT ) *info = -1;
        else if ( A->nrow != A->ncol || A->nrow < 0 ||
            (A->Stype != SLU_NC && A->Stype != SLU_NR) ||
            A->Dtype != SLU_D || A->Mtype != SLU_GE )
            *info = -2;
        if ( *info != 0 ) {
            i = -(*info);
            xerbla_("dgssv", &i);
            return;
        }

        utime = stat->utime;
            
        /* Convert A to SLU_NC format when necessary. */
        if ( A->Stype == SLU_NR ) {
            NRformat *Astore = (NRformat*)(A->Store) ;
            AA = (SuperMatrix *) SUPERLU_MALLOC( sizeof(SuperMatrix) );
            dCreate_CompCol_Matrix(AA, A->ncol, A->nrow, Astore->nnz, 
                (double*)(Astore->nzval), Astore->colind, Astore->rowptr,
                SLU_NC, A->Dtype, A->Mtype);
            trans = TRANS;
        } else {
            if ( A->Stype == SLU_NC ) AA = A;
        }

        t = SuperLU_timer_();
        /*
         * Get column permutation vector perm_c[], according to permc_spec:
         *   permc_spec = NATURAL:  natural ordering 
         *   permc_spec = MMD_AT_PLUS_A: minimum degree on structure of A'+A
         *   permc_spec = MMD_ATA:  minimum degree on structure of A'*A
         *   permc_spec = COLAMD:   approximate minimum degree column ordering
         *   permc_spec = MY_PERMC: the ordering already supplied in perm_c[]
         */
        permc_spec = options->ColPerm;
        if ( permc_spec != MY_PERMC && options->Fact == DOFACT )
            get_perm_c(permc_spec, AA, perm_c);
        utime[COLPERM] = SuperLU_timer_() - t;
            
        etree = intMalloc(A->ncol);

        t = SuperLU_timer_();
        sp_preorder(options, AA, perm_c, etree, &AC);
        utime[ETREE] = SuperLU_timer_() - t;

        panel_size = sp_ienv(1);
        relax = sp_ienv(2);
            
        /*printf("Factor PA = LU ... relax %d\tw %d\tmaxsuper %d\trowblk %d\n", 
          relax, panel_size, sp_ienv(3), sp_ienv(4));*/
        t = SuperLU_timer_(); 
        /* Compute the LU factorization of A. */
        dgstrf(options, &AC, drop_tol, relax, panel_size,
            etree, NULL, lwork, perm_c, perm_r, L, U, stat, info);
        utime[FACT] = SuperLU_timer_() - t;

        SUPERLU_FREE (etree);
        Destroy_CompCol_Permuted(&AC);
        if ( A->Stype == SLU_NR ) {
            Destroy_SuperMatrix_Store(AA);
            SUPERLU_FREE(AA);
        }
    }

    bool invert_super_lu(const SparseMatrix& M, AbstractMatrix& Minv, int* perm_in) {

        ogf_assert(!M.has_symmetric_storage()) ;
        ogf_assert(M.rows_are_stored()) ;
        ogf_assert(M.m() == M.n()) ;
        int n = M.n() ;

        SuperLUMatrix* impl = new SuperLUMatrix ;
        Minv.set_impl(n, impl) ;
        impl->allocate(n) ;
            
        int nnz = 0 ;
        for(int i=0; i<n; i++) {
             nnz += M.row(i).nb_coeffs() ;
         }

        // Allocate storage for sparse matrix and permutation
        int*     xa     = new int[n+1] ;
        double*  a      = new double[nnz] ;
        int*     asub   = new int[nnz] ;


        // Step 1: convert matrix M into SuperLU compressed column 
        //   representation.
        // -------------------------------------------------------
        {
            int count = 0 ;
            for(int i=0; i<n; i++) {
                const SparseMatrix::Row& r = M.row(i) ;
                xa[i] = count ;
                for(int jj=0; jj<r.nb_coeffs(); jj++) {
                    a[count]    = r.coeff(jj).a ;
                    asub[count] = r.coeff(jj).index ; 
                    count++ ;
                }
            }
            xa[n] = nnz ;
        }

        /*
        // Save memory for SuperLU
        bool clear_M = true ;
        if(clear_M) {
            const_cast<SparseMatrix&>(M).clear() ;
        }
        */

        // Rem: symmetric storage does not seem to work with
        //  SuperLU ... (->deactivated in main LinearSolver driver)
        SuperMatrix A;
        dCreate_CompCol_Matrix(
            &A, n, n, nnz, a, asub, xa, 
            SLU_NC,                  // Column_wise, no supernode
            SLU_D,                   // doubles
            SLU_GE                   // general storage
        );

        bool symmetric = M.is_symmetric() ; // Uses the symmetric tag.
        if(perm_in != nil) {
            for(int i=0; i<n; i++) {
                impl->perm_c[i] = perm_in[i] ;
            }
        } else {
            if(symmetric) {
                get_perm_c(MMD_AT_PLUS_A, &A, impl->perm_c) ;
            } else {
                get_perm_c(COLAMD, &A, impl->perm_c) ;
            }
        }

        bool result = true ;

        // Construct invert
        superlu_options_t options ;
        SuperLUStat_t     stat ;
        int info ;

        set_default_options(&options) ;
        options.ColPerm = MY_PERMC ;
        if(symmetric) {
            options.SymmetricMode = YES ;
        }
        StatInit(&stat) ;

        superlu_factorize(
            &options, &A, impl->perm_c, impl->perm_r,
            &impl->L, &impl->U, &stat, &info 
        ) ;

        // ------------------- cleanup -------------------------------


        //  Only the "store" data structure needs to be deallocated 
        // (the arrays have been allocated by us).
        Destroy_SuperMatrix_Store(&A) ;

        delete[] xa     ;
        delete[] a      ;
        delete[] asub   ;

        return result ;
    }

    bool invert_super_lu(const SparseMatrix& M, AbstractMatrix& Minv, Array1d<int>& perm) {
        bool result = true ;
        int n = M.n() ;
        if(perm.size() == 0) {
            perm.allocate(n) ;
            result = invert_super_lu(M, Minv) ;
            if(result) {
                int* perm_c = dynamic_cast<SuperLUMatrix*>(Minv.impl())->perm_c ;
                for(int i=0; i<n; i++) {
                    perm[i] = perm_c[i] ;
                }
            }
        } else {
            result = invert_super_lu(M, Minv, perm.data()) ;
        }
        return result ;
    }

}

