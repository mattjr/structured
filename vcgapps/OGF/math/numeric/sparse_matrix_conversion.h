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
 

#ifndef __OGF_MATH_SPARSE_MATRIX_CONVERSION__
#define __OGF_MATH_SPARSE_MATRIX_CONVERSION__

#include <OGF/math/common/common.h>
#include <OGF/math/linear_algebra/sparse_matrix.h>
#include <OGF/math/numeric/sparse_matrix_crs.h>
#include <typeinfo>

namespace OGF {

    inline void convert_matrix(
        const SparseMatrix& rhs, SparseMatrixPatternCRS& A, bool separate_diag = true,
        const Permutation& permutation = Permutation()
    ) ;

    template <class T> inline void convert_matrix(
        const SparseMatrix& rhs, SparseMatrixCRS<T>& A, bool separate_diag = true, 
        const Permutation& permutation = Permutation()
    ) ;

    template <class T, int BM, int BN> inline void convert_matrix(
        const SparseMatrix& rhs, SparseMatrixBCRS<T, BM, BN>& A,
        const Permutation& permutation = Permutation()
    ) ;

    template <class T, int BM, int BN> inline void convert_matrix(
        const SparseMatrixCRS<T>& rhs, SparseMatrixBCRS<T, BM, BN>& A, 
        const Permutation& permutation = Permutation()
    ) ;

    template <class T> inline void convert_matrix(
        const SparseMatrixCRS<T>& rhs, SparseMatrixCRS<T>& A, 
        const Permutation& permutation = Permutation()
    ) ;

}

//================================================================

namespace OGF {

    inline void convert_matrix(
        const SparseMatrix& rhs, SparseMatrixPatternCRS& A, bool separate_diag,
        const Permutation& permutation 
    ) {
        Permutation inv_permutation ;
        permutation.invert(inv_permutation) ;

        A.symmetric_storage = rhs.has_symmetric_storage() ;
        A.N = rhs.n() ;
        A.rowptr.allocate(rhs.m() + 1) ;
        unsigned int nnz = rhs.nnz() ;
        if(separate_diag) {
            nnz -= rhs.diag_size() ;
        }
        A.colind.allocate(nnz) ;
        
        ogf_assert(rhs.rows_are_stored()) ;
        unsigned int cur = 0 ;
        for(unsigned int i=0; i<(unsigned int)rhs.m(); i++) {
            A.rowptr[i] = cur ;
            const SparseMatrix::Row& R = rhs.row(inv_permutation(i)) ;
            for(unsigned int jj=0; jj<(unsigned int)R.nb_coeffs(); jj++) {
                if(!separate_diag || (R.coeff(jj).index != inv_permutation(i))) {
                    A.colind[cur] = permutation(R.coeff(jj).index) ;
                    cur++ ;
                }
            }
        }
        A.rowptr[rhs.m()] = nnz ;
    }

    template <class T> inline void convert_matrix(
        const SparseMatrix& rhs, SparseMatrixCRS<T>& A, bool separate_diag,
        const Permutation& permutation 
    ) {
        Permutation inv_permutation ;
        permutation.invert(inv_permutation) ;

        A.separate_diag = separate_diag ;
        A.symmetric_storage = rhs.has_symmetric_storage() ;
        A.N = rhs.n() ;
        A.rowptr.allocate(rhs.m() + 1) ;
        unsigned int nnz = rhs.nnz() ;
        if(separate_diag) {
            nnz -= rhs.diag_size() ;
        }
        A.colind.allocate(nnz) ;
        A.a.allocate(nnz, alignment_for_SSE2) ;
        A.diag.allocate(rhs.diag_size(), alignment_for_SSE2) ;
        
        ogf_assert(rhs.rows_are_stored()) ;
        unsigned int cur = 0 ;
        for(unsigned int i=0; i<(unsigned int)rhs.m(); i++) {
            A.rowptr[i] = cur ;
            const SparseMatrix::Row& R = rhs.row(inv_permutation(i)) ;
            for(unsigned int jj=0; jj<(unsigned int)R.nb_coeffs(); jj++) {
                if(!separate_diag || (R.coeff(jj).index != inv_permutation(i))) {
                    A.a[cur] = T(R.coeff(jj).a) ;
                    A.colind[cur] = permutation(R.coeff(jj).index) ;
                    cur++ ;
                }
            }
        }
        A.rowptr[rhs.m()] = nnz ;
        for(unsigned int i=0; i<(unsigned int)rhs.diag_size(); i++) {
            A.diag[i] = T(rhs.diag(inv_permutation(i))) ;
        }
    }

    template <class T, int BM, int BN> inline void convert_matrix(
        const SparseMatrix& rhs, SparseMatrixBCRS<T, BM, BN>& A,
        const Permutation& permutation 
    ) {
        ogf_assert(!rhs.has_symmetric_storage()) ;
        ogf_assert(rhs.rows_are_stored()) ;

        unsigned int BLOC_SIZE = SparseMatrixBCRS<T, BM, BN>::BLOC_SIZE ;

        Permutation inv_permutation ;
        permutation.invert(inv_permutation) ;

        // Compute number of bloc rows and bloc columns
        unsigned int M = rhs.m() / BM ;
        if((rhs.m() % BM) != 0) { M++ ; }

        unsigned int N = rhs.n() / BN ;
        if((rhs.n() % BN) != 0) { N++ ; }

        A.N_ = N ;

        // Step 1: determine blocs to use
        Array1d< std::set<unsigned int> > row_blocs(M) ;
        for(unsigned int i=0; i<(unsigned int)rhs.m(); i++) {
            unsigned int I = i / BM ;
            const SparseMatrix::Row& Ri = rhs.row(inv_permutation(i)) ;
            for(unsigned int jj=0 ; jj < (unsigned int)Ri.nb_coeffs(); jj++) {
                unsigned int j = permutation(Ri.coeff(jj).index) ;
                unsigned int J = j / BN ;
                row_blocs[I].insert(J) ;
            }
        }

        // Step 2: initialize rowptr 
        A.rowptr.allocate(M+1) ;
        A.rowptr[0] = 0 ;
        for(unsigned int I=0; I<M; I++) {
            A.rowptr[I+1] = A.rowptr[I] + row_blocs[I].size() ;
        }
        unsigned int NNZ = A.rowptr[M] ;

        // Step 3: initialize colind
        A.colind.allocate(NNZ) ;
        unsigned int cur = 0 ;
        for(unsigned int I=0; I<M; I++) {
            for(std::set<unsigned int>::iterator it = row_blocs[I].begin(); it != row_blocs[I].end(); it++) {
                A.colind[cur++] = (*it) ;
            }
        }           
        ogf_assert(cur == NNZ) ;
            
        // Step 4: initialize a
        A.a.allocate(NNZ * BLOC_SIZE, alignment_for_SSE2) ;
        A.a.set_all(0.0) ;
        for(unsigned int i=0; i<(unsigned int)rhs.m(); i++) {
            unsigned int I = i / BM ;
            unsigned int di = i % BM ;
            const SparseMatrix::Row& Ri = rhs.row(inv_permutation(i)) ;
            for(unsigned int jj=0; jj < (unsigned int)Ri.nb_coeffs(); jj++) {
                unsigned int j = permutation(Ri.coeff(jj).index) ;
                unsigned int J = j / BN ;
                unsigned int dj = j % BN ;
                bool found = false ;
                for(unsigned int K=A.rowptr[I]; K<A.rowptr[I+1]; K++) {
                    if(A.colind[K] == J) {
                        A.a[K * BLOC_SIZE + di * BN + dj] = Ri.coeff(jj).a ;
                        found = true ;
                        break ;
                    }
                }
                ogf_assert(found) ;
            }
        }
        // Step 5: initialize diag
        A.diag.allocate(rhs.diag_size()) ;
        for(unsigned int i=0; i<A.diag.size(); i++) {
            A.diag[i] = rhs.diag(inv_permutation(i)) ;
        }
    }

    template <class T, int BM, int BN> inline void convert_matrix(
        const SparseMatrixCRS<T>& rhs, SparseMatrixBCRS<T, BM, BN>& A, 
        const Permutation& permutation 
    ) {
        ogf_assert(!rhs.symmetric_storage) ;
        ogf_assert(!rhs.separate_diag) ;

        unsigned int BLOC_SIZE = SparseMatrixBCRS<T, BM, BN>::BLOC_SIZE ;

        Permutation inv_permutation ;
        permutation.invert(inv_permutation) ;

        // Compute number of bloc rows and bloc columns
        unsigned int M = rhs.m() / BM ;
        if((rhs.m() % BM) != 0) { M++ ; }

        unsigned int N = rhs.n() / BN ;
        if((rhs.n() % BN) != 0) { N++ ; }

        A.N_ = N ;

        // Step 1: determine blocs to use
        Array1d< std::set<unsigned int> > row_blocs(M) ;
        for(unsigned int i=0; i<rhs.m(); i++) {
            unsigned int I = i / BM ;
            for(unsigned int jj=rhs.rowptr[inv_permutation(i)] ; jj < rhs.rowptr[inv_permutation(i)+1]; jj++) {
                unsigned int j = permutation(rhs.colind[jj]) ;
                unsigned int J = j / BN ;
                row_blocs[I].insert(J) ;
            }
        }

        // Step 2: initialize rowptr 
        A.rowptr.allocate(M+1) ;
        A.rowptr[0] = 0 ;
        for(unsigned int I=0; I<M; I++) {
            A.rowptr[I+1] = A.rowptr[I] + row_blocs[I].size() ;
        }
        unsigned int NNZ = A.rowptr[M] ;

        // Step 3: initialize colind
        A.colind.allocate(NNZ) ;
        unsigned int cur = 0 ;
        for(unsigned int I=0; I<M; I++) {
            for(std::set<unsigned int>::iterator it = row_blocs[I].begin(); it != row_blocs[I].end(); it++) {
                A.colind[cur++] = (*it) ;
            }
        }           
        ogf_assert(cur == NNZ) ;
            
        // Step 4: initialize a
        A.a.allocate(NNZ * BLOC_SIZE, alignment_for_SSE2) ;
        A.a.set_all(0.0) ;
        for(unsigned int i=0; i<rhs.m(); i++) {
            unsigned int I = i / BM ;
            unsigned int di = i % BM ;
            for(unsigned int jj=rhs.rowptr[inv_permutation(i)] ; jj < rhs.rowptr[inv_permutation(i)+1]; jj++) {
                unsigned int j = permutation(rhs.colind[jj]) ;
                unsigned int J = j / BN ;
                unsigned int dj = j % BN ;
                bool found = false ;
                for(unsigned int K=A.rowptr[I]; K<A.rowptr[I+1]; K++) {
                    if(A.colind[K] == J) {
                        A.a[K * BLOC_SIZE + di * BN + dj] = rhs.a[jj] ;
                        found = true ;
                        break ;
                    }
                }
                ogf_assert(found) ;
            }
        }
        // Step 5: initialize diag
        A.diag.allocate(rhs.diag.size()) ;
        for(unsigned int i=0; i<A.diag.size(); i++) {
            A.diag[i] = rhs.diag[inv_permutation(i)] ;
        }
    }

    template <class T> inline void convert_matrix(
        const SparseMatrixCRS<T>& rhs, SparseMatrixCRS<T>& A, 
        const Permutation& permutation 
    ) {
        Permutation inv_permutation ;
        permutation.invert(inv_permutation) ;
        
        A.N = rhs.N ;
        A.rowptr.allocate(rhs.rowptr.size()) ;
        A.colind.allocate(rhs.colind.size()) ;
        A.a.allocate(rhs.a.size(), alignment_for_SSE2) ;
        A.diag.allocate(rhs.diag.size(), alignment_for_SSE2) ;
        A.symmetric_storage = rhs.symmetric_storage ;
        A.separate_diag = rhs.separate_diag ;
        
        unsigned int cur = 0 ;
        for(unsigned int i=0; i<rhs.n(); i++) {
            unsigned int old_i = inv_permutation(i) ;
            A.rowptr[i] = rhs.cur ;
            A.diag[i] = rhs.diag[old_i] ;
            for(unsigned int jj=rhs.rowptr[old_i]; jj < rhs.rowptr[old_i+1]; jj++) {
                A.a[cur] = rhs.a[jj] ;
                A.colind[cur] = permutation(rhs.colind[jj]) ;
                cur++ ;
            }
        }
        A.rowptr[A.n()] = rhs.a.size() ;
    }

    template <class T> void output_array(
        const Array1d<T>& t, const std::string& name, std::ostream& out
    ) {
        const char* t_name = typeid(T).name() ;
        if(!strcmp(t_name, typeid(int).name())) {
            out << "int" ;
        } else if(!strcmp(t_name, typeid(unsigned int).name())) {
            out << "unsigned int" ;
        } else if(!strcmp(t_name, typeid(float).name())) {
            out << "float" ;
        } else if(!strcmp(t_name, typeid(double).name())) {
            out << "double" ;
        }
        out << " " << name << "[" << t.size() << "] = {" << std::endl ;
        for(unsigned int i=0; i<t.size(); i++) {
            out << "   " << t[i] ;
            if(i < t.size() - 1) {
                out << "," ;
            }
            out << std::endl ;
        }
        out << "};" << std::endl ;
    }

    template <int BM, int BN, class T> void output_matrix(
        const SparseMatrixBCRS<T, BM, BN>& M,
        std::ostream& out
    ) {
        out << "// Bloc matrix" << std::endl ;
        out << "const unsigned int m = " << M.m() << "; // number of rows" << std::endl ;
        out << "const unsigned int n = " << M.n() << "; // number of columns" << std::endl ;
        out << "// Bloc size = BM x BN" << std::endl ;
        out << "const unsigned int BM = " << BM << ";" << std::endl ;
        out << "const unsigned int BN = " << BN << ";" << std::endl ;
        out << "// The matrix has M * N blocs" << std::endl ;
        out << "const unsigned int M = " << M.M() << "; // number of bloc rows" << std::endl ;
        out << "const unsigned int N = " << M.N() << "; // number of bloc columns" << std::endl ;
        out << "// Row pointers" << std::endl ;
        output_array(M.rowptr, "rowptr", out) ;
        out << "// Column indices" << std::endl ;
        output_array(M.colind, "colind", out) ;
        out << "// Coefficients" << std::endl ;
        output_array(M.a, "a", out) ;
    }


    template <class T, int BM, int BN> inline void compress_indices(
        SparseMatrixBCRS<T, BM, BN>& A
    ) {

        int max_rowptr = 0 ;
        int max_colind = 0 ;
        int nb_colind_gt = 0 ;

        A.rowptr_s.allocate(A.rowptr.size() - 1) ;
        for(unsigned int i=0; i<A.rowptr_s.size(); i++) {
            unsigned int val = A.rowptr[i+1] - A.rowptr[i] ;
            max_rowptr = ogf_max(max_rowptr, int(val)) ;
            if(val > 255) {
                std::cerr << "Could not compress rowptr" << std::endl ;
                std::cerr << "val = " << val << std::endl ;
                A.rowptr_s.allocate(0) ;
                return ;
            }
            A.rowptr_s[i] = (unsigned char)val ;
        }

        A.colind_s.allocate(A.colind.size()) ;
        for(unsigned int i=0; i<A.rowptr.size(); i++) {
            for(unsigned int jj=A.rowptr[i]; jj < A.rowptr[i+1]; jj++) {
                int val = int(A.colind[jj]) - i ;  
                max_colind = ogf_max(max_colind, val) ;
                if(val < -32767 || val > 32767) {
                    std::cerr << "Could not compress colind" << std::endl ;                
                    std::cerr << "val = " << val << std::endl ;
                    A.rowptr_s.allocate(0) ;
                    A.colind_s.allocate(0) ;
                    return ;
                }
                if(val < -127 || val > 127) {
                    nb_colind_gt++ ;
                }
                A.colind_s[jj] = (short)val ;
            }
        }
        std::cerr << "   max_rowptr = " << max_rowptr << std::endl ;
        std::cerr << "   max_colind = " << max_colind << std::endl ;
        std::cerr << "   large colind = " << double(nb_colind_gt) / double(A.colind.size()) * 100.0 << std::endl ;
    }
}

#endif
