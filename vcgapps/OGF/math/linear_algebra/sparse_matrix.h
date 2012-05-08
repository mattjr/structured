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
 

#ifndef ___OGF_MATH_LINEAR_ALGEBRA_SPARSE_MATRIX__
#define ___OGF_MATH_LINEAR_ALGEBRA_SPARSE_MATRIX__

#include <OGF/math/common/common.h>
#include <OGF/math/linear_algebra/vector.h>

#include <iostream>

namespace OGF {

    /**
     * An allocator to create dynamically
     * resizable structures. Note: if needed,
     * a more efficient method may be used.
     */
    template <class T> class Allocator {
    public:
        static inline T* allocate(int number) {
            return new T[number] ;
        }
        
        static inline void deallocate(T*& addr) {
            delete[] addr ;
            addr = nil ; // makes it possible to track
                         // access to deallocated memory.
        }
        
        static inline void reallocate(
            T*& addr, int old_number, int new_number
        ) {
            T* new_addr = new T[new_number] ;
            for(int i=0; i<old_number; i++) {
                new_addr[i] = addr[i] ;
            }
            delete[] addr ;
            addr = new_addr ;
        }
    } ;
    
//_________________________________________________________

    /**
     * A coefficient of a SparseMatrix.
     */
    class MATH_API Coeff {
    public:
        double a ;
        int index ;
    } ;

    //__________________________________________________

    /**
     * A row or a column of a SparseMatrix. SparseRowColumn is
     * compressed, and stored in the form of a list of
     * (value,index) couples.
     */
    class MATH_API SparseRowColumn {
    public:
        SparseRowColumn() {
            coeff_ = Allocator<Coeff>::allocate(2) ;
            nb_coeffs_ = 0 ;
            capacity_ = 2 ;
        }

        ~SparseRowColumn() {
            Allocator<Coeff>::deallocate(coeff_) ;
        }

        int nb_coeffs() const { return nb_coeffs_ ; }

        Coeff& coeff(int ii) { 
            ogf_debug_assert(ii >= 0 && ii < nb_coeffs_) ;
            return coeff_[ii] ;
        }
            
        const Coeff& coeff(int ii) const {
            ogf_debug_assert(ii >= 0 && ii < nb_coeffs_) ;
            return coeff_[ii] ;
        }        
            
        /** a_{index} <- a_{index} + val */
        void add(int index, double val) ;

        /** a_{index} <- val */
        void set(int index, double val) ;
            
        /** sorts the coefficients by increasing index */
        void sort() ;


        /** 
         * removes all the coefficients and frees the allocated
         * space.
         */
        void clear() { 
            Allocator<Coeff>::deallocate(coeff_) ;
            coeff_ = Allocator<Coeff>::allocate(2) ;
            nb_coeffs_ = 0 ;
            capacity_ = 2 ;
        }

        /** 
         * removes all the coefficients, but keeps the
         * allocated space, that will be used by subsequent
         * calls to add().
         */
        void zero() { nb_coeffs_ = 0 ; }

    protected:
        void grow() ;
            
    private:
        Coeff* coeff_ ;
        int nb_coeffs_ ;
        int capacity_ ;
    } ;
        
//________________________________________________________________

    class MATH_API SparseMatrix {
    public:

        typedef SparseRowColumn Row ;
        typedef SparseRowColumn Column ;

        enum Storage { NONE, ROWS, COLUMNS, ROWS_AND_COLUMNS} ;

        //__________ constructors / destructor _____

        /**
         * Constructs a m*n sparse matrix.
         * @param Storage can be one of ROWS, COLUMNS, ROWS_AND_COLUMNS
         */
        SparseMatrix(int m, int n, Storage storage = ROWS) ;

        /**
         * Constructs a square n*n sparse matrix.
         * @param Storage can be one of ROWS, COLUMNS, ROWS_AND_COLUMNS
         * @param symmetric_storage if set, only entries a_ij such
         *   that j <= i are stored.
         */
        SparseMatrix(int n, Storage storage, bool symmetric_storage) ;

        SparseMatrix() ;

        ~SparseMatrix() ;

        //___________ access ________________________

        int m() const {
            return m_ ;
        }

        int n() const {
            return n_ ;
        }

        int diag_size() const {
            return diag_size_ ;
        }

        /** number of non-zero coefficients */
        int nnz() const ;

        bool rows_are_stored() const {
            return rows_are_stored_ ;
        }

        bool columns_are_stored() const {
            return columns_are_stored_ ;
        }

        Storage storage() const {
            return storage_ ;
        }

        bool has_symmetric_storage() const {
            return symmetric_storage_ ;
        }

        bool is_square() const {
            return (m_ == n_) ;
        }

        bool is_symmetric() const {
            return (symmetric_storage_ || symmetric_tag_) ;
        }

        /**
         * For symmetric matrices that are not stored in symmetric mode,
         * one may want to give a hint that the matrix is symmetric.
         */
        void set_symmetric_tag(bool x) {
            symmetric_tag_ = x ;
        }

        /**
         * storage should be one of ROWS, ROWS_AND_COLUMNS
         * @param i index of the row, in the range [0, m-1]
         */
        Row& row(int i) {
            ogf_debug_assert(i >= 0 && i < m_) ;
            ogf_debug_assert(rows_are_stored_) ;
            return row_[i] ;
        }

        /**
         * storage should be one of ROWS, ROWS_AND_COLUMNS
         * @param i index of the row, in the range [0, m-1]
         */
        const Row& row(int i) const {
            ogf_debug_assert(i >= 0 && i < m_) ;
            ogf_debug_assert(rows_are_stored_) ;
            return row_[i] ;
        }

        /**
         * storage should be one of COLUMN, ROWS_AND_COLUMNS
         * @param i index of the column, in the range [0, n-1]
         */
        Column& column(int j) {
            ogf_debug_assert(j >= 0 && j < n_) ;
            ogf_debug_assert(columns_are_stored_) ;
            return column_[j] ;
        }

        /**
         * storage should be one of COLUMNS, ROWS_AND_COLUMNS
         * @param i index of the column, in the range [0, n-1]
         */
        const Column& column(int j) const {
            ogf_debug_assert(j >= 0 && j < n_) ;
            ogf_debug_assert(columns_are_stored_) ;
            return column_[j] ;
        }
        
        /**
         * returns aii.
         */
        double diag(int i) const {
            ogf_debug_assert(i >= 0 && i < diag_size_) ;
            return diag_[i] ;
        }

        /**
         * aij <- aij + val
         */
        void add(int i, int j, double val) {
            ogf_debug_assert(i >= 0 && i < m_) ;
            ogf_debug_assert(j >= 0 && j < n_) ;
            if(symmetric_storage_ && j > i) {
                return ;
            }
            if(i == j) {
                diag_[i] += val ;
            } 
            if(rows_are_stored_) {
                row(i).add(j, val) ;
            }
            if(columns_are_stored_) {
                column(j).add(i, val) ;
            }
        }

        /**
         * aij <- val
         */
        void set(int i, int j, double val) {
            ogf_debug_assert(i >= 0 && i < m_) ;
            ogf_debug_assert(j >= 0 && j < n_) ;
            if(symmetric_storage_ && j > i) {
                return ;
            }
            if(i == j) {
                diag_[i] = val ;
            } 
            if(rows_are_stored_) {
                row(i).set(j, val) ;
            }
            if(columns_are_stored_) {
                column(j).set(i, val) ;
            }
        }
            

        /** sorts rows and columns by increasing coefficients */
        void sort() ;
            

        /**
         * removes all the coefficients and frees the allocated
         * space.
         */
        void clear() ;

        /**
         * removes all the coefficients, but keeps the allocated
         * storage, that will be used by subsequent calls to add().
         */
        void zero() ;

        void allocate(
            int m, int n, Storage storage, bool symmetric = false
        ) ;

        void deallocate() ;

    private:
        int m_ ;
        int n_ ;
        int diag_size_ ;

        SparseRowColumn* row_ ;
        SparseRowColumn* column_ ;
        double* diag_ ;

        Storage storage_ ;
        bool rows_are_stored_ ;
        bool columns_are_stored_ ;
        bool symmetric_storage_ ;
        bool symmetric_tag_ ;

        // SparseMatrix cannot be copied.
        SparseMatrix(const SparseMatrix& rhs) ;
        SparseMatrix& operator=(const SparseMatrix& rhs) ;
    } ;


    /**
     * Matrix * vector product: y = A.x
     */
    void MATH_API mult(const SparseMatrix& A, const double* x, double* y) ;    

    inline void mult(
        const SparseMatrix& A, const Vector& x, Vector& y
    ) {
        ogf_assert(x.size() == A.n()) ;
        ogf_assert(y.size() == A.m()) ;
        mult(A, x.data(), y.data()) ;
    }

    /**
     * Matrix * vector product: y = At.x
     */
    void MATH_API mult_transpose(
        const SparseMatrix& A, const double* x, double* y
    ) ;    

    inline void mult_transpose(
        const SparseMatrix& A, const Vector& x, Vector& y
    ) {
        ogf_assert(x.size() == A.m()) ;
        ogf_assert(y.size() == A.n()) ;
        mult_transpose(A, x.data(), y.data()) ;
    }

    MATH_API std::ostream& operator<<(std::ostream& out, const OGF::SparseMatrix& M) ;
    
}
#endif

