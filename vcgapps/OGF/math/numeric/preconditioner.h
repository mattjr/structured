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
 

#ifndef __OGF_MATH_NUMERIC_PRECONDITIONER__
#define __OGF_MATH_NUMERIC_PRECONDITIONER__

#include <OGF/math/common/common.h>
#include <OGF/basic/types/types.h>

namespace OGF {

//_________________________________________________________

    class SparseMatrix ;

    /**
     * Base class for some preconditioners.
     */

    class MATH_API Preconditioner {
    public:

        /**
         * The matrix A should be square.
         */
        Preconditioner(
            const SparseMatrix& A, double omega = 1.0
        ) ;
        
        const SparseMatrix& A() const { return A_ ; }
        double omega() const { return omega_ ; }

        /**
         * To use this function, the matrix A should be symmetric.
         */
        void mult_upper_inverse(const double* x, double* y) const ;

        /**
         * To use this function, the matrix A should be symmetric and
         * should have been constructed with the flag store_transpose.
         */
        void mult_lower_inverse(const double* x, double* y) const ;

        void mult_diagonal(double* xy) const ;
        void mult_diagonal_inverse(double* xy) const ;
        
    private:
        const SparseMatrix& A_ ;
        double omega_ ;
    } ;
    
    //________________________________________________________

    class MATH_API Jacobi_Preconditioner : public Preconditioner {
    public:
        Jacobi_Preconditioner(
            const SparseMatrix& A, double omega = 1.0
        ) ;
    } ;

    void mult(const Jacobi_Preconditioner& A, const double* x, double* y) ;    
    
    //________________________________________________________

    /**
     * The SSOR preconditioner, sharing storage with the matrix.
     */
    class MATH_API SSOR_Preconditioner : public Preconditioner {
    public:

        /**
         * The matrix A should have been constructed with
         * the flag set_transpose.
         */
        SSOR_Preconditioner(
            const SparseMatrix& A, double omega = 1.0
        ) ;
    } ;

    void mult(const SSOR_Preconditioner& A, const double* x, double* y) ;

//_________________________________________________________
    
    
    template <class MATRIX, class PC_MATRIX> class PreconditionedMatrix {
    public:
        PreconditionedMatrix(
            const MATRIX& A, const PC_MATRIX& C
        ) : A_(A), C_(C) {
        }
        
        const MATRIX& matrix() const { return A_ ; }
        const PC_MATRIX& preconditioner() const { return C_ ; }
        
    private:
        const MATRIX& A_ ;
        const PC_MATRIX& C_ ;
    } ;
    
    template <class MATRIX, class PC_MATRIX> void mult(
        const PreconditionedMatrix<MATRIX, PC_MATRIX>& M,
        const double* x, double* y
    ) {
        static double* y1 = nil ;
        static int N = 0 ;
        if(N != M.matrix().n()) {
            N = M.matrix().n() ;
            delete[] y1 ;
            y1 = new double[N] ;
        }
        mult(M.matrix(), x, y1) ;
        mult(M.preconditioner(), y1, y) ;
    }
    
//_________________________________________________________________
    
}    

#endif

