
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
 


#ifndef __OGF_MATH_NUMERIC_EIGEN_SOLVER_ARPACK__
#define __OGF_MATH_NUMERIC_EIGEN_SOLVER_ARPACK__

#include <OGF/math/common/common.h>
#include <OGF/math/numeric/eigen_solver.h>
#include <OGF/math/linear_algebra/sparse_matrix.h>
#include <OGF/math/linear_algebra/abstract_matrix.h>
#include <OGF/math/linear_algebra/vector.h>

namespace OGF {

    class MATH_API EigenSolver_ARPACK : public EigenSolver {
    public:
        EigenSolver_ARPACK() ;
        virtual ~EigenSolver_ARPACK() ;
        
        virtual bool solve() ;
        virtual double* get_eigen_vector(int index) ;
        virtual void get_eigen_vector(int index, Vector& v) ;
        virtual double get_eigen_value(int index) ;
        
    protected:
        
        bool setup_OP() ;

        bool sym_regular(int ido) ;
        bool sym_shift_invert(int ido) ;
        bool sym_with_B_invert(int ido) ;

        bool sym_invert_gen(int ido) ;
        bool sym_shift_invert_gen(int ido) ;

        bool nsym_regular(int ido) ;
        bool nsym_shift_invert(int ido) ;
        bool nsym_regular_gen(int ido) ;
        bool nsym_invert_gen(int ido) ;
        bool nsym_with_B_invert(int ido) ;

        bool bruno_iter(int ido) ;

        /**
         * Returns the base address of a temporary vector pointed by ipntr_.
         * index uses the same convention as ARPACK documentation.
         * In addition, we have temp_ptr(0) that can be used for temporaries as needed.
         */
        double* temp_ptr(int index) {

            // Note: we add temp_ptr(0) for our own usage.
            if(index == 0) {
                if(temp_.size() == 0) {
                    temp_.allocate(n_) ;
                }
                return temp_.data() ;
            }

            //   Fortran - C indices convertion:
            //      - we use ipntr_(index - 1) since indices start by 1 in Fortran
            //      - we also substract 1 from the pointer in ipntr_ for the same reason
            return workd_.data() + (ipntr_(index - 1) - 1)  ;
        }

        /**
         * Copies a temporary vector pointed by ipntr_.
         * index uses the same convention as ARPACK documentation.
         */
        void copy_temp(int from, int to) {
            // Warning: Graphite uses the inverse convention for to/from 
            // in Memory namespace.
            Memory::copy(temp_ptr(to), temp_ptr(from), n_ * sizeof(double)) ;
        }

        void A_mul(int from, int to) { mult(*A_, temp_ptr(from), temp_ptr(to)) ; }
        void B_mul(int from, int to) { mult(*B_, temp_ptr(from), temp_ptr(to)) ; }
        void OP_mul(int from, int to) { 
            // Silly me, y and x are swapped in SuperLU inverse matrix mult.
            OP_.mult( temp_ptr(to), temp_ptr(from)) ; 
        }

        /**
         * as in ARPACK's documentation.
         */
        int & iparam(unsigned int i) {
            // Fortran-C conversion: substract 1 to index i. 
            return iparam_[i-1] ;
        }

        int n_ ;
        bool symmetric_ ;

        Array1d<int> iparam_ ;
        Array1d<int> ipntr_ ;
        Vector v_ ;
        Vector d_ ;
        Vector workd_ ;
        Vector workl_ ;
        Vector temp_ ; // For our own usage.

        AbstractMatrix OP_ ;

        Array1d<int> permut_ ;
        
        Array1d<int> op_permut_cache_ ;
        const SparseMatrix* A_cache_ ;
        const SparseMatrix* B_cache_ ;
        Array1d<double> resid_cache_ ;
        bool uses_perm_cache_ ;
    } ;
}

#endif


