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
 

#ifndef __OGF_MATH_NUMERIC_SUPER_LU__
#define __OGF_MATH_NUMERIC_SUPER_LU__

#include <OGF/math/common/common.h>
#include <OGF/math/linear_algebra/vector.h>
#include <OGF/math/linear_algebra/abstract_matrix.h>
#include <OGF/basic/containers/arrays.h>

// TODO1: write a general API and factory for direct solvers,
// TODO2: remove the symmetric argument (and use the symmetric tag of 
//   the sparse matrix instead).

namespace OGF {

    class SparseMatrix ;

    /**
     * A driver for the SuperLU solver, inspired by Alla Sheffer's
     * "cow flattener" code.
     * @param M should be square, and should have a row storage 
     *             without symmetric storage optimization.
     * @param b is the right hand side.
     * @param x returns the solution.
     * @param perm permutation vector. If non-initialized (i.e. size==0),
     *   constructs a new one.
     * @returns true on success, false on failure (if M is singular).
     */            
    bool MATH_API solve_super_lu(
        SparseMatrix& M, const double* b, double* x,
        Array1d<int>& perm, bool clear_M = false, bool symmetric = false 
    ) ;
        

    /**
     * A driver for the SuperLU solver, inspired by Alla Sheffer's
     * "cow flattener" code.
     * @param M should be square, and should have a row storage 
     *             without symmetric storage optimization.
     * @param b is the right hand side.
     * @param x returns the solution.
     * @param perm toggles approximate minimum degree ordering.
     * @returns true on success, false on failure (if M is singular).
     */            
    bool MATH_API solve_super_lu(
        const SparseMatrix& M, const double* b, double* x,
        bool perm = true, bool symmetric = false 
    ) ;


    /**
     * A driver for the SuperLU solver, inspired by Alla Sheffer's
     * "cow flattener" code.
     * @param M should be square, and should have a row storage 
     *             without symmetric storage optimization.
     * @param b is the right hand side.
     * @param x returns the solution.
     * @param perm permutation vector. If non-initialized (i.e. size==0),
     *   constructs a new one.
     * @param clear_M if set, clears M (saves memory for SuperLU).
     * @returns true on success, false on failure (if M is singular).
     */            
    inline bool solve_super_lu(
        SparseMatrix& M, const Vector& b, Vector& x,
        Array1d<int>& perm, bool clear_M = false, bool symmetric = false
    ) {
        return solve_super_lu(M, b.data(), x.data(), perm, clear_M, symmetric) ;
    }


    /**
     * A driver for the SuperLU solver, inspired by Alla Sheffer's
     * "cow flattener" code.
     * @param M should be square, and should have a row storage 
     *             without symmetric storage optimization.
     * @param b is the right hand side.
     * @param x returns the solution.
     * @param perm toggles approximate minimum degree ordering.
     * @returns true on success, false on failure (if M is singular).
     */            
    inline bool solve_super_lu(
        const SparseMatrix& M, const Vector& b, Vector& x,
        bool perm = true, bool symmetric = false 
    ) {
        return solve_super_lu(M, b.data(), x.data(), perm, symmetric) ;
    }

    // --------------- Lower-level interface (exposes the inverse)


    /**
     * If perm is set, uses the permutation in perm.
     */
    bool invert_super_lu(const SparseMatrix& M, AbstractMatrix& Minv, int* perm = nil) ;

    /**
     * If perm is uninitialized, computes a permutation and stores it in perm,
     * else uses the permutation in perm.
     */
    bool invert_super_lu(const SparseMatrix& M, AbstractMatrix& Minv, Array1d<int>& perm) ;

}

#endif
