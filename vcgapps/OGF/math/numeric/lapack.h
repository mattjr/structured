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

#include <OGF/math/common/common.h>
#include <OGF/basic/types/types.h>
#include <math.h>

#ifndef __OGF_MATH_NUMERIC_LAPACK__
#define __OGF_MATH_NUMERIC_LAPACK__

namespace OGF {

    // LAPACK wrappers
    namespace LAPACK {
    
        /**
         * computes the Cholesky factorization of a symmetric matrix.
         * @param uplo is one of "U","L"
         * @param n dimension of the matrix
         * @param a (in): matrix to factorize, (out): L or U Cholesky factor
         * @param lda: leading dimension of array a
         * @param info (out) info = 0: OK, info < 0: the -info arg is invalid, info > 0: non symmetric positive
         */
        int MATH_API dpotrf(
            char* uplo, int n, double* a, int lda, int* info
        ) ;
    
        /**
         * solves a linear system using a Cholesky factorization 
         * @param uplo is one of "U","L"
         * @param n dimension of the matrix
         * @param nrhs number of right hand sides
         * @param a L or U Cholesky factor, computed e.g. by @ref doptrf.
         * @param lda leading dimension of array a
         * @param bx (in): right hand sides, (out): solution
         * @param ldb leading dimension of array b
         * @param info (out) info = 0: OK, info < 0: the -info arg is invalid
         */
        int MATH_API dpotrs(
            char* uplo, int n, int nrhs, double* a, int lda, double* bx, int ldb, int* info
        ) ;


		/* LU factorization of a general matrix and solve using this factorization */

		int MATH_API dgetrf(int m, int n, double* a, int lda, int* ipiv, int* info);
		int MATH_API dgetrs(char trans, int n, int nrhs, double* a, int lda, int* ipiv, double* b, int ldb, int* info);
    }

}

#endif
