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
#include <OGF/math/numeric/lapack.h>
#include <OGF/math/linear_algebra/matrix.h>
#include <OGF/basic/types/types.h>


#ifndef __OGF_MATH_NUMERIC_DIRECT_SOLVERS__
#define __OGF_MATH_NUMERIC_DIRECT_SOLVERS__

namespace OGF {
    
    namespace Numeric {
        template <int N> inline bool solve_SPD_system(
            const Matrix<double, N>& M, const double* b, double* x
        ) {
            double store[N*N] ;
            Memory::copy(store, (void*)(M.data()), sizeof(double) * N * N) ;
            Memory::copy(x, (void *)b, sizeof(double) * N) ;
            int info ;
            LAPACK::dpotrf("L", N, store, N, &info) ;
            if(info != 0) { return false ; }
            LAPACK::dpotrs("L", N, 1, store, N, x, N, &info) ;
            return (info == 0) ;
        }



        template <int N> inline bool solve_system(const Matrix<double, N>& M, const double* rhs, double* x) 
		{
            double store[N*N];
			int pivots[N];
			int info;

            Memory::copy(store, (void*)(M.data()), sizeof(double) * N * N);  
            LAPACK::dgetrf(N, N, store, N, pivots, &info);

            if(info)
				return false;

			Memory::copy(x, (void*)rhs, sizeof(double) * N);
            LAPACK::dgetrs('N', N, 1, store, N, pivots, x, N, &info); 

            return !info;
        }
    }

}

#endif

