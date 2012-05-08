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

#include <OGF/math/numeric/lapack.h>

#define OGF_FORTRAN(x) x##_

extern "C" {
    int OGF_FORTRAN(dpotrf)(char* uplo, int* n, double* a, int* lda, int* info) ;
    int OGF_FORTRAN(dpotrs)(char* uplo, int* n, int* nrhs, double* a, int* lda, double* bx, int* ldb, int* info) ;

	int OGF_FORTRAN(dgetrf)(int *m, int* n, double* a, int*	lda, int* ipiv, int* info);
	int OGF_FORTRAN(dgetrs)(char* trans, int* n, int* nrhs, double* a, int* lda, int* ipiv, double* b, int* ldb, int* info);
}

namespace OGF {

    namespace LAPACK {
        
        int dpotrf(
            char* uplo, int n, double* a, int lda, int* info
        ) {
            return OGF_FORTRAN(dpotrf)(uplo, &n, a, &lda, info) ;
        }
    
        int dpotrs(
            char* uplo, int n, int nrhs, double* a, int lda, double* bx, int ldb, int* info
        ) {
            return OGF_FORTRAN(dpotrs)(uplo, &n, &nrhs, a, &lda, bx, &ldb, info) ;
        }

		int dgetrf(int m, int n, double* a, int lda, int* ipiv, int* info)
		{
			return OGF_FORTRAN(dgetrf)(&m, &n, a, &lda, ipiv, info);
		}

		int dgetrs(char trans, int n, int nrhs, double* a, int lda, int* ipiv, double* b, int ldb, int* info)
		{
			return OGF_FORTRAN(dgetrs)(&trans, &n, &nrhs, a, &lda, ipiv, b, &ldb, info);
		}
    }

}
