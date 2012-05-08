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
 
 
 
 

#ifndef __OGF_MATH_GEOMETRY_MATRIX_VECTOR__
#define __OGF_MATH_GEOMETRY_MATRIX_VECTOR__

#include <OGF/math/common/common.h>
#include <OGF/math/geometry/types.h>

#ifndef __opencxx

namespace OGF {
    
//_________________________________________________________
    
    template <class FT> GenericPoint2d<FT> inline operator*(
        const GenericPoint2d<FT>& v,
        const Matrix<FT, 4>& m
    ) {
        FT result[4] ;
        FT w[4] ;
        int i,j ;

        for(i=0; i<4; i++) {
            result[i] = 0 ;
        }
        w[0] = v.x() ;
        w[1] = v.y() ;
        w[2] = FT(0) ;
        w[3] = FT(1) ;

        for(i=0; i<4; i++) {
            for(j=0; j<4; j++) {
                result[i] += w[j] * m(j,i) ;
            }
        }
        
        return GenericPoint2d<FT> (
            result[0] / result[3],
            result[1] / result[3]
        ) ; 
    }

//________
    
    template <class FT> GenericVector3d<FT> inline operator*(
        const GenericVector3d<FT>& v,
        const Matrix<FT, 4>& m
    ) {
        int i,j ;
        FT result[4] ;

        for(i=0; i<4; i++) {
            result[i] = 0 ;
        }
        for(i=0; i<4; i++) {
            for(j=0; j<4; j++) {
                result[i] += v.homogeneous(j) * m(j,i) ;
            }
        }
        
        return GenericVector3d<FT>(
            result[0], result[1], result[2]
        ) ; 
    }


    template <class FT> GenericPoint3d<FT>  inline operator*(
        const GenericPoint3d<FT>& v, const Matrix<FT, 4>& m
    ) {
        int i,j ;
        FT result[4] ;
        
        for(i=0; i<4; i++) {
            result[i] = 0 ;
        }
        for(i=0; i<4; i++) {
            for(j=0; j<4; j++) {
                result[i] += v.homogeneous(j) * m(j,i) ;
            }
        }
    
        return GenericPoint3d<FT>(
            result[0] / result[3],
            result[1] / result[3],
            result[2] / result[3] 
        ) ; 
    }


//_________________________________________________________

}

#endif

#endif
