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
 

#include <OGF/math/geometry/average_direction.h>
#include <OGF/math/linear_algebra/matrix_util.h>

namespace OGF {

    //______________________________________________________________

    AverageDirection::AverageDirection() {
        result_is_valid_ = false ;
    }

    void AverageDirection::begin() {
        for(int i=0; i<6; i++) {
            M_[i] = 0.0 ;
        }
    }

    void AverageDirection::add_vector(const Vector3d& e) {
        M_[0] += e.x() * e.x() ;
        M_[1] += e.x() * e.y() ;
        M_[2] += e.y() * e.y() ;
        M_[3] += e.x() * e.z() ;
        M_[4] += e.y() * e.z() ;
        M_[5] += e.z() * e.z() ;        
    }

    void AverageDirection::end() {
        double eigen_vectors[9] ;
        double eigen_values[6] ;
        
        MatrixUtil::semi_definite_symmetric_eigen(M_, 3, eigen_vectors, eigen_values) ;
        
        // Sort the eigen vectors
        int i[3] ;
        i[0] = 0 ;
        i[1] = 1 ;
        i[2] = 2 ;
        
        double l0 = ::fabs(eigen_values[0]) ;
        double l1 = ::fabs(eigen_values[1]) ;
        double l2 = ::fabs(eigen_values[2]) ;
        
        if(l1 > l0) {
            ogf_swap(l0   , l1   ) ;
            ogf_swap(i[0], i[1]) ;
        }
        if(l2 > l1) {
            ogf_swap(l1   , l2   ) ;
            ogf_swap(i[1], i[2]) ;
        }
        if(l1 > l0) {
            ogf_swap(l0   , l1  ) ;
            ogf_swap(i[0],i[1]) ;
        }
        int k = 0 ;
        result_ = Vector3d(
            eigen_vectors[3*i[k]  ],
            eigen_vectors[3*i[k]+1],
            eigen_vectors[3*i[k]+2]
        ) ;
        result_is_valid_ = true ;
    }
        
    //______________________________________________________________


    AverageDirection2d::AverageDirection2d() {
        result_is_valid_ = false ;
    }

    void AverageDirection2d::begin() {
        for(int i=0; i<3; i++) {
            M_[i] = 0.0 ;
        }
    }

    void AverageDirection2d::add_vector(const Vector2d& e) {
        M_[0] += e.x() * e.x() ;
        M_[1] += e.x() * e.y() ;
        M_[2] += e.y() * e.y() ;
    }

    void AverageDirection2d::end() {
        double eigen_vectors[4] ;
        double eigen_values[2] ;
        MatrixUtil::semi_definite_symmetric_eigen(M_, 2, eigen_vectors, eigen_values) ;
        int k = 0 ;
        result_ = Vector2d(
            eigen_vectors[2*k],
            eigen_vectors[2*k+1]
        ) ;
        result_is_valid_ = true ;
    }

    //______________________________________________________________

}

