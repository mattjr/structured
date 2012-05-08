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
 

#ifndef __OGF_MATH_AVERAGE_DIRECTION__
#define __OGF_MATH_AVERAGE_DIRECTION__

#include <OGF/math/common/common.h>
#include <OGF/math/geometry/types.h>

namespace OGF {

//________________________________________________

    class MATH_API AverageDirection {
    public:
        AverageDirection() ;
        void begin() ;
        void add_vector(const Vector3d& V) ;
        void end() ;
        const Vector3d& average_direction() const { 
            ogf_assert(result_is_valid_) ;
            return result_ ; 
        }
    private:
        double M_[6] ;
        Vector3d result_ ;
        bool result_is_valid_ ;
    } ;

//________________________________________________

    class MATH_API AverageDirection2d {
    public:
        AverageDirection2d() ;
        void begin() ;
        void add_vector(const Vector2d& V) ;
        void end() ;
        const Vector2d& average_direction() const { 
            ogf_assert(result_is_valid_) ;
            return result_ ; 
        }
    private:
        double M_[3] ;
        Vector2d result_ ;
        bool result_is_valid_ ;
    } ;

//________________________________________________

}

#endif

