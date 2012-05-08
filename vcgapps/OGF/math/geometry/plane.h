
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

#ifndef __OGF_MATH_GEOMETRY_PLANE__
#define __OGF_MATH_GEOMETRY_PLANE__

#include <OGF/basic/types/types.h>

namespace OGF 
{
    
     // A 3D Plane of equation a.x + b.y + c.z + d = 0
     
    struct Plane {
        Plane(const Point3d& p1, const Point3d& p2, const Point3d& p3) {
            Vector3d n = (p2-p1) ^ (p3-p1) ;
            a = static_cast<float>(n.x()) ;
            b = static_cast<float>(n.y()) ;
            c = static_cast<float>(n.z()) ;
            d = static_cast<float>(-( a*p1.x() + b*p1.y() + c*p1.z() )) ;
        }
        Plane(const Point3d& p, const Vector3d& n) {
            a = static_cast<float>(n.x()) ;
            b = static_cast<float>(n.y()) ;
            c = static_cast<float>(n.z()) ;
            d = static_cast<float>(-( a*p.x() + b*p.y() + c*p.z() )) ;
        }
        Plane() { }
        Vector3d normal() const { return Vector3d(a,b,c) ; }
        float a,b,c,d ;
    } ;

    // Returns 1 or 0 according to the side of the plane P where
    //   the point p is.
    inline int orient(const Plane& P, const Point3d& p) {
        return (P.a*p.x() + P.b*p.y() + P.c*p.z() + P.d) > 0.0 ? 1 : 0 ;
    }
}

#endif
