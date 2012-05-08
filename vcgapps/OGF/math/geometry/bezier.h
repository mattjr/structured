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
 
 

#ifndef __OGF_MATH_GEOMETRY_BEZIER__
#define __OGF_MATH_GEOMETRY_BEZIER__

#include <OGF/math/common/common.h>
#include <OGF/math/geometry/types.h>

namespace OGF {

    /**
     * Computes bezier control meshes for geometry and normals.
     * Geometry: cubic bezier patches 
     * Normals: quadratic bezier patches (decoupled from the geometry)
     * Reference: A. Vlachos, J. Peters, C. Boyd, and J. Mitchell. 
     * Curved PN triangles. In ACM Symposium on Interactive 3D Graphics 2001, 
     * pages 159 166, 2001.
     */
    class MATH_API Bezier {

    public:

        /** geometry only */
        static void compute_bezier_control_mesh(
            const Point3d& b003,
                  Point3d& b012,
                  Point3d& b021,
            const Point3d& b030,
                  Point3d& b102,
                  Point3d& b111,
                  Point3d& b120,
                  Point3d& b201,
                  Point3d& b210,
            const Point3d& b300,

            const Vector3d& n002,
            const Vector3d& n020,
            const Vector3d& n200
        ) ;

        /** geometry and normals */
        static void compute_bezier_control_mesh(
            const Point3d& b003,
                  Point3d& b012,
                  Point3d& b021,
            const Point3d& b030,
                  Point3d& b102,
                  Point3d& b111,
                  Point3d& b120,
                  Point3d& b201,
                  Point3d& b210,
            const Point3d& b300,

            const Vector3d& n002,
                  Vector3d& n011,
            const Vector3d& n020,
                  Vector3d& n101,
                  Vector3d& n110,
            const Vector3d& n200
        ) ;
    } ;
}

#endif
