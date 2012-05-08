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

#include <OGF/math/geometry/bezier.h>

namespace OGF {

    static inline Point3d bezier_tangent_control(
        const Point3d& p1, const Vector3d& n1, const Point3d& p2
    ) {
        double w12 = (p2 - p1) * n1 ;
        return Point3d(
            (2 * p1.x() + p2.x() - w12 * n1.x()) / 3.0,
            (2 * p1.y() + p2.y() - w12 * n1.y()) / 3.0,
            (2 * p1.z() + p2.z() - w12 * n1.z()) / 3.0
        ) ;
    }

    static inline Vector3d bezier_normal_control(
        const Point3d& p1, const Vector3d& n1, const Point3d& p2, const Vector3d& n2
    ) {

        Vector3d p1p2 = p2 - p1 ;
        double l = p1p2.norm2() ;
        Vector3d h ;
        if(l > 1e-30) {
            double v12 = 2.0 * ( p1p2 * (n1 + n2) ) / l ;
            h = n1 + n2 - v12 * p1p2 ;
        } else {
            h = n1 + n2 ;
        }
        h.normalize() ;

        // Farin's improved formula (ensures that n(0.5, 0.5, 0) = h)

        h.set_x( 3.0 * h.x() / 2.0 - n1.x() / 4.0 - n2.x() / 4.0) ;
        h.set_y( 3.0 * h.y() / 2.0 - n1.y() / 4.0 - n2.y() / 4.0) ;
        h.set_z( 3.0 * h.z() / 2.0 - n1.z() / 4.0 - n2.z() / 4.0) ;

        return h ;
    }

    void Bezier::compute_bezier_control_mesh(
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
    ) {
        b012 = bezier_tangent_control(b003, n002, b030) ;
        b021 = bezier_tangent_control(b030, n020, b003) ;
        b102 = bezier_tangent_control(b003, n002, b300) ;
        b120 = bezier_tangent_control(b030, n020, b300) ;
        b201 = bezier_tangent_control(b300, n200, b003) ;
        b210 = bezier_tangent_control(b300, n200, b030) ;

        Point3d e(
            (b012.x() + b021.x() + b102.x() + b120.x() + b201.x() + b210.x()) / 6.0,
            (b012.y() + b021.y() + b102.y() + b120.y() + b201.y() + b210.y()) / 6.0,
            (b012.z() + b021.z() + b102.z() + b120.z() + b201.z() + b210.z()) / 6.0
        ) ;

        Point3d v = Geom::barycenter(b003,b030,b300) ;
        b111 = e + 0.5 * (e - v) ;

        n011 = bezier_normal_control(b003, n002, b030, n020) ;
        n101 = bezier_normal_control(b003, n002, b300, n200) ;
        n110 = bezier_normal_control(b030, n020, b300, n200) ;
    }

    void Bezier::compute_bezier_control_mesh(
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
    ) {
        b012 = bezier_tangent_control(b003, n002, b030) ;
        b021 = bezier_tangent_control(b030, n020, b003) ;
        b102 = bezier_tangent_control(b003, n002, b300) ;
        b120 = bezier_tangent_control(b030, n020, b300) ;
        b201 = bezier_tangent_control(b300, n200, b003) ;
        b210 = bezier_tangent_control(b300, n200, b030) ;

        Point3d e(
            (b012.x() + b021.x() + b102.x() + b120.x() + b201.x() + b210.x()) / 6.0,
            (b012.y() + b021.y() + b102.y() + b120.y() + b201.y() + b210.y()) / 6.0,
            (b012.z() + b021.z() + b102.z() + b120.z() + b201.z() + b210.z()) / 6.0
        ) ;

        Point3d v = Geom::barycenter(b003,b030,b300) ;
        b111 = e + 0.5 * (e - v) ;
    }
}

