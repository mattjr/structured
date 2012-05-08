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

#ifndef __OGF_MATH_GEOMETRY_POLYGON2D__
#define __OGF_MATH_GEOMETRY_POLYGON2D__

#include <OGF/math/common/common.h>
#include <OGF/math/geometry/types.h>
#include <vector>

namespace OGF {

    typedef std::vector<Point2d> Polygon2d ;

    class Segment2d {
    public:
        Segment2d(const Point2d& p1_in, const Point2d& p2_in) : p1(p1_in), p2(p2_in), empty(false) { }
        Segment2d(const Point2d& p1_in, const Vector2d& v_in) : p1(p1_in), p2(p1_in + v_in), empty(false) { }
        Segment2d() : empty(true) { }
        const Point2d& origin() const { return p1 ; }
        Vector2d vector() const { return p2 - p1 ; }
        Point2d p1 ;
        Point2d p2 ;
        bool empty ;
    } ;

    namespace Geom {

        void MATH_API save_polygon(const Polygon2d& P, const std::string& file_name) ;

        double MATH_API signed_area(const Polygon2d& P) ;

        inline double area(const Polygon2d& P) { 
            return ::fabs(signed_area(P)) ; 
        }

        void MATH_API convex_hull(const Polygon2d& P, Polygon2d& result) ;

        /**
         * V1 and V2 are the normalized axes of the
         * minimum area enclosing rectangle.
         */
        void MATH_API minimum_area_enclosing_rectangle(
            const Polygon2d& P, Vector2d& V1, Vector2d& V2
        ) ;


        bool MATH_API point_is_in_kernel(
            const Polygon2d& P, const Point2d& p
        ) ;

        /** 
         * Note: the barycenter of a polygon is not 
         * the barycenter of its vertices 
         */
        Point2d MATH_API barycenter(const Polygon2d& P) ;

        Point2d MATH_API vertices_barycenter(const Polygon2d& P) ;

        void MATH_API clip_polygon_by_half_plane(
            const Polygon2d& P, 
            const Point2d& p1,
            const Point2d& p2,
            Polygon2d& result,
            bool invert = false
        ) ;

        void MATH_API clip_segment_by_half_plane(
            Segment2d& S,
            const Point2d& Q1, const Point2d& Q2,
            bool invert = false 
        ) ;

        void MATH_API kernel(const Polygon2d& P, Polygon2d& result) ;

        /**
         * Precondition: polygon_is_convex(window)
         */
        void MATH_API convex_clip_polygon(
            const Polygon2d& P, const Polygon2d& window, Polygon2d& result
        ) ;

        /**
         * Precondition: polygon_is_convex(window)
         */
        void MATH_API convex_clip_segment(
            Segment2d& S, const Polygon2d& window
        ) ;
        
        bool MATH_API polygon_is_convex(const Polygon2d& P) ;

        /**
         * Computes the barycentric coordinates of the point P
         * inside the polygon defined by a list of points.
         *
         * It uses the formula w = (cot APB + cot BPC) / || BP ||
         * as defined in "Generalized Barycentric Coordinates on Irregular Polygons"
         * Meyer, Lee, Barr, Desbrun 2002
         * www.cs.caltech.edu/~mmeyer/Publications/barycentric.pdf
         *
         * It defaults to the division-free formula (6) in the same article
         * in degenerate cases.
         */
        void MATH_API barycentric_coords(const Polygon2d& P, const Point2d& p, std::vector<double>& bary) ;
        

    }

}


#endif
