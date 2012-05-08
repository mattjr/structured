/*
 *  GXML/Graphite: Geometry and Graphics Programming Library + Utilities
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

#include <OGF/cells/map_algos/map_bezier.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map/map_editor.h>
#include <OGF/math/geometry/bezier.h>

namespace OGF {

    void MapBezier::compute_bezier_control_mesh() {
        // Sanity check: only works for triangulated surfaces
        MapEditor editor(map_) ;
        editor.split_surface(split_triangulate) ;

        MapTexVertexNormal normal(map_) ;
        MapHalfedgeAttribute<Point3d>  h_control(map_, "bezier") ;
        MapFacetAttribute<Point3d>     f_control(map_, "bezier") ;
        MapHalfedgeAttribute<Vector3d> h_normal_control(map_, "bezier_normal") ;

        FOR_EACH_FACET(Map, map_, it) {
            Map::Halfedge* h3 = it->halfedge() ;
            Map::Halfedge* h1 = h3->next() ;
            Map::Halfedge* h2 = h1->next() ;

            const Point3d&  b300 = h2->vertex()->point() ;
            const Vector3d& n200 = normal[h2->tex_vertex()] ;

            const Point3d&  b030 = h3->vertex()->point() ;
            const Vector3d& n020 = normal[h3->tex_vertex()] ;

            const Point3d&  b003 = h1->vertex()->point() ;
            const Vector3d& n002 = normal[h1->tex_vertex()] ;


            Point3d& b120 = h_control[h3] ;
            Point3d& b210 = h_control[h3->opposite()] ;
            Point3d& b012 = h_control[h1] ;
            Point3d& b021 = h_control[h1->opposite()] ;
            Point3d& b201 = h_control[h2] ;
            Point3d& b102 = h_control[h2->opposite()] ;
            Point3d& b111 = f_control[it] ;

            Vector3d& n110 = h_normal_control[h3] ;
            Vector3d& n011 = h_normal_control[h1] ;
            Vector3d& n101 = h_normal_control[h2] ;


            Bezier::compute_bezier_control_mesh(
                b003, b012, b021, b030, b102, b111, b120, b201, b210, b300,
                n002, n011, n020, n101, n110, n200
            ) ;
        }
    }

}
