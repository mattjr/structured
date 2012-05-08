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
 

#ifndef __CELLS_MAP_GEOMETRY__
#define __CELLS_MAP_GEOMETRY__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map.h>
#include <OGF/math/geometry/oriented_line.h>

namespace OGF {

//_________________________________________________________


    class CELLS_API MapNormalizer {
    public:
        MapNormalizer(Map* map) ;
        void apply(double normalized_radius = 1.0) ;
        void unapply() ;
        void normalize_tex_coords() ;

    private:
        Map* map_ ;
        Point3d center_ ;
        double radius_ ;
        double normalized_radius_ ;
    } ;

    // Adds some functions related to Map to the Geom namespace.
    namespace Geom {

        inline Vector3d vector(const Map::Halfedge* h){
            return h->vertex()->point() - h->prev()->vertex()->point();
        }

        inline Vector2d vector2d(const Map::Halfedge* h){
            return h->tex_coord() - h->prev()->tex_coord();
        }
        
        Vector3d CELLS_API vertex_normal(const Map::Vertex* v) ;
        Vector3d CELLS_API facet_normal(const Map::Facet* f) ; 
        Vector3d CELLS_API triangle_normal(const Map::Facet* f) ;

        /**
         * returns the barycenter of all TexVertices
         * associated with v.
         */
        Point2d CELLS_API vertex_barycenter2d(const Map::Vertex* v) ;

        inline Point3d edge_barycenter(const Map::Halfedge* h) {
            return barycenter(
                h->vertex()->point(),
                h->opposite()->vertex()->point()
            ) ;
        }


        inline Point2d edge_barycenter2d(const Map::Halfedge* h) {
            return barycenter(
                h->tex_coord(),
                h->prev()->tex_coord()
            ) ;
        }

        Point3d CELLS_API facet_barycenter(const Map::Facet* f) ;
        Point2d CELLS_API facet_barycenter2d(const Map::Facet* f) ;

        

        double CELLS_API facet_area(const Map::Facet* f) ;

        double CELLS_API facet_signed_area2d(const Map::Facet* f) ;

        inline double facet_area2d(const Map::Facet* f) {
            return ::fabs(facet_signed_area2d(f)) ;
        }

        void CELLS_API facet_barycentric_coords(
            const Map::Facet* f, const Point3d& p, std::vector<double>& bary
        ) ;
        
        void CELLS_API facet_barycentric_coords2d(
            const Map::Facet* f, const Point2d& p, std::vector<double>& bary
        ) ;


        Point2d CELLS_API facet_xyz_to_uv(const Map::Facet* f, const Point3d& p) ;

        Point3d CELLS_API facet_uv_to_xyz(const Map::Facet* f, const Point2d& p) ;

        double CELLS_API border_signed_area2d(const Map::Halfedge* h) ;


        inline double border_area2d(const Map::Halfedge* h) {
            return ::fabs(border_signed_area2d(h)) ;
        }

        inline double edge_length(const Map::Halfedge* h) {
            return vector(h).norm() ;
        }

        inline double edge_length2d(const Map::Halfedge* h) {
            return vector2d(h).norm() ;
        }

        inline double triangle_area(const Map::Facet* f) {
            ogf_assert(f->is_triangle()) ;
            Map::Halfedge* h1 = f->halfedge() ;
            Map::Halfedge* h2 = h1->next() ;
            Map::Halfedge* h3 = h2->next() ;            
            return triangle_area(
                h1->vertex()->point(),
                h2->vertex()->point(),
                h3->vertex()->point()
            ) ;
        }

        inline double triangle_area2d(const Map::Facet* f) {
            ogf_assert(f->is_triangle()) ;
            Map::Halfedge* h1 = f->halfedge() ;
            Map::Halfedge* h2 = h1->next() ;
            Map::Halfedge* h3 = h2->next() ;            
            return triangle_area(
                h1->tex_coord(),
                h2->tex_coord(),
                h3->tex_coord()
            ) ;
        }

        /**
         * Note: I am not sure of this one for
         * non-convex facets.
         */
        bool CELLS_API line_intersects_facet(
            const OrientedLine& line,
            const Map::Facet* f
        ) ;

        double CELLS_API border_length(Map::Halfedge* start) ;
        double CELLS_API border_length2d(Map::Halfedge* start) ;

        double CELLS_API map_area(const Map* map) ;
        double CELLS_API map_area2d(const Map* map) ;

        Box3d CELLS_API map_bbox(const Map* map) ; 
        Box2d CELLS_API map_bbox2d(const Map* map) ;

        void CELLS_API normalize_map_tex_coords(Map* map) ;        

        Box3d CELLS_API border_bbox(Map::Halfedge* h) ;
        Box2d CELLS_API border_bbox2d(Map::Halfedge* h) ;

    }

//_________________________________________________________

}
#endif

