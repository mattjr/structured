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

#include <OGF/cells/map_algos/angles_to_uv.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/math/numeric/linear_solver.h>

#include <stack>

namespace OGF {

//_______________________________________________________________________


    AnglesToUV::AnglesToUV() {
        mode_ = ANGLES ;
    }


    static void angles_to_uv_iterative(Map* map, MapHalfedgeAttribute<double>& angle)  ;

    bool AnglesToUV::do_parameterize_disc(Map* map) {

        switch(mode_) {
        case ANGLES:
            ogf_assert(angle_.is_defined(map, "angle")) ;
            angle_.bind(map, "angle") ;
            break ;
        case ANGLES_ITERATIVE: {
            ogf_assert(angle_.is_defined(map, "angle")) ;
            angle_.bind(map, "angle") ;
            angles_to_uv_iterative(map, angle_) ;
            angle_.unbind() ;
            return true ;
        } break ;
        case DOT_PRODUCTS:
            ogf_assert(gamma_.is_defined(map, "gamma")) ;
            gamma_.bind(map, "gamma") ;
            break ;
        }

        bool result = MapParameterizerLSCM::do_parameterize_disc(map) ;

        if(angle_.is_bound()) {
            angle_.unbind() ;
        }
        if(gamma_.is_bound()) {
            gamma_.unbind() ;
        }
        return result ;
    }

    Complex AnglesToUV::complex_angle(
        Map::Halfedge* h0,
        Map::Halfedge* h1,
        Map::Halfedge* h2
    ) {
        Complex Z ;
        if(angle_.is_bound()) {
            double alpha0 = angle_[h0] ;
            double alpha1 = angle_[h1] ;
            double alpha2 = angle_[h2] ;
            double scaling = sin(alpha1) / sin(alpha2) ;
            Z = scaling * Complex(::cos(alpha0), ::sin(alpha0)) ;
        } else if(gamma_.is_bound()) {
            double g0 = gamma_[h0] ;
            double g1 = gamma_[h1] ;
            double g2 = gamma_[h2] ;
            double area = ::sqrt(g0*g1 + g1*g2 + g2*g0) ;
            Z = Complex(g0,area) / (g0 + g1) ;
        }
        return Z ;
    } 
    
    void AnglesToUV::setup_conformal_map_relations(
        LinearSolver& solver, 
        Map::Halfedge* h0,
        Map::Halfedge* h1,
        Map::Halfedge* h2
    ) {
        
        Map::Vertex* v0 = h0->vertex() ;
        Map::Vertex* v1 = h1->vertex() ;
        Map::Vertex* v2 = h2->vertex() ;

        int id0 = vertex_id_[v0] ;
        int id1 = vertex_id_[v1] ;
        int id2 = vertex_id_[v2] ;

        // Note  : 2*id + 0 --> u
        //         2*id + 1 --> v

        int u0_id = 2*id0     ;
        int v0_id = 2*id0 + 1 ;
        int u1_id = 2*id1     ;
        int v1_id = 2*id1 + 1 ;
        int u2_id = 2*id2     ;
        int v2_id = 2*id2 + 1 ;
        
        const Point3d& p0 = h0->vertex()->point() ;
        const Point3d& p1 = h1->vertex()->point() ;
        const Point3d& p2 = h2->vertex()->point() ;

        double area = Geom::triangle_area(p0,p1,p2) ;
        double s = ::sqrt(area) ;

        Complex Z = complex_angle(h0, h1, h2) ;

        double a = Z.real();
        double b = Z.imaginary();

        solver.begin_row() ;
        solver.set_right_hand_side(0.0) ;
        solver.add_coefficient(u0_id, 1-a) ;
        solver.add_coefficient(v0_id,  b) ;
        solver.add_coefficient(u1_id,  a) ;
        solver.add_coefficient(v1_id, -b) ;
        solver.add_coefficient(u2_id, -1) ;
        solver.scale_row(s) ;
        solver.end_row() ;

        solver.begin_row() ;
        solver.set_right_hand_side(0.0) ;
        solver.add_coefficient(u0_id, -b) ;
        solver.add_coefficient(v0_id, 1-a) ;
        solver.add_coefficient(u1_id,  b) ;
        solver.add_coefficient(v1_id,  a) ;
        solver.add_coefficient(v2_id, -1) ;
        solver.scale_row(s) ;
        solver.end_row() ;
    }

    
//_______________________________________________________________________

    static Point3d map_barycenter(const Map* map) {
        double x=0,y=0,z=0 ;
        FOR_EACH_VERTEX_CONST(Map, map, it) {
            x+= it->point().x() ;
            y+=it->point().y() ;
            z+=it->point().z() ;
        }
        x /= double(map->size_of_vertices()) ;
        y /= double(map->size_of_vertices()) ;
        z /= double(map->size_of_vertices()) ;        
        return Point3d(x,y,z) ;
    }

    static Map::Facet* most_central_facet(Map* map) {
        Point3d g0 = map_barycenter(map) ;
        Map::Facet* result = nil ;
        double d0 = 1e30 ;
        FOR_EACH_FACET(Map, map, it) {
            Point3d g = Geom::facet_barycenter(it) ;
            double d = (g-g0).norm2() ;
            if(d < d0) {
                d0 = d ;
                result = it ;
            }
        }
        return result ;
    }

    static void unfold(
        Map::Halfedge* h, MapHalfedgeAttribute<double>& angle
    ) {

        Map::Halfedge* h0 = h ;
        Map::Halfedge* h1 = h0->prev() ;
        Map::Halfedge* h2 = h1->prev() ;

        double alpha0 = angle[h0] ;
        double alpha1 = angle[h1] ;
        double alpha2 = angle[h2] ;

        const Point2d& p0 = h0->tex_coord() ;
        const Point2d& p1 = h1->tex_coord() ;
        
        Complex E01(
            p1.x() - p0.x(),
            p1.y() - p0.y() 
        ) ;
        
        Complex Z(
            cos(alpha0) * sin(alpha1) / sin(alpha2),
            sin(alpha0) * sin(alpha1) / sin(alpha2)
        ) ;

        Complex E02 = Z * E01 ;
        
        Point2d p2 = Point2d(
            p0.x() + E02.real(),
            p0.y() + E02.imaginary()
        ) ;

        h2->set_tex_coord(p2) ;
    }

    static void project(Map::Facet* f, MapHalfedgeAttribute<double>& angle) {
        Map::Halfedge* h0 = f->halfedge() ;
        Map::Halfedge* h1 = h0->next() ;
        Map::Halfedge* h2 = h1->next() ;

        h0->set_tex_coord(Point2d(0,0)) ;
        h2->set_tex_coord(Point2d(1,0)) ;

        unfold(h0, angle) ;
    } 


    static void angles_to_uv_iterative(Map* map, MapHalfedgeAttribute<double>& angle) {

        Map::Facet* start = most_central_facet(map) ;

        MapFacetAttribute<bool> is_visited(map) ;
        std::stack<Map::Facet*> S ;
        
        FOR_EACH_FACET(Map,map,it) {
                is_visited[it] = false ;
        }

        project(start, angle) ;
        S.push(start) ;
        is_visited[start] = true ;

        while(!S.empty()) {
            Map::Facet* f = S.top() ;
            S.pop() ;
            Map::Halfedge* h = f->halfedge() ;
            do {
                Map::Halfedge* hh = h->opposite() ;
                Map::Facet* ff = hh->facet() ;
                if(
                    (ff != nil) && !is_visited[ff]
                ) {
                    unfold(hh, angle) ;
                    is_visited[ff] = true ;
                    S.push(ff) ;
                }
                h = h->next() ;
            } while(h != f->halfedge()) ;
        }
    }


}
