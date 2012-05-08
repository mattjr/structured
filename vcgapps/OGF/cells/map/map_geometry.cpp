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


#include <OGF/cells/map/geometry.h>
#include <OGF/math/geometry/oriented_line.h>
#include <OGF/math/geometry/polygon2d.h>
#include <OGF/math/geometry/polygon3d.h>

namespace OGF {

    MapNormalizer::MapNormalizer(Map* map) {
        map_ = map ;
        center_ = Point3d(0,0,0) ;
        double num = 0 ;
        { FOR_EACH_VERTEX(Map, map_, it) {
            Point3d p = it-> point() ;
            center_ = Point3d(
                center_.x() + p.x(),
                center_.y() + p.y(),
                center_.z() + p.z()
            ) ;
            num++ ;
        }}
        center_ = Point3d(
            center_.x() / num,
            center_.y() / num,
            center_.z() / num
        ) ;

        radius_ = 0 ;
        { FOR_EACH_VERTEX(Map, map_, it) {
            Vector3d v = it->point() - center_ ;
            radius_ = ogf_max(radius_, v.norm()) ;
        }}

        normalized_radius_ = 1.0 ;
    }    

    void MapNormalizer::apply(double normalized_radius) {
        normalized_radius_ = normalized_radius ;
        { FOR_EACH_VERTEX(Map, map_, it) {
            Vector3d v = it->point() - center_ ;
            v = (normalized_radius_ / radius_) * v ;
            it->set_point(ORIGIN + v) ;
        }}
    }

    void MapNormalizer::unapply() {
        { FOR_EACH_VERTEX(Map, map_, it) {
            const Point3d& p = it->point() ;
            Vector3d v = p - ORIGIN ;
            v = (radius_ / normalized_radius_) * v ;
            it->set_point(center_ + v) ;
        }}
    }

    void MapNormalizer::normalize_tex_coords() {
        double u_min =  Numeric::big_double ;
        double v_min =  Numeric::big_double ;
        double u_max = -Numeric::big_double ;
        double v_max = -Numeric::big_double ;

        { FOR_EACH_VERTEX(Map, map_, it) {
            double u = it->halfedge()->tex_coord().x() ;
            double v = it->halfedge()->tex_coord().y() ;
            u_min = OGF::ogf_min(u_min, u) ;
            v_min = OGF::ogf_min(v_min, v) ;
            u_max = OGF::ogf_max(u_max, u) ;
            v_max = OGF::ogf_max(v_max, v) ;
        }} 
        
        if((u_max - u_min > 1e-6) && (v_max - v_min > 1e-6)) {
            double delta = OGF::ogf_max(u_max - u_min, v_max - v_min) ;
            { FOR_EACH_VERTEX(Map, map_, it) {
                double u = it->halfedge()->tex_coord().x() ;
                double v = it->halfedge()->tex_coord().y() ;
                u = (u - u_min) / delta ;
                v = (v - v_min) / delta ;
                it->halfedge()->set_tex_coord(OGF::Point2d(u,v)) ;
            }}
        }
    }

//________________________________________________________________________

    namespace Geom {

        Vector3d vertex_normal(const Map::Vertex* v) {
            Vector3d result(0,0,0) ;
            Map::Halfedge* cir = v->halfedge();
            do {
                if (!cir->is_border()) {
                    Vector3d v0 = vector(cir->next()) ;
                    Vector3d v1 = vector(cir->opposite());
                    Vector3d n = v0 ^ v1 ;
                    result = result + n ;
                }
                cir = cir->next_around_vertex() ;
            } while (cir != v->halfedge());
            result.normalize();
            return result;
        }


        Vector3d facet_normal(const Map::Facet* f) {
            Vector3d result(0,0,0) ;
            Map::Halfedge* cir = f->halfedge();
            do {
                Vector3d v0 = vector(cir) ;
                Vector3d v1 = vector(cir->prev()->opposite()) ;
                Vector3d n = v0 ^ v1 ;
                result = result + n ;
                cir = cir->next() ;
            } while(cir != f->halfedge()) ;
            result.normalize() ;
            return result ;
        }
        

        Vector3d triangle_normal(const Map::Facet* f){
            ogf_assert(f->is_triangle()) ;
            Vector3d result = (
                vector(f->halfedge()->next()) ^
                vector(f->halfedge()->opposite())
            ) + (
                vector(f->halfedge()) ^
                vector(f->halfedge()->prev()->opposite())
            ) + ( 
                vector(f->halfedge()->next()->next()) ^
                vector(f->halfedge()->next()->opposite())
            ) ;
            result.normalize();
            return result;
        }

        Point2d vertex_barycenter2d(const Map::Vertex* v) {
            double x = 0 ;
            double y = 0 ;
            double nb = 0 ;
            Map::Halfedge* it = v->halfedge() ;
            do {
                const Point2d& p = it-> tex_vertex()-> tex_coord() ;
                x += p.x() ;
                y += p.y() ;
                nb++ ;
                it = it->next_around_vertex() ;
            } while(it != v->halfedge()) ;
            return Point2d(x/nb, y/nb) ;
        }

        Point3d facet_barycenter(const Map::Facet* f){
            double x=0 ; double y=0 ; double z=0 ;
            int nb_vertex = 0 ;
            Map::Halfedge* cir = f->halfedge();
            do {
                nb_vertex++;
                x+= cir->vertex()->point().x() ;
                y+= cir->vertex()->point().y() ;
                z+= cir->vertex()->point().z() ;
                cir = cir->next();
            } while (cir != f->halfedge());
            return  Point3d(
                x / double(nb_vertex),
                y / double(nb_vertex),
                z / double(nb_vertex)
            ) ;
        }


        Point2d facet_barycenter2d(const Map::Facet* f){
            double x=0 ; double y=0 ; 
            int nb_vertex = 0 ;
            Map::Halfedge* cir = f->halfedge();
            do {
                nb_vertex++;
                x+= cir->tex_coord().x() ;
                y+= cir->tex_coord().y() ;
                cir = cir->next();
            } while (cir != f->halfedge());
            return Point2d(
                x / double(nb_vertex),
                y / double(nb_vertex)
            ) ;
        }


/*
  // I do not trust this one for the moment ...
        double facet_area(const Map::Facet* f) {
            Vector3d n = facet_normal(f) ;
            Vector3d w(0,0,0) ;
            Map::Halfedge* it = f->halfedge() ;
            do {
                Vector3d v1(
                    it-> vertex()-> point().x(),
                    it-> vertex()-> point().y(),
                    it-> vertex()-> point().z()
                ) ;
                Vector3d v2(
                    it-> next()-> vertex()-> point().x(),
                    it-> next()-> vertex()-> point().y(),
                    it-> next()-> vertex()-> point().z()
                ) ;
                w = w + (v1 ^ v2) ;
                it = it->next() ;
            } while(it != f->halfedge()) ;
            return 0.5 * ::fabs(w * n) ;
        }
*/

        double facet_area(const Map::Facet* f) {
            double result = 0 ;
            Map::Halfedge* h = f->halfedge() ;
            const Point3d& p = h->vertex()->point() ;
            h = h->next() ;
            do {
                result += triangle_area(
                    p,
                    h->vertex()->point(),
                    h->next()->vertex()->point() 
                ) ;
                h = h->next() ;
            } while(h != f->halfedge()) ;
            return result ;
        }



        double facet_signed_area2d(const Map::Facet* f) {
            double result = 0 ;
            Map::Halfedge* it = f->halfedge() ;
            do {
                const Point2d& t1 = it-> tex_coord() ;
                const Point2d& t2 = it-> next()-> tex_coord() ;
                result += t1.x() * t2.y() - t2.x() * t1.y() ;
                it = it->next() ;
            } while(it != f->halfedge()) ;
            result /= 2.0 ;
            return result ;
        }


/*
        double facet_signed_area2d(const Map::Facet* f) {
            double result = 0 ;
            Map::Halfedge* h = f->halfedge() ;
            const Point2d& p = h->tex_coord() ;
            h = h->next() ;
            do {
                result += triangle_signed_area(
                    p,
                    h->tex_coord(),
                    h->next()->tex_coord()
                ) ;
                h = h->next() ;
            } while(h != f->halfedge()) ;
            return result ;
        }
*/


        void facet_barycentric_coords(
            const Map::Facet* f, const Point3d& p, std::vector<double>& bary
        ) {
            Polygon3d P ;
            Map::Halfedge* h = f->halfedge() ;
            do {
                P.push_back(h->vertex()->point()) ;
                h = h->next() ;
            } while(h != f->halfedge()) ;
            barycentric_coords(P, p, bary) ;
        }
        
        void facet_barycentric_coords2d(
            const Map::Facet* f, const Point2d& p, std::vector<double>& bary
        ) {
            Polygon2d P ;
            Map::Halfedge* h = f->halfedge() ;
            do {
                P.push_back(h->tex_coord()) ;
                h = h->next() ;
            } while(h != f->halfedge()) ;
            barycentric_coords(P, p, bary) ;
        }

        Point2d CELLS_API facet_xyz_to_uv(const Map::Facet* f, const Point3d& p) {
            std::vector<double> bary ;
            facet_barycentric_coords(f, p, bary) ;
            Map::Halfedge* h = f->halfedge() ;
            unsigned int i =0 ;
            double u = 0 ;
            double v = 0 ;
            do {
                u += bary[i] * h->tex_coord().x() ;
                v += bary[i] * h->tex_coord().y() ;
                h = h->next() ;
                i++ ;
            } while(h != f->halfedge()) ;
            return Point2d(u,v) ;
        }

        Point3d CELLS_API facet_uv_to_xyz(const Map::Facet* f, const Point2d& p) {
            std::vector<double> bary ;
            facet_barycentric_coords2d(f, p, bary) ;
            Map::Halfedge* h = f->halfedge() ;
            unsigned int i =0 ;
            double x = 0 ;
            double y = 0 ;
            double z = 0 ;
            do {
                x += bary[i] * h->vertex()->point().x() ;
                y += bary[i] * h->vertex()->point().y() ;
                z += bary[i] * h->vertex()->point().z() ;
                h = h->next() ;
                i++ ;
            } while(h != f->halfedge()) ;
            return Point3d(x,y,z) ;
        }

        double border_signed_area2d(const Map::Halfedge* h) {
            ogf_assert(h->is_border()) ;
            double result = 0 ;
            const Map::Halfedge* it = h ;
            do {
                const Point2d& t1 = it-> tex_coord() ;
                const Point2d& t2 = it-> next()-> tex_coord() ;
                result += t1.x() * t2.y() - t2.x() * t1.y() ;
                it = it->next() ;
            } while(it != h) ;
            result /= 2.0 ;
            return result ;
        }


        bool line_intersects_facet(
            const OrientedLine& line,
            const Map::Facet* f
        ) {
            // Uses Plucker coordinates (see OrientedLine)
            Sign face_sign = ZERO ;
            Map::Halfedge* h = f->halfedge() ;
            do {
                OrientedLine edge_line(
                    h->vertex()->point(), h->opposite()->vertex()->point()
                ) ;
                Sign cur_sign = OrientedLine::side(line, edge_line) ;
                if(
                    cur_sign != ZERO && face_sign != ZERO &&
                    cur_sign != face_sign
                ) {
                    return false ;
                }
                if(cur_sign != ZERO) {
                    face_sign = cur_sign ;
                }
                h = h->next() ;
            } while(h != f->halfedge()) ;
            return true ;
        }

        double border_length(Map::Halfedge* start) {
            ogf_assert(start->is_border()) ;
            double result = 0 ;
            Map::Halfedge* cur = start ;
            do {
                result += edge_length(cur) ;
                cur = cur->next() ;
            } while(cur != start) ;
            return result ;
        }

        double border_length2d(Map::Halfedge* start) {
            ogf_assert(start->is_border()) ;
            double result = 0 ;
            Map::Halfedge* cur = start ;
            do {
                result += edge_length2d(cur) ;
                cur = cur->next() ;
            } while(cur != start) ;
            return result ;
        }

        double map_area(const Map* map) {
            double result = 0 ;
            FOR_EACH_FACET_CONST(Map, map, it) {
                result += facet_area(it) ;
            }
            return result ;
        }

        double map_area2d(const Map* map) {
            double result = 0 ;
            FOR_EACH_FACET_CONST(Map, map, it) {
                result += facet_area2d(it) ;
            }
            return result ;
        }

        Box3d map_bbox(const Map* map) {
            Box3d result ;
            FOR_EACH_VERTEX_CONST(Map, map, it) {
                result.add_point(it->point()) ;
            }
            return result ;
        }
        
        Box2d map_bbox2d(const Map* map) {
            Box2d result ;
            FOR_EACH_HALFEDGE_CONST(Map, map, it) {
                result.add_point(it->tex_vertex()->tex_coord()) ;
            }
            return result ;
        }

        Box3d border_bbox(Map::Halfedge* h) {
            Box3d result ;
            Map::Halfedge* cur = h ;
            do {
                result.add_point(cur->vertex()->point()) ;
                cur = cur->next() ;
            } while(cur != h) ;
            return result ;
        }
        
        Box2d border_bbox2d(Map::Halfedge* h) {
            Box2d result ;
            Map::Halfedge* cur = h ;
            do {
                result.add_point(cur->tex_coord()) ;
                cur = cur->next() ;
            } while(cur != h) ;
            return result ;
        }

        
        void normalize_map_tex_coords(Map* map) {
            if(map->size_of_vertices() == 0) {
                return ;
            }
            Box2d B = map_bbox2d(map) ;
            double sx = (B.width() > 0) ? 1.0/B.width() : 0.0 ;
            double sy = (B.height() > 0) ? 1.0/B.height() : 0.0 ;
            std::set<Map::TexVertex*> tex_vertices ;
            FOR_EACH_HALFEDGE(Map, map, it) {
                tex_vertices.insert(it->tex_vertex()) ;
            }
            for(std::set<Map::TexVertex*>::iterator it = tex_vertices.begin(); it != tex_vertices.end(); it++) {
                Map::TexVertex* cur = *it ;
                double x = cur->tex_coord().x() ;
                double y = cur->tex_coord().y() ;
                cur->set_tex_coord(
                    Point2d(
                        sx * (x - B.x_min()),
                        sy * (y - B.y_min())
                    )
                ) ;
            }
        }


//__________________________________________________________________________

    }

}
