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
 

#include <OGF/cells/map_algos/map_parameterizer.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/cells/map/map_builder.h>
#include <OGF/math/geometry/principal_axes.h>
#include <OGF/math/numeric/solver.h>
#include <OGF/basic/debug/logger.h>
#include <OGF/basic/os/stopwatch.h>
#include <OGF/basic/containers/arrays.h>
#include <OGF/basic/containers/checked_vector.h>

#include <algorithm>
#include <stack>

namespace OGF {
//___________________________________________________________________________

    MapParameterizer::MapParameterizer() : map_(nil) { 
        triangles_only_ = false ;
        use_locked_vertices_as_corners_ = false ;
    }

    MapParameterizer::~MapParameterizer() {
    }

//___________________________________________________________________________
    
    bool MapParameterizer::parameterize_disc(MapComponent* disc) {
        if(do_parameterize_disc(disc)) {
            return true ;
        }

/*
        {
            std::cerr << "MapParameterizer::parameterize_disc - checking MapComponent locked vertices on border" << std::endl ;
            MapVertexLock is_locked(disc->map()) ;
            FOR_EACH_VERTEX(MapComponent, disc, it) {
                if(is_locked[it] && it->is_on_border()) {
                    std::cerr << "============ locked vertex on border" << std::endl ;
                }
            }
        }
*/

        Map* map = component_to_map(disc) ;
//        std::cerr << "component_to_map: map = " << map << std::endl ;
        bool result = do_parameterize_disc(map) ;
        update_from_map(map) ;
        delete map ;
        return result ;
    }
 
    bool MapParameterizer::parameterize_disc(Map* disc) {
        bool result ;
        if(triangles_only_ && !disc->is_triangulated()) {
            Map* map = triangulate(disc) ;
            result = do_parameterize_disc(map) ;
            update_from_map(map) ;
            delete map ;
        } else {
            result = do_parameterize_disc(disc) ;
        }
        return result ;
    }

    Map* MapParameterizer::component_to_map(MapComponent* comp) {

        MapVertexAttribute<int> vertex_id(comp->map()) ;
        {
            int id = 0 ;
            FOR_EACH_VERTEX(MapComponent, comp, it) {
                vertex_id[it] = id ;
                id++ ;
            }
        }
        Map* result = new Map ;
        MapVertexLock from_is_locked(comp->map()) ;
        MapVertexLock to_is_locked(result) ;

        MapVertexAttribute<Map::Vertex*> orig_vertex(result, "orig_vertex") ;
        MapBuilder builder(result) ;
        builder.begin_surface() ;
        FOR_EACH_VERTEX(MapComponent, comp, it) {
            builder.add_vertex(it->point()) ;
            orig_vertex[builder.current_vertex()] = it ;
            to_is_locked[builder.current_vertex()] = from_is_locked[it] ;
        }
        FOR_EACH_FACET(MapComponent, comp, it) {
            std::vector<int> v ;
            Map::Halfedge* h = it->halfedge() ;
            do {
                v.push_back(vertex_id[h->vertex()]) ;
                h = h->next() ;
            } while(h != it->halfedge()) ;
            if(triangles_only_) {
                for(unsigned int i=1; (i+1)<v.size(); i++) {
                    builder.begin_facet() ;
                    builder.add_vertex_to_facet(v[0]) ;
                    builder.add_vertex_to_facet(v[i]) ;
                    builder.add_vertex_to_facet(v[i+1]) ;
                    builder.end_facet() ;
                }
            } else {
                builder.begin_facet() ;
                for(unsigned int i=0; i<v.size(); i++) {
                    builder.add_vertex_to_facet(v[i]) ;
                }
                builder.end_facet() ;
            }
        }
        builder.end_surface() ;
        return result ;
    }

    Map* MapParameterizer::triangulate(Map* map) {
        MapVertexAttribute<int> vertex_id(map) ;
        {
            int id = 0 ;
            FOR_EACH_VERTEX(Map, map, it) {
                vertex_id[it] = id ;
                id++ ;
            }
        }
        Map* result = new Map ;
        MapVertexLock from_is_locked(map) ;
        MapVertexLock to_is_locked(result) ;
        MapVertexAttribute<Map::Vertex*> orig_vertex(result, "orig_vertex") ;
        MapBuilder builder(result) ;
        builder.begin_surface() ;
        FOR_EACH_VERTEX(Map, map, it) {
            builder.add_vertex(it->point()) ;
            orig_vertex[builder.current_vertex()] = it ;
            to_is_locked[builder.current_vertex()] = from_is_locked[it] ;
        }
        FOR_EACH_FACET(Map, map, it) {
            std::vector<int> v ;
            Map::Halfedge* h = it->halfedge() ;
            do {
                v.push_back(vertex_id[h->vertex()]) ;
                h = h->next() ;
            } while(h != it->halfedge()) ;
            
            for(unsigned int i=1; (i+1)<v.size(); i++) {
                builder.begin_facet() ;
                builder.add_vertex_to_facet(v[0]) ;
                builder.add_vertex_to_facet(v[i]) ;
                builder.add_vertex_to_facet(v[i+1]) ;
                builder.end_facet() ;
            }
        }
        builder.end_surface() ;
        return result ;
    }

    
    void MapParameterizer::update_from_map(Map* map) {
        ogf_assert(MapVertexAttribute<Map::Vertex*>::is_defined(map, "orig_vertex")) ;
        MapVertexAttribute<Map::Vertex*> orig_vertex(map, "orig_vertex") ;
        FOR_EACH_VERTEX(Map, map, it) {
            Map::Vertex* v = orig_vertex[it] ;
            if(v != nil) {
                v->halfedge()->set_tex_coord(
                    it->halfedge()->tex_coord()
                ) ;
            } else {
                std::cerr << "nil orig_vertex !" << std::endl ;
            }
        }
    }
    
//___________________________________________________________________________

    void MapParameterizer::parameterize_map(Map* map) {
        MapComponentsExtractor extr ;
        MapComponentList comps = extr.extract_components(map) ;
        for(unsigned int i=0; i<comps.size(); i++) {
            MapComponent* cur = comps[i] ;
            parameterize_disc(cur) ;
        }
    }

    void MapParameterizer::begin_parameterization(Map* map) {
        map_ = map ;
        vertex_id_.bind(map) ;
        get_bounding_box() ;
        enumerate_vertices() ;
        nb_distinct_vertices_ = map_->size_of_vertices() ;

// Note about the following (commented-out) code: 
// this seems to cause more problems than what this fixes
//        if(component->size_of_facets() > 10) {
//            nb_distinct_vertices_ = discard_small_edges() ;
//        } else {
//            nb_distinct_vertices_ = map_->size_of_vertices() ;
//        }
    }

    void MapParameterizer::end_parameterization() {
        map_ = nil ;
        vertex_id_.unbind() ;
    }
    
    void MapParameterizer::get_bounding_box() {
        // Compute center
        double x = 0 ;
        double y = 0 ;
        double z = 0 ;
        FOR_EACH_VERTEX(Map, map_, it) {
            x += it->point().x() ;
            y += it->point().y() ;
            z += it->point().z() ;
        }
        x /= double(map_->size_of_vertices()) ;
        y /= double(map_->size_of_vertices()) ;
        z /= double(map_->size_of_vertices()) ;
        center_ = Point3d(x,y,z) ;

        // Compute radius
        radius_ = 0 ;
        FOR_EACH_VERTEX(Map, map_, it) {
            double r = (it->point() - center_).norm() ;
            radius_ = ogf_max(radius_, r) ;
        }
        
        if(radius_ < 1e-20) {
//            std::cerr << "Tiny component" << std::endl ;
            radius_ = 1.0 ;
        }
    }
    
    void MapParameterizer::normalize(Point3d& p) {
        Vector3d v = 1.0 / radius_ * (p - center_) ;
        p = Point3d(v.x(), v.y(), v.z()) ;
    }

    void MapParameterizer::enumerate_vertices() {
        int cur_id = 0 ;
        FOR_EACH_VERTEX(Map, map_, it) {
            vertex_id_[it] = cur_id ;
            cur_id++ ;
        }        
    }

    bool MapParameterizer::do_parameterize_disc(MapComponent* component) {
        return false ;
    }
    
    int MapParameterizer::discard_small_edges(
        double relative_treshold 
    ) {
        int nb_discarded = 0 ;
        
        double max_edge_len = 0.0 ;
        FOR_EACH_EDGE(Map, map_, it) {
            max_edge_len = ogf_max(max_edge_len, Geom::edge_length(it)) ;
        }
        
        double treshold = relative_treshold * max_edge_len ;
        
        FOR_EACH_EDGE(Map, map_, it) {
            if(Geom::edge_length(it) < treshold) {
                nb_discarded++ ;
                edge_collapse(it) ;
            }
        }


        // Update vertices numbering (i.e. remove "numbering gaps")

        int nb_vertices = map_->size_of_vertices() ;

        Array1d<int> new_id(nb_vertices);

        for(int i=0; i<nb_vertices; i++) {
            new_id(i) = -1 ;
        }
        
        int cur = 0 ;
        FOR_EACH_VERTEX(Map, map_, it) {
            int old_id = vertex_id_[it] ;
            if(new_id(old_id) == -1) {
                new_id(old_id) = cur ;
                cur++ ;
            }
        }

        FOR_EACH_VERTEX(Map, map_, it) {
            int old_id     = vertex_id_[it] ;
            vertex_id_[it] = new_id( old_id ) ;
        }        

        if(nb_discarded > 0) {
//            std::cerr << "discarded " << nb_discarded
//                      << " edge(s) " << std::endl ;
        }

        return cur ;
    }
    
    void MapParameterizer::edge_collapse(Map::Halfedge* h) {

        int id1 = vertex_id_[h->vertex()] ;
        int id2 = vertex_id_[h->opposite()->vertex()] ;

        // Already collapsed (for instance, the third edge of
        // a null triangle)
        if(id1 == id2) {
            return ;
        }
        
        // Propagate the vertex identification (it can happen that
        // the edge collapse merges two sets of already merged 
        // vertices).
        std::stack<Map::Vertex*> stack ;
        stack.push(h->opposite()->vertex()) ;
        while(!stack.empty()) {
            Map::Vertex* top = stack.top() ;
            stack.pop() ;
            vertex_id_[top->halfedge()->vertex()] = id1 ;
            Map::Halfedge* cur = top->halfedge() ;
            do {
                Map::Vertex* neigh = cur->opposite()->vertex() ;
                if(vertex_id_[neigh->halfedge()->vertex()] == id2) {
                    stack.push(neigh) ;
                }
                cur = cur->next_around_vertex() ;
            } while(cur != top->halfedge()) ;
        }
    }
    
    bool MapParameterizer::halfedge_is_discarded(Map::Halfedge* h) {
        return ( 
            vertex_id_[h->prev()->vertex()] == vertex_id_[h->vertex()] 
        ) ;
    }

    bool MapParameterizer::facet_is_discarded(Map::Facet* f) {
        Map::Halfedge* h = f->halfedge() ;
        do {
            if(!halfedge_is_discarded(h)) {
                return false ;
            }
            h = h->next() ;
        } while(h != f->halfedge()) ;
        return true ;
    }
    
    Map::Halfedge* MapParameterizer::largest_border() {
        Map::Halfedge* result = nil ;
        int max_size = 0 ;
        
        MapHalfedgeAttribute<bool> is_visited(map_) ;
        FOR_EACH_HALFEDGE(Map, map_, it) {
            if(it->is_border() && !is_visited[it]) {
                int cur_size = 0 ;
                Map::Halfedge* cur = it ;
                do {
                    cur_size++ ;
                    is_visited[cur] = true ;
                    cur = cur->next() ;
                } while(cur != it) ;
                if(cur_size > max_size) {
                    max_size = cur_size ;
                    result = it ;
                }
            }
        }
        return result ;
    }
    
    void MapParameterizer::principal_axes(
        Point3d& center, Vector3d& v1, Vector3d& v2
    ) {

        Vector3d average_normal(0,0,0) ;
        FOR_EACH_FACET(Map, map_, it) {
            average_normal = average_normal + Geom::facet_normal(it) ;
        }
        if(average_normal.norm() > 1e-30) {
            average_normal.normalize() ;
            double w_min =  1e30 ;
            double w_max = -1e30 ;
            FOR_EACH_VERTEX(Map, map_, it) {
                double w = (it->point() - Origin()) * average_normal ;
                w_min = ogf_min(w, w_min) ;
                w_max = ogf_max(w, w_max) ;
            }
            if((w_max - w_min) < 1e-6) {
                v1 = Geom::perpendicular(average_normal) ;
                v1.normalize() ;
                v2 = average_normal ^ v1 ;
                v2.normalize() ;
                double cx = 0 ;
                double cy = 0 ;
                double cz = 0 ;
                FOR_EACH_VERTEX(Map, map_, it) {
                    cx += it->point().x() ;
                    cy += it->point().y() ;
                    cz += it->point().z() ;
                }
                center = Point3d(
                    cx / double(map_->size_of_vertices()),
                    cy / double(map_->size_of_vertices()),
                    cz / double(map_->size_of_vertices())
                ) ;
                return ;
            }
        }
        

        PrincipalAxes3d axes ;
        axes.begin_points() ;
        FOR_EACH_VERTEX(Map, map_, it) {
            Point3d p = it->point() ;
            normalize(p) ;
            axes.point(p) ;
        }
        axes.end_points() ;
        center = axes.center() ;
        v1 = axes.axis(0) ;
        v2 = axes.axis(1) ;
            
        // If numerical problems occured, project relative to the shortest
        // edge of the bounding box.
        if(Geom::is_nan(center) || Geom::is_nan(v1) || Geom::is_nan(v2)) {
            Box3d box = Geom::map_bbox(map_) ;
            box_axes(box, center, v1, v2) ;
        } 

        if(average_normal.norm() > 1e-30 && (v1 ^ v2)*average_normal < 0) {
            v2 = -1.0 * v2 ;
        }

    }



    void MapParameterizer::principal_axes(
        Map::Halfedge* h, Point3d& center, Vector3d& v1, Vector3d& v2
    ) {
        ogf_assert(h->is_border()) ;

        PrincipalAxes3d axes ;
        axes.begin_points() ;
        Map::Halfedge* cur = h ;
        do {
            Point3d p = cur->vertex()->point() ;
            normalize(p) ;
            axes.point(p) ;
            cur = cur->next() ;
        } while(cur != h) ;
        axes.end_points() ;
        center = axes.center() ;
        v1 = axes.axis(0) ;
        v2 = axes.axis(1) ;
            
        // If numerical problems occured, project relative to the shortest
        // edge of the bounding box.
        if(Geom::is_nan(center) || Geom::is_nan(v1) || Geom::is_nan(v2)) {
            Box3d box = Geom::border_bbox(h) ;
            box_axes(box, center, v1, v2) ;
        }
    }
    
    void MapParameterizer::box_axes(
        const Box3d& box_in, Point3d& center, Vector3d& v1, Vector3d& v2
    ) {
        Point3d p_min(box_in.x_min(), box_in.y_min(), box_in.z_min()) ;
        Point3d p_max(box_in.x_max(), box_in.y_max(), box_in.z_max()) ;

        normalize(p_min) ;
        normalize(p_max) ;

        Box3d box ;
        box.add_point(p_min) ;
        box.add_point(p_max) ;

        center = Geom::barycenter(
            Point3d(box.x_min(), box.y_min(), box.z_min()),
            Point3d(box.x_max(), box.y_max(), box.z_max())
        ) ;

        double dx = box.x_max() - box.x_min() ;
        double dy = box.y_max() - box.y_min() ;
        double dz = box.z_max() - box.z_min() ;
        if(dz > dy && dz > dx) {
            v1 = Vector3d(1,0,0) ;
            v2 = Vector3d(0,1,0) ;
        } else if(dy > dx && dy > dz) {
            v1 = Vector3d(0,0,1) ;
            v2 = Vector3d(1,0,0) ;
        } else {
            v1 = Vector3d(0,1,0) ;
            v2 = Vector3d(0,0,1) ;
        }
    }

    void MapParameterizer::project_on_principal_plane() {
        Point3d center ;
        Vector3d v1 ;
        Vector3d v2 ;
        principal_axes(center, v1, v2) ;
        v1.normalize() ;
        v2.normalize() ;
        FOR_EACH_VERTEX(Map, map_, it) {
            double u = (it->point() - center) * v1 ;
            double v = (it->point() - center) * v2 ;
            it->halfedge()->set_tex_coord(Point2d(u,v)) ;
        }
    }


//___________________________________________________________________________

    ParamValidator::ParamValidator() {
        graph_size_ = 1024 ;
        graph_mem_ = new Numeric::uint8[graph_size_ * graph_size_] ;
        x_left_  = new int[graph_size_] ;
        x_right_ = new int[graph_size_] ;
        max_overlap_ratio_ = 0.005 ;
        max_scaling_ = 20.0 ;
        min_fill_ratio_ = 0.25 ;
    }

    ParamValidator::~ParamValidator() {
        delete[] graph_mem_ ; graph_mem_ = nil ;
        delete[] x_left_ ;  x_left_ = nil ;
        delete[] x_right_ ; x_right_ = nil ;
    }

    bool ParamValidator::component_is_valid(MapComponent* comp) {

        FOR_EACH_HALFEDGE(MapComponent, comp, it) {
            if(Numeric::is_nan(it->tex_coord().x()) || Numeric::is_nan(it->tex_coord().y())) {
                Logger::out("ParamValidator") << "NaN detected in tex coords" << std::endl ;
                return false ;
            }
        }

        //   Check global overlaps and "wire-like" charts
        // (wasting parameter space)
        compute_fill_and_overlap_ratio(comp) ;
        Logger::out("ParamValidator") 
            << "Fill ratio = " << fill_ratio() << std::endl ;
        Logger::out("ParamValidator") 
            << "Overlap ratio = " << overlap_ratio() << std::endl ;

        double comp_scaling = component_scaling(comp) ;
        Logger::out("ParamValidator")
            << "Scaling = " << comp_scaling << std::endl ;


        // Ignore problems for small components.
        // TODO: check if we can remove that
        // (unfortunately, does not seems so...)
        if(comp->size_of_facets() <= 10) {
            Logger::out("ParamValidator")
                << "----> PASS: small component, #facets="
                << comp->size_of_facets()
                << std::endl ;
            return true ;
        }

        // If more than 'min_fill_ratio_' of the parameter space is empty, 
        // reject chart.
        if(Numeric::is_nan(fill_ratio()) || fill_ratio() < min_fill_ratio_) {
            Logger::out("ParamValidator")
                << "----> REJECT: filling ratio"
                << std::endl ;
            return false ;
        }

        // If more than 'max_overlap_ratio_' of the pixels correspond to more than one
        // facet, reject chart.
        if(Numeric::is_nan(overlap_ratio()) || overlap_ratio() > max_overlap_ratio_) {
            Logger::out("ParamValidator")
                << "----> REJECT: overlap ratio"
                << std::endl ;
            return false ;
        }

        if(Numeric::is_nan(comp_scaling) || comp_scaling > max_scaling_) {
            Logger::out("ParamValidator")
                << "----> REJECT: scaling "
                << std::endl ;
            return false ;
        }

        Logger::out("ParamValidator")
            << "----> PASS." << std::endl ;
        return true ;
    }

    double ParamValidator::component_scaling(MapComponent* comp) {

        // Compute largest facet area.
        double max_area = 0 ;
        FOR_EACH_FACET(MapComponent, comp, it) {
            max_area = ogf_max(Geom::facet_area(it), max_area) ;
        }

        // Ignore facets smaller than 1% of the largest facet.
        double area_treshold = 0.001 * max_area ;

        std::vector<double> facet_scaling ;
        facet_scaling.reserve(comp->size_of_facets()) ;

        FOR_EACH_FACET(MapComponent, comp, it) {
            double area   = Geom::facet_area(it)   ;
            double area2d = Geom::facet_area2d(it) ;
            if(area > area_treshold) {
                facet_scaling.push_back(area2d / area) ;
            } 
        }

        // Ignore 1% of the values at each end.
        std::sort(facet_scaling.begin(), facet_scaling.end()) ;
        int offset = int(double(facet_scaling.size()) * 0.01) ;
        int begin = offset ;
        int end = facet_scaling.size() - 1 - offset ;
        return facet_scaling[end] / facet_scaling[begin] ;
    }


    void ParamValidator::compute_fill_and_overlap_ratio(MapComponent* comp) {
        begin_rasterizer(comp) ;
        FOR_EACH_FACET(MapComponent,comp,f) {
            Map::Halfedge* cur = f-> halfedge() ;
            Map::Halfedge* h0 = cur ;
            cur = cur-> next() ;
            Map::Halfedge* h1 = cur ;
            cur = cur-> next() ;
            Map::Halfedge* h2 = cur ;
            do {
                rasterize_triangle(
                    h0->tex_coord(), 
                    h1->tex_coord(), 
                    h2->tex_coord()
                ) ;
                h1 = cur ;
                cur = cur-> next() ;
                h2 = cur ;
            } while (h2 != h0) ;
        }
        end_rasterizer() ;
    }

    void ParamValidator::begin_rasterizer(MapComponent* comp) {
        ::memset(graph_mem_, 0, graph_size_ * graph_size_) ;
        
        Box2d box = Geom::component_bbox2d(comp) ;
        
        user_x_min_  = box.x_min() ;
        user_y_min_  = box.y_min() ;
        user_width_  = box.width() ;
        user_height_ = box.height() ;
        user_size_ = ogf_max(user_width_, user_height_) ;
    }

    
    void ParamValidator::transform(const Point2d& p, int& x, int& y) {
        x = int( double(graph_size_-1) * (p.x() - user_x_min_) / user_size_) ;
        y = int( double(graph_size_-1) * (p.y() - user_y_min_) / user_size_) ;
        ogf_clamp(x,0,graph_size_-1) ;
        ogf_clamp(y,0,graph_size_-1) ;
    }
    
    void ParamValidator::rasterize_triangle(
        const Point2d& p1, const Point2d& p2, const Point2d& p3
    ) {
        int x[3] ;
        int y[3] ;

        transform(p1,x[0],y[0]) ;
        transform(p2,x[1],y[1]) ;
        transform(p3,x[2],y[2]) ;

        int ymin = 32767 ;
        int ymax = -1 ;
        
        { for(int i=0; i<3; i++) {
            ymin = ogf_min(ymin, y[i]) ;
            ymax = ogf_max(ymax, y[i]) ;
        } }

        int signed_area = 
            (x[1] - x[0]) * (y[2] - y[0]) - 
            (x[2] - x[0]) * (y[1] - y[0]) ;
        bool ccw = (signed_area < 0) ;

        if(ymin == ymax) {
            return ;
        }

        { for(int i=0; i<3; i++) {
            int j=(i+1)%3 ;
            int x1 = x[i] ;
            int y1 = y[i] ;
            int x2 = x[j] ;
            int y2 = y[j] ;
            if(y1 == y2) {
                continue ;
            }
            bool is_left = (y2 < y1) ^ ccw ;

            // I want the set of lit pixels to be
            // independant from the order of the 
            // extremities.
            bool swp = false ;
            if(y2 == y1) {
                if(x1 > x2) {
                    swp = 1 ;
                }
            } else {
                if(y1 > y2) {
                    swp = 1 ;
                }
            }
            if(swp) {
                int tmp ;
                tmp = x2 ;
                x2 = x1 ;
                x1 = tmp ;
                tmp = y2 ;
                y2 = y1 ;
                y1 = tmp ;
            }

            // Bresenham algo.
            int dx = x2 - x1 ;
            int dy = y2 - y1 ;
            int sx = dx > 0 ? 1 : -1 ;
            int sy = dy > 0 ? 1 : -1 ;
            dx *= sx ;
            dy *= sy ;
            int x = x1 ;
            int y = y1 ;
            
            int* line_x = is_left ? x_left_ : x_right_ ;
            line_x[y] = x ;
            
            int e = dy - 2 * dx;
            while(y < y2 - 1) {
                
                y += sy ;
                e -= 2 * dx ;
                
                while(e < 0) {
                    x += sx ;
                    e += 2 * dy ;
                }
                
                line_x[y] = x ;
            }
            
            line_x[y2] = x2 ;
        }}
        
        {for(int y = ymin; y < ymax; y++) {
            for(int x = x_left_[y]; x < x_right_[y]; x++) {
                graph_mem_[y * graph_size_ + x]++ ;
            }
        }}
    }
    
    void ParamValidator::end_rasterizer() {
        int nb_filled = 0 ;
        int nb_overlapped = 0 ;

        int width = 0 ;
        int height = 0 ;
        if(user_width_ > user_height_) {
            width  = graph_size_ ;
            height = int((user_height_ * double(graph_size_)) / user_width_) ;
        } else {
            height = graph_size_ ;
            width  = int((user_width_ * double(graph_size_)) / user_height_) ;
        }

        for(int x=0; x<width; x++) {
            for(int y=0; y<height; y++) {
                Numeric::uint8 pixel = graph_mem_[y * graph_size_ + x] ;
                if(pixel > 0) {
                    nb_filled++ ;
                    if(pixel > 1) {
                        nb_overlapped++ ;
                    }
                }
            }
        }

        fill_ratio_ = double(nb_filled) / double(width * height) ;
        overlap_ratio_ = double(nb_overlapped) / double(width * height) ;
    }

    void MapParameterizer::solver_to_map(const Solver& solver) {
        FOR_EACH_VERTEX(Map, map_, it) {
            double u = solver.variable(2 * vertex_id_[it]    ).value() ;
            double v = solver.variable(2 * vertex_id_[it] + 1).value() ;
            it->halfedge()->set_tex_coord(Point2d(u,v)) ;
        }
    }

    void MapParameterizer::get_border_extrema(
        Map::Halfedge* h, 
        const Point3d& center, const Vector3d& V,
        Map::Vertex*& vx_min, Map::Vertex*& vx_max
    ) {
        vx_min = nil ;
        vx_max = nil ;
        Map::Halfedge* cur = h ;
        double v_min =  Numeric::big_double ;
        double v_max = -Numeric::big_double ;
        do {
            Point3d p = cur->vertex()->point() ;
            normalize(p) ;
            double v = (p - center) * V ;

            if(v < v_min) {
                v_min = v ;
                vx_min = cur->vertex() ;
            }
                
            if(v > v_max) {
                v_max = v ;
                vx_max = cur->vertex() ;
            }

            cur = cur->next() ;
        } while(cur != h) ;
    }

    void MapParameterizer::border_to_solver(Solver& solver, Map::Halfedge* b) {
        Map::Halfedge* h = b ;
        do {
            solver.variable(2 * vertex_id_[h->vertex()]    ).lock() ;
            solver.variable(2 * vertex_id_[h->vertex()]    ).set_value(h->tex_coord().x()) ;
            solver.variable(2 * vertex_id_[h->vertex()] + 1).lock() ;
            solver.variable(2 * vertex_id_[h->vertex()] + 1).set_value(h->tex_coord().y()) ;
            h = h->next() ;
        } while(h != b) ;
    }
    
    void MapParameterizer::project_border_on_circle(Map::Halfedge* b) {
        int nb = 0 ;
        Map::Halfedge* h = b ;
        do { 
            nb++ ; 
            h = h->next() ; 
        } while(h != b) ;
        

        int i = 0 ;
        h = b ;
        do { 

            double alpha = 2.0 * M_PI * double(i) / double(nb) ;
            Point2d q(sin(alpha), cos(alpha)) ;
            h->set_tex_coord(q) ;

            i++ ; 
            h = h->next() ; 
        } while(h != b) ;
        
    }


    void MapParameterizer::project_border_on_square(Map::Halfedge* from, bool use_locks) {

        MapVertexLock is_locked(map_) ;

        if(use_locks) {
            bool found_corner = false ;
            Map::Halfedge* h = from ;
            do {
                if(is_locked[h->vertex()]) {
                    from = h ;
                    found_corner = true ;
                } else {
                    h = h->next() ;
                }
            } while(h != from) ;
            if(!found_corner) {
                std::cerr << "did not find corner, projecting on circle" << std::endl ;
                project_border_on_circle(from) ;
                return ;
            }
            ogf_assert(found_corner) ;
        }

        int nb_locks = 0 ;
        checked_vector<Map::Vertex*> border ;
        Map::Halfedge* h = from ;
        do {
            if(is_locked[h->vertex()]) { nb_locks++ ; }
            border.push_back(h->vertex()) ;
            h = h->next() ;
        } while(h != from) ;

        int b[5] ;
        if(use_locks) {
            if(nb_locks != 4) {
                project_border_on_convex_polygon(from) ;
                return ;
            }
            int j = 0 ;
            for(unsigned int i=0; i<border.size(); i++) {
                if(is_locked[border[i]]) {
                    b[j] = i ;
                    j++ ;
                    ogf_assert(j <= 4) ;
                }
            }
            ogf_assert(j == 4) ; // check that we have 4 corners
            b[4] = border.size() ;
        } else {
            b[0] = 0 ;
            b[1] =     border.size() / 4 ;
            b[2] = 2 * border.size() / 4 ;
            b[3] = 3 * border.size() / 4 ;
            b[4] = border.size() ;
        }

        checked_vector<double> cs ;
        cs.push_back(0.0) ;
        {for(unsigned int i=1; i<border.size(); i++) {
            cs.push_back(
                (border[i]->point() - border[i-1]->point()).norm()
            ) ;
            cs[i] += cs[i-1] ;
        }}
        cs.push_back(
            (border[0]->point() - border[ border.size() - 1]->point()).norm()
        ) ;
        cs[cs.size() - 1] += cs[cs.size() - 2] ;

        int i ;
        
        for(i = b[0]; i<b[1]; i++) {
            double s = double(cs[i] - cs[b[0]]) / double(cs[b[1]] - cs[b[0]]) ;
            border[i]->halfedge()->set_tex_coord(Point2d(s,0.0)) ;
        }
        
        for(i = b[1]; i<b[2]; i++) {
            double s = double(cs[i] - cs[b[1]]) / double(cs[b[2]] - cs[b[1]]) ;
            border[i]->halfedge()->set_tex_coord(Point2d(1.0,s)) ;
        }

        for(i = b[2]; i<b[3]; i++) {
            double s = double(cs[i] - cs[b[2]]) / double(cs[b[3]] - cs[b[2]]) ;
            border[i]->halfedge()->set_tex_coord(Point2d(1.0 - s,1.0)) ;
        }

        for(i = b[3]; i<b[4]; i++) {
            double s = double(cs[i] - cs[b[3]]) / double(cs[b[4]] - cs[b[3]]) ;
            border[i]->halfedge()->set_tex_coord(Point2d(0.0,1.0 - s)) ;
        }

    }

    void MapParameterizer::project_border_on_convex_polygon(Map::Halfedge* from) {
        double total_length = 0.0 ;
        std::vector<Map::Halfedge*> corners ;
        MapVertexLock is_locked(map_) ;
        Map::Halfedge* h = from ;
        do {
            total_length += Geom::edge_length(h) ;
            if(is_locked[h->vertex()]) {
                corners.push_back(h) ;
            }
            h = h->next() ;
        } while(h != from) ;
        
        std::vector<Point2d> tex_coord ;
        switch(corners.size()) {
        case 3: {
            tex_coord.push_back(Point2d(0,0)) ;
            tex_coord.push_back(Point2d(1,0)) ;
            tex_coord.push_back(Point2d(0,1)) ;
        }
        case 4: {
            tex_coord.push_back(Point2d(0,0)) ;
            tex_coord.push_back(Point2d(1,0)) ;
            tex_coord.push_back(Point2d(1,1)) ;
            tex_coord.push_back(Point2d(0,1)) ;
        }
        default: {
            tex_coord.push_back(Point2d(1.0, 0.0)) ;
            double cur_alpha = 0.0 ;
            for(unsigned int i=1; i<corners.size(); i++) {
                cur_alpha += 2.0 * M_PI * edge_length(corners[i-1], corners[i]) / total_length ;
                tex_coord.push_back(Point2d(::cos(cur_alpha), ::sin(cur_alpha))) ;
            }
        }
        }

        for(unsigned int i=1; i<corners.size(); i++) {
            parameterize_edge(
                corners[i-1], tex_coord[i-1], corners[i], tex_coord[i]
            ) ;
        }

        parameterize_edge(
            corners[corners.size() -1], tex_coord[corners.size() -1],
            corners[0], tex_coord[0]
        ) ;

    }

    void MapParameterizer::parameterize_edge(
        Map::Halfedge* h1, const Point2d& uv1, Map::Halfedge* h2, const Point2d& uv2
    ) {

        double l = edge_length(h1, h2) ;
        if(l == 0) {
            h1->set_tex_coord(Geom::barycenter(uv1, uv2)) ;
        } else {
            Map::Halfedge* h = h1 ;
            double s = 0.0 ;
            h->set_tex_coord(uv1) ;
            do {
                h = h->next() ;
                s += Geom::edge_length(h) ;
                double w2 = s / l ;
                double w1 = 1.0 - w2 ;
                h->tex_coord().set_x(w1 * uv1.x() + w2 * uv2.x()) ;
                h->tex_coord().set_y(w1 * uv1.y() + w2 * uv2.y()) ;
            } while(h != h2) ;
        }
    }
    
    double MapParameterizer::edge_length(Map::Halfedge* h1, Map::Halfedge* h2) {
        double result = 0.0 ;
        if(h1 == h2) { return result ; }
        Map::Halfedge* h = h1 ;
        do {
            h = h->next() ;
            result += Geom::edge_length(h) ;
        } while(h != h2) ;
        return result ;
    }


//____________________________________________________________________________
    
}
