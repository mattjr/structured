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

#include <OGF/cells/map_algos/iso_curves_extractor.h>
#include <OGF/cells/map/geometry.h>
#include <algorithm>

namespace OGF {


    GraphInMap::GraphInMap(Map* map, Graph* graph) {
        map_ = map ;
        graph_ = graph ;
        vertices_in_edge_.bind(map_) ;
        vertex_in_vertex_.bind(map_) ;
        embedding_.bind(graph, "embedding") ;
        graph_editor_.set_target(graph) ;

        FOR_EACH_VERTEX(Graph, graph, it) {
            const MapCellEmbedding& em = embedding_[it] ;
            ogf_assert(em.map() == map_) ;
            switch(em.dimension()) {
            case 0: {
                vertex_in_vertex_[em.vertex()] = it ;
            } break ;
            case 1: {
                vertices_in_edge_[em.halfedge()->edge_key()].push_back(it) ;
            } break ;
            case 2: {
            } break ;
            default:
                ogf_assert_not_reached ;
            }
        }

    }

    static Graph::Halfedge* find_halfedge_between(Graph::Vertex* v1, Graph::Vertex* v2) {
        Graph::Halfedge* h = v1->halfedge() ;
        do {
            if(h->opposite()->vertex() == v2) {
                return h ;
            }
            h = h->next_around_vertex() ;
        } while(h != v1->halfedge()) ;
        return nil ;
    }

    static Map::Halfedge* find_halfedge_between(Map::Vertex* v1, Map::Vertex* v2) {
        Map::Halfedge* h = v1->halfedge() ;
        do {
            if(h->opposite()->vertex() == v2) {
                return h ;
            }
            h = h->next_around_vertex() ;
        } while(h != v1->halfedge()) ;
        return nil ;
    }

    static bool vertices_are_connected(Graph::Vertex* v1, Graph::Vertex* v2) {
        if(v1->halfedge() == nil) {
            return false ;
        }
        if(v2->halfedge() == nil) {
            return false ;
        }
        return (find_halfedge_between(v1,v2) != nil) ;
    }

    void GraphInMap::connect_vertices(
        Graph::Vertex* v1, Graph::Vertex* v2
    ) {
        if(v1 != v2 && !vertices_are_connected(v1, v2)) {
            graph_editor_.connect_vertices(v1, v2) ;
        }
    }

    Graph::Vertex* GraphInMap::find_or_create(Map::Vertex* v) {
        if(vertex_in_vertex_[v] != nil) {
            return vertex_in_vertex_[v] ;
        }
        Graph::Vertex* gv = graph_editor_.new_vertex(v->point()) ;
        embedding_[gv] = MapCellEmbedding(map_, v) ;
        vertex_in_vertex_[v] = gv ;
        return gv ;
    }

    Graph::Vertex* GraphInMap::find_or_create(
        Map::Halfedge* h, const Point3d& p
    ) {

        if(!h->is_edge_key()) {
            h = h->opposite() ;
        }

        double L = Geom::edge_length(h) ;
        double threshold = L * 1e-3 ;
        
        double d1 = (p - h->vertex()->point()).norm() ; 
        if(d1 < threshold) {
            return find_or_create(h->vertex()) ;
        }

        double d2 = (p - h->opposite()->vertex()->point()).norm() ; 
        if(d2 < threshold) {
            return find_or_create(h->opposite()->vertex()) ;
        }

        Graph::Vertex* nearest = nil ;
        double nearest_dist = 1e30 ;

        std::vector<Graph::Vertex*>& v = vertices_in_edge_[h] ;        
        for(unsigned int i=0; i<v.size(); i++) {
            double cur_dist = (p - v[i]->point()).norm() ;
            if(cur_dist < nearest_dist) {
                nearest_dist = cur_dist ;
                nearest = v[i] ;
            }
        }

        if(nearest_dist < threshold) {
            return nearest ;
        }
        
        Graph::Vertex* result = graph_editor_.new_vertex(p) ;

        embedding_[result] = MapCellEmbedding(map_, h) ;
        vertices_in_edge_[h].push_back(result) ;

        return result ;
    }

    void GraphInMap::extract_border() {
        FOR_EACH_HALFEDGE(Map, map_, it) {
            if(it->is_border()) {
                Map::Vertex* v = it->vertex() ;
                find_or_create(v) ;
            }
        }
        FOR_EACH_HALFEDGE(Map, map_, it) {
            if(it->is_border()) {
                Graph::Vertex* gv1 = vertex_in_vertex_[it->prev()->vertex()] ;
                Graph::Vertex* gv2 = vertex_in_vertex_[it->vertex()] ;
                connect_vertices(gv1, gv2) ;
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------------------------

    MapIsoCurvesExtractor::MapIsoCurvesExtractor(
        Map* map, Graph* graph
    ) : GraphInMap(map, graph) {
        current_facet_ = nil ;
        iso_value_ = 0.0 ;
    }

    Graph::Vertex* MapIsoCurvesExtractor::intersect(
        Map::Halfedge* h, double u1, double u2, double u
    ) {
        if(
            u >= u1 && u <= u2 ||
            u >= u2 && u <= u1
        ) {
            double t = 0.0 ;
            if(::fabs(u2 - u1) > 1e-10) {
                t = (u - u1) / (u2 - u1) ;
            }
            Point3d P = h->prev()->vertex()->point() + t * Geom::vector(h) ;
            return find_or_create(h,P) ;
        }
        return nil ;
    }
    
    void MapIsoCurvesExtractor::create_edges_in_facet(
        Map::Facet* f, const std::vector<Graph::Vertex*>& vertices
    ) {
        if(vertices.size() == 2) {
            connect_vertices(vertices[0], vertices[1]) ;
        } 
        if(vertices.size() > 2) {
              Logger::warn("MapIsoCurvesExtractor") 
              << vertices.size() 
              << " intersections in a facet" 
              << std::endl ;
            for(unsigned int i=0; i<vertices.size(); i++) {
                unsigned int j = ((i+1)%vertices.size()) ; 
                connect_vertices(vertices[i], vertices[j]) ;
            }
        }
    }
    

    void MapIsoCurvesExtractor::begin_facet(Map::Facet* f) {
        current_facet_ = f ;
        map_vertices_.clear() ;
        values_.clear() ;
    }
    
    void MapIsoCurvesExtractor::vertex(Map::Vertex* v, double u) {
        ogf_assert(current_facet_ != nil) ;
        map_vertices_.push_back(v) ;
        values_.push_back(u) ;
    }

    void MapIsoCurvesExtractor::end_facet() {
        std::vector<Graph::Vertex*> isects ;
        ogf_assert(map_vertices_.size() == values_.size()) ;
        unsigned int n = map_vertices_.size() ;
        for(unsigned int i=0; i<n; i++) {
            unsigned int j = (i+1)%n ;
            Map::Halfedge* h = find_halfedge_between(map_vertices_[j], map_vertices_[i]) ;
            ogf_assert(h != nil) ;
            double u1 = values_[i] ;
            double u2 = values_[j] ;
            Graph::Vertex* gv = intersect(h, u1, u2, iso_value_) ;
            if(gv != nil && std::find(isects.begin(), isects.end(),gv) == isects.end()) {
                isects.push_back(gv) ;
            }
            create_edges_in_facet(current_facet_, isects) ;
        }
        current_facet_ = nil ;
    } 

}

