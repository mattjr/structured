/*
 *  OGF/Graphite: Geometry and Graphics Programming Library + Utilities
 *  Copyright (C) 2000-2005 INRIA - Project ALICE
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
 *  Contact: Bruno Levy - levy@loria.fr
 *
 *     Project ALICE
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs. 
 *
 * As an exception to the GPL, Graphite can be linked with the following (non-GPL) libraries:
 *     Qt, SuperLU, WildMagic and CGAL
 */
 

#include <OGF/cells/map_algos/map_adaptive_splitter.h>

namespace OGF {
    
    MapAdaptiveSplitter::MapAdaptiveSplitter(Map* map) : MapMutator(map) { 
    }
    
    MapAdaptiveSplitter::~MapAdaptiveSplitter() { 
    }        

    void MapAdaptiveSplitter::apply() {
        is_new_vertex_.bind(target()) ;
        editor_.set_target(target()) ;
        split_edges() ;
        split_facets() ;
        is_new_vertex_.unbind() ;
    }

    
    void MapAdaptiveSplitter::split_edges() {
        std::vector<Map::Halfedge*> edges ;
        FOR_EACH_EDGE(Map, target(), it) {
            if(needs_to_split(it)) {
                edges.push_back(it) ;
            }
        }
        for(std::vector<Halfedge*>::iterator it = edges.begin() ;  it != edges.end(); it++) {
            Vertex* v = editor_.split_edge(*it) ;
            is_new_vertex_[v] = true ;
        }
    }

    int MapAdaptiveSplitter::nb_new_vertices(Map::Facet* f) {
        int result = 0 ;
        Map::Halfedge* h = f->halfedge() ;
        do {
            if(is_new_vertex_[h->vertex()]) {
                result++ ;
            }
            h = h->next() ;
        } while(h != f->halfedge()) ;
        return result ;
    }
    
    void MapAdaptiveSplitter::find_next_new_vertex(Map::Halfedge*& h) {
        do { 
            h = h->next() ;
        } while(!is_new_vertex_[h->vertex()]) ;
    }
    
    void MapAdaptiveSplitter::split_facet_1(Map::Facet* f) {
        Map::Halfedge* h1 = f->halfedge() ;
        find_next_new_vertex(h1) ;
        editor_.split_facet(h1, h1->next()->next()) ;
    }
        
    void MapAdaptiveSplitter::split_facet_2(Map::Facet* f) {
        Map::Halfedge* h1 = f->halfedge() ;
        find_next_new_vertex(h1) ;
        Map::Halfedge* h2 = h1 ;
        find_next_new_vertex(h2) ;
        editor_.split_facet(h1, h2) ;
        
        editor_.triangulate_facet(h1) ;
        editor_.triangulate_facet(h2) ;
    }

    void MapAdaptiveSplitter::split_facet_n(Map::Facet* f) {
        
        Halfedge* start = f-> halfedge() ;
        do {
            start = start-> next() ;
        } while(
            is_new_vertex_[start-> next()-> vertex()] &&
            start != f-> halfedge() 
        ) ;
        
        ogf_assert(!is_new_vertex_[start-> next()-> vertex()]) ;
        
        Halfedge* cur1 = start ;
        Halfedge* cur2 = cur1-> next()-> next() ;
        Halfedge* next = cur2-> next()-> next() ;
        
        do {
            ogf_assert(!is_new_vertex_[cur1-> next()-> vertex()]) ;
            editor_.split_facet(cur1, cur2) ;
            cur1 = cur2-> next()-> opposite() ;
            cur2 = next ;
            next = next-> next()-> next() ;
        } while(cur1-> vertex() != start->vertex()) ;
    }
    
    void MapAdaptiveSplitter::split_facet(Map::Facet* f) {
        switch(nb_new_vertices(f)) {
        case 0:
            break ;
        case 1:
            split_facet_1(f) ;
            break ;
        case 2:
            split_facet_2(f) ;
            break ;
        case 3:
            split_facet_n(f) ;
            break ;
        default:
            std::cerr << "Not a triangle, this is strange" << std::endl ;
            split_facet_n(f) ;
            break ;
        }
    }

    void MapAdaptiveSplitter::split_facets() {
        std::vector<Map::Facet*> facets ;
        FOR_EACH_FACET(Map, target(), it) {
            if(nb_new_vertices(it) > 0) {
                facets.push_back(it) ;
            }
        }
        for(unsigned int i=0; i<facets.size(); i++) {
            split_facet(facets[i]) ;
        }
    }

    //_______________________________________________________________________________

    AttributeMapAdaptiveSplitter::AttributeMapAdaptiveSplitter(
        Map* map
    ) : MapAdaptiveSplitter(map) {
        threshold_ = 0.0 ;
        split_if_smaller_ = false ;
        vertex_attribute_ = nil ;
        halfedge_attribute_ = nil ;
        facet_attribute_ = nil ;
    }

    void AttributeMapAdaptiveSplitter::set_vertex_attribute(MapVertexAttribute<double>& attribute) {
        vertex_attribute_ = &attribute ;
        halfedge_attribute_ = nil ;
        facet_attribute_ = nil ;
    }
    
    void AttributeMapAdaptiveSplitter::set_halfedge_attribute(MapHalfedgeAttribute<double>& attribute) {
        vertex_attribute_ = nil ;
        halfedge_attribute_ = &attribute ;
        facet_attribute_ = nil ;
    }
    
    void AttributeMapAdaptiveSplitter::set_facet_attribute(MapFacetAttribute<double>& attribute) {
        vertex_attribute_ = nil ;
        halfedge_attribute_ = nil ;
        facet_attribute_ = &attribute ;
    }
    
    bool AttributeMapAdaptiveSplitter::needs_to_split(Map::Halfedge* h) {
        double v = 0.0 ;
        if(vertex_attribute_ != nil) {
            v = 0.5 * ((*vertex_attribute_)[h->vertex()] + (*vertex_attribute_)[h->opposite()->vertex()]) ;
        } else if(halfedge_attribute_ != nil) {
            v = 0.5 * ((*halfedge_attribute_)[h] + (*halfedge_attribute_)[h->opposite()]) ;
        } else if(facet_attribute_ != nil) {
            int nb = 0 ;
            if(h->facet() != nil) { v += (*facet_attribute_)[h->facet()] ; nb++ ; }
            if(h->opposite()->facet() != nil) { v += (*facet_attribute_)[h->opposite()->facet()] ; nb++ ; }
            v /= double(nb) ;
        }
        return split_if_smaller_ ? (v < threshold_) : (v > threshold_) ;
    }

    //_______________________________________________________________________________

}

