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
 

#include <OGF/parameterizer/algos/graph_cleaner.h>
#include <OGF/parameterizer/algos/map_trimmer.h>
#include <OGF/cells/graph/graph_editor.h>
#include <OGF/cells/map/map_embedding.h>
#include <OGF/basic/debug/progress.h>

namespace OGF {

    bool operator==(const MapCellEmbedding& em1, const MapCellEmbedding& em2) {
        if(em1.dimension() != em2.dimension()) {
            return false ;
        }
        switch(em1.dimension()) {
        case 0: {
            return (em1.vertex() == em2.vertex()) ;
        } break ;
        case 1: {
            return (
                (em1.halfedge() == em2.halfedge()) || 
                (em1.halfedge() == em2.halfedge()->opposite())
            ) ;
        } break ;
        case 2: {
            return (em1.facet() == em2.facet()) ;
        } break ;
        default: {
            ogf_assert(false) ;
        } break ;
        }
        return false ;
    }

    bool operator!=(const MapCellEmbedding& em1, const MapCellEmbedding& em2) {
        return !(em1 == em2) ;
    }

    bool GraphCleanerArc::operator==(const GraphCleanerArc& rhs) const {
        ogf_assert(!is_deleted()) ;
        ogf_assert(!rhs.is_deleted()) ;
        GraphVertexAttribute<MapCellEmbedding> embedding(graph_, "embedding") ;
        if(vertices.size() != rhs.vertices.size()) {
            return false ;
        }
        bool ok = true ;
        unsigned int n = vertices.size() ;
        {for(unsigned int i=0; i<n; i++) {
            if(embedding[vertices[i]] != embedding[rhs.vertices[i]]) {
                ok = false ;
                break ;
            }
        }}
        if(ok) {
            return true ;
        }
        ok = true ;
        {for(unsigned int i=0; i<n; i++) {
            if(embedding[vertices[i]] != embedding[rhs.vertices[n - 1 - i]]) {
                ok = false ;
                break ;
            }
        }}
        return ok ;
    }

    static Graph::Halfedge* next(Graph::Halfedge* h) {
        ogf_assert(h->vertex()->degree() == 2) ;
        return h->next_around_vertex()->opposite() ;
    }

    GraphCleanerArc::GraphCleanerArc(Graph* graph, Graph::Halfedge* start) {
        graph_ = graph ;
        closed_ = false ;
        visited_ = false ;
        deleted_ = false ;
        Graph::Halfedge* h = start ;
        while(h->vertex()->degree() == 2) {
            h = next(h) ;
            if(h == start) {
                closed_ = true ;
                break ;
            }
        }
        if(closed_) {
            h = start ;
            do {
                halfedges.push_back(h) ;
                vertices.push_back(h->opposite()->vertex()) ;
                h = next(h) ;
            } while(h != start) ;
        } else {
            h = h->opposite() ;
            vertices.push_back(h->opposite()->vertex()) ;
            while(h->vertex()->degree() == 2) {
                halfedges.push_back(h) ;
                vertices.push_back(h->vertex()) ;
                h = next(h) ;
            } 
            halfedges.push_back(h) ;
            vertices.push_back(h->vertex()) ;
        }
    }
    
    double GraphCleanerArc::length() const {
        ogf_assert(!deleted_) ;
        double result = 0 ;
        for(unsigned int i=0; i<halfedges.size(); i++) {
            Graph::Halfedge* h = halfedges[i] ;
            result += (h->vertex()->point() - h->opposite()->vertex()->point()).norm() ;
        }
        return result ;
    }
    
    Graph::Vertex* GraphCleanerArc::other_extremity(Graph::Vertex* v) {
        ogf_assert(!closed_) ;
        ogf_assert(!deleted_) ;
        if(v == extremity_1()) {
            return extremity_2() ;
        }
        if(v == extremity_2()) {
            return extremity_1() ;
        }
        ogf_assert(false) ;
        return nil ;
    }

//_____________________________________________________________________________________________

    GraphCleaner::GraphCleaner(Graph* graph) : graph_(graph), is_marked_(graph) {
        if(embedding_.is_defined(graph, "embedding")) {
            embedding_.bind(graph, "embedding") ;
        }
        FOR_EACH_HALFEDGE(Graph, graph_, it) {
            if(!is_marked_[it]) {
                arcs_.push_back(GraphCleanerArc(graph_,it)) ;
                mark_arc(*(arcs_.rbegin())) ;
            }
        }
        for(unsigned int i=0; i<arcs_.size(); i++) {
            GraphCleanerArc& it = arcs_[i] ;
            if(!it.is_closed()) {
                extremity_to_arc_[it.extremity_1()].push_back(&it) ;
                extremity_to_arc_[it.extremity_2()].push_back(&it) ;
            }
        }
    }


    void GraphCleaner::mark_arc(GraphCleanerArc& arc) {
        ogf_assert(!arc.is_deleted()) ;
        for(unsigned int i=0; i<arc.halfedges.size(); i++) {
            is_marked_[arc.halfedges[i]] = true ;
            is_marked_[arc.halfedges[i]->opposite()] = true ;
        }
    }
    
    void GraphCleaner::delete_arc(GraphCleanerArc& arc) {

        if(arc.is_deleted()) {
            std::cerr << "deleting same arc twice !!" << std::endl ;
            return ;
        }

//        ogf_assert(!arc.is_deleted()) ;
        arc.set_deleted(true) ;
        GraphEditor editor(graph_) ;
        unsigned int i ;
        for(i=0; i<arc.halfedges.size(); i++) {
            editor.erase_edge(arc.halfedges[i]) ;
        }
    }

    void GraphCleaner::find_arcs_between(
        Graph::Vertex* v1, Graph::Vertex* v2, 
        std::vector<GraphCleanerArc*>& arcs
    ) {
        std::vector<GraphCleanerArc*>& star = extremity_to_arc_[v1] ;
        for(unsigned int i=0; i<star.size(); i++) {
            if(!star[i]->is_deleted() && star[i]->other_extremity(v1) == v2) {
                arcs.push_back(star[i]) ;
                star[i]->set_visited() ;
            }
        }
    }


    bool GraphCleaner::delete_closed_loops() {
        int count = 0 ;
        for(unsigned int i=0; i<arcs_.size(); i++) {
            if(!arcs_[i].is_deleted() && arcs_[i].is_closed()) {
                count++ ;
                delete_arc(arcs_[i]) ;
            }
        }
        std::cerr << "deleted " << count << " arcs (closed loops)" << std::endl ;
        return (count != 0) ;
    }
    

    bool GraphCleaner::delete_dangling_arcs() {
        int count = 0 ;

        for(unsigned int i=0; i<arcs_.size(); i++) {
            GraphCleanerArc& arc = arcs_[i] ;
            if(
                !arc.is_deleted() && !arc.is_closed() && 
                (
                    arc.extremity_1()->degree() == 1 ||
                    arc.extremity_2()->degree() == 1 
                )
            ) {
                delete_arc(arc) ;
                count++ ;
            }
        }

        std::cerr << "deleted " << count << " arcs (dangling)" << std::endl ;
        return (count != 0) ;
    }

    bool GraphCleaner::delete_duplicated_arcs() {

        int count = 0 ;


        {for(unsigned int i=0; i<arcs_.size(); i++) {
            GraphCleanerArc& arc = arcs_[i] ;
            if(!arc.is_deleted() && !arc.is_closed() && !arc.is_visited()) {
                std::vector<GraphCleanerArc*> same_arcs ;
                find_arcs_between(arc.extremity_1(), arc.extremity_2(), same_arcs) ;
                if(same_arcs.size() == 1) {
                    continue ;
                }
                {
                    for(unsigned int j=0; j<same_arcs.size(); j++) {
                        if((*(same_arcs[j]) == arc) && (same_arcs[j] != &arc)) {
                            std::cerr << " --- Duplicated arc found ---" << std::endl ;
                            delete_arc(*(same_arcs[j])) ;
                            count++ ;
                        }
                    }
                }
            }
        }}

        {for(unsigned int i=0; i<arcs_.size(); i++) {
            arcs_[i].set_visited(false) ;
        }}

        if(embedding_.is_bound()) {
            for(unsigned int i=0; i<arcs_.size(); i++) {
                GraphCleanerArc& arc = arcs_[i] ;
                if(!arc.is_deleted() && !arc.is_closed() && !arc.is_visited()) {
                    
                    // Compute the surface normals at the two extremities
                    //   (using the embedding).
                    Vector3d n1 = normal(arc.extremity_1()) ;
                    Vector3d n2 = normal(arc.extremity_2()) ;

                    // We have duplicated arcs only if the two normals 
                    //    point approximatively in the same direction.
                    // Otherwise, the configuration may correspond to 
                    //    two arcs located on both sides of a cylinder.
                    if(n1*n2 < 0.5 * ::sqrt(2.0)) {
                        continue ;
                    }

                    std::vector<GraphCleanerArc*> same_arcs ;
                    find_arcs_between(arc.extremity_1(), arc.extremity_2(), same_arcs) ;
                    if(same_arcs.size() == 1) {
                        continue ;
                    }

                    GraphCleanerArc* shortest = nil ;
                    double shortest_length = 1e30 ;
                    {
                        for(unsigned int j=0; j<same_arcs.size(); j++) {
                            double l = same_arcs[j]->length() ;
                            if(l < shortest_length) {
                                shortest_length = l ;
                                shortest = same_arcs[j] ;
                            }
                        }
                    }
                    {
                        for(unsigned int j=0; j<same_arcs.size(); j++) {
                            if(same_arcs[j] != shortest) {
                                delete_arc(*(same_arcs[j])) ;
                                count++ ;
                            }
                        }
                    }
                }
            }
        }
        std::cerr << "deleted " << count << " arcs (duplicated)" << std::endl ;
        return (count != 0) ;
    }

    void GraphCleaner::clean_locks() {
        GraphVertexLock is_locked(graph_) ;
        GraphHalfedgeAttribute<int> kind(graph_, "kind") ;
        FOR_EACH_HALFEDGE(Graph, graph_, it) {
            if(it->vertex()->degree() == 2 &&
                kind[it] == kind[it->next_around_vertex()->opposite()]
            ) {
                is_locked[it->vertex()] = false ;
            }
        }
    }

    Vector3d GraphCleaner::normal(Graph::Vertex* v) {
        ogf_assert(embedding_.is_bound()) ;
        return MapTrimmer::normal(embedding_[v]) ;
    }

}
