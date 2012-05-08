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

#include <OGF/cells/graph/graph_editor.h>

namespace OGF {
    
    GraphEditor::GraphEditor(Graph* target) : GraphMutator(target) {
    } 
    
    void GraphEditor::set_target(Graph* target) {
        GraphMutator::set_target(target) ;
    }
    
    Graph::Halfedge* GraphEditor::make_loop(int nb_vertices) {
        Vertex* last = nil ;
        Vertex* first = nil ;
        Halfedge* result = nil ;
        for(int i=0; i<nb_vertices; i++) {
            Vertex* cur = new_vertex() ;
            if(last != nil) {
                result = connect_vertices(last, cur) ;
            } else {
                first = cur ;
            }
            last = cur ;
        }
        connect_vertices(last, first) ;
        return result ;
    }
    
    Graph::Halfedge* GraphEditor::make_polyline(int nb_vertices) {
        Vertex* last = nil ;
        Halfedge* result = nil ;
        for(int i=0; i<nb_vertices; i++) {
            Vertex* cur = new_vertex() ;
            if(last != nil) {
                result = connect_vertices(last, cur) ;
            }
            last = cur ;
        }
        return result ;
    }
    
    static Graph::Halfedge* find_halfedge_between(Graph::Vertex* v1, Graph::Vertex* v2) {
        if(v1->halfedge() == nil || v2->halfedge() == nil) {
            return nil ;
        }
        Graph::Halfedge* h = v2->halfedge() ;
        do {
            if(h->opposite()->vertex() == v1) {
                return h ;
            }
            h = h->next_around_vertex() ;
        } while(h != v2->halfedge()) ;
        return nil ;
    }

    Graph::Halfedge* GraphEditor::connect_vertices(
        Vertex* v1, Vertex* v2, Halfedge* copy_attributes_from
    ) {
	    ogf_assert(v1 != v2) ;
        ogf_parano_assert(find_halfedge_between(v1,v2) == nil) ;
        Halfedge* result = nil ;
        if(copy_attributes_from == nil) {
            result = new_edge() ;
        } else {
            result = new_edge(copy_attributes_from) ;
        }
        set_halfedge_vertex(result, v1) ;
        set_halfedge_vertex(result->opposite(), v2) ;
        insert_halfedge_in_ciel(result) ;
        insert_halfedge_in_ciel(result->opposite()) ;
        return result ;
    }
    
/*
    Graph::Halfedge* GraphEditor::connect_vertices(
        Vertex* v1, Vertex* v2, Halfedge* copy_attributes_from
    ) {
        ogf_assert(v1 != v2) ;
        ogf_parano_assert(find_halfedge_between(v1,v2) == nil) ;
        Halfedge* h1 = new_halfedge(copy_attributes_from) ;
        Halfedge* h2 = new_halfedge(copy_attributes_from) ;
        set_halfedge_opposite(h1,h2) ;
        set_halfedge_opposite(h2,h1) ;
        set_halfedge_vertex(h1, v1) ;
        set_halfedge_vertex(h2, v2) ;
        insert_halfedge_in_ciel(h1) ;
        insert_halfedge_in_ciel(h2) ;
        return h2 ;
    }
*/

    Graph::Halfedge* GraphEditor::create_edge_and_vertex_from_vertex(
        Vertex* v1
    ) {
        Vertex* v2 = new_vertex() ;
        Halfedge* result = connect_vertices(v1, v2) ;
        return result ;
    }
    
    Graph::Vertex* GraphEditor::split_edge(Halfedge* h) {
        Vertex* result = new_vertex() ;
        Vertex* v1 = h->vertex() ;
        Vertex* v2 = h->opposite()->vertex() ;
        // Copy attributes from h ---,
        connect_vertices(v1, result, h) ;
        connect_vertices(result, v2, h) ;
        erase_edge(h) ;
        return result ;
    }
    
    bool GraphEditor::erase_center_vertex(Vertex* v) {
        if(v->degree() != 2) {
            return false ;
        }

        Halfedge* h1 = v->halfedge() ;
        Halfedge* h2 = h1->next_around_vertex() ;

        Vertex* v1 = h1->opposite()->vertex() ;
        Vertex* v2 = h2->opposite()->vertex() ;

        connect_vertices(v1,v2) ;
        erase_edge(h1) ;
        erase_edge(h2) ;

        return true ;
    }

    void GraphEditor::erase_vertex(Vertex* v) {
        if(v->halfedge() != nil) {
            std::vector<Graph::Halfedge*> to_delete ;
            Graph::Halfedge* h = v->halfedge() ;
            do {
                Vertex* neigh = h->opposite()->vertex() ;
                remove_halfedge_from_ciel(h->opposite()) ;
                if(neigh->halfedge() == nil) {
                    delete_vertex(neigh) ;
                }
                to_delete.push_back(h) ;
                h = h->next_around_vertex() ;
            } while(h != v->halfedge()) ;
            for(unsigned int i=0; i<to_delete.size(); i++) {
                delete_edge(to_delete[i]) ;
            }
        }
        delete_vertex(v) ;
    }

    Graph::Vertex* GraphEditor::merge_vertices(Graph::Vertex* v1, Graph::Vertex* v2) {
        // Note: it would be more elegant to reuse the existing edges,
        // for the moment, we re-create them and we delete them ...
        ogf_assert(v1 != v2) ;
        Graph::Halfedge* h = v2->halfedge() ;
        do {
            Graph::Vertex* neigh = h->opposite()->vertex() ;
            if(neigh != v1 && find_halfedge_between(v1,neigh) == nil) {
                connect_vertices(v1, neigh, h) ;
            }
            h = h->next_around_vertex() ;
        } while(h != v2->halfedge()) ;
        erase_vertex(v2) ;
        return v1 ;
    }

    void GraphEditor::erase_edge(Halfedge* h, bool erase_vertices) {
        Vertex* v1 = h->vertex() ;
        Vertex* v2 = h->opposite()->vertex() ;
        remove_halfedge_from_ciel(h) ;
        remove_halfedge_from_ciel(h->opposite()) ;
        if(v1->halfedge() == nil && erase_vertices) {
            delete_vertex(v1) ;
        }
        if(v2->halfedge() == nil && erase_vertices) {
            delete_vertex(v2) ;
        }
        delete_edge(h) ;
    }
    
    void GraphEditor::make_hole(Halfedge* h) {
        erase_edge(h) ;
    }

    bool GraphEditor::halfedge_exists_between_vertices(
        Vertex* v1, Vertex* v2
    ) {
        Halfedge* cir = v1->halfedge() ;
        do {
            if(cir-> opposite()-> vertex() == v2) {
                return true ;
            }
            cir = cir->next_around_vertex() ;
        } while(cir != v1->halfedge()) ;
        return false ;
    }


    void GraphEditor::insert_halfedge_in_ciel(Halfedge* h) {
        Vertex* v = h->vertex() ;
        if(v->halfedge() == nil) {
            set_vertex_halfedge(v, h) ;
            set_halfedge_next_around_vertex(h,h) ;
            set_halfedge_prev_around_vertex(h,h) ;
        } else {
            Halfedge* prev = v->halfedge() ;
            Halfedge* next = prev->next_around_vertex() ;
            set_halfedge_next_around_vertex(prev, h) ;
            set_halfedge_prev_around_vertex(h, prev) ;
            set_halfedge_next_around_vertex(h, next) ;
            set_halfedge_prev_around_vertex(next, h) ;
        }
    }
    
    void GraphEditor::remove_halfedge_from_ciel(Halfedge* h) {
        Vertex* v = h->vertex() ;
        if(v->degree() == 1) {
            ogf_assert(v->halfedge() == h) ;
            set_vertex_halfedge(v, nil) ;
        } else {
            Halfedge* prev = h->prev_around_vertex() ;
            Halfedge* next = h->next_around_vertex() ;
            set_halfedge_next_around_vertex(prev, next) ;
            set_halfedge_prev_around_vertex(next, prev) ;
            set_vertex_halfedge(v, prev) ;
        }
    }


    void GraphEditor::copy_attributes(Vertex* to, Vertex* from) {
        target()->vertex_attribute_manager()->copy_record(to, from) ;
    }
    
    void GraphEditor::copy_attributes(Halfedge* to, Halfedge* from) {
        target()->halfedge_attribute_manager()->copy_record(to, from) ;
    }
    
}
