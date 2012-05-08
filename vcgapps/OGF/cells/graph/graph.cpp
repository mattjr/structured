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
 

#include <OGF/cells/graph/graph.h>
#include <OGF/basic/attributes/attribute.h>
#include <algorithm>

#include <stack>

namespace OGF {

//_________________________________________________________
    

    GraphCombelObserver<Graph::Vertex>::GraphCombelObserver(Graph* m) : graph_(m) {
        graph_->add_vertex_observer(this);
    }
    
    GraphCombelObserver<Graph::Vertex>::~GraphCombelObserver() {
        graph_->remove_vertex_observer(this);
    }


    GraphCombelObserver<Graph::Halfedge>::GraphCombelObserver(Graph* m) : graph_(m) {
        graph_->add_halfedge_observer(this);
    }
    
    GraphCombelObserver<Graph::Halfedge>::~GraphCombelObserver() {
        graph_->remove_halfedge_observer(this);
    }

//_________________________________________________________

    Graph::Graph() { 
    }
    
    Graph::~Graph() {
        clear() ;
    } 

    // ____________________ Observers ________________________

    void Graph::add_vertex_observer(GraphCombelObserver<Vertex>* obs) {
        vertex_observers_.push_back(obs) ;
    }
    
    void Graph::remove_vertex_observer(GraphCombelObserver<Vertex>* obs) {
        std::vector<GraphCombelObserver<Vertex>* >::iterator it = std::find(
            vertex_observers_.begin(), vertex_observers_.end(), obs
        ) ;
        ogf_assert(it != vertex_observers_.end()) ;
        vertex_observers_.erase(it) ;
    }
    
    void Graph::add_halfedge_observer(GraphCombelObserver<Halfedge>* obs) {
        halfedge_observers_.push_back(obs)  ;
    }
    
    void Graph::remove_halfedge_observer(GraphCombelObserver<Halfedge>* obs) {
        std::vector<GraphCombelObserver<Halfedge>* >::iterator it = std::find(
            halfedge_observers_.begin(), halfedge_observers_.end(), obs
        ) ;
        ogf_assert(it != halfedge_observers_.end()) ;
        halfedge_observers_.erase(it) ;
    }
    
    void Graph::notify_add_vertex(Vertex* v) {
        for(
            std::vector<GraphCombelObserver<Vertex>* >::iterator
                it=vertex_observers_.begin(); it!=vertex_observers_.end(); it++
        ) {
            (*it)->add(v) ;
        }
    }
    
    void Graph::notify_remove_vertex(Vertex* v) {
        for(
            std::vector<GraphCombelObserver<Vertex>* >::iterator
                it=vertex_observers_.begin(); it!=vertex_observers_.end(); it++
        ) {
            (*it)->remove(v) ;
        }
    }
    
    void Graph::notify_add_halfedge(Halfedge* h) {
        for(
            std::vector<GraphCombelObserver<Halfedge>* >::iterator
                it=halfedge_observers_.begin(); 
            it!=halfedge_observers_.end(); it++
        ) {
            (*it)->add(h) ;
        }
    }
    
    void Graph::notify_remove_halfedge(Halfedge* h) {
        for(
            std::vector<GraphCombelObserver<Halfedge>* >::iterator
                it=halfedge_observers_.begin(); 
            it!=halfedge_observers_.end(); it++
        ) {
            (*it)->remove(h) ;
        }
    }
    
    // ____________________ Modification _____________________

    void Graph::clear() {
        vertices_.clear() ;
        halfedges_.clear() ;

        vertex_attribute_manager_.clear() ;
        halfedge_attribute_manager_.clear() ;
    }

    void Graph::erase_all() {
        clear() ;
    }

    void Graph::clear_inactive_items() {
        // TODO: traverse the inactive items list, 
        //  and remove the attributes ...
        vertices_.clear_inactive_items() ;
        halfedges_.clear_inactive_items() ;
    }

    // _____________________ Low level ______________________

    
    Graph::Halfedge* Graph::new_edge() {
        Halfedge* h1 = new_halfedge() ;
        Halfedge* h2 = new_halfedge() ;
        h1-> set_opposite(h2) ;
        h2-> set_opposite(h1) ;
        return h1 ;
    }

    void Graph::delete_edge(Halfedge* h) {
        delete_halfedge(h-> opposite()) ;
        delete_halfedge(h) ;
    }

    Graph::Vertex* Graph::new_vertex() {
        Vertex* result = vertices_.create() ;
        vertex_attribute_manager_.new_record(result) ;
        notify_add_vertex(result) ;
        return result ;
    }
    
    Graph::Vertex* Graph::new_vertex(const Graph::Vertex* rhs) {
        Vertex* result = vertices_.create() ;
        result->set_point(rhs->point()) ;
        vertex_attribute_manager_.new_record(result, rhs) ;
        notify_add_vertex(result) ;
        return result ;
    }

    Graph::Halfedge* Graph::new_halfedge() {
        Halfedge* result = halfedges_.create() ;
        halfedge_attribute_manager_.new_record(result) ;
        notify_add_halfedge(result) ;
        return result ;
    }

    Graph::Halfedge* Graph::new_halfedge(const Graph::Halfedge* rhs) {
        Halfedge* result = halfedges_.create() ;
        halfedge_attribute_manager_.new_record(result, rhs) ;
        notify_add_halfedge(result) ;
        return result ;
    }
    
    void Graph::delete_vertex(Vertex* v) {
        notify_remove_vertex(v) ;
        vertex_attribute_manager_.delete_record(v) ;
        vertices_.destroy(v) ;
    }
    
    void Graph::delete_halfedge(Halfedge* h) {
        notify_remove_halfedge(h) ;
        halfedge_attribute_manager_.delete_record(h) ;
        halfedges_.destroy(h) ;
    }
    
    void Graph::activate_vertex(Vertex* v) {
        notify_add_vertex(v) ;
        vertices_.activate(v) ;
    }
    
    void Graph::activate_halfedge(Halfedge* h) {
        notify_add_halfedge(h) ;
        halfedges_.activate(h) ;
    }
    
    void Graph::deactivate_vertex(Vertex* v) {
        notify_remove_vertex(v) ;
        vertices_.deactivate(v) ;
    }
    
    void Graph::deactivate_halfedge(Halfedge* h) {
        notify_remove_halfedge(h) ;
        halfedges_.deactivate(h) ;
    }
    
    void Graph::get_connected_component(
        Vertex* h, std::vector<Vertex*>& l
    ) {

        Attribute<Vertex, bool> visited(vertex_attribute_manager()) ;

        std::stack<Vertex*> stack ;
        stack.push(h) ;

        while(!stack.empty()) {
            Vertex* top = stack.top() ;
            stack.pop() ;
            if(!visited[top]) {
                visited[top] = true ;
                l.push_back(top) ;
                Halfedge* cir = top->halfedge() ;
                do {
                    Vertex* cur = cir-> opposite()-> vertex() ;
                    if(!visited[cur]) {
                        stack.push(cur) ;
                    }
                    cir = cir->next_around_vertex() ;
                } while(cir != top->halfedge()) ;
            }
        }
    }

    bool Graph::is_valid() const {
        bool ok = true ;
        { FOR_EACH_VERTEX_CONST(Graph, this, it) {
            ok = ok && it-> is_valid() ;
        }}
        { FOR_EACH_HALFEDGE_CONST(Graph, this, it) {
            ok = ok && it-> is_valid() ;
        }}
        return ok ;
    }


    void Graph::assert_is_valid() const {
        { FOR_EACH_VERTEX_CONST(Graph, this, it) {
            it-> assert_is_valid() ;
        }}
        { FOR_EACH_HALFEDGE_CONST(Graph, this, it) {
            it-> assert_is_valid() ;
        }}
    }


//_________________________________________________________

    GraphMutator::~GraphMutator() { target_ = nil ;  }

    void GraphMutator::set_target(Graph* m) { target_ = m ; }

    Graph::Halfedge* GraphMutator::new_edge(Graph::Halfedge* rhs) {
        Halfedge* h1 = new_halfedge(rhs) ;
        Halfedge* h2 = new_halfedge(rhs->opposite()) ;
        h1-> set_opposite(h2) ;
        h2-> set_opposite(h1) ;
        h1-> set_next_around_vertex(h2) ;
        h2-> set_next_around_vertex(h1) ;
        h1-> set_prev_around_vertex(h2) ;
        h2-> set_prev_around_vertex(h1) ;
        return h1 ;
    }
    

//_________________________________________________________

}

