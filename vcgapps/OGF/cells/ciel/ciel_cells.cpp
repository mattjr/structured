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
 

#include <OGF/cells/ciel/ciel_cells.h>
#include <stack>

namespace OGF {

    namespace CielTypes {

//_________________________________________________________

        void Vertex::insert_halfedge_in_ciel(Halfedge* h) {
            if(halfedge_ == nil) {
                halfedge_ = h ;
                h->set_next_around_vertex(h) ;
            } else {
                Halfedge* first = halfedge_ ;
                Halfedge* second = first->next_around_vertex() ;
                first->set_next_around_vertex(h) ;
                h->set_next_around_vertex(second) ;
            }
        }

        
        void Vertex::get_star(std::set<Cell*>& cells) const {
            Halfedge* cur_edge = halfedge() ;
            do {
                Halfedge* cur_in_edge = cur_edge ;
                do {
                    cells.insert(cur_in_edge->cell()) ;
                    cur_in_edge = cur_in_edge->opposite_facet() ;
                    cur_in_edge = cur_in_edge->opposite_cell() ;
                } while(cur_in_edge != cur_edge) ;
                cur_edge = cur_edge-> next_around_vertex() ;
            } while(cur_edge != halfedge())  ;
        }

        bool Vertex::is_on_border() const {
            Halfedge* h = halfedge() ;
            do {
                if(h->cell() == nil) {
                    return true ;
                }
                h = h->next_around_vertex() ;
            } while(h != halfedge()) ;
            return false ;
        }

        void Cell::get_vertices(std::set<Vertex*>& vertices) const {
            std::stack<Halfedge*> stack ;
            stack.push(halfedge()) ;
            vertices.insert(halfedge()->vertex()) ;
            while(!stack.empty()) {
                Halfedge* cur = stack.top() ;
                stack.pop() ;
                Halfedge* succ1 = cur->next_around_facet() ;
                Halfedge* succ2 = cur->opposite_facet() ;
                
                if(vertices.find(succ1->vertex()) == vertices.end()) {
                    vertices.insert(succ1->vertex()) ;
                    stack.push(succ1) ;
                }
                
                if(vertices.find(succ2->vertex()) == vertices.end()) {
                    vertices.insert(succ2->vertex()) ;
                    stack.push(succ2) ;
                }
            }
        }
    
//_________________________________________________________

    }
}

