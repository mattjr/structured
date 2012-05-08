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
 

#include <OGF/cells/ciel/ciel_builder.h>
#include <OGF/basic/debug/logger.h>

#include <stack>
#include <algorithm>

#include <stdlib.h>



namespace OGF {

//_________________________________________________________

    CielBuilder::CielBuilder() : desc_state_(desc_initial) {
    }

    CielBuilder::CielBuilder(
        Ciel* ciel_in
    ) : CielMutator(ciel_in), desc_state_(desc_initial) {
    }
    
    void CielBuilder::begin_volume() {
        transition(desc_initial, desc_volume) ;
    }
    
    void CielBuilder::end_volume() {
        transition(desc_volume, desc_final) ;
        // sew_borders() ;
        // simplify_edge_adjacency_lists() ;
    }

    void CielBuilder::add_vertex( const Point3d& geometry ) {
        check_description_state(desc_volume) ;    
        Vertex* v = new_vertex() ;
        v->set_point(geometry) ;
        vertices_.push_back(v) ;
    }
    
    void CielBuilder::begin_cell() {
        transition(desc_volume, desc_cell) ;
        current_cell_ = new_cell() ;
    }
    
    void CielBuilder::end_cell() {
        transition(desc_cell, desc_volume) ;
        
        // Check that the current polyhedron is well formed
        for(
            std::vector<Halfedge*>::iterator it=current_cell_halfedges_.begin() ;
            it != current_cell_halfedges_.end() ; it++
        ) {
            Halfedge* cur = *it ;
            if(cur->opposite_facet() == nil) {
                std::cerr << "Open cell encountered " << std::endl ;
            }
        }
        current_cell_halfedges_.clear() ;
    }
    
    void CielBuilder::begin_facet() {
        transition(desc_cell, desc_facet) ;
        first_vertex_in_facet_ = nil ;
        first_vertex_in_facet_id_ = -1 ;    
        first_halfedge_in_facet_ = nil ;
        current_halfedge_in_facet_ = nil ;
        nb_vertices_in_facet_ = 0 ;
    }
    
    void CielBuilder::end_facet() {
        check_description_state(desc_facet) ;
        ogf_assert(nb_vertices_in_facet_ >= 3) ;

        // Close the polygon.
        add_vertex_to_facet(first_vertex_in_facet_id_) ;
        set_halfedge_next_around_facet(
            current_halfedge_in_facet_, first_halfedge_in_facet_ 
        ) ;

        // Should not be positionned before calling vertex() !!!
        desc_state_ = desc_cell ;    

        // Connect it with other polyhedra if a mate face is found.
        Halfedge* second_halfedge_in_face = 
            first_halfedge_in_facet_->next_around_facet() ;
    
        Vertex* v1 = first_vertex_in_facet_ ;
        Vertex* v2 = first_halfedge_in_facet_->vertex() ;
        Vertex* v3 = second_halfedge_in_face->vertex() ;
        
        Halfedge* mate = find_halfedge_in_other_cell(
            v3, v2, v1, current_cell_
        ) ;

        if(
            find_halfedge_in_other_cell(v1, v2, v3, current_cell_) != nil
        ) {
            std::cerr << "Duplicate face encountered " << std::endl ;
            std::cerr << "  probably due to a bad orientation of faces" 
                      << std::endl ;
        }
        
        if(mate != nil && can_sew_cells(second_halfedge_in_face, mate)) {
            sew_cells(second_halfedge_in_face, mate) ;
        }        
    }

    void CielBuilder::add_vertex_to_facet(int id) {
        check_description_state(desc_facet) ;
        ogf_assert( id >= 0 && id < int(vertices_.size()) ) ;
        nb_vertices_in_facet_++ ;
        Vertex* vertex = vertices_[id] ;
        if(first_vertex_in_facet_ == nil) {
            first_vertex_in_facet_ = vertex ;
            first_vertex_in_facet_id_ = id ;
        } else {
            Vertex* prev_vertex = nil ;
            Halfedge* prev_combel = current_halfedge_in_facet_ ;
            
            if(prev_combel != nil) {
                prev_vertex = current_halfedge_in_facet_->vertex() ;
            } else {
                prev_vertex = vertices_[first_vertex_in_facet_id_] ;
            }
            
            Halfedge* combel = create_and_initialize_halfedge(
                prev_vertex, vertex
            ) ;

            current_cell_halfedges_.push_back(combel) ;
            
            if(prev_combel != nil) {
                set_halfedge_next_around_facet(prev_combel, combel) ;
            } else {
                set_cell_halfedge(current_cell_, combel) ;
            }
            
            current_halfedge_in_facet_ = combel ;
            if(first_halfedge_in_facet_ == nil) {
                first_halfedge_in_facet_ = combel ;
            }
        }
    }

    void CielBuilder::reset() {
        transition(desc_final, desc_initial) ;
        vertices_.clear() ;
    }

//_________________________________________________________

    void CielBuilder::sew_borders() {
        FOR_EACH_HALFEDGE(Ciel, target(), it) {
            Halfedge* cur = it ;
            if(cur-> opposite_cell() == nil) {
                Halfedge* mate = cur-> opposite_facet() ;
                if(mate == nil) {
                    std::cerr << "open polyhedron found" << std::endl ;
                    abort() ;
                }
                while(mate-> opposite_cell() != nil) {
                    mate = mate-> opposite_cell()-> opposite_facet() ;
                    if(mate == nil) {
                        std::cerr << "open polyhedron found" << std::endl ;
                        continue ;
//                    abort() ;
                    }
                }
                set_halfedge_opposite_cell(cur, mate) ;
                set_halfedge_opposite_cell(mate, cur) ;

                /*
                cur-> flags |= Halfedge::border_mask ;
                mate-> flags |= Halfedge::border_mask ;
                if(mate == cur-> mate_face) {
                    cur-> flags |= Halfedge::corner_mask ;
                    mate-> flags |= Halfedge::corner_mask ;
                }
                */
            }
        }
    }
    
//_________________________________________________________
    
    Ciel::Halfedge* CielBuilder::create_and_initialize_halfedge(
        Vertex* p1, Vertex* p2
    ) {
        
        // Sanity check
        if(find_halfedge_in_cell(p1, p2, current_cell_) != nil) {
            std::cerr << "Duplicate edge encountered" << std::endl ;
            std::cerr << "  probably due to a bad orientation of faces" 
                      << std::endl ;
        }

        Halfedge* combel = new_halfedge() ;
        set_halfedge_vertex(combel, p2) ;
        insert_halfedge_in_vertex_ciel(p1, combel) ;
        set_halfedge_cell(combel, current_cell_) ;
        
        // Note that p2,p1 are in reversed order, since that
        // if the same edge is defined in another polyhedron,
        // it is in reversed order (Moebius law).
        Halfedge* mate = find_halfedge_in_cell(
            p2, p1, current_cell_
        ) ;
        
        if(mate != nil) {
            set_halfedge_opposite_facet(mate, combel) ;
            set_halfedge_opposite_facet(combel, mate) ;
        }

        return combel ;
    }        
    
    void CielBuilder::sew_cells(Halfedge* p1, Halfedge* p2) {
        std::vector<Halfedge*> face1 ;
        Halfedge* cur = nil ;

        cur = p1 ;
        do {
            face1.push_back(cur) ;
            cur = cur-> next_around_facet() ;
        } while(cur != p1) ;
        
        Halfedge* start2 = p2-> next_around_facet() ;
        
        cur = start2 ;
        for(
            int i = face1.size() - 1; i >= 0; 
            i--, cur = cur-> next_around_facet()
        ) {
            Halfedge* it = face1[i] ;
            set_halfedge_opposite_cell(it, cur) ;
            set_halfedge_opposite_cell(cur, it) ;
            
            // This assertion checks that p2 has not less vertices than p1.
            ogf_assert(
                (it == face1[face1.size() - 1]) || (cur != start2)
            ) ;        
        }
        
        // This assertion checks that p2 has not more vertices than p1.    
        ogf_assert(cur == start2) ;
    }
    
    bool CielBuilder::can_sew_cells(Halfedge* p1, Halfedge* p2) {
        std::vector<Halfedge*> face1 ;
        Halfedge* cur = nil ;

        cur = p1 ;
        do {
            face1.push_back(cur) ;
            cur = cur->next_around_facet() ;
        } while(cur != p1) ;

        Halfedge* start2 = p2->next_around_facet() ;
    
        cur = start2 ;
        for(
            int i = face1.size() - 1; i >= 0; 
            i--, cur = cur->next_around_facet()
        ) {
            Halfedge* it = face1[i] ;
            
            if(!((it == face1[face1.size() - 1]) || (cur != start2))) {
                std::cerr << "Warning: cannot sew" << std::endl ;
                return false ;
            }
        }

        if(cur != start2) {
            std::cerr << "Warning: cannot sew" << std::endl ;
            return false ;
        }

        return true ;
    }
    
    Ciel::Halfedge* CielBuilder::find_halfedge_in_cell(
        Vertex* from, Vertex* to, Cell* cell
    ) {
        Halfedge* cur = from->halfedge() ;
        if(cur == nil) {
            return nil ;
        }
        do {
            if(cur == nil) {
                std::cerr << "Vertex invariant broken" << std::endl ;
                abort() ;
            }
            
            if(cur->vertex() == to && cur->cell() == cell) {
                return cur ;
            }
            cur = cur->next_around_vertex() ;
        } while(cur != from->halfedge()) ;
        return nil ;
    }
    
    Ciel::Halfedge* CielBuilder::find_halfedge_in_other_cell(
        Vertex* from, Vertex* to, Vertex* next,
        Cell* cell
    ) {
        Halfedge* cur = from->halfedge() ;
        if(cur == nil) {
            return nil ;
        }
        do {
            if(
                cur->vertex() == to &&
                cur->cell() != cell &&
                cur->next_around_facet()->vertex() == next
            ) {
                return cur ;
            }
            cur = cur->next_around_vertex() ;
        } while(cur != from->halfedge()) ;
        return nil ;
    }

//_________________________________________________________

    void CielBuilder::check_description_state(description_state expected) {
        if(desc_state_ != expected) {
            std::cerr << 
                "Ciel: Description function called in the wrong state"
                      << std::endl ;
            std::cerr << " Ciel descriptor should be " 
                      << description_state_to_string(expected) ;
            std::cerr << std::endl ;
            std::cerr << " and is " << description_state_to_string(desc_state_) ;
            std::cerr << std::endl ;
            std::cerr << "   Aborting ... " << std::endl ;
            abort() ;
        }
    }

    std::string CielBuilder::description_state_to_string(
        description_state desc
    ) {
        switch(desc) {
        case desc_initial:
            return "initial" ;
            break ;
        case desc_volume:
            return "volume" ;
            break ;
        case desc_cell:
            return "cell" ;
            break ;
        case desc_facet:
            return "facet" ;
            break ;
        case desc_final:
            return "final" ;
            break ;
        default:
            return "unknown state" ;
            break ;
        }
    }
}

