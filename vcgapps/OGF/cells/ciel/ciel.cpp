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
 

#include <OGF/cells/ciel/ciel.h>
#include <algorithm>

namespace OGF {

//_________________________________________________________
    

    CielCombelObserver<Ciel::Vertex>::CielCombelObserver(Ciel* m) : ciel_(m) {
        ciel_->add_vertex_observer(this);
    }
    
    CielCombelObserver<Ciel::Vertex>::~CielCombelObserver() {
        ciel_->remove_vertex_observer(this);
    }


    CielCombelObserver<Ciel::Halfedge>::CielCombelObserver(Ciel* m) : ciel_(m) {
        ciel_->add_halfedge_observer(this);
    }
    
    CielCombelObserver<Ciel::Halfedge>::~CielCombelObserver() {
        ciel_->remove_halfedge_observer(this);
    }


    CielCombelObserver<Ciel::Cell>::CielCombelObserver(Ciel* m) : ciel_(m) {
        ciel_->add_cell_observer(this);
    }
    
    CielCombelObserver<Ciel::Cell>::~CielCombelObserver() {
        ciel_->remove_cell_observer(this);
    }


//_________________________________________________________

    Ciel::Ciel() { 
    }
    
    Ciel::~Ciel() {
        clear() ;
    } 

    // ____________________ Observers ________________________

    void Ciel::add_vertex_observer(CielCombelObserver<Vertex>* obs) {
        vertex_observers_.push_back(obs) ;
    }
    
    void Ciel::remove_vertex_observer(CielCombelObserver<Vertex>* obs) {
        std::vector<CielCombelObserver<Vertex>* >::iterator it = std::find(
            vertex_observers_.begin(), vertex_observers_.end(), obs
        ) ;
        ogf_assert(it != vertex_observers_.end()) ;
        vertex_observers_.erase(it) ;
    }
    
    void Ciel::add_halfedge_observer(CielCombelObserver<Halfedge>* obs) {
        halfedge_observers_.push_back(obs)  ;
    }
    
    void Ciel::remove_halfedge_observer(CielCombelObserver<Halfedge>* obs) {
        std::vector<CielCombelObserver<Halfedge>* >::iterator it = std::find(
            halfedge_observers_.begin(), halfedge_observers_.end(), obs
        ) ;
        ogf_assert(it != halfedge_observers_.end()) ;
        halfedge_observers_.erase(it) ;
    }
    
    void Ciel::add_cell_observer(CielCombelObserver<Cell>* obs) {
        cell_observers_.push_back(obs) ;
    }
    
    void Ciel::remove_cell_observer(CielCombelObserver<Cell>* obs) {
        std::vector<CielCombelObserver<Cell>* >::iterator it = std::find(
            cell_observers_.begin(), cell_observers_.end(), obs
        ) ;
        ogf_assert(it != cell_observers_.end()) ;
        cell_observers_.erase(it) ;
    }


    void Ciel::notify_add_vertex(Vertex* v) {
        for(
            std::vector<CielCombelObserver<Vertex>* >::iterator
                it=vertex_observers_.begin(); it!=vertex_observers_.end(); it++
        ) {
            (*it)->add(v) ;
        }
    }
    
    void Ciel::notify_remove_vertex(Vertex* v) {
        for(
            std::vector<CielCombelObserver<Vertex>* >::iterator
                it=vertex_observers_.begin(); it!=vertex_observers_.end(); it++
        ) {
            (*it)->remove(v) ;
        }
    }
    
    void Ciel::notify_add_halfedge(Halfedge* h) {
        for(
            std::vector<CielCombelObserver<Halfedge>* >::iterator
                it=halfedge_observers_.begin(); 
            it!=halfedge_observers_.end(); it++
        ) {
            (*it)->add(h) ;
        }
    }
    
    void Ciel::notify_remove_halfedge(Halfedge* h) {
        for(
            std::vector<CielCombelObserver<Halfedge>* >::iterator
                it=halfedge_observers_.begin(); 
            it!=halfedge_observers_.end(); it++
        ) {
            (*it)->remove(h) ;
        }
    }
    
    void Ciel::notify_add_cell(Cell* f) {
        for(
            std::vector<CielCombelObserver<Cell>* >::iterator
                it=cell_observers_.begin(); 
            it!=cell_observers_.end(); it++
        ) {
            (*it)->add(f) ;
        }
    }
    
    void Ciel::notify_remove_cell(Cell* f) {
        for(
            std::vector<CielCombelObserver<Cell>* >::iterator
                it=cell_observers_.begin(); 
            it!=cell_observers_.end(); it++
        ) {
            (*it)->remove(f) ;
        }
    }

    // ____________________ Modification _____________________

    void Ciel::clear() {
        vertices_.clear() ;
        halfedges_.clear() ;
        cells_.clear() ;
        
        vertex_attribute_manager_.clear() ;
        halfedge_attribute_manager_.clear() ;
        cell_attribute_manager_.clear() ;
    }

    void Ciel::erase_all() {
        clear() ;
    }

    void Ciel::clear_inactive_items() {
        // TODO: traverse the inactive items list, 
        //  and remove the attributes ...
        vertices_.clear_inactive_items() ;
        halfedges_.clear_inactive_items() ;
        cells_.clear_inactive_items() ;
    }

    // _____________________ Low level ______________________

    
    Ciel::Vertex* Ciel::new_vertex() {
        Vertex* result = vertices_.create() ;
        vertex_attribute_manager_.new_record(result) ;
        notify_add_vertex(result) ;
        return result ;
    }
    
    Ciel::Vertex* Ciel::new_vertex(const Ciel::Vertex* rhs) {
        Vertex* result = vertices_.create() ;
        result->set_point(rhs->point()) ;
        vertex_attribute_manager_.new_record(result, rhs) ;
        notify_add_vertex(result) ;
        return result ;
    }

    Ciel::Halfedge* Ciel::new_halfedge() {
        Halfedge* result = halfedges_.create() ;
        halfedge_attribute_manager_.new_record(result) ;
        notify_add_halfedge(result) ;
        return result ;
    }

    Ciel::Halfedge* Ciel::new_halfedge(const Ciel::Halfedge* rhs) {
        Halfedge* result = halfedges_.create() ;
        halfedge_attribute_manager_.new_record(result, rhs) ;
        notify_add_halfedge(result) ;
        return result ;
    }
    
    Ciel::Cell* Ciel::new_cell() {
        Cell* result = cells_.create() ;
        cell_attribute_manager_.new_record(result) ;
        notify_add_cell(result) ;
        return result ;
    }

    Ciel::Cell* Ciel::new_cell(const Ciel::Cell* rhs) {
        Cell* result = cells_.create() ;
        cell_attribute_manager_.new_record(result, rhs) ;
        notify_add_cell(result) ;
        return result ;
    }

    void Ciel::delete_vertex(Vertex* v) {
        notify_remove_vertex(v) ;
        vertex_attribute_manager_.delete_record(v) ;
        vertices_.destroy(v) ;
    }
    
    void Ciel::delete_halfedge(Halfedge* h) {
        notify_remove_halfedge(h) ;
        halfedge_attribute_manager_.delete_record(h) ;
        halfedges_.destroy(h) ;
    }
    
    void Ciel::delete_cell(Cell* f) {
        notify_remove_cell(f) ;
        cell_attribute_manager_.delete_record(f) ;
        cells_.destroy(f) ;
    }

    void Ciel::activate_vertex(Vertex* v) {
        notify_add_vertex(v) ;
        vertices_.activate(v) ;
    }
    
    void Ciel::activate_halfedge(Halfedge* h) {
        notify_add_halfedge(h) ;
        halfedges_.activate(h) ;
    }
    
    void Ciel::activate_cell(Cell* f) {
        notify_add_cell(f) ;
        cells_.activate(f) ;
    }
    
    void Ciel::deactivate_vertex(Vertex* v) {
        notify_remove_vertex(v) ;
        vertices_.deactivate(v) ;
    }
    
    void Ciel::deactivate_halfedge(Halfedge* h) {
        notify_remove_halfedge(h) ;
        halfedges_.deactivate(h) ;
    }
    
    void Ciel::deactivate_cell(Cell* f) {
        notify_remove_cell(f) ;
        cells_.deactivate(f) ;
    }

    bool Ciel::is_valid() const {
        bool ok = true ;
        { FOR_EACH_VERTEX_CONST(Ciel, this, it) {
            ok = ok && it-> is_valid() ;
        }}
        { FOR_EACH_HALFEDGE_CONST(Ciel, this, it) {
            ok = ok && it-> is_valid() ;
        }}
        { FOR_EACH_CELL_CONST(Ciel, this, it) {
            ok = ok && it-> is_valid() ;
        }}
        return ok ;
    }

    void Ciel::assert_is_valid() const {
        { FOR_EACH_VERTEX_CONST(Ciel, this, it) {
            it-> assert_is_valid() ;
        }}
        { FOR_EACH_HALFEDGE_CONST(Ciel, this, it) {
            it-> assert_is_valid() ;
        }}
        { FOR_EACH_CELL_CONST(Ciel, this, it) {
            it-> assert_is_valid() ;
        }}
    }


//_________________________________________________________

    CielMutator::~CielMutator() { target_ = nil ;  }

    void CielMutator::set_target(Ciel* m) { target_ = m ; }

//_________________________________________________________

}

