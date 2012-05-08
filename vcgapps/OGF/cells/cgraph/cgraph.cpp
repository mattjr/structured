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
 

#include <OGF/cells/cgraph/cgraph.h>
#include <algorithm>

namespace OGF {

//_________________________________________________________
    

    CGraphCombelObserver<CGraph::Vertex>::CGraphCombelObserver(
        CGraph* m
    ) : cgraph_(m) {
        cgraph_->add_vertex_observer(this);
    }
    
    CGraphCombelObserver<CGraph::Vertex>::~CGraphCombelObserver() {
        cgraph_->remove_vertex_observer(this);
    }


    CGraphCombelObserver<CGraph::Cell>::CGraphCombelObserver(
        CGraph* m
    ) : cgraph_(m) {
        cgraph_->add_cell_observer(this);
    }
    
    CGraphCombelObserver<CGraph::Cell>::~CGraphCombelObserver() {
        cgraph_->remove_cell_observer(this);
    }


//_________________________________________________________

    CGraph::CGraph() { 
    }
    
    CGraph::~CGraph() {
        clear() ;
    } 

    // ____________________ Observers ________________________

    void CGraph::add_vertex_observer(CGraphCombelObserver<Vertex>* obs) {
        vertex_observers_.push_back(obs) ;
    }
    
    void CGraph::remove_vertex_observer(CGraphCombelObserver<Vertex>* obs) {
        std::vector<CGraphCombelObserver<Vertex>* >::iterator it = std::find(
            vertex_observers_.begin(), vertex_observers_.end(), obs
        ) ;
        ogf_assert(it != vertex_observers_.end()) ;
        vertex_observers_.erase(it) ;
    }
    
    void CGraph::add_cell_observer(CGraphCombelObserver<Cell>* obs) {
        cell_observers_.push_back(obs) ;
    }
    
    void CGraph::remove_cell_observer(CGraphCombelObserver<Cell>* obs) {
        std::vector<CGraphCombelObserver<Cell>* >::iterator it = std::find(
            cell_observers_.begin(), cell_observers_.end(), obs
        ) ;
        ogf_assert(it != cell_observers_.end()) ;
        cell_observers_.erase(it) ;
    }


    void CGraph::notify_add_vertex(Vertex* v) {
        for(
            std::vector<CGraphCombelObserver<Vertex>* >::iterator
                it=vertex_observers_.begin(); it!=vertex_observers_.end(); it++
        ) {
            (*it)->add(v) ;
        }
    }
    
    void CGraph::notify_remove_vertex(Vertex* v) {
        for(
            std::vector<CGraphCombelObserver<Vertex>* >::iterator
                it=vertex_observers_.begin(); it!=vertex_observers_.end(); it++
        ) {
            (*it)->remove(v) ;
        }
    }
    
    void CGraph::notify_add_cell(Cell* f) {
        for(
            std::vector<CGraphCombelObserver<Cell>* >::iterator
                it=cell_observers_.begin(); 
            it!=cell_observers_.end(); it++
        ) {
            (*it)->add(f) ;
        }
    }
    
    void CGraph::notify_remove_cell(Cell* f) {
        for(
            std::vector<CGraphCombelObserver<Cell>* >::iterator
                it=cell_observers_.begin(); 
            it!=cell_observers_.end(); it++
        ) {
            (*it)->remove(f) ;
        }
    }

    // ____________________ Modification _____________________

    void CGraph::clear() {
        vertices_.clear() ;
        cells_.clear() ;
        vertex_attribute_manager_.clear() ;
        cell_attribute_manager_.clear() ;
    }

    void CGraph::erase_all() {
        clear() ;
    }

    void CGraph::clear_inactive_items() {
        // TODO: traverse the inactive items list, 
        //  and remove the attributes ...
        vertices_.clear_inactive_items() ;
        cells_.clear_inactive_items() ;
    }

    // _____________________ Low level ______________________

    
    CGraph::MetaCell* CGraph::new_meta_cell() {
        MetaCell* result = new MetaCell ;
        meta_cells_.push_back(result) ;
        return result ;
    }

    CGraph::Vertex* CGraph::new_vertex(const Point3d& p) {
        Vertex* result = vertices_.create() ;
        result->set_point(p) ;
        vertex_attribute_manager_.new_record(result) ;
        notify_add_vertex(result) ;
        return result ;
    }
    
    CGraph::Vertex* CGraph::new_vertex(const CGraph::Vertex* rhs) {
        Vertex* result = vertices_.create() ;
        result->set_point(rhs->point()) ;
        vertex_attribute_manager_.new_record(result, rhs) ;
        notify_add_vertex(result) ;
        return result ;
    }

    CGraph::Cell* CGraph::new_cell(MetaCell* meta) {
        Cell* result = cells_.create() ;
        cell_attribute_manager_.new_record(result) ;
        notify_add_cell(result) ;
        result->instanciate_from(meta) ;
        return result ;
    }

    CGraph::Cell* CGraph::new_cell(const CGraph::Cell* rhs) {
        Cell* result = cells_.create() ;
        cell_attribute_manager_.new_record(result, rhs) ;
        notify_add_cell(result) ;
        result->instanciate_from(rhs->meta_cell()) ;
        return result ;
    }

    void CGraph::delete_vertex(Vertex* v) {
        notify_remove_vertex(v) ;
        vertex_attribute_manager_.delete_record(v) ;
        vertices_.destroy(v) ;
    }
    
    void CGraph::delete_cell(Cell* f) {
        notify_remove_cell(f) ;
        cell_attribute_manager_.delete_record(f) ;
        cells_.destroy(f) ;
    }

    void CGraph::activate_vertex(Vertex* v) {
        notify_add_vertex(v) ;
        vertices_.activate(v) ;
    }
    
    void CGraph::activate_cell(Cell* f) {
        notify_add_cell(f) ;
        cells_.activate(f) ;
    }
    
    void CGraph::deactivate_vertex(Vertex* v) {
        notify_remove_vertex(v) ;
        vertices_.deactivate(v) ;
    }
    
    void CGraph::deactivate_cell(Cell* f) {
        notify_remove_cell(f) ;
        cells_.deactivate(f) ;
    }

    bool CGraph::is_valid() const {
        bool ok = true ;
        { FOR_EACH_VERTEX_CONST(CGraph, this, it) {
            ok = ok && it-> is_valid() ;
        }}
        { FOR_EACH_CELL_CONST(CGraph, this, it) {
            ok = ok && it-> is_valid() ;
        }}
        return ok ;
    }

    void CGraph::assert_is_valid() const {
        { FOR_EACH_VERTEX_CONST(CGraph, this, it) {
            it-> assert_is_valid() ;
        }}
        { FOR_EACH_CELL_CONST(CGraph, this, it) {
            it-> assert_is_valid() ;
        }}
    }

//_________________________________________________________

    CGraphMutator::~CGraphMutator() { target_ = nil ;  }

    void CGraphMutator::set_target(CGraph* m) { target_ = m ; }

//_________________________________________________________

}

