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
 

#ifndef __OGF_CELLS_CGRAPH_CGRAPH__
#define __OGF_CELLS_CGRAPH_CGRAPH__

#include <OGF/cells/common/common.h>
#include <OGF/cells/types/iterators.h>
#include <OGF/cells/cgraph/cgraph_cells.h>
#include <OGF/basic/containers/dlist.h>
#include <OGF/basic/attributes/attribute.h>

#include <vector>

namespace OGF {

//_________________________________________________________

    class CGraph ;
    class CGraphMutator ;

    /**
     * Has its members add() and remove() called
     * whenever a vertex is added (respectively
     * removed) from the structure. This class
     * is meant to be subclassed by client code.
     */
    template <class COMBEL> class CGraphCombelObserver {
    public:
        CGraphCombelObserver(CGraph* m) : cgraph_(m) { }
        virtual ~CGraphCombelObserver() { }
        virtual void add(COMBEL* c) { }
        virtual void remove(COMBEL* c) { }
        
    protected:
        CGraph* cgraph() { return cgraph_; }
        
    private:
        CGraph* cgraph_ ;
    } ;


    /**
     * CGraph implements a 3D cellular graph. All the elements
     * are allocated in a high-performance chunk-based allocator.
     */

    class CELLS_API CGraph {
    public:

        // __________________ types ___________________

        typedef CGraphTypes::Vertex Vertex ;
        typedef CGraphTypes::Cell Cell ;
        typedef CGraphTypes::MetaCell MetaCell ;

        typedef DList<Vertex>::iterator Vertex_iterator ;
        typedef DList<Cell>::iterator Cell_iterator ;

        typedef DList<Vertex>::const_iterator Vertex_const_iterator ;
        typedef DList<Cell>::const_iterator Cell_const_iterator ;

        typedef GenericAttributeManager<Vertex> VertexAttributeManager ;
        typedef GenericAttributeManager<Cell> CellAttributeManager ;



        CGraph() ;
        ~CGraph() ;

        // __________________ access ___________________

        Vertex_iterator vertices_begin()    { return vertices_.begin() ;  }
        Vertex_iterator vertices_end()      { return vertices_.end() ;    }
        Cell_iterator cells_begin()         { return cells_.begin() ;    }
        Cell_iterator cells_end()           { return cells_.end() ;      } 

        Vertex_const_iterator vertices_begin() const { 
            return vertices_.begin() ;  
        }

        Vertex_const_iterator vertices_end() const { 
            return vertices_.end() ;    
        }
        
        Cell_const_iterator cells_begin() const { 
            return cells_.begin() ;    
        }
        Cell_const_iterator cells_end() const { 
            return cells_.end() ;      
        } 

        int size_of_vertices() const  { return vertices_.size() ;  }
        int size_of_cells() const     { return cells_.size() ;    }
        int size_of_meta_cells() const { return meta_cells_.size() ; }
        MetaCell* meta_cell(unsigned int i) const {
            ogf_assert(i < meta_cells_.size()) ;
            return meta_cells_[i] ;
        }

        // ___________________ attributes _______________________
        
        VertexAttributeManager* vertex_attribute_manager() const {
            return const_cast<VertexAttributeManager*>(
                &vertex_attribute_manager_ 
            ) ;
        }

        CellAttributeManager* cell_attribute_manager() const {
            return const_cast<CellAttributeManager*>(
                &cell_attribute_manager_ 
            ) ;
        }         
        
        // ___________________ observers ________________________
        
        void add_vertex_observer(CGraphCombelObserver<Vertex>* obs) ;
        void remove_vertex_observer(CGraphCombelObserver<Vertex>* obs) ;
        void add_cell_observer(CGraphCombelObserver<Cell>* obs) ;
        void remove_cell_observer(CGraphCombelObserver<Cell>* obs) ;

        // __________________ modification ______________________

        void clear() ;
        void erase_all() ;
        void clear_inactive_items() ;

        /**
         * checks the validity of the combinatorial structure.
         */
        bool is_valid() const ;

        /**
         * checks the validity of the combinatorial structure,
         * and stops in an assertion failure if an error is detected.
         * This function can be used to debug low-level operations,
         * such as those in the CGraphBuilder and the CGraphEditor classes.
         */
        void assert_is_valid() const ;

    protected:

        MetaCell* new_meta_cell() ;

        Vertex* new_vertex(const Point3d& p) ;
        Cell* new_cell(MetaCell* meta) ;

        /** copies geometry and attributes from rhs. */
        Vertex* new_vertex(const Vertex* rhs) ;

        /** copies attributes from rhs. */
        Cell* new_cell(const Cell* rhs) ;

        void delete_vertex(Vertex* v) ;
        void delete_cell(Cell* c) ;

        void activate_vertex(Vertex* v) ;
        void activate_cell(Cell* c) ;
        
        void deactivate_vertex(Vertex* v) ;
        void deactivate_cell(Cell* c) ;

        friend class ::OGF::CGraphMutator ;
        
    protected:

        // CGraphCombelObservers notification
        void notify_add_vertex(Vertex* v) ;
        void notify_remove_vertex(Vertex* v) ;
        void notify_add_cell(Cell* c) ;
        void notify_remove_cell(Cell* c) ;

    private:
        DList<Vertex> vertices_ ;
        DList<Cell> cells_ ;
        std::vector<MetaCell*> meta_cells_ ;

        VertexAttributeManager vertex_attribute_manager_ ;
        CellAttributeManager cell_attribute_manager_ ;

        std::vector< CGraphCombelObserver<Vertex>* > vertex_observers_ ;
        std::vector< CGraphCombelObserver<Cell>* > cell_observers_ ;
    } ;

    //______________________________________________________________

    /**
     * CGraphMutator is the base class for the classes
     * that can modify the topology of a cgraph.
     */
    
    class CELLS_API CGraphMutator {
    public:

        // _________________ Types ____________________

        typedef CGraph::Vertex Vertex ;
        typedef CGraph::Cell Cell ;
        typedef CGraph::MetaCell MetaCell ;

        typedef CGraph::Vertex_iterator Vertex_iterator ;
        typedef CGraph::Cell_iterator Cell_iterator ;

        typedef CGraph::Vertex_const_iterator Vertex_const_iterator ;
        typedef CGraph::Cell_const_iterator Cell_const_iterator ;

        CGraphMutator(CGraph* target = nil) : target_(target) { }
        virtual ~CGraphMutator() ; 
        CGraph* target() { return target_ ; }
        virtual void set_target(CGraph* m) ; 
        
    protected:
        
        // _________ create / destroy (see CGraph) _________
        
        Vertex*    new_vertex(const Point3d& p) { return target_->new_vertex(p) ; }
        Cell*      new_cell(MetaCell* meta_cell) { 
            return target_-> new_cell(meta_cell) ;
        }
        Vertex* new_vertex(const CGraph::Vertex* rhs) { 
            return target_-> new_vertex(rhs) ;     
        }
        Cell* new_cell(const CGraph::Cell* rhs) { 
            return target_-> new_cell(rhs) ;      
        }

        void delete_vertex(Vertex* v) { target_-> delete_vertex(v) ; }
        void delete_cell(Cell* c) { target_-> delete_cell(c) ; }
        MetaCell* new_meta_cell() { return target_->new_meta_cell() ; }

        // _________ activate / deactivate (see CGraph) _________

        void activate_vertex(Vertex* v) { target_->activate_vertex(v); }
        void activate_cell(Cell* c) { target_->activate_cell(c); }

        void deactivate_vertex(Vertex* v) { 
            target_->deactivate_vertex(v); 
        }
        void deactivate_cell(Cell* c) { 
            target_->deactivate_cell(c); 
        }


        // ________ Low-level access ____________________________

        void set_cell_vertex(Cell* c, unsigned int i, Vertex* v) {
            c->set_vertex(i,v) ;
        }

        void set_cell_adjacent(Cell* c, unsigned int i, Cell* c2) {
            c->set_adjacent(i,c2) ;
        }

    private:
        CGraph* target_ ;
    } ;

//_________________________________________________________

    /**
     * The constructor and destructor of this
     * specialization call cgraph->add_vertex_observer(this)
     * and cgraph->remove_vertex_observer(this) respectively.
     */
    template<> class CGraphCombelObserver<CGraph::Vertex> {
    public:
        CGraphCombelObserver(CGraph* m) ;
        virtual ~CGraphCombelObserver() ;
        virtual void add(CGraph::Vertex* c) { }
        virtual void remove(CGraph::Vertex* c) { }
        
    protected:
        CGraph* cgraph() { return cgraph_; }
        
    private:
        CGraph* cgraph_ ;
    } ;
    
    /**
     * The constructor and destructor of this
     * specialization call cgraph->add_cell_observer(this)
     * and cgraph->remove_cell_observer(this) respectively.
     */
    template<> class CGraphCombelObserver<CGraph::Cell> {
    public:
        CGraphCombelObserver(CGraph* m) ;
        virtual ~CGraphCombelObserver() ;
        virtual void add(CGraph::Cell* c) { }
        virtual void remove(CGraph::Cell* c) { }
        
    protected:
        CGraph* cgraph() { return cgraph_; }
        
    private:
        CGraph* cgraph_ ;
    } ;

//_________________________________________________________

}
#endif

