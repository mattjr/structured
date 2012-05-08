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
 

#ifndef __OGF_CELLS_CIEL_CIEL__
#define __OGF_CELLS_CIEL_CIEL__

#include <OGF/cells/common/common.h>
#include <OGF/cells/types/iterators.h>
#include <OGF/cells/ciel/ciel_cells.h>
#include <OGF/basic/containers/dlist.h>
#include <OGF/basic/attributes/attribute.h>

#include <vector>
#include <set>

namespace OGF {

//_________________________________________________________

    class Ciel ;
    class CielMutator ;

    /**
     * Has its members add() and remove() called
     * whenever a vertex is added (respectively
     * removed) from the structure. This class
     * is meant to be subclassed by client code.
     */
    template <class COMBEL> class CielCombelObserver {
    public:
        CielCombelObserver(Ciel* m) : ciel_(m) { }
        virtual ~CielCombelObserver() { }
        virtual void add(COMBEL* c) { }
        virtual void remove(COMBEL* c) { }
        
    protected:
        Ciel* ciel() { return ciel_; }
        
    private:
        Ciel* ciel_ ;
    } ;



    /**
     * Ciel implements a 3D topological map. All the elements
     * are allocated in a high-performance chunk-based allocator.
     */

    class CELLS_API Ciel {
    public:

        // __________________ types ___________________

        typedef CielTypes::Vertex Vertex ;
        typedef CielTypes::Halfedge Halfedge ;
        typedef CielTypes::Cell Cell ;

        typedef DList<Vertex>::iterator Vertex_iterator ;
        typedef DList<Halfedge>::iterator Halfedge_iterator ;
        typedef DList<Cell>::iterator Cell_iterator ;

        typedef DList<Vertex>::const_iterator Vertex_const_iterator ;
        typedef DList<Halfedge>::const_iterator Halfedge_const_iterator ;
        typedef DList<Cell>::const_iterator Cell_const_iterator ;

        
        typedef GenericAttributeManager<Vertex> VertexAttributeManager ;
        typedef GenericAttributeManager<Halfedge> HalfedgeAttributeManager ;
        typedef GenericAttributeManager<Cell> CellAttributeManager ;



        Ciel() ;
        ~Ciel() ;

        // __________________ access ___________________

        Vertex_iterator vertices_begin()    { return vertices_.begin() ;  }
        Vertex_iterator vertices_end()      { return vertices_.end() ;    }
        Halfedge_iterator halfedges_begin() { return halfedges_.begin() ; }
        Halfedge_iterator halfedges_end()   { return halfedges_.end() ;   }
        Cell_iterator cells_begin()         { return cells_.begin() ;    }
        Cell_iterator cells_end()           { return cells_.end() ;      } 

        Vertex_const_iterator vertices_begin() const { 
            return vertices_.begin() ;  
        }

        Vertex_const_iterator vertices_end() const { 
            return vertices_.end() ;    
        }
        
        Halfedge_const_iterator halfedges_begin() const { 
            return halfedges_.begin() ; 
        }
        Halfedge_const_iterator halfedges_end() const { 
            return halfedges_.end() ;   
        }
        Cell_const_iterator cells_begin() const { 
            return cells_.begin() ;    
        }
        Cell_const_iterator cells_end() const { 
            return cells_.end() ;      
        } 

        int size_of_vertices() const  { return vertices_.size() ;  }
        int size_of_halfedges() const { return halfedges_.size() ; }
        int size_of_cells() const     { return cells_.size() ;    }

        // ___________________ attributes _______________________
        
        VertexAttributeManager* vertex_attribute_manager() const {
            return const_cast<VertexAttributeManager*>(
                &vertex_attribute_manager_ 
            ) ;
        }

        HalfedgeAttributeManager* halfedge_attribute_manager() const {
            return const_cast<HalfedgeAttributeManager*>(
                &halfedge_attribute_manager_ 
            ) ;
        }
        
        CellAttributeManager* cell_attribute_manager() const {
            return const_cast<CellAttributeManager*>(
                &cell_attribute_manager_ 
            ) ;
        }         
        
        // ___________________ observers ________________________
        
        void add_vertex_observer(CielCombelObserver<Vertex>* obs) ;
        void remove_vertex_observer(CielCombelObserver<Vertex>* obs) ;
        void add_halfedge_observer(CielCombelObserver<Halfedge>* obs) ;
        void remove_halfedge_observer(CielCombelObserver<Halfedge>* obs) ;
        void add_cell_observer(CielCombelObserver<Cell>* obs) ;
        void remove_cell_observer(CielCombelObserver<Cell>* obs) ;

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
         * such as those in the CielBuilder and the CielEditor classes.
         */
        void assert_is_valid() const ;

    protected:

        Vertex* new_vertex() ;
        Halfedge* new_halfedge() ;
        Cell* new_cell() ;

        /** copies geometry and attributes from rhs. */
        Vertex* new_vertex(const Vertex* rhs) ;

        /** copies attributes from rhs. */
        Halfedge* new_halfedge(const Halfedge* rhs) ;

        /** copies attributes from rhs. */
        Cell* new_cell(const Cell* rhs) ;

        void delete_vertex(Vertex* v) ;
        void delete_halfedge(Halfedge* h) ;
        void delete_cell(Cell* c) ;

        void activate_vertex(Vertex* v) ;
        void activate_halfedge(Halfedge* h) ;
        void activate_cell(Cell* c) ;
        
        void deactivate_vertex(Vertex* v) ;
        void deactivate_halfedge(Halfedge* h) ;
        void deactivate_cell(Cell* c) ;

        friend class ::OGF::CielMutator ;
        
    protected:

        // CielCombelObservers notification
        void notify_add_vertex(Vertex* v) ;
        void notify_remove_vertex(Vertex* v) ;
        void notify_add_halfedge(Halfedge* h) ;
        void notify_remove_halfedge(Halfedge* h) ;
        void notify_add_cell(Cell* c) ;
        void notify_remove_cell(Cell* c) ;

    private:
        DList<Vertex> vertices_ ;
        DList<Halfedge> halfedges_ ;
        DList<Cell> cells_ ;

        VertexAttributeManager vertex_attribute_manager_ ;
        HalfedgeAttributeManager halfedge_attribute_manager_ ;
        CellAttributeManager cell_attribute_manager_ ;

        std::vector< CielCombelObserver<Vertex>* > vertex_observers_ ;
        std::vector< CielCombelObserver<Halfedge>* > halfedge_observers_ ;
        std::vector< CielCombelObserver<Cell>* > cell_observers_ ;
    } ;

    //______________________________________________________________

    /**
     * CielMutator is the base class for the classes
     * that can modify the topology of a ciel.
     */
    
    class CELLS_API CielMutator {
    public:

        // _________________ Types ____________________

        typedef Ciel::Vertex Vertex ;
        typedef Ciel::Halfedge Halfedge ;
        typedef Ciel::Cell Cell ;

        typedef Ciel::Vertex_iterator Vertex_iterator ;
        typedef Ciel::Halfedge_iterator Halfedge_iterator ;
        typedef Ciel::Cell_iterator Cell_iterator ;

        typedef Ciel::Vertex_const_iterator Vertex_const_iterator ;
        typedef Ciel::Halfedge_const_iterator Halfedge_const_iterator ;
        typedef Ciel::Cell_const_iterator Cell_const_iterator ;

        CielMutator(Ciel* target = nil) : target_(target) { }
        virtual ~CielMutator() ; 
        Ciel* target() { return target_ ; }
        virtual void set_target(Ciel* m) ; 
        
    protected:
        
        // _________ create / destroy (see Ciel) _________
        
        Vertex*    new_vertex()     { return target_->new_vertex() ;      }
        Halfedge*  new_halfedge()   { return target_-> new_halfedge() ;   }
        Cell*      new_cell()       { return target_-> new_cell() ;       }

        Vertex* new_vertex(const Ciel::Vertex* rhs) { 
            return target_-> new_vertex(rhs) ;     
        }
        Halfedge* new_halfedge(const Ciel::Halfedge* rhs) { 
            return target_-> new_halfedge(rhs) ;
        }
        Cell* new_cell(const Ciel::Cell* rhs) { 
            return target_-> new_cell(rhs) ;      
        }


        void delete_vertex(Vertex* v) { target_-> delete_vertex(v) ; }
        void delete_halfedge(Halfedge* h) { 
            target_-> delete_halfedge(h) ; 
        }
        void delete_cell(Cell* c) { target_-> delete_cell(c) ; }

        // _________ activate / deactivate (see Ciel) _________

        void activate_vertex(Vertex* v) { target_->activate_vertex(v); }
        void activate_halfedge(Halfedge* h) { target_->activate_halfedge(h); }
        void activate_cell(Cell* c) { target_->activate_cell(c); }

        void deactivate_vertex(Vertex* v) { 
            target_->deactivate_vertex(v); 
        }
        void deactivate_halfedge(Halfedge* h) { 
            target_->deactivate_halfedge(h); 
        }
        void deactivate_cell(Cell* c) { 
            target_->deactivate_cell(c); 
        }

        // _________ vertex basic functions ____________________
        
        void set_vertex_halfedge(Vertex* v, Halfedge* h) {
            v->set_halfedge(h) ;
        }
        void insert_halfedge_in_vertex_ciel(Vertex* v, Halfedge* h) {
            v->insert_halfedge_in_ciel(h) ;
        }

        // _________ halfedge basic functions ____________________
        
        void set_halfedge_next_around_vertex(Halfedge* h1, Halfedge* h2) { 
            h1->set_next_around_vertex(h2) ;
        }
        void set_halfedge_next_around_facet(Halfedge* h1, Halfedge* h2) { 
            h1->set_next_around_facet(h2) ;
        }
        void set_halfedge_opposite_facet(Halfedge* h1, Halfedge* h2) {
            h1->set_opposite_facet(h2) ;
        }
        void set_halfedge_opposite_cell(Halfedge* h1, Halfedge* h2) { 
            h1->set_opposite_cell(h2) ;
        }
        void set_halfedge_vertex(Halfedge* h, Vertex* v) { 
            h->set_vertex(v) ;
        }
        void set_halfedge_cell(Halfedge* h, Cell* c) {
            h->set_cell(c) ;
        }

        // _________ cell basic functions ____________________

        void set_cell_halfedge(Cell* c, Halfedge* h) {
            c->set_halfedge(h) ;
        }

    private:
        Ciel* target_ ;
    } ;

//_________________________________________________________

    /**
     * The constructor and destructor of this
     * specialization call ciel->add_vertex_observer(this)
     * and ciel->remove_vertex_observer(this) respectively.
     */
    template<> class CielCombelObserver<Ciel::Vertex> {
    public:
        CielCombelObserver(Ciel* m) ;
        virtual ~CielCombelObserver() ;
        virtual void add(Ciel::Vertex* c) { }
        virtual void remove(Ciel::Vertex* c) { }
        
    protected:
        Ciel* ciel() { return ciel_; }
        
    private:
        Ciel* ciel_ ;
    } ;
    
    /**
     * The constructor and destructor of this
     * specialization call ciel->add_halfedge_observer(this)
     * and ciel->remove_halfedge_observer(this) respectively.
     */
    template<> class CielCombelObserver<Ciel::Halfedge> {
    public:
        CielCombelObserver(Ciel* m) ;
        virtual ~CielCombelObserver() ;
        virtual void add(Ciel::Halfedge* c) { }
        virtual void remove(Ciel::Halfedge* c) { }
        
    protected:
        Ciel* ciel() { return ciel_; }
        
    private:
        Ciel* ciel_ ;
    } ;


    /**
     * The constructor and destructor of this
     * specialization call ciel->add_cell_observer(this)
     * and ciel->remove_cell_observer(this) respectively.
     */
    template<> class CielCombelObserver<Ciel::Cell> {
    public:
        CielCombelObserver(Ciel* m) ;
        virtual ~CielCombelObserver() ;
        virtual void add(Ciel::Cell* c) { }
        virtual void remove(Ciel::Cell* c) { }
        
    protected:
        Ciel* ciel() { return ciel_; }
        
    private:
        Ciel* ciel_ ;
    } ;

//_________________________________________________________

}
#endif

