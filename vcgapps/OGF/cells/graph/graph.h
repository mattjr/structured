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
 

#ifndef __OGF_CELLS_GRAPH_GRAPH__
#define __OGF_CELLS_GRAPH_GRAPH__

#include <OGF/cells/common/common.h>
#include <OGF/cells/types/iterators.h>
#include <OGF/cells/graph/graph_cells.h>
#include <OGF/basic/containers/dlist.h>
#include <OGF/basic/attributes/attribute.h>

#include <vector>

namespace OGF {

//_________________________________________________________


    class Graph ;
    class GraphMutator ;


    /**
     * Has its members add() and remove() called
     * whenever a vertex is added (respectively
     * removed) from the structure. This class
     * is meant to be subclassed by client code.
     */
    template <class CELL> class GraphCombelObserver {
    public:
        GraphCombelObserver(Graph* m) : graph_(m) { }
        virtual ~GraphCombelObserver() { }
        virtual void add(CELL* c) { }
        virtual void remove(CELL* c) { }
        
    protected:
        Graph* graph() { return graph_; }
        
    private:
        Graph* graph_ ;
    } ;



    /**
     * Implements a graph. Each arc is composed of two Halfedges.
     * Each vertex has a geometry.
     */

    class CELLS_API Graph {
    public:

        // __________________ types ___________________

        typedef GraphTypes::Vertex Vertex ;
        typedef GraphTypes::Halfedge Halfedge ;

        typedef DList<Vertex>::iterator Vertex_iterator ;
        typedef DList<Halfedge>::iterator Halfedge_iterator ;

        typedef DList<Vertex>::const_iterator Vertex_const_iterator ;
        typedef DList<Halfedge>::const_iterator Halfedge_const_iterator ;
        
        typedef GenericAttributeManager<Vertex> VertexAttributeManager ;
        typedef GenericAttributeManager<Halfedge> HalfedgeAttributeManager ;



        Graph() ;
        virtual ~Graph() ;

        // __________________ access ___________________

        Vertex_iterator vertices_begin()    { return vertices_.begin() ;  }
        Vertex_iterator vertices_end()      { return vertices_.end() ;    }
        Halfedge_iterator halfedges_begin() { return halfedges_.begin() ; }
        Halfedge_iterator halfedges_end()   { return halfedges_.end() ;   }

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

        int size_of_vertices() const  { return vertices_.size() ;  }
        int size_of_halfedges() const { return halfedges_.size() ; }

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
        
        // ___________________ observers ________________________

        void add_vertex_observer(GraphCombelObserver<Vertex>* obs) ;
        void remove_vertex_observer(GraphCombelObserver<Vertex>* obs) ;
        void add_halfedge_observer(GraphCombelObserver<Halfedge>* obs) ;
        void remove_halfedge_observer(GraphCombelObserver<Halfedge>* obs) ;

        // __________________ modification ______________________

        void clear() ;
        void erase_all() ;
        void clear_inactive_items() ;

        // __________________ low level (for experts only) ______

        /**
         * checks the validity of the combinatorial structure.
         */
        bool is_valid() const ;

        /**
         * checks the validity of the combinatorial structure,
         * and stops in an assertion failure if an error is detected.
         * This function can be used to debug low-level operations,
         * such as those in the GraphBuilder and the GraphEditor classes.
         */
        void assert_is_valid() const ;

        /**
         * fills the list l with all the vertices connected to v,
         * directly or indirectly.
         */
        void get_connected_component(
            Vertex* v, std::vector<Vertex*>& l
        ) ;

    protected:

        /**
         * allocates a pair of halfedges connected with next(), prev(),
         * and opposite() links.
         */
        Halfedge* new_edge() ;

        /**
         * deallocates a pair of halfedges, connected with an opposite() link.
         */
        void delete_edge(Halfedge* h) ;

        Vertex* new_vertex() ;
        Halfedge* new_halfedge() ;

        /** copies geometry and attributes from rhs. */
        Vertex* new_vertex(const Vertex* rhs) ;

        /** copies attributes from rhs. */
        Halfedge* new_halfedge(const Halfedge* rhs) ;

        void delete_vertex(Vertex* v) ;
        void delete_halfedge(Halfedge* h) ;

        void activate_vertex(Vertex* v) ;
        void activate_halfedge(Halfedge* h) ;
        
        void deactivate_vertex(Vertex* v) ;
        void deactivate_halfedge(Halfedge* h) ;
        
        friend class ::OGF::GraphMutator ;
        
    protected:

        // GraphCombelObservers notification
        void notify_add_vertex(Vertex* v) ;
        void notify_remove_vertex(Vertex* v) ;
        void notify_add_halfedge(Halfedge* h) ;
        void notify_remove_halfedge(Halfedge* h) ;

    private:
        DList<Vertex> vertices_ ;
        DList<Halfedge> halfedges_ ;

        VertexAttributeManager vertex_attribute_manager_ ;
        HalfedgeAttributeManager halfedge_attribute_manager_ ;

        std::vector< GraphCombelObserver<Vertex>* > vertex_observers_ ;
        std::vector< GraphCombelObserver<Halfedge>* > halfedge_observers_ ;
    } ;

    //______________________________________________________________

    /**
     * GraphMutator is the base class for the classes
     * that can modify the topology of a graph.
     */
    
    class CELLS_API GraphMutator {
    public:

        // _________________ Types ____________________

        typedef Graph::Vertex Vertex ;
        typedef Graph::Halfedge Halfedge ;

        typedef Graph::Vertex_iterator Vertex_iterator ;
        typedef Graph::Halfedge_iterator Halfedge_iterator ;

        typedef Graph::Vertex_const_iterator Vertex_const_iterator ;
        typedef Graph::Halfedge_const_iterator Halfedge_const_iterator ;

        GraphMutator(Graph* target = nil) : target_(target) { }
        virtual ~GraphMutator() ; 
        Graph* target() { return target_ ; }
        virtual void set_target(Graph* m) ; 
        
    protected:
        
        void make_vertex_key(Halfedge* h) { 
            h->vertex()->set_halfedge(h) ; 
        }

        void make_vertex_key(Halfedge* h, Vertex* v) { 
            v->set_halfedge(h) ;
            h->set_vertex(v) ;
        }

        // _________ create / destroy (see Graph) _________
        
        Halfedge* new_edge() { return target_-> new_edge() ; }
        Halfedge* new_edge(Halfedge* copy_attributes_from) ;

        void delete_edge(Halfedge* h) { target_-> delete_edge(h) ; }

        Vertex*    new_vertex()     { return target_-> new_vertex() ;     }
        Halfedge*  new_halfedge()   { return target_-> new_halfedge() ;   }

        Vertex* new_vertex(const Graph::Vertex* rhs) { 
            return target_-> new_vertex(rhs) ;     
        }
        Halfedge* new_halfedge(const Graph::Halfedge* rhs) { 
            return target_-> new_halfedge(rhs) ;
        }

        void delete_vertex(Vertex* v) { target_-> delete_vertex(v) ; }
        void delete_halfedge(Halfedge* h) { 
            target_-> delete_halfedge(h) ; 
        }

        // _________ activate / deactivate (see Graph) _________

        void activate_vertex(Vertex* v) { target_->activate_vertex(v); }
        void activate_halfedge(Halfedge* h) { target_->activate_halfedge(h); }

        void deactivate_vertex(Vertex* v) { 
            target_->deactivate_vertex(v); 
        }
        void deactivate_halfedge(Halfedge* h) { 
            target_->deactivate_halfedge(h); 
        }

        // _________ basic functions ____________________
        
        void set_vertex_halfedge(Vertex* v, Halfedge* h) { 
            v-> set_halfedge(h) ; 
        }
        
        void set_halfedge_opposite(Halfedge* h1, Halfedge* h2) { 
            h1-> set_opposite(h2) ;
        }
        
        void set_halfedge_next_around_vertex(Halfedge* h1, Halfedge* h2) { 
            h1-> set_next_around_vertex(h2) ;
        }
        
        void set_halfedge_prev_around_vertex(Halfedge* h1, Halfedge* h2) { 
            h1-> set_prev_around_vertex(h2) ;
        }
        
        void set_halfedge_vertex(Halfedge* h, Vertex* v) { 
            h-> set_vertex(v) ;
        }

    private:
        Graph* target_ ;
    } ;

//_________________________________________________________

    /**
     * The constructor and destructor of this
     * specialization call graph->add_vertex_observer(this)
     * and graph->remove_vertex_observer(this) respectively.
     */
    template<> class GraphCombelObserver<Graph::Vertex> {
    public:
        GraphCombelObserver(Graph* m) ;
        virtual ~GraphCombelObserver() ;
        virtual void add(Graph::Vertex* c) { }
        virtual void remove(Graph::Vertex* c) { }
        
    protected:
        Graph* graph() { return graph_; }
        
    private:
        Graph* graph_ ;
    } ;
    
    /**
     * The constructor and destructor of this
     * specialization call graph->add_halfedge_observer(this)
     * and graph->remove_halfedge_observer(this) respectively.
     */
    template<> class GraphCombelObserver<Graph::Halfedge> {
    public:
        GraphCombelObserver(Graph* m) ;
        virtual ~GraphCombelObserver() ;
        virtual void add(Graph::Halfedge* c) { }
        virtual void remove(Graph::Halfedge* c) { }
        
    protected:
        Graph* graph() { return graph_; }
        
    private:
        Graph* graph_ ;
    } ;

//_________________________________________________________

}
#endif

