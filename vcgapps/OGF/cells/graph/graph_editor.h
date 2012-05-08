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
 

#ifndef __OGF_CELLS_GRAPH_GRAPH_EDITOR__
#define __OGF_CELLS_GRAPH_GRAPH_EDITOR__

#include <OGF/cells/common/common.h>
#include <OGF/cells/graph/graph.h>

namespace OGF {

//_________________________________________________________

    class CELLS_API GraphEditor : public GraphMutator {
    public:
    
        GraphEditor(Graph* target = nil) ; 
        virtual void set_target(Graph* target) ;

        Vertex* new_vertex(const Point3d& p) {
            Vertex* result = GraphMutator::new_vertex() ;
            result->set_point(p) ;
            return result ;
        }

        Halfedge* make_loop(int nb_vertices) ;
        Halfedge* make_polyline(int nb_vertices) ;

        Halfedge* connect_vertices(Vertex* v1, Vertex* v2, Halfedge* copy_attributes_from = nil) ;
        Halfedge* create_edge_and_vertex_from_vertex(Vertex* v1) ;

        Vertex* split_edge(Halfedge* h) ;

        /**
         * precondition: v->degree() == 2
         */
        bool erase_center_vertex(Vertex* v) ;

        /** 
         * destroys the edge referred to by h.
         */
        void erase_edge(Halfedge* h, bool erase_vertices = true) ;

        /** 
         * destroys the edge referred to by h.
         */
        void make_hole(Halfedge* h) ;

        void erase_vertex(Vertex* v) ;

        Vertex* merge_vertices(Vertex* v1, Vertex* v2) ;

        void copy_attributes(Vertex* to, Vertex* from) ;
        void copy_attributes(Halfedge* to, Halfedge* from) ;

    protected:

        Vertex* new_vertex() {
            return GraphMutator::new_vertex() ;
        }

        //_________________ utilities ____________________

        /**
         * Checks the existence of an half_edge e such that
         * e-> vertex() = v1 and e-> opposite()-> vertex() = v2
         */
        bool halfedge_exists_between_vertices(
            Vertex* v1, Vertex* v2
        ) ;

        void insert_halfedge_in_ciel(Halfedge* h) ;
        void remove_halfedge_from_ciel(Halfedge* h) ;

    private:
    } ;


//_________________________________________________________

}
#endif

