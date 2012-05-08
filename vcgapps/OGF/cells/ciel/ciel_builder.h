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
 

#ifndef __CIEL_BUILDER__
#define __CIEL_BUILDER__

#include <OGF/cells/common/common.h>
#include <OGF/cells/ciel/ciel.h>

#include <vector>
#include <set>

namespace OGF {

//_________________________________________________________

    class CELLS_API CielBuilder : public CielMutator {
        
    public:

        CielBuilder() ;
        CielBuilder(Ciel* ciel_in) ;

        //______________________ description ________________________
        
        /**
         * starts the description of the object to be rendered. 
         * CIEL constructs an alternate representation of
         * the object, optimized for its rendering algorithm.
         */
        void begin_volume() ;

        /**
         * should be called when the description of the object
         * is complete. This function is meant to realize the
         * internal structures of CIEL before rendering
         * starts.
         */
        void end_volume();


        /**
         * adds a vertex into this CIEL vertices block. When properties
         * are attached to vertices, in shaded mode, the gradients are
         * automatically computed by CIEL. It is also possible to
         * specify a client code supplied gradient.
         * @see CIEL#describe_vertex_with_gradient
         *
         * @param geometry 3D location of the vertex
         */
        void add_vertex(const Point3d& geometry) ;

        /** starts a new cell */
        void begin_cell() ;

        /** ends the description of a cell */
        void end_cell() ;

        /**
         * starts the description of a facet of a cell. The
         * face is an ordered list of vertices ids, specified by
         * calls to vertex(vertex_id id).
         */
        void begin_facet() ;
        
        /** ends the description of a facet of a cell. */
        void end_facet() ;

        /**
         * appends a vertex to a facet of a cell
         * @param vertex_id the id of the vertex, between 0 and
         *    nb_vertices - 1
         */
        void add_vertex_to_facet(int vertex_id) ;

        /**
         * This function needs to be called if the same builder is used
         * several times.
         */
        void reset() ;

    protected:

        // Helper functions

        Halfedge* create_and_initialize_halfedge(
            Vertex* p1, Vertex* p2
        ) ;        

        void sew_cells(Halfedge* p1, Halfedge* p2) ;

        bool can_sew_cells(Halfedge* p1, Halfedge* p2) ;

        Halfedge* find_halfedge_in_cell(
            Vertex* from, Vertex* to, Cell* cell
        ) ;
        
        Halfedge* find_halfedge_in_other_cell(
            Vertex* from, Vertex* to, Vertex* next, Cell* cell
        ) ;

        void sew_borders() ;

    protected:  
        // Finite state automaton

        enum description_state {
            desc_initial,
            desc_volume,
            desc_cell,
            desc_facet,
            desc_final
        } ;

        void check_description_state(description_state expected) ;
        std::string description_state_to_string(
            description_state state
        ) ;
        void transition(description_state from, description_state to) {
            check_description_state(from) ;
            desc_state_ = to ;
        } 

    private:
        // construction
        description_state desc_state_ ;
        std::vector<Vertex*> vertices_ ;
        int nb_vertices_in_facet_ ;
        int first_vertex_in_facet_id_ ;
        Vertex* first_vertex_in_facet_ ;
        Halfedge* first_halfedge_in_facet_ ;
        Halfedge* current_halfedge_in_facet_ ;
        Cell* current_cell_ ;

        // for debugging
        std::vector<Halfedge*> current_cell_halfedges_ ;
    } ;
//_________________________________________________________

}
#endif

