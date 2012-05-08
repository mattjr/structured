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
 

#ifndef __CELLS_CGRAPH_CGRAPH_BUILDER__
#define __CELLS_CGRAPH_CGRAPH_BUILDER__

#include <OGF/cells/common/common.h>
#include <OGF/cells/cgraph/cgraph.h>
#include <OGF/cells/cgraph/cgraph_attributes.h>
#include <OGF/cells/map/map_builder.h>

namespace OGF {

//_________________________________________________________
    
    /** 
     * A volumic Cellular Graph optimized for rendering.
     */
    class CELLS_API CGraphBuilder : public CGraphMutator {

    public:
        CGraphBuilder() ;
        virtual void set_target(CGraph* cgraph) ;
        void begin_volume() ;
        void begin_meta_cell(int nb_vertices) ;
        void begin_meta_facet() ;
        void add_vertex_to_meta_facet(int index) ;
        void end_meta_facet() ;
        void end_meta_cell() ;
        
        void add_vertex(const Point3d& p) ;

        void begin_cell(unsigned int meta_cell_id) ;
        void add_vertex_to_cell(unsigned int vertex_id) ;
        void end_cell() ;
        
        void add_cell(unsigned int meta_cell_id, unsigned int* vertices_id) ;
        void end_volume() ;


        Vertex* current_vertex() const ;
        Cell* current_cell() const ;

        /** 
         * Each cell type has a unique index
         */
        const MetaCell* meta_cell( unsigned int index ) const {
            ogf_assert(index < meta_cells_.size()) ;
            return meta_cells_[index] ;
        }

        /** 
         * Binds 'cell' with its neighbors by creating
         * its edges and setting its face connectivities.
         */
        void sew_cell( Cell* cell );


        void build_meta_tetrahedron() ;
        void build_meta_hexahedron() ;
        void build_meta_prism() ;
        void build_meta_pyramid() ;

    private:
        enum description_state {
            desc_initial,
            desc_volume,
            desc_meta_cell,
            desc_meta_facet,
            desc_cell,
            desc_final
        } ;
        description_state desc_state_;

        void check_description_state(description_state expected) ;
        std::string description_state_to_string(
            description_state state
        ) ;
        void transition(description_state from, description_state to) {
            check_description_state(from) ;
            desc_state_ = to ;
        } 
 
    private:
        typedef std::vector<Cell*> Star ;
        CGraphVertexAttribute< Star > star_ ;
        std::vector<Vertex*> vertices_ ;
        Cell* current_cell_ ;
        unsigned int vertex_in_cell_ ;

        std::vector<MetaCell*> meta_cells_ ;
        MetaCell* current_meta_cell_ ;
        MapBuilder map_builder_ ;
    } ;


//_________________________________________________________




}
#endif

