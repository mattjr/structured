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
 

#include <OGF/cells/cgraph/cgraph_builder.h>
#include <OGF/basic/debug/logger.h>

namespace OGF {

//_________________________________________________________

    CGraphBuilder::CGraphBuilder() {
        current_cell_ = nil ;
        vertex_in_cell_ = 0 ;
        current_meta_cell_ = nil ;
        desc_state_ = desc_initial ;
    }

    void CGraphBuilder::set_target(CGraph* cgraph) {
        CGraphMutator::set_target(cgraph) ;
        if(cgraph != nil) {
            for(int i=0; i<cgraph->size_of_meta_cells(); i++) {
                meta_cells_.push_back(cgraph->meta_cell(i)) ;
            }
        }
    }

    CGraph::Vertex* CGraphBuilder::current_vertex() const {
        return *(vertices_.rbegin()) ;
    }

    CGraph::Cell* CGraphBuilder::current_cell() const {
        return current_cell_ ;
    }

    void CGraphBuilder::begin_volume() {
        transition(desc_initial, desc_volume) ;
    }

    void CGraphBuilder::begin_meta_cell(int nb_vertices) {
        transition(desc_volume, desc_meta_cell) ;
        current_meta_cell_ = new_meta_cell() ;
        meta_cells_.push_back(current_meta_cell_) ;
        map_builder_.set_target(&(current_meta_cell_->map())) ;
        map_builder_.begin_surface() ;
        for(int i = 0; i < nb_vertices; i++) {
            map_builder_.add_vertex(Point3d()) ;
        }
    }
    
    void CGraphBuilder::begin_meta_facet() {
        transition(desc_meta_cell, desc_meta_facet) ;
        map_builder_.begin_facet() ;
    }
    
    void CGraphBuilder::add_vertex_to_meta_facet(int index) {
        map_builder_.add_vertex_to_facet(index) ;
    }
    
    void CGraphBuilder::end_meta_facet() {
        map_builder_.end_facet() ;
        transition(desc_meta_facet, desc_meta_cell) ;
    }
    
    void CGraphBuilder::end_meta_cell() {
        map_builder_.end_surface() ;
        map_builder_.reset() ;
        current_meta_cell_->initialize_from_map() ;
        transition(desc_meta_cell, desc_volume) ;
    }
    
    void CGraphBuilder::add_vertex(const Point3d& p) {
        check_description_state( desc_volume );
        Vertex* result = new_vertex(p) ;
        vertices_.push_back(result) ;
    }

    void CGraphBuilder::begin_cell( unsigned int cell_type ) {
        transition(desc_volume, desc_cell) ;
        cgraph_assert( cell_type < meta_cells_.size() );
        current_cell_ = new_cell(meta_cells_[cell_type]) ;
        vertex_in_cell_ = 0;
    }

    void CGraphBuilder::add_vertex_to_cell( unsigned int vertex_index ) {
        check_description_state( desc_cell );
    
        cgraph_assert( vertex_index < vertices_.size() );
        cgraph_assert( vertex_in_cell_ < current_cell_->meta_cell()->nb_vertices() );
    
        // Add the vertex to current cell
        Vertex* vertex = vertices_[vertex_index];
        set_cell_vertex(current_cell_, vertex_in_cell_, vertex) ;
        vertex_in_cell_++;
    }

    void CGraphBuilder::end_cell() {
        transition(desc_cell, desc_volume) ;
        ogf_assert(vertex_in_cell_ == current_cell_->nb_vertices()) ;
    }

    void CGraphBuilder::end_volume() {
        transition(desc_volume, desc_final) ;

        star_.bind(target()) ;
        Logger::out("CGraph") << "creating stars" << std::endl ;
        // Creating stars
        { FOR_EACH_CELL(CGraph, target(), it) { 
            for(unsigned int i=0; i<it-> nb_vertices(); i++) {
                star_[it->vertex(i)].push_back(it) ;
            }
        }}

        Logger::out("CGraph") << "sewing cells" << std::endl ;
        // Sew cells, make each cell know its 2-connected neighbors
        { FOR_EACH_CELL(CGraph, target(), it) { 
            sew_cell(it) ;
        }}


        Logger::out("CGraph") << "removing stars" << std::endl ;
        star_.unbind() ;

        Logger::out("CGraph") << "CGraph Build complete" << std::endl;
        Logger::out("CGraphBuilder") << "nb cells=" << target()->size_of_cells()
                                     << " nb vertices=" << target()->size_of_vertices()
                                     << " nb meta cells=" << target()->size_of_meta_cells()
                                     << std::endl ;
    }

    
//______________________ Protected construction______________________________
    
    void CGraphBuilder::sew_cell( Cell* cell ) {

        // Sew faces;
        // A face can be sewn with another face if it shares 
        // two common edges.
        for( unsigned int face_i=0; face_i < cell->nb_facets(); ++face_i ) {
            if( cell->adjacent(face_i) != nil ) {
                continue;
            }
            Vertex* v1 = cell-> facet_vertex( face_i, 0 );
            Vertex* v2 = cell-> facet_vertex( face_i, 1 );
            Vertex* v3 = cell-> facet_vertex( face_i, 2 );

            Vertex* vertex = v2 ;

            const Star& star = star_[vertex] ;

            for( 
                Star::const_iterator itc = star.begin();
                itc != star.end(); itc++ 
            ) {
                Cell* neighbor_cell = *(itc);

                if( neighbor_cell == cell ) continue;

                Vertex* neighbor_v1 = nil;
                Vertex* neighbor_v3 = nil;

                for( 
                    unsigned int neigh_face_i=0;
                    neigh_face_i<neighbor_cell->nb_facets(); ++neigh_face_i 
                ) {

                    if(neighbor_cell->nb_vertices_in_facet(neigh_face_i) != cell->nb_vertices_in_facet(face_i)) {
//                      std::cerr << "Warning: potential unconformity" << std::endl ;
                        continue ;
                    }
                    
                    // get_dcel returns false if the face does
                    //  not contain the vertex.
                    if(
                        !neighbor_cell->get_dcel( 
                            neigh_face_i, v2, 
                            neighbor_v1, neighbor_v3
                        )
                    ) {
                        continue ;
                    }
                    
                    if( 
                        (neighbor_v1 == v1 && neighbor_v3 == v3) 
                        || 
                        (neighbor_v3 == v1 && neighbor_v1 == v3) 
                    ) {
                        set_cell_adjacent(cell, face_i, neighbor_cell) ;
                        set_cell_adjacent(neighbor_cell, neigh_face_i, cell) ;
                        break;
                    }
                }
            }
        }
    }


    //___________________________ Finite state automaton _________________________

    void CGraphBuilder::check_description_state(description_state expected) {
        if(desc_state_ != expected) {
            std::cerr << 
                "CGraph: Description function called in the wrong state"
                      << std::endl ;
            std::cerr << " CGraph descriptor should be " 
                      << description_state_to_string(expected) ;
            std::cerr << std::endl ;
            std::cerr << " and is " << description_state_to_string(desc_state_) ;
            std::cerr << std::endl ;
            std::cerr << "   Aborting ... " << std::endl ;
            abort() ;
        }
    }

    std::string CGraphBuilder::description_state_to_string(
        description_state desc
    ) {
        switch(desc) {
        case desc_initial:
            return "initial" ;
            break ;
        case desc_volume:
            return "volume" ;
            break ;
        case desc_meta_cell:
            return "meta cell" ;
            break ;
        case desc_meta_facet:
            return "meta facet" ;
            break ;
        case desc_cell:
            return "cell" ;
            break ;
        case desc_final:
            return "final" ;
            break ;
        default:
            return "unknown state" ;
            break ;
        }
    }

//------------------------------------------------------------------------

    void CGraphBuilder::build_meta_tetrahedron() {
        begin_meta_cell(4) ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(1) ;
        add_vertex_to_meta_facet(3) ;
        add_vertex_to_meta_facet(2) ;
        end_meta_facet() ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(0) ;
        add_vertex_to_meta_facet(2) ;
        add_vertex_to_meta_facet(3) ;
        end_meta_facet() ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(3) ;
        add_vertex_to_meta_facet(1) ;
        add_vertex_to_meta_facet(0) ;
        end_meta_facet() ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(0) ;
        add_vertex_to_meta_facet(1) ;
        add_vertex_to_meta_facet(2) ;
        end_meta_facet() ;

        end_meta_cell() ;
    }
    
    void CGraphBuilder::build_meta_hexahedron() {
        begin_meta_cell(8) ;
        
        begin_meta_facet() ;
        add_vertex_to_meta_facet(0) ;
        add_vertex_to_meta_facet(4) ;
        add_vertex_to_meta_facet(6) ;
        add_vertex_to_meta_facet(2) ;
        end_meta_facet() ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(3) ;
        add_vertex_to_meta_facet(7) ;
        add_vertex_to_meta_facet(5) ;
        add_vertex_to_meta_facet(1) ;
        end_meta_facet() ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(1) ;
        add_vertex_to_meta_facet(5) ;
        add_vertex_to_meta_facet(4) ;
        add_vertex_to_meta_facet(0) ;
        end_meta_facet() ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(2) ;
        add_vertex_to_meta_facet(6) ;
        add_vertex_to_meta_facet(7) ;
        add_vertex_to_meta_facet(3) ;
        end_meta_facet() ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(1) ;
        add_vertex_to_meta_facet(0) ;
        add_vertex_to_meta_facet(2) ;
        add_vertex_to_meta_facet(3) ;
        end_meta_facet() ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(4) ;
        add_vertex_to_meta_facet(5) ;
        add_vertex_to_meta_facet(7) ;
        add_vertex_to_meta_facet(6) ;
        end_meta_facet() ;

        end_meta_cell() ;
    }

    void CGraphBuilder::build_meta_prism() {
        begin_meta_cell(6) ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(0) ;
        add_vertex_to_meta_facet(1) ;
        add_vertex_to_meta_facet(2) ;
        end_meta_facet() ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(3) ;
        add_vertex_to_meta_facet(5) ;
        add_vertex_to_meta_facet(4) ;
        end_meta_facet() ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(0) ;
        add_vertex_to_meta_facet(3) ;
        add_vertex_to_meta_facet(4) ;
        add_vertex_to_meta_facet(1) ;
        end_meta_facet() ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(0) ;
        add_vertex_to_meta_facet(2) ;
        add_vertex_to_meta_facet(5) ;
        add_vertex_to_meta_facet(3) ;
        end_meta_facet() ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(1) ;
        add_vertex_to_meta_facet(4) ;
        add_vertex_to_meta_facet(5) ;
        add_vertex_to_meta_facet(2) ;
        end_meta_facet() ;

        end_meta_cell() ;
    }

    void CGraphBuilder::build_meta_pyramid() {
        begin_meta_cell(5) ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(0) ;
        add_vertex_to_meta_facet(1) ;
        add_vertex_to_meta_facet(2) ;
        add_vertex_to_meta_facet(3) ;
        end_meta_facet() ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(0) ;
        add_vertex_to_meta_facet(4) ;
        add_vertex_to_meta_facet(1) ;
        end_meta_facet() ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(0) ;
        add_vertex_to_meta_facet(3) ;
        add_vertex_to_meta_facet(4) ;
        end_meta_facet() ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(2) ;
        add_vertex_to_meta_facet(4) ;
        add_vertex_to_meta_facet(3) ;
        end_meta_facet() ;

        begin_meta_facet() ;
        add_vertex_to_meta_facet(2) ;
        add_vertex_to_meta_facet(1) ;
        add_vertex_to_meta_facet(4) ;
        end_meta_facet() ;

        end_meta_cell() ;
    }


}

