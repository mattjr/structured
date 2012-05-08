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
 
#ifndef __SCGRAPH__
#define __SCGRAPH__

#include <OGF/cells/common/common.h>
#include <OGF/cells/cgraph/cgraph_cells.h>
#include <OGF/basic/containers/arrays.h>


#ifdef OGF_PARANOID
#define scgraph_assert(x) ogf_assert(x)
#else
#define scgraph_assert(x)
#endif

namespace OGF {

//_________________________________________________________

    /**
     * Structured Cellular Graph : an implementation
     * of Cellular Graph specialized for structured grids.
     */
    class CELLS_API SCGraph {
    public:


        typedef CGraphTypes::VertexCell VertexCell ;
        typedef CGraphTypes::VertexCell Vertex ;
        typedef CGraphTypes::VertexCell Cell ;

        typedef int cell_id ;
        typedef int vertex_id ;
        typedef cell_id Cell_iterator ;
        typedef vertex_id Vertex_iterator ;

        SCGraph() ;

        //__________ description __________

        /**
         * nu, nv, nw are dimensions in vertices
         */
        void allocate(int nu, int nv, int nw) ;
        void set_point(int u, int v, int w, const Point3d& p) ;
        void remove_cell(int u, int v, int w) ;

        int nu() const { return user_nu_ ; }
        int nv() const { return user_nv_ ; }
        int nw() const { return user_nw_ ; }

        //__________ access _______________

        Cell_iterator cells_begin() const   { return 0 ; }
        Cell_iterator cells_end() const { return nuvw_ ; }

        Vertex_iterator vertices_begin() const   { return 0 ; }
        Vertex_iterator vertices_end() const { return nuvw_ ; }

        Cell& cell(cell_id c) {
            return store_.from_linear_index(c) ;
        }

        Vertex& vertex(vertex_id c) {
            return store_.from_linear_index(c) ;
        }

        const Cell& cell(cell_id c) const {
            return store_.from_linear_index(c) ;
        }

        const Vertex& vertex(vertex_id c) const {
            return store_.from_linear_index(c) ;
        }


        cell_id get_cell_id(int u, int v, int w) const {
            scgraph_assert(u >= 0 && u < user_nu_) ;
            scgraph_assert(v >= 0 && v < user_nv_) ;
            scgraph_assert(w >= 0 && w < user_nw_) ;
            int result =  
                (u+1) + store_.size(0)*((v+1) + store_.size(1)*(w+1)) ;
            scgraph_assert(
                result >= 0 && 
                result < store_.size(0)*store_.size(1)*store_.size(2)
            ) ;
            return result ;
        }

        void get_cell_uvw(cell_id c, int& u, int& v, int& w) const {
            scgraph_assert(c >= 0 && c < nuvw_) ;
            u = c % store_.size(0) ;
            c = c / store_.size(0) ;
            v = c % store_.size(1) ;
            w = c / store_.size(1) ;
            u-- ; v--; w-- ;
        }

        cell_id adjacent_cell(cell_id c, int face) const {
            scgraph_assert(c >= 0 && c < nuvw_) ;
            scgraph_assert(face >= 0 && face < 6) ;
//            scgraph_assert(!cell(c).is_ghost()) ;
            return c + adjacent_offset_[face] ;
        }
        
        vertex_id cell_vertex(cell_id c, int vertex) const {
            scgraph_assert(c >= 0 && c < nuvw_) ;
            scgraph_assert(vertex >= 0 && vertex < 8) ;
//            scgraph_assert(!cell(c).is_ghost()) ;
            return c + vertex_offset_[vertex] ;
        }

    private:
        CGraphTypes::MetaCell meta_cell_ ;

    private:
        Array3d<VertexCell> store_ ;
        int nuvw_ ;
        int user_nu_ ;
        int user_nv_ ;
        int user_nw_ ;
        int vertex_offset_[8] ;
        int adjacent_offset_[6] ;
    } ;

//_________________________________________________________

}
#endif

