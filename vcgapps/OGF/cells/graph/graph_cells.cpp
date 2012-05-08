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
 

#include <OGF/cells/graph/graph_cells.h>

namespace OGF {

//_________________________________________________________

    namespace GraphTypes {

        //______________________________________

        bool Vertex::is_valid() const {
            return halfedge()->vertex() == this ;
        }

        void Vertex::assert_is_valid() const {
            ogf_assert(halfedge()-> vertex() == this) ;
        }

        int Vertex::degree() const {
            int result = 0 ;
            Halfedge* it = halfedge() ;
            do {
                result++ ;
                it = it->next_around_vertex() ;
            } while(it != halfedge()) ;
            return result ;
        }

        bool Vertex::is_extremity() const {
            return (degree() == 1) ;
        }

        //______________________________________

        bool Halfedge::is_valid() const {
            return (
                (opposite()-> opposite() == this) &&
                (next_around_vertex()->prev_around_vertex() == this) &&
                (prev_around_vertex()->next_around_vertex() == this) 
            ) ;
        }

        void Halfedge::assert_is_valid() const {
            ogf_assert(opposite()-> opposite() == this) ;
            ogf_assert(next_around_vertex()->prev_around_vertex() == this) ;
            ogf_assert(prev_around_vertex()->next_around_vertex() == this) ;
        }

    }

//_________________________________________________________

}

