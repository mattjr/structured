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

#include <OGF/cells/vertex_set/vertex_set.h>

namespace OGF {
    
    VertexSet::VertexSet() {
    }

    VertexSet::~VertexSet() {
    }

    void VertexSet::clear() {
        vertices_.clear() ;
        vertex_attribute_manager_.clear() ;
    }
    
    void VertexSet::erase_all() {
        clear() ;
    }
    

    VertexSet::Vertex* VertexSet::new_vertex() {
        Vertex* result = vertices_.create() ;
        vertex_attribute_manager_.new_record(result) ;
        // TODO: notify_add_vertex(result) if we
        // add VertexObserver
        return result ;
    }
    
    VertexSet::Vertex* VertexSet::new_vertex(const Point3d& p) {
        Vertex* result = new_vertex() ;
        result->set_point(p) ;
        return result ;
    }
    
    void VertexSet::delete_vertex(Vertex* v) {
        // TODO: notify_remove_vertex(v) if we
        // add VertexObserver
        vertex_attribute_manager_.delete_record(v) ;
        vertices_.destroy(v) ;
    }
}
