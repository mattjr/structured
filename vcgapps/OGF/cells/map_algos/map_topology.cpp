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
 

#include <OGF/cells/map_algos/map_topology.h>
#include <OGF/cells/map_algos/map_components.h>

namespace OGF {

//_________________________________________________________

    MapComponentTopology::MapComponentTopology(const MapComponent* comp) {
        component_ = comp ;

        // Compute number_of_borders_

        Attribute<Map::Halfedge,bool> is_marked(
            comp->map()->halfedge_attribute_manager()
        ) ;

        number_of_borders_ = 0 ;
        largest_border_size_ = 0 ;
        
        { FOR_EACH_HALFEDGE_CONST(MapComponent, component_, it) {
            const Map::Halfedge* cur = it ;
            if(cur->is_border() && !is_marked[cur]) {
                number_of_borders_++ ;
                int border_size = 0 ;
                do {
                    border_size++ ;
                    is_marked[cur] = true ;
                    cur = cur->next() ;
                } while(cur != it) ;
                largest_border_size_ = ogf_max(
                    largest_border_size_, border_size
                ) ;
            }
        }}
    }

    int MapComponentTopology::xi() const {
        // xi = #vertices - #edges + #facets
        // #edges = #halfedges / 2 
        return 
            int(component_->size_of_vertices())        - 
            int(component_->size_of_halfedges() / 2)   +
            int(component_->size_of_facets())          ;
    }

    bool MapComponentTopology::is_almost_closed(int max_border_size) const {
        if(component_->size_of_facets() == 1) {
            return false ;
        }
        return largest_border_size_ <= max_border_size ;
    }

//_________________________________________________________

}

