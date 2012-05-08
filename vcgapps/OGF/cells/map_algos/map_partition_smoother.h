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

#ifndef __OGF_CELLS_MAP_ALGOS_MAP_PARTITION_SMOOTHER__
#define __OGF_CELLS_MAP_ALGOS_MAP_PARTITION_SMOOTHER__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map_algos/map_approx.h>
#include <algorithm>

namespace OGF {

    class CELLS_API SmoothVertex {
    public:
        SmoothVertex(Map::Vertex* v, MapFacetAttribute<int>& chart) ;

        bool operator<(const SmoothVertex& rhs) const {
            return (delta_len_ < rhs.delta_len_) ;
        }

        bool apply(
            MapFacetAttribute<int>& chart, 
            MapVertexAttribute<bool>& locked
        ) ;

        bool is_valid() const { return is_valid_ ; }

    private:
        Map::Vertex* vertex_ ;
        int chart_id_ ;
        double delta_len_ ;
        bool is_valid_ ;
    } ;
    
    template <class MAP> void smooth_partition(
        MAP* map, MapFacetAttribute<int>& chart, MapVertexAttribute<bool>& locked,
        int max_iter
    ) {
        std::vector<SmoothVertex> smooth_vertices ;
        for(int iter = 0; iter < max_iter; iter++) {
            smooth_vertices.clear() ;
            FOR_EACH_VERTEX_GENERIC(MAP, map, it) {
                locked[it] = false ;
                SmoothVertex sv(it, chart) ;
                if(sv.is_valid()) {
                    smooth_vertices.push_back(sv) ;
                }
            }
            bool changed = false ;
            std::sort(smooth_vertices.begin(), smooth_vertices.end()) ;
            for(unsigned int i=0; i<smooth_vertices.size(); i++) {
                if(smooth_vertices[i].apply(chart, locked)) {
                    changed = true ;
                }
            }
            if(!changed) {
                break ;
            }
        }
    }

}

#endif

