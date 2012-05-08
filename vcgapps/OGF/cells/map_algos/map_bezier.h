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

#ifndef __OGF_CELLS_MAP_ALGO_MAP_BEZIER__
#define __OGF_CELLS_MAP_ALGO_MAP_BEZIER__

#include <OGF/cells/map/map.h>

namespace OGF {

    /**
     * Computes bezier control meshes for geometry and normals.
     * Geometry: cubic bezier patches 
     * Normals: quadratic bezier patches (decoupled from the geometry)
     * Reference: A. Vlachos, J. Peters, C. Boyd, and J. Mitchell. 
     * Curved PN triangles. In ACM Symposium on Interactive 3D Graphics 2001, 
     * pages 159 166, 2001.
     */
    class CELLS_API MapBezier {
    public:
        MapBezier(Map* map) : map_(map) { }
        void compute_bezier_control_mesh() ;

    private:
        Map* map_ ;
    } ;

} 

#endif
