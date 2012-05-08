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

#ifndef __OGF_CELLS_CGRAPH_ALGO_MARCHING_CUBES__
#define __OGF_CELLS_CGRAPH_ALGO_MARCHING_CUBES__

#include <OGF/cells/common/common.h>

namespace OGF {
    namespace MarchingCubes {

        /*
         *         4 ---- 5
         *                 
         *         |  W+  |
         *         |      |
         *                  
         *         6 ---- 7
         *                 
         *         |  V+  |
         *         |      |
         *                  
         *  6 ---- 2 ---- 3 ---- 7
         *
         *  |  U-  |  W-  |  U+  |
         *  |      |      |      |
         *
         *  4 ---- 0 ---- 1 ---- 5
         *
         *         |  V-  |
         *         |      |
         *   ^
         *   |     4------5
         *   V
         *   |
         *   0 -- U -- > (basis from point 0)
         *   |
         *   W
         *   |
         *   v
         * 
         */

        /**
         * Gives the two extremities of each edge of a cube
         */
        extern CELLS_API signed char edge_vertex[12][2] ;

        /**
         * Gives the number of intersected edges and the
         * list of intersected edges.
         */
        extern CELLS_API signed char cell_config[256][13] ;
    }
}

#endif
