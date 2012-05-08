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
 
#include <iostream>
// iostream should be included before anything
// else, otherwise 'cin', 'cout' and 'cerr' will
// be uninitialized.

#ifndef OGF_cells_common_common_h
#define OGF_cells_common_common_h

#include <OGF/basic/common/common.h>
#ifdef CELLS_EXPORTS
#   define CELLS_API OGF_EXPORT
#else
#   define CELLS_API OGF_IMPORT
#endif

namespace OGF {

    class Map ;

    /**
     * If the map is a grob, updates the grahics display.
     * This is used to animate the algorithms (for educational
     * purposes)
     */
    void CELLS_API update_graphics(Map* map) ;

    typedef void (*update_map_graphics_func)(Map* map) ;
    /**
     * specifies the function to be used to update map graphics.
     * Using this callback mechanism avoids introducing a dependency
     * between cells and scene_graph. The surface module (that knows
     * about scene_graph) registers the right update_graphics function.
     */
    void CELLS_API set_update_map_graphics_func(update_map_graphics_func f) ;

    static class CELLS_API cells_libinit {
    public:
        cells_libinit() ;
        ~cells_libinit() ;
        
        static void increment_users() ;
        static void decrement_users() ;
        
        
    private:
        static void initialize() ;
        static void terminate() ;
        static int count_ ;
    } cells_libinit_instance ;
    
}

#endif
