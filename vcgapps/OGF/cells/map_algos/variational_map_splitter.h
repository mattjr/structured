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

#ifndef __OGF_CELLS_MAP_ALGOS_VAR_MAP_SPLITTER__
#define __OGF_CELLS_MAP_ALGOS_VAR_MAP_SPLITTER__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map_algos/map_splitter.h>

namespace OGF {

    class CELLS_API VariationalMapComponentSplitter : public MapComponentSplitter {
    public:
        VariationalMapComponentSplitter() ;
        virtual bool split_component(
            MapComponent* component, 
            std::vector<MapComponent_var>& charts
        ) ;
        void set_error_decrease_factor(double x) { error_decrease_factor_ = x ; }
        void set_max_components(int x) { max_components_ = x ; }
        
    protected:
        double error_decrease_factor_ ;
        int max_components_ ;
        MapVertexLock is_locked_ ;
        bool smooth_ ;
    } ;

    class CELLS_API SmoothVariationalMapComponentSplitter : public VariationalMapComponentSplitter {
    public:
        SmoothVariationalMapComponentSplitter() ;
    } ;

}

#endif
