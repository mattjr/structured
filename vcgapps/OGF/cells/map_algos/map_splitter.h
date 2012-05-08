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

#ifndef __OGF_CELLS_MAP_ALGOS_MAP_SPLITTER__
#define __OGF_CELLS_MAP_ALGOS_MAP_SPLITTER__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map_algos/map_components.h>
#include <OGF/basic/types/counted.h>
#include <OGF/basic/types/smart_pointer.h>

namespace OGF {

    class CELLS_API MapComponentSplitter : public Counted, public MapComponentMutator {
    public:
        virtual bool split_component(
            MapComponent* component, 
            std::vector<MapComponent_var>& charts
        ) = 0 ;
        virtual ~MapComponentSplitter() ;

    protected:
        int nb_charts(MapComponent* component) ;
        void unglue_charts(MapComponent* component) ;
        int chart(Map::Halfedge* h) {
            if(h->is_border()) {
                h = h->opposite() ;
            }
            return chart_[h->facet()] ;
        }
        void get_charts(MapComponent* component, std::vector<MapComponent_var>& charts) ;
        
    protected:
        MapFacetAttribute<int> chart_ ;
    } ;

    typedef SmartPointer<MapComponentSplitter> MapComponentSplitter_var ;

}

#endif
