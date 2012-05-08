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

#include <OGF/cells/map_algos/map_splitter.h>
#include <OGF/cells/map/map_editor.h>
#include <OGF/basic/containers/arrays.h>
#include <set>

namespace OGF {

    MapComponentSplitter::~MapComponentSplitter() {
    }

    int MapComponentSplitter::nb_charts(MapComponent* component) {
        int max_chart = -1 ;
        FOR_EACH_FACET(MapComponent, component, it) {
            max_chart = ogf_max(max_chart, chart_[it]) ;
        }
        Array1d<int> histo(max_chart + 1) ;
        histo.set_all(0) ;
        FOR_EACH_FACET(MapComponent, component, it) {
            histo(chart_[it])++ ;
        }            
        int result = 0 ;
        for(unsigned int i=0; i<histo.size(); i++) {
            if(histo(i) != 0) {
                result++ ;
            }
        }
        return result ;
    }
    
    void MapComponentSplitter::unglue_charts(MapComponent* component) {
        MapEditor editor(component->map()) ;
        std::vector<Map::Halfedge*> to_unglue ;
        FOR_EACH_EDGE(MapComponent, component, it) {
            if(
                !it->is_border_edge() && 
                chart_[it->facet()] != chart_[it->opposite()->facet()]
            ) {
                to_unglue.push_back(it) ;
            }
        }
        for(unsigned int i=0; i<to_unglue.size(); i++) {
            Map::Halfedge* h = to_unglue[i] ;
            editor.unglue(h, false) ;
        }
    }
    
    void MapComponentSplitter::get_charts(MapComponent* component, std::vector<MapComponent_var>& charts) {
        
        charts.clear() ;
        
        int max_chart_id = -1 ;
        FOR_EACH_FACET(MapComponent, component, it) {
            max_chart_id = ogf_max(max_chart_id, chart_[it]) ;
        }
        
        for(int i=0; i<=max_chart_id; i++) {
            charts.push_back(new MapComponent(component->map())) ;
        }
        
        std::set<Map::Vertex*> border_vertices ;
        
        FOR_EACH_FACET(MapComponent, component, it) {
            ogf_assert(it != nil) ;
            set_target(charts[chart_[it]]) ;
            facets().push_back(it) ;
            Map::Halfedge* h = it->halfedge() ;
            do {
                halfedges().push_back(h) ;
                if(h->opposite()->is_border()) {
                    halfedges().push_back(h->opposite()) ;
                    Map::Vertex* v = h->vertex() ;
                    if(border_vertices.find(v) == border_vertices.end()) {
                        border_vertices.insert(v) ;
                        vertices().push_back(v) ;
                    }
                }
                h = h->next() ;
            } while(h != it->halfedge()) ;
        }
        
        FOR_EACH_VERTEX(MapComponent, component, it) {
            if(!it->is_on_border()) {
                set_target(charts[chart(it->halfedge())]) ;
                vertices().push_back(it) ;
            }
        }
    }
    

}
