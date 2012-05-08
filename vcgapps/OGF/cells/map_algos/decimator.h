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
 

#ifndef __CELLS_MAP_ALGOS_DECIMATOR__
#define __CELLS_MAP_ALGOS_DECIMATOR__

#include <OGF/cells/map/map_editor.h>
#include <OGF/cells/map/map_cell_heap.h>  

namespace OGF{

    class Progress ;
    
    class CELLS_API Decimator {
    public:
        enum Strategy {
            LENGHT_BASED,
            VOLUME_AND_BORDER_BASED,
            ANGULAR_DEFAULT
        } ;
    
        Decimator(Map* s);
        virtual ~Decimator() ;
        virtual void collapse_edge(Map::Halfedge* h);
        void set_proportion_to_remove(double d);
        void set_threshold(double threshold);
        void set_strategy(Strategy strat) { strategy_=strat; }    
        void apply();

        Progress* get_progress()    { return progress_ ; }
        void set_progress(Progress* p) { progress_ = p ; }

    protected:
        double edge_collapse_importance(Map::Halfedge* h);
        virtual bool can_collapse(Map::Halfedge* h);
        double compute_vertex_importance(Map::Vertex* v);


        double delta_volume_cause_by_collapse(Map::Halfedge* h);
        double delta_area_on_border_cause_by_collapse(Map::Halfedge* h);
        double delta_shape_cause_by_collapse(Map::Halfedge* h);
        double delta_angular_default_cause_by_collapse(Map::Halfedge* h);


        double threshold_;
        int nb_vertex_to_remove_;
        Attribute<Map::Vertex,double> importance_;
        Attribute<Map::Vertex,Map::Halfedge*> halfedge_to_collapse_;
        Attribute<Map::Vertex,double> accumulated_cost_;
        Attribute<Map::Vertex,double> vertex_density_ ;
        Map* map_;
        Strategy strategy_;
        Progress* progress_;
    } ;
 
}
  
#endif

