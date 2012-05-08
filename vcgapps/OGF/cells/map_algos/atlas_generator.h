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

#ifndef __ATLAS_GENERATOR__
#define __ATLAS_GENERATOR__

#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map_algos/map_components.h>
#include <OGF/cells/map_algos/map_parameterizer.h>
#include <OGF/cells/map_algos/map_splitter.h>

#include <deque>

namespace OGF {

//______________________________________________________________________


    class CELLS_API AtlasGenerator {
    public :
        AtlasGenerator(Map* map) ;
        void apply() ;

        bool get_unglue_hardedges() { return unglue_hardedges_ ; }
        void set_unglue_hardedges(bool b) { unglue_hardedges_ = b ; }

        void set_parameterizer(const std::string& parameterizer_name) ;
        MapParameterizer* parameterizer() const { return parameterizer_ ; }

        double get_max_overlap_ratio() const { return max_overlap_ratio_ ; }
        void set_max_overlap_ratio(double x) { max_overlap_ratio_ = x ; }
        double get_max_scaling() const { return max_scaling_ ; }
        void set_max_scaling(double x) { max_scaling_ = x ; }
        double get_min_fill_ratio() const { return min_fill_ratio_ ; }
        void set_min_fill_ratio(double x) { min_fill_ratio_ = x ; }

        bool get_auto_cut() const { return auto_cut_ ; }
        void set_auto_cut(bool x) { auto_cut_ = x ; }
        
        bool get_auto_cut_cylinders() const { return auto_cut_cylinders_ ; }
        void set_auto_cut_cylinders(bool x) { auto_cut_cylinders_ = x ; }

        bool get_pack() const { return pack_ ; }
        void set_pack(bool x) { pack_ = x ; }

        void set_max_iter(int x) { max_iter_ = x ; }
        int get_max_iter() const { return max_iter_ ; }

        void set_splitter(MapComponentSplitter* splitter) { splitter_ = splitter ; }
        void set_splitter(const std::string& name) ;

    protected:

        /**
         * Performs topological pre-processing, and 
         * initializes active_components_
         */
        void get_components() ;

        /**
         * Just to keep track of the iteration number.
         */
        struct ActiveComponent {
            ActiveComponent(int i = 0) : iter(i) { }
            int iter ;
            MapComponent_var component ;
        } ;

        /**
         * Pushes the specified component on active_components_,
         * and increments its iteration number.
         */
        void activate_component(MapComponent* comp, int iter = 0) ;
        void split_component(MapComponent* comp, int iter = 0) ;
        void split_thin_component(MapComponent* comp, int iter = 0) ;

        Map* map_ ;
        MapParameterizer_var parameterizer_ ;

        std::deque<ActiveComponent> active_components_ ;
        int max_iter_ ;

        int total_faces_ ;
        int parameterized_faces_ ;

        MapVertexLock is_locked_ ;
        bool unglue_hardedges_ ;
        
        double max_overlap_ratio_ ;
        double max_scaling_ ;
        double min_fill_ratio_ ;
        
        bool auto_cut_ ;
        bool auto_cut_cylinders_ ;

        bool pack_ ;

        MapComponentSplitter_var splitter_ ;

        // For partition smoother
        MapVertexAttribute<bool> vertex_is_locked_ ;
    } ;
  
//______________________________________________________________________

}

#endif
