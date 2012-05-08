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

#include <OGF/cells/map_algos/atlas_generator.h>
#include <OGF/cells/types/cells_library.h>
#include <OGF/cells/map/map.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/cells/map/map_editor.h>
#include <OGF/cells/map_algos/map_components.h>
#include <OGF/cells/map_algos/map_topology.h>
#include <OGF/cells/map_algos/map_cutter.h>
#include <OGF/cells/map_algos/map_parameterizer.h>
#include <OGF/cells/map_algos/packer.h>
#include <OGF/cells/map_algos/lscm.h>
#include <OGF/cells/map_algos/map_approx.h>
#include <OGF/cells/map_algos/map_partition_smoother.h>
#include <OGF/cells/map_algos/enumerate.h>
#include <OGF/math/numeric/eigen_solver.h>
#include <OGF/math/types/math_library.h>
#include <OGF/basic/containers/arrays.h>
#include <OGF/basic/debug/logger.h>
#include <OGF/basic/debug/progress.h>
#include <OGF/basic/os/stopwatch.h>

#include <deque>

// TODO: when validator fails with filling factor,
//    use another cutting strategy ...

namespace OGF {
    
//___________________________________________________________________________

    // Default treshold corresponds to a cube minus one
    //  of its faces.
    static double default_sock_treshold = 5.0 / (4.0 * 4.0) ;

    static bool component_is_sock(
        MapComponent* comp, double treshold = default_sock_treshold
    ) {
        double area      = Geom::component_area(comp) ;
        double perimeter = Geom::component_border_length(comp) ;
        if(::fabs(perimeter) < 1e-30) {
            return false ;
        }
        // Note: perimeter is squared to be scale invariant 
        double ratio = area / (perimeter * perimeter) ;
        return (ratio > treshold) ;
    }

    AtlasGenerator::AtlasGenerator(Map* map) : map_(map), vertex_is_locked_(map) { 
        max_iter_ = -1; 
        is_locked_.bind(map_) ;
        unglue_hardedges_ = true ;
        max_overlap_ratio_ = 0.005 ;
        max_scaling_ = 20.0 ;
        min_fill_ratio_ = 0.25 ;
        auto_cut_ = true ;
        auto_cut_cylinders_ = true ;
        pack_ = true ;
        parameterizer_ = new MapParameterizerLSCM ;
        set_splitter("VSA") ;
    }

    void AtlasGenerator::set_splitter(const std::string& name) {
        set_splitter(CellsLibrary::instance()->create_map_component_splitter(name)) ;
        ogf_assert(splitter_ != nil) ;
    }

    void AtlasGenerator::set_parameterizer(const std::string& name) {
        parameterizer_ = CellsLibrary::instance()->create_map_parameterizer(name) ;
        ogf_assert(parameterizer_ != nil) ;
    }

    void AtlasGenerator::activate_component(
        MapComponent* comp, int iter 
    ) {
        if(comp->size_of_facets() == 0) {
            return ;
        }
        if(max_iter_ >= 0 && iter >= max_iter_) {
            std::cerr << "Component depth exceeded max_iter=" << max_iter_ << std::endl ;
            return ;
        }
        ActiveComponent i(iter+1) ;
        active_components_.push_back(i) ;
        active_components_.back().component = comp ;
    }

    void AtlasGenerator::split_component(MapComponent* component, int iter) {
        std::vector<MapComponent_var> charts ;
        if(splitter_->split_component(component, charts)) {
            for(unsigned int i=0; i<charts.size(); i++) {
                activate_component(charts[i],iter) ;
            }
        } else {
            split_thin_component(component, iter) ;
        }
        update_graphics(map_) ;
    }

    void AtlasGenerator::split_thin_component(MapComponent* component, int iter) {
        Logger::out("AtlasGenerator") << "cutting thin component" << std::endl ;
        MapSplitter splitter(component) ;
        splitter.set_auto_cut_cylinders(auto_cut_cylinders_) ;
        splitter.apply(MapSplitter::FURTHEST_PAIR) ;
        Logger::out("AtlasGenerator") << "...cut" << std::endl ;
        
        MapComponentsExtractor extractor ;
        
        MapComponent* c0 = extractor.extract_component(
            map_, splitter.last_seed(0) 
        ) ;

        MapComponent* c1 = extractor.extract_component(
            map_, splitter.last_seed(1) 
        ) ;

        activate_component(c0,iter) ;
        activate_component(c1,iter) ;
    }

    void AtlasGenerator::apply() {
        SystemStopwatch watch ;

        if(!parameterizer()->use_locked_vertices_as_corners()) {
            FOR_EACH_VERTEX(Map, map_, it) {
                is_locked_[it] = false ;
            }
        }

        if(unglue_hardedges_) {
            HardEdgeCutter cutter(map_) ;
            cutter.apply() ;
        }

        parameterized_faces_ = 0 ;
        total_faces_ = map_->size_of_facets() ;

        ProgressLogger progress(total_faces_) ;

        get_components() ;
        while(!active_components_.empty()) {
            int iter = active_components_.front().iter ;
            MapComponent_var cur = active_components_.front().component ;
            active_components_.pop_front() ;
            Logger::out("AtlasGenerator")
                << "Processing component, iter=" << iter 
                << " size=" << cur->size_of_facets() 
                << std::endl ;


            bool thin_component = false ;
            bool is_valid = parameterizer_->parameterize_disc(cur) ;
            if(is_valid && auto_cut_) {
                ParamValidator validator ;
                validator.set_max_overlap_ratio(max_overlap_ratio_) ;
                validator.set_max_scaling(max_scaling_) ;
                validator.set_min_fill_ratio(min_fill_ratio_) ;
                is_valid = validator.component_is_valid(cur) ;
                thin_component = validator.fill_ratio() < min_fill_ratio_ ;
            }
            if(is_valid) {
                parameterized_faces_ += cur->size_of_facets() ;
            } else {
                if(false && thin_component) {  // [Bruno]
                    split_thin_component(cur, iter) ;
                } else {
                    split_component(cur, iter) ;
                }
            }
            
            Logger::out("AtlasGenerator")
                << "====> "
                << parameterized_faces_ << "/" << total_faces_
                << " parameterized ("
                << int(
                    double(parameterized_faces_) * 100.0 / 
                    double(total_faces_)
                ) 
                << "%)" << std::endl ;
            if(progress.is_canceled()) {
                return ;
            }
            progress.notify(parameterized_faces_) ;
        }
        
        FOR_EACH_VERTEX(Map, map_, it) {
            is_locked_[it] = false ;
        }
        
        if(pack_ && max_iter_ < 0) {
            Packer packer ;
            packer.pack_map(map_) ;
        }        

        Logger::out("AtlasGenerator") 
            << "total elapsed time: " << watch.elapsed_user_time()
            << std::endl ;
    }


    void AtlasGenerator::get_components() {
        
        // Topological precomputing : make discs
        MapSplitter splitter(map_) ;
        splitter.set_auto_cut_cylinders(auto_cut_cylinders_) ;

        MapComponentsExtractor extractor ;
        MapComponentList components = 
            extractor.extract_components(map_) ;
        
        for(
            MapComponentList::iterator it = components.begin(); 
            it != components.end(); it++
        ) {
            MapComponent* cur = (*it) ;
            MapComponentTopology topology(cur) ;
            if( topology.is_disc() && !topology.is_almost_closed()) {
                Logger::out("AtlasGenerator")
                    << "component is disc"
                    << std::endl ;
                activate_component(cur) ;
            } else {
                Logger::out("AtlasGenerator")
                    << "Processing non-disc component"
                    << std::endl ;
                Logger::out("AtlasGenerator")
                    << "   xi=" << topology.xi() 
                    << " #borders="  << topology.number_of_borders()
                    << " #vertices=" << cur->size_of_vertices()
                    << " #facets="   << cur->size_of_facets()
                    << " largest border size=" 
                    << topology.largest_border_size()
                    << std::endl ;
                if( 
                    (
                        topology.is_almost_closed() || 
                        (
                            topology.is_disc() &&
                            component_is_sock(cur)   
                        )
                    ) 
                ) {
                    Logger::out("AtlasGenerator")
                        << "   Almost closed component, cutting ..."
                        << std::endl ;
                    split_component(cur) ;
                } else if (
                    auto_cut_cylinders_ &&
                    topology.is_cylinder()
                ) {
                    Logger::out("AtlasGenerator")
                        << "   Cutting cylinder ..."
                        << std::endl ;
                    split_component(cur) ;
                } else {
                    //   If this is not a disc, not a closed surface
                    // and not a cylinder, this may be a surface with
                    // topological noise (some internal borders may be
                    // caused by some edges that the gluer failed to
                    // associate). In this case, we try to parameterize
                    // the component, and let the validator decide
                    // whether the component should be splitted.
                    Logger::out("AtlasGenerator")
                        << "   topological monster, try to parameterize"
                        << std::endl ;
                    activate_component(cur) ;
                }
            } 
        }
    }

//____________________________________________________________

}
