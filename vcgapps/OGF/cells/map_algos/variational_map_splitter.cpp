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

#include <OGF/cells/map_algos/variational_map_splitter.h>
#include <OGF/cells/map_algos/map_approx.h>
#include <OGF/cells/map_algos/map_partition_smoother.h>

namespace OGF {

    VariationalMapComponentSplitter::VariationalMapComponentSplitter() {
        error_decrease_factor_ = 0.75 ;
        max_components_  = 10 ;
        smooth_ = false ;
    }

    bool VariationalMapComponentSplitter::split_component(
        MapComponent* component, 
        std::vector<MapComponent_var>& charts
    ) {
        typedef MapVariationalApprox<
            L12LinearProxy, L12LinearProxyFitter, MapComponent
        > Approximer ;
        is_locked_.bind(component->map()) ;
        charts.clear() ;
        std::cerr << "Splitting component ...." << std::endl ;
        Approximer approx(component) ;
        chart_.bind(component->map(), "chart") ;
        approx.init(1,0) ;
        double initial_error = approx.optimize(1) ;
        double expected_error = initial_error * error_decrease_factor_ ;

        int max_components = max_components_ ;

        approx.add_charts(max_components, 4, expected_error, 2) ;
        smooth_partition(component, chart_, is_locked_, 100) ;
        
        while(nb_charts(component) < 2) {
            std::cerr << "Did not manage to create charts, relaunching ..."
                      << std::endl ;
            expected_error *= 0.5 ;
            max_components++ ;
            approx.add_charts(max_components, 4, expected_error, 2) ;
            if(smooth_) {
                smooth_partition(component, chart_, is_locked_, 100) ;
            }
        }
        unglue_charts(component) ;
        get_charts(component, charts) ;
        std::cerr << "Created " << charts.size() << " charts" << std::endl ;
        chart_.unbind() ;
        is_locked_.unbind() ;
		return true ;
    }


    SmoothVariationalMapComponentSplitter::SmoothVariationalMapComponentSplitter(
    ) : VariationalMapComponentSplitter() {
        smooth_ = true ;
    }

}
