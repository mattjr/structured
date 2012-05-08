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

#include <OGF/parameterizer/algos/circle_floater.h>
#include <OGF/cells/map_algos/map_components.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/math/numeric/linear_solver.h>

namespace OGF {

    bool MapParameterizerCircleFloater::do_parameterize_disc(Map* map) {

        map_ = map ;

        // Filter the case where there is a single triangle
        // Note: flat parts could be also detected here.
        // TODO: something smarter: we do not need the principal plane
        //  etc..., just ortho-normalize the triangle's basis and project
        //  on it...
        if(map_->size_of_facets() < 2 || 
            map_->size_of_vertices() < 4
        ) {
            get_bounding_box() ;
            project_on_principal_plane() ;
            return true ;
        }

        // Step 0 : normalization, vertex numbering, attributes
        begin_parameterization(map) ;
        
        if(map_->size_of_facets() < 2 || nb_distinct_vertices_ < 4) {
            project_on_principal_plane() ;
            end_parameterization() ;
            return true ;
        }
        
        // Step 1 : find the starting halfedge
        Map::Halfedge* h = nil ;
        FOR_EACH_HALFEDGE(Map, map, it) {
            if(it->is_border()){
                if((it->vertex()->point()-start_).norm()==0) { h = it; break ;}
            }
        }
		
        if(h==nil){
            std::cerr<<"no starting halfedge for circle floater"<<std::endl ;
            return false;
        }

        // Step 2 : construct a Sparse Least Squares solver
        LinearSolver solver(2 * nb_distinct_vertices_) ;
        solver.set_least_squares(false) ;
        solver.set_symmetric(false) ;
        SystemSolverParameters params ;
        params.set_arg_value("method", std::string("SUPERLU")) ;        
        solver.set_system_solver(params) ;

        // Step 3 : Pin the border
        project_border_on_circle(h) ;
        border_to_solver(solver, h) ;

        // Step 4 : build the linear system to solve
        solver.begin_system() ;
        setup_barycentric_relations(solver) ;
        solver.end_system() ;

        // Step 5 : solve the system and 
        //   copy the solution to the texture coordinates.
        if(solver.solve()) {
            solver_to_map(solver) ;
        } else {
            // If solving fails, try a projection.
            // Note that when used in the AtlasMaker,
            // if the result of the projection is invalid,
            // it will be rejected by the ParamValidator.
            project_on_principal_plane() ;
        }

        // Step 6 : unbind the attributes
        end_parameterization() ;

        return true ;
    }

}

