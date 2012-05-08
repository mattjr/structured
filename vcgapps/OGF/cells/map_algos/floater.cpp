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

#include <OGF/cells/map_algos/floater.h>
#include <OGF/cells/map_algos/map_components.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/math/numeric/linear_solver.h>

namespace OGF {

    MapParameterizerFloater::MapParameterizerFloater() {
    }

    bool MapParameterizerFloater::do_parameterize_disc(Map* map) {

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
        
        // Step 1 : find the two vertices to pin
        Map::Halfedge* h = largest_border() ;
        bool closed = (h == nil) ;
        ogf_assert(!closed) ;

        // Step 2 : construct a Sparse Least Squares solver
        LinearSolver solver(2 * nb_distinct_vertices_) ;
        solver.set_least_squares(false) ;
        solver.set_symmetric(false) ;
        SystemSolverParameters params ;
		params.set_arg_value("method", std::string("BICGSTAB")) ;
        solver.set_system_solver(params) ;

        // Step 3 : Pin the border
        project_border_on_square(h, use_locked_vertices_as_corners_) ;
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

    void MapParameterizerFloater::setup_barycentric_relations( LinearSolver& solver ) {
        FOR_EACH_VERTEX(Map, map_, it) {
            if(!it->is_on_border()) {
                setup_barycentric_relations(solver, it) ;
            }
        }
    }
    
    void MapParameterizerFloater::setup_barycentric_relations(
        LinearSolver& solver, Map::Vertex* v
    ) {
        for(int i=0; i<2; i++) {
            solver.begin_row() ;
            solver.set_right_hand_side(0.0) ;
            solver.add_coefficient(2 * vertex_id_[v] + i, -double(v->degree())) ;

            Map::Halfedge* h = v->halfedge() ;
            do {
                Map::Vertex* w = h->opposite()->vertex() ;
                solver.add_coefficient(2 * vertex_id_[w] + i, 1.0) ;
                h = h->next_around_vertex() ;
            } while(h != v->halfedge()) ;

            solver.end_row() ;
        }
    }

//------------------------------------------------------------------------------------

    void MapParameterizerMVC::setup_barycentric_relations(
        LinearSolver& solver, Map::Vertex* it
    ) {
        for(int coord=0; coord<2; coord++) {

            double S = 0 ;
                
            solver.begin_row() ;
            solver.set_right_hand_side(0.0) ;
                
            Map::Halfedge* jt = it->halfedge() ;
            do {
                // Mean value coordinates :
                Vector3d v0 = Geom::vector(jt->opposite()->prev()->opposite());
                Vector3d v1 = Geom::vector(jt->opposite());
                Vector3d v2 = Geom::vector(jt->next());
                double dist = v1.norm();
                v0.normalize();
                v1.normalize();
                v2.normalize();
                double alpha_i_0 = acos(v0*v1);
                double alpha_i_1 = acos(v1*v2);
		    
		    
                double coeff = (
                      tan(alpha_i_0 / 2.0) 
                    + tan(alpha_i_1 / 2.0)
                ) / dist;


                if (!(coeff <500000 && coeff >0)){
                    coeff =500000;
                }
		
                S += coeff ;
                    
                solver.add_coefficient(
                    2 * vertex_id_[jt-> opposite()-> vertex()] + coord, coeff
                ) ;

                jt = jt->next_around_vertex() ;
            } while(jt != it->halfedge()) ;
                
            
            solver.add_coefficient(2 * vertex_id_[it] + coord, -S) ;
            solver.normalize_row() ;
            
            solver.end_row() ;
        }
    }

}

