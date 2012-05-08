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

#include <OGF/cells/map_algos/lscm.h>
#include <OGF/cells/map_algos/map_components.h>
#include <OGF/math/numeric/linear_solver.h>
#include <sstream>

namespace OGF {

    MapParameterizerLSCM::MapParameterizerLSCM() {
        user_locks_ = false ;
        nb_iter_ = 0 ;
    }

    bool MapParameterizerLSCM::do_parameterize_disc(Map* map) {

        map_ = map ;
        MapVertexLock is_locked(map_) ;

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
        ogf_assert(h != nil) ; // Happens if the part is closed.

        Point3d center ;
        Vector3d V1,V2 ;
        principal_axes(h, center, V1, V2) ;

        Map::Vertex* vx_min = nil ;
        Map::Vertex* vx_max = nil ;
        get_border_extrema(h, center, V1, vx_min, vx_max) ;

        // Step 2 : construct a Sparse Least Squares solver
        LinearSolver solver(2 * nb_distinct_vertices_) ;
        solver.set_least_squares(true) ;

        if(system_solver_parameters_.arg_string_value("method") != "DEFAULT") {
			std::cerr << "Using solver:" << system_solver_parameters_.arg_string_value("method") << std::endl ;
            solver.set_system_solver(system_solver_parameters_) ;
        } else {
            SystemSolverParameters params ;
            if(nb_iter_ == 0) {
				params.set_arg_value("method", std::string("CHOLMOD")) ;
            } else {
				params.set_arg_value("method", std::string("FASTCG")) ;
                std::ostringstream nb_iter_str ;
                nb_iter_str << nb_iter_ ;
				params.set_arg_value("nb_iter", nb_iter_str.str()) ;
            }
            solver.set_system_solver(params) ;
        }

        // Step 3 : Pin the two extrema vertices
        if(user_locks_) {
            FOR_EACH_VERTEX(Map, map_, it) {
                if(is_locked[it]) {
                    const Point2d& t = it->halfedge()->tex_coord() ;
                    solver.variable(2*vertex_id_[it]  ).lock() ;
                    solver.variable(2*vertex_id_[it]  ).set_value(t.x()) ;
                    solver.variable(2*vertex_id_[it]+1).lock() ;
                    solver.variable(2*vertex_id_[it]+1).set_value(t.y()) ;
                }
            }
        } else {
            solver.variable(2 * vertex_id_[vx_min]    ).lock() ;
            solver.variable(2 * vertex_id_[vx_min]    ).set_value(0) ;
            solver.variable(2 * vertex_id_[vx_min] + 1).lock() ;     
            solver.variable(2 * vertex_id_[vx_min] + 1).set_value(0) ;   
            solver.variable(2 * vertex_id_[vx_max]    ).lock() ;
            solver.variable(2 * vertex_id_[vx_max]    ).set_value(0) ;
            solver.variable(2 * vertex_id_[vx_max] + 1).lock() ;        
            solver.variable(2 * vertex_id_[vx_max] + 1).set_value(1) ;        
            is_locked[vx_min] = true ;
            is_locked[vx_max] = true ;
        }

        // Iterative solver: put initial values
        if(nb_iter_ != 0) {
            FOR_EACH_VERTEX(Map, map_, it) {
                const Point2d& t = it->halfedge()->tex_coord() ;
                solver.variable(2*vertex_id_[it]  ).set_value(t.x()) ;
                solver.variable(2*vertex_id_[it]+1).set_value(t.y()) ;
            }
        }

        // Step 4 : build the linear system to solve
        solver.begin_system() ;
        setup_conformal_map_relations(solver) ;
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

    
    void MapParameterizerLSCM::setup_conformal_map_relations( 
        LinearSolver& solver 
    ) {
        FOR_EACH_FACET(Map, map_, it) {
            setup_conformal_map_relations(solver,it) ;            
        }
    }
    
    void MapParameterizerLSCM::setup_conformal_map_relations(
        LinearSolver& solver, Map::Facet* f
    ) {
        // Do not install relations for null facets.
        if(facet_is_discarded(f)) {
            return ;
        }

        // "virtually triangulate" the facet
        Map::Halfedge* cur = f-> halfedge() ;
        Map::Halfedge* h0 = cur ;
        cur = cur-> next() ;
        Map::Halfedge* h1 = cur ;
        cur = cur-> next() ;
        Map::Halfedge* h2 = cur ;

        do {
            setup_conformal_map_relations(solver, h0, h1, h2) ;
            h1 = cur ;
            cur = cur-> next() ;
            h2 = cur ;
        } while (h2 != h0) ;
    }


    static void project_triangle(
        const Point3d& p0, 
        const Point3d& p1, 
        const Point3d& p2,
        Point2d& z0,
        Point2d& z1,
        Point2d& z2
    ) {
        Vector3d X = p1 - p0 ;
        X.normalize() ;
        Vector3d Z = X ^ (p2 - p0) ;
        Z.normalize() ;
        Vector3d Y = Z ^ X ;
        const Point3d& O = p0 ;

        double x0 = 0 ;
        double y0 = 0 ;
        double x1 = (p1 - O).norm() ;
        double y1 = 0 ;
        double x2 = (p2 - O) * X ;
        double y2 = (p2 - O) * Y ;        
        
        z0 = Point2d(x0,y0) ;
        z1 = Point2d(x1,y1) ;
        z2 = Point2d(x2,y2) ;        
    }

        
    void MapParameterizerLSCM::setup_conformal_map_relations(
        LinearSolver& solver, 
        Map::Halfedge* h0,
        Map::Halfedge* h1,
        Map::Halfedge* h2
    ) {
        
        Map::Vertex* v0 = h0->vertex() ;
        Map::Vertex* v1 = h1->vertex() ;
        Map::Vertex* v2 = h2->vertex() ;

        int id0 = vertex_id_[v0] ;
        int id1 = vertex_id_[v1] ;
        int id2 = vertex_id_[v2] ;

        // Skip degenerate triangles
        // (see MapParameterizer for the robust vertex numbering
        //  algorithm)
        if(id0 == id1 || id1 == id2 || id2 == id0) {
            return ;
        }

        Point3d p0 = v0->point() ; 
        Point3d p1 = v1->point() ;
        Point3d p2 = v2->point() ;

        normalize(p0) ;
        normalize(p1) ;
        normalize(p2) ;

        // LSCM equation, geometric form :
        // (Z1 - Z0)(U2 - U0) = (Z2 - Z0)(U1 - U0)
        // Where Uk = uk + i.vk is the complex number 
        //                       corresponding to (u,v) coords
        //       Zk = xk + i.yk is the complex number 
        //                       corresponding to local (x,y) coords
        // cool: no divide with this expression,
        //  makes it more numerically stable in
        //  the presence of degenerate triangles.

        Point2d z0,z1,z2 ;
        project_triangle(p0,p1,p2,z0,z1,z2) ;
        Vector2d z01 = z1 - z0 ;
        Vector2d z02 = z2 - z0 ;
        double a = z01.x() ;
        double b = z01.y() ;
        double c = z02.x() ;
        double d = z02.y() ;
        ogf_assert(b == 0.0) ;


        // Note  : 2*id + 0 --> u
        //         2*id + 1 --> v
        int u0_id = 2*id0     ;
        int v0_id = 2*id0 + 1 ;
        int u1_id = 2*id1     ;
        int v1_id = 2*id1 + 1 ;
        int u2_id = 2*id2     ;
        int v2_id = 2*id2 + 1 ;

        // Note : b = 0

        // Real part
        solver.begin_row() ;
        solver.set_right_hand_side(0.0) ;
        solver.add_coefficient(u0_id, -a+c) ;
        solver.add_coefficient(v0_id, b-d) ;
        solver.add_coefficient(u1_id, -c) ;
        solver.add_coefficient(v1_id, d) ;
        solver.add_coefficient(u2_id, a) ;
        solver.end_row() ;

        // Imaginary part
        solver.begin_row() ;
        solver.set_right_hand_side(0.0) ;
        solver.add_coefficient(u0_id, -b+d) ;
        solver.add_coefficient(v0_id, -a+c) ;
        solver.add_coefficient(u1_id,  -d) ;
        solver.add_coefficient(v1_id,  -c) ;
        solver.add_coefficient(v2_id,  a) ;
        solver.end_row() ;
    }

}

