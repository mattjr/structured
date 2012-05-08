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

#include <OGF/parameterizer/algos/blended_lsam.h>
#include <OGF/math/numeric/linear_solver.h>

namespace OGF {

//_______________________________________________________________________


    BlendedLSAM::BlendedLSAM() : weight_(0.5) {
    }

    bool BlendedLSAM::do_parameterize_disc(Map* map) {
        
        if(!Z1_.is_defined(map, "Z1")) {
            Logger::err("BlendedLSAM") << "Missing Z1 attribute" << std::endl ;
            return false ;
        }

        if(!Z2_.is_defined(map, "Z2")) {
            Logger::err("BlendedLSAM") << "Missing Z2 attribute" << std::endl ;
            return false ;
        }

        Z1_.bind(map, "Z1") ;
        Z2_.bind(map, "Z2") ;


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
        SystemSolverParameters params ;
        params.set_arg_value("method", std::string("SUPERLU")) ;        
        solver.set_system_solver(params) ;

        // Step 3 : Pin the two extrema vertices
        solver.variable(2 * vertex_id_[vx_min]    ).lock() ;
        solver.variable(2 * vertex_id_[vx_min]    ).set_value(0) ;
        solver.variable(2 * vertex_id_[vx_min] + 1).lock() ;     
        solver.variable(2 * vertex_id_[vx_min] + 1).set_value(0) ;   
        solver.variable(2 * vertex_id_[vx_max]    ).lock() ;
        solver.variable(2 * vertex_id_[vx_max]    ).set_value(0) ;
        solver.variable(2 * vertex_id_[vx_max] + 1).lock() ;        
        solver.variable(2 * vertex_id_[vx_max] + 1).set_value(1) ;        

        // Step 4 : build the linear system to solve
        solver.begin_system() ;
        FOR_EACH_FACET(Map, map_, it) {
            setup_conformal_map_relations(
                solver, it, Z1_, weight_
            ) ;
            setup_conformal_map_relations(
                solver, it, Z2_, 1.0 - weight_
            ) ;
        }
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
        

        Z1_.unbind() ;
        Z2_.unbind() ;

        return true ;
    }

    void BlendedLSAM::setup_conformal_map_relations(
        LinearSolver& solver, 
        Map::Facet* f,
        MapHalfedgeAttribute<Complex>& Z_attrib,
        double weight
    ) {

        Map::Halfedge* h0 = f->halfedge() ;
        Map::Halfedge* h1 = h0->next() ;
        Map::Halfedge* h2 = h1->next() ;
        
        Map::Vertex* v0 = h0->vertex() ;
        Map::Vertex* v1 = h1->vertex() ;
        Map::Vertex* v2 = h2->vertex() ;

        int id0 = vertex_id_[v0] ;
        int id1 = vertex_id_[v1] ;
        int id2 = vertex_id_[v2] ;

        // Note  : 2*id + 0 --> u
        //         2*id + 1 --> v

        int u0_id = 2*id0     ;
        int v0_id = 2*id0 + 1 ;
        int u1_id = 2*id1     ;
        int v1_id = 2*id1 + 1 ;
        int u2_id = 2*id2     ;
        int v2_id = 2*id2 + 1 ;
        
        const Point3d& p0 = h0->vertex()->point() ;
        const Point3d& p1 = h1->vertex()->point() ;
        const Point3d& p2 = h2->vertex()->point() ;

        double area = Geom::triangle_area(p0,p1,p2) ;
        double s = ::sqrt(area) * weight ;

        Complex Z = Z_attrib[h0] ;

        double a = Z.real();
        double b = Z.imaginary();

        solver.begin_row() ;
        solver.set_right_hand_side(0.0) ;
        solver.add_coefficient(u0_id, 1-a) ;
        solver.add_coefficient(v0_id,  b) ;
        solver.add_coefficient(u1_id,  a) ;
        solver.add_coefficient(v1_id,  -b) ;
        solver.add_coefficient(u2_id,  -1) ;
        solver.scale_row(s) ;
        solver.end_row() ;

        solver.begin_row() ;
        solver.set_right_hand_side(0.0) ;
        solver.add_coefficient(u0_id,  -b) ;
        solver.add_coefficient(v0_id, 1-a) ;
        solver.add_coefficient(u1_id,  b) ;
        solver.add_coefficient(v1_id,  a) ;
        solver.add_coefficient(v2_id,  -1) ;
        solver.scale_row(s) ;
        solver.end_row() ;
    }

    
//_______________________________________________________________________

}
