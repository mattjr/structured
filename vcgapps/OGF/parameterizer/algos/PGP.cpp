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

#include <OGF/parameterizer/algos/PGP.h>
#include <OGF/parameterizer/algos/map_trimmer.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/cells/map/map_editor.h>
#include <OGF/cells/map_algos/enumerate.h>
#include <OGF/cells/map_algos/map_curvature.h>
#include <OGF/cells/map_algos/iso_curves_extractor.h>
#include <OGF/math/geometry/trgl_grad.h>

#include <stack>

#include <sstream>

namespace OGF {

//----------------------------------------------------------------------------------------------------------------------

    PeriodicSolver::PeriodicSolver(Map* map, int nb_variables_per_vertex, const std::string& base_attr_name) {
        map_ = map ;
        nb_variables_per_vertex_ = nb_variables_per_vertex ;
        vertex_id_.bind(map) ;
        enumerate_vertices(map, vertex_id_, 0, 2*nb_variables_per_vertex_) ;
        use_non_linear_constraints_ = false ;
        use_non_linear_solver_ = false ;
        use_triangle_integral_ = true ;
        lock_borders_ = false ;
        initialize_ = true ;
        linear_solver_ = nil ;
        non_linear_solver_ = nil ;
        max_newton_iter_ = 10 ;
        U_ = new MapVertexAttribute<Complex>[nb_variables_per_vertex] ;
        for(int i=0; i<nb_variables_per_vertex_; i++) {
            std::ostringstream s ;
            s << base_attr_name << (i+1) ;
            U_[i].bind(map_, s.str()) ;
            std::cerr << "binding attribute: " << s.str() << std::endl ;
        }
    }

    PeriodicSolver::~PeriodicSolver() {
        delete linear_solver_ ;
        delete non_linear_solver_ ;
        delete[] U_ ;
        U_ = nil ;
    }

    void PeriodicSolver::compute_singularity_weight() {
        singularity_.bind_if_defined(map_,"error") ;
        if(!singularity_.is_bound()) {
            Logger::err("PeriodicSolver") << "No error attribute defined" << std::endl ;
            return ;
        }

        double vmin =   1e30 ;
        double vmax = -1e30 ;
        FOR_EACH_VERTEX(Map, map_, it) {
            vmin = ogf_min(vmin, singularity_[it]) ;
            vmax = ogf_max(vmax, singularity_[it]) ;
        }
        double d = ogf_max(vmax - vmin, 1e-10) ;
        FOR_EACH_VERTEX(Map, map_, it) {
            double v = singularity_[it] ;
            singularity_[it] = (v - vmin) / d ;
        }        
    }

    void PeriodicSolver::setup_solver() {
        delete non_linear_solver_ ; non_linear_solver_ = nil ;
        delete linear_solver_ ; linear_solver_ = nil ;
        if(use_non_linear_solver_) {
            non_linear_solver_ = new NonLinearSolver(2 * map_->size_of_vertices() * nb_variables_per_vertex_) ;
            non_linear_solver_->set_max_newton_iter(max_newton_iter_) ;
            non_linear_solver_->set_system_solver(system_solver_parameters_) ;
            { 
                using namespace Symbolic ;
                edge_eqn_ = non_linear_solver_->declare_stencil(
                    c[0] * (( c[1] * x[1] + c[2] * x[2] - x[0])^2)
                ) ;
                norm_eqn_ = non_linear_solver_->declare_stencil(
                    c[0] * (((x[0]^2) + (x[1]^2) - 1.0)^2)
                ) ;
                val_eqn_ = non_linear_solver_->declare_stencil(
                    c[0] * ((x[0] - c[1])^2)
                ) ;
            }
        } else {
            linear_solver_ = new LinearSolver(2 * map_->size_of_vertices() * nb_variables_per_vertex_) ;
            linear_solver_->set_least_squares(true) ;
            linear_solver_->set_system_solver(system_solver_parameters_) ;
        }
    }

    void PeriodicSolver::begin_equation() {
        if(use_non_linear_solver_) {
            non_linear_solver_->begin_equation() ;
        } else {
            linear_solver_->begin_system() ;
        }
    }
    
    void PeriodicSolver::end_equation() {
        if(use_non_linear_solver_) {
            non_linear_solver_->end_equation() ;
        } else {
            linear_solver_->end_system() ;
        }
    }
    
    void PeriodicSolver::solve() {
        if(use_non_linear_solver_) {
            non_linear_solver_->minimize() ;
        } else {
            linear_solver_->solve() ;
        }
    }

    void PeriodicSolver::setup_norm_equation() {
        if(!use_non_linear_solver_) {
            Logger::warn("PeriodicSolver")
                << "cannot use norm equation without non-linear solver"
                << std::endl ;
            return ;
        }

        FOR_EACH_VERTEX(Map, map_, it) {
            int base = vertex_id_[it] ;
            for(int i=0; i<nb_variables_per_vertex_; i++) {
                double w = 0.1 ;
                if(singularity_.is_bound()) {
                    w *= (1.0 - singularity_[it]) ;
                }
                non_linear_solver_->begin_stencil_instance(norm_eqn_) ;
                non_linear_solver_->stencil_variable(base + 2*i) ;
                non_linear_solver_->stencil_variable(base + 2*i +1) ;
                non_linear_solver_->stencil_parameter(w) ;
                non_linear_solver_->end_stencil_instance() ;
            }
        }
    }

    void PeriodicSolver::add_term(double w, int id, double angle) {
        int a_id = id ;
        int b_id = id + 1 ;
        double s = ::sin(angle) ;
        double c = ::cos(angle) ;

        if(use_non_linear_solver_) {

            non_linear_solver_->begin_stencil_instance(val_eqn_) ;
            non_linear_solver_->stencil_parameter(w) ;
            non_linear_solver_->stencil_parameter(c) ;
            non_linear_solver_->stencil_variable(a_id) ;
            non_linear_solver_->end_stencil_instance() ;

            non_linear_solver_->begin_stencil_instance(val_eqn_) ;
            non_linear_solver_->stencil_parameter(w) ;
            non_linear_solver_->stencil_parameter(s) ;
            non_linear_solver_->stencil_variable(b_id) ;
            non_linear_solver_->end_stencil_instance() ;

        } else {

            w = ::sqrt(w) ;

            linear_solver_->begin_row();
            linear_solver_->add_coefficient(a_id, 1.0);
            linear_solver_->set_right_hand_side(c);
            linear_solver_->scale_row(w);
            linear_solver_->end_row();

            linear_solver_->begin_row();
            linear_solver_->add_coefficient(b_id, 1.0);
            linear_solver_->set_right_hand_side(s);
            linear_solver_->scale_row(w);
            linear_solver_->end_row();

        }
    }
    

    void PeriodicSolver::add_term(
        double w, int id1, int id2, double delta, bool swap_sign_id2        
    ) {

        int a1_id = id1     ;
        int b1_id = id1 + 1 ;
        int a2_id = id2     ;
        int b2_id = id2 + 1 ;

        double s = ::sin(delta) ;
        double c = ::cos(delta) ;
        
        double orient = (swap_sign_id2) ? -1.0 : 1.0 ;

        if(use_non_linear_solver_) {
            // Real part
            non_linear_solver_->begin_stencil_instance(edge_eqn_) ;
            non_linear_solver_->stencil_parameter(w) ;
            non_linear_solver_->stencil_parameter(c) ;
            non_linear_solver_->stencil_parameter(-s) ;
            non_linear_solver_->stencil_variable(a1_id) ;
            non_linear_solver_->stencil_variable(a2_id) ;
            non_linear_solver_->stencil_variable(b2_id) ;
            non_linear_solver_->end_stencil_instance() ;

            // Imaginary part
            // Rem: if we change the orientation, only the 
            // imaginary part changes (since we replace theta with -theta)
            non_linear_solver_->begin_stencil_instance(edge_eqn_) ;
            non_linear_solver_->stencil_parameter(w) ;
            non_linear_solver_->stencil_parameter(s*orient) ;
            non_linear_solver_->stencil_parameter(c*orient) ;
            non_linear_solver_->stencil_variable(b1_id) ;
            non_linear_solver_->stencil_variable(a2_id) ;
            non_linear_solver_->stencil_variable(b2_id) ;
            non_linear_solver_->end_stencil_instance() ;
        } else {
            w = (w < 0) ? -::sqrt(-w) : sqrt(w) ;
            // Real part
            linear_solver_->begin_row() ;
            linear_solver_->add_coefficient(a1_id, -1) ;
            linear_solver_->add_coefficient(a2_id,  c) ;
            linear_solver_->add_coefficient(b2_id, -s) ;
            linear_solver_->scale_row(w) ;
            linear_solver_->end_row() ;

            // Imaginary part
            // Rem: if we change the orientation, only the 
            // imaginary part changes (since we replace theta with -theta)
            linear_solver_->begin_row() ;
            linear_solver_->add_coefficient(b1_id, -1) ;
            linear_solver_->add_coefficient(a2_id,  s * orient) ;
            linear_solver_->add_coefficient(b2_id,  c * orient) ;
            linear_solver_->scale_row(w) ;
            linear_solver_->end_row() ;
        }
    }

    static double det(const Matrix<double,3>& M) {
        Vector3d V1(M(0,0), M(0,1), M(0,2)) ;
        Vector3d V2(M(1,0), M(1,1), M(1,2)) ;
        Vector3d V3(M(2,0), M(2,1), M(2,2)) ;
        return V1 * (V2 ^ V3) ;
    }

    double PeriodicSolver::halfedge_weight(Map::Halfedge* h,const Vector3d& K) {
        Vector3d field = K ;
        if(!use_triangle_integral_) {
            return 1.0 ;
        }
        if (h->is_border()) return 0 ; 
        double area = Geom::facet_area(h->facet());
        
        if (area<1e-15) {
            std::cerr<<"Facet area too small... I'm unhappy..." << std::endl ;
            return 0;
        }

        double inv_area = 1.0/area;
        double scale = sqrt(inv_area);

        // Seems that scale does not improve anything, temporarily deactivated...
        scale = 1.0 ;

        field.normalize();
        Vector3d field_perp = Geom::facet_normal(h->facet())^field;
        field_perp.normalize();
        Matrix<double,3> M;

        for (int side = 0;side<3;side++){
            double ex = Geom::vector(h) *field;
            double ey = Geom::vector(h) *field_perp;
            M(side,0) =       (scale*ex)*(scale*ex);
            M(side,1) =       (scale*ey)*(scale*ey);
            M(side,2) = 2.0*(scale*ex)*(scale*ey); 
            h=h->next();
        }
        
        if (fabs(det(M))<1e-30){
            std::cerr << "Singular triangle matrix, .... return 0 ???" << std::endl;
            return 0;
        }
        
        Matrix<double,3> M_inv = M.inverse();
        double result = area * scale * scale * (M_inv(0,0)+M_inv(1,0)); 

        if(singularity_.is_bound()) {
            double w = 0.5 * (singularity_[h->vertex()] + singularity_[h->opposite()->vertex()]) ;
            result *= (1.0 - w) ;
        }

        return result;
    }
    
    double PeriodicSolver::edge_weight(Map::Halfedge* h, const Vector3d& field) {
        double res = 0;
        if (!h->is_border()) res += halfedge_weight(h,field) ;
        if (!h->opposite()->is_border()) res += halfedge_weight(h->opposite(),field) ;
        return res;
    }
    
    double PeriodicSolver::average_edge_length() {
        if(map_->size_of_halfedges() == 0) {
            return 0.0 ;
        }
        double result = 0 ;
        FOR_EACH_EDGE(Map, map_, it) {
            result += Geom::edge_length(it) ;
        }
        result /= double(map_->size_of_halfedges() / 2) ;
        return result ;
    }

    Vector3d PeriodicSolver::rot(const Vector3d& K,  const Vector3d& N_in, double angle) {
        Vector3d N = N_in ;
        N.normalize() ;
        double c = ::cos(angle) ;
        double s = ::sin(angle) ;
        Vector3d X = K - (K*N) * N ; 
        X.normalize() ;
        Vector3d Y = N ^ X ;
        Vector3d result = c * X + s * Y ;
        result.normalize() ;
        return result ;
    }

    void PeriodicSolver::lock_borders() {
        MapVertexLock is_locked(map_) ;
        FOR_EACH_VERTEX(Map, map_, it) {
            is_locked[it] = it->is_on_border() ;
        }
    }

    void PeriodicSolver::map_to_solver(bool lock_at_least_one) {
        MapVertexLock is_locked(map_) ;

        if(lock_borders_) {
            lock_borders() ;
        } else {
            bool has_one_lock = false ;
            FOR_EACH_VERTEX(Map, map_, it) {
                if(is_locked[it]) {
                    has_one_lock = true ; 
                    break ; 
                }
            }
            if(lock_at_least_one && !has_one_lock) {
                is_locked[map_->vertices_begin()] = true ;
            }
        }
        
        FOR_EACH_VERTEX(Map, map_, it) {
            bool lock = is_locked[it] ;
            int id = vertex_id_[it] ;
            for(int i=0; i<nb_variables_per_vertex_; i++) {
                if(lock) {
                    if(lock_borders_ && it->is_on_border() && i >0) {
                    } else {
                        solver().variable(id + 2*i).lock() ;
                        solver().variable(id + 2*i + 1).lock() ;
                    }
                }
                if(initialize_) {
                    double val = 1.0 ;
                    if(!use_non_linear_solver_) { val = 1e4 ; }
                    solver().variable(id + 2*i).set_value(val) ;
                    solver().variable(id + 2*i + 1).set_value(0.0) ;
                } else {
                    solver().variable(id + 2*i).set_value(U_[i][it].real()) ;
                    solver().variable(id + 2*i + 1).set_value(U_[i][it].imaginary()) ;
                }
            }
        }
    }

    void PeriodicSolver::solver_to_map() {
        FOR_EACH_VERTEX(Map, map_, it) {
            int id = vertex_id_[it] ;
            double prodcos = 1.0 ;
            for(int i=0; i<nb_variables_per_vertex_; i++) {
                U_[i][it] = Complex(
                    solver().variable(id + 2*i).value(),
                    solver().variable(id + 2*i+1).value() 
                ) ;
                prodcos = prodcos * U_[i][it].real() / (U_[i][it].modulus() + 1e-30) ;
            }
            if(cosprod_.is_bound()) {
                cosprod_[it] = prodcos ;
            }
        }
    }

//----------------------------------------------------------------------------------------------------------------------

    static Complex complex_pow(Complex Z, double arg){
        double r = ::pow(Z.modulus(), arg);
        Z.normalize();
        double angle = double(arg) * Z.angle() ;
        return Complex(r*cos(angle),r*sin(angle));
    }

    static Complex project_in_vertex_basis(const Vector3d& vect, Map::Vertex* vx){
        Vector3d N = Geom::vertex_normal(vx) ;
        Vector3d u = Geom::perpendicular(N) ;
        Vector3d v = N ^ u ;
        return Complex(vect * u,vect * v);
    }

    static Vector3d vertex_basis_to_3d(const Complex& vect, Map::Vertex* vx){
        Vector3d N = Geom::vertex_normal(vx) ;
        Vector3d u = Geom::perpendicular(N) ;
        Vector3d v = N ^ u ;
        return (vect.real() * u) +  (vect.imaginary() * v);
    }

    static double compute_delta_between_vertices_basis(Map::Halfedge* h){
        Map::Vertex* v1 = h->vertex() ;
        Map::Vertex* v2 = h->opposite()->vertex() ;
        Complex h_in_v1 = project_in_vertex_basis(Geom::vector(h),v1) ;
        Complex h_in_v2 = project_in_vertex_basis(Geom::vector(h),v2) ;
        h_in_v1.normalize();
        h_in_v2.normalize();
        double a1 = h_in_v1.angle();
        double a2 = h_in_v2.angle();
        return a1-a2;
    }

    VectorFieldSmoother::VectorFieldSmoother(Map* map, int modulo) : PeriodicSolver(map,1, "Z") {
        modulo_ = modulo ;
        compute_error_ = false ;
        smoothing_coefficient_ = 0.8 ;
        K_.bind(map_, "K1") ;
    }


    void VectorFieldSmoother::smooth_vector_field() {
        for(int i=0; i<max_newton_iter_; i++) {
            bool initialize = (i == 0 && initialize_) ;
            one_iteration(initialize) ;
            update_graphics(map_) ;
        }
    }

    void VectorFieldSmoother::one_iteration(bool initialize) {
        if(initialize) {
            double neighborhood_size = 0.01 ;
            Box3d bbox = Geom::map_bbox(map_) ;
            neighborhood_size *= (2.0 * bbox.radius()) ;
            MapCurvature curvatures(map_) ;
            curvatures.set_Kmin_attribute("K1") ;
            curvatures.set_Kmax_attribute("K2") ;
            curvatures.set_N_attribute("N") ;
            curvatures.set_kmin_attribute("k1") ;
            curvatures.set_kmax_attribute("k2") ;
            curvatures.set_n_attribute("n") ;
            curvatures.set_radius(neighborhood_size) ;
            curvatures.compute_curvature_tensor() ;
        }

        MapVertexAttribute<Complex> U0(map_) ;
        setup_solver() ;
        map_to_solver() ;
        FOR_EACH_VERTEX(Map, map_, it) {
            Complex Z = project_in_vertex_basis(K_[it],it) ;
            if (Z.modulus() > 1e-10) {
                Z.normalize() ;
            } else {
                Z = Complex(1.0,0.0) ;
            }           
            Z = complex_pow(Z, modulo_) ;
            U0[it] = Z ;
            int id = vertex_id_[it] ;
            solver().variable(id).set_value(Z.real());
            solver().variable(id+1).set_value(Z.imaginary());
        }

        begin_equation() ;
        FOR_EACH_EDGE(Map, map_, it) {
            double w = edge_weight(it, Geom::vector(it)) ;
            int id1 = vertex_id_[it->vertex()];
            int id2 = vertex_id_[it->opposite()->vertex()];
            double delta = compute_delta_between_vertices_basis(it);
            delta *= double(modulo_) ;
            add_term(w, id1, id2, delta, false) ;
        }
        FOR_EACH_VERTEX(Map, map_, it) {
            double w = 1.0 - smoothing_coefficient_ ;
            double angle = U0[it].angle() ;
            int id = vertex_id_[it] ;
            add_term(w, id, angle) ;
        }
        end_equation() ;
        solve() ;
        solver_to_map() ;

        if(compute_error_) {
            compute_error() ;
        }

        FOR_EACH_VERTEX(Map, map_, it) {
            U_[0][it] = complex_pow(U_[0][it], 1.0 / double(modulo_)) ;
            K_[it] = vertex_basis_to_3d(U_[0][it], it) ;
        }

        compute_K2() ;
    }

    void VectorFieldSmoother::compute_K2() {
        MapVertexAttribute<Vector3d> K2(map_, "K2") ;
        FOR_EACH_VERTEX(Map, map_, it) {
            K2[it] = rot(K_[it], Geom::vertex_normal(it),  M_PI / 2.0) ;
        }
    }

    void VectorFieldSmoother::compute_error() {
        MapVertexAttribute<double> error_v(map_, "error") ;
        FOR_EACH_VERTEX(Map, map_, it) {
            error_v[it] = 0.0 ;
        }
        FOR_EACH_EDGE(Map, map_, it) {
            Map::Vertex* v1 = it->vertex() ;
            Map::Vertex* v2 = it->opposite()->vertex() ;
            double angle = compute_delta_between_vertices_basis(it) ;
            double e = compute_error(angle, U_[0][v1], U_[0][v2]) ;
            error_v[it->vertex()] += e ;
            error_v[it->opposite()->vertex()] += e ;
        }
        FOR_EACH_VERTEX(Map, map_, it) {
            error_v[it] /= double(it->degree()) ;
        }
    }
    
    double VectorFieldSmoother::compute_error(
        double angle, const Complex& Z1, const Complex& Z2
    ) {
        angle *= double(modulo_) ;
        double u1 = Z1.real() ;
        double v1 = Z1.imaginary() ;
        double u2 = Z2.real() ;
        double v2 = Z2.imaginary() ;
        double errx = - u1 + ::cos(angle) * u2 - ::sin(angle) * v2 ;
        double erry = - v1 + ::sin(angle) * u2 + ::cos(angle) * v2 ;
        return errx*errx + erry*erry ;
    }

    void VectorFieldSmoother::lock_borders() {
        MapVertexLock is_locked(map_) ;
        FOR_EACH_VERTEX(Map, map_, it) {
            if(it->is_on_border()) {
                is_locked[it] = true ;
                Map::Halfedge* h = it->halfedge() ;
                do {
                    if(h->is_border()) {
                        Vector3d K = Geom::vector(h) + Geom::vector(h->next()) ;
                        K.normalize() ;
                        Complex Z = project_in_vertex_basis(K, it) ;
                        K_[it] = vertex_basis_to_3d(Z, it) ;
                        U_[0][it] = complex_pow(Z, modulo_) ;
                        break ;
                    }
                    h = h->next_around_vertex() ;
                } while(h != it->halfedge()) ;
            } else {
                is_locked[it] = false ;
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------------------------------------

    PeriodicParameterizer::PeriodicParameterizer(
        Map* map, int nb_variables_per_vertex
    ) : PeriodicSolver(map, nb_variables_per_vertex) {
        wavelength_.bind(map_, "wavelength") ;
        K_.bind(map_, "K1") ;
        theta_ = new MapHalfedgeAttribute<double>[nb_variables_per_vertex] ;
        for(int i=0; i<nb_variables_per_vertex_; i++) {
            std::ostringstream s ;
            s << "theta" << (i+1) ;
            theta_[i].bind(map_, s.str()) ;
            std::cerr << "binding attribute: " << s.str() << std::endl ;
        }
    }

    PeriodicParameterizer::~PeriodicParameterizer() {
        delete[] theta_ ;
    }

    void PeriodicParameterizer::set_wavelength(
        double omega, bool relative, bool use_curl_correction
    ) {   
        if(relative) {
            omega *= average_edge_length() ;
        }
        MapVertexAttribute<double> w ;
        if(use_curl_correction) {
            w.bind_if_defined(map_, "w") ;
        }
        if(w.is_bound()) {
            Logger::out("PeriodicParameterizer") << "using curl-correction" << std::endl ;
        } else {
            Logger::out("PeriodicParameterizer") << "plain equation" << std::endl ;
        }
        FOR_EACH_VERTEX(Map, map_, it) {
            wavelength_[it] = omega ;
            if(w.is_bound()) {
                wavelength_[it] *= w[it] ;
            }
        }
    }

    void PeriodicParameterizer::parameterize() {
        cosprod_.bind(map_, "cosprod") ;
        if(use_non_linear_solver_) {
            one_iteration(false) ;
        } else {
            for(int i=0; i<max_newton_iter_; i++) {
                bool smooth = (i > 0 || !initialize_) ;
                one_iteration(smooth) ;
            }
        }
    }

    void PeriodicParameterizer::one_iteration(bool smooth) {
        setup_solver() ;
        map_to_solver(true) ;
        begin_equation() ;
        if(use_non_linear_solver_) {
            compute_singularity_weight() ;
        }
        setup_non_linear_constraints() ;
        FOR_EACH_EDGE(Map, map_, it) {
            add_edge_terms(it) ;
        }
        if(smooth) {
            FOR_EACH_VERTEX(Map, map_, it) {
                for(int i=0; i<nb_variables_per_vertex_; i++) {
                    int id = vertex_id_[it] ;
                    if(!solver().variable(id).is_locked()) {
                        add_term(1e-6, id, U_[i][it].angle()) ;
                    }
                }
            }
        }
        end_equation() ;
        solve() ;
        solver_to_map() ;
        update_graphics(map_) ;
    }

    void PeriodicParameterizer::setup_non_linear_constraints() {
        if(use_non_linear_solver_) {
            setup_norm_equation() ;
        }
    }

    void PeriodicParameterizer::add_edge_term(
        Map::Halfedge* h, int id1, const Vector3d& K1, int id2, const Vector3d& K2
    ) {
        double orient = 1.0 ;
        if(K1*K2 < 0) {
            orient = -1.0 ;
        }
        Vector3d K12  = K2 + orient * K1 ;
        K12.normalize() ;
        Vector3d e = Geom::vector(h) ;
        double dist = (K12 * e) ;
        double wavelength = 0.5 * (wavelength_[h->prev()->vertex()] + wavelength_[h->vertex()])  ;
        double dtheta = 2.0 * M_PI * dist / wavelength ; 
        double w = edge_weight(h,K12);
        add_term(w, id1, id2, dtheta, (orient < 0)) ;
    }

    int PeriodicParameterizer::nearest(const Vector3d& K, Vector3d* K_ref, int n) {
        int result = -1 ;
        double max_dot = -1e30 ;
        for(int i=0; i<n; i++) {
            double cur_dot = ::fabs(Geom::cos_angle(K, K_ref[i])) ;
            if(cur_dot > max_dot) {
                max_dot = cur_dot ;
                result = i ;
            }
        }
        return result ;
    }

    void PeriodicParameterizer::add_edge_terms(Map::Halfedge* h) {
        ogf_assert(nb_variables_per_vertex_ <= 10) ;
        Vector3d K_from[10] ;
        Vector3d K_dest[10] ;

        Map::Vertex* v0 = h->opposite()->vertex() ;
        Map::Vertex* v1 = h->vertex() ;

        Vector3d N0 = Geom::vertex_normal(v0) ;
        Vector3d N1 = Geom::vertex_normal(v1) ;

        K_from[0] = K_[v0] ;
        K_dest[0] = K_[v1] ;

        for(int i=1; i<nb_variables_per_vertex_; i++) {
            K_from[i] = rot(K_from[i-1], N0, M_PI / double(nb_variables_per_vertex_)) ;
            K_dest[i] = rot(K_dest[i-1], N1, M_PI / double(nb_variables_per_vertex_)) ;
        }

        for(int i=0; i<nb_variables_per_vertex_; i++) {
            int j = nearest(K_from[i], K_dest, nb_variables_per_vertex_) ;
            int id_from = vertex_id_[v0] + 2*i ;
            int id_to = vertex_id_[v1] + 2*j ;
            add_edge_term(h, id_from, K_from[i], id_to, K_dest[j]) ;
        }
    }


    static bool check_nan(double x, const std::string& x_name) {
        bool result = false ;
        if(Numeric::is_nan(x)) {
            std::cerr << x_name << " is a NAN !!! (ouh le pas bo)" << std::endl ;
            result = true ;
        }
        return result ;
    }

#define IS_NAN(x) check_nan(x,#x)

    void PeriodicParameterizer::solve_curl_correction(
        Map* map, const SystemSolverParameters& solver_config
    ) {

        // Rem1: on multiplie et on divise par l'aire du triangle, c'est idiot !!
        // (les TX et les TY sont divises par l'aire du triangle, et on scale
        //  la row ensuite ...)
        // Rem2: on multiplie par l'aire et on divise par le carre de l'aire,
        // donc on divise de toute facon
        
        MapVertexAttribute<double> w(map,"w");
        MapVertexAttribute<Vector3d> K(map,"K1");
        FOR_EACH_VERTEX(Map,map,vi) {
            K[vi].normalize();
        }
        
        MapVertexAttribute<int> vertex_id(map);
        enumerate_vertices(map, vertex_id) ;
        
        double solver_scale = 1e3;
        double locked_value =1;

        LinearSolver solver(map->size_of_vertices());
        solver.set_least_squares(true);
        solver.set_system_solver(solver_config) ;
        solver.variable(0).set_value(locked_value);
        solver.variable(0).lock();
        solver.begin_system();
        FOR_EACH_FACET(Map,map,fi) {

            Map::Halfedge* h= fi->halfedge();
            // (u,v) form a orthogonal base of the facet f, and u is the field projected
            // in facet f
            Vector3d u = (Geom::facet_normal(fi)^K[h->vertex()]) ^ Geom::facet_normal(fi);
            u.normalize() ;
            Vector3d v = Geom::facet_normal(fi)^u;
            v.normalize();
            
            // fieldx is the 2d field at point x
            // the field are rotated in a coherant way to take the modulus into account
            // notice that it will be better if the field is rotated to the facet
            // instead of being projected
            Vector2d fieldA (1,0);
            Vector2d fieldB (K[h->next()->vertex()] * u,K[h->next()->vertex()] * v) ;
            fieldB.normalize();
            int nb_iter = 0; 
            while (fieldB.x()<.7 && nb_iter++<4) {
                fieldB = Vector2d(-fieldB.y(),fieldB.x());
            }
            Vector2d fieldC (K[h->prev()->vertex()] * u,K[h->prev()->vertex()] * v) ;
            fieldC.normalize();
            nb_iter = 0;     
            while (fieldC.x()<.7 && nb_iter++<4) {
                fieldC = Vector2d(-fieldC.y(),fieldC.x());
            }
            
            // define the (A,B,C) triangle in the (u,v) basis
            Vector3d AB = h->next()->vertex()->point() - h->vertex()->point();
            Vector3d AC = h->prev()->vertex()->point() - h->vertex()->point();
            Point2d A(0,0);
            Point2d B(AB*u,AB*v);
            Point2d C(AC*u,AC*v);
            
            // hummm... flat triangles ?
            double a=0;
            double b=0;
            ParamTrglGradient trg(A,B,C);
            if (trg.is_flat() || (AB^AC).norm()<1e-10 || ::fabs(fieldB.x()) < 1e-20 || ::fabs(fieldC.x()) < 1e-20) {
                std::cerr<<"bad triangle ..." << std::endl ;
                
                solver.begin_row() ;
                solver.add_coefficient(vertex_id[h->vertex()]	,1e-2) ;
                solver.add_coefficient(vertex_id[h->next()->vertex()]	,-1e-2) ;
                solver.set_right_hand_side(0.0) ;
                solver.end_row() ;

                solver.begin_row() ;
                solver.add_coefficient(vertex_id[h->vertex()]	,1e-2) ;
                solver.add_coefficient(vertex_id[h->prev()->vertex()]	,-1e-2) ;
                solver.set_right_hand_side(0.0) ;
                solver.end_row() ;

                solver.begin_row() ;
                solver.add_coefficient(vertex_id[h->prev()->vertex()]	,1e-2) ;
                solver.add_coefficient(vertex_id[h->next()->vertex()]	,-1e-2) ;
                solver.set_right_hand_side(0.0) ;
                solver.end_row() ;
                
                continue ;
            } else {

            // the direction field is represented by angles
            //  double alpha_A = 0;
            //  double alpha_B = atan(fieldB.y()/fieldB.x());
            //  double alpha_C = atan(fieldC.y()/fieldC.x());

                // Order 1 Taylor Expansion ....
                double alpha_A = 0;
                double alpha_B = fieldB.y()/fieldB.x();
                double alpha_C = fieldC.y()/fieldC.x();
                
                Vector2d grad_alpha (	
                    trg.TX(0)*alpha_A + trg.TX(1)*alpha_B + trg.TX(2)*alpha_C,
                    trg.TY(0)*alpha_A + trg.TY(1)*alpha_B + trg.TY(2)*alpha_C
                ) ;
                a = grad_alpha.x();
                b = grad_alpha.y();
            }

            double sqrt_area =  ::sqrt(Geom::facet_area(fi)) ;

            if(
                IS_NAN(sqrt_area) ||
                IS_NAN(trg.TX(0)) ||
                IS_NAN(trg.TX(1)) ||                
                IS_NAN(trg.TX(2)) ||
                IS_NAN(trg.TY(0)) ||
                IS_NAN(trg.TY(1)) ||
                IS_NAN(trg.TY(2)) ||
                IS_NAN(a) ||
                IS_NAN(b) 
            ) {
                std::cerr << "Found NAN !!" << std::endl ;
            }
  
            solver.begin_row() ;
            solver.add_coefficient(vertex_id[h->vertex()]		,trg.TX(0)) ;
            solver.add_coefficient(vertex_id[h->next()->vertex()]	,trg.TX(1)) ;
            solver.add_coefficient(vertex_id[h->prev()->vertex()]	,trg.TX(2)) ;
            solver.set_right_hand_side(solver_scale*b);
            solver.scale_row(sqrt_area) ;
            solver.end_row() ;

            solver.begin_row() ;
            solver.add_coefficient(vertex_id[h->vertex()]		,trg.TY(0)) ;
            solver.add_coefficient(vertex_id[h->next()->vertex()]	,trg.TY(1)) ;
            solver.add_coefficient(vertex_id[h->prev()->vertex()]	,trg.TY(2)) ;
            solver.set_right_hand_side(solver_scale*-a);
            solver.scale_row(sqrt_area) ;
            solver.end_row() ;
        }

        solver.end_system();
        solver.solve();
        
        double min = 1e6;
        double max = -1e6;
        // retreive values
        FOR_EACH_VERTEX(Map,map,vi){
            w[vi] = ::exp((solver.variable(vertex_id[vi]).value()-locked_value)/solver_scale);
            min = ogf_min(min,w[vi]);
            max = ogf_max(max,w[vi]);
        }
        // normalize the result
        FOR_EACH_VERTEX(Map,map,vi){
            w[vi] /=max;
        }
    }

    void PeriodicParameterizer::extract_tex_coords_in_facets() {
        MapEditor editor(map_) ;
        editor.split_all_tex_vertices() ;
        FOR_EACH_FACET(Map, map_, it) {
            extract_tex_coords_in_facet(it) ;
        }
//        show_tex_coords() ;
    }
    
    void PeriodicParameterizer::extract_tex_coords_in_components() {
        std::stack<Map::Halfedge*> S ;
        MapFacetAttribute<bool> is_visited(map_) ;
        FOR_EACH_FACET(Map, map_, it) {
            if(!is_visited[it]) {
                init_extract_tex_coords(it->halfedge()->prev()) ;
                S.push(it->halfedge())  ;
                is_visited[it] = true ;
                while(S.size() != 0) {
                    Map::Halfedge* top = S.top() ;
                    Map::Halfedge* h = top ;
                    S.pop() ;
                    do {
                        propagate_along(h) ;
                        // Even if there is no opposite facet, we compute
                        // the tex coord on the opposite halfedge, to have
                        // correct tex coords everywhere
                        if(h->opposite()->facet() == nil) {
                            propagate_accross(h) ;
                        } else  if(!is_visited[h->opposite()->facet()]) {
                            propagate_accross(h) ;
                            S.push(h->opposite()) ;
                            is_visited[h->opposite()->facet()] = true ;
                        }
                        h = h->next() ;
                    } while(h != top) ;
                }
            }
        }
        MapEditor editor(map_) ;
        editor.merge_all_tex_vertices() ;
    }


    void PeriodicParameterizer::show_tex_coords(Map::Halfedge* h) {
        h->set_tex_coord(
            Point2d( theta_[0][h] / M_PI, theta_[1][h] / M_PI) 
        ) ;
    }
    
    void PeriodicParameterizer::show_tex_coords() {
        // For display purposes, initialize tex coords.
        // Divide by pi, so that texture repeat naturally
        // shows periodicity
        FOR_EACH_HALFEDGE(Map, map_, it) {
            show_tex_coords(it) ;
        }
    }

    void PeriodicParameterizer::extract_tex_coords_in_facet(Map::Facet* f) {
        ogf_assert(nb_variables_per_vertex_ <= 10) ;

        std::vector<Vector3d> K_backup ;
        std::vector<Complex> U_backup[10] ;
        
        Map::Halfedge* h = f->halfedge() ;

        init_extract_tex_coords(h->prev()) ;

        do {
            K_backup.push_back(K_[h->vertex()]) ;
            for(int i=0; i<nb_variables_per_vertex_; i++) {
                U_backup[i].push_back(U_[i][h->vertex()]) ;
            }
            propagate_along(h) ;
            h = h->next() ;
        } while(h != f->halfedge()) ;

        h = f->halfedge() ;
        int id = 0 ;
        do {
            K_[h->vertex()] = K_backup[id] ;
            for(int i=0; i<nb_variables_per_vertex_; i++) {
                U_[i][h->vertex()] = U_backup[i][id] ;
            }
            h = h->next() ;
            id++ ;
        } while(h != f->halfedge()) ;
    }


    void PeriodicParameterizer::extract_tex_coords_in_path(const std::vector<Map::Halfedge*>& path) {
        std::vector<Vector3d> K_backup ;
        std::vector<Complex> U_backup[10] ;

        for(unsigned int i=0; i<path.size(); i++) {
            Map::Vertex* v = path[i]->vertex() ;
            K_backup.push_back(K_[v]) ;
            for(int j=0; j<nb_variables_per_vertex_; j++) {
                U_backup[j].push_back(U_[j][v]) ;
            }
        }

        init_extract_tex_coords(path[0]->prev()) ;
        propagate_along(path[0]) ;
        copy_tex_coords(path[0], path[0]->next()) ;
        for(unsigned int i=1; i<path.size(); i++) {
            copy_tex_coords(path[i-1], path[i]->prev()) ;
            propagate_along(path[i]) ;
        }


        for(unsigned int i=0; i<path.size(); i++) {
            Map::Vertex* v = path[i]->vertex() ;
            K_[v] = K_backup[i] ;
            for(int j=0; j<nb_variables_per_vertex_; j++) {
                U_[j][v] = U_backup[j][i] ;
            }
        }

    }

    void PeriodicParameterizer::extract_iso_curves(Graph* line, double value, double modulo, bool border) {

        ogf_assert(nb_variables_per_vertex_ <= 10) ;

        MapIsoCurvesExtractor extractor(map_, line) ;

        double mins[10] ;
        double maxes[10] ;

        FOR_EACH_FACET(Map, map_, it) {
            extract_tex_coords_in_facet(it) ;
            get_facet_min_max(it, mins, maxes) ;

            for(int i=0; i<nb_variables_per_vertex_; i++) {
                double u = value ;
                while(u > mins[i]) { u -= modulo ; }
                while(u < mins[i]) { u += modulo ; }
                do {
                    extractor.set_iso_value(u) ;
                    extractor.begin_facet(it) ;
                    Map::Halfedge* h = it->halfedge() ;
                    do {
                        extractor.vertex(h->vertex(), theta_[i][h]) ;
                        h = h->next() ;
                    } while(h != it->halfedge()) ;
                    extractor.end_facet() ;
                    u += modulo ;
                } while(u < maxes[i]) ;
            }
        }        

        if(border) {
            extractor.extract_border() ;
        }

        extract_tex_coords_in_facets() ;

        MapTrimmer trimmer(line) ;
        std::cerr << "Intersect" << std::endl ;
        trimmer.intersect_trimming_curve() ;
    }

    void PeriodicParameterizer::get_facet_min_max(Map::Facet* f, double* mins, double* maxes) {
        for(int i=0; i<nb_variables_per_vertex_; i++) {
            mins[i] = 1e30 ;
            maxes[i] = -1e30 ;
        }
        Map::Halfedge* h = f->halfedge() ;
        do {
            for(int i=0; i<nb_variables_per_vertex_; i++) {
                mins[i] = ogf_min(mins[i], theta_[i][h]) ;
                maxes[i] = ogf_max(maxes[i], theta_[i][h]) ;
            }
            h = h->next() ;
        } while(h != f->halfedge()) ;
    }

    static Vector3d normed_perp_part(Vector3d& v, Vector3d n) {
        n.normalize();
        Vector3d result = v - (v*n)*n;
        result.normalize();
        return result;
    }

    // ensure that alpha belongs to [ref - M_PI, ref + M_PI]
    static double normalize(double alpha, double ref) {
        if(Numeric::is_nan(alpha)) {
            std::cerr << "Reconstruct: alpha is a Nan" << std::endl ;
            return 0.0 ;
        }
        if(Numeric::is_nan(ref)) {
            std::cerr << "Reconstruct: ref is a Nan" << std::endl ;
            return 0.0 ;
        }
        double result = alpha ;
        int count = 0 ;
        while(ref - result > M_PI) {
            result += 2.0 * M_PI ;
            count ++ ;
            if(count > 100) {
                std::cerr << "Reconstruct: more than 100 iters" << std::endl ;
                return 0.0 ;
            }
        }
        count = 0 ;
        while(result - ref > M_PI) {
            result -= 2.0 * M_PI ;
            count ++ ;
            if(count > 100) {
                std::cerr << "Reconstruct: more than 100 iters" << std::endl ;
                return 0.0 ;
            }
        }
        return result ;
    }

    void PeriodicParameterizer::init_extract_tex_coords(Map::Halfedge* h) {
        for(int i=0; i<nb_variables_per_vertex_; i++) {
            theta_[i][h] = U_[i][h->vertex()].angle() ;
        }
        show_tex_coords(h) ;
    }


    void PeriodicParameterizer::reorient(Map::Vertex* org, Map::Vertex* dest) {
        Vector3d& vector_org = K_[org] ;
        Vector3d& vector_dest = K_[dest] ;
        
        Vector3d normal_org = Geom::vertex_normal(org);
        Vector3d normal_dest = Geom::vertex_normal(dest);
        Vector3d vector_org_proj = normed_perp_part(vector_org, normal_dest);
        vector_dest = normed_perp_part(vector_dest, normal_dest);

        double delta = M_PI / double(nb_variables_per_vertex_) ;

        int num_iter=0;
        // find the field at dest vertex in the same direction as the field at org
        // update the solution (complex numbers) to remain compatible with the field orientation
        // it solves both the problem of symetry (due to tensor) and the pb of rotation (due to quads)
        double mincos = ::cos(delta/2.0) - 1e-6; 
        
        while (vector_dest * vector_org_proj < mincos && num_iter++< nb_variables_per_vertex_ * 2 + 1) {
            vector_dest = rot(vector_dest, normal_dest, delta) ;
            Complex U = U_[0][dest] ;
            for(int i=0; i<(nb_variables_per_vertex_-1); i++) {
                U_[i][dest] = U_[i+1][dest] ;
            }
            U_[nb_variables_per_vertex_-1][dest] = Complex(U.real(), -U.imaginary()) ;
        }
        if(num_iter == 2*nb_variables_per_vertex_) {
            Logger::warn("NicoStuff") << "Oulah" << std::endl ;
        }
    }

    void PeriodicParameterizer::propagate_along(Map::Halfedge* h) {
        Map::Vertex* org = h->prev()->vertex();
        Map::Vertex* dest = h->vertex();
        reorient(org, dest) ;

        Vector3d& vector_org = K_[org] ;
        Vector3d& vector_dest = K_[dest] ;
        Vector3d normal_org = Geom::vertex_normal(org);
        Vector3d normal_dest = Geom::vertex_normal(dest);
        double wavelength = 0.5 * (wavelength_[org] + wavelength_[dest]) ;
        double delta = M_PI / double(nb_variables_per_vertex_) ;
        Vector3d Vh = Geom::vector(h) ;
        Vector3d V1 = vector_org ;
        Vector3d V2 = vector_dest ;
        for(int i=0; i<nb_variables_per_vertex_; i++) {
            Vector3d V12 = 0.5 * (V1 + V2) ;
            V12.normalize() ;
            double org_u = theta_[i][h->prev()] ;
            double expected_u = org_u - 2.0 * M_PI * (Vh * V12)  / wavelength ;
            theta_[i][h] = normalize(U_[i][dest].angle(), expected_u) ;
            V1 = rot(V1, normal_org, delta) ;
            V1.normalize() ;
            V2 = rot(V2, normal_dest, delta) ;
            V2.normalize() ;
        }
        show_tex_coords(h) ;
    }

    void PeriodicParameterizer::copy_tex_coords(Map::Halfedge* from, Map::Halfedge* to) {
        for(int i=0; i<nb_variables_per_vertex_; i++) {
            theta_[i][to] = theta_[i][from] ;
        }
        show_tex_coords(to) ;
    }

    void PeriodicParameterizer::propagate_accross(Map::Halfedge* h) {
        copy_tex_coords(h->prev(), h->opposite()) ;
        copy_tex_coords(h, h->opposite()->prev()) ;
    }

    void PeriodicParameterizer::interpolate(Map::Vertex* v, Map::Vertex* org, Map::Vertex* dest) {
        Vector3d K_backup = K_[dest];
        Complex U_backup[10] ;
        for(int i=0; i<nb_variables_per_vertex_; i++) {
            U_backup[i] = U_[i][dest] ;
        }

        reorient(org, dest) ;
        Vector3d e = dest->point() - org->point() ;
        if(e.norm() < 1e-30) {
            std::cerr << "zero len edge" << std::endl ;
        }
        double w = (v->point() - org->point()) * e / e.norm2() ;
        K_[v] = w * K_[dest] + (1.0 - w) * K_[org] ;
        Vector3d K1 = K_[org] ;
        Vector3d K2 = K_[dest] ;
        Vector3d normal_org = Geom::vertex_normal(org);
        Vector3d normal_dest = Geom::vertex_normal(dest);
        double wavelength = 0.5 * (wavelength_[org] + wavelength_[dest]) ;

        wavelength_[v] = wavelength ;

        double delta = M_PI / double(nb_variables_per_vertex_) ;
        for(int i=0; i<nb_variables_per_vertex_; i++) {
            double a1 = U_[i][org].angle() ;
            double a2 = U_[i][dest].angle() ;
            Vector3d K = K1 + K2 ; K.normalize() ;
            double expected_a2 = a1 - 2.0 * M_PI * (e * K)  / wavelength ;
            a2 = normalize(a2, expected_a2) ;
            double a = w * a2 + (1.0 - w) * a1 ;
            U_[i][v] = Complex(::cos(a), ::sin(a)) ;
            K1 = rot(K1, normal_org, delta) ;
            K1.normalize() ;
            K2 = rot(K2, normal_dest, delta) ;
            K2.normalize() ;
        }        
        K_[dest]  = K_backup;
        for(int i=0; i<nb_variables_per_vertex_; i++) {
            U_[i][dest] = U_backup[i]  ;
        }
    }

    void PeriodicParameterizer::parameterize_new_vertices() {

        MapVertexLock is_locked(map_) ;

        FOR_EACH_VERTEX(Map, map_, it) {
            is_locked[it] = (wavelength_[it] == 0.0) ;
        }

        FOR_EACH_VERTEX(Map, map_, it) {
            if(wavelength_[it] == 0.0) {
                std::vector<Map::Vertex*> neighbors ;
                Map::Halfedge* h = it->halfedge() ;
                do {
                    Map::Vertex* neigh = h->opposite()->vertex() ;
                    if(!is_locked[neigh]) {
                        neighbors.push_back(neigh) ;
                    }
                    h = h->next_around_vertex() ;
                } while(h != it->halfedge()) ;
                if(neighbors.size() == 2) {
                    interpolate(it, neighbors[0], neighbors[1]) ;
                } 
            }
        }

        FOR_EACH_VERTEX(Map, map_, it) {
            is_locked[it] = (wavelength_[it] == 0.0) ;
        }
        
        FOR_EACH_EDGE(Map, map_, it) {
            Map::Vertex* v1 = it->opposite()->vertex() ;
            Map::Vertex* v2 = it->vertex() ;
            if(is_locked[v1] && is_locked[v2] && v1->degree() == 4 && v2->degree() == 4) {
                Map::Vertex* v0 = it->opposite()->next()->opposite()->next()->vertex() ;
                Map::Vertex* v3 = it->next()->opposite()->next()->vertex() ;
                if(!is_locked[v0] && !is_locked[v3]) {
                    interpolate(v1, v0, v3) ;
                    interpolate(v2, v0, v3) ;
                }
            }
        }

        FOR_EACH_VERTEX(Map, map_, it) {
            is_locked[it] = (wavelength_[it] == 0.0) ;
        }

        FOR_EACH_VERTEX(Map, map_, it) {
            if(is_locked[it]) {
                if(it->degree() == 4) {
                    Map::Halfedge* h = it->halfedge() ;
                    Map::Vertex* v1 = h->opposite()->vertex() ;
                    Map::Vertex* v2 =  h->next()->opposite()->next()->vertex() ;
                    if(!is_locked[v1] && !is_locked[v2]) {
                        interpolate(it, v1, v2) ;
                    } else {
                        h = h->next_around_vertex() ;
                        v1 = h->opposite()->vertex() ;
                        v2 =  h->next()->opposite()->next()->vertex() ;
                        if(!is_locked[v1] && !is_locked[v2]) {
                            interpolate(it, v1, v2) ;
                        }
                    }
                } else if(it->degree() == 3) {
                    double best_cos = -1.0 ;
                    Map::Halfedge* cur_h = it->halfedge()->opposite() ;
                    Map::Halfedge* h = cur_h ;
                    do {
                        Map::Halfedge* next_h = cur_h->prev()->opposite() ;
                        double cur_cos = Geom::cos_angle(Geom::vector(cur_h), Geom::vector(next_h)) ;
                        if(::fabs(cur_cos + 1.0) < ::fabs(best_cos + 1.0)) {
                            best_cos = cur_cos ;
                            h = cur_h ;
                        }
                        cur_h = next_h ;
                    } while(cur_h != it->halfedge()->opposite()) ;
                    if(::fabs(best_cos + 1.0) < 0.2) {
                        Map::Vertex* v1 = h->vertex() ;
                        Map::Vertex* v2 = h->prev()->opposite()->vertex() ;
                        if(!is_locked[v1] && !is_locked[v2]) {
                            interpolate(it, v1, v2) ;
                        }
                    }
                }
            }
        }

        FOR_EACH_VERTEX(Map, map_, it) {
            is_locked[it] = (wavelength_[it] == 0.0) ;
        }

/*
        MapVertexAttribute<Graph::Vertex*> embedded(map_, "embedded") ;
        FOR_EACH_VERTEX(Map, map_, it) {
            if(embedded[it] != nil && embedded[it]->degree() > 2) {
                for(int i=0; i<nb_variables_per_vertex_; i++) {
                    U_[i][it] = Complex(1,0) ;
                }
            }
        }        
*/
        extract_tex_coords_in_facets() ;
    }

    //---------------------------------------------------------------------------------------------------------------------------------------------------------

}
