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

// Idea#1 It is also possible to let client code
// specify beta's as an halfedge attribute (to
// do anisotropic parameterizations, for instance ...)
//
// Idea#2 Check the influence of the tolerance on the result
//
// Idea#3 Try compiling SuperLU with ICC
//
// Idea#4 Try a X-Large model on the Origin3K with //-SuperLU
//

// - Delta_inv diagonal matrix ( 1.0 / 2.w(i) ); do not store w anymore.
// - Some little optimizations to do when low_mem_ is set (no need to 
//    zero everything)

// #define OGF_PARANOID

#include <OGF/cells/map_algos/abf_plus_plus.h>
#include <OGF/cells/map_algos/angles_to_uv.h>
#include <OGF/cells/map_algos/enumerate.h>
#include <OGF/cells/map_algos/map_components.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/math/numeric/super_lu.h>


namespace OGF {


//____________________________________________________________________________
    
    ABFPlusPlusMapParameterizer::ABFPlusPlusMapParameterizer(
    ) : epsilon_(1e-5),
        newton_tolf_(1.0),
        newton_tolx_(1.0),
        max_newton_iter_(10),
        positive_angle_ro_(1.2),
        step_length_factor_(1.0) {
        triangles_only_ = true ;
        log_mode_ = false ;
    }

    void ABFPlusPlusMapParameterizer::allocate_variables() {

        // ------- sizes & indexes ------
        nf_ = nb_triangles(map_) ;
        nalpha_ = 3*nf_ ;
        nint_ = nb_interior_vertices(map_) ;
        nlambda_ = nf_ + 2*nint_ ;
        ntot_ = nalpha_ + nlambda_ ;
        angle_index_.bind(map_) ;

        // ------- ABF variables --------
        alpha_.allocate(nalpha_) ;
        lambda_.allocate(nlambda_) ;
        beta_.allocate(nalpha_) ;
        w_.allocate(nalpha_) ;

        // ------- Step vectors ---------
        dalpha_.allocate(nalpha_) ;
        dlambda1_.allocate(nf_) ;
        dlambda2_.allocate(2*nint_) ;

        // ------- Gradients ------------
        b1_.allocate(nalpha_) ;
        b2_.allocate(nlambda_) ;

        // ------- Jacobian -------------
        J2_.allocate(2*nint_, nalpha_, SparseMatrix::COLUMNS) ;

        // ------- ABF++ ----------------
        J_star_.allocate(2*nint_, nf_, SparseMatrix::COLUMNS) ;
        M_.allocate(2*nint_, 2*nint_, SparseMatrix::ROWS) ;
    }

    void ABFPlusPlusMapParameterizer::deallocate_variables() {
        // ------- sizes & indexes ------
        angle_index_.unbind() ;

        // ------- ABF variables --------
        alpha_.clear() ;
        lambda_.clear() ;
        beta_.clear() ;
        w_.clear() ;

        // ------- Step vectors ---------
        dalpha_.clear() ;
        dlambda1_.clear() ;
        dlambda2_.clear() ;

        // ------- Gradients ------------
        b1_.clear() ;
        b2_.clear() ;

        J2_.deallocate() ;

        // ------- ABF++ ----------------
        J_star_.deallocate() ;
        M_.deallocate() ;
    }


    bool ABFPlusPlusMapParameterizer::do_parameterize_disc(Map* map) {

        map_ = map ;
        allocate_variables() ;

        // Sanity check.
        { FOR_EACH_FACET(Map, map_, it) {
            ogf_assert(it->is_triangle()) ;
        }}

        enumerate_angles() ;
        compute_beta() ;

        MapHalfedgeAttribute<double> angle(map_, "angle") ;

        if(solve_angles()) {
            FOR_EACH_HALFEDGE(Map, map_, it) {
                angle[it] = alpha_(angle_index_[it]) ;
            }
        } else {
            Logger::err("ABF++") << "Did not converge." << std::endl ; 
            Logger::err("ABF++") << "Switching to LSCM" << std::endl ; 
            // Note: AnglesToUV with angles measured on the mesh (i.e. beta's) = LSCM !!!
            FOR_EACH_HALFEDGE(Map, map_, it) {
                angle[it] = beta_(angle_index_[it]) ;
            }
        } 

        deallocate_variables() ;

        AnglesToUV a_to_uv ;
//        a_to_uv.set_mode(ANGLES_ITERATIVE) ;
        a_to_uv.parameterize_disc(map_) ;
        map_ = nil ;
        return true ;
    }

    int ABFPlusPlusMapParameterizer::nb_interior_vertices(Map* map) {
        int result = 0 ;
        FOR_EACH_VERTEX(Map, map, it) {
            if(!it->is_on_border()) {
                result++ ;
            }
        }
        return result ;
    }

    int ABFPlusPlusMapParameterizer::nb_triangles(Map* map) {
        int result = 0 ;
        FOR_EACH_FACET(Map, map, it) {
            result += it->degree() - 2 ;
        }
        return result ;
    }
    

    void ABFPlusPlusMapParameterizer::enumerate_angles() {
        int cur = 0 ;
        FOR_EACH_FACET(Map, map_, f) {
            Map::Halfedge* h = f->halfedge() ;
            do {
                angle_index_[h] = cur ;
                cur++ ;
                h = h->next() ;
            } while(h != f->halfedge()) ;
        }
        // sanity check
        ogf_assert(cur == int(map_->size_of_facets()) * 3) ;
    }

    // Had a hard time finding what was wrong with my code:
    // OK, got it, use h->next() instead of h->next_around_vertex()
    // because h->next_around_vertex() does not return the same
    // angle.
    static inline double corner_angle(OGF::Map::Halfedge* h) {
        double result = M_PI - Geom::angle(
            Geom::vector(h), 
            Geom::vector(h->next())
        ) ;
        result = ogf_max(result, 2.0 * M_PI / 360.0) ;
        return result ;
    }
    
    void ABFPlusPlusMapParameterizer::compute_beta() {
        
        FOR_EACH_VERTEX(Map, map_, vit) {
            // Compute sum_angles
            double sum_angle = 0.0 ;
            {
                Map::Halfedge* h = vit->halfedge() ;
                do {
                    double angle = corner_angle(h) ;
                    if( angle < 5.*M_PI/180 ){
//                        Logger::out("ABF++") << "Small angle encountered: "
//                                        << angle*180.0/M_PI << std::endl ; 
                    }
					if( angle > 170.*M_PI/180 ){
//                        Logger::out("ABF++") << "Large angle encountered: "
//                                        << angle*180.0/M_PI << std::endl ; 
                    }

                    sum_angle += angle ;
                    h = h->next_around_vertex() ;
                } while(h != vit->halfedge()) ; 
            }

            double ratio = 1.0 ;
        
            if(!vit->is_on_border()) {
                ratio =  2.0 * M_PI / sum_angle ;
            }
            
            {
                Map::Halfedge* h = vit->halfedge() ;
                do {
                    if(!h->is_border()) {
                        int angle_ind = angle_index_[h] ;
                        beta_(angle_ind) = corner_angle(h) * ratio ;
                        if( beta_(angle_ind) < 3.*M_PI/180. ) { 
                            beta_(angle_ind) =  3.*M_PI/180.;
                        } else if( beta_(angle_ind) > 175.*M_PI/180. ) { 
                            beta_(angle_ind) =  175.*M_PI/180.;
                        }
                    }
                    h = h->next_around_vertex() ;
                } while(h != vit->halfedge()) ; 
            }
        }
    }


    bool ABFPlusPlusMapParameterizer::solve_angles() {

        bool is_grob = false ; // (dynamic_cast<Grob*>(map_) != nil) ;
        
        MapHalfedgeAttribute<double> angle ;
        if(is_grob) {
            angle.bind(map_, "angle") ;
        }
        
        // Initial values

        perm_.clear() ;
        lambda_.zero() ;
        { for(int i=0; i<nalpha_; i++) {
            alpha_(i) = beta_(i) ;
            w_(i) = 1.0 / (beta_(i) * beta_(i)) ;
        }}

        double errx_k_m_1 = Numeric::big_double ;

        for(int k=0; k<max_newton_iter_; k++) {

            // Compute Jacobian
            J2_.zero() ;
            add_JC2() ;
            add_JC3() ;

            // Compute rhs
            b1_.zero() ;
            b2_.zero() ;
            sub_grad_F() ;
            sub_grad_C1() ;
            sub_grad_C2() ;
            sub_grad_C3() ;

            double errf_k = errf() ;


            Logger::out("ABF++") << "iter= " << k << " errf= " << errf_k 
                               << std::endl ;

/*
            if(Numeric::is_nan(errf_k) || errf_k > 1e9) {
                return false ;
            }
*/

            if(Numeric::is_nan(errf_k) || errf_k > 1e18) {
                return false ;
            }


            if(errf_k <= newton_tolf_) {
                Logger::out("ABF++") << "converged" << std::endl ;
                return true ;
            }

            solve_current_iteration() ;

            double errx ;
            double s = compute_step_length_and_update_weights() ;
            s *= step_length_factor_ ;
            errx = compute_errx_and_update_x(s) ;

            // TODO: since errx is dependent on the size of the mesh,
            // weight the threshold by the size of the mesh.
/*
            if(Numeric::is_nan(errx) || errx > 1e6) {
                Logger::err("ABF++") << "errx: " << errx << std::endl ;
                return false ;
            }
*/

            if(Numeric::is_nan(errx) || errx > 1e15) {
                Logger::err("ABF++") << "errx: " << errx << std::endl ;
                return false ;
            }


            Logger::out("ABF++") << "iter= " << k << " errx= " << errx
                               << std::endl ;

            if(errx <= newton_tolx_) {
                Logger::out("ABF++") << "converged" << std::endl ;
                return true ;
            }

            errx_k_m_1 = errx ;

            if(is_grob) {
                FOR_EACH_HALFEDGE(Map, map_, it) {
                    angle[it] = alpha_(angle_index_[it]) ;
                }
                AnglesToUV a_to_uv ;
                a_to_uv.parameterize_disc(map_) ;
                update_graphics(map_) ;
            }
        }

        Logger::out("ABF++") << "ran out of Newton iters" << std::endl ;
        return true ;
    }

    void ABFPlusPlusMapParameterizer::solve_current_iteration() {
        
        Delta_inv_.allocate(nalpha_) ;
        if(log_mode_) {
            {for(int i=0; i<nalpha_; i++) {
                Delta_inv_(i) = 2.0 * w_(i) ;
            }}
/*  
    // Note: the Hessian terms coming from the modified wheel
    // compatibility constraint can make the coefficients of
    // Delta equal to zero ... commented out.
            {FOR_EACH_VERTEX(Map, map_, it) {
                if(it->is_on_border()) {
                    continue ;
                }
                Map::Halfedge* h = it->halfedge() ;
                do {
                    int i = angle_index_[h->next()] ;
                    Delta_inv_(i) -= lambda_(i) / ogf_sqr(sin(alpha_(i))) ;
                    i = angle_index_[h->prev()] ;
                    Delta_inv_(i) += lambda_(i) / ogf_sqr(sin(alpha_(i))) ;
                    h = h->next_around_vertex() ;
                } while(h != it->halfedge()) ;
            }}
*/
            {for(int i=0; i<nalpha_; i++) {
                ogf_assert(::fabs(Delta_inv_(i)) > 1e-30) ;
                Delta_inv_(i) = 1.0 / Delta_inv_(i) ;
            }}
        } else {
            for(int i=0; i<nalpha_; i++) {
                Delta_inv_(i) = 1.0 / (2.0 * w_(i)) ;
            }
        }

        
        // 1) Create the pieces of J.Delta^-1.Jt
        // 1.1) Diagonal part: Delta*^-1
        Delta_star_inv_.allocate(nf_) ;
        {for(int i=0; i<nf_; i++) {
            Delta_star_inv_(i) = 
                1.0 / (
                    Delta_inv_(3*i) + Delta_inv_(3*i+1) + Delta_inv_(3*i+2) 
                ) ;
        }}

        // 1.2) J*  = J2.Delta^-1.J1^t
        J_star_.zero() ;
        { for(int j=0; j<nalpha_; j++) {
            const SparseMatrix::Column& Cj = J2_.column(j) ;
            for(int ii=0; ii<Cj.nb_coeffs(); ii++) {
                const Coeff& c = Cj.coeff(ii) ;
                J_star_.add(c.index, j / 3, c.a * Delta_inv_(j)) ;
            }
        }}
        // Note: J** does not need to be built, it is directly added to M.

        // 2) Right hand side: b1* and b2*

        // 2.1) b1* = J1.Delta^-1.b1 - b2[1..nf]
        b1_star_.allocate(nf_) ;
        {for(int i=0; i<nf_; i++) {
            b1_star_(i) = 
                Delta_inv_(3*i  ) * b1_(3*i  ) + 
                Delta_inv_(3*i+1) * b1_(3*i+1) + 
                Delta_inv_(3*i+2) * b1_(3*i+2) - b2_(i) ;
        }}
        // 2.2) b2* = J2.Delta^-1.b1 - b2[nf+1 .. nf+2.nint-1]
        b2_star_.allocate(2*nint_) ;
        b2_star_.zero() ;
        add_J_D_x(b2_star_, J2_, Delta_inv_, b1_) ;
        {for(int i=0; i<2*nint_; i++) {
            b2_star_(i) -= b2_(nf_+i) ;
        }}


        // 3) create final linear system 
        
        // 3.1) M = J*.Delta*^-1.J*^t - J**
        //       where J** = J2.Delta^-1.J2^t
        M_.zero() ;
        add_J_D_Jt(M_, J_star_, Delta_star_inv_) ;
        sub_J_D_Jt(M_, J2_, Delta_inv_) ;

        // 3.2) r = J*.Delta*^-1.b1* - b2*
        r_.allocate(2*nint_) ;
        r_.zero() ;
        add_J_D_x(r_, J_star_, Delta_star_inv_, b1_star_) ;
        r_ -= b2_star_ ;
        
        // First call initializes perm_
        solve_super_lu(M_, r_, dlambda2_, perm_, true) ;

        // 4) compute dlambda1 and dalpha in function of dlambda2
        
        // 4.1) dlambda1 = Delta*^-1 ( b1* - J*^t dlambda2 )
        mult_transpose(J_star_, dlambda2_, dlambda1_) ;
        {for(int i=0; i<nf_; i++) {
            dlambda1_(i) = Delta_star_inv_(i) * (b1_star_(i) - dlambda1_(i)) ;
        }}
        
        // 4.2) Compute dalpha in function of dlambda:
        // dalpha = Delta^-1( b1 -  J^t.dlambda                    )
        //        = Delta^-1( b1 - (J1^t.dlambda1 + J2^t.dlambda2) )
        mult_transpose(J2_, dlambda2_, dalpha_) ;
        {for(int i=0; i<nf_; i++) {
            dalpha_(3*i)   += dlambda1_(i) ;
            dalpha_(3*i+1) += dlambda1_(i) ;
            dalpha_(3*i+2) += dlambda1_(i) ;
        }}
        {for(int i=0; i<nalpha_; i++) {
            dalpha_(i) = Delta_inv_(i) * (b1_(i) - dalpha_(i)) ;
        }}

        // Release as much memory as possible.
        Delta_inv_.clear() ;
        Delta_star_inv_.clear() ;
        J_star_.clear() ;
        b1_star_.clear() ;
        b2_star_.clear() ;
        // Note: M_ already cleared by SuperLU wrapper when in low-mem mode
        r_.clear() ;
    }


    // ----------------------------- Jacobian -----------------------------

    void ABFPlusPlusMapParameterizer::add_JC2() {
        int i = 0 ;
        FOR_EACH_VERTEX(Map, map_, it) {
            if(it->is_on_border()) {
                continue ;
            }
            Map::Halfedge* h = it->halfedge() ;
            do {
                J2_.add(i, angle_index_[h], 1.0) ;
                h = h->next_around_vertex() ;
            } while(h != it->halfedge()) ;
            i++ ;
        }
    }
    
    void ABFPlusPlusMapParameterizer::compute_product_sin_angles(
        Map::Vertex* v, double& prod_prev_sin, double& prod_next_sin
    ) {
        prod_prev_sin = 1.0 ;
        prod_next_sin = 1.0 ;
        OGF::Map::Halfedge* h = v->halfedge() ;
        do {
            prod_prev_sin *= sin(alpha_(angle_index_[h->prev()])) ;
            prod_next_sin *= sin(alpha_(angle_index_[h->next()])) ;
            h = h->next_around_vertex() ;
        } while(h != v->halfedge()) ;
    }

    void ABFPlusPlusMapParameterizer::compute_sum_log_sin_angles(
        Map::Vertex* v, double& sum_prev_sin, double& sum_next_sin
    ) {
        sum_prev_sin = 0.0 ;
        sum_next_sin = 0.0 ;
        OGF::Map::Halfedge* h = v->halfedge() ;
        do {
            sum_prev_sin += log(sin(alpha_(angle_index_[h->prev()]))) ;
            sum_next_sin += log(sin(alpha_(angle_index_[h->next()]))) ;
            h = h->next_around_vertex() ;
        } while(h != v->halfedge()) ;
    }


    void ABFPlusPlusMapParameterizer::add_JC3() {
        if(log_mode_) {
            int i = nint_ ;
            FOR_EACH_VERTEX(Map, map_, it) {
                if(it->is_on_border()) {
                    continue ;
                }
                Map::Halfedge* h = it->halfedge() ;
                do {
                    int j = angle_index_[h->next()] ;
                    J2_.add(i,j, cos(alpha_(j)) / sin(alpha_(j))) ;
                    j = angle_index_[h->prev()] ;
                    J2_.add(i,j,-cos(alpha_(j)) / sin(alpha_(j))) ;
                    h = h->next_around_vertex() ;
                } while(h != it->halfedge()) ;
                i++ ;
            }
        } else {
            int i = nint_ ;
            FOR_EACH_VERTEX(Map, map_, it) {
                if(it->is_on_border()) {
                    continue ;
                }
                double prod_prev_sin ;
                double prod_next_sin ;
                compute_product_sin_angles(it, prod_prev_sin, prod_next_sin) ;
                Map::Halfedge* h = it->halfedge() ;
                do {
                    int j = angle_index_[h->next()] ;
                    J2_.add(i, j, prod_next_sin * cos(alpha_(j)) / sin(alpha_(j)));
                    j = angle_index_[h->prev()] ;
                    J2_.add(i, j,-prod_prev_sin * cos(alpha_(j)) / sin(alpha_(j)));
                    h = h->next_around_vertex() ;
                } while(h != it->halfedge()) ;
                i++ ;
            }
        }
    }


    // ----------------------- Right hand side -----------------------

    void ABFPlusPlusMapParameterizer::sub_grad_F() {
        for(int i = 0; i < nalpha_; i++ ) {
            b1_(i) -= 2.0 * w_(i) * ( alpha_(i) - beta_(i) );
        }
    }
    
    // For each triangle: sum angles - PI
    void ABFPlusPlusMapParameterizer::sub_grad_C1() {
        { for(int i=0; i < nf_; i++) {
            for(int j=0; j<3; j++) {
                b1_(3*i+j) -= lambda_(i) ;
            }
        }}
        { for(int i=0; i < nf_; i++) {
            b2_(i) -= 
                alpha_(3*i) + alpha_(3*i+1) + alpha_(3*i+2) - M_PI ;
        }}
    }
    
    // For each vertex: sum incident angles - 2.PI
    void ABFPlusPlusMapParameterizer::sub_grad_C2() {
        int i = nf_ ;
        FOR_EACH_VERTEX(Map, map_, it) {
            if(it->is_on_border()) {
                continue ;
            }
            Map::Halfedge* h = it->halfedge() ;
            do {
                b2_(i) -= alpha_(angle_index_[h]) ;
                b1_(angle_index_[h]) -= lambda_(i) ;
                h = h->next_around_vertex() ;
            } while(h != it->halfedge()) ;
            b2_(i) += 2.0 * M_PI ;
            i++ ;
        }
    }
    
    // For each vertex: prod sin(next angle) - prod sin(prev angle)
    void ABFPlusPlusMapParameterizer::sub_grad_C3() {
        if(log_mode_) {
            int i = nf_ + nint_ ;
            FOR_EACH_VERTEX(Map, map_, it) {
                if(it->is_on_border()) {
                    continue ;
                }
                
                double sum_prev_sin ;
                double sum_next_sin ;
                compute_sum_log_sin_angles(it, sum_prev_sin, sum_next_sin) ;
                
                b2_(i) -= (sum_next_sin - sum_prev_sin) ;
                
                Map::Halfedge* h = it->halfedge() ;
                do {
                    int j = angle_index_[h->next()] ;
                    b1_(j) -= lambda_(i) * cos(alpha_(j)) / sin(alpha_(j)) ;
                    j = angle_index_[h->prev()] ;
                    b1_(j) += lambda_(i) * cos(alpha_(j)) / sin(alpha_(j)) ;
                    
                    h = h->next_around_vertex() ;
                } while(h != it->halfedge()) ;
                i++ ;
            }
        } else {
            int i = nf_ + nint_ ;
            FOR_EACH_VERTEX(Map, map_, it) {
                if(it->is_on_border()) {
                    continue ;
                }
                double prod_prev_sin ;
                double prod_next_sin ;
                compute_product_sin_angles(it, prod_prev_sin, prod_next_sin) ;
                
                b2_(i) -= prod_next_sin - prod_prev_sin ;
                
                Map::Halfedge* h = it->halfedge() ;
                do {
                    int j = angle_index_[h->next()] ;
                    b1_(j) -= 
                        lambda_(i) * prod_next_sin * cos(alpha_(j)) / sin(alpha_(j)) ;
                    
                    j = angle_index_[h->prev()] ;
                    b1_(j) += 
                        lambda_(i) * prod_prev_sin * cos(alpha_(j)) / sin(alpha_(j)) ;
                    
                    h = h->next_around_vertex() ;
                } while(h != it->halfedge()) ;
                i++ ;
            }
        }
    }

    double ABFPlusPlusMapParameterizer::compute_errx_and_update_x(double s) {
        double result = 0 ;

        // alpha += s * dalpha 
        {for(int i=0; i<nalpha_; i++) {
            double dai = s * dalpha_(i) ;
            alpha_(i) += dai ;
            result += ::fabs(dai) ;
        }}

        // lambda += s * dlambda
        {for(int i=0; i<nf_; i++) {
            double dai = s * dlambda1_(i) ;
            lambda_(i) += dai ;
            result += ::fabs(dai) ;
        }}
        {for(int i=0; i<2*nint_; i++) {
            double dai = s * dlambda2_(i) ;
            lambda_(nf_+i) += dai ;
            result += ::fabs(dai) ;
        }}

        return result ;
    }
    
    // ______ Convergence control ____________________________________

    double ABFPlusPlusMapParameterizer::compute_step_length_and_update_weights(
    ) {
        double ratio = 1.0 ;

        for(int i=0; i<nalpha_; i++) {
            if(alpha_(i) + dalpha_(i) < 10.0 * epsilon_) {
                double r1 = -.5 * (alpha_(i) - 10.0 * epsilon_)/ dalpha_(i) ;
                ratio = ogf_min(ratio, r1) ;
                w_(i) *= positive_angle_ro_ ;
/*
                std::cerr << " w  ( at 0 ) " 
                          << i << " is " << w_(i) << std::endl;
                std::cerr << " val " 
                          << alpha_(i) + dalpha_(i) << " was " 
                          << alpha_(i) << std::endl;
*/
                
            } else if(alpha_(i) + dalpha_(i) > M_PI - 10.0 * epsilon_) {
                double r1 = .5*(M_PI - alpha_(i)+10.0 * epsilon_) / dalpha_(i);
//                ratio = ogf_min(ratio, r1) ;
                w_(i) *= positive_angle_ro_ ;
/*
                std::cerr << " w ( over 180) " << i << " is " 
                          << w_(i) << std::endl;
                std::cerr << " val " << alpha_(i) + dalpha_(i) << " was " 
                          << alpha_(i) << std::endl;
				*/           
			}
			
		}
        return ratio ;
    }

    double ABFPlusPlusMapParameterizer::errf() const {
        double result = 0 ;
        {for(int i=0; i<nalpha_; i++) {
            result += ::fabs(b1_(i)) ;
        }}
        {for(int i=0; i<nlambda_; i++) {
            result += ::fabs(b2_(i)) ;
        }}
        return result ;
    }

    //____________ Utilities _____________________________________________

    void ABFPlusPlusMapParameterizer::add_J_D_x(
        Vector& y, 
        const SparseMatrix& J, Vector& D, Vector& x
    ) {
        ogf_parano_assert(y.size() == J.m()) ;
        ogf_parano_assert(D.size() == J.n()) ;
        ogf_parano_assert(x.size() == J.n()) ;

        for(unsigned int j=0; j<D.size(); j++) {
            const SparseMatrix::Column& Cj = J.column(j) ;
            for(int ii=0; ii<Cj.nb_coeffs(); ii++) {
                const Coeff& c = Cj.coeff(ii) ;
                y(c.index) += c.a * D(j) * x(j) ;
            }
        }
    }
    
    void ABFPlusPlusMapParameterizer::add_J_D_Jt(
        SparseMatrix& M, const SparseMatrix& J, Vector& D
    ) {
        ogf_parano_assert(M.m() == J.m()) ;
        ogf_parano_assert(M.n() == J.m()) ;
        ogf_parano_assert(D.size() == J.n()) ;

        for(unsigned int j=0; j<D.size(); j++) {
            const SparseMatrix::Column& Cj = J.column(j) ;
            for(int ii1=0; ii1<Cj.nb_coeffs(); ii1++) {        
                for(int ii2=0; ii2<Cj.nb_coeffs(); ii2++) {                
                    M.add(
                        Cj.coeff(ii1).index, Cj.coeff(ii2).index,
                        Cj.coeff(ii1).a * Cj.coeff(ii2).a * D(j)
                    ) ;
                }
            }
        }
    }

    void ABFPlusPlusMapParameterizer::sub_J_D_Jt(
        SparseMatrix& M, const SparseMatrix& J, Vector& D
    ) {
        ogf_parano_assert(M.m() == J.m()) ;
        ogf_parano_assert(M.n() == J.m()) ;
        ogf_parano_assert(D.size() == J.n()) ;

        for(unsigned int j=0; j<D.size(); j++) {
            const SparseMatrix::Column& Cj = J.column(j) ;
            for(int ii1=0; ii1<Cj.nb_coeffs(); ii1++) {        
                for(int ii2=0; ii2<Cj.nb_coeffs(); ii2++) {                
                    M.add(
                        Cj.coeff(ii1).index, Cj.coeff(ii2).index,
                        - Cj.coeff(ii1).a * Cj.coeff(ii2).a * D(j)
                    ) ;
                }
            }
        }
    }

    //___________________________________________________________________

}
