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
 *
 * This code from B. Vallet derivates from A. Sheffer's "Angle Based Flattening",
 * adapted for Graphite by B. Levy.
 * The idea is that using dot products of adjacent faces of each triangles,
 * instead of angles in ABF, as variables is more efficient
 * to constrain the jacobian of a mapping.
 */

#include <OGF/parameterizer/algos/dpbf.h>
#include <OGF/parameterizer/algos/map_statistics.h>
#include <OGF/cells/map_algos/angles_to_uv.h>
#include <OGF/cells/map_algos/enumerate.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/math/numeric/super_lu.h>
#include <math.h>

//#include <OGF/scene_graph/grob/grob.h>

//________________________________________________________________________
namespace OGF {

    /*-+-+-* Constructor *-+-+-*/

    MapParameterizerDPBF::MapParameterizerDPBF() :
        mu_(1000.0),		
        mu_multiplier_(0.4),	
        mu_moderator_(0.9),	
        mu_max_(1e9), 
        step_length_(0.05),	
        details_(false),	
        isoarea_(false),	
        remember_(false),	
        tol_(5.0),		
        max_pass_(10),		
        pass_length_(6) {
        triangles_only_ = true ;
        min_detail_ = 0.5 ;
    }

    void MapParameterizerDPBF::allocate_variables() {
        nf_     = map_->size_of_facets() ;
        ngamma_ = 3 * nf_ ;
        nie_    = nb_interior_edges(map_) ;
        ntot_   = ngamma_ + nie_ ;
        dp_index_.bind(map_) ;
        triangle_index_.bind(map_) ;

        dp_.allocate(ngamma_) ;
        errp_.allocate(ngamma_) ;
        area_.allocate(nf_) ;
        w_.allocate(nf_) ;
        x_.allocate(ntot_) ;
        J_.allocate(ntot_,ntot_, SparseMatrix::ROWS) ;
        b_.allocate(ntot_) ;
        delta_.allocate(ntot_) ;

        // Infos on the surface
        Logger::out("DPBF") << "Model has "
                            << nf_ << " faces and "
                            << nie_ << " interior edges, Solver has "
                            << ntot_ << " variables, "
                            << ngamma_ << " gammas and "
                            << nie_ << " lambdas."
                            << std::endl ;
    }

    void MapParameterizerDPBF::deallocate_variables() {
        dp_index_.unbind() ;
        triangle_index_.unbind() ;
        dp_.clear() ;
        errp_.clear() ;
        area_.clear() ;
        w_.clear() ;
        x_.clear() ;
        J_.deallocate() ;
        b_.clear() ;
        delta_.clear() ;
        perm_.clear() ;
    }


    int MapParameterizerDPBF::nb_interior_edges(Map* map) {
        int result = 0 ;
        FOR_EACH_EDGE(Map, map, it) {
            if(!it->is_border_edge()) {
                result++ ;
            }
        }
        return result ;
    }

    /*-+-+-* Tuning *-+-+-*/

    // Authalic and planar energy multiplier
    void MapParameterizerDPBF::set_mu(double val){mu_ = val ;}

    // Increases mu after each iteration
    void MapParameterizerDPBF::set_mu_multiplier(double val){mu_multiplier_ = val ;}

    // Decreases mu_multiplier after each iteration */
    void MapParameterizerDPBF::set_mu_moderator(double val){mu_moderator_ = val ;}

    // True for parameterization adapted to the geometric level of detail
    void MapParameterizerDPBF::set_details(bool val){details_ = val ;}

    // Funny parameterisation where all 2D triangles has same area
    void MapParameterizerDPBF::set_isoarea(bool val){isoarea_ = val ;}

    // Remembers the dot products of a previous DPBF if it exist
    void MapParameterizerDPBF::set_remember(bool val){remember_ = val ;}

    // Newton's method step length
    void MapParameterizerDPBF::set_step_length(double val){step_length_ = val ;}

    // Minimum geometric level of detail
    // (added to the real value so the objective Jacobian doesn't go too low)
    void MapParameterizerDPBF::set_min_detail(double val){min_detail_ = val ;}

    // Maximum number of passes
    void MapParameterizerDPBF::set_max_pass(int val){max_pass_ = val ;}

    // Number of iterations of a pass
    void MapParameterizerDPBF::set_pass_length(int val){pass_length_ = val ;}

    /*-+-+-* Main steps *-+-+-*/
		
    // Initializations of the DPBF
    void MapParameterizerDPBF::init() {
        enumerate_and_compute_3D_constants() ;

        // Initial values
        if(remember_) {
            FOR_EACH_HALFEDGE(Map, map_, Hit) {
                if(!Hit->is_border()) {
                    int i = dp_index_[Hit] ;
                    x_(i) = - (Geom::vector2d(Hit) * Geom::vector2d(Hit->next())) ;
                }
            }
        } else {
            for(int i = 0; i < ngamma_; i++) {
                x_(i) = dp_(i) ;
            }
        }
    }
	
    // Application of the DPBF.
    bool MapParameterizerDPBF::do_parameterize_disc(Map* map) {
        map_ = map ;
        allocate_variables() ;
        init() ;
        bool result = true ;
        double prev_def = Numeric::big_double ;
        for(int i = 0; i < max_pass_; i++) {
            // shortcut when superLU fails
            if(!pass()) {
                result = false ;
                break ;
            } 

            /*Grob* grob = dynamic_cast<Grob*>(map_) ;
            if(grob != nil) {
                grob->update() ;
            }
*/
            MapStatistics stats(map_) ;
            stats.init() ;
            double def = stats.average_area_deformation() ;
            Logger::out("DPBF") << "Pass " << i+1 << " -> " << def << "%" << std::endl ;

            // convergence criterion

            if(def < tol_) {
                result = true ;
                break ;
            }
            if(def > prev_def) {
//                deallocate_variables() ;
//                return false ;
//                break ;
            }
            prev_def = def ;
        }
        deallocate_variables() ;
        return result ;
    }

    // Computes objective Jacobian, areas, weights and dot products on the 3D surface
    void MapParameterizerDPBF::enumerate_and_compute_3D_constants() {
        int cur_tri = 0, cur_dp = 0 ;
        // for visualisation
        // Jacobian estimated at vertices and facets
        MapVertexAttribute<double> Jv(map_, "Jv") ;
        MapFacetAttribute<double> Jf(map_, "Jf") ;
        {FOR_EACH_VERTEX(Map, map_, v) { Jv[v] = 0 ;}}

        // Areas
        {FOR_EACH_FACET(Map, map_, f) {
            triangle_index_[f] = cur_tri ;
            Map::Halfedge* h = f->halfedge() ;
            area_(cur_tri) = (isoarea_? 1.0 : 2.0 * Geom::triangle_area(f)) ;
            Jf[f] = (isoarea_? 1 / (2.0 * Geom::triangle_area(f)) : 1.0) ;
			
            // computation of geometric level of detail
            if(details_) {	
                Vector3d n = Geom::facet_normal(f) ;
                double coef = 0 ;
                do{
                    if(!h->opposite()->is_border())
                        coef += tol_ + 
                            Geom::angle(n, Geom::facet_normal(h->opposite()->facet())) ;
                    h = h->next() ;
                } while(h != f->halfedge()) ;
                area_(cur_tri) *= coef ;
                Jf[f] *= coef ;
            }
            // estimate of the Jacobian on a vertex to allow
            // its representation with GRAPHITE's "scalar" shader
            do{
                Jv[h->vertex()] += Jf[f] ;
                h = h->next() ;
            } while(h != f->halfedge()) ;

            // weight = 1 / area^2
            w_(cur_tri) = 1/(area_(cur_tri) * area_(cur_tri)) ;
            cur_tri++ ;
        }}

        {FOR_EACH_VERTEX(Map, map_, v) {
            Map::Halfedge* h = v->halfedge() ;
            int nt = 0;
            do{
                if(!h->is_border()) nt++ ;
                h = h->next_around_vertex() ;
            } while(h != v->halfedge()) ;
            Jv[v] = Jv[v]/nt ;
        }}

        {FOR_EACH_HALFEDGE(Map, map_, Hit) {
            if(!Hit->is_border()) {
                dp_index_[Hit] = cur_dp ;
                dp_(cur_dp) = 
                    - Jf[Hit->facet()]*(Geom::vector(Hit) * Geom::vector(Hit->next())) ;
                cur_dp++ ;
            }
        }}
        // sanity check
        // ogf_assert(cur_tri == map_->size_of_facets() && cur_dp == map_->size_of_facets() * 3) ;
    }

    // Reconstruction of the tex coords from the dot products
    void MapParameterizerDPBF::reconstruct() {
        // Generating (u,v) coordinates
        MapHalfedgeAttribute<double> dp(map_, "gamma") ;
        FOR_EACH_HALFEDGE(Map, map_, Hit) {
            if(!Hit->is_border()) {
                dp[Hit] = x_(dp_index_[Hit]) ;
            }
        }
        AnglesToUV* atouv = new AnglesToUV;
        atouv->set_mode(DOT_PRODUCTS) ;
        do_parameterize_disc_using_parameterizer(atouv, map_) ;
        delete atouv ;
    }


    bool is_nan(const Vector& v) {
        for(unsigned int i=0; i<v.size(); i++) {
            if(Numeric::is_nan(v(i))) {
                return true ;
            }
        }
        return false ;
    }

    /*-+-+-* Solver *-+-+-*/
		
    // Application of only one pass. Initialize first.
    bool MapParameterizerDPBF::pass() {

        for(int i = 0; i < pass_length_; i++) {
            // Computation Hessian (matrix J) and -gradient (vector b) at point x
            J_.clear() ;
            b_.zero() ;

            if(is_nan(b_)) {
                std::cerr << "b is nan" << std::endl ;
                return false ;
            }

            if(is_nan(x_)) {
                std::cerr << "x is nan" << std::endl ;
                return false ;
            }

            if(is_nan(dp_)) {
                std::cerr << "dp is nan" << std::endl ;
                return false ;
            }

            if(is_nan(errp_)) {
                std::cerr << "errp is nan" << std::endl ;
                return false ;
            }

            if(is_nan(area_)) {
                std::cerr << "area is nan" << std::endl ;
                return false ;
            }


            compute_Ec_derivatives() ;
            compute_C1_derivatives() ;
            compute_C2_derivatives() ;
            compute_C3_derivatives() ;

            // Solves J.delta = b
            // The first call initializes the permutation matrix.

            
            if(is_nan(b_)) {
                std::cerr << "b is nan" << std::endl ;
                return false ;
            }

            Logger::out("DPBF") << "solver: calling SuperLU" << std::endl ;
            if(!solve_super_lu(J_, b_, delta_, perm_)) {
                // reconstructs last valid iteration
                reconstruct() ;
                Logger::out("DPBF")<< "Failed" << std::endl ;
                // TODO: update surface 
                return false ;
            }
            Logger::out("DPBF") << "... solved" << std::endl ;

            // update x
            for(int i=0; i<ntot_; i++) x_(i) += step_length_ * delta_(i) ;

            // updates energy multiplier
            mu_ *= 1 + mu_multiplier_ ;
            mu_multiplier_ *= mu_moderator_ ;

            // prevents mu from being too big
            if(mu_ > mu_max_) mu_ = mu_max_ ;
        }
        Logger::out("DPBF")<< "Mu is now " << mu_ << std::endl ;
        reconstruct() ;
        return true ;
    }


    /*-+-+-*
      Derivatives of the Lagrangian
      Adds Hessian of the Lagrangian at point x_ to matrix J_
      Adds gradient of the Lagrangian at point x_ to vector b_
      *-+-+-*/

    // Ec : Shape deformation minimising energy
    // 1/2 * ((gamma(i)-dp(i))/St)^2
    // for each dot product
    void MapParameterizerDPBF::compute_Ec_derivatives() {
        FOR_EACH_FACET(Map, map_, Fit) {
            Map::Halfedge* h = Fit->halfedge() ;
            do{
                int i = dp_index_[h] ;
                J_.add(i, i, w_(triangle_index_[Fit])) ;
                b_(i) -= ( x_(i) - dp_(i) ) * w_(triangle_index_[Fit]) ;
                h = h->next() ;
            } while(h != Fit->halfedge()) ;
        }
    }

    // C1 : Conservation of shared edge length
    // (gamma_i,t + gamma_j,t - gamma_i',t' - gamma_j',t')/(St + St') = 0
    // for each interior edge e shared by triangles t and t'
    void MapParameterizerDPBF::compute_C1_derivatives() {
        int e = ngamma_, ind[4] ;
        FOR_EACH_EDGE(Map, map_, Eit) if(!Eit->is_border_edge()){
            // Indices
            ind[0] = dp_index_[Eit], ind[1] = dp_index_[Eit->prev()] ;
            ind[2] = dp_index_[Eit->opposite()], ind[3] = dp_index_[Eit->opposite()->prev()] ;

            // Error and derivative of error
            double err = 0 ;
            errp_(ind[0]) = errp_(ind[1]) =
                1/(area_(triangle_index_[Eit->facet()]) + area_(triangle_index_[Eit->opposite()->facet()]));
            errp_(ind[2]) = errp_(ind[3]) = -errp_(ind[0]) ;
            for(int i = 0; i < 4; i++) err += errp_(ind[i]) * x_(ind[i]) ;
            // Implementation of constraint 1 wEith Lagrange multipliers
            {for(int i = 0; i < 4; i++){
                //J_ coefficients
                J_.add(e, ind[i], errp_(ind[i])) ;
                J_.add(ind[i], e, errp_(ind[i])) ;
                //b_ coefficients
                b_(ind[i]) -= errp_(ind[i]) * x_(e) ;
            }}
            b_(e) -= err;
            e++;
        }
    }

    // C2 : planarity around interior vertices
    // Sum acotg(gamma_i,t/S_t) - 2PI = 0
    // for each interior vertex
    void MapParameterizerDPBF::compute_C2_derivatives() {
        FOR_EACH_VERTEX(Map, map_, Vit) if(!Vit->is_on_border()){
            double err = 0, S = 0 ;
            Map::Halfedge* h1 = Vit->halfedge() ;
            // Error and derivative of error
            // Computing real area from dp
            do{
                int i = dp_index_[h1], ip =dp_index_[h1->prev()], in = dp_index_[h1->next()] ; 
                S = sqrt(x_(i)*x_(in) + x_(in)*x_(ip) + x_(ip)*x_(i)) ;
                err += atan2(S, x_(i)) ;
                errp_(i) = -1.0 / (S + x_(i) * x_(i) / S) ;
                h1 = h1->next_around_vertex() ;
            } while(h1 != Vit->halfedge()) ;

            // Idem + dependancies in derivative
            err -= 2 * M_PI ;

            // Implementation of constraint 2 with a quadratic barrier
            do{
                int i = dp_index_[h1] ;
                Map::Halfedge* h2 = Vit->halfedge() ;
                do{
                    int j = dp_index_[h2] ;
                    //J_ coefficients
                    J_.add(i, j, mu_ * errp_(i) * errp_(j)) ;
                    h2 = h2->next_around_vertex() ;
                } while(h2 != Vit->halfedge()) ;
                //b_ coefficients
                b_(i) -= mu_ * err * errp_(i) ;
                h1 = h1->next_around_vertex() ;
            } while(h1 != Vit->halfedge()) ;
        }
    }

    // C3 : Authalism
    // (sum gamma_i * gamma_i+1)/St^2 - 1 = 0
    // for each triangle.
    void MapParameterizerDPBF::compute_C3_derivatives() {
        FOR_EACH_FACET(Map, map_, Fit) {
            Map::Halfedge* h1 = Fit->halfedge() ;
            double err = 0;
            // Error and derivative of error
            do{
                int i = dp_index_[h1], ip =dp_index_[h1->prev()], in = dp_index_[h1->next()] ;
                err += x_(in) * x_(ip) ;
                errp_(i) = (x_(in) + x_(ip)) * w_(triangle_index_[Fit]) ;
                h1 = h1->next() ;
            } while(h1 != Fit->halfedge()) ;
            err = err * w_(triangle_index_[Fit]) - 1.0 ;

            // Implementation of constraint 3 with a quadratic barrier
            do {
                int i = dp_index_[h1] ;
                Map::Halfedge* h2 = Fit->halfedge() ;
                do{
                    int j = dp_index_[h2] ;
                    //J_ coefficients
                    J_.add(i, j, mu_ * errp_(i) * errp_(j)) ;
                    h2 = h2->next() ;
                } while(h2 != Fit->halfedge()) ;
                //b_ coefficients
                b_(i) -= mu_ * err * errp_(i) ;
                h1 = h1->next() ;
            } while(h1 != Fit->halfedge()) ;
        }
    }
}
//________________________________________________________________________
