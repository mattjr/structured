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

// #define OGF_PARANOID

#include <OGF/parameterizer/algos/abf.h>
#include <OGF/cells/map_algos/angles_to_uv.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/cells/map_algos/enumerate.h>
#include <OGF/math/numeric/super_lu.h>

namespace OGF {

//____________________________________________________________________________
    
    ABFMapParameterizer::ABFMapParameterizer() :
        epsilon_(1e-5),
        newton_tolf_(1e-6),
        newton_tolx_(1e-6),
        max_newton_iter_(200),
        positive_angle_ro_(1.2),
        step_length_factor_(1.0) {
        triangles_only_ = true ;
        log_mode_ = false ;

        max_newton_iter_ = 1 ;
    }
    
    void ABFMapParameterizer::allocate_variables() {
        nf_     = nb_triangles(map_) ;
        nalpha_ = 3*nf_ ;
        nint_   = nb_interior_vertices(map_) ;
        ntot_   = nalpha_ + nf_ + 2 * nint_ ;
        
        angle_index_.bind(map_) ;
        
        x_.allocate(ntot_) ;
        beta_.allocate(nalpha_) ;
        w_.allocate(nalpha_) ;
        J_.allocate(ntot_,ntot_, SparseMatrix::ROWS) ;
        b_.allocate(ntot_) ;
        delta_.allocate(ntot_) ;
    }

    void ABFMapParameterizer::deallocate_variables() {
        angle_index_.unbind() ;
        x_.clear() ;
        beta_.clear() ;
        w_.clear() ;
        J_.deallocate() ;
        b_.clear() ;
        delta_.clear() ;
    }

    bool ABFMapParameterizer::do_parameterize_disc(Map* map) {
        map_ = map ;
        allocate_variables() ;

        enumerate_angles() ;
        compute_beta() ;
        solve_angles() ;

        MapHalfedgeAttribute<double> angle(map_, "angle") ;
        FOR_EACH_HALFEDGE(Map, map_, it) {
            angle[it] = x_(angle_index_[it]) ;
        }

        MapParameterizer_var a_to_uv = new AnglesToUV ;
        do_parameterize_disc_using_parameterizer(a_to_uv, map_) ;
        
        deallocate_variables() ;
        return true ;
    }

    int ABFMapParameterizer::nb_interior_vertices(Map* map) {
        int result = 0 ;
        FOR_EACH_VERTEX(Map, map, it) {
            if(!it->is_on_border()) {
                result++ ;
            }
        }
        return result ;
    }

    int ABFMapParameterizer::nb_triangles(Map* map) {
        return map->size_of_facets() ;
    }
    

    void ABFMapParameterizer::enumerate_angles() {
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
        ogf_assert(cur == map_->size_of_facets() * 3) ;
    }

    // Had a hard time finding what was wrong with my code:
    // OK, got it, use h->next() instead of h->next_around_vertex()
    // because h->next_around_vertex() does not return the same
    // angle.
    static inline double corner_angle(OGF::Map::Halfedge* h) {
        return M_PI - Geom::angle(
            Geom::vector(h), 
            Geom::vector(h->next())
        ) ;
    }


    void ABFMapParameterizer::compute_beta() {
    
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


    void ABFMapParameterizer::solve_angles() {

        // TODO: check if memory footprint could
        //  be reduced by freeing some sparse
        //  matrices during iteration (before
        //  calling SuperLU).

        // Initial values
        { for(int i=0; i<nalpha_; i++) {
            x_(i) = beta_(i) ;
            w_(i) = 1.0 / (beta_(i) * beta_(i)) ;
        }}

        Array1d<int> perm ; // column permutations (used by SuperLU)

        for(int k=0; k<max_newton_iter_; k++) {

            // Compute Jacobian
            J_.clear() ;
            add_JC1() ;
            add_JC2() ;
            add_JC3() ;
            for(int i=0; i<nalpha_; i++) {
                J_.add(i,i,2.0 * w_(i)) ;
            }

            if(log_mode_) {
                int offset = nalpha_ + nf_ + nint_ ;
                FOR_EACH_VERTEX(Map, map_, it) {
                    if(it->is_on_border()) {
                        continue ;
                    }
                    Map::Halfedge* h = it->halfedge() ;
                    do {
                        int i = angle_index_[h->next()] ;
                        J_.add(i,i, -x_(offset) / ogf_sqr(sin(x_(i)))) ;
                        i = angle_index_[h->prev()] ;
                        J_.add(i,i,  x_(offset) / ogf_sqr(sin(x_(i)))) ;
                        h = h->next_around_vertex() ;
                    } while(h != it->halfedge()) ;
                    offset++ ;
                }
            }

            // Compute rhs
            b_.zero() ;
            sub_grad_F() ;
            sub_grad_C1() ;
            sub_grad_C2() ;
            sub_grad_C3() ;

            double errf_k = errf() ;
            double errC_k = errC() ;
            Logger::out("ABF") << "iter= " << k << " errf= " << errf_k 
                               << std::endl ;
            Logger::out("ABF") << "iter= " << k << " errC= " << errC_k 
                               << std::endl ;
            if(errf_k <= newton_tolf_) {
                Logger::out("ABF") << "converged" << std::endl ;
                return ;
            }

            //   Solve J.delta = b
            // The first call initializes the permutation matrix.

            solve_super_lu(J_, b_, delta_, perm) ;

            double s = compute_step_length_and_update_weights() ;
            double errx = compute_errx_and_update_x(s) ;

            Logger::out("ABF") << "iter= " << k << " errx= " << errx
                               << std::endl ;

            if(errx <= newton_tolx_) {
                return ;
            }
        }

    }

    // ----------------------------- Jacobian -----------------------------

    void ABFMapParameterizer::add_JC1() {
        int i_offset = nalpha_ ;
        for(int i=0; i<nf_; i++) {
            for(int j=0; j<3; j++) {
                J_.add(i+i_offset, 3*i+j,      1.0) ;
                J_.add(3*i+j,      i+i_offset, 1.0) ;
            }
        }
    }

    void ABFMapParameterizer::add_JC2() {
        int i = nalpha_ + nf_ ;
        FOR_EACH_VERTEX(Map, map_, it) {
            if(it->is_on_border()) {
                continue ;
            }
            Map::Halfedge* h = it->halfedge() ;
            do {
                J_.add(i,angle_index_[h],  1.0) ;
                J_.add(angle_index_[h], i, 1.0) ;
                h = h->next_around_vertex() ;
            } while(h != it->halfedge()) ;
            i++ ;
        }
    }
    

    void ABFMapParameterizer::compute_sum_log_sin_angles(
        Map::Vertex* v, double& sum_prev_sin, double& sum_next_sin
    ) {
        sum_prev_sin = 0.0 ;
        sum_next_sin = 0.0 ;
        OGF::Map::Halfedge* h = v->halfedge() ;
        do {
            sum_prev_sin += log(sin(x_(angle_index_[h->prev()]))) ;
            sum_next_sin += log(sin(x_(angle_index_[h->next()]))) ;
            h = h->next_around_vertex() ;
        } while(h != v->halfedge()) ;
    }


    void ABFMapParameterizer::compute_product_sin_angles(
        Map::Vertex* v, double& prod_prev_sin, double& prod_next_sin
    ) {
        prod_prev_sin = 1.0 ;
        prod_next_sin = 1.0 ;
        OGF::Map::Halfedge* h = v->halfedge() ;
        do {
            prod_prev_sin *= sin(x_(angle_index_[h->prev()])) ;
            prod_next_sin *= sin(x_(angle_index_[h->next()])) ;
            h = h->next_around_vertex() ;
        } while(h != v->halfedge()) ;
    }

    void ABFMapParameterizer::add_JC3() {
        if(log_mode_) {
            int i = nalpha_ + nf_ + nint_ ;
            FOR_EACH_VERTEX(Map, map_, it) {
                if(it->is_on_border()) {
                    continue ;
                }
                Map::Halfedge* h = it->halfedge() ;
                do {
                    int j = angle_index_[h->next()] ;
                    J_.add(i,j, cos(x_(j)) / sin(x_(j))) ;
                    j = angle_index_[h->prev()] ;
                    J_.add(i,j,-cos(x_(j)) / sin(x_(j))) ;
                    h = h->next_around_vertex() ;
                } while(h != it->halfedge()) ;
                i++ ;
            }
        } else {
            int i = nalpha_ + nf_ + nint_ ;
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
                    J_.add(i, j, prod_next_sin * cos(x_(j)) / sin(x_(j))) ;
                    J_.add(j, i, prod_next_sin * cos(x_(j)) / sin(x_(j))) ;
                    j = angle_index_[h->prev()] ;
                    J_.add(i, j,-prod_prev_sin * cos(x_(j)) / sin(x_(j))) ;
                    J_.add(j, i,-prod_prev_sin * cos(x_(j)) / sin(x_(j))) ;
                    h = h->next_around_vertex() ;
                } while(h != it->halfedge()) ;
                i++ ;
            }
        }
    }


    // ----------------------- Right hand side -----------------------

    void ABFMapParameterizer::sub_grad_F() {
        for(int i = 0; i < nalpha_; i++ ) {
            b_(i) -= 2.0 * w_(i) * ( x_(i) - beta_(i) );
        }
    }
    
    // For each triangle: sum angles - PI
    void ABFMapParameterizer::sub_grad_C1() {
        int i_offset = nalpha_ ;
        { for(int i=0; i < nf_; i++) {
            for(int j=0; j<3; j++) {
                b_(3*i+j) -= x_(i_offset + i) ;
            }
        }}
        { for(int i=0; i < nf_; i++) {
            b_(i+i_offset) -= 
                x_(3*i) + x_(3*i+1) + x_(3*i+2) - M_PI ;
        }}
    }
    
    // For each vertex: sum incident angles - 2.PI
    void ABFMapParameterizer::sub_grad_C2() {
        int i = nalpha_ + nf_ ;
        FOR_EACH_VERTEX(Map, map_, it) {
            if(it->is_on_border()) {
                continue ;
            }
            Map::Halfedge* h = it->halfedge() ;
            do {
                b_(i) -= x_(angle_index_[h]) ;
                b_(angle_index_[h]) -= x_(i) ;
                h = h->next_around_vertex() ;
            } while(h != it->halfedge()) ;
            b_(i) += 2.0 * M_PI ;
            i++ ;
        }
    }
    
    // For each vertex: prod sin(next angle) - prod sin(prev angle) = 0
    void ABFMapParameterizer::sub_grad_C3() {
        if(log_mode_) {
            int i = nalpha_ + nf_ + nint_ ;
            FOR_EACH_VERTEX(Map, map_, it) {
                if(it->is_on_border()) {
                    continue ;
                }
                double sum_prev_sin ;
                double sum_next_sin ;
                compute_sum_log_sin_angles(it, sum_prev_sin, sum_next_sin) ;
                b_(i) -= (sum_next_sin - sum_prev_sin) ;
                Map::Halfedge* h = it->halfedge() ;
                do {
                    int j = angle_index_[h->next()] ;
                    b_(j) -= x_(i) * cos(x_(j)) / sin(x_(j)) ;
                    j = angle_index_[h->prev()] ;
                    b_(j) += x_(i) * cos(x_(j)) / sin(x_(j)) ;
                    h = h->next_around_vertex() ;
                } while(h != it->halfedge()) ;
                i++ ;
            }
        } else {
            int i = nalpha_ + nf_ + nint_ ;
            FOR_EACH_VERTEX(Map, map_, it) {
                if(it->is_on_border()) {
                    continue ;
                }
                double prod_prev_sin ;
                double prod_next_sin ;
                compute_product_sin_angles(it, prod_prev_sin, prod_next_sin) ;
                
                b_(i) -= prod_next_sin - prod_prev_sin ;
                
                Map::Halfedge* h = it->halfedge() ;
                do {
                    int j = angle_index_[h->next()] ;
                    b_(j) -= 
                        x_(i) * prod_next_sin * cos(x_(j)) / sin(x_(j)) ;
                    
                    j = angle_index_[h->prev()] ;
                    b_(j) += 
                        x_(i) * prod_prev_sin * cos(x_(j)) / sin(x_(j)) ;
                    
                    h = h->next_around_vertex() ;
                } while(h != it->halfedge()) ;
                i++ ;
            }
        }
    }

    double ABFMapParameterizer::compute_errx_and_update_x(double s) {
        double result = 0 ;

        // alpha +=  s * dalpha ; lambda += s * dlambda
        for(int i=0; i<ntot_; i++) {
            double dai = s * delta_(i) ;
            x_(i) += dai ;
            result += ::fabs(dai) ;
        }
        return result ;
    }
    
    // ______ Convergence control ____________________________________

    double ABFMapParameterizer::compute_step_length_and_update_weights() {
        double ratio = 1.0 ;
        for(int i=0; i<nalpha_; i++) {
            if(x_(i) + delta_(i) < 10.0 * epsilon_) {
                double r1 = -.5 * (x_(i) - 10.0 * epsilon_)/ delta_(i) ;
                ratio = ogf_min(ratio, r1) ;
                w_(i) *= positive_angle_ro_ ;
/*
                std::cerr << " w  ( at 0 ) " 
                          << i << " is " << w_(i) << std::endl;
                std::cerr << " val " 
                          << alpha_(i) + dalpha_(i) << " was " 
                          << alpha_(i) << std::endl;
*/
                
            } else if(x_(i) + delta_(i) > M_PI - 10.0 * epsilon_) {
                double r1 = .5*(M_PI - x_(i)+10.0 * epsilon_) / delta_(i);
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

    double ABFMapParameterizer::errf() const {
        double result = 0 ;
        {for(int i=0; i<ntot_; i++) {
            result += ::fabs(b_(i)) ;
        }}
        return result ;
    }

    double ABFMapParameterizer::errC() const {
        double result = 0 ;
        {for(int i=3*nf_; i<ntot_; i++) {
            result += ::fabs(b_(i)) ;
        }}
        return result ;
    }

    //___________________________________________________________________

}
