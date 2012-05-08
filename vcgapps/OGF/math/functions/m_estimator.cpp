/*
 *  OGF/Graphite: Geometry and Graphics Programming Library + Utilities
 *  Copyright (C) 2000-2005 INRIA - Project ALICE
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
 *  Contact: Bruno Levy - levy@loria.fr
 *
 *     Project ALICE
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs. 
 *
 * As an exception to the GPL, Graphite can be linked with the following (non-GPL) libraries:
 *     Qt, SuperLU, WildMagic and CGAL
 */
 

#include <OGF/math/functions/m_estimator.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/debug/assert.h>

namespace OGF {

    static double weight_One(double r, double w) {
        return 1.0 ;
    }

    static double weight_L1_L2(double r, double w) {
        return 1.0 / sqrt(1.0 + ogf_sqr(r) / 2.0) ;
    }

    static double weight_Lp(double r, double w) {
        const double nu = 1.2 ;
        double abs_r = ogf_max(1e-6, ::fabs(r)) ;
        return ::pow(abs_r, nu - 2.0) ;
    }
    
    static double weight_Fair(double r, double w) {
        return 1.0 / (1.0 + r / w) ;
    }

    static double weight_Huber(double r, double w) {
        double abs_r = ::fabs(r) ;
        return (abs_r < w) ? 1.0 : (w / abs_r) ;
    }

    static double weight_Cauchy(double r, double w) {
        return 1.0 / (1.0 + ogf_sqr(r / w)) ;
    }

    static double weight_Geman_McClure(double r, double w) {
        return 1.0 / ogf_sqr(1.0 + ogf_sqr(r)) ;
    }

    static double weight_Welsch(double r, double w) {
        return ::exp(-ogf_sqr(r/w)) ;
    }

    static double weight_Tuckey(double r, double w) {
        return (::fabs(r) > w) ? 0.0 : ogf_sqr(1.0 - ogf_sqr(r/w)) ;
    }

    MEstimator::MEstimator() :  weight_func_ptr_(&weight_Lp), width_(0.5) { 
    }

    void MEstimator::set_weighting_function(MEstimatorWeight x) {
        switch(x) {
        case M_EST_L2: {
            weight_func_ptr_ = &weight_One ;
        } break ;
        case M_EST_L1_L2: {
            weight_func_ptr_ = &weight_L1_L2 ;
        } break ;
        case M_EST_LP: {
            weight_func_ptr_ = &weight_Lp ;
        } break ;
        case M_EST_FAIR: {
            weight_func_ptr_ = &weight_Fair ;
        } break ;
        case M_EST_HUBER: {
            weight_func_ptr_ = &weight_Huber ;
        } break ;
        case M_EST_CAUCHY: {
            weight_func_ptr_ = &weight_Cauchy ;
        } break ;
        case M_EST_GEMAN_MCCLURE: {
            weight_func_ptr_ = &weight_Geman_McClure ;
        } break ;
        case M_EST_WELSCH: {
            weight_func_ptr_ = &weight_Welsch ;
        } break ;
        case M_EST_TUCKEY: {
            weight_func_ptr_ = &weight_Tuckey ;
        } break ;
        default: {
            ogf_assert_not_reached ;
        } break ;
        }
    }
        
}

