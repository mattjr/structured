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
 

#ifndef __OGF_MATH_FUNCTIONS_M_ESTIMATOR__
#define __OGF_MATH_FUNCTIONS_M_ESTIMATOR__

#include <OGF/math/common/common.h>

namespace OGF {

    enum MEstimatorWeight { 
        // Plain least squares (no M-estimator)
        M_EST_L2,  
        
        // Unique minimum guaranteed (but do not completely eliminate outliers)
        M_EST_L1_L2, M_EST_LP, M_EST_FAIR, M_EST_HUBER,  

        // Completely eliminate outliers (but unique minimum is not guaranteed)
        M_EST_CAUCHY, M_EST_GEMAN_MCCLURE, M_EST_WELSCH, M_EST_TUCKEY
    } ;

    class MATH_API MEstimator {
    public:
        MEstimator() ;
        void set_weighting_function(MEstimatorWeight x) ;
        double get_width() const { return width_ ; }
        void set_width(double x) { width_ = x ; }
        double weight(double r) const { return weight_func_ptr_(r, width_) ; }
    protected:
    private:
        typedef double (*WeightFuncPtr)(double r, double width) ;
        WeightFuncPtr weight_func_ptr_ ;
        double width_ ;
    } ;

}
#endif

