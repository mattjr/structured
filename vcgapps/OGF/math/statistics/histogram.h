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

#ifndef __OGF_MATH_HISTOGRAM__
#define __OGF_MATH_HISTOGRAM__

#include <OGF/math/common/common.h>
#include <OGF/basic/containers/arrays.h>
#include <iostream>

namespace OGF {

    class Histogram {
    public:
        Histogram(
            double minv, double maxv, unsigned int nb_bins
        ) : minv_(minv), maxv_(maxv), stats_(nb_bins) {
            reset() ;
        }
        void reset() {
            stats_.set_all(0) ;
	    actual_minv_ =  1e30 ;
	    actual_maxv_ = -1e30 ;
        }
        void new_val(double x) {
            int index = int((x - minv_) * double(stats_.size()) / (maxv_ - minv_)) ;
            if(index >= 0 && index < stats_.size()) {
                stats_[index]++ ;
            }
	    actual_minv_ = ogf_min(actual_minv_, x) ;
	    actual_maxv_ = ogf_max(actual_maxv_, x) ;	   
        }
        
        void print(std::ostream& out) {
            std::cerr << "Histogram: min=" << min_value() << " max=" << max_value() << std::endl ;
  	    out << "# min:" << min_value() << " max:" << max_value() << std::endl ;
            for(unsigned int i=0; i<stats_.size(); i++) {
                double v = minv_ + double(i) * (maxv_ - minv_) / double(stats_.size()) ;
                out << v << " " << stats_[i] << std::endl ;
            }
        }
        double min_value() const { return actual_minv_ ; }
        double max_value() const { return actual_maxv_ ; }	  
    private:
        double minv_ ;
        double maxv_ ;
        double actual_minv_ ;
        double actual_maxv_ ; 
        Array1d<unsigned int> stats_ ;
    } ;

}

#endif

