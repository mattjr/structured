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

#ifndef __OGF_MATH_MEDIAN__
#define __OGF_MATH_MEDIAN__

#include <OGF/math/common/common.h>
#include <OGF/basic/types/types.h>
#include <vector>
#include <algorithm>

namespace OGF {

    class ValueWeight {
    public:
        ValueWeight(double v, double w) : value(v), weight(w) {   }
        bool operator<(const ValueWeight& rhs) const { return (rhs.value < value) ; }
        double value ;
        double weight ;
    } ;

    inline void add(std::vector<ValueWeight>& vals, double w, double v) {
        w = ogf_max(w , 1e-30) ;
        vals.push_back(ValueWeight(v,w)) ;
    }
    

    inline double median(std::vector<double>& vals) {
        int n = vals.size() ;
        if(n == 0) { return 0.0 ; }
        std::sort(vals.begin(), vals.end()) ;
        if((n & 1) != 0) {  return vals[n/2] ; } 
        return 0.5 * (vals[n/2-1] + vals[n/2]) ;
    }

    inline double total_weight(std::vector<ValueWeight>& vals) {
        double result = 0.0 ;
        for(unsigned int i=0; i<vals.size(); i++) {
            result += vals[i].weight ;
        }
        return result ;
    }

    inline double median(std::vector<ValueWeight>& vals) {
        if(vals.size() == 0) { return 0.0 ; }
        if(vals.size() == 1) { return vals[0].value ; }

        std::sort(vals.begin(), vals.end()) ;

        double w = total_weight(vals) / 2.0 ;
        double cumul = 0.0 ;
        int index = -1 ;
        do {
            index++ ;
            cumul += vals[index].weight ;
        } while(cumul < w) ;
        
        int prev = index - 1 ;
        if(prev < 0) {
            return vals[index].value ;
        }
        
        double prev_cumul = cumul - vals[index].weight ;
        double s = (w - prev_cumul) / vals[index].weight ;

        return s * vals[index].value + (1.0 - s) * vals[prev].value ;
    }

    inline double mean(std::vector<ValueWeight>& vals) {
        double w = 0.0 ;
        double S = 0.0 ;
        for(unsigned int i=0; i<vals.size(); i++) {
            w += vals[i].weight ;
            S += vals[i].value ;
        }
        return S/w ;
    }

}

#endif
