
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
 
#ifndef __OGF_MATH_FUNCTIONS_FUNCTION__
#define __OGF_MATH_FUNCTIONS_FUNCTION__

#include <OGF/math/common/common.h>
#include <OGF/basic/types/types.h>

namespace OGF {

//__________________________________________________________________________________
    
    template <int DIM> class Function {
    public:
        enum { dimension = DIM } ;
        typedef Function<DIM> thisclass ;
        
        Function() { }

        Function(const thisclass& rhs) { 
            for(unsigned int i=0; i<dimension; i++) {
                coeff[i] = rhs.coeff[i] ;
            }
        }

        thisclass& operator=(const thisclass& rhs) { 
            for(unsigned int i=0; i<dimension; i++) {
                coeff[i] = rhs.coeff[i] ;
            }
            return *this ;
        }

        double& operator[](int i) { return coeff[i] ; }
        const double& operator[](int i) const { return coeff[i] ; }
        void clear() { Memory::clear(coeff, dimension * sizeof(double)) ; }

        double coeff[dimension] ;
    } ;

//__________________________________________________________________________________

    template <int DIM> std::ostream& operator<< (std::ostream& os, const Function<DIM>& f) {
        for(unsigned int i=0; i<DIM; i++) {
            os << f.coeff[i] << " " ;
        }
        return os ;
    }

    template <int DIM> std::istream& operator>> (std::istream& is, Function<DIM>& f) {
        for(unsigned int i=0; i<DIM; i++) {
            is >> f.coeff[i]  ;
        }
        return is ;
    }

//__________________________________________________________________________________

}

#endif
