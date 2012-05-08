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

#ifndef __OGF_MATH_LINEAR_ALGEBRA_VECTOR__
#define __OGF_MATH_LINEAR_ALGEBRA_VECTOR__

#include <OGF/math/common/common.h>
#include <OGF/basic/containers/arrays.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/debug/assert.h>
#include <iomanip>
#include <iostream>

namespace OGF {

    class Vector : public Array1d<double> {
    public:
        typedef Array1d<double> superclass ;
    public:
        void zero() { Memory::clear(superclass::data(), superclass::size() * sizeof(double)); } 
        Vector() { }
        Vector(unsigned int n, unsigned int alignment = 1) : superclass(n, alignment) { zero(); }
        Vector(const Vector& rhs) {
            allocate(rhs.size(), rhs.alignment()) ;
            Memory::copy(superclass::data(), rhs.data(), superclass::size() * sizeof(double)) ;
        }
        void allocate(unsigned int n, unsigned int alignment=1) { superclass::allocate(n, alignment); zero() ; } 
        void operator += (const Vector& rhs) {
            ogf_debug_assert(rhs.size() == superclass::size()) ;
            for(unsigned int i=0; i<superclass::size(); i++) {
                (*this)(i) += rhs(i) ;
            }
        }
        void operator -= (const Vector& rhs) {
            ogf_debug_assert(rhs.size() == superclass::size()) ;
            for(unsigned int i=0; i<superclass::size(); i++) {
                (*this)(i) -= rhs(i) ;
            }
        }
        Vector& operator=(const Vector& rhs) {
            ogf_debug_assert(rhs.size() == superclass::size()) ;
            Memory::copy(superclass::data(), rhs.data(), superclass::size() * sizeof(double)) ;
            return *this ;
        }
            
        // used by operator << and may be used to print C-Style vectors
        static void print(std::ostream& out, unsigned int n, const double* x) {
            out << "[ " ;
            for(unsigned int i=0; i<n; i++) {
                out << std::setw(8) << x[i] << " " ;
            }
            out << "]" ;
        }
    } ;

    namespace Numeric {
        bool has_nan(const Vector& v) ;
    } 

} 

inline std::ostream& operator<<(std::ostream& out, const OGF::Vector& v) {
    v.print(out, v.size(), v.data()) ;
    return out ;
}

#endif
