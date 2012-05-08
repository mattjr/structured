
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
 
#ifndef __OGF_MATH_FUNCTIONS_PN__
#define __OGF_MATH_FUNCTIONS_PN__

#include <OGF/math/functions/function.h>

namespace OGF {

    class P0BivariateFunction : public Function<1> {
        typedef P0BivariateFunction thisclass ;
        typedef Function<1> baseclass ;
    public:
        P0BivariateFunction() { }
        P0BivariateFunction(const thisclass& rhs) : baseclass(rhs) { }
        thisclass& operator=(const thisclass& rhs) { 
            baseclass::operator=(rhs) ; 
            return *this ;
        }
    } ;


    class P1BivariateFunction : public Function<3> {
        typedef P1BivariateFunction thisclass ;
        typedef Function<3> baseclass ;
    public:
        P1BivariateFunction() { }
        P1BivariateFunction(const thisclass& rhs) : baseclass(rhs) { }
        thisclass& operator=(const thisclass& rhs) { 
            baseclass::operator=(rhs) ; 
            return *this ;
        }
    } ;

    class P2BivariateFunction : public Function<6> {
        typedef P2BivariateFunction thisclass ;
        typedef Function<6> baseclass ;
    public:
        P2BivariateFunction() { }
        P2BivariateFunction(const thisclass& rhs) : baseclass(rhs) { }
        thisclass& operator=(const thisclass& rhs) { 
            baseclass::operator=(rhs) ; 
            return *this ;
        }
    } ;

    class P3BivariateFunction : public Function<10> {
        typedef P3BivariateFunction thisclass ;
        typedef Function<10> baseclass ;
    public:
        P3BivariateFunction() { }
        P3BivariateFunction(const thisclass& rhs) : baseclass(rhs) { }
        thisclass& operator=(const thisclass& rhs) { 
            baseclass::operator=(rhs) ; 
            return *this ;
        }
    } ;
    
}

#endif
