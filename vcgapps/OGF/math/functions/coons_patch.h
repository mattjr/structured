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
 

#ifndef __OGF_MATH_COONS_PATCH__
#define __OGF_MATH_COONS_PATCH__

#include <OGF/math/common/common.h>
#include <OGF/math/geometry/types.h>

namespace OGF {
    
    template <class Fu0, class Fu1, class F0v, class F1v> class CoonsPatch {
    public:
        typedef CoonsPatch<Fu0, Fu1, F0v, F1v> thisclass ;
        CoonsPatch() { }
        CoonsPatch(
            const Fu0& fu0, const Fu1& fu1, const F0v& f0v, const F1v& f1v
        ) { 
            initialize(fu0, fu1, f0v, f1v) ;
        }
        CoonsPatch(const thisclass& rhs) {
            initialize(rhs.fu0_, rhs.fu1_, rhs.f0v_, rhs.f1v_) ;
        }
        thisclass& operator=(const thisclass& rhs) {
            initialize(rhs.fu0_, rhs.fu1_, rhs.f0v_, rhs.f1v_) ;
            return *this ;
        }
        void initialize(const Fu0& fu0, const Fu1& fu1, const F0v& f0v, const F1v& f1v) {
            fu0_ = fu0 ;
            fu1_ = fu1 ;
            f0v_ = f0v ;
            f1v_ = f1v ;
            p00_ = fu0_.eval(0.0) ;
            p10_ = fu0_.eval(1.0) ;
            p01_ = fu1_.eval(0.0) ;
            p11_ = fu1_.eval(1.0) ;
        }
        Point3d eval(const Point2d& uv) {
            double u = uv.x() ;
            double v = uv.y() ;
            double u0 = 1.0 - u ;
            double u1 = u ;
            double v0 = 1.0 - v ;
            double v1 = v ;
            Point3d p0v = f0v_.eval(v) ;
            Point3d p1v = f1v_.eval(v) ;
            Point3d pu0 = fu0_.eval(u) ;
            Point3d pu1 = fu1_.eval(u) ;
            Point3d result(0.0, 0.0, 0.0) ;
            for(unsigned int i=0; i<3; i++) {
                result[i] += u0 * p0v[i] + u1 * p1v[i] ;
                result[i] += v0 * pu0[i] + v1 * pu1[i] ;
                result[i] -= u0 * v0 * p00_[i] ;
                result[i] -= u0 * v1 * p01_[i] ;
                result[i] -= u1 * v0 * p10_[i] ;
                result[i] -= u1 * v1 * p11_[i] ;            
            }
            return result ;
        }
    protected:
        Fu0 fu0_ ;
        Fu1 fu1_ ;
        F0v f0v_ ;
        F1v f1v_ ;
        Point3d p00_ ;
        Point3d p10_ ;
        Point3d p01_ ;
        Point3d p11_ ;
    } ;
        
}

#endif

