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
 

#ifndef __OGF_MATH_ATTRIBUTES_POINT_ATTRIBUTE_INTERPOLATOR__
#define __OGF_MATH_ATTRIBUTES_POINT_ATTRIBUTE_INTERPOLATOR__

#include <OGF/math/common/common.h>
#include <OGF/basic/attributes/attribute_interpolator.h>
#include <OGF/math/geometry/types.h>

namespace OGF {

    //___________________________________________________________________________

    class MATH_API Point2dAttributeInterpolator : public AttributeInterpolator {
    public:
        Point2dAttributeInterpolator() : target_(nil) { }
        virtual void interpolate_begin(Memory::pointer to) ;
        virtual void interpolate_add(double a, Memory::pointer x) ;
        virtual void interpolate_end() ;
    private:
        Point2d* target_ ;
    } ;

    //___________________________________________________________________________

    class MATH_API Point3dAttributeInterpolator : public AttributeInterpolator {
    public:
        Point3dAttributeInterpolator() : target_(nil) { }
        virtual void interpolate_begin(Memory::pointer to) ;
        virtual void interpolate_add(double a, Memory::pointer x) ;
        virtual void interpolate_end() ;
    private:
        Point3d* target_ ;
    } ;

    //___________________________________________________________________________

}

#endif
