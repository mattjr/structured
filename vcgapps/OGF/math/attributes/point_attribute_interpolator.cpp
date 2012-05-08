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

#include <OGF/math/attributes/point_attribute_interpolator.h>

namespace OGF {

    //___________________________________________________________________________

    void Point2dAttributeInterpolator::interpolate_begin(Memory::pointer to) {
        target_ = (Point2d*)to ;
        *target_ = Point2d() ;
    }
    
    void Point2dAttributeInterpolator:: interpolate_add(double a, Memory::pointer x) {
        Point2d* v = (Point2d*)x ;
        target_->set_x(target_->x() + a * v->x()) ;
        target_->set_y(target_->y() + a * v->y()) ;
    }
    
    void Point2dAttributeInterpolator::interpolate_end() {
        target_ = nil ;
    }
    
    void Point3dAttributeInterpolator::interpolate_begin(Memory::pointer to) {
        target_ = (Point3d*)to ;
        *target_ = Point3d() ;
    }
    
    void Point3dAttributeInterpolator::interpolate_add(double a, Memory::pointer x) {
        Point3d* v = (Point3d*)x ;
        target_->set_x(target_->x() + a * v->x()) ;
        target_->set_y(target_->y() + a * v->y()) ;
        target_->set_z(target_->z() + a * v->z()) ;
    }
    
    void Point3dAttributeInterpolator::interpolate_end() {
        target_ = nil ;
    }
    
    //___________________________________________________________________________

}
