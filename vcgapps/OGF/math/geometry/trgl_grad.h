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
 
 

#ifndef __MATH_GEOMETRY_TRGL_GRAD__
#define __MATH_GEOMETRY_TRGL_GRAD__

#include <OGF/math/common/common.h>
#include <OGF/math/geometry/types.h>

namespace OGF {
    
//_________________________________________________________
    
    /**
     * computes gradients in a triangle in 3D.
     */
    class MATH_API TrglGradient {
    public:
        TrglGradient(
            const Point3d& p0, const Point3d& p1, const Point3d& p2
        ) ;
        
        /** Creates an uninitialized TrglGradient */
        TrglGradient() ;
        
        void initialize(
            const Point3d& p0, const Point3d& p1, const Point3d& p2
        ) ;

        /**
         * Returns the ith vertex of the triangle.
         * @param i is the index of the vertex, 
         *        which can be one of 0,1,2.
         */
        const Point3d& vertex(int i) const ;
        
        /**
         * Returns the orthonormal basis in which gradient computation are
         * performed.
         */
        void basis(
            Point3d& origin, Vector3d& X, Vector3d& Y, Vector3d& Z
        ) const ;
        
        /**
         * Returns the coefficients determining the gradient in this triangle.
         *
         * grad_X = sum_i { TX(i) * vertex(i)-> embedding(prop) }
         * Note: TX(2) == 0
         */
        double TX(int i) const ;
        
        /**
         * Returns the coefficients determining the gradient in this triangle.
         *
         * grad_Y = sum_i { TY(i) * vertex(i)-> embedding(prop) }
         */
        double TY(int i) const ;
        bool is_flat() const { return is_flat_ ; }

        Vector3d gradient_3d(double value0, double value1, double value2) const ;

    private:
        double TX_[3] ;
        double TY_[3] ;
        Point3d vertex_[3] ;
        bool is_flat_ ;
    } ;
    
//___________________________________________________

    /**
     * computes gradients in a triangle in 2D.
     */
    class MATH_API ParamTrglGradient {
    public:
        ParamTrglGradient() ;
        ParamTrglGradient(
            const Point2d& p1, const Point2d& p2, const Point2d& p3
        ) ;
        const Point2d& vertex(int i) const ;
        double TX(int i) const ;
        double TY(int i) const ;
        bool is_flat() const { return is_flat_ ; }

    private:
        double TX_[3] ;
        double TY_[3] ;
        Point2d vertex_[3] ;
        bool is_flat_ ;
    } ;

//___________________________________________________    
    
    inline const Point3d& TrglGradient::vertex(int i) const { 
        return vertex_[i] ; 
    }

    inline double TrglGradient::TX(int i) const { return TX_[i] ; }
    inline double TrglGradient::TY(int i) const { return TY_[i] ; }

//_________________________________________________________

    inline const Point2d& ParamTrglGradient::vertex(int i) const { 
        return vertex_[i] ; 
    }

    inline double ParamTrglGradient::TX(int i) const { return TX_[i] ; }
    inline double ParamTrglGradient::TY(int i) const { return TY_[i] ; }
    
//_________________________________________________________
}

#endif

