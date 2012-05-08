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
 

#ifndef __OGF_MATH_GEOMETRY_TYPES__
#define __OGF_MATH_GEOMETRY_TYPES__

#include <OGF/math/common/common.h>
#include <OGF/math/geometry/point_vector_2d.h>
#include <OGF/math/geometry/point_vector_3d.h>
#include <OGF/math/geometry/box.h>
#include <OGF/math/linear_algebra/matrix.h>
#include <OGF/basic/types/types.h>

namespace OGF {


//_________________________________________________________

/**
 * Gathers different base types for geometric operations.
 * Types are defined here for points, vectors, matrices
 * in 2D and 3D. The coordinates of a point may be specified in
 * cartesian or homogeneous space (the classes for objects having
 * homogeneous coordinates are prefixed with an 'H'). Most users
 * may use instead the default types Point2d, Vector2d, 
 * Matrix2d, Point3d, Vector3d, Matrix3d.
 */


    class MATH_API GeometricTypes {

    public:
        
        //---------------------- Points in 2d

        typedef GenericPoint2d<Numeric::int16>   Point2d_int16 ;
        typedef GenericPoint2d<Numeric::int32>   Point2d_int32 ;
        typedef GenericPoint2d<Numeric::float32> Point2d_float32 ;
        typedef GenericPoint2d<Numeric::float64> Point2d_float64 ;

        //_______________________ Vectors in 2d

        typedef GenericVector2d<Numeric::int8>    Vector2d_int8 ;
        typedef GenericVector2d<Numeric::int16>   Vector2d_int16 ;
        typedef GenericVector2d<Numeric::int32>   Vector2d_int32 ;
        typedef GenericVector2d<Numeric::float32> Vector2d_float32 ;
        typedef GenericVector2d<Numeric::float64> Vector2d_float64 ;

        //_______________________ Points in 3d

        typedef GenericPoint3d<Numeric::int16>   Point3d_int16 ;
        typedef GenericPoint3d<Numeric::int32>   Point3d_int32 ;
        typedef GenericPoint3d<Numeric::float32> Point3d_float32 ;
        typedef GenericPoint3d<Numeric::float64> Point3d_float64 ;
    
        //_______________________ Vectors in 3d

        typedef GenericVector3d<Numeric::int8>    Vector3d_int8 ;
        typedef GenericVector3d<Numeric::int16>   Vector3d_int16 ;
        typedef GenericVector3d<Numeric::int32>   Vector3d_int32 ;
        typedef GenericVector3d<Numeric::float32> Vector3d_float32 ;
        typedef GenericVector3d<Numeric::float64> Vector3d_float64 ;

        //_______________________ Matrices

        typedef Matrix<Numeric::float32,3> HMatrix2d_float32 ;
        typedef Matrix<Numeric::float64,3> HMatrix2d_float64 ;

        typedef Matrix<Numeric::float32,4> HMatrix3d_float32 ;
        typedef Matrix<Numeric::float64,4> HMatrix3d_float64 ;

    } ;

// Default geometric types, globally declared.

    typedef GeometricTypes::Point2d_float64   Point2d ;
    typedef GeometricTypes::Vector2d_float64  Vector2d ;
    typedef GeometricTypes::HMatrix2d_float64 Matrix2d ;
    
    typedef GeometricTypes::Point3d_float64   Point3d ;
    typedef GeometricTypes::Vector3d_float64  Vector3d ;
    typedef GeometricTypes::HMatrix3d_float64 Matrix3d ;

    typedef GenericBox2d<double> Box2d ;
    typedef GenericBox3d<double> Box3d ;

//_________________________________________________________


// This namespace gathers some global geometric utilities.
    namespace Geom {

        inline Point3d barycenter(const Point3d& p1, const Point3d& p2) {
            return Point3d(
                0.5 * (p1.x() + p2.x()),
                0.5 * (p1.y() + p2.y()),
                0.5 * (p1.z() + p2.z())
            ) ;
        }

        inline Point2d barycenter(const Point2d& p1, const Point2d& p2) {
            return Point2d(
                0.5 * (p1.x() + p2.x()),
                0.5 * (p1.y() + p2.y())
            ) ;
        }

        inline Point3d barycenter(
            const Point3d& p1, const Point3d& p2, const Point3d& p3
        ) {
            return Point3d(
                (p1.x() + p2.x() + p3.x()) / 3.0 ,
                (p1.y() + p2.y() + p3.y()) / 3.0 ,
                (p1.z() + p2.z() + p3.z()) / 3.0
            ) ;
        }

        inline Point2d barycenter(
            const Point2d& p1, const Point2d& p2, const Point2d& p3
        ) {
            return Point2d(
                (p1.x() + p2.x() + p3.x()) / 3.0 ,
                (p1.y() + p2.y() + p3.y()) / 3.0 
            ) ;
        }


        inline double cos_angle(const Vector3d& a, const Vector3d& b) {
            double na2 = a.norm2() ;
            double nb2 = b.norm2() ;
            return (a * b) / ::sqrt(na2 * nb2) ;
        }

        inline double angle(const Vector3d& a, const Vector3d& b) {
            return ::acos(cos_angle(a,b)) ;
        }


        inline double cos_angle(const Vector2d& a, const Vector2d& b) {
            double na2 = a.norm2() ;
            double nb2 = b.norm2() ;
            return (a * b) / ::sqrt(na2 * nb2) ;
        }

        inline double det(const Vector2d& a, const Vector2d& b) {
            return a.x()*b.y() - a.y()*b.x() ;            
        }

        
        /* returns the angle in the interval [-pi .. pi] */
        inline double angle(const Vector2d& a, const Vector2d& b) {
            return det(a,b) > 0         ? 
                 ::acos(cos_angle(a,b)) : 
                -::acos(cos_angle(a,b)) ;
        }

        inline double triangle_area(
            const Point3d& p1, const Point3d& p2, const Point3d& p3
        ) {
            return 0.5 * ((p2 - p1) ^ (p3 - p1)).norm() ;
        }

        inline double triangle_signed_area(
            const Point2d& p1, const Point2d& p2, const Point2d& p3
        ) {
            return 0.5 * det(p2-p1,p3-p1) ;
        }

        inline double triangle_area(
            const Point2d& p1, const Point2d& p2, const Point2d& p3
        ) {
            return ::fabs(triangle_signed_area(p1,p2,p3)) ;
        }

        double MATH_API triangle_beauty(
            const Point2d& p1, const Point2d& p2, const Point2d& p3
        ) ;

        double MATH_API triangle_beauty(
            const Point3d& p1, const Point3d& p2, const Point3d& p3
        ) ;

        Point2d MATH_API triangle_circum_center(
            const Point2d& p1, const Point2d& p2, const Point2d& p3
        ) ;

        inline bool exact_equal(const Point3d& a,const Point3d& b) {
            return a.x()==b.x() && a.y()==b.y() && a.z()==b.z();
        }

        inline bool exact_equal(const Point2d& a,const Point2d& b) {
            return a.x()==b.x() && a.y()==b.y() ;
        }

        inline bool exact_equal(const Vector3d& a,const Vector3d& b) {
            return a.x()==b.x() && a.y()==b.y() && a.z()==b.z();
        }

        inline bool exact_equal(const Vector2d& a,const Vector2d& b) {
            return a.x()==b.x() && a.y()==b.y() ;
        }

        inline bool is_nan(const Vector3d& v) {
            return 
                Numeric::is_nan(v.x()) ||
                Numeric::is_nan(v.y()) ||
                Numeric::is_nan(v.z()) ;
        }

        inline bool is_nan(const Point3d& v) {
            return 
                Numeric::is_nan(v.x()) ||
                Numeric::is_nan(v.y()) ||
                Numeric::is_nan(v.z()) ;
        }

        inline bool is_nan(const Vector2d& v) {
            return 
                Numeric::is_nan(v.x()) ||
                Numeric::is_nan(v.y()) ;
        }

        inline bool is_nan(const Point2d& p) {
            return 
                Numeric::is_nan(p.x()) ||
                Numeric::is_nan(p.y()) ;
        }

        /**
         * Wrapper for shewchuk's robust orientation predicate.
         */
        Sign MATH_API orient(
            const Point2d& p1, const Point2d& p2, const Point2d& p3
        ) ;

        bool MATH_API segments_intersect(
            const Point2d& p1, const Point2d& p2,
            const Point2d& p3, const Point2d& p4
        ) ;

        inline double segments_intersection_parameter(
            const Point2d& p1, const Vector2d& v1,
            const Point2d& p2, const Vector2d& v2
        ) {
            double delta = v1.x() * v2.y() - v1.y() * v2.x() ;
            double t1 = (v2.y() * (p2.x() - p1.x()) - v2.x() * (p2.y() - p1.y())) / delta ;
            return t1 ;
        }

        inline Point2d segments_intersection(
            const Point2d& p1, const Vector2d& v1,
            const Point2d& p2, const Vector2d& v2
        ) {
            double t1 = segments_intersection_parameter(p1, v1, p2, v2) ;
            return p1 + t1 * v1 ;
        }

        inline Point2d segments_intersection(
            const Point2d& p1, const Point2d& p2,
            const Point2d& p3, const Point2d& p4
        ) {
            return segments_intersection(
                p1, (p2 -p1), p3, (p4-p3)
            ) ;
        }

        Vector3d MATH_API perpendicular(const Vector3d& V) ;

        inline double tetra_signed_volume(
            const Point3d& p1, const Point3d& p2, 
            const Point3d& p3, const Point3d& p4
        ) {
            return ((p2 - p1) * ((p3 - p1) ^ (p4 - p1))) / 6.0 ;
        }

        inline double tetra_volume(
            const Point3d& p1, const Point3d& p2, 
            const Point3d& p3, const Point3d& p4
        ) {
            return ::fabs(tetra_signed_volume(p1,p2,p3,p4)) ;
        }

        Point3d MATH_API tetra_circum_center(
            const Point3d& p1, const Point3d& p2, 
            const Point3d& p3, const Point3d& p4
        ) ;

    }
}

#endif


