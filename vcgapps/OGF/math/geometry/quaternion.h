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
 
 
 
 

#ifndef __OGF_MATH_GEOMETRY_QUATERNION__
#define __OGF_MATH_GEOMETRY_QUATERNION__

#include <OGF/math/common/common.h>
#include <OGF/math/linear_algebra/matrix.h>
#include <OGF/math/geometry/point_vector_3d.h>

#include <iostream>

namespace OGF {


//_________________________________________________________

/**
 * Quaternions are usefull for representing rotations. 
 * This class is inspired by an implementation written 
 * by Paul Rademacher, in his glui library.
 */
    template <class FT> class Quaternion {
    public:
        typedef Matrix<FT,4> matrix_type ;
        typedef GenericVector3d<FT> vector_type ;
        typedef FT scalar_type ;
    
        Quaternion() ;
        Quaternion(const Quaternion<FT>& rhs) ;
        Quaternion(
            scalar_type x, scalar_type y, scalar_type z, scalar_type w
        );
        Quaternion( const vector_type& v, scalar_type s ); 
    
    
        Quaternion<FT>& operator = ( const Quaternion &v ); 
        Quaternion<FT>& operator += ( const Quaternion &v );
        Quaternion<FT>& operator -= ( const Quaternion &v );
        Quaternion<FT>& operator *= ( const scalar_type d );
        Quaternion<FT>& operator /= ( const scalar_type d );
        scalar_type& operator [] ( int i); /* indexing */
  
        scalar_type length();
        scalar_type length2(); 
        void normalize(); 
        void set( scalar_type x, scalar_type y, scalar_type z ); 
        void set( const vector_type& v, scalar_type s ); 
        void print( std::ostream& out ) const ;  

        /** q * v * q^-1 */
        vector_type transform( const vector_type &v ); 

        matrix_type to_matrix() const ;
        void  set_angle( scalar_type f );               
        void  scale_angle( scalar_type f );  
        scalar_type angle() const ; 
        vector_type axis() const ; 

        static Quaternion<FT> spherical_interpolation( 
            const Quaternion<FT>& from, const Quaternion<FT>& to, 
            scalar_type t 
        ) ;

        inline const vector_type& v() const ;
        inline scalar_type s() const ;

    
    protected:
    private:
        vector_type v_ ;
        scalar_type s_ ;
    } ;


    template <class T> Quaternion<T> operator - (
        const Quaternion<T> &v
    );    

    template <class FT>
    Quaternion<FT> operator + (
        const Quaternion<FT> &a, const Quaternion<FT> &b
    ) ;

    template <class FT>
    Quaternion<FT> operator - (
        const Quaternion<FT> &a, const Quaternion<FT> &b
    ) ; 

    template <class FT>
    Quaternion<FT> operator * (
        const Quaternion<FT> &a, typename Quaternion<FT>::scalar_type d
    ) ;

    template <class FT>
    Quaternion<FT> operator * (
        typename Quaternion<FT>::scalar_type d, const Quaternion<FT> &a
    ) ;

    template <class FT>
    Quaternion<FT> operator * (
        const Quaternion<FT> &a, const Quaternion<FT> &b
    ) ;

    template <class FT>
    Quaternion<FT> operator / (
        const Quaternion<FT> &a, typename Quaternion<FT>::scalar_type d
    ) ;

    template <class FT>
    bool operator == (
        const Quaternion<FT> &a, const Quaternion<FT> &b
    ) ;

    template <class FT>
    bool operator != (
        const Quaternion<FT> &a, const Quaternion<FT> &b
    ) ;

//______________________________________________________________________


    template <class FT>
    inline const typename Quaternion<FT>::vector_type& Quaternion<FT>::v() const {
        return v_ ;
    }

    template <class FT>
    inline typename Quaternion<FT>::scalar_type Quaternion<FT>::s() const {
        return s_ ;
    }


    template <class FT>
    inline std::ostream& operator<<(std::ostream& out, const Quaternion<FT>& q) {
        q.print(out) ;
        return out ;
    }

    template <class FT>
    inline std::istream& operator>>(std::istream& in, Quaternion<FT>& q) {
        FT x,y,z,w ;
        in >> x >> y >> z >> w ;
        q.set(Quaternion<FT>::vector_type(x,y,z),w) ;
        return in ;
    }



    static const double SMALL = .00001 ;

    template <class FT> 
    Quaternion<FT>::Quaternion() : v_(0,0,0), s_(1) {
    }

    template <class FT> 
    Quaternion<FT>::Quaternion(
        scalar_type x, scalar_type y, scalar_type z, scalar_type w
    ) : v_(x,y,z), s_(w) {
    }

    template <class FT> 
    Quaternion<FT>::Quaternion( 
        const vector_type& v, scalar_type s 
    ) : v_(v), s_(s) {
    }

    template <class FT> 
    Quaternion<FT>::Quaternion( 
        const Quaternion<FT>& rhs
    ) : v_(rhs.v_), s_(rhs.s_) {
    }

    template <class FT> 
    void Quaternion<FT>::set( const vector_type& v, scalar_type s ) {
        v_ = v ;
        s_ = s ;
    }

    template <class FT> 
    Quaternion<FT>& Quaternion<FT>::operator = (const Quaternion<FT>& q) { 
        v_ = q.v_ ;  
        s_ = q.s_ ; 
        return *this ; 
    }

    template <class FT> 
    Quaternion<FT> operator + (const Quaternion<FT>& a, const Quaternion<FT>& b) {
        return Quaternion<FT>(
            a.v() + b.v(), 
            a.s() + b.s() 
        ) ;
    }

    template <class FT> 
    Quaternion<FT> operator - (const Quaternion<FT>& a, const Quaternion<FT>& b) {
        return Quaternion<FT>( 
            a.v() - b.v(), 
            a.s() - b.s() 
        ) ;
    }

    template <class FT> 
    Quaternion<FT> operator - (const Quaternion<FT>& a ) {
        return Quaternion<FT>( -1.0 * a.v(), -a.s() );
    }

    template <class FT> 
    Quaternion<FT> operator * ( const Quaternion<FT>& a, const Quaternion<FT>& b) {
        return Quaternion<FT>( 
            a.s() * b.v() + b.s() * a.v() + a.v()^b.v(), 
            a.s() * b.s() - a.v() * b.v() 
        );
    }

    template <class FT> 
    Quaternion<FT> operator * ( 
        const Quaternion<FT>& a, typename Quaternion<FT>::scalar_type t
    ) {
        return Quaternion<FT>( t * a.v(), a.s() * t );
    }

    template <class FT> 
    Quaternion<FT> operator * ( 
        typename Quaternion<FT>::scalar_type t, const Quaternion<FT>& a 
    ) {
        return Quaternion<FT>( t * a.v(), a.s() * t );
    }

    template <class FT> 
    typename Quaternion<FT>::matrix_type Quaternion<FT>::to_matrix() const {
        scalar_type t, xs, ys, zs, wx, wy, wz, xx, xy, xz, yy, yz, zz;
//    vector_type  a, c, b, d;
  
        t  = 2.0 / (v_ * v_ + s_ * s_);
  
        xs = v_.x() * t ; 
        ys = v_.y() * t ;
        zs = v_.z() * t ;
    
        wx = s_ * xs ;
        wy = s_ * ys ; 
        wz = s_ * zs ;

        xx = v_.x() * xs ;
        xy = v_.x() * ys ;
        xz = v_.x() * zs ;

        yy = v_.y() * ys ;
        yz = v_.y() * zs ;
        zz = v_.z() * zs ;
  
        matrix_type matrix ;


        matrix(0,0) = 1.0 - (yy+zz) ;
        matrix(1,0) = xy + wz ;
        matrix(2,0) = xz - wy ;
        matrix(0,1) = xy - wz ;
        matrix(1,1) = 1.0 - (xx+zz) ;
        matrix(2,1) = yz+wx ;
        matrix(0,2) = xz + wy ;
        matrix(1,2) = yz - wx ;
        matrix(2,2) = 1.0 - (xx+yy) ;

        return matrix;
    }


    template <class FT> 
    Quaternion<FT> Quaternion<FT>::spherical_interpolation( 
        const Quaternion<FT>& from, 
        const Quaternion<FT>& to, 
        typename Quaternion<FT>::scalar_type t 
    ) {
        Quaternion<FT> to1;
        double omega, cosom, sinom, scale0, scale1;
    
        /* calculate cosine */
        cosom = from.v() * to.v() + from.s() + to.s();
    
        /* Adjust signs (if necessary) */
        if ( cosom < 0.0 ) {
            cosom = -cosom;
            to1 = -to;
        } else {
            to1 = to;
        }
    
        /* Calculate coefficients */
        if ((1.0 - cosom) > SMALL ) {
            /* standard case (slerp) */
            omega = acos( cosom );
            sinom = sin( omega );
            scale0 = sin((1.0 - t) * omega) / sinom;
            scale1 = sin(t * omega) / sinom;
        } else {
            /* 'from' and 'to' are very close - just do linear interpolation */
            scale0 = 1.0 - t;
            scale1 = t;      
        }
        return scale0 * from + scale1 * to1;
    }

    template <class FT> 
    void Quaternion<FT>::set_angle( scalar_type f ) {
        vector_type axis = axis();
        s_ = cos(f) / 2.0 ;
        v_ = axis * sin(f / 2.0);
    }

    template <class FT> 
    void Quaternion<FT>::scale_angle( scalar_type f ) {
        set_angle( f * angle() );
    }

    template <class FT> 
    typename Quaternion<FT>::scalar_type Quaternion<FT>::angle() const {
        return 2.0 * acos( s_ ) ;
    }


    template <class FT> 
    typename Quaternion<FT>::vector_type Quaternion<FT>::axis() const {
        scalar_type scale;
    
        scale = sin( acos( s ) );
        if ( scale < SMALL && scale > -SMALL ) {
            return vector_type( 0.0, 0.0, 0.0 );
        } else {
            return  v_ / scale;
        }
    }

    template <class FT> 
    void Quaternion<FT>::print( std::ostream& out ) const {
        out << v_.x() << " " << v_.y() << " " << v_.z() << " " << s_ ;
    }	
    
//_________________________________________________________

}

#endif
