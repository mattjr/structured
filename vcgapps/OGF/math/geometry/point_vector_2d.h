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
 

#ifndef __OGF_MATH_GEOMETRY_POINT_VECTOR_2D__
#define __OGF_MATH_GEOMETRY_POINT_VECTOR_2D__

#include <OGF/math/common/common.h>
#include <OGF/math/geometry/origin.h>

#include <iostream>

#include <math.h>

namespace OGF {

//_________________________________________________________


    template <class FT> class GenericPoint2d {
    public:
        GenericPoint2d()           { data_[0] = 0 ; data_[1] = 0 ;  }
        GenericPoint2d(FT x, FT y) { data_[0] = x ; data_[1] = y ;  }
        FT x() const { return data_[0] ;  }
        FT y() const { return data_[1] ;  }        
        void set_x(FT x) { data_[0] = x ; }
        void set_y(FT y) { data_[1] = y ; }
        FT operator[](int i) const { return data_[i] ; }
        FT& operator[](int i) { return data_[i] ; }

        // Compatibility with CGAL
        FT homogeneous(int i) const { return i < 2 ? data_[i] : FT(1) ; }
        FT cartesian(int i) const { return data_[i] ; } 

        // Low-level access
        const FT* data() const { return data_ ; }
        FT* data() { return data_ ; }

    private:
        FT data_[2] ;
    } ;


    template <class FT> class GenericVector2d {
    public:
        typedef GenericVector2d<FT> thisclass ;

        GenericVector2d()           { data_[0] = 0 ; data_[1] = 0 ;  }
        GenericVector2d(FT x, FT y) { data_[0] = x ; data_[1] = y ;  }
        FT x() const { return data_[0] ;  }
        FT y() const { return data_[1] ;  }        
        void set_x(FT x) { data_[0] = x ; }
        void set_y(FT y) { data_[1] = y ; }
        FT operator[](int i) const { return data_[i] ; }
        FT& operator[](int i) { return data_[i] ; }

        // Compatibility with CGAL
        FT homogeneous(int i) const { return i < 2 ? data_[i] : FT(0) ; }
        FT cartesian(int i) const { return data_[i] ; } 
        
        FT norm2() const    { return data_[0]*data_[0] + data_[1]*data_[1] ; }
        double norm() const { return ::sqrt(norm2()) ; } 
        void normalize() { 
            double n = norm() ; 
            double s = n < 1e-20 ? 0.0 : 1.0 / n ;
            data_[0] = FT(s * data_[0]) ;
            data_[1] = FT(s * data_[1]) ;
        }
        GenericVector2d<FT> normalized() const {
            GenericVector2d<FT> result = *this ;
            result.normalize() ;
            return result ;
        }

        thisclass& operator += (const GenericVector2d<FT>& rhs) {
            data_[0] += rhs.data_[0] ;
            data_[1] += rhs.data_[1] ;
            return *this ;
        }

        thisclass& operator -= (const GenericVector2d<FT>& rhs) {
            data_[0] -= rhs.data_[0] ;
            data_[1] -= rhs.data_[1] ;
            return *this ;
        }

        // Low-level access
        const FT* data() const { return data_ ; }
        FT* data() { return data_ ; }
        
    private:
        FT data_[2] ;
    } ;


//___________________________________________________________

    //------------- Points operators --------------------

    template <class FT> GenericPoint2d<FT> operator+ (
        const GenericPoint2d<FT>& p,
        const GenericVector2d<FT>& v
    ) {
        return GenericPoint2d<FT>(p.x()+v.x(), p.y()+v.y()) ;
    }

    template <class FT> GenericPoint2d<FT> operator- (
        const GenericPoint2d<FT>& p,
        const GenericVector2d<FT>& v
    ) {
        return GenericPoint2d<FT>(p.x()-v.x(), p.y()-v.y()) ;
    }

    template <class FT> GenericVector2d<FT> operator- (
        const GenericPoint2d<FT>& p1,
        const GenericPoint2d<FT>& p2
    ) {
        return GenericVector2d<FT>(p1.x()-p2.x(), p1.y()-p2.y()) ;
    }

    //------------- Vectors operators --------------------

    template <class FT> GenericVector2d<FT> operator+ (
        const GenericVector2d<FT>& v1,
        const GenericVector2d<FT>& v2
    ) {
        return GenericVector2d<FT>(v1.x()+v2.x(), v1.y()+v2.y()) ;
    }

    template <class FT> GenericVector2d<FT> operator- (
        const GenericVector2d<FT>& v1,
        const GenericVector2d<FT>& v2
    ) {
        return GenericVector2d<FT>(v1.x()-v2.x(), v1.y()-v2.y()) ;
    }

    // dot-product

    template <class FT> FT operator* (
        const GenericVector2d<FT>& v1,
        const GenericVector2d<FT>& v2
    ) {
        return v1.x()*v2.x() + v1.y()*v2.y() ;
    }

    template <class FT> GenericVector2d<FT> operator* (
        FT k, const GenericVector2d<FT>& v
    ) {
        return GenericVector2d<FT>(k*v.x(), k*v.y()) ;
    }

    template <class FT> GenericVector2d<FT> operator/ (
        const GenericVector2d<FT>& v, FT k
    ) {
        return FT(1.0 / double(k)) * v ;
    }

    //---- Points-vector conversions (a-la CGAL) ------

    template <class FT> GenericVector2d<FT> operator-(
        const GenericPoint2d<FT>& p, Origin
    ) {
        return GenericVector2d<FT>(p.x(), p.y()) ;
    }

    template <class FT> GenericPoint2d<FT> operator+(
        Origin, const GenericVector2d<FT>& v
    ) {
        return GenericPoint2d<FT>(v.x(), v.y()) ;
    }

    //----------- Input/Output -------------------------

    template <class FT>
    std::istream& operator>>(std::istream& in, GenericPoint2d<FT>& p) {
        FT x, y ;
        in >> x >> y ;
        p.set_x(x) ;
        p.set_y(y) ;
        return in ;
    }

    template <class FT>
    std::istream& operator>>(std::istream& in, GenericVector2d<FT>& v) {
        FT x, y ;
        in >> x >> y ;
        v.set_x(x) ;
        v.set_y(y) ;
        return in ;
    }


    template <class FT>
    std::ostream& operator<<(std::ostream& out, const GenericPoint2d<FT>& p) {
        return out << p.x() << ' ' << p.y() ;
    }


    template <class FT>
    std::ostream& operator<<(std::ostream& out, const GenericVector2d<FT>& v) {
        return out << v.x() << ' ' << v.y() ;
    }

    //----------- Squared distance --------------

    template <class FT> FT squared_distance(
        const GenericPoint2d<FT>& p1,
        const GenericPoint2d<FT>& p2
    ) {
        return 
            ogf_sqr(p2.x() - p1.x()) +
            ogf_sqr(p2.y() - p1.y()) ;
    }
    
//___________________________________________________________


}
#endif

