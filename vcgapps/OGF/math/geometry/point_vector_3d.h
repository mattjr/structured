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
 

#ifndef __OGF_MATH_GEOMETRY_POINT_VECTOR_3D__
#define __OGF_MATH_GEOMETRY_POINT_VECTOR_3D__

#include <OGF/math/common/common.h>
#include <OGF/math/geometry/origin.h>

#include <iostream>

#include <math.h>


namespace OGF {


//_________________________________________________________



    template <class FT> class GenericPoint3d {
    public:
        GenericPoint3d() { 
            data_[0] = 0 ; data_[1] = 0 ;  data_[2] = 0 ; 
        }
        GenericPoint3d(FT x, FT y, FT z) { 
            data_[0] = x ; data_[1] = y ;  data_[2] = z ; 
        }
        FT x() const { return data_[0] ;  }
        FT y() const { return data_[1] ;  }        
        FT z() const { return data_[2] ;  }        
        void set_x(FT x) { data_[0] = x ; }
        void set_y(FT y) { data_[1] = y ; }
        void set_z(FT z) { data_[2] = z ; }
        FT operator[](int i) const { return data_[i] ; }
        FT& operator[](int i) { return data_[i] ; }

        // Compatibility with CGAL
        FT homogeneous(int i) const { return (i < 3) ? data_[i] : FT(1) ; }
        FT cartesian(int i) const { return data_[i] ; } 


        // Low-level access
        const FT* data() const { return data_ ; }
        FT* data() { return data_ ; }

    private:
        FT data_[3] ;
    } ;


    template <class FT> class GenericVector3d {
    public:
        typedef GenericVector3d<FT> thisclass ;

        GenericVector3d() { 
            data_[0] = 0 ; data_[1] = 0 ;  data_[2] = 0 ; 
        }
        GenericVector3d(FT x, FT y, FT z) { 
            data_[0] = x ; data_[1] = y ;  data_[2] = z ; 
        }
        FT x() const { return data_[0] ;  }
        FT y() const { return data_[1] ;  }        
        FT z() const { return data_[2] ;  }        
        void set_x(FT x) { data_[0] = x ; }
        void set_y(FT y) { data_[1] = y ; }
        void set_z(FT z) { data_[2] = z ; }
        FT operator[](int i) const { return data_[i] ; }
        FT& operator[](int i) { return data_[i] ; }

        // Compatibility with CGAL
        FT homogeneous(int i) const { return (i < 3) ? data_[i] : FT(0) ; }
        FT cartesian(int i) const { return data_[i] ; } 
        
        FT norm2() const    { 
            return data_[0]*data_[0] + data_[1]*data_[1] + data_[2]*data_[2]; 
        }
        double norm() const { return ::sqrt(norm2()) ; } 
        void normalize() { 
            double n = norm() ; 
            double s = n < 1e-20 ? 0.0 : 1.0 / n ;
            data_[0] = FT(s * data_[0]) ;
            data_[1] = FT(s * data_[1]) ;
            data_[2] = FT(s * data_[2]) ;
        }
        GenericVector3d<FT> normalized() const {
            GenericVector3d<FT> result = *this ;
            result.normalize() ;
            return result ;
        }

        thisclass& operator += (const GenericVector3d<FT>& rhs) {
            data_[0] += rhs.data_[0] ;
            data_[1] += rhs.data_[1] ;
            data_[2] += rhs.data_[2] ;
            return *this ;
        }

        thisclass& operator -= (const GenericVector3d<FT>& rhs) {
            data_[0] -= rhs.data_[0] ;
            data_[1] -= rhs.data_[1] ;
            data_[2] -= rhs.data_[2] ;
            return *this ;
        }

        // Low-level access
        const FT* data() const { return data_ ; }
        FT* data() { return data_ ; }
        
    private:
        FT data_[3] ;
    } ;


//___________________________________________________________

    //------------- Points operators --------------------

    template <class FT> GenericPoint3d<FT> operator+ (
        const GenericPoint3d<FT>& p,
        const GenericVector3d<FT>& v
    ) {
        return GenericPoint3d<FT>(p.x()+v.x(), p.y()+v.y(), p.z()+v.z()) ;
    }

    template <class FT> GenericPoint3d<FT> operator- (
        const GenericPoint3d<FT>& p,
        const GenericVector3d<FT>& v
    ) {
        return GenericPoint3d<FT>(p.x()-v.x(), p.y()-v.y(), p.z()-v.z()) ;
    }

    template <class FT> GenericVector3d<FT> operator- (
        const GenericPoint3d<FT>& p1,
        const GenericPoint3d<FT>& p2
    ) {
        return GenericVector3d<FT>(
            p1.x()-p2.x(), p1.y()-p2.y(), p1.z()-p2.z()
        ) ;
    }

    //------------- Vectors operators --------------------

    template <class FT> GenericVector3d<FT> operator+ (
        const GenericVector3d<FT>& v1,
        const GenericVector3d<FT>& v2
    ) {
        return GenericVector3d<FT>(
            v1.x()+v2.x(), v1.y()+v2.y(), v1.z()+v2.z()
        ) ;
    }

    template <class FT> GenericVector3d<FT> operator- (
        const GenericVector3d<FT>& v1,
        const GenericVector3d<FT>& v2
    ) {
        return GenericVector3d<FT>(
            v1.x()-v2.x(), v1.y()-v2.y(), v1.z()-v2.z()
        ) ;
    }

    // dot-product
    template <class FT> FT operator* (
        const GenericVector3d<FT>& v1,
        const GenericVector3d<FT>& v2
    ) {
        return v1.x()*v2.x() + v1.y()*v2.y() + v1.z()*v2.z() ;
    }

    template <class FT> GenericVector3d<FT> operator* (
        FT k, const GenericVector3d<FT>& v
    ) {
        return GenericVector3d<FT>(k*v.x(), k*v.y(), k*v.z()) ;
    }

    template <class FT> GenericVector3d<FT> operator/ (
        const GenericVector3d<FT>& v, FT k
    ) {
        return FT(1.0 / double(k)) * v ;
    }


    // cross-product
    template <class FT> GenericVector3d<FT> operator^ (
        const GenericVector3d<FT>& v1,
        const GenericVector3d<FT>& v2
    ) {
        return GenericVector3d<FT>(
            v1.y()*v2.z() - v2.y()*v1.z(),
            v1.z()*v2.x() - v2.z()*v1.x(),
            v1.x()*v2.y() - v2.x()*v1.y()
        ) ;
    }


    //---- Points-vector conversions (a-la CGAL) ------

    template <class FT> GenericVector3d<FT> operator-(
        const GenericPoint3d<FT>& p, Origin
    ) {
        return GenericVector3d<FT>(p.x(), p.y(), p.z()) ;
    }

    template <class FT> GenericPoint3d<FT> operator+(
        Origin, const GenericVector3d<FT>& v
    ) {
        return GenericPoint3d<FT>(v.x(), v.y(), v.z()) ;
    }

    //----------- Input/Output -------------------------

    template <class FT>
    std::istream& operator>>(std::istream& in, GenericPoint3d<FT>& p) {
        FT x, y, z ;
        in >> x >> y >> z ;
        p.set_x(x) ;
        p.set_y(y) ;
        p.set_z(z) ;
        return in ;
    }

    template <class FT>
    std::istream& operator>>(std::istream& in, GenericVector3d<FT>& v) {
        FT x, y, z ;
        in >> x >> y >> z ;
        v.set_x(x) ;
        v.set_y(y) ;
        v.set_z(z) ;
        return in ;
    }


    template <class FT>
    std::ostream& operator<<(std::ostream& out, const GenericPoint3d<FT>& p) {
        return out << p.x() << ' ' << p.y() << ' ' << p.z() ;
    }


    template <class FT>
    std::ostream& operator<<(std::ostream& out, const GenericVector3d<FT>& v) {
        return out << v.x() << ' ' << v.y() << ' ' << v.z() ;
    }


    //----------- Squared distance --------------

    template <class FT> FT squared_distance(
        const GenericPoint3d<FT>& p1,
        const GenericPoint3d<FT>& p2
    ) {
        return 
            ogf_sqr(p2.x() - p1.x()) +
            ogf_sqr(p2.y() - p1.y()) +
            ogf_sqr(p2.z() - p1.z()) ;
    }

//_________________________________________________________

}
#endif

