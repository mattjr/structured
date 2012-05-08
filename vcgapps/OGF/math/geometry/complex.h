/*
 *  GXML/Graphite: Geometry and Graphics Programming Library + Utilities
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

#ifndef __OGF_MATH_GEOMETRY_COMPLEX__
#define __OGF_MATH_GEOMETRY_COMPLEX__
 
#include <OGF/math/common/common.h>
#include <OGF/math/geometry/types.h>
#include <math.h>

namespace OGF {

// Fabien 25/11/2003
// in debug mode it causes some link problem;
// as it's defined in the .h removed the MATH_API
// now it seems to work fine
    class /*MATH_API*/ Complex {
    public:
        Complex(double a=0, double b=0) : real_(a), imaginary_(b) { }
        Complex(const Point2d& p) : real_(p.x()), imaginary_(p.y()) { }
        double real() const { return real_ ; }
        double imaginary() const { return imaginary_ ; }

        double squared_modulus() const {
            return real_*real_ + imaginary_*imaginary_ ;
        }

        double modulus() const {
            return ::sqrt(squared_modulus()) ;
        }

        double angle() const {
            double result = 0.0 ;
            double m = modulus() ;
            if(m > 1e-30) {
                result = acos(real_ / modulus() );
                if (imaginary_ < 0) {
                    result = - result ;
                }
            }
            return result;
        }

        void normalize() { 
            double n = modulus() ; 
            double s = n < 1e-20 ? 0.0 : 1.0 / n ;
            real_ *= s ;
            imaginary_ *= s ; 
        }

        Complex normalized() const {
            Complex result = *this ;
            result.normalize() ;
            return result ;
        }

        Complex& operator+=(const Complex& rhs) {
            real_ += rhs.real_ ;
            imaginary_ += rhs.imaginary_ ;
			return *this ;
        }

        Complex& operator-=(const Complex& rhs) {
            real_ -= rhs.real_ ;
            imaginary_ -= rhs.imaginary_ ;
			return *this ;
        }
        
    private:
        double real_ ;
        double imaginary_ ;
    } ;

    inline Complex operator*(double a, const Complex& z) {
        return Complex(a*z.real(), a*z.imaginary()) ;
    }

    inline Complex operator+(const Complex& z1, const Complex& z2) {
        return Complex(
            z1.real() + z2.real(),
            z1.imaginary() + z2.imaginary()
        ) ;
    }

    inline Complex operator-(const Complex& z1, const Complex& z2) {
        return Complex(
            z1.real() - z2.real(),
            z1.imaginary() - z2.imaginary()
        ) ;
    }

    inline Complex operator*(const Complex& z1, const Complex& z2) {
        return Complex(
            z1.real()*z2.real() - z1.imaginary()*z2.imaginary(),
            z1.real()*z2.imaginary() + z1.imaginary()*z2.real()
        ) ;
    }

    inline Complex operator/(const Complex& z1, const Complex& z2) {
        double d = z2.squared_modulus() ;
        return Complex(
            (z1.real()*z2.real()+z1.imaginary()*z2.imaginary()) / d,
            (z1.imaginary()*z2.real() - z1.real()*z2.imaginary()) / d
        ) ;
    }

    inline Complex complex_sqrt(const Complex& z){
        double r = ::sqrt(z.modulus());
        double angle = 0.5*z.angle() ;
        return Complex(r*::cos(angle),r*::sin(angle));
    }

    inline std::ostream& operator<<(std::ostream& out, const Complex& z) {
        return out << z.real() << " " << z.imaginary() ;
    }

    inline std::istream& operator>>(std::istream& in, Complex& z) {
        double real,imag ;
        in >> real >> imag ;
        z = Complex(real,imag) ;
        return in ;
    }


} 

#endif
