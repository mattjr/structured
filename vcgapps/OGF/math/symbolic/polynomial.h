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
 
#ifndef __OGF_MATH_SYMBOLIC_POLYNOMIAL__
#define __OGF_MATH_SYMBOLIC_POLYNOMIAL__

#include <OGF/math/common/common.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/debug/assert.h>
#include <iostream>

namespace OGF {

    //___________________________________________________________________________

    template <int N> class Polynomial {
    public:
        enum { degree = N} ;
        enum { nb_coeffs = N+1} ;
        typedef Polynomial<N> thisclass ;

        void clear() { Memory::clear(a_, nb_coeffs * sizeof(double)) ;  }

        double& operator()(unsigned int i) {
            ogf_assert(i <= degree) ;
            return a_[i] ;
        }

        const double& operator()(unsigned int i) const {
            ogf_assert(i <= degree) ;
            return a_[i] ;
        }

        Polynomial() { clear() ; }

        Polynomial(const thisclass& rhs) {
            Memory::copy(a_, rhs.a_, nb_coeffs * sizeof(double)) ;
        }

        thisclass& operator=(const thisclass& rhs) {
            Memory::copy(a_, rhs.a_, nb_coeffs * sizeof(double)) ;
            return *this ;
        }

        double eval(double x) const {
            double result = a_[N] ;
            for(int i=N-1; i>=0; i--) {
                result  = x * result + a_[i] ;
            }
            return result ;
        }

        Polynomial<N-1> derivative() const {
            Polynomial<N-1> result ;
            for(unsigned int i=1; i<=N; i++) {
                result(i-1) = i*a_[i] ;
            }
        }

        static thisclass x() {
            thisclass result ;
            result(1) = 1 ;
            return result ;
        }

        static thisclass constant(double c) {
            thisclass result ;
            result(0) = c ;
            return result ;
        }

        double a_[nb_coeffs] ;
    } ;

    //___________________________________________________________________________
    
    class MATH_API BiPolyBase {
    protected:
        /** sum(n) returns  1 + 2 + ... + n + (n+1) */
        unsigned int sum(int n) {  return (n < 4) ? sum_[n] : ((n+1) * (n+2) / 2) ; }
    private:
        static unsigned int sum_[] ;
    } ;

    template <int N> class BivariatePolynomial : public BiPolyBase {
    public:
        enum { degree = N} ;
        enum { nb_coeffs = (N+1)*(N+2)/2} ;
        typedef BivariatePolynomial<N> thisclass ;

        unsigned int offset(unsigned int i) const { 
            ogf_assert(i <= degree) ;
            return nb_coeffs - BiPolyBase::sum(degree - i) ;  
        }

        void clear() { Memory::clear(a_, nb_coeffs * sizeof(double)) ;  }

        double& operator()(unsigned int i, unsigned int j) {
            ogf_assert(i+j <= degree) ;
            return a_[offset(i)+j] ;
        }

        const double& operator()(unsigned int i, unsigned int j) const {
            ogf_assert(i+j <= degree) ;
            return a_[offset(i)+j] ;
        }

        BivariatePolynomial() { 
            clear() ; 
        }

        BivariatePolynomial(const thisclass& rhs) {
            Memory::copy(a_, rhs.a_, nb_coeffs * sizeof(double)) ;
        }

        thisclass& operator=(const thisclass& rhs) {
            Memory::copy(a_, rhs.a_, nb_coeffs * sizeof(double)) ;
            return *this ;
        }

        double eval(double u, double v) const {
            double U[degree+1] ;
            double V[degree+1] ;
            U[0] = 1 ; U[1] = u ;
            V[0] = 1 ; V[1] = v ;
            for(unsigned int i=2; i<degree+1; i++) {
                U[i] = u * U[i-1] ;
                V[i] = v * V[i-1] ;
            }
            double result = 0 ;
            for(unsigned int i=0; i<=degree; i++) {
                unsigned int ofs = offset(i) ;
                for(unsigned int j=0; (i+j) <= degree; i++) {
                    result += U[i] * V[j] * a_[ofs + j] ;
                }
            }
            return result ;
        }

        void print(std::ostream& out) const {
            for(unsigned int i=0; i<=N; i++) {
                unsigned int ofs = offset(i) ;
                for(unsigned int j=0; (i+j)<=N; j++) {
                    double x = a_[ofs + j] ;
                    if(i==0 && j==0) {
                        out << x ;
                    } else if(x != 0.0)  {
                        out << " + " << x ;
                        if(i >= 1) { out << " u" ; }
                        if(i >= 2) { out << "^" << i ; }                        
                        if(j >= 1) { out << " v" ; }
                        if(j >= 2) { out << "^" << j ; }
                    }
                }
            }
        }

        Polynomial<N> project_u(double u) const {
            Polynomial<N> result ;
            double U[degree+1] ;
            U[0] = 1 ; U[1] = u ;
            for(unsigned int i=2; i<degree+1; i++) {
                U[i] = u * U[i-1] ;
            }
            for(unsigned int i=0; i<=degree; i++) {
                for(unsigned int j=0; i+j<=degree; j++) {
                    result(i) += (*this)(i,j)*U[j] ;
                }
            }
            return result ;
        }

        Polynomial<N> project_v(double v) const {
            Polynomial<N> result ;
            double V[degree+1] ;
            V[0] = 1 ; V[1] = u ;
            for(unsigned int i=2; i<degree+1; i++) {
                V[i] = v * V[i-1] ;
            }
            for(unsigned int i=0; i<=degree; i++) {
                for(unsigned int j=0; i+j<=degree; j++) {
                    result(j) += (*this)(i,j)*V[i] ;
                }
            }
            return result ;
        }

        static thisclass u() {
            thisclass result ;
            result(1,0) = 1.0 ;
            return result ;
        }

        static thisclass v() {
            thisclass result ;
            result(0,1) = 1.0 ;
            return result ;
        }

        static thisclass constant(double c) {
            thisclass result ;
            result(0,0) = c ;
            return result ;
        }

        double a_[nb_coeffs] ;
    } ;

    template <int N> BivariatePolynomial<N> operator*(
        double op1, const BivariatePolynomial<N>& op2
    ) {
        BivariatePolynomial<N> result ;
        for(int i=0; i<BivariatePolynomial<N>::nb_coeffs; i++) {
            result.a_[i] = op1 * op2.a_[i] ;
        }
        return result ;
    }

    template <int N> BivariatePolynomial<N> operator*(
        const BivariatePolynomial<N>& op1, const BivariatePolynomial<N>& op2
    ) {
        BivariatePolynomial<N> result ;
        for(unsigned int i1=0; i1<=N; i1++) {
            for(unsigned int j1=0; (i1+j1)<=N; j1++) {
                double x1 = op1(i1,j1) ;
                for(unsigned int i2=0; i2<=N; i2++) {
                    for(unsigned int j2=0; (i2+j2)<=N; j2++) {
                        double x2 = op2(i2,j2) ;
                        double x = x1 * x2 ;
                        if(i1+j1+i2+j2 <= N) {
                            result(i1+i2,j1+j2) += x ;
                        } else {
                            ogf_assert(x == 0.0) ;
                        }
                    }
                }
            }
        }
        return result ;
    }

    template <int N> BivariatePolynomial<N> pow(
        const BivariatePolynomial<N>& op1, int op2
    ) {
        BivariatePolynomial<N> result = op1 ;
        for(int i=2; i<=op2; i++) {
            result = result * op1 ;
        }
        return result ;
    }

    template <int N> BivariatePolynomial<N> operator+(
        const BivariatePolynomial<N>& op1, const BivariatePolynomial<N>& op2
    ) {
        BivariatePolynomial<N> result ;
        for(int i=0; i<BivariatePolynomial<N>::nb_coeffs; i++) {
            result.a_[i] = op1.a_[i] + op2.a_[i] ;
        }
        return result ;
    }

    template <int N> BivariatePolynomial<N> operator-(
        const BivariatePolynomial<N>& op1, const BivariatePolynomial<N>& op2
    ) {
        BivariatePolynomial<N> result ;
        for(int i=0; i<BivariatePolynomial<N>::nb_coeffs; i++) {
            result.a_[i] = op1.a_[i] - op2.a_[i] ;
        }
        return result ;
    }

    template <int N> std::ostream& operator<<(std::ostream& out, const BivariatePolynomial<N>& P) {
        P.print(out) ;
        return out ;
    }

}

#endif
 

