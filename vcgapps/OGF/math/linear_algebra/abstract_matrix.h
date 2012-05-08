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
 

#ifndef ___OGF_MATH_LINEAR_ALGEBRA_ABSTRACT_MATRIX__
#define ___OGF_MATH_LINEAR_ALGEBRA_ABSTRACT_MATRIX__

#include <OGF/math/common/common.h>
#include <OGF/math/linear_algebra/vector.h>
#include <OGF/basic/types/counted.h>
#include <OGF/basic/types/smart_pointer.h>

namespace OGF {

    /**
     * Client code may not access this class directly.
     */
    class AbstractMatrixImpl : public Counted {
    public:
        virtual ~AbstractMatrixImpl() ;
        virtual void mult(double* y, const double* x) = 0 ;
        virtual int* permutation() const ;
    } ;

    typedef SmartPointer<AbstractMatrixImpl> AbstractMatrixImpl_var ;

    /**
     * The abstract class to represent the result of a direct sparse factorization
     * method.
     */
    class AbstractMatrix {
    public:
        AbstractMatrix() : n_(0) { }
        void mult(double* y, const double* x) { impl_->mult(y, x) ; }
        void mult(Vector& y, const Vector& x) { mult(y.data(), x.data()) ; }
        int n() const { return n_ ; }
        int* permutation() const { return impl_->permutation() ; }

        /** client code may not use this function directly */
        void set_impl(int n, AbstractMatrixImpl* impl) { n_ = n ; impl_ = impl ; }
        /** client code may not use this function directly */        
        AbstractMatrixImpl* impl() { return impl_ ; }
    private:
        int n_ ;
        AbstractMatrixImpl_var impl_ ;
    } ;

}

#endif
