/*
 *  OGF/Graphite: Geometry and Graphics Programming Library + Utilities
 *  Copyright (C) 2000-2005 INRIA - Project ALICE
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
 *  Contact: Bruno Levy - levy@loria.fr
 *
 *     Project ALICE
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs. 
 *
 * As an exception to the GPL, Graphite can be linked with the following (non-GPL) libraries:
 *     Qt, SuperLU, WildMagic and CGAL
 */

#include <OGF/math/common/common.h>
#include <OGF/math/linear_algebra/vector.h>
#include <OGF/basic/types/types.h>
#include <math.h>

#ifndef __OGF_MATH_NUMERIC_QUICK_BLAS__
#define __OGF_MATH_NUMERIC_QUICK_BLAS__

namespace OGF {

    namespace QuickBLAS {

        void MATH_API initialize() ;

        extern Numeric::uint64 flops_counter ;

        bool MATH_API needs_alignment() ;
        unsigned int MATH_API aligned_size(unsigned int N, unsigned int item_size) ;

        //------------------------------------------------------------------------------------------------------------

        inline unsigned int daligned_size(unsigned int N) { return aligned_size(N, sizeof(double)) ; }

        /** y <- a*x+y */
        void  MATH_API daxpy(unsigned int N, double a, const double* x, double* y) ;
        
        /** y <- x */
        void MATH_API dcopy(unsigned int N, const double* x, double* y) ;

        /** returns x^T*y */
        double  MATH_API ddot(unsigned int N, const double* x, const double* y)  ;
        
        /** returns |x|^2 */
        double  MATH_API dsquared_nrm2(unsigned int N, const double *x) ;
        
        /** x <- alpha * x */
        void MATH_API dscal(unsigned int N, double alpha, double* x) ;
    
        /** y <- 0 */
        void MATH_API dzero(unsigned int N, double* y) ;

        /** y <- x1 * x2 (coefficient by coefficient) */
        void MATH_API dmul(unsigned int N, const double* x1, const double* x2, double* y) ;

        //-----------------------------------------------------

        /** y <- a*x+y */
        inline void axpy(double a, const Array1d<double>& x, Array1d<double>& y) {
            ogf_debug_assert(x.size() == y.size()) ;
            daxpy(x.size(), a, x.data(), y.data()) ;
        }
        
        /** y <- x */
        inline void copy(const Array1d<double>& x, Array1d<double>& y) {
            ogf_debug_assert(x.size() == y.size()) ;
            dcopy(x.size(), x.data(), y.data()) ;
        }

        /** returns x^T*y */
        inline double dot(const Array1d<double>& x, const Array1d<double>& y)  {
            ogf_debug_assert(x.size() == y.size()) ;
            return ddot(x.size(), x.data(), y.data()) ;
        }
        
        /** returns |x|^2 */
        inline double squared_nrm2(const Array1d<double>& x) {
            return dsquared_nrm2(x.size(), x.data()) ;
        }
        
        /** x <- alpha * x */
        inline void scal(double a, Array1d<double>& x) {
            dscal(x.size(), a, x.data()) ;
        }
    
        /** y <- 0 */
        inline void zero(Array1d<double>& y) {
            dzero(y.size(), y.data()) ;
        }

        /** y <- x1 * x2 (coefficient by coefficient) */
        inline void mul(const Array1d<double>& x1, const Array1d<double>& x2, Array1d<double>& y) {
            ogf_debug_assert(x1.size() == x2.size()) ;
            ogf_debug_assert(x2.size() == y.size()) ;
            dmul(x1.size(), x1.data(), x2.data(), y.data()) ;
        }

        //------------------------------------------------------------------------------------------------------------

        inline unsigned int saligned_size(unsigned int N) { return aligned_size(N, sizeof(float)) ; }

        /** y <- a*x+y */
        void  MATH_API saxpy(unsigned int N, float a, const float* x, float* y) ;
        
        /** y <- x */
        void MATH_API scopy(unsigned int N, const float* x, float* y) ;

        /** returns x^T*y */
        float  MATH_API sdot(unsigned int N, const float* x, const float* y)  ;
        
        /** returns |x|^2 */
        float  MATH_API ssquared_nrm2(unsigned int N, const float *x) ;
        
        /** x <- alpha * x */
        void MATH_API sscal(unsigned int N, float alpha, float* x) ;
    
        /** y <- 0 */
        void MATH_API szero(unsigned int N, float* y) ;

        /** y <- x1 * x2 (coefficient by coefficient) */
        void MATH_API smul(unsigned int N, const float* x1, const float* x2, float* y) ;

        //------------------------------------------------------------------------------------------------------------

        /** y <- a*x+y */
        inline void axpy(float a, const Array1d<float>& x, Array1d<float>& y) {
            ogf_debug_assert(x.size() == y.size()) ;
            saxpy(x.size(), a, x.data(), y.data()) ;
        }
        
        /** y <- x */
        inline void copy(const Array1d<float>& x, Array1d<float>& y) {
            ogf_debug_assert(x.size() == y.size()) ;
            scopy(x.size(), x.data(), y.data()) ;
        }

        /** returns x^T*y */
        inline float dot(const Array1d<float>& x, const Array1d<float>& y)  {
            ogf_debug_assert(x.size() == y.size()) ;
            return sdot(x.size(), x.data(), y.data()) ;
        }
        
        /** returns |x|^2 */
        inline float squared_nrm2(const Array1d<float>& x) {
            return ssquared_nrm2(x.size(), x.data()) ;
        }
        
        /** x <- alpha * x */
        inline void scal(float a, Array1d<float>& x) {
            sscal(x.size(), a, x.data()) ;
        }
    
        /** y <- 0 */
        inline void zero(Array1d<float>& y) {
            szero(y.size(), y.data()) ;
        }

        /** y <- x1 * x2 (coefficient by coefficient) */
        inline void mul(const Array1d<float>& x1, const Array1d<float>& x2, Array1d<float>& y) {
            ogf_debug_assert(x1.size() == x2.size()) ;
            ogf_debug_assert(x2.size() == y.size()) ;
            smul(x1.size(), x1.data(), x2.data(), y.data()) ;
        }

        //------------------------------------------------------------------------------------------------------------

    }

}

#endif
