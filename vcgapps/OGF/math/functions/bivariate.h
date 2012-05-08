
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
 
#ifndef __OGF_MATH_FUNCTIONS_BIVARIATE__
#define __OGF_MATH_FUNCTIONS_BIVARIATE__

#include <OGF/math/common/common.h>
#include <OGF/math/functions/function.h>
#include <OGF/math/functions/m_estimator.h>
#include <OGF/math/geometry/types.h>
#include <OGF/math/linear_algebra/matrix.h>
#include <OGF/math/numeric/conjugate_gradient.h>
#include <OGF/math/numeric/direct_solvers.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/debug/assert.h>

namespace OGF {

//__________________________________________________________

    template <int N> class BivariateBasis {
    public:
        enum { dimension = N } ;
    } ;
    

    class BivariateLinearBasis : public BivariateBasis<3> {
    public:
        static void bases(double x, double y, double* a) {
            a[0] = 1.0 ;
            a[1] = x ; 
            a[2] = y ;
        }
    } ;

    class BivariateQuadraticBasis : public BivariateBasis<6> {
    public:
        static void bases(double x, double y, double* a) {
            a[0] = 1.0 ;
            a[1] = x ;
            a[2] = y ;
            a[3] = x*x ;
            a[4] = x*y ;
            a[5] = y*y ;
        }
    } ;

    class BivariateCubicBasis : public BivariateBasis<10> {
    public:
        static void bases(double x, double y, double* a) {
            a[0] = 1.0 ;
            a[1] = x ;
            a[2] = y ;
            a[3] = x*x ;
            a[4] = x*y ;
            a[5] = y*y ;
            a[6] = a[3] * x ;
            a[7] = a[4] * x ;
            a[8] = a[4] * y ;
            a[9] = a[5] * y ;
        }
    } ;

    template <class BASIS> class BivariateFunction : public Function<BASIS::dimension> {
    public:
        enum { dimension = BASIS::dimension } ;
        typedef BASIS Basis ;
        typedef BivariateFunction<BASIS> thisclass ;
        typedef Function<dimension> baseclass ;

        BivariateFunction() { }

        BivariateFunction(const thisclass& rhs) : baseclass(rhs) { }
        thisclass& operator=(const thisclass& rhs) {
            baseclass::operator=(rhs) ;
            return *this ;
        }

        double eval(double x, double y) const {
            double a[dimension] ;
            BASIS::bases(x,y,a) ;
            double result = 0.0 ;
            for(unsigned int i=0; i<dimension; i++) {
                result += baseclass::coeff[i] * a[i] ;
            }
            return result ;
        }

        double eval(const Point2d& p) const {
            return eval(p.x(), p.y()) ;
        }
    } ;

    typedef BivariateFunction<BivariateLinearBasis>       BivariateLinearFunction ;
    typedef BivariateFunction<BivariateQuadraticBasis> BivariateQuadraticFunction ;
    typedef BivariateFunction<BivariateCubicBasis>       BivariateCubicFunction ;

    //____________________________________________________________________________________

    template <class FUNC> class BivariateFitting : public MEstimator {
    public:
        typedef FUNC Function ;
        typedef typename FUNC::Basis Basis ;
        enum { dimension = Basis::dimension } ;

        BivariateFitting() : M_mode_(false) { }

        void set_M_mode(bool x) { M_mode_ = x ; }
        void set_reference(const Function& ref) { reference_ = ref ; }

        void begin() {
            M_.load_zero() ;
            for(unsigned int i=0; i<Function::dimension; i++) {  
                b_[i] = 0.0 ;   
            }
        }

        void end() {
            if(!Numeric::solve_SPD_system(M_, b_, x_.coeff)) {

                iterative_solve() ;
                return ;

                // Note: here we use the special feature of the switch() 
                // construct in C++ (i.e. since there is no break statement,
                // it continues to other cases if solve_with_clamped_index()
                // did not succeed).
                switch(dimension) {
                case 10:
                    if(solve_with_clamped_index(6)) {
                        return ;
                    }
                case 6:
                    if(solve_with_clamped_index(3)) {
                        return ;
                    }
                case 3:
                    if(solve_with_clamped_index(1)) {
                        return ;
                    }
                default: 
                    iterative_solve() ;
                }
            }
        }

        void add_sample(double x, double y, double g, double importance = 1.0) {
            double v[Function::dimension] ;
            Function::Basis::bases(x,y,v) ;
            if(M_mode_) {
                importance *= weight(reference_.eval(x,y) - g) ;
            }
            add_sample(v,g,importance) ;
        }

        const Function& result() const { return x_ ; }

        Matrix<double, Function::dimension>& matrix() { return M_ ; }
        const Matrix<double, Function::dimension>& matrix() const { return M_ ; }
        double* rhs() { return b_ ; }
        const double* rhs() const { return b_ ; }

    protected:

        // If direct solve with LAPACK failed, we copy the lower
        // triangle of M on the upper triangle, and called the
        // conjugate gradient solver (note: we could make it
        // work without the upper triangle...)
        void iterative_solve() {
            x_.clear() ;
            for(unsigned int i=0; i<Function::dimension; i++) {
                for(unsigned int j=0; j<i; j++) {
                    M_(i,j) = M_(j,i) ;
                }
            }
            solve_conjugate_gradient_quiet(
                Function::dimension, M_, b_, x_.coeff, 1e-30, 5*Function::dimension
            )  ;
        }

        bool solve_with_clamped_index(unsigned int clamp_index) {
            Matrix<double, Function::dimension> M = M_ ;
            for(unsigned int i=clamp_index; i<Function::dimension; i++) {
                b_[i] = 0.0 ;
                for(unsigned int j=0; j<Function::dimension; j++) {
                    if(i == j) {
                        M(i,j) = 1.0 ;
                    } else {
                        M(i,j) = 0.0 ;
                        M(j,i) = 0.0 ;
                    }
                }
            }
            return Numeric::solve_SPD_system(M, b_, x_.coeff) ;
        }

        void add_sample(double* v, double g, double importance) {
            // Since M is symmetric, we compute only its lower triangle
            for(unsigned int i=0; i<Function::dimension; i++) {
                b_[i] += importance * g * v[i] ;
                for(unsigned int j=i; j<Function::dimension; j++) {
                    M_(i,j) += importance * v[i] * v[j] ;
                }
            }
        }

    private:
        bool M_mode_ ;
        Matrix<double, Function::dimension> M_ ;
        double b_[Function::dimension] ;
        Function x_ ;
        Function reference_ ;
    } ;

    //____________________________________________________________________________________


	template<typename AttributeType>
	void apply_src_permutation(AttributeType& func, unsigned int* perm)
	{
		AttributeType tmp;

		for(unsigned int i = 0 ; i < AttributeType::dimension ; i++)
			tmp[i] = func[perm[i]];

		for(unsigned int i = 0 ; i < AttributeType::dimension ; i++)
			func[i] = tmp[i];
	}	
	
	
	
	template<typename AttributeType>
	void apply_tgt_permutation(AttributeType& func, unsigned int* perm)
	{
		AttributeType tmp;

		for(unsigned int i = 0 ; i < AttributeType::dimension ; i++)
			tmp[perm[i]] = func[i];

		for(unsigned int i = 0 ; i < AttributeType::dimension ; i++)
			func[i] = tmp[i];
	}

}

#endif
