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
 
#ifndef __OGF_MATH_NUMERIC_NL_SOLVER__
#define __OGF_MATH_NUMERIC_NL_SOLVER__

#include <OGF/math/common/common.h>
#include <OGF/math/numeric/solver.h>
#include <OGF/math/numeric/system_solver.h>
#include <OGF/math/symbolic/symbolic.h>
#include <OGF/math/linear_algebra/vector.h>
#include <OGF/math/linear_algebra/sparse_matrix.h>
#include <OGF/basic/containers/arrays.h>
#include <OGF/basic/containers/triangular_array.h>

#include <deque>
#include <iostream>

namespace OGF {

    /**
     * Abstract class for stencils.
     */
    class MATH_API Stencil {
    public:
        Stencil(
            int nb_variables, int nb_parameters
        ) : nb_variables_(nb_variables), nb_parameters_(nb_parameters) {
        }
        virtual ~Stencil() ;
        int nb_variables() const  { return nb_variables_ ;  }
        int nb_parameters() const { return nb_parameters_ ; }
        virtual double f(const Symbolic::Context& args) = 0 ;
        virtual double g(int i, const Symbolic::Context& args) = 0 ;
        virtual double G(int i, int j, const Symbolic::Context& args) = 0 ;
    private:
        int nb_variables_ ;
        int nb_parameters_ ;
    } ;
    

    namespace Symbolic {
        
        typedef Array1d<Expression> Gradient ;
        typedef TriangularArray<Expression> Hessian ;

        /**
         * A Stencil doing formal derivation to compute
         * the gradient and Hessian. 
         */
        class MATH_API Stencil : public ::OGF::Stencil {
        public:
            Stencil(const Expression& f) ;
            
            virtual double f(const Context& args) ;
            virtual double g(int i, const Context& args) ;
            virtual double G(int i, int j, const Context& args) ;

            Expression& f() { return f_ ; }
            Gradient& g() { return gradient_ ; }
            Expression& g(int i) { return gradient_(i) ; }
            Hessian& G() { return hessian_ ; }
            Expression& G(int i, int j) { return hessian_(ogf_max(i,j),ogf_min(i,j)) ; }
            void print(std::ostream& out) ;
        private:
            Expression f_ ;
            Gradient gradient_ ;
            Hessian  hessian_ ;
        } ;
    }

    //________________________________________________________________________________________
    
    class MATH_API StencilInstance {
    public:
        StencilInstance() : stencil_(nil), global_indices_(nil), parameters_(nil) { }
        StencilInstance(const StencilInstance& rhs) : 
            stencil_(nil), global_indices_(nil), parameters_(nil) { 
                ogf_parano_assert(rhs.stencil_ == nil) ;
                ogf_parano_assert(rhs.global_indices_ == nil) ;
                ogf_parano_assert(rhs.parameters_ == nil) ;
            }
        void bind(Stencil* stencil) ;
        ~StencilInstance() ;

        int nb_variables() const { return stencil_->nb_variables() ; }
        int nb_parameters() const { return stencil_->nb_parameters() ; }
        int& global_variable_index(int i) { 
            ogf_debug_assert(i >= 0 && i < stencil_->nb_variables()) ; 
            return global_indices_[i] ; 
        }
        double& parameter(int i) {
            ogf_debug_assert(i >= 0 && i < stencil_->nb_parameters()) ; 
            return parameters_[i] ;
        }
        Stencil* stencil() { return stencil_ ; }
        bool is_initialized() ;

    private:
        Stencil* stencil_ ;
        int* global_indices_ ;
        double* parameters_ ;
    private:
        StencilInstance& operator=(const StencilInstance& rhs) ;
    } ;

    //________________________________________________________________________________________

    class MATH_API NonLinearSolver : public Solver {
    public:
        NonLinearSolver(int nb_variables) ;
        virtual ~NonLinearSolver() ;
        
        // __________________ Construction _____________________
        
        void begin_equation() ;

        /** 
         * declares a stencil from its expression. If more performance is
         * needed, use declare_stencil(Stencil* S).
         */
        int declare_stencil(const Symbolic::Expression& f) ;

        /** 
         * declares a specialized stencil. Client code needs to derive
         * a Stencil class, and implement evaluation, gradient and Hessian.
         * If performance is not critical, use 
         * declare_stencil(const Expression& f) instead.
         */
        int declare_stencil(Stencil* S) ;

        
        void begin_stencil_instance(int stencil_id) ;
        void stencil_variable(int global_variable_id) ;
        void stencil_variable(int local_variable_id, int global_variable_id) ;
        void stencil_parameter(double value) ;
        void stencil_parameter(int local_parameter_id, double value) ;
        void end_stencil_instance() ;

        void end_equation() ;
        void minimize() ;
            
        // __________________ Access ____________________________

        Stencil* stencil(int id) { 
            ogf_assert(id >= 0 && id < int(stencil_.size())) ;
            return stencil_[id] ;  
        }

        virtual void update_variables() ;

        // ________________ Solver tuning _______________________

        void set_system_solver(const SystemSolverParameters& args) ;
        void set_system_solver(SystemSolver* solver) { system_solver_ = solver ; }
        SystemSolver* system_solver() { return system_solver_ ; }
        const SystemSolver* system_solver() const { return system_solver_ ; }        

        void set_max_newton_iter(int x)                 { max_newton_iter_ = x ;    }
        void set_gradient_threshold(double x)           { gradient_threshold_ = x ; }
        void set_steepest_descent(bool x)               { steepest_descent_ = x ;   }
        void set_steepest_descent_factor(double x)      { steepest_descent_factor_ = x ;  }

    protected:
        StencilInstance& cur_stencil_instance() {
            return *(eqn_.rbegin()) ;
        }

        void instanciate(
            StencilInstance& RS, bool gradient, bool hessian 
        ) ;

        bool is_free(int id)   { return (id < nb_free_variables_) ;  }
        bool is_locked(int id) { return (id >= nb_free_variables_) ; }

        double norm_grad_f(bool recompute_gradient = false) ;
        double f() ;

        virtual void solve_one_iteration() ;
        virtual void solve_system() ;

    private:
        // User representation
        int nb_free_variables_ ;
        int nb_locked_variables_ ;
        std::vector<Stencil*> stencil_ ;

        // State
        enum State {INITIAL, IN_EQN, IN_STENCIL_REF, CONSTRUCTED, MINIMIZED} ;
        State state_ ;
        int cur_stencil_var_ ;
        int cur_stencil_prm_ ;

        // Internal representation: problem setting
        typedef std::deque<StencilInstance> Eqn ;
        Eqn eqn_ ; // Equation of the problem

        // Internal representation: current iteration
        SystemSolver_var system_solver_ ;
        Vector dx_ ;             // Unknown delta vector for the variables
        Vector b_ ;               // -gradient
        SparseMatrix G_ ;    // Hessian 
        Vector xc_ ;              // Variables + constants
        double fk_ ;             // value of the function at current step
        double gk_ ;            // norm of the gradient at current step
        
        // Tuning
        int max_newton_iter_ ;
        double gradient_threshold_ ;

        bool steepest_descent_ ;
        double steepest_descent_factor_ ;
    } ;

    //________________________________________________________________________________________

}

#endif
