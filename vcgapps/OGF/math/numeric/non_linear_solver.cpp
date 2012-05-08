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

// #define OGF_PARANOID

#include <OGF/math/numeric/non_linear_solver.h>
#include <OGF/math/types/math_library.h>
#include <OGF/basic/os/stopwatch.h>
#include <OGF/basic/debug/progress.h>

// Optimization: if all variables of a stencil are locked, do not
// instanciate stencil (note: we can do that earlier, at
// StencilInstance creation time) -> Rem: this will introduce
// a bias in f(): but we can memorize the constants somewhere
// TODO: add sufficient descent conditions (for the moment,
// we do a 'infinite trust region' algorithm, we trust the quadratic
// model function too much !!!!

namespace OGF {

//__________________________________________________________________________________________________

    Stencil::~Stencil() {
    }
        
    //__________________________________________________________________________________________

    namespace Symbolic {

        Stencil::Stencil(
            const Expression& f
        ) : ::OGF::Stencil(f->max_variable_index() + 1, f->max_parameter_index()+1),
            f_(f), 
            gradient_(nb_variables()), 
            hessian_(nb_variables()) 
        {
            bool ok = true ;
            {
                for(int i=0; i<nb_variables(); i++) {
                    if(!f_->depends_on_variable(i)) {
                        std::cerr << "variable " << i << " not used in stencil" << std::endl ;
                        ok = false ;
                    }
                }
            }
            {
                for(int i=0; i<nb_parameters(); i++) {
                    if(!f_->depends_on_parameter(i)) {
                        std::cerr << "parameter " << i << " not used in stencil" << std::endl ;
                        ok = false ;
                    }
                }
            }
            ogf_assert(ok) ;
            {
                for(int i=0; i<nb_variables(); i++) {
                    gradient_(i) = der(f_, i) ;
                    for(int j=0; j<=i; j++) {
                        hessian_(i,j) = der(gradient_(i),j) ;
                    }
                }
            }
        }

        void Stencil::print(std::ostream& out) {
            out << "f=" << f_ << std::endl ;
            out << std::endl ;
            {
                for(int i=0; i<nb_variables(); i++) {
                    out << "g" << i << "=" << gradient_(i) << std::endl ;
                }
            }
            out << std::endl ;
            {
                for(int i=0; i<nb_variables(); i++) {
                    for(int j=0; j<=i; j++) {
                        out << "G" << i << "," << j << "=" << hessian_(i,j) << std::endl ;
                    }
                }
            }
            out << std::endl ;
        }

        double Stencil::f(const Symbolic::Context& args) {
            return f()->eval(args) ;
        }
        
        double Stencil::g(int i, const Symbolic::Context& args) {
            return g(i)->eval(args) ;
        }
        
        double Stencil::G(int i, int j, const Symbolic::Context& args) {
            return G(i,j)->eval(args) ;
        }
    }

//__________________________________________________________________________________________________
        
    void StencilInstance::bind(Stencil* stencil) {
        ogf_parano_assert(stencil_ == nil) ;
        ogf_parano_assert(global_indices_ == nil) ;
        ogf_parano_assert(parameters_ == nil) ;
        stencil_ = stencil ;
        global_indices_ = new int[stencil_->nb_variables()] ;
        parameters_     = new double[stencil_->nb_parameters()] ;
        {for(int i=0; i<stencil_->nb_variables(); i++) {
            global_indices_[i] = -1 ;
        }}
        {for(int i=0; i<stencil_->nb_parameters(); i++) {
            parameters_[i] = 0 ;
        }}
    }

    bool StencilInstance::is_initialized() {
        if(stencil_ == nil) { return false ; }
        for(int i=0; i<stencil_->nb_variables(); i++) {
            if(global_indices_[i] == -1) {
                std::cerr << "global index:" << i << " : unitialized" << std::endl ;
                return false ;
            }
        }
        return true ;
    }

    StencilInstance::~StencilInstance() { 
        delete[] global_indices_ ; 
        global_indices_ = nil ; 
        delete[] parameters_ ;
        parameters_ = nil ;
        stencil_ = nil ;
    }

//__________________________________________________________________________________________________

    NonLinearSolver::NonLinearSolver(int nb_variables) : Solver(nb_variables) {
        state_ = INITIAL ;
        max_newton_iter_ = 10 ;
        gradient_threshold_ = 1e-3 ;
        steepest_descent_ = false ;
        steepest_descent_factor_ = 1.0 ;
    }

    NonLinearSolver::~NonLinearSolver() {
        for(unsigned int i=0; i<stencil_.size(); i++) {
            delete stencil_[i] ;
        }
    }

    void NonLinearSolver::set_system_solver(const SystemSolverParameters& args) {
        system_solver_ = MathLibrary::instance()->create_system_solver(args) ;
    }
    

    void NonLinearSolver::begin_equation() {
        ogf_assert(state_ == INITIAL) ;
        state_ = IN_EQN ;

        nb_free_variables_ = 0 ;
        nb_locked_variables_ = 0 ;
        int cur_index = 0 ;
        {for(int i=0; i<nb_variables_; i++) {
            if(!variable_[i].is_locked()) {
                set_variable_index(variable_[i],cur_index) ;
                nb_free_variables_++ ;
                cur_index++ ;
            }
        }}
        {for(int i=0; i<nb_variables_; i++) {
            if(variable_[i].is_locked()) {
                set_variable_index(variable_[i],cur_index) ;
                nb_locked_variables_++ ;
                cur_index++ ;
            }
        }}
    }

    void NonLinearSolver::end_equation() {
        ogf_assert(state_ == IN_EQN) ;
        state_ = CONSTRUCTED ;

        dx_.allocate(nb_free_variables_) ;
        b_.allocate(nb_free_variables_) ;
        xc_.allocate(nb_variables_) ;
        {for(int i=0; i<nb_variables_; i++) {
            xc_(variable_[i].index()) = variable_[i].value() ;
        }}

        // Construct internal representation.
        if(system_solver_.is_nil()) {
            system_solver_ = new SystemSolver_CG() ;
        }
        SparseMatrix::Storage storage = SparseMatrix::ROWS ;
        if(system_solver_->needs_columns()) {
            storage = SparseMatrix::ROWS_AND_COLUMNS ;
        }
        bool sym = system_solver_->supports_symmetric_storage() ;
        DirectSystemSolver* ds = dynamic_cast<DirectSystemSolver*>(system_solver()) ;
        if(ds != nil) {
            ds->set_keep_permutation_matrix(true) ;
        }
        G_.allocate(nb_free_variables_, nb_free_variables_, storage, sym) ;
    }

    int NonLinearSolver::declare_stencil(Stencil* S) {
        stencil_.push_back(S) ;
        return (stencil_.size() - 1) ;
    }

    int NonLinearSolver::declare_stencil(const Symbolic::Expression& f) {
        Symbolic::Stencil* S = new Symbolic::Stencil(f) ;
        std::cerr << "Stencil " << stencil_.size() << std::endl ;
        std::cerr << "------------------------" << std::endl ;
        S->print(std::cerr) ;
        return declare_stencil(S) ;
    }

    void NonLinearSolver::begin_stencil_instance(int stencil_id) {
        ogf_assert(state_ == IN_EQN) ;
        state_ = IN_STENCIL_REF ;
        ogf_debug_assert(stencil_id >= 0 && stencil_id < int(stencil_.size())) ;
        cur_stencil_var_ = 0 ;
        cur_stencil_prm_ = 0 ;
        eqn_.push_back(StencilInstance()) ;
        eqn_.back().bind(stencil_[stencil_id]) ;
    }

    void NonLinearSolver::end_stencil_instance() {
        ogf_debug_assert(cur_stencil_instance().is_initialized()) ;
        ogf_assert(state_ == IN_STENCIL_REF) ;
        state_ = IN_EQN ;
    }

    void NonLinearSolver::stencil_variable(int global_variable_id) {
        stencil_variable(cur_stencil_var_, global_variable_id) ;
        cur_stencil_var_++ ;
    }

    void NonLinearSolver::stencil_variable(int local_variable_id, int global_variable_id) {
        ogf_assert(state_ == IN_STENCIL_REF) ;
        ogf_debug_assert(local_variable_id < cur_stencil_instance().nb_variables()) ;
        ogf_debug_assert(global_variable_id >= 0 && global_variable_id < nb_variables_) ;
        int internal_id = variable_[global_variable_id].index() ;
        cur_stencil_instance().global_variable_index(local_variable_id) = internal_id ;
    }

    void NonLinearSolver::stencil_parameter(double value) {
        stencil_parameter(cur_stencil_prm_, value) ;
        cur_stencil_prm_++ ;
    }

    void NonLinearSolver::stencil_parameter(int local_prm_id, double value) {
        ogf_assert(state_ == IN_STENCIL_REF) ;
        ogf_debug_assert(local_prm_id < cur_stencil_instance().nb_parameters()) ;
        cur_stencil_instance().parameter(local_prm_id) = value ;
    }

    void NonLinearSolver::instanciate(
        StencilInstance& RS, bool gradient, bool hessian
    ) {
        Stencil* S = RS.stencil() ;
        int N = RS.nb_variables() ;
        Symbolic::Context args ;
        for(int i=0; i<N; i++) {
            args.variables.push_back(xc_(RS.global_variable_index(i))) ;
        }
        for(int i=0; i<S->nb_parameters(); i++) {
            args.parameters.push_back(RS.parameter(i)) ;
        }
        for(int i=0; i<N; i++) {
            int gi = RS.global_variable_index(i) ;
            if(is_free(gi)) {
                // Compute the coefficients of the gradient.
                if(gradient) {
                    b_(gi) -= S->g(i,args) ;
                }
                // Compute the coefficients of the Hessian.
                if(hessian) {
                    for(int j=0; j<=i; j++) {
                        int gj = RS.global_variable_index(j) ;
                        if(is_free(gj)) {
                            double gij = S->G(i,j,args) ;
                            if(gij != 0.0) {
                                G_.add(gi, gj, gij) ;
                                // G (Hessian) is symmetric, but watch out:
                                // do not add diagonal elements twice !!!
                                if(gi != gj) {
                                    G_.add(gj, gi, gij) ;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    void NonLinearSolver::minimize() {
        ProgressLogger progress(max_newton_iter_) ;
        ogf_assert(state_ == CONSTRUCTED) ;
        for(int k=0; k<max_newton_iter_; k++) {
            solve_one_iteration() ;
            fk_ = f() ;
            gk_ = norm_grad_f() ;
            Logger::out("NonLinearSolver") 
                << "Iter= " << k << " F= " << fk_ << " ||gradF|| = " << gk_ << std::endl ;
            if(gk_ < gradient_threshold_) {
                Logger::out("NonLinearSolver") << "converged" << std::endl ;
                break ;
            }
            if(progress.is_canceled()) {
                Logger::out("NonLinearSolver") << "stopped by user" << std::endl ;
                break ;
            }
            progress.next() ;
            if(progress_ != nil) {
                progress_->notify(k) ;
            }
        }
        update_variables() ;
        state_ = MINIMIZED ;
    }

    void NonLinearSolver::update_variables() {
        for(int i=0; i<nb_variables_; i++) {
            variable_[i].set_value(xc_(variable_[i].index())) ;
        }
    }

    void NonLinearSolver::solve_one_iteration() {

        dx_.zero() ;
        G_.zero() ;
        b_.zero() ;
            
        if(system_solver_->write_logs()) {
            SystemStopwatch w ;
            for(Eqn::iterator it = eqn_.begin(); it != eqn_.end(); it++) {
                instanciate(*it, true, !steepest_descent_) ;
            }
            double sym_time = w.elapsed_user_time() ;
            solve_system() ;
            double num_time = w.elapsed_user_time() - sym_time ;
            Logger::out("NonLinearSolver") 
                << "Times  sym: " << sym_time << " num: " << num_time << std::endl ; 
        } else {
            for(Eqn::iterator it = eqn_.begin(); it != eqn_.end(); it++) {
                instanciate(*it, true, !steepest_descent_) ;
            }
            solve_system() ;
        }
        for(int i=0; i<nb_free_variables_; i++) {
            xc_(i) += dx_(i) ;
        }
    }

    void NonLinearSolver::solve_system() {
        if(steepest_descent_) {
            for(int i=0; i<nb_free_variables_; i++) {
                dx_(i) = steepest_descent_factor_ * b_(i) ;
            }
        } else {
            system_solver_->solve(G_, dx_, b_) ;
        }
    }

    double NonLinearSolver::norm_grad_f(bool recompute_gradient) {
        if(recompute_gradient) {
            b_.zero() ;
            for(Eqn::iterator it = eqn_.begin(); it != eqn_.end(); it++) {
                instanciate(*it, true, false) ;
            }
        }
        double result = 0 ;
        for(int i=0; i<nb_free_variables_; i++) {
            result += (b_(i) * b_(i)) ;
        }
        return ::sqrt(result) ;
    }

    double NonLinearSolver::f() {
        double result = 0 ;
        for(Eqn::iterator it = eqn_.begin(); it != eqn_.end(); it++) {
            StencilInstance& RS = *it ;
            Stencil* S = RS.stencil() ;
            int N = RS.nb_variables() ;
            Symbolic::Context args ;
            for(int i=0; i<N; i++) {
                args.variables.push_back(xc_(RS.global_variable_index(i))) ;
            }
            for(int i=0; i<S->nb_parameters(); i++) {
                args.parameters.push_back(RS.parameter(i)) ;
            }
            result += S->f(args) ;
        }
        return result ;
    }

//__________________________________________________________________________________________________


}
