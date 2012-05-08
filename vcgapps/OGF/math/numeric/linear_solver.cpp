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

#include <OGF/math/numeric/linear_solver.h>
#include <OGF/math/numeric/system_solver_fastcg.h>
#include <OGF/math/types/math_library.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/debug/progress.h>
#include <OGF/basic/os/stopwatch.h>

#include <fstream>

namespace OGF {

//_________________________________________________________

    LinearSolver::LinearSolver(int nb_variables) : Solver(nb_variables) {
        state_ = INITIAL ;
        least_squares_ = true ;
        symmetric_ = true ;
        update_ = false ;
        invert_matrix_ = false ;
    }
   
    LinearSolver::~LinearSolver() {
    }

    //__________________________________________________________________

    void LinearSolver::set_system_solver(const SystemSolverParameters& args) {
        system_solver_ = MathLibrary::instance()->create_system_solver(args) ;
    }

    bool LinearSolver::solve() {
        ok_ = true ;
        system_solver_->set_progress(progress_) ;
        check_state(CONSTRUCTED) ;
        if(invert_matrix_) {
            if(update_) {
                M_inv_.mult(x_.data(),b_.data()) ;
            } else {
                MathLibrary::instance()->invert_matrix(G_, M_inv_, "SuperLU") ;
                M_inv_.mult(x_.data(),b_.data()) ;
            }
        } else {
            SystemStopwatch w ;
            IterativeSystemSolver* is = dynamic_cast<IterativeSystemSolver*>(system_solver()) ;
            if(is != nil) {
                int nb_iter = is->nb_iters() ;
                if(nb_iter == 0) {
                    nb_iter = G_.n() * 5 ;
                }
                if(!quiet_) {
                    Logger::out("LinearSolver") 
                        << "Solving ... - dim=" << G_.n()
                        << " - nb_iters="          << nb_iter
                        << std::endl ;
                }
            } else {
                if(!quiet_) {
                    Logger::out("LinearSolver") 
                        << "Solving ... - dim=" << G_.n()
                        << std::endl ;
                }
            }
            ok_ = system_solver_->solve(G_, x_, b_) ; 
            if(!quiet_) {
                Logger::out("LinearSolver") << "Solved - elapsed time = " 
                                            << w.elapsed_user_time() 
                                            << std::endl ;
            }
        }
        vector_to_variables(x_) ;
        transition(CONSTRUCTED, SOLVED) ;
        return ok_ ;
    }

    void LinearSolver::restart() {
        transition(SOLVED, INITIAL) ;
        update_ = true ;
    }

    // _______________________________________________________________
    

    void LinearSolver::begin_system() {
        current_row_ = 0 ;
        transition(INITIAL, IN_SYSTEM) ;
        if(!update_) {
            // Enumerate free variables.
            int index = 0 ;
            for(int ii=0; ii < nb_variables() ; ii++) {
                SolverVariable& v = variable(ii) ;
                if(!v.is_locked()) {
                    set_variable_index(v,index) ;
                    index++ ;
                }
            }

            // Size of the system to solve.
            int n = index ;

            // Construct internal representation.
            if(system_solver_.is_nil()) {
                if(symmetric_) {
                    system_solver_ = new SystemSolver_FASTCG() ;
                } else {
                    system_solver_ = new SystemSolver_BICGSTAB() ;
                }
            }

            SparseMatrix::Storage storage = SparseMatrix::ROWS ;
            if(system_solver_->needs_columns()) {
                storage = SparseMatrix::ROWS_AND_COLUMNS ;
            }
            bool sym = system_solver_->supports_symmetric_storage() && symmetric() ;
            x_.allocate(n) ;
            b_.allocate(n) ;
            G_.allocate(n,n, storage, sym) ;
        }
        x_.zero() ;
        b_.zero() ;
        variables_to_vector(x_) ;
    }
    
    void LinearSolver::end_system() {
        transition(IN_SYSTEM, CONSTRUCTED) ;
    }
    
    void LinearSolver::begin_row() {
        transition(IN_SYSTEM, IN_ROW) ;
        af_.clear() ;
        if_.clear() ;
        al_.clear() ;
        xl_.clear() ;
        bk_ = 0.0 ;
        negative_row_scaling_ = false ;
    }
    
    void LinearSolver::set_right_hand_side(double b) {
        check_state(IN_ROW) ;
        bk_ = b ;
    }
    
    void LinearSolver::add_coefficient(int iv, double a) {
        check_state(IN_ROW) ;
        SolverVariable& v = variable(iv) ;
        if(v.is_locked()) {
            al_.push_back(a) ;
            xl_.push_back(v.value()) ;
        } else {
            af_.push_back(a) ;
            if_.push_back(v.index()) ;
        }
    }
    
    void LinearSolver::normalize_row(double weight) {
        check_state(IN_ROW) ;
        double norm = 0.0 ;
        int nf = af_.size() ;
        { for(int i=0; i<nf; i++) {
                norm += af_[i] * af_[i] ;
            }}
        int nl = al_.size() ;
        { for(int i=0; i<nl; i++) {
                norm += al_[i] * al_[i] ;
            }}
        norm = ::sqrt(norm) ;
        scale_row(weight / norm) ;
    }
    
    void LinearSolver::scale_row(double s) {
        check_state(IN_ROW) ;
        negative_row_scaling_ = (s < 0) ;
        s = ::fabs(s) ;
        int nf = af_.size() ;
        { for(int i=0; i<nf; i++) {
                af_[i] *= s ;
            }}
        int nl = al_.size() ;
        { for(int i=0; i<nl; i++) {
                al_[i] *= s ;
            }}
        bk_ *= s ; 
    }
    
    void LinearSolver::end_row() {
        
        if(least_squares()) {
            int nf = af_.size() ;
            int nl = al_.size() ;
                
            if(!update_) { 
                for(int i=0; i<nf; i++) {
                    for(int j=0; j<nf; j++) {
                        if (!negative_row_scaling_) {
                            G_.add(if_[i], if_[j], af_[i] * af_[j]) ;
                        } else {
                            G_.add(if_[i], if_[j], -af_[i] * af_[j]) ;
                        }
                    }
                }
            }

            double S = - bk_ ;
                
            { for(int j=0; j<nl; j++) {
                S += al_[j] * xl_[j] ;
            }}
                
            { for(int i=0; i<nf; i++) {
                if (!negative_row_scaling_) {
                    b_(if_[i]) -= af_[i] * S ;
                } else {
                    b_(if_[i]) += af_[i] * S ;
                }
            }}
        } else {
            int nf = af_.size() ;
            int nl = al_.size() ;
            if(!update_) { 
                for(int i=0; i<nf; i++) {
                    G_.add(current_row_, if_[i], af_[i]) ;
                }
            }                
            b_(current_row_) = bk_ ;
            { for(int i=0; i<nl; i++) {
                 b_(current_row_) -= al_[i] * xl_[i] ;
            }}
        }
        current_row_++ ;
        transition(IN_ROW, IN_SYSTEM) ; 
    }

    // _______________________________________________________________

    void LinearSolver::vector_to_variables(const Vector& x) {
        for(int ii=0; ii < nb_variables(); ii++) {
            SolverVariable& v = variable(ii) ;
            if(!v.is_locked()) {
                v.set_value(x(v.index())) ;
            }
        }
    }

    void LinearSolver::variables_to_vector(Vector& x) {
        for(int ii=0; ii < nb_variables(); ii++) {
            SolverVariable& v = variable(ii) ;
            if(!v.is_locked()) {
                x(v.index()) = v.value() ;
            }
        }
    }


    // _______________________________________________________________
        
    std::string LinearSolver::state_to_string(State s) {
        switch(s) {
        case INITIAL:
            return "initial" ;
        case IN_SYSTEM:
            return "in system" ;
        case IN_ROW: 
            return "in row" ;
        case CONSTRUCTED: 
            return "constructed" ;
        case SOLVED:
            return "solved" ;
        } 
        // Should not go there.
        ogf_assert(false) ;
        return "undefined" ;
    }
        
    void LinearSolver::check_state(State s) {
        if(state_ != s) {
            Logger::err("Solver") << "Wrong state, expected: "
                                  << state_to_string(s)
                                  << " got:"
                                  << state_to_string(state_)
                                  << std::endl ;
            Logger::err("Solver") << "exiting ..." << std::endl ;
        }
        ogf_assert(state_ == s) ;
    }
        
    void LinearSolver::transition(State from, State to) {
        check_state(from) ;
        state_ = to ;
    }

    void LinearSolver::update_variables() {
        vector_to_variables(x_) ;
    }

    void LinearSolver::set_progress(Progress* progress) {
        Solver::set_progress(progress) ;
    }

    void LinearSolver::set_quiet(bool x) {
        system_solver_->set_quiet(x) ;
        Solver::set_quiet(x) ;
    }

//_______________________________________________________________________


}

