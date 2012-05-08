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

#include <OGF/math/numeric/system_solver.h>
#include <OGF/math/numeric/preconditioner.h>
#include <OGF/math/numeric/conjugate_gradient.h>
#include <OGF/math/numeric/bicgstab.h>
#include <OGF/math/numeric/gmres.h>
#include <OGF/math/numeric/super_lu.h>
#include <OGF/math/linear_algebra/sparse_matrix.h>
#include <OGF/basic/os/stopwatch.h>
#include <OGF/basic/debug/assert.h>

namespace OGF {

    SystemSolverParameters::SystemSolverParameters() {
		restore_defaults();
	}
	
	void SystemSolverParameters::restore_defaults(){
		clear();		
		create_arg("method", std::string("DEFAULT")) ;
        create_arg("nb_iters", 0) ;
        create_arg("threshold", std::string("1e-6")) ;
	}

    std::ostream& operator<<(std::ostream& out, const SystemSolverParameters& params ) {
        for(int i=0; i<params.nb_args(); i++) {
            if(i!=0) {
                out << ";" ;
            }
            out << params.ith_arg(i)->name() << ";" << params.ith_arg(i)->string_value() ;
        }
        return out ;
    }

    std::istream& operator>>(
        std::istream& in, SystemSolverParameters& params
    ) {
        params.clear() ;
        std::string fields ;
        in >> fields ;

        if(fields == "defaults") {
            params.restore_defaults() ;
            return in ;
        }

        std::vector<std::string> field ;
        // Last arg of split_string: do not skip empty fields
        String::split_string(fields,';', field,false) ;
        unsigned int cur=0; 
        while(cur < field.size()) {
            std::string name = field[cur] ; cur++ ;
            std::string value ;
            if(cur < field.size()) {
                value = field[cur] ; cur++ ;
            }
            if(name.length() > 0) {
                params.create_arg(name,value) ;
            }
        }
        return in ;
    }

    SystemSolver::SystemSolver() {
        progress_ = nil ;
        write_logs_ = false ;
        quiet_ = false ;
    }

    void SystemSolver::set_parameters(const SystemSolverParameters& args) {
		write_logs_ = (args.arg_string_value("write_logs") == "true") ;
		
		/*for(int i=0; i<args.nb_args(); i++) {
            const Arg* arg = args.ith_arg(i) ;
            if(arg->name() == "write_logs") {
                write_logs_ = (arg->value() == "true") ;
            }
        }*/
    }

    bool SystemSolver::needs_rows() const {
        return true ;
    }
    
    bool SystemSolver::needs_columns() const {
        return false ;
    }
    
    bool SystemSolver::supports_symmetric_storage() const {
        return false ;
    }

    //_______________________________________________________________________________________________

    IterativeSystemSolver::IterativeSystemSolver() {
        nb_iters_ = 0 ;
        threshold_ = 1e-6 ;
        preconditioner_ = PRECOND_DEFAULT ;
        omega_ = 1.5 ;
    }

    void IterativeSystemSolver::set_parameters(const SystemSolverParameters& args) {
        for(int i=0; i<args.nb_args(); i++) {
            const ArgBase* arg = args.ith_arg(i) ;
            if(arg->name() == "nb_iters") {
				nb_iters_ = arg->int_value() ;
            } else if(arg->name() == "threshold") {
                threshold_ = arg->double_value() ;
            } else if(arg->name() == "preconditioner") {
                if(arg->string_value() == "NONE") {
                    preconditioner_ = PRECOND_NONE ;
                } else if(arg->string_value() == "DEFAULT") {
                    preconditioner_ = PRECOND_DEFAULT ;
                } else if(arg->string_value() == "JACOBI") {
                    preconditioner_ = PRECOND_JACOBI ;
                } else if(arg->string_value() == "SSOR") {
                    preconditioner_ = PRECOND_SSOR ;
                } else {
                    Logger::err("SystemSolver") 
                        << arg->string_value() << ": no such preconditioner" << std::endl ;
                    preconditioner_ = PRECOND_DEFAULT ;
                }
            } else if(arg->name() == "omega") {
                omega_ = arg->double_value() ;
            } 
        }
        SystemSolver::set_parameters(args) ;
    }

    bool IterativeSystemSolver::needs_rows() const {
        return true ;
    }
    
    bool IterativeSystemSolver::needs_columns() const {
        return (preconditioner_ == PRECOND_SSOR) ;
    }
    
    bool IterativeSystemSolver::supports_symmetric_storage() const {
        return true ;
    }

    //_______________________________________________________________________________________________

    bool SystemSolver_CG::solve(const SparseMatrix& A, Vector& x, const Vector& b) {
        int n = x.size() ;
        if(n == 0) { return true ; }
        int nb_iters = (nb_iters_ != 0) ? nb_iters_ : n*5 ;
        int retval = -1 ;
        switch(preconditioner_) {
        default:
            Logger::warn("SparseSystemSolver") 
                << " CG: invalid preconditioner, falling-back to default (JACOBI)"
                << std::endl ;
        case PRECOND_DEFAULT:
        case PRECOND_JACOBI: {
            Jacobi_Preconditioner M(A, omega_) ;
            retval = solve_preconditioned_conjugate_gradient(
                n, A, M, b.data(), x.data(), threshold_, nb_iters, 
                write_logs_, progress_, "jacobicg", quiet_
            ) ;
            if(Numeric::has_nan(x)) {
                Logger::out("Solver")
                    << "Jacobi preconditoner failed, caused nan"
                    << std::endl ;
                Logger::out("Solver")
                    << "Relaunching with regular conjugate gradient"
                    << std::endl ;
                retval = solve_conjugate_gradient(
                    n, A, b.data(), x.data(), threshold_, nb_iters, 
                    write_logs_, progress_, "cg", quiet_
                ) ;
            }
        } break ;
        case PRECOND_SSOR: {
            SSOR_Preconditioner M(A, omega_) ;
            retval = solve_preconditioned_conjugate_gradient(
                n, A, M, b.data(), x.data(), threshold_, nb_iters, 
                write_logs_, progress_, "ssorcg", quiet_
            ) ;
            if(Numeric::has_nan(x)) {
                Logger::out("Solver")
                    << "Jacobi preconditoner failed, caused nan"
                    << std::endl ;
                Logger::out("Solver")
                    << "Relaunching with regular conjugate gradient"
                    << std::endl ;
                retval = solve_conjugate_gradient(
                    n, A, b.data(), x.data(), threshold_, nb_iters, 
                    write_logs_, progress_, "cg", quiet_
                ) ;
            }
        } break ;
        case PRECOND_NONE: {
            retval = solve_conjugate_gradient(
                n, A, b.data(), x.data(), threshold_, nb_iters, 
                write_logs_, progress_, "cg", quiet_
            ) ;
        } break ;
        } ;
        return true ;
    }

    bool SystemSolver_BICGSTAB::solve(const SparseMatrix& A, Vector& x, const Vector& b) {
        int n = x.size() ;
        if(n == 0) { return true ; }
        int nb_iters = (nb_iters_ != 0) ? nb_iters_ : n*5 ;
        int retval = -1 ;
        switch(preconditioner_) {
        default:
            Logger::warn("SparseSystemSolver") 
                << " BICGSTAB: invalid preconditioner, falling-back to default (JACOBI)"
                << std::endl ;
        case PRECOND_DEFAULT:
        case PRECOND_JACOBI: {
            Jacobi_Preconditioner M(A, omega_) ;
            retval = solve_preconditioned_bicgstab(
                n, A, M, b.data(), x.data(), threshold_, nb_iters, 
                write_logs_, progress_, "jacobi_bicgstab", quiet_
            ) ;
            if(Numeric::has_nan(x)) {
                Logger::out("Solver")
                    << "Jacobi preconditoner failed, caused nan"
                    << std::endl ;
                Logger::out("Solver")
                    << "Relaunching with regular bicgstab"
                    << std::endl ;
                retval = solve_bicgstab(
                    n, A, b.data(), x.data(), threshold_, nb_iters, 
                    write_logs_, progress_, "bicgstab", quiet_
                ) ;
            }
        } break ;
        case PRECOND_NONE: {
            retval = solve_bicgstab(
                n, A, b.data(), x.data(), threshold_, nb_iters, 
                write_logs_, progress_, "bicgstab", quiet_
            ) ;
        } break ;
        }
        return true ;
    }

    bool SystemSolver_GMRES::solve(const SparseMatrix& A, Vector& x, const Vector& b) {
        int n = x.size() ;
        if(n == 0) { return true ; }
        int nb_iters = (nb_iters_ != 0) ? nb_iters_ : n*5 ;
        if(preconditioner_ != PRECOND_NONE) {
            Logger::warn("SparseSystemSolver") 
                << "GMRES: preconditioner not implemented yet"
                << std::endl ;
        }
        solve_gmres(
            nb_inner_iters_, n, A, b.data(), x.data(), threshold_, nb_iters, 
            write_logs_, progress_, "gmres", quiet_
        ) ;
        return true ;
    }

    void SystemSolver_GMRES::set_parameters(const SystemSolverParameters& args) {
        for(int i=0; i<args.nb_args(); i++) {
            const ArgBase* arg = args.ith_arg(i) ;
            if(arg->name() == "nb_inner_iters") {
                nb_inner_iters_ = arg->int_value() ;
            }
        }
        IterativeSystemSolver::set_parameters(args) ;
    }

    //_______________________________________________________________________________________________

    DirectSystemSolver::DirectSystemSolver() {
        keep_permutation_matrix_ = false ;
        ordering_ = ORDER_COLAMD ;
    }
    
    void DirectSystemSolver::set_parameters(const SystemSolverParameters& args) {
        for(int i=0; i<args.nb_args(); i++) {
            const ArgBase* arg = args.ith_arg(i) ;
			if(arg->name() == "ordering") {
                std::string arg_value = arg->string_value();
				if(arg_value == "NONE") {
                    ordering_ = ORDER_NONE ;
                } else if(arg_value == "COLAMD") {
                    ordering_ = ORDER_COLAMD ;
                } else if(arg_value == "MMD_AT_PLUS_A") {
                    ordering_ = ORDER_MMD_AT_PLUS_A ;
                } else if(arg_value == "DEFAULT") {
                    ordering_ = ORDER_DEFAULT ;
                } else {
                    Logger::err("SystemSolver") << arg_value << ": no such ordering" << std::endl ;
                    ordering_ = ORDER_COLAMD ;
                }
            }
        }
        SystemSolver::set_parameters(args) ;
    }

    void DirectSystemSolver::set_permutation_matrix(const Array1d<int>& perm) {
        perm_.allocate(perm.size()) ;
        for(unsigned int i=0; i<perm.size(); i++) {
            perm_(i) = perm(i) ;
        }
    }

    //_______________________________________________________________________________________________


    bool SystemSolver_SUPERLU::supports_symmetric_storage() const {
        return false ;
    }

    bool SystemSolver_SUPERLU::solve(const SparseMatrix& A_in, Vector& x, const Vector& b) {
        int n = x.size() ;
        if(n == 0) { return true ; }
        SparseMatrix& A = const_cast<SparseMatrix&>(A_in) ;
        bool retval = false ;
        switch(ordering_) {
        default:
            Logger::warn("SparseSystemSolver") 
                << " SUPERLU: invalid pre-ordering, falling-back to default (COLAMD)"
                << std::endl ;
        case ORDER_DEFAULT:
        case ORDER_COLAMD: {
            retval = solve_super_lu(A, b.data(), x.data(), perm_, true, false) ;
        } break ;
        case ORDER_MMD_AT_PLUS_A: {
            retval = solve_super_lu(A, b.data(), x.data(), perm_, false, true) ;
        } break ;
        case ORDER_NONE: {
            retval = solve_super_lu(A, b.data(), x.data(), false) ;
        } break ;
        }
        return retval ;
    }
}
