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
 
#include <OGF/math/types/math_library.h>
#include <OGF/basic/debug/logger.h>
#include <OGF/math/numeric/super_lu.h>
#include <OGF/math/numeric/system_solver_taucs.h>
#include <OGF/math/numeric/lbfgs_optimizers.h>

namespace OGF {

//_________________________________________________________

    MathLibrary* MathLibrary::instance_ = nil ;

    void MathLibrary::initialize() {
        ogf_assert(instance_ == nil) ;
        instance_ = new MathLibrary() ;
        Environment::instance()->add_environment(instance_) ;
    }

    void MathLibrary::terminate() {
        ogf_assert(instance_ != nil) ;
        delete instance_ ;
        instance_ = nil ;
    }

    MathLibrary::MathLibrary() {   }
    MathLibrary::~MathLibrary() {   }

    bool MathLibrary::bind_system_solver_factory(
        const std::string& name, SystemSolverFactory* factory
    ) {
        if(system_solvers_.factory_is_bound(name)) {
            return false ;
        }
        system_solvers_.register_factory(name, factory) ;
        Environment::notify_observers("system_solvers") ;
        return true ;
    }

    bool MathLibrary::bind_eigen_solver_factory(
        const std::string& name, EigenSolverFactory* factory
    ) {
        if(eigen_solvers_.factory_is_bound(name)) {
            return false ;
        }
        eigen_solvers_.register_factory(name, factory) ;
        Environment::notify_observers("eigen_solvers") ;
        return true ;
    }


    SystemSolver* MathLibrary::create_system_solver(const SystemSolverParameters& parameters) {
        ogf_assert(parameters.has_arg("method")) ;
        std::string name = parameters.arg_string_value("method") ;
        if(name == "DEFAULT") {  return nil ;  }  // Let the solver decide which is best.
        if(!system_solvers_.factory_is_bound(name)) {
            Logger::err("MathLibrary") << name << ": no such SystemSolver" << std::endl ;
            return nil ;
        }
        SystemSolver* result = system_solvers_.create(name) ;
        result->set_parameters(parameters) ;
        return result ;
    }

    bool MathLibrary::solve(const SparseMatrix& A, Vector& x, const Vector& b, const SystemSolverParameters& args) {
        SystemSolver_var solver = create_system_solver(args) ;
        if(solver.is_nil()) { return false ; }
        return solver->solve(A,x,b) ;
    }
    
    bool MathLibrary::solve(const SparseMatrix& A, Vector& x, const Vector& b, const std::string& solver_name) {
        SystemSolverParameters args ;
        args.create_arg("method", solver_name) ;
        return solve(A,x,b,args) ;
    }

    EigenSolver* MathLibrary::create_eigen_solver(const std::string& name) {
        if(!eigen_solvers_.factory_is_bound(name)) {
            Logger::err("MathLibrary") << name << ": no such EigenSolver" << std::endl ;
            return nil ;
        }
        EigenSolver* result = eigen_solvers_.create(name) ;
        return result ;
    }
    


//_________________________________________________________

    bool MathLibrary::invert_matrix(
        const SparseMatrix& M, AbstractMatrix& M_inv, const std::string& method, int* permutation 
    ) {

        if(method == "SuperLU") {
            return invert_super_lu(M, M_inv, permutation) ;
        } 

        if(method == "TAUCS_ldlt"){
            return invert_matrix_TAUCS(M, M_inv, permutation, OGF_TAUCS_LDLT) ;
        }

        if(method == "TAUCS_OOC_ldlt"){
            return invert_matrix_TAUCS(M, M_inv, permutation, OGF_TAUCS_OOC_LDLT) ;
        }
       
       
        Logger::err("MathLibrary") << method << " : no such direct solver" << std::endl ;
        return false ;
    }
    

    bool MathLibrary::invert_matrix(
        const SparseMatrix& M, AbstractMatrix& M_inv, const std::string& method, Array1d<int>& permutation
    ) {

        permutation.clear() ;

        if(permutation.size() != 0) {
            return invert_matrix(M, M_inv, method, permutation.data()) ;
        }

        if(!invert_matrix(M, M_inv, method)) {
            return false ;
        }

        // Note: TAUCS seems to use a sentry in the permutation vector
        // (or maybe fortran 1-base indexing), so we need n+1 entries in 
        // perm.
        int perm_size = M.n() ;
        if(method == "TAUCS_ldlt") { perm_size++ ; }

        permutation.allocate(perm_size) ;
        int* perm = M_inv.permutation() ;
        for(int i=0; i<=perm_size; i++) {
            permutation[i] = perm[i] ;
        }
		return true;
    }


    Optimizer* MathLibrary::create_optimizer(const std::string& name) {
        if(name == "LBFGSB") {
            return new LBFGSBOptimizer ;
        } else if(name == "HLBFGS") {
            return new HLBFGSOptimizer ;
        } else if(name == "HM1QN3") {
            HLBFGSOptimizer* opt = new HLBFGSOptimizer ;
            opt->set_m1qn3(true) ;
            return opt ;
        } else if(name == "HCG") {
            HLBFGSOptimizer* opt = new HLBFGSOptimizer ;
            opt->set_cg(true) ;
            return opt ;
        } else if(name == "HLBFGS_HESS") {
            return new HLBFGS_HessOptimizer ;
        } else {
            Logger::err("MathLibrary") << name << ": no such optimizer" << std::endl ;
            return nil ;
        }
    }

//--------------------------------------------------------------------------------------

    bool MathLibrary::resolve(
        const std::string& name, std::string& value
    ) const {
        if(name=="system_solvers") {
            value = "DEFAULT;" + system_solvers_.factory_names() ;
            return true ;
        } else if(name == "eigen_solvers") { 
            value = eigen_solvers_.factory_names() ;
            return true ;
        } else if(name == "direct_solvers") {
	    value = "SuperLU;TAUCS_ldlt;TAUCS_OOC_ldlt" ;
	    return true ;
        } else if(name == "optimizers") {
            value = "LBFGSB;HLBFGS;HM1QN3;HCG;HLBFGS_HESS" ;
            return true ;
        } else {
            return false ;
        }
    }

}

