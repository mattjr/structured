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
 

#ifndef __OGF_TYPES_MATH_LIBRARY__
#define __OGF_TYPES_MATH_LIBRARY__

#include <OGF/math/common/common.h>
#include <OGF/math/linear_algebra/abstract_matrix.h>
#include <OGF/math/linear_algebra/sparse_matrix.h>
#include <OGF/math/numeric/system_solver.h>
#include <OGF/math/numeric/optimizer.h>
#include <OGF/math/numeric/eigen_solver.h>
#include <OGF/basic/os/environment.h>
#include <OGF/basic/types/counted.h>
#include <OGF/basic/types/smart_pointer.h>
#include <OGF/basic/types/basic_factory.h>

#include <string>
#include <map>
#include <stack>

namespace OGF {

//_________________________________________________________

    typedef BasicFactory<SystemSolver> SystemSolverFactory ;
    typedef BasicFactory<EigenSolver> EigenSolverFactory ;

    class MATH_API MathLibrary : public Environment {
    public:
        static MathLibrary* instance() { return instance_ ; }
        static void initialize() ;
        static void terminate() ;
        
        bool bind_system_solver_factory(
            const std::string& name, SystemSolverFactory* factory
        ) ;

        bool bind_eigen_solver_factory(
            const std::string& name, EigenSolverFactory* factory
        ) ;

        SystemSolver* create_system_solver(const SystemSolverParameters& args) ;
        EigenSolver* create_eigen_solver(const std::string& name) ;

        bool solve(const SparseMatrix& A, Vector& x, const Vector& b, const SystemSolverParameters& args) ;
        bool solve(const SparseMatrix& A, Vector& x, const Vector& b, const std::string& solver_name) ;

        virtual bool resolve(const std::string& name, std::string& value) const ;

        bool invert_matrix(
            const SparseMatrix& M, AbstractMatrix& M_inv, const std::string& method, int* permutation = nil
        ) ;

        bool invert_matrix(
            const SparseMatrix& M, AbstractMatrix& M_inv, const std::string& method, Array1d<int>& permutation
        ) ;

        Optimizer* create_optimizer(const std::string& name) ;

    protected:
        MathLibrary() ;
        ~MathLibrary() ;
        friend class World ;
        
    private:
        static MathLibrary* instance_ ;
        BasicFactories<SystemSolver> system_solvers_ ;
        BasicFactories<EigenSolver> eigen_solvers_ ;
    } ;
    
//_________________________________________________________
    
    template <class T> class ogf_declare_system_solver {
    public:
        ogf_declare_system_solver(const std::string& name) {
            bool ok = OGF::MathLibrary::instance()->bind_system_solver_factory(
                name, new GenericBasicFactory<SystemSolver,T>
            ) ;
            ogf_assert(ok) ;
        }
    } ;

//_________________________________________________________

    template <class T> class ogf_declare_eigen_solver {
    public:
        ogf_declare_eigen_solver(const std::string& name) {
            bool ok = OGF::MathLibrary::instance()->bind_eigen_solver_factory(
                name, new GenericBasicFactory<EigenSolver,T>
            ) ;
            ogf_assert(ok) ;
        }
    } ;

//_________________________________________________________    

}
#endif

