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
 

#ifndef __OGF_MATH_NUMERIC_SYSTEM_SOLVER__
#define __OGF_MATH_NUMERIC_SYSTEM_SOLVER__

#include <OGF/math/common/common.h>
#include <OGF/math/linear_algebra/sparse_matrix.h>
#include <OGF/basic/containers/arrays.h>
#include <OGF/basic/types/smart_pointer.h>
#include <OGF/basic/types/counted.h>
#include <OGF/basic/types/arg.h>


#include <map>

namespace OGF {

    class Progress ;
    class SparseMatrix ;

    class MATH_API SystemSolverParameters : public ArgList {  
    public:
        SystemSolverParameters() ;
		void restore_defaults();
    } ;

    MATH_API std::ostream& operator<<( std::ostream& out, const SystemSolverParameters& params ) ;
    MATH_API std::istream& operator>>( std::istream& in, SystemSolverParameters& params  ) ;

    class MATH_API SystemSolver : public Counted {
    public:
        SystemSolver() ;
        virtual bool solve(const SparseMatrix& A, Vector& x, const Vector& b) = 0 ;
        virtual void set_parameters(const SystemSolverParameters& args) ;
        void set_progress(Progress* p) { progress_ = p ; }
        Progress* progress() const { return progress_ ; }
        virtual bool needs_rows() const ;                             // default returns true
        virtual bool needs_columns() const ;                       // default returns false
        virtual bool supports_symmetric_storage() const ; // default returns false
        bool write_logs() const { return write_logs_ ; }
        void set_write_logs(bool x) { write_logs_ = x ; }
        void set_quiet(bool x) { quiet_ = x ; }
    protected:
        bool write_logs_ ;
        bool quiet_ ;
        Progress* progress_ ;
    } ;

    typedef SmartPointer<SystemSolver> SystemSolver_var ;

    //-----------------------------------------------------------------------------------------------------------------

    enum SystemSolverPreconditioner {
        PRECOND_NONE, 
        PRECOND_DEFAULT,       // Use "best" preconditioner for selected method
        PRECOND_JACOBI,        // For iterative methods (CG, BICGSTAB, GMRES)
        PRECOND_SSOR,          // For iterative methods (CG, BICGSTAB, GMRES)
    } ;

    class MATH_API IterativeSystemSolver : public SystemSolver {
    public:
        IterativeSystemSolver() ;
        virtual void set_parameters(const SystemSolverParameters& args) ;
        virtual bool needs_rows() const ;                             
        virtual bool needs_columns() const ;                       
        virtual bool supports_symmetric_storage() const ; 
        int nb_iters() const { return nb_iters_ ; }
        void set_nb_iters(int x) { nb_iters_ = x ; }
        double threshold() const { return threshold_ ; }
        void set_threshold(double x) { threshold_ = x ; }
        SystemSolverPreconditioner preconditioner() const { return preconditioner_ ; }
        void set_preconditioner(SystemSolverPreconditioner x) { preconditioner_ = x ; }
        double omega() const { return omega_ ; }
        void set_omega(double x) { omega_ = x ; }
    protected:
        int nb_iters_ ;
        double threshold_ ;
        SystemSolverPreconditioner preconditioner_ ;
        double omega_ ;
    } ;

    class MATH_API SystemSolver_CG : public IterativeSystemSolver {
    public:
        virtual bool solve(const SparseMatrix& A, Vector& x, const Vector& b) ;
    } ;

    class MATH_API SystemSolver_BICGSTAB : public IterativeSystemSolver {
    public:
        virtual bool solve(const SparseMatrix& A, Vector& x, const Vector& b) ;
    } ;

    class MATH_API SystemSolver_GMRES : public IterativeSystemSolver {
    public:
        SystemSolver_GMRES() : nb_inner_iters_(5) { }
        virtual bool solve(const SparseMatrix& A, Vector& x, const Vector& b) ;
        virtual void set_parameters(const SystemSolverParameters& args) ;
    protected:
        int nb_inner_iters_ ;
    } ;

   //--------------------------------------------------------------------------------------------------------------------------------------    

    enum SystemSolverOrdering {
        ORDER_NONE,
        ORDER_DEFAULT,
        ORDER_COLAMD,                // Approximate minimum degree ordering
        ORDER_MMD_AT_PLUS_A  // Symmetric mode (for nearly symmetric matrices)
    } ;

    class MATH_API DirectSystemSolver : public SystemSolver {
    public:
        DirectSystemSolver() ;
        virtual void set_parameters(const SystemSolverParameters& args) ;
        bool keep_permutation_matrix() const  { return keep_permutation_matrix_ ; }
        void set_keep_permutation_matrix(bool x) { keep_permutation_matrix_ = x ; }
        void set_permutation_matrix(const Array1d<int>& perm) ;
        const Array1d<int>& permutation_matrix() const { return perm_ ; }
    protected:
        bool keep_permutation_matrix_ ;
        Array1d<int> perm_ ;
        SystemSolverOrdering ordering_ ;
    } ;

    class MATH_API SystemSolver_SUPERLU : public DirectSystemSolver {
    public:
        virtual bool solve(const SparseMatrix& A, Vector& x, const Vector& b) ;
        virtual bool supports_symmetric_storage() const ; 
    } ;
    
   //--------------------------------------------------------------------------------------------------------------------------------------    
}


#endif

