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
 

#ifndef __OGF_MATH_NUMERIC_LINEAR_SOLVER__
#define __OGF_MATH_NUMERIC_LINEAR_SOLVER__

#include <OGF/math/common/common.h>
#include <OGF/math/numeric/solver.h>
#include <OGF/math/numeric/system_solver.h>
#include <OGF/math/numeric/super_lu.h>
#include <OGF/math/linear_algebra/sparse_matrix.h>
#include <OGF/math/linear_algebra/abstract_matrix.h>
#include <OGF/math/linear_algebra/vector.h>

namespace OGF {

    //___________________________________________________________________

    /**
     * Solver for linear systems and quadratic optimization problems.
     */
    class MATH_API LinearSolver : public Solver {
    public:

        // __________________ Constructor / Destructor ___________
        
        LinearSolver(int nb_variables) ;
        ~LinearSolver() ;

        // __________________ Solver _____________________________
        
        bool solve() ;

        /** iterative refinement mode */
        void restart() ;

        // __________________ Parameters _________________________

        void set_system_solver(const SystemSolverParameters& args) ;
        void set_system_solver(SystemSolver* solver) { system_solver_ = solver ; }
        SystemSolver* system_solver() { return system_solver_ ; }
        const SystemSolver* system_solver() const { return system_solver_ ; }        

        bool invert_matrix() const { return invert_matrix_ ;  }
        void set_invert_matrix(bool x) { invert_matrix_ = x ; }
        bool least_squares() const { return least_squares_ ;  }
        void set_least_squares(bool b) {
            least_squares_ = b ;
            if(b) {
                symmetric_ =  true ;
            }
        }
        bool symmetric() const  { return symmetric_ ; }
        void set_symmetric(bool b) { symmetric_ = b ; }

        virtual void set_progress(Progress* progress) ;
        virtual void set_quiet(bool x) ;
        SparseMatrix * get_matrix(){ return &G_; }

        // __________________ Construction _____________________

        void begin_system() ;
        void end_system() ;
        
        void begin_row() ;
        void set_right_hand_side(double b) ;
        void add_coefficient(int iv, double a) ;
        void normalize_row(double weight = 1.0) ;
        void scale_row(double s) ;
        void end_row() ;
            
        virtual void update_variables() ;

    protected:
        
        void vector_to_variables(const Vector& x) ;
        void variables_to_vector(Vector& x) ;
        
        // Finite state automaton
        
        enum State {
            INITIAL, IN_SYSTEM, IN_ROW, CONSTRUCTED, SOLVED
        } ;
        
        std::string state_to_string(State s) ;
        void check_state(State s) ;
        void transition(State from, State to) ;

    private:
        
        // Internal representation
        SystemSolver_var system_solver_ ;
        Vector x_ ; 
        SparseMatrix G_ ;
        Vector b_ ;
        
        // Parameters 
        bool least_squares_ ;
        bool symmetric_ ;
        
        // Current row of A
        std::vector<double> af_ ;
        std::vector<int>    if_ ;
        std::vector<double> al_ ;
        std::vector<double> xl_ ;
        double bk_ ;
        bool negative_row_scaling_ ;

        int current_row_ ;

        // Finite state automaton
        State state_ ;
        
        // Iterative refinement
        bool invert_matrix_ ;
        bool update_ ;
        Array1d<int> perm_ ;
        AbstractMatrix M_inv_ ;
    } ;

//_________________________________________________________

}
#endif

