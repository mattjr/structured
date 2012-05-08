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
 
#ifndef __CELLS_MAP_ALGOS_ABF_PLUS_PLUS__
#define __CELLS_MAP_ALGOS_ABF_PLUS_PLUS__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map/map_builder.h>
#include <OGF/cells/map_algos/map_parameterizer.h>
#include <OGF/math/linear_algebra/sparse_matrix.h>
#include <OGF/math/linear_algebra/vector.h>
#include <OGF/basic/containers/arrays.h>


namespace OGF {

    //_______________________________________________________________________

    /**
     * Sheffer & De Sturler's "Angle Based Flattening"
     * combined with robust angle to (u,v) 
     * conversion, matrix splitting and 
     * Graphite's optimized sparse matrices.
     *
     * Initial reference:
     * ------------------
     * A. Sheffer and E. de Sturler, Parameterization of 
     * Faceted Surfaces for Meshing Using Angle Based Flattening, 
     * Engineering with Computers, 17 (3), 326-337, 2001.
     *
     * This code is the result of several theoretical and 
     * implementation-related optimizations performed on
     * ABF original code by B. Levy and A. Sheffer.
     */
    class CELLS_API ABFPlusPlusMapParameterizer : public MapParameterizer {
    public:
        ABFPlusPlusMapParameterizer() ;

        virtual bool do_parameterize_disc(Map* map) ;

        // _____________ Tuning parameters _____________________
        // Most users will not need to change them.


        // Enables Zayer et.al's modified wheel condition
        void set_log_mode(bool x) { log_mode_ = x ; }
        bool get_log_mode() const { return log_mode_ ; }

        // Tolerance for the norm of the gradient
        double get_newton_tolf() const { return newton_tolf_ ; }
        void set_newton_tolf(double x) { newton_tolf_ = x ; }

        // Tolerance for the norm of delta (step vector)
        double get_newton_tolx() const { return newton_tolx_ ; }
        void set_newton_tolx(double x) { newton_tolx_ = x ; }

        // Maximum number of newton iterations (outer loop)
        int get_max_newton_iters() const { return max_newton_iter_ ; }
        void set_max_newton_iters(int n) { max_newton_iter_ = n ; }

        double get_step_length_factor() const { return step_length_factor_ ; }
        void set_step_length_factor(double x) { step_length_factor_ = x ; }

    protected:

        void allocate_variables() ;
        void deallocate_variables() ;

        // takes into account polygonal facets
        static int nb_triangles(Map* map) ; 
        static int nb_interior_vertices(Map* map) ;

        // ------------------ Main driver & Utilities

        void enumerate_angles() ;
        void compute_beta() ;
        bool solve_angles() ;

        // ------------------ Jacobian of the constraints:

        // C1 : alpha_i1 + alpha_i2 + alpha_i3 - PI = 0
        //        where Ti = (i1,i2,i3)
        // C2 : Sum alpha_ij - 2.PI = 0
        //        for alpha_ij incident to vertex i
        // C3 : Prod sin(gamma_ij) - Prod sin(gamma'_ij) = 0
        //        for gamma_ij and gamma'_ij opposite incident to vertex i
        //

        void compute_product_sin_angles(
            Map::Vertex* v, double& prod_prev_sin, double& prod_next_sin
        ) ;

        void compute_sum_log_sin_angles(
            Map::Vertex* v, double& sum_prev_sin, double& sum_next_sin
        ) ;

        // JC1 is implicit.
        void add_JC2() ;
        void add_JC3() ;

        // ------------------  Right hand side: gradients

        // Gradient of the quadratic form
        void sub_grad_F() ;

        // For each triangle: sum angles - PI
        void sub_grad_C1() ;

        // For each vertex: sum incident angles - 2.PI
        void sub_grad_C2() ;

        // For each vertex: prod sin(next angle) - prod sin(prev angle)
        void sub_grad_C3() ;

        // ------------------ Solver for one iteration

        // computes dalpha, dlambda1 and dlambda2
        void solve_current_iteration() ;

        // ------------------ Convergence control

        // increases the weights of negative angles.
        double compute_step_length_and_update_weights() ;

        // returns the norm of the gradient (quadratic form + cnstrs).
        double errf() const ;

        // returns the norm of the step vector.
        double compute_errx_and_update_x(double s) ;

        // ------------------ Utilities
        
        // y += J.D.x
        void add_J_D_x(
            Vector& y, 
            const SparseMatrix& J, Vector& D, Vector& x
        ) ;

        // M += J.D.J^t
        void add_J_D_Jt(
            SparseMatrix& M, const SparseMatrix& J, Vector& D
        ) ;

        // M -= J.D.J^t
        void sub_J_D_Jt(
            SparseMatrix& M, const SparseMatrix& J, Vector& D
        ) ;

    private:

        Map* map_ ;

        int nf_ ;      // Number of facets
        int nalpha_ ;  // Number of angles (= 3.nf)
        int nint_ ;    // Number of interior nodes
        int nlambda_ ; // Number of constraints (= nf+2.nint)
        int ntot_ ;    // Total number of unknowns (= nalpha + nlamda)

        MapHalfedgeAttribute<int> angle_index_ ;


        // ------ ABF variables & Lagrange multipliers ----------------
        Vector alpha_ ;    // Unknown angles. size = nalpha
        Vector lambda_ ;   // Lagrange multipliers. size = nlambda
        Vector beta_ ;     // Optimum angles.  size = nalpha
        Vector w_ ;        // Weights.         size = nalpha

        // ------ Step vectors ----------------------------------------
        Vector dalpha_   ; // size = nalpha ; angles
        Vector dlambda1_ ; // size = nf     ; C1 part
        Vector dlambda2_ ; // size = 2.nint ; C2 and C3 part

        // ------ Right-hand side ( - gradients ) ---------------------
        Vector b1_ ;      // size = nalpha
        Vector b2_ ;      // size = nlambda

        // ------ Jacobian of the constraints -------------------------
        // J1 (Jacobian of constraint 1) is not stored, it is implicit
        SparseMatrix J2_ ; // size = 2.nint * nalpha


        // ------ ABF++ variables -------------------------------------
        Vector Delta_inv_      ; // size = nalpha
        Vector Delta_star_inv_ ; // size = nf ; 
        SparseMatrix J_star_   ; // size = 2.nint * nf
        Vector b1_star_        ; // size = nf
        Vector b2_star_        ; // size = 2.nint

        // ------ Final linear system ---------------------------------
        SparseMatrix M_        ; // size = 2.nint * 2.nint
        Vector       r_        ; // size = 2.nint

        Array1d<int> perm_ ;   // column permutations (used by SuperLU)

        //_______________ constants, tuning parameters ____________________

        const double epsilon_ ;    // threshold for small angles 

        double newton_tolf_;   // threshold for gradient norm (rhs)
        double newton_tolx_ ;  // threshold for delta norm 
        int max_newton_iter_ ; 
        const double positive_angle_ro_ ;
        double step_length_factor_ ;

        bool low_mem_ ; // If set, tries to reduce memory footprint
        
        bool log_mode_ ;
    } ;

    //_______________________________________________________________________
}

#endif
