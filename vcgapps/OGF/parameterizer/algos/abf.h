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
 

#ifndef __CELLS_MAP_ALGOS_ABF__
#define __CELLS_MAP_ALGOS_ABF__

#include <OGF/parameterizer/common/common.h>
#include <OGF/cells/map_algos/map_parameterizer.h>
#include <OGF/math/linear_algebra/sparse_matrix.h>
#include <OGF/math/linear_algebra/vector.h>
#include <OGF/basic/containers/arrays.h>

namespace OGF {

    /**
     * A. Sheffer's "Angle Based Flattening"
     * Reference:
     * A. Sheffer and E. de Sturler, Parameterization of 
     * Faceted Surfaces for Meshing Using Angle Based Flattening, 
     * Engineering with Computers, 17 (3), 326-337, 2001.
     *
     * Note: This class is here for educational purposes,
     * we recommend using ABF++ instead (more efficient)
     */
    class PARAMETERIZER_API ABFMapParameterizer : public MapParameterizer {
    public:
        ABFMapParameterizer() ;
        void apply() ;

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
        virtual bool do_parameterize_disc(Map* map) ;
        void allocate_variables() ;
        void deallocate_variables() ;

        // takes into account polygonal facets
        static int nb_triangles(Map* map) ; 
        static int nb_interior_vertices(Map* map) ;

        // ------------------ Drivers

        void enumerate_angles() ;
        void compute_beta() ;
        void solve_angles() ;

        // ------------------ Jacobian of the constraints:

        // C1 : alpha_i1 + alpha_i2 + alpha_i3 - PI = 0
        //        where Ti = (i1,i2,i3)
        // C2 : Sum alpha_ij - 2.PI = 0
        //        for alpha_ij incident to vertex i
        // C3 : Prod sin(gamma_ij) - Prod sin(gamma'_ij) = 0
        //        for gamma_ij and gamma'_ij opposite incident to vertex i
        //

        void compute_sum_log_sin_angles(
            Map::Vertex* v, double& prod_prev_sin, double& prod_next_sin
        ) ;

        void compute_product_sin_angles(
            Map::Vertex* v, double& prod_prev_sin, double& prod_next_sin
        ) ;

        void add_JC1() ;
        void add_JC2() ;
        void add_JC3() ;

        // ------------------  Right hand side: gradients

        // Gradient of the quadratic form
        void sub_grad_F () ;

        // For each triangle: sum angles - PI
        void sub_grad_C1() ;

        // For each vertex: sum incident angles - 2.PI
        void sub_grad_C2() ;

        // For each vertex: prod sin(next angle) - prod sin(prev angle)
        void sub_grad_C3() ;

        // ------------------ Convergence control

        double compute_step_length_and_update_weights() ;
        double errf() const ;
        double errC() const ;
        double compute_errx_and_update_x(double s) ;

    private:
        int nf_ ;     // Number of facets
        int nalpha_ ; // Number of angles (= 3.nf )
        int nint_ ;   // Number of interior nodes
        int ntot_ ;   // Total number of unknowns ( nalpha + nf + 2 * nint )

        MapHalfedgeAttribute<int> angle_index_ ;

        Vector x_ ; // Unknown angles + Lagrance multipliers.  size = ntot
                         // alpha  = x[0..nalpha - 1]
                         // lambda = x[nalpha..ntot - 1] 

        Vector beta_ ;  // Optimum angles.  size = nalpha
        Vector w_ ;     // Weights.         size = nalpha


        SparseMatrix J_ ; // size = ntot x ntot
        Vector b_ ;       // size = ntot
        Vector delta_ ;   // size = ntot

        //_______________ constants, tuning parameters ____________________

        const double epsilon_ ;    // threshold for small angles 
        double newton_tolf_;   // threshold for gradient norm (rhs)
        double newton_tolx_ ;  // threshold for delta norm 
        int max_newton_iter_ ; 
        const double positive_angle_ro_ ;
        double step_length_factor_ ;

        bool log_mode_ ;
    } ;

}

#endif
