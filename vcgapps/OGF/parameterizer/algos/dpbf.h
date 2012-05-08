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
 *
 * This code from B. Vallet derivates from A. Sheffer's "Angle Based Flattening",
 * adapted for Graphite by B. Levy.
 * The idea is that using dot products of adjacent faces of each triangles,
 * instead of angles in ABF, as variables is more efficient
 * to constrain the jacobian of a mapping.
 */
 

#ifndef __CELLS_MAP_ALGOS_CJP_DotProducts__
#define __CELLS_MAP_ALGOS_CJP_DotProducts__

#include <OGF/parameterizer/common/common.h>
#include <OGF/cells/map_algos/map_parameterizer.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/math/linear_algebra/sparse_matrix.h>
#include <OGF/math/linear_algebra/vector.h>
#include <OGF/basic/containers/arrays.h>
#include <OGF/basic/types/types.h>

namespace OGF {

    /**
     * Dot Product Based Flattening.
     * (minimizes the variations of the Jacobian)
     * Warning: this is a research prototype (not stable
     * enough for industrialization)
     */
    class PARAMETERIZER_API MapParameterizerDPBF : public MapParameterizer {
    public:
        MapParameterizerDPBF() ;

        /*-+-+-* Main *-+-+-*/

        /** Initializations of the CJP_DotProducts */
        void init() ;
        /** Application of the CJP_DotProducts. */
        bool apply() ;
        /** Application of only one pass. Initialize first. */
        bool pass() ;

        /*-+-+-* Tuning *-+-+-*/

        /** Authalic and planar energy multiplier */
        void set_mu(double val);
        /** Increases mu after each iteration */
        void set_mu_multiplier(double val);
        /** Decreases mu_multiplier after each iteration */
        void set_mu_moderator(double val);
        /** True for parameterization adapted to the geometric level of detail */
        void set_details(bool val);
        /** Funny parameterisation where all 2D triangles has same area */
        void set_isoarea(bool val);
        /** Remembers the dot products of a previous CJP_DotProducts if it exists */
        void set_remember(bool val);
        /** Newton's method step length */
        void set_step_length(double val);
        /** Minimum geometric level of detail
            (added to the real value so the objective Jacobian doesn't go too low) */
        void set_min_detail(double val);
        /** Maximum number of passes */
        void set_max_pass(int val);
        /** Number of iterations of a pass */
        void set_pass_length(int val);

    protected:
        bool do_parameterize_disc(Map* map) ;

        void allocate_variables() ;
        void deallocate_variables() ;

        static int nb_interior_edges(Map* map) ;

        /** Computes objective Jacobian, areas, weights, dot products on the 3D surface */
        void enumerate_and_compute_3D_constants() ;
        /** Reconstruction of the tex coords from the dot products */
        void reconstruct() ;

        /*-+-+-*
          Derivatives of the Lagrangian
          Adds Hessian of the Lagrangian at point x_ to matrix J_
          Adds gradient of the Lagrangian at point x_ to vector b_
          *-+-+-*/

        /** Ec : Shape deformation minimising energy
            1/2 * ((gamma(i)-dp(i))/St)^2
            for each dot product */
        void compute_Ec_derivatives() ;
        /** C1 : Conservation of shared edge length
            (gamma_i,t + gamma_j,t - gamma_i',t' - gamma_j',t')/(St + St') = 0
            for each interior edge e shared by triangles t and t' */
        void compute_C1_derivatives() ;
        /** C2 : planarity around interior vertices
            Sum acotg(gamma_i,t/S_t) - 2PI = 0
            for each interior vertex */
        void compute_C2_derivatives() ;
        /** C3 : Authalism
            (sum gamma_i * gamma_i+1)/St^2 - 1 = 0
            for each triangle. */
        void compute_C3_derivatives() ;

        /** Rem : C2 and C3 are implemented with a quadratic barrier */

    private:

        Map* map_ ;

        /** column permutations (used by SuperLU) */
        Array1d<int> perm_ ;

        /*-+-+-* Mesh characteristics/solver constants *-+-+-*/

        /** Number of facets */
        int nf_ ;
        /** Number of dot products (= 3*nf ) */
        int ngamma_ ;
        /** Number of interior edges */
        int nie_ ;
        /** Total number of unknowns ( ngamma + nie ) */
        int ntot_ ;		

        /*-+-+-* Attributes *-+-+-*/

        /** Index of the dot products */
        MapHalfedgeAttribute<int> dp_index_ ;
        /** Index of the triangles */
        MapFacetAttribute<int> triangle_index_ ;

        /** Vector of dot products + Lagrange multipliers.  size = ntot
            gamma  = x[0..nalpha - 1]
            lambda = x[nalpha..ntot - 1] */
        Vector x_ ;


        /*-+-+-* Precomputed geometrics *-+-+-*/

        /** Optimum dot products. size = ngamma */
        Vector dp_ ;
        /** Derivative of error. size = ngamma */
        Vector errp_ ;
        /** Triangle areas(3D). size = nf */
        Vector area_ ;
        /** Weigths (1/area^2). size = nf */
        Vector w_ ;

        /*-+-+-* Containers for solver input/output *-+-+-*/

        /** Jacobian Matrix. size = ntot x ntot */
        SparseMatrix J_ ;
        /** Gradient of the Lagrangian. size = ntot */
        Vector b_ ;
        /** Step of the newton method. size = ntot
            Solution of J.delta = b */
        Vector delta_ ;

        /*-+-+-* Tuning parameters *-+-+-*/

        /** Authalic and planar energy multiplier */
        double mu_ ;
        /** increases mu after each iteration */
        double mu_multiplier_ ;
        /** decreases mu_multiplier after each iteration */
        double mu_moderator_ ;
        /** upper boundary for mu */
        double mu_max_ ;
        /** Newton's method step length */
        double step_length_ ;		
        /** true for parameterization adapted to the geometric level of detail */
        bool details_ ;
        /** Minimum geometric level of detail
            added to the real value so the objective Jacobian doesn't go too low */
        double min_detail_ ;
        /** funny parameterisation where all 2D triangles has same area */
        bool isoarea_ ;
        /** remembers the dot products of a previous jcdpbf if it exists */
        bool remember_ ;
        /** Tolerance on area deformation */
        double tol_ ;
        /** max number of passes */
        int max_pass_ ;
        /** number of iterations of a pass */
        int pass_length_ ;
    } ;

}

#endif
