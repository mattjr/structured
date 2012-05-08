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

#ifndef __OGF_PARAMETERIZER_ALGOS_PGP_
#define __OGF_PARAMETERIZER_ALGOS_PGP_

#include <OGF/parameterizer/common/common.h>
#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/math/numeric/system_solver.h>
#include <OGF/math/numeric/non_linear_solver.h>
#include <OGF/math/numeric/linear_solver.h>
#include <OGF/math/geometry/complex.h>

namespace OGF {

    class Graph ;

    //--------------------------------------------------------------------------------------------------------------------------------------

    class Point5d {
    public:
        Point5d(const Point3d& xyz, const Point2d& uv) : xyz_(xyz), uv_(uv) { }
        const Point3d& xyz() const { return xyz_ ; }
        void set_xyz(const Point3d& xyz) { xyz_ = xyz ; }
        const Point2d& uv() const { return uv_ ; }
        void set_uv(const Point2d& uv) { uv_ = uv ; }
    private:
        Point3d xyz_ ;
        Point2d uv_ ;
    } ;

    class PARAMETERIZER_API PeriodicSolver {
    public:
        PeriodicSolver(Map* map, int nb_variables_per_vertex, const std::string& base_attr_name = "U") ;
        virtual ~PeriodicSolver() ;
        int nb_variables_per_vertex() const { return nb_variables_per_vertex_ ; }
        void set_use_non_linear_constraints(bool x) { use_non_linear_constraints_ = x ; }
        void set_system_solver_parameters(const SystemSolverParameters& params) {
            system_solver_parameters_ = params ;
        }
        bool get_use_non_linear_constraints() const { return use_non_linear_constraints_ ; }
        void set_use_non_linear_solver(bool x) { use_non_linear_solver_ = x ; }
        bool get_use_non_linear_solver() const { return use_non_linear_solver_ ; }
        void set_initialize(bool x) { initialize_ = x ; }
        bool get_initialize() const { return initialize_ ; }
        void set_max_newton_iter(int x) { max_newton_iter_ = x ; }
        int get_max_newton_iter() const { return max_newton_iter_ ; }
        void set_lock_borders(bool x) { lock_borders_ = x ; }
        bool get_lock_borders() const { return lock_borders_ ; }

        Map* map() { return map_ ; }

        void compute_singularity_weight() ;

        void set_use_triangle_integral(bool x) { use_triangle_integral_ = x ; }

    protected:

        /**
         * Adds the term: w | Zi - e^(i angle) |^2
         */
        void add_term(double w, int id, double angle) ;

        /** 
         *  Adds the term:  w | Z1 - e^(i delta) Z2 |^2
         *  If swap_sign_id2 is set, Z2 is replaced by its conjugate in the eqn.
         */
        void add_term(double w, int id1, int id2, double delta, bool swap_sign_id2) ;

        void setup_solver() ;
        void begin_equation() ;
        void end_equation() ;
        void solve() ;

        void setup_norm_equation() ;
        virtual void lock_borders() ;

        NonLinearSolver& non_linear_solver()  { return *non_linear_solver_ ; }
        LinearSolver& linear_solver() { return *linear_solver_ ; }
        Solver& solver() { 
            Solver* result = 
                use_non_linear_solver_ ? static_cast<Solver*>(non_linear_solver_) : static_cast<Solver*>(linear_solver_) ;
            return *result ;
        }

        Vector3d rot(const Vector3d& V, const Vector3d& N, double angle) ;

        MapVertexAttribute<int> vertex_id_ ;

    protected:
        double halfedge_weight(Map::Halfedge* h, const Vector3d& field) ;
        double edge_weight(Map::Halfedge* h, const Vector3d& field) ;
        double average_edge_length() ;

    protected:
        void map_to_solver(bool lock_at_least_one = false) ;
        void solver_to_map() ;

    protected:
        Map* map_ ;
        int nb_variables_per_vertex_ ;
        int max_newton_iter_ ;
        bool use_non_linear_constraints_ ;
        bool use_non_linear_solver_ ;
        bool use_triangle_integral_ ;
        bool lock_borders_ ;

        NonLinearSolver* non_linear_solver_ ;
        LinearSolver* linear_solver_ ;
        SystemSolverParameters system_solver_parameters_ ;
        bool initialize_ ;
        int edge_eqn_ ;
        int norm_eqn_ ;
        int val_eqn_ ;

        MapVertexAttribute<Complex>* U_ ;
        MapVertexAttribute<double> cosprod_ ;

        MapVertexAttribute<double> singularity_ ;
    } ;

    //--------------------------------------------------------------------------------------------------------------------------------------

    class PARAMETERIZER_API VectorFieldSmoother : public PeriodicSolver {
    public:
        VectorFieldSmoother(Map* map, int modulo) ;
        void smooth_vector_field() ;
        void set_smoothing_coefficient(double x) { smoothing_coefficient_ = x ; }
        void set_compute_error(bool x) { compute_error_ = x ; }

    protected:
        virtual void lock_borders() ;
        void one_iteration(bool initialize) ;
        void compute_K2() ;
        void compute_error() ;
        double compute_error(
            double angle, const Complex& Z1, const Complex& Z2
        ) ;
    private:
        int modulo_ ;
        double smoothing_coefficient_ ;
        bool compute_error_ ;
        MapVertexAttribute<Vector3d> K_ ;
    } ;

    //--------------------------------------------------------------------------------------------------------------------------------------

    class PARAMETERIZER_API PeriodicParameterizer : public PeriodicSolver {
    public:
        PeriodicParameterizer(Map* map, int nb_variables_per_vertex) ;
        virtual ~PeriodicParameterizer() ;
        virtual void parameterize()  ;
        void set_wavelength(double omega, bool relative, bool use_curl_correction) ;
        static void solve_curl_correction(Map* map, const SystemSolverParameters& solver_config) ;

        void extract_iso_curves(Graph* line, double value, double modulo, bool extract_border = false) ;
        void extract_tex_coords_in_facets() ;
        void extract_tex_coords_in_components() ;

        void extract_tex_coords_in_facet(Map::Facet* f) ;
        void extract_tex_coords_in_path(const std::vector<Map::Halfedge*>& path) ;
        void init_extract_tex_coords(Map::Halfedge* h) ;
        void propagate_along(Map::Halfedge* h) ;
        void propagate_accross(Map::Halfedge* h) ;
        void copy_tex_coords(Map::Halfedge* from, Map::Halfedge* to) ;
        void reorient(Map::Vertex* org, Map::Vertex* dest) ;
        void interpolate(Map::Vertex* v, Map::Vertex* org, Map::Vertex* dest) ;

        /** 
         * parameterizes the newly inserted vertices. For instance, those
         * new vertices can come from a set of iso-curves inserted into
         * the surface.
         */
        void parameterize_new_vertices() ;

    protected:
        void show_tex_coords() ;
        void show_tex_coords(Map::Halfedge* h) ;
        void one_iteration(bool smooth) ;
        void get_facet_min_max(Map::Facet* f, double* mins, double* maxes) ;
        void add_edge_term(Map::Halfedge* h, int id1, const Vector3d& K1, int id2, const Vector3d& K2) ;
        void add_edge_terms(Map::Halfedge* h) ;
        int nearest(const Vector3d& K, Vector3d* K_ref, int n) ;
        virtual void setup_non_linear_constraints() ;

    protected:
        MapVertexAttribute<double> wavelength_ ;
        MapVertexAttribute<Vector3d> K_ ;
        MapHalfedgeAttribute<double>* theta_ ;
    } ;

    //--------------------------------------------------------------------------------------------------------------------------------------

} 



#endif
