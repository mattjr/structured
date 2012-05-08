/*
 *  GXML/Graphite: Geometry and Graphics Programming Library + Utilities
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
 

#ifndef __OGF_CELLS_MAP_ALGOS_HIERARCHICAL_MAP_PARAMETERIZER__
#define __OGF_CELLS_MAP_ALGOS_HIERARCHICAL_MAP_PARAMETERIZER__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map_algos/map_parameterizer.h>
#include <OGF/math/geometry/polygon2d.h>

namespace OGF {

    class Map ;
    class PMManager ;
    class Progress ;
    class VertexSplit ;

//_________________________________________________________

    class CELLS_API HierarchicalMapParameterizer : public MapParameterizer {
    public:
        HierarchicalMapParameterizer() ;

        double get_reduction_factor() const  { return reduction_factor_ ; }
        void set_reduction_factor(double x) { reduction_factor_ = x ; }

    protected:
        virtual bool do_parameterize_disc(Map* map) ;

        virtual void optimize() ;
        virtual void compute_initial_parameterization() = 0 ;
        virtual void optimize_parameterization(int nbiter) = 0 ;

        bool get_valid_tex_coord(VertexSplit& split, Point2d& result) ;
        Point2d kernel_barycenter(const Polygon2d& neighbors, bool& terminate) ;
        bool vertex_is_in_kernel(Map::Vertex* v) ;
        bool check_kernels() ;
        void set_hierarchical(bool x) { hierarchical_ = x ; }

    protected:
        bool strict_kernels_ ;
        bool lock_borders_ ;
        bool hierarchical_ ;

    private:
        PMManager* pm_manager_ ;
        double reduction_factor_ ;
    } ;

//_________________________________________________________

    class CELLS_API RandomDescentMapParameterizer : public HierarchicalMapParameterizer {
    public:
        RandomDescentMapParameterizer() ;

    protected:
        virtual bool do_parameterize_disc(Map* disc) ;
        virtual void optimize_parameterization(int nbiter) ;

        /**
         * Default implementation returns the sum of facet_criterion
         * for the facets incident to v.
         * Derived classes may overload this function, or
         *  overload facet_criterion().
         */
        virtual double vertex_criterion(Map::Vertex* v) ;

        /**
         * Default implementation returns 0.
         * Derived classes may overload this function, or
         *  overload vertex_criterion().
         * Note that if vertex_criterion() is overloaded,
         *  facet_criterion() is not used.
         */
        virtual double facet_criterion(Map::Facet* f) ;

        struct OptimVertex {
            OptimVertex(Map::Vertex* v, double crit) : vertex(v), criterion(crit) { }
            Map::Vertex* vertex ;
            double criterion ;
        } ;

        class OptimVertexCompareGT {
        public:
            inline bool operator()(const OptimVertex& v1, const OptimVertex& v2) const {
                return (v1.criterion > v2.criterion) ;
            }
        } ;

        /**
         * Optimizes a vertex using a random direction.
         * The step is found using a dichotomy alglorithm
         * (calls vertex_criterion).
         */
        virtual void optimize_vertex(OptimVertex& o) ;

        /** called by optimize_vertex() */
        virtual void linear_search(OptimVertex& o, const Vector2d& direction) ;

        /**
         * Returns the radius of the region where
         * the optimum will be searched around a
         * given vertex.
         */
        double trust_region_radius(Map::Vertex* v) ;

        Vector2d random_direction() ;

        MapFacetAttribute<double> facet_area_ ;
    } ;

//_________________________________________________________

}
#endif

