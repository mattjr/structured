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
 

#ifndef __CELLS_MAP_ALGOS_PARAMETERIZER__
#define __CELLS_MAP_ALGOS_PARAMETERIZER__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map_algos/map_components.h>
#include <OGF/basic/types/counted.h>
#include <OGF/basic/types/smart_pointer.h>
#include <OGF/math/numeric/system_solver.h>

namespace OGF {

    class Progress ; 
    class Solver ;

//_________________________________________________________

    class MapComponent ;
    
    class CELLS_API MapParameterizer : public Counted {
    public:

        MapParameterizer() ;
        virtual ~MapParameterizer() ;

        void parameterize_map(Map* map) ;
        virtual bool parameterize_disc(MapComponent* disc) ;
        virtual bool parameterize_disc(Map* disc) ;

        void use_locked_vertices_as_corners(bool x) {
            use_locked_vertices_as_corners_ = x ;
        }

        bool use_locked_vertices_as_corners() const {
            return use_locked_vertices_as_corners_ ;
        }

        void set_system_solver_parameters(const SystemSolverParameters& x) {
            system_solver_parameters_ = x;
        }

    protected:

        virtual bool do_parameterize_disc(Map* disc) = 0 ;
        virtual bool do_parameterize_disc(MapComponent* component) ;
        void do_parameterize_disc_using_parameterizer(MapParameterizer* param, Map* disc) {
            param->do_parameterize_disc(disc) ;
        }

        Map* component_to_map(MapComponent* comp) ;
        Map* triangulate(Map* map) ;
        void update_from_map(Map* map) ;

        void border_to_solver(Solver& solver, Map::Halfedge* h) ;
        virtual void project_border_on_circle(Map::Halfedge* h) ;
        virtual void project_border_on_square(Map::Halfedge* h, bool use_locks = false) ;
        void project_border_on_convex_polygon(Map::Halfedge* h) ;
        void parameterize_edge(Map::Halfedge* h1, const Point2d& uv1, Map::Halfedge* h2, const Point2d& uv2) ;
        double edge_length(Map::Halfedge* h1, Map::Halfedge* h2) ;
        
        /**
         * finds the two vertices that will be pinned by LSCM.
         */
        void get_border_extrema(
            Map::Halfedge* h, 
            const Point3d& center, const Vector3d& V,
            Map::Vertex*& vxmin, Map::Vertex*& vxmax
        ) ;

        /**
         * copies u,v coordinates from the solver to the map.
         */
        void solver_to_map(const Solver& solver) ;

        /**
         * Derived classes need to call this function at the beginning of
         * parameterizeMapComponent(). It has the following roles:
         * 1) Binds the attributes (m_vertexId and m_texCoord)
         * 2) Computes a vertex numbering that "virtually" discards
         * small edges.
         * 3) Computes the bbox of the surface (required by the normalize()
         *   function).
         */
        void begin_parameterization(Map* map) ;

        /**
         * Derived classes need to call this function at the end of
         * parameterizeMapComponent(). It has the following roles:
         * 1) Unbinds the attributes.
         * 2) Sets m_part to NULL
         */
        void end_parameterization() ;

        /**
         * Transformed p into normalized space.
         */
        void normalize(Point3d& p) ;

        /**
         * Checks whether the edge h has been collapsed
         */
        bool halfedge_is_discarded(Map::Halfedge* h) ;

        /**
         * Checks whether all the edges of a facet have been collapsed
         */
        bool facet_is_discarded(Map::Facet* f) ;
        
        /**
         * In LSCM, the two pinned vertices are selected on the largest
         * border of the surface.
         */
        Map::Halfedge* largest_border() ;

        /**
         * Computes the center and principal axes of the surface.
         * If numerical problems occur, returns the center and the two 
         * largest sides of the bounding box.
         */
        void principal_axes(
            Point3d& p, Vector3d& v1, Vector3d& v2
        ) ;


        /**
         * Computes the center and principal axes of the border incident
         * to h.
         * If numerical problems occur, returns the center and the two 
         * largest sides of the bounding box.
         */
        void principal_axes(
            Map::Halfedge* h, Point3d& p, Vector3d& v1, Vector3d& v2
        ) ;

        /**
         * Returns the center and the two largest sides of the bounding box.
         */           
        void box_axes(
            const Box3d& box, Point3d& p, Vector3d& v1, Vector3d& v2
        ) ;

        void project_on_principal_plane() ;

        /**
         * For numerical stability reasons,
         * all the computations are done in normalized space.
         * This function initializes m_center and m_radius.
         */
        void get_bounding_box() ;
        

        /** Assigns ids to the Vertices */
        void enumerate_vertices() ;
        
        /**
         * Gives the same ids to the vertices of small edges.
         * enumerateVertices should have been called before.
         * Returns the number of variables.
         */
        int discard_small_edges(
            double relative_treshold = 0.0001
        ) ;
        
        /**
         * Gives the same ids to the two vertices of the specified
         * edge.
         */
        void edge_collapse(Map::Halfedge* h) ;
        
    protected:
        Map* map_ ;
        MapVertexAttribute<int> vertex_id_ ;
        Point3d  center_ ;
        double   radius_ ;
        int      nb_distinct_vertices_ ;
        bool     triangles_only_ ;
        bool     use_locked_vertices_as_corners_ ;
        SystemSolverParameters system_solver_parameters_ ;
    } ;
    
    typedef SmartPointer<MapParameterizer> MapParameterizer_var ; 


//_________________________________________________________


    class CELLS_API ParamValidator {
    public:
        ParamValidator() ;
        ~ParamValidator() ;

        bool component_is_valid(MapComponent* comp) ;
        double component_scaling(MapComponent* comp) ;
        void compute_fill_and_overlap_ratio(MapComponent* comp) ;
        double fill_ratio() const { return fill_ratio_ ; }
        double overlap_ratio() const { return overlap_ratio_ ; }

        double get_max_overlap_ratio() const { return max_overlap_ratio_ ; }
        void set_max_overlap_ratio(double x) { max_overlap_ratio_ = x ; }
        double get_max_scaling() const { return max_scaling_ ; }
        void set_max_scaling(double x) { max_scaling_ = x ; }
        double get_min_fill_ratio() const { return min_fill_ratio_ ; }
        void set_min_fill_ratio(double x) { min_fill_ratio_ = x ; }

    protected:
        void begin_rasterizer(MapComponent* comp) ;
        void end_rasterizer() ;

        void rasterize_triangle(
            const Point2d& p1, const Point2d& p2, const Point2d& p3
        ) ;
        void transform(const Point2d& p, int& x, int& y) ;

    protected:
        int graph_size_ ;
        Numeric::uint8* graph_mem_ ;
        int* x_left_ ;
        int* x_right_ ;

        double user_x_min_ ;
        double user_y_min_ ;
        double user_width_ ;
        double user_height_ ;
        double user_size_ ;

        double fill_ratio_ ;
        double overlap_ratio_ ;

        double max_overlap_ratio_ ;
        double max_scaling_ ;
        double min_fill_ratio_ ;

    private:
        // ParamValidator cannot be copied.
        ParamValidator(const ParamValidator& rhs) ;
        ParamValidator& operator=(const ParamValidator& rhs) ;
    } ;

}
#endif

