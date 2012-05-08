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
 

#ifndef __ANISOTROPY_ALGOS_MAP_TRIMMER__
#define __ANISOTROPY_ALGOS_MAP_TRIMMER__

#include <OGF/parameterizer/common/common.h>
#include <OGF/cells/graph/graph.h>
#include <OGF/cells/graph/graph_editor.h>
#include <OGF/cells/graph/graph_attributes.h>
#include <OGF/cells/map/map_embedding.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map/ext_map_editor.h>
#include <OGF/cells/map/geometry.h>

#include <vector>
#include <algorithm>

namespace OGF {

    class PARAMETERIZER_API MapTrimmer {
    public:
        MapTrimmer(Graph* trimming_curve) ;
        ~MapTrimmer() ;
        
    public:

        void set_lock_branchings(bool x) {
            lock_branchings_ = x ;
        }

        /**
         * Adds intersection where two segments of
         * the trimming curve are embedded in the
         * same facet.
         * Note: current version can handle only one
         * intersection per facet.
         */
        void intersect_trimming_curve() ;
        
        /**
         * removes segments of the trimming curve
         * located in singularities.
         */
        void fix_trimming_curve_singularities(bool merge = true) ;


        void create_map_from_trimming_line(Map* target, bool squares_only) ;

        Map* map() const { return map_ ; }
        Graph* graph() const { return graph_ ; }

        
        void set_merge_intersections_in_facets(bool x) {
            merge_intersections_in_facets_ = x ;
        }

        // ---------------------- Lower level functions ---------------------------------------------

        void intersect_trimming_curve_in_facets() ;

        void intersect_trimming_curve_in_facet(
            Map::Facet* f, std::vector<Graph::Halfedge*>& edges
        ) ;

        void intersect_trimming_curve_in_facet_and_merge(
            Map::Facet* f, std::vector<Graph::Halfedge*>& edges
        ) ;
        

        class IsectEdges : public std::vector<Graph::Halfedge*> {
        public:
            Graph::Vertex* v1() { return (*begin())->opposite()->vertex() ; }
            Graph::Vertex* v2() { return (*rbegin())->vertex() ;            }
            static double local_coord(Graph::Halfedge* h, Graph::Vertex* v) ;
        } ;

        void insert_intersection(IsectEdges& edges, Graph::Vertex* v) ;

        /**
         * @param threshold is relative to current edge length.
         * @param extremities_only if set, merge vertices only
         *   if at least one of them is an extremity.
         */
        void intersect_trimming_curve_in_edges(
            double threshold = 1e-2, bool extremities_only = false
        ) ;
        void intersect_trimming_curve_in_vertices() ;

        Graph::Vertex* insert_intersection(Graph::Halfedge* h1, Graph::Halfedge* h2, const Point3d& I) ;
        Graph::Vertex* insert_intersection(Graph::Halfedge* h1, const Point3d& I, int kind = 0) ;
        void insert_intersection(Graph::Halfedge* h1, Graph::Vertex* v, int kind = 0) ;


        /** Does some combinatorial validity tests and outputs the result in the console */
        void check_conformity() ;

        /**
         * Traverses the graph as if it was a map. The ordered_next() function
         * makes it possible to traverse the loops of the graph (it uses the
         * embedding to determine the correct order)
         */
        Graph::Halfedge* ordered_next(Graph::Halfedge* h) ;


        /** Geometric version, used by ordederd_next() */
        void get_graph_ordered_star_from_geometry(
            Graph::Vertex* gv, std::vector<Graph::Halfedge*>& gstar
        ) ;

        //   Note: for the moment, we prefer using the geometric version,
        // since there are still fonfigurations for which the combinatorial
        // version fails (i.e. it is *bugged* :-( )
        /** Combinatorial version. */
        void get_graph_ordered_star(Graph::Vertex* v, std::vector<Graph::Halfedge*>& star) ;

        /** used by get_graph_ordered_star() */
        void get_star_border(const MapCellEmbedding& em, std::vector<Map::Halfedge*>& star_border) ;

        bool is_chart_corner(Graph::Halfedge* h) ;

        static Vector3d normal(const MapCellEmbedding& em) ;
        Vector3d normal(Graph::Vertex* v) ;

        // Calls GraphEditor::merge_vertices() + kind_ management
        Graph::Vertex* merge_vertices(Graph::Vertex* v1, Graph::Vertex* v2) ;

        Graph::Halfedge* check_connect_vertices(Graph::Vertex* v1, Graph::Vertex* v2) ;

        // used by fix_trimming_line_singularities
        void connect_vertices(
            GraphEditor& editor, std::vector<Graph::Vertex*>& to_connect, MapCellEmbedding& em 
        ) ;

    private:
        Graph* graph_ ;
        Map* map_ ;
        GraphVertexAttribute<MapCellEmbedding> embedding_ ;
        GraphVertexAttribute<bool> is_corner_ ;
        MapVertexAttribute<Graph::Vertex*> embedded_ ;
        GraphVertexAttribute<Vector3d> normal_ ;
        GraphHalfedgeAttribute<int> kind_ ;
        ExtMapEditor map_editor_ ;
        GraphEditor graph_editor_ ;
        bool merge_intersections_in_facets_ ;
        bool lock_branchings_ ;
    } ;

    //________________________________________________________________________________

    /**
     * MapTrimmerVertexInEdge is used to sort the vertices
     * embedded in an edge.
     */
    struct MapTrimmerVertexInEdge {
        MapTrimmerVertexInEdge(Graph::Vertex* v_in, Map::Halfedge* h) {
            Vector3d X = Geom::vector(h) ;
            v = v_in ;
            s = (v->point() - h->prev()->vertex()->point()) * X ;
        }
        Graph::Vertex* v ;
        double s ;
        bool operator<(const MapTrimmerVertexInEdge& rhs) const { 
            return (s < rhs.s) ;
        }
    } ;

    class MapTrimmerVerticesInEdge : public std::vector<MapTrimmerVertexInEdge> {
    public:
        void sort() { std::sort(this->begin(), this->end()) ; }
    } ;

    //________________________________________________________________________________

}

#endif
