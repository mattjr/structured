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

#ifndef __OGF_CELLS_MAP_ALGOS_ISO_CURVES_EXTRACT__
#define __OGF_CELLS_MAP_ALGOS_ISO_CURVES_EXTRACT__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map/map_embedding.h>
#include <OGF/cells/graph/graph.h>
#include <OGF/cells/graph/graph_attributes.h>
#include <OGF/cells/graph/graph_editor.h>

namespace OGF {

    class CELLS_API GraphInMap {
    public:
        GraphInMap(Map* map, Graph* graph) ;

        /** copies the border of the Map in the Graph */
        void extract_border() ;

        void connect_vertices(Graph::Vertex* v1, Graph::Vertex* v2) ;
        Graph::Vertex* find_or_create(Map::Vertex* v) ;
        Graph::Vertex* find_or_create(Map::Halfedge* h, const Point3d& p)  ;
        Graph::Vertex* find_or_create(Map::Facet* f, const Point3d& p) ;

    protected:
        Map* map_ ;
        Graph* graph_ ;
        MapHalfedgeAttribute< std::vector<Graph::Vertex*> > vertices_in_edge_ ;
        MapVertexAttribute<Graph::Vertex*> vertex_in_vertex_ ;
        GraphVertexAttribute<MapCellEmbedding> embedding_ ;
        GraphEditor graph_editor_ ;
    } ;

    //-----------------------------------------------------------------------------------------------------------------------------

    /**
     * Extracts iso-curves relative to user-defined values. It also initializes
     * a MapCellEmbedding attribute in the extracted graph (can be used
     * to cut the surface by the iso-curve, see e.g. MapTrimmer).
     */

    class CELLS_API MapIsoCurvesExtractor : public GraphInMap {
    public:
        MapIsoCurvesExtractor(Map* map, Graph* graph) ;
        void set_iso_value(double u) { iso_value_ = u ; }

        void begin_facet(Map::Facet* f) ;
        void vertex(Map::Vertex* v, double u) ;
        void end_facet() ;


    protected:
        Graph::Vertex* intersect(Map::Halfedge* h, double u1, double u2, double u) ;        
        void create_edges_in_facet(Map::Facet* f, const std::vector<Graph::Vertex*>& vertices) ;

        
    private:

        Map::Facet* current_facet_ ;
        double iso_value_ ;
        std::vector<Map::Vertex*> map_vertices_ ;
        std::vector<double> values_ ;
    } ;
} 

#endif
