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
 

#ifndef __GRAPHITE_TEX_MESH_EDITOR__
#define __GRAPHITE_TEX_MESH_EDITOR__

#include <OGF/cells/map/map_editor.h>
#include <vector>

namespace OGF {

//_________________________________________________________

    class MapBorder ;

    /**
     * Adds a few functionalities to the standard
     * MapEditor (for the moment, they are not stable 
     * enough to be included in the library)
     */

    class CELLS_API ExtMapEditor : public MapEditor {
    public:
        ExtMapEditor(Map* map) : MapEditor(map) { }

        /**
         * If a single border arrives at this edge,
         * connects the two corresponding edges.
         */
        bool zip_edge(
            Map::Vertex* from
        ) ;

        /**
         * Disconnects the edges forming the shortest
         * path between from and to. 
         */
        bool cut_edges(
            Map::Vertex* from, Map::Vertex* to
        ) ;

        /**
         * Resamples the two borders incident to h1 and h2
         * so that they have the same number of vertices.
         * If merge_geo_vertices is set, connects the borders.
         * If merge_tex_vertices is set, connects the texture
         * vertices.
         */
        bool match_borders(
            Map::Halfedge* h1,
            Map::Halfedge* h2,
            bool merge_geo_vertices,
            bool merge_tex_vertices
        ) ;

        /**
         * Recomputes vertex normals and facet normals
         * (if present) in the neighborhood of v.
         */
        void compute_normals_around_vertex(Map::Vertex* v) ;

        /**
         * Recomputes vertex normals and facet normals
         * (if present) in the neighborhood of f.
         */
        void compute_normals_around_facet(Map::Facet* f) ;

        /**
         * Recomputes vertex normals and facet normals
         * (if present) in the neighborhood of h.
         */
        void compute_normals_around_edge(Map::Halfedge* h) ;

        
        /**
         * Recursively adds edges in facet f until the pieces have
         * no more than max_nb_vertices. Inserted edges minimize their
         * lengthes and try to balance the size of the parts.
         */
        void subdivide_facet(Map::Facet* f, int max_nb_vertices = 3) ;


        /**
         * Creates a square facet connecting the two specified edges.
         * Precondition: h1->is_border() && h2->is_border().
         */
        Map::Facet* create_facet_between_edges(Map::Halfedge* h1, Map::Halfedge* h2) ;

    protected:
        friend class ::OGF::MapBorder ;
        static Map::Halfedge* next_around_border(
            Map::Halfedge* h
        ) ;

        static Map::Halfedge* prev_around_border(
            Map::Halfedge* h
        ) ;

        static void get_border_between_anchors(
            Map* s,
            Map::Halfedge* h,
            MapBorder& out
        ) ;
    } ;
    
//_________________________________________________________

}
#endif

