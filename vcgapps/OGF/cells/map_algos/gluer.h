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

#ifndef __CELLS_MAP_ALGOS_GLUER__
#define __CELLS_MAP_ALGOS_GLUER__

#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map/map_editor.h>
#include <OGF/cells/map_algos/map_components.h>
#include <OGF/math/geometry/types.h>

#include <vector>
#include <map>

namespace OGF {
    
    //_________________________________________________________
    
    /**
     * Merges the edges having the same geometric location.
     */
    class CELLS_API Gluer : public MapMutator {
    public:
        enum GlueFunc { 
            GLUE_ALWAYS, GLUE_NEQ, GLUE_EQ, GLUE_1_OR_2, GLUE_1_AND_2
        } ;

        Gluer() ;
        virtual ~Gluer() ;
        void set_map(Map* s) {
            map_ = s ;
            set_target(s) ;
        }
        Map* map() const {
            return map_ ;
        }

        void set_glue_func(GlueFunc f) { glue_func_ = f ; }
        GlueFunc glue_func() const { return glue_func_ ;  }
        
        void set_vertex_attribute(Attribute<Map::Vertex, int>& attr) { 
            vertex_attribute_ = attr ;
        } 
        Attribute<Map::Vertex, int>& vertex_attribute() {
            return vertex_attribute_ ;
        }

        int id1() { return id1_ ; }
        void set_id1(int id) { id1_ = id ; }

        int id2() { return id2_ ; }
        void set_id2(int id) { id2_ = id ; }

        virtual bool test_glue_func(
            Map::Halfedge* h1, Map::Halfedge* h2
        ) const ;

        /**
         * Points nearer than tolerance are merged.
         */
        void set_tolerance(double tolerance) {
            tolerance_ = tolerance ;
        }
        
        double tolerance() const {
            return tolerance_ ;
        }
        
        virtual void apply() = 0 ;

        void set_parent(Gluer* parent) { parent_ = parent ; }

    protected:
        double  tolerance_;
        Map* map_;
        Attribute<Map::Vertex, int> vertex_attribute_ ;
        int id1_ ;
        int id2_ ;
        GlueFunc glue_func_ ;
        
    private:
        // A gluer cannot be copied
        Gluer(const Gluer& rhs) ;
        const Gluer& operator=(const Gluer& rhs) ;
        Gluer* parent_ ;
    } ;
    
    //_________________________________________________________

    /**
     * An implementation of Gluer based on a table.
     */
    class CELLS_API DefaultGluer : public Gluer {
        class Gluer_PtsCmp {
        public:
            bool operator() (const Point3d& p0,const Point3d& p1) const ;
        } ;
    public:
        virtual void apply();
        
    protected:
        Point3d grid_node(const Point3d& p);
        void add_new_point(Point3d& p, int index);
 	void add_halfedge(Map::Halfedge* hi);
        bool have_same_location(
            Map::Halfedge* h0,
            Map::Halfedge* h1
        ) ;

        bool is_non_manifold(Map::Halfedge* hi) ;
        int nb_halfedges_like(Map::Halfedge* hi) ;
        bool test_info(Map::Halfedge* h1, Map::Halfedge* h2) ;
        bool can_merge(TexVertex* tv1, TexVertex* tv2) ;

    private:
        std::vector<bool> exist;
        std::vector<Map::Halfedge*> halfedges; 
        
        typedef std::map<Point3d,std::vector<int>, Gluer_PtsCmp> 
        PointSearcher ;
        PointSearcher point_searcher;

        MapFacetMaterialId facet_material_id_ ;
        MapTexVertexNormal tex_vertex_normal_ ;
        bool has_attributes_ ;
    } ;
    
    //________________________________________________________

    /**
     * An implementation of Gluer using the DefaultGluer
     * with increasing tolerances.
     */
    class CELLS_API RecursiveGluer : public Gluer {
    public:
        virtual void apply() ;
    } ;
    
   
    //________________________________________________________   
   
    /**
     * A RecursiveGluer that sets its parameters automatically
     */
    class CELLS_API AutoSetGluer : public RecursiveGluer {
    public:
        virtual void apply() ;
    } ;

    //________________________________________________________   

    /**
     * Used by ReorientGluer.
     */
    class CELLS_API FastMapComponentExtractor : public MapComponentMutator {
    public:
        FastMapComponentExtractor(Map* map) ;
        MapComponent* extract_component(Map::Facet* start) ;
    private:
        Map* map_ ;
        MapFacetAttribute<bool> visited_facet_ ;
        MapVertexAttribute<bool> visited_vertex_ ;
    } ;

    //________________________________________________________   

    /**
     * a Gluer that reorients facets if needed.
     */
    class CELLS_API ReorientGluer {
    public:
        typedef std::pair<int, int> IntPair ;
        ReorientGluer(Map* map) ;
        
        void glue(MapVertexAttribute<int>& vertex_id) ;

        bool facet_is_isolated(Map::Facet* f) ;
        void glue_reorient(Map::Halfedge* h1, Map::Halfedge* h2, bool reorient) ;
        void glue_reorient(
            Map::Halfedge* h1, Map::Halfedge* h2, 
            MapVertexAttribute<int>& vertex_id
        ) ;
        void glue_reorient(
            Map::Halfedge* h1, Map::Halfedge* h2, 
            MapVertexAttribute<IntPair>& vertex_id
        ) ;
        Map::Halfedge* find_halfedge(Map::Vertex* v1, Map::Vertex* v2, bool& inverted) ;
        void glue_reorient(
            Map::Vertex* v11, Map::Vertex* v12, 
            Map::Vertex* v21, Map::Vertex* v22
        ) ;
        void glue_reorient(
            Map::Halfedge* h1, Map::Halfedge* h2
        ) ;
    private:
        FastMapComponentExtractor extractor_ ;
        MapEditor editor_ ;
    } ;

}



//_________________________________________________________

#endif

