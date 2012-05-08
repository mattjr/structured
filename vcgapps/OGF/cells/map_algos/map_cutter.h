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
 

#ifndef __OGF_CELLS_MAP_ALGOS_MAP_CUTTER__
#define __OGF_CELLS_MAP_ALGOS_MAP_CUTTER__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map_algos/map_components.h>
#include <deque>

namespace OGF {

    class MapComponent ;

//_________________________________________________________

    /**
     * Base class for algorithms that unglue edges. Provides
     * functions to mark edges, to simplify the tree of marked
     * edges, and to unglue the set of marked edges.
     */
    class CELLS_API MapEdgeCutter {
    public:
        MapEdgeCutter(Map* map) : map_(map) { }

    protected:

        /**
         * Should be called at the beginning of the apply()
         * function of derived classes.
         */
        void bind_attributes() {
            on_border_.bind(map_->halfedge_attribute_manager()) ;
        }

        /**
         * Should be called at the end of the apply()
         * function of derived classes.
         */
        void unbind_attributes() {
            on_border_.unbind() ;
        }

        /** 
         * Removes all dangling edges.
         */
        void simplify_spanning_tree() ;

        /** 
         * Removes all dangling edges. This faster version can be
         * used if the connected component is known.
         */
        void simplify_spanning_tree(MapComponent* comp) ;

        /**
         * Unglues all the marked edges.
         */
        void unglue_border() ;

        /**
         * Unglues all the marked edges. This faster version can be
         * used if the connected component is known.
         */
        void unglue_border(MapComponent* comp) ;

        /**
         * used by the two previous ones.
         */
        void unglue_border(std::vector<Map::Halfedge*>& border) ;

        /**
         * Checks whether h can be removed from the cut graph,
         * i.e. whether h is a dangling edge
         */
        bool is_removable(Map::Halfedge* h) ;

        /**
         * Returns the number of edges on the border plus
         * the number of edges marked as to be cut, incident
         * to a given vertex.
         */
        int nb_border_edges_from(Map::Vertex* v) ;

        /**
         * Removes a dangling edge and returns the newly created
         * dangling edge (or nil if none)
         */
        Map::Halfedge* nibble(Map::Halfedge* h) ;

    protected:
        Map* map_ ;
        Attribute<Map::Halfedge, bool> on_border_ ;
    } ;

    //___________________________________________________________________

    /**
     * Transforms a connected component into a topological disc
     * by cutting (ungluing) some edges.
     */
    class CELLS_API DiscCutter : public MapEdgeCutter {
    public:
        DiscCutter(Map* map) : MapEdgeCutter(map), spanning_tree_(false) { }

        /** 
         * Transforms the connected component adjacent to the specified
         * facet into a topological disc.
         */
        void apply(Map::Facet* from) ;

        /**
         * Transforms the specified connected component into a topological
         * disc.
         */
        void apply(MapComponent* comp) ;

        void set_spanning_tree_mode(bool x) {
            spanning_tree_ = x ;
        }

    protected:

        void bind_attributes() {
            MapEdgeCutter::bind_attributes() ;
            visited_.bind(map_->facet_attribute_manager()) ;
        }

        void unbind_attributes() {
            visited_.unbind() ;
            MapEdgeCutter::unbind_attributes() ;
        }

        /**
         * Appends the specified facet to the topological disc
         * under construction.
         */
        void visit_facet(Map::Facet* f) ;

        /** 
         * Computes a cut graph transforming the connected component
         * incident to the seed facet into a topological disc. 
         * The resulting disc is constructed by the 'glue' operation only.
         */
        void create_spanning_tree(Map::Facet* seed) ;

    private:
        bool spanning_tree_ ;
        Attribute<Map::Facet, bool> visited_ ;
        std::deque<Map::Halfedge*> front_ ;
    } ;

    //___________________________________________________________________

    /**
     * Cuts (unglues) the hard edges of a Map. 
     */
    class CELLS_API HardEdgeCutter : public MapEdgeCutter {
    public:
        HardEdgeCutter(Map* map) : MapEdgeCutter(map), threshold_(1.0) { }
        void apply() ;
        double get_threshold() const { return threshold_ ; }
        void set_threshold(double x) { threshold_ = x ; }
    protected:
        void bind_attributes() ;
        void unbind_attributes() ;
    private:
        MapHalfedgeAttribute<double> hardness_ ;
        double threshold_ ;
    } ;

    //___________________________________________________________________

    /**
     * Splits a connected component using different strategies.
     */
    // TODO: add a multi-seed method.
    class CELLS_API MapSplitter {
        // Note: this class does not need to inherit from MapEdgeCutter,
        // since the cut tree does not need to be simplified (the
        // 'component growing' algorithm does not generate dangling 
        // edges)
    public:
        enum Strategy { 
            AUTO, FURTHEST_PAIR, SHORTEST_PRINCIPAL_AXIS, MAKE_DISC, MAKE_SPANNING_TREE
        } ;
        MapSplitter(Map* map) : 
            map_(map), component_(nil), auto_cut_cylinders_(false) {
        }

        MapSplitter(MapComponent* c) :
            map_(c->map()), component_(c), auto_cut_cylinders_(false) {
        }

        bool auto_cut_cylinders() const { return auto_cut_cylinders_ ; }
        void set_auto_cut_cylinders(bool b) {
            auto_cut_cylinders_ = b ;
        }

        void apply(Strategy strategy) ;
        void apply(MapComponent* comp, Strategy strategy) ;
        void apply(Map::Facet* start, Strategy strategy) ;
        void apply(Map::Facet* seed1, Map::Facet* seed2) ;

        /**
         *  After calling apply, returns the seeds used
         * by the algorithm.
         */
        Map::Facet* last_seed(unsigned int i) const {
            ogf_assert(i < 2) ;
            return last_seed_[i] ;
        }

        /**
         * Can be used to know which strategy has
         * been used by the algorithm when AUTO 
         * has been specified.
         */
        Strategy last_strategy() const {
            return last_strategy_ ;
        }

    protected:

        void split_two_facets(Map::Facet* f0, Map::Facet* f1) ;
        Map::Facet* find_neighbour(Map::Facet* f0) ;

        void get_furthest_facet_pair(
            Map::Facet* from, Map::Facet*& f0, Map::Facet*& f1
        ) ;

        /**
         * @param axis in {0,1,2}, where 0 is the longest, and
         *   2 the shortest axis.
         * @param intersect if set, selects the facets intersecting
         *   the specified axis.
         */
        void get_furthest_facet_pair_along_principal_axis(
            MapComponent* comp, Map::Facet*& f0, Map::Facet*& f1,
            int axis, bool intersect
        ) ;

        void split_component_if_needed(Map::Facet* start) ;

    private:

        class FacetCmp {
        public :  
            FacetCmp() { 
                ogf_assert(false) ;
                // just to enable compile, should not be called.
            }

            FacetCmp(Attribute<Map::Facet,double>& distance) {
                distance_ = &distance ;
            }

            bool operator()(const Map::Facet* f0, const Map::Facet* f1) const {
                return (*distance_)[f0] < (*distance_)[f1] ;                
            }

        private:
            Attribute<Map::Facet,double> *distance_;
        } ;

        Map* map_ ;
        MapComponent* component_ ;
        bool auto_cut_cylinders_ ;
        Map::Facet* last_seed_[2] ;
        Strategy last_strategy_ ;
    } ;

//_______________________________________________________________________

}
#endif

