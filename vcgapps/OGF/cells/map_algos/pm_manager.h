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
 

#ifndef __CELLS_MAP_MAP_ALGOS_PM_MANAGER__
#define __CELLS_MAP_MAP_ALGOS_PM_MANAGER__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_attributes.h>

namespace OGF {

//_________________________________________________________

    class Progress ;

    /**
     * Stores all the information relative to a Vertex Split
     * operation. This class is used internally by the Progressive
     * Mesh representation, client code should not need to use it.
     */
    class CELLS_API VertexSplit {
    public :
        VertexSplit() {
            new_block_ = nil ;
            left_position_ = nil ;
            right_position_ = nil ;
        }

        Map::Halfedge* left_position()  { return left_position_ ; }
        Map::Halfedge* right_position() { return right_position_ ; }
        Map::Halfedge* new_block() { return new_block_; }

        bool is_valid() const {
            return (new_block_ != nil) && (left_position_ != nil) ;
            // Note: right_position_ can be nil when the operation
            // is applied on the border.
        }

    private:
        Map::Halfedge* new_block_;
        Map::Halfedge* left_position_;
        Map::Halfedge* right_position_;

        friend class PMManager;
        friend class PMDecimator;
    } ;

//_________________________________________________________

    /**
     * Constructs and stores a Progressive Mesh data
     * structure.
     */

    class CELLS_API PMManager : public MapMutator {
    public :
        PMManager(Map* map) ;

        bool init();

        /**
         * applies the next vertex split operation of the list
         * and returns it. Returns an invalid VertexSplit when
         * the finest level has been reached.
         */
        VertexSplit refine();

        /**
         * applies the next edge collapse operation of the list
         * and returns it. Returns an invalid VertexSplit when
         * the coarsest level has been reached.
         */
        VertexSplit coarsen();

        /**
         * returns the next vertex split operation of the list,
         * without applying it.
         */
        VertexSplit next_refinement();

        /**
         * returns the next edge collapse operation of the list,
         * without applying it.
         */
        VertexSplit next_coarsening();

        int size() const { return int(vsplit_.size()) ; }
        
        void set_level(int level) ;
        int level() const { return cur_ ; }

        Progress* get_progress()    { return progress_ ; }
        void set_progress(Progress* p) { progress_ = p ; }

        void set_decimate_border(bool b) { decim_border_ = b ; }
        bool get_decimate_border() const { return decim_border_ ; }

    protected:

        bool can_collapse(Halfedge* h);
        bool can_undo_split(VertexSplit split);
        
        int vertex_orbit_size(Vertex* v);
        void undo_split(VertexSplit split);
        
        void apply_split(VertexSplit split);
        bool is_active(Halfedge* h);
        
        void check_surface_integrity();

        int get_vertex_kind(Map::Vertex* v) ;
        void get_neighbors(Map::Vertex* v, int& neigh1, int& neigh2) ;

    private:
        int cur_;
        bool decim_border_ ;
        std::vector<VertexSplit> vsplit_;
        Progress* progress_ ;
        friend class PMDecimator ;
        MapFacetAttribute<int> chart_id_ ;
        MapVertexAttribute<int> vertex_kind_ ;
        MapVertexAttribute<bool> pm_vertex_lock_ ;
  } ;

//_________________________________________________________

}
#endif

