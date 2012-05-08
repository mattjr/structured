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
 * This chartizer has been implemented by Bruno Vallet 05/06/2003.
 * It is an implementation in Graphite of H. Hoppes' MCGIM with a few improvements.
 *
 */

#ifndef __OGF_CELLS_MAP_ALGOS_MAP_CHARTIZER__
#define __OGF_CELLS_MAP_ALGOS_MAP_CHARTIZER__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map_algos/map_components.h>
#include <deque>
#include <stack>
#include <queue>

namespace OGF {

//_________________________________________________________

    // contains charts' center and border
    // rem: charts are also defined by a facet attribute
    class CELLS_API Chart {
    public:
        Map::Facet* center ;
        std::vector<Map::Halfedge*> border ;
    } ;

    // used to compare facets by their distance from seeds in the priority list
    class CELLS_API FacetCmp {
    public :  
        FacetCmp() { 
            // ogf_assert(false) ;
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


    // Main class
    class CELLS_API MapChartizer {

    public:
        MapChartizer(Map* map) ;
        void apply() ;
        std::vector<Map::Halfedge*>& get_border() ;
        std::vector<Map::Halfedge*>& get_authalic_border() ;
        int get_connex_num() {return find_connex() ; }
        int build_connex() ;

        // tuning
        void set_num_charts(int num_charts) {
            num_charts_ = (num_charts < 1 ? 1 : num_charts) ; 
        }
        void set_geom_dist(bool geom_dist) {geom_dist_ = geom_dist ;}
        void set_cos(bool cos) {cos_ = cos ;}
        void set_angle(bool angle) {angle_ = angle ;}
        void set_curv(bool curv) {curv_ = curv ;}
        void set_epsilon(double epsilon) {epsilon_ = ::fabs(epsilon) ; }
        void set_chart_id_attribute_name(const std::string& name) {
            chart_index_name_ = name ;
        }

    protected:

        bool is_Aborder(Map::Halfedge* h) {
            return (h->is_border() || is_set_[h->facet()]) &&
                !(h->opposite()->is_border() || is_set_[h->opposite()->facet()]) ;
        } 
		
        void compute_cost() ;
        void reset_span() ;
        void create_chart(Map::Facet* f) ;
		
        int find_connex() ;
        void paint_connex(Map::Facet* f, int i) ;
        Map::Facet* new_connex_center() ;

        void compute_borders() ;
		
        Map::Facet* grow_charts() ;
        Map::Facet* center(int i) ;
		
        void set(Map::Facet* f) ;
        void set(Map::Facet* f, int i) ;

        // each edge has a cg(chart growing) cost and a cf(center finder) cost
        // as each edge has 2 halfedges, we will store a cost on each.
        inline Map::Halfedge* cg_edge(Map::Halfedge* h) {
            return (h > h->opposite() ? h : h->opposite()) ; 
        }
        
        inline Map::Halfedge* cf_edge(Map::Halfedge* h) {
            return (h > h->opposite() ? h->opposite() : h) ; 
        }
		
        // class members
		
        Map* map_ ;
        std::vector<Chart*> chart_ ;
        std::vector<Map::Halfedge*> global_border_ ;

        // Tuning
        // -------------------------------------------------------
        // wanted number of charts
        int num_charts_ ;

        // the distance function between 2 triangles is obtained by
        // multiplying the following factors if their flag is on :

        // geometric distance between triangle barycenters
        bool geom_dist_ ;

        // terms used to estimate the curvature
        // 1 - cos(angle between normals)
        bool cos_ ;

        // angle between normals
        bool angle_ ;

        // sum around shared edge's vertices - 2pi
        bool curv_ ;

        // we prevent the product of the curvature terms
        // from getting lower than a small threshold epsilon
        double epsilon_ ;

        // Attributes
        std::string chart_index_name_ ;
        MapHalfedgeAttribute<double> cost_ ;
        MapFacetAttribute<int> chart_index_ ;
        MapFacetAttribute<double> dist_ ;
        MapFacetAttribute<bool> is_set_ ;
        MapFacetAttribute<bool> is_old_center_ ;

        MapVertexAttribute<double> k1_ ;
        MapVertexAttribute<double> k2_ ;
		
        // Priority queue
        FacetCmp eval_ ;
		std::multiset<Map::Facet*,FacetCmp> S_;
    } ;
}
#endif

