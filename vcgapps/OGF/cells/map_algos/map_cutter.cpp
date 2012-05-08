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
 

#include <OGF/cells/map_algos/map_cutter.h>
#include <OGF/cells/map_algos/map_components.h>
#include <OGF/cells/map_algos/map_topology.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/cells/map/map_editor.h>
#include <OGF/math/geometry/principal_axes.h>
#include <OGF/basic/debug/logger.h>

#include <deque>
#include <stack>
#include <queue>

namespace OGF {

//_________________________________________________________

    void MapEdgeCutter::simplify_spanning_tree() {
        { FOR_EACH_HALFEDGE(Map, map_, it) {
            Map::Halfedge* h = it ;
            if(is_removable(h)) {
                while(h != nil) {
                    h = nibble(h) ;
                }
            }
        }}
    }

    void MapEdgeCutter::simplify_spanning_tree(MapComponent* comp) {
        { FOR_EACH_HALFEDGE(MapComponent, comp, it) {
            Map::Halfedge* h = it ;
            if(is_removable(h)) {
                while(h != nil) {
                    h = nibble(h) ;
                }
            }
        }}
    }


    void MapEdgeCutter::unglue_border() {
        std::vector<Map::Halfedge*> to_unglue ;
        { FOR_EACH_HALFEDGE(Map, map_, it) {
            if(on_border_[it]) {
                to_unglue.push_back(it) ;
            }
        }}

        unglue_border(to_unglue) ;
    }

    void MapEdgeCutter::unglue_border(MapComponent* comp) {
        std::vector<Map::Halfedge*> to_unglue ;

        {FOR_EACH_HALFEDGE(MapComponent, comp, it) {
            if(on_border_[it]) {
                to_unglue.push_back(it) ;
            }
        }}

        unglue_border(to_unglue) ;
    }

    void MapEdgeCutter::unglue_border(std::vector<Map::Halfedge*>& border) {
        int nb_edges_to_unglue = border.size() ;
        if(nb_edges_to_unglue == 0) {
            Logger::out("MapEdgeCutter") << "No edge to unglue"
                                         << std::endl ;
        } else {
            Logger::out("MapEdgeCutter") 
                << nb_edges_to_unglue << " edge(s) to unglue" << std::endl ;
        }

        MapEditor editor(map_) ;
        { for(
            std::vector<Map::Halfedge*>::iterator 
                it = border.begin(); it != border.end();
            it++
        ) {
            editor.unglue(*it,false) ;
        }}
    }

    bool MapEdgeCutter::is_removable(Map::Halfedge* h) {
        if(!on_border_[h]) {
            return false ;
        }
        int n1 = nb_border_edges_from(h->vertex()) ;
        int n2 = nb_border_edges_from(h->opposite()->vertex()) ;
        return (n1 == 1 || n2 == 1) ;
    }

    int MapEdgeCutter::nb_border_edges_from(Map::Vertex* v) {
        int result = 0 ;
        Map::Halfedge* h = v->halfedge() ;
        do {
            if(h->is_border() || on_border_[h]) {
                result++ ;
            }
            Map::Halfedge* opp = h->opposite() ;
            if(opp->is_border() || on_border_[opp]) {
                result++ ;
            }
            h = h->next_around_vertex() ;
        } while(h != v->halfedge()) ;
        return result ;
    }

    Map::Halfedge* MapEdgeCutter::nibble(Map::Halfedge* h) {
        // Find the vertex that the successor is incident to.
        Map::Vertex* v = nil ;
        if(nb_border_edges_from(h->vertex()) == 1) {
            v = h->opposite()->vertex() ;
        } else {
            v = h->vertex() ;
        }
        
        // Remove h from the cut tree
        on_border_[h] = false ;

        // Find the successor, i.e. an halfedge incident
        // to v and removable. 
        Map::Halfedge* cur = v->halfedge() ;
        do {
            if(is_removable(cur)) {
                return cur ;
            }
            if(is_removable(cur->opposite())) {
                return cur->opposite() ;
            }
            cur = cur->next_around_vertex() ;
        } while(cur != v->halfedge()) ;

        // If there is no successor, return nil.
        return nil ;
    }

//_________________________________________________________

    void DiscCutter::apply(Map::Facet* f) {
        Logger::out("DiscCutter") << "cutting surface..." << std::endl ;
        bind_attributes() ;
        create_spanning_tree(f) ;
        if(!spanning_tree_) {
            simplify_spanning_tree() ;
        }
        unglue_border() ;
        unbind_attributes() ;
    }

    void DiscCutter::apply(MapComponent* comp) {
        Logger::out("DiscCutter") << "cutting surface..." << std::endl ;
        bind_attributes() ;
        create_spanning_tree(comp->facets_begin()) ;
        if(!spanning_tree_) {
            simplify_spanning_tree(comp) ;
        }
        unglue_border(comp) ;
        unbind_attributes() ;
    }
    
    void DiscCutter::create_spanning_tree(Map::Facet* seed) {
        visit_facet(seed) ;
        while(!front_.empty()) {
            Map::Halfedge* h = front_.front() ;
            front_.pop_front() ;
            Map::Facet* f = h->opposite()->facet() ;
            if(f != nil && !visited_[f]) {
                on_border_[h] = false ;
                visit_facet(f) ;
            }
        }
    }

    void DiscCutter::visit_facet(Map::Facet* f) {
        if(f == nil || visited_[f]) {
            return ;
        }
        visited_[f] = true ;
        Map::Halfedge* h = f->halfedge() ;
        do {
            Map::Facet* neigh = h->opposite()->facet() ;
            if(neigh != nil && !visited_[neigh]) {
                on_border_[h] = true ;
                front_.push_back(h) ;
            }
            h = h->next() ;
        } while(h != f->halfedge()) ;
    }

//_________________________________________________________

    /**
     * Utility class for HardEdgeCutter, could be made public
     * if needed.
     * Measures the hardness of the edges, and stores it
     * in their value field (will be an Attribute in the future).
     */
    class HardEdgeDetector {
    public :
        HardEdgeDetector(Map* map, MapHalfedgeAttribute<double> value) 
            : map_(map), value_(value) { 
        }    
        /**
         * Approximates the surface by a cylinder that passes through
         * the edge and all vertices of neighbour triangles. Returns
         * the maximum distance between the left triangle and the
         * cylinder.
         */
        double distance_on_left(Map::Halfedge* h) ;
        void apply() ;
    private:
        Map* map_;
        MapHalfedgeAttribute<double>& value_ ;
    } ;
    

  
    double HardEdgeDetector::distance_on_left(Map::Halfedge* h) {

        /*
        if(
            Geom::facet_normal(h->facet()) * 
            Geom::facet_normal(h->opposite()->facet())
            > 0.999
        ) {
            return 0.0 ;
        }
        */

        // direction of h
        Vector3d dir_h = Geom::vector(h).normalized() ;
        
        // vertices of the neighbour triangles
        Point3d org = h->opposite()->vertex()->point();
        Point3d B = h->opposite()->next()->vertex()->point();
        Point3d A = h->next()->vertex()->point();
        
        // smallest vector from edge h to point A and B
        Vector3d edgeToA =  (dir_h ^ (A-org)) ^ dir_h;
        Vector3d edgeToB =  (dir_h ^ (B-org)) ^ dir_h;
        
        // define a 2d basis (u,v) where u*dir_ = 0, v*dir_ = 0 
        // and u^edgeToB=(0,0,0)
        Vector3d u = edgeToB; u.normalize();
        
        // A and B coordinates in (u,v)
        double xA = edgeToA*u;
        double yA = (edgeToA^u).norm();
        double xB = edgeToB.norm();
     

        if(::fabs(yA) < 1e-30) {
            Logger::warn("HardEdgeDetector")
                << "Possible numerical problem"
                << std::endl ;
            return 0.0 ;
        }
   
        // compute coordinate of C : the center of the circle that passes 
        // through A, B and org projected relative to h
        double xC = 0.5 * xB;
        double yC = (yA*yA + xA*xA - 2*xA*xC) / (2.0*yA);
        double radius = ::sqrt(xC*xC + yC*yC);
        
        // return the max distance between the circle and the segment
        return ::fabs(yC- radius);
    }

/*
    static double normal_angle(Map::Halfedge* h) {
        ogf_assert(!h->is_border_edge()) ;
        Vector3d n1 = Geom::facet_normal(h->facet()) ;
        Vector3d n2 = Geom::facet_normal(h->opposite()->facet()) ;
        return Geom::angle(n1,n2) ;
    }

    static bool edge_is_flat(Map::Halfedge* h) {
        double angle = normal_angle(h) ;
        return (angle > (M_PI - 0.001) ) ;
    }

    static bool edge_is_sharp(Map::Halfedge* h) {
        double angle = normal_angle(h) ;
        return (angle < (M_PI / 2.0 + 0.001)) ;
    }
*/

    void HardEdgeDetector::apply() {
        FOR_EACH_HALFEDGE(Map, map_, hi) {
            if (
                !hi->is_border() && 
                !hi->opposite()->is_border()
            ) {
                value_[hi] = ogf_max(
                    distance_on_left(hi),
                    distance_on_left(hi->opposite())
                ) ;
            } else {
                value_[hi] = 0 ;
            }
        }
    }

    //______________________________
  
    void HardEdgeCutter::bind_attributes() {
        MapEdgeCutter::bind_attributes() ;
        hardness_.bind(map_) ;
    }
    
    void HardEdgeCutter::unbind_attributes() {
        hardness_.unbind() ;
        MapEdgeCutter::unbind_attributes() ;
    }

    void HardEdgeCutter::apply() {
        bind_attributes() ;

        // Init hard edge index on halfedge AND 
        // global maximum hard edge index
        HardEdgeDetector detect(map_, hardness_);
        detect.apply();
        double max_val=0;
        {FOR_EACH_HALFEDGE(Map, map_, hi) {
            if (hardness_[hi] > max_val) {
                max_val = hardness_[hi] ;
            }
        }}

        if(::fabs(max_val) < 1e-10) {
            return ;
        }

        // Find a threshold :
        // threshold is 5 * tau where tau is minimal and assume 
        //             that 80% of halfedges hardedge indice is under tau
        double threshold=0;
        int density[1000];
        for (int i=0;i<1000;i++) {
            density[i]=0;
        }
        {FOR_EACH_HALFEDGE(Map, map_, hi) {
            density[int(hardness_[hi]*999 / max_val)]++;
        }}
    
        int accum_density=0;
        {for(int i=0; i<1000; i++) {
            accum_density += density[i];
            if (accum_density > 0.8 * map_->size_of_halfedges()){
                threshold = float(i+1)*10.0*max_val/1000.0;
                break;
            }
        }}

        threshold *= threshold_ ;

        // Tag the edges having a hardness larger than
        // the threshold, simplify the cut tree, and 
        // unglue the tagged edge, using MapEdgeCutter.
        { FOR_EACH_HALFEDGE(Map, map_, it) {
            if(
                !it->is_border() &&
                hardness_[it] > threshold &&
                !on_border_[it->opposite()]
            ) {
                on_border_[it] = true ;
            }
        }}
        simplify_spanning_tree() ;
        unglue_border() ;

        unbind_attributes() ;
    }

//_________________________________________________________

    /**
     * Utility class for MapSplitter, could be made public
     * if needed.
     * Makes it possible to find a pair of vertices
     * far away one from eachother.
     */
    class FurthestVertexFinder {
    public:
        enum Strategy { DEFAULT, ON_BORDER } ;
        FurthestVertexFinder(Map* map) : map_(map), component_(nil) { }
        FurthestVertexFinder(MapComponent* c) : map_(c->map()), component_(c) {
        }
        Map::Vertex* apply(Map::Vertex* seed, Strategy s = DEFAULT);

    private:
        class VertexCmp {
        public :
            VertexCmp() { 
                // Just to enable compile, should never be called.
                ogf_assert(false) ; 
            }
            VertexCmp(Attribute<Map::Vertex,double> &i) {
                distance_ = &i ;
            }
            bool operator()(
                const Map::Vertex* v0,const Map::Vertex* v1
            ) const {
                return (*distance_)[v0] < (*distance_)[v1] ;
            }
        private:
            Attribute<Map::Vertex,double> *distance_;
        } ;
        
        Attribute<Map::Vertex,double> distance_;
        Map* map_;
        MapComponent* component_ ;
    } ;
    

    Map::Vertex* FurthestVertexFinder::apply(
        Map::Vertex* seed, Strategy s 
    ) {
        Map::Vertex* result = nil;
        distance_.bind(map_->vertex_attribute_manager());
        VertexCmp eval(distance_);
        std::multiset< Map::Vertex*, VertexCmp > queue(eval);
	
        queue.insert(seed) ;

        if(component_ != nil) {
            FOR_EACH_VERTEX(MapComponent, component_, vi)  {
                distance_[vi] = Numeric::big_double ;
            }
        } else {
            FOR_EACH_VERTEX(Map, map_, vi)  {
                distance_[vi] = Numeric::big_double ;
            }
        }
        distance_[seed] = 0.0;
	
        // expand...
        while (!queue.empty()) {
            Map::Vertex* top = *(queue.begin());
			queue.erase(queue.begin());
            Map::Halfedge* cir = top->halfedge();   
            do {
                double new_value = Geom::edge_length(cir) + distance_[top];
                Map::Vertex* other = cir->opposite()->vertex();
                if (distance_[other] > new_value ) {
                    queue.erase(other);
					distance_[other] = new_value;
                    queue.insert(other);
                    if (s == DEFAULT || cir->is_border()){
                        result = other ;
                    }
                }
                cir =cir->next_around_vertex();
            } while (cir != top->halfedge());
        }
        distance_.unbind() ;
        return result;
    }

    //______________________________

    void MapSplitter::apply(Strategy strategy) {
        ogf_assert(component_ != nil) ;
        apply(component_, strategy) ;
    }

    //______________________________

    void MapSplitter::apply(MapComponent* comp, Strategy strategy) {
        last_strategy_ = strategy ;
        component_ = comp ;
        switch(strategy) {
        case AUTO: {
            MapComponentTopology topology(comp) ;
            if(topology.number_of_borders() == 0) {
                strategy = SHORTEST_PRINCIPAL_AXIS ;
            } else if(topology.is_cylinder()) {
                strategy = MAKE_DISC ;
            } else {
                strategy = FURTHEST_PAIR ;
            }
            apply(strategy) ;
        } break ;
        case FURTHEST_PAIR: {
            Map::Facet* seed1 = nil ;
            Map::Facet* seed2 = nil ;
            get_furthest_facet_pair(comp->facets_begin(), seed1, seed2) ; 
            if(seed1 == seed2) {
                get_furthest_facet_pair_along_principal_axis(
                    comp, seed1, seed2, 0, false
                ) ; 
                apply(seed1, seed2) ;
            } else {
                apply(seed1, seed2) ;
            }
        } break ;
        case SHORTEST_PRINCIPAL_AXIS: {
            Map::Facet* seed1 = nil ;
            Map::Facet* seed2 = nil ;
            get_furthest_facet_pair_along_principal_axis(
                comp, seed1, seed2, 2, true
            ) ; 
            apply(seed1, seed2) ;
        } break ;
        case MAKE_DISC: {
            last_seed_[0] = comp->facets_begin() ;
            last_seed_[1] = nil ;
            DiscCutter disc_cutter(map_) ;
            disc_cutter.apply(comp) ;
        } break ;
        case MAKE_SPANNING_TREE: {
            last_seed_[0] = comp->facets_begin() ;
            last_seed_[1] = nil ;
            DiscCutter disc_cutter(map_) ;
            disc_cutter.set_spanning_tree_mode(true) ;
            disc_cutter.apply(comp) ;
        } break ;
        }
    }
    
    void MapSplitter::apply(Map::Facet* start, Strategy strategy) {
        last_strategy_ = strategy ;
        switch(strategy) {
        case AUTO: 
        case SHORTEST_PRINCIPAL_AXIS:
        {
            MapComponentsExtractor extractor ;
            MapComponent_var comp = extractor.extract_component(map_,start) ;
            apply(comp, strategy) ;
        } break ;
        case FURTHEST_PAIR: {
            Map::Facet* seed1 = nil ;
            Map::Facet* seed2 = nil ;
            get_furthest_facet_pair(start, seed1, seed2) ; 
            if(seed1 == seed2) {
                MapComponentsExtractor extractor ;
                MapComponent_var comp = 
                    extractor.extract_component(map_,start) ;
                apply(comp, SHORTEST_PRINCIPAL_AXIS) ;
            } else {
                apply(seed1, seed2) ;
            }
        } break ;
        case MAKE_DISC: {
            last_seed_[0] = start ;
            last_seed_[1] = nil ;
            DiscCutter disc_cutter(map_) ;
            disc_cutter.apply(start) ;
        } break ;
        case MAKE_SPANNING_TREE: {
            last_seed_[0] = start ;
            last_seed_[1] = nil ;
            DiscCutter disc_cutter(map_) ;
            disc_cutter.set_spanning_tree_mode(true) ;
            disc_cutter.apply(start) ;
        } break ;
        }
    }

    void MapSplitter::apply(Map::Facet* f0, Map::Facet* f1) {

//        ogf_assert(f0 != f1) ;
        if(f0 == f1) {
            f1 = find_neighbour(f0) ;
            ogf_assert(f1 != nil) ;
        }

        last_seed_[0] = f0 ;
        last_seed_[1] = f1 ;

        Attribute<Map::Facet,double> 
            distance(map_->facet_attribute_manager());

        Attribute<Map::Facet,int> 
            num_patch(map_->facet_attribute_manager());

        FacetCmp eval(distance);
        std::multiset< Map::Facet*, FacetCmp > queue(eval);
	
        if(component_ == nil) {
            FOR_EACH_FACET(Map, map_,fi) {
                distance[fi] = Numeric::big_double ;
                num_patch[fi] = -1;
            }
        } else {
            FOR_EACH_FACET(MapComponent, component_, fi) {
                distance[fi] = Numeric::big_double ;
                num_patch[fi] = -1;
            }
        }

        distance[f0] = 0.0;      
        distance[f1] = 0.0;

        num_patch[f0]= 1;        
        num_patch[f1]= 2;

        queue.insert(f0);
        queue.insert(f1);

        // expand from seeds f0 and f1--------------------------------
        while (!queue.empty()) {
            Map::Facet* top = *(queue.begin());
            queue.erase(queue.begin());
            Map::Halfedge* cir = top->halfedge();
            do {
                if (!cir->opposite()->is_border()) {
                    Map::Facet* other = cir->opposite()->facet();
                    double new_value = (
                        Geom::facet_barycenter(top) - 
                        Geom::facet_barycenter(other)
                    ).norm() + distance[top];
                    if ( num_patch[other] == -1) {
                        queue.erase(other);
						distance[other] = new_value;
                        num_patch[other] = num_patch[top];
                        queue.insert(other);
                    }
                }
                cir =cir->next();
            } while (cir != top->halfedge());
        }
  

        // smooth frontiers between patches------------------------------      

        if(component_ != nil) {
            FOR_EACH_FACET(MapComponent, component_, fi) {
                if (num_patch[fi] != -1) {
                    Map::Halfedge* h = fi->halfedge();
                    if (
                        !h->opposite()->is_border() &&
                        !h->next()->opposite()->is_border() &&
                        !h->next()->next()->opposite()->is_border()
                    ){
                        int nb_1 = 0;
                        int nb_2 = 0;
                        do{
                            if(h->opposite()->facet() != nil) {
                                if (num_patch[h->opposite()->facet()] ==1) {
                                    nb_1++;
                                } else { 
                                    nb_2++;
                                }
                            }
                            h=h->next();
                        } while (h != fi->halfedge());
                        
                        if(nb_1 != 0 && nb_2 != 0) {
                            if ( nb_1 > nb_2) {
                                num_patch[fi]=1;
                            }
                            if ( nb_1 < nb_2) {
                                num_patch[fi]=2;
                            }
                        }
                    }
                }
            }
        } else {
            FOR_EACH_FACET(Map, map_, fi) {
                if (num_patch[fi] != -1) {
                    Map::Halfedge* h = fi->halfedge();
                    if (
                        !h->opposite()->is_border() &&
                        !h->next()->opposite()->is_border() &&
                        !h->next()->next()->opposite()->is_border()
                    ){
                        int nb_1 = 0;
                        int nb_2 = 0;
                        do{
                            if(h->opposite()->facet() != nil) {
                                if (num_patch[h->opposite()->facet()] ==1) {
                                    nb_1++;
                                } else { 
                                    nb_2++;
                                }
                            }
                            h=h->next();
                        } while (h != fi->halfedge());
                        
                        if(nb_1 != 0 && nb_2 != 0) {
                            if ( nb_1 > nb_2) {
                                num_patch[fi]=1;
                            }
                            if ( nb_1 < nb_2) {
                                num_patch[fi]=2;
                            }
                        }
                    }
                }
            }
        }
        
        // unglue frontiers----------------------------------------------

        std::vector<Map::Halfedge*> to_unglue ;
        if(component_ != nil) {
            FOR_EACH_HALFEDGE(MapComponent, component_, hi) {
                if (
                    !hi->is_border() &&
                    !hi->opposite()->is_border() &&
                    num_patch[hi->facet()] != 
                    num_patch[hi->opposite()->facet()]
                ) {
                    to_unglue.push_back(hi);
                }
            }
        } else {
            FOR_EACH_HALFEDGE(Map, map_, hi) {
                if (
                    !hi->is_border() &&
                    !hi->opposite()->is_border() &&
                    num_patch[hi->facet()] != 
                    num_patch[hi->opposite()->facet()]
                ) {
                    to_unglue.push_back(hi);
                }
            }
        }
      
        MapEditor editor(map_);
        { for(unsigned int i=0; i<to_unglue.size(); i++) {
            Map::Halfedge* h = to_unglue[i] ;
            if (
                !h->opposite()->is_border() && 
                !h->is_border()
            ) {
                editor.unglue(h, false) ;
            }
        }}

        if(auto_cut_cylinders_) {
            split_component_if_needed(f0) ;
            split_component_if_needed(f1) ;            
        }

    }

    void MapSplitter::split_component_if_needed(Map::Facet* start) {
        MapComponentsExtractor extractor ;
        MapComponent_var comp = extractor.extract_component(map_, start) ;
        MapComponentTopology topology(comp) ;
        DiscCutter disc_cutter(map_) ;
        if(topology.is_cylinder()) {
            Logger::out("MapSplitter")
                << "- cutting cylinder"
                << std::endl ;
            disc_cutter.apply(comp) ;
        } 
        // TODO: check for socks
    }


    void MapSplitter::get_furthest_facet_pair(
        Map::Facet* from, Map::Facet*& f0, Map::Facet*& f1
    ) {
        Map::Vertex* v0 = nil ;
        Map::Vertex* v1 = nil ;
        if(component_ == nil) {
            FurthestVertexFinder far_find(map_);
            v0 = far_find.apply(from->halfedge()->vertex());
            v1 = far_find.apply(v0);
        } else {
            FurthestVertexFinder far_find(component_);
            v0 = far_find.apply(from->halfedge()->vertex());
            v1 = far_find.apply(v0);
        }        

        if (!v0->halfedge()->is_border()) {
            f0 = v0->halfedge()->facet();
        } else {
            f0 = v0->halfedge()->opposite()->facet();
        }

        if (!v1->halfedge()->is_border()) {
            f1 = v1->halfedge()->facet();
        } else { 
            f1 = v1->halfedge()->opposite()->facet();
        }
    }

    void MapSplitter::get_furthest_facet_pair_along_principal_axis(
        MapComponent* comp, Map::Facet*& f0, Map::Facet*& f1,
        int axis, bool intersect
    ) {
        f0 = nil ;
        f1 = nil ;
        double min_z =  Numeric::big_float ;
        double max_z = -Numeric::big_float ;

        PrincipalAxes3d axes ;
        axes.begin_points() ;
        { FOR_EACH_VERTEX(MapComponent, comp, it) {
            const Point3d& p = it->point() ;
            axes.point(p) ;
        }}
        axes.end_points() ;
        Point3d center = axes.center() ;

        Vector3d Z = axes.axis(axis) ;
        
        // Try to find two facets intersecting the shortest axis
        if(intersect) {
            OrientedLine line(center, center + Z) ;
            { FOR_EACH_FACET(MapComponent, comp, it) {
                if(Geom::line_intersects_facet(line, it)) {
                    Point3d p = Geom::facet_barycenter(it) ;
                    double z = (p - center) * Z ;
                    if(z < min_z) {
                        min_z = z ;
                        f0 = it ;
                    }
                    if(z > max_z) {
                        max_z = z ;
                        f1 = it ;
                    }
                } 
            }}
        }
        
        // If these facets do not exist (for instance, in the case
        //  of a torus), find two facets far away one from eachother
        //  along the longest axis.
        Vector3d X = axes.axis(2 - axis) ;
        if(f0 == nil || f1 == nil || f0 == f1) {
            { FOR_EACH_FACET(MapComponent, comp, it) {
                Point3d p = Geom::facet_barycenter(it) ;
                double z = (p - center) * X ;
                if(z < min_z) {
                    min_z = z ;
                    f0 = it ;
                }
                if(z > max_z) {
                    max_z = z ;
                    f1 = it ;
                }
            }} 
        }
    }
    

    void MapSplitter::split_two_facets(Map::Facet* f0, Map::Facet* f1) {
        std::vector<Map::Halfedge*> to_unglue ;

        Map::Halfedge* h = f0->halfedge() ;
        do {
            if(h < h->opposite() && !h->is_border()) {
                to_unglue.push_back(h) ;
            }
            h = h->next() ;
        } while(h != f0->halfedge()) ;

        h = f1->halfedge() ;
        do {
            if(h < h->opposite() && !h->is_border()) {
                to_unglue.push_back(h) ;
            }
            h = h->next() ;
        } while(h != f1->halfedge()) ;


        MapEditor editor(map_) ;
        { for(
            std::vector<Map::Halfedge*>::iterator 
                it = to_unglue.begin(); it != to_unglue.end();
            it++
        ) {
            editor.unglue(*it,false) ;
        }}
    }

    Map::Facet* MapSplitter::find_neighbour(Map::Facet* f0) {
        Map::Halfedge* h = f0->halfedge() ;
        do {
            if(!h->opposite()->is_border()) {
                return h->opposite()->facet() ;
            }
            h = h->next() ;
        } while(h != f0->halfedge()) ;
        return nil ;
    }
//_________________________________________________________

}

