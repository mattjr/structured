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

#ifndef __SORTED_ROBUST_ITERATOR_H__
#define __SORTED_ROBUST_ITERATOR_H__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map.h>
#include <OGF/basic/attributes/attribute.h>

#include <set>

namespace OGF {

//_________________________________________________________

   /** Deprecated, please use MapCellHeap instead */
    template<class CELL>
    class MapSortedDynamicIterator : public MapCombelObserver<CELL> {
    public :
        
        MapSortedDynamicIterator(
            Map *s,                       // surface to iterate on
            Attribute<CELL,double> &cost, // how to sort
            double threshold = 1e20       // ignore elements larger than
        ) : MapCombelObserver<CELL>(s) {
            cost_ = cost;
            threshold_=threshold;
            top_has_changed_ = false;
        } 
    
        ~MapSortedDynamicIterator() { }
    
        // access to current elt
        CELL* operator*() {
            ogf_parano_assert(!heap_.empty()); 
// TODO: ask Nico, this often fails in shortest_path            
//            ogf_parano_assert(!top_has_changed_);
            return heap_[0] ;
        }

        CELL* operator->() {
            ogf_parano_assert(!heap_.empty());
// TODO: ask Nico, this often fails in shortest_path            
//            ogf_parano_assert(!top_has_changed_);
            return heap_[0] ;
        }
        
    
        // iter
        MapSortedDynamicIterator& operator++() {
            if(!heap_.empty()) {
                if (!top_has_changed_) {
                    remove(heap_[0]) ; 
                }
                top_has_changed_ = false;  
            }
            return *this;
        }

        MapSortedDynamicIterator& operator++(int houba) {
            if(!heap_.empty()) {
                if (!top_has_changed_) {
                    remove(heap_[0]) ;
                }
                top_has_changed_ = false;
            }
            return *this;
        }

        bool finish() { return heap_.empty();  }

        // _________________ modify _____________________________

        // modify cost....
        void update_cost(CELL *v) {
            ogf_parano_assert (contains(v));
            if (threshold_ > cost_[v]) {
                place_up_in_heap(v);
                place_down_in_heap(v);
            } else {
                remove(v) ;
            }
        }
        
        // modify set of elts....
        void init_with_all_surface_vertices();
 

        bool contains(CELL *v) { return pos_[v] != -1; }
        void insert(CELL* v) {
            if (threshold_ > cost_[v]) {
                pos_[v] = int(heap_.size());
                heap_.push_back(v);
                place_up_in_heap(v);
            } 
        }
        
        // ________________ MapCombelObserver overloads ______________

        // only called from surface
        virtual void add(CELL* v) {
            pos_[v] = -1;
        }

        virtual void remove(CELL* v) {
            if(contains(v)) {
                int pos = pos_[v]; 
                
                if (pos==0) {
                    top_has_changed_=true;
                }

                if (pos != int(heap_.size())-1) {
                    switch_elements(pos,int(heap_.size())-1);
                }
     
                pos_[heap_.back()] = -1;
                
                heap_.pop_back();
                
                if (pos < int(heap_.size())) {
                    update_cost(heap_[pos]);
                }
            }
        }
        
    private:    
        void switch_elements(int i,int j) {
            CELL* tmp;
            tmp = heap_[i];
            heap_[i] =  heap_[j];
            heap_[j] =  tmp;
            pos_[heap_[i] ]=i;
            pos_[heap_[j] ]=j;
        }

        inline long int father(long int i) const {
            return  (i+1)/2 - 1 ;
        }
        
        inline long int child0(long int i) const {
            return 2*(i+1)-1;
        }
        
        inline long int child1(long int i) const {
            return 2*(i+1)+1-1;
        }
    
        inline void place_up_in_heap(CELL *v) {
            if (pos_[v]!=0 && cost_[v] < cost_[heap_[father(pos_[v])] ]) {
                switch_elements(pos_[v],father(pos_[v]));
                if(pos_[v] == 0) {
                    top_has_changed_ = true ;
                }
                place_up_in_heap(v);
            }
        }

        inline void place_down_in_heap(CELL *v) {
            ogf_parano_assert(pos_[v] < int(heap_.size()));
            ogf_parano_assert(pos_[v] >=0 );
            if (
                (child0(pos_[v]) < int(heap_.size())
                    && cost_[heap_[child0(pos_[v])]] < cost_[v])
                || 
                (child1(pos_[v]) < int(heap_.size())
                    && cost_[heap_[child1(pos_[v])]] < cost_[v])
            ) {
                if (child1(pos_[v]) == int(heap_.size())
                    || 
                    cost_[heap_[child0(pos_[v])]] < 
                    cost_[heap_[child1(pos_[v])]]
                ) {
                    switch_elements(pos_[v],child0(pos_[v]));
                    place_down_in_heap(v);
                } else {
                    switch_elements(pos_[v],child1(pos_[v]));
                    place_down_in_heap(v);
                }
            }
        }
        
    private:// protect against copy
        
        MapSortedDynamicIterator(const MapSortedDynamicIterator& rhs) ;
        MapSortedDynamicIterator& operator=(
            const MapSortedDynamicIterator& rhs
        );
        
    protected :
        Attribute<CELL,int> pos_;      // -1 means not in heap
        Attribute<CELL,double> cost_;
        std::vector<CELL*> heap_;
        bool top_has_changed_;
        double threshold_;
    } ;


    //---------------------------------------------------------------------

    class MapSortedDynamicVertexIterator : 
        public MapSortedDynamicIterator<Map::Vertex> {
    public :
        MapSortedDynamicVertexIterator(
            Map* s,
            Attribute<Map::Vertex,double>& cost,
            double threshold = 10e20
        ) : MapSortedDynamicIterator<Map::Vertex>(s,cost,threshold){
            pos_.bind(s->vertex_attribute_manager());
            FOR_EACH_VERTEX(Map, map(), vi) {
                pos_[vi] = -1;
            }
        }  

        void init_with_all_surface_vertices() {
            FOR_EACH_VERTEX(Map, map(), vi) {
                insert(vi);
            }
            top_has_changed_ = false;  
        }
    } ;

    //---------------------------------------------------------------------

    class MapSortedDynamicHalfedgeIterator : 
        public MapSortedDynamicIterator<Map::Halfedge> {
    public :
        MapSortedDynamicHalfedgeIterator(
            Map* s,
            Attribute<Map::Halfedge,double>& cost,
            double threshold
        ) : MapSortedDynamicIterator<Map::Halfedge>(s,cost,threshold){
            pos_.bind(s->halfedge_attribute_manager());
            FOR_EACH_HALFEDGE(Map, map(), hi) {
                pos_[hi] = -1;
            }
        }  

        void init_with_all_surface_halfedges(){
            FOR_EACH_HALFEDGE(Map, map(), hi) {
                insert(hi) ;
            }      
            top_has_changed_ = false;  
        }
    } ;

    //---------------------------------------------------------------------

    class MapSortedDynamicFacetIterator : 
        public MapSortedDynamicIterator<Map::Facet> {
    public :
        MapSortedDynamicFacetIterator(
            Map* s,
            Attribute<Map::Facet,double>& cost,
            double threshold
        ) : MapSortedDynamicIterator<Map::Facet>(s,cost,threshold){
            pos_.bind(s->facet_attribute_manager());
            FOR_EACH_FACET(Map, map(), fi) {
                pos_[fi] = -1;
            }
        }  

        void init_with_all_surface_facets(){
            FOR_EACH_FACET(Map, map(), fi) {
                insert(fi) ;
            }      
            top_has_changed_ = false;  
        }
    } ;

//_________________________________________________________

}

#endif
