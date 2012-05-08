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

#include <OGF/cells/map_algos/map_partition_smoother.h>

namespace OGF {

    SmoothVertex::SmoothVertex(Map::Vertex* v, MapFacetAttribute<int>& chart) {
        
        chart_id_  = -1 ;
        delta_len_ = 0.0 ;

        Map::Halfedge* h = v->halfedge() ;
        do {
            // Check if vertex is on border
            if(h->facet() == nil) {
                is_valid_ = false ; return ; 
            }
            // Check if vertex's star is adjacent to the border
            if(h->prev()->opposite()->facet() == nil) {
                is_valid_ = false ; return ; 
            }
            h = h->next_around_vertex() ;
        } while(h != v->halfedge()) ;


        // Check if we got two and only two different chart ids 
        // in star (and get them)

        int chart1 = chart[h->facet()] ;
        int nb_chart1 = 0 ;
        int chart2 = -1 ;
        int nb_chart2 = 0 ;

        h = h->next_around_vertex() ;
        do {
            if(chart[h->facet()] == chart1) {
                nb_chart1++ ;
            } else {
                if(chart2 == -1) {
                    chart2 = chart[h->facet()] ;
                }
                if(chart[h->facet()] != chart2) {
                    is_valid_ = false ; return ; 
                }
                if(chart[h->prev()->opposite()->facet()] != chart2) {
                    is_valid_ = false ; return ; 
                }
                nb_chart2++ ;
            }
            h = h->next_around_vertex() ;
        } while(h != v->halfedge()) ;

        if(chart2 == -1) {
            is_valid_ = false ; return ; 
        }

        if(nb_chart1 > nb_chart2) {
            chart_id_ = chart1 ;
        } else {
            chart_id_ = chart2 ;
        }

        // Check that the partition is manifold at vertex v
        int nb_change = 0 ;
        do {
            if(
                chart[h->next_around_vertex()->facet()] !=
                chart[h->facet()]
            ) {
                nb_change++ ;
            }
            h = h->next_around_vertex() ;
        } while(h != v->halfedge()) ;

        if(nb_change != 2) {
            is_valid_ = false ; return ; 
        }

        // Compute difference of frontier length
        delta_len_ = 0.0 ;
        do {
            if(chart[h->facet()] != chart[h->opposite()->facet()]) {
                delta_len_ += Geom::edge_length(h) ;
            }
            if(chart[h->facet()] != chart_id_) {
                delta_len_ -= Geom::edge_length(h->prev()) ;
            }
            h = h->next_around_vertex() ;
        } while(h != v->halfedge()) ;


        if(delta_len_ < 0) {
            is_valid_ = false ; return ; 
        }

        vertex_ = v ;
        is_valid_ = true ;
    }

    bool SmoothVertex::apply(
        MapFacetAttribute<int>& chart, 
        MapVertexAttribute<bool>& locked
    ) {
        if(locked[vertex_]) {
            return false ;
        }
        Map::Halfedge* h = vertex_->halfedge() ;
        do {
            chart[h->facet()] = chart_id_ ;
            locked[h->opposite()->vertex()] = true ;
            h = h->next_around_vertex() ;
        } while(h != vertex_->halfedge()) ;
        return true ;
    }

}
