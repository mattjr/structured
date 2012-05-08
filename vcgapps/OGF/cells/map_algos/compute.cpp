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
 

#include <OGF/cells/map_algos/compute.h>
#include <OGF/basic/attributes/attribute.h>
#include <OGF/basic/debug/assert.h>

namespace OGF {

//_________________________________________________________

    void transfer_attribute(
        Map* map, 
        Attribute<Map::Vertex,double>& from,
        Attribute<Map::Halfedge,double>& to
    ) {
        ogf_assert(
            from.attribute_manager() == map->vertex_attribute_manager()
        ) ;

        ogf_assert(
            to.attribute_manager() == map->halfedge_attribute_manager()
        ) ;

        FOR_EACH_HALFEDGE(Map,map,it) {
            to[it] = 0.5 * (
                from[it->vertex()] + from[it->opposite()->vertex()]
            ) ;
        } 
    }

    void transfer_attribute(
        Map* map, 
        Attribute<Map::Vertex,double>& from,
        Attribute<Map::Facet,double>& to
    ) {
        ogf_assert(
            from.attribute_manager() == map->vertex_attribute_manager()
        ) ;

        ogf_assert(
            to.attribute_manager() == map->facet_attribute_manager()
        ) ;


        FOR_EACH_FACET(Map,map,it) {
            to[it] = 0 ;
            int count = 0 ;
            Map::Halfedge* h = it->halfedge() ;
            do {
                to[it] += from[h->vertex()] ;
                h = h->next() ;
                count++ ;
            } while(h != it->halfedge()) ;
            to[it] /= double(count) ;
        }
    }

    void transfer_attribute(
        Map* map,
        Attribute<Map::Halfedge,double>& from,
        Attribute<Map::Vertex,double>& to
    ) {
        ogf_assert(
            from.attribute_manager() == map->halfedge_attribute_manager()
        ) ;

        ogf_assert(
            to.attribute_manager() == map->vertex_attribute_manager()
        ) ;


        FOR_EACH_VERTEX(Map,map,it) {
            to[it] = 0 ;
            int count = 0 ;
            Map::Halfedge* h = it->halfedge() ;
            do {
                to[it] += from[h] ;
                h = h->next_around_vertex() ;
                count++ ;
            } while(h != it->halfedge()) ;
            to[it] /= double(count) ;
        }
    }

    void transfer_attribute(
        Map* map,
        Attribute<Map::Halfedge,double>& from,
        Attribute<Map::Facet,double>& to
    ) {
        ogf_assert(
            from.attribute_manager() == map->halfedge_attribute_manager()
        ) ;
        
        ogf_assert(
            to.attribute_manager() == map->facet_attribute_manager()
        ) ;


        FOR_EACH_FACET(Map,map,it) {
            to[it] = 0 ;
            int count = 0 ;
            Map::Halfedge* h = it->halfedge() ;
            do {
                to[it] += from[h] ;
                h = h->next() ;
                count++ ;
            } while(h != it->halfedge()) ;
            to[it] /= double(count) ;
        }
    }

    void transfer_attribute(
        Map* map,
        Attribute<Map::Facet,double>& from,
        Attribute<Map::Vertex,double>& to
    ) {
        ogf_assert(
            from.attribute_manager() == map->facet_attribute_manager()
        ) ;

        ogf_assert(
            to.attribute_manager() == map->vertex_attribute_manager()
        ) ;


        FOR_EACH_VERTEX(Map,map,it) {
            to[it] = 0 ;
            int count = 0 ;
            Map::Halfedge* h = it->halfedge() ;
            do {
                if(h->facet() != nil) {
                    to[it] += from[h->facet()] ;
                    count++ ;
                }
                h = h->next_around_vertex() ;
            } while(h != it->halfedge()) ;
            to[it] /= double(count) ;
        }
    }

    void transfer_attribute(
        Map* map,
        Attribute<Map::Facet,double>& from,
        Attribute<Map::Halfedge,double>& to
    ) {
        ogf_assert(
            from.attribute_manager() == map->facet_attribute_manager()
        ) ;

        ogf_assert(
            to.attribute_manager() == map->halfedge_attribute_manager()
        ) ;


        FOR_EACH_EDGE(Map,map,it) {
            int count = 0 ;
            to[it] = 0 ;
            if(it->facet() != nil) {
                to[it] += from[it->facet()] ;
                count++ ;
            }
            if(it->opposite()->facet() != nil) {
                to[it] += from[it->opposite()->facet()] ;
                count++ ;
            }
            to[it] /= double(count) ;
            to[it->opposite()] = to[it] ;
        }
    }

//_________________________________________________________


    double min_attribute(
        Map* map, Attribute<Map::Vertex,double>& value
    ) {
        ogf_assert(
            value.attribute_manager() == map->vertex_attribute_manager()
        ) ;
        double result = Numeric::big_double ;
        FOR_EACH_VERTEX(Map,map, it) {
            result = ogf_min(result,value[it]) ;
        }
        return result ;
    }
    
    double max_attribute(
        Map* map, Attribute<Map::Vertex,double>& value
    ) {
        ogf_assert(
            value.attribute_manager() == map->vertex_attribute_manager()
        ) ;
        double result = -Numeric::big_double ;
        FOR_EACH_VERTEX(Map,map, it) {
            result = ogf_max(result,value[it]) ;
        }
        return result ;
    }

    double min_attribute(
        Map* map, Attribute<Map::Halfedge,double>& value
    ) {
        ogf_assert(
            value.attribute_manager() == map->halfedge_attribute_manager()
        ) ;
        double result = Numeric::big_double ;
        FOR_EACH_HALFEDGE(Map, map, it) {
            result = ogf_min(result,value[it]) ;
        }
        return result ;
    }

    double max_attribute(
        Map* map, Attribute<Map::Halfedge,double>& value
    ) {
        ogf_assert(
            value.attribute_manager() == map->halfedge_attribute_manager()
        ) ;
        double result = -Numeric::big_double ;
        FOR_EACH_HALFEDGE(Map, map, it) {
            result = ogf_max(result,value[it]) ;
        }
        return result ;
    }
    
    double min_attribute(
        Map* map, Attribute<Map::Facet,double>& value
    ) {
        ogf_assert(
            value.attribute_manager() == map->facet_attribute_manager()
        ) ;
        double result = Numeric::big_double ;
        FOR_EACH_FACET(Map, map, it) {
            result = ogf_min(result,value[it]) ;
        }
        return result ;
    }
    
    double max_attribute(
        Map* map, Attribute<Map::Facet,double>& value
    ) {
        ogf_assert(
            value.attribute_manager() == map->facet_attribute_manager()
        ) ;
        double result = -Numeric::big_double ;
        FOR_EACH_FACET(Map, map, it) {
            result = ogf_max(result,value[it]) ;
        }
        return result ;
    }

//_________________________________________________________

}

