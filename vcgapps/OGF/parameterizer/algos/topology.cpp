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

#include <OGF/parameterizer/algos/topology.h>
#include <OGF/cells/map/map_attributes.h>
#include <set>
#include <stack>

namespace OGF {

    namespace Topo {

        //_________________________________ Graph ____________________________
        
        Graph::Halfedge* find_halfedge_between(Graph::Vertex* v1, Graph::Vertex* v2) {
            if(v1->halfedge() == nil || v2->halfedge() == nil) {
                return nil ;
            }
            Graph::Halfedge* h = v2->halfedge() ;
            do {
                if(h->opposite()->vertex() == v1) {
                    return h ;
                }
                h = h->next_around_vertex() ;
            } while(h != v2->halfedge()) ;
            return nil ;
        }


        //_________________________________ Map _____________________________


        Map::Halfedge* find_halfedge_between(Map::Vertex* v1, Map::Vertex* v2) {
            if(v1->halfedge() == nil || v2->halfedge() == nil) {
                return nil ;
            }
            Map::Halfedge* h = v2->halfedge() ;
            do {
                if(h->opposite()->vertex() == v1) {
                    return h ;
                }
                h = h->next_around_vertex() ;
            } while(h != v2->halfedge()) ;
            return nil ;
        }

        Map::Halfedge* find_halfedge_between(Map::Facet* f1, Map::Facet* f2) {
            Map::Halfedge* h = f1->halfedge() ;
            do {
                if(h->opposite()->facet() == f2) {
                    return h ;
                }
                h = h->next() ;
            } while(h != f1->halfedge()) ;
            return nil ;
        }

        bool facet_has_vertex(Map::Facet* f, Map::Vertex* v) {
            Map::Halfedge* h = f->halfedge() ;
            do {
                if(h->vertex() == v) {
                    return true ;
                }
                h = h->next() ;
            } while(h != f->halfedge()) ;
            return false ;
        }

        Map::Facet* find_facet_between(Map::Vertex* v1, Map::Vertex* v2) {
            Map::Halfedge* h = v1->halfedge() ;
            do {
                if(h->facet() != nil && facet_has_vertex(h->facet(), v2)) {
                    return h->facet() ;
                }
                h = h->next_around_vertex() ;
            } while(h != v1->halfedge()) ;
            return nil ;
        }

        //_________________________________ MapEmbedding ____________________        
        
        class MapStar {
        public:
            void clear() {
                vertices.clear() ; 
                edges.clear() ;
                facets.clear() ;
            }
            void insert(Map::Vertex* v) {
                vertices.insert(v) ;
            }
            void insert(Map::Halfedge* h) {
                edges.insert(h->edge_key()) ;
            }
            void insert(Map::Facet* f) {
                facets.insert(f) ;
            }
            std::set<Map::Vertex*> vertices ;
            std::set<Map::Halfedge*> edges ;
            std::set<Map::Facet*> facets ;
        } ;

        template <class T> inline void sets_intersection(
            std::set<T>& result, const std::set<T>& set1, const std::set<T>& set2
        ) {
            result.clear() ;
            if(set1.size() < set2.size()) {
                for(typename std::set<T>::const_iterator it = set1.begin(); it != set1.end(); it++) {
                    if(set2.find(*it) != set2.end()) {
                        result.insert(*it) ;
                    }
                }
            } else {
                for(typename std::set<T>::const_iterator it = set2.begin(); it != set2.end(); it++) {
                    if(set1.find(*it) != set1.end()) {
                        result.insert(*it) ;
                    }
                }
            }
        }
        
        static inline void stars_intersection(MapStar& result, const MapStar& s1, const MapStar& s2) {
            sets_intersection(result.vertices, s1.vertices, s2.vertices) ;
            sets_intersection(result.edges, s1.edges, s2.edges) ;
            sets_intersection(result.facets, s1.facets, s2.facets) ;            
        }

        static inline void get_star(Map::Vertex* v,  MapStar& star) {
            star.clear() ;
            star.insert(v) ;
            Map::Halfedge* h = v->halfedge() ;
            do {
                star.insert(h) ;
                if(h->facet() != nil) {
                    star.insert(h->facet()) ;
                }
                h = h->next_around_vertex() ;
            } while(h != v->halfedge()) ;
        }
        
        static inline void get_star(Map::Halfedge* h,  MapStar& star) {
            star.clear() ;
            star.insert(h) ;
            if(h->facet() != nil) {
                star.insert(h->facet()) ;
            }
            h = h->opposite() ;
            if(h->facet() != nil) {
                star.insert(h->facet()) ;
            }
        }

        static inline void get_star(Map::Facet* f,  MapStar& star) {
            star.clear() ;
            star.insert(f) ;
        }

        void get_star(const MapCellEmbedding& em, MapStar& star) {
            switch(em.dimension()) {
            case 0:
                get_star(em.vertex(), star) ;
                break ;
            case 1:
                get_star(em.halfedge(), star) ;
                break ;
            case 2:
                get_star(em.facet(), star) ;
                break ;
            default:
                ogf_assert_not_reached ;
            }
        }

        MapCellEmbedding segment_embedding(
            Map* map, const MapCellEmbedding& v1_em, const MapCellEmbedding& v2_em 
        ) {
            MapStar star1 ;
            get_star(v1_em, star1) ;
            MapStar star2 ;
            get_star(v2_em, star2) ;
            MapStar star_isect ;
            stars_intersection(star_isect, star1, star2) ;
            int nb_vertices = star_isect.vertices.size() ;
            int nb_edges  = star_isect.edges.size() ;
            int nb_facets = star_isect.facets.size() ;
            if(nb_vertices != 0) {
                if(nb_vertices != 1) {
                    std::cerr << "Argh !!! " << nb_vertices << " vertices in stars intersection" << std::endl ;
                    return MapCellEmbedding() ;
                }
                return MapCellEmbedding(map, *(star_isect.vertices.begin())) ;
            } else if(nb_edges != 0) {
                if(nb_edges != 1) {
                    std::cerr << "Argh !!! " << nb_edges << " edges in stars intersection" << std::endl ;
                    return MapCellEmbedding() ;
                }
                return MapCellEmbedding(map, *(star_isect.edges.begin())) ;
            } else if(nb_facets != 0) {
                if(nb_facets != 1) {
                    std::cerr << "Argh !!! " << nb_facets << " facets in stars intersection" << std::endl ;
                    return MapCellEmbedding() ;
                }
                return MapCellEmbedding(map, *(star_isect.facets.begin())) ;
            }
            // Empty stars intersection
            return MapCellEmbedding() ;
        }

    }
}
