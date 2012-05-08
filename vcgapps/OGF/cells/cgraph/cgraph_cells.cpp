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
 

#include <OGF/cells/cgraph/cgraph_cells.h>
#include <OGF/basic/debug/logger.h>

#include <stdio.h>

namespace OGF {

    namespace CGraphTypes {

//_________________________________________________________

        MetaCell::MetaCell() {
        }

        MetaCell::~MetaCell() {
            map_.clear() ;
            //   Make sure the attributes are destroyed before
            // the attribute managers. 
            vertex_id_.unbind() ;
            halfedge_id_.unbind() ;
            facet_id_.unbind() ;
        }

        bool MetaCell::check_map() const {
            Logger::out("MetaCell") << "nb facets = " 
                                     << map_.size_of_facets() << std::endl ;
            Logger::out("MetaCell") << "nb vertices = " 
                                     << map_.size_of_vertices() << std::endl ;
            Logger::out("MetaCell") << "nb halfedges = " 
                                     << map_.size_of_halfedges() << std::endl ;
            for(
                Map::Halfedge_const_iterator it = 
                    map_.halfedges_begin() ;
                it != map_.halfedges_end(); it++
            ) {
                if(it-> is_border()) {
                    Logger::err("MetaCell") 
                        << "Dammit !! an open cell" << std::endl ;
                    return false ;
                }
            }
            return true ;
        }

        void MetaCell::initialize_from_map() {
            if(!check_map()) {
                exit(-1) ;
            }

            nb_vertices_ = map_.size_of_vertices() ;
            nb_edges_ = map_.size_of_halfedges() / 2 ;
            nb_facets_ = map_.size_of_facets() ;

            enumerate_cells() ;

            // initialize (edge -> vertices) array
            e2v_.allocate(nb_edges_, 2) ;
            {FOR_EACH_HALFEDGE(Map,&map_,it) {
                int edge = halfedge_id_[it] ;
                int v1 = vertex_id_[it-> vertex()] ;
                int v2 = vertex_id_[it-> opposite()-> vertex()] ;
                e2v_(edge, 0) = v1 ;
                e2v_(edge, 1) = v2 ;
            } }

            // initialize (vertex -> edge) array
            v2e_.allocate(nb_vertices_, nb_vertices_) ;
            v2e_.set_all(-1) ;
            {FOR_EACH_HALFEDGE(Map,&map_,it) {
                int edge = halfedge_id_[it] ;
                int v1 = vertex_id_[it-> vertex()] ;
                int v2 = vertex_id_[it-> opposite()-> vertex()] ;
                v2e_(v1, v2) = edge ;
            } }     

            // initialize (face -> nb vertices) array
            // and compute max_facet_nb_vertices_
            f2n_.allocate(nb_facets_) ;
            max_facet_nb_vertices_ = 0 ;
            {FOR_EACH_FACET(Map,&map_,it) {
                int facet_nb_vertices = it-> nb_vertices() ;
                max_facet_nb_vertices_ = ogf_max(
                    max_facet_nb_vertices_, facet_nb_vertices
                ) ;
                f2n_(facet_id_[it]) = facet_nb_vertices ;
            } }     

            // initialize (face, vertex in face -> vertex) array
            fv2v_.allocate(nb_facets_, max_facet_nb_vertices_) ;
            fv2v_.set_all(-1) ;
            {FOR_EACH_FACET(Map,&map_,it) {
                Map::Halfedge* jt = it->halfedge() ;
                int vertex_in_facet = 0 ;
                do {
                    fv2v_(facet_id_[it],vertex_in_facet) = 
                        vertex_id_[jt->vertex()];
                    jt = jt->next() ;
                    vertex_in_facet++ ;
                } while(jt != it->halfedge()) ;
            } }        

            // initialize (face, vertex in cell -> prev vrtx,next vrtx) array
            fv2u_.allocate(nb_facets_, nb_vertices_, 2) ;
            fv2u_.set_all(-1) ;
            {FOR_EACH_HALFEDGE(Map,&map_,it) {
                int f = facet_id_[it-> facet()] ;
                int v = vertex_id_[it-> vertex()] ;
                int prev = vertex_id_[it-> prev()-> vertex()] ;
                int next = vertex_id_[it-> next()-> vertex()] ;
                fv2u_(f,v,0) = prev ;
                fv2u_(f,v,1) = next ;
            } }     

            compute_configs() ;

            vertex_id_.unbind() ;
            halfedge_id_.unbind() ;
            facet_id_.unbind() ;
        }

        void MetaCell::unmark_all_vertices() {
            {FOR_EACH_VERTEX(Map,&map_,v) {
                vertex_id_[v] = 0 ;
            }}
        }

        void MetaCell::enumerate_cells() {

            vertex_id_.bind(map_.vertex_attribute_manager()) ;
            halfedge_id_.bind(map_.halfedge_attribute_manager()) ;
            facet_id_.bind(map_.facet_attribute_manager()) ;

            vertices_.allocate(nb_vertices_) ;

            // enumerate vertices
            int cur_vertex = 0 ;
            {FOR_EACH_VERTEX(Map,&map_,it) {
                vertices_(cur_vertex) = &*it ;
                vertex_id_[it] = cur_vertex ;
                cur_vertex++ ;
            }}
        
            // enumerate halfedges
            {FOR_EACH_HALFEDGE(Map,&map_,it) {
                halfedge_id_[it] = -1 ;
            }}  
            int cur_edge = 0 ;
            {FOR_EACH_HALFEDGE(Map,&map_,it) {
                if(halfedge_id_[it] == -1) {
                    halfedge_id_[it] = cur_edge ;
                    halfedge_id_[it->opposite()] = cur_edge ;
                    cur_edge++ ;
                }
            }}
        
            int cur_facet = 0 ; 
            {FOR_EACH_FACET(Map,&map_,it) {
                facet_id_[it] = cur_facet ;
                cur_facet++ ;
            }}
        }


        void MetaCell::compute_configs() {
            int nb_configs = (1 << nb_vertices_) ;

            config_.allocate(nb_configs, nb_edges_) ;
            config_.set_all(-1) ;

            config_size_.allocate(nb_configs) ;
            config_size_.set_all(0) ;

            config_face_.allocate(nb_configs, nb_facets_) ;
            config_face_.set_all(-1) ;

            config_is_ambiguous_.allocate(nb_configs) ;
            config_is_ambiguous_.set_all(false) ;

            Logger::out("MetaCell") << "nb_vertices=" << nb_vertices_ 
                                    << " nb_configs=" << nb_configs 
                                    << std::endl ;

            for(int config_code=0; config_code<nb_configs; config_code++) {
                compute_config(config_code) ;
            }
        }

        void MetaCell::compute_config(int config_code) {
            unmark_all_vertices() ;        
            int nb_vertices = vertices_.size() ;
            for(int i=0; i<nb_vertices; i++) {
                if((config_code & (1 << i)) != 0) {
                    mark_vertex(vertices_(i)) ;
                }
            }

//            std::ostream& out = Logger::out("MetaCellConfigs") ;
//            out << "config " << config_code << " : ";
            if(current_config_is_ambiguous()) {
//                out << "ambiguous" << std::endl ;
                config_is_ambiguous_(config_code) = true ;
            } else {
                int cur_in_config = 0 ;
                Map::Halfedge* first = nil ;
                for(
                    Map::Halfedge_iterator it = 
                        map_.halfedges_begin();
                    it != map_.halfedges_end(); it++
                ) {
                    if(edge_is_intersected(it)) {
                        first = it ;
                        break ;
                    }
                }
                if(first == nil) {
//                    out << "empty" << std::endl ;
                } else {
                    Map::Halfedge* cur_edge_of_face = first ;
                    do {
                        config_(
                            config_code, cur_in_config
                        ) = halfedge_id_[cur_edge_of_face] ;
                        config_face_(
                            config_code, cur_in_config
                        ) = facet_id_[cur_edge_of_face-> facet()] ;
                        cur_in_config++ ;
//                        out << halfedge_id_[cur_edge_of_face] << " " ;
                        Map::Halfedge* cur_in_face = cur_edge_of_face ;
                        do {
                            cur_in_face = cur_in_face-> next() ;
                        } while(!edge_is_intersected(cur_in_face)) ;
                        cur_edge_of_face = cur_in_face-> opposite() ;
                    } while(cur_edge_of_face != first) ;
//                    out << std::endl ;
                }
                config_size_(config_code) = cur_in_config ;
            }
        }

        bool MetaCell::current_config_is_ambiguous() const {
            for(
                Map::Facet_const_iterator it = map_.facets_begin() ;
                it != map_.facets_end(); it++
            ) {
                int nb_chs = 0 ;
                Map::Halfedge* jt = it-> halfedge() ;
                do {
                    if(edge_is_intersected(jt)) {
                        nb_chs++ ;
                    }
                    jt = jt-> next() ;
                } while(jt != it-> halfedge()) ;
                if(nb_chs > 2) {
                    return true ;
                }
            }
            return false ;
        }

//_________________________________________________________

        void Cell::instanciate_from( const MetaCell* class_in ) {
            class_ = class_in ;
            unsigned int i ;
            adjacent_ = new Cell*[class_->nb_facets()] ;
            for(i = 0 ; i < class_->nb_facets() ; i++) {
                adjacent_[i] = nil ;
            }
            vertices_ = new Vertex*[class_->nb_vertices()] ;
            for(i = 0 ; i < class_->nb_vertices() ; i++) {
                vertices_[i] = nil ;
            }
        } 

        bool Cell::has_edge(Vertex* vertex1, Vertex* vertex2) const {
            cgraph_assert(vertex1 != vertex2) ;
            int v1 = -1;
            int v2 = -1;
            for( unsigned int i=0; i < nb_vertices(); ++i ) {
                if( vertices_[i] == vertex1 ) {
                    v1 = i;
                } else if( vertices_[i] == vertex2 ) {
                    v2 = i;
                }
            }
            if(v1 == -1 || v2 == -1) {
                return false ;
            }
            return class_-> has_edge(v1, v2) ;
        }

        int Cell::vertex_index( const Vertex* vertex ) const {
            for( unsigned int i=0; i < nb_vertices(); ++i ) {
                if( vertex == vertices_[i] ) {
                    return i;
                }
            }
            return -1;
        }

        bool Cell::get_dcel(
            unsigned int face_index,
            Vertex* v, 
            Vertex*& prev, Vertex*& next
        ) const {
            int v_i = vertex_index( v );
            if( v_i == -1 ) {
                prev = nil; next = nil;
                return false;
            }
            int i_prev, i_next;
            if(!class_->get_dcel( face_index, v_i, i_prev, i_next)) {
                prev = nil ;
                next = nil ;
                return false ;
            } 
            prev = vertices_[i_prev] ;
            next = vertices_[i_next] ;
            return true ;
        }


//_________________________________________________________

    }

}
