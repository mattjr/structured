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
 

#ifndef __OGF_CELLS_CGRAPH_CELLS__
#define __OGF_CELLS_CGRAPH_CELLS__

#ifdef OGF_PARANOID
#define CGRAPH_CHECK
#endif

#ifdef CGRAPH_CHECK
#define OGF_ARRAY_BOUND_CHECK
#define cgraph_assert(x) ogf_assert(x)
#else
#define cgraph_assert(x)
#endif

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map.h>
#include <OGF/basic/containers/arrays.h>
#include <OGF/basic/types/types.h>

namespace OGF {

    class CGraph ;
    class CGraphMutator ;


    class VertexCellFlags ;

//_________________________________________________________

    namespace CGraphTypes {

        /**
         * Combinatorial Element. Combel is
         * the base class for vertices, halfedges
         * and cells.
         */
        class CELLS_API Combel : public Record {
        public:
            Combel()  { }
            ~Combel() { }
        } ;
        
        class Cell;

//_________________________________________________________
        
        class CELLS_API MetaCell {

        public:
            MetaCell() ;
            ~MetaCell() ;
        
            unsigned int nb_vertices() const { return nb_vertices_ ;  } 
            unsigned int nb_facets() const   { return nb_facets_ ;    }
            unsigned int nb_edges() const    { return nb_edges_ ;     }

            /**
             * Extremity of an edge.
             * @param edge in 0 .. nb_edges - 1
             * @param vertex in 0 .. 1
             */
            int edge_vertex(unsigned int edge, unsigned int vertex) const {
                return e2v_(edge, vertex) ;
            }

            /** 
             * Extremities of an edge.
             * @param edge in 0 .. nb_edges - 1
             */
            void edge_vertices( 
                unsigned int edge, int& vertex0, int& vertex1
            ) const {
                vertex0 = e2v_(edge, 0) ;
                vertex1 = e2v_(edge, 1) ;
            }

            /**
             * @return the index of an edge from its extremities.
             * Returns -1 if the edges are not connected through an edge.
             */
            int edge_between_vertices( 
                unsigned int vertex0, unsigned int vertex1 
            ) const {
                int result = v2e_(vertex0, vertex1) ;
                cgraph_assert(result != -1) ;
                return result ;
            }
        
            bool has_edge( unsigned int vertex0, unsigned int vertex1 ) const {
                cgraph_assert(vertex0 != vertex1) ;
                return (v2e_(vertex0, vertex1) != -1) ;
            }

            unsigned int nb_vertices_in_facet(unsigned int facet) const {
                return f2n_(facet) ;
            }

            unsigned int facet_vertex(
                unsigned int face, unsigned int vertex_in_face
            ) const {
                int result = fv2v_(face, vertex_in_face) ;
                cgraph_assert(result != -1) ;
                return result ;
            }


            /**
             * computes the vertex id of the successor and the predecessor
             * of the specified vertex within the specified face. 
             * returns false if the specified vertex does not belong to
             * the specified face.
             */
            bool get_dcel( 
                unsigned int face, unsigned int vertex, int& prev, int& next
            ) const {
                prev = fv2u_(face, vertex, 0) ;
                next = fv2u_(face, vertex, 1) ;
                return (prev != -1 && next != -1) ;
            }

            unsigned int config_size(unsigned int config_code) const {
                return config_size_(config_code) ;
            }

            unsigned int config_intersected_edge(
                unsigned int config_code, unsigned int edge_index_in_config
            ) const {
                int result = config_(config_code, edge_index_in_config) ;
                cgraph_assert(result != -1) ;
                return (unsigned int)(result) ;
            }

            bool config_is_ambiguous(unsigned int config_code) const {
                return config_is_ambiguous_(config_code) ;
            }

            unsigned int config_intersected_facet(
                unsigned int config_code, unsigned int face_index_in_config
            ) const {
                int result = config_face_(config_code, face_index_in_config) ;
                cgraph_assert(result != -1) ;
                return (unsigned int)(result) ;
            }

        public:
            Map& map()             { return map_ ; }
            const Map& map() const { return map_ ; }
            bool check_map() const ;
            
            void initialize_from_map() ;

        protected:
            void enumerate_cells() ;
            void compute_configs() ;
            void compute_config(int config_code) ;
            bool current_config_is_ambiguous() const ;

            void mark_vertex(Map::Vertex* v) {
                vertex_id_[v] = 1 ;
            }

            void unmark_all_vertices() ;

            bool vertex_is_marked(Map::Vertex* v) const {
                return (vertex_id_[v] != 0) ;
            } 

            bool edge_is_intersected(Map::Halfedge* h) const {
                return (
                    vertex_id_[h->vertex()] != 
                    vertex_id_[h->opposite()->vertex()]
                ) ;
            }

        private:
            Map map_ ;
            Attribute<Map::Vertex, int> vertex_id_ ;
            Attribute<Map::Halfedge, int> halfedge_id_ ;
            Attribute<Map::Facet, int> facet_id_ ;
            Array1d<Map::Vertex*> vertices_ ;
            int nb_vertices_ ;
            int nb_facets_ ;
            int nb_edges_ ;
            int max_facet_nb_vertices_ ;

            Array2d<int> e2v_ ;
            Array2d<int> v2e_ ;
            Array1d<int> f2n_ ;
            Array2d<int> fv2v_ ;
            Array3d<int> fv2u_ ;
            Array2d<int> config_ ;
            Array2d<int> config_face_ ;
            Array1d<unsigned int> config_size_ ;
            Array1d<bool> config_is_ambiguous_ ;
        } ;

//_________________________________________________________

        class CELLS_API Vertex : public Combel {
        public :
            Vertex() { }
            ~Vertex() { }
            const Point3d& point() const  { return point_ ; }
            Point3d& point() { return point_ ; }
            void set_point(const Point3d& p) { point_ = p ; }
            bool is_valid() const { return true ; }
            void assert_is_valid() const { ogf_assert(is_valid()) ; }
        private:
            Point3d point_ ;
            friend class CGraphMutator ;
        } ;
    
//_________________________________________________________

        class CELLS_API Cell : public Combel {           
        public:            
            Cell() {
                class_ = nil ; adjacent_ = nil ;
            } 
            
            ~Cell() {
                delete[] adjacent_ ; adjacent_ = nil ;
                delete[] vertices_ ; vertices_ = nil ;
                class_ = nil ;
            } 
            
            const MetaCell* meta_cell() const { return class_ ; }

            void instanciate_from( const MetaCell* class_in ) ;
            unsigned int nb_vertices() const { return class_->nb_vertices(); } 
            unsigned int nb_edges() const { return class_->nb_edges(); } 
            unsigned int nb_facets() const { return class_->nb_facets(); }

            Vertex* edge_vertex(
                unsigned int edge, unsigned int vertex
            ) const ;

            void edge_vertices(
                unsigned int edge, Vertex*& v1, Vertex*& v2
            ) const ;

            bool has_edge(Vertex* vertex1, Vertex* vertex2) const ;
            
            Vertex* facet_vertex( 
                unsigned int face_index, unsigned int vertex_in_face
            ) const {
                unsigned int i = class_->facet_vertex(
                    face_index, vertex_in_face
                ) ;
                return vertices_[i] ;
            }

            unsigned int nb_vertices_in_facet(unsigned int face_index) const {
                return class_-> nb_vertices_in_facet(face_index) ;
            }
        
            /** 
             * @return the index of a vertex in this cell.
             * Returns -1 if the vertex does not belong to this cell.
             */
            int vertex_index( const Vertex* vertex ) const ;
            
            bool get_dcel(
                unsigned int face_index,
                Vertex* v, Vertex*& prev, Vertex*& next
            ) const ;

            Vertex* vertex(unsigned int i) const {
                cgraph_assert(i < nb_vertices()) ;
                return vertices_[i] ;
            }
            
            Cell* adjacent(unsigned int i) const {
                cgraph_assert(i < nb_facets()) ;
                return adjacent_[i] ;
            }

            /** 
             * @return the index of the common face in the neighbor.
             * Returns -1 if the face is on border.
             */
            int facet_in_neighbor( unsigned int face ) const ; 

            bool is_valid() const { return true ; }
            void assert_is_valid() const { ogf_assert(is_valid()) ; }
            
        protected:
            void set_adjacent(unsigned int i, Cell* c) {
                cgraph_assert(i < nb_facets()) ;
                adjacent_[i] = c ;
            }
            void set_vertex(unsigned int i, Vertex* v) {
                cgraph_assert(i < nb_vertices()) ;
                vertices_[i] = v ;
            }

        private:
            const MetaCell* class_ ;
            Cell** adjacent_ ;
            Vertex** vertices_ ;

            friend class ::OGF::CGraphMutator ;
        } ;


//_________________________________________________________


        /** used by Structured Cellular Graph */
        class CELLS_API VertexCell {
        public:
            VertexCell() : flags_(0) { }
            const Point3d& point() const  { return point_ ; }
            void set_point(const Point3d& p) { point_ = p ; }

        private:
            Point3d point_ ;
            Numeric::uint8 flags_ ;
            friend class SCGraphMutator ;
            friend class ::OGF::VertexCellFlags ;
        } ;

    
//===========================================================================

        inline Vertex* Cell::edge_vertex( 
            unsigned int edge, unsigned int vertex
        ) const {
            cgraph_assert( edge < nb_edges() );
            int iv = class_->edge_vertex( edge, vertex);
            cgraph_assert( iv != -1 );
            return vertices_[iv] ; 
        }

        inline void Cell::edge_vertices( 
            unsigned int edge, Vertex*& v1, Vertex*& v2 
        ) const {
            cgraph_assert( edge < nb_edges() );
            int iv1, iv2;
            class_->edge_vertices( edge, iv1, iv2 );
            cgraph_assert( iv1 != -1 && iv2 != -1 );
            v1 = vertices_[iv1] ; v2 = vertices_[iv2] ;
        }

        inline int Cell::facet_in_neighbor( unsigned int face ) const {
            cgraph_assert( face < nb_facets() );
            
            Cell* neighbor_cell = adjacent_[face];
            if( neighbor_cell == nil ) {
                return -1;
            }
            
            for( unsigned int f_in_neighbor = 0; 
                 f_in_neighbor < neighbor_cell->nb_facets(); f_in_neighbor++
            ) {
                if( neighbor_cell->adjacent_[f_in_neighbor] == this ) {
                    return f_in_neighbor;
                }
            }
            // We should never get there.
            cgraph_assert( false );
            return -1;
        }
    }


    class CELLS_API VertexCellFlags {
    private:
        enum {GHOST_BIT=0} ;
    public:
        bool is_ghost(const CGraphTypes::VertexCell& c) {
            return (c.flags_ & (1 << GHOST_BIT)) != 0 ;
        }
        void make_ghost(CGraphTypes::VertexCell& c) {
            c.flags_ |= (1 << GHOST_BIT) ;
        }
        void unmake_ghost(CGraphTypes::VertexCell& c) {
            c.flags_ &= ~(1 << GHOST_BIT) ;
        }
    } ;
}

#endif

