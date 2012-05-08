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
 

#ifndef __OGF_CELLS_TRI_DELAUNAY___
#define __OGF_CELLS_TRI_DELAUNAY___

#include <OGF/cells/common/common.h>
#include <OGF/basic/types/counted.h>
#include <OGF/basic/types/smart_pointer.h>
#include <OGF/math/geometry/types.h>

namespace OGF {

    //________________________________________________________________________________

    /**
     * DelaunaySkeleton stores the vertex graph of a Delaunay triangulation
     * and the equations of the bisectors associated with each primal edge.
     * All information is stored in a Compressed Row Storage array.
     */
    class DelaunaySkeleton {
    public:
        DelaunaySkeleton() {
            star_ptr_.push_back(0) ;
        }

        void clear() {
            // Note: we use resize(0) instead of clear()
            // since this is guaranteed to keep the reserved
            // memory (this is what we want since we keep 
            // clearing and filling the same DelaunaySkeleton).
            star_ptr_.resize(0) ; 
            neighbor_.resize(0) ; 
            star_ptr_.push_back(0) ;
        }

        unsigned int nb_vertices() const { return (unsigned int)star_ptr_.size() - 1 ; }
        unsigned int star_begin(unsigned int i) const {
            return star_ptr_[i] ;
        }
        unsigned int star_end(unsigned int i) const {
            return star_ptr_[i+1] ;
        }
        unsigned int nb_neighbors(unsigned int i) const {
            return star_end(i) - star_begin(i) ;
        }
        unsigned int neighbor(unsigned int j) const {
            return neighbor_[j] ;
        }
        unsigned int neighbor(unsigned int i, unsigned int j) const {
            ogf_debug_assert(j < nb_neighbors(i)) ;
            return neighbor(star_begin(i) + j) ;
        }
        void begin_star() { }
        void add_to_star(unsigned int neighbor) {
            neighbor_.push_back(neighbor) ;
        }
        void end_star() {
            star_ptr_.push_back((unsigned int)neighbor_.size()) ;
        }

        // Used internally by copy_Delaunay_to_skel
        bool last_star_has_vertex(unsigned int i) const {
            for(
                unsigned int k = star_ptr_[star_ptr_.size() - 1]; k < neighbor_.size(); k++) {
                if(neighbor_[k] == i) { return true ; }
            }
            return false ;
        }


    private:
        std::vector<unsigned int> star_ptr_ ;
        std::vector<unsigned int> neighbor_ ;
    } ;

    //________________________________________________________________________________

    class CELLS_API DelaunayAPI : public Counted {
    public:
        DelaunayAPI() ;
        virtual ~DelaunayAPI() ;
        virtual void update_skeleton() const = 0 ;
        const DelaunaySkeleton* skeleton() const { 
            if(skeleton_is_dirty_) { update_skeleton() ; }
            return &skeleton_ ; 
        }
    protected:
        mutable bool skeleton_is_dirty_ ;
        mutable DelaunaySkeleton skeleton_ ;
    } ;

    //________________________________________________________________________________

    class CELLS_API DelaunayAPI2d : public DelaunayAPI {
    public:
        DelaunayAPI2d() ;
        virtual ~DelaunayAPI2d() ;
        virtual void set_vertices(const std::vector<Point2d>& vertices) = 0 ;
        virtual unsigned int nearest_vertex(const Point2d& p) const = 0 ;
        unsigned int nb_vertices() const { return vertices_ ? vertices_->size() : 0 ; }
        const Point2d& vertex(unsigned int i) const {
            ogf_debug_assert(i < nb_vertices()) ; 
            return (*vertices_)[i] ;
        }
        /**
         * Returns the array of triangles vertices indices. 
         *  [t1v1, t1v2, t1v3, t2v1, t2v2, t2v3, ... ]
         *   If finite_only is false, triangles connected to the 'vertex at infinity' are
         *   also returned (the vertex at infinity has id -1).
         */
        virtual void get_triangles(std::vector<int>& triangles, bool finite_only = true) ;

        /**
         * Returns the array ot triangles adjacent to each triangle.
         */
        virtual void get_triangles_adjacent(std::vector<int>& adj_tri, bool finite_only = true) ;

    protected:
        const std::vector<Point2d>* vertices_ ;
    } ;

    typedef SmartPointer<DelaunayAPI2d> DelaunayAPI2d_var ;

    //________________________________________________________________________________

    /**
     * VoronoiCell stores the dual facets in a Compressed Row Storage array.
     * - Each facet knows the bisector it is on, and the list of vertices/edges.
     *    - Each vertex knows the tet it is dual to.
     *    - Each edge knows the other bisector it is on (an edge is defined as the
     * intersection between the facet bisector and the edge bisector).
     */
    class VoronoiCell3d {
    public:
        VoronoiCell3d() { facet_ptr_.push_back(0) ; }
        void clear() {
            facet_ptr_.resize(0) ;
            facet_bisector_.resize(0) ;
            edge_bisector_.resize(0) ;
            vertex_.resize(0) ;
            infinite_.resize(0) ;
            facet_ptr_.push_back(0) ;
        }

        unsigned int nb_facets() const { return (unsigned int)facet_ptr_.size() - 1 ; }

        unsigned int facet_begin(unsigned int f) const {
            ogf_debug_assert(f < nb_facets()) ;
            return facet_ptr_[f] ;
        }

        unsigned int facet_end(unsigned int f) const {
            ogf_debug_assert(f < nb_facets()) ;
            return facet_ptr_[f+1] ;
        }

        unsigned int nb_vertices(unsigned int f) const {
            ogf_debug_assert(f < nb_facets()) ;
            return facet_end(f) - facet_begin(f) ;
        }

        unsigned int next_around_facet(unsigned int f, unsigned int i) const {
            ogf_debug_assert(i >= facet_begin(f) && i < facet_end(f)) ;
            return (i+1 == facet_end(f) ? facet_begin(f) : i+1) ;
        }

        unsigned int prev_around_facet(unsigned int f, unsigned int i) const {
            ogf_debug_assert(i >= facet_begin(f) && i < facet_end(f)) ;            
            return (i == facet_begin(f) ? facet_end(f)-1 : i-1) ;
        }

        unsigned int facet_bisector(unsigned int f) const {
            ogf_debug_assert(f < nb_facets()) ;
            return facet_bisector_[f] ;
        }

        int edge_bisector(unsigned int i) const {
            ogf_debug_assert(i < edge_bisector_.size()) ;
            return edge_bisector_[i] ;
        }

        const Point3d& vertex(unsigned int i) const {
            ogf_debug_assert(i < vertex_.size()) ;
            return vertex_[i] ;
        }

        bool vertex_is_infinite(unsigned int i) const {
            ogf_debug_assert(i < infinite_.size()) ;
            return infinite_[i] ;
        }

        void begin_facet(unsigned int f_bisector) {
            facet_bisector_.push_back(f_bisector) ;
        }

        void add_to_facet(
            int e_bisector, const Point3d& v, bool infinite
        ) {
            edge_bisector_.push_back(e_bisector) ;
            vertex_.push_back(v) ;
            infinite_.push_back(infinite) ;
        }

        void add_to_facet(
            int e_bisector, bool infinite
        ) {
            edge_bisector_.push_back(e_bisector) ;
            infinite_.push_back(infinite) ;
        }

        void end_facet() {
            facet_ptr_.push_back((unsigned int)edge_bisector_.size()) ;            
        }

        unsigned int find_facet(unsigned int bisector) {
            for(unsigned int i=0; i<facet_bisector_.size(); i++) {
                if(facet_bisector_[i] == bisector) {
                    return i ;
                }
            }
            std::cerr << "bisector = " << bisector ;
            std::cerr << "facet = [" ;
            for(unsigned int i=0; i<facet_bisector_.size(); i++) {
                std::cerr << facet_bisector_[i] << " " ;
            }
            std::cerr << "]" << std::endl ;
            ogf_assert_not_reached ;
            return 0 ;
        }

    private:
        std::vector<unsigned int> facet_ptr_ ;
        std::vector<unsigned int> facet_bisector_ ;
        std::vector<int> edge_bisector_ ;
        std::vector<Point3d> vertex_ ;
        std::vector<bool> infinite_ ; 
    } ;

    //________________________________________________________________________________

    class CELLS_API DelaunayAPI3d : public DelaunayAPI {
    public:
        DelaunayAPI3d() ;
        virtual ~DelaunayAPI3d() ;
        virtual void set_vertices(const std::vector<Point3d>& vertices) = 0 ;
        virtual unsigned int nearest_vertex(const Point3d& p) const = 0 ;
        unsigned int nb_vertices() const { return vertices_ ? vertices_->size() : 0 ; }
        const Point3d& vertex(unsigned int i) const {
            ogf_debug_assert(i < nb_vertices()) ; 
            return (*vertices_)[i] ;
        }
        /**
         * Returns the array of tetrahedra vertices indices. 
         *  [t1v1, t1v2, t1v3, t1v4, t2v1, t2v2, t2v3, t2v4, ... ]
         *   If finite_only is false, tetrahedra connected to the 'vertex at infinity' are
         *   also returned (the vertex at infinity has id -1).
         */
        virtual void get_tetrahedra(std::vector<int>& tetrahedra, bool finite_only = true) ;

        /**
         * Returns the array ot tetrahedra adjacent to each tetrahedron.
         */
        virtual void get_tetrahedra_adjacent(std::vector<int>& adj_tet, bool finite_only = true) ;

        /**
         * Retreives the Voronoi cell associated with vertex v.
         */
        virtual void get_voronoi_cell(unsigned int v, VoronoiCell3d& cell, bool geometry=true) ;

    protected:
        const std::vector<Point3d>* vertices_ ;
    } ;

    typedef SmartPointer<DelaunayAPI3d> DelaunayAPI3d_var ;

    //________________________________________________________________________________

} ;

#endif

