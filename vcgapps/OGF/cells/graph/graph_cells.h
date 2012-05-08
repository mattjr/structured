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
 

#ifndef __OGF_CELLS_GRAPH_GRAPH_CELLS__
#define __OGF_CELLS_GRAPH_GRAPH_CELLS__

#include <OGF/cells/common/common.h>
#include <OGF/math/geometry/types.h>
#include <OGF/basic/attributes/record_id.h>
#include <OGF/basic/attributes/attribute_manager.h>

#include <string.h>

// Turns on assertion checks and
//  clearing destructors.

#define GRAPH_DEBUG

namespace OGF {

//_________________________________________________________

    class Graph ;
    class GraphMutator ;

    namespace GraphTypes {

        class Vertex ;
        class Halfedge ;

        //___________________________________________________

        /**
         * Combinatorial Element. Combel is the base class for 
         * vertices and half-edges
         */
        class CELLS_API Combel : public Record {
        public:
            Combel()  { }
            ~Combel() { }
        } ;

        //___________________________________________________

        /**
         * A vertex of a Graph. Each Vertex has a geometry (i.e. a Point3d).
         */
        class CELLS_API Vertex : public Combel {
        public:
            Vertex() : halfedge_(nil) {  }
            Vertex(const Point3d& p) : point_(p), halfedge_(nil) {  }
            ~Vertex() { halfedge_ = nil ; }
            
            const Point3d& point() const     { return point_ ; }
            Point3d& point()                 { return point_ ; }
            void set_point(const Point3d& p) { point_ = p ;    }

            Halfedge* halfedge() const { return halfedge_ ; }

            bool is_valid() const ;
            void assert_is_valid() const ;

            int degree() const ;
            bool is_extremity() const ;

        protected:
            void set_halfedge(Halfedge* h) { halfedge_ = h ; }
            friend class ::OGF::Graph ;
            friend class ::OGF::GraphMutator ;

        private:
            Point3d point_ ;
            Halfedge* halfedge_ ;
        } ;

        //______________________________________________

        /**
         * Each edge of a Map is composed of two Halfedges.
         */
        
        class CELLS_API Halfedge : public Combel {
        public:
            Halfedge() : 
                prev_around_vertex_(nil),
                next_around_vertex_(nil),
                opposite_(nil),
                vertex_(nil) {
            }
            ~Halfedge() { 
                prev_around_vertex_ = nil ;
                next_around_vertex_ = nil ;
                opposite_ = nil ;
                vertex_ = nil ;
            }

            Halfedge* opposite() const { return opposite_ ; }
            Halfedge* next_around_vertex() const {
                return next_around_vertex_ ;
            }
            Halfedge* prev_around_vertex() const {
                return prev_around_vertex_ ;
            }

            Vertex* vertex() const { return vertex_ ; }

            /** One halfedge per vertex exacly is the vertex key. */
            bool is_vertex_key() const ;

            /** 
             * One halfedge per edge exacly is the edge key. 
             * Note: this can be used for loops, to traverse
             * one halfedge per edge exaclty (for instance, to
             * draw the mesh).
             */
            bool is_edge_key() const ;

            bool is_valid() const ;
            void assert_is_valid() const ;

        protected:
            void set_opposite(Halfedge* h) { opposite_ = h; }
            void set_next_around_vertex(Halfedge* h) { next_around_vertex_ = h; }
            void set_prev_around_vertex(Halfedge* h) { prev_around_vertex_ = h; }
            void set_vertex(Vertex* v) { vertex_ = v ; }

            friend class ::OGF::Graph ;
            friend class ::OGF::GraphMutator ;

        private:
            Halfedge* prev_around_vertex_ ;
            Halfedge* next_around_vertex_ ;
            Halfedge* opposite_ ;
            Vertex* vertex_ ;
        } ;

        //______________________________________________

        inline bool Halfedge::is_vertex_key() const {
            return (vertex_->halfedge() == this) ;
        }
        
        inline bool Halfedge::is_edge_key() const {
            // TODO: if the GarbageCollector is added, 
            // watch out, the edge keys can change...
            return (this < opposite_) ;
        }

//_________________________________________________________
    
    }

}
#endif

