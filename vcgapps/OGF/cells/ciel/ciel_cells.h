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
 

#ifndef __CIEL_CELLS__
#define __CIEL_CELLS__

#include <OGF/cells/common/common.h>
#include <OGF/math/geometry/types.h>
#include <OGF/basic/attributes/record_id.h>
#include <OGF/basic/attributes/attribute_manager.h>

#include <set>

#ifdef OGF_PARANOID
#define CIEL_CHECK
#endif

#ifdef CIEL_CHECK
#define OGF_ARRAY_BOUND_CHECK
#define ciel_assert(x) ogf_assert(x)
#else
#define ciel_assert(x)
#endif

namespace OGF {
    
    class Ciel ;
    class CielMutator ;

    namespace CielTypes {

//_________________________________________________________

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

        class Halfedge ;
        class Vertex ;
        class Cell ;
    
        class CELLS_API Vertex : public Combel {
        public:
            Vertex() : halfedge_(nil) { }
            Halfedge* halfedge() const { return halfedge_ ; }
            const Point3d& point() const  { return point_ ; }
            void set_point(const Point3d& p) { point_ = p ; }
            
            /** 
             * retreives the set of cells incident to this vertex 
             * TODO: make it work if border is not sewn.
             */
            void get_star(std::set<Cell*>& cells) const ;
            
            bool is_valid() const { return true ; } // TODO
            void assert_is_valid() const { ogf_assert(is_valid()) ; }

            bool is_on_border() const ;

//        protected:
            void set_halfedge(Halfedge* h) { halfedge_ = h ; }
            void insert_halfedge_in_ciel(Halfedge* h) ;

        private:
            Halfedge* halfedge_ ;
            Point3d point_ ;
            friend class CielMutator ;
        } ;
    
        class CELLS_API Halfedge : public Combel {
        public:
            Halfedge() : next_around_vertex_(nil), 
                         next_around_facet_(nil),
                         opposite_facet_(nil),
                         opposite_cell_(nil),
                         vertex_(nil),
                         cell_(nil) {
            }
            Halfedge* next_around_vertex() const { return next_around_vertex_ ; }
            Halfedge* next_around_facet() const  { return next_around_facet_ ; }
            Halfedge* opposite_facet() const     { return opposite_facet_ ; }
            Halfedge* opposite_cell() const      { return opposite_cell_ ; }
            Vertex* vertex() const               { return vertex_ ; }
            Cell* cell() const                   { return cell_ ; }

            bool is_valid() const { return true ; } // TODO
            void assert_is_valid() const { ogf_assert(is_valid()) ; }

        protected:
            void set_next_around_vertex(Halfedge* h) { next_around_vertex_ = h ; }
            void set_next_around_facet(Halfedge* h)  { next_around_facet_ = h ; }
            void set_opposite_facet(Halfedge* h)     { opposite_facet_ = h ; }
            void set_opposite_cell(Halfedge* h)      { opposite_cell_ = h ; }
            void set_vertex(Vertex* v)               { vertex_ = v ; }
            void set_cell(Cell* c)                   { cell_ = c ; }

        private:
            Halfedge* next_around_vertex_ ;        
            Halfedge* next_around_facet_ ;
            Halfedge* opposite_facet_ ;
            Halfedge* opposite_cell_ ;
            Vertex* vertex_ ;
            Cell*   cell_ ;
            friend class Vertex ;
            friend class ::OGF::CielMutator ;
        } ;

        class CELLS_API Cell : public Combel {
        public:
            Cell() : halfedge_(nil) { }
            Halfedge* halfedge() const { return halfedge_ ; }
            void get_vertices(std::set<Vertex*>& vertices) const ;

            bool is_valid() const { return true ; } // TODO
            void assert_is_valid() const { ogf_assert(is_valid()) ; }

 //       protected:
            void set_halfedge(Halfedge* h) { halfedge_ = h ; }

        private:
            Halfedge* halfedge_ ;
            friend class CielMutator ;
        } ;

//__________________________________________________________

    }
    
}
#endif

