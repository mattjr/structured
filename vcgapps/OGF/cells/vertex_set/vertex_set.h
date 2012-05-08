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
 

#ifndef __OGF_CELLS_VERTEX_SET_VERTEX_SET__
#define __OGF_CELLS_VERTEX_SET_VERTEX_SET__

#include <OGF/cells/common/common.h>
#include <OGF/cells/types/iterators.h>
#include <OGF/math/geometry/types.h>
#include <OGF/basic/containers/dlist.h>
#include <OGF/basic/attributes/attribute.h>

namespace OGF {

    namespace VertexSetTypes {

        /**
         * Combinatorial Element. Combel is the base class for 
         * vertices.
         */
        class CELLS_API Combel : public Record {
        public:
            Combel()  { }
            ~Combel() { }
        } ;


        class CELLS_API Vertex : public Combel {
        public:
            Vertex() { } ;
            const Point3d& point() const     { return point_ ; }
            Point3d& point()                 { return point_ ; }
            void set_point(const Point3d& p) { point_ = p ;    }
        private:
            Point3d point_ ;
        } ;
    } 

    class CELLS_API VertexSet {
    public:

        // __________________ types ___________________

        typedef VertexSetTypes::Vertex Vertex ;
        typedef DList<Vertex>::iterator Vertex_iterator ;
        typedef DList<Vertex>::const_iterator Vertex_const_iterator ;
        typedef GenericAttributeManager<Vertex> VertexAttributeManager ;

        VertexSet() ;
        virtual ~VertexSet() ;

        // __________________ access ___________________

        Vertex_iterator vertices_begin()    { return vertices_.begin() ;  }
        Vertex_iterator vertices_end()      { return vertices_.end() ;    }
        Vertex_const_iterator vertices_begin() const { 
            return vertices_.begin() ;  
        }
        Vertex_const_iterator vertices_end() const { 
            return vertices_.end() ;    
        }

        int size_of_vertices() const  { return vertices_.size() ;  }

        // ___________________ attributes _______________________
        
        VertexAttributeManager* vertex_attribute_manager() const {
            return const_cast<VertexAttributeManager*>(
                &vertex_attribute_manager_ 
            ) ;
        }

        // __________________ modification ______________________

        void clear() ;
        void erase_all() ;

        // Note: there is no VertexSetEditor or VertexSetMutator classes,
        //   possible modifications are simple and are made public here.

        Vertex* new_vertex() ;
        Vertex* new_vertex(const Point3d& p) ;
        void delete_vertex(Vertex* v) ;

    private:
        DList<Vertex> vertices_ ;
        VertexAttributeManager vertex_attribute_manager_ ;        
    } ;

    //___________________________________________________________________

    template <class T> 
    class VertexSetVertexAttribute : public Attribute<VertexSet::Vertex, T> {
    public:
        typedef Attribute<VertexSet::Vertex, T> superclass ;

        void bind(VertexSet* vertex_set, const std::string& name) {
            superclass::bind(
                vertex_set->vertex_attribute_manager(), name 
            ) ;
        }

        void bind(VertexSet* vertex_set) { 
            superclass::bind(vertex_set->vertex_attribute_manager()) ; 
        }
        
        VertexSetVertexAttribute() { }

        VertexSetVertexAttribute(VertexSet* vertex_set) {
            bind(vertex_set) ;
        }

        VertexSetVertexAttribute(VertexSet* vertex_set, const std::string& name) { 
            bind(vertex_set, name) ;
        }
        
        static bool is_defined(VertexSet* vertex_set, const std::string& name) {
            return superclass::is_defined(
                vertex_set->vertex_attribute_manager(), name
            ) ;
        }
    } ;

    //______________________________________________________________

}

#endif
