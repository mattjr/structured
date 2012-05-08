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
 

#ifndef ___CIEL_ATTRIBUTES__
#define ___CIEL_ATTRIBUTES__

#include <OGF/cells/common/common.h>
#include <OGF/cells/ciel/ciel.h>

namespace OGF {

//_________________________________________________________

    template <class T> 
    class CielVertexAttribute : public Attribute<Ciel::Vertex, T> {
    public:
        typedef Attribute<Ciel::Vertex, T> superclass ;

        void bind(Ciel* ciel, const std::string& name) {
            superclass::bind(
                ciel->vertex_attribute_manager(), name 
            ) ;
        }

        void bind(Ciel* ciel) { 
            superclass::bind(ciel->vertex_attribute_manager()) ; 
        }
        
        CielVertexAttribute() { }

        CielVertexAttribute(Ciel* ciel) {
            bind(ciel) ;
        }

        CielVertexAttribute(Ciel* ciel, const std::string& name) { 
            bind(ciel, name) ;
        }
        
        static bool is_defined(Ciel* ciel, const std::string& name) {
            return superclass::is_defined(
                ciel->vertex_attribute_manager(), name
            ) ;
        }
    } ;

//_________________________________________________________

    template <class T> 
    class CielHalfedgeAttribute : public Attribute<Ciel::Halfedge, T> {
    public:
        typedef Attribute<Ciel::Halfedge, T> superclass ;

        void bind(Ciel* ciel, const std::string& name) {
            superclass::bind(
                ciel->halfedge_attribute_manager(), name
            ) ;
        }

        void bind(Ciel* ciel) { 
            superclass::bind(ciel->halfedge_attribute_manager()) ; 
        }
        
        CielHalfedgeAttribute() { }

        CielHalfedgeAttribute(Ciel* ciel) {
            bind(ciel) ;
        }

        CielHalfedgeAttribute(Ciel* ciel, const std::string& name) { 
            bind(ciel, name) ; 
        }
        
        static bool is_defined(Ciel* ciel, const std::string& name) {
            return superclass::is_defined(
                ciel->halfedge_attribute_manager(), name
            ) ;
        }
    } ;

//_________________________________________________________

    template <class T> 
    class CielCellAttribute : public Attribute<Ciel::Cell, T> {
    public:
        typedef Attribute<Ciel::Cell, T> superclass ;

        void bind(Ciel* ciel, const std::string& name) {
            superclass::bind(
                ciel->cell_attribute_manager(), name
            ) ;
        }

        void bind(Ciel* ciel) { 
            superclass::bind(ciel->cell_attribute_manager()) ; 
        }
        
        CielCellAttribute() { }

        CielCellAttribute(Ciel* ciel) {
            bind(ciel) ;
        }

        CielCellAttribute(Ciel* ciel, const std::string& name) { 
            bind(ciel, name) ; 
        }
        
        static bool is_defined(Ciel* ciel, const std::string& name) {
            return superclass::is_defined(
                ciel->cell_attribute_manager(), name
            ) ;
        }
    } ;

//=====================================================================
//
//                    standard ciel attributes
//
//=====================================================================


    class CielVertexLock : public CielVertexAttribute<bool> {
    public:
        typedef CielVertexAttribute<bool> superclass ;
        CielVertexLock() { }
        CielVertexLock(Ciel* ciel) : superclass(ciel, "lock") { }
        void bind(Ciel* ciel) { superclass::bind(ciel, "lock") ; }
        static bool is_defined(Ciel* ciel) {
            return superclass::is_defined(ciel,"lock") ;
        }
    } ;

    //_________________________________________________________


}
#endif

