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
 

#ifndef ___CGRAPH_ATTRIBUTES__
#define ___CGRAPH_ATTRIBUTES__

#include <OGF/cells/common/common.h>
#include <OGF/cells/cgraph/cgraph.h>

namespace OGF {

//_________________________________________________________

    template <class T> 
    class CGraphVertexAttribute : public Attribute<CGraph::Vertex, T> {
    public:
        typedef Attribute<CGraph::Vertex, T> superclass ;

        void bind(CGraph* cgraph, const std::string& name) {
            superclass::bind(
                cgraph->vertex_attribute_manager(), name 
            ) ;
        }

        void bind(CGraph* cgraph) { 
            superclass::bind(cgraph->vertex_attribute_manager()) ; 
        }
        
        CGraphVertexAttribute() { }

        CGraphVertexAttribute(CGraph* cgraph) {
            bind(cgraph) ;
        }

        CGraphVertexAttribute(CGraph* cgraph, const std::string& name) { 
            bind(cgraph, name) ;
        }
        
        static bool is_defined(CGraph* cgraph, const std::string& name) {
            return superclass::is_defined(
                cgraph->vertex_attribute_manager(), name
            ) ;
        }
    } ;

//_________________________________________________________

    template <class T> 
    class CGraphCellAttribute : public Attribute<CGraph::Cell, T> {
    public:
        typedef Attribute<CGraph::Cell, T> superclass ;

        void bind(CGraph* cgraph, const std::string& name) {
            superclass::bind(
                cgraph->cell_attribute_manager(), name
            ) ;
        }

        void bind(CGraph* cgraph) { 
            superclass::bind(cgraph->cell_attribute_manager()) ; 
        }
        
        CGraphCellAttribute() { }

        CGraphCellAttribute(CGraph* cgraph) {
            bind(cgraph) ;
        }

        CGraphCellAttribute(CGraph* cgraph, const std::string& name) { 
            bind(cgraph, name) ; 
        }
        
        static bool is_defined(CGraph* cgraph, const std::string& name) {
            return superclass::is_defined(
                cgraph->cell_attribute_manager(), name
            ) ;
        }
    } ;

//=====================================================================
//
//                    standard cgraph attributes
//
//=====================================================================


    class CGraphVertexLock : public CGraphVertexAttribute<bool> {
    public:
        typedef CGraphVertexAttribute<bool> superclass ;
        CGraphVertexLock() { }
        CGraphVertexLock(CGraph* cgraph) : superclass(cgraph, "lock") { }
        void bind(CGraph* cgraph) { superclass::bind(cgraph, "lock") ; }
        static bool is_defined(CGraph* cgraph) {
            return superclass::is_defined(cgraph,"lock") ;
        }
    } ;

    //_________________________________________________________

}
#endif

