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
 

#ifndef ___GRAPH_ATTRIBUTES__
#define ___GRAPH_ATTRIBUTES__

#include <OGF/cells/common/common.h>
#include <OGF/cells/graph/graph.h>
#include <OGF/basic/types/counted.h>

#include <string>
#include <vector>
#include <iostream>

namespace OGF {

//_________________________________________________________

    template <class T> 
    class GraphVertexAttribute : public Attribute<Graph::Vertex, T> {
    public:
        typedef Attribute<Graph::Vertex, T> superclass ;

        void bind(Graph* graph, const std::string& name) {
            superclass::bind(
                graph->vertex_attribute_manager(), name 
            ) ;
        }

        void bind(Graph* graph) { 
            superclass::bind(graph->vertex_attribute_manager()) ; 
        }
        
        GraphVertexAttribute() { }

        GraphVertexAttribute(Graph* graph) {
            bind(graph) ;
        }

        GraphVertexAttribute(Graph* graph, const std::string& name) { 
            bind(graph, name) ;
        }
        
        static bool is_defined(Graph* graph, const std::string& name) {
            return superclass::is_defined(
                graph->vertex_attribute_manager(), name
            ) ;
        }
    } ;

//_________________________________________________________

    template <class T> 
    class GraphHalfedgeAttribute : public Attribute<Graph::Halfedge, T> {
    public:
        typedef Attribute<Graph::Halfedge, T> superclass ;

        void bind(Graph* graph, const std::string& name) {
            superclass::bind(
                graph->halfedge_attribute_manager(), name
            ) ;
        }

        void bind(Graph* graph) { 
            superclass::bind(graph->halfedge_attribute_manager()) ; 
        }
        
        GraphHalfedgeAttribute() { }

        GraphHalfedgeAttribute(Graph* graph) {
            bind(graph) ;
        }

        GraphHalfedgeAttribute(Graph* graph, const std::string& name) { 
            bind(graph, name) ; 
        }
        
        static bool is_defined(Graph* graph, const std::string& name) {
            return superclass::is_defined(
                graph->halfedge_attribute_manager(), name
            ) ;
        }
    } ;

//=====================================================================
//
//                    standard graph attributes
//
//=====================================================================

    class GraphVertexLock : public GraphVertexAttribute<bool> {
    public:
        typedef GraphVertexAttribute<bool> superclass ;
        GraphVertexLock() { }
        GraphVertexLock(Graph* graph) : superclass(graph, "lock") { }
        void bind(Graph* graph) { superclass::bind(graph, "lock") ; }
        static bool is_defined(Graph* graph) {
            return superclass::is_defined(graph,"lock") ;
        }
    } ;

    //_________________________________________________________

}
#endif

