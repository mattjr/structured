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
 

#ifndef ___MAP_ATTRIBUTES__
#define ___MAP_ATTRIBUTES__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map.h>
#include <OGF/basic/types/counted.h>
#include <OGF/math/attributes/attribute_adapter.h>

#include <string>
#include <vector>
#include <iostream>

namespace OGF {

//_________________________________________________________

    template <class T> 
    class MapTexVertexAttribute : public Attribute<Map::TexVertex, T> {
    public:
        typedef Attribute<Map::TexVertex, T> superclass ;

        void bind(Map* map, const std::string& name) {
            superclass::bind(
                map->tex_vertex_attribute_manager(), name
            ) ;
        }

        void bind(Map* map) { 
            superclass::bind(map->tex_vertex_attribute_manager()) ; 
        }
        
        bool bind_if_defined(Map* map, const std::string& name) {
            return superclass::bind_if_defined(map->tex_vertex_attribute_manager(), name) ;
        }

        MapTexVertexAttribute() { }

        MapTexVertexAttribute(Map* map) {
            bind(map) ;
        }

        MapTexVertexAttribute(Map* map, const std::string& name) { 
            bind(map, name) ; 
        }
        
        static bool is_defined(Map* map, const std::string& name) {
            return superclass::is_defined(
                map->tex_vertex_attribute_manager(), name
            ) ;
        }
    } ;

//_________________________________________________________

    template <class T> 
    class MapVertexAttribute : public Attribute<Map::Vertex, T> {
    public:
        typedef Attribute<Map::Vertex, T> superclass ;

        void bind(Map* map, const std::string& name) {
            superclass::bind(
                map->vertex_attribute_manager(), name 
            ) ;
        }

        void bind(Map* map) { 
            superclass::bind(map->vertex_attribute_manager()) ; 
        }
        
        bool bind_if_defined(Map* map, const std::string& name) {
            return superclass::bind_if_defined(map->vertex_attribute_manager(), name) ;
        }

        MapVertexAttribute() { }

        MapVertexAttribute(Map* map) {
            bind(map) ;
        }

        MapVertexAttribute(Map* map, const std::string& name) { 
            bind(map, name) ;
        }
        
        static bool is_defined(Map* map, const std::string& name) {
            return superclass::is_defined(
                map->vertex_attribute_manager(), name
            ) ;
        }
    } ;

//_________________________________________________________

    template <class T> 
    class MapHalfedgeAttribute : public Attribute<Map::Halfedge, T> {
    public:
        typedef Attribute<Map::Halfedge, T> superclass ;

        void bind(Map* map, const std::string& name) {
            superclass::bind(
                map->halfedge_attribute_manager(), name
            ) ;
        }

        void bind(Map* map) { 
            superclass::bind(map->halfedge_attribute_manager()) ; 
        }
        
        bool bind_if_defined(Map* map, const std::string& name) {
            return superclass::bind_if_defined(map->halfedge_attribute_manager(), name) ;
        }

        MapHalfedgeAttribute() { }

        MapHalfedgeAttribute(Map* map) {
            bind(map) ;
        }

        MapHalfedgeAttribute(Map* map, const std::string& name) { 
            bind(map, name) ; 
        }
        
        static bool is_defined(Map* map, const std::string& name) {
            return superclass::is_defined(
                map->halfedge_attribute_manager(), name
            ) ;
        }
    } ;

//_________________________________________________________

    template <class T> 
    class MapFacetAttribute : public Attribute<Map::Facet, T> {
    public:
        typedef Attribute<Map::Facet, T> superclass ;

        void bind(Map* map, const std::string& name) {
            superclass::bind(
                map->facet_attribute_manager(), name
            ) ;
        }

        void bind(Map* map) { 
            superclass::bind(map->facet_attribute_manager()) ; 
        }

        bool bind_if_defined(Map* map, const std::string& name) {
            return superclass::bind_if_defined(map->facet_attribute_manager(), name) ;
        }
        
        MapFacetAttribute() { }

        MapFacetAttribute(Map* map) {
            bind(map) ;
        }

        MapFacetAttribute(Map* map, const std::string& name) { 
            bind(map, name) ; 
        }
        
        static bool is_defined(Map* map, const std::string& name) {
            return superclass::is_defined(
                map->facet_attribute_manager(), name
            ) ;
        }
    } ;

    //_________________________________________________________

    class MapTexVertexAttributeAdapter : public AttributeAdapter<Map::TexVertex> {
    public:
        typedef AttributeAdapter<Map::TexVertex> superclass ;
        bool bind_if_defined(Map* map, const std::string& name) {
            return superclass::bind_if_defined(map->tex_vertex_attribute_manager(), name) ;
        }
    } ;

    class MapVertexAttributeAdapter : public AttributeAdapter<Map::Vertex> {
    public:
        typedef AttributeAdapter<Map::Vertex> superclass ;
        bool bind_if_defined(Map* map, const std::string& name) {
            return superclass::bind_if_defined(map->vertex_attribute_manager(), name) ;
        }
    } ;

    class MapHalfedgeAttributeAdapter : public AttributeAdapter<Map::Halfedge> {
    public:
        typedef AttributeAdapter<Map::Halfedge> superclass ;
        bool bind_if_defined(Map* map, const std::string& name) {
            return superclass::bind_if_defined(map->halfedge_attribute_manager(), name) ;
        }
    } ;

    class MapFacetAttributeAdapter : public AttributeAdapter<Map::Facet> {
    public:
        typedef AttributeAdapter<Map::Facet> superclass ;
        bool bind_if_defined(Map* map, const std::string& name) {
            return superclass::bind_if_defined(map->facet_attribute_manager(), name) ;
        }
    } ;

//=====================================================================
//
//                    standard map attributes
//
//=====================================================================

    class MapTexVertexNormal : public MapTexVertexAttribute<Vector3d> {
    public:
        typedef MapTexVertexAttribute<Vector3d> superclass ;
        MapTexVertexNormal() { }
        MapTexVertexNormal(Map* map) : superclass(map, "normal") { }
        void bind(Map* map) { superclass::bind(map, "normal");  }
        static bool is_defined(Map* map) {
            return superclass::is_defined(map,"normal") ;
        }
    } ;

    //_________________________________________________________

    class MapVertexLock : public MapVertexAttribute<bool> {
    public:
        typedef MapVertexAttribute<bool> superclass ;
        MapVertexLock() { }
        MapVertexLock(Map* map) : superclass(map, "lock") { }
        void bind(Map* map) { superclass::bind(map, "lock") ; }
        static bool is_defined(Map* map) {
            return superclass::is_defined(map,"lock") ;
        }
    } ;

    //_________________________________________________________

    class MapFacetMaterialId : public MapFacetAttribute<int> {
    public:
        typedef MapFacetAttribute<int> superclass ;
        MapFacetMaterialId() { }
        MapFacetMaterialId(Map* map) : superclass(map, "material_id") { }
        void bind(Map* map) { superclass::bind(map, "material_id") ; }
        static bool is_defined(Map* map) {
            return superclass::is_defined(map,"material_id") ;
        }
    } ;

    //___________________________________________________________________

    class MapFacetNormal : public MapFacetAttribute<Vector3d> {
    public:
        typedef MapFacetAttribute<Vector3d> superclass ;
        MapFacetNormal() { }
        MapFacetNormal(Map* map) : superclass(map, "normal") { }
        void bind(Map* map) { superclass::bind(map, "normal");  }
        static bool is_defined(Map* map) {
            return superclass::is_defined(map,"normal") ;
        }
    } ;

    //___________________________________________________________________

}
#endif

