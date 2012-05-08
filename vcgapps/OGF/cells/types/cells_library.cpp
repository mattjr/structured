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
 
#include <OGF/cells/types/cells_library.h>
#include <OGF/cells/io/map_serializer.h>
//#include <OGF/cells/io/map_serializer_inventor.h>
#include <OGF/cells/map/map_builder.h>
#include <OGF/basic/debug/logger.h>
#include <OGF/basic/os/file_manager.h>
//#include <gzstream/gzstream.h>
#include <iostream>
#include <fstream>

namespace OGF {

//_________________________________________________________

    CellsLibrary* CellsLibrary::instance_ = nil ;

    void CellsLibrary::initialize() {
        ogf_assert(instance_ == nil) ;
        instance_ = new CellsLibrary() ;
        Environment::instance()->add_environment(instance_) ;
    }

    void CellsLibrary::terminate() {
        ogf_assert(instance_ != nil) ;
        delete instance_ ;
        instance_ = nil ;
    }

    CellsLibrary::CellsLibrary() {
    }

    CellsLibrary::~CellsLibrary() {
    }

    void CellsLibrary::push_path(const std::string& path) {
        path_.push(path) ;
    }
    
    void CellsLibrary::pop_path() {
        path_.pop() ;
    }
    
    std::string CellsLibrary::current_path() {
        if(path_.size() != 0) {
            return path_.top() ;
        }
        return std::string() ;
    }

    bool CellsLibrary::bind_map_serializer(
        const std::string& extension, MapSerializer* serializer
    ) {
        std::string upper_extension = extension ;
        String::to_uppercase(upper_extension) ;
        if(
            resolve_map_serializer(extension) != nil ||
            resolve_map_serializer(upper_extension) != nil
        ) {
            return false ;
        }
        map_serializers_[extension] = serializer ;
        map_serializers_[upper_extension] = serializer ;
        Environment::notify_observers("object_read_extensions") ;
        Environment::notify_observers("object_write_extensions") ;
        return true ;
    }

    MapSerializer* CellsLibrary::resolve_map_serializer(
        const std::string& extension
    ) const {
        std::map<std::string, MapSerializer_var>::const_iterator it = 
            map_serializers_.find(extension) ;
        if(it == map_serializers_.end()) {
            return nil ;
        }
        return it->second ;
    }


 /*   static bool load_compressed_map(const std::string& file_name_in, Map* map) {
        std::string extension = FileManager::instance()->extension(file_name_in) ;
        ogf_assert(extension == "gz") ;
        std::string file_name = 
            FileManager::instance()->dir_name(file_name_in) + "/" +
            FileManager::instance()->base_name(file_name_in) ;

        extension = FileManager::instance()->extension(file_name) ;

        MapSerializer* serializer = CellsLibrary::instance()->resolve_map_serializer(extension) ;

        if(serializer == nil) {
            Logger::err("CellsLibrary") 
                << "could not find a serializer for extension " 
                << "\'" << extension << "\'"
                << std::endl ;
            return false ;
        }

        if(!serializer->streams_supported()) {
            Logger::err("CellsLibrary") 
                << "MapSerializer for extension " 
                << "\'" << extension << "\'"
                << " does not support streams"
                << std::endl ;
            return false ;
        }

        if(!serializer->read_supported()) {
            Logger::err("CellsLibrary") 
                << "MapSerializer for extension " 
                << "\'" << extension << "\'"
                << " does not support reading"
                << std::endl ;
            return false ;
        }
        
        igzstream in(file_name_in.c_str()) ;
        if(!in) {
            Logger::err("CellsLibrary") 
                << "could not open file \'"
                << file_name_in 
                << "\'" << std::endl ;
            return false ;
        }
        MapBuilder builder(map) ;
        return serializer->serialize_read(in, builder) ;
    }


    static bool save_compressed_map(const std::string& file_name_in, Map* map) {
        std::string extension = FileManager::instance()->extension(file_name_in) ;
        ogf_assert(extension == "gz") ;
        std::string file_name = 
            FileManager::instance()->dir_name(file_name_in) + "/" +
            FileManager::instance()->base_name(file_name_in) ;

        extension = FileManager::instance()->extension(file_name) ;

        MapSerializer* serializer = CellsLibrary::instance()->resolve_map_serializer(extension) ;

        if(serializer == nil) {
            Logger::err("CellsLibrary") 
                << "could not find a serializer for extension " 
                << "\'" << extension << "\'"
                << std::endl ;
            return false ;
        }

        if(!serializer->streams_supported()) {
            Logger::err("CellsLibrary") 
                << "MapSerializer for extension " 
                << "\'" << extension << "\'"
                << " does not support streams"
                << std::endl ;
            return false ;
        }

        if(!serializer->write_supported()) {
            Logger::err("CellsLibrary") 
                << "MapSerializer for extension " 
                << "\'" << extension << "\'"
                << " does not support reading"
                << std::endl ;
            return false ;
        }
        
        ogzstream out(file_name_in.c_str()) ;
        if(!out) {
            Logger::err("CellsLibrary") 
                << "could not open file \'"
                << file_name_in 
                << "\'" << std::endl ;
            return false ;
        }

        return serializer->serialize_write(map, out) ;
    }
*/

    bool CellsLibrary::load_map(const std::string& file_name, Map* map) {
        std::string extension = FileManager::instance()->extension(file_name) ;
        if(extension.length() == 0) {
            Logger::err("CellsLibrary") << "No extension in file name"
                                        << std::endl ;
            return false ;
        }

      /*  if(extension == "gz") {
            return load_compressed_map(file_name, map) ;
        }*/

        MapSerializer* serializer = resolve_map_serializer(extension) ;
        if(serializer == nil) {
            Logger::err("CellsLibrary") 
                << "could not find a serializer for extension " 
                << "\'" << extension << "\'"
                << std::endl ;
            return false ;
        }

        std::string path = FileManager::instance()->dir_name(file_name) ;
        push_path(path) ;

        MapBuilder builder(map) ;
        bool result = serializer->serialize_read(file_name, builder) ;
        
        pop_path() ;

        /*if(dynamic_cast<MapSerializer_Inventor*>(serializer) == nil) {
            map->compute_vertex_normals() ;
        }*/
        return result ;
    }
    
    bool CellsLibrary::save_map(const std::string& file_name, Map* map) {
        std::string extension = FileManager::instance()->extension(file_name) ;
        if(extension.length() == 0) {
            Logger::err("CellsLibrary") << "No extension in file name"
                                        << std::endl ;
            return false ;
        }
        /*if(extension == "gz") {
            return save_compressed_map(file_name, map) ;
        }*/
        MapSerializer* serializer = resolve_map_serializer(extension) ;
        if(serializer == nil) {
            Logger::err("CellsLibrary") 
                << "could not find a serializer for extension " 
                << "\'" << extension << "\'"
                << std::endl ;
            return false ;
        }

        return serializer->serialize_write(file_name, map) ;
    }

    bool CellsLibrary::bind_map_parameterizer_factory(
        const std::string& name, MapParameterizerFactory* factory
    ) {
        if(map_parameterizers_.factory_is_bound(name)) {
            return false ;
        }
        map_parameterizers_.register_factory(name, factory) ;
        Environment::notify_observers("map_parameterizers") ;
        return true ;
    }

    MapParameterizer* CellsLibrary::create_map_parameterizer(
        const std::string& name
    ) {
        if(!map_parameterizers_.factory_is_bound(name)) {
            return nil ;
        }
        return map_parameterizers_.create(name) ;
    }

    bool CellsLibrary::bind_DelaunayAPI2d_factory(
        const std::string& name, DelaunayAPI2dFactory* factory
    ) {
        if(delaunay_APIs_2d_.factory_is_bound(name)) {
            return false ;
        }
        delaunay_APIs_2d_.register_factory(name, factory) ;
        Environment::notify_observers("DelaunayAPIs2d") ;
        return true ;
    }

    DelaunayAPI2d* CellsLibrary::create_DelaunayAPI2d(
        const std::string& name
    ) {
        if(!delaunay_APIs_2d_.factory_is_bound(name)) {
            return nil ;
        }
        return delaunay_APIs_2d_.create(name) ;
    }

    bool CellsLibrary::bind_DelaunayAPI3d_factory(
        const std::string& name, DelaunayAPI3dFactory* factory
    ) {
        if(delaunay_APIs_3d_.factory_is_bound(name)) {
            return false ;
        }
        delaunay_APIs_3d_.register_factory(name, factory) ;
        Environment::notify_observers("DelaunayAPIs3d") ;
        return true ;
    }

    DelaunayAPI3d* CellsLibrary::create_DelaunayAPI3d(
        const std::string& name
    ) {
        if(!delaunay_APIs_3d_.factory_is_bound(name)) {
            return nil ;
        }
        return delaunay_APIs_3d_.create(name) ;
    }


    bool CellsLibrary::bind_SpatialSearch_factory(
        const std::string& name, SpatialSearchFactory* factory
    ) {
        if(kdtree_searchs_.factory_is_bound(name)) {
            return false ;
        }
        kdtree_searchs_.register_factory(name, factory) ;
        Environment::notify_observers("SpatialSearchs") ;
        return true ;
    }

    SpatialSearch* CellsLibrary::create_SpatialSearch(
        const std::string& name
    ) {
        if(!kdtree_searchs_.factory_is_bound(name)) {
            return nil ;
        }
        return kdtree_searchs_.create(name) ;
    }

    bool CellsLibrary::bind_map_component_splitter_factory(
        const std::string& name, MapComponentSplitterFactory* factory
    ) {
        if(map_component_splitters_.factory_is_bound(name)) {
            return false ;
        }
        map_component_splitters_.register_factory(name, factory) ;
        Environment::notify_observers("map_component_splitters") ;
        return true ;
    }

    MapComponentSplitter* CellsLibrary::create_map_component_splitter(
        const std::string& name
    ) {
        if(!map_component_splitters_.factory_is_bound(name)) {
            return nil ;
        }
        return map_component_splitters_.create(name) ;
    }

    bool CellsLibrary::resolve(
        const std::string& name, std::string& value
    ) const {
        if(name == "object_read_extensions") {
            value = "" ;
            for(std::map<std::string, MapSerializer_var>::const_iterator 
                    it = map_serializers_.begin(); 
                it != map_serializers_.end(); it++
            ) {
                if(it->second->read_supported()) {
                    if(value.length() != 0) {
                        value += ";" ;
                    }
                    value += "*." ;
                    value += it->first ;
                }
            }
            value += ";*.gz" ;
            return true ;
        } else if(name=="object_write_extensions") {
            value = "" ;
            for(std::map<std::string, MapSerializer_var>::const_iterator 
                    it = map_serializers_.begin(); 
                it != map_serializers_.end(); it++
            ) {
                if(it->second->write_supported()) {
                    if(value.length() != 0) {
                        value += ";" ;
                    }
                    value += "*." ;
                    value += it->first ;
                }
            }
            value += ";*.gz" ;
            return true ;
        } else if(name=="map_parameterizers") {
            value = map_parameterizers_.factory_names() ;
            return true ;
        } else if(name=="DelaunayAPIs2d") {
            value = delaunay_APIs_2d_.factory_names() ;
            return true ;
        } else if(name=="DelaunayAPIs3d") {
            value = delaunay_APIs_3d_.factory_names() ;
            return true ;
	} else if(name=="SpatialSearchs") {
            value = kdtree_searchs_.factory_names() ;
            return true ;
        } else if(name == "map_component_splitters") {
            value = map_component_splitters_.factory_names() ;
            return true ;
        } else {
            return false ;
        }
    }

//_________________________________________________________

}

