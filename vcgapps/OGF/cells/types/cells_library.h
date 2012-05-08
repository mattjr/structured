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
 

#ifndef __OGF_CELLS_TYPES_CELLS_LIBRARY__
#define __OGF_CELLS_TYPES_CELLS_LIBRARY__

#include <OGF/cells/common/common.h>
#include <OGF/cells/io/map_serializer.h>
#include <OGF/basic/os/environment.h>
#include <OGF/basic/types/counted.h>
#include <OGF/basic/types/smart_pointer.h>
#include <OGF/basic/types/basic_factory.h>

#include <string>
#include <map>
#include <stack>

namespace OGF {

//_________________________________________________________

    class NamingContext ;
    class MapSerializer ;
    class Map ;
    class MapParameterizer ;
    class DelaunayAPI2d ;
    class DelaunayAPI3d ;
    class MapComponentSplitter ;
    class SpatialSearch ;

//_________________________________________________________

    typedef BasicFactory<MapParameterizer> MapParameterizerFactory ;
    typedef BasicFactory<DelaunayAPI2d> DelaunayAPI2dFactory ;
    typedef BasicFactory<DelaunayAPI3d> DelaunayAPI3dFactory ;
    typedef BasicFactory<SpatialSearch> SpatialSearchFactory ;
    typedef BasicFactory<MapComponentSplitter> MapComponentSplitterFactory ;

    class CELLS_API CellsLibrary : public Environment {
    public:
        static CellsLibrary* instance() { return instance_ ; }
        static void initialize() ;
        static void terminate() ;
        
        bool bind_map_serializer(
            const std::string& extension, MapSerializer* serializer
        ) ;
        
        MapSerializer* resolve_map_serializer(
            const std::string& extension
        ) const ;

        bool load_map(const std::string& file_name, Map* map) ;
        bool save_map(const std::string& file_name, Map* map) ;

        void push_path(const std::string& path) ;
        void pop_path() ;
        std::string current_path() ;

        bool bind_map_parameterizer_factory(
            const std::string& name, MapParameterizerFactory* factory
        ) ;
        MapParameterizer* create_map_parameterizer(const std::string& name) ;

        //----------- Delaunay new API ------------------------------
        bool bind_DelaunayAPI2d_factory(const std::string& name, DelaunayAPI2dFactory* factory) ;
        DelaunayAPI2d* create_DelaunayAPI2d(const std::string& name) ;

        bool bind_DelaunayAPI3d_factory(const std::string& name, DelaunayAPI3dFactory* factory) ;
        DelaunayAPI3d* create_DelaunayAPI3d(const std::string& name) ;


        //----------- Spatial search --------------------------------
        bool bind_SpatialSearch_factory(
            const std::string& name, SpatialSearchFactory* factory 
        ) ;
        SpatialSearch* create_SpatialSearch(const std::string& name) ;

	//----------- Map component splitter ------------------------
        bool bind_map_component_splitter_factory(
            const std::string& name, MapComponentSplitterFactory* factory 
        ) ;
        MapComponentSplitter* create_map_component_splitter(const std::string& name) ;

        virtual bool resolve(const std::string& name, std::string& value) const ;


    protected:
        CellsLibrary() ;
        ~CellsLibrary() ;
        friend class World ;
        
    private:
        static CellsLibrary* instance_ ;
        std::map<std::string, MapSerializer_var> map_serializers_ ;
        std::stack<std::string> path_ ;

        BasicFactories<MapParameterizer> map_parameterizers_ ;
        BasicFactories<DelaunayAPI2d> delaunay_APIs_2d_ ;
        BasicFactories<DelaunayAPI3d> delaunay_APIs_3d_ ;
        BasicFactories<SpatialSearch> kdtree_searchs_ ; 
        BasicFactories<MapComponentSplitter> map_component_splitters_ ; 
    } ;
    
//_________________________________________________________
    
    template <class T> class ogf_declare_map_serializer {
    public:
        ogf_declare_map_serializer(const std::string& extension) {
            bool ok = OGF::CellsLibrary::instance()->bind_map_serializer(
                extension, new T
            ) ;
            ogf_assert(ok) ;
        }
    } ;

//_________________________________________________________

    template <class T> class ogf_declare_map_parameterizer {
    public:
        ogf_declare_map_parameterizer(const std::string& name) {
            bool ok = OGF::CellsLibrary::instance()->bind_map_parameterizer_factory(
                name, new GenericBasicFactory<MapParameterizer,T>
            ) ;
            ogf_assert(ok) ;
        }
    } ;

//_________________________________________________________

    template <class T> class ogf_declare_DelaunayAPI2d {
    public:
        ogf_declare_DelaunayAPI2d(const std::string& name) {
            bool ok = OGF::CellsLibrary::instance()->bind_DelaunayAPI2d_factory(
                name, new GenericBasicFactory<DelaunayAPI2d,T>
            ) ;
            ogf_assert(ok) ;
        }
    } ;

//_________________________________________________________

    template <class T> class ogf_declare_DelaunayAPI3d {
    public:
        ogf_declare_DelaunayAPI3d(const std::string& name) {
            bool ok = OGF::CellsLibrary::instance()->bind_DelaunayAPI3d_factory(
                name, new GenericBasicFactory<DelaunayAPI3d,T>
            ) ;
            ogf_assert(ok) ;
        }
    } ;

//_________________________________________________________
    
    template <class T> class ogf_declare_SpatialSearch {
    public:
        ogf_declare_SpatialSearch(const std::string& name) {
            bool ok = OGF::CellsLibrary::instance()->bind_SpatialSearch_factory(
                name, new GenericBasicFactory<SpatialSearch,T>
            ) ;
            ogf_assert(ok) ;
        }
    } ;

//_________________________________________________________
    
    template <class T> class ogf_declare_map_component_splitter {
    public:
        ogf_declare_map_component_splitter(const std::string& name) {
            bool ok = OGF::CellsLibrary::instance()->bind_map_component_splitter_factory(
                name, new GenericBasicFactory<MapComponentSplitter,T>
            ) ;
            ogf_assert(ok) ;
        }
    } ;

//_________________________________________________________

}
#endif

