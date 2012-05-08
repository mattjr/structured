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
 

#include <OGF/parameterizer/io/map_serializer_abf.h>
#include <OGF/cells/map/map_builder.h>
#include <OGF/cells/map/map_editor.h>
#include <OGF/cells/map_algos/enumerate.h>
#include <OGF/basic/debug/logger.h>
#include <OGF/basic/attributes/attribute.h>

namespace OGF {

//_________________________________________________________

    MapSerializer_abf::MapSerializer_abf() {
        read_supported_ = true ;
        write_supported_ = false ;
    }

    bool MapSerializer_abf::serialize_read(
        std::istream& input, AbstractMapBuilder& builder_in
    ) {

        MapBuilder* builder = dynamic_cast<MapBuilder*>(&builder_in) ;

        ogf_assert(builder != nil) ;

        Attribute<Map::Halfedge, double> angle(
            builder->map()->halfedge_attribute_manager(),
            "angle"
        ) ;

        // Vertices indexes start by 1 in abf format.

        int nb_vertices ;
        int nb_facets ;

        input >> nb_vertices >> nb_facets ;
        
        builder->begin_surface() ;
        {for(int i=0; i<nb_vertices; i++) {
            int index ;
            Point3d p ;
            input >> index >> p ;
            builder->add_vertex(p) ;
        }}
        {for(int i=0; i<nb_facets; i++) {

            int index ;
            int p1,p2,p3 ;
            double a1,a2,a3 ;

            input >> index >> p1 >> p2 >> p3 >> a1 >> a2 >> a3 ;

            builder->begin_facet() ;
            builder->add_vertex_to_facet(p1-1) ;
            builder->add_vertex_to_facet(p2-1) ;
            builder->add_vertex_to_facet(p3-1) ;
            builder->end_facet() ;

            Map::Facet* f = builder->current_facet() ;
            
            Map::Halfedge* h2 = f->halfedge() ;
            Map::Halfedge* h3 = h2->next() ;
            Map::Halfedge* h1 = h3->next() ;

            angle[h1] = a1 ;
            angle[h2] = a2 ;
            angle[h3] = a3 ;

        }}

        builder->end_surface() ;
        
        return true ;
    }
    
    bool MapSerializer_abf::serialize_write(
        Map* map, std::ostream& output
    ) {
        return false ; // Not implemented yet ...
    }

//_________________________________________________________


    MapSerializer_fl::MapSerializer_fl() {
        read_supported_ = true ;
        write_supported_ = true ;
    }

    bool MapSerializer_fl::serialize_read(
        std::istream& input, AbstractMapBuilder& builder_in
    ) {

        MapBuilder* builder = dynamic_cast<MapBuilder*>(&builder_in) ;

        ogf_assert(builder != nil) ;

        // Vertices indexes start by 1 in abf format.
        
        int nb_vertices ;
        int nb_facets ;
        
        input >> nb_vertices >> nb_facets ;
        
        builder->begin_surface() ;
        {for(int i=0; i<nb_vertices; i++) {
            int index ;
            Point3d p ;
            input >> index >> p ;
            builder->add_vertex(p) ;
        }}
        {for(int i=0; i<nb_facets; i++) {

            int index ;
            int p1,p2,p3 ;

            input >> index >> p1 >> p2 >> p3 ;

            builder->begin_facet() ;
            builder->add_vertex_to_facet(p1-1) ;
            builder->add_vertex_to_facet(p2-1) ;
            builder->add_vertex_to_facet(p3-1) ;
            builder->end_facet() ;
        }}

        builder->end_surface() ;
        
        return true ;
    }
    
    bool MapSerializer_fl::serialize_write(
        Map* map, std::ostream& output
    ) {

        Logger::out("MapSerializer_fl")
            << "sanity: triangulating model"
            << std::endl ;
        MapEditor editor(map) ;
        editor.split_surface(split_triangulate) ;

        output << map->size_of_vertices() << " " 
               << map->size_of_facets() << " "
               << std::endl ;

        Attribute<Vertex,int> vertex_id(
            map->vertex_attribute_manager()
        ) ;
        
        // Alla's files numbering starts with 1
        enumerate_vertices(map, vertex_id, 1) ;

        // Output Vertices
        { FOR_EACH_VERTEX(Map,map,it) {
            output 
                << vertex_id[it]    << " "
                << it->point().x() << " " 
                << it->point().y() << " " 
                << it->point().z() << std::endl ;
        }} 

        // Output facets

        int fid = 1 ;
        { FOR_EACH_FACET(Map,map,it) {
            output << fid << "   " ;
            fid++ ;
            Halfedge* jt = it->halfedge() ;
            do {
                output << vertex_id[jt-> vertex()] 
                       << " " ;
                jt = jt->next() ;
            } while(jt != it->halfedge()) ;
            output << std::endl ;
        }}
        
        return true ;
    }

//_________________________________________________________

}

