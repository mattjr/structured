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
 

#include <OGF/parameterizer/io/map_serializer_jmd.h>
#include <OGF/cells/map_algos/enumerate.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/basic/debug/logger.h>

namespace OGF {

//_________________________________________________________

    MapSerializer_JMD::MapSerializer_JMD() {
        read_supported_ = false ;
        write_supported_ = true ;
    }

    bool MapSerializer_JMD::serialize_read(
        std::istream& input, AbstractMapBuilder& builder
    ) {
        Logger::err("MapSerializer") 
            << "\'read\' is not implemented yet for the JMD format" 
            << std::endl ;
        
        return false ;
    }

	bool MapSerializer_JMD::serialize_write(
        Map* map, std::ostream& out
    ) {
		MapVertexAttribute<Vector3d> tangent ;
		if(!tangent.is_defined(map, "tangent")) {
			Logger::warn("MapSerializer_JMD") << "missing tangent attribute"
											  << std::endl ;	
		}
		tangent.bind(map, "tangent") ;

		MapTexVertexNormal normal(map) ;

		MapVertexAttribute<int> vertex_id(map) ;
		enumerate_vertices(map, vertex_id, 1) ;

		out << map->size_of_vertices() << std::endl ;
		{FOR_EACH_VERTEX(Map, map, it) {
			out << it->point() << " " << it->halfedge()->tex_coord() 
				<< " " << normal[it->halfedge()->tex_vertex()]
				<< " " << tangent[it]
				<< std::endl ;
		}}

		out << map->size_of_facets() << std::endl ;
		{FOR_EACH_FACET(Map, map, it) {
			out << it->nb_vertices() << " " ;
			Map::Halfedge* h = it->halfedge() ;
			do {
				out << vertex_id[h->vertex()] << " " ;
				h = h->next() ;
			} while(h != it->halfedge()) ;
			out << std::endl ;
		}}

		return true ;
	}

/*
    bool MapSerializer_JMD::serialize_write(
        Map* map, std::ostream& output
    ) {
        int nb_border_halfedges = 0 ;
        { FOR_EACH_HALFEDGE(Map,map,it) {
            if(it-> is_border()) {
                nb_border_halfedges++ ;
            }
        }}
        
        int nb_vertices = map-> size_of_vertices() ;
        int nb_halfedges = 
            map-> size_of_halfedges() - nb_border_halfedges ;
        int nb_facets   = map-> size_of_facets() ;

        Attribute<Vertex,int> vertex_id(map->vertex_attribute_manager());
        Attribute<TexVertex,int> tex_vertex_id(
            map->tex_vertex_attribute_manager()
        ) ;

        //=========== off file =================
        
        output << "OFF" << std::endl ;
        output 
            << nb_vertices  << " " 
            << nb_facets    << " "
            << nb_halfedges << std::endl ;
        
        // Enumerate and output vertices
        int id = 0 ; // OFF files numbering starts with 0
        { FOR_EACH_VERTEX(Map,map,it) {
            output << it-> point().x() << " " 
                   << it-> point().y() << " " 
                   << it-> point().z() << std::endl ;
            vertex_id[it] = id ;
            id++ ;
        } }
        
        // Output facets
        { FOR_EACH_FACET(Map,map,it) {
            // count vertices in facet
            int nb_vertices_in_facet = 0 ;
            Halfedge* jt = it->halfedge() ;
            do {
                nb_vertices_in_facet++ ;
                jt = jt->next() ;
            } while(jt != it->halfedge()) ;
            
            output << nb_vertices_in_facet << " " ;
            
            jt = it->halfedge() ;
            do {
                output << vertex_id[jt-> vertex()] << " " ;
                jt=jt->next() ;
            } while(jt != it->halfedge()) ;
            output << std::endl ;
        }}

        //=========== map file =================

        std::ostream& map_output = output ;
        map_output << "MAP" << std::endl ;

        // count TexVertices
        // (note: cannot enumerate and output at the same
        //  time, since the number of tex vertices should
        //  appear in the file prior to the tex vertices).

        {FOR_EACH_HALFEDGE(Map,map,it) {
            tex_vertex_id[it->tex_vertex()] = -1 ;
        }}
        
        int nb_tex_vertices = 0 ;

        {FOR_EACH_FACET(Map,map,it) {
            Halfedge* jt = it->halfedge() ;
            do {
                if(tex_vertex_id[jt-> tex_vertex()] == -1) {
                    tex_vertex_id[jt-> tex_vertex()] = 1 ;
                    nb_tex_vertices++ ;
                }
                jt = jt->next() ;
            } while(jt != it->halfedge()) ;
        }}
        
        map_output << nb_tex_vertices << " " << nb_halfedges << " "
                   << -1 << std::endl ;
        
        
        // Enumerate and output tex vertices
        {FOR_EACH_HALFEDGE(Map,map,it) {
            tex_vertex_id[it->tex_vertex()] = -1 ;
        }}
        id = 0 ;

        {FOR_EACH_FACET(Map,map,it) {
            Halfedge* jt = it->halfedge() ;
            do {
                if(tex_vertex_id[jt-> tex_vertex()] == -1) {
                    tex_vertex_id[jt-> tex_vertex()] = id ;
                    map_output << jt-> tex_vertex()-> tex_coord().x()
                               << " "
                               << jt-> tex_vertex()-> tex_coord().y()
                               << std::endl ;
                    id++ ;
                }
                jt = jt->next() ;
            } while(jt != it->halfedge()) ;
        }}

        // Output facet's texvertices
        {FOR_EACH_FACET(Map,map,it) {
            Halfedge* jt = it->halfedge() ;
            do {
                map_output << tex_vertex_id[jt-> tex_vertex()]
                           << std::endl ;
                jt = jt->next() ;
            } while(jt != it->halfedge()) ;
        }}

        //=========== gmap file =================

        std::ostream& gmap_output = output ;
        gmap_output << "GMAP" << std::endl ;
        gmap_output 
            << nb_vertices  << " " 
            << nb_facets    << " "
            << nb_halfedges << std::endl ;

        // header
        gmap_output << 1 << std::endl ;
        gmap_output << 0 << std::endl ;


        // not needed anymore, free associated memory
        vertex_id.unbind() ;
        tex_vertex_id.unbind() ;

        // enumerate facets
        Attribute<Facet,int> facet_id(map->facet_attribute_manager()) ;
        enumerate_facets(map, facet_id) ;

        {FOR_EACH_VERTEX(Map,map,it) {
            int nb_incident_facets = 0 ;
            Halfedge* jt = it->halfedge() ;
            do {
                if(jt-> facet() != nil) {
                    nb_incident_facets++ ;
                }
                jt = jt->next_around_vertex() ;
            } while(jt != it->halfedge()) ;
            
            gmap_output << nb_incident_facets << " " ;
            
            jt = it->halfedge() ;
            do {
                if(jt-> facet() != nil) {
                    gmap_output << facet_id[jt-> facet()] << " " ;
                }
                jt = jt->next_around_vertex() ;
            } while(jt != it->halfedge()) ;
            gmap_output << std::endl ;
        }}
        
        {FOR_EACH_FACET(Map,map,it) {
            int nb_adjacent_facets = 0 ;
            Halfedge* jt = it->halfedge() ;
            do {
                if(jt-> opposite()-> facet() != nil) {
                    nb_adjacent_facets++ ;
                }
                jt = jt->next() ;
            } while(jt != it->halfedge()) ;
            
            gmap_output << nb_adjacent_facets << " " ;
            
            jt = it-> halfedge() ;
            do {
                if(jt-> opposite()-> facet() != nil) {
                    gmap_output << facet_id[jt-> opposite()-> facet()] << " " ;
                }
                jt++ ;
            } while(jt != it-> halfedge()) ;
            gmap_output << std::endl ;
        }}

        // trailer
        gmap_output << 0 << std::endl ;

        return true ;
    }
*/

//_________________________________________________________

}

