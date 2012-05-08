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
 

#include <OGF/parameterizer/io/map_serializer_m.h>
#include <OGF/cells/map/map_builder.h>
#include <OGF/cells/map/map_editor.h>
#include <OGF/cells/map_algos/enumerate.h>
#include <OGF/basic/debug/logger.h>
#include <OGF/basic/attributes/attribute.h>

#include <sstream>

namespace OGF {

//_________________________________________________________

    MapSerializer_m::MapSerializer_m() {
        read_supported_ = true ;
        write_supported_ = false ;
    }

    bool MapSerializer_m::serialize_read(
        std::istream& input, AbstractMapBuilder& builder_in
    ) {
        MapBuilder& builder = dynamic_cast<MapBuilder&>(builder_in) ;
        set_target(builder.target()) ;
        MapVertexAttribute<int> vertex_id(builder.target()) ;

        std::vector<Map::Facet*> facets ;
        builder.begin_surface() ;
        while(input) {
            char line[1024] ;
            input.getline(line, 1024) ;

            std::istringstream line_input(line) ;

            std::string keyword ;
            line_input >> keyword ;

            if(keyword == "Vertex") {
                int id ;
                Point3d p ;
                line_input >> id >> p ;
                builder.add_vertex(id, p) ;
                vertex_id[builder.current_vertex()] = id ;
                char* puv = (char*)strstr(line, "uv") ;
                if(puv != nil) {
                    puv = strchr(puv, '(') ;
                    if(puv != nil) {
                        puv++ ;
                        float u,v ;
                        sscanf(puv,"%f %f", &u, &v) ;
                        builder.add_tex_vertex(id, Point2d(u,v)) ;
                    } else {
                        builder.add_tex_vertex(id, Point2d(0,0)) ;
                    }
                }
            } else if(keyword == "Face") {
                int id, p1,p2,p3 ;
                line_input >> id >> p1 >> p2 >> p3 ;
                builder.begin_facet() ;
                builder.add_vertex_to_facet(p1) ;
                builder.set_corner_tex_vertex(p1) ;
                builder.add_vertex_to_facet(p2) ;
                builder.set_corner_tex_vertex(p2) ;
                builder.add_vertex_to_facet(p3) ;
                builder.set_corner_tex_vertex(p3) ;
                builder.end_facet() ;
                while((int)facets.size() <= id) {
                    facets.push_back(nil) ;
                }
                facets[id] = builder.current_facet() ;
            } else if(keyword == "Corner") {
                int idf,idv ;
                float u = 0 ;
                float v = 0 ;
                line_input >> idv >> idf ;                
                char* puv = (char*)strstr(line, "uv") ;
                if(puv != nil) {
                    puv = strchr(puv, '(') ;
                    if(puv != nil) {
                        puv++ ;
                        sscanf(puv,"%f %f", &u, &v) ;
                    }
                }
                Map::Halfedge* h = facets[idf]->halfedge() ;
                do {
                    if(vertex_id[h->vertex()] == idv) {
                        Map::TexVertex* tv = new_tex_vertex() ;
                        tv->set_tex_coord(Point2d(u,v)) ;
                        set_halfedge_tex_vertex(h, tv) ;
                    }
                    h = h->next() ;
                } while(h != facets[idf]->halfedge()) ;
            } else if(keyword == "#") {
                std::string second_keyword ;
                line_input >> second_keyword ;
                if(second_keyword == "anchor") {
                    int index ;
                    line_input >> index ;
                    builder.lock_vertex(index - 1) ;
                } 
            } 
        }
        
        builder.end_surface() ;
        
        return true ;
    }
    
    bool MapSerializer_m::serialize_write(
        Map* map, std::ostream& output
    ) {
        return false ; // Not implemented yet ...
    }

//_________________________________________________________


}

