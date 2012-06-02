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
 

#include <OGF/cells/io/map_serializer_obj.h>
#include <OGF/cells/io/generic_attributes_io.h>
#include <OGF/cells/map/map_builder.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map_algos/enumerate.h>
#include <OGF/basic/io/line_stream.h>
#include <OGF/basic/os/file_system.h>

namespace OGF {

//_________________________________________________________

    MapSerializer_obj::MapSerializer_obj() {
        read_supported_ = true ;
        write_supported_ = true ;
    }

    bool MapSerializer_obj::serialize_read(
        const std::string& file_name, AbstractMapBuilder& out
    ) {
        current_directory_ = FileSystem::dir_name(file_name) ;
        return MapSerializer::serialize_read(file_name, out) ;
        current_directory_ = "" ;
    }

    void MapSerializer_obj::read_mtl_lib(std::istream& input) {
        LineInputStream in(input) ;
        std::string keyword ;
        std::string cur_mtl = "default" ;
        while(!in.eof()) {
            in.get_line() ;
            in >> keyword ;
            if(keyword == "newmtl") {
                in >> cur_mtl ;
            } else if(keyword == "Kd") {
                Color c ;
                in >> c ;
                material_lib_[cur_mtl] = c ;
            }
        }        
    }

    bool MapSerializer_obj::serialize_read(
        std::istream& input, AbstractMapBuilder& builder
    ) {
        MapFacetAttribute<Color> color_ ;
        LineInputStream in(input) ;
        MapBuilder* concrete_builder = dynamic_cast<MapBuilder*>(&builder) ;

        builder.begin_surface() ;
        while(!in.eof()) {
            in.get_line() ;
            
            std::string keyword ;
            in >> keyword ;
            
            if(keyword == "v") {
                Point3d p ;
                in >> p ;
                builder.add_vertex(p) ;
            } else if(keyword == "vt") {
                Point2d q ;
                in >> q ;
                builder.add_tex_vertex(q) ;
            } else if(keyword == "f") {
                builder.begin_facet() ;
                while(!in.eol()) {
                    std::string s ;
                    in >> s ;
                    if(s.length() > 0) {
                        std::istringstream v_input(s) ;
                        int index ;
                        v_input >> index ;
                        builder.add_vertex_to_facet(index - 1) ;
                        char c ;
                        v_input >> c ;
                        if(c == '/') {
                            v_input >> index ;
                            builder.set_corner_tex_vertex(index - 1) ;
                        }
                    }
                }
                builder.end_facet() ;
                if(color_.is_bound() && concrete_builder != nil) {
                    color_[concrete_builder->current_facet()] = current_material_ ;
                }
            } else if(keyword == "#") {
                std::string second_keyword ;
                in >> second_keyword ;
                if(second_keyword == "anchor") {
                    int index ;
                    in >> index ;
                    builder.lock_vertex(index - 1) ;
                } 
            } else if(keyword == "mtllib") {
                std::string mtl_lib_filename ;
                in >> mtl_lib_filename ;
                mtl_lib_filename = current_directory_ + "/" + mtl_lib_filename ;
                std::ifstream mtl_lib_in(mtl_lib_filename.c_str()) ;
                if(mtl_lib_in) {
                    Logger::out("MapSerializer_obj") << "using material lib: " << mtl_lib_filename << std::endl ;
                    read_mtl_lib(mtl_lib_in) ;
                } else {
                    Logger::err("MapSerializer_obj") << mtl_lib_filename << ": no such file" << std::endl ;
                }
            } else if(keyword == "usemtl") {
                std::string material ;
                in >> material ;
                MaterialLib::iterator it = material_lib_.find(material) ;
                if(it == material_lib_.end()) {
                    current_material_ = Color(1,1,1,1) ;
                } else {
                    current_material_ = it->second ;
                }
                if(!color_.is_bound() && concrete_builder != nil) {
                    color_.bind(builder.map(), "color") ;
                }
            }
        }
        
        builder.end_surface() ;
        color_.unbind() ;
        material_lib_.clear() ;
        return true ;
    }
    
    bool MapSerializer_obj::serialize_write(
        Map* map, std::ostream& output
    ) {
        Attribute<Vertex,int> vertex_id(
            map->vertex_attribute_manager()
        ) ;
        Attribute<TexVertex,int> tex_vertex_id(
            map->tex_vertex_attribute_manager()
        ) ;

        MapVertexLock is_locked(map) ;
        
        // Obj files numbering starts with 1 (instead of 0)
        enumerate_vertices(map, vertex_id, 1) ;

        // Output Vertices
        { FOR_EACH_VERTEX(Map,map,it) {
            output << "v " 
                   << it-> point().x() << " " 
                   << it-> point().y() << " " 
                   << it-> point().z() << std::endl ;
        }} 

        // Enumerate and output TexVertices
        { FOR_EACH_HALFEDGE(Map, map, it) {
            tex_vertex_id[it->tex_vertex()] = 0 ;
        }}
        
        int cur_id = 1 ;
        
        { FOR_EACH_FACET(Map, map, it) {
            Halfedge* jt = it->halfedge() ;
            do {
                if(tex_vertex_id[jt-> tex_vertex()] == 0) {
                    tex_vertex_id[jt->tex_vertex()] = cur_id ;
                    output << "vt " 
                           << jt-> tex_vertex()-> tex_coord().x()
                           << " "
                           << jt-> tex_vertex()-> tex_coord().y()
                           << std::endl ;
                    cur_id++ ;
                }
                jt = jt->next() ;
            } while(jt != it->halfedge()) ;
        }}  

        // Output facets

        { FOR_EACH_FACET(Map,map,it) {
            Halfedge* jt = it->halfedge() ;
            output << "f " ;
            do {
                output << vertex_id[jt-> vertex()]
                       << "/"
                       << tex_vertex_id[jt-> tex_vertex()]
                       << " " ;
                jt = jt->next() ;
            } while(jt != it->halfedge()) ;
            output << std::endl ;
        }}
        
        { FOR_EACH_VERTEX(Map,map,it) {
            if(is_locked[it]) {
                output << "# anchor " << vertex_id[it] << std::endl ;
            }
        }}

        return true ;
    }

//_________________________________________________________

    MapSerializer_eobj::MapSerializer_eobj() : MapSerializer_obj() {
    }

    static Map::Halfedge* find_halfedge_between(Map::Vertex* v1, Map::Vertex* v2) {
        Map::Halfedge* h = v2->halfedge() ;
        do {
            if(h->opposite()->vertex() == v1) {
                return h ;
            }
            h = h->next_around_vertex() ;
        } while(h != v2->halfedge()) ;
        return nil ;
    }


    bool MapSerializer_eobj::serialize_read(
        std::istream& input, AbstractMapBuilder& out
    ) {

        MapBuilder& builder = dynamic_cast<MapBuilder&>(out) ;

        Map* map = builder.target() ;

        std::vector< SerializedAttribute<Map::Vertex>   > v_attributes ;
        std::vector< SerializedAttribute<Map::Halfedge> > h_attributes ;
        std::vector< SerializedAttribute<Map::Facet>    > f_attributes ;

        std::vector<Map::Facet*> facets ;
        
        bool surface_terminated = false ;
        
        LineInputStream in(input) ;

        builder.begin_surface() ;
        while(!in.eof()) {
            in.get_line() ;
            std::string keyword ;

            in >> keyword ;
            
            if(keyword == "v") {
                Point3d p ;
                in >> p ;
                builder.add_vertex(p) ;
            } else if(keyword == "vt") {
                Point2d q ;
                in >> q ;
                builder.add_tex_vertex(q) ;
            } else if(keyword == "f") {
                builder.begin_facet() ;
                while(!in.eol()) {
                    std::string s ;
                    in >> s ;
                    if(s.length() > 0) {
                        std::istringstream v_input(s) ;
                        int index ;
                        v_input >> index ;
                        builder.add_vertex_to_facet(index - 1) ;
                        char c ;
                        v_input >> c ;
                        if(c == '/') {
                            v_input >> index ;
                            builder.set_corner_tex_vertex(index - 1) ;
                        }
                    }
                }
                builder.end_facet() ;
                facets.push_back(builder.current_facet()) ;
            } else if(keyword == "#") {
                std::string second_keyword ;
                in >> second_keyword ;

                if(second_keyword == "attribute") {
                    if(!surface_terminated) {
		       // Quick and dirty fix to ensure that
		       // border edges exist before we put
		       // attributes on them !!!
                        builder.terminate_surface() ;
                        surface_terminated = true ;
                    }
                    std::string name ;
                    std::string localisation ;
                    std::string type ;
                    in >> name >> localisation >> type ;
                    std::cerr << "Attribute " << name << " on " << localisation << " : " << type << std::endl ;
                    if(localisation == "vertex") {
                        v_attributes.push_back(SerializedAttribute<Map::Vertex>()) ;
                        v_attributes.rbegin()->bind(map->vertex_attribute_manager(), name, type) ;
                    } else if(localisation == "halfedge") {
                        h_attributes.push_back(SerializedAttribute<Map::Halfedge>()) ;
                        h_attributes.rbegin()->bind(map->halfedge_attribute_manager(), name, type) ;
                    } else if(localisation == "facet") {
                        f_attributes.push_back(SerializedAttribute<Map::Facet>()) ;
                        f_attributes.rbegin()->bind(map->facet_attribute_manager(), name, type) ;
                    } else {
                        Logger::warn("MapSerializer_eobj") << "Invalid attribute localisation:" 
                                                           << localisation << std::endl ; 
                    }
                } else if(second_keyword == "attrs") {
                    
                    std::string localisation ;
                    in >> localisation ;
                    if(localisation == "v") {
                        int id ;
                        in >> id ;
                        id-- ;
                        Map::Vertex* v = builder.vertex(id) ;
                        serialize_read_attributes(in.line(), v, v_attributes) ;
                    } else if(localisation == "h") {
                        int id1, id2 ;
                        in >> id1 >> id2 ;
                        id1-- ; id2-- ;
                        Map::Vertex* v1 = builder.vertex(id1) ;
                        Map::Vertex* v2 = builder.vertex(id2) ;
                        Map::Halfedge* h = find_halfedge_between(v1,v2) ;
                        if(h == nil) {
                            Logger::warn("MapSerializer_eobj") << "Halfedge does not exist" << std::endl ;
                        } else {
                            serialize_read_attributes(in.line(), h, h_attributes) ;
                        }
                    } else if(localisation == "f") {
                        int id ;
                        in >> id ;
                        id-- ;
                        Map::Facet* f = facets[id] ;
                        serialize_read_attributes(in.line(), f, f_attributes) ;
                    } else {
                        Logger::warn("MapSerializer_eobj") << "Invalid attribute localisation:" 
                                                           << localisation << std::endl ; 
                    }
                } else if(second_keyword == "anchor") {
                    int index ;
                    in >> index ;
                    builder.lock_vertex(index - 1) ;
                } else if(second_keyword == "END") {
                    break ;
                }
            } 
        }
        
        builder.end_surface() ;
        
        return true ;
    }



    // Note: we cannot use MapSerializer_obj::serialize_write,
    // since it saves anchors using special keywords. In our
    // case, anchors are handled by the generic mechanism.

    bool MapSerializer_eobj::serialize_write(
        Map* map, std::ostream& output
    ) {
        Attribute<Vertex,int> vertex_id(
            map->vertex_attribute_manager()
        ) ;
        Attribute<TexVertex,int> tex_vertex_id(
            map->tex_vertex_attribute_manager()
        ) ;

        MapVertexLock is_locked(map) ;
        
        // Obj files numbering starts with 1 (instead of 0)
        enumerate_vertices(map, vertex_id, 1) ;

        // Output Vertices
        { FOR_EACH_VERTEX(Map,map,it) {
            output << "v " 
                   << it-> point().x() << " " 
                   << it-> point().y() << " " 
                   << it-> point().z() << std::endl ;
        }} 

        // Enumerate and output TexVertices
        { FOR_EACH_HALFEDGE(Map, map, it) {
            tex_vertex_id[it->tex_vertex()] = 0 ;
        }}
        
        int cur_id = 1 ;
         
        { FOR_EACH_FACET(Map, map, it) {
            Halfedge* jt = it->halfedge() ;
            do {
                if(tex_vertex_id[jt-> tex_vertex()] == 0) {
                    tex_vertex_id[jt->tex_vertex()] = cur_id ;
                    output << "vt " 
                           << jt-> tex_vertex()-> tex_coord().x()
                           << " "
                           << jt-> tex_vertex()-> tex_coord().y()
                           << std::endl ;
                    cur_id++ ;
                }
                jt = jt->next() ;
            } while(jt != it->halfedge()) ;
        }}  

        // Output facets

        { FOR_EACH_FACET(Map,map,it) {
            Halfedge* jt = it->halfedge() ;
            output << "f " ;
            do {
                output << vertex_id[jt-> vertex()]
                       << "/"
                       << tex_vertex_id[jt-> tex_vertex()]
                       << " " ;
                jt = jt->next() ;
            } while(jt != it->halfedge()) ;
            output << std::endl ;
        }}


        {
            std::vector<SerializedAttribute<Map::Vertex> > attributes ;
            if(get_serializable_attributes(map->vertex_attribute_manager(), attributes, output, "vertex")) {
                int vid = 1 ;
                FOR_EACH_VERTEX(Map, map, it) {
                    Map::Vertex* v = it ;
                    output << "# attrs v " << vid << " " ;
                    serialize_write_attributes(output, v, attributes) ;
                    output << std::endl ;
                    vid++ ;
                }
            }
        }

        {
            std::vector<SerializedAttribute<Map::Halfedge> > attributes ;
            if(get_serializable_attributes(map->halfedge_attribute_manager(), attributes, output, "halfedge")) {
                FOR_EACH_HALFEDGE(Map, map, it) {
                    Map::Halfedge* h = it ;
                    output << "# attrs h " 
                           << vertex_id[h->opposite()->vertex()] << " " << vertex_id[h->vertex()] << " " ;
                    serialize_write_attributes(output, h, attributes) ;
                    output << std::endl ;
                }
            }
        }

        {
            std::vector<SerializedAttribute<Map::Facet> > attributes ;
            if(get_serializable_attributes(map->facet_attribute_manager(), attributes, output, "facet")) {
                int fid = 1 ;
                FOR_EACH_FACET(Map, map, it) {
                    Map::Facet* f = it ;
                    output << "# attrs f " << fid << " " ;
                    serialize_write_attributes(output, f, attributes) ;
                    output << std::endl ;
                    fid++ ;
                }
            }
        }

        output << "# END" << std::endl ;

        return true ;
    }
    


//_________________________________________________________

}

