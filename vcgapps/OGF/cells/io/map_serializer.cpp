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
 

#include <OGF/cells/io/map_serializer.h>

namespace OGF {

//_________________________________________________________

    MapSerializer::MapSerializer() {
        read_supported_  = false ;
        write_supported_ = false ;
    }

    MapSerializer::~MapSerializer() {
    }

    bool MapSerializer::serialize_read(
        std::istream& in, AbstractMapBuilder& out
    ) {
        bool implemented = false ;
        ogf_assert(implemented) ;
        return nil ;
    }
    
    bool MapSerializer::serialize_write(
        Map* map, std::ostream& out
    ) {
        bool implemented = false ;
        ogf_assert(implemented) ;
        return false ;
    }

    bool MapSerializer::serialize_read(const std::string& file_name, AbstractMapBuilder& out) {
        std::fstream::openmode mode = binary() ?
            (std::fstream::in | std::fstream::binary) :
            std::fstream::in ;
        
        std::ifstream input(file_name.c_str(),mode) ;
        if(!input) {
            Logger::err("MapSerializer") 
                << "could not open file\'" 
                << file_name << "\'" << std::endl ;
            return false ;
        }
        return serialize_read(input, out) ;
    }

    bool MapSerializer::serialize_write(
        const std::string& file_name, Map* map
    ) {
        std::fstream::openmode mode = binary() ?
            (std::fstream::out | std::fstream::trunc | std::fstream::binary) :
            (std::fstream::out | std::fstream::trunc) ;

        std::ofstream output(file_name.c_str(), mode) ;

        if(!output) {
            Logger::err("MapSerializer") 
                << "could not open file\'" 
                << file_name << "\'" << std::endl ;
            return false ;
        }

        if(!binary()) {
            output.precision(16) ;
        }
        
        return serialize_write(map, output) ;
    }



    bool MapSerializer::binary() const {
        return false ;
    }

    bool MapSerializer::streams_supported() const {
        return true ;
    }
    
    bool MapSerializer::read_supported() const {
        return read_supported_ ;
    }
    
    bool MapSerializer::write_supported() const {
        return write_supported_ ;
    }

//_________________________________________________________

}

