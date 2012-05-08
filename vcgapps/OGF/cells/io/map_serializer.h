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
 

#ifndef ___MAP_SERIALIZER__
#define ___MAP_SERIALIZER__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map.h>
#include <OGF/basic/types/counted.h>
#include <OGF/basic/types/smart_pointer.h>

#include <iostream>

namespace OGF {

    class AbstractMapBuilder ;

//_________________________________________________________

    class CELLS_API MapSerializer : public Counted {
    public:
        typedef Map::Vertex Vertex ;
        typedef Map::Halfedge Halfedge ;
        typedef Map::Facet Facet ;
        typedef Map::TexVertex TexVertex ;

        typedef Map::Vertex_iterator Vertex_iterator ;
        typedef Map::Halfedge_iterator Halfedge_iterator ;
        typedef Map::Facet_iterator Facet_iterator ;

        MapSerializer() ;
        virtual ~MapSerializer() ;

        virtual bool serialize_read(
            const std::string& file_name, AbstractMapBuilder& out
        ) ;
        virtual bool serialize_write(
            const std::string& file_name, Map* map
        ) ;

        virtual bool serialize_read(
            std::istream& in, AbstractMapBuilder& out
        ) ;
        
        virtual bool serialize_write(
            Map* map, std::ostream& out
        ) ;

        /**
         * checks whether the stream should be opened
         * in text or binary mode. Default returns false.
         */
        virtual bool binary() const ;

        /**
         * checks whether reading and writing to streams is
         * supported.
         */
        virtual bool streams_supported() const ;

        /**
         * checks whether reading is implemented.
         */
        virtual bool read_supported() const ;

        /**
         * checks whether writing is implemented.
         */
        virtual bool write_supported() const ;

        

    protected:
        bool read_supported_ ;
        bool write_supported_ ;
    } ;

    typedef SmartPointer<MapSerializer> MapSerializer_var ;

//_________________________________________________________



}
#endif

