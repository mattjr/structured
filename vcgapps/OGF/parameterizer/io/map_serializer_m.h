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
 

#ifndef ___MAP_SERIALIZER_M__
#define ___MAP_SERIALIZER_M__

#include <OGF/parameterizer/common/common.h>
#include <OGF/cells/io/map_serializer.h>

#include <string.h>
#include <stdio.h>

namespace OGF {

//_________________________________________________________

    class PARAMETERIZER_API MapSerializer_m : public MapSerializer, public MapMutator {
    public:

        MapSerializer_m() ;
        
        virtual bool serialize_read(
            std::istream& in, AbstractMapBuilder& out
        ) ;

        virtual bool serialize_write(
            Map* map, std::ostream& output
        ) ; // Not implemented yet

    } ;


//_________________________________________________________

}
#endif

