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
 
 
 

#ifndef __OGF_BASIC_DEBUG_TRACED__
#define __OGF_BASIC_DEBUG_TRACED__

#include <OGF/basic/common/common.h>
#include <OGF/basic/types/counted.h>

//____________________________________________________________________________

namespace OGF {

/**
 *
 * This class can be used to debug containers.
 * Each instance is provided with a unique id,
 * and its constructor, copy contructor, destructor 
 * and operator = issue a message on the standard
 * output.
 *
 */

    class BASIC_API Traced : public virtual Counted {
        
    public:
        Traced() ;
        Traced(const Traced& rhs) ;
        virtual ~Traced() ;
        
        Traced& operator=(const Traced& rhs) ;
        
    private:
        int id_ ;
        static int last_id_ ;
    } ;

}

#endif
