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
 

#include <OGF/basic/debug/traced.h>
#include <iostream>

namespace OGF {

    int Traced::last_id_ = 0 ;
    
    Traced::Traced() {
        last_id_++ ;
        id_ = last_id_ ;
        std::cout << "constr Traced # " << id_ << std::endl ;
    }

    Traced::Traced(const Traced& rhs) {
        last_id_++ ;
        id_ = last_id_ ;
        std::cout << "copy constr Traced # " << id_ 
                  << " from Traced # " << rhs.id_ << std::endl ;
    }

    Traced::~Traced() {
        std::cout << "destr  Traced # " << id_ << std::endl ;    
    }

    Traced& Traced::operator=(const Traced& rhs) {
        std::cout << "operator= Traced # " << id_ 
                  << " from Traced # " << rhs.id_ << std::endl ;
        return *this ;
    }

}
