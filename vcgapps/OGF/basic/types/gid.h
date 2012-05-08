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
 
 
 
 

#ifndef __OGF_BASIC_TYPES_GID__
#define __OGF_BASIC_TYPES_GID__

#include <OGF/basic/common/common.h>
#include <OGF/basic/types/types.h>

namespace OGF {

//_________________________________________________________

/**
 * Represents an opaque id meant to referencing an object
 * within a container. A Gid should be considered to be an 
 * opaque identifier by client code.
 */

    // full inline functions class, don't have to export
    class Gid {
    
    public:
    
        inline Gid() ;
        inline Gid(const void* ptr) ;   
        inline Gid(Numeric::int64 i) ;
        inline Gid(const Gid& other) ;
        inline Gid& operator=(const Gid& other) ;
        inline ~Gid() ;
    
        inline bool operator==(const Gid& other) const ;
        inline bool operator!=(const Gid& other) const ;
        inline bool is_nil() const ;
    
        inline void* data_pointer() const ;
        inline Numeric::int64 data_int() const ;
    
    protected:
        union {
            void* pointer_ ;
            Numeric::int64 integer_ ;
        } data_ ;
    private:
    } ;

//____________________________________________________________________________

    inline Gid::Gid()  {
        data_.pointer_ = nil ;
    }

    inline Gid::Gid(const void* ptr)  {
        data_.pointer_ = (void*)ptr ;
    }

    inline Gid::Gid(Numeric::int64 i)  {
        data_.integer_ = i ;
    }

    inline Gid::Gid(const Gid& other)  {
        data_.pointer_ = other.data_.pointer_ ;
    }

    inline Gid& Gid::operator=(const Gid& other)  {
        data_.pointer_ = other.data_.pointer_ ;
        return *this ;
    }

    inline Gid::~Gid()  {
    }

    inline bool Gid::operator==(const Gid& other) const {
        return data_.pointer_ == other.data_.pointer_ ;
    }

    inline bool Gid::operator!=(const Gid& other) const {
        return data_.pointer_ != other.data_.pointer_ ;
    }

    inline bool Gid::is_nil() const  {
        return data_.pointer_ == nil ;
    }

    inline void* Gid::data_pointer() const {
        return data_.pointer_ ;
    }

    inline Numeric::int64 Gid::data_int() const {
        return data_.integer_ ;
    }

//_________________________________________________________

}
#endif

