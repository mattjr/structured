/*
 *  GXML/Graphite: Geometry and Graphics Programming Library + Utilities
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
 
#ifndef __OGF_BASIC_IO_STDIO_COMPAT__
#define __OGF_BASIC_IO_STDIO_COMPAT__

#include <OGF/basic/common/common.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/debug/assert.h>

#include <iostream>

#include <stdio.h>

// These functions facilitate the porting of old C code,
// by providing an stdio-like API acting on C++ streams.

namespace OGF {

//____________________________________________________________________________
    
    inline size_t fread(
        void* ptr_in, size_t size, size_t nbelt, std::istream* in
    ) {
        char* ptr = (char *)ptr_in ;
        size_t j ;
        for(j=0; j<nbelt; j++) {
            for(size_t i=0; i<size; i++) {
                if(in->eof()) {
                    return j ;
                }
                in->get(*ptr) ;
                ptr++ ;
            }
        }
        return j ;
    } 

    inline int fgetc(std::istream* in) {
        char result ;
        in->get(result) ;
        return int(result) ;
    }


    inline int fseek(
        std::istream* in, long offset, int whence
    ) {
        if(whence == SEEK_CUR) {
            offset += in->tellg() ;
        }
        ogf_assert(whence != SEEK_END) ;
        in->seekg(offset) ;
        return 0 ;
    }

    inline size_t ftell(std::istream* in) {
        return in->tellg() ;
    }

    inline void fclose(std::istream* in) { 
    }


//____________________________________________________________________________

}

#endif
