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
 
 
 
 

#include <OGF/image/types/image_store.h>
#include <string.h>
 
namespace OGF {

//_________________________________________________________

    ImageStore::ImageStore(int bytes_per_pixel, int dim_x) {
        bytes_per_pixel_ = bytes_per_pixel ;
        size_[0] = dim_x ;
        size_[1] = 1 ;
        size_[2] = 1 ;
        dimension_ = 1 ;
        base_mem_ = new Memory::byte[bytes()] ;
        Memory::clear(base_mem_, bytes()) ;
    }

    ImageStore::ImageStore(int bytes_per_pixel, int dim_x, int dim_y) {
        bytes_per_pixel_ = bytes_per_pixel ;
        size_[0] = dim_x ;
        size_[1] = dim_y ;
        size_[2] = 1 ;
        dimension_ = 2 ;
        base_mem_ = new Memory::byte[bytes()] ;
        Memory::clear(base_mem_, bytes()) ;
    }

    ImageStore::ImageStore(
        int bytes_per_pixel, int dim_x, int dim_y, int dim_z
    ) {
        bytes_per_pixel_ = bytes_per_pixel ;
        size_[0] = dim_x ;
        size_[1] = dim_y ;
        size_[2] = dim_z ;
        dimension_ = 3 ;
        base_mem_ = new Memory::byte[bytes()] ;
        Memory::clear(base_mem_, bytes()) ;
    }

    ImageStore::~ImageStore() {
        delete[] base_mem_ ;
    }




//_________________________________________________________

}

