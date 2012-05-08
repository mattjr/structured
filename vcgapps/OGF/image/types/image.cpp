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
 
 
 
 

#include <OGF/image/types/image.h>

namespace OGF {

//_________________________________________________________


    Image::Image() {
        bytes_per_pixel_ = 0 ;
        dimension_ = 0 ;
        size_[0] = 0 ;
        size_[1] = 0 ;
        size_[2] = 0 ;
        factor_[0] = 0 ;
        factor_[1] = 0 ;
        factor_[2] = 0 ;
        base_mem_ = nil ;
    }

    Image::Image(ColorEncoding color_rep, int size_x) {
        initialize(color_rep, size_x) ;
    }
    
    Image::Image(ColorEncoding color_rep, int size_x, int size_y) {
        initialize(color_rep, size_x, size_y) ;
    }
    
    Image::Image(
        ColorEncoding color_rep, int size_x, int size_y, int size_z
    ) {
        initialize(color_rep, size_x, size_y, size_z) ;
    }

    Image::Image(const Image* rhs) {
        initialize(
            rhs->color_encoding(), rhs->width(), rhs->height(), rhs->depth()
        ) ;
        ::memcpy(
            base_mem(), rhs->base_mem(),
            bytes_per_pixel() * width() * height() * depth() 
        ) ;
    }

    void Image::initialize(ColorEncoding color_rep, int size_x) {
        color_encoding_ = color_rep ;
        bytes_per_pixel_ = bytes_per_pixel_from_color_encoding(color_rep) ;
        dimension_ = 1 ;
        size_[0] = size_x ;
        size_[1] = 1 ;
        size_[2] = 1 ;
        factor_[0] = bytes_per_pixel_ ;
        factor_[1] = 0 ;
        factor_[2] = 0 ;
        store_ = new ImageStore(bytes_per_pixel_, size_x) ;
        base_mem_ = store_-> base_mem() ;
    }

    void Image::initialize(ColorEncoding color_rep, int size_x, int size_y) {
        color_encoding_ = color_rep ;
        bytes_per_pixel_ = bytes_per_pixel_from_color_encoding(color_rep) ;
        size_[0] = size_x ;
        size_[1] = size_y ;
        size_[2] = 1 ;
        dimension_ = 2 ;
        if(size_y == 1) {
            dimension_ = 1 ;
        }
        factor_[0] = bytes_per_pixel_ ;
        factor_[1] = factor_[0] * size_x ;
        factor_[2] = 0 ;
        store_ = new ImageStore(bytes_per_pixel_, size_x, size_y) ;
        base_mem_ = store_-> base_mem() ;
    }

    void Image::initialize(
        ColorEncoding color_rep, int size_x, int size_y, int size_z
    ) {
        color_encoding_ = color_rep ;
        bytes_per_pixel_ = bytes_per_pixel_from_color_encoding(color_rep) ;
        size_[0] = size_x ;
        size_[1] = size_y ;
        size_[2] = size_z ;
        dimension_ = 3 ;
        if(size_z == 1) {
            dimension_ = 2 ;
            if(size_y == 1) {
                dimension_ = 1 ;
            }
        }
        factor_[0] = bytes_per_pixel_ ;
        factor_[1] = factor_[0] * size_x ;
        factor_[2] = factor_[1] * size_y ;
        store_ = new ImageStore(bytes_per_pixel_, size_x, size_y, size_z) ;
        base_mem_ = store_-> base_mem() ;
    }

    void Image::acquire() {
    }

    Image::~Image() {
    }

    int Image::bytes_per_pixel_from_color_encoding(
        ColorEncoding rep
    ) {
        int result = 0 ;
        switch(rep) {
        case GRAY:
            result = 1 ;
            break ;
        case INDEXED: 
            result = 1 ;
            break ;
        case RGB:
        case BGR:
        case YUV:
            result = 3 ;
            break ;
        case RGBA: 
            result = 4 ;
            break ;
        case INT16:
            result = 2 ;
            break ;
        case INT32:
            result = 4 ;
            break ;
        case FLOAT32:
            result = 4 ;
            break ;
        case FLOAT64:
            result = 8 ;
            break ;
        case RGB_FLOAT32:
            result = 12 ;
            break ;
        case RGBA_FLOAT32:
            result = 16 ;
            break ;

        }
        return result ;
    }



//_________________________________________________________

}

