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
 

#include <OGF/image/io/image_serializer_ppm.h>
#include <OGF/image/types/image.h>
#include <OGF/basic/debug/logger.h>

#include <sstream>

namespace OGF {

//_________________________________________________________

    Image* ImageSerializer_ppm::serialize_read(std::istream& in,bool flip) {
        // Read header
        char buff[256] ;

        std::string magic ;
        in >> magic ;
        if(magic != "P6") {
            Logger::err("PPM loader: cannot load this type of PPM") ;
            return nil ;
        }
        // read end of line
        in.getline(buff, 256) ;

        // read comments (# CREATOR etc...)
        do {
            in.getline(buff, 256) ;
        } while(buff[0] == '#') ;

        int width, height, max_comp_value ;
        std::istringstream line_in(buff) ;
        line_in >> width >> height ;
        in >> max_comp_value ;
        if(width < 1 || height < 1) {
            Logger::err("PPM loader") << "invalid image size: " 
                                      << width << " x " 
                                      << height << std::endl ;
            return nil ;
        }
        if(max_comp_value != 255) {
            Logger::err(
                "PPM loader: invalid max component value (should be 255)"
            ) ;
            return nil ;
        }
        in.getline(buff, 256) ;

        int image_size = width * height ;
        Image* result = new Image(Image::RGB, width, height) ;
        for(int i=0; i<image_size; i++) {
            char r,g,b ;
            in.get(r) ;
            in.get(g) ;
            in.get(b) ;


            result->base_mem()[3*i]   = r ;
            result->base_mem()[3*i+1] = g ;
            result->base_mem()[3*i+2] = b ;
        }
        if(flip)
            flip_image(*result) ;
        return result ;
    }

    bool ImageSerializer_ppm::read_supported() const {
        return true ;
    }

    bool ImageSerializer_ppm::serialize_write(
        std::ostream& out, const Image* image
    ) {

        if(
            image-> color_encoding() != Image::RGB &&
            image-> color_encoding() != Image::RGBA
        ) {
            Logger::err("Image") 
                << "PPM writer implemented for RGB or RGBA images only"
                << std::endl ;
            return false ;
        }

        out << "P6 " << std::endl ;
        out << "# CREATOR: Graphite" << std::endl ;
        out << image->width() << " " << image->height() << std::endl ;
        out << 255 << std::endl ;
        
        int image_size = image->width()*image->height() ;
        
        switch(image-> color_encoding()) {
        case Image::RGB: {
            for(int i=0; i<image_size; i++) {
                out.put(image->base_mem()[3*i]) ;
                out.put(image->base_mem()[3*i+1]) ;
                out.put(image->base_mem()[3*i+2]) ;
            }
        }
            break ;
        case Image::RGBA: {
            for(int i=0; i<image_size; i++) {
                out.put(image->base_mem()[4*i]) ;
                out.put(image->base_mem()[4*i+1]) ;
                out.put(image->base_mem()[4*i+2]) ;
            }
        }
            break ;
        default: {
            ogf_assert(false) ;
        }
            break ;
        }
        return true ;
    }


    bool ImageSerializer_ppm::write_supported() const {
        return true ;
    }

//_________________________________________________________

}

