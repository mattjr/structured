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

#include <OGF/image/io/image_serializer_tga.h>
#include <OGF/image/io/tga.h>
#include <OGF/image/types/image.h>
#include <stdio.h>

namespace OGF {

    Image* ImageSerializer_tga::serialize_read(const std::string& file_name) {
//        gliVerbose(1) ;
        FILE* in = ::fopen(file_name.c_str(), "rb") ;
        gliGenericImage* image = gliReadTGA(in, "tga_image") ;
        ::fclose(in) ;

        int width  = image->width ;
        int height = image->height ;
        Image::ColorEncoding encoding = Image::RGB ;
        switch(image->format) {
        case GL_COLOR_INDEX:
            encoding = Image::INDEXED;
            break ;
        case GL_LUMINANCE:
            encoding = Image::GRAY;
            break ;
        case GL_BGRA_EXT:
            encoding = Image::RGBA;
            break ;
        case GL_BGR_EXT:
            encoding = Image::RGB;
            break ;
        }
        
        Colormap* colormap = nil ;
        if(image->format == GL_COLOR_INDEX) {
            colormap = new Colormap(image->cmapEntries) ;
            switch(image->cmapFormat) {
            case GL_BGR_EXT: {
                for(int i=0; i<image->cmapEntries; i++) {
                    Memory::byte r = image->cmap[i*3+2] ;
                    Memory::byte g = image->cmap[i*3+1] ;
                    Memory::byte b = image->cmap[i*3  ] ;
                    colormap->color_cell(i) = Colormap::ColorCell(r,g,b,255) ;
                }
            } break ;
            case GL_BGRA_EXT: {
                for(int i=0; i<image->cmapEntries; i++) {
                    Memory::byte r = image->cmap[i*4+2] ;
                    Memory::byte g = image->cmap[i*4+1] ;
                    Memory::byte b = image->cmap[i*4  ] ;
                    Memory::byte a = image->cmap[i*4+3] ;
                    colormap->color_cell(i) = Colormap::ColorCell(r,g,b,a) ;
                }
            } break ;
            }
        }

        Image* result = new Image(encoding, width, height) ;
        result->set_colormap(colormap) ;
        
        memcpy(
            result->base_mem(), image->pixels, 
            result->width() * result->height() * result->bytes_per_pixel() 
        ) ;

        // Flip red and blue
        if(image->format == GL_BGRA_EXT || image->format == GL_BGR_EXT) {
            rgb_to_bgr(*result);
        }

        gliFreeImage(image) ;

        return result ;
    }
    

    bool ImageSerializer_tga::read_supported() const {
        return true ;
    }

    bool ImageSerializer_tga::streams_supported() const {
        return false ;
    }


}
