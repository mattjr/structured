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

#include <OGF/image/algos/morpho_math.h>
#include <OGF/image/types/image.h>
#include <vector>
#include <string.h>

namespace OGF {

    static inline bool has_value(Memory::byte* p, int bytes_per_pixel_) {
        switch(bytes_per_pixel_) {
        case 1:
            return *p != 0 ;
        case 3:
            return p[0] != 0 || p[1] != 0 || p[2] != 0 ;
        case 4:
            return p[3] != 0 ;
        default:
            ogf_assert_not_reached ;
        }
        return false ;
    }

    void StructuringElement::convolve(
        Memory::byte* from, Memory::byte* to
    ) const {
        if(has_value(from, bytes_per_pixel_)) {
            for(int i=0; i<bytes_per_pixel_; i++) {
                to[i] = from[i] ;
            }
        } else {
            int rgb[4] ;
            int nb_neigh = 0 ;
            rgb[0] = 0 ;
            rgb[1] = 0 ;
            rgb[2] = 0 ;
            rgb[3] = 0 ;

            for(int i=0; i<nb_offsets_; i++) {
                for(int j=0; j<bytes_per_pixel_; j++) {
                    rgb[j] += (from + offset_[i])[j] ;
                }
                if(has_value(from + offset_[i], bytes_per_pixel_)) {
                    nb_neigh++ ;
                }
            }

            if(nb_neigh == 0) {
                nb_neigh = 1 ;
            }
            
            for(int j=0; j<bytes_per_pixel_; j++) {
                to[j] = Memory::byte(rgb[j] / nb_neigh) ;
            }

        }
    }

    
    MorphoMath::MorphoMath(Image* target) {
        target_ = target ;
        ogf_assert(
            target_->color_encoding() == Image::RGB ||
            target_->color_encoding() == Image::RGBA ||
            target_->color_encoding() == Image::GRAY ||
            target_->color_encoding() == Image::INDEXED
        ) ;
        width_ = target_->width() ;
        height_ = target_->height() ;
        bytes_per_pixel_ = target_->bytes_per_pixel() ;
        bytes_per_line_ = bytes_per_pixel_ * width_ ;
        graph_mem_ = target_->base_mem() ;
    }

    MorphoMath::~MorphoMath() {
        target_ = nil ;
    }


    void MorphoMath::dilate(int nb_iterations) {
        StructuringElement str(target_) ;
        str.add_neighbor(-1,-1) ;
        str.add_neighbor(-1, 0) ;
        str.add_neighbor(-1, 1) ;
        str.add_neighbor( 0,-1) ;
        str.add_neighbor( 0, 0) ;
        str.add_neighbor( 0, 1) ;
        str.add_neighbor( 1,-1); 
        str.add_neighbor( 1, 0); 
        str.add_neighbor( 1, 1) ;
        dilate(str, nb_iterations) ;
    }

    void MorphoMath::dilate(const StructuringElement& str, int nb_iterations) {
        Image_var tmp = new Image(target_) ;

        int R = str.radius() ;

        int line_offset = (R * bytes_per_pixel_) ;

        for(int iter=0; iter<nb_iterations; iter++) {
            Memory::byte* from_line = target_->base_mem() + R * bytes_per_line_ ;
            Memory::byte* to_line   = tmp->base_mem()     + R * bytes_per_line_ ;
            for(int y=R; y<height_ - R; y++) {
                Memory::byte* from = from_line + line_offset ;
                Memory::byte* to   = to_line   + line_offset ;
                for(int x=R; x<width_ - R; x++) {
                    from += bytes_per_pixel_ ;
                    to   += bytes_per_pixel_ ;
                    str.convolve(from, to) ;
                }
                from_line += bytes_per_line_ ;
                to_line   += bytes_per_line_ ;
            }
            Memory::copy(
                target_->base_mem(), tmp->base_mem(), 
                target_->width() * target_->height() * 
                target_->bytes_per_pixel()
            ) ;
            std::cerr << "." << std::flush ;
        }
        std::cerr << std::endl ;
    }
}

