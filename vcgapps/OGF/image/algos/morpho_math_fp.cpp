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

#include <OGF/image/algos/morpho_math_fp.h>
#include <OGF/image/types/image.h>
#include <vector>
#include <string.h>

namespace OGF {

    FPStructuringElement::FPStructuringElement(Image* source) : source_(source) {
        base_mem_ = (float*)(source_->base_mem()) ;
        width_ = source_->width() ;
        radius_ = 0 ;
        nb_offsets_ = 0 ;
        switch(source->color_encoding()) {
        case Image::FLOAT32:
            floats_per_pixel_ = 1 ;
            break ;
        case Image::RGB_FLOAT32:
            floats_per_pixel_ = 3 ;
            break ;
        case Image::RGBA_FLOAT32:
            floats_per_pixel_ = 4 ;
            break ;
        default:
            ogf_assert_not_reached ;
        }
    }

    void FPStructuringElement::convolve(
        float* from, float* to
    ) const {
        if(!is_zero(from)) {
            for(int i=0; i<floats_per_pixel_; i++) {
                to[i] = from[i] ;
            }
        } else {
            int nb_neigh = 0 ;
            {for(int j=0; j<floats_per_pixel_; j++) {
                to[j] = 0.0 ;
            }}
            for(int i=0; i<nb_offsets_; i++) {
                if(!is_zero(from + offset_[i])) {
                    nb_neigh++ ;
                    for(int j=0; j<floats_per_pixel_; j++) {
                        to[j] += from[offset_[i] + j] ;
                    }
                }
            }
            if(nb_neigh == 0) {
                nb_neigh = 1 ;
            }
            {for(int j=0; j<floats_per_pixel_; j++) {
                to[j] /= float(nb_neigh) ;
            }}
        }
    }
    
    FPMorphoMath::FPMorphoMath(Image* target) {
        target_ = target ;
        ogf_assert(
            target_->color_encoding() == Image::RGB_FLOAT32 ||
            target_->color_encoding() == Image::RGBA_FLOAT32 ||
            target_->color_encoding() == Image::FLOAT32 
        ) ;
        width_ = target_->width() ;
        height_ = target_->height() ;
        floats_per_pixel_ = target_->bytes_per_pixel() / 4 ;
        floats_per_line_ = width_ * floats_per_pixel_ ;
        graph_mem_ = (float*)(target_->base_mem()) ;
    }

    FPMorphoMath::~FPMorphoMath() {
        target_ = nil ;
    }


    void FPMorphoMath::dilate(int nb_iterations) {
        FPStructuringElement str(target_) ;
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

    void FPMorphoMath::dilate(const FPStructuringElement& str, int nb_iterations) {
        Image_var tmp = new Image(target_) ;

        int R = str.radius() ;
        int line_offset = (R * floats_per_line_) ;

        for(int iter=0; iter<nb_iterations; iter++) {
            float* from_line = (float*)(target_->base_mem()) + R * floats_per_line_;
            float* to_line   = (float*)(tmp->base_mem())     + R * floats_per_line_;
            for(int y=R; y<height_ - R; y++) {
                float* from = from_line + line_offset ;
                float* to   = to_line   + line_offset ;
                for(int x=R; x<width_ - R; x++) {
                    from +=floats_per_pixel_ ;
                    to   +=floats_per_pixel_ ;
                    str.convolve(from, to) ;
                }
                from_line += floats_per_line_ ;
                to_line   += floats_per_line_ ;
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

