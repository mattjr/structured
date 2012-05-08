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

#ifndef __OGF_IMAGE_ALGOS_MORPHO_MATH__
#define __OGF_IMAGE_ALGOS_MORPHO_MATH__

#include <OGF/image/common/common.h>
#include <OGF/image/types/image.h>
#include <OGF/math/geometry/types.h>

namespace OGF {
    
    class IMAGE_API FPStructuringElement {
    public:

        FPStructuringElement(Image* source) ;
        int radius() const { return radius_ ; }

        void add_neighbor(int xrel, int yrel) {
            ogf_assert(nb_offsets_ < 99) ;
            offset_[nb_offsets_] = ( (xrel + width_ * yrel) * floats_per_pixel_ ) ;
            nb_offsets_++ ;
            radius_ = ogf_max(radius_, ogf_abs(xrel)) ;
            radius_ = ogf_max(radius_, ogf_abs(yrel)) ;
        }

        void convolve(float* from, float* to) const ;
        bool is_zero(const float* from) const {
            for(int i=0; i<floats_per_pixel_; i++) {
                if(from[i] != 0.0) { return false ; }
            }
            return true ;
        }

        inline void convolve(int x, int y, Image* target_img) const {
            int pixel_base = ((x + width_ * y) * floats_per_pixel_) ;
            convolve(base_mem_ + pixel_base, (float*)(target_img->base_mem()) + pixel_base) ;
        }

    private:
        float* base_mem_ ;
        int floats_per_pixel_ ;
        int width_ ;
        Image* source_ ;
        int radius_ ;
        int offset_[100] ;
        int nb_offsets_ ;
    } ;


    class IMAGE_API FPMorphoMath {
    public:
        FPMorphoMath(Image* target) ;
        ~FPMorphoMath() ;
        void dilate(const FPStructuringElement& elt, int nb_iterations = 1) ;

        /** uses a default structuring element */
        void dilate(int nb_iterations = 1) ;
        
    private:
        Image* target_ ;
        float* graph_mem_ ;
        int width_ ;
        int height_ ;
        int floats_per_line_ ;
        int floats_per_pixel_ ;
    } ;
}

#endif
