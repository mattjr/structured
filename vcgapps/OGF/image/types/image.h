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
 
 
 
 

#ifndef __OGF_IMAGE_TYPES_IMAGE__
#define __OGF_IMAGE_TYPES_IMAGE__

#include <OGF/image/common/common.h>
#include <OGF/image/types/types.h>
#include <OGF/image/types/colormap.h>
#include <OGF/image/types/image_store.h>

namespace OGF {

//_________________________________________________________

// TODO: add subimage capabilities and optimized access.

    class IMAGE_API Image : public Counted {
    public:
        enum ColorEncoding {
            GRAY, INDEXED, RGB, BGR, RGBA,
            INT16, INT32, FLOAT32, FLOAT64,
            RGB_FLOAT32, RGBA_FLOAT32,
            YUV
        } ;

        Image() ;
        Image(ColorEncoding color_rep, int size_x) ;
        Image(ColorEncoding color_rep, int size_x, int size_y) ;
        Image(
            ColorEncoding color_rep, int size_x, int size_y, int size_z
        ) ;
        Image(const Image* rhs) ;

        virtual void initialize(ColorEncoding color_rep, int size_x) ;
        virtual void initialize(
            ColorEncoding color_rep, int size_x, int size_y
        ) ;
        virtual void initialize(
            ColorEncoding color_rep, int size_x, int size_y, int size_z
        ) ;

        virtual ~Image() ;

        virtual void acquire() ;

        int dimension() const { return dimension_ ; }
        int size(int axis) const { 
            ogf_assert(axis >= 0 && axis < 3) ;
            return size_[axis] ;
        }
        int width() const  { return size_[0] ; }
        int height() const { return size_[1] ; }
        int depth() const  { return size_[2] ; }
        int bytes_per_pixel() const { return bytes_per_pixel_ ; }
        int nb_pixels() const { return size_[0] * size_[1] * size_[2] ; }
        int bytes() const { return nb_pixels() * bytes_per_pixel_ ; }
        ColorEncoding color_encoding() const { return color_encoding_ ; }
        void set_color_encoding(ColorEncoding x) { color_encoding_ = x ; }
    
        Memory::pointer base_mem() const {
            return base_mem_ ;
        }

        Memory::byte* base_mem_byte_ptr() const {
            return byte_ptr(base_mem_) ;
        }

        Numeric::int16* base_mem_int16_ptr() const {
            return int16_ptr(base_mem_) ;
        }

        Numeric::int32* base_mem_int32_ptr() const {
            return int32_ptr(base_mem_) ;
        }

        Numeric::float32* base_mem_float32_ptr() const {
            return float32_ptr(base_mem_) ;
        }

        Numeric::float64* base_mem_float64_ptr() const {
            return float64_ptr(base_mem_) ;
        }

        static int bytes_per_pixel_from_color_encoding(
            ColorEncoding rep
        ) ;

        const Colormap* colormap() const { return colormap_ ; }
        Colormap* colormap()             { return colormap_ ; }
        void set_colormap(Colormap* colormap) {
            colormap_ = colormap ; 
        }

        const ImageStore* store() const { return store_ ; }
        ImageStore* store()             { return store_ ; }

        Memory::pointer pixel_base(int x) {
            return base_mem() + x * factor_[0] ;
        }
        
        Memory::pointer pixel_base(int x, int y) {
            return base_mem() + x * factor_[0] + y * factor_[1] ;
        }
        
        Memory::pointer pixel_base(int x, int y, int z) {
            return base_mem() + 
                x * factor_[0] + y * factor_[1] + z * factor_[2] ;
        }

        inline Memory::byte* byte_ptr(Memory::pointer ptr) const {
            ogf_assert(
                color_encoding_ == GRAY ||
                color_encoding_ == RGB ||
                color_encoding_ == BGR ||
                color_encoding_ == RGBA ||
                color_encoding_ == YUV
            ) ;
            return ptr ;
        }

        inline Numeric::int16* int16_ptr(Memory::pointer ptr) const {
            ogf_assert(color_encoding_ == INT16) ;
            return (Numeric::int16*)(ptr) ;
        }

        inline Numeric::int32* int32_ptr(Memory::pointer ptr) const {
            ogf_assert(color_encoding_ == INT32) ;
            return (Numeric::int32*)(ptr) ;
        }

        inline Numeric::float32* float32_ptr(Memory::pointer ptr) const {
            ogf_assert(
                color_encoding_ == FLOAT32 ||
                color_encoding_ == RGB_FLOAT32 ||
                color_encoding_ == RGBA_FLOAT32
            ) ;
            return (Numeric::float32*)(ptr) ;
        }

        inline Numeric::float64* float64_ptr(Memory::pointer ptr) const {
            ogf_assert(color_encoding_ == FLOAT64) ;
            return (Numeric::float64*)(ptr) ;
        }

    protected:
        ColorEncoding color_encoding_ ;
        Colormap_var colormap_ ;
        ImageStore_var store_ ;
        int factor_[3] ;
        Memory::pointer base_mem_ ;
        int dimension_ ;
        int size_[3] ;
        int bytes_per_pixel_ ;

    private:
        Image(const Image& rhs) ;
        Image& operator=(const Image& rhs) ;
    } ;

    typedef SmartPointer<Image> Image_var ;

//_________________________________________________________

}
#endif

