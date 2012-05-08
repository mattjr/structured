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
 
 
 
 

#ifndef __OGF_IMAGE_TYPES_IMAGE_STORE__
#define __OGF_IMAGE_TYPES_IMAGE_STORE__

#include <OGF/image/common/common.h>
#include <OGF/image/types/types.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/types/counted.h>

namespace OGF {


//_________________________________________________________

/**
 * Internal storage associated with a Image. A ImageStore
 * is a multi-dimensional array of bytes. Note that several
 * Images can share the same ImageStore.
 */

    class IMAGE_API ImageStore : public Counted {
    public:
        ImageStore(int bytes_per_pixel, int dim_x) ;
        ImageStore(int bytes_per_pixel, int dim_x, int dim_y) ;
        ImageStore(int bytes_per_pixel, int dim_x, int dim_y, int dim_z) ;
        virtual ~ImageStore() ;

        /**
         * Returns the dimension of this image. An image can be 1D, 2D or 3D. 
         */
        int dimension() const ;

        /**
         * @return the size of the image along the specified axis, or
         *   0 if axis is greater than the dimension of this image.
         * @param axis can be one of 0,1,2 for X,Y,Z respectively.
         */
        int size(int axis) const ;

        /** 
         * number of bytes allocated by this ImageStore
         */
        int bytes() const ;

        /** equivalent to size(0) */
        int width() const ;

        /** equivalent to size(1). Returns 1 for a 1D image. */
        int height() const ;

        /** equivalent to size(2). Returns 1 for a 1D or a 2D image. */
        int depth() const ;

        int bytes_per_pixel() const ;

        Memory::pointer base_mem() const ;

    private:
        int bytes_per_pixel_ ;
        int dimension_ ;
        int size_[3] ;
        Memory::byte* base_mem_ ;
    } ;

    typedef SmartPointer<ImageStore> ImageStore_var ;

//_________________________________________________________


    inline int ImageStore::dimension() const {
        return dimension_ ;
    }

    inline int ImageStore::size(int axis) const {
        ogf_assert(axis >= 0 && axis < 3) ;
        return size_[axis] ;
    }

    inline int ImageStore::width() const {
        return size_[0] ;
    }

    inline int ImageStore::height() const {
        return size_[1] ;
    }

    inline int ImageStore::depth() const {
        return size_[2] ;
    }

    inline int ImageStore::bytes_per_pixel() const {
        return bytes_per_pixel_ ;
    }

    inline Memory::pointer ImageStore::base_mem() const {
        return base_mem_ ;
    }

    inline int ImageStore::bytes() const {
        return size_[0] * size_[1] * size_[2] * bytes_per_pixel_ ;
    }
//_________________________________________________________

}

#endif

