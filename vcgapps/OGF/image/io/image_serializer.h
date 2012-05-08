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
 
#ifndef __OGF_IMAGE_IO_IMAGE_SERIALIZER__
#define __OGF_IMAGE_IO_IMAGE_SERIALIZER__

#include <OGF/image/common/common.h>
#include <OGF/image/types/image.h>
#include <OGF/basic/types/counted.h>

#include <iostream>

namespace OGF {

//_________________________________________________________


    class Image ;

    class IMAGE_API ImageSerializer : public Counted {
    public:

        virtual Image* serialize_read(const std::string& file_name) ;
        virtual bool serialize_write(
            const std::string& file_name, const Image* image
        ) ;

        virtual Image* serialize_read(std::istream& stream) ;
        virtual bool serialize_write(
            std::ostream& stream, const Image* image
        ) ;

        /**
         * checks whether the stream should be opened
         * in text or binary mode. Default returns true.
         */
        virtual bool binary() const ;

        /**
         * checks whether reading and writing to streams is
         * supported.
         */
        virtual bool streams_supported() const ;

        /**
         * checks whether reading is implemented.
         */
        virtual bool read_supported() const ;

        /**
         * checks whether writing is implemented.
         */
        virtual bool write_supported() const ;

        /**
         * function provided to flip an image vertically
         */
        virtual bool flip_image(Image& image) const;

        /**
         * function provided to swap channels on image
         */
        virtual bool swap_channels(Image& image, int channel0, int channel1) const;

        /**
         * function provided to convert rgb image to bgr images 
         * (with or without alpha channel)
         */
        virtual bool rgb_to_bgr(Image& image) const;
    } ; 

    typedef SmartPointer<ImageSerializer> ImageSerializer_var ;

//_________________________________________________________

}
#endif

