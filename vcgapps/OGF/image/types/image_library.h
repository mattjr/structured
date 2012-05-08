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
 
 
 
 

#ifndef __OGF_IMAGE_TYPES_IMAGE_LIBRARY__
#define __OGF_IMAGE_TYPES_IMAGE_LIBRARY__

#include <OGF/image/common/common.h>
#include <OGF/image/types/image.h>
#include <OGF/image/io/image_serializer.h>
#include <OGF/basic/os/environment.h>
#include <OGF/basic/types/basic_factory.h>

#include <string>
#include <map>

namespace OGF {


//_________________________________________________________

    class ImageSerializer ;
    class Image ;

    class IMAGE_API ImageLibrary : public Environment {
    public:
        static ImageLibrary* instance() { return instance_; }
        static void initialize() ;
        static void terminate() ;

        bool bind_image_serializer(
            const std::string& extension, ImageSerializer* serializer
        ) ;
    
        ImageSerializer* resolve_image_serializer(
            const std::string& extension
        ) const ;

        bool bind_image(const std::string& name, Image* image) ;
        bool unbind_image(const std::string& name) ;
        Image* resolve_image(const std::string& name) const ;

        Image* load_image(const std::string& file_name) ;
        bool save_image(const std::string& file_name, Image* image) ;

        void copy_image_to_clipboard(Image* image) ;

        /**
         * Provides the following environment variables:
         * image_read_extensions
         * image_write_extensions
         */
        virtual bool resolve(const std::string& name, std::string& value) const ;

    protected:
        ImageLibrary() ;
        ~ImageLibrary() ;
        friend class World ;

    private:
        static ImageLibrary* instance_ ;
        std::map<std::string, ImageSerializer_var> image_serializers_ ;
        std::map<std::string, Image_var> images_ ;
    } ;

//_________________________________________________________

    template <class T> class ogf_declare_image_serializer {
    public:
        ogf_declare_image_serializer(const std::string& extension) {
            ImageLibrary::instance()->bind_image_serializer(
                extension, new T
            ) ;
        }
    } ;

//_________________________________________________________

}
#endif

