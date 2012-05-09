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
 

#include <OGF/image/common/common.h>
#include <OGF/basic/debug/logger.h>
#include <OGF/basic/modules/module.h>

#include <OGF/image/types/image.h>
/*#include <OGF/image/types/image_library.h>
#include <OGF/image/io/image_serializer_xpm.h>
#include <OGF/image/io/image_serializer_ppm.h>
#include <OGF/image/io/image_serializer_rgb.h>
#include <OGF/image/io/image_serializer_tga.h>
#include <OGF/image/io/image_serializer_png.h>
#include <OGF/image/io/image_serializer_bmp.h>
#include <OGF/image/io/image_serializer_jpeg.h>
*/
#if 0
namespace OGF {
    
/****************************************************************/

    void image_libinit::initialize() {
        Logger::out("Init") << "Initializing library \'" 
                            << "image" << "\'" << std::endl ; 
        //_____________________________________________________________

        ImageLibrary::initialize() ;
        ogf_declare_image_serializer<ImageSerializer_ppm>("ppm") ;
        ogf_declare_image_serializer<ImageSerializer_rgb>("rgb") ;
        ogf_declare_image_serializer<ImageSerializer_rgb>("rgba") ;
        ogf_declare_image_serializer<ImageSerializer_tga>("tga") ;
        ogf_declare_image_serializer<ImageSerializer_xpm>("xpm") ;
        ogf_declare_image_serializer<ImageSerializer_png>("png") ;
        ogf_declare_image_serializer<ImageSerializer_bmp>("bmp") ;
        ogf_declare_image_serializer<ImageSerializer_jpeg>("jpg") ;

        //_____________________________________________________________

        Module* module_info = new Module ;
        module_info->set_name("image") ;
        module_info->set_vendor("OGF") ;
        module_info->set_version("1.0-a4") ;
        module_info->set_info("Image representations and I/O") ;
        Module::bind_module("image", module_info) ;

        Logger::out("Init") << "Initialized library \'" 
                            << "image" << "\'" << std::endl ; 
    }
    
    void image_libinit::terminate() {
        Logger::out("Init") << "Terminating library \'" 
                            << "image" << "\'" << std::endl ; 

        //_____________________________________________________________

        ImageLibrary::terminate() ;

        //_____________________________________________________________

        Module::unbind_module("image") ;
        
        Logger::out("Init") << "Terminated library \'" 
                            << "image" << "\'" << std::endl ; 
    }
    
// You should not need to modify this file below that point.
    
/****************************************************************/
    
    image_libinit::image_libinit() {
        increment_users() ;
    }

    image_libinit::~image_libinit() {
        decrement_users() ;
    }
    
    void image_libinit::increment_users() {
        // Note that count_ is incremented before calling
        // initialize, else it would still be equal to
        // zero at module initialization time, which 
        // may cause duplicate initialization of libraries.
        count_++ ;
        if(count_ == 1) {
            initialize() ;
        }
    }
    
    void image_libinit::decrement_users() {
        count_-- ;
        if(count_ == 0) {
            terminate() ;
        }
    }
    
    int image_libinit::count_ = 0 ;
    
}

// The initialization and termination functions
// are also declared using C linkage in order to 
// enable dynamic linking of modules.

extern "C" void IMAGE_API OGF_image_initialize() {
    OGF::image_libinit::increment_users() ;
}

extern "C" void IMAGE_API OGF_image_terminate() {
    OGF::image_libinit::decrement_users() ;
}


#endif
