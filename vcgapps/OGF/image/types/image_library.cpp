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
 
#include <OGF/image/types/image_library.h>
#include <OGF/image/types/image.h>
#include <OGF/image/io/image_serializer.h>
#include <OGF/basic/os/file_manager.h>
#include <OGF/basic/debug/logger.h>

// For clipboard 
#ifdef WIN32
#include <windows.h>
#endif

namespace OGF {

//_________________________________________________________

    ImageLibrary* ImageLibrary::instance_ = nil ;

    void ImageLibrary::initialize() {
        ogf_assert(instance_ == nil) ;
        instance_ = new ImageLibrary() ;
        Environment::instance()->add_environment(instance_) ;
    }

    void ImageLibrary::terminate() {
        ogf_assert(instance_ != nil) ;
        delete instance_ ;
        instance_ = nil ;
    }

    ImageLibrary::ImageLibrary() {
    }

    ImageLibrary::~ImageLibrary() {
    }


    bool ImageLibrary::bind_image_serializer(
        const std::string& extension, ImageSerializer* serializer
    ) {
        std::string upper_extension = extension ;
        String::to_uppercase(upper_extension) ;
        if(
            resolve_image_serializer(extension) != nil ||
            resolve_image_serializer(upper_extension) != nil 
        ) {
            return false ;
        }
        image_serializers_[extension] = serializer ;
        image_serializers_[upper_extension] = serializer ;
        Environment::notify_observers("image_read_extensions") ;
        Environment::notify_observers("image_write_extensions") ;
        return true ;
    }

    ImageSerializer* ImageLibrary::resolve_image_serializer(
        const std::string& extension
    ) const {
        std::map<std::string, ImageSerializer_var>::const_iterator it = 
            image_serializers_.find(extension) ;
        if(it == image_serializers_.end()) {
            return nil ;
        }
        return it->second ;
    }

    bool ImageLibrary::bind_image(const std::string& name, Image* image) {
        if(resolve_image(name) != nil) {
            return false ;
        }
        images_[name] = image ;
        return true ;
    }

    bool ImageLibrary::unbind_image(const std::string& name) {
        std::map<std::string, Image_var>::iterator it = 
            images_.find(name) ;
        if(it == images_.end()) {
            return false ;
        }
        images_.erase(it) ;
        return true ;
    }

    Image* ImageLibrary::resolve_image(const std::string& name) const {
        std::map<std::string, Image_var>::const_iterator it = 
            images_.find(name) ;
        if(it == images_.end()) {
            return nil ;
        }
        return it->second ;
    }

    Image* ImageLibrary::load_image(const std::string& file_name) {
        std::string extension = FileManager::instance()->extension(file_name) ;
        if(extension.length() == 0) {
            Logger::err("ImageLibrary") 
                << "no extension in file name" << std::endl ;
            return nil ;
        }
        
        ImageSerializer* serializer = resolve_image_serializer(extension) ;
        if(serializer == nil) {
            Logger::err("ImageLibrary") 
                << "could not find serializer for extension \'"
                << extension << "\'" << std::endl ;
            return nil ;
        }

        if(!serializer->read_supported()) {
            Logger::err("ImageLibrary") 
                << "serializer for extension \'"
                << extension << "\' does not have a \'read\' function" 
                << std::endl ;
            return nil ;
        }

        return serializer->serialize_read(file_name) ;
    }

    bool ImageLibrary::save_image(
        const std::string& file_name, Image* image
    ) {

        std::string extension = FileManager::instance()->extension(file_name) ;
        if(extension.length() == 0) {
            Logger::err("ImageLibrary") 
                << "no extension in file name" << std::endl ;
            return false ;
        }
        
        ImageSerializer* serializer = resolve_image_serializer(extension) ;
        if(serializer == nil) {
            Logger::err("ImageLibrary") 
                << "could not find serializer for extension \'"
                << extension << "\'" << std::endl ;
            return false ;
        }

        if(!serializer->write_supported()) {
            Logger::err("ImageLibrary") 
                << "serializer for extension \'"
                << extension << "\' does not have a \'write\' function" 
                << std::endl ;
            return false ;
        }
        
        return serializer->serialize_write(file_name, image) ;
    }

    void ImageLibrary::copy_image_to_clipboard(Image* image) {

#ifdef WIN32

        if(image->color_encoding() != Image::RGB) {
            Logger::err("ImageLibrary") << "copy_image_to_clipboard() "
                                        << "not implemented for this color encoding" 
                                        << std::endl ;
            return ;
        }
        
        // Thanks to Pierre Alliez for his help with
        // Windows clipboard programming.
        
        // Step 1: Try to open Window's clipboard
        //   nil -> bind to current process
        if(!::OpenClipboard( nil )) {
            return ;
        }
        
        int h = image->height() ;
        int w = image->width() ;
        
        // Step 2: Prepare the image for Windows: flip the image and flip rgb -> bgr
        {
            int row_len = image->width() * 3 ;
            for(int j=0; j< h/2; j++) {
                Memory::pointer row1 = image->base_mem() + j * row_len ;
                Memory::pointer row2 = image->base_mem() + (h - 1 - j) * row_len ;
                for(int i=0; i<w; i++) {
                    ogf_swap(row1[3*i+2], row2[3*i  ]) ;
                    ogf_swap(row1[3*i+1], row2[3*i+1]) ;
                    ogf_swap(row1[3*i  ], row2[3*i+2]) ;
                }
            }
        }
        
        // Step 3: create a shared memory segment, with
        // a DIB (Device Independent Bitmap) in it.
        HANDLE handle;
        
        int image_size = 3 * image->width() * image->height();
        int size = sizeof(BITMAPINFOHEADER) + image_size ;
        
        handle = (HANDLE)::GlobalAlloc(GHND,size);
        if(handle != nil) {
            char *pData = (char *) ::GlobalLock((HGLOBAL)handle);
            BITMAPINFOHEADER header ;
            header.biSize          = sizeof(BITMAPINFOHEADER);
            header.biWidth         = image->width() ;
            header.biHeight        = image->height() ;
            header.biPlanes        = 1 ;
            header.biBitCount      = 24 ;
            header.biCompression   = BI_RGB ;
            header.biSizeImage     = 0 ;
            header.biXPelsPerMeter = 1000000 ;
            header.biYPelsPerMeter = 1000000 ;
            header.biClrUsed       = 0 ;
            header.biClrImportant  = 0 ;
            ::memcpy(pData,&header,sizeof(BITMAPINFOHEADER));	 
            ::memcpy(
                pData+sizeof(BITMAPINFOHEADER),image->base_mem(),image_size
            ) ;

            // Step 4: put the data in the clipboard.
            ::GlobalUnlock((HGLOBAL)handle);
            ::EmptyClipboard() ;
            ::SetClipboardData(CF_DIB,handle);
            ::CloseClipboard();

            // Step 5: restore the image
            {
                int row_len = image->width() * 3 ;
                for(int j=0; j< h/2; j++) {
                    Memory::pointer row1 = image->base_mem() + j * row_len ;
                    Memory::pointer row2 = image->base_mem() + (h - 1 - j) * row_len ;
                    for(int i=0; i<w; i++) {
                        ogf_swap(row1[3*i+2], row2[3*i  ]) ;
                        ogf_swap(row1[3*i+1], row2[3*i+1]) ;
                        ogf_swap(row1[3*i  ], row2[3*i+2]) ;
                    }
                }
            }
        }
#else
        Logger::err("ImageLibrary") << "copy_image_to_clipboard() "
                                    << "not implemented for this OS" 
                                    << std::endl ;
#endif
    }

    bool ImageLibrary::resolve(
        const std::string& name, std::string& value
    ) const {
        if(name == "image_read_extensions") {
            value = "" ;
            for(std::map<std::string, ImageSerializer_var>::const_iterator 
                    it = image_serializers_.begin(); it != image_serializers_.end(); it++
            ) {
                if(it->second->read_supported()) {
                    if(value.length() != 0) {
                        value += ";" ;
                    }
                    value += "*." ;
                    value += it->first ;
                }
            }
            return true ;
        } else if(name=="image_write_extensions") {
            value = "" ;
            for(std::map<std::string, ImageSerializer_var>::const_iterator 
                    it = image_serializers_.begin(); it != image_serializers_.end(); it++
            ) {
                if(it->second->write_supported()) {
                    if(value.length() != 0) {
                        value += ";" ;
                    }
                    value += "*." ;
                    value += it->first ;
                }
            }
            return true ;
        } else {
            return false ;
        }
    }

//_________________________________________________________

}

