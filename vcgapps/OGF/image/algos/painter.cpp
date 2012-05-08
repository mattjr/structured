/*
 *  OGF/Graphite: Geometry and Graphics Programming Library + Utilities
 *  Copyright (C) 2004 Bruno Levy
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
 

#include <OGF/image/algos/painter.h>
#include <OGF/image/types/image_library.h>
#include <OGF/basic/os/file_manager.h>

#include <string.h>

#include <iostream>

namespace OGF {

//_________________________________________________________

    Painter::Painter() {
        load_brush("brush_3") ;
        pattern_ = new Image(Image::RGBA, 32, 32) ;
        image_ = new Image(Image::RGBA, 1024, 1024) ;
        clear_image(image_) ;
        create_undo_buffer() ;
        align_pattern_ = true ;
    }
    
    Painter::Painter(Image* image) : image_(image) {
        load_brush("brush_3") ;
        pattern_ = new Image(Image::RGBA, 32, 32) ;
        create_undo_buffer() ;
        align_pattern_ = true ;
    }

    Painter::~Painter() {
    }
    
    void Painter::set_image(Image* image) {
        image_ = to_rgba(image) ;
        create_undo_buffer() ;
        update() ;
    }

    void Painter::load_image(const std::string& file_name) {
        Logger::out("Painter") << "Loading image: name=" << file_name << std::endl ;
        Image_var image = ImageLibrary::instance()->resolve_image(
            FileManager::instance()->base_name(file_name) 
        ) ;
        if(image != nil) {
            image-> acquire() ;
        } else {
            image_ = ImageLibrary::instance()->load_image(file_name) ;
        }
        if(image_ == nil) {
            image_ = new Image(Image::RGBA, 256, 256) ;
        } else {
            image_ = to_rgba(image_) ;
        }
        create_undo_buffer() ;
        update() ;
    }
    
    void Painter::save_image(const std::string& file_name) {
        ImageLibrary::instance()-> save_image(file_name, image_) ;
    }

    void Painter::new_image(int width, int height) {
        image_ = new Image(Image::RGBA, width, height) ;
        clear_image(image_) ;
        create_undo_buffer() ;
        update() ;
    }

    
    void Painter::load_brush(const std::string& file_name_in) {
        std::string file_name = "brushes/" + file_name_in + ".xpm";
        FileManager::instance()-> find_file(file_name) ;
        brush_ = ImageLibrary::instance()-> load_image(file_name) ;
        if(brush_ == nil) {
            brush_ = new Image(Image::RGBA, 32, 32) ;
        } else {
            brush_ = to_rgba(brush_) ;
        }
    }
    
    void Painter::load_pattern(const std::string& file_name_in) {
//        std::string file_name = "patterns/" + file_name_in + ".xpm" ;
        std::string file_name = file_name_in ;
        FileManager::instance()-> find_file(file_name) ;
        pattern_ = ImageLibrary::instance()->load_image(file_name) ;
        if(pattern_ == nil) {
            pattern_ = new Image(Image::RGBA, 32, 32) ;
        } else {
            pattern_ = to_rgba(pattern_) ;
        }
    }

    void Painter::update() {
        // TODO
    }


    void Painter::paint(double x_in, double y_in, PaintMode mode) {

        int width  = image_-> width() ;
        int height = image_-> height() ;

        int x = int(x_in * width ) ;
        int y = int(y_in * height) ;

        if(x < 0 || y < 0 || x >= width || y >= height) {
            return ;
        }

        x -= brush_-> width()  / 2 ;
        y -= brush_-> height() / 2 ;

        switch(mode) {
        case BRUSH:
            blit_modulate(image_, x, y, brush_, brush_color_) ;
            break ;
        case ERASE:
            blit_modulate(image_, x, y, brush_, Color(1,1,1,1)) ;
            break ;
        case PATTERN:
            blit_pattern(image_, x, y, brush_, pattern_) ;
            break ;
        case XOR:
            blit_xor(image_, x, y, brush_) ;
            update() ;
            blit_xor(image_, x, y, brush_) ;
            break ;
        }

        update() ;
    }

    Image* Painter::to_rgba(Image* image_in) {
        switch(image_in-> color_encoding()) {
        case Image::INDEXED:
        {
            Image* result = new Image(
                Image::RGBA, image_in-> width(), image_in-> height()
            ) ;
            
            Memory::pointer from = image_in-> base_mem() ;
            Memory::pointer to = result-> base_mem() ;
            Colormap* cmap = image_in-> colormap() ;

            int nb = image_in-> width() * image_in-> height() ;
            for(int i=0; i<nb; i++) {
                Colormap::ColorCell& color = cmap-> color_cell(*from) ;
                *to = color.r() ; to++ ;
                *to = color.g() ; to++ ;
                *to = color.b() ; to++ ;
                *to = color.a() ; to++ ;
                from++ ;
            }
            return result ;
        }
        break ;
        case Image::RGB:
        {
            Image* result = new Image(
                Image::RGBA, image_in-> width(), image_in-> height()
            ) ;
            
            Memory::pointer from = image_in-> base_mem() ;
            Memory::pointer to = result-> base_mem() ;
            
            int nb = image_in-> width() * image_in-> height() ;
            for(int i=0; i<nb; i++) {
                *to = *from ; from++; to++ ;
                *to = *from ; from++; to++ ;
                *to = *from ; from++; to++ ;
                *to = 255 ;   to++ ;
            }
            return result ;
        }
            break ;
        case Image::RGBA:
            return image_in ;
            break ;
        default:
            ogf_assert(false) ;
            break ;
        }
        return nil ;
    }

    static inline void combine_colors(
        Numeric::uint8* out,
        Numeric::uint8* brush,
        Numeric::uint8* color
    ) {
        int neg = *brush ;
        int pos = 256 - neg ;
        for(int i=0; i<4; i++) { 
            out[i] = Numeric::uint8(
                (int(out[i]) * neg + int(color[i]) * pos) >> 8
            ) ;
        }
    }
    
    static inline void xor_colors(
        Numeric::uint8* out,
        Numeric::uint8* brush
    ) {
        Numeric::uint8 arg = 255 - *brush ;
        for(int i=0; i<4; i++) { 
            out[i] = (out[i] ^ arg) ;
        }        
    }

    static inline void get_color(
        Numeric::uint8* out,
        const Color& c
    ) {
        out[0] = Numeric::uint8(c.r() * 255.0) ;
        out[1] = Numeric::uint8(c.g() * 255.0) ;
        out[2] = Numeric::uint8(c.b() * 255.0) ;
        out[3] = Numeric::uint8(c.a() * 255.0) ;
    }


    static inline Numeric::uint8* get_pixel(
        Image* image, int x, int y
    ) {
        return image-> base_mem() + 4 * (y * image-> width() + x ) ;
    }

    static inline bool in_image(
        Image* image, int x, int y
    ) {
        return (
            (x >= 0) && (y >= 0) && 
            (x < image->width()) && (y < image->height())
        ) ;
    }

    void Painter::blit_modulate(
        Image* dest, int x, int y, Image* brush, const Color& color_in
    ) {

        int width = brush-> width() ;
        int height = brush-> height() ;

        Numeric::uint8 color[4] ;
        get_color(color, color_in) ;

        for(int cur_x = 0; cur_x < width; cur_x++) {
            for(int cur_y = 0; cur_y < height; cur_y++) {
                if(in_image(dest, x+cur_x, y+cur_y)) {
                    combine_colors(
                        get_pixel(dest , x+cur_x, y+cur_y),
                        get_pixel(brush, cur_x  , cur_y  ),
                        color
                    ) ;
                }
            }
        } 
    }

    void Painter::blit_pattern(
        Image* dest, int x, int y, Image* brush, Image* pattern
    ) {
        int width = brush-> width() ;
        int height = brush-> height() ;

        for(int cur_x = 0; cur_x < width; cur_x++) {
            for(int cur_y = 0; cur_y < height; cur_y++) {
                if(in_image(dest, x+cur_x, y+cur_y)) {

                    int pat_x ;
                    int pat_y ;

                    if(align_pattern_) {
                        pat_x = (x+cur_x)%pattern->width() ;
                        pat_y = (y+cur_y)%pattern->height() ;
                    } else {
                        pat_x = cur_x%pattern->width() ;
                        pat_y = cur_y%pattern->height() ;
                    }

                    combine_colors(
                        get_pixel(dest   ,x+cur_x, y+cur_y),
                        get_pixel(brush  ,cur_x  , cur_y ),
                        get_pixel(pattern,pat_x  , pat_y ) 
                    ) ;
                }
            }
        } 
    }


    void Painter::blit_xor(
        Image* dest, int x, int y, Image* brush
    ) {

        int width = brush-> width() ;
        int height = brush-> height() ;

        for(int cur_x = 0; cur_x < width; cur_x++) {
            for(int cur_y = 0; cur_y < height; cur_y++) {
                if(in_image(dest, x+cur_x, y+cur_y)) {
                    xor_colors(
                        get_pixel(dest , x+cur_x, y+cur_y),
                        get_pixel(brush, cur_x  , cur_y  )
                    ) ;
                }
            }
        } 
    }


    void Painter::clear_image(Image* image_in) {
        int n = 
            image_in-> bytes_per_pixel() * 
            image_in-> width() * image_in-> height() ;
        ::memset(image_in-> base_mem(), 255, n) ;
    }

    void Painter::copy_image(Image* to, Image* from) {
        ogf_assert(to-> color_encoding() == from-> color_encoding()) ;
        ogf_assert(to-> width() == from-> width()) ;
        ogf_assert(to-> height() == from-> height()) ;
        ::memcpy(
            to->base_mem(), from->base_mem(), 
            to->width()*to->height()*to->bytes_per_pixel()
        ) ;
    }

    void Painter::commit() {
        copy_image(undo_buffer_, image_) ;
    }
    
    void Painter::rollback() {
        copy_image(image_, undo_buffer_) ;
        update() ;
    }

    void Painter::undo() {
        rollback() ;
    }

    void Painter::create_undo_buffer() {
        undo_buffer_ = new Image(
            image_-> color_encoding(), image_-> width(), image_-> height()
        ) ;
        clear_image(undo_buffer_) ;
    }

//_________________________________________________________

}

