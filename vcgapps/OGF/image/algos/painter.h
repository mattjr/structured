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
 

#ifndef __OGF_IMAGE_ALGOS_PAINTER__
#define __OGF_IMAGE_ALGOS_PAINTER__

#include <OGF/image/types/image.h>

namespace OGF {

//_________________________________________________________

    class IMAGE_API Painter {
    public:
        Painter() ;
        Painter(Image* image) ;
        virtual ~Painter() ;

        enum PaintMode {BRUSH, PATTERN, ERASE, XOR} ;
        void paint(double x, double y, PaintMode mode) ;

        void load_image(const std::string& file_name) ;
        void save_image(const std::string& file_name) ;
        void new_image(int width, int height) ;
        void load_brush(const std::string& file_name) ;
        void load_pattern(const std::string& file_name) ;
        void set_image(Image* image) ;
        Image* get_image() const { return image_ ; }

        const Color& get_brush_color() const { return brush_color_ ; }
        void set_brush_color(const Color& c) { brush_color_ = c ;    }
        bool get_align_pattern() const       { return align_pattern_ ;  }
        void set_align_pattern(bool b)       { align_pattern_ = b ; }

        void commit() ;
        void rollback() ;

        void undo() ;

    protected:

        void blit_modulate(
            Image* dest, int x, int y, Image* brush, const Color& color
        ) ;

        void blit_pattern(
            Image* dest, int x, int y, Image* brush, Image* pattern
        ) ;

        void blit_xor(Image* dest, int x, int y, Image* brush) ;

        void update() ;

        Image* to_rgba(Image* image_in) ;
        void clear_image(Image* image_in) ;
        void copy_image(Image* to, Image* from) ;
        void create_undo_buffer() ;

    private:
        Image_var image_ ;
        Image_var undo_buffer_ ;
        Image_var brush_ ;
        Image_var pattern_ ;
        Color brush_color_ ;
        bool align_pattern_ ;
    } ;

//_________________________________________________________

}
#endif

