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

#ifndef __IMAGE_ALGOS_RASTERIZER__
#define __IMAGE_ALGOS_RASTERIZER__

#include <OGF/image/common/common.h>
#include <OGF/image/types/types.h>
#include <OGF/math/geometry/types.h>


namespace OGF {
    
    class Image ;

    /**
     * Draws gouraud-shaded triangles and polygons in an Image.
     */
    class IMAGE_API Rasterizer {
    public:
        enum {MAX_POLYGON=100} ;

        /** RGBA ColorEncoding only is supported for the moment */
        Rasterizer(Image* target) ;
        ~Rasterizer() ;

        /** 
         * Clipping is not implemented yet,
         * All coordinates are clamped in [0..width-1],[0..height-1] 
         */
        void triangle(
            const Point2d& p1, const Color& n1,
            const Point2d& p2, const Color& n2,
            const Point2d& p3, const Color& n3
        ) ;
        void begin_polygon() ;
        /** 
         * Clipping is not implemented yet,
         * All coordinates are clamped in [0..width-1],[0..height-1] 
         */
        void vertex(const Point2d& p1, const Color& n1) ;
        void end_polygon() ;

    protected:
        struct Vertex {
            int x ;
            int y ;
            int r ;
            int g ;
            int b ;
            int a ;
        } ;

        void rasterize(Vertex* v, int nb_vertices) ;

    private:
        Image* target_ ;
        Numeric::uint8* graph_mem_ ;
        int width_ ;
        int height_ ;
        int bytes_per_line_ ;
        Vertex current_polygon_[MAX_POLYGON] ;
        int current_polygon_index_ ;

        struct ScanLineExtremity {
            int x ;
            int r ;
            int g ;
            int b ;
            int a ;
        } ;
        ScanLineExtremity* left_  ;
        ScanLineExtremity* right_ ;
    } ;
}

#endif
