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

#ifndef __PARAMETERIZER_ALGOS_GIM_RASTERIZER__
#define __PARAMETERIZER_ALGOS_GIM_RASTERIZER__

#include <OGF/parameterizer/common/common.h>
#include <OGF/image/types/types.h>
#include <OGF/math/geometry/types.h>


class PBuffer ;

namespace OGF {
    
    class Image ;

    /**
     * Draws triangles in a geometry image.
     * Note: we absolutely need a more accurate algorithm, this ones
     * is bad for low sampling rates.
     */
    class PARAMETERIZER_API GimRasterizer {
    public:
        enum {MAX_POLYGON=100} ;

        /** RGBFLOAT and RGBAFLOAT ColorEncoding only are supported for the moment */
        GimRasterizer(Image* target) ;
        ~GimRasterizer() ;
        Image* target() { return target_ ; }

        void depth_test(bool x) ;
        void begin_frame() ;
        void begin_polygon() ;
        void end_polygon() ;
        void begin_line(double width = 1.0) ;
        void end_line() ;
        void vertex(const Point2d& uv, const Point3d& p) ;
        void vertex(const Point3d& uvw, const Point3d& p) ;
        void end_frame() ;

        /**
         * We got precision problems with the hardware rasterizer. To fix these problems, this 
         * function can be used (after end_frame() was called) to redraw the borders properly,
         * using the exact equation.
         */
        void square_border(const std::vector<Point2d>& uv, const std::vector<Point3d>& xyz) ;

        static PBuffer* get_pbuffer(int width, int height) ;

    protected:
        static void init_extensions() ;
        void set_pixel(int x, int y, const Point3d& p) ;
        void set_pixel(const Point2d& uv, const Point3d& p) ;

    private:
        Image* target_ ;
        PBuffer* pbuffer_ ;
        enum { PBUFFER_POOL_SIZE = 2 } ;
        static PBuffer* pbuffer_pool_[PBUFFER_POOL_SIZE] ;
        static double pbuffer_pool_timestamp_[PBUFFER_POOL_SIZE] ;
    } ;
}

#endif
