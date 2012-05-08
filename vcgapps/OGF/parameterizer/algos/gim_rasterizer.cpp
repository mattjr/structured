#if 0
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


/*
 * Notes: 
 *  - When using a floating point buffer, fragment color
 *    is transformed into an int in [0...255], if we want
 *    floating point values, we can use tex coords or
 *    generic attributes.
 *
 *  - I still got small gaps between consecutive geometry 
 *    images.
 */

#include <OGF/parameterizer/algos/gim_rasterizer.h>
#include <OGF/image/types/image.h>
#include <OGF/basic/os/stopwatch.h>

#include <gl_stuff/pbuffer/pbuffer.h>

namespace OGF {

    PBuffer* GimRasterizer::pbuffer_pool_[2] = { nil, nil } ;
    double GimRasterizer::pbuffer_pool_timestamp_[2] = { 0, 0 } ;
    static GLuint fragment_program = 0 ;


    // Note: Even when using a floating point
    // frame buffer, color values attached to 
    // vertices are integers. For this reason,
    // we pass the color as the TEX0 texture
    // coordinates (texture coordinates are
    // floating point values).
    // At the end, the output color in the frame
    // buffer is a floating point value.
    const char fragment_program_source[] =
    "!!FP1.0\n"
    "MOVR o[COLR], f[TEX0];\n"
    "END\n";

    GimRasterizer::GimRasterizer(Image* target) {
        target_ = target ;
        ogf_assert( 
            target_->color_encoding() == Image::RGB_FLOAT32 ||
            target_->color_encoding() == Image::RGB
        ) ;
        pbuffer_ = get_pbuffer(target_->width(), target_->height()) ;
    }

    GimRasterizer::~GimRasterizer() {
        target_ = nil ;
    }


    void GimRasterizer::begin_frame() {
        pbuffer_->Activate() ;

        glViewport(0, 0, target()->width(), target()->height()) ;
        glMatrixMode(GL_PROJECTION) ;
        glLoadIdentity() ;
        glMatrixMode(GL_MODELVIEW) ;
        glLoadIdentity() ;

        glClearColor(0.0, 0.0, 0.0, 1.0) ;
        glClearDepth(0.0) ;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) ;

        glBindProgramNV(GL_FRAGMENT_PROGRAM_NV, fragment_program);
        glLoadProgramNV(
            GL_FRAGMENT_PROGRAM_NV, 
            fragment_program,
            strlen(fragment_program_source), 
            (const GLubyte *)fragment_program_source
        ) ;
        glEnable(GL_FRAGMENT_PROGRAM_NV);
    }

    void GimRasterizer::depth_test(bool x) {
        if(x) {
            glEnable(GL_DEPTH_TEST) ;
            glDepthFunc(GL_GREATER) ;
        } else {
            glDisable(GL_DEPTH_TEST) ;
        }
    }

    static bool is_zero(Image* img) {
        int size = img->width() * img->height() * img->bytes_per_pixel() ;
        for(int i=0; i<size; i++) {
            if(img->base_mem()[i] != 0) {
                return false ;
            }
        }
        return true ;
    }

    void GimRasterizer::end_frame() {
        glDisable(GL_FRAGMENT_PROGRAM_NV) ;
        glPixelStorei(GL_PACK_ALIGNMENT, 1) ; 
        glPixelStorei(GL_PACK_ROW_LENGTH, target()->width()) ; 

        switch(target()->color_encoding()) {
        case Image::RGB_FLOAT32:
            glReadPixels(
                0, 0, target()->width(), target()->height(), 
                GL_RGB, GL_FLOAT, target()->base_mem() 
            ) ;
            break ;
        case Image::RGB: {
            glReadPixels(
                0, 0, target()->width(), target()->height(), 
                GL_RGB, GL_UNSIGNED_BYTE, target()->base_mem() 
            ) ;
        } break ;
        default:
            ogf_assert_not_reached ;
        }
        pbuffer_->Deactivate() ;
        if(is_zero(target())) {
            std::cerr << "Image is empty" << std::endl ;
        }
    }

    void GimRasterizer::begin_polygon() {
        glBegin(GL_POLYGON) ;
    }

    void GimRasterizer::end_polygon() {
        glEnd() ;
    }

    void GimRasterizer::begin_line(double w) {
        glLineWidth(w) ;
        glBegin(GL_LINE_STRIP) ;
    }

    void GimRasterizer::end_line() {
        glEnd() ;
    }
    
    // Note: we do not use glColor because glColor is clamped to [0,1]
    // and converted to an int !
    void GimRasterizer::vertex(const Point2d& uv, const Point3d& xyz) {
        float w = static_cast<float>(target_->width()) ;
//        float s = (float(w) / (float(w) + 1.0)) ;
        float s = ((float(w) - 0.99) / float(w)) ;

        glTexCoord3d(xyz.x(), xyz.y(), xyz.z()) ;

        // TODO: setup viewing matrix instead ...
        float u = s * 2.0 * (uv.x() - 0.5) ;
        float v = s * 2.0 * (uv.y() - 0.5) ;

        glVertex2d(u,v) ;
    }

    void GimRasterizer::vertex(const Point3d& uvw, const Point3d& xyz) {
        float wd = static_cast<float>(target_->width()) ;
        float s = ((float(wd) - 0.99) / float(wd)) ;

        glTexCoord3d(xyz.x(), xyz.y(), xyz.z()) ;

        // TODO: setup viewing matrix instead ...
        float u = s * 2.0 * (uvw.x() - 0.5) ;
        float v = s * 2.0 * (uvw.y() - 0.5) ;
        float w = uvw.z() ;

        glVertex3d(u,v,w) ;
    }
    

    PBuffer* GimRasterizer::get_pbuffer(int width, int height) {

        if(pbuffer_pool_[0] == nil) {
            init_extensions() ;
        }

        for(unsigned int i=0; i<PBUFFER_POOL_SIZE; i++) {
            if(
                pbuffer_pool_[i] != nil &&
                width  == pbuffer_pool_[i]->GetWidth() &&
                height == pbuffer_pool_[i]->GetHeight()
            ) {
                pbuffer_pool_timestamp_[i] = SystemStopwatch::now() ;
                return pbuffer_pool_[i] ;
            }
        }

        PBuffer* new_pbuffer = new PBuffer("float=32 depth") ; 
        bool ok = new_pbuffer->Initialize(
            width, height, false, true
        ) ;
        ogf_assert(ok) ;

        for(unsigned int i=0; i<PBUFFER_POOL_SIZE; i++) {
            if(pbuffer_pool_[i] == nil) {
                pbuffer_pool_[i] = new_pbuffer ;
                pbuffer_pool_timestamp_[i] = SystemStopwatch::now() ;
                return new_pbuffer ;
            }
        }

        int lru = 0 ;
        for(unsigned int i=0; i<PBUFFER_POOL_SIZE; i++) {
            if(pbuffer_pool_timestamp_[i] < pbuffer_pool_timestamp_[lru]) {
                lru = i ;
            }
        }

        delete pbuffer_pool_[lru] ;
        pbuffer_pool_[lru] = new_pbuffer ;
        pbuffer_pool_timestamp_[lru] = SystemStopwatch::now() ;
        return new_pbuffer ;

    }


    void GimRasterizer::square_border(const std::vector<Point2d>& uv, const std::vector<Point3d>& xyz) {

        ogf_assert(uv.size() == xyz.size()) ;

        int count = uv.size() ;

        const Point2d& uv0 = uv[0] ;
        const Point2d& uv1 = uv[uv.size() - 1] ;
        int varying_coord = ::fabs(uv1.x() - uv0.x()) > ::fabs(uv1.y() - uv0.y()) ? 0 : 1 ;
        int fixed_coord = 1 - varying_coord ;

        int w = target_->size(varying_coord) ;
        int h = target_->size(fixed_coord) ;

        int fixed_coord_val = int(double(h-1) * (0.5 * (uv0[fixed_coord] + uv1[fixed_coord]))) ;

        bool flipped = (uv1[varying_coord] < uv0[varying_coord]) ;
        bool swapped = (fixed_coord == 0) ;


        // Flips the curvilinear absissa if needed
        std::vector<double> s ;
        if(flipped) {
            for(unsigned int i=0; i<uv.size(); i++) {
                s.push_back(1.0 - uv[i][varying_coord]) ;
            }
        } else {
            for(unsigned int i=0; i<uv.size(); i++) {
                s.push_back(uv[i][varying_coord]) ;
            }
        }

        double ds = 1.0 / double(w - 1.0) ;
        double cur_s = 0.0 ;
        unsigned int cur_index = 0 ;
        
        for(unsigned int i=0; i<w; i++) {

            while(s[cur_index+1] < cur_s) {
                cur_index++ ;
            }

            double local_ds = s[cur_index+1] - s[cur_index] ;
            double local_s2 ;
            if(::fabs(local_ds) < 1e-30) {
                local_s2 = 1.0 ;
            } else {
                local_s2 = (cur_s - s[cur_index]) / local_ds ;
            }
            double local_s1 = 1.0 - local_s2 ;
            const Point3d& p1 = xyz[cur_index] ;
            const Point3d& p2 = xyz[cur_index+1] ;

            Point3d cur_p(
                local_s1 * p1.x() + local_s2 * p2.x(),
                local_s1 * p1.y() + local_s2 * p2.y(),
                local_s1 * p1.z() + local_s2 * p2.z()
            ) ;

            if(swapped) {
                set_pixel(fixed_coord_val, flipped ? w - 1 - i : i, cur_p) ;
            } else {
                set_pixel(flipped ? w - 1 - i : i, fixed_coord_val, cur_p) ;
            }
            cur_s += ds ;
        }

        set_pixel(uv[0], xyz[0]) ;
        set_pixel(uv[uv.size() - 1], xyz[xyz.size() - 1]) ;
    }


    void GimRasterizer::set_pixel(int x, int y, const Point3d& p) {
        ogf_range_assert(x, 0, target_->width() - 1) ;
        ogf_range_assert(y, 0, target_->height() - 1) ;
        float* ptr = (float*)(target_->pixel_base(x,y)) ;
        ptr[0] = p.x() ; ptr[1] = p.y(); ptr[2] = p.z() ;
    }

    void GimRasterizer::set_pixel(const Point2d& uv, const Point3d& p) {
        int x = int(uv.x() * (target_->width() - 1)) ;
        int y = int(uv.y() * (target_->height() - 1)) ;
        set_pixel(x,y,p) ;
    }


//________________________________________________________________________

#if defined(WIN32)
#define REQUIRED_EXTENSIONS "GL_ARB_texture_compression " \
                            "GL_EXT_texture_compression_s3tc " \
                            "GL_NV_texture_rectangle " \
                            "GL_NV_vertex_program " \
                            "GL_NV_fragment_program " \
                            "GL_NV_float_buffer " \
                            "WGL_ARB_pixel_format " \
                            "WGL_ARB_pbuffer "
#else
#define REQUIRED_EXTENSIONS "GL_ARB_texture_compression " \
                            "GL_EXT_texture_compression_s3tc " \
                            "GL_NV_texture_rectangle " \
                            "GL_NV_vertex_program " \
                            "GL_NV_fragment_program " \
                            "GLX_NV_float_buffer " \
                            "GLX_SGIX_fbconfig " \
                            "GLX_SGIX_pbuffer "
#endif



    void GimRasterizer::init_extensions() {
        if(!glewIsSupported(REQUIRED_EXTENSIONS)) {
            Logger::err("Fatal") << "Could not initialize OpenGL extensions"
                                 << std::endl ;
            ogf_assert_not_reached ;
        }
        
        glGenProgramsNV(1, &fragment_program);
    }

}

#endif
