/*
 *  OGF/Graphite: Geometry and Graphics Programming Library + Utilities
 *  Copyright (C) 2000-2005 INRIA - Project ALICE
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
 *  Contact: Bruno Levy - levy@loria.fr
 *
 *     Project ALICE
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs. 
 *
 * As an exception to the GPL, Graphite can be linked with the following (non-GPL) libraries:
 *     Qt, SuperLU, WildMagic and CGAL
 */
 

#ifndef __OGF_IMAGE_ALGOS_IMAGE_FILTER__
#define __OGF_IMAGE_ALGOS_IMAGE_FILTER__

#include <OGF/image/common/common.h>
#include <OGF/image/types/image.h>
#include <OGF/math/geometry/types.h>

namespace OGF {

    enum ColorSpace { 
        COLOR_SPACE_RGB, 
        COLOR_SPACE_YIQ, 
        COLOR_SPACE_XYZ, 
        COLOR_SPACE_LUV,
        COLOR_SPACE_HSV
    } ;

    enum ImageCurvature {
        IMG_CURV_K, IMG_CURV_H, IMG_CURV_KMIN, IMG_CURV_KMAX
    } ;

    namespace Img {

        bool IMAGE_API have_same_size(const Image* img1, const Image* img2) ;

        /**
         *  returns a new image of same size as the template. No data is copied.
         */
        inline Image* create_image_from_template(
            const Image* template_img, Image::ColorEncoding encoding
        ) {
            return new Image(
                encoding, template_img->width(), template_img->height(), template_img->depth()
            ) ;
        }

        /**
         *  returns a new image of same size and color encoding as the template. 
         *  No data is copied.
         */
        inline Image* create_image_from_template(const Image* template_img) {
            return create_image_from_template(template_img, template_img->color_encoding()) ;
        }
        
        /**
         * computes the minimum and maximum values in image.
         * @param Image should be one of Image::GRAY, Image::INT32, Image::FLOAT32.
         */
        void IMAGE_API get_min_max(Image* from, double& min_val, double& max_val) ;
        
        /**
         * converts an image in grayscale encoding. 
         * from and dest encodings should be one of Image::GRAY, Image::INT32, Image::FLOAT32.
         * mininum - maximum values are remapped to [0,255].
         */
        void IMAGE_API gray_to_gray(Image* from, Image* to) ;

        /**
         * converts an image into Image::GRAY encoding with specified range.
         * @param from encoding should be one of Image::GRAY, Image::INT32, Image::FLOAT32.
         * @param to encoding should be Image::GRAY
         * @param minrange value mapped to 0
         * @param maxrange value mapped to 255
         */
        void IMAGE_API gray_to_gray_with_range(
            Image* from, Image* to, double minrange, double maxrange
        ) ;

        /**
         * converts an image from RGB or RGBA to GRAY
         */
        void IMAGE_API to_gray(Image* from, Image* to) ;

        /**
         * extracts R,G,B or A from an RGB or RGBA image.
         * @param from should be in RGB or RGBA color encoding.
         * @param to should be in GRAY or FLOAT32 color encoding.
         */
        void IMAGE_API get_component(Image* from, int comp, Image* to) ;

        /**
         * converts an image from GRAY to RGB.
         */
        void IMAGE_API gray_to_rgb(Image* from, Image* to) ;

        /**
         * converts an image from GRAY to FLOAT32.
         * @param from should be in GRAY or INDEXED color encoding.
         * @param to should be in FLOAT32 color encoding.
		 * @param minrange_target_value value 0 gets mapped to
		 * @param minrange_target_value value 255 gets mapped to
         */
		void IMAGE_API gray_to_float32(Image* from_image, Image* to_image, double minrange_target_value = 0.0, double maxrange_target_value = 1.0);

        typedef double Neighborhood[3][3] ;
        typedef Neighborhood FilterKernel ;
        typedef Vector3d NormalKernel[8] ;

        typedef double Neighborhood3d[3][3][3] ;
        typedef Neighborhood3d FilterKernel3d ;
		typedef double TimeNeighborhood3d[5] ;
        typedef TimeNeighborhood3d TimeFilterKernel3d ;

        extern IMAGE_API FilterKernel PREWITT ;
        extern IMAGE_API FilterKernel SOBEL ;
        extern IMAGE_API FilterKernel KIRSCH ;
        extern IMAGE_API FilterKernel ROBINSON ;        


        void IMAGE_API filter_gradient(Image* from, Image* to) ;
        void IMAGE_API filter_compass(Image* from, Image* to, FilterKernel& K) ;

        void IMAGE_API flip_y(Image* image) ;
		void IMAGE_API filter_canny_2d(Image* from_image, Image* to_image, int Treshold_High, int Treshold_Low, int Hysteresis_neighborhood, double canny_value, double sobel_merge_factor) ;  // added by Nicolas S. 7/6/2007

        void IMAGE_API curvature(Image* from, Image* to, int radius = 2, ImageCurvature kind = IMG_CURV_KMIN) ;
        void IMAGE_API gradient(Image* from, Image* to, int radius = 1) ;
        void IMAGE_API normal(Image* from_image, Image* to_image) ;

        void IMAGE_API convert_color_space(
            Image* from_image, ColorSpace from_color_space,
            Image* to_image, ColorSpace to_color_space
        ) ;

        /**
         * finds the pixels in image object_ids that are on the iso-graylevel value.
         * The selected pixels are set to 255 in to_image.
         * @param from_image should be in GRAY color encoding.
         * @param value belongs to 0..255
         * @param to_image should be in GRAY color encoding.
         */
        void IMAGE_API extract_iso_curve(Image* from_image, double value, Image* to_image) ;

        void IMAGE_API magnify_2x(Image* from_image, Image* to_image) ;

        void IMAGE_API make_dimensions_multiple_of_four(Image* image, bool center = true) ;
        void IMAGE_API make_square(Image* image, bool center = true) ;

        //============================= Low-level functions ==========================

        inline int get_pixel_component(Image* image, int x, int y, int comp) {
            ogf_clamp(x, 0, image->width() - 1) ;
            ogf_clamp(y, 0, image->height() - 1) ;
            return image->pixel_base(x,y)[comp] ;
        }

        inline double get_pixel_value(Image* image, int x, int y) {
            ogf_clamp(x, 0, image->width() - 1) ;
            ogf_clamp(y, 0, image->height() - 1) ;
            double result = 0.0 ;
            Memory::pointer p = image->pixel_base(x,y) ;
            switch(image->color_encoding()) {
            case Image::GRAY: {
                result = *p ;
            } break ;
            case Image::INT16: {
                result = *((Numeric::int16*)p) ;
            } break ;
            case Image::INT32: {
                result = *((Numeric::int32*)p) ;
            } break ;
            case Image::FLOAT32: {
                result = *((Numeric::float32*)p) ;
            } break ;
            case Image::FLOAT64: {
                result = *((Numeric::float64*)p) ;
            } break ;
            default: {
                ogf_assert_not_reached ;
            }
            }
            return result ;
        }

        inline double get_pixel_value(Image* image, int x, int y, int z) {
            ogf_clamp(x, 0, image->width() - 1) ;
            ogf_clamp(y, 0, image->height() - 1) ;
            ogf_clamp(z, 0, image->depth() - 1) ;
            double result = 0.0 ;
            Memory::pointer p = image->pixel_base(x,y,z) ;
            switch(image->color_encoding()) {
            case Image::GRAY: {
                result = *p ;
            } break ;
            case Image::INT16: {
                result = *((Numeric::int16*)p) ;
            } break ;
            case Image::INT32: {
                result = *((Numeric::int32*)p) ;
            } break ;
            case Image::FLOAT32: {
                result = *((Numeric::float32*)p) ;
            } break ;
            case Image::FLOAT64: {
                result = *((Numeric::float64*)p) ;
            } break ;
            default: {
                ogf_assert_not_reached ;
            }
            }
            return result ;
        }
        
        inline void get_pixel_neighborhood(Image* image, int x, int y, Neighborhood& N) {
            N[0][0] = get_pixel_value(image, x-1, y-1) ;
            N[0][1] = get_pixel_value(image, x   , y-1) ;
            N[0][2] = get_pixel_value(image, x+1, y-1) ;
            N[1][0] = get_pixel_value(image, x-1, y) ;
            N[1][1] = get_pixel_value(image, x   , y) ;
            N[1][2] = get_pixel_value(image, x+1, y) ;
            N[2][0] = get_pixel_value(image, x-1, y+1) ;
            N[2][1] = get_pixel_value(image, x   , y+1) ;
            N[2][2] = get_pixel_value(image, x+1, y+1) ;
        }

        inline void get_pixel_neighborhood(Image* image, int x, int y, int z, Neighborhood3d& N) {
            for(int k=0; k<3; k++) {
                for(int j=0; j<3; j++) {
                    for(int i=0; i<3; i++) {
                        N[i][j][k] = get_pixel_value(image, x+i-1, y+j-1, z+k-1) ;
                    }
                }
            }
        }

        inline void get_pixel_neighborhood(Image* image, int x, int y, NormalKernel& N) {
            Point3d center(x, y, double(get_pixel_value(image, x, y)) / 255.0) ;
            N[0] = Point3d(x+1, y  , double(get_pixel_value(image, x+1, y  )) / 255.0) - center ;
            N[1] = Point3d(x+1, y+1, double(get_pixel_value(image, x+1, y+1)) / 255.0) - center ;
            N[2] = Point3d(x  , y+1, double(get_pixel_value(image, x  , y+1)) / 255.0) - center ;
            N[3] = Point3d(x-1, y+1, double(get_pixel_value(image, x-1, y+1)) / 255.0) - center ;
            N[4] = Point3d(x-1, y  , double(get_pixel_value(image, x-1, y  )) / 255.0) - center ;
            N[5] = Point3d(x-1, y-1, double(get_pixel_value(image, x-1, y-1)) / 255.0) - center ;
            N[6] = Point3d(x  , y-1, double(get_pixel_value(image, x  , y-1)) / 255.0) - center ;
            N[7] = Point3d(x+1, y-1, double(get_pixel_value(image, x+1, y-1)) / 255.0) - center ;
        }

        inline void set_pixel_value(Image* image, int x, int y, double value) {
            Memory::pointer p = image->pixel_base(x,y) ;
            switch(image->color_encoding()) {
            case Image::GRAY: {
                ogf_clamp(value, 0.0, 255.0) ;
                *p = Memory::byte(value) ;
            } break ;
            case Image::INT16: {
                *((Numeric::int16*)p) = Numeric::int16(value) ;
            } break ;
            case Image::INT32: {
                *((Numeric::int32*)p) = Numeric::int32(value) ;
            } break ;
            case Image::FLOAT32: {
                *((Numeric::float32*)p) = Numeric::float32(value) ;
            } break ;
            case Image::FLOAT64: {
                *((Numeric::float64*)p) = Numeric::float64(value) ;
            } break ;
            default: {
                ogf_assert_not_reached ;
            }
            }
        }

		class AngleTable2d { // added by Nicolas S. 7/6/2007
		public:
			AngleTable2d() ;
			AngleTable2d(int width, int height) ;

			void set_orientation(int w, int h, double angle) ;
			void get_orientation(int w, int h, int& theta) ;
			bool angle_check_theta[4];

		private:
			static const int maxindex = 1024 ; 
			Memory::byte** orientation_theta ;
		} ;

    }
}

#endif

