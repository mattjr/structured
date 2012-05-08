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
 

#include <OGF/image/algos/image_filter.h>
#include <OGF/image/types/image.h>

#include <OGF/math/functions/bivariate.h>

namespace OGF {

    namespace Img {

        bool have_same_size(const Image* img1, const Image* img2) {
            return (
                img1->width() == img2->width() &&
                img1->height() == img2->height() &&
                img1->depth() == img2->depth()
            ) ;
        }

        //---------------------------------------------------------------------------------------------------------------------------------

        template <class T> inline void get_min_max(T* data, unsigned int nb, T& vmin, T& vmax) {
            vmin = data[0] ;
            vmax = data[0] ;
            for(unsigned int i=1; i<nb; i++) {
                vmin = ogf_min(vmin, data[i]) ;
                vmax = ogf_max(vmax, data[i]) ;
            }
        }

        template <class T> inline void get_min_max_double(
            T* data, unsigned int nb, double& vmin, double& vmax
        ) {
            T vminT, vmaxT ;
            get_min_max(data, nb, vminT, vmaxT) ;
            vmin = double(vminT) ;
            vmax = double(vmaxT) ;
        }

        template <class Tfrom, class Tto> inline void down_convert(
            Tfrom* from, Tto* to, unsigned int nb, const Tfrom& maxval
        ) {
            Tfrom vmin,vmax ;
            get_min_max(from, nb, vmin, vmax) ;
            std::cerr << "min = " << vmin << " max = " << vmax << std::endl ;
            for(unsigned int i=0; i<nb; i++) {
                to[i] = Tto(maxval * (from[i] - vmin) / (vmax - vmin)) ;
            }
        }

        template <class Tfrom> inline void down_convert_to_bytes(
            Tfrom* from, Memory::byte* to, unsigned int nb, double minrange, double maxrange
        ) {
            for(unsigned int i=0; i<nb; i++) {
                double x = 255.0 * (from[i] - minrange) / (maxrange - minrange) ;
                ogf_clamp(x, 0.0, 255.0) ;
                to[i] = Memory::byte(x) ;
            }
        }

        template <class Tfrom, class Tto> inline void up_convert(
            Tfrom* from, Tto* to, unsigned int nb
        ) {
            for(unsigned int i=0; i<nb; i++) {
                to[i] = Tto(from[i]) ;
            }
        }

        void get_min_max(Image* from, double& min_val, double& max_val) {
            unsigned int nuvw = from->nb_pixels() ;
            switch(from->color_encoding()) {
            case Image::GRAY: {
                get_min_max_double(from->base_mem_byte_ptr(), nuvw, min_val, max_val) ;
            } break ;
            case Image::INT16: {
                get_min_max_double(from->base_mem_int16_ptr(), nuvw, min_val, max_val) ;
            } break ;
            case Image::INT32: {
                get_min_max_double(from->base_mem_int32_ptr(), nuvw, min_val, max_val) ;
            } break ;
            case Image::FLOAT32: {
                get_min_max_double(from->base_mem_float32_ptr(), nuvw, min_val, max_val) ;
            } break ;
            case Image::FLOAT64: {
                get_min_max_double(from->base_mem_float64_ptr(), nuvw, min_val, max_val) ;
            } break ;
            default: {
                ogf_assert_not_reached;
            } 
            }
        }

        void gray_to_gray_with_range(Image* from, Image* to, double min_val, double max_val) {
            ogf_assert(have_same_size(from, to)) ;
            ogf_assert(to->color_encoding() == Image::GRAY) ;
            Memory::byte* to_ptr = to->base_mem_byte_ptr() ;
            unsigned int nuvw = from->nb_pixels() ;
            switch(from->color_encoding()) {
            case Image::GRAY: {
                down_convert_to_bytes(from->base_mem_byte_ptr(), to_ptr, nuvw, min_val, max_val) ;
            } break ;
            case Image::INT16: {
                down_convert_to_bytes(from->base_mem_int16_ptr(), to_ptr, nuvw, min_val, max_val) ;
            } break ;
            case Image::INT32: {
                down_convert_to_bytes(from->base_mem_int32_ptr(), to_ptr, nuvw, min_val, max_val) ;
            } break ;
            case Image::FLOAT32: {
                down_convert_to_bytes(from->base_mem_float32_ptr(), to_ptr, nuvw, min_val, max_val) ;
            } break ;
            case Image::FLOAT64: {
                down_convert_to_bytes(from->base_mem_float64_ptr(), to_ptr, nuvw, min_val, max_val) ;
            } break ;
            default: {
                ogf_assert_not_reached;
            } 
            }
        }


        void gray_to_gray(Image* from, Image* to) {
            ogf_assert(have_same_size(from, to)) ;
            int nuvw = from->nb_pixels() ;

            if(from->color_encoding() == Image::GRAY && to->color_encoding() == Image::GRAY) {
                up_convert(from->base_mem_byte_ptr(), to->base_mem_byte_ptr(), nuvw) ;
                return ;
            }
            if(from->color_encoding() == Image::GRAY && to->color_encoding() == Image::INT32) {
                up_convert(from->base_mem_byte_ptr(), to->base_mem_int32_ptr(), nuvw) ;
                return ;
            }
            if(from->color_encoding() == Image::GRAY && to->color_encoding() == Image::FLOAT32) {
                up_convert(from->base_mem_byte_ptr(), to->base_mem_float32_ptr(), nuvw) ;
                return ;
            }

            if(from->color_encoding() == Image::INT32 && to->color_encoding() == Image::GRAY) {
                down_convert(from->base_mem_int32_ptr(), to->base_mem_byte_ptr(), nuvw, 255) ;
                return ;
            }
            if(from->color_encoding() == Image::INT32 && to->color_encoding() == Image::INT32) {
                up_convert(from->base_mem_int32_ptr(), to->base_mem_int32_ptr(), nuvw) ;
                return ;
            }
            if(from->color_encoding() == Image::INT32 && to->color_encoding() == Image::FLOAT32) {
                up_convert(from->base_mem_int32_ptr(), to->base_mem_float32_ptr(), nuvw) ;
                return ;
            }

            if(from->color_encoding() == Image::FLOAT32 && to->color_encoding() == Image::GRAY) {
                down_convert(from->base_mem_float32_ptr(), to->base_mem_byte_ptr(), nuvw, 255.0f) ;
                return ;
            }
            if(from->color_encoding() == Image::FLOAT32 && to->color_encoding() == Image::INT32) {
                down_convert(from->base_mem_float32_ptr(), to->base_mem_int32_ptr(), nuvw, 32768.0f) ;
                return ;
            }
            if(from->color_encoding() == Image::FLOAT32 && to->color_encoding() == Image::FLOAT32) {
                up_convert(from->base_mem_float32_ptr(), to->base_mem_float32_ptr(), nuvw) ;
                return ;
            }
            ogf_assert_not_reached ;
        }

        //===============================================================================

        void  to_gray(Image* from_image, Image* to_image) {
            
            ogf_assert(
                from_image->color_encoding() == Image::RGB || 
                from_image->color_encoding() == Image::RGBA 
            ) ;
            ogf_assert(to_image->color_encoding() == Image::GRAY) ;

            ogf_assert(have_same_size(from_image, to_image)) ;

            int nb_pixels = from_image->nb_pixels() ;
            Memory::pointer from = from_image->base_mem() ;
            Memory::pointer to = to_image->base_mem() ;
            for(int i=0; i<nb_pixels; i++) {
                float graylevel = 0.3f * float(from[0]) + 0.59f * float(from[1]) + 0.11f * float(from[2]) ;
                *to = Memory::byte(graylevel) ;
                from += from_image->bytes_per_pixel() ;
                to++ ;
            }
        }

        //____________________________________________________________________________________

        void  gray_to_rgb(Image* from_image, Image* to_image) {
            
            ogf_assert(from_image->color_encoding() == Image::GRAY) ;
            ogf_assert(to_image->color_encoding() == Image::RGB) ;
            ogf_assert(have_same_size(from_image, to_image)) ;
            
            int nb_pixels = from_image->nb_pixels() ;
            Memory::pointer from = from_image->base_mem() ;
            Memory::pointer to = to_image->base_mem() ;
            for(int i=0; i<nb_pixels; i++) {
                to[0] = to[1] = to[2] = *from ;
                from++ ;
                to += to_image->bytes_per_pixel() ;
            }
        }


        //____________________________________________________________________________________

        void gray_to_float32(Image* from_image, Image* to_image, double minrange_target_value, double maxrange_target_value)
		{            
            ogf_assert(from_image->color_encoding() == Image::GRAY || from_image->color_encoding() == Image::INDEXED) ;
            ogf_assert(to_image->color_encoding() == Image::FLOAT32) ;
            ogf_assert(have_same_size(from_image, to_image)) ;
            
			float remapped_values[256];

			for(unsigned int i = 0 ; i < 256 ; i++)
				remapped_values[i] = (float)(i / 255.0 * (maxrange_target_value - minrange_target_value) + minrange_target_value);

            int nb_pixels = from_image->nb_pixels() ;
            Memory::pointer from = from_image->base_mem() ;
            Memory::pointer to = to_image->base_mem() ;

            for(int i=0; i<nb_pixels; i++) {
                *(float*)to = remapped_values[*from];
                from++ ;
                to += to_image->bytes_per_pixel() ;
            }
        }


        //____________________________________________________________________________________

        void get_component(Image* from_image, int comp, Image* to_image) {
            ogf_assert(
                from_image->color_encoding() == Image::RGB || 
                from_image->color_encoding() == Image::RGBA 
            ) ;
            ogf_assert(
                to_image->color_encoding() == Image::GRAY || to_image->color_encoding() == Image::FLOAT32
            ) ;
            ogf_assert(have_same_size(from_image, to_image)) ;
            ogf_assert(comp >= 0 && comp < from_image->bytes_per_pixel()) ;
            int nb_pixels = from_image->nb_pixels() ;

            switch(to_image->color_encoding()) {
            case Image::GRAY: {
                Memory::byte* from = from_image->base_mem_byte_ptr() ;
                Memory::byte* to = to_image->base_mem_byte_ptr() ;
                for(int i=0; i<nb_pixels; i++) {
                    *to = from[comp] ;
                    from += from_image->bytes_per_pixel() ;
                    to++ ;
                }
            } break ;
            case Image::FLOAT32: {
                Memory::byte* from = from_image->base_mem_byte_ptr() ;
                Numeric::float32* to = to_image->base_mem_float32_ptr() ;
                for(int i=0; i<nb_pixels; i++) {
                    *to = Numeric::float32(from[comp]) ;
                    from += from_image->bytes_per_pixel() ;
                    to++ ;
                }
            } break ;
            default: {
                ogf_assert_not_reached ;
            } break ;
            }
        }

        //____________________________________________________________________________________

        FilterKernel PREWITT = {
            { -1, 1, 1},
            { -1,-2, 1},
            { -1, 1, 1}
        } ;

        FilterKernel SOBEL = {
            { -1, 0, 1},
            { -2, 0, 2},
            { -1, 0, 1}
        } ;

		FilterKernel SOBEL_90 = {
            { -1, -2, -1},
            { 0, 0, 0},
            { 1, 2, 1}
        } ;

        FilterKernel KIRSCH = {
            { -3, -3, 5},
            { -3,  0, 5},
            { -3, -3, 5}
        } ;

        FilterKernel ROBINSON = {
            { -1, 0, 1},
            { -1, 0, 1},
            { -1, 0, 1}
        } ;

		FilterKernel GAUSS = {
			{ 1.0/16.0, 2.0/16.0, 1.0/16.0},
            { 2.0/16.0, 4.0/16.0, 2.0/16.0},
            { 1.0/16.0, 2.0/16.0, 1.0/16.0}
		} ;

        //  00 01 02        01 02 12
        //  10 11 12  -->  00 11 22 
        //  20 21 22        10 20 21

        static void rotate_filter_kernel(const FilterKernel& from, FilterKernel& to) {
            to[0][0] = from[0][1] ;
            to[0][1] = from[0][2] ;
            to[0][2] = from[1][2] ;
            to[1][0] = from[0][0] ;
            to[1][1] = from[1][1] ;
            to[1][2] = from[2][2] ;
            to[2][0] = from[1][0] ;
            to[2][1] = from[2][0] ;
            to[2][2] = from[2][1] ;
        }

        static inline double convolve_filter(const FilterKernel& K, const Neighborhood& N) {
            double result = 0 ;
            for(int i=0; i<3; i++) {
                for(int j=0; j<3; j++) {
                    result += K[i][j] * N[i][j] ;
                }
            }
            return result ;
        }

        static void filter_gradient_2d(Image* from_image, Image* to_image) {
            ogf_assert(from_image->color_encoding() == Image::GRAY) ;
            ogf_assert(to_image->color_encoding() == Image::GRAY) ;
            ogf_assert(have_same_size(from_image, to_image)) ;
            ogf_assert(from_image->dimension() == 2) ;
            FilterKernel N ;
            for(int j=0; j<from_image->height(); j++) {
                for(int i=0; i<from_image->width(); i++) {
                    get_pixel_neighborhood(from_image, i, j, N) ;
                    double grad = 0.0 ;
                    for(int dj = 0 ; dj < 3; dj++) {
                        for(int di = 0; di < 3; di++) {
                            if(dj == 1 && di == 1) continue ;
                            double ddi = di - 1 ;
                            double ddj = dj - 1 ;
                            double cur = 1.0 / ::sqrt(ddi*ddi + ddj*ddj) * ::fabs(double(N[di][dj] - N[1][1])) ;
                            grad += cur ;
                        }
                    }
                    grad = grad / 8.0 ;
                    ogf_clamp(grad, 0.0, 255.0) ;
                    *to_image->pixel_base(i,j) = Memory::byte(grad) ;
                }
            }
        }

        static void filter_gradient_3d(Image* from_image, Image* to_image) {
            ogf_assert(from_image->color_encoding() == Image::GRAY) ;
            ogf_assert(to_image->color_encoding() == Image::GRAY) ;
            ogf_assert(have_same_size(from_image, to_image)) ;
            ogf_assert(from_image->dimension() == 3) ;
            FilterKernel3d N ;
            for(int k=0; k<from_image->depth(); k++) {
                for(int j=0; j<from_image->height(); j++) {
                    for(int i=0; i<from_image->width(); i++) {
                        get_pixel_neighborhood(from_image, i, j, k, N) ;
                        double grad = 0.0 ;
                        for(int dk = 0; dk < 3; dk++) {
                            for(int dj = 0 ; dj < 3; dj++) {
                                for(int di = 0; di < 3; di++) {
                                    if(dk == 1 && dj == 1 && di == 1) continue ;
                                    double ddi = di - 1 ;
                                    double ddj = dj - 1 ;
                                    double ddk = dk - 1 ;
                                    double cur = 1.0 / ::sqrt(ddi*ddi + ddj*ddj + ddk * ddk) * ::fabs(double(N[di][dj][dk] - N[1][1][1])) ;
                                    grad += cur ;
                                }
                            }
                        }
                        grad = grad / 26.0 ;
                        ogf_clamp(grad, 0.0, 255.0) ;
                        *to_image->pixel_base(i,j,k) = Memory::byte(grad) ;
                    }
                }
            }
        }
        
        void filter_gradient(Image* from_image, Image* to_image) {
            ogf_assert(have_same_size(from_image, to_image)) ;
            switch(from_image->dimension()) {
            case 2: {
                filter_gradient_2d(from_image, to_image) ;
            } break ;
            case 3: {
                filter_gradient_3d(from_image, to_image) ;
            } break ;
            default:
                ogf_assert_not_reached ;
            }
        }

        void filter_compass(Image* from_image, Image* to_image, FilterKernel& K_in) {
            
            ogf_assert(have_same_size(from_image, to_image)) ;
            ogf_assert(from_image->dimension() == 2) ;

            FilterKernel K[8] ;
            Memory::copy(&K[0],&K_in,sizeof(FilterKernel)) ;
            for(int i=1; i<8; i++) {
                rotate_filter_kernel(K[i-1], K[i]) ;
            }

            Neighborhood N ;
            for(int j=0; j<from_image->height(); j++) {
                for(int i=0; i<from_image->width(); i++) {
                    double g = 0.0 ;
                    get_pixel_neighborhood(from_image, i, j, N) ;
                    for(int k=0; k<8; k++) {
                        g = ogf_max(g, convolve_filter(K[k],N)) ;
                    }
                    g += 1.0 ; // for vectorizer
                    set_pixel_value(to_image, i, j, g) ;
                }
            }
        }

        void  flip_y(Image* image) {
            int bytes_per_line = image->bytes_per_pixel() * image->width() ;
            Memory::byte* buff = new Memory::byte[bytes_per_line] ;
            for(int y =0; y<image->height() / 2; y++) {
                Memory::pointer p1 = image->base_mem() + y * bytes_per_line ;
                Memory::pointer p2 = image->base_mem() + (image->height() - 1 - y) * bytes_per_line ;
                Memory::copy(buff, p1, bytes_per_line) ;
                Memory::copy(p1, p2, bytes_per_line) ;
                Memory::copy(p2, buff,  bytes_per_line) ;
            }
            delete[] buff ;
        }


		void filter_2d_image(Image* from_image, Image* to_image, const FilterKernel& kernel) { // added by Nicolas S. 7/6/2007
			Neighborhood N ;
			for(int j=0; j<from_image->height(); j++) {
                for(int i=0; i<from_image->width(); i++) {
                    double g = 0.0 ;
					get_pixel_neighborhood(from_image, i, j, N) ;
					g = convolve_filter(kernel,N) ;
					set_pixel_value(to_image, i, j, g) ;
				}
			}
		}


		void filter_canny_2d(Image* from_image, Image* to_image, int Treshold_High, int Treshold_Low, int Hysteresis_neighborhood, double canny_value, double sobel_merge_factor) { // added by Nicolas S. 7/6/2007
			ogf_assert(have_same_size(from_image, to_image)) ;
            ogf_assert(from_image->dimension() == 2) ;
			if ((canny_value < 0) && (canny_value>256)) { Logger::err("ARDECO II") << "Filtering : bad value for parameter : canny value ... exiting ..." << std::endl ; return;}
			if ((sobel_merge_factor < 0.0) && (canny_value>50.0)) { Logger::err("ARDECO II") << "Filtering : bad value for parameter : sobel merge factor ... exiting ..." << std::endl ; return;}
			bool use_float = false ;
			if (to_image->color_encoding() == Image::GRAY) { Logger::out("ARDECO II") << "Gray level filtering (8 bits)" << std::endl ; }
			else if (to_image->color_encoding() == Image::FLOAT32) { Logger::out("ARDECO II") << "Float filtering (32 bits)" << std::endl ; use_float = true ; }
			else { Logger::err("ARDECO II") << "Filtering : only GRAY and FLOAT32 color encodings are supported" << std::endl ; }
			double max_code_value = 255.0 ;
			ogf_assert((Treshold_High>0) && (Treshold_High<(max_code_value+1))) ;
			if (Treshold_Low>Treshold_High) { 
				Logger::out("ARNOOVO") << "Canny Filter Problem: Treshold_Low > Treshold_High... assumed Treshold_Low = Treshold_High = " << Treshold_High << std::endl ;
				Treshold_Low = Treshold_High ;
			}
			int width = from_image->width();
			int height = from_image->height();

			Image* gradient_temp = Img::create_image_from_template(from_image) ;
			Image* gaussian_filtered = Img::create_image_from_template(from_image) ;

			filter_2d_image(from_image, gaussian_filtered, GAUSS) ;
			Logger::out("ARDECO II") << "filter canny-style 2d... 3-pass filtering"  << std::endl ;
			AngleTable2d to_angletable(width, height) ;
			FilterKernel* K[2] ;
			K[0]= &SOBEL_90 ; K[1]= &SOBEL ;
			Neighborhood N ;
			ProgressLogger* progressbar;
			progressbar = new ProgressLogger( 3*(from_image->height()), "canny filter 3d") ;
			Logger::status() << "filtering..." <<std::endl ;
            double g = 0.0 ;
			for(int j=0; j<height; j++) {
		        for(int i=0; i<width; i++) {
				    get_pixel_neighborhood(gaussian_filtered, i, j, N) ;
					double gx = convolve_filter(*K[0],N) ;
					double gy = convolve_filter(*K[1],N) ;
					Sign sgx = ogf_sgn(gx) ;
					Sign sgy = ogf_sgn(gy) ;
					gx = ogf_abs(gx) ;
					gy = ogf_abs(gy) ;
					/******* orientation ********/
					if (gx==0) {
						if (gy==0) {
							// gx = gy = 0
							to_angletable.set_orientation(i,j,0.0) ;
						}
						else { // gy != 0
							to_angletable.set_orientation(i,j,90.0) ;
						}
					}
					else { // gx != 0
						if (gy==0) {
							to_angletable.set_orientation(i,j,0.0) ;
						}
						else { // all != 0
							if ((sgx*sgy)==NEGATIVE) {to_angletable.set_orientation(i,j,(180-(atan(gy/gx)*180)/M_PI)) ;}
							else {to_angletable.set_orientation(i,j,(atan(gy/gx)*180)/M_PI) ;}
						}
					}
					/******* /orientation ********/
					g = sqrt(gx*gx + gy*gy) ;
					//int current_theta;
					//to_angletable.get_orientation(i, j, current_theta) ;
					//set_pixel_value(to_image, i, j, current_theta) ;
					//set_pixel_value(to_image, i, j, g/5.66*2) ;
					set_pixel_value(gradient_temp, i, j, g/5.66/**2*/) ;
					//int thetheta;
					//to_angletable.get_orientation(i,j,thetheta) ;
				}
				progressbar->next() ;
			}
			delete(gaussian_filtered) ;
			//Logger::out("THETA") << "0 : " << to_angletable.angle_check_theta[0] << "| 45 : " << to_angletable.angle_check_theta[1] << "| 90 : " << to_angletable.angle_check_theta[2] << "| 135 : " << to_angletable.angle_check_theta[3] << std::endl ;

			/******* nonmax suppression ********/
			Image* gradient_temp2 = new Image(gradient_temp) ;
			double gmin = max_code_value + 1 ;
			double gmax = 0 ;
			double maxes[4] ;
			int nonmax_count = 0 ;
			for(int j=0; j<height; j++) {
				for(int i=0; i<width; i++) {
					int current_theta;
					to_angletable.get_orientation(i, j,current_theta) ;
					double g = get_pixel_value(gradient_temp, i, j) ;
					switch (current_theta){
						case 0:
							maxes[0] = get_pixel_value(gradient_temp, i, j+1) ;
							maxes[1] = get_pixel_value(gradient_temp, i, j-1) ;
							//maxes[2] = get_pixel_value(gradient_temp, i, j+2) ;
							//maxes[3] = get_pixel_value(gradient_temp, i, j-2) ;
							//set_pixel_value(to_image, i, j-1, 255.0) ;
							//set_pixel_value(to_image, i, j+1, 255.0) ;
							break;
						case 45:
							maxes[0] = get_pixel_value(gradient_temp, i-1, j+1) ;
							maxes[1] = get_pixel_value(gradient_temp, i+1, j-1) ;
							//maxes[2] = get_pixel_value(gradient_temp, i-2, j+2) ;
							//maxes[3] = get_pixel_value(gradient_temp, i+2, j-2) ;
							//set_pixel_value(to_image, i-1, j+1, 255.0) ;
							//set_pixel_value(to_image, i+1, j-1, 255.0) ;
							break;
						case 90:
							maxes[0] = get_pixel_value(gradient_temp, i-1, j) ;
							maxes[1] = get_pixel_value(gradient_temp, i+1, j) ;
							//maxes[2] = get_pixel_value(gradient_temp, i-2, j) ;
							//maxes[3] = get_pixel_value(gradient_temp, i+2, j) ;
							//set_pixel_value(to_image, i-1, j, 255.0) ;
							//set_pixel_value(to_image, i+1, j, 255.0) ;
							break ;
						case 135:
							maxes[0] = get_pixel_value(gradient_temp, i-1, j-1) ;
							maxes[1] = get_pixel_value(gradient_temp, i+1, j+1) ;
							//maxes[2] = get_pixel_value(gradient_temp, i-2, j-2) ;
							//maxes[3] = get_pixel_value(gradient_temp, i+2, j+2) ;
							//set_pixel_value(to_image, i-1, j-2, 255.0) ;
							//set_pixel_value(to_image, i+1, j+2, 255.0) ;
							break;
						default:
							ogf_assert_not_reached ;
					}

					if (g!= 0) { 
						if (g<gmin) {gmin=g;}
						if (g>gmax) {gmax=g;}
					}	
					//bool real_max = true ;
					//for (int ite=0;ite<2;ite++) {
					//	real_max &= (g>maxes[i]) ;
					//}
					//bool real_max = ((g>maxes[0])&(g>maxes[1])&(g>maxes[2])&(g>maxes[3])) ;
					bool real_max = ((g>=maxes[0])&(g>=maxes[1])) ;
					if (!real_max) {
						set_pixel_value(gradient_temp2, i, j, 0.0) ;
						//set_pixel_value(to_image, i, j, k, 0.0) ;
						nonmax_count++ ;
					}
				}
			progressbar->next() ;
			}

			/******* hysteresis thresholding ********/
			double final_value = 0.0 ;
			int thresholded = 0 ; int highed_in = 0;int lowed_out = 0; int thres_ok = 0 ; int thres_nok =0 ;
			for(int j=0; j<height; j++) {
		        for(int i=0; i<width; i++) {
					double g = get_pixel_value(gradient_temp2, i, j) ;
					if (g>Treshold_High) {
						//set_pixel_value(to_image, i, j, k, canny_value ;
						final_value = canny_value ;
						highed_in++;
					}
					else if (g>Treshold_Low) {
						//set_pixel_value(to_image, i, j, k, 0) ;
						final_value = 0.0 ;
						lowed_out++ ;
					}
					else { // value is between 2 thresholds
						thresholded++ ;
						int thetheta ;
						to_angletable.get_orientation(i,j,thetheta) ;
						//bool is_edge = verify_edge_connectivity(gradient_temp2,i,j,k,thetheta,thephi,Treshold_High,Treshold_Low,Hysteresis_neighborhood) ;
						bool is_edge = false ;
						if (is_edge) {/*set_pixel_value(to_image, i, j, k, canny_value) ;*/ final_value = canny_value ; thres_ok++;}
						else {/*set_pixel_value(to_image, i, j, k, 0) ;*/ final_value = 0 ;thres_nok++;}
					}
					final_value += sobel_merge_factor*get_pixel_value(gradient_temp, i, j) ;
					
					set_pixel_value(to_image, i, j, final_value) ;
				}
			progressbar->next() ;
			}
			Logger::out("ARNOOVO") << "STATS : high = " << highed_in << " | low = " << lowed_out << std::endl ;
			Logger::out("ARNOOVO") << "STATS : threshold = " << thresholded << " : ok = " << thres_ok << " = " << thres_ok*100/thresholded << "% | nok = " << thres_nok << " = " << thres_nok*100/thresholded << "%" << std::endl ;
			
			/******* /hysteresis thresholding ********/
			delete gradient_temp ;
			delete gradient_temp2 ;
			delete progressbar ;
		}


/***************************************************************************************************************************************/
		// added by Nicolas S. 7/6/2007
		AngleTable2d::AngleTable2d() {
			Logger::out("ARNOOVO") << "Wrong constructor used" << std::endl ;
		}

		AngleTable2d::AngleTable2d(int width, int height) {
			orientation_theta = new Memory::byte*[width] ;
			if (orientation_theta==nil) { ogf_assert_not_reached; }
			for ( int i = 0 ; i < width ; i++ ) {
				orientation_theta[i] = new Memory::byte[height] ;
				if (orientation_theta[i]==nil) { ogf_assert_not_reached; }
			}
			for(int i = 0 ; i<width ; i++) {
				for(int j = 0 ; j<height ; j++) {
					orientation_theta[i][j] = 0 ;
				}
			}
			Logger::out("ARDECO II") << "Orientation table initialized" << std::endl ;
			for (int i= 0;i<4;i++) {
				angle_check_theta[i] = false;
			}
		}

		void AngleTable2d::set_orientation(int w, int h, double angle) {
			int rest_angle = 0 ;
			if (angle < 22.5) { rest_angle = 0 ; angle_check_theta[0] = true; }
			else if(angle  < 67.5) { rest_angle = 45 ; angle_check_theta[1] = true;}
			else if(angle < 112.5) { rest_angle = 90 ; angle_check_theta[2] = true;}
			else if(angle < 157.5) { rest_angle = 135 ; angle_check_theta[3] = true;}
			else { rest_angle = 0 ; angle_check_theta[0] = true; }
			orientation_theta[w][h] = Memory::byte(rest_angle) ;
		}

		void AngleTable2d::get_orientation(int w, int h, int& theta) {
			theta = int(orientation_theta[w][h]) ;
		}
/***************************************************************************************************************************************/


        //_______________________________________________________________________________________

        static double pixel_curvature(Image* image, int x, int y, int radius, ImageCurvature kind) {
            double wx = 1.0 / double(image->width()) ;
            double wy = 1.0 / double(image->height()) ;
            BivariateFitting<BivariateQuadraticFunction> fitting ;
            fitting.begin() ;
            for(int ly = y-radius; ly <= y+radius; ly++) {
                for(int lx = x-radius; lx <= x+radius; lx++) {
                    fitting.add_sample(double(lx)*wx, double(ly)*wy, get_pixel_value(image, lx,ly) / 1024.0) ;
                }
            }
            fitting.end() ;
            BivariateQuadraticFunction F = fitting.result() ;

            double xf = double(x)*wx ;
            double yf = double(y)*wy ;


            double a1 = F.coeff[1] ;
            double a2 = F.coeff[2] ;
            double a3 = F.coeff[3] ;
            double a4 = F.coeff[4] ;
            double a5 = F.coeff[5] ;

            double Fx = a1 + 2.0 * a3 * xf + a4 * yf ;
            double Fxx = 2.0 * a3 ;
            double Fy = a2 + 2.0 * a5 * yf + a4 * xf ;
            double Fyy = 2.0 * a5 ;
            double Fxy = a4 ;

            double Fx2 = ogf_sqr(Fx) ;
            double Fy2 = ogf_sqr(Fy) ;

            double K = (Fxx * Fyy - ogf_sqr(Fxy)) / ogf_sqr(1.0 + Fx2 + Fy2) ;

            double H = 0.5 * (
                (1.0 + Fx2) * Fyy - 2.0 * Fx * Fy * Fxy + (1.0 + Fy2) * Fxx
            ) * ::pow(1.0 + Fx2 + Fy2, -3.0/2.0) ;

            double H2 = ogf_sqr(H) ;
            double K1 = H - ::sqrt(H2 - K) ;
            double K2 = H + ::sqrt(H2 - K) ;
            
            double result = 0 ;
            switch(kind) {
            case IMG_CURV_K: {
                result =  ::fabs(K) ;
            } break ;
            case IMG_CURV_H: {
                result =  ::fabs(H) ;
            } break ;
            case IMG_CURV_KMIN: {
                result =  ogf_min(::fabs(K1), ::fabs(K2)) ;
            } break ;
            case IMG_CURV_KMAX: {
                result =  ogf_max(::fabs(K1), ::fabs(K2)) ;
            } break ;
            default:
                ogf_assert_not_reached ;
            }
            return result ;
        } 


        void  curvature(Image* from_image, Image* to_image_in, int radius, ImageCurvature kind) {
            ogf_assert(from_image->color_encoding() == Image::GRAY) ;
            ogf_assert(have_same_size(from_image, to_image_in)) ;
            ogf_assert(from_image->dimension() == 2) ;
            Image_var to_image = to_image_in ;
            if(to_image_in->color_encoding() != Image::FLOAT32) {
                to_image = create_image_from_template(from_image, Image::FLOAT32) ;
            }
            for(int y=0; y<from_image->height(); y++) {
                std::cerr << "." << std::flush ;
                for(int x=0; x<from_image->width(); x++) {
                    double k = pixel_curvature(from_image, x, y, radius, kind) ;
                    *(float*)(to_image->pixel_base(x,y)) = float(k) ;
                }
            }
            std::cerr << std::endl ;
            if(to_image_in->color_encoding() != Image::FLOAT32) {
                gray_to_gray(to_image, to_image_in) ;
            }
        }

        static double pixel_gradient(Image* image, int x, int y, int radius = 2) {
            double wx = 1.0 / double(image->width()) ;
            double wy = 1.0 / double(image->height()) ;
            typedef BivariateQuadraticFunction ModelFunction ;
            BivariateFitting<ModelFunction> fitting ;
            fitting.begin() ;
            for(int ly = y-radius; ly <= y+radius; ly++) {
                for(int lx = x-radius; lx <= x+radius; lx++) {
                    fitting.add_sample(double(lx)*wx, double(ly)*wy, get_pixel_value(image, lx,ly)) ;
                }
            }
            fitting.end() ;
            ModelFunction F = fitting.result() ;
            /*
              double xf = double(x)*wx ;
              double yf = double(y)*wy ;
              double grad_x = F.coeff[1] + 2.0 * F.coeff[3] * xf + F.coeff[4] * yf ;
              double grad_y = F.coeff[2] + 2.0 * F.coeff[5] * yf + F.coeff[4] * xf ;
            */
            double grad_x = F.coeff[1] ;
            double grad_y = F.coeff[2] ;
            return ::sqrt(ogf_sqr(grad_x) + ogf_sqr(grad_y)) ;
        } 

        void  gradient(Image* from_image, Image* to_image_in, int radius) {
            ogf_assert(have_same_size(from_image, to_image_in)) ;
            ogf_assert(from_image->dimension() == 2) ;

            Image_var to_image = to_image_in ;
            if(to_image_in->color_encoding() != Image::FLOAT32) {
                to_image = create_image_from_template(from_image, Image::FLOAT32) ;
            }

            for(int y=0; y<from_image->height(); y++) {
                std::cerr << "." << std::flush ;
                for(int x=0; x<from_image->width(); x++) {
                    double g = pixel_gradient(from_image, x, y, radius) ;
                    *(Numeric::float32*)(to_image->pixel_base(x,y)) = Numeric::float32(g) ; 
                }
            }
            std::cerr << std::endl ;

            if(to_image_in->color_encoding() != Image::FLOAT32) {
                gray_to_gray(to_image, to_image_in) ;
            }
        }
		
        void normal(Image* from_image, Image* to_image) {
            ogf_assert(to_image->color_encoding() == Image::RGB) ;
            ogf_assert(have_same_size(from_image, to_image)) ;
            ogf_assert(from_image->dimension() == 2) ;

            NormalKernel N ;
            for(int y=0; y<from_image->height(); y++) {
                std::cerr << "." << std::flush ;
                for(int x=0; x<from_image->width(); x++) {
                    Vector3d result(0.0,0.0,0.0) ; ;
                    get_pixel_neighborhood(from_image, x, y, N) ;
                    for(int i=0;i<8;i++) {
                        Vector3d n = N[i] ^ N[(i+1)%8] ;
                        result = result + n ;
                    }
                    result.normalize() ;
                    Memory::pointer mem = (to_image->pixel_base(x,y)) ;
                    mem[0] = Memory::byte(result[0] * 255.0) ;
                    mem[1] = Memory::byte(result[1] * 255.0) ;
                    mem[2] = Memory::byte(result[2] * 255.0) ;
                }
            }
            std::cerr << std::endl ;
        }

        //________________________________________________________________________________________
        // Linear conversions

        typedef double ColorMatrix[3][3] ;

        static ColorMatrix RGB_TO_YIQ = 
        {      
            {0.299, 0.587,   0.114},
            {0.596, -0.275, -0.321},
            {0.212, -0.523,  0.311}
        } ;

        static ColorMatrix RGB_TO_XYZ = 
        {
            {0.607, 0.174, 0.200},
            {0.299, 0.587, 0.114},
            {0.0,    0.066,  1.111}
        } ;

        static inline void rgb_convert_pixel(
            double r, double g, double b, double& x1, double& x2, double& x3, ColorMatrix& M
        ) {
            x1 = M[0][0] * r + M[0][1] * g + M[0][2] * b ;
            x2 = M[1][0] * r + M[1][1] * g + M[1][2] * b ;
            x3 = M[2][0] * r + M[2][1] * g + M[2][2] * b ;
            ogf_clamp(x1, 0.0, 255.0) ;
            ogf_clamp(x2, 0.0, 255.0) ;
            ogf_clamp(x3, 0.0, 255.0) ;
        }

        static void rgb_convert_image(Image* from_image, Image* to_image, ColorMatrix& M) {
            ogf_assert(have_same_size(from_image, to_image)) ;
            ogf_assert(from_image->color_encoding() == Image::RGB) ;
            ogf_assert(to_image->color_encoding() == Image::RGB) ;
            int nb_pixels = from_image->nb_pixels() ;
            Memory::pointer from = from_image->base_mem() ;
            Memory::pointer to     = to_image->base_mem() ;
            for(int i=0; i<nb_pixels; i++) {
                double r = from[0] ;
                double g = from[1] ;
                double b = from[2] ;
                double x1,x2,x3 ;
                rgb_convert_pixel(r,g,b,x1,x2,x3,M) ;
                to[0] = Memory::byte(x1) ;
                to[1] = Memory::byte(x2) ;
                to[2] = Memory::byte(x3) ;
                from += 3 ;
                to += 3 ;
            }
        }

        static const double XYZ[3][3] = {	
            {  0.4125,  0.3576,  0.1804 },
            {  0.2125,  0.7154,  0.0721 },
            {  0.0193,  0.1192,  0.9502 }	
        } ;

//________________________________________________________________________________________
// RGB to LUV conversion        

        static void RGBtoLUV(Memory::byte *rgbVal, float *luvVal) {

            // Constants
//            const double Xn = 0.95050;
            const double Yn = 1.00000;
//            const double Zn = 1.08870;
            const double Un_prime	= 0.19784977571475;
            const double Vn_prime	= 0.46834507665248;
            const double Lt = 0.008856;
            

            //declare variables
            double x, y, z, L0, u_prime, v_prime, constant;

            //convert RGB to XYZ...
            x	= XYZ[0][0]*rgbVal[0] + XYZ[0][1]*rgbVal[1] + XYZ[0][2]*rgbVal[2];
            y	= XYZ[1][0]*rgbVal[0] + XYZ[1][1]*rgbVal[1] + XYZ[1][2]*rgbVal[2];
            z	= XYZ[2][0]*rgbVal[0] + XYZ[2][1]*rgbVal[1] + XYZ[2][2]*rgbVal[2];
            
            //convert XYZ to LUV...
            
            //compute L*
            L0	= y / (255.0 * Yn);
            if(L0 > Lt) {
                luvVal[0]	= (float)(116.0 * (pow(L0, 1.0/3.0)) - 16.0);
            } else {
                luvVal[0]	= (float)(903.3 * L0);
            }
            
            //compute u_prime and v_prime
            constant	= x + 15 * y + 3 * z;
            if(constant != 0) {
                u_prime	= (4 * x) / constant;
                v_prime = (9 * y) / constant;
            } else {
                u_prime	= 4.0;
                v_prime	= 9.0/15.0;
            }

            //compute u* and v*
            luvVal[1] = (float) (13 * luvVal[0] * (u_prime - Un_prime));
            luvVal[2] = (float) (13 * luvVal[0] * (v_prime - Vn_prime));
        }
        

        static void rgb_to_luv_convert_image(Image* from_image, Image* to_image) {
            ogf_assert(have_same_size(from_image, to_image)) ;
            ogf_assert(from_image->color_encoding() == Image::RGB) ;
            ogf_assert(to_image->color_encoding() == Image::RGB_FLOAT32) ;
            Memory::byte* from = from_image->base_mem_byte_ptr() ;
            Numeric::float32* to = to_image->base_mem_float32_ptr() ;
            unsigned int nb_pixels = from_image->nb_pixels() ;
            for(unsigned int i=0; i<nb_pixels; i++) {
                RGBtoLUV(from, to) ;
                from += 3 ; to += 3 ;
            }
        }

//________________________________________________________________________________________
// RGB to HSV conversion


        // Needed by rgb2hsv()
        static inline double maxrgb(double r, double g, double b) {
            return ogf_max(r, ogf_max(g,b)) ;
        }

        // Needed by rgb2hsv()
        static inline double minrgb(double r, double g, double b) {
            return ogf_min(r, ogf_min(g,b)) ;
        }

/* Taken from "Fund'l of 3D Computer Graphics", Alan Watt (1989)
    Assumes (r,g,b) range from 0.0 to 1.0
    Sets h in degrees: 0.0 to 360.;
    s,v in [0.,1.]
*/
        void RGB2HSV(
            double r, double g, double b,
            double *hout, double *sout, double *vout
        ) {
            double h=0,s=1.0,v=1.0;
            double max_v,min_v,diff,r_dist,g_dist,b_dist;
            double undefined = 0.0;
            max_v = maxrgb(r,g,b);
            min_v = minrgb(r,g,b);
            diff = max_v - min_v;
            v = max_v;
            
            if( max_v != 0 )
                s = diff/max_v;
            else
                s = 0.0;
            
            if( s == 0 )
                h = undefined;
            else {
                r_dist = (max_v - r)/diff;
                g_dist = (max_v - g)/diff;
                b_dist = (max_v - b)/diff;
                if( r == max_v ) 
                    h = b_dist - g_dist;
                else
                    if( g == max_v )
                        h = 2 + r_dist - b_dist;
                    else
                        if( b == max_v )
                            h = 4 + g_dist - r_dist;
                        else
                            ogf_assert_not_reached ;
                h *= 60;
                if( h < 0)
                    h += 360.0;
            }
            *hout = h;
            *sout = s;
            *vout = v;
        }

        static void rgb_to_hsv_convert_image(Image* from_image, Image* to_image) {
            ogf_assert(have_same_size(from_image, to_image)) ;
            ogf_assert(from_image->color_encoding() == Image::RGB) ;
            ogf_assert(to_image->color_encoding() == Image::RGB_FLOAT32) ;
            Memory::byte* from = from_image->base_mem_byte_ptr() ;
            Numeric::float32* to = to_image->base_mem_float32_ptr() ;
            unsigned int nb_pixels = from_image->nb_pixels() ;
            for(unsigned int i=0; i<nb_pixels; i++) {
                double r = double(from[0] / 255.0) ;
                double g = double(from[1] / 255.0) ;
                double b = double(from[2] / 255.0) ;
                double h,s,v ;
                RGB2HSV(r,g,b,&h,&s,&v) ;
                v /= 360.0 ;
                to[0] = float(h) ;
                to[1] = float(s) ;
                to[2] = float(v) ;
                from += 3 ; to += 3 ;
            }
        }
        
//________________________________________________________________________________________

        void IMAGE_API convert_color_space(
            Image* from_image, ColorSpace from_color_space,
            Image* to_image, ColorSpace to_color_space
        ) {
            if(from_color_space == to_color_space) {
                return ;
            }
            if(from_color_space == COLOR_SPACE_RGB && to_color_space == COLOR_SPACE_YIQ) {
                rgb_convert_image(from_image, to_image, RGB_TO_YIQ) ;
                return ;
            }
            if(from_color_space == COLOR_SPACE_RGB && to_color_space == COLOR_SPACE_XYZ) {
                rgb_convert_image(from_image, to_image, RGB_TO_XYZ) ;
                return ;
            }
            if(from_color_space == COLOR_SPACE_RGB && to_color_space == COLOR_SPACE_LUV) {
                rgb_to_luv_convert_image(from_image, to_image) ;
                return ;
            }
            if(from_color_space == COLOR_SPACE_RGB && to_color_space == COLOR_SPACE_HSV) {
                rgb_to_hsv_convert_image(from_image, to_image) ;
                return ;
            }
            // Other conversions are not implemented yet
            ogf_assert_not_reached ;
        }

        void extract_iso_curve(Image* from_image, double value, Image* to_image) {

            ogf_assert(from_image->color_encoding() == Image::GRAY) ;
            ogf_assert(to_image->color_encoding() == Image::GRAY) ;
            ogf_assert(have_same_size(from_image, to_image)) ;
            ogf_assert(from_image->dimension() == 2) ;

            FilterKernel N ;
            for(int j=0; j<from_image->height(); j++) {
                for(int i=0; i<from_image->width(); i++) {
                    get_pixel_neighborhood(from_image, i, j, N) ;
                    bool found_inf = false ;
                    bool found_sup = false ;
                    for(int dj=0; dj<3; dj++) {
                        for(int di=0; di<3; di++) {
                            if(di != 1 && dj != 1) {
                                if(double(N[di][dj]) < value) {
                                    found_inf = true ;
                                }
                                if(double(N[di][dj]) > value) {
                                    found_sup = true ;
                                }
                            }
                        }
                    }
                    *to_image->pixel_base(i,j) = (found_inf && found_sup) ? 255 : 0 ;
                }
            }
        }

        void magnify_2x(Image* from_image, Image* to_image) {
            ogf_assert(from_image->dimension() == 2) ;
            ogf_assert(to_image->dimension() == 2) ;
            ogf_assert(from_image->color_encoding() == to_image->color_encoding()) ;
            ogf_assert(
                from_image->color_encoding() == Image::RGB ||
                from_image->color_encoding() == Image::RGBA ||
                from_image->color_encoding() == Image::GRAY ||
                from_image->color_encoding() == Image::INDEXED
            ) ;
            ogf_assert(to_image->width() == 2 * from_image->width()) ;
            ogf_assert(to_image->height() == 2 * from_image->height()) ;

            int w = from_image->width() ;
            int h = from_image->height() ;
            int bpp = from_image->bytes_per_pixel() ;

            for(int j=0; j<h; j++) {
                for(int i=0; i<w; i++) {
                    for(int k=0; k<bpp; k++) {
                        int p00 = get_pixel_component(from_image, i, j, k) ;
                        int p01 = get_pixel_component(from_image, i, j+1, k) ;
                        int p10 = get_pixel_component(from_image, i+1, j, k) ;
                        int p11 = get_pixel_component(from_image, i+1, j+1, k) ;
                        to_image->pixel_base(2*i, 2*j)[k] = Memory::byte(p00) ;
                        to_image->pixel_base(2*i+1, 2*j)[k] = Memory::byte((p00 + p10)/2) ;
                        to_image->pixel_base(2*i, 2*j+1)[k] = Memory::byte((p00 + p01)/2) ;
                        to_image->pixel_base(2*i+1, 2*j+1)[k] = Memory::byte(
                            (p00 + p01 + p10 + p11)/4
                        ) ;
                    }
                }
            }
        }

        void make_dimensions_multiple_of_four(Image* image, bool center) {
            ogf_assert(image->dimension() == 2) ;
            int w = image->width() ;
            int h = image->height() ;
            while(w & 3) w++ ;
            while(h & 3) h++ ;
            if(w == image->width() && h == image->height()) {
                return ;
            }
            Image_var temp = new Image(image) ;
            int ow = image->width() ;
            int oh = image->height() ;
            image->initialize(temp->color_encoding(), w, h) ;

            if(!center) {
                for(int j=0; j<h; j++) {
                    for(int i=0; i<w; i++) {
                        if(i < ow && j < oh) {
                            for(int k=0; k<image->bytes_per_pixel(); k++) {
                                image->pixel_base(i,j)[k] = temp->pixel_base(i, j)[k] ;
                            } 
                        } else {
                            for(int k=0; k<image->bytes_per_pixel(); k++) {
                                image->pixel_base(i,j)[k] = 255 ;
                            } 
                        }
                    }
                }
                return ;
            }

            int dw = (w - ow) / 2 ;
            int dh = (h - oh) / 2 ;

            for(int j=0; j<h; j++) {
                for(int i=0; i<w; i++) {
                    int oi = i - dw ; 
                    int oj = j - dh ; 
                    ogf_clamp(oi, 0, ow - 1) ;
                    ogf_clamp(oj, 0, oh - 1) ;
                    for(int k=0; k<image->bytes_per_pixel(); k++) {
                        image->pixel_base(i,j)[k] = temp->pixel_base(oi, oj)[k] ;
                    } 
                }
            }
        }

        void make_square(Image* image, bool center) {
            ogf_assert(image->dimension() == 2) ;
            int w = image->width() ;
            int h = image->height() ;
            w = ogf_max(w,h) ;
            h = w ;
            
            if(w == image->width() && h == image->height()) {
                return ;
            }

            Image_var temp = new Image(image) ;

            int ow = image->width() ;
            int oh = image->height() ;

            image->initialize(temp->color_encoding(), w, h) ;

            if(!center) {
                for(int j=0; j<h; j++) {
                    for(int i=0; i<w; i++) {
                        if(i < ow && j < oh) {
                            for(int k=0; k<image->bytes_per_pixel(); k++) {
                                image->pixel_base(i,j)[k] = temp->pixel_base(i, j)[k] ;
                            } 
                        } else {
                            for(int k=0; k<image->bytes_per_pixel(); k++) {
                                image->pixel_base(i,j)[k] = 255 ;
                            } 
                        }
                    }
                }
                return ;
            }

            int dw = (w - ow) / 2 ;
            int dh = (h - oh) / 2 ;

            for(int j=0; j<h; j++) {
                for(int i=0; i<w; i++) {
                    int oi = i - dw ; 
                    int oj = j - dh ; 
                    if(oi < 0 || oj < 0 || oi >= ow || oj >= oh) {
                        for(int k=0; k<image->bytes_per_pixel(); k++) {
                            image->pixel_base(i,j)[k] = 255 ;
                        } 
                    } else {
                        for(int k=0; k<image->bytes_per_pixel(); k++) {
                            image->pixel_base(i,j)[k] = temp->pixel_base(oi, oj)[k] ;
                        } 
                    }
                }
            }
        }
    }
}

