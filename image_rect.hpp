/* * structured - Tools for the Generation and Visualization of Large-scale
 * Three-dimensional Reconstructions from Image Data. This software includes
 * source code from other projects, which is subject to different licensing,
 * see COPYING for details. If this project is used for research see COPYING
 * for making the appropriate citations.
 * Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
 *
 * This file is part of structured.
 *
 * structured is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * structured is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with structured.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef IMAGE_RECT_HPP
#define IMAGE_RECT_HPP

#include <cv.h>
#include "opencv2/imgproc/imgproc.hpp"

#include "calibFile.h"


#define NUM_CAMERAS 2
#define LEFT_CAM 0
#define RIGHT_CAM 1

//!
//! A class to store precalculated data to remove the rect from images.
//!
class Rect_Data
{
public:

   //!
   //! Create an Rect_Data object
   //!
   //! NOTE: This function assumes that the third radial rect parameter 
   //!       kc5 is zero. A warning is printed if this is not the case.
   //!
   //! \param calib        Intrinsic parameters of the camera.
   //! \param image        An arbitrary image from the camera.
   //! \param interpolate  Should interpolation be used.
   //!
   Rect_Data( const StereoCalib &calib,
	      const IplImage     *image,


	      bool                undist,bool interpolate=false ) ;
   
   //!
   //! Destroy an RectData object
   //!
   ~Rect_Data( void );          
   
  //Undistort_Data *left_undist_data;
  //Undistort_Data *color_undist_data;

  //Undistort_Data *right_undist_data;
 // CvStereoCamera cv_stereo;
  IplImage *undistL;
  IplImage *undistR;
  IplImage *undistC;
cv::Mat Q;  // 4x4 matrix


  cv::Mat  rectMap[NUM_CAMERAS][2];

   
   bool undist;     //!< Should interpolation be used.
};



//!
//! Rect an image. 
//!
//! This function removes the effect of the radial rects defined by
//! kc1 and kc2, and the tangential distortions defined by kc3 and kc4.
//!
//! This function does not shift the image to center the principal point
//! (ccx and ccy). These still need to be considered along with the focal
//! length when calculating azimuth and elevation angles.
//!
//! NOTE: This function assumes that the third radial distortion parameter 
//!       kc5 is zero. A warning is printed if this is not the case.
//!
//! \param data    Precalculated image rection data.
//!
//! \param source  The distorted source image.
//!
//! \param dest    The destination for the rected image. This image 
//!                must be initialised by the called to have the same 
//!                size and format as the source image.
//!
void rect_image(  Rect_Data &data,
		  const IplImage       *sourceL,
		  IplImage             *&destL ,
		  const IplImage       *sourceR,
		  IplImage             *&destR,
		  const IplImage       *sourceC,
		  IplImage             *&destC); 






#endif //!IMAGE_RECT

