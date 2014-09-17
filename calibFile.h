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

#ifndef CAMERACALIBFILE_H
#define CAMERACALIBFILE_H
#include <string>
#include <iostream>
#include <opencv/cv.h>
//! A Class to store calibration parameters for a camera
class CameraCalib
{
public:
   unsigned int width;   //! Image width  (number of pixels along image X-axis)
   unsigned int height;  //! Image height (number of pixels along image Y-axis)

   double ccx;  //!< X-axis coordinate of the central point (in pixel coords)
   double ccy;  //!< Y-axis coordinate of the central point (in pixel coords)
   double fcx;  //!< X-axis focal length (measured in pixel units)
   double fcy;  //!< Y-axis focal length (measured in pixel units)
   double kc1;  //!< Second order radial distortion coefficient
   double kc2;  //!< Fourth order radial distortion coefficient
   double kc3;  //!< Radial distortion coefficient
   double kc4;  //!< Radial distortion coefficient
   double kc5;  //!< Sixth order radial distortion coefficient (often not used)

   double   rotMatr[9];
   double   transVect[3];
};

class DidsonParams
{
public:
    DidsonParams (const std::string &filename);

    int windowStart;
    int windowLength;
};

class StereoCalib
{
public:
    StereoCalib(const std::string &calib_file_name);
    std::vector<CameraCalib> camera_calibs;  //!< Intrinsic parameters for the all cameras
    void load_from_file( const std::string &calib_file_name );
    CameraCalib extract_from_file( std::istream &in_file,
                                                 bool have_3rd_radial_param);
    bool have_3rd_radial_param;
    bool have_rect_params;
    std::string filename;
    double Qloaded[16];
    double rectK[9];
    double left_R[9];
    double right_R[9];


};

#endif // CALIBFILE_H
