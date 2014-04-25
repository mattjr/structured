//
// structured - Tools for the Generation and Visualization of Large-scale
// Three-dimensional Reconstructions from Image Data. This software includes
// source code from other projects, which is subject to different licensing,
// see COPYING for details. If this project is used for research see COPYING
// for making the appropriate citations.
// Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
//
// This file is part of structured.
//
// structured is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// structured is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with structured.  If not, see <http://www.gnu.org/licenses/>.
//

#include "calibFile.h"
#include <iostream>
#include <fstream>

#include <stdexcept>
#define EPSILON 1e-6

using namespace std;
StereoCalib::StereoCalib(const std::string &calib_file_name):have_3rd_radial_param(false),
have_rect_params(false)
{
    load_from_file(calib_file_name);
}
void StereoCalib::load_from_file( const string &calib_file_name )
{
    filename=calib_file_name;
    // Open the calibration file
    ifstream in_file( calib_file_name.c_str( ) );
    if( !in_file )
    {
        stringstream buf;
        buf << "Unable to load stereo config file: " << calib_file_name;
        throw runtime_error( buf.str() );
    }

    // Count the number of parameters in the calibration file
    int num_param = 0;
    double temp;
    while( in_file >> temp )
        num_param++;
    in_file.clear();
    in_file.seekg( 0 );

    // Figure out if the 3rd (6th order) radial distortion parameter is present
    if( num_param == 55 ){
        have_3rd_radial_param = false;
        have_rect_params = false;
    }
    else if( num_param == 57  ){
        have_3rd_radial_param = true;
        have_rect_params = false;
    } else if( num_param == 100){
        have_3rd_radial_param = true;
        have_rect_params = true;
    }
    else
    {
        stringstream buf;
        buf << "Invalid stereo config file: '" << calib_file_name
            << "'. Number of parameters is " << num_param
            << " expected 55 or 57.";
        throw runtime_error( buf.str() );
    }

    // Load the number of cameras in the calibration file
    unsigned int num_cameras;
    in_file >> num_cameras;
    if( num_cameras != 2 )
    {
        stringstream buf;
        buf << "Invalid stereo config file: '" << calib_file_name
            << "'. Number of cameras is " << num_cameras
            << " expected two.";
        throw runtime_error( buf.str() );
    }


    // Load left camera intrinsics
    CameraCalib pri_calib = extract_from_file( in_file, have_3rd_radial_param );

    /* rotation matrix and transition vector relatively*/

    for( unsigned int i=0 ; i < 9 ; i++ )
        in_file >> pri_calib.rotMatr[i];

    for( unsigned int i=0 ; i < 3 ; i++ )
        in_file >> pri_calib.transVect[i];
    camera_calibs.push_back(pri_calib);

    // Load right camera intrinsics
    CameraCalib sec_calib = extract_from_file( in_file, have_3rd_radial_param );
    for( unsigned int i=0 ; i < 9 ; i++ )
        in_file >> sec_calib.rotMatr[i];

    for( unsigned int i=0 ; i < 3 ; i++ )
        in_file >> sec_calib.transVect[i];

    camera_calibs.push_back(sec_calib);

    // Calculate fundamental matrices
    double temp2[9];
    //calc_fundamental_matrices( );
    if(have_rect_params){
        for( unsigned int i=0 ; i < 9 ; i++ )
            in_file >> left_R[i];

        for( unsigned int i=0 ; i < 9 ; i++ )
            in_file >> right_R[i];

        for( unsigned int i=0 ; i < 9 ; i++ )
            in_file >> rectK[i];

        for( unsigned int i=0 ; i < 16 ; i++ )
	  in_file >> Qloaded[i];
    }

    // Clean-up
    in_file.close( );
}
CameraCalib StereoCalib::extract_from_file( istream &in_file,
                                             bool have_3rd_radial_param)
{
    CameraCalib calib;

    // Load image size
    double w, h;
    in_file >> w >> h;
    calib.width = (int)w;
    calib.height = (int)h;


    //
    // Load camera parameters. Camera matrix format is:
    //    fx  0   cx
    //    0   fy  cy
    //    0   0   1
    //
    double temp[5];
    in_file >> calib.fcx >> temp[0]   >> calib.ccx
            >> temp[1]   >> calib.fcy >> calib.ccy
            >> temp[2]   >> temp[3]   >> temp[4];

    if( fabs(temp[0]-0.0) > EPSILON ||
            fabs(temp[1]-0.0) > EPSILON ||
            fabs(temp[2]-0.0) > EPSILON ||
            fabs(temp[3]-0.0) > EPSILON ||
            fabs(temp[4]-1.0) > EPSILON )
    {
        stringstream buf;
        buf << "Invalid camera calibration matrix";
        throw runtime_error( buf.str() );
    }

    // Load distortion coefficients
    in_file >> calib.kc1
            >> calib.kc2
            >> calib.kc3
            >> calib.kc4;
    if( have_3rd_radial_param )
        in_file >> calib.kc5;
    else
        calib.kc5 = 0;

    return calib;
}
