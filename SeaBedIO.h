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

#ifndef SEABEDIO_H
#define SEABEDIO_H

#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>

//! Base class for exceptions thrown while reading or writing seabed slam
//! output files
class Seabed_SLAM_IO_Exception : public std::runtime_error
{
protected:
   Seabed_SLAM_IO_Exception( const std::string &error );
};

//! Exception thrown when an input file is missing, or an output file cannot
//! be created
class Seabed_SLAM_IO_File_Exception : public Seabed_SLAM_IO_Exception
{
public:
   Seabed_SLAM_IO_File_Exception( const std::string &error );
};

//! Exception thrown when a file cannot be parsed as expected
class Seabed_SLAM_IO_Parse_Exception : public Seabed_SLAM_IO_Exception
{
public:
   Seabed_SLAM_IO_Parse_Exception( const std::string &error );
};

//! Exception thrown when an obsolete file format is encountered
class Seabed_SLAM_IO_Obsolete_File_Exception : public Seabed_SLAM_IO_Exception
{
public:
   Seabed_SLAM_IO_Obsolete_File_Exception( const std::string &error );
};


//----------------------------------------------------------------------------//
//   Stereo Pose Estimate File                                                //
//----------------------------------------------------------------------------//

//! Information about a stereo-rig pose augmented to the SLAM state vector.
class Stereo_Pose
{
public:
   Stereo_Pose( void );

   unsigned int pose_id;
   double pose_time;
   double latitude;
   double longitude;
   std::vector<double> pose_est; // size AUV_NUM_POSE_STATES

   std::string left_image_name;
   std::string right_image_name;
   double altitude; // Altitude above the seafloor (not altitude above the
                    // ellipsoid/geoid from GPS)
   double image_footprint_radius;
   bool likely_overlap;
};


//! Data stored in a stereo pose file
class Stereo_Pose_File
{
public:
   double origin_latitude;  //!< Latitude defining the origin of the X/Y coords
   double origin_longitude; //!< Longitude defining the origin of the X/Y coords

   std::vector<Stereo_Pose> poses;
};

Stereo_Pose_File read_stereo_pose_est_file( const std::string &file_name );

void write_stereo_pose_est_file( const std::string &file_name,
                                 const std::string &custom_header,
                                 const Stereo_Pose_File &data );

#endif // SEABEDIO_H
