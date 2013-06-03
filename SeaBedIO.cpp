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

#include "SeaBedIO.h"
#include "TexPyrAtlas.h"
#include "stereo_cells.hpp"
#include <sstream>


using namespace std;

// Lines in the data files beginning with this character are ignored
#define SEABED_SLAM_FILE_COMMENT_CHAR '%'

// A string used in the first line of a file specifying file type and version
#define VERSION_STRING "VERSION"

// Precision for output data
#define PRECISION 16
#define NO_ALTITUDE 0
//----------------------------------------------------------------------------//
//   Generic File Reading (Private)                                           //
//----------------------------------------------------------------------------//

static void open_input_stream( const string &file_name, ifstream &in_file )
{
   in_file.open( file_name.c_str() );
   if( !in_file )
   {
      stringstream ss;
      ss << "Unable to open input data file '" << file_name << "'";
      throw Seabed_SLAM_IO_File_Exception( ss.str() );
   }
}

//
// The format for a named value is a name, followed by a space,
// followed by the value.
//
template<class T>
void parse_named_value( istream &in,
                        const string &name,
                        T &data,
                        const string &comment )
{
   string current;
   if( !(in>>current) || current.compare(name) != 0  ||
       !(in>>data) )
   {
      stringstream message;
      message << "Unable to parse " << comment;
      throw Seabed_SLAM_IO_Parse_Exception( message.str() );
   }
}



// Skip over lines starting with the comment character
static void skip_comments( istream &in_file )
{
   bool done = false;
   while( !done )
   {
      int pos = in_file.tellg();

      char next_char;
      if( !(in_file >> next_char) )
         return;

      if( next_char == SEABED_SLAM_FILE_COMMENT_CHAR )
      {
         string comment_line;
         getline( in_file, comment_line );
      }
      else
      {
         in_file.seekg( pos );
         done = true;
      }
   }
}


// Try to read the file version at the beginning of the file.
// If it doesn't exist, we assume it is version 1
static int read_file_version(  istream &in_file )
{
   // Try to read the file type and version
   char c;
   string type_str;
   string ver_str;
   int version;
   if( !(in_file>>c)        || c!=SEABED_SLAM_FILE_COMMENT_CHAR ||
       !(in_file>>type_str) ||
       !(in_file>>ver_str)  || ver_str.compare(VERSION_STRING)!=0 ||
       !(in_file>>version) )
   {
      // File type and version info not present. The file must be version 1
      version = 1;
   }
   else
   {

      // Check that the version number is sensible
      int max_version = 2;
      if( version > max_version )
      {
         stringstream ss;
         ss << "Invalid file version "<< version << ". Current version is "
            << max_version;
         throw Seabed_SLAM_IO_Parse_Exception( ss.str() );
      }
   }

   // Return to the start of the file and clear any error state flags
   in_file.seekg( 0, ios::beg );
   in_file.clear();

   return version;
}


//
// Try to read a value, and throw an exception if it fails
//
// FIXME: Extend this to do version checking?
//
template<class T>
static void parse( istream &in, const string &name, T &value )
{
   if( !(in>>value) )
   {
      string err = "Unable to parse " + name;
      throw Seabed_SLAM_IO_Parse_Exception( err );
   }
}

template<class T>
void read_data( const string &file_name, unsigned int version,
                istream &in_file, vector<T> &data )
{
   try
   {
      while( !in_file.eof() )
      {
         skip_comments( in_file );
         if( in_file.eof() )
            break;

         pair<unsigned int, T> new_data;
         new_data.first = version;
         if( !(in_file >> new_data ) )
         {
            stringstream ss;
            ss << "Error parsing file '" << file_name << "'";
            throw Seabed_SLAM_IO_Parse_Exception( ss.str() );
         }
         data.push_back( new_data.second );
      }
   }
   catch( Seabed_SLAM_IO_Parse_Exception &e )
   {
      stringstream ss;
      ss << e.what() << " in file '" << file_name << "'";
      throw Seabed_SLAM_IO_Parse_Exception( ss.str() );
   }
}

// Generic file reading
template<class T>
static vector<T> read_file( const string &file_name
                              )
{
   // Open file
   ifstream in_file;
   open_input_stream( file_name, in_file );

   // Get file format version number
   int version = read_file_version( in_file );

   // Read the data
   vector<T> data;
   read_data( file_name, version, in_file, data );

   return data;
}
Stereo_Pose::Stereo_Pose( void )
   : pose_est( NUM_POSE_STATES ),
     altitude( NO_ALTITUDE )
{

}

// Input operator: first in 'pair' is the file version number, second is the
// datastructure to be initialised.
static istream &operator>>( istream &in, pair<unsigned int,Stereo_Pose> &pair )
{
   unsigned int version = pair.first;
   Stereo_Pose &data = pair.second;

   parse( in, "pose id"  , data.pose_id   );
   parse( in, "pose time", data.pose_time );
   if( version>=2 )
   {
      parse( in, "latitude" , data.latitude  );
      parse( in, "longitude", data.longitude );
   }
   parse( in, "X"                     , data.pose_est[POSE_INDEX_X]     );
   parse( in, "Y"                     , data.pose_est[POSE_INDEX_Y]     );
   parse( in, "Z"                     , data.pose_est[POSE_INDEX_Z]     );
   parse( in, "roll"                  , data.pose_est[POSE_INDEX_PHI]   );
   parse( in, "pitch"                 , data.pose_est[POSE_INDEX_THETA] );
   parse( in, "yaw"                   , data.pose_est[POSE_INDEX_PSI]   );
   parse( in, "left image name"       , data.left_image_name                );
   parse( in, "right image name"      , data.right_image_name               );
   parse( in, "altitude"              , data.altitude                       );
   parse( in, "image footprint radius", data.image_footprint_radius         );
   parse( in, "likely overlap flag"   , data.likely_overlap                 );

   return in;
}


Stereo_Pose_File read_stereo_pose_est_file( const string &file_name )
{
   Stereo_Pose_File data;

   // Open file
   ifstream in_file;
   open_input_stream( file_name, in_file );

   // Get file format version number
   int version = read_file_version(  in_file );

   // Check file version
   if( version < 2 )
   {
      stringstream ss;
      ss << "Obsolete stereo pose file version " << version << "." << endl
         << endl
         << "This file version contains no geographic (latitude/longitude) "
         << "coordinates, and the inappropriate map projection used by "
         << "seabed_slam to obtain the X/Y coordinates may have introduced "
         << "significant errors in the navigation estimates." << endl
         << endl
         << "If you are trying to reproduce previous results created using "
         << "this file, the format can be updated to the latest version "
         << "including geographic coordinates using the program "
         << "update_vehicle_pose_file. However, the errors in the navigation "
         << "estimates caused by the poor map projection will remain." << endl
         << endl
         << "Re-run seabed_slam to produce improved navigation estimates."
         << endl;

      throw Seabed_SLAM_IO_Obsolete_File_Exception( ss.str() );
   }

   skip_comments( in_file );

   // Read origin latitude and longitude
   parse_named_value( in_file, "ORIGIN_LATITUDE", data.origin_latitude,
                      "Origin latitude" );
   parse_named_value( in_file, "ORIGIN_LONGITUDE", data.origin_longitude,
                      "Origin longitude" );

   // Read the data
   read_data( file_name, version, in_file, data.poses );

   return data;
}
//----------------------------------------------------------------------------//
//   Generic File Writing (Private)                                           //
//----------------------------------------------------------------------------//

static void open_output_stream( const string &file_name, ofstream &out_file )
{
   out_file.open( file_name.c_str() );
   if( !out_file )
   {
      stringstream ss;
      ss << "Unable to open output file '" << file_name << "'";
      throw Seabed_SLAM_IO_File_Exception( ss.str() );
   }

   // Configure output stream
   out_file << fixed << setprecision(PRECISION) << endl;
}


// Write the file header, adding a comment char at the start of each line
static void write_header( const string &header, ostream &out )
{
   stringstream ss;
   ss << header;

   string line;
   while( getline(ss,line) )
      out << SEABED_SLAM_FILE_COMMENT_CHAR << " " << line << endl;
}


static void write_file_version_info(
                                     ostream &out )
{
   out << SEABED_SLAM_FILE_COMMENT_CHAR << " "
       << "STEREO_POSE_FILE"<< " "
       << VERSION_STRING << " "
       << 2 << endl;
}

//----------------------------------------------------------------------------//
//   Stereo Pose Estimate File                                                //
//----------------------------------------------------------------------------//
//
// Version 1: Initial version
// Version 2: Added latitude and longitude. Local grid coordinate calculation
//            was changed to using a local Transverse Mercator projection
//            instead of UTM, which may produced large errors when the vehicle
//            is operating a long way from the central meridian. Files with
//            older version should only be used with caution and understanding
//            of this problem.
//

#define STEREO_POSE_FILE_INFO \
"Each line of this file describes the pose of the stereo-vision system relative\n\
to the local navigation frame at the time a pair of stereo images were\n\
acquired. The reference frame of the stereo-vision system is defined to be\n\
coincident with the left camera.\n\
\n\
The X and Y coordinates are produced using a local transverse Mercator \n\
projection using the WGS84 ellipsoid and a central meridian at the origin\n\
latitude. You will probably want to use the provided latitude and longitude to\n\
produce coordinates in what map projection you require.\n\
\n\
The first two lines of the data contain the latitude and longitude of the\n\
origin.\n\
\n\
Each line contains the following items describing the pose of the stereo rig:\n\
\n\
1) Pose identifier                   - integer value\n\
2) Timestamp                         - in seconds\n\
3) Latitude                          - in degrees\n\
4) Longitude                         - in degrees\n\
5) X position (North)                - in meters, relative to local nav frame\n\
6) Y position (East)                 - in meters, relative to local nav frame\n\
7) Z position (Depth)                - in meters, relative to local nav frame\n\
8) X-axis Euler angle                - in radians, relative to local nav frame\n\
9) Y-axis Euler angle                - in radians, relative to local nav frame\n\
10) Z-axis Euler angle               - in radians, relative to local nav frame\n\
11) Left image name\n\
12) Right image name\n\
13) Vehicle altitude                   - in meters\n\
14) Approx. bounding image radius      - in meters\n\
15) Likely trajectory cross-over point - 1 for true, 0 for false\n\
\n\
Data items 14 and 15 are used within our 3D mesh building software, and can\n\
safely be ignored in other applications.\n\
\n\
Note: The Euler angles correspond to the orientation of the stereo-rig, and\n\
do not correspond to the roll, pitch and heading of the vehicle. The stereo-\n\
frame is defined such that the positive Z-axis is along the principal ray of\n\
the camera (in the direction the camera is pointed), and the X and Y axes are\n\
aligned with the image axes. The positive X axis is pointing towards the\n\
right of the image, while the positive Y axis points to the bottom of the\n\
image. The Euler angles specify the sequence of rotations in XYZ order, that \n\
align the navigation frame axes (North, East, Down) with the stereo frame."





static ostream &operator<<( ostream &out, const Stereo_Pose &data )
{
   assert( data.pose_est.size() == NUM_POSE_STATES );

   out << data.pose_id                        << " \t"
       << data.pose_time                      << " \t"
       << data.latitude                       << " \t"
       << data.longitude                      << " \t"
       << data.pose_est[POSE_INDEX_X]     << " \t"
       << data.pose_est[POSE_INDEX_Y]     << " \t"
       << data.pose_est[POSE_INDEX_Z]     << " \t"
       << data.pose_est[POSE_INDEX_PHI]   << " \t"
       << data.pose_est[POSE_INDEX_THETA] << " \t"
       << data.pose_est[POSE_INDEX_PSI]   << " \t"
       << data.left_image_name                << " \t"
       << data.right_image_name               << " \t"
       << data.altitude                       << " \t"
       << data.image_footprint_radius         << " \t"
       << data.likely_overlap;

   return out;
}

void write_stereo_pose_est_file( const string &file_name,
                                 const string &custom_header,
                                 const Stereo_Pose_File &data )
{
   // Open file and configure output stream
   ofstream out_file;
   open_output_stream( file_name, out_file );

   // Write the file version info
   write_file_version_info( out_file );

   // Write the file header
   write_header( "\n", out_file );
   write_header( custom_header, out_file );
   write_header( "\n", out_file );
   write_header( STEREO_POSE_FILE_INFO, out_file );
   write_header( "\n", out_file );

   // Write the data
   out_file << "ORIGIN_LATITUDE  " << data.origin_latitude  << endl;
   out_file << "ORIGIN_LONGITUDE " << data.origin_longitude << endl;

   for( unsigned int i=0; i<data.poses.size(); i++ )
      out_file << data.poses[i] << endl;
}


//----------------------------------------------------------------------------//
//   Exceptions                                                               //
//----------------------------------------------------------------------------//

Seabed_SLAM_IO_Exception::Seabed_SLAM_IO_Exception( const string &error )
   : runtime_error( error ) { }

Seabed_SLAM_IO_File_Exception::Seabed_SLAM_IO_File_Exception( const string &error )
   : Seabed_SLAM_IO_Exception( error ) { }

Seabed_SLAM_IO_Parse_Exception::Seabed_SLAM_IO_Parse_Exception( const string &error )
   : Seabed_SLAM_IO_Exception( error ) { }

Seabed_SLAM_IO_Obsolete_File_Exception::Seabed_SLAM_IO_Obsolete_File_Exception( const string &error )
   : Seabed_SLAM_IO_Exception( error ) { }




