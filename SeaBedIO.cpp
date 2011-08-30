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




