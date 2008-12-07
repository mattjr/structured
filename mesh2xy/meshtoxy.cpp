//
// prepare_lat_long_xy.cpp
//
// A program to process a vehicle_pose_est.data file 
// and generate a lat / long xy of the vehicle position for import
// into arc_gis etc...

//
// Duncan Mercer,
// Aug 2008


#include <iostream>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <sstream>

#include <auv_config_file.hpp>

#include <GeographicConversions/ufRedfearn.h>

#include <cv.h>
#include <highgui.h>
#include <TriMesh.h>
using namespace std;
using namespace libplankton;

//
// Global variables for command-line arguments
//
static string localiser_config_file_name;
static string mesh_file_name;
static string xy_name;

double first_timestamp;

//****************************************************************


static bool parse_args( int argc, char *argv[ ] )
{
   bool have_localiser_config_file = false;
   bool have_mesh_file = false;
   bool have_xy_name = false;
  
   int i=1;
   while( i < argc )
   {
      if( !have_localiser_config_file )
      {
         localiser_config_file_name = argv[i];
         have_localiser_config_file = true;
         i++;
      }
      else if( !have_mesh_file )
      {
         mesh_file_name = argv[i];
         have_mesh_file = true;
         i++;
      }
      else if( !have_xy_name )
      {
         xy_name = argv[i];
         have_xy_name = true;
         i++;
      }
      else
      {
         cerr << "Error - unknown parameter: " << argv[i] << endl;
         return false;
      }
   }
   
   return(  have_localiser_config_file &&
            have_mesh_file &&
            have_xy_name);

}

//****************************************************************


static void print_usage( void )
{
   cout << "USAGE: meshtoxy <localiser config> <mesh_file> <xy_name>" << endl; 
   cout << endl;
}

//****************************************************************


int main( int argc, char *argv[ ] )
{
   double lat_origin, long_origin;

   int index;
   double timestamp;
   double x, y, z, roll, pitch, heading;
   double gridConvergence, pointScale;
   string zone;

   // IMPORTANT:- specify the ellipsoid and projection for GeographicConversions
   UF::GeographicConversions::Redfearn gpsConversion("WGS84","UTM");
   double easting_orig, northing_orig, easting, northing;

   //
   // Parse command line arguments
   //
   if( !parse_args( argc, argv ) )
   {
      print_usage( );
      exit( 1 );
   }


   Config_File *config_file;

   try
   {
      config_file = new Config_File( localiser_config_file_name );
   }
   catch( string error )
   {
      cerr << "ERROR - " << error << endl;
      exit( 1 );
   }

  
   if( !config_file->get_value( "LATITUDE", lat_origin ) )
   {
      cerr << "ERROR - the localiser config file doesn't define option"
           << " LATITUDE. This was previously LATITUDE"
           << " in the stereo config file." << endl;
      exit(1);
   }
   if( !config_file->get_value( "LONGITUDE", long_origin ) )
   {
      cerr << "ERROR - the localiser config file doesn't define option"
           << " LONGITUDE. This was previously LONGITUDE"
           << " in the stereo config file." << endl;
      exit(1);
   }


   // Get the mission origin in UTM grid co-ords.
   gpsConversion.GetGridCoordinates( lat_origin, long_origin,
                                     zone,
                                     easting_orig, northing_orig, 
                                     gridConvergence, pointScale );

   TriMesh *mesh = TriMesh::read(mesh_file_name.c_str());

 

   // 
   // Open an output file, and truncate it, and add header line
   string output_name = xy_name;
   ofstream output_file(output_name.c_str(), ios_base::trunc | ios_base::out);

   output_file << fixed << setprecision(7);


   // Run through all the data and convert from grid to lat / long...
   for(int i=0; i < (int) mesh->vertices.size(); i++){

     x=mesh->vertices[i][0];
     y=mesh->vertices[i][1];

     northing = northing_orig + x;
     easting  = easting_orig + y;

     double pos_lat, pos_long;
     gpsConversion.GetGeographicCoordinates(zone, easting, northing, 
					    pos_lat, pos_long, 
					    gridConvergence, 
					    pointScale);

     output_file << setprecision(7) << pos_lat << pos_long <<endl;
   }

   
   output_file.close();


}
