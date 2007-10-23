//
// stereo_feature_finder_test.cpp
//
// A program to test the feature finding classes by running through a set of
// images loaded from a contents file.
//
// Each line of the contents file should have the following format:
//    <timestamp> <left_image_name> <right_image_name>
//
// Ian Mahon
// 06-03-05
//

#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <sys/time.h>
#include <time.h>

#include "cv.h"
#include "highgui.h"
#include "auv_image_distortion.hpp"
#include "auv_stereo_geometry.hpp"

#include "auv_stereo_corner_finder.hpp"
#include "auv_stereo_ncc_corner_finder.hpp"
#include "auv_stereo_keypoint_finder.hpp"
#include "adt_raw_file.hpp"
#include "OSGExport.h"
#include  <boost/thread/xtime.hpp> 
#include "auv_mesh_utils.hpp"
#include "auv_mesh_io.hpp"
using namespace std;
using namespace libplankton;
using namespace libsnapper;


//
// Command-line arguments
//
static string stereo_config_file_name;
static string contents_file_name;
static string dir_name;

static bool no_simp=false;
static bool use_undistorted_images = false;
static bool have_max_frame_count = false;
static unsigned int max_frame_count;

static bool display_debug_images = true;

static bool compress_textures = false;


static string stereo_calib_file_name;


static std::vector<std::string> texture_file_names;
static FILE *fpp,*fpp2;
static int tex_size=512;

static FILE *conf_ply_file;

//
// Parse command line arguments into global variables
//
static bool parse_args( int argc, char *argv[ ] )
{
  bool have_stereo_config_file_name = false;
  bool have_contents_file_name = false;
   
  int i=1;
  while( i < argc )
    {
      if( strcmp( argv[i], "-r" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  tex_size = atoi( argv[i+1] );
	  i+=2;
	}
      else if( strcmp( argv[i], "-f" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  dir_name=string( argv[i+1]) ;
	  i+=2;
	}
      else if( strcmp( argv[i], "--nosimp" ) == 0 )
	{
	  no_simp=true;
	  i+=1;
	}
      else if( strcmp( argv[i], "--compress-tex" ) == 0 )
	{
	  compress_textures=true;
	  i+=1;
	}
      else if( strcmp( argv[i], "-n" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  have_max_frame_count = true;
	  max_frame_count = atoi( argv[i+1] );
	  i+=2;
	}
      else if( !have_stereo_config_file_name )
	{
	  stereo_config_file_name = argv[i];
	  have_stereo_config_file_name = true;
	  i++;
	}
      else if( !have_contents_file_name )
	{
	  contents_file_name = argv[i];
	  have_contents_file_name = true;
	  i++;
	}
      else
	{
	  cerr << "Error - unknown parameter: " << argv[i] << endl;
	  return false;
	}
    }




  return (have_contents_file_name && have_stereo_config_file_name);
}

bool get_camera_params( Config_File *config_file, Vector &camera_pose )
{

 double tmp_pose[6];

   if( config_file->get_value( "STEREO_POSE_X", tmp_pose[AUV_POSE_INDEX_X]) == 0 )
   {
      return false;
   }
   if( config_file->get_value( "STEREO_POSE_Y", tmp_pose[AUV_POSE_INDEX_Y]) == 0 )
   {
      return false;
   }
   if( config_file->get_value( "STEREO_POSE_Z", tmp_pose[AUV_POSE_INDEX_Z]) == 0 )
   {
      return false;
   }
   if( config_file->get_value( "STEREO_POSE_PHI", tmp_pose[AUV_POSE_INDEX_PHI]) == 0 )
   {
      return false;
   }
   if( config_file->get_value( "STEREO_POSE_THETA", tmp_pose[AUV_POSE_INDEX_THETA]) == 0 )
   {
      return false;
   }
   if( config_file->get_value( "STEREO_POSE_PSI", tmp_pose[AUV_POSE_INDEX_PSI]) == 0 )
   {
      return false;
   }
   
   camera_pose[AUV_POSE_INDEX_X]=tmp_pose[AUV_POSE_INDEX_X];
   camera_pose[AUV_POSE_INDEX_Y]=tmp_pose[AUV_POSE_INDEX_Y];
   camera_pose[AUV_POSE_INDEX_Z]=tmp_pose[AUV_POSE_INDEX_Z];
   camera_pose[AUV_POSE_INDEX_PHI]=tmp_pose[AUV_POSE_INDEX_PHI];
   camera_pose[AUV_POSE_INDEX_THETA]=tmp_pose[AUV_POSE_INDEX_THETA];
   camera_pose[AUV_POSE_INDEX_PSI]=tmp_pose[AUV_POSE_INDEX_PSI];

   return true;
}
//
// Display information on how to use this program
//
static void print_usage( void )
{
  cout << "USAGE:" << endl;
  cout << "   stereo_feature_finder_test [OPTIONS] <stereo_cfg> <contents_file>" << endl; 
  cout << endl;
  cout << "OPTIONS:" << endl;
  cout << "   -r <resize_scale>       Resize the images by a scaling factor." << endl;
 
  cout << "   -n <max_frame_count>    Set the maximum number of frames to be processed." << endl;
 cout << "    --compress-tex           Compress Textures" << endl;
  cout << endl;
}

int main( int argc, char *argv[ ] )
{
  //
  // Parse command line arguments
  //
  if( !parse_args( argc, argv ) )
    {
      print_usage( );
      exit( 1 );
    }


  //
  // Figure out the directory that contains the config file 
  //
  string config_dir_name;
  int slash_pos = stereo_config_file_name.rfind( "/" );
  if( slash_pos != -1 )
    config_dir_name = stereo_config_file_name.substr( 0, slash_pos+1 );


  //
  // Open the config file
  // Ensure the option to display the feature finding debug images is on.
  //
  Config_File *config_file;
  try
    {
      config_file = new Config_File( stereo_config_file_name );
    }
  catch( string error )
    {
      cerr << "ERROR - " << error << endl;
      exit( 1 );
    }
  config_file->set_value( "SKF_SHOW_DEBUG_IMAGES"    , display_debug_images );
  config_file->set_value( "SCF_SHOW_DEBUG_IMAGES"    , display_debug_images );
  config_file->set_value( "NCC_SCF_SHOW_DEBUG_IMAGES", display_debug_images );

  


  //
  // Load the stereo camera calibration 
  //
  Stereo_Calib *calib = NULL;
  bool have_stereo_calib = false;
   
  if( config_file->get_value( "STEREO_CALIB_FILE", stereo_calib_file_name) )
    {
      stereo_calib_file_name = config_dir_name+stereo_calib_file_name;
      try
	{
	  calib = new Stereo_Calib( stereo_calib_file_name );
	}
      catch( string error )
	{
	  cerr << "ERROR - " << error << endl;
	  exit( 1 );
	}
      have_stereo_calib = true;
    }      

  if( use_undistorted_images == true && have_stereo_calib == false )
    {
      cerr << "ERROR - A stereo calibration file is required to undistort "
           << "images." << endl;
      exit( 1 );     
    }
 
  //
  // Open the contents file
  //
  ifstream contents_file( contents_file_name.c_str( ) );
  if( !contents_file )
    {
      cerr << "ERROR - unable to open contents file: " << contents_file_name
           << endl;
      exit( 1 );     
    }


   
  //
  // Figure out the directory that contains the contents file 
  //
   
   
         
  //
  // Run through the data
  //

  
  string left_frame_name;
  string right_frame_name;
  unsigned int stereo_pair_count = 0;
 
  Vector camera_pose(AUV_NUM_POSE_STATES);
  get_camera_params(config_file,camera_pose);

  std::vector<GtsMatrix *> gts_trans;

  
  auv_data_tools::makedir("mesh");
  if(!fpp2)
    fpp2=fopen("mesh/vehpath.txt","w");
  if(!fpp)	
    fpp=fopen("mesh/campath.txt","w");

  while( !have_max_frame_count || stereo_pair_count < max_frame_count )
    {
      //
      // Load the images
      //
      double timestamp;
      int index;
      // printf("Loading images %d\n",stereo_pair_count);
      
      Vector veh_pose(AUV_NUM_POSE_STATES);
      double alt;
      
      if( !(contents_file >> index &&
	    contents_file >> timestamp &&
	    contents_file >> left_frame_name &&
	    contents_file >> right_frame_name &&
	    contents_file >> veh_pose[AUV_POSE_INDEX_X] &&
	    contents_file >> veh_pose[AUV_POSE_INDEX_Y] &&
	    contents_file >> veh_pose[AUV_POSE_INDEX_Z] &&
	    contents_file >> veh_pose[AUV_POSE_INDEX_PHI] &&
	    contents_file >> veh_pose[AUV_POSE_INDEX_THETA] &&
	    contents_file >> veh_pose[AUV_POSE_INDEX_PSI] &&
	    contents_file >> alt))
	{
	  // we've reached the end of the contents file
	  break;
	}      
      
      if(left_frame_name == "DeltaT" || right_frame_name == "DeltaT")
	continue;
      GtsMatrix *m=get_sensor_to_world_trans(veh_pose,camera_pose);
     
      texture_file_names.push_back(left_frame_name);
      GtsMatrix *invM=gts_matrix_inverse(m);
      gts_matrix_destroy(m);

      gts_trans.push_back(invM);
    

      
      fprintf(fpp2,"%f %f %f %f %f %f %f %f %f %f\n",   
	      timestamp,
	      veh_pose[AUV_POSE_INDEX_X],
	      veh_pose[AUV_POSE_INDEX_Y],
	      veh_pose[AUV_POSE_INDEX_Z],
	      veh_pose[AUV_POSE_INDEX_PHI],
	      veh_pose[AUV_POSE_INDEX_THETA],
	      fmod(veh_pose[AUV_POSE_INDEX_PSI],(M_PI)),
	      0.0,0.0,0.0);

      fprintf(fpp,"%f %f %f %f %f %f %f\n",   
	      timestamp,
	      veh_pose[AUV_POSE_INDEX_X],
	      veh_pose[AUV_POSE_INDEX_Y],
	      veh_pose[AUV_POSE_INDEX_Z],
	      veh_pose[AUV_POSE_INDEX_PHI],
	      veh_pose[AUV_POSE_INDEX_THETA],
	      fmod(veh_pose[AUV_POSE_INDEX_PSI],(M_PI))
	      );

      stereo_pair_count++;
      
    }

       
  

  FILE *bboxfp = fopen("mesh-agg/bbox.txt","r");
  int count;
  std::vector<GtsBBox *> bboxes;
  if(bboxfp){
    
    double x1,x2,y1,y2,z1,z2;
    while (fscanf(bboxfp,"%d %lf %lf %lf %lf %lf %lf\n" ,&count, 
		  &x1,&y1,&z1,&x2,&y2,&z2) != EOF){
      GtsBBox *bbox= gts_bbox_new(gts_bbox_class(),NULL,x1,y1,z1,x2,y2,z2);
      bboxes.push_back(bbox);
    }
    fclose(bboxfp);
  }

  printf("Loading Surface....\n");
  FILE *surfFP = fopen("mesh-agg/out.ply","r");
  GtsSurface *surf = auv_read_ply(surfFP);
   
  //fclose(surfFP);
    
  if(!surf){
    printf("Failed to load\n");
    exit(-1);
  }
  printf("Done\n");
  int sets,edgestotal;
  double set_size=2.5;
  int edgeperset=2000;
  double area=gts_surface_area(surf);
  sets=(int)ceil(area/set_size);
  edgestotal=sets*edgeperset;
 
  if(!no_simp){
    printf("Area %.2fm allocating %d edges per %.1fm swaths\nTotal sets: %d, %d edges\n",area,edgeperset,set_size,sets,edgestotal); 
   printf("Coarsen...\n");
    coarsen(surf,edgestotal);
    printf("Done\n");
  }
  printf("Gen texture coordinates\n");
  boost::xtime xt, xt2;
  long time;
  double secs;
 
  boost::xtime_get(&xt, boost::TIME_UTC);
  gen_mesh_tex_coord(surf,&calib->left_calib,gts_trans,bboxes,tex_size);
  boost::xtime_get(&xt2, boost::TIME_UTC);
  time = (xt2.sec*1000000000+xt2.nsec - xt.sec*1000000000 - xt.nsec) / 1000000;
  secs=time/1000.0;
  printf("Done Took %.2f secs\n",secs);
  
  printf("Converting to model for export\n");
  OSGExporter *osgExp=new OSGExporter(dir_name,false,compress_textures,tex_size);    
  boost::xtime_get(&xt, boost::TIME_UTC);
  osgExp->convertModelOSG(surf,texture_file_names,"mesh/blended.ive");
  boost::xtime_get(&xt2, boost::TIME_UTC);
  time = (xt2.sec*1000000000+xt2.nsec - xt.sec*1000000000 - xt.nsec) / 1000000;
  secs=time/1000.0;
  printf("Done Took %.2f secs\n",secs);

  fclose(fpp);
  fclose(fpp2);
  if(conf_ply_file)
    fclose(conf_ply_file);

  // 
  // Clean-up
  //
  for(int i=0; i < (int)gts_trans.size(); i++)
     gts_matrix_destroy(gts_trans[i]);
  delete config_file;
  delete calib;
  
   
  return 0;
}
