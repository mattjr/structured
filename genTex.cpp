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

static bool have_max_frame_count = false;
static unsigned int max_frame_count;

static bool display_debug_images = true;

static bool compress_textures = false;


static string stereo_calib_file_name;


static std::map<int,std::string> texture_file_names;

static int tex_size=512;

static FILE *conf_ply_file;
extern std::vector<GtsBBox *> bboxes_all;

//
// Parse command line arguments into global variables
//
static bool parse_args( int argc, char *argv[ ] )
{
  bool have_stereo_config_file_name = false;
  
   
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
   
      else
	{
	  cerr << "Error - unknown parameter: " << argv[i] << endl;
	  return false;
	}
    }




  return ( have_stereo_config_file_name);
}


//
// Display information on how to use this program
//
static void print_usage( void )
{
  cout << "USAGE:" << endl;
  cout << "   stereo_feature_finder_test [OPTIONS] <stereo_cfg> " << endl; 
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

  std::map<int,GtsMatrix *> gts_trans_map;
 
  
  auv_data_tools::makedir("mesh");
 
       
  

  FILE *bboxfp = fopen("mesh-agg/bbox.txt","r");
  int count;
  GNode *bboxTree=NULL;
  GSList * bboxes = NULL;
  if(bboxfp){
    char name[255];
    double x1,x2,y1,y2,z1,z2;
    GtsMatrix *mtmp=gts_matrix_identity(NULL);
    int eof0, eof1,eof2;
    while (eof0 != EOF && eof1 != EOF && eof2 != EOF){
      
      eof0 = fscanf(bboxfp,"%d %s %lf %lf %lf %lf %lf %lf" ,&count, name,
	     &x1,&y1,&z1,&x2,&y2,&z2);
      
      for(int i=0; i < 4; i++)
	for(int j=0; j < 4; j++)
	 eof1 = fscanf(bboxfp," %lf",&mtmp[i][j]);
     eof2 = fscanf(bboxfp,"\n");
      
     
      
      texture_file_names[count]=(name);
      GtsBBox *bbox= gts_bbox_new(gts_bbox_class(),NULL,x1,y1,z1,x2,y2,z2);
      bbox->bounded=(void *)count;
      bboxes= g_slist_prepend (bboxes,bbox);
      bboxes_all.push_back(bbox);
      gts_trans_map[count]=gts_matrix_inverse(mtmp);
    
    }
    gts_matrix_destroy(mtmp);
    fclose(bboxfp);

    bboxTree=gts_bb_tree_new(bboxes);
  }

  printf("Loading Surface....\n");

  FILE *surfFP = fopen("mesh-agg/out.ply","r");
  GtsSurface *surf = auv_read_ply(surfFP);
  /*FILE *gFP=fopen("surf2.gts","w");
  gts_surface_write(surf,gFP);
  fclose(gFP);*/
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
  gen_mesh_tex_coord(surf,&calib->left_calib,gts_trans_map,bboxTree,tex_size);
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


  if(conf_ply_file)
    fclose(conf_ply_file);

  // 
  // Clean-up
  //
  //for(int i=0; i < (int)gts_trans.size(); i++)
  // gts_matrix_destroy(gts_trans[i]);
  delete config_file;
  delete calib;
  
   
  return 0;
}
