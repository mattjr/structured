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
#include <sys/types.h>
#include <sys/stat.h>

#include <sys/time.h>
#include <time.h>

#include "cv.h"
#include "highgui.h"
#include "ulapack/eig.hpp"
#include "auv_image_distortion.hpp"
#include "auv_stereo_geometry.hpp"
#include "adt_raw_file.hpp"
#include "auv_mesh_utils.hpp"
#include "auv_stereo_corner_finder.hpp"
#include "auv_stereo_ncc_corner_finder.hpp"
#include "auv_stereo_keypoint_finder.hpp"
#include "auv_mesh.hpp"
#include "auv_mesh_io.hpp"
//#define WITH_LOGGING
#include "auv_concurrency.hpp"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "OSGExport.h"
using namespace std;
using namespace libplankton;
using namespace ulapack;
using namespace libsnapper;
static int meshNum;
static double start_time = 0.0;
static double stop_time = numeric_limits<double>::max();
//
// Command-line arguments
//
static int vrip_split=250;
static FILE *pts_cov_fp;
static string stereo_config_file_name;
static string contents_file_name;
static string dir_name;
static bool output_uv_file=false;
static bool use_undistorted_images = false;
static bool pause_after_each_frame = false;
static double image_scale = 1.0;
static int max_feature_count = 5000;
static double eps=1e-1;
static double subvol=25.0;
static bool have_max_frame_count = false;
static unsigned int max_frame_count=INT_MAX;
static bool display_debug_images = true;
static bool output_pts_cov=false;
static bool use_sift_features = false;
static bool use_surf_features = false;
static bool use_ncc = false;
static int skip_counter=0;
static int num_skip=0;
static vector<string> mb_ply_filenames;
static bool have_mb_ply=false;
static bool have_cov_file=false;
static string stereo_calib_file_name;
static bool use_rect_images=false;
static FILE *uv_fp;
static ofstream file_name_list;
static string base_dir;
static double feature_depth_guess = AUV_NO_Z_GUESS;
static int num_threads=1;
static FILE *fpp,*fpp2;
static bool use_dense_feature=false;
//StereoMatching* stereo;
//StereoImage simage;
static bool gen_mb_ply=false;
static bool output_ply_and_conf =true;
static FILE *conf_ply_file;
static bool output_3ds=false;
static char cov_file_name[255];

static string basepath;
static bool single_run=false;
static int single_run_start=0;
static int single_run_stop=0;

enum {END_FILE,NO_ADD,ADD_IMG};
char subvoldir[255];
static string deltaT_config_name;
static string deltaT_dir;
static bool hardware_compress=true;
void print_uv_3dpts( list<Feature*>          &features,
		     list<Stereo_Feature_Estimate> &feature_positions,
		     unsigned int                   left_frame_id,
		     unsigned int                   right_frame_id,
		     double timestamp, string leftname,string rightname);
void get_cov_mat(ifstream *cov_file,Matrix &mat){
 
  for(int i=0; i < 0; i ++)
    for(int j=0; j < 0; j ++)
      (*cov_file) >>   mat(i,j);

}

gboolean image_count_verbose ( guint number, guint total)
{
 

  if(single_run)
    return true;

  if (timer == NULL) {
    boost::call_once(&timer_init, once);
    nmax = nold = number;
  }

  if (number != nold && number % 1 == 0 ){// && number % 1 == 0 ){//&& number < nmax && nmax > total) {
    gdouble total_elapsed = g_timer_elapsed (total_timer, NULL);
    gdouble remaining;
    gdouble hours, mins, secs;
    gdouble hours1, mins1, secs1;

    g_timer_stop (timer);

    hours = floor (total_elapsed/3600.);
    mins = floor ((total_elapsed - 3600.*hours)/60.);
    secs = floor (total_elapsed - 3600.*hours - 60.*mins);

    remaining = ((total_elapsed/(gdouble)number) *((gdouble)total-number));
    hours1 = floor (remaining/3600.);
    mins1 = floor ((remaining - 3600.*hours1)/60.);
    secs1 = floor (remaining - 3600.*hours1 - 60.*mins1);

    fprintf (stderr, 
	     "\rImage: %6u/%6u %3.0f%% "// %2.2f s/img "
	     "Elapsed: %02.0f:%02.0f:%02.0f "
	     "Remaining: %02.0f:%02.0f:%02.0f",
	     number, total,
	     100.*( number)/( total),
	     //g_timer_elapsed (timer, NULL)/ (number - nold  ),
	     hours, mins, secs,
	     hours1, mins1, secs1);
    fflush (stderr);

    nold = number;
    g_timer_start (timer);
  }
  if (number == total) {
    boost::call_once(&timer_destroy, once2);
    printf("\n");
    return TRUE;
  }
  return FALSE;
}
void print_uv_3dpts( list<Feature*>          &features,
		     list<Stereo_Feature_Estimate> &feature_positions,
		     unsigned int                   left_frame_id,
		     unsigned int                   right_frame_id,
		     double timestamp, string leftname,string rightname){
  list<Stereo_Feature_Estimate>::iterator litr;
  list<Feature *>::iterator fitr;
  fprintf(uv_fp,"%f %s %s %d %d\n",timestamp, leftname.c_str(),
	  rightname.c_str(),(int)feature_positions.size(),(int)features.size());
  
  for( litr  = feature_positions.begin( ),
	 fitr = features.begin();
       litr != feature_positions.end( ) || fitr != features.end(); 
       litr++,fitr++ ){
    const Feature_Obs *p1_left_obs;
    p1_left_obs = (*fitr)->get_observation( left_frame_id );
    const Feature_Obs *p1_right_obs;
    p1_right_obs = (*fitr)->get_observation( right_frame_id );
    fprintf(uv_fp,"%f %f %f %f %f %f %f\n",litr->x[0],litr->x[1],
	    litr->x[2], p1_left_obs->u, 
	    p1_left_obs->v,
	    p1_right_obs->u, 
	    p1_right_obs->v);
    
  } 
  fprintf(uv_fp,"\n");
}

bool get_camera_params( Config_File *config_file, Vector &camera_pose ){

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
// Parse command line arguments into global variables
//
static bool parse_args( int argc, char *argv[ ] )
{
  bool have_stereo_config_file_name = true;
  bool have_contents_file_name = true;
  bool have_base_dir=false;

  stereo_config_file_name = "stereo.cfg";
  contents_file_name = "pose_file.data";
  dir_name = "img/";
  strcpy(subvoldir,"mesh-agg/");
  int i=0;
  while( i < argc )
    {
      if( strcmp( argv[i], "-r" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  image_scale = strtod( argv[i+1], NULL );
	  i+=2;
	}
      else if( strcmp( argv[i], "-m" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  max_feature_count = atoi( argv[i+1] );
	  i+=2;
	}  
      else if( strcmp( argv[i], "-f" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  dir_name=string( argv[i+1]) ;
	  i+=2;
	}
      else if( strcmp( argv[i], "--mbfile" ) == 0 )
	{
	
	  if( i == argc-1 ) return false;
	
	  have_mb_ply=true;
	  for(i++; i < argc && argv[i][0] != '-'; i++)
	    mb_ply_filenames.push_back(string( argv[i])) ;
	
	}
      else if( strcmp( argv[i], "--genmb" ) == 0 )
	{
	  if( i == argc-2 ) return false;	
	  gen_mb_ply=true;
	  have_mb_ply=true;
	  mb_ply_filenames.push_back(string("mb.ply")) ;
	  deltaT_config_name=string( argv[i+1]) ;
	  deltaT_dir=string( argv[i+2]) ;
	  i+=3;
	}
      else if( strcmp( argv[i], "-z" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  feature_depth_guess = atof( argv[i+1] );
	  i+=2;
	}
      else if( strcmp( argv[i], "-s" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  num_skip = atoi( argv[i+1] );
	  skip_counter=num_skip;
	  i+=2;
	}
      else if( strcmp( argv[i], "--single-run" ) == 0 )
	{
	  if( i == argc-2 ) return false;
	  single_run_start = atoi( argv[i+1] );
	  single_run_stop = atoi( argv[i+2] );
	  i+=3;
	  single_run=true;
	  display_debug_images = false;
	}
      else if( strcmp( argv[i], "-t" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  num_threads = atoi( argv[i+1] );
	  i+=2;
	}
      else if( strcmp( argv[i], "--cov" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  strcpy(cov_file_name, argv[i+1] );
	  have_cov_file=true;
	  i+=2;
	}  
      else if( strcmp( argv[i], "-n" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  have_max_frame_count = true;
	  max_frame_count = atoi( argv[i+1] );
	  i+=2;
	}
      else if( strcmp( argv[i], "-u" ) == 0 )
	{
	  use_undistorted_images = true;
	  i+=1;
	}
      else if( strcmp( argv[i], "--ptscov" ) == 0 )
	{
	  output_pts_cov = true;
	  i+=1;
	}
      else if( strcmp( argv[i], "-y" ) == 0 )
	{
	  use_rect_images = true;
	  i+=1;
	}
      else if( strcmp( argv[i], "-d" ) == 0 )
	{
	  display_debug_images = false;
	  i+=1;
	}
      else if( strcmp( argv[i], "-p" ) == 0 )
	{
	  pause_after_each_frame = true;
	  i+=1;
	}
      else if( strcmp( argv[i], "-c" ) == 0 )
	{
	  use_ncc = true;
	  i+=1;
	}
      else if( strcmp( argv[i], "--split" ) == 0 )
	{  
	  if( i == argc-1 ) return false;
	  vrip_split = atoi( argv[i+1] );
	  i+=2;
	}
      else if( strcmp( argv[i], "--subvoldir" ) == 0 )
	{  
	  if( i == argc-1 ) return false;
	  strcpy(subvoldir , argv[i+1] );
	  i+=2;
	}
      else if( strcmp( argv[i], "--dicevol" ) == 0 )
	{  
	  if( i == argc-1 ) return false;
	  subvol = atof( argv[i+1] );
	  i+=2;
	}
      else if( strcmp( argv[i], "--no-hardware-compress" ) == 0 )
	{  
	  hardware_compress=false;
	  i+=1;
	}
      else if( strcmp( argv[i], "--uv" ) == 0 )
	{
	  output_uv_file = true;
	  i+=1;
	}
      else if( strcmp( argv[i], "--sift" ) == 0 )
	{
	  use_sift_features = true;
	  i+=1;
	}
      else if( strcmp( argv[i], "--dense-features" ) == 0 )
	{
	  use_dense_feature = true;
	  i+=1;
	}
      else if( strcmp( argv[i], "--surf" ) == 0 )
	{
	  use_surf_features = true;
	  i+=1;
	}
      else if( strcmp( argv[i], "--start" ) == 0 )
	{	
	  if( i == argc-1 ) return false;
	  start_time = strtod( argv[i+1], NULL );
	  i+=2;
	}
      else if( strcmp( argv[i], "--stop" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  stop_time = strtod( argv[i+1], NULL );
	  i+=2;
	}
      else if( strcmp( argv[i], "--3ds" ) == 0 )
	{
	  output_3ds=true;
	  i+=1;
	}
      else if( strcmp( argv[i], "--no-hardware-compress" ) == 0 )
	{
	  output_3ds=true;
	  i+=1;
	}
      else if(strcmp( argv[i], "--stereo-config" ) == 0)
	{
	  if( i == argc-1 ) return false;
	  stereo_config_file_name = argv[i+1];

	  i+=2;
	}
      else if(strcmp( argv[i], "--contents-file" ) == 0)
	{
	  if( i == argc-1 ) return false;
	  contents_file_name = argv[i+1];

	  i+=2;
	}
      else if(!have_base_dir)
	{
	  if( i == argc-1 ) return false;
	  base_dir = argv[i+1];
	  have_base_dir = true;
	  i+=2;
	}
      else
	{
	  cerr << "Error - unknown parameter: " << argv[i] << endl;
	  return false;
	}
    }
  if(!output_3ds && !output_ply_and_conf){
    cerr << "Must do ply or 3ds output\n";
    return false;
  }

  if(have_base_dir){
    stereo_config_file_name= base_dir+string("/")+stereo_config_file_name;
    contents_file_name= base_dir+string("/")+contents_file_name;
    dir_name= base_dir+string("/")+dir_name;
  }

  struct stat statinfo;
  if(stat(stereo_config_file_name.c_str(), &statinfo) < 0 ){
    have_stereo_config_file_name = false;
    cerr << "Don't have stereo config " << stereo_config_file_name << endl;

  }
  if(stat(contents_file_name.c_str(), &statinfo) < 0 ){
    have_contents_file_name = false;
      cerr << "Don't have contents " << contents_file_name << endl;
  }
#ifndef HAVE_LIBKEYPOINT
  if( use_sift_features || use_surf_features )
    {
      cerr << "ERROR - libsnapper was compiled without sift support" << endl;
      return false;
    }
#endif
  
  return (have_contents_file_name && have_stereo_config_file_name);
}
   
//
// Display information on how to use this program
// 
static void print_usage( void )
{
  cout << "USAGE:" << endl;
  cout << "   threadedStereo [OPTIONS] <basedir>" << endl; 
  cout << "   <basedir> allows you to choose one directory under which the program "<<endl;

  cout << "   will look for stereo.cfg pose_file.data and dir img for images"<< endl;
cout << "     I suggest creating symlinks to those files allowing for varible configuration."<< endl;
  cout << "OPTIONS:" << endl;
  cout << "   -r <texture size>       Final texture output size." << endl;
  cout << "   -m <max_feature_count>  Set the maximum number of features to be found." << endl;
  cout << "   -n <max_frame_count>    Set the maximum number of frames to be processed." << endl;
  cout << "   -z <feature_depth>      Set an estimate for the feature depth relative to cameras." << endl;
  cout << "   -t <num_threads>       Number of threads to run" << endl;
  cout << "   --mbfile       Multibeam ply mesh" << endl;
  cout << "   --cov <file>               Input covar file." << endl; 
  cout << "   -c                      Use the normalised cross correlation feature descriptor" << endl;
  cout << "   --sift                  Find SIFT features." << endl;
  cout << "   --surf                  Find SURF features." << endl;
  cout << "   -d                      Do not display debug images." << endl;
  cout << "   --confply               Output confply file." << endl;
  cout << "   --genmb               Generate MB mesh" << endl;
  cout << "   --split <num>         Split individual meshes passed to vrip at num" << endl;
  cout << "   --dicevol <vol>         Dice mesh to subvolume pieces of <vol> size" << endl;

  cout << "   -p                      Pause after each frame." << endl;
  cout << "   --uv                    Output UV File." << endl;
  cout << "   --3ds                   Output 3ds Files." << endl;
  cout << "   --dense-features        Dense features ." << endl;
  cout << "   --ptscov                Output pts and cov ." << endl;
  cout << "   --stereo-config              Specify diffrent stereo config" << endl;
 cout << "   --contents-file         Specify diffrent contents file ." << endl;

  cout << endl;
}


void update_bbox (GtsPoint * p, GtsBBox * bb)
{
  if (p->x < bb->x1) bb->x1 = p->x;
  if (p->y < bb->y1) bb->y1 = p->y;
  if (p->z < bb->z1) bb->z1 = p->z;
  if (p->x > bb->x2) bb->x2 = p->x;
  if (p->y > bb->y2) bb->y2 = p->y;
  if (p->z > bb->z2) bb->z2 = p->z;
}

void save_bbox_frame (GtsBBox * bb, FILE * fptr){
  g_return_if_fail (bb != NULL);

  fprintf (fptr, "%g %g %g %g %g %g\n",
	   bb->x1, bb->y1, bb->z1,
	   bb->x2, bb->y2, bb->z2);

}


//
// Load the next pair of images from the contents file
//
static bool get_stereo_pair( const string left_image_name,
                             const string right_image_name,
			     const string dir,
                             IplImage     *&left_image,
                             IplImage     *&right_image,
			     IplImage     *&color_image)
                      
			   
{
 
  //
  // Load the images (-1 for unchanged grey/rgb)
  //
  string complete_left_name( dir_name+left_image_name );
  left_image  = cvLoadImage( complete_left_name.c_str( ) , -1 );
  if( left_image == NULL )
    {
      cerr << "ERROR - unable to load image: " << complete_left_name << endl;
      return false;
    }

  string complete_right_name( dir_name+right_image_name );
  right_image = cvLoadImage( complete_right_name.c_str( ), -1 );
  if( right_image == NULL )
    {
      cerr << "ERROR - unable to load image: " << complete_right_name << endl;
      cvReleaseImage( &left_image );
      return false;
    }

  //
  // Convert to greyscale. Use cvCvtColor for consistency. Getting cvLoadImage to load
  // the images as greyscale may use different RGB->greyscale weights
  //
  if( left_image->nChannels == 3 )
    {
     
      color_image=left_image;
      IplImage *grey_left  = cvCreateImage( cvGetSize(left_image) , IPL_DEPTH_8U, 1 );
      cvCvtColor( left_image , grey_left , CV_BGR2GRAY );
      //IplImage *temp = left_image;
      left_image = grey_left;
      //  cvReleaseImage( &temp );
    }

  if( left_image->nChannels == 3 )
    {
      color_image=right_image;
      IplImage *grey_right = cvCreateImage( cvGetSize(right_image), IPL_DEPTH_8U, 1 );
      cvCvtColor( right_image, grey_right, CV_BGR2GRAY );
      IplImage *temp = right_image;
      right_image = grey_right;
      cvReleaseImage( &temp );
    }

  //
  // Scale images if required
  // 
  if( image_scale != 1.0 )
    {
      CvSize scaled_size = cvGetSize( left_image );
      scaled_size.width  = (int)(scaled_size.width*image_scale  );
      scaled_size.height = (int)(scaled_size.height*image_scale );

      IplImage *scaled_left  = cvCreateImage( scaled_size, 8, 1 );
      IplImage *scaled_right = cvCreateImage( scaled_size, 8, 1 );
      cvResize( left_image, scaled_left );
      cvResize( right_image, scaled_right );

      cvReleaseImage( &left_image );
      cvReleaseImage( &right_image );
      left_image = scaled_left;
      right_image = scaled_right;
    }

  
  return true;
}   
typedef struct _auv_images_names{
  std::string left_name;
  std::string right_name;
  std::string mesh_name;
  std::string dir;
  GtsMatrix *m;
  GtsBBox *bbox;
  Vector *veh_pose;
  double alt;
  double timestamp;
  int index;
  bool valid;
}auv_image_names;


typedef class threadedStereo{
public:
  threadedStereo(const string config_file_name, const string dense_config_file_name ){
    frame_id=0;
    tex_size=512;
    // Create the stereo feature finder
    //
    //
    // Figure out the directory that contains the config file 
    //
    string config_dir_name;
    int slash_pos = config_file_name.rfind( "/" );
    if( slash_pos != -1 )
      config_dir_name = config_file_name.substr( 0, slash_pos+1 );
    
    config_file= new Config_File(config_file_name.c_str());
    if(use_dense_feature)
      dense_config_file= new Config_File(dense_config_file_name.c_str());
     
    if( config_file->get_value( "STEREO_CALIB_FILE", stereo_calib_file_name) ){
      stereo_calib_file_name = config_dir_name+stereo_calib_file_name;
      try {
	calib = new Stereo_Calib( stereo_calib_file_name );
      }
      catch( string error ) {
	cerr << "ERROR - " << error << endl;
	exit( 1 );
      }

    }
    config_file->set_value( "SKF_SHOW_DEBUG_IMAGES" , display_debug_images );
    config_file->set_value( "SCF_SHOW_DEBUG_IMAGES"  , display_debug_images );
    config_file->set_value( "NCC_SCF_SHOW_DEBUG_IMAGES", display_debug_images );
    config_file->set_value( "MESH_TEX_SIZE", tex_size );
     
    if( use_sift_features )
      config_file->set_value( "SKF_KEYPOINT_TYPE", "SIFT" );
    else if( use_surf_features )   
      config_file->set_value( "SKF_KEYPOINT_TYPE", "SURF" );
    finder = NULL;
    finder_dense = NULL;
    if( use_sift_features || use_surf_features ){
#ifdef HAVE_LIBKEYPOINT
      finder = new Stereo_Keypoint_Finder( *config_file, 
					   use_undistorted_images, 
					   image_scale, 
					   calib );
#endif
    }
    else if( use_ncc )
      {
	finder = new Stereo_NCC_Corner_Finder(*config_file, 
                                              use_undistorted_images, 
                                              image_scale, 
                                              calib );
      }
    else
      {

     
	finder = new Stereo_Corner_Finder( *config_file, 
					   use_undistorted_images, 
					   image_scale, 
					   calib );
	if(use_dense_feature)
	  finder_dense = new Stereo_Corner_Finder(* dense_config_file, 
						  use_undistorted_images, 
						  image_scale, 
						  calib );
      }
     
    osgExp=new OSGExporter(dir_name,false,false,tex_size);    
   
  }

  ~threadedStereo(){

    delete finder;
    delete osgExp;
    delete calib;
    if(use_dense_feature)
      delete finder_dense;
    
  }
  bool runP(auv_image_names &name);
private:

  int frame_id;
  Matrix *image_coord_covar;
  Stereo_Feature_Finder *finder;
  Stereo_Feature_Finder *finder_dense;
  int tex_size;
  Stereo_Calib *calib;
  Config_File *config_file; 
  Config_File *dense_config_file; 
  Vector *camera_pose;
  OSGExporter *osgExp;
}threadedStereo;


// data structs


// PC model related 

typedef auv_image_names  Slice; // row index of matrix srcA...

// convenient typedefs...
typedef std::vector<Slice> Slices; 
typedef SPACE_YIN::Pool<Slice> SlicePool;
typedef SPACE_YIN::Consumer<Slice, SlicePool > SliceConsumer;


// forward declare
struct Convoluter; // subclass of SliceConsumer
struct Scheduler;  // subclass of SliceProducer
ts_counter doneCount(0);
int totalTodoCount;
typedef SPACE_YIN::Consuming<Slice, SlicePool, Convoluter > Convolute;
struct Convoluter : public SliceConsumer
{
  //InNOut* ino;
  threadedStereo *ts;

  Convoluter( SlicePool& slices, 
	      SPACE_YIN::Latch& lh) 
    : SliceConsumer(slices, lh) {}

protected:
  void consume(Slice slice_i) 
  {
	 
    if(!ts->runP(slice_i))
      slice_i.valid=false;
    else
      slice_i.valid=true;
	  
  } 
  bool cancel() { 
    return !channel_.channel_.size(); // may stop if no more tasks
  } 
public:
  void initThread(const string config_file_name,const string dense_config_file_name){
   
    
    ts= new threadedStereo(config_file_name,dense_config_file_name);
  }
 
};


// consumer pool test
struct Convolution : public Convolute
{
  //InNOut* ino;


  Convolution(SlicePool& channel, size_t nc ,const string config_file_name,const string dense_config_file_name)
    : Convolute (channel, nc,true,false,true),config_file_name(config_file_name),dense_config_file_name(dense_config_file_name) {} 

  void consumerModelCreated(Convoluter& consumer) 
  {
    consumer.initThread(config_file_name,dense_config_file_name);
			   
  }
  const string config_file_name, dense_config_file_name;
};






static int get_auv_image_name( const string  &contents_dir_name,
			       ifstream      &contents_file,
			       auv_image_names &name
			       )
{
 
  name.veh_pose = new Vector(AUV_NUM_POSE_STATES);
  name.m =gts_matrix_identity (NULL);
  name.bbox = gts_bbox_new(gts_bbox_class(),NULL,0,0,0,0,0,0);
  //
  // Try to read timestamp and file names
  //
  bool readok;
  int index;
  do{
     
    readok =(contents_file >> index &&
	     contents_file >> name.timestamp &&
	     contents_file >> name.left_name &&
	     contents_file >> name.right_name &&
	     contents_file >>  (*name.veh_pose)[AUV_POSE_INDEX_X] &&
	     contents_file >>   (*name.veh_pose)[AUV_POSE_INDEX_Y] &&
	     contents_file >>   (*name.veh_pose)[AUV_POSE_INDEX_Z] &&
	     contents_file >>   (*name.veh_pose)[AUV_POSE_INDEX_PHI] &&
	     contents_file >>   (*name.veh_pose)[AUV_POSE_INDEX_THETA] &&
	     contents_file >>   (*name.veh_pose)[AUV_POSE_INDEX_PSI] &&
	     contents_file >> name.alt );
  
  }
  while (readok && (name.timestamp < start_time || (skip_counter++ < num_skip)));
  skip_counter=0;
   
  if(!readok || name.timestamp >= stop_time) {
    // we've reached the end of the contents file
    return END_FILE;
  }      
  if (name.left_name == "DeltaT" || name.right_name == "DeltaT")
    return NO_ADD;
  if(!single_run){
    fprintf(fpp,"%f %f %f %f %f %f %f %f\n",   
	  name.timestamp,
	  (*name.veh_pose)[AUV_POSE_INDEX_X],
	  (*name.veh_pose)[AUV_POSE_INDEX_Y],
	  (*name.veh_pose)[AUV_POSE_INDEX_Z],
	  (*name.veh_pose)[AUV_POSE_INDEX_PHI],
	  (*name.veh_pose)[AUV_POSE_INDEX_THETA],
	  (*name.veh_pose)[AUV_POSE_INDEX_PSI],
	  name.alt);
  }
    
  return ADD_IMG;
         
}

   
bool threadedStereo::runP(auv_image_names &name){
  IplImage *left_frame;
  IplImage *right_frame;
  IplImage *color_frame;
  unsigned int left_frame_id=frame_id++;
  unsigned int right_frame_id=frame_id++;
  string left_frame_name;
  string right_frame_name;
  
  //
  // Load the images
  //
    
  if( !get_stereo_pair( name.left_name,name.right_name,name.dir,
			left_frame, right_frame,
			color_frame))
    {
      printf("Failed to get pair\n");
      return false;
    }                          
  
  if(feature_depth_guess == AUV_NO_Z_GUESS)
    feature_depth_guess = name.alt;
  //
  // Find the features
  //
  
  list<Feature *> features;
  finder->find( left_frame,
		right_frame,
		left_frame_id,
		right_frame_id,
		max_feature_count,
		features,
		feature_depth_guess );
  if(use_dense_feature) 
    finder_dense->find( left_frame,
			right_frame,
			left_frame_id,
			right_frame_id,
			max_feature_count,
			features,
			feature_depth_guess );
      
            
  //
  // Triangulate the features if requested
  //

     

  Stereo_Reference_Frame ref_frame = STEREO_LEFT_CAMERA;
       


  /*Matrix pose_cov(4,4);
    get_cov_mat(cov_file,pose_cov);
  */
  //cout << "Cov " << pose_cov << "Pose "<< veh_pose<<endl;
  list<Stereo_Feature_Estimate> feature_positions;
  stereo_triangulate( *calib,
		      ref_frame,
		      features,
		      left_frame_id,
		      right_frame_id,
		      NULL,//image_coord_covar,
		      feature_positions );
      
  //   static ofstream out_file( triangulation_file_name.c_str( ) );
      
  static Vector stereo1_nav( AUV_NUM_POSE_STATES );
  // Estimates of the stereo poses in the navigation frame
      
      
  list<Stereo_Feature_Estimate>::iterator litr;
  GPtrArray *localV = g_ptr_array_new ();
  GtsRange r;
  gts_range_init(&r);
  TVertex *vert;
  for( litr  = feature_positions.begin( ) ;
       litr != feature_positions.end( ) ;
       litr++ )
    {
	  
      vert=(TVertex*)  gts_vertex_new (t_vertex_class (),
				       litr->x[0],litr->x[1],litr->x[2]);
      //double confidence=1.0;
      Vector max_eig_v(3);
      /*  if(have_cov_file){
	    
	  Matrix eig_vecs( litr->P );
	  Vector eig_vals(3);
	  int work_size = eig_sym_get_work_size( eig_vecs, eig_vals );
	    
	  Vector work( work_size );
	  eig_sym_inplace( eig_vecs, eig_vals, work );
	   
	  double maxE=DBL_MIN;
	  int maxEidx=0;
	  for(int i=0; i < 3; i++){
	  if(eig_vals[i] > maxE){
	  maxE=eig_vals[i];
	  maxEidx=i;
	  }
	  }
	    
	  for(int i=0; i<3; i++)
	  max_eig_v(i)=eig_vecs(i,maxEidx);
	    
	  confidence= 2*sqrt(maxE);
	  max_eig_v = max_eig_v / sum(max_eig_v);
	  //    cout << "  eig_ max: " << max_eig_v << endl;
	    
	    
	  }
	  vert->confidence=confidence;
	  vert->ex=max_eig_v(0);
	  vert->ey=max_eig_v(1);
	  vert->ez=max_eig_v(2);
	  gts_range_add_value(&r,vert->confidence);*/
      g_ptr_array_add(localV,GTS_VERTEX(vert));
	 
	  
    }
  /*	 gts_range_update(&r);
	 for(unsigned int i=0; i < localV->len; i++){
	 TVertex *v=(TVertex *) g_ptr_array_index(localV,i);
	 if(have_cov_file){
	 float val= (v->confidence -r.min) /(r.max-r.min);
	 jet_color_map(val,v->r,v->g,v->b);
	 }
	 }
  */ 
  if(!localV->len)
    return false;
	 
  GtsSurface *surf= auv_mesh_pts(localV,0.5,0); 
	 
  Vector camera_pose(AUV_NUM_POSE_STATES);
  get_camera_params(config_file,camera_pose);
	 
  get_sensor_to_world_trans(*name.veh_pose,camera_pose,name.m);
  gts_surface_foreach_vertex (surf, (GtsFunc) gts_point_transform, name.m);
	   
	 
  char filename[255];
	 
  if(output_3ds){
    map<int,string>textures;
    textures[0]=(name.dir+name.left_name);
    sprintf(filename,"mesh/surface-%08d.3ds",
	    name.index);
	     
    std::map<int,GtsMatrix *> gts_trans;
    GtsMatrix *invM = gts_matrix_inverse(name.m);
    gts_trans[0]=(invM);
    gen_mesh_tex_coord(surf,&calib->left_calib,gts_trans,
		       NULL,tex_size,num_threads,0);
    std::vector<string> lodnames;
    osgExp->convertModelOSG(surf,textures,filename,512,NULL,NULL);
    gts_matrix_destroy (invM);
  }
  if(output_ply_and_conf){
	   
    GtsBBox *tmpBBox=gts_bbox_surface(gts_bbox_class(),surf);
    gts_bbox_set(name.bbox,tmpBBox->bounded,
		 tmpBBox->x1,
		 tmpBBox->y1,
		 tmpBBox->z1,
		 tmpBBox->x2,
		 tmpBBox->y2,
		 tmpBBox->z2);
    gts_object_destroy (GTS_OBJECT (tmpBBox));
	   
    FILE *fp;
    sprintf(filename,"%s/surface-%08d.ply",
	    subvoldir,name.index);
    fp = fopen(filename, "w" );
    auv_write_ply(surf, fp,have_cov_file,"test");
    fclose(fp);
	  
    sprintf(filename,"%s/surface-%08d.trans",
	    subvoldir,name.index);
    fp = fopen(filename, "w" );
    fprintf(fp,"%s\n",name.left_name.c_str());
    for(int n=0; n< 4; n++)
      for(int p=0; p<4; p++)
	fprintf(fp,"%f ",name.m[n][p]);
    fprintf(fp,"\n");
    fclose(fp);
	  
  }
  //Destory Surf
  if(surf)
    gts_object_destroy (GTS_OBJECT (surf)); 
	 
  //
  // Pause between frames if requested.
  //
  if( display_debug_images && pause_after_each_frame )
    cvWaitKey( 0 );
  else if( display_debug_images )
    cvWaitKey( 100 );
	 
	 
  //
  // Clean-up
  //
	 
  cvReleaseImage( &left_frame );
  cvReleaseImage( &right_frame );
  cvReleaseImage( &color_frame);
        
  list<Feature*>::iterator fitr;
  for( fitr  = features.begin( ) ;
       fitr != features.end( ) ;
       fitr++ )
    {
      delete *fitr;
    }     
  int progCount=doneCount.increment();
  image_count_verbose (progCount, totalTodoCount);
  // printf("\rStereo processing on image %u/%u complete.",progCount,totalTodoCount);
  //fflush(stdout);
  // doneCount.increment();
      
  return true;
}



void runC(auv_image_names &name){
 
  printf("%s Written Out to  %s\n",name.left_name.c_str(),name.mesh_name.c_str());
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
  string path=string(argv[0]);
  unsigned int loc=path.rfind("/");
   
  string basepath= loc == string::npos ? "./" : path.substr(0,loc+1);
  basepath= osgDB::getRealPath (basepath);
  cout << "Basepath " <<basepath <<endl;
 
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
      cerr << "a ERROR - " << error << endl;
      exit( 1 );
    }
  //
  // Run through the data
  //conf
  ifstream *cov_file;
  ifstream      contents_file;
  Vector *camera_pose;
  const char *uname="mesh";
  const char *uname2="mesh-agg";
 
  auv_data_tools::makedir(uname);
  auv_data_tools::makedir(uname2);
  chmod(uname,   0777);
  chmod(uname2,   0777);
  auv_data_tools::makedir(subvoldir);
  chmod(subvoldir,   0777);
  //
  // Open the contents file
  //
  contents_file.open( contents_file_name.c_str( ) );
  if( !contents_file )
    {
      cerr << "ERROR - unable to open contents file: " << contents_file_name
           << endl;
      exit( 1 );     
    }
  
  cov_file = new ifstream;
  if(have_cov_file){ 
    cov_file->open( cov_file_name);
    if( !cov_file )
      {
	cerr << "ERROR - unable to open contents file: " << cov_file_name
	     << endl;
	exit( 1 );     
      }
  }
  if(!single_run){
    fpp=fopen("mesh/campath.txt","w");
    if(!fpp ){
      fprintf(stderr,"Cannot open mesh/campath.txt\n");
      exit(-1);
    }
  }
  if(output_pts_cov)
    pts_cov_fp=  fopen("pts_cov.txt","w");

  //
  // Figure out the directory that contains the contents file 
  //
  if(output_uv_file)
    uv_fp= fopen("uvfile.txt","w");
  if(output_3ds){
    const char *uname="mesh";
    auv_data_tools::makedir(uname); 
    file_name_list.open("mesh/filenames.txt");
  }  
  //
  camera_pose =new Vector(AUV_NUM_POSE_STATES);
  get_camera_params(config_file,*camera_pose);
  Matrix *image_coord_covar;
  if(have_cov_file){
    image_coord_covar = new Matrix(4,4);
    image_coord_covar->clear( );
    config_file->get_value("STEREO_LEFT_X_VAR",(*image_coord_covar)(0,0) );
    config_file->get_value("STEREO_LEFT_Y_VAR",(*image_coord_covar)(1,1) );
    config_file->get_value("STEREO_RIGHT_X_VAR",(*image_coord_covar)(2,2));
    config_file->get_value("STEREO_RIGHT_Y_VAR",(*image_coord_covar)(3,3));
  }else
    image_coord_covar=NULL;

  Slices tasks;
  unsigned int stereo_pair_count =0;
  
  if(single_run){
    have_max_frame_count = true;
    if(single_run_stop < (int)max_frame_count  )
      max_frame_count = single_run_stop-single_run_start;
    printf("Single run %d %d\n",single_run_start,single_run_start+max_frame_count-1);
  }
  int start_skip=0;
  while( !have_max_frame_count || stereo_pair_count < max_frame_count ){
    auv_image_names name;
    int ret=get_auv_image_name( dir_name, contents_file, name) ;
    if(ret == ADD_IMG ){
      if(start_skip++ < single_run_start)
	continue;
      name.index= single_run_start+stereo_pair_count++;
      name.valid=true;
      tasks.push_back(name);
    }else if(ret == NO_ADD){
      continue;
    }else if(ret == END_FILE)
      break;
  }
  start_time=tasks[0].timestamp;
  stop_time=tasks[tasks.size()-1].timestamp;
  totalTodoCount=stereo_pair_count;
  boost::xtime xt, xt2;
  long time;
  // consumer pool model...
  if(num_threads == 1){
     
    boost::xtime_get(&xt, boost::TIME_UTC);
    threadedStereo *ts= new threadedStereo(stereo_config_file_name,"semi-dense.cfg");
    for(unsigned int i=0; i < tasks.size(); i++){
      if(!ts->runP(tasks[i]))
	tasks.erase(tasks.begin()+i);
    }    
    boost::xtime_get(&xt2, boost::TIME_UTC);
    time = (xt2.sec*1000000000 + xt2.nsec - xt.sec*1000000000 - xt.nsec) / 1000000;
     
     
    double secs=time/1000.0;
    printf("single thread: %.2f sec\n", secs);
     
    delete ts;
  }
  else{
    g_thread_init (NULL);
    display_debug_images = false; 
    boost::xtime_get(&xt, boost::TIME_UTC);
    SlicePool pool(tasks);
    Convolution convolution(pool,(int)tasks.size() > num_threads ? num_threads : tasks.size(),stereo_config_file_name,"semi-dense.cfg");
    boost::thread thrd(convolution);
    thrd.join();
     
    boost::xtime_get(&xt2, boost::TIME_UTC);
    time = (xt2.sec*1000000000 + xt2.nsec - xt.sec*1000000000 - xt.nsec) / 1000000; 
    double secs=time/1000.0;
    printf("max %d consumer pool: %.2f sec\n", num_threads, secs);
    for(Slices::iterator itr=tasks.begin(); itr != tasks.end(); itr++)
      if(!itr->valid)
	tasks.erase(itr);
  }
  if(!single_run){
    char conf_name[255];
    sprintf(conf_name,"%s/surface.conf",subvoldir);
     
    conf_ply_file=fopen(conf_name,"w");
      
    for(unsigned int i=0; i < tasks.size(); i++){
      if(tasks[i].valid)
	fprintf(conf_ply_file,
		"bmesh surface-%08d.ply 0.033 1\n"
		,tasks[i].index); 
    
    }
   
    if(!single_run && have_mb_ply){
      for(int i=0; i < (int)mb_ply_filenames.size(); i++){
	string simpname =osgDB::getSimpleFileName(mb_ply_filenames[i]);
	string inname=mb_ply_filenames[i];
	string outname=(string(subvoldir)+string("/")+simpname);
	cout << "Copying " << inname << " to " << outname<<endl;
	std::ifstream  IN (inname.c_str());
	std::ofstream  OUT(outname.c_str()); 
	OUT << IN.rdbuf();
	fprintf(conf_ply_file,
		"bmesh %s .1 0\n"
		,simpname.c_str()); 
      }
    }
    if(!single_run)
      fclose(conf_ply_file);
    
    if(output_uv_file)
      fclose(uv_fp);
    if(output_3ds)
      file_name_list.close();
   
    if(fpp)
      fclose(fpp);
    if(output_3ds){
      fpp = fopen("mesh/meshinfo.txt","w");
      fprintf(fpp,"%d\n",meshNum);
      fclose(fpp);
    }
    if(fpp2)
      fclose(fpp2);
    if(conf_ply_file){
      if(output_pts_cov)
	fclose(pts_cov_fp);
      if(gen_mb_ply){
	FILE *genmbfp=fopen("genmb.sh","w");
	fprintf(genmbfp,"#!/bin/bash\n%s/../seabed_localisation/bin/seabed_pipe --start %f --stop %f --mesh %s %s %s",basepath.c_str(),start_time,stop_time,deltaT_config_name.c_str(),deltaT_dir.c_str(),contents_file_name.c_str());
	fchmod(fileno(genmbfp),   0777);
	fclose(genmbfp);
	system("./genmb.sh");
      }
      if(!single_run){
	conf_ply_file=fopen("./runvrip.sh","w+"); 
	fprintf(conf_ply_file,"#!/bin/bash\nOUTDIR=$PWD\nVRIP_HOME=%s/vrip\nexport VRIP_DIR=$VRIP_HOME/src/vrip/\nPATH=$PATH:$VRIP_HOME/bin\ncd %s/\n",basepath.c_str(),subvoldir);
	fprintf(conf_ply_file,"%s/vrip/bin/vripnew auto.vri surface.conf surface.conf 0.033 -rampscale 400 > vriplog.txt\n%s/vrip/bin/vripsurf auto.vri total.ply > vripsurflog.txt",basepath.c_str(),basepath.c_str());
	fchmod(fileno(conf_ply_file),0777);
	fclose(conf_ply_file);
	system("./runvrip.sh");
      
	
	FILE *dicefp=fopen("./dice.sh","w+");
	fprintf(dicefp,"#!/bin/bash\necho 'Dicing...\n'\nVRIP_HOME=%s/vrip\nexport VRIP_DIR=$VRIP_HOME/src/vrip/\nPATH=$PATH:$VRIP_HOME/bin\nRUNDIR=$PWD\nDICEDIR=$PWD/mesh-agg/\nmkdir -p $DICEDIR\ncd $DICEDIR\n%s/vrip/bin/plydice -writebboxall bbtmp.txt  -writebbox range.txt -dice %f %f %s total.ply | tee diced.txt\n" ,basepath.c_str(),basepath.c_str(),subvol,eps,"diced");
	fprintf(dicefp,"%s/vrip/bin/vripdicebbox surface.conf $DICEDIR\n",
       	basepath.c_str());
	fprintf(dicefp,"cd $RUNDIR\n%s/genTex %s -f %s ",basepath.c_str(),stereo_config_file_name.c_str(),dir_name.c_str());
	if(!hardware_compress)
	  fprintf(dicefp,"--no-hardware-compress\n");
	else
	  fprintf(dicefp,"\n");

	fchmod(fileno(dicefp),0777);
	fclose(dicefp);
	system("./dice.sh");
      }
    }
  }
   
  
  // 
  // Clean-up
  //

  //  for(int i =0; i < (int)tasks.size(); i++)
  // delete tasks[i].veh_pose; 
  delete config_file;
  delete camera_pose;

  delete cov_file;

}








 

