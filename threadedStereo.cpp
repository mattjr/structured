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

#include "auv_args.hpp"
#include "Clean.h"
#include <sys/time.h>
#include <time.h>
#include <unistd.h> 
#include "cv.h"
#include "highgui.h"
#include "ulapack/eig.hpp"
#include "auv_image_distortion.hpp"
#include "auv_stereo_geometry.hpp"
#include "adt_file_utils.hpp"
#include "auv_stereo_corner_finder.hpp"
#include "auv_stereo_ncc_corner_finder.hpp"
#include "auv_stereo_keypoint_finder.hpp"
#include "adt_image_norm.hpp"
#include "auv_stereo_dense.hpp"
#include "auv_concurrency.hpp"
#include "OSGExport.h"
#include "keypoint.hpp"
#include "XForm.h"
#include "TriMesh_algo.h"
#include "ShellCmd.h"
#include "stereo_cells.hpp"
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
static int dist_gentex_range=0;
static int vrip_split;
static FILE *pts_cov_fp;
static string stereo_config_file_name;
static string contents_file_name;
static string dir_name;
static bool use_cached=true;
static bool output_uv_file=false;
static bool further_clean=false;
static bool use_undistorted_images = false;
static bool pause_after_each_frame = false;
static double image_scale = 1.0;
static bool use_poisson_recon=true;
static int max_feature_count;
static double eps=1.0;
static double subvol;
static bool run_pos=true;
static bool do_novelty=false;
static double dense_scale;
static bool have_max_frame_count = false;
static unsigned int max_frame_count=INT_MAX;
static bool display_debug_images = true;
static bool output_pts_cov=false;
static bool use_sift_features = false;
static bool sing_gen_tex=false;
static bool use_surf_features = false;
static bool use_ncc = false;
static int skip_counter=0;
static double vrip_ramp;
static int num_skip=0;
static bool use_proj_tex=false;
static vector<string> mb_ply_filenames;
static bool have_mb_ply=false;
static bool have_cov_file=false;
static string stereo_calib_file_name;
static bool no_simp=true;
static double simp_res[3];
static FILE *uv_fp;
static ofstream file_name_list;
static string base_dir;
static bool no_depth=false;
static double feature_depth_guess = AUV_NO_Z_GUESS;
static int num_threads=1;
static FILE *fpp,*fpp2,*pos_fp;
static double connected_comp_size_clean;
static double hole_fill_size;
static bool even_split=false;
static double cell_scale=1.0;
static bool use_dense_feature=false;
//StereoMatching* stereo;
//StereoImage simage;
static bool gen_mb_ply=false;
static bool output_ply_and_conf =true;
static FILE *conf_ply_file;
static bool output_3ds=false;
static char cov_file_name[255];
static bool no_gen_tex=false;
static int mono_skip=2;
static bool no_vrip=false;
static double vrip_res;
static bool regen_tex=false;
static string basepath;
static bool single_run=false;
static int single_run_start=0;
static int single_run_stop=0;
static bool dice_lod=false;
static bool clean_pos_pts;
static bool use_dense_stereo=false;
static int non_cached_meshes=0;
static double edgethresh;
static bool no_merge=false;
enum {END_FILE,NO_ADD,ADD_IMG};
char cachedmeshdir[255];
char cachedtexdir[255];
static bool no_pos_clip=true;
static string deltaT_config_name;
static string deltaT_dir;
static bool use_vrip_recon=false;
static bool hardware_compress=true;
const char *uname="mesh";
const char *dicedir="mesh-diced";
const char *aggdir="mesh-agg";
static string recon_config_file_name;
static string deltaT_pose;
static string dense_method="";
bool dist_run=false;
static bool mono_cam=false;

static bool do_hw_blend=false;
// Image normalisation
#define USE_IMAGE_NORMALISATION true
int normalised_mean;
int normalised_var;
static Stereo_Calib *calib;
 static Config_File *config_file; 
 static Config_File *recon_config_file; 

  static Config_File *dense_config_file; 
static  int tex_size;
#ifdef USE_DENSE_STEREO
Stereo_Dense *sdense=NULL;
#endif

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



//
// Parse command line arguments into global variables
//
static bool parse_args( int argc, char *argv[ ] )
{
  libplankton::ArgumentParser argp(&argc,argv);
  if(argp.isOption(argp[1])){
    fprintf(stderr,"First arg must be base dir");
    exit(-1);
  }
  base_dir=argp[1];
  recon_config_file_name = "recon.cfg";
  stereo_config_file_name = "stereo.cfg";
  contents_file_name = "pose_file.data";

  dir_name = "img/";
  strcpy(cachedtexdir,"cache-tex/");


  argp.read("--stereo-config",stereo_config_file_name);
  argp.read("--poses",contents_file_name );

  deltaT_config_name=base_dir+string("/")+"localiser.cfg";
  deltaT_dir=base_dir+string("/")+"DT/";
  deltaT_pose=base_dir+string("/")+"deltat_pose_est.data";
  stereo_config_file_name= base_dir+string("/")+stereo_config_file_name;
  recon_config_file_name= base_dir+string("/")+recon_config_file_name;
  contents_file_name= base_dir+string("/")+contents_file_name;
  dir_name= base_dir+string("/")+dir_name;
 
  // cout <<dir_name<<endl;
  // Create the stereo feature finder
  //
  //
  // Figure out the directory that contains the config file 
  //
  try {
    recon_config_file= new Config_File(recon_config_file_name.c_str());
  }   catch( string error ) {
     cerr << "ERROR - " << error << endl;
     exit( 1 );
   }
  string config_dir_name;
  int slash_pos = stereo_config_file_name.rfind( "/" );
  if( slash_pos != -1 )
    config_dir_name = stereo_config_file_name.substr( 0, slash_pos+1 );
   try {
     config_file= new Config_File(stereo_config_file_name.c_str());
   }   catch( string error ) {
     cerr << "ERROR - " << error << endl;
     exit( 1 );
   }
   
  if(use_dense_feature)
    dense_config_file= new Config_File("semi-dense.cfg");
  
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
  config_file->get_value( "SD_SCALE", dense_scale,1.0);
 
  
  

  bool vrip_on;
  recon_config_file->get_value("USE_VRIP",vrip_on,false);
  
  mono_cam=argp.read("--mono");
  if(!mono_cam)
    recon_config_file->get_value("MONO_CAM",mono_cam,false);
 


  recon_config_file->get_value("VRIP_SUBVOL",subvol,40.0);
  recon_config_file->get_value("MAX_FEAT_COUNT",max_feature_count,5000);
  recon_config_file->get_value("VRIP_RAMP",vrip_ramp,500.0);
  recon_config_file->get_value("EDGE_THRESH",edgethresh,2.0);
  recon_config_file->get_value("VRIP_SPLIT",vrip_split,250);
  string split_method;
  recon_config_file->get_value("SPLIT_METHOD",split_method,"cost");
  if(split_method == "even"){
    even_split=true;
  }
  recon_config_file->get_value("SPLIT_CELL_SCALE",cell_scale,1.0);


  recon_config_file->get_value("VRIP_RES",vrip_res,0.033);
  recon_config_file->get_value("NORMALISED_VAR",normalised_var,400);
  recon_config_file->get_value("NORMALISED_MEAN",normalised_mean,128);
  recon_config_file->get_value("HOLE_FILL_SIZE",hole_fill_size,10.0);
  recon_config_file->get_value("CC_CLEAN_SIZE",connected_comp_size_clean,5.0);
  recon_config_file->get_value("EXTRA_CLEAN",further_clean,false);
  recon_config_file->get_value("CLEAN_POS_PTS",clean_pos_pts,false);
  recon_config_file->get_value("SIMP_RES_1",simp_res[0],0.005);
  recon_config_file->get_value("SIMP_RES_2",simp_res[1],0.1);
  recon_config_file->get_value("SIMP_RES_3",simp_res[2],0.5);
  recon_config_file->get_value("DIST_GENTEX_RANGE",dist_gentex_range,10);

  
 

  string mbfile;

   
  argp.read("-r",image_scale);	  
  argp.read( "--edgethresh" ,edgethresh);
  argp.read("-m", max_feature_count );
  argp.read("-f",dir_name);
  have_mb_ply=argp.read("--mbfile",mbfile);
  if(have_mb_ply)
    mb_ply_filenames.push_back(mbfile) ;
      
  argp.read( "--monoskip" ,mono_skip);

  no_pos_clip=(!argp.read(  "--posclip"));

  if(argp.read("--genmb")){
    gen_mb_ply=true;
    have_mb_ply=true;
    mb_ply_filenames.push_back(string("mb.ply")) ;
  }
  argp.read("-z",feature_depth_guess );
  use_dense_stereo=argp.read("--ds" );
      
  argp.read("--dense-method",dense_method);
  no_depth=argp.read("--no-depth" );
      
  argp.read("-s" ,num_skip);

  single_run= argp.read( "--single-run",single_run_start, single_run_stop );
  if(single_run)
    display_debug_images = false;
     
  
  argp.read( "-t" ,num_threads);
  argp.read( "--res",vrip_res );
  string cov_file;
  if(argp.read("--cov" ,cov_file))
    have_cov_file=true;
  if(argp.read( "-n",max_frame_count))
    have_max_frame_count = true;
  use_undistorted_images = argp.read("-u" );
  use_proj_tex=argp.read("--projtex");
  even_split= argp.read("--evensplit" );
  argp.read("--cellscale",cell_scale );
  if(!vrip_on)
    vrip_on=argp.read("--vrip");
  
  if(vrip_on ){
    run_pos=false;
    use_vrip_recon = true;
    no_simp = false;
  }
  argp.read("--vrip-ramp",vrip_ramp );
  dist_run=argp.read("--dist" );
  sing_gen_tex =  argp.read("--threaded-gentex");
  dice_lod=argp.read("--dicelod" );

  no_simp=argp.read( "--nosimp" );

  do_hw_blend=argp.read("--blend" );

  use_cached=(!argp.read("--no_cached" ));
  do_novelty=argp.read("--novelty");
  output_pts_cov=argp.read("--ptscov");
  display_debug_images = !argp.read("-d");
  regen_tex=argp.read("--regen");
  pause_after_each_frame = argp.read("-p");
  use_ncc=argp.read("-c");
  argp.read("--split",vrip_split);

  argp.read("--dicevol",subvol);
  hardware_compress =   !argp.read("--no-hardware-compress");
     
  output_uv_file=argp.read("--uv" ) ;
  use_sift_features=argp.read("--sift");
 
  use_dense_feature = argp.read("--dense-features");
  use_surf_features = argp.read("--surf");
  argp.read("--start",start_time);
  argp.read("--stop",stop_time);
  output_3ds=  argp.read("--3ds");
  no_gen_tex=argp.read("--nogentex");
  no_vrip=argp.read("--novrip");
  no_merge=argp.read("--nomerge");
  if(no_merge){
    run_pos = false;
    use_vrip_recon=true;
    no_simp=false;
  }

  if(!output_3ds && !output_ply_and_conf){
    cerr << "Must do ply or 3ds output\n";
    return false;
  }
  
 if(dense_method == "")
    config_file->get_value( "SD_METHOD", dense_method);
  else
    config_file->set_value( "SD_METHOD", dense_method);

  
  if( use_sift_features )
    config_file->set_value( "SKF_KEYPOINT_TYPE", "SIFT" );
    else if( use_surf_features )   
      config_file->set_value( "SKF_KEYPOINT_TYPE", "SURF" );

  strcpy(cachedmeshdir,"cache-mesh");
  if(use_dense_stereo)
    sprintf(cachedmeshdir,"%s-dense/",cachedmeshdir);
  else
    sprintf(cachedmeshdir,"%s-feat/",cachedmeshdir);

  strcpy(cachedmeshdir,string(base_dir+string("/")+cachedmeshdir).c_str());
  strcpy(cachedtexdir,string(base_dir+string("/")+cachedtexdir).c_str());


#ifndef HAVE_LIBKEYPOINT
  if( use_sift_features || use_surf_features )
    {
      cerr << "ERROR - libsnapper was compiled without sift support" << endl;
      return false;
    }
#endif
  if (argp.errors())
    {
      argp.writeErrorMessages(std::cout);
      return false;
    }
  return true;
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
  cout << "   --nogentex              Don't texture" << endl;
  cout << "   --genmb               Generate MB mesh" << endl;
  cout << "   --split <num>         Split individual meshes passed to vrip at num" << endl;
  cout << "   --dicevol <vol>         Dice mesh to subvolume pieces of <vol> size" << endl;
  cout << "   --res <resolution>      Vrip at resolution res default 0.033" << endl;
  cout << "   -p                      Pause after each frame." << endl;
  cout << "   --uv                    Output UV File." << endl;
  cout << "   --3ds                   Output 3ds Files." << endl;
  cout << "   --dense-features        Dense features ." << endl;
  cout << "   --ptscov                Output pts and cov ." << endl;
  cout << "   --dicelod                Dice lods" << endl;
  cout << "   --stereo-config              Specify diffrent stereo config" << endl;
  cout << "   --contents-file         Specify diffrent contents file ." << endl;
  cout << "   --nosimp         Specify diffrent contents file ." << endl;
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

  fprintf (fptr, "%g %g %g %g %g %g",
	   bb->x1, bb->y1, bb->z1,
	   bb->x2, bb->y2, bb->z2);

}

// Remove edge longer then thresh
void edge_len_thresh(TriMesh *mesh,double thresh)
{
  mesh->need_faces();
  int numfaces = mesh->faces.size();


  vector<bool> toremove(numfaces, false);
  for (int i = 0; i < numfaces; i++) {
    const point &v0 = mesh->vertices[mesh->faces[i][0]];
    const point &v1 = mesh->vertices[mesh->faces[i][1]];
    const point &v2 = mesh->vertices[mesh->faces[i][2]];
    float d01 = dist2(v0, v1);
    float d12 = dist2(v1, v2);
    float d20 = dist2(v2, v0);
    if (d01 > thresh || d12 > thresh || d20 > thresh)
      toremove[i] = true;
  }
  remove_faces(mesh, toremove);
  remove_unused_vertices(mesh);
}

// Remove edge longer then thresh
void edge_len_thresh_percent(TriMesh *mesh,double thresh)
{
  double sum=0.0;
  mesh->need_faces();
  int numfaces = mesh->faces.size();
  for (int i = 0; i < numfaces; i++) {
    const point &v0 = mesh->vertices[mesh->faces[i][0]];
    const point &v1 = mesh->vertices[mesh->faces[i][1]];
    const point &v2 = mesh->vertices[mesh->faces[i][2]];
    sum+= dist2(v0, v1);
    sum += dist2(v1, v2);
    sum+= dist2(v2, v0);
  }
  sum/=numfaces;
  double threshlen=sum * thresh;
  //printf("Avg len %f Thresh %f\n",sum,threshlen);
  vector<bool> toremove(numfaces, false);
  for (int i = 0; i < numfaces; i++) {
    const point &v0 = mesh->vertices[mesh->faces[i][0]];
    const point &v1 = mesh->vertices[mesh->faces[i][1]];
    const point &v2 = mesh->vertices[mesh->faces[i][2]];
    float d01 = dist2(v0, v1);
    float d12 = dist2(v1, v2);
    float d20 = dist2(v2, v0);
    if (d01 > threshlen || d12 > threshlen || d20 > threshlen)
      toremove[i] = true;
  }
  remove_faces(mesh, toremove);
  remove_unused_vertices(mesh);
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
  color_image=NULL;
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

  if( right_image->nChannels == 3 )
    {
      color_image=right_image;
      IplImage *grey_right = cvCreateImage( cvGetSize(right_image), IPL_DEPTH_8U, 1 );
      cvCvtColor( right_image, grey_right, CV_BGR2GRAY );
      //   IplImage *temp = right_image;
      right_image = grey_right;
      // cvReleaseImage( &temp );
    }

  if(color_image == NULL)
    color_image = cvLoadImage( complete_left_name.c_str( ), 1 );
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
  //
  // Normalise the mean and variance of the pixel intensity values
  //
  if( USE_IMAGE_NORMALISATION )
    {
      normalise_image( normalised_mean, normalised_var, left_image  );
      normalise_image( normalised_mean, normalised_var, right_image );
      
    }

  
  return true;
}   
//typedef class Stereo_Pose_Data  auv_images_names ;
/*{
  std::string left_name;
  std::string right_name;
  std::string mesh_name;
  std::string dir;
  GtsMatrix *m;
  GtsBBox *bbox;
  Vector *cam_pose;
  double alt;
  int crossover;
  double prob;
  double timestamp;
  int index;
  bool valid;
  }auv_image_names;
*/

typedef class threadedStereo{
public:
  threadedStereo(const string config_file_name, const string dense_config_file_name ){
    frame_id=0;
    tex_size=512;
  
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
     
    osgExp=new OSGExporter(dir_name,false,true,tex_size);    

  }

  ~threadedStereo(){

    delete finder;
    delete osgExp;
    delete calib;
    if(use_dense_feature)
      delete finder_dense;
    
  }
  bool runP(Stereo_Pose_Data &name);
private:

  int frame_id;
  Matrix *image_coord_covar;
  Stereo_Feature_Finder *finder;
  Stereo_Feature_Finder *finder_dense;

 
 
  
  OSGExporter *osgExp;
}threadedStereo;


// data structs


// PC model related 

typedef Stereo_Pose_Data  Slice; // row index of matrix srcA...

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
			       Stereo_Pose_Data &name
			       )
{
 
  //name.cam_pose = new Vector(AUV_NUM_POSE_STATES);
  name.m =gts_matrix_identity (NULL);
  name.bbox = gts_bbox_new(gts_bbox_class(),NULL,0,0,0,0,0,0);
  //
  // Try to read timestamp and file names
  //
  bool readok;
  int index;
  do{
     
    readok =(contents_file >> index &&
	     contents_file >> name.time &&
	     contents_file >>  name.pose[AUV_POSE_INDEX_X] &&
	     contents_file >>   name.pose[AUV_POSE_INDEX_Y] &&
	     contents_file >>   name.pose[AUV_POSE_INDEX_Z] &&
	     contents_file >>   name.pose[AUV_POSE_INDEX_PHI] &&
	     contents_file >>   name.pose[AUV_POSE_INDEX_THETA] &&
	     contents_file >>   name.pose[AUV_POSE_INDEX_PSI] &&
	     contents_file >> name.left_name &&
	     contents_file >> name.right_name &&
	     contents_file >> name.alt &&
	     contents_file >> name.radius &&
	     contents_file >> name.overlap 
	     );
  
  }
  while (readok && (name.time < start_time || (skip_counter++ < num_skip)));
  skip_counter=0;
   
  if(!readok || name.time >= stop_time) {
    // we've reached the end of the contents file
    return END_FILE;
  }      
  if (name.left_name == "DeltaT" || name.right_name == "DeltaT")
    return NO_ADD;

  fill_gts_matrix(name.pose,name.m);

    
  return ADD_IMG;
         
}

static int get_mono_image_name( const string  &contents_dir_name,
				ifstream      &contents_file,
				Mono_Image_Name  &name
				)
{
 
  name.pose=new Vector(AUV_NUM_POSE_STATES);
  //
  // Try to read timestamp and file names
  //
  bool readok;
  int index;
  do{
   
    readok =(contents_file >> index &&
	     contents_file >> name.time &&
	     contents_file >> (*name.pose)(AUV_POSE_INDEX_X) &&
	     contents_file >> (*name.pose)(AUV_POSE_INDEX_Y) &&
	     contents_file >> (*name.pose)(AUV_POSE_INDEX_Z) &&
	     contents_file >> (*name.pose)(AUV_POSE_INDEX_PHI) &&
	     contents_file >> (*name.pose)(AUV_POSE_INDEX_THETA) &&
	     contents_file >> (*name.pose)(AUV_POSE_INDEX_PSI) &&
	     contents_file >> name.img_name);
  
  }
  while (readok && (name.time < start_time || (skip_counter++ < num_skip)));
  skip_counter=0;
   
  if(!readok || name.time >= stop_time) {
    // we've reached the end of the contents file
    return END_FILE;
  }      
  if (name.img_name == "DeltaT")
    return NO_ADD;

    
  return ADD_IMG;
         
}

   
bool threadedStereo::runP(Stereo_Pose_Data &name){
  IplImage *left_frame;
  IplImage *right_frame;
  IplImage *color_frame;
  unsigned int left_frame_id=frame_id++;
  unsigned int right_frame_id=frame_id++;
  string left_frame_name;
  string right_frame_name;
  char filename[255];
  char meshfilename[1024];
  char texfilename[1024];
  bool meshcached=false;
  bool texcached=false;
  
  
  if(mono_cam)
    calib=name.calib;

#ifdef USE_DENSE_STEREO
  if(!sdense && use_dense_stereo){
    sdense= new Stereo_Dense(*config_file,
			     dense_scale ,
			     calib  );
    printf("Dense Scale %f\n",dense_scale);
  }
#endif
  
  /*  FILE *fp;
      sprintf(filename,"%s/surface-%s.xf",
      aggdir,osgDB::getStrippedName(name.left_name).c_str());
      fp = fopen(filename, "w" );
      if(!fp){
      fprintf(stderr,"Failed to open %s for writing\n",filename);
      return false;
      }
 
      for(int n=0; n< 4; n++){
      for(int p=0; p<4; p++)
      fprintf(fp,"%f ",name.m[n][p]);
      fprintf(fp,"\n");
      }
      fclose(fp);
      chmod(filename,   0666);
  */
  sprintf(texfilename,"%s/%s.dds",
	  cachedtexdir,osgDB::getStrippedName(name.left_name).c_str());
  if(use_dense_stereo)
    sprintf(meshfilename,"%s/surface-%s-%s.ply",
	    cachedmeshdir,osgDB::getStrippedName(name.left_name).c_str(),
	    dense_method.c_str());
  else
    sprintf(meshfilename,"%s/surface-%s.ply",
	    cachedmeshdir,osgDB::getStrippedName(name.left_name).c_str());

  //printf("Mesh cached check %s\n",meshfilename);

  if(!use_cached){
 
    //     printf("Redoing cache\n");
    meshcached=false;
    texcached=false;
  }else{
    meshcached=FileExists(meshfilename);
    texcached=FileExists(texfilename);
  }
 
  sprintf(filename,"%s/surface-%s.tc.ply",
	  aggdir,osgDB::getStrippedName(name.left_name).c_str());

 
  GtsSurface *surf=NULL;
  if(!meshcached || !texcached ){
  

  
    //
    // Load the images
    //
    
    if( !get_stereo_pair( name.left_name,name.right_name,name.dir,
			  left_frame, right_frame,
			  color_frame))
      {
	printf("Failed to get pair %s %s\n",name.left_name.c_str(),name.right_name.c_str());
	int progCount=doneCount.increment();
	image_count_verbose (progCount, totalTodoCount);
	return false;
      } 
    
    if(!texcached){
      //      printf("\nCaching texture %s\n",texfilename);
      osgExp->cacheCompressedImage(color_frame,texfilename,512);
    }
    
    if(!meshcached){
    
      //printf("Not cached creating\n");
      non_cached_meshes++;
      
      if(feature_depth_guess == AUV_NO_Z_GUESS && !no_depth&& name.alt >0.0)
	feature_depth_guess = name.alt;
      
      list<Stereo_Feature_Estimate> feature_positions;
      GPtrArray *localV=NULL;  
      list<Feature *> features;
      if(!use_dense_stereo){
	//
	// Find the features
	//
       
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
	localV = g_ptr_array_new ();
	GtsRange r;
	gts_range_init(&r);
	TVertex *vert;
	for( litr  = feature_positions.begin( ) ;
	     litr != feature_positions.end( ) ;
	     litr++ )
	  {
	    // if(litr->x[2] > 8.0 || litr->x[2] < 0.25)
	    //continue;
	    //  if(name.alt > 5.0 || name.alt < 0.75)
	    //litr->x[2]=name.alt;
	    vert=(TVertex*)  gts_vertex_new (t_vertex_class (),
					     litr->x[0],litr->x[1],litr->x[2]);
	    //printf("%f %f %f\n", litr->x[0],litr->x[1],litr->x[2]);
	    
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
	list<Feature*>::iterator fitr;
	for( fitr  = features.begin( ) ;
	     fitr != features.end( ) ;
	     fitr++ )
	  {
	    delete *fitr;
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
      }else{ 
#ifdef USE_DENSE_STEREO   
	sdense->dense_stereo(left_frame,right_frame);
	std::vector<libplankton::Vector> points;   
	sdense->get_points(points);
	localV = g_ptr_array_new ();
	TVertex *vert;
	for(int i=0; i<(int)points.size(); i++){
	  //printf("%f %f %f\n",points[i](0),points[i](1),points[i](2));
	  if(points[i](2) > 4.0 )
	    continue;
	  
	  vert=(TVertex*)  gts_vertex_new (t_vertex_class (),
					   points[i](0),points[i](1),
					   points[i](2));
	  g_ptr_array_add(localV,GTS_VERTEX(vert));
	} 
#else 
	fprintf(stderr,"Dense support not compiled\n");
	exit(0);
#endif
	
      }
    
      
      //printf("Valid %d\n",localV->len);
      if(!localV->len){
	int progCount=doneCount.increment();
	image_count_verbose (progCount, totalTodoCount);
	return false;
      }
      double mult=0.00;
      
      surf = auv_mesh_pts(localV,mult,0); 
    }
    
   
    FILE *fp = fopen(meshfilename, "w" );
    auv_write_ply(surf, fp,have_cov_file,"test");
    fflush(fp);   
    fclose(fp);
    //Destory Surf
 
  }


  if(output_3ds){
    char fname_3ds[255];
    surf=  gts_surface_new(gts_surface_class(),
			   (GtsFaceClass *)t_face_class(), 
			   gts_edge_class(), t_vertex_class());
    TriMesh::verbose=0;
    TriMesh *mesh = TriMesh::read(meshfilename);
    convert_ply(  mesh ,surf,0);
   


    gts_surface_foreach_vertex (surf, (GtsFunc) gts_point_transform, name.m);

   

    map<int,string>textures;
    textures[0]=(name.dir+name.left_name);
    sprintf(fname_3ds,"mesh/surface-%08d.3ds",
	    name.id);
	     
    std::map<int,GtsMatrix *> gts_trans;
    GtsMatrix *invM = gts_matrix_inverse(name.m);
    gts_trans[0]=(invM);
    gen_mesh_tex_coord(surf,&calib->left_calib,gts_trans,
		       NULL,tex_size,num_threads,0,0);

    GtsVector v;
    v[0]=-1;
    v[1]=0;    
    v[2]=0;
    GtsMatrix *rot= gts_matrix_rotate(NULL,v,M_PI);
    gts_surface_foreach_vertex (surf, (GtsFunc) gts_point_transform, rot);

    std::vector<string> lodnames;
 
    osgExp->Export3DS(surf,fname_3ds,textures,512,NULL);
    gts_matrix_destroy (invM);
  }

  if(surf)
    gts_object_destroy (GTS_OBJECT (surf)); 
  
  if(output_ply_and_conf){
    TriMesh::verbose=0;
    TriMesh *mesh = TriMesh::read(meshfilename);
    edge_len_thresh(mesh,edgethresh);
    
    xform xf(name.m[0][0],name.m[1][0],name.m[2][0],name.m[3][0],
	     name.m[0][1],name.m[1][1],name.m[2][1],name.m[3][1],
	     name.m[0][2],name.m[1][2],name.m[2][2],name.m[3][2],
	     name.m[0][3],name.m[1][3],name.m[2][3],name.m[3][3]);
  
    apply_xform(mesh,xf);
    mesh->need_bbox();
    
    mesh->write(filename);

    if(use_poisson_recon){
      
      dump_pts(pos_fp,filename,clean_pos_pts);
      /*
     
      for(fi=cm.face.begin(); fi!=cm.face.end();++fi){
      
	for (int i = 0; i < nv; i++) {
	for(int j=0; j<3; j++){
	if(j==2){
	   
	buf[j]=(float)mesh->vertices[i][j];
	}else{
	buf[j]=(float)mesh->vertices[i][j];
	}
	printf("%f ",buf[j]);
	}
	printf("\n");

	for(int j=0; j<3; j++)
	buf[j+3]=(float)mesh->normals[i][j];

       }
      */
    }
      name.mesh_name = osgDB::getSimpleFileName(filename);

    gts_bbox_set(name.bbox,NULL,
		 mesh->bbox.min[0],
		 mesh->bbox.min[1],
		 mesh->bbox.min[2],
		 
		 mesh->bbox.max[0],
		 mesh->bbox.max[1],
		 mesh->bbox.max[2]);

  }
  


	 
  if(!meshcached){
    
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
    
  }

  int progCount=doneCount.increment();
  image_count_verbose (progCount, totalTodoCount);
  // printf("\rStereo processing on image %u/%u complete.",progCount,totalTodoCount);
  //fflush(stdout);
  // doneCount.increment();
      
  return true;
}



void runC(Stereo_Pose_Data &name){
 
  //printf("%s Written Out to  %s\n",name.left_name.c_str(),name.mesh_name.c_str());
}
bool gen_stereo_from_mono(std::vector<Mono_Image_Name> &mono_names,Slices &tasks,Camera_Calib *cam_calib,unsigned int &stereo_pair_count){

  for(int i=0; i <(int) mono_names.size()-mono_skip; i+=3){
    Stereo_Pose_Data name;
    name.time=mono_names[i].time;
    //name.index=mono_names[i].index;
    name.pose[AUV_POSE_INDEX_X] = (*mono_names[i].pose)(AUV_POSE_INDEX_X);
    name.pose[AUV_POSE_INDEX_Y] = (*mono_names[i].pose)(AUV_POSE_INDEX_Y);
    name.pose[AUV_POSE_INDEX_Z] = (*mono_names[i].pose)(AUV_POSE_INDEX_Z);
    name.pose[AUV_POSE_INDEX_PHI] = (*mono_names[i].pose)(AUV_POSE_INDEX_PHI);
    name.pose[AUV_POSE_INDEX_THETA]=(*mono_names[i].pose)(AUV_POSE_INDEX_THETA);
    name.pose[AUV_POSE_INDEX_PSI] = (*mono_names[i].pose)(AUV_POSE_INDEX_PSI);
    name.m =gts_matrix_identity (NULL);
    name.bbox = gts_bbox_new(gts_bbox_class(),NULL,0,0,0,0,0,0);
    name.left_name=mono_names[i].img_name;
    name.right_name=mono_names[i+mono_skip].img_name;
    name.calib=new Stereo_Calib(*cam_calib,*mono_names[i].pose,*mono_names[i+mono_skip].pose);
    name.overlap=false;
    name.valid=true;
    name.radius=5;
    name.alt=-1.0;
    name.id=stereo_pair_count++;
    fill_gts_matrix(name.pose,name.m);
    tasks.push_back(name);
    
  }
    
  return true;
}

int main( int argc, char *argv[ ] )
{
  FILE *rerunfp=fopen("rerun.sh","w");
  fprintf(rerunfp,"#!/bin/bash\n");
  for(int i=0; i < argc; i++)
    fprintf(rerunfp,"%s ",argv[i]);
  fprintf(rerunfp,"\n");;
  fchmod(fileno(rerunfp),0777);
  fclose(rerunfp);
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
  //cout << "Binary Path " <<basepath <<endl;
 
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
  
  

  auv_data_tools::makedir(aggdir);

  chmod(aggdir,   0777);


  auv_data_tools::makedir(dicedir);
  chmod(dicedir,   0777);

  auv_data_tools::makedir(uname);

  chmod(uname,   0777);

  auv_data_tools::makedir("mesh-pos");

  chmod("mesh-pos",   0777);

  auv_data_tools::makedir(cachedmeshdir);
  chmod(cachedmeshdir,   0777);
  auv_data_tools::makedir(cachedtexdir);
  chmod(cachedtexdir,   0777);
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
  
  if(use_poisson_recon){
    pos_fp=fopen("mesh-agg/pos_pts.bnpts","wb");
    if(!pos_fp){      
      fprintf(stderr,"Can't open mesh-agg/pos_pts.bnpts\n");
      exit(-1);    
    }
  }
 
  //
 
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

  unsigned int mono_img_count=0;
  int start_skip=0;
  if(mono_cam){
    std::vector<Mono_Image_Name> mono_names;
    while( !have_max_frame_count || mono_img_count < max_frame_count*(mono_skip+1) ){
      
      Mono_Image_Name mono;
      int ret=get_mono_image_name( dir_name, contents_file, mono) ;
      if(ret == ADD_IMG ){
	if(start_skip++ < single_run_start)
	  continue;
	mono.id= single_run_start+mono_img_count++;
	mono.valid=true;
	mono_names.push_back(mono);
      }else if(ret == NO_ADD){
	printf("Noadd\n");
	continue;
      }else if(ret == END_FILE)
	break;
    }
     
    string config_dir_name;
    Stereo_Calib *calib=NULL;
    int slash_pos = stereo_config_file_name.rfind( "/" );
    if( slash_pos != -1 )
      config_dir_name = stereo_config_file_name.substr( 0, slash_pos+1 );
    
    config_file= new Config_File(stereo_config_file_name.c_str());
  
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
    gen_stereo_from_mono(mono_names,tasks,&calib->left_calib,stereo_pair_count);
    
    
  }else{
    while( !have_max_frame_count || stereo_pair_count < max_frame_count ){
      
      Stereo_Pose_Data name;
      int ret=get_auv_image_name( dir_name, contents_file, name) ;
      
      
      if(ret == ADD_IMG ){
	if(start_skip++ < single_run_start)
	  continue;
	name.id= single_run_start+stereo_pair_count++;
	name.valid=true;
	tasks.push_back(name);
      }else if(ret == NO_ADD){
	printf("Noadd\n");
	continue;
      }else if(ret == END_FILE)
	break;
    }
  }


  if(tasks.size() <= 0){
    fprintf(stderr,"No tasks loaded check %s\n",contents_file_name.c_str());
    exit(-1);
  }
  start_time=tasks[0].time;
  stop_time=tasks[tasks.size()-1].time;
  
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
  for(Slices::iterator itr=tasks.begin(); itr != tasks.end(); itr++){
    Slice name=(*itr);
    fprintf(fpp,"%f",   
	    name.time);
    fprintf(fpp,"%f %f %f %f %f %f",name.bbox->x1,name.bbox->y1,name.bbox->z1,
	    name.bbox->x2,name.bbox->y2,name.bbox->z2);
    
    for(int i=0; i< 4; i++){
      for(int j=0; j < 4; j++){
	fprintf(fpp," %f",name.m[i][j]);
      }
    }
    fprintf(fpp,"\n");
  }

  if(!single_run){
    
    char conf_name[255];
    
    sprintf(conf_name,"%s/meshes.txt",aggdir);
    
    
    conf_ply_file=fopen(conf_name,"w");
    if(!conf_ply_file){
      fprintf(stderr,"Can't open %s\n",conf_name);
      exit(-1);
    }
      
    chmod(conf_name,0666);
  
    int valid=0;
    for(unsigned int i=0; i < tasks.size(); i++){
      if(tasks[i].valid){
	fprintf(conf_ply_file,
		"surface-%s.tc.ply %f 1\n"
		,osgDB::getStrippedName(tasks[i].left_name).c_str(),vrip_res);
	valid++;
      }
      
    }
 
    fclose(conf_ply_file);

    if(use_poisson_recon){
      fclose(pos_fp);
    }

    if(valid <= 0){
      fprintf(stderr,"No valid meshes bailing\n");
      exit(-1);
    }
    FILE *vrip_seg_fp;
    char vrip_seg_fname[255];
    FILE *bboxfp;
    FILE *vripcmds_fp=fopen("mesh-agg/vripcmds","w");
    FILE *diced_fp=fopen("mesh-diced/diced.txt","w");

    if(!vripcmds_fp){
      printf("Can't open vripcmds\n");
      exit(-1);
    }
    char cwd[255];
    if(!dist_run)
      getcwd(cwd,255);
    else
      strcpy(cwd,"/mnt/shared/");


    const char *texlogdir="/mnt/shared/log-tex";
    const char *simplogdir="/mnt/shared/log-simp";
    const char *pos_simp_log_dir="/mnt/shared/log-possimp";
    const char *vriplogdir="/mnt/shared/log-vrip";
    
    float simp_mult=1.0;
    
    if(have_mb_ply)
      simp_mult=1.0;
    else
      simp_mult=2.0;
    
    ShellCmd shellcm(basepath.c_str(),simp_mult,pos_simp_log_dir,dist_run,cwd,aggdir,have_mb_ply);
    std::vector<Cell_Data> cells;
    if(even_split)
      cells=calc_cells(tasks,EVEN_SPLIT,cell_scale);
    else
      cells=calc_cells(tasks,AUV_SPLIT,cell_scale);

    for(int i=0; i <(int)cells.size(); i++){
      if(cells[i].poses.size() == 0)
	continue;
      
      sprintf(vrip_seg_fname,"mesh-agg/vripseg-%08d.txt",i);
      sprintf(conf_name,"mesh-diced/bbox-clipped-diced-%08d.ply.txt",i);

      vrip_seg_fp=fopen(vrip_seg_fname,"w");
      bboxfp = fopen(conf_name,"w");
      if(!vrip_seg_fp || !bboxfp){
	printf("Unable to open %s\n",vrip_seg_fname);
      }	
      char redirstr[255];
      if(!dist_run)
	sprintf(redirstr,">  vripsurflog-%08d.txt",i);
      else
	sprintf(redirstr," ");

      fprintf(diced_fp,"clipped-diced-%08d.ply\n",i);
      fprintf(vripcmds_fp,"set BASEDIR=\"%s\"; set OUTDIR=\"mesh-agg/\";set VRIP_HOME=\"$BASEDIR/vrip\";setenv VRIP_DIR \"$VRIP_HOME/src/vrip/\";set path = ($path $VRIP_HOME/bin);cd %s/$OUTDIR;",basepath.c_str(),cwd);

      if(have_mb_ply){
	fprintf(vripcmds_fp,"plycullmaxx %f %f %f %f %f %f %f < ../mesh-agg/mb.ply > ../mesh-agg/clipped-mb-%08d.ply;set VISLIST=`cat ../%s | grep surface |cut -f1 -d\" \"`; plyclipbboxes -e %f $VISLIST ../mesh-agg/clipped-mb-%08d.ply > ../mesh-agg/vis-mb-%08d.ply;plyclipbboxes -e %f $VISLIST ../mesh-agg/clipped-mb-%08d.ply > ../mesh-agg/sub-mb-%08d.ply;", cells[i].bounds.min_x,
		cells[i].bounds.min_y,
		FLT_MIN,
		cells[i].bounds.max_x,
		cells[i].bounds.max_y,
		FLT_MAX,
		eps,i,vrip_seg_fname,2.0,i,i,1.0,i,i);
      }
      if(!no_merge)
	fprintf(vripcmds_fp,"$BASEDIR/vrip/bin/vripnew auto-%08d.vri ../%s ../%s %f -rampscale %f;$BASEDIR/vrip/bin/vripsurf auto-%08d.vri ../mesh-agg/seg-%08d.ply %s ;",i,vrip_seg_fname,vrip_seg_fname,vrip_res,vrip_ramp,i,i,redirstr);
      else
	fprintf(vripcmds_fp,"cat ../%s | cut -f1 -d\" \" | xargs $BASEDIR/vrip/bin/plymerge > ../mesh-agg/seg-%08d.ply;",vrip_seg_fname,i);

      fprintf(vripcmds_fp,"plycullmaxx %f %f %f %f %f %f %f < ../mesh-agg/seg-%08d.ply > ../mesh-diced/clipped-diced-%08d.ply;",
	      cells[i].bounds.min_x,
	      cells[i].bounds.min_y,
	      FLT_MIN,
	      cells[i].bounds.max_x,
	      cells[i].bounds.max_y,
	      FLT_MAX,
	      eps,i,i);
    
      /* if(have_mb_ply){
	 fprintf(vripcmds_fp,"mv ../mesh-diced/clipped-diced-%08d.ply ../mesh-diced/nomb-diced-%08d.ply; plymerge ../mesh-diced/nomb-diced-%08d.ply ../mesh-agg/inv-mb-%08d.ply > ../mesh-diced/clipped-diced-%08d.ply;",i,i,i,i,i);
    
	 }*/
      fprintf(vripcmds_fp,"cd ..\n");

   

      for(unsigned int j=0; j <cells[i].poses.size(); j++){
	const Stereo_Pose_Data *pose=cells[i].poses[j];
	//Vrip List
	fprintf(vrip_seg_fp,"%s %f 1\n",pose->mesh_name.c_str(),vrip_res);
	//Gen Tex File bbox
	fprintf(bboxfp, "%d %s " ,pose->id,pose->left_name.c_str());
	save_bbox_frame(pose->bbox,bboxfp);
	for(int k=0; k < 4; k++)
	  for(int n=0; n < 4; n++)
	    fprintf(bboxfp," %lf",pose->m[k][n]);
	fprintf(bboxfp,"\n");
      }
      if(have_mb_ply)
	fprintf(vrip_seg_fp,"vis-mb-%08d.ply  0.1 0\n",i);

      fclose(vrip_seg_fp);
      fclose(bboxfp);
    }
    fclose(vripcmds_fp);
    fclose(diced_fp);
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
    
    {
      if(output_pts_cov)
	fclose(pts_cov_fp);
      if(gen_mb_ply){

	FILE *genmbfp=fopen("genmb.sh","w");
	fprintf(genmbfp,"#!/bin/bash\nPATH=$PATH:%s/tridecimator:/usr/lib/gmt/bin/:%s../mbsystems/bin/\ncd %s\n"
		"if [ -e %s/mb.ply ]; then\n"
		"cp %s/mb.ply .\n"
		"exit 0;\n"
		"fi\n"
		"find . -name 'mb*.ply' | xargs rm -f\n"
		"%s/../seabed_localisation/bin/copy_deltaT %s %s %s\n",
		basepath.c_str(),	basepath.c_str(),
		aggdir,deltaT_dir.c_str(),deltaT_dir.c_str(),basepath.c_str(),
		//--start %f --stop %fstart_time,stop_time,
		deltaT_config_name.c_str(),deltaT_dir.c_str(),
		deltaT_pose.c_str());
	fprintf(genmbfp,"ls -1 | grep .gsf$ > tmplist\n"
		"mbdatalist -F-1 -I tmplist > formattedlist\n"
		"mbdatalist -F-1 -I formattedlist -N\n"
		"mbm_grid -Iformattedlist  -C3 -E1.0/1.0/m\n"
		"./formattedlist_mbgrid.cmd\n"
		"grd2xyz -S formattedlist.grd  -bo > tmp.xyz\n"
		"%s/../seabed_localisation/bin/mbstomesh %s tmp.xyz\n"
		"mbm_grdtiff -I formattedlist.grd -W1/1 -G5 -A0.25/280/15  -Q -V\n"
		"./formattedlist.grd_tiff.cmd\n"
		"tridecimator mb.ply mb-tmp.ply 0 -e500%%\n"
		"mv mb.ply mb-uncleaned.ply\n"
		"mv mb-tmp.ply mb.ply\n"
		"cp mb.ply %s/ \n",
		basepath.c_str(),deltaT_config_name.c_str(),
		deltaT_dir.c_str());
	fchmod(fileno(genmbfp),   0777);
	fclose(genmbfp);
	system("./genmb.sh");
      }
  
      /* if(!single_run && have_mb_ply){
	 for(int i=0; i < (int)mb_ply_filenames.size(); i++){
	 string simpname =osgDB::getSimpleFileName(mb_ply_filenames[i]);
	 string inname=mb_ply_filenames[i];
	 string outname=(string(aggdir)+string("/")+simpname);
	 cout << "Copying " << inname << " to " << outname<<endl;
	 std::ifstream  IN (inname.c_str());
	 std::ofstream  OUT(outname.c_str()); 
	 OUT << IN.rdbuf();
	 fprintf(conf_ply_file,
	 "bmesh %s .1 0\n"
	 ,simpname.c_str()); 
	 }
	 }
      */
      
      if(use_poisson_recon && !no_merge){
	conf_ply_file=fopen("./runpos.sh","w+"); 
	fprintf(conf_ply_file,"#!/bin/bash\nBASEPATH=%s/\nOUTDIR=$PWD/%s\n"
		"VRIP_HOME=%s/vrip\nexport VRIP_DIR=$VRIP_HOME/src/vrip/\n"
		"PATH=$PATH:$VRIP_HOME/bin:%s/tridecimator:%s/poisson/\n"
		"cd mesh-agg/\n",
		basepath.c_str(),aggdir,basepath.c_str(),basepath.c_str(),
		basepath.c_str());
	fprintf(conf_ply_file, "cat pos_pts.bnpts > pos_out.bnpts\n");

	if(have_mb_ply)
	  fprintf(conf_ply_file,"%s/poisson/dumpnormpts %s mb.bnpts -flip\n"
		  "cat mb.bnpts >> pos_out.bnpts\n",basepath.c_str(),
		  osgDB::getSimpleFileName( mb_ply_filenames[0]).c_str());
	int mintridepth;
	if(have_mb_ply)
	  mintridepth=0;
	else
	  mintridepth=8;
	if( no_pos_clip){
	  fprintf(conf_ply_file,"PoissonRecon --binary --depth %d --in pos_out.bnpts --solverDivide %d --samplesPerNode %f --verbose  --out ../mesh-pos/pos_rec-lod2.ply\n",8,6,1.0);
	  fprintf(conf_ply_file,"PoissonRecon --binary --depth %d --in pos_out.bnpts --solverDivide %d --samplesPerNode %f --verbose  --out ../mesh-pos/pos_raw.ply\n",11,6,4.0);
	}else{
	  fprintf(conf_ply_file,"PoissonRecon --binary --depth %d --in pos_out.bnpts --solverDivide %d --samplesPerNode %f --verbose --mintridepth %d --out ../mesh-pos/pos_rec-lod2.ply\n",8,6,1.0,mintridepth-2);
	  fprintf(conf_ply_file,"PoissonRecon --binary --depth %d --in pos_out.bnpts --solverDivide %d --samplesPerNode %f --verbose --mintridepth %d --out ../mesh-pos/pos_raw.ply\n",11,6,4.0,mintridepth);
	}

	fprintf(conf_ply_file,"$BASEPATH/tridecimator/tridecimator ../mesh-pos/pos_raw.ply ../mesh-pos/pos_rec-lod0.ply 0 -e15.0\n");
	fprintf(conf_ply_file,"$BASEPATH/tridecimator/tridecimator ../mesh-pos/pos_raw.ply ../mesh-pos/pos_rec-lod1.ply 0 -e15.0\n");
	fchmod(fileno(conf_ply_file),0777);
	fclose(conf_ply_file);
	if(run_pos){
	  system("./runpos.sh");
	}
	shellcm.pos_dice(cells,eps,run_pos);
	//shellcm.pos_simp_cmd(use_poisson_recon);
	
      }else{
	conf_ply_file=fopen("./runpos.sh","w+"); 
	fprintf(conf_ply_file,"#!/bin/bash\nBASEPATH=%s/\nOUTDIR=$PWD/%s\n"
		"VRIP_HOME=%s/vrip\nexport VRIP_DIR=$VRIP_HOME/src/vrip/\n"
		"PATH=$PATH:$VRIP_HOME/bin:%s/tridecimator:%s/poisson/\n"
		"cd mesh-agg/\n",
		basepath.c_str(),aggdir,basepath.c_str(),basepath.c_str(),
		basepath.c_str());
	fprintf(conf_ply_file,"cat meshes.txt| cut -f1 -d\" \" | xargs $BASEPATH/vrip/bin/plymerge  > ../mesh-pos/pos_rec-lod0.ply\n");
	fprintf(conf_ply_file,"cp ../mesh-pos/pos_rec-lod0.ply ../mesh-pos/pos_rec-lod1.ply\n");
	fprintf(conf_ply_file,"cp ../mesh-pos/pos_rec-lod0.ply ../mesh-pos/pos_rec-lod2.ply\n");
	fchmod(fileno(conf_ply_file),0777);
	fclose(conf_ply_file);
	if(run_pos){
	  system("./runpos.sh");
	  shellcm.pos_dice(cells,eps);
	}
      }
      

      if(!single_run){
	conf_ply_file=fopen("./runvrip.sh","w+"); 
	fprintf(conf_ply_file,"#!/bin/bash\nBASEPATH=%s/\nOUTDIR=$PWD/%s\nVRIP_HOME=%s/vrip\nexport VRIP_DIR=$VRIP_HOME/src/vrip/\nPATH=$PATH:$VRIP_HOME/bin:%s/tridecimator\n cd mesh-agg/\necho -e 'Vriping...\\n'\n",basepath.c_str(),aggdir,basepath.c_str(),basepath.c_str());
	fprintf(conf_ply_file,"VRIPLOGDIR=%s\n"
		,vriplogdir);
	fprintf(conf_ply_file,"find . -name 'mb*.ply' | sort  > mbmeshes.txt\n");
	if(have_mb_ply)
	  fprintf(conf_ply_file,"cat meshes.txt| cut -f1 -d\" \" | xargs $BASEPATH/vrip/bin/plymerge  > unblended.ply\n$BASEPATH/tridecimator/tridecimator $OUTDIR/unblended.ply $OUTDIR/unblended.stl 0 -F\n"
		  "cat mbmeshes.txt | xargs $BASEPATH/vrip/bin/plymerge  > joined-mb.ply\n"
		  "echo -e \"1.0 0.0 0.0 0.0\\n0.0 1.0 0.0 0.0\\n0.0 0.0 1.0 0.0\\n0.0 0.0 0.0 1.0\\n\" > unblended.xf\necho -e \"1.0 0.0 0.0 0.0\\n0.0 1.0 0.0 0.0\\n0.0 0.0 1.0 0.0\\n0.0 0.0 0.0 1.0\\n\" > joined-mb.xf\n#auv_mesh_align unblended.ply joined-mb.ply\n#$BASEPATH/vrip/bin/plyxform -f joined-mb.xf  < joined-mb.ply > mb.ply\n");

	fprintf(conf_ply_file,"cd ..\n");

	if(dist_run){
	  fprintf(conf_ply_file,"time $BASEPATH/vrip/bin/loadbalance ~/loadlimit mesh-agg/vripcmds -logdir $VRIPLOGDIR\n");
	}else{
	  fprintf(conf_ply_file,"time %s/runtp.py mesh-agg/vripcmds\n",basepath.c_str());
	}
	if(have_mb_ply){
	  fprintf(conf_ply_file,"cd mesh-agg\n");
	  fprintf(conf_ply_file,"if [ -e sub-mb-%08d.ply ]; then\n"
		  "\tplysubtract mb.ply sub-mb-%08d.ply > inv-mb-%08d.ply\n"
		  "else\n"
		  "\tcp  mb.ply inv-mb-%08d.ply\n"
		  "fi\n",0,0,0,0); 
	  if(cells.size()-1 > 0){
	    fprintf(conf_ply_file,"for f in `echo {1..%d}`\n"
		  "do\n"
		  "i=`printf \"%%08d\\n\" \"$f\"`\n"
		  "ilast=`printf \"%%08d\" \"$(($f - 1 ))\"`\n"
		  "if [ -e sub-mb-$i.ply ]; then\n"
		  "\tplysubtract inv-mb-$ilast.ply sub-mb-$i.ply > inv-mb-$i.ply;\n"
		  "else\n"
		  "\tcp inv-mb-$ilast.ply  inv-mb-$i.ply\n" 
		  "fi\n"
		    "done\n",cells.size()-1);
	  }

	  fprintf(conf_ply_file,
		  "tridecimator inv-mb-$i.ply ../mesh-diced/inv-mb.ply 0 -F\n");

	    
	}
	

	 
      
	fchmod(fileno(conf_ply_file),0777);
	fclose(conf_ply_file);
	if(!no_vrip && use_vrip_recon)
	  system("./runvrip.sh");
      
	FILE *dicefp=fopen("./simp.sh","w+");
	fprintf(dicefp,"#!/bin/bash\necho -e 'Simplifying...\\n'\nBASEPATH=%s/\nVRIP_HOME=$BASEPATH/vrip\nMESHAGG=$PWD/mesh-agg/\nexport VRIP_DIR=$VRIP_HOME/src/vrip/\nPATH=$PATH:$VRIP_HOME/bin\nRUNDIR=$PWD\nDICEDIR=$PWD/mesh-diced/\nmkdir -p $DICEDIR\ncd $MESHAGG\n",basepath.c_str());
	fprintf(dicefp,"cd $DICEDIR\n");
	fprintf(dicefp,"NUMDICED=`wc -l diced.txt |cut -f1 -d\" \" `\n"  
		"REDFACT=(%f %f %f)\n",simp_res[0],simp_res[1],simp_res[2]);
	 
     
	//	fprintf(conf_ply_file,//"if [ -e clipped-diced-%08d.ply ]; then\n"
	//	"\tplysubtract mb.ply sub-mb-%08d.ply > inv-mb-%08d.ply\n"
	//"else\n"
	//"\tcp  mb.ply inv-mb-%08d.ply\n"
	//"fi\n",0,0,0,0); 

	
	char simpargstr[255];
	if(further_clean)
	  sprintf(simpargstr," -H%f -S%f ",hole_fill_size,connected_comp_size_clean);
	else
	  sprintf(simpargstr," ");
	fprintf(dicefp, "LOGDIR=%s\n"
		"if [[ -d $LOGDIR ]] ; then\n"
		"find $LOGDIR -name 'loadbal*' | xargs rm &>/dev/null\nfi\n"
		"if [[ -d $DICEDIR ]] ; then\n"
		"find $DICEDIR -name '*lod*' | xargs rm &> /dev/null\nfi\n"
		"rm -f simpcmds\n"
		"rm -f valid.txt\n"
		"cat diced.txt | while read MESHNAME; do\n"
		"FACES=`plyhead $MESHNAME | grep face | cut -f 3 -d\" \"`\n"
		"if [ $FACES == 0 ]; then\n continue;\n fi\n"
		"echo $MESHNAME >> valid.txt\n"
		"SIMPCMD=\"cd $DICEDIR/\" \n"
		"\tfor f in `echo {0..2}`\n"
		"\tdo\n"
		"\t\tif [ $f == 0 ]; then\n"
		"\t\t\tNEWNAME=`echo $MESHNAME | sed s/.ply/-lod$f.ply/g`\n"
		"FLIPCMD=\"-F\"\n"
		"\t\telse\n"
		"FLIPCMD="
		"\t\t\tNEWNAME=`echo $MESHNAME | sed s/-lod$(($f - 1 )).ply/-lod$f.ply/g`\n"
		"\t\tfi\n"
		"\t\tSIMPCMD=$SIMPCMD\";\"\"$BASEPATH/tridecimator/tridecimator $MESHNAME $NEWNAME ${REDFACT[$f]}r -b2.0 $FLIPCMD %s >& declog-$MESHNAME.txt ;chmod 0666 $NEWNAME  \"\n"
		"MESHNAME=$NEWNAME\n"
		"\tdone\n"
		"echo $SIMPCMD >> simpcmds\n"
		"done\n",simplogdir,simpargstr);
	
	
	if(dist_run){
	  fprintf(dicefp,"cd $DICEDIR\n"
		  "time $BASEPATH/vrip/bin/loadbalance ~/loadlimit simpcmds -logdir $LOGDIR\n");
	} else {
	  fprintf(dicefp,"time %s/runtp.py simpcmds\n",basepath.c_str());
	}
	if(have_mb_ply){
	  float mbres[3]={0,0,0};
	  fprintf(dicefp,"echo inv-mb.ply >> valid.txt\n");
	  for(int k=0; k < 3; k++)
	    fprintf(dicefp,"%s/tridecimator/tridecimator inv-mb.ply inv-mb-lod%d.ply %fr\n",basepath.c_str(),k,mbres[k]);
	}
	fprintf(dicefp,"cat valid.txt | xargs plybbox > range.txt\n");
	/*	fprintf(conf_ply_file,"for f in `echo {1..%d}`\n"
		"do\n"
		"i=`printf \"%%08d\\n\" \"$f\"`\n"
		"ilast=`printf \"%%08d\" \"$(($f - 1 ))\"`\n"
		"if [ -e clipped-diced-$i.ply ]; then\n"
		"\tplysubtract -t 0.4 clipped-diced-$ilast-lod0.ply clipped-diced-$i-lod0.ply > cd-$ilast-lod0.ply;\n"
		"mv   clipped-diced-$ilast-lod0.ply unsub-clipped-diced-$ilast-lod0.ply\n"
		"cp  cd-$ilast-lod0.ply  clipped-diced-$ilast-lod0.ply\n"
		"else\n"
		"\tcp clipped-diced-$ilast-lod0.ply  cd-$ilast-lod0.ply\n" 
		"fi\n"
		"done\n",cells.size());
	*/

	fchmod(fileno(dicefp),0777);
	fclose(dicefp);
	if(!no_simp && use_vrip_recon)
	  system("./simp.sh");
	vector<string> gentexnames;
	gentexnames.push_back("./gentex.sh");
	gentexnames.push_back("./posgentex.sh");

	vector<string> gentexdir;
	gentexdir.push_back("mesh-diced");
	gentexdir.push_back("mesh-pos");
	
	for(int i=0; i <2; i++){
	 

	  dicefp=fopen(gentexnames[i].c_str(),"w+");
	  fprintf(dicefp,"#!/bin/bash\necho 'TexGen...\n'\nBASEPATH=%s/\nVRIP_HOME=$BASEPATH/vrip\nMESHAGG=$PWD/mesh-agg/\nexport VRIP_DIR=$VRIP_HOME/src/vrip/\nPATH=$PATH:$VRIP_HOME/bin\nRUNDIR=$PWD\nDICEDIR=$PWD/%s\nmkdir -p $DICEDIR\ncd $DICEDIR\n",basepath.c_str(),gentexdir[i].c_str());

	  char argstr[255];
	  strcpy(argstr,"");
	  if(do_novelty)
	    strcat(argstr," --novelty ");
	  if(do_hw_blend)
	    strcat(argstr," --blend ");
	  if(!hardware_compress)
	    strcat(argstr," --no-hardware-compress ");
	  if(no_simp)
	    strcat(argstr," --nosimp");
	  if(have_mb_ply){
	    char tp[255];
	    sprintf(tp," --nonvis %d ",(int)cells.size());
	    strcat(argstr,tp);
	  }

	  if(no_simp)
	    fprintf(dicefp,"cat diced.txt | xargs plybbox > range.txt;cp diced.txt valid.txt\n");

	  if(!sing_gen_tex){
	    fprintf(dicefp,"cd $DICEDIR\n"
		    "NUMDICED=`wc -l valid.txt |cut -f1 -d\" \" `\n"
		    "GENTEX_RANGE=%d\n"
		    "rm -f gentexcmds\n"
		    "for i in `seq 0 $GENTEX_RANGE $(($NUMDICED - 1))`;\n"
		    "do\n"
		    "\techo \"setenv DISPLAY :0.0;cd $DICEDIR/..;$BASEPATH/genTex %s %s -f %s  --dicedir %s/ --range-run $i $(($i + $GENTEX_RANGE)) %s\" >> gentexcmds\n"
		    "done\n"
		    "LOGDIR=%s\n"
		    "cd $DICEDIR\n"
		    ,dist_gentex_range,stereo_config_file_name.c_str(),recon_config_file_name.c_str(),cachedtexdir,gentexdir[i].c_str(),argstr,texlogdir);
	    if(dist_run)
	      fprintf(dicefp,
		      "time $BASEPATH/vrip/bin/loadbalance ~/loadlimit-hwcard gentexcmds -logdir $LOGDIR\n");
		
	    else
	      fprintf(dicefp,"time %s/runtp.py gentexcmds\n",basepath.c_str());
	    
	  }else{  
	    fprintf(dicefp,"cd $RUNDIR\ntime %s/genTex %s %s -f %s ",basepath.c_str(),stereo_config_file_name.c_str(),recon_config_file_name.c_str(),cachedtexdir);
	    
	    fprintf(dicefp,"%s \n",argstr);
	    
	  }

	  fchmod(fileno(dicefp),0777);
	  fclose(dicefp);
	  if(no_gen_tex)
	    continue;
	  if(i==0 && use_vrip_recon)
	    system(gentexnames[i].c_str());
	  if(i==1 && !use_poisson_recon)
	    system(gentexnames[i].c_str());
	   
	}
	/*	if(use_poisson_recon){
	  dicefp=fopen("./posgentex.sh","w+");
	  fprintf(dicefp,"#!/bin/bash\necho 'TexGen...\n'\nBASEPATH=%s/\nVRIP_HOME=$BASEPATH/vrip\nMESHAGG=$PWD/mesh-agg/\nexport VRIP_DIR=$VRIP_HOME/src/vrip/\nPATH=$PATH:$VRIP_HOME/bin\nRUNDIR=$PWD\nDICEDIR=$PWD/mesh-diced/\nmkdir -p $DICEDIR\ncd $MESHAGG\n",basepath.c_str());
	  
	  char argstr[255];
	  strcpy(argstr,"");
	  if(do_novelty)
	    strcat(argstr," --novelty ");
	  if(do_hw_blend)
	    strcat(argstr," --blend ");
	  if(!hardware_compress)
	    strcat(argstr," --no-hardware-compress ");
	  if(have_mb_ply){
	    char tp[255];
	    sprintf(tp," --nonvis %d ",(int)cells.size());
	    strcat(argstr,tp);
	  }
	  
	  if(!sing_gen_tex){
	    fprintf(dicefp,"cd $DICEDIR\n"
		    "NUMDICED=`wc -l valid.txt |cut -f1 -d\" \" `\n"
		    "rm -f gentexcmds\n"
		    "for i in `echo {0..$(($NUMDICED - 1))}`;\n"
		    "do\n"
		    "\techo \"setenv DISPLAY :0.0;cd $DICEDIR/..;$BASEPATH/genTex --dicedir mesh-pos/ %s -f %s --single-run $i %s\" >> gentexcmds\n"
		    "done\n"
		    "LOGDIR=%s\n"
		    "cd $DICEDIR\n"
		    ,stereo_config_file_name.c_str(),cachedtexdir,argstr,texlogdir);
	    if(dist_run)
	      fprintf(dicefp,
		      "time $BASEPATH/vrip/bin/loadbalance ~/loadlimit-hwcard gentexcmds -logdir $LOGDIR\n");
	    
	    else
	      fprintf(dicefp,"time %s/runtp.py gentexcmds\n",basepath.c_str());
	    
	  }else{  
	    fprintf(dicefp,"cd $RUNDIR\ntime %s/genTex --dicedir mesh-pos/ --margins %d %d %d %s -f %s ",basepath.c_str(),texmargin[0],texmargin[1],texmargin[2],stereo_config_file_name.c_str(),cachedtexdir);
	    
	    fprintf(dicefp,"%s \n",argstr);
	    
	  }
	  
	  fchmod(fileno(dicefp),0777);
	  fclose(dicefp);
	  if(!no_gen_tex && run_pos)
	    system("./posgentex.sh");
	    }*/
	if(!no_gen_tex || use_poisson_recon){
	  FILE *lodfp=fopen("lodgen.sh","w");
	  char ar[255];
	  if(run_pos)
	    strcpy(ar,"--dicedir mesh-pos/");
	  else
	    strcpy(ar,"--dicedir mesh-diced/");
	  fprintf(lodfp,"#!/bin/bash\n"
		  "echo LODGen... \n"
		  "%s/lodgen %s\n",
		  basepath.c_str(),ar);
	 
	  fchmod(fileno(lodfp),0777);
	  fclose(lodfp);
	  if(!no_gen_tex )
	    system("./lodgen.sh");
	}
	
	{
	  char dicedir[255];
	  if(run_pos)
	    strcpy(dicedir,"mesh-pos/");
	  else
	    strcpy(dicedir,"mesh-diced/");

	  FILE *rgfp=fopen("regen.sh","w");
	  fprintf(dicefp,"#!/bin/bash\necho 'Regen...\n'\nBASEPATH=%s/\nVRIP_HOME=$BASEPATH/vrip\nMESHAGG=$PWD/mesh-agg/\nexport VRIP_DIR=$VRIP_HOME/src/vrip/\nPATH=$PATH:$VRIP_HOME/bin\nRUNDIR=$PWD\nDICEDIR=$PWD/mesh-diced/\nmkdir -p $DICEDIR\n",basepath.c_str());
	  fprintf(rgfp,"mkdir -p mesh-regen-tex\n"
		  "chmod 777 mesh-regen-tex\n"
		  "cd mesh-regen-tex\n"
		  "$BASEPATH/osgretex -pathfile ../mesh/campath.txt -config %s ../mesh/final.ive\n"
		  "cd $RUNDIR\ntime $BASEPATH/genTex --regen --dicedir %s --margins 10 10 1000000000000000 %s -f %s\n"
		  "$BASEPATH/lodgen --dicedir %s --mdir mesh-blend\n",
		  stereo_config_file_name.c_str(), dicedir,
		  stereo_config_file_name.c_str(),"mesh-regen-tex/",dicedir);
	  
	  fchmod(fileno(rgfp),0777);
	  fclose (rgfp);
	  if(regen_tex)
	    system("./regen.sh"); 
	}
      }
    }
  }
   
  
  // 
  // Clean-up
  //

  //  for(int i =0; i < (int)tasks.size(); i++)
  // delete tasks[i].veh_pose; 
  delete config_file;
  

  delete cov_file;


}






 

