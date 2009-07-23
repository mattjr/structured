//
// stereo_feature_finder_test.cpp
//
// A program to test the feature finding classes by running through a set of
// images loaded from a contents file.
//
// Each line of the contents file should have the following format:
//    <timestamp> <left_image_name> <right_image_name>
//
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
#include "auv_stereo_dense_new.hpp"
 
#include "auv_stereo_corner_finder.hpp"
//#include "auv_stereo_ncc_corner_finder.hpp"
#include "auv_stereo_keypoint_finder.hpp"
#include "auv_mesh.hpp"
#include "auv_mesh_io.hpp"
using namespace std;
using namespace libplankton;
using namespace libsnapper;
#ifdef USE_DENSE_STEREO
Stereo_Dense_New *sdense=NULL;
#endif

//
// Command-line arguments
//
static string stereo_config_file_name;
static string left_file_name;
static string right_file_name;
static double dense_scale;
static string dense_method;

static double dense_z_cutoff=4.0;
static bool use_undistorted_images = false;
static bool pause_after_each_frame = false;
static double image_scale = 1.0;
static int max_feature_count = 200;

static bool have_max_frame_count = false;
static unsigned int max_frame_count;

static bool display_debug_images = true;
static bool save_debug_images = false;
static bool use_dense_stereo=false;
static bool use_sift_features = false;
static bool use_surf_features = false;
static bool use_gpu_sift_features = false;
static bool use_cuda_sift_features = false;
//static bool use_ncc = false;

static bool triangulate = false;
static string triangulation_file_name;
static float subsample_ratio=1.0;
static bool dump_timing=false;
string timing_filename;
static double feature_depth_guess = AUV_NO_Z_GUESS;


//
// Parse command line arguments into global variables
//
static bool parse_args( int argc, char *argv[ ] )
{
   bool have_stereo_config_file_name = false;
   bool have_left_name = false;
   bool have_right_name = false;
   
   int i=1;
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
      else if( strcmp( argv[i], "-z" ) == 0 )
      {
         if( i == argc-1 ) return false;
         feature_depth_guess = atof( argv[i+1] );
         i+=2;
      }
      else if( strcmp( argv[i], "--method" ) == 0 )
      {
         if( i == argc-1 ) return false;
         dense_method = string( argv[i+1] );
         i+=2;
      }
    else if( strcmp( argv[i], "--save" ) == 0 )
      {

	save_debug_images=true;
         i+=1;
      }
      else if( strcmp( argv[i], "-n" ) == 0 )
      {
         if( i == argc-1 ) return false;
         have_max_frame_count = true;
         max_frame_count = atoi( argv[i+1] );
         i+=2;
      }
      else if( strcmp( argv[i], "--subsample" ) == 0 )
      {
         if( i == argc-1 ) return false;
        
         subsample_ratio = atof( argv[i+1] );
         i+=2;
      }
      else if( strcmp( argv[i], "-t" ) == 0 )
      {
         if( i == argc-1 ) return false;
         triangulate = true;
         triangulation_file_name = argv[i+1];
         i+=2;
      }
      else if( strcmp( argv[i], "-u" ) == 0 )
      {
         use_undistorted_images = true;
         i+=1;
      }
      else if( strcmp( argv[i], "--ds" ) == 0 )
      {
	use_dense_stereo = true;
         i+=1;
      }
  else if( strcmp( argv[i], "--cutoff" ) == 0 )
      {

       dense_z_cutoff = strtod( argv[i+1], NULL );
         i+=2; 
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
      /*
      else if( strcmp( argv[i], "-c" ) == 0 )
      {
         use_ncc = true;
         i+=1;
      }
      */
      else if( strcmp( argv[i], "--sift" ) == 0 )
      {
         use_sift_features = true;
         i+=1;
      }
      else if( strcmp( argv[i], "--timing" ) == 0 )
      {
	if( i == argc-1 ) return false;
	dump_timing=true;

	timing_filename = argv[i+1];
	i+=2;
      }
      else if( strcmp( argv[i], "--surf" ) == 0 )
      {
         use_surf_features = true;
         i+=1;
      }
      else if( strcmp( argv[i], "--gpu-sift" ) == 0 )
      {
         use_gpu_sift_features = true;
         i+=1;
      }
      else if( strcmp( argv[i], "--cuda-sift" ) == 0 )
      {
         use_cuda_sift_features = true;
         i+=1;
      }
      else if( !have_stereo_config_file_name )
      {
         stereo_config_file_name = argv[i];
         have_stereo_config_file_name = true;
         i++;
      }
      else if( !have_left_name )
      {
         left_file_name = argv[i];
         have_left_name = true;
         i++;
      }


      else if( !have_right_name )
      {
         right_file_name = argv[i];
         have_right_name = true;
         i++;
      }

      else
      {
         cerr << "Error - unknown parameter: " << argv[i] << endl;
         return false;
      }
   }

   return (have_left_name && have_right_name &&have_stereo_config_file_name);
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
   cout << "   -m <max_feature_count>  Set the maximum number of features to be found." << endl;
   cout << "   -n <max_frame_count>    Set the maximum number of frames to be processed." << endl;
   cout << "   -z <feature_depth>      Set an estimate for the feature depth relative to cameras." << endl;
   cout << "   -t <output_file>        Save triangulate feature positions to a file" << endl;
   cout << "   -c                      Use the normalised cross correlation feature descriptor" << endl;
   cout << "   --sift                  Find SIFT features." << endl;
   cout << "   --surf                  Find SURF features." << endl;
   cout << "   -d                      Do not display debug images." << endl;
   cout << "   -p                      Pause after each frame." << endl;
   cout << "   --ds                    Dense Stereo" << endl;
   cout << endl;
}



//
// Get the time in seconds (since the Unix Epoch in 1970)
//
static double get_time( void )
{
   struct timeval tv;
   
   gettimeofday( &tv, NULL );
   return tv.tv_sec+(double)tv.tv_usec/1e6;
}



//
// Remove radial and tangential distortion from the pair of stereo images
//
static void undistort_images( const Stereo_Calib  *calib,
                              const IplImage      *left_image,
                              const IplImage      *right_image,
                              IplImage           *&undist_left_image,
                              IplImage           *&undist_right_image )
{
   assert( calib != NULL );

   static Undistort_Data *left_undist_data = NULL;
   static Undistort_Data *right_undist_data = NULL;

   if( left_undist_data == NULL )
   {
      bool interpolate = false;
      left_undist_data = new Undistort_Data( calib->left_calib,
                                             left_image,
                                             interpolate );

      right_undist_data = new Undistort_Data( calib->right_calib,
                                              right_image,
                                              interpolate );
   }
   
   undist_left_image = cvCreateImage( cvGetSize(left_image),
                                      left_image->depth,
                                      left_image->nChannels );
   undist_right_image = cvCreateImage( cvGetSize(right_image),
                                       right_image->depth,
                                       right_image->nChannels );
   
   undistort_image( *left_undist_data, left_image, undist_left_image );
   undistort_image( *right_undist_data, right_image, undist_right_image );
} 



//
// Load the next pair of images from the contents file
//
static bool get_stereo_pair(
                             Stereo_Calib *stereo_calib,
                             IplImage     *&left_image,
                             IplImage     *&right_image,
                             unsigned int  &left_frame_id,
                             unsigned int  &right_frame_id,
                             string        &left_image_name,
                             string        &right_image_name )
{
   static unsigned int frame_id = 0;

   //
   // Try to read timestamp and file names
   //
   string contents_dir_name;   

   //
   // Load the images (-1 for unchanged grey/rgb)
   //
   string complete_left_name( contents_dir_name+left_image_name );
   left_image  = cvLoadImage( complete_left_name.c_str( ) , -1 );
   if( left_image == NULL )
   {
      cerr << "ERROR - unable to load image: " << complete_left_name << endl;
      return false;
   }

   string complete_right_name( contents_dir_name+right_image_name );
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
      IplImage *grey_left  = cvCreateImage( cvGetSize(left_image) , IPL_DEPTH_8U, 1 );
      cvCvtColor( left_image , grey_left , CV_BGR2GRAY );
      IplImage *temp = left_image;
      left_image = grey_left;
      cvReleaseImage( &temp );
   }

   if( left_image->nChannels == 3 )
   {
      IplImage *grey_right = cvCreateImage( cvGetSize(right_image), IPL_DEPTH_8U, 1 );
      cvCvtColor( right_image, grey_right, CV_BGR2GRAY );
      IplImage *temp = right_image;
      right_image = grey_right;
      cvReleaseImage( &temp );
   }

 
   //
   // Undistrort images if required
   // 
   if( use_undistorted_images )
   {
      IplImage *undist_left_image;
      IplImage *undist_right_image;

      undistort_images( stereo_calib, left_image, right_image,
                        undist_left_image, undist_right_image );

      cvReleaseImage( &left_image );
      cvReleaseImage( &right_image );
      left_image = undist_left_image;
      right_image = undist_right_image;
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

   left_frame_id = frame_id++;
   right_frame_id = frame_id++;
   return true;
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
   config_file->set_value( "SD_SHOW_DEBUG_IMAGES"    , display_debug_images );
   config_file->set_value( "SD_SAVE_DEBUG_IMAGES"    , save_debug_images );
   config_file->set_value( "SKF_SAVE_DEBUG_IMAGES"    , save_debug_images );
   config_file->set_value( "SCF_SAVE_DEBUG_IMAGES"    , save_debug_images );

   //config_file->set_value( "NCC_SCF_SHOW_DEBUG_IMAGES", display_debug_images );
   if(dense_method == "")
    config_file->get_value( "SD_METHOD", dense_method);
  else
    config_file->set_value( "SD_METHOD", dense_method);



   if( use_sift_features )
      config_file->set_value( "SKF_KEYPOINT_TYPE", "SIFT" );
   else if( use_surf_features )   
      config_file->set_value( "SKF_KEYPOINT_TYPE", "SURF" );
   else if( use_gpu_sift_features )   
      config_file->set_value( "SKF_KEYPOINT_TYPE", "GPU-SIFT" );
   else if( use_cuda_sift_features )   
     config_file->set_value( "SKF_KEYPOINT_TYPE", "CUDA-SIFT" );
   
   config_file->get_value( "SD_SCALE", dense_scale,1.0);


   //
   // Load the stereo camera calibration 
   //
   Stereo_Calib *calib = NULL;
   bool have_stereo_calib = false;
   string stereo_calib_file_name;
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
 
   // Create the stereo feature finder
   //
   Stereo_Feature_Finder *finder = NULL;
   if( use_sift_features || use_surf_features || use_gpu_sift_features || use_cuda_sift_features)
   {
      finder = new Stereo_Keypoint_Finder( *config_file, 
                                            use_undistorted_images, 
                                            image_scale, 
                                            calib );
   }
   /*
   else if( use_ncc )
   {
      finder = new Stereo_NCC_Corner_Finder( *config_file, 
                                              use_undistorted_images, 
                                              image_scale, 
                                              calib );
   }
   */

   else if(!sdense && use_dense_stereo){
     sdense= new Stereo_Dense_New(*config_file,
			      dense_scale ,
			      calib  );
   }

   else
   {
      finder = new Stereo_Corner_Finder( *config_file, 
                                         use_undistorted_images, 
                                         image_scale, 
                                         calib );
   }
   
         
   //
   // Run through the data
   //
   IplImage *left_frame;
   IplImage *right_frame;
   unsigned int left_frame_id;
   unsigned int right_frame_id;

   unsigned int stereo_pair_count = 0;


      //
      // Load the images
      //
      cout << "Loading images..." << endl;
      double load_start_time = get_time( );
      if( !get_stereo_pair(  calib,
                            left_frame, right_frame,
                            left_frame_id, right_frame_id,
			     left_file_name, right_file_name ) )
      {
	return -1;
      }                            
      double load_end_time = get_time( );
      
      cout << "Loaded images: " << left_file_name << " and "
           << right_file_name << endl;


      
      //
      // Find the features
      //
      cout << "Finding features..." << endl;
      double find_start_time = get_time( );
      list<Feature *> features;
      if(!use_dense_stereo){   

	finder->find( left_frame,
		      right_frame,
		      left_frame_id,
		      right_frame_id,
		      max_feature_count,
		      features,
		      feature_depth_guess );
      }
	
      else{ 
	
	sdense->dense_stereo(left_frame,right_frame);  
	
      }

      double find_end_time = get_time( );
  //
      // Display useful info 
      //
      cout << endl;
      cout << "Left Image : " << left_file_name << endl;
      cout << "Right Image: " << right_file_name << endl;
      cout << endl;
    
      cout << "Image loading time     : " << load_end_time-load_start_time << endl;
      cout << "Feature finding time   : " << find_end_time-find_start_time << endl;

      double tri_start_time = get_time( );
      //
      // Triangulate the features if requested
      //

      if( triangulate )
      {
	
	GPtrArray *localV=NULL;
         cout << "Saving triangulation to file: "
              << triangulation_file_name << endl;

	 if(!use_dense_stereo) {
	   Stereo_Reference_Frame ref_frame = STEREO_LEFT_CAMERA;
	   SymMatrix *image_coord_covar = NULL;
	   
	   list<Stereo_Feature_Estimate> feature_positions;
	   stereo_triangulate( *calib,
			       ref_frame,
			       features,
			       left_frame_id,
			       right_frame_id,
			       image_coord_covar,
			       feature_positions );
	   cout << "Number of features found: " << features.size( ) << endl;
	   cout << endl;
	   static ofstream out_file( triangulation_file_name.c_str( ) );
	   list<Stereo_Feature_Estimate>::iterator itr;
	   TVertex *vert;
	   localV= g_ptr_array_new ();
	   for( itr  = feature_positions.begin( ) ;
		itr != feature_positions.end( ) ;
		itr++ )
	     {
	       vert=(TVertex*)  gts_vertex_new (t_vertex_class (),
						itr->x[0],itr->x[1],itr->x[2]);
	       g_ptr_array_add(localV,GTS_VERTEX(vert));
	       
	     }
	 }else{ 
	   
	   std::vector<libplankton::Vector> points;   
	   sdense->get_points(points,subsample_ratio);
	   cout << "Number of points found: " << points.size( ) << endl;
	  
	   localV = g_ptr_array_new ();
	   TVertex *vert;
	   for(int i=0; i<(int)points.size(); i++){
	     //	  printf("%f %f %f\n",points[i](0),points[i](1),points[i](2));
	     //  if(-points[i](2) > dense_z_cutoff )
	     //    continue;
	     
	     vert=(TVertex*)  gts_vertex_new (t_vertex_class (),
					      points[i](0),points[i](1),
					      points[i](2));
	     //	     printf("%f %f %f\n",  points[i](0),points[i](1),
	     //    points[i](2));
	     g_ptr_array_add(localV,GTS_VERTEX(vert));
	   }
	 }
	 if(localV->len){
	   GtsSurface *surf= auv_mesh_pts(localV,0.0,0); 
	   FILE *fp = fopen(triangulation_file_name.c_str(), "w" );
	   auv_write_ply(surf, fp,false,"test");
	   fflush(fp);   
	   fclose(fp);
	 }else
	   fprintf(stderr,"No pts found\n");
      }

      double tri_end_time = get_time( );
    
      cout << endl;
      cout << "------------------------------------" << endl;
      cout << endl;
      if(dump_timing){
	FILE *fp=fopen(timing_filename.c_str(),"w");
	double total= (find_end_time-find_start_time)+(tri_end_time-tri_start_time);
	fprintf(fp,"%f\n",total);
	fclose(fp);
      }
	

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
      
      list<Feature*>::iterator itr;
      for( itr  = features.begin( ) ;
           itr != features.end( ) ;
           itr++ )
      {
         delete *itr;
      }     

      stereo_pair_count++;
   


   // 
   // Clean-up
   //
   delete config_file;
   delete calib;
   delete finder;
   
   return 0;
}
