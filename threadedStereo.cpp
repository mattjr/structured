//
// threadedStereo.cpp
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
#include <libseabedcommon/seabed_slam_file_io.hpp>
#include "auv_image_distortion.hpp"
#include "auv_stereo_geometry.hpp"
#include "adt_file_utils.hpp"
#include "auv_system.hpp"
#include "auv_stereo_corner_finder.hpp"
//#include "auv_stereo_ncc_corner_finder.hpp"
#include "auv_stereo_keypoint_finder.hpp"
#include "adt_image_norm.hpp"
#include "auv_stereo_dense.hpp"
#include <osg/ComputeBoundsVisitor>
#include "auv_concurrency.hpp"
#include "OSGExport.h"
#include "keypoint.hpp"
#include "XForm.h"
#include "TriMesh_algo.h"
#include "ShellCmd.h"
#include "stereo_cells.hpp"
#include "matrix.h"
#include "parser.h"
#include "mesh2hmap.h"
#include "output.h"
#include "quadmerge/envelope.hpp"
#include "VPBInterface.hpp"
#include "quadmerge/fileio.hpp"
#include <osgDB/WriteFile>
using namespace std;
using namespace libplankton;
using namespace ulapack;
using namespace libsnapper;
#define DEFAULT_NUM_THREADS (int)(get_num_processors( )*1.5)

bool reimage=true;
static int meshNum;
static double start_time = 0.0;
static bool apply_aug=false;
static double stop_time = numeric_limits<double>::max();
static int vpblod_override=0;
//
// Command-line arguments
//
static bool untex;
using namespace std;
int proj_tex_size;
int poster_tiles=40;
static bool rugosity=false;

static int dist_gentex_range=0;
static int vrip_split;
static FILE *pts_cov_fp;
static int skip_sparse=0;
static string background_mb;
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
static bool use_mb_ply =false;
static double eps=1.0;
static double subvol;
static bool useTextureArray=true;
static double longOrigin,latOrigin;
static bool interp_quad=false;
static int _tileRows;
static int _tileColumns;
static int targetFaces=40000;
static bool run_pos=false;
static bool do_novelty=false;
static double dense_scale;
static bool no_tex=false;
static bool no_vttex=false;

static vector<string> mb_xyz_files;
static int desired_area=250.0;
static bool have_max_frame_count = false;
static bool do_shader_color=false;
static unsigned int max_frame_count=INT_MAX;
static bool display_debug_images = true;
static bool output_pts_cov=false;
static bool use_sift_features = false;
static bool sing_gen_tex=false;
static bool use_surf_features = false;
static double max_alt_cutoff=10.0;
//static bool use_ncc = false;
static int skip_counter=0;
static double vrip_ramp;
static int num_skip=0;
static bool use_proj_tex=false;
static vector<string> mb_ply_filenames;
static bool have_mb_ply=false;
static bool have_mb_grd=false;
static bool have_cov_file=false;
static string stereo_calib_file_name;
static bool no_simp=true;
static bool no_split=false;
static FILE *uv_fp;
static ofstream file_name_list;
static string base_dir;
static double dense_z_cutoff=4.0;
static bool no_depth=false;
static string classes_file;
static double feature_depth_guess = AUV_NO_Z_GUESS;
static int num_threads=1;
static FILE *fpp,*fpp2,*pos_fp;
static double connected_comp_size_clean;
static double hole_fill_size;
static bool even_split=true;
static double cell_scale=1.0;
static bool do_classes=false;
static bool do_classes_interp=false;
static bool useVirtTex=true;
static bool use_dense_feature=false;
//StereoMatching* stereo;
//StereoImage simage;
static bool gen_mb_ply=false;
static bool output_ply_and_conf =true;
static FILE *conf_ply_file;
static bool output_3ds=false;
static char cov_file_name[2048];
static bool no_gen_tex=false;
static int mono_skip=2;
static bool no_vrip=false;
static bool quad_integration=true;
static bool no_quadmerge=true;
static double vrip_res;
static bool poster=false;
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
char cachedmeshdir[2048];
texcache_t cachedtexdir;
char cachedsegtex[2048];

static bool pos_clip=false;
static string deltaT_config_name;
static string deltaT_dir;
static bool hardware_compress=true;
const char *uname="mesh";
const char *dicedir="mesh-diced";
const char *quaddir="mesh-quad";
const char *aggdir="mesh-agg";
static bool use_shadows=false;
static string recon_config_file_name;
static string mbdir="mb";
static string deltaT_pose;
static string dense_method="";
static bool use_new_mb;
bool dist_run=false;
static int lodTexSize[3];
static bool mono_cam=false;
static       int sysres=0;
static double vrip_mb_clip_margin;
static double vrip_mb_clip_margin_extra;
static bool do_hw_blend=false;
// Image normalisation
bool image_norm=true;
int normalised_mean;
int normalised_var;
static Stereo_Calib *calib;
//static   GTimer  * overall_timer;
static time_t start_timer, end_timer; 
static Config_File *recon_config_file; 
static bool hmap_method=false;
static Config_File *dense_config_file;
static  int tex_size;
static float mb_grd_res=0.1;
static int spline_dist=max((int)round(1.0/mb_grd_res),5);
static int overlap=50;
static   double local_easting, local_northing;
using mapnik::Envelope;
Envelope<double> total_env;
osg::Vec2 reimageSize(1024,1024);
int pos_lod0_min_depth,pos_lod2_min_depth;
int pos_lod0_depth,pos_lod2_depth;
void runC(Stereo_Pose_Data &name);
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
        fprintf(stderr,"\n");
        //    printf("\n");
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



typedef struct _class_id_t{
    int class_id;
    string name;
    double time;
}class_id_t;
typedef struct _picture_cell{
    int row;
    int col;
    osg::BoundingBox bbox;
    std::string name;
    std::vector<int> images;
}picture_cell;
//
// Parse command line arguments into global variables
//
static bool parse_args( int argc, char *argv[ ] )
{
    libplankton::ArgumentParser argp(&argc,argv);
    argp.getApplicationUsage()->setApplicationName(argp.getApplicationName());
    argp.getApplicationUsage()->setDescription(argp.getApplicationName()+" example demonstrates the use of ImageStream for rendering movies as textures.");
    argp.getApplicationUsage()->setCommandLineUsage(argp.getApplicationName()+" <basedir>  [options]  ...\nwill look for recon.cfg stereo.calib stereo_pose_est.data and dir img for images\n I suggest creating symlinks to those files allowing for varible configuration.\n");
    argp.getApplicationUsage()->addCommandLineOption( "-r <texturesize>","       Final texture output size." );
    argp.getApplicationUsage()->addCommandLineOption( "-m <max_feature_count>" ," Set the maximum number of features to be found." );
    argp.getApplicationUsage()->addCommandLineOption( "-n <max_frame_count>","   Set the maximum number of frames to be processed." );
    argp.getApplicationUsage()->addCommandLineOption( "-z <feature_depth>","Set an estimate for the feature depth relative to cameras." );
    argp.getApplicationUsage()->addCommandLineOption( "-t <num_threads>","Number of threads to run" );
    argp.getApplicationUsage()->addCommandLineOption( "--mbfile","Multibeam ply mesh" );
    argp.getApplicationUsage()->addCommandLineOption( "--cov <file>","Input covar file." );
    argp.getApplicationUsage()->addCommandLineOption( "-c","Use the normalised cross correlation feature descriptor" );
    argp.getApplicationUsage()->addCommandLineOption( "--sift","Find SIFT features." );
    argp.getApplicationUsage()->addCommandLineOption( "--surf","Find SURF features." );
    argp.getApplicationUsage()->addCommandLineOption( "-d","Do not display debug images." );
    argp.getApplicationUsage()->addCommandLineOption( "--confply","Output confply file." );
    argp.getApplicationUsage()->addCommandLineOption( "--nogentex","Don't texture" );
    argp.getApplicationUsage()->addCommandLineOption( "--genmb","Generate MB mesh" );
    argp.getApplicationUsage()->addCommandLineOption( "--split <num>","Split individual meshes passed to vrip at num" );
    argp.getApplicationUsage()->addCommandLineOption( "--dicevol <vol>","Dice mesh to subvolume pieces of <vol> size" );
    argp.getApplicationUsage()->addCommandLineOption( "--res <resolution>","Vrip at resolution res default 0.033" );
    argp.getApplicationUsage()->addCommandLineOption( "-p","Pause after each frame." );
    argp.getApplicationUsage()->addCommandLineOption( "--uv","Output UV File." );
    argp.getApplicationUsage()->addCommandLineOption( "--3ds","Output 3ds Files." );
    argp.getApplicationUsage()->addCommandLineOption( "--dense-features","Dense features ." );
    argp.getApplicationUsage()->addCommandLineOption( "--ptscov","Output pts and cov ." );
    argp.getApplicationUsage()->addCommandLineOption( "--dicelod","Dice lods" );
    argp.getApplicationUsage()->addCommandLineOption( "--stereo-calib","Specify diffrent stereo calib" );
    argp.getApplicationUsage()->addCommandLineOption( "--contents-file","Specify diffrent contents file ." );
    argp.getApplicationUsage()->addCommandLineOption( "--nosimp","Specify diffrent contents file ." );

    if(argp.argc() < 2 || argp.isOption(argp[1]) ){
        fprintf(stderr,"First arg must be base dir\n");
        argp.getApplicationUsage()->write(std::cerr,osg::ApplicationUsage::COMMAND_LINE_OPTION);
        exit(-1);
    }
    base_dir=argp[1];
    if(base_dir=="."){
        int path_max = (unsigned int) PATH_MAX;
        char dirname[PATH_MAX];
        char *ret = getcwd(dirname, path_max);
        if(ret != NULL)
            base_dir=ret;
        printf("%s\n",base_dir.c_str());
    }
    recon_config_file_name = "recon.cfg";
    stereo_calib_file_name = "stereo.calib";
    contents_file_name = "stereo_pose_est.data";

    dir_name = "img/";

    argp.read("--split-area",desired_area);
    rugosity=argp.read("--rugosity");
    argp.read("--stereo-calib",stereo_calib_file_name);
    argp.read("--z-cutoff",dense_z_cutoff);
    argp.read("--poses",contents_file_name );
    apply_aug =argp.read("--apply_aug");
    argp.read("--skipsparse",skip_sparse);
    argp.read("--bkmb",background_mb);
    argp.read("--overlap",overlap);
    argp.read("--reimageres",reimageSize.x(),reimageSize.y());
    for(int i=0; i< 2; i++){
        if(osg::Image::computeNearestPowerOfTwo(reimageSize[i]) != reimageSize[i]){
            fprintf(stderr,"Clamping reimage %d res %f to ",i,reimageSize[i]);
            reimageSize[i]=osg::Image::computeNearestPowerOfTwo(reimageSize[i]);
            fprintf(stderr,"%f\n ",reimageSize[i]);

        }
    }



    if(argp.read("--noarray"))
        useTextureArray=false;


    if(argp.read("--mbply"))
        use_mb_ply=true;
    deltaT_config_name=base_dir+string("/")+"localiser.cfg";
    double lat_orig,lon_orig;

    try {

        libplankton::Config_File config_file( deltaT_config_name );
        if( config_file.get_value( "LATITUDE", lat_orig) == 0 )      {
            fprintf(stderr,"Couldn't get geoconf gloabal params\n");

        }
        if( config_file.get_value( "LONGITUDE", lon_orig) == 0 ){
            fprintf(stderr,"Couldn't get geoconf gloabal params\n");

        }
    }   catch( std::string error ) {
        std::cerr << "ERROR - " << error << endl;
        exit( 1 );
    }



    cout << "Lat Origin "<<lat_orig << " Long Ori " << lon_orig<<endl;
    UF::GeographicConversions::Redfearn gpsConversion("WGS84","UTM");

    double gridConvergence, pointScale;
    std::string zone;
    gpsConversion.GetGridCoordinates( lat_orig, lon_orig,
				      zone, local_easting, local_northing,
				      gridConvergence, pointScale);

    deltaT_dir=base_dir+string("/")+"DT/";
    deltaT_pose=base_dir+string("/")+"deltat_pose_est.data";
    mbdir=base_dir+"/"+mbdir+"/";
    stereo_calib_file_name= base_dir+string("/")+stereo_calib_file_name;
    recon_config_file_name= base_dir+string("/")+recon_config_file_name;
    contents_file_name= base_dir+string("/")+contents_file_name;
    dir_name= base_dir+string("/")+dir_name;

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


    if(use_dense_feature)
        dense_config_file= new Config_File("semi-dense.cfg");

    try {
        calib = new Stereo_Calib( stereo_calib_file_name );
    }
    catch( string error ) {
        cerr << "ERROR - " << error << endl;
        exit( 1 );
    }



    //recon_config_file->set_value( "NCC_SCF_SHOW_DEBUG_IMAGES", display_debug_images );
    //recon_config_file->set_value( "MESH_TEX_SIZE", tex_size );
    recon_config_file->get_value( "SD_SCALE", dense_scale,0.5);




    bool vrip_on;
    recon_config_file->get_value("USE_VRIP",vrip_on,true);

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
    /*recon_config_file->get_value("SIMP_RES_1",simp_res[0],0.005);
    recon_config_file->get_value("SIMP_RES_2",simp_res[1],0.1);
    recon_config_file->get_value("SIMP_RES_3",simp_res[2],0.5);*/
    recon_config_file->get_value("DIST_GENTEX_RANGE",dist_gentex_range,10);
    recon_config_file->get_value("POS_LOD2_MIN_DEPTH",pos_lod2_min_depth,6);
    recon_config_file->get_value("POS_LOD2_DEPTH",pos_lod2_depth,8);
    recon_config_file->get_value("IMAGE_SPLIT_COL",_tileColumns,8);
    recon_config_file->get_value("IMAGE_SPLIT_ROW",_tileRows,8);

    recon_config_file->get_value("POS_LOD0_MIN_DEPTH",pos_lod0_min_depth,8);
    recon_config_file->get_value("POS_LOD0_DEPTH",pos_lod0_depth,11);
    recon_config_file->get_value("POS_MIN_CLIP",pos_clip,true);
    recon_config_file->get_value("VRIP_MB_CLIP_MARGIN",vrip_mb_clip_margin,0.1);
    recon_config_file->get_value("VRIP_MB_CLIP_MARGIN_EXTRA",vrip_mb_clip_margin_extra,0.1);

    recon_config_file->get_value( "TEX_SIZE_LOD0", lodTexSize[0],
                                  512);

    recon_config_file->get_value( "POSTER_TILE_SCALE", poster_tiles,
                                  10);
    proj_tex_size=lodTexSize[0];

    //sprintf(cachedtexdir,"cache-tex-%d/",lodTexSize[1]);

    sprintf(cachedsegtex,"cache-seg-coords/");


    string mbfile;

    argp.read("--grdres",mb_grd_res);
    spline_dist=max((int)round(1.0/mb_grd_res),5);
    argp.read("--spline_dist",spline_dist);
    if(  argp.read("--clean"))
        further_clean=true;
    argp.read("-poster-scale",poster_tiles);
    untex=argp.read( "--untex" );

    argp.read("-r",image_scale);
    argp.read( "--edgethresh" ,edgethresh);
    argp.read("-m", max_feature_count );
    argp.read("-f",dir_name);
    have_mb_ply=argp.read("--mbfile",mbfile);
    if(have_mb_ply)
        mb_ply_filenames.push_back(mbfile) ;

    argp.read( "--monoskip" ,mono_skip);
    argp.read( "--vpblod" ,vpblod_override);

    if(argp.read(  "--noposclip"))
        pos_clip=false;
    argp.read(  "--imagesplit",_tileRows,_tileColumns);

    if(argp.read("--usenewmb"))
        use_new_mb=true;

    if(argp.read("--havembgrd"))
        have_mb_grd=true;

    if(argp.read("--genmb") || argp.read("--mb")){
        gen_mb_ply=true;
        have_mb_grd=true;
        no_vrip=true;
        no_quadmerge=false;
        do_shader_color=true;
        //    have_mb_ply=true;
        //  mb_ply_filenames.push_back(string("mb.ply")) ;
    }
    argp.read("-z",feature_depth_guess );
    use_dense_stereo=argp.read("--ds" );

    argp.read("--dense-method",dense_method);
    argp.read("--dense-scale",dense_scale);
    no_depth=argp.read("--no-depth" );

    argp.read("-s" ,num_skip);

    single_run= argp.read( "--single-run",single_run_start, single_run_stop );
    if(single_run)
        display_debug_images = false;

    num_threads=DEFAULT_NUM_THREADS;

    argp.read( "-t" ,num_threads);
    if(num_threads > 1)
        display_debug_images = false;
    argp.read( "--res",vrip_res );
    string cov_file;
    if(argp.read("--cov" ,cov_file))
        have_cov_file=true;
    if(argp.read( "-n",max_frame_count))
        have_max_frame_count = true;
    //    max_frame_count=20;
    //  have_max_frame_count = true;

    use_undistorted_images = argp.read("-u" );
    use_proj_tex=argp.read("--projtex");

    if(argp.read("--evensplit" ))
        even_split= true;
    argp.read("--cellscale",cell_scale );

    if(!useVirtTex){
        for(int i=0; (lodTexSize[0]/pow(2,i)) >= 8; i++){
            char tmppp[1024];
            int size=lodTexSize[0]/pow(2,i);
            if(size <8)
                continue;
            sprintf(tmppp,"cache-tex-%d/",size);
            cachedtexdir.push_back(make_pair<string,int> (string(tmppp),size));
            if(i==0 && reimage)
                break;
        }
    }else{
        char tmppp[1024];
        sprintf(tmppp,"cache-tex-%d/",lodTexSize[0]);
        cachedtexdir.push_back(make_pair<string,int> (string(tmppp),lodTexSize[0]));
    }

    if(argp.read("--pos")){
        vrip_on=false;
        no_vrip=true;
        run_pos=true;
    }



    if(argp.read("--new")){
        no_vrip=true;
        no_quadmerge=false;
        do_shader_color=true;
        if( argp.read("--vrip"))
            no_vrip=false;
    }
    if(  argp.read("--noquad"))
        no_quadmerge=true;

    if(vrip_on ){
        run_pos=false;
        no_simp = false;
    }

    if(argp.read("--interp-quad"))
        interp_quad=true;

    argp.read("--vrip-ramp",vrip_ramp );
    dist_run=argp.read("--dist" );
    sing_gen_tex =  argp.read("--threaded-gentex");
    dice_lod=argp.read("--dicelod" );

    no_simp=argp.read( "--nosimp" );

    no_split=argp.read( "--nosplit" );

    do_hw_blend=argp.read("--blend" );
    argp.read("--maxaltcutoff",max_alt_cutoff);

    do_classes=argp.read("--classes",classes_file );
    int interp_stride=4;
    do_classes_interp=argp.read("--classes-interp",classes_file,interp_stride );

    if(do_classes || do_classes_interp){
        FILE *fp =fopen(classes_file.c_str(),"r");
        if(!fp){
            fprintf(stderr,"No classes file %s file Quitting\n",classes_file.c_str());
            exit(-1);
        }



        char img_name[2048];
        std::vector<class_id_t> classes;

        while(!feof(fp)){
            class_id_t tmp;
            int ret=fscanf(fp,"%lf %s %d\n",&tmp.time,img_name,&tmp.class_id);
            if(ret !=3)
                printf("Read error %s\n",classes_file.c_str());
            tmp.name=img_name;
            classes.push_back(tmp);
        }
        fclose(fp);
        printf("Loaded %d class labels\n",(int)classes.size());
        if(do_classes_interp){
            int last_class=-1;
            int last_idx=-1;
            int left_unknown=0;
            for(int i=0; i < (int)classes.size(); i++){
                if(classes[i].class_id == 0){
                    if(last_class != -1 && i-last_idx <= interp_stride  )
                        classes[i].class_id=last_class;
                    else
                        left_unknown++;
                }else{
                    last_class=classes[i].class_id;
                    last_idx=i;
                }
            }
            printf("Interpolating class labels stride %d\nFinal unknown count %d\n",interp_stride,left_unknown);
        }
        fp =fopen("classid.txt","w");
        if(!fp){
            fprintf(stderr,"No classes out file classid.txt file Quitting\n");
            exit(-1);
        }

        for(int i=0; i < (int)classes.size(); i++)
            fprintf(fp,"%f %s %d\n",classes[i].time,classes[i].name.c_str(),
                    classes[i].class_id);

    }
    use_cached=(!argp.read("--no_cached" ));
    do_novelty=argp.read("--novelty");
    output_pts_cov=argp.read("--ptscov");
    if( argp.read("-d"))
        display_debug_images = false;

    poster=argp.read("--poster");
    pause_after_each_frame = argp.read("-p");
    //use_ncc=argp.read("-c");
    argp.read("--split",vrip_split);
    use_shadows=argp.read("--shadows");
    argp.read("--dicevol",subvol);
    hardware_compress =   !argp.read("--no-hardware-compress");

    output_uv_file=argp.read("--uv" ) ;
    use_sift_features=argp.read("--sift");

    use_dense_feature = argp.read("--dense-features");
    use_surf_features = argp.read("--surf");
    if(argp.read("-no_norm"))
        image_norm=false;

    argp.read("--start",start_time);
    argp.read("--stop",stop_time);
    output_3ds=  argp.read("--3ds");
    no_gen_tex= (!argp.read("--gentex"));
    if(argp.read("--novrip"))
        no_vrip=true;
    if(argp.read("--notex"))
        no_tex=true;

    if(argp.read("--novttex"))
        no_vttex=true;
    no_merge=argp.read("--nomerge");
    if(no_merge){
        run_pos = false;
        no_simp=false;
    }

    if(!output_3ds && !output_ply_and_conf){
        cerr << "Must do ply or 3ds output\n";
        return false;
    }

    if(dense_method == "")
        recon_config_file->get_value( "SD_METHOD", dense_method);
    else
        recon_config_file->set_value( "SD_METHOD", dense_method);


    if( use_sift_features )
        recon_config_file->set_value( "SKF_KEYPOINT_TYPE", "SIFT" );
    else if( use_surf_features )   
        recon_config_file->set_value( "SKF_KEYPOINT_TYPE", "SURF" );


    if(use_dense_stereo)
        sprintf(cachedmeshdir,"cache-mesh-dense/");
    else
        sprintf(cachedmeshdir,"cache-mesh-feat/");

    strcpy(cachedmeshdir,string(base_dir+string("/")+cachedmeshdir).c_str());
    for(int i=0; i < (int)cachedtexdir.size(); i++){
        cachedtexdir[i].first=base_dir+"/"+cachedtexdir[i].first;
    }
    strcpy(cachedsegtex,string(base_dir+string("/")+cachedsegtex).c_str());


    recon_config_file->set_value( "SKF_SHOW_DEBUG_IMAGES" , display_debug_images );
    recon_config_file->set_value( "SCF_SHOW_DEBUG_IMAGES"  , display_debug_images );
    recon_config_file->set_value( "SD_SHOW_DEBUG_IMAGES"  , display_debug_images );


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

    cout << "   will look for recon.cfg stereo.calib stereo_pose_est.data and dir img for images"<< endl;
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
    cout << "   --stereo-calib              Specify diffrent stereo calib" << endl;
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
    if( image_norm )
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
        tex_size=lodTexSize[0];

        finder = NULL;
        finder_dense = NULL;
        if( use_sift_features || use_surf_features ){
            finder = new Stereo_Keypoint_Finder( *recon_config_file,
                                                 use_undistorted_images,
                                                 image_scale,
                                                 calib );
        }


#ifdef USE_DENSE_STEREO
        if(use_dense_stereo){
            sdense= new Stereo_Dense(*recon_config_file,
                                     dense_scale ,
                                     calib  );
            //printf("Dense Scale %f\n",dense_scale);
        }
#endif

        /*
    else if( use_ncc )
      {
	finder = new Stereo_NCC_Corner_Finder(*recon_config_file, 
                                              use_undistorted_images, 
                                              image_scale, 
                                              calib );
      }
      */
        else
        {


            finder = new Stereo_Corner_Finder( *recon_config_file,
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

#ifdef USE_DENSE_STEREO
    Stereo_Dense *sdense;
#endif


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
			       Stereo_Pose pose,
			       Stereo_Pose_Data &name
			       )
{

    //name.cam_pose = new Vector(AUV_NUM_POSE_STATES);
    name.m =gts_matrix_identity (NULL);
    name.bbox = gts_bbox_new(gts_bbox_class(),NULL,0,0,0,0,0,0);
    //
    // Try to read timestamp and file names
    //

    int index;

    index = pose.pose_id;
    name.time = pose.pose_time;
    name.pose[AUV_POSE_INDEX_X] = pose.pose_est[AUV_POSE_INDEX_X];
    name.pose[AUV_POSE_INDEX_Y] = pose.pose_est[AUV_POSE_INDEX_Y];
    name.pose[AUV_POSE_INDEX_Z] = pose.pose_est[AUV_POSE_INDEX_Z];
    name.pose[AUV_POSE_INDEX_PHI] = pose.pose_est[AUV_POSE_INDEX_PHI];
    name.pose[AUV_POSE_INDEX_THETA] = pose.pose_est[AUV_POSE_INDEX_THETA];
    name.pose[AUV_POSE_INDEX_PSI] = pose.pose_est[AUV_POSE_INDEX_PSI];
    name.left_name = pose.left_image_name;
    name.right_name = pose.right_image_name;
    name.alt= pose.altitude;
    name.radius= pose.image_footprint_radius;
    name.overlap = pose.likely_overlap;


    name.mesh_name = "surface-"+osgDB::getStrippedName(name.left_name)+".tc.ply";
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
    char filename[2048];
    char meshfilename[2048];
    char tmpF[2048];

    vector<string> texfilename;
    bool meshcached=false;
    bool texcached=false;


    if(mono_cam)
        calib=name.calib;

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

    for(int i=0; i< (int)cachedtexdir.size(); i++){
        sprintf(tmpF,"%s/%s.png",
                cachedtexdir[i].first.c_str(),osgDB::getStrippedName(name.left_name).c_str());
        texfilename.push_back(tmpF);

    }

    if(use_dense_stereo)
        sprintf(meshfilename,"%s/surface-%s-%s.ply",
                cachedmeshdir,osgDB::getStrippedName(name.left_name).c_str(),
                dense_method.c_str());
    else
        sprintf(meshfilename,"%s/surface-%s.ply",
                cachedmeshdir,osgDB::getStrippedName(name.left_name).c_str());

    //printf("Mesh cached check %s\n",meshfilename);
    TriMesh *mesh=NULL;
    TriMesh::verbose=0;

    if(!use_cached){

        //     printf("Redoing cache\n");
        meshcached=false;
        texcached=false;
    }else{
        if(FileExists(meshfilename)){
            mesh = TriMesh::read(meshfilename);
            if(!mesh){
                meshcached=false;
                fprintf(stderr,"\nWARNING - %s invalid redoing\n",meshfilename);
            }else
                meshcached=true;
        }
        texcached=true;
        for(int i=0; i< (int)texfilename.size(); i++){
            if(!FileExists(texfilename[i]))
                texcached=false;
        }
    }

    sprintf(filename,"%s/%s.ply",
            aggdir,osgDB::getStrippedName(name.mesh_name).c_str());


    GtsSurface *surf=NULL;
    if(!meshcached || !texcached ){



        //
        // Load the images
        //

        if( !get_stereo_pair( name.left_name,name.right_name,name.dir,
                              left_frame, right_frame,
                              color_frame))
        {
            int progCount=doneCount.increment();
            image_count_verbose (progCount, totalTodoCount);
            fprintf(stderr,"\nWARNING - Failed to get imagepair %s %s\n",name.left_name.c_str(),name.right_name.c_str());
            fflush(stderr);
            name.valid=false;
            return false;
        }

        if(!texcached){
            //      printf("\nCaching texture %s\n",texfilename);
            //if(!no_atlas)
            for(int i=0; i< (int)texfilename.size(); i++){
                osgExp->cacheImage(color_frame,texfilename[i],cachedtexdir[i].second,false);
            }
            //else
            //osgExp->cacheCompressedImage(color_frame,texfilename,tex_size);
        }


        if(!meshcached){
            int mesh_verts=0;
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
                mesh_verts=localV->len;
                double mult=0.00;
                if(mesh_verts){
                    surf = auv_mesh_pts(localV,mult,0);
                    FILE *fp = fopen(meshfilename, "w" );
                    if(!fp){
                        fprintf(stderr,"\nWARNING - Can't open mesh file %s\n",meshfilename);
                        fflush(stderr);
                        name.valid=false;

                        // clean up
                        cvReleaseImage( &left_frame );
                        cvReleaseImage( &right_frame );
                        cvReleaseImage( &color_frame);
                        return false;
                    }
                    auv_write_ply(surf, fp,have_cov_file,"test");
                    fflush(fp);
                    fclose(fp);
                    //Destory Surf
                    mesh = TriMesh::read(meshfilename);
                    if(!mesh)
                        fprintf(stderr,"\nWARNING - mesh null after doing dense stereo\n");

                    // delete the vertices
                    g_ptr_array_free( localV, true );
                }
            }else{
#ifdef USE_DENSE_STEREO   

                sdense->dense_stereo(left_frame,right_frame);
                std::vector<libplankton::Vector> points;
                IplImage *mask =cvCreateImage(cvSize(sdense->getDisp16()->width,sdense->getDisp16()->height),IPL_DEPTH_8U,1);
                cvZero(mask);
                sdense->get_points(points,mask);
                mesh=get_dense_grid(mask,points);
                cvReleaseImage(&mask);
                mesh_verts = points.size();
                if(mesh && mesh_verts)
                    mesh->write(meshfilename);

                /*TVertex *vert;
	for(int i=0; i<(int)points.size(); i++){
	  //	  printf("%f %f %f\n",points[i](0),points[i](1),points[i](2));
	  if(points[i](2) > dense_z_cutoff )
	    continue;

	  vert=(TVertex*)  gts_vertex_new (t_vertex_class (),
					   points[i](0),points[i](1),
					   points[i](2));
	  g_ptr_array_add(localV,GTS_VERTEX(vert));
        } */
#else 
                fprintf(stderr,"Dense support not compiled\n");
                exit(0);
#endif

            }


            //printf("Valid %d\n",mesh_verts);
            if(!mesh_verts){
                int progCount=doneCount.increment();
                image_count_verbose (progCount, totalTodoCount);
                fprintf(stderr,"\nWARNING - Empty vertex array %s\n",meshfilename);
                fflush(stderr);
                name.valid=false;
                if( display_debug_images && pause_after_each_frame )
                    cvWaitKey( 0 );
                // clean up
                cvReleaseImage( &left_frame );
                cvReleaseImage( &right_frame );
                cvReleaseImage( &color_frame);

                return false;
            }
        }




    }



    if(output_3ds){
        char fname_3ds[2048];
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

        osgExp->Export3DS(surf,fname_3ds,textures,tex_size,NULL);
        gts_matrix_destroy (invM);
    }

    if(surf)
        gts_object_destroy (GTS_OBJECT (surf));

    if(output_ply_and_conf){
        if(!mesh){
            fprintf(stderr,"\nWARNING - No cached mesh file %s\n",meshfilename);
            fflush(stderr);
            name.valid=false;

            // clean up
            cvReleaseImage( &left_frame );
            cvReleaseImage( &right_frame );
            cvReleaseImage( &color_frame);
            return false;
        }

        edge_len_thresh(mesh,edgethresh);
        if(!mesh->faces.size()){
            fprintf(stderr,"\nWARNING - Mesh empty after edge threshold clipping %s\n",meshfilename);
            fflush(stderr);
            name.valid=false;

            // clean up
            cvReleaseImage( &left_frame );
            cvReleaseImage( &right_frame );
            cvReleaseImage( &color_frame);
            return false;
        }

        xform xf(name.m[0][0],name.m[1][0],name.m[2][0],name.m[3][0],
                 name.m[0][1],name.m[1][1],name.m[2][1],name.m[3][1],
                 name.m[0][2],name.m[1][2],name.m[2][2],name.m[3][2],
                 name.m[0][3],name.m[1][3],name.m[2][3],name.m[3][3]);


        if(apply_aug &&    mesh->confidences.empty()){
            //   mesh->confidences.empty();
            int nv = mesh->vertices.size();
            mesh->confidences.resize(nv, 1);
            for(int i=0; i< nv; i++)
                mesh->confidences[i]=sqrt(pow(mesh->vertices[i][0],2)+
                                          pow(mesh->vertices[i][1],2)+
                                          pow(mesh->vertices[i][2],2));
        }
        apply_xform(mesh,xf);
        mesh->need_bbox();
        mesh->write(filename);





        gts_bbox_set(name.bbox,NULL,
                     mesh->bbox.min[0],
                     mesh->bbox.min[1],
                     mesh->bbox.min[2],

                     mesh->bbox.max[0],
                     mesh->bbox.max[1],
                     mesh->bbox.max[2]);



        if(hmap_method){
            mesh_t hmmesh;
            trimesh2mesh(&hmmesh, mesh);
            int x, y, z, invert;
            float  x_m_pix, y_m_pix;
            int width, height;
            width = height = 0;

            invert = 0;
            z = 2; x = 0; y = 1;
            x_m_pix = y_m_pix = 0.01;

            hmap_t hmap;
            hmap.rows = height;
            hmap.cols = width;
            mesh2hmap(&hmap, &hmmesh, x, y, z, invert, x_m_pix, y_m_pix);
            //print_hmap(&hmap); return 0;
            fmatrix_free(hmmesh.vert);
            irowarray_free(hmmesh.poly, hmmesh.num_poly);
            printf("%f %f\n",hmap.z_min, hmap.z_max);
            char tmp[2048];
            int bpp=8;
            int range = (int) pow((double)2,bpp) - 1;
            int min=0;
            int bg = 0;
            int max=range;
            sprintf(tmp,"%s.ppm",filename);
            char max_clr[3*11 + 3];
            char min_clr[3*11 + 3];
            char bg_clr[3*11 + 3];
            sprintf(max_clr, "%d %d %d", range, range, range);
            sprintf(min_clr, "%d %d %d", min, min, min);
            sprintf(bg_clr, "%d %d %d", bg, bg, bg);
            hmap2ppm(tmp,&hmap, range, min, max,
                     min_clr, max_clr, bg_clr);

            sprintf(tmp,"%s.hm",filename);
            write_hmap_file(tmp, &hmap);
        }
        delete mesh;
    }
    



    if(!meshcached || !texcached){

        // Pause between frames if requested.
        //
        if(!meshcached){
            if( display_debug_images && pause_after_each_frame )
                cvWaitKey( 0 );
            else if( display_debug_images )
                cvWaitKey( 100 );
        }
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
    name.valid=true;

    return true;
}



void runC(Stereo_Pose_Data &name){

    printf("%s Written Out to  %s\n",name.left_name.c_str(),name.mesh_name.c_str());
}
bool gen_stereo_from_mono(std::vector<Mono_Image_Name> &mono_names,Slices &tasks,Camera_Calib *cam_calib,unsigned int &stereo_pair_count){

    for(int i=0; i <(int) mono_names.size()-mono_skip; i++){
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
        name.mesh_name = "surface-"+osgDB::getStrippedName(name.left_name)+".tc.ply";
        name.id=stereo_pair_count++;
        fill_gts_matrix(name.pose,name.m);
        tasks.push_back(name);

    }
    
    return true;
}

int main( int argc, char *argv[ ] )
{
    start_timer = time(NULL);

    FILE *rerunfp=fopen("rerun.sh","w");
    FILE *timing_fp=fopen("timing.txt","w");
    fprintf(rerunfp,"#!/bin/bash\n");
    for(int i=0; i < argc; i++)
        fprintf(rerunfp,"%s ",argv[i]);
    fprintf(rerunfp," $*\n");;
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

    // Run through the data
    //conf
    ifstream *cov_file;
    ifstream      contents_file;


    auv_data_tools::makedir(quaddir);

    chmod(quaddir,   0777);

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
    for(int i=0; i < (int)cachedtexdir.size(); i++){
        auv_data_tools::makedir(cachedtexdir[i].first.c_str());
        chmod(cachedtexdir[i].first.c_str(),   0777);
    }
    auv_data_tools::makedir(cachedsegtex);
    chmod(cachedsegtex,   0777);
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
        FILE *imgpathfp=fopen("mesh/imgpath.txt","w");
        if(!imgpathfp ){
            fprintf(stderr,"Cannot open mesh/imgpath.txt\n");
            exit(-1);
        }
        fprintf(imgpathfp,"%s\n","../img");
        fclose(imgpathfp);
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
    if(num_threads > 1)
        printf("Threaded Stereo: %d threads initialized\n",num_threads);
    else
        printf("Threaded Stereo: single thread initialized\n");

    printf("Processing Meshes...\n");

    Matrix *image_coord_covar;
    if(have_cov_file){
        image_coord_covar = new Matrix(4,4);
        image_coord_covar->clear( );
        recon_config_file->get_value("STEREO_LEFT_X_VAR",(*image_coord_covar)(0,0) );
        recon_config_file->get_value("STEREO_LEFT_Y_VAR",(*image_coord_covar)(1,1) );
        recon_config_file->get_value("STEREO_RIGHT_X_VAR",(*image_coord_covar)(2,2));
        recon_config_file->get_value("STEREO_RIGHT_Y_VAR",(*image_coord_covar)(3,3));
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
    int skip_sparse_cnt=skip_sparse;
    if(skip_sparse)
        printf("Sparsifying Mesh Skipping  %d\n",skip_sparse_cnt);
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

        try {
            calib = new Stereo_Calib( stereo_calib_file_name );
        }
        catch( string error ) {
            cerr << "ERROR - " << error << endl;
            exit( 1 );
        }

        gen_stereo_from_mono(mono_names,tasks,&calib->left_calib,stereo_pair_count);


    }else{
        Stereo_Pose_File pose_data=read_stereo_pose_est_file(contents_file_name );
        latOrigin=pose_data.origin_latitude;
        longOrigin=pose_data.origin_longitude;
        FILE *fpp1=fopen("mesh/origin.txt","w");
        if(!fpp1 ){
            fprintf(stderr,"Cannot open mesh/origin.txt\n");
            exit(-1);
        }
        fprintf(fpp1,"%f %f\n",latOrigin,longOrigin);
        fclose(fpp1);

        vector<Stereo_Pose>::const_iterator cii;
        cii=pose_data.poses.begin();
        while( cii != pose_data.poses.end() && (!have_max_frame_count || stereo_pair_count < max_frame_count) ){



            Stereo_Pose_Data name;
            get_auv_image_name( dir_name, *cii, name) ;
            cii++;
            if(cii->pose_time < start_time || cii->altitude > max_alt_cutoff){
                continue;
            }
            if(cii->pose_time >= stop_time)
                break;

            name.id= single_run_start+stereo_pair_count++;
            name.valid=false;
            tasks.push_back(name);

        }
    }


    if(tasks.size() <= 0){
        fprintf(stderr,"No tasks loaded check %s\n",contents_file_name.c_str());
        exit(-1);
    }
    start_time=tasks[0].time;
    stop_time=tasks[tasks.size()-1].time;

    totalTodoCount=stereo_pair_count;

    fprintf(timing_fp,"%d\n",(int)tasks.size());
    boost::xtime xt, xt2;
    long time_num;
    // consumer pool model...
    //unsigned int taskSize=tasks.size();
    //if(num_threads > 1){
    {
        g_thread_init (NULL);
        if(num_threads > 1){
            display_debug_images = false;
        }
        boost::xtime_get(&xt, boost::TIME_UTC);

#pragma omp parallel num_threads(num_threads)
        {
            threadedStereo *ts=new threadedStereo(recon_config_file_name,"semi-dense.cfg");
            cvSetNumThreads(1);
#pragma omp for
            for(unsigned int i=0; i < tasks.size(); i++){
                if(!ts->runP(tasks[i])){
                    tasks[i].valid=false;
                }
            }
            //delete ts;
        }


        boost::xtime_get(&xt2, boost::TIME_UTC);
        time_num = (xt2.sec*1000000000 + xt2.nsec - xt.sec*1000000000 - xt.nsec) / 1000000;


        double secs=time_num/1000.0;
        printf("Threads %d Time: %.2f sec\n", num_threads, secs);
        fprintf(timing_fp,"Stereo %f\n",secs);
    }
    /*  else{
    g_thread_init (NULL);
    display_debug_images = false; 
    boost::xtime_get(&xt, boost::TIME_UTC);
    SlicePool pool(tasks);
    //Needed or open mp will try to create threads within threads which is 
    //not a good thing opencv openmp support blows anyway as of late.
    //weird bug
    cvSetNumThreads(1);
    Convolution convolution(pool,(int)tasks.size() > num_threads ? num_threads : tasks.size(),recon_config_file_name,"semi-dense.cfg");
    boost::thread thrd(convolution);
    thrd.join();

    boost::xtime_get(&xt2, boost::TIME_UTC);
    time_num = (xt2.sec*1000000000 + xt2.nsec - xt.sec*1000000000 - xt.nsec) / 1000000; 
    double secs=time_num/1000.0;
    printf("Threads %d Time: %.2f sec\n", num_threads, secs);
    fprintf(timing_fp,"Stereo %f\n",secs);


  }
for(Slices::iterator itr=tasks.begin(); itr != tasks.end(); itr++){
    if(!itr->valid){
        fprintf(stderr,"%s Not valid\n",itr->mesh_name.c_str());
        tasks.erase(itr);
    }
    //printf("%s %d\n",itr->mesh_name.c_str(),itr->valid);
}
printf("Task Size %d Valid %d Invalid %d\n",taskSize,(int)tasks.size(),(int)taskSize-tasks.size());
*/
    fclose(timing_fp);

    if(!single_run){
        int ct=0;
        for(Slices::iterator itr=tasks.begin(); itr != tasks.end(); itr++){
            Slice name=(*itr);

            if(ct==0)
                total_env.init(name.bbox->x1,name.bbox->y1,name.bbox->x2,name.bbox->y2);
            else
                total_env.expand_to_include(Envelope<double>(name.bbox->x1,name.bbox->y1,name.bbox->x2,name.bbox->y2));


            fprintf(fpp,"%d %f %s %s",
                    ct++,name.time,name.left_name.c_str(),name.right_name.c_str());
            fprintf(fpp," %f %f %f %f %f %f",name.bbox->x1,name.bbox->y1,
                    name.bbox->z1,
                    name.bbox->x2,name.bbox->y2,name.bbox->z2);

            for(int i=0; i< 4; i++){
                for(int j=0; j < 4; j++){
                    fprintf(fpp," %f",name.m[i][j]);
                }
            }
            fprintf(fpp,"\n");

            //      if(use_poisson_recon){
            //	dump_pts(pos_fp,string("mesh-agg/"+name.mesh_name).c_str(),clean_pos_pts);
            //    }
        }

        char conf_name[2048];

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

            }else{
                fprintf(stderr,"Not valid %s\n", osgDB::getStrippedName(tasks[i].left_name).c_str()     );
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

        if(use_new_mb){
            ifstream mblist("mb_grd/mblist.txt");
            char tmp[2048];
            while(mblist && !mblist.eof()){
                mblist.getline(tmp,2048);
                if(strlen(tmp)> 1)
                    mb_ply_filenames.push_back(string(tmp));
            }
            mblist.close();
        }

        if(mb_ply_filenames.size()){
            have_mb_ply=true;
            //  printf("Integrating Vision clipped mb from %d files.\n",(int)mb_ply_filenames.size());
        }

        FILE *vrip_seg_fp;
        char vrip_seg_fname[2048];
        FILE *bboxfp;
        string vripcmd_fn="mesh-agg/vripcmds";
        FILE *vripcmds_fp=fopen(vripcmd_fn.c_str(),"w");
        FILE *diced_fp=fopen("mesh-diced/diced.txt","w");
        FILE *diced_lod_fp=fopen("mesh-diced/dicedld.txt","w");

        if(!vripcmds_fp){
            printf("Can't open vripcmds\n");
            exit(-1);
        }
        char cwd[2048];
        char *dirres;
        if(!dist_run)
            dirres=getcwd(cwd,2048);
        else
            strcpy(cwd,"/mnt/shared/");



        //const char *simplogdir="/mnt/shared/log-simp";
        const char *pos_simp_log_dir="/mnt/shared/log-possimp";
        //const char *vriplogdir="/mnt/shared/log-vrip";

        float simp_mult=1.0;

        if(have_mb_ply)
            simp_mult=1.0;
        else
            simp_mult=2.0;

        ShellCmd shellcm(basepath.c_str(),simp_mult,pos_simp_log_dir,dist_run,cwd,aggdir,dicedir,have_mb_ply,num_threads);
        shellcm.write_setup();


        if(gen_mb_ply){

            FILE *genmbfp=fopen("genmb.sh","w");
            /*	fprintf(genmbfp,"#!/bin/bash\nPATH=$PATH:%s/tridecimator:/usr/lib/gmt/bin/:%s../mbsystems/bin/\ncd %s\n"
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
	*/
            /*	char bkarg[2048];

	if(background_mb.c_str() > 0)
	  sprintf(bkarg," %s ",background_mb.c_str());
	else
	sprintf(bkarg," ");*/
            string mb_extension = use_mb_ply ? "ply" : "xyz";
            fprintf(genmbfp,"#!/bin/bash\nmkdir -p mb_proc; cd mb_proc; mbgsf_to_xyz -start %f -stop %f %s  %s/mb ;ls *.%s > mbxyzlist\n",

                    start_time,stop_time,deltaT_config_name.c_str(),deltaT_dir.c_str(),mb_extension.c_str());

            if(use_mb_ply){
                fprintf(genmbfp,"for i in `ls *.xyz`; do\n"
                        "echo auv_tri $i $i.ply >> tricmds\n"
                        "done\n"
                        "%s/runtp.py tricmds %d Tri\n",basepath.c_str(),num_threads);
            }
            fchmod(fileno(genmbfp),   0777);
            fclose(genmbfp);
            sysres=system("./genmb.sh");
            FILE *dtfp=fopen("mb_proc/mbxyzlist","r");
            char tmp[1024];
            while(dtfp && !feof(dtfp)){
                if(fscanf(dtfp,"%s\n",tmp))
                    mb_xyz_files.push_back(tmp);
            }
            fclose(dtfp);
        }


        string mbfile=mbdir+"/"+"mb-total.ply";
        Bounds bounds( tasks );
        std::vector<Cell_Data> vrip_cells;
        if(even_split){
            vrip_cells=calc_cells(tasks,EVEN_SPLIT,cell_scale);
            printf("Even Split\n");
        }
        else
            vrip_cells=calc_cells(tasks,AUV_SPLIT,cell_scale);
        printf("Split into %d cells for VRIP\n",(int)vrip_cells.size());

        for(int i=0; i <(int)vrip_cells.size(); i++){

            if(vrip_cells[i].poses.size() == 0)
                continue;

            sprintf(vrip_seg_fname,"mesh-agg/vripseg-%08d.txt",i);
            sprintf(conf_name,"mesh-diced/bbox-clipped-diced-%08d.ply.txt",i);

            vrip_seg_fp=fopen(vrip_seg_fname,"w");
            bboxfp = fopen(conf_name,"w");
            if(!vrip_seg_fp || !bboxfp){
                printf("Unable to open %s\n",vrip_seg_fname);
            }

            char redirstr[2048];
            if(!dist_run)
                sprintf(redirstr,">  vripsurflog-%08d.txt",i);
            else
                sprintf(redirstr," ");

            fprintf(diced_fp,"clipped-diced-%08d.ply\n",i);
            fprintf(diced_lod_fp,"clipped-diced-%08d-lod0.ply\n",i);
            fprintf(vripcmds_fp,"set BASEDIR=\"%s\"; set OUTDIR=\"mesh-agg/\";set VRIP_HOME=\"$BASEDIR/vrip\";setenv VRIP_DIR \"$VRIP_HOME/src/vrip/\";set path = ($path $VRIP_HOME/bin);cd %s/$OUTDIR;",basepath.c_str(),cwd);

            if(have_mb_ply){
                for(int k=0; k < (int)mb_ply_filenames.size(); k++){
                    fprintf(vripcmds_fp,"plycullmaxx %f %f %f %f %f %f %f < %s > ../mesh-agg/dirty-clipped-mb-%08d-%08d.ply;tridecimator ../mesh-agg/dirty-clipped-mb-%08d-%08d.ply ../mesh-agg/clipped-mb-%08d-%08d.ply 0 -e%f;set VISLIST=`cat ../%s | grep surface |cut -f1 -d\" \"`; plyclipbboxes -e %f $VISLIST ../mesh-agg/clipped-mb-%08d-%08d.ply > ../mesh-agg/vis-mb-%08d-%08d.ply;", vrip_cells[i].bounds.min_x,
                            vrip_cells[i].bounds.min_y,
                            -FLT_MAX,
                            vrip_cells[i].bounds.max_x,
                            vrip_cells[i].bounds.max_y,
                            FLT_MAX,
                            eps,
                            mb_ply_filenames[k].c_str(),k,i,k,i,k,i,edgethresh,vrip_seg_fname,vrip_mb_clip_margin+vrip_mb_clip_margin_extra,k,i,k,i);



                    fprintf(vripcmds_fp,	  " plyclipbboxes -e %f $VISLIST ../mesh-agg/clipped-mb-%08d-%08d.ply > ../mesh-agg/sub-mb-%08d-%08d.ply;",vrip_mb_clip_margin,k,i,k,i);
                }

            }
            if(!no_merge)
                fprintf(vripcmds_fp,"$BASEDIR/vrip/bin/vripnew auto-%08d.vri ../%s ../%s %f -rampscale %f;$BASEDIR/vrip/bin/vripsurf auto-%08d.vri ../mesh-agg/seg-%08d.ply %s ;",i,vrip_seg_fname,vrip_seg_fname,vrip_res,vrip_ramp,i,i,redirstr);
            else
                fprintf(vripcmds_fp,"cat ../%s | cut -f1 -d\" \" | xargs $BASEDIR/vrip/bin/plymerge > ../mesh-agg/seg-%08d.ply;",vrip_seg_fname,i);
            /*
            fprintf(vripcmds_fp,"$BASEDIR/treeBBClip ../mesh-agg/seg-%08d.ply %f %f %f %f %f %f -dump ../mesh-diced/gap-clipped-diced-%08d.ive --outfile ../mesh-diced/tmp-clipped-diced-%08d.ply;",
                    i,
                    vrip_cells[i].bounds.min_x,
                    vrip_cells[i].bounds.min_y,
                    -FLT_MAX,
                    vrip_cells[i].bounds.max_x,
                    vrip_cells[i].bounds.max_y,
                    FLT_MAX,i,
                    i);
*/

            fprintf(vripcmds_fp,"$BASEDIR/treeBBClip ../mesh-agg/seg-%08d.ply %f %f %f %f %f %f -dup --outfile ../mesh-diced/tmp-clipped-diced-%08d.ply;",
                    i,
                    vrip_cells[i].bounds.min_x,
                    vrip_cells[i].bounds.min_y,
                    -FLT_MAX,
                    vrip_cells[i].bounds.max_x,
                    vrip_cells[i].bounds.max_y,
                    FLT_MAX,
                    i);

            if(have_mb_ply){
                fprintf(vripcmds_fp,"mv ../mesh-diced/clipped-diced-%08d.ply ../mesh-diced/nomb-diced-%08d.ply;",i,i);
                for(int k=0; k < (int)mb_ply_filenames.size(); k++){
                    fprintf(vripcmds_fp,"plysubtract  ../mesh-agg/clipped-mb-%08d-%08d.ply ../mesh-agg/vis-mb-%08d-%08d.ply >  ../mesh-agg/tmp-mb-%08d-%08d.ply ;",k,i,k,i,k,i);
                    fprintf(vripcmds_fp,"clip_delaunay  ../mesh-diced/nomb-diced-%08d.ply ../mesh-agg/tmp-mb-%08d-%08d.ply  ../mesh-agg/unclean-inv-mb-%08d-%08d.ply ;",i,k,i,k,i);
                    fprintf(vripcmds_fp,"tridecimator ../mesh-agg/unclean-inv-mb-%08d-%08d.ply ../mesh-agg/inv-mb-%08d-%08d.ply 0 -e0.25;",k,i,k,i);

                }
                fprintf(vripcmds_fp,"plymerge ../mesh-diced/nomb-diced-%08d.ply ",i );
                for(int k=0; k < (int)mb_ply_filenames.size(); k++)
                    fprintf(vripcmds_fp," ../mesh-agg/inv-mb-%08d-%08d.ply ",k,i);
                fprintf(vripcmds_fp,"> ../mesh-diced/clipped-diced-%08d.ply;",i);
            }
            fprintf(vripcmds_fp,"cd ..\n");


            for(unsigned int j=0; j <vrip_cells[i].poses.size(); j++){
                const Stereo_Pose_Data *pose=vrip_cells[i].poses[j];
                if(!pose->valid)
                    continue;
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
                for(int k=0; k < (int)mb_ply_filenames.size(); k++)
                    fprintf(vrip_seg_fp,"vis-mb-%08d-%08d.ply  0.1 0\n",k,i);

            fclose(vrip_seg_fp);
            fclose(bboxfp);
        }
        fclose(vripcmds_fp);
        fclose(diced_fp);
        fclose(diced_lod_fp);
        FILE *quadmerge_seg_fp;
        char quadmerge_seg_fname[2048];

        string quadmergecmd_fn="mesh-quad/quadmergecmds";
        FILE *quadmergecmds_fp=fopen(quadmergecmd_fn.c_str(),"w");
        diced_fp=fopen("mesh-quad/diced.txt","w");

        if(!quadmergecmds_fp){
            printf("Can't open quadmergecmds\n");
            exit(-1);
        }


        char quadprecmd[2048];
        sprintf(quadmerge_seg_fname,"mesh-quad/quadmergeseg.txt");
        string shadowstr;
        if(use_shadows)
            shadowstr="-shadow -color";

        quadmerge_seg_fp=fopen(quadmerge_seg_fname,"w");
        //mv ../mesh-quad/quad-lod0.ply ../mesh-quad/quad-lod0.uncleaned.ply; %s/tridecimator/bin/tridecimator ../mesh-quad/quad-lod0.uncleaned.ply ../mesh-quad/quad-lod0.ply 0 -e20%%;
        sprintf(quadprecmd,"cd mesh-quad;%s/bin/quadmerge -geoconf %s -lod %s -input ../%s -edgethresh %f -output ../mesh-quad/quad.ply -range ../mesh-quad/range2.txt;%s/vrip/bin/plybbox ../mesh-quad/quad-lod0.ply > range.txt;",
                basepath.c_str(),deltaT_config_name.c_str(),shadowstr.c_str(),quadmerge_seg_fname,edgethresh,basepath.c_str());

        for(int i=0; i < (int)tasks.size(); i++){
            //Quadmerge List
            if(tasks[i].valid)
                fprintf(quadmerge_seg_fp,"../mesh-agg/%s %f %d\n",tasks[i].mesh_name.c_str(),vrip_res,interp_quad);
        }
        //    cout << total_env <<endl;

        for(int i=0; i<(int) mb_xyz_files.size(); i++)
            fprintf(quadmerge_seg_fp,"../mb_proc/%s  %f 0\n",mb_xyz_files[i].c_str(),mb_grd_res);

        if(have_mb_grd){

            int count=0;
            FILE *lafp=fopen("mb_grd/low_alt_mb.txt","r");
            if(lafp){
                while(!feof(lafp)){
                    char tmp[2048];
                    int ret=fscanf(lafp,"%s\n",tmp);
                    if(ret != 1)
                        fprintf(stderr,"Read error lowalt\n");

                    mesh_input m;
                    double zmin,zmax;
                    m.name="mb_grd/grdfiles/"+string(tmp);
                    if(bound_grd(m,zmin,zmax,local_easting,local_northing))
                        total_env.expand_to_include(m.envelope);
                    // cout << total_env <<endl;
                    fprintf(quadmerge_seg_fp,"../mb_grd/grdfiles/%s  %f 0\n",tmp,mb_grd_res);
                    count++;
                }
                fclose(lafp);
                if(count)
                    printf("Using MB GRD found %d lowalt hires files\n",count);
            }
            string background_mb("mb_grd/blendedhi.grd");
            struct stat stFileInfo;
            float background_res=1.0;
            if(stat(background_mb.c_str(),&stFileInfo) == 0 ){
                printf("Found hi-alt background MB GRD file.\n");
                mesh_input m;
                double zmin,zmax;
                m.name=background_mb;
                if(bound_grd(m,zmin,zmax,local_easting,local_northing))
                    total_env.expand_to_include(m.envelope);
                //cout << total_env << endl;;
                fprintf(quadmerge_seg_fp,"../%s  %f 0\n",background_mb.c_str(),
                        background_res);
            }
        }

        fclose(quadmerge_seg_fp);
        //int numcells= total_env.width() *total_env.height() / 50.0;
        std::vector<Cell_Data> quad_cells=calc_cells(tasks,total_env.minx(),total_env.maxx(),total_env.miny(),total_env.maxy(),desired_area);


        FILE *bboxfp_total = fopen("mesh-quad/bbox.txt","w");
        if( !bboxfp_total){
            printf("Unable to open mesh-quad/bbox.txt\n");
        }
        for(unsigned int j=0; j <tasks.size(); j++){
            const Stereo_Pose_Data *pose=&(tasks[j]);

            //Gen Tex File bbox
            fprintf(bboxfp_total, "%d %s " ,pose->id,pose->left_name.c_str());
            save_bbox_frame(pose->bbox,bboxfp_total);
            for(int k=0; k < 4; k++)
                for(int n=0; n < 4; n++)
                    fprintf(bboxfp_total," %lf",pose->m[k][n]);
            fprintf(bboxfp_total,"\n");
        }
        fclose(bboxfp_total);

        for(int i=0; i <(int)quad_cells.size(); i++){



            sprintf(conf_name,"mesh-quad/bbox-clipped-diced-%08d.ply.txt",i);


            bboxfp = fopen(conf_name,"w");
            if( !bboxfp){
                printf("Unable to open %s\n",conf_name);
            }


            fprintf(diced_fp,"clipped-diced-%08d.ply\n",i);
            fprintf(quadmergecmds_fp,"set BASEDIR=\"%s\"; set OUTDIR=\"mesh-quad/\";set VRIP_HOME=\"$BASEDIR/vrip\";setenv VRIP_DIR \"$VRIP_HOME/src/vrip/\";set path = ($path $VRIP_HOME/bin);cd %s/$OUTDIR;",basepath.c_str(),cwd);


            for(int t=0; t < 3; t++){
                fprintf(quadmergecmds_fp,"plycullmaxx %f %f %f %f %f %f %f < ../mesh-quad/quad-lod%d.ply > ../mesh-quad/clipped-diced-%08d-lod%d.ply;",
                        quad_cells[i].bounds.min_x,
                        quad_cells[i].bounds.min_y,
                        -FLT_MAX,
                        quad_cells[i].bounds.max_x,
                        quad_cells[i].bounds.max_y,
                        FLT_MAX,
                        eps,t,i,t);
                if(rugosity){
                    fprintf(quadmergecmds_fp,"auv_mesh_shade ../mesh-quad/clipped-diced-%08d-lod%d.ply rugosity 0.5 ../mesh-quad/tmp-diced-%08d-lod%d.ply; mv ../mesh-quad/tmp-diced-%08d-lod%d.ply ../mesh-quad/clipped-diced-%08d-lod%d.ply;",i,t,i,t,i,t,i,t);
                }
            }
            fprintf(quadmergecmds_fp,"cd ..\n");

            for(unsigned int j=0; j <quad_cells[i].poses.size(); j++){
                const Stereo_Pose_Data *pose=quad_cells[i].poses[j];

                //Gen Tex File bbox
                fprintf(bboxfp, "%d %s " ,pose->id,pose->left_name.c_str());
                save_bbox_frame(pose->bbox,bboxfp);
                for(int k=0; k < 4; k++)
                    for(int n=0; n < 4; n++)
                        fprintf(bboxfp," %lf",pose->m[k][n]);
                fprintf(bboxfp,"\n");
            }



            fclose(bboxfp);
        }
        fclose(quadmergecmds_fp);
        fclose(diced_fp);
        // int lodPick[]={0,0,0,0,1,1,1,1,2,2};

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




            if(use_poisson_recon && !no_merge){
                string runpos_fn = "./runpos.py";
                string poscmd_fn= string("mesh-pos")+"/posgencmds";
                FILE *poscmd_fp=fopen(poscmd_fn.c_str(),"w");
                std::vector<string> precmds;
                std::vector<string> postcmds;
                char cmdtmp[2048];
                precmds.push_back( "cat mesh-agg/pos_pts.bnpts > mesh-pos/pos_out.bnpts");

                if(have_mb_ply){
                    for(int i=0; i <(int) mb_ply_filenames.size(); i++){
                        sprintf(cmdtmp,"%s/poisson/dumpnormpts %s mesh-pos/mb-%03d.bnpts -flip >> mesh-pos/log-dumpmbpts.txt 2>&1" ,
                                basepath.c_str(),
                                mb_ply_filenames[i].c_str(),i);
                        precmds.push_back(cmdtmp);
                        sprintf(cmdtmp,"cat mesh-pos/mb-%03d.bnpts >> mesh-pos/pos_out.bnpts",i);
                        precmds.push_back(cmdtmp);
                    }
                }

                int mintridepth;
                if(have_mb_ply)
                    mintridepth=0;
                else
                    mintridepth=8;

                fprintf(poscmd_fp,"%s/poisson/PoissonRecon --binary --depth %d "
                        "--in mesh-pos/pos_out.bnpts --solverDivide %d --samplesPerNode %f "
                        "--verbose  --out mesh-pos/pos_rec-lod2.ply ",basepath.c_str(),
                        8,6,1.0);
                if( pos_clip)
                    fprintf(poscmd_fp," --mintridepth %d" ,pos_lod2_min_depth);
                fprintf(poscmd_fp,"\n");

                fprintf(poscmd_fp,"%s/poisson/PoissonRecon --binary --depth %d "
                        "--in mesh-pos/pos_out.bnpts --solverDivide %d --samplesPerNode %f "
                        "--verbose  --out mesh-pos/pos_raw.ply ",basepath.c_str(),
                        11,6,4.0);
                if( pos_clip)
                    fprintf(poscmd_fp," --mintridepth %d" ,pos_lod0_min_depth);
                fprintf(poscmd_fp,"\n");

                fclose(poscmd_fp);

                sprintf(cmdtmp,"tridecimator "
                        "mesh-pos/pos_raw.ply mesh-pos/pos_rec-lod0.ply 0 -e15.0 > mesh-pos/log-pos_rec-lod0.ply");
                postcmds.push_back(cmdtmp);
                sprintf(cmdtmp,"tridecimator "
                        "mesh-pos/pos_raw.ply mesh-pos/pos_rec-lod1.ply 0 -e15.0> mesh-pos/log-pos_rec-lod1.ply");
                postcmds.push_back(cmdtmp);
                shellcm.write_generic(runpos_fn,poscmd_fn,"Pos",&precmds,&postcmds);
                if(run_pos){
                    sysres=system(runpos_fn.c_str());
                }
                shellcm.pos_dice(vrip_cells,eps,run_pos);


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
                    sysres=system("./runpos.sh");
                    shellcm.pos_dice(vrip_cells,eps);
                }
            }


            if(!single_run){
                if(quad_integration){
                    string quadmergecmd="runquadmerge.py";
                    std::vector<string> precmds;
                    precmds.push_back(quadprecmd);
                    std::vector<string> postcmds;
                    postcmds.push_back("cp mesh-quad/diced.txt mesh-quad/valid.txt");
                    //postcmds.push_back("cd mesh-quad/;cat valid.txt | xargs plybbox > range.txt");
                    shellcm.write_generic(quadmergecmd,quadmergecmd_fn,"Quadmerge",&precmds,&postcmds);
                    if(!no_quadmerge)
                        sysres=system("./runquadmerge.py");

                }

                vector<string> mergeandcleanCmds;
                mergeandcleanCmds.push_back(shellcm.generateMergeAndCleanCmd(vrip_cells,"tmp-clipped-diced","total",vrip_res));
                //mergeandcleanCmds.push_back("cd mesh-diced;");
                /* string tcmd2;
                char tmp100[8096];

                tcmd2 =basepath+"/texturedDecimator/bin/triGap";

                for(int i=0; i <(int)vrip_cells.size(); i++){
                    if(vrip_cells[i].poses.size() == 0)
                        continue;
                    sprintf(tmp100, " mesh-diced/gap-clipped-diced-%08d.ive ",i);

                  tcmd2+=tmp100;
              }
                tcmd2+= " --thresh 0.1 --outfile mesh-diced/gap-total.ply ";
                mergeandcleanCmds.push_back(tcmd2);

                string tcmd;
                tcmd =basepath+"/vrip/bin/plymerge ";
                for(int i=0; i <(int)vrip_cells.size(); i++){
                    if(vrip_cells[i].poses.size() == 0)
                        continue;
                    sprintf(tmp100, " mesh-diced/tmp-clipped-diced-%08d.ply ",i);

                    tcmd+=tmp100;

                }
                tcmd+= " mesh-diced/gap-total.ply > mesh-diced/total-unmerged.ply;";
                sprintf(tmp100,"  %s/bin/mergeMesh mesh-diced/total-unmerged.ply -thresh %f -out mesh-diced/total.ply",basepath.c_str(),0.9*vrip_res);
                tcmd+=tmp100;
                mergeandcleanCmds.push_back(tcmd);*/

                string vripcmd="runvrip.py";
                shellcm.write_generic(vripcmd,vripcmd_fn,"Vrip",NULL,&mergeandcleanCmds);
                if(!no_vrip)
                    sysres=system("./runvrip.py");
                osg::BoundingBox totalbb;
                osg::BoundingBox totalbb_unrot;

                osg::BoundingSphere bs;
                osg::Matrix rotM;
                float rx=0,ry=180.0,rz=-90;
                int numberFacesAll=0;
                {
                    char tmp[1024];
                    sprintf(tmp,".%d,%d,%d.rot",(int)rx,(int)ry,(int)rz);
                    string rot=string(tmp);

                    rotM =osg::Matrix::rotate(
                            osg::DegreesToRadians( rx ), osg::Vec3( 1, 0, 0 ),
                            osg::DegreesToRadians( ry ), osg::Vec3( 0, 1, 0 ),
                            osg::DegreesToRadians( rz ), osg::Vec3( 0, 0, 1 ) );

                    osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("mesh-diced/total.ply"+rot);
                    osg::Drawable *drawable = model->asGroup()->getChild(0)->asGeode()->getDrawable(0);
                    if(!drawable){
                        fprintf(stderr,"Failed to load model\n");
                        exit(-1);
                    }
                    osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
                    numberFacesAll=geom->getPrimitiveSet(0)->getNumPrimitives();

                    if(!model.valid() || !model->getBound().radius()){
                        std::cerr << " Cant open total.ply\n";
                        exit(-1);
                    }
                    bs=model->getBound();
                    osgUtil::Optimizer::FlattenStaticTransformsVisitor fstv(NULL);
                    model->accept(fstv);
                    fstv.removeTransforms(model);
                    osgDB::writeNodeFile(*model,"mesh-diced/totalrot.ive");
                    osg::ComputeBoundsVisitor cbbv(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
                    model->accept(cbbv);
                    totalbb = cbbv.getBoundingBox();
                    totalbb_unrot.expandBy(totalbb._min*osg::Matrix::inverse(rotM));
                    totalbb_unrot.expandBy(totalbb._max*osg::Matrix::inverse(rotM));

                }

                int vpblod=1;
                int faceTmp=numberFacesAll;
                do{
                    faceTmp /= pow(4,vpblod++);
                }while(faceTmp > targetFaces);
                std::cout << "Target LOD height is : " << vpblod <<std::endl;

                osg::Vec3d eye(totalbb.center()+osg::Vec3(0,0,3.5*totalbb.radius()));
                osg::Matrixd matrix;
                matrix.makeTranslate( eye );
                osg::Matrixd view=osg::Matrix::inverse(matrix);
                osg::Matrixd proj= osg::Matrixd::ortho2D(-bs.radius(),bs.radius(),-bs.radius(),bs.radius());

                std::stringstream os2;
                os2<< "view.mat";

                std::fstream _file(os2.str().c_str(),std::ios::binary|std::ios::out);
                for(int i=0; i<4; i++)
                    for(int j=0; j<4; j++)
                        _file.write(reinterpret_cast<char*>(&(view(i,j))),sizeof(double));
                for(int i=0; i<4; i++)
                    for(int j=0; j<4; j++)
                        _file.write(reinterpret_cast<char*>(&(proj(i,j))),sizeof(double));
                _file.close();



                std::fstream _file2("rot.mat",std::ios::binary|std::ios::out);
                for(int i=0; i<4; i++)
                    for(int j=0; j<4; j++)
                        _file2.write(reinterpret_cast<char*>(&(rotM(i,j))),sizeof(double));
                _file2.close();

                std::vector<picture_cell> cells;

                char tmp4[1024];
                for(int row=0; row< _tileRows; row++){
                    for(int col=0; col<_tileColumns; col++){
                        osg::Matrix offsetMatrix=   osg::Matrix::scale(_tileColumns, _tileRows, 1.0) *osg::Matrix::translate(_tileColumns-1-2*col, _tileRows-1-2*row, 0.0);
                        double left,right,bottom,top,znear,zfar;
                        (view*proj*offsetMatrix).getOrtho(left,right,bottom,top,znear,zfar);

                        osg::BoundingBox thisCellBbox(left,bottom,bs.center()[2]-bs.radius(),right,top,bs.center()[2]+bs.radius());

                        picture_cell cell;
                        cell.bbox=thisCellBbox;
                        cell.row=row;
                        cell.col=col;
                        sprintf(tmp4,"mesh-diced/tex-clipped-diced-r_%04d_c_%04d-lod%d.ive",row,col,vpblod);
                        cell.name=string(tmp4);
                        for(int i=0; i < (int)tasks.size(); i++){
                            osg::Vec3 m1=osg::Vec3(tasks[i].pose[AUV_POSE_INDEX_X]-tasks[i].radius,tasks[i].pose[AUV_POSE_INDEX_Y]-tasks[i].radius,tasks[i].pose[AUV_POSE_INDEX_Z]-tasks[i].radius)*rotM;
                            osg::Vec3 m2=osg::Vec3(tasks[i].pose[AUV_POSE_INDEX_X]+tasks[i].radius,tasks[i].pose[AUV_POSE_INDEX_Y]+tasks[i].radius,tasks[i].pose[AUV_POSE_INDEX_Z]+tasks[i].radius)*rotM;

                            osg::BoundingBox imgBox;
                            imgBox.expandBy(m1);
                            imgBox.expandBy(m2);
                            //  cout << m1 << " "<< m2 << " bounds \n";
                            //        cout <<thisCellBbox._min << " "<< thisCellBbox._max<<" bbox\n";
                            if(thisCellBbox.intersects(imgBox)){
                                cell.images.push_back(i);
                            }
                        }
                        cells.push_back(cell);
                    }
                }

                //  fprintf(reFP,"%f %f %f %f clipped-diced-%08d-lod%d.ive\n",vrip_cells[i].bounds.min_x,vrip_cells[i].bounds.max_x,vrip_cells[i].bounds.min_y,vrip_cells[i].bounds.max_y,i,vpblod);

                string splitcmds_fn="mesh-diced/splitcmds";

                FILE *splitcmds_fp=fopen(splitcmds_fn.c_str(),"w");
                FILE *reFP=fopen("rebbox.txt","w");
                fprintf(reFP,"%.16f %.16f %.16f %.16f %.16f %.16f %d %d total\n",totalbb.xMin(),totalbb.xMax(),totalbb.yMin(),totalbb.yMax(),totalbb.zMin(),
                        totalbb.zMax(),_tileColumns,_tileRows);

                for(int i=0; i <(int)cells.size(); i++){

                    if(cells[i].images.size() == 0){
                        fprintf(reFP,"%.16f %.16f %.16f %.16f %.16f %.16f %d %d %s\n",cells[i].bbox.xMin(),cells[i].bbox.xMax(),cells[i].bbox.yMin(),cells[i].bbox.yMax(),cells[i].bbox.zMin(),
                                cells[i].bbox.zMax(),cells[i].col,cells[i].row,"null");
                        continue;

                    }
                    fprintf(reFP,"%.16f %.16f %.16f %.16f %.16f %.16f %d %d %s\n",cells[i].bbox.xMin(),cells[i].bbox.xMax(),cells[i].bbox.yMin(),cells[i].bbox.yMax(),cells[i].bbox.zMin(),
                            cells[i].bbox.zMax(),cells[i].col,cells[i].row,cells[i].name.c_str());
                    fprintf(splitcmds_fp,"cd %s;%s/treeBBClip mesh-diced/totalrot.ive %.16f %.16f %.16f %.16f %.16f %.16f -dup --outfile mesh-diced/tex-clipped-diced-r_%04d_c_%04d.ply;",
                            cwd,
                            basepath.c_str(),
                            cells[i].bbox.xMin(),
                            cells[i].bbox.yMin(),
                            -FLT_MAX,
                            cells[i].bbox.xMax(),
                            cells[i].bbox.yMax(),
                            FLT_MAX,
                            cells[i].row,cells[i].col);
                    fprintf(splitcmds_fp,"cp mesh-diced/tex-clipped-diced-r_%04d_c_%04d.ply mesh-diced/tex-clipped-diced-r_%04d_c_%04d-lod%d.ply \n",
                            //basepath.c_str(),
                            cells[i].row,cells[i].col,  cells[i].row,cells[i].col,vpblod);
                    char tp[1024];
                    sprintf(tp,"mesh-diced/bbox-tex-clipped-diced-r_%04d_c_%04d.ply.txt",cells[i].row,cells[i].col);
                    FILE *bboxfp=fopen(tp,"w");

                    for(int k=0; k < (int)cells[i].images.size(); k++){
                        const Stereo_Pose_Data *pose=(&tasks[cells[i].images[k]]);
                        if(pose && pose->valid){
                            fprintf(bboxfp, "%d %s " ,pose->id,pose->left_name.c_str());
                            save_bbox_frame(pose->bbox,bboxfp);
                            for(int k=0; k < 4; k++)
                                for(int n=0; n < 4; n++)
                                    fprintf(bboxfp," %lf",pose->m[k][n]);
                            fprintf(bboxfp,"\n");
                        }

                    }
                    fclose(bboxfp);


                }
                for(int i=0; i <(int)vrip_cells.size(); i++){
                    if(vrip_cells[i].poses.size() == 0)
                        continue;
                    fprintf(splitcmds_fp,"cd %s;%s/treeBBClip mesh-diced/total.ply %f %f %f %f %f %f -dup --outfile mesh-diced/clipped-diced-%08d.ply;",
                            cwd,
                            basepath.c_str(),
                            vrip_cells[i].bounds.min_x,
                            vrip_cells[i].bounds.min_y,
                            -FLT_MAX,
                            vrip_cells[i].bounds.max_x,
                            vrip_cells[i].bounds.max_y,
                            FLT_MAX,
                            i);
                    fprintf(splitcmds_fp,"cp mesh-diced/clipped-diced-%08d.ply mesh-diced/clipped-diced-%08d-lod%d.ply \n",
                            //basepath.c_str(),
                            i,i,vpblod);

                }

                fclose(splitcmds_fp);
                fclose(reFP);

                string splitcmd="split.py";
                shellcm.write_generic(splitcmd,splitcmds_fn,"Split");
                if(!no_split)
                    sysres=system("./split.py");

                std::vector<int > sizeStepTotal(vpblod+1);

                for(int j=vpblod; j >=0; j--){
                    sizeStepTotal[j]= numberFacesAll/(pow(4,vpblod-j));
                    printf("%d\n",sizeStepTotal[j]);

                }

                double minFrac=0.2;
                std::vector<int> numFaces(vrip_cells.size(),0);
                std::vector<double> resFrac(vrip_cells.size(),0);
                std::vector<double> cur_res(vrip_cells.size(),0);

                int totalFaces=0;
                int largestNumFaces=0;
                double desiredRes=0.1;
                for(int i=0; i <(int)vrip_cells.size(); i++){
                    if(vrip_cells[i].poses.size() == 0)
                        continue;
                    char tmpf[1024];
                    sprintf(tmpf,"mesh-diced/clipped-diced-%08d-lod%d.ply",i,vpblod);
                    osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(tmpf);
                    assert(model.valid());
                    if(!model.valid()){
                        OSG_ALWAYS<<"No valid model ";
                        exit(-1);
                    }
                    osg::Drawable *drawable = model->asGeode()->getDrawable(0);
                    osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
                    int faces=geom->getPrimitiveSet(0)->getNumPrimitives();
                    totalFaces+=faces;
                    numFaces[i]=faces;
                    if(faces > largestNumFaces)
                        largestNumFaces=faces;
                    cur_res[i]=vrip_cells[i].bounds.area()/totalFaces;
                    double dRes=desiredRes-cur_res[i];
                    resFrac[i]=(dRes/(float)vpblod);

                    printf("%d: %d \n",i,faces);
                }
                //                    sizeStep[i]=(int)round(numFaces[i]*resFrac);
                //                 printf("Step size %d %f\n",sizeSteps[i],resFrac);

                //  printf("Total Faces %d %f\n",totalFaces,cur_res);
                // printf("%f %f dres %f\n",cur_res,desiredRes,dRes);
                std::vector< std::vector<int > >sizeStep;
                sizeStep.resize(vrip_cells.size());
                for(int i=0; i< (int)vrip_cells.size(); i++){
                    sizeStep[i].resize(vpblod+1);
                    for(int j=vpblod; j >=0; j--){
                        if(vrip_cells[i].poses.size() == 0){
                            sizeStep[i][j]=0;
                            continue;
                        }
                        double newRes=(cur_res[i]+(vpblod-j)*resFrac[i]);
                        sizeStep[i][j]= std::max(numFaces[i]/(pow(4,vpblod-j)),(largestNumFaces*minFrac));
                        OSG_ALWAYS << "Level " << j << "Res " <<  newRes << "Orig Faces " << numFaces[i] << "New faces " << sizeStep[i][j] <<endl;
                    }
                    //                    sizeStep[i]=(int)round(numFaces[i]*resFrac);
                    //                 printf("Step size %d %f\n",sizeSteps[i],resFrac);
                }

                string texcmds_fn="mesh-diced/texcmds";

                FILE *texcmds_fp=fopen(texcmds_fn.c_str(),"w");

                for(int i=0; i <(int)cells.size(); i++){
                    if(cells[i].images.size() == 0)
                        continue;
                    fprintf(texcmds_fp,"cd %s;%s/calcTexCoord %s mesh-diced/tex-clipped-diced-r_%04d_c_%04d-lod%d.ply --outfile mesh-diced/tex-clipped-diced-r_%04d_c_%04d-lod%d.ply",
                            cwd,
                            basepath.c_str(),
                            base_dir.c_str(),
                            cells[i].row,cells[i].col,
                            vpblod,
                            cells[i].row,cells[i].col,
                            vpblod);
                    fprintf(texcmds_fp," --tex_cache %s %d --invrot %f %f %f\n",cachedtexdir[0].first.c_str(),cachedtexdir[0].second,rx,ry,rz);


                }
                fclose(texcmds_fp);
                int tileBorder=1;

                string texcmd="tex.py";
                vector<std::string> postcmdv;
                std::ostringstream p;
                int tileSize=256;

                p << basepath << "/dicedImage rebbox.txt  " << cwd  << " --pbuffer-only " << (int)reimageSize.x()-((reimageSize.x()/tileSize)*2*tileBorder) << " "<< (int)reimageSize.y()-((reimageSize.y()/tileSize)*2*tileBorder)  << setprecision(28) <<" -lat " << latOrigin << " -lon " << longOrigin;
                if(untex)
                    p<< " -untex " << totalbb_unrot.zMin() << " " << totalbb_unrot.zMax();
                postcmdv.push_back(p.str());
#define SINGLE_MESH_TEX 1
#if SINGLE_MESH_TEX
                std::ostringstream p2;
                p2 << basepath << "/singleImageTex " << "mesh-diced/total.ply --outfile mesh-diced/totaltex.ply";
                postcmdv.push_back(p2.str());


#endif

                //int sizeX=reimageSize.x()*_tileRows;
                //int adjustedSize=tileSize-(2*tileBorder);
                // int intDiv=sizeX/tileSize;
                //int embedSize=(intDiv* adjustedSize);
                std::ostringstream p3;
                // p3 << "vips " << " im_extract_area " << "out.tif "<< " tex.tif " << " 0 0 " <<  embedSize << " "<<embedSize<< ";";

                p3 << basepath << "/generateVirtualTextureTiles.py " << "-f=jpg  -b="<<tileBorder<<" tex.tif ";
                postcmdv.push_back(p3.str());

                shellcm.write_generic(texcmd,texcmds_fn,"Tex",NULL,&(postcmdv),num_threads);
                if(!no_tex)
                    sysres=system("./tex.py");

                if(useVirtTex){
                    string vttexcmds_fn="mesh-diced/vttexcmds";
                    string vttex="vttex.py";

                    FILE *vttexcmds_fp=fopen(vttexcmds_fn.c_str(),"w");
                    for(int i=0; i <(int)vrip_cells.size(); i++){
                        if(vrip_cells[i].poses.size() == 0)
                            continue;
                        fprintf(vttexcmds_fp,"%s/singleImageTex mesh-diced/clipped-diced-%08d.ply --outfile mesh-diced/clipped-diced-%08d-lod%d.ply\n",
                                basepath.c_str(),
                                i,i,vpblod);
                    }
                    fclose(vttexcmds_fp);
                    shellcm.write_generic(vttex,vttexcmds_fn,"VT Tex",NULL,NULL,num_threads);
                    if(!no_vttex)
                        sysres=system("./vttex.py");

                }


                string simpcmds_fn="mesh-diced/simpcmds";

                FILE *simpcmds_fp=fopen(simpcmds_fn.c_str(),"w");
                string app;
                if(0)
                    app="tridecimator";
                else
                    app="texturedDecimator";
                std::vector<std::vector<string> > datalist_lod;
                vector<string> mergeandcleanCmdsSimp;

#ifdef SINGLE_MESH_TEX


                {
                    char tmp[1024];

                    fprintf(simpcmds_fp,"cd %s/mesh-diced;cp totaltex.ply total-lod%d.ply;",cwd,vpblod);
                    sprintf(tmp,"mesh-diced/total-lod%d.ply",vpblod);//std::min(lod,2)
                    std::vector<string> level;

                    for(int j=vpblod-1; j >= 0; j--){

                            fprintf(simpcmds_fp,"cd %s/mesh-diced;%s/texturedDecimator/bin/%s total-lod%d.ply total-lod%d.ply %d -P -Oy -By;",
                                    cwd,
                                    basepath.c_str(),
                                    app.c_str(),
                                    j+1,j, sizeStepTotal[j]);

                    }
                    for(int lod=0; lod <= vpblod; ){
                        std::vector<string> level;
                        if(datalist_lod.size() >3)
                            lod ++;
                        //for(int lod=0; lod <= vpblod; lod ++){

                        char tmp[1024];
                        sprintf(tmp,"mesh-diced/total-lod%d.ply",lod);//std::min(lod,2)
                        level.push_back(tmp);


                        datalist_lod.push_back(level);

                    }
                }
#else
                for(int i=0; i <(int)vrip_cells.size(); i++){
                    if(vrip_cells[i].poses.size() == 0)
                        continue;
                    for(int j=vpblod; j >0; j--){
                        fprintf(simpcmds_fp,"cd %s/mesh-diced;%s/texturedDecimator/bin/%s clipped-diced-%08d-lod%d.ply clipped-diced-%08d-lod%d.ply %d -By -P;",
                                cwd,
                                basepath.c_str(),
                                app.c_str(),
                                i,j,i,j-1, sizeStep[i][j-1]);
                    }
                    fprintf(simpcmds_fp,"\n");
                }
                for(int i=vpblod-2; i >= 0; i--){
                    char tmp8[8192];
                    sprintf(tmp8,"mesh-diced/tmp-total-lod%d.ply",i);
                    mergeandcleanCmdsSimp.push_back(shellcm.generateMergeAndCleanCmd(vrip_cells,"clipped-diced","tmp-total",vrip_res,i));
                    /* osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(tmp8);
                    assert(model.valid());
                    if(!model.valid()){
                        OSG_ALWAYS<<"No valid model ";
                        exit(-1);
                    }
                    osg::Drawable *drawable = model->asGeode()->getDrawable(0);
                    osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
                    int faces=geom->getPrimitiveSet(0)->getNumPrimitives();
                    if(faces > sizeStepTotal[i]){*/
                    if(1){
                        char srcfile[1024];
                        if(i==vpblod-2)
                            sprintf(srcfile,"tmp-total-lod%d.ply",i);
                        else
                            sprintf(srcfile,"total-lod%d.ply",i+1);

                        sprintf(tmp8,"cd %s/mesh-diced;time %s/texturedDecimator/bin/%s %s total-lod%d.ply %d -By -P;",
                                cwd,
                                basepath.c_str(),
                                app.c_str(),
                                srcfile,i,sizeStepTotal[i]);
                        mergeandcleanCmdsSimp.push_back(tmp8);
                    }
                }
                for(int lod=0; lod <= vpblod; ){
                    std::vector<string> level;
                    if(datalist_lod.size() >3)
                        lod ++;
                    //   for(int i=0; i <(int)cells.size(); i++){
                    //     if(cells[i].images.size() == 0)
                    for(int i=0; i <(int)vrip_cells.size(); i++){
                        if(vrip_cells[i].poses.size() == 0)
                            continue;
                        char tmp[1024];
                        sprintf(tmp,"mesh-diced/clipped-diced-%08d-lod%d.ply",i,lod);//std::min(lod,2)
                        level.push_back(tmp);
                    }


                    datalist_lod.push_back(level);
                }
#endif
                fclose(simpcmds_fp);
                string simpcmd="simp.py";
                shellcm.write_generic(simpcmd,simpcmds_fn,"simp",NULL,&mergeandcleanCmdsSimp);
                if(!no_simp)
                    sysres=system("./simp.py");




                if(!mgc)
                    mgc = new MyGraphicsContext();
                bool useVirtTex=true;
                bool useReimage=false;
                doQuadTreeVPB(cachedsegtex,datalist_lod,bounds,calib->left_calib,cachedtexdir,useTextureArray,useReimage,useVirtTex,totalbb);


                vector<string> gentexnames;
                gentexnames.push_back("./gentex.py");
                gentexnames.push_back("./posgentex.py");
                gentexnames.push_back("./quadgentex.py");

                vector<string> gentexdir;
                gentexdir.push_back("mesh-diced");
                gentexdir.push_back("mesh-pos");
                gentexdir.push_back("mesh-quad");
                const int quad_idx=2;
                std::vector<string> precmds;
                if(no_simp){
                    string cmdtmp="cd mesh-diced;cat diced.txt | xargs plybbox > range.txt;cp diced.txt valid.txt;cd ..";
                    precmds.push_back(cmdtmp);
                }
                string gentexcmd_fn;
                for(int i=0; i <(int)gentexdir.size(); i++){
                    gentexcmd_fn=(gentexdir[i]+"/gentexcmds");
                    FILE *dicefp=fopen(gentexcmd_fn.c_str(),"w+");


                    char argstr[2048];
                    strcpy(argstr,"");
                    if(do_novelty)
                        strcat(argstr," --novelty --planedist ");
                    if(do_shader_color)
                        strcat(argstr," --shader ");
                    if(do_hw_blend)
                        strcat(argstr," --blend ");
                    if(do_classes || do_classes_interp){
                        char tp[2048];
                        sprintf(tp," --classes %s ","classid.txt");
                        strcat(argstr,tp);
                    }
                    if(!hardware_compress)
                        strcat(argstr," --no-hardware-compress ");
                    if(no_simp)
                        strcat(argstr," --nosimp");
                    if(use_proj_tex)
                        strcat(argstr," --projtex");
                    if(have_mb_ply){
                        char tp[2048];
                        if(i ==quad_idx)
                            sprintf(tp," --nonvis %d ",(int)quad_cells.size());
                        else
                            sprintf(tp," --nonvis %d ",(int)vrip_cells.size());
                        strcat(argstr,tp);
                    }
                    int gentex_limit= (i==quad_idx) ?quad_cells.size() : vrip_cells.size();

                    if(gentex_limit < num_threads*dist_gentex_range)
                        dist_gentex_range = (int) std::max(gentex_limit/num_threads,1);
                    int j1=0;
                    do{
                        fprintf(dicefp,"setenv DISPLAY :0.0;cd %s/..;%s/genTex %s %s "
                                "-f %s  --dicedir %s/ --stereo-calib %s --range-run %d %d %s\n"
                                ,gentexdir[i].c_str(),basepath.c_str(),
                                recon_config_file_name.c_str(),
                                recon_config_file_name.c_str(),
                                cachedtexdir[0].first.c_str(),gentexdir[i].c_str(),
                                stereo_calib_file_name.c_str(),j1,j1+dist_gentex_range,argstr);

                        j1+=dist_gentex_range;

                    }while(j1 <= gentex_limit);


                    fclose(dicefp);
                    int thread_override= do_hw_blend ? (num_threads/2):0;
                    shellcm.write_generic(gentexnames[i],gentexcmd_fn,"Gentex",&precmds,NULL,thread_override);
                    if(no_gen_tex)
                        continue;
                    if(i==0 && !no_vrip)
                        sysres=system(gentexnames[i].c_str());
                    if(i==1 && run_pos)
                        sysres=system(gentexnames[i].c_str());
                    if(i==2 && !no_quadmerge)
                        sysres=system(gentexnames[i].c_str());

                }

                if(!no_gen_tex || use_poisson_recon){
                    FILE *lodfp=fopen("lodgen.sh","w");
                    char ar[2048];
                    if(run_pos)
                        strcpy(ar,"--dicedir mesh-pos/");
                    else if(quad_integration && !no_quadmerge)
                        strcpy(ar,"--dicedir mesh-quad/");
                    else
                        strcpy(ar,"--dicedir mesh-diced/");
                    fprintf(lodfp,"#!/bin/bash\n"
                            "echo LODGen... \n"
                            "%s/lodgen %s\n",
                            basepath.c_str(),ar);

                    fchmod(fileno(lodfp),0777);
                    fclose(lodfp);
                    if(!no_gen_tex )
                        sysres=system("./lodgen.sh");
                }
            }
        }
    }

    end_timer = time(NULL);
    gdouble total_elapsed =  end_timer - start_timer;

    gdouble hours, mins, secs;
    hours = floor (total_elapsed/3600.);
    mins = floor ((total_elapsed - 3600.*hours)/60.);
    secs = floor (total_elapsed - 3600.*hours - 60.*mins);
    printf("Total Time for Completion: %02.0f:%02.0f:%02.0f\n",hours,mins,secs);
    //
    // Clean-up
    //

    //  for(int i =0; i < (int)tasks.size(); i++)
    // delete tasks[i].veh_pose;
    //delete config_file;

    if(have_cov_file && cov_file)
        delete cov_file;


}








