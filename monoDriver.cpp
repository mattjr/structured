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
using namespace std;
using namespace libplankton;
using namespace ulapack;
using namespace libsnapper;
#define DEFAULT_NUM_THREADS (int)(get_num_processors( )*1.5)


static int meshNum;
static double start_time = 0.0;
static bool apply_aug=false;
static double stop_time = numeric_limits<double>::max();
//
// Command-line arguments
//
int proj_tex_size;
int poster_tiles=40;
static bool rugosity=false;
static int vpblod=3;

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
static bool run_pos=false;
static bool do_novelty=false;
static double dense_scale;
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
static  vector<double>simp_res(255);
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

static bool use_dense_feature=false;
//StereoMatching* stereo;
//StereoImage simage;
static bool gen_mb_ply=false;
static bool no_atlas=false;
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
char cachedtexdir[2048];
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

int pos_lod0_min_depth,pos_lod2_min_depth;
int pos_lod0_depth,pos_lod2_depth;
void runC(Stereo_Pose_Data &name);


typedef struct _class_id_t{
    int class_id;
    string name;
    double time;
}class_id_t;
void save_bbox_frame (GtsBBox * bb, FILE * fptr){
    g_return_if_fail (bb != NULL);

    fprintf (fptr, "%g %g %g %g %g %g",
             bb->x1, bb->y1, bb->z1,
             bb->x2, bb->y2, bb->z2);

}


static int get_mono_image_name( const string  &contents_dir_name,
                                ifstream      &contents_file,
                                Stereo_Pose_Data  &name
                                )
{

    //
    // Try to read timestamp and file names
    //
    bool readok;
    int index;
    osg::Vec3 minV;
    osg::Vec3 maxV;

    do{

        readok =(contents_file >> name.id &&
                 contents_file >> name.left_name &&

                 contents_file >> minV.x()&&
                 contents_file >> minV.y() &&
                 contents_file >> minV.z() &&
                 contents_file >> maxV.x()&&
                 contents_file >> maxV.y()&&
                 contents_file >> maxV.z());
        name.m =gts_matrix_identity (NULL);
        double xcen=maxV.x()-minV.x();
        double ycen=maxV.y()-minV.y();
        for(int i=0; i < 4; i++)
            for(int j=0; j<4; j++)
                contents_file >> name.m[i][j];
        name.bbox = gts_bbox_new(gts_bbox_class(),NULL,minV.x(),minV.y(),minV.z(),maxV.x(),maxV.y(),maxV.z());
        name.radius= sqrt(pow(fabs(xcen)/2.0,2.0)+pow(fabs(ycen)/2.0,2.0));
        name.mesh_name=contents_dir_name+"../"+osgDB::getNameLessExtension(name.left_name)+".ply";
        name.pose[0] = xcen;
        name.pose[1] = ycen;


    }
    while (readok && (name.time < start_time || (skip_counter++ < num_skip)));
    skip_counter=0;

    if(!readok || name.time >= stop_time) {
        // we've reached the end of the contents file
        return END_FILE;
    }
    //if (name.img_name == "DeltaT")
    //  return NO_ADD;


    return ADD_IMG;

}

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
    contents_file_name = "camboxdata.txt";

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
    recon_config_file->get_value("SIMP_RES_1",simp_res[0],0.005);
    recon_config_file->get_value("SIMP_RES_2",simp_res[1],0.1);
    recon_config_file->get_value("SIMP_RES_3",simp_res[2],0.5);
    recon_config_file->get_value("DIST_GENTEX_RANGE",dist_gentex_range,10);
    recon_config_file->get_value("POS_LOD2_MIN_DEPTH",pos_lod2_min_depth,6);
    recon_config_file->get_value("POS_LOD2_DEPTH",pos_lod2_depth,8);

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
    sprintf(cachedtexdir,"cache-tex-%d/",lodTexSize[0]);
    sprintf(cachedsegtex,"cache-seg-coords/");


    string mbfile;

    argp.read("--grdres",mb_grd_res);
    spline_dist=max((int)round(1.0/mb_grd_res),5);
    argp.read("--spline_dist",spline_dist);
    if(  argp.read("--clean"))
        further_clean=true;
    argp.read("-poster-scale",poster_tiles);

    argp.read("-r",image_scale);
    argp.read( "--edgethresh" ,edgethresh);
    argp.read("-m", max_feature_count );
    argp.read("-f",dir_name);
    have_mb_ply=argp.read("--mbfile",mbfile);
    if(have_mb_ply)
        mb_ply_filenames.push_back(mbfile) ;

    argp.read( "--monoskip" ,mono_skip);
    argp.read( "--vpblod" ,vpblod);

    if(argp.read(  "--noposclip"))
        pos_clip=false;

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
    use_undistorted_images = argp.read("-u" );
    use_proj_tex=argp.read("--projtex");

    if(argp.read("--evensplit" ))
        even_split= true;
    argp.read("--cellscale",cell_scale );




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
    no_gen_tex=argp.read("--nogentex");
    if(argp.read("--novrip"))
        no_vrip=true;
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
    strcpy(cachedtexdir,string(base_dir+string("/")+cachedtexdir).c_str());
    strcpy(cachedsegtex,string(base_dir+string("/")+cachedsegtex).c_str());


    recon_config_file->set_value( "SKF_SHOW_DEBUG_IMAGES" , display_debug_images );
    recon_config_file->set_value( "SCF_SHOW_DEBUG_IMAGES"  , display_debug_images );
    recon_config_file->set_value( "SD_SHOW_DEBUG_IMAGES"  , display_debug_images );
    simp_res[0]=0.1;
    simp_res[1]=0.1;
    simp_res[2]=0.1;

    for(int i=3;i < vpblod; i++)
        simp_res[i]=0.2;//(simp_res[i-1]*1.2);
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

int main( int argc, char *argv[ ] )
{
    start_timer = time(NULL);

    FILE *rerunfp=fopen("rerun.sh","w");
    FILE *timing_fp=fopen("timing.txt","w");
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
    auv_data_tools::makedir(cachedtexdir);
    chmod(cachedtexdir,   0777);
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


    std::vector<Stereo_Pose_Data> tasks;

    int stereo_pair_count=0;
    while( !have_max_frame_count || (!have_max_frame_count || stereo_pair_count < max_frame_count)  ){

        Stereo_Pose_Data mono;
        int ret=get_mono_image_name( dir_name, contents_file, mono) ;
        if(ret == ADD_IMG ){
            stereo_pair_count++;
            //   mono.id= single_run_start+mono_img_count++;
            mono.valid=true;
            // mono_names.push_back(mono);
            tasks.push_back(mono);
        }else if(ret == NO_ADD){
            printf("Noadd\n");
            continue;
        }else if(ret == END_FILE)
            break;
    }



    if(tasks.size() <= 0){
        fprintf(stderr,"No tasks loaded check %s\n",contents_file_name.c_str());
        exit(-1);
    }
    start_time=tasks[0].time;
    stop_time=tasks[tasks.size()-1].time;



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
                    "%s.ply %f 1\n"
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

        fprintf(vripcmds_fp,"$BASEDIR/vrip/bin/vripnew auto-%08d.vri ../%s ../%s %f -rampscale %f;$BASEDIR/vrip/bin/vripsurf auto-%08d.vri ../mesh-agg/seg-%08d.ply %s ;",i,vrip_seg_fname,vrip_seg_fname,vrip_res,vrip_ramp,i,i,redirstr);

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
    std::vector<std::vector<string> > datalist_lod;
    for(int lod=0; lod < vpblod; lod ++){
        std::vector<string> level;
        if(!no_quadmerge){
            for(int i=0; i <(int)quad_cells.size(); i++){
                char tmp[1024];
                sprintf(tmp,"mesh-quad/clipped-diced-%08d-lod%d.ply",i,lod);//std::min(lod,2)
                level.push_back(tmp);
            }
        }else{
            for(int i=0; i <(int)vrip_cells.size(); i++){
                char tmp[1024];
                sprintf(tmp,"mesh-diced/clipped-diced-%08d-lod%d.ply",i,lod);//std::min(lod,2)
                level.push_back(tmp);
            }
        }

        datalist_lod.push_back(level);
    }
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
            //mergeandcleanCmds.push_back("cd mesh-diced;");
            char tmp100[8096];
            string tcmd;
            tcmd =basepath+"/vrip/bin/plymerge ";
            for(int i=0; i <(int)vrip_cells.size(); i++){
                if(vrip_cells[i].poses.size() == 0)
                    continue;
                sprintf(tmp100, " mesh-diced/tmp-clipped-diced-%08d.ply ",i);
                tcmd+=tmp100;
            }
            tcmd+= " > mesh-diced/total-unmerged.ply;";
            sprintf(tmp100,"  %s/bin/mergeMesh mesh-diced/total-unmerged.ply -thresh %f -out mesh-diced/total.ply",basepath.c_str(),0.9*vrip_res);
            tcmd+=tmp100;
            mergeandcleanCmds.push_back(tcmd);

            string vripcmd="runvrip.py";
            shellcm.write_generic(vripcmd,vripcmd_fn,"Vrip",NULL,&mergeandcleanCmds);
            if(!no_vrip)
                sysres=system("./runvrip.py");

            string splitcmds_fn="mesh-diced/splitcmds";

            FILE *splitcmds_fp=fopen(splitcmds_fn.c_str(),"w");

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
            string splitcmd="split.py";
            shellcm.write_generic(splitcmd,splitcmds_fn,"Split");
            if(!no_vrip)
                sysres=system("./split.py");

            std::vector<int> numFaces(vrip_cells.size(),0);
            std::vector<double> resFrac(vrip_cells.size(),0);
            std::vector<double> cur_res(vrip_cells.size(),0);

            int totalFaces=0;
            double desiredRes=0.1;
            for(int i=0; i <(int)vrip_cells.size(); i++){
                if(vrip_cells[i].poses.size() == 0)
                    continue;
                char tmpf[1024];
                sprintf(tmpf,"mesh-diced/clipped-diced-%08d-lod%d.ply",i,vpblod);
                osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(tmpf);
                if(!model.valid()){
                    cerr << tmpf << "not valid\n";
                    continue;
                }
                osg::Drawable *drawable = model->asGeode()->getDrawable(0);
                osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
                int faces=geom->getPrimitiveSet(0)->getNumPrimitives();
                totalFaces+=faces;
                numFaces[i]=faces;
                cur_res[i]=vrip_cells[i].bounds.area()/totalFaces;
                double dRes=desiredRes-cur_res[i];
                resFrac[i]=(dRes/(float)vpblod);

                printf("%d: %d \n",i,faces);
            }


            /*   printf("Total Faces %d %f\n",totalFaces,cur_res);
                printf("%f %f dres %f\n",cur_res,desiredRes,dRes);*/
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
                    sizeStep[i][j]=vrip_cells[i].bounds.area()/newRes;
                    OSG_ALWAYS << "Level " << j << "Res " <<  newRes << "Orig Faces " << numFaces[i] << "New faces " << sizeStep[i][j] <<endl;
                }
                //                    sizeStep[i]=(int)round(numFaces[i]*resFrac);
                //                 printf("Step size %d %f\n",sizeSteps[i],resFrac);
            }
            string texcmds_fn="mesh-diced/texcmds";

            FILE *texcmds_fp=fopen(texcmds_fn.c_str(),"w");

            for(int i=0; i <(int)vrip_cells.size(); i++){
                if(vrip_cells[i].poses.size() == 0)
                    continue;
                fprintf(texcmds_fp,"cd %s;%s/calcTexCoord %s mesh-diced/clipped-diced-%08d-lod%d.ply --outfile mesh-diced/clipped-diced-%08d-lod%d.ply\n",
                        cwd,
                        basepath.c_str(),
                        base_dir.c_str(),
                        i,
                        vpblod,
                        i,
                        vpblod);
            }
            fclose(texcmds_fp);
            string texcmd="tex.py";
            shellcm.write_generic(texcmd,texcmds_fn,"Tex");
            if(!no_vrip)
                sysres=system("./tex.py");


            string simpcmds_fn="mesh-diced/simpcmds";

            FILE *simpcmds_fp=fopen(simpcmds_fn.c_str(),"w");

            for(int i=0; i <(int)vrip_cells.size(); i++){
                if(vrip_cells[i].poses.size() == 0)
                    continue;
                for(int j=vpblod; j >0; j--){
                    fprintf(simpcmds_fp,"cd %s/mesh-diced;%s/texturedDecimator/bin/texturedDecimator clipped-diced-%08d-lod%d.ply clipped-diced-%08d-lod%d.ply %d -P;",
                            cwd,
                            basepath.c_str(),
                            i,vpblod,i,j-1, sizeStep[i][j]);
                }
                fprintf(simpcmds_fp,"\n");
            }
            fclose(simpcmds_fp);
            string simpcmd="simp.py";
            shellcm.write_generic(simpcmd,simpcmds_fn,"simp");
            if(!no_vrip)
                sysres=system("./simp.py");

            if(!mgc)
                mgc = new MyGraphicsContext();
            texcache_t cached_dir_name;
            cached_dir_name.push_back(make_pair<string,int>(dir_name,1024));
            doQuadTreeVPB(cachedsegtex,datalist_lod,bounds,calib->left_calib,cached_dir_name,useTextureArray,true,false);


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
                            cachedtexdir,gentexdir[i].c_str(),
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

            {
                char dicedir[2048];
                if(run_pos)
                    strcpy(dicedir,"mesh-pos/");
                else
                    strcpy(dicedir,"mesh-diced/");

                FILE *rgfp=fopen("poster.sh","w");
                fprintf(rgfp,"#!/bin/bash\necho 'Regen...\n'\nBASEPATH=%s/\nVRIP_HOME=$BASEPATH/vrip\nMESHAGG=$PWD/mesh-agg/\nexport VRIP_DIR=$VRIP_HOME/src/vrip/\nPATH=$PATH:$VRIP_HOME/bin\nRUNDIR=$PWD\nDICEDIR=$PWD/mesh-diced/\nmkdir -p $DICEDIR\n",basepath.c_str());
                /*fprintf(rgfp,"mkdir -p mesh-regen-tex\n"
		  "chmod 777 mesh-regen-tex\n"
		  "cd mesh-regen-tex\n"
		  "$BASEPATH/osgretex -pathfile ../mesh/campath.txt -config %s ../mesh/final.ive\n"
		  "cd $RUNDIR\ntime $BASEPATH/genTex --regen --dicedir %s --margins 10 10 1000000000000000 %s -f %s\n"
		  "$BASEPATH/lodgen --dicedir %s --mdir mesh-blend\n",
		  recon_config_file_name.c_str(), dicedir,
		  recon_config_file_name.c_str(),"mesh-regen-tex/",dicedir);*/
                fprintf(rgfp,"mkdir -p mesh-regen-tex\n"
                        "chmod 777 mesh-regen-tex\n"
                        "cd mesh-regen-tex\n"
                        "$BASEPATH/poster/poster --color 1.0 1.0 1.0 --inactive --tilesize %d %d --finalsize %d %d --enable-output-heightmap --enable-output-poster --depth --origin %.18g %.18g --poster poster.tif --heightmap height.tif ../mesh/lod/twostep.%s\n"

                        ,proj_tex_size,proj_tex_size,proj_tex_size*poster_tiles,proj_tex_size*poster_tiles,latOrigin,longOrigin,OSG_EXT);/*recon_config_file_name.c_str(), recon_config_file_name.c_str(),proj_tex_size,stereo_calib_file_name.c_str(),dicedir,
                  "mesh-regen-tex/",dicedir);*/

                fchmod(fileno(rgfp),0777);
                fclose (rgfp);
                if(poster)
                    sysres=system("./poster.sh");
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








