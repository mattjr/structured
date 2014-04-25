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

//
// threadedStereo.cpp
//

#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>

#include "Clean.h"
#include <sys/time.h>
#include <time.h>
#include <osg/io_utils>
#include <unistd.h> 
#include "cv.h"
#include "highgui.h"
#include "SeaBedIO.h"
#include "StereoEngine.h"
#include <osgUtil/DelaunayTriangulator>
#include "PLYWriterNodeVisitor.h"
#include <osg/ComputeBoundsVisitor>
#include "ShellCmd.h"
#include "stereo_cells.hpp"
#include <limits>
#include "VPBInterface.hpp"
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
#include "vertexData.h"
#include "MemUtils.h"
#include "SplitBounds.h"
#include <vips/vips.h>
static bool hw_image=false;
static bool blending=true;
static bool externalMode=false;
void formatBar(string name,osg::Timer_t startTick,unsigned int count,unsigned int totalCount);
string dst_proj4_coord_system;
using namespace std;
static bool useOrthoTex=true;
static bool runIpad=false;
static bool no_atlas=false;
bool silent=false;
bool reimage=true;
static double start_time = 0.0;
static bool no_rangeimg=true;
static bool apply_aug=false;
static double stop_time = numeric_limits<double>::max();
static double marginExpand=2;
static int vpblod_override=0;
static bool compositeMission=false;
static bool writeout_meshvar=true;
static double smallCCPer;
static unsigned int faceChunkTarget;
static unsigned int texRemapChunk;
static int expand_vol=PLYMERGE_STEP;
//
// Command-line arguments
//
static bool untex=true;
static bool reparamTex=true;
static double scaleRemapTex=1.0;
using namespace std;
static double sparseRatio=0.2;
static bool isSparse=false;
static FILE *conf_ply_file;
static string background_mb;
static string contents_file_name;
static string dir_name;
int targetScreenFaces;
int targetBaseLODRes;
static bool novpb=false;
static bool use_cached=true;
static bool further_clean=false;
static bool pause_after_each_frame = false;
static double image_scale = 1.0;
static int max_feature_count;
//static bool exportForStaticRender=true;
static bool useTextureArray=true;
static double longOrigin,latOrigin;
static int _tileRows;
static int jpegQuality=95;
static int _tileColumns;
static int targetFaces=40000;
static bool storeTexMesh=false;
static bool do_novelty=false;
static double dense_scale;
static bool no_tex=false;
static bool no_run_remap=false;
static bool no_tc=false;
static bool no_vttex=false;
static bool use_debug_shader=false;
static vector<string> mb_xyz_files;
static int desired_area=250.0;
static bool have_max_frame_count = false;
static unsigned int max_frame_count=INT_MAX;
static bool display_debug_images = true;
static bool var_tex=false;
static bool run_stereo=true;
static double max_alt_cutoff=20.0;
//static bool use_ncc = false;
static double vrip_ramp;
static int num_skip=0;
static bool clusterRun=false;
static int gpunum=0;
static string stereo_calib_file_name;
static bool no_simp=true;
static bool no_split=false;
static bool no_init=false;
static bool no_mosaic=false;
static ofstream file_name_list;
static string base_dir;
static bool no_depth=false;
static double feature_depth_guess = AUV_NO_Z_GUESS;
static int num_threads=1;
static FILE *fpp;
static bool useVirtTex=false;
static bool useReimage=true;
static float tex_margin=0.01;
static float bbox_margin=0.2;
static bool no_vrip=false;
static bool externalStereo=false;

static double vrip_res;
static string basepath;
static bool use_dense_stereo=false;
static double edgethresh;
enum {END_FILE,NO_ADD,ADD_IMG};
char cachedmeshdir[2048];
texcache_t cachedtexdir;
static bool no_hw_context=false;
static string deltaT_config_name;
static string deltaT_dir;

static string recon_config_file_name;
static int lodTexSize[3];
static       int sysres=0;
static bool useAtlas=false;
// Image normalisation
static bool cmvs=false;
static time_t start_timer, end_timer; 
static Config_File *recon_config_file;
osg::Vec2 reimageSize(1024,1024);
double mmperpixel=4;
int vrip_img_per_cell;
int tex_img_per_cell;
static double min_feat_dist;
static double epipolar_dist;
static double feat_quality_level;
const char *aggdir="tmp/mesh-agg";
static const char *serfile="localserver";
static double plymc_expand_by=0.0;
static int extern_mesh_smooth=0;
static double extern_err_stop;
static bool newmvs=false;
static bool no_runmvs_tri=false;
static double triangCleanEdge=5.0;
static bool no_runmvs=false;
static bool mvs_tex_only_color=false;
MyGraphicsContext *mgc=NULL;
#define diced_fopen(x,y) fopen((string(diced_dir)+string(x)).c_str(),y)

static int imageWidth,imageHeight;
//imageWidth=imageHeight=-1;

vector<Stereo_Pose_Data> load_tex_pose_file( const string &file_name )
{
    vector<Stereo_Pose_Data> poses;

    ifstream in_file( file_name.c_str() );
    if( !in_file )
    {
        cerr << "ERROR - Unable to load stereo pose file '" << file_name << "'"
             << endl;
        exit(1);
    }

    bool done = false;
    while( done == false )
    {
        Stereo_Pose_Data new_pose;

        if( in_file >> new_pose.id
                >> new_pose.file_name
                >> new_pose.bbox.xMin()
                >> new_pose.bbox.yMin()
                >> new_pose.bbox.zMin()
                >> new_pose.bbox.xMax()
                >> new_pose.bbox.yMax()
                >> new_pose.bbox.zMax()
                >> new_pose.mat(0,0)
                >> new_pose.mat(0,1)
                >> new_pose.mat(0,2)
                >> new_pose.mat(0,3)

                >> new_pose.mat(1,0)
                >> new_pose.mat(1,1)
                >> new_pose.mat(1,2)
                >> new_pose.mat(1,3)

                >> new_pose.mat(2,0)
                >> new_pose.mat(2,1)
                >> new_pose.mat(2,2)
                >> new_pose.mat(2,3)

                >> new_pose.mat(3,0)
                >> new_pose.mat(3,1)
                >> new_pose.mat(3,2)
                >> new_pose.mat(3,3)
                )
        {
            new_pose.valid = true;

            //         new_pose.mat = osg::Matrix::inverse(new_pose.mat);
            //cout <<new_pose.mat<<endl;
            poses.push_back( new_pose );
            if(have_max_frame_count && poses.size() >= max_frame_count)
                done=true;
        }
        else
        {
            done = true;
        }
    }

    return poses;
}
#include <dirent.h>
osgDB::DirectoryContents getDirectoryContents(const std::string& dirName,std::vector<string> validExts)
{
    osgDB::DirectoryContents contents;

    DIR *handle = opendir(dirName.c_str());
    if (handle)
    {
        dirent *rc;
        while((rc = readdir(handle))!=NULL)
        {
            bool foundExt=false;
            int len = strlen (rc->d_name);

            if (len >= 4) {
                for(int i=0; i < (int) validExts.size(); i++)
                    if (strcmp (validExts[i].c_str(), &(rc->d_name[len - 4])) == 0) {
                        foundExt=true;
                        break;
                    }

                if(foundExt)
                    contents.push_back(rc->d_name);
            }
        }
        closedir(handle);
    }

    return contents;
}

//
// Parse command line arguments into global variables
//
static bool parse_args( int argc, char *argv[ ] )
{
    osg::ArgumentParser argp(&argc,argv);
    argp.getApplicationUsage()->setApplicationName(argp.getApplicationName());
    argp.getApplicationUsage()->setDescription(argp.getApplicationName()+" example demonstrates the use of ImageStream for rendering movies as textures.");
    argp.getApplicationUsage()->setCommandLineUsage(argp.getApplicationName()+" <basedir>  [options]  ...\nwill look for mesh.cfg stereo.calib stereo_pose_est.data and dir img for images\n I suggest creating symlinks to those files allowing for varible configuration.\n");
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
    recon_config_file_name = "mesh.cfg";
    stereo_calib_file_name = "stereo.calib";
    contents_file_name = "stereo_pose_est.data";

    dir_name = "img/";
    if(argp.read("--mvs")){
            externalMode=true;
            newmvs=true;
            printf("Using MVS Mode for computation of structure\n");

    }
    if(argp.read("--onlymvscolor")){
        mvs_tex_only_color=true;
    }
    if(argp.read("--extern")){
        externalMode=true;
        printf("Using EXTERNAL Mode for outside computation of structure\n");


    }
    silent=argp.read("--silent");

    argp.read("--split-area",desired_area);
    argp.read("--stereo-calib",stereo_calib_file_name);
    argp.read("--poses",contents_file_name );

    if(argp.read("--noremap"))
        no_run_remap=true;

        if(argp.read("--flat"))
            reparamTex=false;

    apply_aug =argp.read("--apply_aug");
    argp.read("--gpu",gpunum);
    if(argp.read("--range"))
        no_rangeimg=false;
    runIpad=argp.read("--ipad");

    cmvs=argp.read("--mvs");
    useAtlas=argp.read("--atlas");
    novpb=argp.read("--novpb");
    storeTexMesh=argp.read("--storetex");
    argp.read("--jpeg-quality",jpegQuality);
    externalStereo= argp.read("--extstereo");
    if(argp.read("--cluster")){
        externalStereo=true;
        clusterRun=true;
    }

    if(argp.read("--debug-shader")){
        use_debug_shader=true;
        useAtlas=true;
        storeTexMesh=true;
    }
    if(argp.read("--noarray"))
        useTextureArray=false;

    if(argp.read("--var"))
        var_tex=true;
    hw_image=argp.read("--hwblend");
    deltaT_config_name=base_dir+string("/")+"localiser.cfg";
    double lat_orig,lon_orig;

    try {

        Config_File config_file( deltaT_config_name );
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
    recon_config_file->get_value( "TEX_CLAMP_MARGIN", tex_margin,0.01);
    recon_config_file->get_value( "BBOX_MARGIN_PERCENTAGE", bbox_margin,0.2);


    recon_config_file->get_value( "SD_SCALE", dense_scale,0.5);
    recon_config_file->get_value("TEX_IMG_PER_CELL",tex_img_per_cell,80);
    recon_config_file->get_value("VRIP_IMG_PER_CELL",vrip_img_per_cell,1000);
    recon_config_file->get_value("FEAT_MIN_DIST",min_feat_dist,3.0);
    recon_config_file->get_value("FEAT_QUALITY_LEVEL",feat_quality_level,0.0001);
    recon_config_file->get_value("CC_CLEAN_PERCENT",smallCCPer,0.1);
    recon_config_file->get_value("MAX_FEAT_COUNT",max_feature_count,5000);
    recon_config_file->get_value("VRIP_RAMP",vrip_ramp,500.0);
    recon_config_file->get_value("EDGE_THRESH",edgethresh,0.5);
    recon_config_file->get_value("STRUCT_FACE_CHUNK_SIZE",faceChunkTarget,200000);
    recon_config_file->get_value("REMAP_FACE_CHUNK_SIZE",texRemapChunk,50000);
    recon_config_file->get_value("MARGIN_EXPAND_REMAP_BBOX",marginExpand,10);
    recon_config_file->get_value("SPARSE_RATIO",sparseRatio,0.2);
    recon_config_file->get_value("VOLFILL_HOLES",expand_vol,PLYMERGE_STEP);

    recon_config_file->get_value("VRIP_RES",vrip_res,0.033);
    recon_config_file->get_value("IMAGE_SPLIT_COL",_tileColumns,-1);
    recon_config_file->get_value("IMAGE_SPLIT_ROW",_tileRows,-1);
    recon_config_file->get_value( "SRC_TEX_SIZE", lodTexSize[0],
                                  512);
    recon_config_file->get_value( "PROJ4_DEST_COORD",dst_proj4_coord_system,
                                  "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs");
    recon_config_file->get_value( "TARGET_SCREEN_FACES", targetScreenFaces,
                                  150000);
    recon_config_file->get_value( "TARGET_BASE_LOD_DIVRES", targetBaseLODRes,
                                  1024);
    recon_config_file->get_value( "SCALE_REMAP_TEX", scaleRemapTex,
                                  1.0);
    recon_config_file->get_value( "MM_PER_PIXEL", mmperpixel,
                                  4);
    recon_config_file->get_value( "JPEG_QUALITY", jpegQuality,
                                  95);
    recon_config_file->get_value( "SCF_MAX_EPIPOLAR_DIST", epipolar_dist,
                                  4.0);
    recon_config_file->get_value( "PLYMC_EXPAND_BY", plymc_expand_by,
                                  0.0);

    recon_config_file->get_value( "EXTERN_MESH_SMOOTH", extern_mesh_smooth,
                                  5);
    recon_config_file->get_value( "TRIANG_CLEAN_LG", triangCleanEdge,
                                  5.0);
     recon_config_file->get_value( "EXTERN_MESH_SIMP_ERR",extern_err_stop,5e-15);
    if(recon_config_file->get_value( "REIMAGE_RES",reimageSize.x(),-1)){
        reimageSize.y()=reimageSize.x();
    }

    //sprintf(cachedtexdir,"cache-tex-%d/",lodTexSize[1]);


    argp.read("--reimageres",reimageSize.x(),reimageSize.y());
    if(reimageSize.x() >0 ){
        for(int i=0; i< 2; i++){
            if(osg::Image::computeNearestPowerOfTwo(reimageSize[i]) != reimageSize[i]){
                fprintf(stderr,"Clamping reimage %d res %f to ",i,reimageSize[i]);
                reimageSize[i]=osg::Image::computeNearestPowerOfTwo(reimageSize[i]);
                fprintf(stderr,"%f\n ",reimageSize[i]);

            }
        }
    }

    string mbfile;


    if(  argp.read("--clean"))
        further_clean=true;
    if(argp.read( "--nountex" ))
        untex=false;

    argp.read("-r",image_scale);
    argp.read( "--edgethresh" ,edgethresh);
    argp.read("-m", max_feature_count );
    argp.read("-f",dir_name);
    argp.read("--target-screen-faces",targetScreenFaces);
    argp.read( "--vpblod" ,vpblod_override);
    argp.read( "--scale" ,scaleRemapTex);
    argp.read("--tex-margin",tex_margin);
    argp.read("--bbox-margin",bbox_margin);
    argp.read( "--sparseratio" ,sparseRatio);
    argp.read( "--expand" ,plymc_expand_by);

    useReimage=false;
    useVirtTex=true;
    if(argp.read("--novt")){
        useReimage=true;
        useVirtTex=false;
    }
    argp.read("--mm-per-pixel",mmperpixel);

    argp.read(  "--imagesplit",_tileRows,_tileColumns);



    argp.read("-z",feature_depth_guess );
    use_dense_stereo=argp.read("--ds" );

    argp.read("--dense-scale",dense_scale);
    no_depth=argp.read("--no-depth" );
    if(argp.read("--nostereo")){
        run_stereo=false;
    }
    argp.read("-s" ,num_skip);



    num_threads=OpenThreads::GetNumberOfProcessors();

    argp.read( "-t" ,num_threads);
    if(num_threads > 1)
        display_debug_images = false;

    recon_config_file->set_value( "SKF_SHOW_DEBUG_IMAGES" , display_debug_images );
    recon_config_file->set_value( "SCF_SHOW_DEBUG_IMAGES"  , display_debug_images );
    recon_config_file->set_value( "SD_SHOW_DEBUG_IMAGES"  , display_debug_images );
    argp.read( "--res",vrip_res );

    if(argp.read( "-n",max_frame_count))
        have_max_frame_count = true;
    //    max_frame_count=20;
    //  have_max_frame_count = true;



    if(!externalMode){
        if(!useOrthoTex){
            for(int i=0; (lodTexSize[0]/pow(2.0,i)) >= 8; i++){
                char tmppp[1024];
                int size=lodTexSize[0]/pow(2.0,i);
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

    }





    argp.read("--vrip-ramp",vrip_ramp );
    no_simp=argp.read( "--nosimp" );

    no_split=argp.read( "--nosplit" );
    no_init=argp.read("--noinit");
    no_mosaic=argp.read("--nomosaic");
    argp.read("--maxaltcutoff",max_alt_cutoff);

    use_cached=(!argp.read("--no_cached" ));
    do_novelty=argp.read("--novelty");
    if( argp.read("-d"))
        display_debug_images = false;

    pause_after_each_frame = argp.read("-p");


    argp.read("--start",start_time);
    argp.read("--stop",stop_time);
    if(argp.read("--novrip"))
        no_vrip=true;

    if(argp.read("--noatlas"))
        no_atlas=true;
    if(argp.read("--notex"))
        no_tex=true;
    if(argp.read("--notc"))
        no_tc=true;
    if(argp.read("--nomvstri"))
        no_runmvs_tri=true;
    if(argp.read("--norunmvs"))
        no_runmvs=true;
    if(argp.read("--novttex"))
        no_vttex=true;





    if(use_dense_stereo)
        sprintf(cachedmeshdir,"tmp/cache-mesh-dense/");
    else
        sprintf(cachedmeshdir,"tmp/cache-mesh-feat/");

    strcpy(cachedmeshdir,string(base_dir+string("/")+cachedmeshdir).c_str());
    for(int i=0; i < (int)cachedtexdir.size(); i++){
        cachedtexdir[i].first=base_dir+"/"+cachedtexdir[i].first;
    }



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

    cout << "   will look for localiser.cfg mesh.cfg stereo.calib stereo_pose_est.data and dir img for images"<< endl;
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



void save_bbox_frame (const osg::BoundingBox &bb, FILE * fptr){


    fprintf (fptr, "%g %g %g %g %g %g",
             bb.xMin(), bb.yMin(), bb.zMin(),
             bb.xMax(), bb.yMax(), bb.zMax());

}


void fill_osg_matrix(double *cam_pose,osg::Matrix &trans){





    double slam_x = cam_pose[POSE_INDEX_X];
    double slam_y = cam_pose[POSE_INDEX_Y];
    double slam_z = cam_pose[POSE_INDEX_Z];
    double slam_phi   =cam_pose[POSE_INDEX_PHI];
    double slam_theta =cam_pose[POSE_INDEX_THETA];
    double slam_psi   = cam_pose[POSE_INDEX_PSI];
    trans.makeRotate( slam_phi,osg::Vec3(1,0,0), slam_theta,osg::Vec3(0,1,0), slam_psi ,osg::Vec3(0,0,1));
    //cout << trans<<endl;

    trans*=osg::Matrix::translate(slam_x,slam_y,slam_z);
    //  trans=osgTranspose(trans);


    //   trans=osg::Matrix::inverse(trans);
    // cout << trans<<endl;


    /* trans(0, 3) = slam_x;
  trans(1, 3) = slam_y;
  trans(2, 3) = slam_z;
  trans(3, 3) = 1;
*/
}



static int get_auv_image_name( const string  &contents_dir_name,
                               Stereo_Pose pose,
                               Stereo_Pose_Data &name
                               )
{

    //name.cam_pose = new Vector(AUV_NUM_POSE_STATES);
    //name.m =gts_matrix_identity (NULL);

    //
    // Try to read timestamp and file names
    //

    //int index;

    name.id= pose.pose_id;
    name.time = pose.pose_time;
    name.pose[POSE_INDEX_X] = pose.pose_est[POSE_INDEX_X];
    name.pose[POSE_INDEX_Y] = pose.pose_est[POSE_INDEX_Y];
    name.pose[POSE_INDEX_Z] = pose.pose_est[POSE_INDEX_Z];
    name.pose[POSE_INDEX_PHI] = pose.pose_est[POSE_INDEX_PHI];
    name.pose[POSE_INDEX_THETA] = pose.pose_est[POSE_INDEX_THETA];
    name.pose[POSE_INDEX_PSI] = pose.pose_est[POSE_INDEX_PSI];
    name.left_name = pose.left_image_name;
    name.right_name = pose.right_image_name;
    name.file_name=name.left_name;
    name.alt= pose.altitude;
    name.radius= pose.image_footprint_radius;
    name.overlap = pose.likely_overlap;


    name.mesh_name = "surface-"+osgDB::getStrippedName(name.left_name)+".tc.ply";
    osg::Matrix trans;
    fill_osg_matrix(name.pose, name.mat);
    //fill_gts_matrix(name.pose,name.m);
    /*name.mat= osg::Matrix(name.m[0][0],name.m[1][0],name.m[2][0],name.m[3][0],
                          name.m[0][1],name.m[1][1],name.m[2][1],name.m[3][1],
                          name.m[0][2],name.m[1][2],name.m[2][2],name.m[3][2],
                          name.m[0][3],name.m[1][3],name.m[2][3],name.m[3][3]);*/
    /*name.mat= osg::Matrix( name.mat(0,0) ,name.mat(0,1),name.mat(0,2) ,name.mat(0,3)
                              , name.mat(1,0) ,name.mat(1,1),name.mat(1,2) ,name.mat(1,3)
                              , name.mat(2,0) ,name.mat(2,1),name.mat(2,2) ,name.mat(2,3)
                              , name.mat(3,0) ,name.mat(3,1),name.mat(3,2) ,name.mat(3,3));

*/

    //cout <<"New" << trans<<endl;
    //cout << "orig " << name.mat<<endl;
    
    return ADD_IMG;

}


bool getBBoxFromMesh(Stereo_Pose_Data &name){
    string fname=string(string(aggdir)+string("/")+name.mesh_name);
    ifstream mesh(fname.c_str());
    if(!mesh.good()){
        cerr <<"file not found "<<fname<<endl;
        return false;
    }
    bool got_bbox=false;
    bool got_faces=false;

    string line;
    getline(mesh,line);
    if(line.substr(0,3) != "ply"){
        fprintf(stderr,"Not ply file\n");
        return false;
    }
    while(mesh.good()){
        getline(mesh,line);
        if(line.substr(0,10) == "end_header"){
            if(!got_bbox)
                cerr << "valid file but no comment in header for bbox\n";
            if(!got_faces)
                cerr << "can't parse face number file not standard header\n";
            return (got_faces && got_bbox);
        }else if(line.substr(0,7) == "comment"){
            std::string bboxstr=line.substr(8,line.size());
            istringstream iss(bboxstr);
            osg::Vec3d minB,maxB;
            iss >> minB;
            iss >> maxB;
            name.bbox = osg::BoundingBox(minB,maxB);
            got_bbox=true;
        }else if(line.substr(0,12) == "element face"){
            std::string fstr=line.substr(13,line.size());
            istringstream iss(fstr);
            iss >> name.faces;
            got_faces=true;
        }
    }

    return false;
}

int main( int argc, char *argv[ ] )
{
    start_timer = time(NULL);

    FILE *rerunfp=fopen("rerun.sh","w");
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
#if VIPS_MINOR_VERSION > 24

    vips_init(argv[0]);
#endif
    string basepath= loc == string::npos ? "./" : path.substr(0,loc+1);
    basepath= osgDB::getRealPath (basepath);
    //cout << "Binary Path " <<basepath <<endl;

    // Run through the data
    //conf
    ifstream      contents_file;


    osgDB::makeDirectory(mosdir);

    chmod(mosdir,   0777);

    osgDB::makeDirectory(aggdir);

    chmod(aggdir,   0777);
    osgDB::makeDirectory(diced_img_dir);
    chmod(diced_img_dir,   0777);




    osgDB::makeDirectory(diced_dir);
    chmod(diced_dir,   0777);
    const char *uname="mesh";
    osgDB::makeDirectory(uname);

    chmod(uname,   0777);

    /*  osgDB::makeDirectory("mesh-pos");

    chmod("mesh-pos",   0777);*/
    osgDB::makeDirectory("debug");

    chmod("debug",   0777);


    osgDB::makeDirectory(cachedmeshdir);
    chmod(cachedmeshdir,   0777);
    for(int i=0; i < (int)cachedtexdir.size(); i++){
        osgDB::makeDirectory(cachedtexdir[i].first.c_str());
        chmod(cachedtexdir[i].first.c_str(),   0777);
    }

    vector<Stereo_Pose_Data> tasks;

    if(!externalMode){
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
    }else{
        if(!newmvs){
            tasks=load_tex_pose_file("camboxdata.txt");
            printf("Loaded %d metadata images for texturing\n",(int)tasks.size());
        }else{

        }
    }
    StereoCalib *calib=NULL;
    if(!newmvs){
     calib = new StereoCalib(stereo_calib_file_name);
     imageHeight=calib->camera_calibs[0].height;
     imageWidth=calib->camera_calibs[0].width;
    }


    FILE *fpp_ic=fopen("mesh/img_num.txt","w");
    if(!fpp_ic ){
        fprintf(stderr,"Cannot open mesh/img_num.txt\n");
        exit(-1);
    }
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


    //
    // Figure out the directory that contains the contents file
    //




    //
    if(num_threads > 1)
        printf("Threaded Stereo: %d threads initialized\n",num_threads);
    else
        printf("Threaded Stereo: single thread initialized\n");

    printf("Processing Meshes...\n");
    double max_alt=0;
    char stereo_conf_name[2048];
    char cwd[2048];
    char *dirres;
    dirres=getcwd(cwd,2048);
    if(dirres != cwd){
        fprintf(stderr,"Can't get current working dir\n");

    }

    //const char *simplogdir="/mnt/shared/log-simp";
    const char *pos_simp_log_dir="/mnt/shared/log-possimp";
    //const char *vriplogdir="/mnt/shared/log-vrip";

    float simp_mult=1.0;
    ShellCmd shellcm(basepath.c_str(),simp_mult,pos_simp_log_dir,cwd,aggdir,diced_dir,num_threads);
    shellcm.write_setup();
    std::vector<bool> isGrayImageList;

    string mvs_output_dir=string("/tmp/mvs/result.nvm.cmvs/00/");
    if(externalMode){
        if(newmvs){
            osgDB::makeDirectory("tmp/mvs/");

            if(!osgDB::fileExists(string(cwd)+"/tmp/mvs/image_list.txt")){
                std::vector<string> validExts;
                validExts.push_back(".ppm");
                validExts.push_back(".pgm");
                validExts.push_back(".jpg");
                validExts.push_back(".png");

               osgDB::DirectoryContents dirConts= getDirectoryContents(base_dir,validExts);
               ofstream fileList((string(cwd)+"/tmp/mvs/image_list.txt").c_str());
               string tmp_imgdir=string(cwd)+"/tmp/cvt_images/";
               osgDB::makeDirectory(tmp_imgdir);
               for(int i=0; i < (int)dirConts.size(); i++){
                   string stripped=osgDB::getNameLessExtension(dirConts[i]);
cout << "Stripped: " << stripped << endl;
                   if(mvs_tex_only_color)
                       isGrayImageList.push_back(stripped.substr(stripped.size()-3)=="M16");
                   else
                       isGrayImageList.push_back(false);
                   fileList <<  tmp_imgdir<<"/"<<stripped <<".jpg" << std::endl;
               }
               fileList.close();


               vector<std::string> precmd;
               vector<std::string> postcmd;

               string bundle_cmd="bundle.py";
               char tmp_bundle[8192];
               char optstring[1024];
               validExts.clear();
               validExts.push_back(".gcp");

               osgDB::DirectoryContents gcpfile= getDirectoryContents(base_dir,validExts);
               if(gcpfile.size()){
                   osgDB::copyFile(base_dir+"/"+gcpfile.front(),string(cwd)+"/tmp/mvs/image_list.txt.gcp");
                   sprintf(optstring,"sfm+gcp+sort");
               }else{
                   sprintf(optstring,"sfm+sort");
               }

               sprintf(tmp_bundle,"cd %s;mogrify -format jpg -path %s \\'*.png\\';cd %s/tmp/mvs/;%s/VisualSFM %s image_list.txt tmp.nvm; %s/VisualSFM sfm+loadnvm+cmvs tmp.nvm result.nvm",base_dir.c_str(),tmp_imgdir.c_str(),cwd,basepath.c_str(),optstring,basepath.c_str());
               precmd.push_back(tmp_bundle);
               sprintf(tmp_bundle,"cd %s/%s/;mv visualize visualize.color;mkdir visualize; cd visualize.color; mogrify -path ../visualize -type Grayscale -gamma 2.2 *.jpg;cd  ../visualize; mogrify -type TrueColor -colorspace RGB *.jpg",cwd,mvs_output_dir.c_str());
               postcmd.push_back(tmp_bundle);
               sprintf(tmp_bundle,"cd %s/%s/;PATH=$PATH:%s bash pmvs.sh",cwd,mvs_output_dir.c_str(),basepath.c_str());
               postcmd.push_back(tmp_bundle);

               shellcm.write_generic(bundle_cmd,"","Bundle",&(precmd),&(postcmd),1, "");
               if(!no_runmvs)
                   sysres=system((string(cwd)+"/"+bundle_cmd).c_str());


               //  precmd.clear();
              // string mvs_cmd="mvs.py";
               //shellcm.write_generic(mvs_cmd,"","MVS",&(precmd),NULL,1, "");

            }
            // Get info on multi-view stereo point clouds
            ifstream ifstr;
            char filename[4096];
            sprintf(filename,"%s/%s/ske.dat",cwd,mvs_output_dir.c_str());
            ifstr.open(filename);
            if(!ifstr){
                fprintf(stderr,"Failed to open %s/%s/ske.dat\n",cwd,mvs_output_dir.c_str());
                exit(-1);
            }
            ofstream idList((string(cwd)+"/tmp/mvs/id_image_list.txt").c_str());
            ifstream ifstr2;
            sprintf(filename,"%s/%s/list.txt",cwd,mvs_output_dir.c_str());
            ifstr2.open(filename);
            if(!ifstr2){
                fprintf(stderr,"Failed to open %s\n",filename);
                exit(-1);
            }
            int c_id=0;
            int valid_cnt=0;

            while(ifstr2.good()){
                string line;

                getline (ifstr2,line);
                if(line.size() < 1)
                    continue;
                sprintf(filename,"%s/%s/%s",cwd,mvs_output_dir.c_str(),line.c_str());
                if(c_id ==0){


                    cv::Mat img=cv::imread(filename);
                    if(!img.data){
                        fprintf(stderr,"%s cannot be found no valid images in list\n",filename);
                        exit(-1);
                    }
                    imageWidth=img.cols;
                    imageHeight=img.rows;
                }

                if(mvs_tex_only_color){
                    sprintf(filename,"%s/%s/visualize.color/%s",cwd,mvs_output_dir.c_str(),osgDB::getSimpleFileName(line.c_str()).c_str());
                    cv::Mat img=cv::imread(filename);
                    if(!img.data){
                        fprintf(stderr,"Cannot open %s for scanning grayscale in mvs\n",filename);
                        exit(-1);
                    }
                    //printf("%d %d\n",img.rows,img.cols);
                    bool isgray=true;
                    for(int k=0; k<img.rows; k++){
                        for(int l=0; l<img.cols; l++){
                            cv::Vec3b p = img.at<cv::Vec3b>(k,l);
                            //printf("%d %d %d\n",p[0],p[1],p[2]);
                            if(p[0] != p[1] || p[1] != p[2]){
                                isgray=false;
                                break;
                            }
                        }
                    }
                    if(isgray){
                        c_id++;
                        continue;
                    }

                }

                idList << c_id++ << " "<<osgDB::getSimpleFileName(line);
                for(int j=0; j <22; j++)
                    idList<< " "<< 0.0 ;
                idList  << std::endl;
                valid_cnt++;
            }
            idList.close();
            if(valid_cnt==0){
                fprintf(stderr,"No valid images Bailing!\n");
                exit(-1);
            }
            string header;
            int inum, cnum;
            ifstr >> header >> inum >> cnum;
            ifstr.close();
            string mvs_tri_cmd="mvs_tri.py";
            string mvs_cmd_fn=aggdir+string("/mvs_tri_cmds");
            FILE *fpp2=fopen(mvs_cmd_fn.c_str(),"w");
            if(!fpp2 ){
                fprintf(stderr,"Cannot open mvs_tri_cmds\n");
                exit(-1);
            }
            for(int i=0; i < cnum; i++){
                fprintf(fpp2,"%s/delaunay %s/cameras.ply %s/%s/models/option-%04d -o %s/%s/mvs-%04d.cgal;",basepath.c_str(),cwd,cwd,mvs_output_dir.c_str(),i,cwd,aggdir,i);
                fprintf(fpp2,"%s/triangclean %s/%s/mvs-%04d.cgal %s/%s/mvs-%04d.ply -lg %f -flip\n",basepath.c_str(),cwd,aggdir,i,cwd,aggdir,i,triangCleanEdge);

            }
            fclose(fpp2);
            vector<std::string> precmd;
            char tmp11[4096];
            sprintf(tmp11,"%s/triangulation/drawcameras.py -o %s/cameras.ply -i %s/%s/bundle.rd.out",basepath.c_str(),cwd,cwd,mvs_output_dir.c_str());
            precmd.push_back(tmp11);


            shellcm.write_generic(mvs_tri_cmd,mvs_cmd_fn,"MVSTRI",&(precmd),NULL,1, "");
            if(!no_runmvs_tri){
                sysres=system((string(cwd)+"/"+mvs_tri_cmd).c_str());
            }
            for(int i=0; i < cnum; i++){
                char tmpf[1024];
                sprintf(tmpf,"%s/%s/mvs-%04d.ply",cwd,aggdir,i);

                {
                    osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(tmpf);
                    if(!model.valid()){
                        fprintf(stderr,"Failed to load model %s\n",tmpf);
                        exit(-1);
                    }
                    osg::Drawable *drawable = model->asGeode()->getDrawable(0);
                    if(!drawable){
                        fprintf(stderr,"Failed to load model\n");
                        exit(-1);
                    }
                    osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
                    int faces=geom->getPrimitiveSet(0)->getNumPrimitives();
                    osg::ComputeBoundsVisitor cbbv(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
                    model->asGeode()->accept(cbbv);

                    Stereo_Pose_Data tmp_pose;
                    tmp_pose.mesh_name=tmpf;
                    tmp_pose.left_name=tmpf;
                    tmp_pose.file_name=tmpf;
                    tmp_pose.bbox = cbbv.getBoundingBox();
                    float margin=(tmp_pose.bbox.radius() * bbox_margin);
                    tmp_pose.bbox_margin.expandBy(tmp_pose.bbox._min-osg::Vec3(margin,margin,margin));
                    tmp_pose.bbox_margin.expandBy(tmp_pose.bbox._max+osg::Vec3(margin,margin,margin));
                    tmp_pose.faces=faces;
                    tmp_pose.valid=true;
                    tasks.push_back(tmp_pose);


                }

            }


        }else{
            char cptmp[8192];
            sprintf(cptmp,"%s/vcgapps/bin/mergeMesh %s/surface.ply -smooth %d -out %s/%s/full-total.ply",basepath.c_str(),cwd,extern_mesh_smooth,cwd,diced_dir);

            //printf("%s\n",cptmp);
            if(!no_init)
                sysres=system(cptmp);
            sprintf(cptmp,"%s/vcgapps/bin/tridecimator %s/%s/full-total.ply %s/%s/vis-total.ply 1 -q0.3 -P -e%g",basepath.c_str(),cwd,diced_dir,cwd,diced_dir,extern_err_stop);
            // printf("%s\n",cptmp);
            if(!no_init)
                sysres=system(cptmp);
        }
        FILE *firstpt=diced_fopen("firstpt.txt","w");
        if(!firstpt ){
            fprintf(stderr,"Cannot open mesh/firstpt.txt\n");
            exit(-1);
        }
        fprintf(firstpt,"%f %f\n",latOrigin,longOrigin);
        fclose(firstpt);
    }
    if(!externalMode){
        unsigned int stereo_pair_count =0;


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
        bool firstPose=false;

        vector<Stereo_Pose>::const_iterator cii;
        cii=pose_data.poses.begin();
        while( cii != pose_data.poses.end() && (!have_max_frame_count || stereo_pair_count < max_frame_count) ){



            Stereo_Pose_Data name;
            get_auv_image_name( dir_name, *cii, name) ;
            cii++;
            if(cii->pose_time < start_time || cii->altitude > max_alt_cutoff){
                continue;
            }
            if(!firstPose){
                fpp1=diced_fopen("firstpt.txt","w");
                if(!fpp1 ){
                    fprintf(stderr,"Cannot open mesh/firstpt.txt\n");
                    exit(-1);
                }
                fprintf(fpp1,"%f %f\n",cii->latitude,cii->longitude);
                fclose(fpp1);
                firstPose=true;
            }
            if(cii->pose_time >= stop_time)
                break;

            stereo_pair_count++;
            name.valid=false;
            tasks.push_back(name);
            if(name.alt > max_alt){
                max_alt=name.alt;
            }

        }



        if(tasks.size() <= 0){
            fprintf(stderr,"No tasks loaded check %s\n",contents_file_name.c_str());
            exit(-1);
        }
        start_time=tasks[0].time;
        stop_time=tasks[tasks.size()-1].time;






        sprintf(stereo_conf_name,"%s/meshes.txt",aggdir);


        conf_ply_file=fopen(stereo_conf_name,"w");
        if(!conf_ply_file){
            fprintf(stderr,"Can't open %s\n",stereo_conf_name);
            exit(-1);
        }

        chmod(stereo_conf_name,0666);

        int valid=0;
        int taskSplitSize=100;
        for(unsigned int i=0; i < tasks.size(); i+=taskSplitSize){
            // if(tasks[i].valid)
            if(1){
               /*const osg::Matrix &mat=tasks[i].mat;
                 fprintf(conf_ply_file,
                        "cd %s;%s/stereo_mesh_gen %s --batch %s %d %d --mat-1-8 %f %f %f %f %f %f %f %f --mat-8-16 %f %f %f %f %f %f %f %f --edgethresh %f -m %d -z %f\n"
                        ,cwd
                        ,basepath.c_str(),base_dir.c_str(),contents_file_name.c_str(),i,i+taskSplitSize,
                        mat(0,0),mat(1,0),mat(2,0),mat(3,0),
                        mat(0,1),mat(1,1),mat(2,1),mat(3,1),
                        mat(0,2),mat(1,2),mat(2,2),mat(3,2),mat(0,3),mat(1,3),mat(2,3),mat(3,3),edgethresh,
                        max_feature_count,tasks[i].alt);*/
                fprintf(conf_ply_file,
                        "cd %s;%s/stereo_mesh_gen %s --batch %s %d %d --edgethresh %f -m %d \n"
                        ,cwd
                        ,basepath.c_str(),base_dir.c_str(),contents_file_name.c_str(),i,i+taskSplitSize,
                        edgethresh,
                        max_feature_count);
                valid++;

            }else{
                fprintf(stderr,"Not valid %s\n", osgDB::getStrippedName(tasks[i].left_name).c_str()     );
            }
        }

        fclose(conf_ply_file);



        if(valid <= 0){
            fprintf(stderr,"No valid meshes bailing\n");
            exit(-1);
        }

    }

    /*FILE *vrip_seg_fp;
    char vrip_seg_fname[2048];
    FILE *bboxfp;
    string vripcmd_fn=aggdir+string("/vripcmds");
    FILE *vripcmds_fp=fopen(vripcmd_fn.c_str(),"w");*/
    FILE *diced_fp=diced_fopen("diced.txt","w");
    FILE *diced_lod_fp=diced_fopen("dicedld.txt","w");

    /*if(!vripcmds_fp){
        printf("Can't open vripcmds\n");
        exit(-1);
    }
*/

    if(!diced_fp){
        printf("Can't open diced/diced.txt");
        exit(-1);
    }
    if(!diced_lod_fp){
        printf("Can't open diced/dicedld.txt");
        exit(-1);
    }


    if(!externalMode){

double totalValidArea=0;
        if(externalStereo){
        string stereocmd="stereo.py";

        shellcm.write_generic(stereocmd,stereo_conf_name,"Stereo");
        if(run_stereo)
            sysres=system("./stereo.py");

        for(unsigned int i=0; i < tasks.size(); i++){
            Stereo_Pose_Data &name=tasks[i];
            if(getBBoxFromMesh(name)){
                name.valid=true;
            }else{
                fprintf(stderr,"Not valid %s\n", osgDB::getStrippedName(name.mesh_name).c_str()     );
                name.valid=false;

            }
        }

        }else{

        osg::Timer_t startTick= osg::Timer::instance()->tick();
        int totalTodoCount=tasks.size();
        int progCount=0;
        OpenThreads::Mutex mutex;
        int delayMessageFrame=0;
        double max_triangulation_len =  max_alt > 0.0 ? max_alt*3 : edgethresh * 20;
#pragma omp parallel num_threads(num_threads)
        if(run_stereo){
            StereoEngine engine(*calib,*recon_config_file,edgethresh,max_triangulation_len,max_feature_count, lodTexSize[0],mutex,use_dense_stereo,pause_after_each_frame);
            cvSetNumThreads(1);
#pragma omp for
            for(int i=0; i < (int)tasks.size(); i++){
                MatchStats stats;
                StereoStatusFlag statusFlag=engine.processPair(base_dir,tasks[i].left_name,
                                                               tasks[i].right_name,tasks[i].mat,tasks[i].bbox,stats,feature_depth_guess,hw_image,use_cached);
                if(statusFlag == FAIL_FEAT_THRESH || statusFlag == FAIL_TRI_EDGE_THRESH){
                    fprintf(stderr,"Rerunning: %s\n",statusFlag == FAIL_FEAT_THRESH ? "fail feat thresh": "fail tri edge thresh");
                    statusFlag=engine.processPair(base_dir,tasks[i].left_name,
                                       tasks[i].right_name,tasks[i].mat,tasks[i].bbox,stats,feature_depth_guess,hw_image,false,true);

                }
#pragma omp critical
                {
                    if(statusFlag == FALLBACK_KEYPOINT){
                        printf("Fail fallback: %s\n",tasks[i].left_name.c_str());
                    }
                    if(statusFlag == FAIL_OTHER)
                        tasks[i].valid=false;
                    else
                        tasks[i].valid=true;
                    tasks[i].faces=stats.total_faces;
                    progCount++;
                    if(statusFlag == FAIL_FEAT_THRESH || statusFlag == FAIL_TRI_EDGE_THRESH){
                        delayMessageFrame=10;
                        printf("\r%s[ TrackRej %03d EpiRej %03d TriRej %03d OK %03d: %s ]%s", KBGRDRED,stats.total_tracking_fail,
                               stats.total_epi_fail,stats.total_tri_fail,stats.total_accepted_feat,tasks[i].left_name.c_str(),KNRM); fflush(stdout);
                    }else{
                        if(delayMessageFrame > 0){
                            delayMessageFrame--;
                        }else{
                            formatBar("Stereo",startTick,progCount,totalTodoCount);
                        }
                    }
                }
            }
        }
        if(!silent)
            formatBar("Stereo",startTick,totalTodoCount,totalTodoCount);


        if(!run_stereo){
            char tp[2048];
            for(unsigned int i=0; i < tasks.size(); i++){
                sprintf(tp,"%s/surface-%s.tc.ply",aggdir,osgDB::getSimpleFileName(osgDB::getNameLessExtension(tasks[i].left_name)).c_str());
                if(getBBoxFromMesh(tasks[i]))
                    tasks[i].valid=true;
                else
                    tasks[i].valid=false;
                if(!silent)
                    formatBar("Stereo-FCK",startTick,progCount,totalTodoCount);
                progCount++;
            }
            if(!silent)
                formatBar("Stereo-FCK",startTick,totalTodoCount,totalTodoCount);

        }
        printf("\n");
}

        FILE *totalfp=diced_fopen("totalbbox.txt","w");


        int ct=0;
       // printf("Bbox margin is %f\n",bbox_margin);
        for(vector<Stereo_Pose_Data>::iterator itr=tasks.begin(); itr != tasks.end(); itr++){
            Stereo_Pose_Data &name=(*itr);
            if(name.valid){

                double area= ((name.bbox.xMax()-name.bbox.xMin())*
                              (name.bbox.yMax()-name.bbox.yMin()));
                if(isfinite(area)){
                    totalValidArea+=area;
                    float margin=(name.bbox.radius() * bbox_margin);
                    name.bbox_margin.expandBy(name.bbox._min-osg::Vec3(margin,margin,margin));
                    name.bbox_margin.expandBy(name.bbox._max+osg::Vec3(margin,margin,margin));
                }
            }
            fprintf(fpp_ic,"%d %f %s\n",
                    name.id,name.time,osgDB::getNameLessExtension(name.left_name).c_str());
            fprintf(fpp,"%d %f %s %s ",
                    ct++,name.time,name.left_name.c_str(),name.right_name.c_str());
            fprintf(totalfp,"%d %s ",
                    ct,name.left_name.c_str());
            save_bbox_frame(name.bbox_margin,fpp);
            save_bbox_frame(name.bbox_margin,totalfp);


            if(ct==1){
                if(name.left_name.size() != osgDB::getSimpleFileName(name.left_name).size()){
                    compositeMission=true;
                }
            }
            for(int i=0; i< 4; i++){
                for(int j=0; j < 4; j++){
                    osg::Matrix texmat=osgTranspose(name.mat);
                    texmat=osg::Matrix::inverse(texmat);

                    //   trans=osg::Matrix::inverse(trans);
                    fprintf(fpp," %f",name.mat(i,j));
                }
            }
            fprintf(fpp,"\n");


            osg::Matrix texmat=osgTranspose(name.mat);
            texmat=osg::Matrix::inverse(texmat);
            for(int k=0; k < 4; k++)
                for(int n=0; n < 4; n++)
                    fprintf(totalfp," %lf",texmat(k,n));
            fprintf(totalfp,"\n");
            //      if(use_poisson_recon){
            //	dump_pts(pos_fp,string("mesh-agg/"+name.mesh_name).c_str(),clean_pos_pts);
            //    }
        }
        fclose(totalfp);
        if(fpp_ic)
            fclose(fpp_ic);
    }
    Bounds bounds( tasks );

    int numberFacesAll=0;
#if 0

    std::vector<Cell_Data<Stereo_Pose_Data> > vrip_cells;
    vrip_cells=calc_cells<Stereo_Pose_Data> (tasks,vrip_img_per_cell);
    //for(int i=0; i< (int)vrip_cells.size(); i++)
    //   vrip_cells[i].bounds.bbox._min<< " "<<   vrip_cells[i].bounds.bbox._max<<endl;
    if(!externalMode){
        double fillRatio=(totalValidArea/bounds.area());
        printf("Valid Area %.2f Total Area %.2f\n",totalValidArea,bounds.area());
        printf("Fill Percentage %d%% (Doesn't account for overlap) Cut Off for Sparse %d%%\n",(int)floor(fillRatio*100.0),(int)floor(sparseRatio*100.0));
        if(fillRatio < sparseRatio){
            isSparse=true;
            printf("Using sparse mode\n");
        }
        printf("Split into %d cells for VRIP\n",(int)vrip_cells.size());
        char conf_name[2048];

        for(int i=0; i <(int)vrip_cells.size(); i++){


            if(vrip_cells[i].poses.size() == 0)
                continue;

            sprintf(vrip_seg_fname,"%s/vripseg-%08d.txt",aggdir,i);
            sprintf(conf_name,"%s/bbox-clipped-diced-%08d.ply.txt",diced_dir,i);

            vrip_seg_fp=fopen(vrip_seg_fname,"w");
            bboxfp = fopen(conf_name,"w");
            if(!vrip_seg_fp || !bboxfp){
                printf("Unable to open %s\n",vrip_seg_fname);
            }

            char redirstr[2048];

            sprintf(redirstr," ");

            fprintf(diced_fp,"clipped-diced-%08d.ply\n",i);
            fprintf(diced_lod_fp,"clipped-diced-%08d-lod0.ply\n",i);
            fprintf(vripcmds_fp,"set BASEDIR=\"%s\"; set OUTDIR=\"%s/\";set VRIP_HOME=\"$BASEDIR/vrip\";setenv VRIP_DIR \"$VRIP_HOME/src/vrip/\";set path = ($path $VRIP_HOME/bin);cd %s/$OUTDIR;",basepath.c_str(),aggdir,cwd);
            fprintf(vripcmds_fp,"$BASEDIR/vrip/bin/vripnew auto-%08d.vri %s %s %f -rampscale %f;$BASEDIR/vrip/bin/vripsurf auto-%08d.vri seg-%08d.ply %s ;",i,osgDB::getSimpleFileName(vrip_seg_fname).c_str(),osgDB::getSimpleFileName(vrip_seg_fname).c_str(),vrip_res,vrip_ramp,i,i,redirstr);

            fprintf(vripcmds_fp,"$BASEDIR/treeBBClip seg-%08d.ply --bbox %f %f %f %f %f %f -gap --outfile %s/%s/tmp-clipped-diced-%08d.ply;",
                    i,
                    vrip_cells[i].bounds.bbox.xMin(),
                    vrip_cells[i].bounds.bbox.yMin(),
                    -FLT_MAX,
                    vrip_cells[i].bounds.bbox.xMax(),
                    vrip_cells[i].bounds.bbox.yMax(),
                    FLT_MAX,
                    cwd,
                    diced_dir,
                    i);

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
                osg::Matrix texmat=osgTranspose(pose->mat);
                texmat=osg::Matrix::inverse(texmat);
                for(int k=0; k < 4; k++)
                    for(int n=0; n < 4; n++)
                        fprintf(bboxfp," %lf",texmat(k,n));
                fprintf(bboxfp,"\n");
            }


            fclose(vrip_seg_fp);
            fclose(bboxfp);
        }
        fclose(vripcmds_fp);
        fclose(diced_fp);
        fclose(diced_lod_fp);
        if(cmvs){
            FILE *cfp= fopen("runmvs.sh","w");
            fprintf(cfp,"#!/bin/bash\n");
            fprintf(cfp,"bash %s/../auv2mv/runauv2mv.sh %s %d %d %f\n",basepath.c_str(), base_dir.c_str(),max_frame_count,num_threads,0.15);

            fprintf(cfp,"%s/vcgapps/bin/triangulate  pmvs/models/option-*.ply\n",basepath.c_str());

            fprintf(cfp,"cp out.ply %s/vis-total.ply\n",diced_dir);
            fchmod(fileno(cfp),0777);
            fclose(cfp);
            sysres=system("./runmvs.sh");
        }




        if(fpp)
            fclose(fpp);
        if(fpp2)
            fclose(fpp2);

        {









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
            // if(!no_vrip && !cmvs)
            //   sysres=system("python runvrip.py");
            char tmpfn[8192];
            //std::vector<Cell_Data<Stereo_Pose_Data> > stereo_poses_tmp;
            /*  for(int i=0; i <(int)vrip_cells.size(); i++){
                if(vrip_cells[i].poses.size() == 0)
                    continue;
                sprintf(tmpfn,"%s/tmp-clipped-diced-%08d.ply",diced_dir,i);
                osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(tmpfn);
                osg::Drawable *drawable = NULL;
                if(model.valid())
                    drawable = model->asGeode()->getDrawable(0);
                if(!drawable){
                    fprintf(stderr,"Failed to load model %s VRIP failed on one chunk\nThere may be holes in it!!!!!!\nAlso check if this is part of ascent or decent where lots of Z variation occurs.\n",tmpfn);
                    //sleep(1);
                    continue;
                    exit(-1);
                }
                osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
                numberFacesAll+=geom->getPrimitiveSet(0)->getNumPrimitives();
                //stereo_poses_tmp.push_back(vrip_cells[i]);


            }*/
        }
    }
#endif
    //vrip_cells.swap(stereo_poses_tmp);
    osg::BoundingBox totalbb;
    osg::BoundingBox totalbb_unrot;

    osg::BoundingSphere bs;
    osg::Matrix rotM;
    // float rx=0,ry=180.0,rz=-90;
    float rx=0,ry=0.0,rz=0;

    rotM =osg::Matrix::rotate(
                osg::DegreesToRadians( rx ), osg::Vec3( 1, 0, 0 ),
                osg::DegreesToRadians( ry ), osg::Vec3( 0, 1, 0 ),
                osg::DegreesToRadians( rz ), osg::Vec3( 0, 0, 1 ) );
    if(externalMode && !newmvs){
        cout << "Loading full mesh...\n";
        char tmp[1024];
        sprintf(tmp,".%d,%d,%d.rot",(int)rx,(int)ry,(int)rz);
        string rot=string(tmp);
        osg::ref_ptr<osg::Node> model = osgDB::readNodeFile((string(diced_dir)+"/vis-total.ply"+rot).c_str());
        osg::Drawable *drawable = model->asGroup()->getChild(0)->asGeode()->getDrawable(0);
        if(!drawable){
            fprintf(stderr,"Failed to load model\n");
            exit(-1);
        }
        osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
        numberFacesAll=geom->getPrimitiveSet(0)->getNumPrimitives();

        if(!model.valid() || !model->getBound().radius()){
            std::cerr << " Cant open vis-total.ply\n";
            exit(-1);
        }
        bs=model->getBound();
        osgUtil::Optimizer::FlattenStaticTransformsVisitor fstv(NULL);
        model->accept(fstv);
        fstv.removeTransforms(model);
        cout << "Writing out rotated...\n";
        osgDB::writeNodeFile(*model,(string(diced_dir)+"/totalrot.ive").c_str());
        osg::ComputeBoundsVisitor cbbv(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
        model->accept(cbbv);
        totalbb = cbbv.getBoundingBox();
        totalbb_unrot.expandBy(totalbb._min*osg::Matrix::inverse(rotM));
        totalbb_unrot.expandBy(totalbb._max*osg::Matrix::inverse(rotM));

    }else{

        totalbb_unrot.expandBy(bounds.bbox._min);
        totalbb_unrot.expandBy(bounds.bbox._max);
        totalbb=totalbb_unrot;//totalbb.expandBy(osg::Vec3(totalbb_unrot._min[0],totalbb_unrot._min[1],-totalbb_unrot._min[2]));
        //totalbb.expandBy(osg::Vec3(totalbb_unrot._max[0],totalbb_unrot._max[1],-totalbb_unrot._max[2]));
    }


    // cout << totalbb_unrot._min<< " " << totalbb_unrot._max<<endl;
    //cout << totalbb._min<< " " << totalbb._max<<endl;
    /* osg::BoundingBox tmp=totalbb;
        tmp._min[2]= tmp._min[1];
        tmp._max[2]= tmp._max[1];
*/

    bs.expandBy(totalbb);
    double rangeX=totalbb.xMax()-totalbb.xMin();
    double rangeY=totalbb.yMax()-totalbb.yMin();
    double largerAxis = std::max(rangeX , rangeY);

    CellDataT<Stereo_Pose_Data>::type vol;
    int minSplits=-1;
    int faceGuess = (externalMode && !newmvs) ? numberFacesAll : ((rangeX * rangeY) / (vrip_res*vrip_res));
    //printf("%d %f\n",faceGuess,largerAxis);
    double targetSide=((double)faceChunkTarget/faceGuess)*largerAxis;
    //printf("Target Side %f\n",targetSide);
  //  double cubicRootSide =pow(targetVolume ,1./3.);
    //printf("side %f\n", largerAxis);
    //cout << bounds.bbox._min << " "<< bounds.bbox._max<<endl;
    //double targetVolume=10.0;
  //  split_boundsOctree<Stereo_Pose_Data>(bounds,tasks , targetSide,minSplits,vol);
    split_boundsOctree<Stereo_Pose_Data>(bounds,tasks , faceChunkTarget,minSplits,vol);
    std::vector<osg::BoundingBox> kd_bboxes;
    double avgEdgeLen=0.0;
    if(!externalMode || newmvs){
        {
            WriteBoundTP wbtp(vrip_res,string(aggdir)+"/plymccmd",basepath,cwd,tasks,plymc_expand_by,smallCCPer);
            WriteBoundVRIP wbvrip(vrip_res,string(aggdir)+"/vripcmd",basepath,cwd,tasks,vrip_ramp,smallCCPer,expand_vol);

           // int splits[3]={0,0,0};
            foreach_vol(cur,vol){
                //  cout <<cur->bounds.bbox._min<<" "<<cur->bounds.bbox._max<<endl;
                //    cout <<"Poses " <<cur->poses.size()<<endl;
                wbvrip.write_cmd(*cur);
                wbtp.write_cmd(*cur);

              /*  splits[0]=cur->splits[0];
                splits[1]=cur->splits[1];
                splits[2]=cur->splits[2];
*/
            }
           // cout << splits[0] << " " << splits[1] << " " <<splits[2] <<endl;
            string plymccmd="plymc.py";
            string vripccmd="runvrip.py";

            std::vector<string> postcmd;
            char tmpcmd[1024];
            tmpcmd[0]='\0';
            /* const char *opts=" ";//-w0  -L0 -q50 -R0";
       // sprintf(tmpcmd,"cd %s;%s/vcgapps/bin/plymc_outofcore -i1 -M -V%f %s -S %d %d %d -o%s/vol %s",
        sprintf(tmpcmd,"cd %s;%s/vcgapps/bin/plymc -M -V%f %s -S %d %d %d -o%s/vol %s",
                cwd,basepath.c_str(),
                vrip_res,
                opts,

                           splits[0],
                          splits[1],
                           splits[2],
                           aggdir,
                           wbtp.bboxfn.c_str());
        // postcmd.push_back(string(tmpcmd));
         sprintf(tmpcmd,"os.system(setupts.basepath +'/runtp_dist.py %s %s %s')\n",
                 (string(aggdir)+"/plymccmd2").c_str(),serfile,"PlyMC2");*/
            shellcm.write_generic(plymccmd,wbtp.getCmdFileName(),"PlyMC",NULL,&postcmd,0,string(tmpcmd));
            shellcm.write_generic(vripccmd,wbvrip.getCmdFileName(),"Vrip",NULL,&postcmd,0,wbvrip.getPostCmds(vol));
            wbvrip.close();
            wbtp.close();
        }
        if(!no_vrip){
            sysres=system("python runvrip.py");
            if(sysres <0){
                fprintf(stderr,"Failed to run runvrip.py\n");
                exit(-1);
            }
        }
        char tmpfn[8096];
        {
            osg::ref_ptr<osg::Node> model;
            ply::VertexData vertexData;

            foreach_vol(cur,vol){
                sprintf(tmpfn,"%s/clean_%04d%04d%04d.ply",aggdir,cur->volIdx[0],cur->volIdx[1],cur->volIdx[2]);
                // osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(tmpfn);
                if(osgDB::fileExists(tmpfn+string(".empty")))
                    continue;
                model= vertexData.readPlyFile(tmpfn);

                osg::Drawable *drawable = NULL;
                if(model.valid())
                    drawable = model->asGeode()->getDrawable(0);
                if(!drawable){
                    fprintf(stderr,"Failed to load model %s VRIP/PLYMC failed on one chunk should have %d poses\nThere may be holes in it!!!!!!\nAlso check if this is part of ascent or decent where lots of Z variation occurs.\n",
                            tmpfn,(int)cur->poses.size());
                    //sleep(1);
                    continue;
                    exit(-1);
                }
                //osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
                // numberFacesAll+=geom->getPrimitiveSet(0)->getNumPrimitives();
                //stereo_poses_tmp.push_back(vrip_cells[i]);


            }
            if(!getFaceDivision(model,avgEdgeLen,numberFacesAll,texRemapChunk,kd_bboxes)){
                fprintf(stderr,"Can't load divisions\n");
                exit(-1);
            }

        }

    }


    //printf("Face Guess %d Real %d\n",faceGuess,numberFacesAll);

#if 0
    // float rx=0,ry=180.0,rz=-90;
    // int numberFacesAll=0;
    {
        char tmp[1024];
        sprintf(tmp,".%d,%d,%d.rot",(int)rx,(int)ry,(int)rz);
        string rot=string(tmp);

        rotM =osg::Matrix::rotate(
                    osg::DegreesToRadians( rx ), osg::Vec3( 1, 0, 0 ),
                    osg::DegreesToRadians( ry ), osg::Vec3( 0, 1, 0 ),
                    osg::DegreesToRadians( rz ), osg::Vec3( 0, 0, 1 ) );
        cout << "Loading full mesh...\n";
        osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("mesh-diced/vis-total.ply"+rot);
        osg::Drawable *drawable = model->asGroup()->getChild(0)->asGeode()->getDrawable(0);
        if(!drawable){
            fprintf(stderr,"Failed to load model\n");
            exit(-1);
        }
        osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
        numberFacesAll=geom->getPrimitiveSet(0)->getNumPrimitives();

        if(!model.valid() || !model->getBound().radius()){
            std::cerr << " Cant open vis-total.ply\n";
            exit(-1);
        }
        bs=model->getBound();
        osgUtil::Optimizer::FlattenStaticTransformsVisitor fstv(NULL);
        model->accept(fstv);
        fstv.removeTransforms(model);
        cout << "Writing out rotated...\n";
        osgDB::writeNodeFile(*model,"mesh-diced/totalrot.ive");
        osg::ComputeBoundsVisitor cbbv(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
        model->accept(cbbv);
        totalbb = cbbv.getBoundingBox();
        totalbb_unrot.expandBy(totalbb._min*osg::Matrix::inverse(rotM));
        totalbb_unrot.expandBy(totalbb._max*osg::Matrix::inverse(rotM));

    }
#endif
    enum {REMAP,DEPTH,FLAT,REMAP_FLAT_SIZE,NUM_TEX_FILES};
    int selectedTexMode=REMAP;



    int minTexSize=8192;

    double largerAxisPixels=largerAxis * ((1.0/mmperpixel) *1000.0);
    int adjustedSize=osg::Image::computeNearestPowerOfTwo(largerAxisPixels,1.0);
    adjustedSize=std::max(adjustedSize,minTexSize);
    printf("mm per pixel: %.1f Larger Axis %.2f m POT Image Size %dx%d\n",mmperpixel,
           largerAxis,adjustedSize,adjustedSize);
    if(_tileRows <0 || _tileColumns<0){
        if(selectedTexMode == REMAP){

            int validCount=0;
            for(int i=0; i< (int)tasks.size(); i++)
                if(tasks[i].valid)
                    validCount++;
            //printf("Auto computing tile and column splits %d valid images...\n",validCount);
            int numCells=(int)max(ceil(sqrt(ceil(validCount / tex_img_per_cell))),1.0);
            //printf("Selected %d\n",numCells);

            while((largerAxis*largerAxis)/(float)(numCells*numCells)>(targetSide*targetSide)){
                numCells++;
            }
            //printf("Adjusted to %d based on targetVolume %0.3f final volume %0.3f total splits %d\n",numCells,targetSide*targetSide,(largerAxis*largerAxis)/(float)(numCells*numCells),numCells*numCells);

            _tileRows=numCells;
            _tileColumns=numCells;
        }else{
            int numCells=(int)max(ceil(adjustedSize/osg::Image::computeNearestPowerOfTwo(adjustedSize/sqrt(num_threads))),1.0);
            _tileRows=numCells;
            _tileColumns=numCells;
        }

    }


    int split= std::max(std::max(_tileRows,_tileColumns),1);
    if(reimageSize.x() < 0.0)
        reimageSize = reparamTex ? osg::Vec2(osg::Image::computeNearestPowerOfTwo(adjustedSize / split,1.0),osg::Image::computeNearestPowerOfTwo(adjustedSize / split,1.0)) : osg::Vec2(adjustedSize / (double)split,adjustedSize / (double)split);
   // cout << reimageSize <<endl << adjustedSize<<endl << split<<endl;
    if(reimageSize.x() > 8192){
        fprintf(stderr, "Can't have an imaging size %f its larger then 8192 dropping\n",reimageSize.x());
        double mult=reimageSize.x()/8192;
        fprintf(stderr, "Adjusting rows and cols by %f\n",mult);

        int larger=std::max(ceil(_tileColumns*mult),ceil(_tileRows*mult));
        _tileRows=larger;
        _tileColumns=larger;
        reimageSize=osg::Vec2(8192,8192);
    }
    printf("Texture Cells %dx%d\n",_tileRows,_tileColumns);
    printf("Tile Size Pixels %dx%d\n",(int)reimageSize.x(),(int)reimageSize.y());
    int vpblod=0;
    // int tmpImg=adjustedSize;
    if(!useVirtTex){
        //vpblod++;
        int faceTmp=numberFacesAll;
        do{
            faceTmp /= pow(4.0,vpblod++);
            //tmpImg /= pow(2.0,vpblod);
            // printf("%d %d\n",faceTmp,adjustedSize/(int)pow(2.0,vpblod));
        }while(faceTmp > targetFaces || (adjustedSize/(int)pow(2.0,vpblod))>targetBaseLODRes );
        std::cout << "Target LOD height is : " << vpblod <<std::endl;
    }else{
        int faceTmp=numberFacesAll;
        do{    //cout << "faces: "<<faceTmp<<" "<<vpblod<<endl;
            faceTmp /= pow(4.0,++vpblod);
        }while(faceTmp > targetFaces );

        std::cout << "Target LOD height is : " << vpblod <<std::endl;
    }
    osg::Vec3d eye(totalbb.center()+osg::Vec3(0,0,3.5*totalbb.radius()));
    double xrange=totalbb.xMax()-totalbb.xMin();
    double yrange=totalbb.yMax()-totalbb.yMin();
    double largerSide=std::max(xrange,yrange);
    /*   osg::Matrixd matrix;
    matrix.makeTranslate( eye );
    osg::Matrixd view=osg::Matrix::inverse(matrix);
    osg::Matrixd proj= osg::Matrixd::ortho2D(-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0));*/
    /* proj= osg::Matrixd::ortho2D(-bs.radius(),bs.radius(),-bs.radius(),bs.radius());
 printf("%f %f %f %f AAAA\n",-bs.radius(),bs.radius(),-bs.radius(),bs.radius());
 printf("%f %f %f %f AAAA\n",-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0));
*/
    /*
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

    */
    double chunkSize=(totalbb.xMax()-totalbb.xMin())/(double)_tileRows;

    double widthEnd=totalbb.xMin()+((_tileRows)*chunkSize);
    double widthStart=totalbb.xMin()+((0)*chunkSize);
    double heightEnd=totalbb.yMin()+((_tileColumns)*chunkSize);
    double heightStart=totalbb.yMin()+((0)*chunkSize);
    osg::Matrix viewproj;
    viewproj(0,0)=0;
    viewproj(0,1)=2/largerSide;
    viewproj(0,2)=0;
    viewproj(0,3)= (-(heightEnd+heightStart)/(heightEnd-heightStart));


    viewproj(1,0)=2/largerSide;
    viewproj(1,1)=0;
    viewproj(1,2)=0;
    viewproj(1,3)= (-(widthEnd+widthStart)/(widthEnd-widthStart));


    viewproj(2,0)=0;
    viewproj(2,1)=0;
    viewproj(2,2)=2;
    viewproj(2,3)=-1;

    viewproj(3,0)=0;
    viewproj(3,1)=0;
    viewproj(3,2)=0;
    viewproj(3,3)= 1;
    //  cout <<viewproj<<endl;

    std::stringstream os2;
    os2<< "viewproj.mat";

    std::fstream _file(os2.str().c_str(),std::ios::binary|std::ios::out);
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            //Write Transpose
            _file.write(reinterpret_cast<char*>(&(viewproj(j ,i))),sizeof(double));
    _file.close();



    std::fstream _file2("rot.mat",std::ios::binary|std::ios::out);
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            _file2.write(reinterpret_cast<char*>(&(rotM(i,j))),sizeof(double));
    _file2.close();

    if(externalMode && !newmvs){
        osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(string(diced_dir)+"/vis-total.ply");
        if(!getFaceDivision(model,avgEdgeLen,numberFacesAll,texRemapChunk,kd_bboxes)){
            fprintf(stderr,"Can't load divisions\n");
            exit(-1);
        }
    }
    //char tmp4[1024];
    cout << "Assign splits...\n";
    //int validF=0;
    std::vector<picture_cell> cells;
    std::vector<picture_cell> cells_mosaic;

   // splitPictureCellsEven(cells,vol,_tileRows,_tileColumns,totalbb,vpblod,tasks);
    double margin =avgEdgeLen*marginExpand;
    splitPictureCells(cells,vol,margin,kd_bboxes,totalbb,vpblod,tasks);
    splitPictureCellsEven(cells_mosaic,vol,_tileRows,_tileColumns,totalbb,vpblod,tasks,!clusterRun);

   int  faketileRows=kd_bboxes.size();
   int faketileColumns=1;

    if(!externalMode || newmvs){


        {
            WriteSplitTP wstp(vrip_res,string(diced_dir)+"/splitcmds",basepath,cwd,tasks,vol);
            for(int i=0; i<(int)cells.size(); i++){

                wstp.write_cmd(cells[i]);
            }
            wstp.close();
            string splitcmd="split.py";
            shellcm.write_generic(splitcmd,wstp.getCmdFileName(),"Split",NULL,NULL);
        }


#if 0
        vector<std::string> postcmdv;
        string tcmd =basepath+string("/vrip/bin/plymerge ");
        //char tmp100[8096];
        std::vector<string> cfiles;
        // sprintf(tmp100," --invrot %f %f %f ",rx,ry,rz);
        // tcmd+=tmp100;
        string splitcmds_fn=string(diced_dir)+"/splitcmds";

        FILE *splitcmds_fp=fopen(splitcmds_fn.c_str(),"w");
        char shr_tmp[8192];
        for(int i=0; i <(int)cells.size(); i++){
            int v_count=0;
            //cout << cells[i].bboxMarginUnRot._max << " " <<cells[i].bboxMargin._max <<endl;
            sprintf(shr_tmp,"cd %s;%s/treeBBClip %s/total.ply",           cwd,
                    basepath.c_str(),diced_dir);
            for(int j=0; j <(int)vrip_cells.size(); j++){
                if(vrip_cells[j].poses.size() == 0 || !cells[i].bboxMarginUnRot.intersects(vrip_cells[j].bounds.bbox))
                    continue;
                //  sprintf(shr_tmp,"%s %s/tmp-clipped-diced-%08d.ply",shr_tmp,diced_dir,j);
                v_count++;
            }
            if(v_count== 0)
                continue;
            sprintf(shr_tmp,"%s --invrot %f %f %f --bbox %.16f %.16f %.16f %.16f %.16f %.16f -gap -F --outfile %s/un-tmp-tex-clipped-diced-r_%04d_c_%04d-lod%d.ply;",
                    shr_tmp,rx,ry,rz,
                    cells[i].bboxUnRot.xMin(),
                    cells[i].bboxUnRot.yMin(),
                    -FLT_MAX,
                    cells[i].bboxUnRot.xMax(),
                    cells[i].bboxUnRot.yMax(),
                    FLT_MAX,
                    diced_dir,
                    cells[i].row,cells[i].col,vpblod);
            /*  sprintf(shr_tmp,"%s  %s/vcgapps/bin/mergeMesh %s/un-tmp-tex-clipped-diced-r_%04d_c_%04d.ply -flip -cleansize %f -P -thresh %f -out %s/tmp-tex-clipped-diced-r_%04d_c_%04d.ply ;",shr_tmp,
                    basepath.c_str(),diced_dir,cells[i].row,cells[i].col,smallCCPer,0.9*vrip_res,diced_dir,cells[i].row,cells[i].col);*/

            fprintf(splitcmds_fp,"%s;",shr_tmp);/*
            fprintf(splitcmds_fp,"%s/treeBBClip --bbox %.16f %.16f %.16f %.16f %.16f %.16f %s/tmp-tex-clipped-diced-r_%04d_c_%04d.ply -gap -F --outfile %s/tmp1-tex-clipped-diced-r_%04d_c_%04d-lod%d.ply ;",
                    basepath.c_str(),
                    cells[i].bboxUnRot.xMin(),
                    cells[i].bboxUnRot.yMin(),
                    -FLT_MAX,
                    cells[i].bboxUnRot.xMax(),
                    cells[i].bboxUnRot.yMax(),
                    FLT_MAX,
                    diced_dir,
                    cells[i].row,cells[i].col,  diced_dir,cells[i].row,cells[i].col,vpblod);*/
            fprintf(splitcmds_fp,"setenv DISPLAY :0.0; %s/vcgapps/bin/sw-shadevis -P -n64 %s/un-tmp-tex-clipped-diced-r_%04d_c_%04d-lod%d.ply ;",
                    basepath.c_str(),diced_dir,
                    cells[i].row,cells[i].col,vpblod);
            fprintf(splitcmds_fp,"%s/treeBBClip --bbox %.16f %.16f %.16f %.16f %.16f %.16f %s/vis-un-tmp-tex-clipped-diced-r_%04d_c_%04d-lod%d.ply -dup -F --outfile %s/vis-tmp-tex-clipped-diced-r_%04d_c_%04d-lod%d.ply \n",
                    basepath.c_str(),
                    -FLT_MAX,
                    -FLT_MAX,
                    -FLT_MAX,
                    FLT_MAX,
                    FLT_MAX,
                    FLT_MAX,
                    diced_dir,
                    cells[i].row,cells[i].col,
                    vpblod,
                    diced_dir,cells[i].row,cells[i].col,vpblod);
            char tp[1024];
            sprintf(tp,"%s/bbox-vis-tmp-tex-clipped-diced-r_%04d_c_%04d.ply.txt",diced_dir,cells[i].row,cells[i].col);
            FILE *bboxfp=fopen(tp,"w");

            for(int k=0; k < (int)cells[i].imagesMargin.size(); k++){
                const Stereo_Pose_Data *pose=(&tasks[cells[i].imagesMargin[k]]);
                if(pose && pose->valid){
                    fprintf(bboxfp, "%d %s " ,pose->id,pose->left_name.c_str());
                    save_bbox_frame(pose->bbox,bboxfp);
                    osg::Matrix texmat=osgTranspose(pose->mat);
                    texmat=osg::Matrix::inverse(texmat);
                    for(int f=0; f < 4; f++)
                        for(int n=0; n < 4; n++)
                            fprintf(bboxfp," %lf",texmat(f,n));
                    fprintf(bboxfp,"\n");
                }

            }
            fclose(bboxfp);
        }
        /*  for(int i=0; i <(int)vrip_cells.size(); i++){
            if(vrip_cells[i].poses.size() == 0)
                continue;
            sprintf(tmp100, "mesh-diced/vis-clipped-diced-%08d-lod%d.ply",i,vpblod);
            cfiles.push_back(tmp100);
        }
*/

        //   sprintf(tmp100, " > mesh-diced/vis-total.ply; %s/vertCheck mesh-diced/vis-total.ply  --normcolor --outfile mesh-diced/vis-total.ply > /dev/null",basepath.c_str());
        /*  sprintf(tmp100,"%s; %s/treeBBClip --bbox %.16f %.16f %.16f %.16f %.16f %.16f mesh-diced/vis-total.ive -dup --outfile mesh-diced/vis-total.ply ",
                    tmp100,basepath.c_str(),
                    -FLT_MAX,-FLT_MAX,-FLT_MAX,
                    FLT_MAX,FLT_MAX,FLT_MAX);*/
        //tcmd+=tmp100;
        //postcmdv.push_back(tcmd);

        /*   for(int i=0; i <(int)vrip_cells.size(); i++){
            if(vrip_cells[i].poses.size() == 0)
                continue;
            int v_count=0;
            //cout << cells[i].bboxMarginUnRot._max << " " <<cells[i].bboxMargin._max <<endl;
            sprintf(shr_tmp,"cd %s;%s/treeBBClip ",           cwd,
                    basepath.c_str());
            double xRange=vrip_cells[i].bounds.bbox.xMax()-vrip_cells[i].bounds.bbox.xMin();
            double yRange=vrip_cells[i].bounds.bbox.yMax()-vrip_cells[i].bounds.bbox.yMin();
            double xMargin=xRange*0.1;
            double yMargin=yRange*0.1;
            osg::BoundingBox tmp_bbox=vrip_cells[i].bounds.bbox;
            tmp_bbox.expandBy(osg::Vec3(tmp_bbox._min[0]-xMargin,tmp_bbox._min[1]-yMargin,tmp_bbox._min[2]));
            tmp_bbox.expandBy(osg::Vec3(tmp_bbox._max[0]+xMargin,tmp_bbox._max[1]+yMargin,tmp_bbox._max[2]));

            for(int j=0; j <(int)vrip_cells.size(); j++){

                if(vrip_cells[j].poses.size() == 0 || !vrip_cells[j].bounds.bbox.intersects(tmp_bbox))
                    continue;
                sprintf(shr_tmp,"%s mesh-diced/tmp-clipped-diced-%08d.ply",shr_tmp,j);
                v_count++;
            }
            if(v_count== 0)
                continue;



            sprintf(shr_tmp,"%s --bbox %.16f %.16f %.16f %.16f %.16f %.16f -dup --outfile mesh-diced/un-clipped-diced-%08d.ply;",
                    shr_tmp,
                    tmp_bbox.xMin(),
                    tmp_bbox.yMin(),
                    -FLT_MAX,
                    tmp_bbox.xMax(),
                    tmp_bbox.yMax(),
                    FLT_MAX,
                    i);
            sprintf(shr_tmp,"%s    %s/vcgapps/bin/mergeMesh mesh-diced/un-clipped-diced-%08d.ply -thresh %f -cleansize %f -out mesh-diced/un2-clipped-diced-%08d-lod%d.ply ;",shr_tmp,
                    basepath.c_str(),i,0.9*vrip_res,smallCCPer,i,vpblod);
            sprintf(shr_tmp,"%s setenv DISPLAY :0.0; %s/vcgapps/bin/sw-shadevis -n64 -f mesh-diced/un2-clipped-diced-%08d-lod%d.ply ;",shr_tmp,basepath.c_str(),i,vpblod);
            //sprintf(shr_tmp,"%s setenv DISPLAY:0.0; cp mesh-diced/un2-clipped-diced-%08d-lod%d.ply mesh-diced/vis-un2-clipped-diced-%08d-lod%d.ply;",shr_tmp,i,vpblod,i,vpblod);


            sprintf(shr_tmp,"%s %s/treeBBClip  mesh-diced/vis-un2-clipped-diced-%08d-lod%d.ply --bbox %.16f %.16f %.16f %.16f %.16f %.16f -dup -F --outfile mesh-diced/vis-clipped-diced-%08d-lod%d.ply ",
                    shr_tmp,
                    basepath.c_str(),
                    i,vpblod,
                    vrip_cells[i].bounds.bbox.xMin(),
                    vrip_cells[i].bounds.bbox.yMin(),
                    -FLT_MAX,
                    vrip_cells[i].bounds.bbox.xMax(),
                    vrip_cells[i].bounds.bbox.yMax(),
                    FLT_MAX,
                    i,vpblod);
            fprintf(splitcmds_fp,"%s\n",shr_tmp);


        }*/


        fclose(splitcmds_fp);
        string splitcmd="split.py";
        string cwdmeshdiced=cwd;
        //  std::string extraCheckCmd=createFileCheckPython(tcmd,cwdmeshdiced,cfiles,string(tmp100),4);

        shellcm.write_generic(splitcmd,splitcmds_fn,"Split",NULL,NULL,0);
#endif
        if(!no_split){
            sysres=system("python split.py");
            if(sysres <0){
                fprintf(stderr,"Failed to run split\n");
                exit(-1);
            }
        }
    }else{
        //printf("Split for Tex %d\n",(int)cells.size());
        osg::Timer_t startTick= osg::Timer::instance()->tick();
        int totalTodoCount=cells.size()+count_vol(vol);
        int progCount=0;
        char tmpname[1024];
        if(!no_split){

            {
                osgDB::Registry::instance()->setBuildKdTreesHint(osgDB::ReaderWriter::Options::BUILD_KDTREES);
                osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(string(diced_dir)+"/totalrot.ive");

                if(!model.valid()){
                    fprintf(stderr,"Failed to load mesh-diced/totalrot.ive can't split bailing!\n");
                    exit(-1);
                }


                /*    OpenThreads::Mutex mutex;

        #pragma omp parallel num_threads(num_threads)
                if(run_stereo){
                    StereoEngine engine(calib,edgethresh,max_triangulation_len,max_feature_count,lodTexSize[0],mutex);
                    cvSetNumThreads(1);
        #pragma omp for
                    for(unsigned int i=0; i < tasks.size(); i++){
                        tasks[i].valid=engine.processPair(base_dir,tasks[i].left_name,
                                                          tasks[i].right_name,tasks[i].mat,tasks[i].bbox,tasks[i].alt);

        #pragma omp atomic
                        progCount++;
*/
                //Not thread SAFE!!!!!! TODO fix
                osg::ref_ptr<KdTreeBbox> kdbb=setupKdTree(model);
                //#pragma omp parallel num_threads(num_threads)
                {
                    {
                        //#pragma omp for
                        for(int i=0; i <(int)cells.size(); i++){
                            sprintf(tmpname,"%s/vis-tmp-tex-clipped-diced-r_%04d_c_%04d.ply",
                                    diced_dir,cells[i].row,cells[i].col);
                            int outputFaces=0;
                            cut_model(outputFaces,kdbb,tmpname,cells[i].bboxMargin,TWOBOX,&(cells[i].bbox));
                            if(outputFaces == 0){
                                string empt=tmpname+string(".empty");
                                FILE *fp=fopen(empt.c_str(),"w");
                                fprintf(fp,"0\n");
                                fclose(fp);

                            }
                            //  printf("Cell %d\n",outputFaces);
                            formatBar("Split",startTick,progCount,totalTodoCount);
                            //#pragma omp atomic
                            progCount++;
                            char tp[1024];
                            sprintf(tp,"%s/bbox-vis-tmp-tex-clipped-diced-r_%04d_c_%04d.ply.txt",diced_dir,cells[i].row,cells[i].col);
                            FILE *bboxfp=fopen(tp,"w");

                            for(int k=0; k < (int)cells[i].imagesMargin.size(); k++){
                                const Stereo_Pose_Data *pose=(&tasks[cells[i].imagesMargin[k]]);
                                if(pose && pose->valid){
                                    fprintf(bboxfp, "%d %s " ,pose->id,pose->file_name.c_str());
                                    save_bbox_frame(pose->bbox,bboxfp);
                                    for(int k=0; k < 4; k++)
                                        for(int n=0; n < 4; n++)
                                            fprintf(bboxfp," %lf",pose->mat(k,n));
                                    fprintf(bboxfp,"\n");
                                }

                            }
                            fclose(bboxfp);


                        }
                    }
                }

                {
                    osgDB::Registry::instance()->setBuildKdTreesHint(osgDB::ReaderWriter::Options::BUILD_KDTREES);
                    osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(string(diced_dir)+"/vis-total.ply");
                    if(!model.valid()){
                        fprintf(stderr,"Failed to load mesh-diced/totalrot.ive can't split bailing!\n");
                        exit(-1);
                    }
                    osg::ref_ptr<KdTreeBbox> kdbb=setupKdTree(model);
                    //#pragma omp parallel num_threads(num_threads)
                    {
                        //#pragma omp for
                        foreach_vol(cur,vol){
                            if(cur->poses.size() == 0){
                                cur->valid=false;
                                progCount++;
                                continue;
                            }
                            osg::BoundingBox box=cur->bounds.bbox;
                            sprintf(tmpname,"%s/clean_%04d%04d%04d.ply",aggdir,cur->volIdx[0],cur->volIdx[1],cur->volIdx[2]);
                            int outputFaces=0;
                            cur->valid=cut_model(outputFaces,kdbb,tmpname,box,DUP);
                            formatBar("Split",startTick,progCount,totalTodoCount);
                            //#pragma omp atomic
                            progCount++;

                        }
                    }
                }
            }
            formatBar("Split",startTick,totalTodoCount,totalTodoCount);
        }
    }
    //  fprintf(reFP,"%f %f %f %f clipped-diced-%08d-lod%d.ive\n",vrip_cells[i].bounds.min_x,vrip_cells[i].bounds.max_x,vrip_cells[i].bounds.min_y,vrip_cells[i].bounds.max_y,i,vpblod);
#ifdef USE_PROCESSES_SPLIT
    string splitcmds_fn="mesh-diced/splitcmds";

    FILE *splitcmds_fp=fopen(splitcmds_fn.c_str(),"w");


    for(int i=0; i <(int)cells.size(); i++){


        fprintf(splitcmds_fp,"cd %s;%s/treeBBClip mesh-diced/totalrot.ive --bbox %.16f %.16f %.16f %.16f %.16f %.16f -dup --outfile mesh-diced/tmp-tex-clipped-diced-r_%04d_c_%04d.ive;",
                cwd,
                basepath.c_str(),
                cells[i].bboxMargin.xMin(),
                cells[i].bboxMargin.yMin(),
                -FLT_MAX,
                cells[i].bboxMargin.xMax(),
                cells[i].bboxMargin.yMax(),
                FLT_MAX,
                cells[i].row,cells[i].col);
        fprintf(splitcmds_fp,"%s/vertCheck mesh-diced/tmp-tex-clipped-diced-r_%04d_c_%04d.ive mesh-diced/tmp-tex-clipped-diced-r_%04d_c_%04d-lod%d.ive \n",
                basepath.c_str(),
                cells[i].row,cells[i].col,  cells[i].row,cells[i].col,vpblod);
        char tp[1024];
        sprintf(tp,"mesh-diced/bbox-tmp-tex-clipped-diced-r_%04d_c_%04d.ply.txt",cells[i].row,cells[i].col);
        FILE *bboxfp=fopen(tp,"w");

        for(int k=0; k < (int)cells[i].imagesMargin.size(); k++){
            const Stereo_Pose_Data *pose=(&tasks[cells[i].imagesMargin[k]]);
            if(pose && pose->valid){
                fprintf(bboxfp, "%d %s " ,pose->id,pose->left_name.c_str());
                save_bbox_frame(pose->bbox,bboxfp);
                for(int k=0; k < 4; k++)
                    for(int n=0; n < 4; n++)
                        fprintf(bboxfp," %lf",pose->mat(k,n));
                fprintf(bboxfp,"\n");
            }

        }
        fclose(bboxfp);


    }
    for(int i=0; i <(int)vrip_cells.size(); i++){
        if(vrip_cells[i].poses.size() == 0)
            continue;
        fprintf(splitcmds_fp,"cd %s;%s/treeBBClip mesh-diced/vis-total.ply --bbox %f %f %f %f %f %f -dup --outfile mesh-diced/clipped-diced-%08d.ply;",
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
    if(!no_split)
        sysres=system("./split.py");
    //#else
    //printf("Split for Tex %d\n",(int)cells.size());
    osg::Timer_t startTick= osg::Timer::instance()->tick();
    int totalTodoCount=cells.size()+vrip_cells.size();
    int progCount=0;
    char tmpname[1024];
    if(!no_split){

        {
            osgDB::Registry::instance()->setBuildKdTreesHint(osgDB::ReaderWriter::Options::BUILD_KDTREES);
            osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("mesh-diced/totalrot.ive");
            if(!model.valid()){
                fprintf(stderr,"Failed to load mesh-diced/totalrot.ive can't split bailing!\n");
                exit(-1);
            }

            /*    OpenThreads::Mutex mutex;

        #pragma omp parallel num_threads(num_threads)
                if(run_stereo){
                    StereoEngine engine(calib,edgethresh,max_triangulation_len,max_feature_count,lodTexSize[0],mutex);
                    cvSetNumThreads(1);
        #pragma omp for
                    for(unsigned int i=0; i < tasks.size(); i++){
                        tasks[i].valid=engine.processPair(base_dir,tasks[i].left_name,
                                                          tasks[i].right_name,tasks[i].mat,tasks[i].bbox,tasks[i].alt);

        #pragma omp atomic
                        progCount++;
*/
            //Not thread SAFE!!!!!! TODO fix
            osg::ref_ptr<KdTreeBbox> kdbb=setupKdTree(model);
            //#pragma omp parallel num_threads(num_threads)
            {
                {
                    //#pragma omp for
                    for(int i=0; i <(int)cells.size(); i++){
                        sprintf(tmpname,"mesh-diced/1tmp-tex-clipped-diced-r_%04d_c_%04d-lod%d.ive",
                                cells[i].row,cells[i].col,vpblod);
                        cut_model(kdbb,tmpname,cells[i].bboxMargin,DUP);
                        formatBar("Split",startTick,progCount,totalTodoCount);
                        //#pragma omp atomic
                        progCount++;
                        char tp[1024];
                        sprintf(tp,"mesh-diced/bbox-tmp-tex-clipped-diced-r_%04d_c_%04d.ply.txt",cells[i].row,cells[i].col);
                        FILE *bboxfp=fopen(tp,"w");

                        for(int k=0; k < (int)cells[i].imagesMargin.size(); k++){
                            const Stereo_Pose_Data *pose=(&tasks[cells[i].imagesMargin[k]]);
                            if(pose && pose->valid){
                                fprintf(bboxfp, "%d %s " ,pose->id,pose->left_name.c_str());
                                save_bbox_frame(pose->bbox,bboxfp);
                                osg::Matrix texmat=osgTranspose(pose->mat);
                                texmat=osg::Matrix::inverse(texmat);
                                for(int k=0; k < 4; k++)
                                    for(int n=0; n < 4; n++)
                                        fprintf(bboxfp," %lf",texmat(k,n));
                                fprintf(bboxfp,"\n");
                            }

                        }
                        fclose(bboxfp);


                    }
                }
            }

            {
                osgDB::Registry::instance()->setBuildKdTreesHint(osgDB::ReaderWriter::Options::BUILD_KDTREES);
                osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("mesh-diced/vis-total.ply");
                if(!model.valid()){
                    fprintf(stderr,"Failed to load mesh-diced/totalrot.ive can't split bailing!\n");
                    exit(-1);
                }
                osg::ref_ptr<KdTreeBbox> kdbb=setupKdTree(model);
                //#pragma omp parallel num_threads(num_threads)
                {
                    //#pragma omp for
                    for(int i=0; i <(int)vrip_cells.size(); i++){
                        if(vrip_cells[i].poses.size() == 0){
                            vrip_cells[i].valid=false;
                            continue;
                        }
                        osg::BoundingBox box=vrip_cells[i].bounds.bbox;
                        sprintf(tmpname,"mesh-diced/clipped-diced-%08d-lod%d.ply",
                                i,vpblod);
                        vrip_cells[i].valid=cut_model(kdbb,tmpname,box,DUP);
                        formatBar("Split",startTick,progCount,totalTodoCount);
                        //#pragma omp atomic
                        progCount++;

                    }
                }
            }
        }
    }
    formatBar("Split",startTick,totalTodoCount,totalTodoCount);

#endif

    std::vector<int > sizeStepTotal(vpblod+1);


    for(int j=vpblod; j >=0; j--){
        sizeStepTotal[j]= std::max(numberFacesAll/((int)pow(4.0,vpblod-j)),std::min(targetScreenFaces,numberFacesAll));
        // printf("%d\n",sizeStepTotal[j]);

    }
#ifdef SINGLE_MESH_TEX

    double minFrac=0.2;
    std::vector<int> numFaces(vrip_cells.size(),0);
    std::vector<double> resFrac(vrip_cells.size(),0);
    std::vector<double> cur_res(vrip_cells.size(),0);

    int totalFaces=0;
    int largestNumFaces=0;
    double desiredRes=0.1;
    for(int i=0; i <(int)vrip_cells.size(); i++){
        if(vrip_cells[i].poses.size() == 0 || !vrip_cells[i].valid)
            continue;
        char tmpf[1024];
        sprintf(tmpf,"mesh-diced/vis-clipped-diced-%08d-lod%d.ply",i,vpblod);
        osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(tmpf);
        //assert(model.valid());
        if(!model.valid()){
            osg::notify(osg::ALWAYS) <<"No valid model ";
            cout << "valid ?  " << tmpf << " " <<vrip_cells[i].valid<<"\n";
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

        //printf("%d: %d \n",i,faces);
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
            //double newRes=(cur_res[i]+(vpblod-j)*resFrac[i]);
            sizeStep[i][j]= std::max(numFaces[i]/(pow(4.0,vpblod-j)),(largestNumFaces*minFrac));
            //   osg::notify(osg::ALWAYS)  << "Level " << j << "Res " <<  newRes << "Orig Faces " << numFaces[i] << "New faces " << sizeStep[i][j] <<endl;
        }
        //                    sizeStep[i]=(int)round(numFaces[i]*resFrac);
        //                 printf("Step size %d %f\n",sizeSteps[i],resFrac);
    }
#endif

    string texcmds_fn[]={string(diced_dir)+"/remapcmds",string(diced_dir)+"/depthtexcmds",string(diced_dir)+"/flattexcmds",string(diced_dir)+"/sizetexcmds"};

    string vartexcmds_fn=string(diced_dir)+"/vartexcmds";

    string imgbase;
    if(newmvs)
        imgbase=mvs_output_dir+"/visualize.color/";
    else
        imgbase=(compositeMission? "/":"/img/");
    int VTtileSize=256;
    int tileBorder=1;
    int ajustedGLImageSizeX=(int)reimageSize.x() - (!useVirtTex ?  0 : ((reimageSize.x()/VTtileSize)*2*tileBorder));
    int ajustedGLImageSizeY=(int)reimageSize.y() - (!useVirtTex ?  0 : ((reimageSize.y()/VTtileSize)*2*tileBorder));
    //char tmpsize[1024];
    int totalX=ajustedGLImageSizeX*faketileRows;
    //int totalY=ajustedGLImageSizeY*_tileColumns;

    /*if(!reparamTex){
        sprintf(tmpsize," --noatlas --size %d %d ",totalX,totalY);
    }else{
        sprintf(tmpsize," ");

    }*/

    //string tcmd =basepath+string("/atlasmesh -mat viewproj.mat -cells image_areas.txt ");// + string(tmpsize);
    char tmpfn2[8096];

    string calcTexFn=string(diced_dir)+"/"+"calctexcmds";
    FILE *calcTexFn_fp=fopen(calcTexFn.c_str(),"w");


    std::vector<string> cfiles;

{
        FILE *texcmds_fp[4]={fopen(texcmds_fn[REMAP].c_str(),"w"),fopen(texcmds_fn[DEPTH].c_str(),"w"),fopen(texcmds_fn[FLAT].c_str(),"w"),fopen(texcmds_fn[REMAP_FLAT_SIZE].c_str(),"w")};

    FILE *FP2=fopen("image_areas.txt","w");
    FILE *reFP=fopen("rebbox.txt","w");



    fprintf(reFP,"%.16f %.16f %.16f %.16f %.16f %.16f %d %d total\n",totalbb.xMin(),totalbb.xMax(),totalbb.yMin(),totalbb.yMax(),totalbb.zMin(),
            totalbb.zMax(),faketileColumns,faketileRows);

    fprintf(FP2,"0 %d 0 %d total total 0\n",ajustedGLImageSizeX*faketileRows,ajustedGLImageSizeY*faketileColumns);

    for(int i=0; i <(int)cells.size(); i++){
        char tmpfn[1024];
        sprintf(tmpfn,"%s/vis-tmp-tex-clipped-diced-r_%04d_c_%04d.ply", diced_dir,cells[i].row,cells[i].col);

        if( cells[i].images.size() == 0 ||  !osgDB::fileExists(tmpfn) || checkIsEmptyPly(tmpfn)){

            fprintf(reFP,"%.16f %.16f %.16f %.16f %.16f %.16f %d %d %s\n",cells[i].bbox.xMin(),cells[i].bbox.xMax(),cells[i].bbox.yMin(),cells[i].bbox.yMax(),cells[i].bbox.zMin(),
                    cells[i].bbox.zMax(),cells[i].col,cells[i].row,"null");
            fprintf(FP2,"-1 -1 -1 -1 null null 0\n");
            if( !osgDB::fileExists(tmpfn) && !osgDB::fileExists(string(tmpfn)+".empty")){
                printf("Failed cell images %d exists %d empty %d %s\n",(int)cells[i].images.size(),osgDB::fileExists(tmpfn),
                       checkIsEmptyPly(tmpfn),tmpfn);
                exit(-1);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            exit(-1);

            }
            continue;



        }
        fprintf(reFP,"%.16f %.16f %.16f %.16f %.16f %.16f %d %d %s\n",cells[i].bbox.xMin(),cells[i].bbox.xMax(),cells[i].bbox.yMin(),cells[i].bbox.yMax(),cells[i].bbox.zMin(),
                cells[i].bbox.zMax(),cells[i].col,cells[i].row,cells[i].name.c_str());

        string remap_mesh_ext=reparamTex ? "remap-" : "flat-";

        sprintf(tmpfn2,"%s/%stex-clipped-diced-r_%04d_c_%04d-lod%d.ply", diced_dir,remap_mesh_ext.c_str(),cells[i].row,cells[i].col,vpblod);
        cfiles.push_back(tmpfn2);

        fprintf(calcTexFn_fp,"cd %s",
                cwd);
        if(newmvs){
            fprintf(calcTexFn_fp,";%s/calcTexCoordBundler %s/%s %s/vis-tmp-tex-clipped-diced-r_%04d_c_%04d.ply --camimgfile  %s/tmp/mvs/id_image_list.txt --outfile %s/tex-clipped-diced-r_%04d_c_%04d-lod%d.ply --zrange %f %f --invrot %f %f %f --tex-margin %f --bbox-margin %f\n",
                basepath.c_str(),
                cwd,
                mvs_output_dir.c_str(),
                diced_dir,
                cells[i].row,cells[i].col,
                cwd,
             //   diced_dir,cells[i].row,cells[i].col,
                diced_dir,
                cells[i].row,cells[i].col,
                vpblod,totalbb_unrot.zMin(),totalbb_unrot.zMax(),
                rx,ry,rz,tex_margin,bbox_margin);
        }else{
            fprintf(calcTexFn_fp,";%s/calcTexCoord %s %s/vis-tmp-tex-clipped-diced-r_%04d_c_%04d.ply --bbfile  %s/bbox-vis-tmp-tex-clipped-diced-r_%04d_c_%04d.ply.txt --outfile %s/tex-clipped-diced-r_%04d_c_%04d-lod%d.ply --zrange %f %f --invrot %f %f %f --tex-margin %f --bbox-margin %f\n",
                basepath.c_str(),
                base_dir.c_str(),
                diced_dir,
                cells[i].row,cells[i].col,
                diced_dir,cells[i].row,cells[i].col,
                diced_dir,
                cells[i].row,cells[i].col,
                vpblod,totalbb_unrot.zMin(),totalbb_unrot.zMax(),
                rx,ry,rz,tex_margin,bbox_margin);
        }

        for(int z=0; z<NUM_TEX_FILES; z++){
            fprintf(texcmds_fp[z],"cd %s",
                    cwd);

            if(hw_image){
                fprintf(texcmds_fp[z]," --tex_cache %s %d  --mat %s/tex-clipped-diced-r_%04d_c_%04d.mat -lat %.28f -lon %.28f ",cachedtexdir[0].first.c_str(),cachedtexdir[0].second,
                        diced_dir,
                        cells[i].row,cells[i].col,
                        latOrigin , longOrigin);

                if(use_debug_shader)
                    fprintf(texcmds_fp[z]," --debug-shader ");
                if(!storeTexMesh)
                    fprintf(texcmds_fp[z]," --imageNode %d %d %d %d %d %d --untex",cells[i].row,cells[i].col,faketileRows,faketileColumns,ajustedGLImageSizeX,ajustedGLImageSizeY);
                if(useAtlas)
                    fprintf(texcmds_fp[z]," --atlas");
            }
            else{
                string teximgcmd;
                ostringstream sizestr;
                char tmpfn[1024];
                switch(z){
                case REMAP:
                    teximgcmd = "vcgapps/bin/reparam";
                    sizestr << "--srcsize " <<imageWidth << " "<<imageHeight << " ";
                    sizestr<<"--scale "<<scaleRemapTex;
                    break;
                case DEPTH:
                    teximgcmd="depthmap";
                    sizestr<<"--size "<<ajustedGLImageSizeX<<" "<<ajustedGLImageSizeY;

                    break;
                case FLAT:
                    teximgcmd="nonmem";
                    sizestr<<"--size "<<ajustedGLImageSizeX<<" "<<ajustedGLImageSizeY;

                    break;
                case REMAP_FLAT_SIZE:
                    teximgcmd = "vcgapps/bin/renderReorderAA";
                    sizestr << "--srcsize " <<imageWidth << " "<<imageHeight << " ";

//                    sizestr<<"--size "<<ajustedGLImageSizeX<<" "<<ajustedGLImageSizeY;
                    sprintf(tmpfn," --remap %s/param-tex-clipped-diced-r_%04d_c_%04d-lod%d.ply ",
                            diced_dir,
                            cells[i].row,cells[i].col,
                            vpblod);
                    sizestr << " -pyr ";
                    sizestr << " -scalefile " << diced_dir <<"/globalscale.txt ";
                    sizestr << tmpfn;
                    break;
                }

                if(useVirtTex)
                    sizestr<< " --vt " <<VTtileSize<< " " <<tileBorder<< " ";
                if(newmvs){
                fprintf(texcmds_fp[z],";%s/%s %s/tex-clipped-diced-r_%04d_c_%04d-lod%d.ply tmp/mvs/id_image_list.txt %s  --invrot %f %f %f  --image %d %d %d %d -lat %.28f -lon %.28f --jpeg-quality %d --mosaicid %d %s",
                        basepath.c_str(),
                        teximgcmd.c_str(),
                        diced_dir,
                        cells[i].row,cells[i].col,
                        vpblod,
                        (cwd+imgbase).c_str(),
                        rx,ry,rz,
                        cells[i].row,cells[i].col,_tileRows,_tileColumns,
                        latOrigin , longOrigin,jpegQuality,i,sizestr.str().c_str());
                }else{
                    fprintf(texcmds_fp[z],";%s/%s  %s/tex-clipped-diced-r_%04d_c_%04d-lod%d.ply %s/bbox-vis-tmp-tex-clipped-diced-r_%04d_c_%04d.ply.txt %s  --invrot %f %f %f  --image %d %d %d %d -lat %.28f -lon %.28f --jpeg-quality %d --mosaicid %d %s",
                            basepath.c_str(),
                            teximgcmd.c_str(),
                            diced_dir,
                            cells[i].row,cells[i].col,
                            vpblod,
                            diced_dir,
                            cells[i].row,cells[i].col,
                            (base_dir+imgbase).c_str(),
                            rx,ry,rz,
                            cells[i].row,cells[i].col,_tileRows,_tileColumns,
                            latOrigin , longOrigin,jpegQuality,i,sizestr.str().c_str());

                }
                if(blending)
                    fprintf(texcmds_fp[z]," --blend");

                /*                fprintf(texcmds_fp[z],";%s/depthmap  %s/tex-clipped-diced-r_%04d_c_%04d-lod%d.ply  --mat %s/tex-clipped-diced-r_%04d_c_%04d.mat --invrot %f %f %f  --image %d %d %d %d -lat %.28f -lon %.28f --jpeg-quality %d --mosaicid %d --size %d %d",
                        basepath.c_str(),
                        diced_dir,
                        cells[i].row,cells[i].col,
                        vpblod,

                        diced_dir,
                        cells[i].row,cells[i].col,
                        rx,ry,rz,
                        cells[i].row,cells[i].col,_tileRows,_tileColumns,
                        latOrigin , longOrigin,jpegQuality,i,ajustedGLImageSizeX,ajustedGLImageSizeY);*/
            }
        }
     /*   if(!hw_image){

            fprintf(vartexcmds_fp,"%s/meshvar  %s/tex-clipped-diced-r_%04d_c_%04d-lod%d.ply %s/bbox-vis-tmp-tex-clipped-diced-r_%04d_c_%04d.ply.txt %s --mat %s/tex-clipped-diced-r_%04d_c_%04d.mat --invrot %f %f %f --size %d %d --image %d %d %d %d -lat %.28f -lon %.28f",
                    basepath.c_str(),
                    diced_dir,
                    cells[i].row,cells[i].col,
                    vpblod,
                    diced_dir,
                    cells[i].row,cells[i].col,
                    (base_dir+imgbase).c_str(),
                    diced_dir,
                    cells[i].row,cells[i].col,
                    rx,ry,rz,
                    ajustedGLImageSizeX,ajustedGLImageSizeY,
                    cells[i].row,cells[i].col,_tileRows,_tileColumns,
                    latOrigin , longOrigin);
            if(writeout_meshvar)
                fprintf(vartexcmds_fp," --write");
        }*/
       /* char tmp_ds[1024];
        tmp_ds[0]='\0';
        int num_samples=6;
        for(int p=0; p<num_samples; p++)
            sprintf(tmp_ds,"%s %d",tmp_ds,(int)pow(2.0,p+1));
     */
            for(int z=0; z<NUM_TEX_FILES; z++){
                    fprintf(texcmds_fp[z],"\n");



        }

        //tmp_ds[0]='\0';
        //  num_samples=0;
        int levels=useVirtTex ? 0:(int)ceil(log( max( ajustedGLImageSizeX, ajustedGLImageSizeY ))/log(2.0) );
        string remap_ext= reparamTex ? "remap" : "tmp";
        //  for(int p=0; p<num_samples; p++)
        //      sprintf(tmp_ds,"%s %d",tmp_ds,(int)pow(2,p+1));
        fprintf(FP2,"%d %d %d %d %s/image_r%04d_c%04d_rs%04d_cs%04d-%s.ppm %s/image_r%04d_c%04d_rs%04d_cs%04d.ppm %d\n",(totalX-(ajustedGLImageSizeX*(cells[i].row+1))),
                (totalX-ajustedGLImageSizeX*(cells[i].row)),ajustedGLImageSizeY*cells[i].col,ajustedGLImageSizeY*(cells[i].col+1),diced_img_dir,cells[i].row,cells[i].col,_tileRows,_tileColumns,
                remap_ext.c_str(),diced_img_dir,cells[i].row,cells[i].col,faketileRows,faketileColumns,levels);


/*
        fprintf(FP3,"image_r%04d_c%04d_rs%04d_cs%04d.tif ",cells[i].row,cells[i].col,_tileRows,_tileColumns);
        fprintf(FP5,"depth_r%04d_c%04d_rs%04d_cs%04d.tif ",cells[i].row,cells[i].col,_tileRows,_tileColumns);

        fprintf(FP4,"var_r%04d_c%04d_rs%04d_cs%04d.tif ",cells[i].row,cells[i].col,_tileRows,_tileColumns);
*/
        // fprintf(texcmds_fp,"\n");

        /* fprintf(texcmds_fp,"osgconv -O \"compressed=1 noTexturesInIVEFile=1 noLoadExternalReferenceFiles=1 useOriginalExternalReferences=1\" mesh-diced/tex-clipped-diced-r_%04d_c_%04d-lod%d-uncomp.ive  mesh-diced/tex-clipped-diced-r_%04d_c_%04d-lod%d.ive\n",

                            cells[i].row,cells[i].col,
                            vpblod,
                            cells[i].row,cells[i].col
                            ,vpblod);*/

    }
    fclose(reFP);
    fclose(FP2);
    for(int z=0; z<NUM_TEX_FILES; z++)
        fclose(texcmds_fp[z]);
}

    enum {MOSAIC,DEPTHMOSAIC,NUM_MOSAIC_FILES};
    FILE *vartexcmds_fp=fopen(vartexcmds_fn.c_str(),"w");

    string mosaiccmmds_fn[]={string(diced_dir)+"/mosaiccmds",string(diced_dir)+"/depthmosaiccmds"};

    FILE *mosaiccmds_fp[2]={fopen(mosaiccmmds_fn[MOSAIC].c_str(),"w"),fopen(mosaiccmmds_fn[DEPTHMOSAIC].c_str(),"w")};

    FILE *FP3=fopen("createmosaic.sh","w");
    FILE *FP4=fopen("createmosaicvar.sh","w");
    FILE *FP5=fopen("createmosaicdepth.sh","w");
    FILE *mosaic_txt_fp=fopen("mosaic_files.txt","w");
    // FILE *FP5=fopen("createrangeimg.sh","w");

    if( ! FP3){
        fprintf(stderr,"Can't open mosaic scripts\n");
        exit(-1);
    }
    fprintf(FP3,"#!/bin/bash\n cd mosaic \n gdalbuildvrt  mosaic.vrt ");
    fprintf(FP4,"#!/bin/bash\n cd mosaic \n gdalbuildvrt  mosaicvar.vrt ");
    fprintf(FP5,"#!/bin/bash\n cd mosaic \n gdalbuildvrt  depth.vrt ");


    fprintf(mosaic_txt_fp,"%d %d %d %d total\n",adjustedSize,adjustedSize,_tileRows,_tileColumns);

    for(int i=0; i <(int)cells_mosaic.size(); i++){
        char tmpfn[1024];
        string mesh_list;

        for(int j=0; j <(int)cells.size(); j++){
            if(cells_mosaic[i].bbox.intersects(cells.at(j).bbox)){

                sprintf(tmpfn,"%s/tex-clipped-diced-r_%04d_c_%04d-lod%d.ply ", diced_dir,cells[j].row,cells[j].col,vpblod);
                mesh_list+=tmpfn;
            }
        }



            if( cells_mosaic[i].images.size() == 0 || mesh_list.size() == 0){
                fprintf(mosaic_txt_fp,"%.16f %.16f %.16f %.16f %.16f %.16f %d %d %s\n",cells_mosaic[i].bbox.xMin(),cells_mosaic[i].bbox.xMax(),cells_mosaic[i].bbox.yMin(),cells_mosaic[i].bbox.yMax(),cells_mosaic[i].bbox.zMin(),
                        cells_mosaic[i].bbox.zMax(),cells_mosaic[i].col,cells_mosaic[i].row,"null");
                continue;
            }
            fprintf(mosaic_txt_fp,"%.16f %.16f %.16f %.16f %.16f %.16f %d %d %s\n",cells_mosaic[i].bbox.xMin(),cells_mosaic[i].bbox.xMax(),cells_mosaic[i].bbox.yMin(),cells_mosaic[i].bbox.yMax(),cells_mosaic[i].bbox.zMin(),
                           cells_mosaic[i].bbox.zMax(),cells_mosaic[i].col,cells_mosaic[i].row,cells_mosaic[i].name.c_str());
/*
        if( cells_mosaic[i].images.size() == 0 || mesh_list.size() == 0){

            fprintf(reFP,"%.16f %.16f %.16f %.16f %.16f %.16f %d %d %s\n",cells_mosaic[i].bbox.xMin(),cells_mosaic[i].bbox.xMax(),cells_mosaic[i].bbox.yMin(),cells_mosaic[i].bbox.yMax(),cells_mosaic[i].bbox.zMin(),
                    cells_mosaic[i].bbox.zMax(),cells_mosaic[i].col,cells_mosaic[i].row,"null");
            fprintf(FP2,"-1 -1 -1 -1 null null 0\n");
            ///if( !osgDB::fileExists(tmpfn) && !osgDB::fileExists(string(tmpfn)+".empty")){
               // printf("Failed cell images %d exists %d empty %d %s\n",(int)cells_mosaic[i].images.size(),osgDB::fileExists(tmpfn),
                //       checkIsEmptyPly(tmpfn),tmpfn);
               // exit(-1);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            exit(-1);

           // }
            continue;



        }
*/
      /*  fprintf(reFP,"%.16f %.16f %.16f %.16f %.16f %.16f %d %d %s\n",cells_mosaic[i].bbox.xMin(),cells_mosaic[i].bbox.xMax(),cells_mosaic[i].bbox.yMin(),cells_mosaic[i].bbox.yMax(),cells_mosaic[i].bbox.zMin(),
                cells_mosaic[i].bbox.zMax(),cells_mosaic[i].col,cells_mosaic[i].row,cells_mosaic[i].name.c_str());

        string remap_mesh_ext=reparamTex ? "remap-" : "flat-";

        sprintf(tmpfn2,"%s/%stex-clipped-diced-r_%04d_c_%04d-lod%d.ply", diced_dir,remap_mesh_ext.c_str(),cells_mosaic[i].row,cells_mosaic[i].col,vpblod);
        cfiles.push_back(tmpfn2);

*/

        for(int z=0; z<NUM_MOSAIC_FILES; z++){
            fprintf(mosaiccmds_fp[z],"cd %s",
                    cwd);


                string teximgcmd;
                ostringstream sizestr;
                //char tmpfn[1024];
                switch(z){
                case MOSAIC:
                    teximgcmd = "vcgapps/bin/renderSquareAA";
                    sizestr<<"--size "<<ajustedGLImageSizeX<<" "<<ajustedGLImageSizeY;

                    break;
                case DEPTHMOSAIC:
                    teximgcmd="depthmap";
                    sizestr<<"--size "<<ajustedGLImageSizeX<<" "<<ajustedGLImageSizeY;

                    break;

          /*      case REMAP_FLAT_SIZE:
                    teximgcmd = "vcgapps/bin/renderReorderAA";
                    sizestr << "--srcsize " <<imageWidth << " "<<imageHeight << " ";

//                    sizestr<<"--size "<<ajustedGLImageSizeX<<" "<<ajustedGLImageSizeY;
                    sprintf(tmpfn," --remap %s/param-tex-clipped-diced-r_%04d_c_%04d-lod%d.ply ",
                            diced_dir,
                            cells[i].row,cells[i].col,
                            vpblod);
                    sizestr << tmpfn;
                    break;*/
                }

                if(useVirtTex)
                    sizestr<< " --vt " <<VTtileSize<< " " <<tileBorder<< " ";
                if(newmvs){
                    fprintf(mosaiccmds_fp[z],";%s/%s  %s  --bbox %.16f %.16f %.16f %.16f %.16f %.16f --imglist tmp/mvs/id_image_list.txt --imagedir %s --mat %s/tex-clipped-diced-r_%04d_c_%04d.mat --invrot %f %f %f  --image %d %d %d %d -lat %.28f -lon %.28f --jpeg-quality %d --mosaicid %d %s",
                            basepath.c_str(),
                            teximgcmd.c_str(),
                            mesh_list.c_str(),
                            cells_mosaic[i].bbox.xMin(),
                            cells_mosaic[i].bbox.yMin(),
                            cells_mosaic[i].bbox.zMin(),
                            cells_mosaic[i].bbox.xMax(),
                            cells_mosaic[i].bbox.yMax(),
                            cells_mosaic[i].bbox.zMax(),
                            //diced_dir,
                            //cells_mosaic[i].row,cells_mosaic[i].col,_tileRows,_tileColumns,
                            (cwd+imgbase).c_str(),
                            diced_dir,
                            cells_mosaic[i].row,cells_mosaic[i].col,
                            rx,ry,rz,
                            cells_mosaic[i].row,cells_mosaic[i].col,_tileRows,_tileColumns,
                            latOrigin , longOrigin,jpegQuality,i,sizestr.str().c_str());
                }else{
                fprintf(mosaiccmds_fp[z],";%s/%s  %s  --bbox %.16f %.16f %.16f %.16f %.16f %.16f --imglist %s/even-bbox-vis-tmp-tex-clipped-diced-r_%04d_c_%04d_rs%04d_cs%04d.ply.txt --imagedir %s --mat %s/tex-clipped-diced-r_%04d_c_%04d.mat --invrot %f %f %f  --image %d %d %d %d -lat %.28f -lon %.28f --jpeg-quality %d --mosaicid %d %s",
                        basepath.c_str(),
                        teximgcmd.c_str(),
                        mesh_list.c_str(),
                        cells_mosaic[i].bbox.xMin(),
                        cells_mosaic[i].bbox.yMin(),
                        cells_mosaic[i].bbox.zMin(),
                        cells_mosaic[i].bbox.xMax(),
                        cells_mosaic[i].bbox.yMax(),
                        cells_mosaic[i].bbox.zMax(),
                        diced_dir,
                        cells_mosaic[i].row,cells_mosaic[i].col,_tileRows,_tileColumns,
                        (base_dir+imgbase).c_str(),
                        diced_dir,
                        cells_mosaic[i].row,cells_mosaic[i].col,
                        rx,ry,rz,
                        cells_mosaic[i].row,cells_mosaic[i].col,_tileRows,_tileColumns,
                        latOrigin , longOrigin,jpegQuality,i,sizestr.str().c_str());
                }
                if(blending)
                    fprintf(mosaiccmds_fp[z]," --blend");



        }
        if(!hw_image){

            fprintf(vartexcmds_fp,"%s/meshvar  %s/tex-clipped-diced-r_%04d_c_%04d-lod%d.ply %s/bbox-vis-tmp-tex-clipped-diced-r_%04d_c_%04d.ply.txt %s --mat %s/tex-clipped-diced-r_%04d_c_%04d.mat --invrot %f %f %f --size %d %d --image %d %d %d %d -lat %.28f -lon %.28f",
                    basepath.c_str(),
                    diced_dir,
                    cells_mosaic[i].row,cells_mosaic[i].col,
                    vpblod,
                    diced_dir,
                    cells_mosaic[i].row,cells_mosaic[i].col,
                    (base_dir+imgbase).c_str(),
                    diced_dir,
                    cells_mosaic[i].row,cells_mosaic[i].col,
                    rx,ry,rz,
                    ajustedGLImageSizeX,ajustedGLImageSizeY,
                    cells_mosaic[i].row,cells_mosaic[i].col,_tileRows,_tileColumns,
                    latOrigin , longOrigin);
            if(writeout_meshvar)
                fprintf(vartexcmds_fp," --write");
        }
        char tmp_ds[1024];
        tmp_ds[0]='\0';
        //int num_samples=6;
        int num_samples= (int)floor(std::log(max(ajustedGLImageSizeX,ajustedGLImageSizeY)) / std::log(2.0));
        //printf("Num samples %d\n",num_samples);
        for(int p=0; p<num_samples; p++)
            sprintf(tmp_ds,"%s %d",tmp_ds,(int)pow(2.0,p+1));
        sprintf(tmp_ds,"%s %d",tmp_ds,max(ajustedGLImageSizeX,ajustedGLImageSizeY));
       // printf("levels %s\n",tmp_ds);
       // if(jpegQuality<0)
        const char *mosaicname[2]={"image","depth"};
        {
            for(int z=0; z<NUM_MOSAIC_FILES; z++){
                fprintf(mosaiccmds_fp[z],";gdaladdo -r average --config COMPRESS_OVERVIEW DEFLATE mosaic/%s_r%04d_c%04d_rs%04d_cs%04d.tif %s\n",mosaicname[z],cells_mosaic[i].row,cells_mosaic[i].col,_tileRows,_tileColumns, tmp_ds);
               // else
                   // fprintf(texcmds_fp[z],"\n");
            }
            fprintf(vartexcmds_fp,";gdaladdo -r average --config COMPRESS_OVERVIEW DEFLATE  mosaic/var_r%04d_c%04d_rs%04d_cs%04d.tif %s\n",cells_mosaic[i].row,cells_mosaic[i].col,_tileRows,_tileColumns, tmp_ds);

        }/*else{
            for(int z=0; z<NUM_MOSAIC_FILES; z++){
               // if(z != REMAP)
                    fprintf(mosaiccmds_fp[z],";gdaladdo -r average --config COMPRESS_OVERVIEW JPEG --config PHOTOMETRIC_OVERVIEW YCBCR --config INTERLEAVE_OVERVIEW PIXEL --config JPEG_QUALITY_OVERVIEW %d mosaic/image_r%04d_c%04d_rs%04d_cs%04d.tif %s\n",jpegQuality,cells_mosaic[i].row,cells_mosaic[i].col,_tileRows,_tileColumns, tmp_ds);
                //else
                   // fprintf(texcmds_fp[z],"\n");

            }
            fprintf(vartexcmds_fp,";gdaladdo -r average --config COMPRESS_OVERVIEW JPEG --config PHOTOMETRIC_OVERVIEW YCBCR --config INTERLEAVE_OVERVIEW PIXEL --config JPEG_QUALITY_OVERVIEW %d mosaic/var_r%04d_c%04d_rs%04d_cs%04d.tif %s\n",jpegQuality,cells_mosaic[i].row,cells_mosaic[i].col,_tileRows,_tileColumns, tmp_ds);
        }*/

        tmp_ds[0]='\0';
        //  num_samples=0;
        //int levels=useVirtTex ? 0:(int)ceil(log( max( ajustedGLImageSizeX, ajustedGLImageSizeY ))/log(2.0) );
        string remap_ext= reparamTex ? "remap" : "tmp";
        /*fprintf(FP2,"%d %d %d %d %s/image_r%04d_c%04d_rs%04d_cs%04d-%s.ppm %s/image_r%04d_c%04d_rs%04d_cs%04d.ppm %d\n",(totalX-(ajustedGLImageSizeX*(cells_mosaic[i].row+1))),
                (totalX-ajustedGLImageSizeX*(cells_mosaic[i].row)),ajustedGLImageSizeY*cells_mosaic[i].col,ajustedGLImageSizeY*(cells_mosaic[i].col+1),diced_img_dir,cells_mosaic[i].row,cells_mosaic[i].col,_tileRows,_tileColumns,
                remap_ext.c_str(),diced_img_dir,cells_mosaic[i].row,cells_mosaic[i].col,_tileRows,_tileColumns,levels);
*/
        fprintf(FP3,"image_r%04d_c%04d_rs%04d_cs%04d.tif ",cells_mosaic[i].row,cells_mosaic[i].col,_tileRows,_tileColumns);
        fprintf(FP5,"depth_r%04d_c%04d_rs%04d_cs%04d.tif ",cells_mosaic[i].row,cells_mosaic[i].col,_tileRows,_tileColumns);
        fprintf(FP4,"var_r%04d_c%04d_rs%04d_cs%04d.tif ",cells_mosaic[i].row,cells_mosaic[i].col,_tileRows,_tileColumns);

    }
    string cwdmeshdiced=cwd;

    //std::string extraCheckCmd;
    //sprintf(tmp100, " -F -outfile %s/unclean-tex-total.ply ; %s/vcgapps/bin/cleanTexMesh %s/unclean-tex-total.ply --normcolor -out %s/tex-total.ply",diced_dir,basepath.c_str(),diced_dir,diced_dir);

    //  extraCheckCmd= reparamTex ? createFileCheckPython(tcmd,cwdmeshdiced,cfiles,string(tmp100),4): "";
    //extraCheckCmd=  createFileCheckPython(tcmd,cwdmeshdiced,cfiles,string(tmp100),4);
    for(int z=0; z<NUM_MOSAIC_FILES; z++)
        fclose(mosaiccmds_fp[z]);
    fclose(vartexcmds_fp);
    fclose(mosaic_txt_fp);
    fclose(calcTexFn_fp);
    fprintf(FP3,"\nrm -f thumb.tif\ngdalwarp mosaic.vrt -ts 512 512 thumb.tif\nconvert -quiet thumb.tif thumb.png\n");
        //#gdaladdo -ro --config INTERLEAVE_OVERVIEW PIXEL --config COMPRESS_OVERVIEW JPEG mosaic.vrt 2 4 8 16 32\n");
    fchmod(fileno(FP3),0777);

    fclose(FP3);

    fprintf(FP4,"\n#gdaladdo -ro --config INTERLEAVE_OVERVIEW PIXEL --config COMPRESS_OVERVIEW JPEG mosaic.vrt 2 4 8 16 32\n");
    fchmod(fileno(FP4),0777);
    fprintf(FP5,"\n#gdaladdo -ro --config INTERLEAVE_OVERVIEW PIXEL --config COMPRESS_OVERVIEW JPEG depth.vrt 2 4 8 16 32\n");
    fprintf(FP5,"\ngdaldem slope depth.vrt slope.tif\n"\
            "echo -n -e '90 0 0 0\\n0 255 255 255\\n' > slope-ramp.txt\n"\
            "echo -n -e '0%% 191   0 255\\n10%%  63   0 255\\n20%%   0  63 255\\n30%%   0 191 255\\n"\
            "40%%   0 255 191\\n50%%   0 255  63\\n60%%  63 255   0\\n70%% 191 255   0\\n80%% 255 191   0\\n"\
            "90%% 255  63   0\\nnv    255   255   255\\n' > ramp.txt\n"\
            "gdaldem color-relief slope.tif slope-ramp.txt slope_render.tif\n"\
            "gdaldem hillshade depth.vrt hillshade.tif\n"\
            "mogrify -fill \"#b5b5b5\" -opaque \"#000\" hillshade.tif\n"\
            "gdaldem color-relief  depth.vrt ramp.txt color.tif\n"\
            "convert color.tif "\
            "-compose soft-light hillshade.tif -composite "\
            " -compose multiply slope_render.tif -composite "\
            " relief.tif\n");
    fchmod(fileno(FP5),0777);
    /*fprintf(FP5,"#!/bin/bash\n%s/rangeimg  mesh-diced/vis-total.ply mesh-diced/totalbbox.txt --size %d %d -calib %s\n",
            basepath.c_str(),
            imageWidth,
            imageHeight,
            stereo_calib_file_name.c_str()
            );*/
    fclose(FP4);
    vector<std::string> precmd;
    std::ostringstream p1;
    if(hw_image){
        p1 <<basepath << "/createSem";
        precmd.push_back(p1.str());
    }
    string mosaiccmd[4]={"mosaic.py","depthmosaic.py"};
    string mosiacnameCmd[4]={"Mosaic","Depth"};

    for(int z=0; z<NUM_MOSAIC_FILES; z++){

        vector<std::string> postcmdv;


        std::ostringstream p;

        if(z == MOSAIC){
            p<< "cd " << cwd <<";sh createmosaic.sh";
        }else if(z== DEPTHMOSAIC){
            p<< "cd " << cwd <<";sh createmosaicdepth.sh";
        }

        postcmdv.push_back(p.str());

        shellcm.write_generic(mosaiccmd[z],mosaiccmmds_fn[z],mosiacnameCmd[z],&(precmd),&(postcmdv),num_threads, "");
    }
    /*FILE *dBFP=fopen("diced-bounds.txt","w");

    fprintf(dBFP,"%.16f %.16f %.16f %.16f %.16f %.16f %d total\n",totalbb_unrot.xMin(),totalbb_unrot.xMax(),totalbb_unrot.yMin(),totalbb_unrot.yMax(),totalbb_unrot.zMin(),
            totalbb_unrot.zMax(),(int)vrip_cells.size());
    for(int i=0; i <(int)vrip_cells.size(); i++){
        if(vrip_cells[i].poses.size() == 0)
            continue;
        char tmpt[1024];
        sprintf(tmpt,"%s/vis-clipped-diced-%08d-lod%d.ply ",diced_dir,i,vpblod);
        fprintf(dBFP,"%.16f %.16f %.16f %.16f %.16f %.16f %d %s\n",vrip_cells[i].bounds.bbox.xMin(),vrip_cells[i].bounds.bbox.xMax(),vrip_cells[i].bounds.bbox.yMin(),vrip_cells[i].bounds.bbox.yMax(),vrip_cells[i].bounds.bbox.zMin(),
                vrip_cells[i].bounds.bbox.zMax(),i,tmpt);
    }
    fclose(dBFP);

    */
    FILE *dBFP=fopen("diced-bounds.txt","w");
    int totalVolCells=0;
    foreach_vol(cur,vol){
        totalVolCells++;
    }
    fprintf(dBFP,"%.16f %.16f %.16f %.16f %.16f %.16f %d total\n",totalbb_unrot.xMin(),totalbb_unrot.xMax(),totalbb_unrot.yMin(),totalbb_unrot.yMax(),totalbb_unrot.zMin(),
            totalbb_unrot.zMax(),totalVolCells);
    int cnt=0;
    foreach_vol(cur,vol){
        if(cur->poses.size() == 0)
            continue;
        char tmpt[1024];
        sprintf(tmpt,"%s/clean_%04d%04d%04d.ply",aggdir,cur->volIdx[0],cur->volIdx[1],cur->volIdx[2]);
        fprintf(dBFP,"%.16f %.16f %.16f %.16f %.16f %.16f %d %s\n",cur->bounds.bbox.xMin(),cur->bounds.bbox.xMax(),cur->bounds.bbox.yMin(),cur->bounds.bbox.yMax(),cur->bounds.bbox.zMin(),
                cur->bounds.bbox.zMax(),cnt++,tmpt);
    }
    fclose(dBFP);
    fclose(FP5);
    if(!externalMode || newmvs){
        double margin=vrip_res*10;
        string rangeimgcmds_fn[]={(string(diced_dir)+"/rangeimgcmds").c_str(),(string(diced_dir)+"/globalimgcmds").c_str()};
        string rangecmd[]={"rangeimg.py","globaldepth.py"};
        FILE *rangeimgcmds_fp[2];
        for(int t=0; t< 2; t++)
            rangeimgcmds_fp[t]  =fopen(rangeimgcmds_fn[t].c_str(),"w");
        std::set<string> usedName;
        foreach_vol(cur,vol){
            if(cur->poses.size() == 0)
                continue;

            char tmpp[1024];
            sprintf(tmpp,"%s/range-clean_%04d%04d%04d.txt",aggdir,cur->volIdx[0],cur->volIdx[1],cur->volIdx[2]);
            FILE *rfp=fopen(tmpp,"w");
            string mesh_list;
            for(int t=0; t<2; t++){
                fprintf(rangeimgcmds_fp[t],"%s/rangeimg ",basepath.c_str());
                osg::BoundingBox expanded(cur->bounds.bbox._min[0]-margin,cur->bounds.bbox._min[1]-margin,cur->bounds.bbox._min[2]-margin,
                                          cur->bounds.bbox._max[0]+margin,cur->bounds.bbox._max[1]+margin,cur->bounds.bbox._max[2]+margin);
                foreach_vol(cur2,vol){
                    if(expanded.intersects(cur2->bounds.bbox)){
                        char tmpt[1024];
                        sprintf(tmpt,"%s/clean_%04d%04d%04d.ply",aggdir,cur->volIdx[0],cur->volIdx[1],cur->volIdx[2]);
                        mesh_list+=tmpt;
                    }

                }
                const char *globalstr= (t==1) ? "--global" : "";
                fprintf(rangeimgcmds_fp[t],"%s --bbox %s/range-clean_%04d%04d%04d.txt --size %d %d --calib %s %s\n",
                        mesh_list.c_str(),
                        aggdir,
                        cur->volIdx[0],cur->volIdx[1],cur->volIdx[2],
                        imageWidth,
                        imageHeight,
                        stereo_calib_file_name.c_str(),
                        globalstr
                        );
            }
            int ct=0;
            for(int k=0; k<(int)cur->poses.size(); k++){
                if(usedName.count(cur->poses[k]->left_name) >0 )
                    continue;
                const Stereo_Pose_Data *name=cur->poses[k];
                usedName.insert(name->left_name);
                fprintf(rfp,"%d %s ",
                        ct++,name->left_name.c_str());
                save_bbox_frame(name->bbox,rfp);
                osg::Matrix texmat=osgTranspose(name->mat);
                texmat=osg::Matrix::inverse(texmat);
                for(int k=0; k < 4; k++)
                    for(int n=0; n < 4; n++)
                        fprintf(rfp," %lf",texmat(k,n));
                fprintf(rfp,"\n");
            }
            fclose(rfp);
        }
        const char *namestr[2]={"Range","Global Range"};
        for(int t=0; t< 2; t++){
            fclose(rangeimgcmds_fp[t]);
            shellcm.write_generic(rangecmd[t],rangeimgcmds_fn[t],namestr[t],NULL,NULL,num_threads);
        }
        if(!no_rangeimg)
            sysres=system("python rangeimg.py");

    }




    string vartexcmd="vartex.py";

    string texcmd[4]={"tex.py","depth.py","flat.py","render.py"};
    string nameCmd[4]={"Remap","Depth","RenderFlat","Render3D"};

    for(int z=0; z<2/*Just first two not render ones which need data from remap step*/; z++){

        vector<std::string> postcmdv;


       /* std::ostringstream p;

        if(z == REMAP_FLAT_SIZE || z == FLAT){
            p<< "cd " << cwd <<";sh createmosaic.sh";
        }else if(z== DEPTH){
            p<< "cd " << cwd <<";sh createmosaicdepth.sh";
        }

        postcmdv.push_back(p.str());
*/
        shellcm.write_generic(texcmd[z],texcmds_fn[z],nameCmd[z],&(precmd),&(postcmdv),num_threads);
    }
    vector<std::string> varpostcmdv;
    std::ostringstream pvar;

    pvar<< "cd " << cwd <<";sh createmosaicvar.sh";

    varpostcmdv.push_back(pvar.str());

    string calcTexCmd="tc_calc.py";
    shellcm.write_generic(calcTexCmd,calcTexFn,"TC",NULL,NULL,num_threads);
    if(!no_tc){
        sysres=system(string("python "+calcTexCmd).c_str());
        if(sysres <0){
            fprintf(stderr,"Failed to run %s\n",calcTexCmd.c_str());
            exit(-1);
        }
    }
    shellcm.write_generic(vartexcmd,vartexcmds_fn,"Var Tex",NULL,&(varpostcmdv),std::max(num_threads/2,1));
    long int totalSide=0;
    int POTatlasSize=0;
    std::vector<std::pair<int,int> > remapSizes;
    if(reparamTex){
        if(!no_run_remap)
            sysres=system(string("python "+texcmd[REMAP]).c_str());

        for(int i=0; i <(int)cells.size(); i++){
            osg::Vec2 minTC,maxTC;
            int origX,origY;
            char tmp11[8192];
            int sizeX,sizeY;
            sprintf(tmp11,"%s/image_r%04d_c%04d_rs%04d_cs%04d-remap.size.txt",diced_dir,cells[i].row,cells[i].col,_tileRows,_tileColumns);
            FILE *fp=fopen(tmp11,"r");
            if(!fp){
                fprintf(stderr,"Can't open %s\n",tmp11);
                exit(-1);
            }
            int ret=fscanf(fp,"%f %f %f %f %d %d %d %d\n",&(minTC.x()),&(minTC.y()),&(maxTC.x()),&(maxTC.y()),&origX,&origY,&sizeX,&sizeY);
            if(ret != 8){
                fprintf(stderr,"Can't parse %s\n",tmp11);
                exit(-1);
            }
            fclose(fp);
            int maxSide=std::max(sizeX,sizeY);
            totalSide+=(maxSide*maxSide);
            remapSizes.push_back(make_pair<int,int>(sizeX,sizeY));
        }
        int sqrtSize=    ceil(sqrt(totalSide));
        if(sqrtSize < 8){
            fprintf(stderr,"Atlas size %d invalid no texures generated\n",sqrtSize);
            exit(-1);
        }
        // printf("Un adjusted size %d\n",sqrtSize);
        int ajustedGLImageSize=(int)sqrtSize+((sqrtSize/VTtileSize)*2*tileBorder);
        int maxVTSize=(int)pow(2,17); //~128k max size
        POTatlasSize=min(osg::Image::computeNearestPowerOfTwo(ajustedGLImageSize),(int)maxVTSize);
      /*  double marginAtlas=1.3;
        float closeFitScale=(ajustedGLImageSize > POTatlasSize) ? POTatlasSize/(double)(ajustedGLImageSize*marginAtlas) : 1.0;*/
        float closeFitScale=1.0;
        if(!getScaleFactorForAtlasFit(closeFitScale,remapSizes,POTatlasSize, VTtileSize, tileBorder)){
                   fprintf(stderr,"Can't get atlas fit for remapping bailing!\n");
                   exit(-1);
        }
        printf("Adjusted POT %d using %d scale %f\n",ajustedGLImageSize,POTatlasSize,closeFitScale);
        FILE *fpscale=fopen((string(diced_dir)+string("/globalscale.txt")).c_str(),"w");
        fprintf(fpscale,"%f\n",closeFitScale);
        fclose(fpscale);
        char atlasSizeStr[1024];
        sprintf(atlasSizeStr," -potsize %d  ",POTatlasSize);
        string tcmd =basepath+string("/atlasmesh -mat viewproj.mat -cells image_areas.txt ")+atlasSizeStr;// + string(tmpsize);
        char tmp100[8096];
        sprintf(tmp100, " -F -outfile %s/unclean-tex-total.ply ; %s/vcgapps/bin/cleanTexMesh %s/unclean-tex-total.ply --normcolor -out %s/tex-total.ply",diced_dir,basepath.c_str(),diced_dir,diced_dir);

        std::string extraCheckCmd=  createFileCheckPython(tcmd,cwdmeshdiced,cfiles,string(tmp100),4);
        for(int z=2; z<NUM_TEX_FILES; z++){

            vector<std::string> postcmdv;
            shellcm.write_generic(texcmd[z],texcmds_fn[z],nameCmd[z],&(precmd),&(postcmdv),num_threads,extraCheckCmd);
        }
        if(!no_tex){
            sysres=system(string("python "+texcmd[REMAP_FLAT_SIZE]).c_str());
            if(sysres <0){
                fprintf(stderr,"Failed to run %s\n",texcmd[REMAP_FLAT_SIZE].c_str());
                exit(-1);
            }
        }
    }/*else{
        if(!no_tex)
            sysres=system(string("python "+texcmd[FLAT]).c_str());
    }*/
    if(!no_mosaic)
        sysres=system(string("python "+mosaiccmd[MOSAIC]).c_str());

    if(var_tex)
        sysres=system("python vartex.py");
    //if(useVirtTex)
    {
        int sizeForLevel=POTatlasSize;
        int sizeLevel=sizeForLevel;

        int tileSize=256;
        int border=1;

        //int maxLevels=    (int)ceil(std::log((double)sizeForLevel/tileSize) / std::log(2.0));

        string vttexcmds_fn=string(diced_dir)+"/vttexcmds";
        string vttex="vttex.py";

        FILE *vttexcmds_fp=fopen(vttexcmds_fn.c_str(),"w");
       // fchmod(fileno(vttexcmds_fp),0777);
        char flatflag[1024];
        if(reparamTex)
            sprintf(flatflag," ");
        else
            sprintf(flatflag,"-flatatlas ");
       // fprintf(vttexcmds_fp,"#!/bin/bash\n");
        int adjustedTileSize=tileSize-(border *2);
        for(int level=0; sizeLevel>=adjustedTileSize; level++,sizeLevel/=2 ){

            int numXtiles=(sizeLevel/adjustedTileSize);
            if(numXtiles <= 4){
                fprintf(vttexcmds_fp,"cd %s;",cwd);
                fprintf(vttexcmds_fp,"%s/vipsVTAtlas -mat %s -cells %s %s -level %d -potsize %d \n",basepath.c_str(),"viewproj.mat","image_areas.txt",flatflag,level,POTatlasSize);


            }else{
                for(int x=0; x<numXtiles; x++){
                    fprintf(vttexcmds_fp,"cd %s;",cwd);
                    fprintf(vttexcmds_fp,"%s/vipsVTAtlas -mat %s -cells %s %s -level %d -row %d  -potsize %d \n",basepath.c_str(),"viewproj.mat","image_areas.txt",flatflag,level,x,POTatlasSize);
                }
            }
        }
        /*   for(int i=0; i <(int)vrip_cells.size(); i++){
            if(vrip_cells[i].poses.size() == 0)
                continue;
            fprintf(vttexcmds_fp,"%s/singleImageTex %s/clipped-diced-%08d.ply --outfile %s/clipped-diced-%08d-lod%d.ply\n",
                    basepath.c_str(),
                    diced_dir,
                    i,diced_dir,i,vpblod);
        }*/
        fclose(vttexcmds_fp);
        shellcm.write_generic(vttex,vttexcmds_fn,"VT Tex",NULL,NULL,num_threads);
   //     fclose(vttexcmds_fp);
        if(useVirtTex && !no_vttex){
            sysres=system(("python "+vttex).c_str());
            if(sysres <0){
                fprintf(stderr,"Failed to run %s\n",vttex.c_str());
                exit(-1);
            }
        }
    }

    FILE *ipadViewerFP=fopen("createtabletdata.sh","w");
    if(!ipadViewerFP){
        fprintf(stderr,"Can't open create createtabletdata\n");
        exit(-1);
    }
    // int smallPOTX=osg::Image::computeNearestPowerOfTwo(totalX,0.0);
    // int smallPOTY=osg::Image::computeNearestPowerOfTwo(totalY,0.0);

    //int totalXborder=(int)smallPOTX-((smallPOTX/VTtileSize)*2*tileBorder);
    // int totalYborder=(int)smallPOTY-((smallPOTX/VTtileSize)*2*tileBorder);
    double maxResIpad=pow(2,15);//32k
    int numOctrees=3;
    int maxFaceSizeIpad=((0xffff-1)-10)*(numOctrees-0.5);
    double scaleFactor= (POTatlasSize > 16384 ) ? 0.5 : 1.0;
    fprintf(ipadViewerFP,"#!/bin/bash\n");
    fprintf(ipadViewerFP,"mkdir ipad\n");
    fprintf(ipadViewerFP,"cd %s;%s/vcgapps/bin/texturedDecimator %s/tex-total.ply %s/ipad.ply %d -Oy -V -P ;",
            cwd,
            basepath.c_str(),
            diced_dir,
            diced_dir,maxFaceSizeIpad);

    fprintf(ipadViewerFP,"%s/vcgapps/bin/splitForTablet %s/ipad.ply -uipad/octree -s%d;",
            basepath.c_str(),
            diced_dir,
            (0xffff-1));

    std::ostringstream p2;
    //  p2 << basepath << "/singleImageTex " << diced_dir<<"/ipad.ply --outfile "<<diced_dir<<"/ipadtex.ply "<< "--size " << totalXborder << " "<<totalYborder;
    // p2      <<" --extra ipad/octree";
    fprintf(ipadViewerFP,"cd ipad\n");
    fprintf(ipadViewerFP,"n=`cat octree.txt`\n");
    fprintf(ipadViewerFP,"nm=$[$n-1]\n");
    fprintf(ipadViewerFP,"for i in `seq 0 $nm`;do\n");
    fprintf(ipadViewerFP,"\tfilesuf=`printf \"%%04d\" $i`\n");
    fprintf(ipadViewerFP,"\t%s/generateOctreeFromObj.py -o=vtex-$filesuf.octree octree-$filesuf.obj\n",basepath.c_str());
    fprintf(ipadViewerFP,"done\n");
    fprintf(ipadViewerFP,"cd ..;%s/vipsVTAtlas -mat %s -cells %s -maxRes %f -scale %f -dir %s -potsize %d\n",basepath.c_str(),"viewproj.mat","image_areas.txt",maxResIpad,scaleFactor,"ipad",POTatlasSize);

    //fprintf(ipadViewerFP,"(gdalwarp -overwrite -ts %d %d mosaic/mosaic.vrt vttex.tif; ",totalXborder,totalYborder);
    /* std::ostringstream p4;

    p4 << basepath << "/generateVirtualTextureTiles.py " << "-f=jpg  -b="<<tileBorder<<" vttex.tif ) &\nwait\necho 'Done'\n";

    //postcmdv.push_back(p2.str());

    fprintf(ipadViewerFP,"%s",p4.str().c_str());*/
    fchmod(fileno(ipadViewerFP),0777);

    fclose(ipadViewerFP);

    FILE *metaFP=fopen("ipadtar.sh","w");

    if(!metaFP){
        fprintf(stderr,"Can't open create ipadtar");
        exit(-1);
    }
    fprintf(metaFP,"#!/bin/bash\n");
    fprintf(metaFP,"EXPECTED_ARGS=1\nE_BADARGS=65\n");
    fprintf(metaFP,"if [ $# -ne $EXPECTED_ARGS ]\nthen\n\techo \"Usage: `basename $0` {basename}\"\nexit $E_BADARGS\nfi\n");
    fprintf(metaFP,"cd ipad\n");
    fprintf(metaFP,"mkdir $1\n");
    fprintf(metaFP,"ln -sf $PWD/vtex $PWD/$1/\n");
    fprintf(metaFP,"n=`cat octree.txt`\n");
    fprintf(metaFP,"nm=$[$n-1]\n");
     fprintf(metaFP,"for i in `seq 0 $nm`;do\n");
    fprintf(metaFP,"\tfilesuf=`printf \"%%04d\" $i`\n");
    fprintf(metaFP,"\tln -sf $PWD/vtex-$filesuf.octree $PWD/$1/m-$filesuf.octree\n");
    fprintf(metaFP,"done\n");
    fprintf(metaFP,"ln -sf $PWD/octree.txt $PWD/$1/cnt\n");
    fprintf(metaFP,"gdalwarp ../mosaic/mosaic.vrt -ts 256 256 out.tif\n");
    fprintf(metaFP,"convert $PWD/out.tif $1/m.jpg\n");
    fprintf(metaFP,"rm -f $1/m.xml\n");
    fprintf(metaFP,"tar cvfh $1.tar $1\n");
    fprintf(metaFP,"bash %s/getmeta.sh $1 > $1/m.xml\n",basepath.c_str());
    fprintf(metaFP,"tar rvf $1.tar $1/m.xml\n");
    fprintf(metaFP,"echo -n $1 > tarname\n");
    fchmod(fileno(metaFP),0777);

    fclose(metaFP);

    FILE *uploadFP=fopen("uploadmesh.sh","w");

    if(!uploadFP){
        fprintf(stderr,"Can't open create uploadmesh");
        exit(-1);
    }
    fprintf(uploadFP,"#!/bin/bash\n");
    fprintf(uploadFP,"EXPECTED_ARGS=1\nE_BADARGS=65\n");
    fprintf(uploadFP,"if [ $# -ne $EXPECTED_ARGS ]\nthen\n\techo \"Usage: `basename $0` {basename}\"\nexit $E_BADARGS\nfi\n");
    fprintf(uploadFP,"cd ipad\n");
    fprintf(uploadFP,"if [ ! -f tarname ]; then\n");
    fprintf(uploadFP,"\techo \"tarname file is missing\" \n\texit $E_BADARGS\n");
    fprintf(uploadFP,"fi\n");
    fprintf(uploadFP,"TARNAME=`cat tarname`\n");
    fprintf(uploadFP,"sed -i'' \"s/__REPLACEFOLDERNAME__/$1/g\" $TARNAME/m.xml\n");
    fprintf(uploadFP,"tar rvf $TARNAME.tar $TARNAME/m.xml\n");
    fprintf(uploadFP,"scp $TARNAME.tar mattjr@aguacate.acfr.usyd.edu.au:benthos/$1/\n");
    fprintf(uploadFP,"scp %s/updatemodelxml.sh mattjr@aguacate.acfr.usyd.edu.au:benthos/\n",basepath.c_str());
    fprintf(uploadFP,"scp %s/create_main_feed.sh mattjr@aguacate.acfr.usyd.edu.au:benthos/\n",basepath.c_str());
    fprintf(uploadFP,"ssh mattjr@aguacate.acfr.usyd.edu.au \"cd benthos;bash updatemodelxml.sh\"\n");
    fprintf(uploadFP,"ssh mattjr@aguacate.acfr.usyd.edu.au \"cd benthos;bash create_main_feed.sh\"\n");

    fchmod(fileno(uploadFP),0777);

    fclose(uploadFP);


    string simpcmds_fn=string(diced_dir)+"/simpcmds";

    FILE *simpcmds_fp=fopen(simpcmds_fn.c_str(),"w");
    string app;
    /* if(useReimage)
        app="tridecimator";
    else*/
    app="texturedDecimator";
    //    app="tridecimator";
    std::vector<std::vector<string> > datalist_lod;
    vector<string> mergeandcleanCmdsSimp;
#define SINGLE_MESH_TEX 1
#ifdef SINGLE_MESH_TEX


    {
        FILE *tp=diced_fopen("/bbox-total.ply.txt","w");
        for(int i=0; i< (int)tasks.size(); i++){
            const Stereo_Pose_Data *pose=(&tasks[i]);
            if(pose && pose->valid){
                fprintf(tp, "%d %s " ,pose->id,pose->file_name.size() ==0 ? pose->left_name.c_str(): pose->file_name.c_str());
                save_bbox_frame(pose->bbox,tp);
                for(int k=0; k < 4; k++)
                    for(int n=0; n < 4; n++)
                        fprintf(tp," %lf",pose->mat(k,n));
                fprintf(tp,"\n");
            }

        }
        fclose(tp);
        char tmp[1024];
        //if(useVirtTex)
        //   fprintf(simpcmds_fp,"cd %s/%s;cp totaltex.ply total-lod%d.ply;",cwd,diced_dir,vpblod);
        // else
        //   fprintf(simpcmds_fp,"cd %s/%s;cp tex-total.ply total-lod%d.ply;",cwd,diced_dir,vpblod);



        sprintf(tmp,"%s/total-lod%d.ply",diced_dir,vpblod);//std::min(lod,2)
        std::vector<string> level;
        string clean_str= 1 ? "-P" : "";
        string boundry_preserve = isSparse ? "" : "-By";
        fprintf(simpcmds_fp,"cd %s/%s;%s/vcgapps/bin/%s tex-total.ply total-lod%d.ply %d %s -Oy %s;",
                cwd,
                diced_dir,
                basepath.c_str(),
                app.c_str(),
                vpblod,0,clean_str.c_str(),boundry_preserve.c_str());
        for(int j=vpblod-1; j >= 0; j--){

            fprintf(simpcmds_fp,"cd %s/%s;%s/vcgapps/bin/%s total-lod%d.ply total-lod%d.ply %d %s -Oy %s;",
                    cwd,
                    diced_dir,
                    basepath.c_str(),
                    app.c_str(),
                    j+1,j, sizeStepTotal[j],clean_str.c_str(),boundry_preserve.c_str());

        }
        for(int lod=0; lod < vpblod; ){
            std::vector<string> level;
            // if(datalist_lod.size() >3)
            //for(int lod=0; lod <= vpblod; lod ++){

            char tmp[1024];
            sprintf(tmp,"%s/total-lod%d.ive",diced_dir,lod);//std::min(lod,2)
            level.push_back(tmp);
            lod ++;

            datalist_lod.push_back(level);

        }
    }
#else
    for(int i=0; i <(int)vrip_cells.size(); i++){
        if(vrip_cells[i].poses.size() == 0)
            continue;
        for(int j=vpblod; j >0; j--){
            fprintf(simpcmds_fp,"cd %s/mesh-diced;%s/vcgapps/bin/%s clipped-diced-%08d-lod%d.ply clipped-diced-%08d-lod%d.ply %d -By -P;",
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

            sprintf(tmp8,"cd %s/mesh-diced;time %s/vcgapps/bin/%s %s total-lod%d.ply %d -By -P;",
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
        sysres=system("python simp.py");

    //  char szProj4[4096];
    //  char wkt[4096];
    // sprintf(wkt,"PROJCS[\"unnamed\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],TOWGS84[0,0,0,0,0,0,0],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9108\"]],AUTHORITY[\"EPSG\",\"4326\"]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"latitude_of_origin\",%.12f],PARAMETER[\"central_meridian\",%.12f],PARAMETER[\"scale_factor\",1],PARAMETER[\"false_easting\",0],PARAMETER[\"false_northing\",0],UNIT[\"Meter\",1]]",
    //       latOrigin,longOrigin);
    //sprintf( szProj4,
    //       "\"+proj=tmerc +lat_0=%.24f +lon_0=%.24f +k=%.12f +x_0=%.12f +y_0=%.12f +datum=WGS84 +ellps=WGS84 +units=m +no_defs\"",latOrigin,longOrigin,1.0,0.0,0.0);

    string src_proj4=getProj4StringForAUVFrame(latOrigin,longOrigin);
    if(runIpad){
        sysres=system("bash createtabletdata.sh");
    }

    if(!novpb) {
        if(!mgc && !useVirtTex){
            mgc = new MyGraphicsContext();
            if(!mgc->valid()){
                no_hw_context=true;
                printf("Forcing no hw compression due to lack of graphics card context\n");
            }
        }else{
            no_hw_context=true;
        }
        CameraCalib *cam_calib;
        if(newmvs)
            cam_calib=new CameraCalib();
        else
            cam_calib=&(calib->camera_calibs[0]);

        doQuadTreeVPB(basepath,datalist_lod,bounds,*cam_calib,cachedtexdir,POTatlasSize,useTextureArray,useReimage,useVirtTex,totalbb_unrot,src_proj4,dst_proj4_coord_system,sparseRatio,no_hw_context,no_atlas);
    }

    char zipstr[1024];
    sprintf(zipstr,"7z a -r -sfx7z.sfx mesharchive.7z.exe mesh/ -m0=lzma2 -mmt%d -mx9",num_threads);
    FILE *zfp=fopen("zip.sh","w");
    fprintf(zfp,"#!/bin/bash\n%s\n",zipstr);
    fclose(zfp);





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




}








