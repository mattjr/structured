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
#include "configFile.h"
#include "ShellCmd.h"
#include "stereo_cells.hpp"
#include <limits>
#include "VPBInterface.hpp"
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
static bool hw_image=false;
static bool blending=true;
void formatBar(string name,osg::Timer_t startTick,unsigned int count,unsigned int totalCount);
string wkt_coord_system;
using namespace std;
static bool useOrthoTex=true;
bool reimage=true;
static double start_time = 0.0;
static bool apply_aug=false;
static double stop_time = numeric_limits<double>::max();
static int vpblod_override=0;
static bool compositeMission=false;
//
// Command-line arguments
//
static bool untex=true;
using namespace std;
static FILE *conf_ply_file;
static string background_mb;
static string contents_file_name;
static string dir_name;
int targetScreenFaces;
static bool novpb=false;
static bool use_cached=true;
static bool further_clean=false;
static bool pause_after_each_frame = false;
static double image_scale = 1.0;
static int max_feature_count;
static bool exportForStaticRender=true;
static bool useTextureArray=true;
static double longOrigin,latOrigin;
static int _tileRows;
static int _tileColumns;
static int targetFaces=40000;
static bool storeTexMesh=false;
static bool do_novelty=false;
static double dense_scale;
static bool no_tex=false;
static bool no_vttex=false;
static bool use_debug_shader=false;
static vector<string> mb_xyz_files;
static int desired_area=250.0;
static bool have_max_frame_count = false;
static unsigned int max_frame_count=INT_MAX;
static bool display_debug_images = true;

static bool run_stereo=true;
static double max_alt_cutoff=10.0;
//static bool use_ncc = false;
static double vrip_ramp;
static int num_skip=0;

static int gpunum=0;
static string stereo_calib_file_name;
static bool no_simp=true;
static bool no_split=false;
static ofstream file_name_list;
static string base_dir;
static bool no_depth=false;
static double feature_depth_guess = AUV_NO_Z_GUESS;
static int num_threads=1;
static FILE *fpp,*fpp2;
static bool useVirtTex=false;
static bool useReimage=true;


static bool no_vrip=false;
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
const char *uname="mesh";
const char *dicedir="mesh-diced";
const char *mosdir="mosaic";
const char *aggdir="mesh-agg";
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
static double feat_quality_level;

MyGraphicsContext *mgc=NULL;


typedef struct _picture_cell{
    int row;
    int col;
    osg::BoundingBox bbox;
    osg::BoundingBox bboxMargin;

    osg::BoundingBox bboxUnRot;
    osg::BoundingBox bboxMarginUnRot;
    osg::Matrixd m;
    std::string name;
    std::vector<int> images;
    std::vector<int> imagesMargin;

}picture_cell;
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

    argp.read("--split-area",desired_area);
    argp.read("--stereo-calib",stereo_calib_file_name);
    argp.read("--poses",contents_file_name );
    apply_aug =argp.read("--apply_aug");
    argp.read("--gpu",gpunum);

    cmvs=argp.read("--mvs");
    useAtlas=argp.read("--atlas");
    novpb=argp.read("--novpb");
    storeTexMesh=argp.read("--storetex");

    if(argp.read("--debug-shader")){
        use_debug_shader=true;
        useAtlas=true;
        storeTexMesh=true;
    }
    if(argp.read("--noarray"))
        useTextureArray=false;

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

    recon_config_file->get_value( "SD_SCALE", dense_scale,0.5);
    recon_config_file->get_value("TEX_IMG_PER_CELL",tex_img_per_cell,80);
    recon_config_file->get_value("VRIP_IMG_PER_CELL",vrip_img_per_cell,1000);
    recon_config_file->get_value("FEAT_MIN_DIST",min_feat_dist,3.0);
    recon_config_file->get_value("FEAT_QUALITY_LEVEL",feat_quality_level,0.0001);

    recon_config_file->get_value("MAX_FEAT_COUNT",max_feature_count,5000);
    recon_config_file->get_value("VRIP_RAMP",vrip_ramp,500.0);
    recon_config_file->get_value("EDGE_THRESH",edgethresh,0.5);


    recon_config_file->get_value("VRIP_RES",vrip_res,0.033);
    recon_config_file->get_value("IMAGE_SPLIT_COL",_tileColumns,-1);
    recon_config_file->get_value("IMAGE_SPLIT_ROW",_tileRows,-1);
    recon_config_file->get_value( "SRC_TEX_SIZE", lodTexSize[0],
                                  512);
    recon_config_file->get_value( "WKT_DEST_COORD",wkt_coord_system,
                                  "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],TOWGS84[0,0,0,0,0,0,0],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9108\"]],AXIS[\"Lat\",NORTH],AXIS[\"Long\",EAST],AUTHORITY[\"EPSG\",\"4326\"]]");
    recon_config_file->get_value( "TARGET_SCREEN_FACES", targetScreenFaces,
                                  150000);

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
    if(argp.read("--vt")){
        useReimage=false;
        useVirtTex=true;
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
    argp.read( "--res",vrip_res );

    if(argp.read( "-n",max_frame_count))
        have_max_frame_count = true;
    //    max_frame_count=20;
    //  have_max_frame_count = true;




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







    argp.read("--vrip-ramp",vrip_ramp );
    no_simp=argp.read( "--nosimp" );

    no_split=argp.read( "--nosplit" );

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
    if(argp.read("--notex"))
        no_tex=true;

    if(argp.read("--novttex"))
        no_vttex=true;





    if(use_dense_stereo)
        sprintf(cachedmeshdir,"cache-mesh-dense/");
    else
        sprintf(cachedmeshdir,"cache-mesh-feat/");

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

osg::Matrix  osgTranspose( const osg::Matrix& src )
{
    osg::Matrix dest;
    for( int i = 0; i < 4; i++ )
        for( int j = 0; j < 4; j++ )
            dest(i,j) = src(j,i);
    return dest;
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

    int index;

    index = pose.pose_id;
    name.time = pose.pose_time;
    name.pose[POSE_INDEX_X] = pose.pose_est[POSE_INDEX_X];
    name.pose[POSE_INDEX_Y] = pose.pose_est[POSE_INDEX_Y];
    name.pose[POSE_INDEX_Z] = pose.pose_est[POSE_INDEX_Z];
    name.pose[POSE_INDEX_PHI] = pose.pose_est[POSE_INDEX_PHI];
    name.pose[POSE_INDEX_THETA] = pose.pose_est[POSE_INDEX_THETA];
    name.pose[POSE_INDEX_PSI] = pose.pose_est[POSE_INDEX_PSI];
    name.left_name = pose.left_image_name;
    name.right_name = pose.right_image_name;
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
    ifstream mesh(string("mesh-agg/"+name.mesh_name).c_str());
    if(!mesh.good())
        return false;
    string line;
    getline(mesh,line);
    if(line.substr(0,3) != "ply")
        return false;
    while(mesh.good()){
        getline(mesh,line);
        if(line.substr(0,9) == "end_header"){
            cerr << "valid file but no comment in header for bbox\n";
            return false;
        }
        if(line.substr(0,7) == "comment"){
            std::string bboxstr=line.substr(8,line.size());
            istringstream iss(bboxstr);
            osg::Vec3d minB,maxB;
            iss >> minB;
            iss >> maxB;
            name.bbox = osg::BoundingBox(minB,maxB);
            return true;
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




    osgDB::makeDirectory(dicedir);
    chmod(dicedir,   0777);

    osgDB::makeDirectory(uname);

    chmod(uname,   0777);

    osgDB::makeDirectory("mesh-pos");

    chmod("mesh-pos",   0777);

    osgDB::makeDirectory(cachedmeshdir);
    chmod(cachedmeshdir,   0777);
    for(int i=0; i < (int)cachedtexdir.size(); i++){
        osgDB::makeDirectory(cachedtexdir[i].first.c_str());
        chmod(cachedtexdir[i].first.c_str(),   0777);
    }
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
    StereoCalib calib(stereo_calib_file_name);



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


    vector<Stereo_Pose_Data> tasks;
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
    double max_alt=0;
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

        name.id= stereo_pair_count++;
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
        // if(tasks[i].valid)
        if(1){
            const osg::Matrix &mat=tasks[i].mat;
            fprintf(conf_ply_file,
                    "%s/stereo_mesh_gen %s %s %s --mat-1-8 %f %f %f %f %f %f %f %f --mat-8-16 %f %f %f %f %f %f %f %f --edgethresh %f -m %d -z %f\n"
                    ,basepath.c_str(),base_dir.c_str(),tasks[i].left_name.c_str(),tasks[i].right_name.c_str(),
                    mat(0,0),mat(1,0),mat(2,0),mat(3,0),
                    mat(0,1),mat(1,1),mat(2,1),mat(3,1),
                    mat(0,2),mat(1,2),mat(2,2),mat(3,2),mat(0,3),mat(1,3),mat(2,3),mat(3,3),edgethresh,
                    max_feature_count,tasks[i].alt);
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
    char cwd[2048];
    char *dirres;
    dirres=getcwd(cwd,2048);


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


    if(!diced_fp){
        printf("Can't open mesh-diced/diced.txt");
        exit(-1);
    }
    if(!diced_lod_fp){
        printf("Can't open mesh-diced/dicedld.txt");
        exit(-1);
    }

    //const char *simplogdir="/mnt/shared/log-simp";
    const char *pos_simp_log_dir="/mnt/shared/log-possimp";
    //const char *vriplogdir="/mnt/shared/log-vrip";

    float simp_mult=1.0;


    ShellCmd shellcm(basepath.c_str(),simp_mult,pos_simp_log_dir,cwd,aggdir,dicedir,num_threads);
    shellcm.write_setup();
    /* string stereocmd="stereo.py";

        shellcm.write_generic(stereocmd,conf_name,"Stereo");
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
*/

    osg::Timer_t startTick= osg::Timer::instance()->tick();
    int totalTodoCount=tasks.size();
    int progCount=0;
    OpenThreads::Mutex mutex;
    double max_triangulation_len =  max_alt > 0.0 ? max_alt*3 : edgethresh * 20;
#pragma omp parallel num_threads(num_threads)
    if(run_stereo){
        StereoEngine engine(calib,edgethresh,max_triangulation_len,max_feature_count,  min_feat_dist, feat_quality_level,lodTexSize[0],mutex);
        cvSetNumThreads(1);
#pragma omp for
        for(int i=0; i < (int)tasks.size(); i++){
            tasks[i].valid=engine.processPair(base_dir,tasks[i].left_name,
                                              tasks[i].right_name,tasks[i].mat,tasks[i].bbox,tasks[i].alt,hw_image);

#pragma omp atomic
            progCount++;

            formatBar("Stereo",startTick,progCount,totalTodoCount);

        }
    }
    formatBar("Stereo",startTick,totalTodoCount,totalTodoCount);


    if(!run_stereo){
        char tp[2048];
        for(unsigned int i=0; i < tasks.size(); i++){
            sprintf(tp,"mesh-agg/surface-%s.tc.ply",osgDB::getSimpleFileName(osgDB::getNameLessExtension(tasks[i].left_name)).c_str());
            if(getBBoxFromMesh(tasks[i]))
                tasks[i].valid=true;
            else
                tasks[i].valid=false;
            formatBar("Stereo-FCK",startTick,progCount,totalTodoCount);
            progCount++;
        }
        formatBar("Stereo-FCK",startTick,totalTodoCount,totalTodoCount);

    }

    FILE *totalfp=fopen("mesh-diced/totalbbox.txt","w");

    int ct=0;
    for(vector<Stereo_Pose_Data>::iterator itr=tasks.begin(); itr != tasks.end(); itr++){
        const Stereo_Pose_Data &name=(*itr);



        fprintf(fpp,"%d %f %s %s ",
                ct++,name.time,name.left_name.c_str(),name.right_name.c_str());
        fprintf(totalfp,"%d %s ",
                ct,name.left_name.c_str());
        save_bbox_frame(name.bbox,fpp);
        save_bbox_frame(name.bbox,totalfp);

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
    Bounds bounds( tasks );
    std::vector<Cell_Data<Stereo_Pose_Data> > vrip_cells;
    vrip_cells=calc_cells<Stereo_Pose_Data> (tasks,vrip_img_per_cell);

    printf("Split into %d cells for VRIP\n",(int)vrip_cells.size());
    int numberFacesAll=0;

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

        sprintf(redirstr," ");

        fprintf(diced_fp,"clipped-diced-%08d.ply\n",i);
        fprintf(diced_lod_fp,"clipped-diced-%08d-lod0.ply\n",i);
        fprintf(vripcmds_fp,"set BASEDIR=\"%s\"; set OUTDIR=\"mesh-agg/\";set VRIP_HOME=\"$BASEDIR/vrip\";setenv VRIP_DIR \"$VRIP_HOME/src/vrip/\";set path = ($path $VRIP_HOME/bin);cd %s/$OUTDIR;",basepath.c_str(),cwd);
        fprintf(vripcmds_fp,"$BASEDIR/vrip/bin/vripnew auto-%08d.vri ../%s ../%s %f -rampscale %f;$BASEDIR/vrip/bin/vripsurf auto-%08d.vri ../mesh-agg/seg-%08d.ply %s ;",i,vrip_seg_fname,vrip_seg_fname,vrip_res,vrip_ramp,i,i,redirstr);

        fprintf(vripcmds_fp,"$BASEDIR/treeBBClip ../mesh-agg/seg-%08d.ply --bbox %f %f %f %f %f %f -dup --outfile ../mesh-diced/tmp-clipped-diced-%08d.ply;",
                i,
                vrip_cells[i].bounds.bbox.xMin(),
                vrip_cells[i].bounds.bbox.yMin(),
                -FLT_MAX,
                vrip_cells[i].bounds.bbox.xMax(),
                vrip_cells[i].bounds.bbox.yMax(),
                FLT_MAX,
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

        fprintf(cfp,"cp out.ply mesh-diced/vis-total.ply\n");
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
        // mergeandcleanCmds.push_back(shellcm.generateMergeAndCleanCmd(vrip_cells,"tmp-clipped-diced","total",vrip_res));
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
        if(!no_vrip && !cmvs)
            sysres=system("python runvrip.py");
        char tmpfn[8192];
        for(int i=0; i <(int)vrip_cells.size(); i++){
            if(vrip_cells[i].poses.size() == 0)
                continue;
            sprintf(tmpfn,"mesh-diced/tmp-clipped-diced-%08d.ply",i);
            osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(tmpfn);
            osg::Drawable *drawable = NULL;
            if(model.valid())
                drawable = model->asGeode()->getDrawable(0);
            if(!drawable){
                fprintf(stderr,"Failed to load model %s\n",tmpfn);
                exit(-1);
            }
            osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
            numberFacesAll+=geom->getPrimitiveSet(0)->getNumPrimitives();

        }
        osg::BoundingBox totalbb;
        osg::BoundingBox totalbb_unrot;

        osg::BoundingSphere bs;
        osg::Matrix rotM;
        float rx=0,ry=180.0,rz=-90;

        rotM =osg::Matrix::rotate(
                osg::DegreesToRadians( rx ), osg::Vec3( 1, 0, 0 ),
                osg::DegreesToRadians( ry ), osg::Vec3( 0, 1, 0 ),
                osg::DegreesToRadians( rz ), osg::Vec3( 0, 0, 1 ) );


        totalbb_unrot.expandBy(bounds.bbox._min);
        totalbb_unrot.expandBy(bounds.bbox._max);
        totalbb.expandBy(osg::Vec3(totalbb_unrot._min[1],totalbb_unrot._min[0],-totalbb_unrot._min[2]));
        totalbb.expandBy(osg::Vec3(totalbb_unrot._max[1],totalbb_unrot._max[0],-totalbb_unrot._max[2]));

        cout << totalbb_unrot._min<< " " << totalbb_unrot._max<<endl;
        cout << totalbb._min<< " " << totalbb._max<<endl;
        /* osg::BoundingBox tmp=totalbb;
        tmp._min[2]= tmp._min[1];
        tmp._max[2]= tmp._max[1];
*/

        bs.expandBy(totalbb);
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
        double rangeX=totalbb.xMax()-totalbb.xMin();
        double rangeY=totalbb.yMax()-totalbb.yMin();
        int minTexSize=8192;
        double largerAxis = std::max(rangeX , rangeY);
        double largerAxisPixels=largerAxis * ((1.0/mmperpixel) *1000.0);
        int adjustedSize=osg::Image::computeNearestPowerOfTwo(largerAxisPixels,1.0);
        adjustedSize=std::max(adjustedSize,minTexSize);
        printf("mm per pixel: %.1f Larger Axis %.2f m POT Image Size %dx%d\n",mmperpixel,
               largerAxis,adjustedSize,adjustedSize);
        if(_tileRows <0 || _tileColumns<0){
            int validCount=0;
            for(int i=0; i< (int)tasks.size(); i++)
                if(tasks[i].valid)
                    validCount++;
            printf("Auto computing tile and column splits %d valid images...\n",validCount);
            int numCells=(int)max(ceil(sqrt(ceil(validCount / tex_img_per_cell))),1.0);
            _tileRows=numCells;
            _tileColumns=numCells;
        }


        int split= std::max(std::max(_tileRows,_tileColumns),1);
        if(reimageSize.x() < 0.0)
            reimageSize = osg::Vec2(osg::Image::computeNearestPowerOfTwo(adjustedSize / split,1.0),osg::Image::computeNearestPowerOfTwo(adjustedSize / split,1.0));

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
        int vpblod=1;
        int faceTmp=numberFacesAll;
        do{
            faceTmp /= pow(4.0,vpblod++);
        }while(faceTmp > targetFaces);
        std::cout << "Target LOD height is : " << vpblod <<std::endl;

        osg::Vec3d eye(totalbb.center()+osg::Vec3(0,0,3.5*totalbb.radius()));
        double xrange=totalbb.xMax()-totalbb.xMin();
        double yrange=totalbb.yMax()-totalbb.yMin();
        double largerSide=std::max(xrange,yrange);
        osg::Matrixd matrix;
        matrix.makeTranslate( eye );
        osg::Matrixd view=osg::Matrix::inverse(matrix);
        osg::Matrixd proj= osg::Matrixd::ortho2D(-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0));
        /* proj= osg::Matrixd::ortho2D(-bs.radius(),bs.radius(),-bs.radius(),bs.radius());
 printf("%f %f %f %f AAAA\n",-bs.radius(),bs.radius(),-bs.radius(),bs.radius());
 printf("%f %f %f %f AAAA\n",-(largerSide/2.0),(largerSide/2.0),-(largerSide/2.0),(largerSide/2.0));
*/

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
        cout << "Assign splits...\n";
        //#pragma omp parallel num_threads(num_threads)
        {
            //#pragma omp parallel for shared(_tileRows, _tileColumns)
            for(int row=0; row< _tileRows; row++){
                for(int col=0; col<_tileColumns; col++){
                    osg::Matrix offsetMatrix=   osg::Matrix::scale(_tileColumns, _tileRows, 1.0) *osg::Matrix::translate(_tileColumns-1-2*col, _tileRows-1-2*row, 0.0);
                    double left,right,bottom,top,znear,zfar;
                    osg::Matrix m=(view*proj*offsetMatrix);
                    m.getOrtho(left,right,bottom,top,znear,zfar);
                    double margin=vrip_res*10;
                    osg::BoundingBox thisCellBbox(left,bottom,bs.center()[2]-bs.radius(),right,top,bs.center()[2]+bs.radius());
                    osg::BoundingBox thisCellBboxMargin(left-(margin),bottom-(margin),bs.center()[2]-bs.radius(),right+(margin),top+(margin),bs.center()[2]+bs.radius());
                  //  std::cout<< thisCellBbox._max-thisCellBbox._min <<"\n";
                    //  std::cout<< "A"<<thisCellBboxMargin._min << " "<< thisCellBboxMargin._max<<"\n\n";

                    picture_cell cell;
                    cell.bbox=thisCellBbox;
                    cell.bboxMargin=thisCellBboxMargin;
                    cell.bboxUnRot.expandBy(cell.bbox._min*osg::Matrix::inverse(rotM));
                    cell.bboxUnRot.expandBy(cell.bbox._max*osg::Matrix::inverse(rotM));

                    cell.bboxMarginUnRot.expandBy(cell.bboxMargin._min*osg::Matrix::inverse(rotM));
                    cell.bboxMarginUnRot.expandBy(cell.bboxMargin._max*osg::Matrix::inverse(rotM));

                    cell.row=row;
                    cell.col=col;
                    cell.m=m;
                    sprintf(tmp4,"mesh-diced/tex-clipped-diced-r_%04d_c_%04d-lod%d.ive",row,col,vpblod);
                    cell.name=string(tmp4);
                    sprintf(tmp4,"mesh-diced/tex-clipped-diced-r_%04d_c_%04d.mat",row,col);
                    std::fstream _file(tmp4,std::ios::binary|std::ios::out);

                    for(int i=0; i < (int)tasks.size(); i++){
                        if(!tasks[i].valid || !tasks[i].bbox.valid())
                            continue;
                        for(int k=0; k<4; k++)
                            for(int l=0; l<4; l++)
                                _file.write(reinterpret_cast<char*>(&(cell.m(k,l))),sizeof(double));
                        _file.close();
                        osg::Vec3 m1=osg::Vec3(tasks[i].bbox.xMin(),tasks[i].bbox.yMin(),tasks[i].bbox.zMin())*rotM;
                        osg::Vec3 m2=osg::Vec3(tasks[i].bbox.xMax(),tasks[i].bbox.yMax(),tasks[i].bbox.zMax())*rotM;

                        osg::BoundingBox imgBox;
                        imgBox.expandBy(m1);
                        imgBox.expandBy(m2);
                        //  cout << m1 << " "<< m2 << " bounds \n";
                        //        cout <<thisCellBbox._min << " "<< thisCellBbox._max<<" bbox\n";
                        if(thisCellBbox.intersects(imgBox)){
                            cell.images.push_back(i);
                        }
                        if(thisCellBboxMargin.intersects(imgBox)){
                            cell.imagesMargin.push_back(i);
                        }
                    }
                    //#pragma omp critical
                    {
                        cells.push_back(cell);
                    }

                }
            }

        }
        {
            vector<std::string> postcmdv;
            string tcmd =basepath+string("/vrip/bin/plymerge ");
            char tmp100[8096];
            std::vector<string> cfiles;
            // sprintf(tmp100," --invrot %f %f %f ",rx,ry,rz);
            // tcmd+=tmp100;
            string splitcmds_fn="mesh-diced/splitcmds";

            FILE *splitcmds_fp=fopen(splitcmds_fn.c_str(),"w");
            char shr_tmp[8192];
            for(int i=0; i <(int)cells.size(); i++){
                int v_count=0;
                //cout << cells[i].bboxMarginUnRot._max << " " <<cells[i].bboxMargin._max <<endl;
                sprintf(shr_tmp,"cd %s;%s/treeBBClip ",           cwd,
                        basepath.c_str());
                for(int j=0; j <(int)vrip_cells.size(); j++){
                    if(vrip_cells[j].poses.size() == 0 || !cells[i].bboxMarginUnRot.intersects(vrip_cells[j].bounds.bbox))
                        continue;
                    sprintf(shr_tmp,"%s mesh-diced/tmp-clipped-diced-%08d.ply",shr_tmp,j);
                    v_count++;
                }
                if(v_count== 0)
                    continue;
                sprintf(shr_tmp,"%s --invrot %f %f %f --bbox %.16f %.16f %.16f %.16f %.16f %.16f -dup --outfile mesh-diced/un-tmp-tex-clipped-diced-r_%04d_c_%04d.ply;",
                        shr_tmp,rx,ry,rz,
                        cells[i].bboxMarginUnRot.xMin(),
                        cells[i].bboxMarginUnRot.yMin(),
                        -FLT_MAX,
                        cells[i].bboxMarginUnRot.xMax(),
                        cells[i].bboxMarginUnRot.yMax(),
                        FLT_MAX,
                        cells[i].row,cells[i].col);
                sprintf(shr_tmp,"%s  %s/vcgapps/bin/mergeMesh mesh-diced/un-tmp-tex-clipped-diced-r_%04d_c_%04d.ply -flip -cleansize %f -thresh %f -out mesh-diced/tmp-tex-clipped-diced-r_%04d_c_%04d.ply ;",shr_tmp,
                        basepath.c_str(),cells[i].row,cells[i].col,0.05,0.9*vrip_res,cells[i].row,cells[i].col);

                fprintf(splitcmds_fp,"%s;",shr_tmp);
                fprintf(splitcmds_fp,"%s/treeBBClip --bbox %.16f %.16f %.16f %.16f %.16f %.16f mesh-diced/tmp-tex-clipped-diced-r_%04d_c_%04d.ply -dup -F --outfile mesh-diced/tmp-tex-clipped-diced-r_%04d_c_%04d-lod%d.ply \n",
                        basepath.c_str(),
                        -FLT_MAX,-FLT_MAX,-FLT_MAX,
                        FLT_MAX,FLT_MAX,FLT_MAX,
                        cells[i].row,cells[i].col,  cells[i].row,cells[i].col,vpblod);

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
            for(int i=0; i <(int)vrip_cells.size(); i++){
                if(vrip_cells[i].poses.size() == 0)
                    continue;
                sprintf(tmp100, "mesh-diced/vis-clipped-diced-%08d-lod%d.ply",i,vpblod);
                cfiles.push_back(tmp100);
            }


            sprintf(tmp100, " > mesh-diced/vis-total.ply");
            /*  sprintf(tmp100,"%s; %s/treeBBClip --bbox %.16f %.16f %.16f %.16f %.16f %.16f mesh-diced/vis-total.ive -dup --outfile mesh-diced/vis-total.ply ",
                    tmp100,basepath.c_str(),
                    -FLT_MAX,-FLT_MAX,-FLT_MAX,
                    FLT_MAX,FLT_MAX,FLT_MAX);*/
            //tcmd+=tmp100;
            //postcmdv.push_back(tcmd);

            for(int i=0; i <(int)vrip_cells.size(); i++){
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
                sprintf(shr_tmp,"%s    %s/vcgapps/bin/mergeMesh mesh-diced/un-clipped-diced-%08d.ply -thresh %f -out mesh-diced/un2-clipped-diced-%08d-lod%d.ply ;",shr_tmp,
                        basepath.c_str(),i,0.9*vrip_res,i,vpblod);
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


            }


            fclose(splitcmds_fp);
            string splitcmd="split.py";
            string cwdmeshdiced=cwd;
            std::string extraCheckCmd=createFileCheckPython(tcmd,cwdmeshdiced,cfiles,string(tmp100),4);

            shellcm.write_generic(splitcmd,splitcmds_fn,"Split",NULL,NULL,0,extraCheckCmd);
            if(!no_split)
                sysres=system("python split.py");
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
                            cut_model(kdbb,tmpname,cells[i].bboxMargin,IntersectKdTreeBbox::DUP);
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
                            vrip_cells[i].valid=cut_model(kdbb,tmpname,box,IntersectKdTreeBbox::DUP);
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

        string texcmds_fn="mesh-diced/texcmds";
        string imgbase=(compositeMission? "/":"/img/");
        //int VTtileSize=256;
        int tileBorder=1;
        int ajustedGLImageSizeX=(int)reimageSize.x();//-((reimageSize.x()/VTtileSize)*2*tileBorder);
        int ajustedGLImageSizeY=(int)reimageSize.y();//-((reimageSize.y()/VTtileSize)*2*tileBorder);
        FILE *texcmds_fp=fopen(texcmds_fn.c_str(),"w");
        int totalX=ajustedGLImageSizeX*_tileRows;
        int totalY=ajustedGLImageSizeY*_tileColumns;

        FILE *FP2=fopen("image_areas.txt","w");
        FILE *reFP=fopen("rebbox.txt","w");
        FILE *FP3=fopen("createmosaic.sh","w");
        if(!FP2 || ! FP3){
            fprintf(stderr,"Can't open mosaic scripts\n");
            exit(-1);
        }
        fprintf(FP3,"#!/bin/bash\n cd mosaic \n gdalbuildvrt mosaic.vrt ");

        fprintf(reFP,"%.16f %.16f %.16f %.16f %.16f %.16f %d %d total\n",totalbb.xMin(),totalbb.xMax(),totalbb.yMin(),totalbb.yMax(),totalbb.zMin(),
                totalbb.zMax(),_tileColumns,_tileRows);

        fprintf(FP2,"0 %d 0 %d total total 0\n",ajustedGLImageSizeX*_tileRows,ajustedGLImageSizeY*_tileColumns);

        for(int i=0; i <(int)cells.size(); i++){
            char tmpfn[1024];
            sprintf(tmpfn,"mesh-diced/tmp-tex-clipped-diced-r_%04d_c_%04d-lod%d.ply", cells[i].row,cells[i].col,vpblod);
            if(cells[i].images.size() == 0 || !osgDB::fileExists(tmpfn)){
                fprintf(reFP,"%.16f %.16f %.16f %.16f %.16f %.16f %d %d %s\n",cells[i].bbox.xMin(),cells[i].bbox.xMax(),cells[i].bbox.yMin(),cells[i].bbox.yMax(),cells[i].bbox.zMin(),
                        cells[i].bbox.zMax(),cells[i].col,cells[i].row,"null");
                continue;
            }
            fprintf(reFP,"%.16f %.16f %.16f %.16f %.16f %.16f %d %d %s\n",cells[i].bbox.xMin(),cells[i].bbox.xMax(),cells[i].bbox.yMin(),cells[i].bbox.yMax(),cells[i].bbox.zMin(),
                    cells[i].bbox.zMax(),cells[i].col,cells[i].row,cells[i].name.c_str());


            fprintf(texcmds_fp,"cd %s;setenv DISPLAY :0.%d;%s/calcTexCoord %s mesh-diced/tmp-tex-clipped-diced-r_%04d_c_%04d-lod%d.ply --outfile mesh-diced/tex-clipped-diced-r_%04d_c_%04d-lod%d.ply --zrange %f %f --invrot %f %f %f",
                    cwd,
                    gpunum,
                    basepath.c_str(),
                    base_dir.c_str(),
                    cells[i].row,cells[i].col,
                    vpblod,
                    cells[i].row,cells[i].col,
                    vpblod,totalbb_unrot.zMin(),totalbb_unrot.zMax(),
                    rx,ry,rz);
            if(hw_image){
                fprintf(texcmds_fp," --tex_cache %s %d  ",cachedtexdir[0].first.c_str(),cachedtexdir[0].second);
                if(use_debug_shader)
                    fprintf(texcmds_fp," --debug-shader ");
                if(!storeTexMesh)
                    fprintf(texcmds_fp," --imageNode %d %d %d %d %d %d --untex",cells[i].row,cells[i].col,_tileRows,_tileColumns,ajustedGLImageSizeX,ajustedGLImageSizeY);
                if(useAtlas)
                    fprintf(texcmds_fp," --atlas");
            }
            else{
                fprintf(texcmds_fp,";%s/nonmem  mesh-diced/tex-clipped-diced-r_%04d_c_%04d-lod%d.ply mesh-diced/bbox-tmp-tex-clipped-diced-r_%04d_c_%04d.ply.txt %s --mat mesh-diced/tex-clipped-diced-r_%04d_c_%04d.mat --invrot %f %f %f --size %d %d --image %d %d %d %d -lat %.28f -lon %.28f",
                        basepath.c_str(),
                        cells[i].row,cells[i].col,
                        vpblod,
                        cells[i].row,cells[i].col,
                        (base_dir+imgbase).c_str(),
                        cells[i].row,cells[i].col,
                        rx,ry,rz,
                        ajustedGLImageSizeX,ajustedGLImageSizeY,
                        cells[i].row,cells[i].col,_tileRows,_tileColumns,
                        latOrigin , longOrigin);
                if(blending)
                    fprintf(texcmds_fp," --blend");
            }
            char tmp_ds[1024];
            tmp_ds[0]='\0';
            int num_samples=6;
            for(int p=0; p<num_samples; p++)
                sprintf(tmp_ds,"%s %d",tmp_ds,(int)pow(2,p+1));
            fprintf(texcmds_fp,";gdaladdo -r average mosaic/image_r%04d_c%04d_rs%04d_cs%04d.tif %s\n",cells[i].row,cells[i].col,_tileRows,_tileColumns, tmp_ds);
            tmp_ds[0]='\0';
            num_samples=0;
            for(int p=0; p<num_samples; p++)
                sprintf(tmp_ds,"%s %d",tmp_ds,(int)pow(2,p+1));
            fprintf(FP2,"%d %d %d %d mesh-diced/image_r%04d_c%04d_rs%04d_cs%04d-tmp.ppm mosaic/image_r%04d_c%04d_rs%04d_cs%04d.tif %d 1%s\n",(totalX-(ajustedGLImageSizeX*(cells[i].row+1))),
                    (totalX-ajustedGLImageSizeX*(cells[i].row)),ajustedGLImageSizeY*cells[i].col,ajustedGLImageSizeY*(cells[i].col+1),cells[i].row,cells[i].col,_tileRows,_tileColumns,
                    cells[i].row,cells[i].col,_tileRows,_tileColumns,num_samples+1,tmp_ds);
            fprintf(FP3,"image_r%04d_c%04d_rs%04d_cs%04d.tif ",cells[i].row,cells[i].col,_tileRows,_tileColumns);

           // fprintf(texcmds_fp,"\n");

            /* fprintf(texcmds_fp,"osgconv -O \"compressed=1 noTexturesInIVEFile=1 noLoadExternalReferenceFiles=1 useOriginalExternalReferences=1\" mesh-diced/tex-clipped-diced-r_%04d_c_%04d-lod%d-uncomp.ive  mesh-diced/tex-clipped-diced-r_%04d_c_%04d-lod%d.ive\n",

                            cells[i].row,cells[i].col,
                            vpblod,
                            cells[i].row,cells[i].col
                            ,vpblod);*/

        }
        fclose(texcmds_fp);
        fclose(reFP);
        fclose(FP2);
        fprintf(FP3,"\n#gdaladdo -ro --config INTERLEAVE_OVERVIEW PIXEL --config COMPRESS_OVERVIEW JPEG mosaic.vrt 2 4 8 16 32\n");
        fchmod(fileno(FP3),0777);

        fclose(FP3);
        std::ostringstream p1;

        vector<std::string> precmd;
        if(hw_image){
            p1 <<basepath << "/createSem";
            precmd.push_back(p1.str());
        }



        string texcmd="tex.py";
        vector<std::string> postcmdv;
        std::ostringstream p;
        //bool singleThreadedDicedTex=false;
        /*p<<"#"<<basepath << "/sparseJoin rebbox.txt  " << cwd  << " --pbuffer-only " << ajustedGLImageSizeX << " "<< ajustedGLImageSizeY  << setprecision(28) <<" -lat " << latOrigin << " -lon " << longOrigin;
        if(!singleThreadedDicedTex)
            p<<" -nogfx " << (hw_image ? "png" : "ppm");
        if(untex)
            p<< " -untex ";*/
        p<< "cd " << cwd <<";sh createmosaic.sh";

        postcmdv.push_back(p.str());
#define SINGLE_MESH_TEX 1
#if SINGLE_MESH_TEX
        std::ostringstream p2;
        p2 << basepath << "/singleImageTex " << "mesh-diced/vis-total.ply --outfile mesh-diced/totaltex.ply "<< "--size " << totalX << " "<<totalY;
        if(exportForStaticRender)
            p2      <<" --extra totaltex.obj";

        postcmdv.push_back(p2.str());


#endif

        //int sizeX=reimageSize.x()*_tileRows;
        //int adjustedSize=tileSize-(2*tileBorder);
        // int intDiv=sizeX/tileSize;
        //int embedSize=(intDiv* adjustedSize);
        std::ostringstream p3;
        // p3 << "vips " << " im_extract_area " << "out.tif "<< " tex.tif " << " 0 0 " <<  embedSize << " "<<embedSize<< ";";
        if(!useVirtTex)
            p3<<"#";
        p3 << basepath << "/generateVirtualTextureTiles.py " << "-f=jpg  -b="<<tileBorder<<" tex.tif ";
        postcmdv.push_back(p3.str());

        shellcm.write_generic(texcmd,texcmds_fn,"Tex",&(precmd),&(postcmdv),num_threads);
        if(!no_tex)
            sysres=system("python tex.py");

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
                sysres=system("python vttex.py");

        }


        string simpcmds_fn="mesh-diced/simpcmds";

        FILE *simpcmds_fp=fopen(simpcmds_fn.c_str(),"w");
        string app;
        if(useReimage)
            app="tridecimator";
        else
            app="texturedDecimator";
        std::vector<std::vector<string> > datalist_lod;
        vector<string> mergeandcleanCmdsSimp;

#ifdef SINGLE_MESH_TEX


        {
            char tmp[1024];
            if(useVirtTex)
                fprintf(simpcmds_fp,"cd %s/mesh-diced;cp totaltex.ply total-lod%d.ply;",cwd,vpblod);
            else
                fprintf(simpcmds_fp,"cd %s/mesh-diced;cp vis-total.ply total-lod%d.ply;",cwd,vpblod);

            sprintf(tmp,"mesh-diced/total-lod%d.ply",vpblod);//std::min(lod,2)
            std::vector<string> level;

            for(int j=vpblod-1; j >= 0; j--){

                fprintf(simpcmds_fp,"cd %s/mesh-diced;%s/vcgapps/bin/%s total-lod%d.ply total-lod%d.ply %d -P -Oy -By;",
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

        char szProj4[4096];
        char wkt[4096];
        sprintf(wkt,"PROJCS[\"unnamed\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],TOWGS84[0,0,0,0,0,0,0],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9108\"]],AUTHORITY[\"EPSG\",\"4326\"]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"latitude_of_origin\",%.12f],PARAMETER[\"central_meridian\",%.12f],PARAMETER[\"scale_factor\",1],PARAMETER[\"false_easting\",0],PARAMETER[\"false_northing\",0],UNIT[\"Meter\",1]]",
                latOrigin,longOrigin);
        sprintf( szProj4,
                 "\"+proj=tmerc +lat_0=%.24f +lon_0=%.24f +k=%.12f +x_0=%.12f +y_0=%.12f +datum=WGS84 +ellps=WGS84 +units=m +no_defs\"",latOrigin,longOrigin,1.0,0.0,0.0);




        if(!novpb) {
            if(!mgc){
                mgc = new MyGraphicsContext();
                if(!mgc->valid()){
                    no_hw_context=true;
                    printf("Forcing no hw compression due to lack of graphics card context\n");
                }
            }
            doQuadTreeVPB(basepath,datalist_lod,bounds,calib.camera_calibs[0],cachedtexdir,useTextureArray,useReimage,useVirtTex,totalbb_unrot,wkt_coord_system,string(wkt),no_hw_context);
        }

        char zipstr[1024];
        sprintf(zipstr,"7z a -r -sfx7z.sfx mesharchive.7z.exe real.ive real_root_L0_X0_Y0/ -m0=lzma2 -mmt%d -mx9",num_threads);
        FILE *zfp=fopen("zip.sh","w");
        fprintf(zfp,"#!/bin/bash\n%s\n",zipstr);
        fclose(zfp);


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




}








