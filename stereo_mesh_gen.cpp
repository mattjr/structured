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
#include <osg/Timer>
#include <osg/Vec3>
#include <osg/Array>
#include <osg/ref_ptr>
#include <osgUtil/DelaunayTriangulator>
#include "PLYWriterNodeVisitor.h"
#include "StereoEngine.h"
#include <osgUtil/Optimizer>
#include <osg/io_utils>
#include <osg/ArgumentParser>
#include <osg/ComputeBoundsVisitor>
#include "stereo_cells.hpp"

#include "SeaBedIO.h"
#include <fstream>
using namespace std;


//
// Command-line arguments
//
static string stereo_config_file_name;
static string left_file_name;
static string right_file_name;


static bool pause_after_each_frame = false;
static double image_scale = 1.0;
static int max_feature_count = 200;
static double max_triangulation_len=10.0;

static bool display_debug_images = false;

static int tex_size=512;
static double feature_depth_guess = AUV_NO_Z_GUESS;
static double edgethresh=0.5;
static bool use_dense_stereo=false;
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

    return 1;

}


int main( int argc, char *argv[ ] )
{



    osg::ArgumentParser arguments(&argc,argv);
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" <basedir> <primary image> <secondary image> ");
    arguments.getApplicationUsage()->addCommandLineOption("--mat-1-8","REQUIRED mat(0,0),mat(1,0),mat(2,0),mat(3,0),mat(0,1),mat(1,1),mat(2,1),mat(3,1)");
    arguments.getApplicationUsage()->addCommandLineOption("--mat-8-16","REQUIRED mat(0,2),mat(1,2),mat(2,2),mat(3,2),mat(0,3),mat(1,3),mat(2,3),mat(3,3)");

    arguments.getApplicationUsage()->addCommandLineOption("--ds","                    Dense Stereo");

    arguments.getApplicationUsage()->addCommandLineOption("-r", "<resize_scale>       Resize the images by a scaling factor.");
    arguments.getApplicationUsage()->addCommandLineOption("-m", "<max_feature_count>  Set the maximum number of features to be found.");
    arguments.getApplicationUsage()->addCommandLineOption("-n", "<max_frame_count>    Set the maximum number of frames to be processed.");
    arguments.getApplicationUsage()->addCommandLineOption("-z", "<feature_depth>      Set an estimate for the feature depth relative to cameras.");
    arguments.getApplicationUsage()->addCommandLineOption("-t", "<output_file>        Save triangulate feature positions to a file");
    arguments.getApplicationUsage()->addCommandLineOption("-d", "                    Do not display debug images.");
    arguments.getApplicationUsage()->addCommandLineOption("-p","                      Pause after each frame.");
    arguments.getApplicationUsage()->addCommandLineOption("--ds","                    Dense Stereo");

    osg::Matrix mat;
    if(arguments.argc() < 4){
        arguments.getApplicationUsage()->write(cout);
        return -1;
    }
    string basedir=arguments[1];
    string imgdir=basedir+"/img/";
    bool batch=false;
    string posefile;
    int start=0,end=0;

    if(arguments.read("--batch",posefile,start,end)){
        batch=true;
    }else{
        left_file_name=arguments[2];
        right_file_name=arguments[3];
        if(!(arguments.read("--mat-1-8",mat(0,0),mat(1,0),mat(2,0),mat(3,0),
                            mat(0,1),mat(1,1),mat(2,1),mat(3,1)) && arguments.read("--mat-8-16",mat(0,2),mat(1,2),mat(2,2),mat(3,2),
                                                                                   mat(0,3),mat(1,3),mat(2,3),mat(3,3)))){
            cout << "Can't load mat\n";
            arguments.getApplicationUsage()->write(cout);
            return -1;
        }
    }


    arguments.read("-r",image_scale);
    arguments.read("-m",max_feature_count);
    arguments.read("--tex-size",tex_size);

    arguments.read("-z" ,feature_depth_guess);
    use_dense_stereo=arguments.read("--ds");
    //bool verbose=arguments.read("-v");
    display_debug_images=arguments.read("-d");
    arguments.read("--maxlen",max_triangulation_len);

    if(!arguments.read("--edgethresh",edgethresh)){
        cout << "Can't load edgethresh\n";
        arguments.getApplicationUsage()->write(cout);
        return -1;

    }
    string stereo_calib_file_name="stereo.calib";
     Config_File *recon_config_file;
    try {
        recon_config_file= new Config_File((basedir+"/"+"mesh.cfg").c_str());
    }   catch( string error ) {
        cerr << "ERROR - " << error << endl;
        exit( 1 );
    }
  vector<Stereo_Pose_Data> tasks;
    int goodMeshes=0;
     if(batch){

         Stereo_Pose_File pose_data=read_stereo_pose_est_file(posefile );
         int cnt=0;
         vector<Stereo_Pose>::const_iterator cii;
         cii=pose_data.poses.begin();
         while( cii != pose_data.poses.end() ){



             Stereo_Pose_Data name;
             get_auv_image_name( imgdir, *cii, name) ;
             cii++;
             cnt++;
             if(cnt-1 < start || cnt-1 >= end){
                 continue;
             }


             name.valid=false;
             tasks.push_back(name);

         }



         if(tasks.size() <= 0){
             fprintf(stderr,"No tasks loaded check %s\n",posefile.c_str());
             exit(-1);
         }

     }
    StereoCalib stereocal((basedir+"/"+stereo_calib_file_name).c_str());
    OpenThreads::Mutex mutex;
    double min_feat_dist=3.0;
    double quality=0.0001;

    StereoEngine engine(stereocal,*recon_config_file,edgethresh,max_triangulation_len,max_feature_count,min_feat_dist,quality,tex_size,mutex,false,false);
    cvSetNumThreads(1);

    osg::BoundingBox bbox;
    MatchStats stats;
    if(batch){
        for(int i=0; i<(int)tasks.size(); i++){
            left_file_name=tasks[i].left_name;
            right_file_name=tasks[i].right_name;
            feature_depth_guess=tasks[i].alt;
            mat=tasks[i].mat;
            if(engine.processPair(basedir,left_file_name,right_file_name,mat,bbox,stats,feature_depth_guess,true,false) == STEREO_OK)
                goodMeshes++;
            printf("\r%03d/%03d",i,(int)tasks.size());
            fflush(stdout);
        }
        printf("\n");

    }else{
        engine.processPair(basedir,left_file_name,right_file_name,mat,bbox,stats,feature_depth_guess,true,false);

    }

    //
    // Clean-up
    //
    if(batch && goodMeshes ==0){
        fprintf(stderr,"All meshes in this batch failed to process correctly!\n");
        return -1;
    }
    return 0;
}
