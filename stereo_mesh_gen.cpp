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
    left_file_name=arguments[2];
    right_file_name=arguments[3];

    if(!(arguments.read("--mat-1-8",mat(0,0),mat(1,0),mat(2,0),mat(3,0),
                        mat(0,1),mat(1,1),mat(2,1),mat(3,1)) && arguments.read("--mat-8-16",mat(0,2),mat(1,2),mat(2,2),mat(3,2),
                                                                               mat(0,3),mat(1,3),mat(2,3),mat(3,3)))){
        cout << "Can't load mat\n";
        arguments.getApplicationUsage()->write(cout);
        return -1;
    }
    arguments.read("-r",image_scale);
    arguments.read("-m",max_feature_count);
    arguments.read("--tex-size",tex_size);

    arguments.read("-z" ,feature_depth_guess);
    use_dense_stereo=arguments.read("--ds");
    bool verbose=arguments.read("-v");
    display_debug_images=arguments.read("-d");
    arguments.read("--maxlen",max_triangulation_len);

    if(!arguments.read("--edgethresh",edgethresh)){
        cout << "Can't load edgethresh\n";
        arguments.getApplicationUsage()->write(cout);
        return -1;

    }
    string stereo_calib_file_name="stereo.calib";


    StereoCalib stereocal((basedir+"/"+stereo_calib_file_name).c_str());
    OpenThreads::Mutex mutex;
    double min_feat_dist=3.0;
    double quality=0.0001;
    StereoEngine engine(stereocal,edgethresh,max_triangulation_len,max_feature_count,min_feat_dist,quality,tex_size,mutex);
    osg::BoundingBox bbox;
    engine.processPair(basedir,left_file_name,right_file_name,mat,bbox,feature_depth_guess);


    //
    // Clean-up
    //

    return 0;
}
