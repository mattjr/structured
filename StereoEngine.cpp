/*
 * StereoEngine.cc
 * Copyright (C) Brandon Wampler 2009 <bwampler@purdue.edu>
 *
 * StereoEngine.cc is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * StereoEngine.cc is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "StereoEngine.h"
#include <iostream>
#include <stdlib.h>
#include <osgViewer/Viewer>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/PointSprite>
#include <osg/Point>
#include <osgGA/TrackballManipulator>
#include <math.h>
#include <iomanip>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include "PLYWriterNodeVisitor.h"
#include <osgUtil/DelaunayTriangulator>
#include <osg/ComputeBoundsVisitor>
#include <osgUtil/Optimizer>
#include <osg/io_utils>
#include <sys/time.h>
#ifdef OLD_AUV_CODE
#include "auv_stereo_geometry_compat.hpp"
#include "auv_stereo_keypoint_finder.hpp"
#include "adt_image_norm.hpp"
#include "auv_stereo_dense.hpp"
#include "auv_geometry.hpp"

using namespace libsnapper;
#endif
#include "gts.h"
#include "TriMesh.h"
#include "TriMesh_algo.h"
#include "XForm.h"
#include "mesh_proc.hpp"
#include "RobustMatcher.h"
using namespace std;
using namespace cv;
using namespace mesh_proc;
void cacheImage(IplImage *img,string name,int tex_size){


    IplImage *tex_img=cvCreateImage(cvSize(tex_size,tex_size),
                                    IPL_DEPTH_8U,3);

    if(img && tex_img)
        cvResize(img,tex_img,CV_INTER_AREA);
    else
        printf("Invalid Images\n");


    cvSaveImage(name.c_str(),tex_img);
    cvReleaseImage(&tex_img);
}



std::ostream& operator<<(std::ostream& os, const MatchStats& ms)
{
    os << ms.total_init_feat << "/" <<ms.total_matched_feat<< "/"<<ms.total_accepted_feat<<"/"<<ms.total_tracking_fail<<"/"
       <<ms.total_epi_fail<<"/"<<ms.total_tri_fail;
    return os;
}

//
// Get the time in seconds (since the Unix Epoch in 1970)
//
double get_time( void )
{
    struct timeval tv;

    gettimeofday( &tv, NULL );
    return tv.tv_sec+(double)tv.tv_usec/1e6;
}

//
// Load the next pair of images from the contents file
//
bool get_stereo_pair(
        const string contents_dir_name,
        IplImage     *&left_image,
        IplImage     *&right_image,
        IplImage *&color_image,
        const string        &left_image_name,
        const string        &right_image_name )
{

    //
    // Try to read timestamp and file names
    //

    //
    // Load the images (-1 for unchanged grey/rgb)
    //
    string complete_left_name( contents_dir_name+left_image_name );
    left_image  = cvLoadImage( complete_left_name.c_str( ) , CV_LOAD_IMAGE_ANYCOLOR );
    color_image=NULL;
    if( left_image == NULL )
    {
        cerr << "ERROR - unable to load image: " << complete_left_name << endl;
        return false;
    }

    string complete_right_name( contents_dir_name+right_image_name );
    right_image = cvLoadImage( complete_right_name.c_str( ), CV_LOAD_IMAGE_ANYCOLOR );
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
    bool color_exists=false;
    if( left_image->nChannels == 3 )
    {
        IplImage *grey_left  = cvCreateImage( cvGetSize(left_image) , IPL_DEPTH_8U, 1 );
        cvCvtColor( left_image , grey_left , CV_BGR2GRAY );
        color_image=left_image;
        left_image = grey_left;
        color_exists=true;
    }

    if( right_image->nChannels == 3 )
    {
        IplImage *grey_right = cvCreateImage( cvGetSize(right_image), IPL_DEPTH_8U, 1 );
        cvCvtColor( right_image, grey_right, CV_BGR2GRAY );
        color_image = right_image;
        right_image = grey_right;
        color_exists=true;
    }
    if(!color_exists){
        IplImage *color_image  = cvCreateImage( cvGetSize(left_image) , IPL_DEPTH_8U, 3 );
        cvCvtColor( left_image , color_image , CV_GRAY2BGR );

    }
    //
    // Scale images if required
    //
    /*if( image_scale != 1.0 )
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
    }*/
    return true;
}

bool checkmkdir(std::string dir){
    if(!osgDB::fileExists(dir)){
        fprintf(stderr, "Creating %s\n",dir.c_str());
        if(!osgDB::makeDirectory(dir)){
            fprintf(stderr, "Failed creating %s\n",dir.c_str());
            return false;
        }
    }
    return true;
}
int apply_epipolar_constraints(
        const CvMat *F,  double max_dist,
        const vector<CvPoint2D64f> &undist_coords1,
        const vector<CvPoint2D64f> &undist_coords2,
        char *valid,
        char *valid2,
        const StereoCalib &_calib)
{
    int failNum=0;
    assert( undist_coords1.size()==undist_coords2.size() );

    unsigned int num_coords = undist_coords1.size();
    //vector<bool> results( num_coords, false );

    // cvComputeCorrespondEpilines throws an exception on less than three points
    if( num_coords<3 ){
        for(unsigned int i=0; i <num_coords; i++)
            valid[i]=false;
        return -1;
    }

    // Create array of coordinates in frame 1
    CvMat *feature_matrix = cvCreateMat( 2, num_coords, CV_32F );
    for( unsigned int i=0; i<num_coords; i++ )
    {
        cvmSet( feature_matrix, 0, i, undist_coords1[i].x*_calib.camera_calibs[0].fcx
                +_calib.camera_calibs[0].ccx);
        cvmSet( feature_matrix, 1, i, undist_coords1[i].y*_calib.camera_calibs[0].fcy
                +_calib.camera_calibs[0].ccy );
    }

    // Calculate epipolar lines for coords in frame 2
    CvMat *epi_lines = cvCreateMat( 3, num_coords, CV_32F );
    cvComputeCorrespondEpilines( feature_matrix,
                                 1,
                                 const_cast<CvMat *>(F),
                                 epi_lines );


    // Check the distances of the current features from the epipolar lines
    for( unsigned int i=0; i<num_coords; i++ )
    {
        if(!valid[i] || !valid2[i])
            continue;

        // Line equation Ax+By+C = 0;
        double A = cvmGet( epi_lines, 0, i );
        double B = cvmGet( epi_lines, 1, i );
        double C = cvmGet( epi_lines, 2, i );

        double x = undist_coords2[i].x*_calib.camera_calibs[1].fcx+
                +_calib.camera_calibs[1].ccx;
        double y = undist_coords2[i].y*_calib.camera_calibs[1].fcy+
                +_calib.camera_calibs[1].ccy;
        double dist = fabs(A*x+B*y+C);
        if( dist<=max_dist )
            valid[i] = true;
        else{
            valid[i]=false;
            failNum++;
        }
    }

    // Clean-up
    cvReleaseMat( &epi_lines);
    cvReleaseMat( &feature_matrix);

    return failNum;
}





StereoEngine::StereoEngine(const StereoCalib &calib,Config_File &recon,double edgethresh,double max_triangulation_len,int max_feature_count,int tex_size,OpenThreads::Mutex &mutex,bool use_dense_stereo,bool pause_after_each_frame):
    _calib(calib),_recon(recon),edgethresh(edgethresh),max_triangulation_len(max_triangulation_len),max_feature_count(max_feature_count), tex_size(tex_size),
    verbose(false),display_debug_images(false),_osgDBMutex(mutex),use_dense_stereo(use_dense_stereo),pause_after_each_frame(pause_after_each_frame)
{


    imageSize = cvSize(calib.camera_calibs[0].width,calib.camera_calibs[0].height);
    _recon.get_value( "SCF_SHOW_DEBUG_IMAGES"  , display_debug_images );


    // leftImageR = cvCreateImage(imageSize, IPL_DEPTH_8U, 3);

    rightPyr = cvCreateImage(imageSize, IPL_DEPTH_8U, 1);
    leftPyr = cvCreateImage(imageSize, IPL_DEPTH_8U, 1);
    eig = cvCreateImage( imageSize, IPL_DEPTH_32F, 1 );
    temp = cvCreateImage( imageSize, IPL_DEPTH_32F, 1 );
    // rightGreyR = cvCreateImage(imageSize, IPL_DEPTH_8U, 1);
    // leftGreyR = cvCreateImage(imageSize, IPL_DEPTH_8U, 1);
    // img1r = cvCreateMat( imageSize.height, imageSize.width, CV_8U );
    //img2r = cvCreateMat( imageSize.height, imageSize.width, CV_8U );


    //rightGrey =rightImage; //cvCreateImage(imageSize, IPL_DEPTH_8U, 1);
    //leftGrey = leftImage;//cvCreateImage(imageSize, IPL_DEPTH_8U, 1);
    // eig = cvCreateImage( imageSize, IPL_DEPTH_32F, 1 );
    // temp = cvCreateImage( imageSize, IPL_DEPTH_32F, 1 );

    /*mx1 = cvCreateMat( imageSize.height, imageSize.width, CV_32F );
    my1 = cvCreateMat( imageSize.height, imageSize.width, CV_32F );
    mx2 = cvCreateMat( imageSize.height, imageSize.width, CV_32F );
    my2 = cvCreateMat( imageSize.height, imageSize.width, CV_32F );
    R1 = cvCreateMat(3, 3, CV_64F);
    R2 = cvCreateMat(3, 3, CV_64F);
    P1 = cvCreateMat(3, 4, CV_64F);
    P2 = cvCreateMat(3, 4, CV_64F);

    Q = cvCreateMat( 4, 4, CV_32F);
    M1 = cvCreateMat(3, 3, CV_64F);
    M2 = cvCreateMat(3, 3, CV_64F);
    D1 = cvCreateMat(5, 1, CV_64F);
    D2 = cvCreateMat(5, 1, CV_64F);*/
    /* disp = cvCreateMat( imageSize.height, imageSize.width, CV_16S );
    vdisp = cvCreateMat( imageSize.height, imageSize.width, CV_8U );
    realDisp = cvCreateMat( imageSize.height, imageSize.width, CV_32FC1);
    pair = cvCreateMat( imageSize.height, imageSize.width*2, CV_8UC3 );
    threeD = cvCreateImage(imageSize,IPL_DEPTH_32F,3);
*/
    //Set Default BMState for StereoEngine
    BMState = cvCreateStereoBMState();

    BMState->preFilterSize=41;
    BMState->preFilterCap=31;
    BMState->SADWindowSize=41;
    BMState->minDisparity=1;
    BMState->numberOfDisparities=128;
    BMState->textureThreshold=10;
    BMState->uniquenessRatio=15;

    /*std::cout<<"Default BMState Values"<<std::endl;
    std::cout<<"preFilterSize- "<<BMState->preFilterSize<<std::endl;//=41;
    std::cout<<"preFilterCap- "<<BMState->preFilterCap<<std::endl;//=31;
    std::cout<<"SADWindowSize- "<<BMState->SADWindowSize<<std::endl;//=41;
    std::cout<<"minDisparity- "<<BMState->minDisparity<<std::endl;//=1;
    std::cout<<"numberOfDisparities- "<<BMState->numberOfDisparities<<std::endl;//=128;
    std::cout<<"textureThreshold- "<<BMState->textureThreshold<<std::endl;//=10;
    std::cout<<"uniquenessRatio- "<<BMState->uniquenessRatio<<std::endl;//=15;
*/
#ifdef OLD_AUV_CODE
    _auv_stereo_calib = new libsnapper::Stereo_Calib(calib.filename);
    double image_scale=1.0;
    bool use_undistorted_images=false;
    finder = new Stereo_Corner_Finder( recon,
                                       use_undistorted_images,
                                       image_scale,
                                       _auv_stereo_calib );


    frame_id=0;
    recon.get_value( "SCF_DEBUG_PER_REJECTED_OUTPUT_DEBUG", thresh_per_rejected_output_debug, SCF_THRESH_PER_REJECT );
    recon.get_value( "DEBUG_MIN_FEAT_PER_FRAME",minFeatPerFrameThresh,100);
    recon.get_value("SCF_SAVE_DEBUG_IMAGES",_writeDebugImages,false);
    recon.get_value( "SCF_MAX_EPIPOLAR_DIST", _max_epi_dist,4.0);
    recon.get_value( "SKF_SHOW_DEBUG_IMAGES",
                     show_debug_images);
    /*_F.create(3,3,CV_64FC1);
               for( int i=0 ; i<3 ; i++ )
               {
                  for( int j=0 ; j<3 ; j++ )
                     _F.at<float>( i, j)= _auv_stereo_calib->F_left_to_right(i,j) ;
               }
               F = cvCreateMat(3,3,CV_32F);
                   for( int i=0 ; i<3 ; i++ )
                   {
                      for( int j=0 ; j<3 ; j++ )
                         cvmSet( F, i, j, _auv_stereo_calib->F_left_to_right(i,j) );
                   }*/
    for(int i=0; i< 2; i++){
        local_calib[i].ccx=_calib.camera_calibs[i].ccx;
        local_calib[i].ccy=_calib.camera_calibs[i].ccy;
        local_calib[i].fcx=_calib.camera_calibs[i].fcx;
        local_calib[i].fcy=_calib.camera_calibs[i].fcy;

        local_calib[i].kc1=_calib.camera_calibs[i].kc1;
        local_calib[i].kc2=_calib.camera_calibs[i].kc2;
        local_calib[i].kc3=_calib.camera_calibs[i].kc3;
        local_calib[i].kc4=_calib.camera_calibs[i].kc4;
        local_calib[i].kc5=_calib.camera_calibs[i].kc5;

    }
    undist_left=NULL;
    undist_right=NULL;
#endif

    if(use_dense_stereo)
      sdense= new Stereo_Dense(recon,1.0,&_calib);

    loadMatrices();
}

StereoEngine::~StereoEngine()
{
    //Release Images
    //if (leftImage) cvReleaseImage(&leftImage);
    //if (leftImageR) cvReleaseImage(&leftImageR);
    //if (rightImage) cvReleaseImage(&rightImage);
    //if (leftGrey) cvReleaseImage(&leftGrey);
    // if (rightGrey) cvReleaseImage(&rightGrey);
    /* if (leftGreyR) cvReleaseImage(&leftGreyR);
    if (rightGreyR) cvReleaseImage(&rightGreyR);

*/
    if (eig) cvReleaseImage(&eig);
    if (temp) cvReleaseImage(&temp);
    if(rightPyr) cvReleaseImage(&rightPyr);
    if(leftPyr) cvReleaseImage(&leftPyr);

    //Release arrays
    //if (points[0]) cvFree(&points[0]);
    //if (points[1]) cvFree(&points[1]);

    //Release Matrices
    //if (F) cvReleaseMat(&F);
    /*  if (Q) cvReleaseMat(&Q);
    if (E) cvReleaseMat(&E);
    if (T) cvReleaseMat(&T);
    if (R) cvReleaseMat(&R);
    if (M1) cvReleaseMat(&M1);
    if (M2) cvReleaseMat(&M2);
    if (D1) cvReleaseMat(&D1);
    if (D2) cvReleaseMat(&D2);
    if (R1) cvReleaseMat(&R1);
    if (R2) cvReleaseMat(&R2);
    if (P1) cvReleaseMat(&P1);
    if (P2) cvReleaseMat(&P2);

    if (mx1) cvReleaseMat(&mx1);
    if (my1) cvReleaseMat(&my1);
    if (mx2) cvReleaseMat(&mx2);
    if (my2) cvReleaseMat(&my2);
    if (pair) cvReleaseMat(&pair);
    if (img1r) cvReleaseMat(&img1r);
    if (img2r) cvReleaseMat(&img2r);

    if (disp) cvReleaseMat(&disp);
    if (vdisp) cvReleaseMat(&vdisp);
    if (realDisp) cvReleaseMat(&realDisp);
*/
}


void StereoEngine::drawPoints(CvArr* image,CvPoint2D32f points[], int count)
{
    for (int i = 0; i<count; i++)
    {
        cvCircle(image, cvPointFrom32f(points[i]), 3, CV_RGB(0,255,0), -1, 8,0);

    }
}
void print_matrix(const char *str,CvMat *C){

    cout << setprecision( 15 ) << right << fixed;
    cout <<str<<"=\n";
    for ( int row = 0; row < 3; ++ row )
    {
        for ( int col = 0; col < 3; ++ col )
        {
            cout << setw( 15 ) << (double)cvmGet( C, row, col ) << " ";
        }
        cout << endl;
    }
}
bool is_new_corner_valid(
        double                x,
        double                y,
        int skip,
        double                min_dist,
        const CvPoint2D32f *corners,
        const char *valid,
        int count)
{
    double min_dist_squared = min_dist*min_dist;

    for( int i=0; i< skip; i++)
    {
        if(!valid[i])
            continue;
        double current_x = corners[i].x;
        double current_y = corners[i].y;

        double diff_x = x - current_x;
        double diff_y = y - current_y;

        double dist_squared = diff_x*diff_x + diff_y*diff_y;
        if( dist_squared < min_dist_squared ){
            return false;
        }
    }

    return true;
}

void StereoEngine::loadMatrices(){
    double camInt1_a[9] = {     _calib.camera_calibs[0].fcx,  0,  _calib.camera_calibs[0].ccx,
                                0,  _calib.camera_calibs[0].fcy, _calib.camera_calibs[0].ccy,
                                0,  0 ,  1};
    double camInt2_a[9] = {     _calib.camera_calibs[1].fcx,  0,  _calib.camera_calibs[1].ccx,
                                0,  _calib.camera_calibs[1].fcy, _calib.camera_calibs[1].ccy,
                                0,  0 ,  1};
    CvMat camInt1 = cvMat(3,3,CV_64F,camInt1_a);
    CvMat camInt2 = cvMat(3,3,CV_64F,camInt2_a);

    //the camera calibration also delivers some distortion parameters
    double camDist1_a[5] = { _calib.camera_calibs[0].kc1, _calib.camera_calibs[0].kc2,
                             _calib.camera_calibs[0].kc3,
                             _calib.camera_calibs[0].kc4, _calib.camera_calibs[0].kc5};
    double camDist2_a[5] = { _calib.camera_calibs[1].kc1, _calib.camera_calibs[1].kc2,
                             _calib.camera_calibs[1].kc3,
                             _calib.camera_calibs[1].kc4, _calib.camera_calibs[1].kc5};
    CvMat camDist1 = cvMat(5,1,CV_64F,camDist1_a);
    CvMat camDist2 = cvMat(5,1,CV_64F,camDist2_a);


    //now we have to define the extrinsic of our stereo frame
    //for this we need to know the camera view matrices...
    //the first camera is easy, because we define this view as our system origin
    double cam1Ext_a[12] = {    1,0,0,    0,
                                0,1,0,    0,
                                0,0,1,    0};
    CvMat cam1Ext = cvMat(3,4,CV_64F,cam1Ext_a);
    //-0.289405 0.323173 2.549763
    //531.403870 748.795776 483.031281 720.312500

    //to get the position of the second camera relative to the first there
    //is again a smart opencv function: cvStereoCalibrate
    //we feed this function with the known intrinsic's and a few points from a
    //calibration chessboard and we get R and T of the second camera.
    double rv1_a[3] = { 0,0,0 };
    CvMat RV1 = cvMat(3,1,CV_64F,rv1_a);
    // double rv2_a[3] = { 0,0,0 };
    // CvMat RV2 = cvMat(3,1,CV_64F,rv2_a);
    _RV2 = cvCreateMat(3,1,CV_64F);
    //rotation vector (same for both cameras)
    double *rm2_a= _calib.camera_calibs[1].rotMatr; //  = { 0,0,0, 0,0,0,  0,0,0};
    CvMat RM2 = cvMat(3,3,CV_64F,rm2_a);                //calculate rotation matrix
    cvRodrigues2( &RM2,_RV2);
    double t1_a[3] = {  0.0, 0.0, 0.0 };
    CvMat T1 = cvMat(3,1,CV_64F,t1_a);                //transformation vector camera 1
    double t2_a[3] = {  _calib.camera_calibs[1].transVect[0],
                        _calib.camera_calibs[1].transVect[1],
                        _calib.camera_calibs[1].transVect[2] };

    CvMat T2 = cvMat(3,1,CV_64F,t2_a);                //transformation vector camera 2

    //the camera matrix of the second camera is then [R,T]
    double cam2Ext_a[12] = {   rm2_a[0],rm2_a[1],rm2_a[2], t2_a[0],
                               rm2_a[3],rm2_a[4],rm2_a[5], t2_a[1],
                               rm2_a[6],rm2_a[7],rm2_a[8], t2_a[2]};
    CvMat cam2Ext = cvMat(3,4,CV_64F,cam2Ext_a);

    double* t = T2.data.db;
    double tx[] =
    {
        0, -t[2], t[1],
        t[2], 0, -t[0],
        -t[1], t[0], 0
    };
    CvMat Tx = cvMat(3, 3, CV_64F, tx);
    //print_matrix("ATX",&Tx);
    // print_matrix("AR",&RM2);
    double ikL[9];
    CvMat iKL = cvMat(3, 3, CV_64F, ikL);
    double ikR[9];
    CvMat iKR = cvMat(3, 3, CV_64F, ikR);
    cvInvert(&camInt1, &iKL);
    cvInvert(&camInt2, &iKR);
    double e[9], f[9],f_real[9];
    CvMat E = cvMat(3, 3, CV_64F, e);
    CvMat tmpF = cvMat(3, 3, CV_64F, f);

    cvMatMul(&RM2,&iKL,&E);
    cvMatMul( &Tx, &E, &E );


    // print_matrix("AiKL=",&iKL);

    // print_matrix("AiKR=",&iKR);
    cvGEMM( &iKR, &E, 1, 0, 0, &E, CV_GEMM_A_T );


    //  cvMatMul(&E, &iK, &tmpF);
    CvMat F = cvMat(3, 3, CV_64F, f_real);
    cvCopy(&E,&F);
    _T2=cvCloneMat(&T2);
    _camInt1=cvCloneMat(&camInt1);
    _camInt2=cvCloneMat(&camInt2);
    _camDist1=cvCloneMat(&camDist1);
    _camDist2=cvCloneMat(&camDist2);
    _cam1Ext=cvCloneMat(&cam1Ext);
    _cam2Ext=cvCloneMat(&cam2Ext);

    _F=cvCloneMat(&F);
    _recon.get_value( "SCF_MAX_EPIPOLAR_DIST", _l_to_r_max_epipolar_dist);
    _recon.get_value( "SCF_DEBUG_PER_REJECTED_OUTPUT_DEBUG", thresh_per_rejected_output_debug, 0.33 );
    _recon.get_value( "DEBUG_MIN_FEAT_PER_FRAME",minFeatPerFrameThresh,100);
    _recon.get_value( "MIN_FEAT_RERUN",min_feat_rerun,100);

    _recon.get_value("SCF_SAVE_DEBUG_IMAGES",_writeDebugImages,false);
    _recon.get_value( "SKF_SHOW_DEBUG_IMAGES",
                      show_debug_images);

    _recon.get_value( "ROBUSTMATCHER_MIN_FEAT",
                      _robust_matcher_min_feat,5000);
    _recon.get_value( "ROBUSTMATCHER_MAX_FEAT",
                      _robust_matcher_max_feat,8000);
    _recon.get_value( "ROBUSTMATCHER_NN_RATIO",
                      _robust_matcher_nn_ratio,0.65);

    _recon.get_value( "SCF_MIN_DISTANCE"     , _min_distance      );
    _recon.get_value( "SCF_SEARCH_WINDOW_X"  , _search_window_x   );
    _recon.get_value( "SCF_SEARCH_WINDOW_Y"  , _search_window_y   );
    _recon.get_value( "SCF_ZERO_ZONE_X"      , _zero_zone_x       );
    _recon.get_value( "SCF_ZERO_ZONE_Y"      , _zero_zone_y       );
    _recon.get_value( "SCF_SEARCH_ITERATIONS", _search_iterations );
    _recon.get_value( "SCF_QUALITY_LEVEL"    , _quality_level     );
    _recon.get_value( "SCF_BLOCK_SIZE"       , _block_size        );
    _recon.get_value( "SCF_USE_HARRIS"       , _use_harris        );
    _recon.get_value( "SCF_TRACK_MAX_ERROR", _track_max_error  );
    _recon.get_value( "SCF_TRACK_WINDOW_X"     , _track_window_x      );
    _recon.get_value( "SCF_TRACK_WINDOW_Y"     , _track_window_y      );
    _recon.get_value( "SCF_TRACK_MAX_ERROR"    , _track_max_error     );
    _recon.get_value( "SCF_PYRAMID_LEVEL"      , _pyramid_level       );
    _recon.get_value( "SCF_TRACK_ITERATIONS"   , _track_iterations    );
    _recon.get_value( "SCF_TRACK_EPSILON"      , _track_epsilon       );

}
int StereoEngine::keypointDepth(IplImage *leftGrey,IplImage *rightGrey,std::string left_file_name,MatchStats &stats,list<osg::Vec3> &points){


    //printf("Orig %d\n",new_method_pts.size());
    // SurfFeatureDetector detector(50);
    // DynamicAdaptedFeatureDetector
    // int n_features=400;
    //Ptr<AdjusterAdapter> adjuster = new FastAdjuster(10);
    // DynamicAdaptedFeatureDetector detector(adjuster, n_features * 0.9, n_features * 1.1, 1000);
    // Ptr<FeatureDetector> detector(new DynamicAdaptedFeatureDetector (100, 110, 10,
    //                          new FastAdjuster(20,true)));
    vector<KeyPoint> keypoints1, keypoints2;
    /*  detector.detect(left_frame, keypoints1);
detector.detect(right_frame, keypoints2);
*/
    // computing descriptors
    /*  SurfDescriptorExtractor extractor;
Mat descriptors1, descriptors2;
extractor.compute(left_frame, keypoints1, descriptors1);
extractor.compute(right_frame, keypoints2, descriptors2);*/
    /*    IplImage *left_undist_image=cvCreateImage(cvSize(left_frame->width,left_frame->height),IPL_DEPTH_8U,1);
    IplImage *right_undist_image=cvCreateImage(cvSize(right_frame->width,right_frame->height),IPL_DEPTH_8U,1);
    IplImage *invalidMask=cvCreateImage(cvSize(right_frame->width,right_frame->height),IPL_DEPTH_8U,1);
    IplImage *tmp=cvCreateImage(cvSize(right_frame->width,right_frame->height),IPL_DEPTH_8U,1);

    if(!undist_left)
        undist_left=new Undistort_Data(local_calib[0],left_frame,true);
    if(!undist_right)
        undist_right=new Undistort_Data(local_calib[1],right_frame,true);

    undistort_image(*undist_left,left_frame,left_undist_image);
    undistort_image(*undist_right,right_frame,right_undist_image);
    cvCmpS( left_undist_image, 1.0, invalidMask,  CV_CMP_LT );
    cvCmpS( right_undist_image, 1.0, tmp,  CV_CMP_LT );
    cvAdd(invalidMask,tmp,invalidMask);
    cvReleaseImage(&tmp);
*/
    RobustMatcher matcher(_robust_matcher_min_feat,
                          _robust_matcher_max_feat,
                          _robust_matcher_nn_ratio,_l_to_r_max_epipolar_dist);
    vector<DMatch> matches;
    matcher.match(leftGrey,rightGrey,matches,keypoints1, keypoints2);
    /*  // matching descriptors
FlannBasedMatcher matcher;
vector<DMatch> matches;
matcher.match(descriptors1, descriptors2, matches);
double max_dist = 0; double min_dist = 100;



//-- Quick calculation of max and min distances between keypoints
for( int i = 0; i < descriptors1.rows; i++ )
{ double dist = matches[i].distance;
if( dist < min_dist ) min_dist = dist;
if( dist > max_dist ) max_dist = dist;
}

printf("-- Max dist : %f \n", max_dist );
printf("-- Min dist : %f \n", min_dist );

//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist )
//-- PS.- radiusMatch can also be used here.
std::vector< DMatch > good_matches;*/
    std::vector< DMatch > good_matches;
    //  std::vector<CvPoint2D64f>  left_coords;
    //  std::vector<CvPoint2D64f>  right_coords;
    CvPoint2D32f* left_coords,*right_coords;
    char* Lstatus = 0;
    char* Rstatus = 0;

    Lstatus = (char*)cvAlloc(matches.size()*sizeof(char));
    Rstatus = (char*)cvAlloc(matches.size()*sizeof(char));
    memset(Lstatus,0,sizeof(char)*matches.size());
    memset(Rstatus,0,sizeof(char)*matches.size());
    left_coords = (CvPoint2D32f*)cvAlloc(matches.size()*sizeof(CvPoint2D32f));
    right_coords = (CvPoint2D32f*)cvAlloc(matches.size()*sizeof(CvPoint2D32f));

    //vector<bool>status(descriptors1.rows,true);

    for( int i = 0; i < matches.size(); i++ )
    { //if( matches[i].distance < 2*min_dist )
        {
            // cv::Point2f lp,rp;
            // lp=keypoints1[matches[i].queryIdx].pt;
            // rp=keypoints1[matches[i].trainIdx].pt;
            // double dist=pointToEpipolarLineCost(lp,rp,_F);
            //printf("%f\n",dist);
            // if(dist < 2.0)

            {

                CvPoint2D64f l,r;

                l.x=keypoints1[matches[i].queryIdx].pt.x;
                l.y=keypoints1[matches[i].queryIdx].pt.y;
                r.x=keypoints2[matches[i].trainIdx].pt.x;
                r.y=keypoints2[matches[i].trainIdx].pt.y;
                left_coords[i].x=l.x;
                left_coords[i].y=l.y;
                right_coords[i].x=r.x;
                right_coords[i].y=r.y;
                Rstatus[i]=true;
                Lstatus[i]=true;
                good_matches.push_back( matches[i]);

                //   printf("bad %f %f -- %f %f\n",l.x,l.y,r.x,r.y);
                // cvShowImage( "Good Matches", invalidMask );
                //  waitKey(0);
                /*  if( CV_IMAGE_ELEM(invalidMask,uchar,(int)l.y,(int)l.x) ==0 &&  CV_IMAGE_ELEM(invalidMask,uchar,(int)r.y,(int)r.x) == 0){
                    left_coords.push_back(l);
                    right_coords.push_back(r);
                    good_matches.push_back( matches[i]);
                }*/

            }
        }
    }
    //cout << "Number of matched points (after dist clean): "<<good_matches.size()<<endl;
    /*apply_epipolar_constraints(F,2.0,right_coords,left_coords,status,_calib);
for(int i=0; i< status.size(); i++){
  if(status[i]){
      good_matches.push_back( matches[i]);

  }
}*/
    //free(status);
    if(show_debug_images|| _writeDebugImages){

        Mat img_matches;
        drawMatches( leftGrey, keypoints1, rightGrey, keypoints2,
                     good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                     vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        //-- Show detected matches
        if(show_debug_images)
            imshow( "Good Matches", img_matches );

        string debugfilename=string("debug/"+osgDB::getSimpleFileName(left_file_name)+".surf.png");
        // cout <<debugfilename <<endl;
        if(_writeDebugImages)
        {
            if(!imwrite(debugfilename.c_str(),img_matches))
                fprintf(stderr,"Failed to write debug image %s\n",debugfilename.c_str());
        }
        if(show_debug_images)
            waitKey(5);
    }
    /*cvReleaseImage(&left_undist_image);
    cvReleaseImage(&right_undist_image);
    cvReleaseImage(&invalidMask);
*/

    triangulatePoints(matches.size(),left_coords,right_coords,Lstatus,Rstatus,stats,points);
    cvFree(&Lstatus);
    cvFree(&Rstatus);
    cvFree(&left_coords);
    cvFree(&right_coords);
    /*
    std::vector<bool>          valid;
    std::vector<libplankton::Vector> feats;
    stereo_triangulate( *_auv_stereo_calib,
                        ref_frame,
                        left_coords,
                        right_coords,
                        valid,

                        feats );
    for(int i=0; i<feats.size(); i++)
    {

        vert=(TVertex*)  gts_vertex_new (t_vertex_class (),
                                         feats[i][0],feats[i][1],feats[i][2]);
        // printf("%f %f %f\n", feats[i][0],feats[i][1],feats[i][2]);

        g_ptr_array_add(localV,GTS_VERTEX(vert));


    }
    */
    return good_matches.size();
}

//void StereoEngine::calculateOpticalFlow()
void StereoEngine::sparseDepth(IplImage *leftGrey,IplImage *rightGrey,MatchStats &stats,const int MAX_COUNT, list<osg::Vec3> &points,double zguess)
{
    //int win_size = 5;
    //const int MAX_COUNT = 500;
    char* Lstatus = 0;
    char* Rstatus = 0;

    int count = 0;
    int flags = 0;




    Lstatus = (char*)cvAlloc(MAX_COUNT);
    Rstatus = (char*)cvAlloc(MAX_COUNT);
    memset(Lstatus,0,sizeof(char)*MAX_COUNT);
    memset(Rstatus,0,sizeof(char)*MAX_COUNT);

    flags = 0;

    /* automatic initialization */

    count = MAX_COUNT;

    //Point found on grey image then projected to rectified grey image
    CvPoint2D32f* goodPoints[4]={0,0,0,0};
    goodPoints[0] = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(goodPoints[0][0]));
    goodPoints[1] = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(goodPoints[1][0]));
    goodPoints[2] = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(goodPoints[2][0]));
    goodPoints[3] = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(goodPoints[2][0]));

    //   int blocksize=5;
    //   bool useharris=true;




    cvGoodFeaturesToTrack( leftGrey, eig, temp, goodPoints[0], &count,
                           _quality_level, _min_distance, 0, _block_size, _use_harris);
    //  printf("m: GFTT 1 %d\n",count);
    CvSize win = cvSize( _search_window_x, _search_window_y );
    CvSize zero_zone = cvSize( _zero_zone_x, _zero_zone_y );
    CvTermCriteria criteria_csp = cvTermCriteria( CV_TERMCRIT_ITER,
                                                  _search_iterations,
                                                  0.0f );

    cvFindCornerSubPix( leftGrey, goodPoints[0], count,
                        win,zero_zone,
                        criteria_csp);
    //  printf("m: GFTT 2 %d\n",count);





    /*


    //Draw points on image
    int i, valid;
    for (i = valid = 0; i<count; i++)
    {
        cvCircle(leftGrey, cvPointFrom32f(goodPoints[0][i]), 3, CV_RGB(255,255,255), -1, 8,0);
        //   cvCircle(vdisp, cvPointFrom32f(goodPoints[1][i]), 3, CV_RGB(255,255,255), -1, 8,0);
        // cvCircle(rightGrey, cvPointFrom32f(goodPoints[1][i]), 3, CV_RGB(255,255,255), -1, 8,0);

        if ( !status[i] ) continue;
        goodPoints[0][valid] = goodPoints[0][i];
        goodPoints[2][valid++] = goodPoints[2][i];
        cvCircle( rightGrey, cvPointFrom32f(goodPoints[2][i]), 3, CV_RGB(0,255,0), -1, 8,0);
    }
*/

    //cvConvertScale( &E, &F, fabs(e[8]) > 0 ? 1./e[8] : 1 );
    //print_matrix("F=",&F);
    //for(int i=0; i<3; i++)
    //     printf("%.10f %.10f %.10f\n",cvmGet(&F,0,i),cvmGet(&F,1,i),cvmGet(&F,2,i));

    //now we define a random point somewhere in front of both cameras
    flags=0;
    if(zguess != AUV_NO_Z_GUESS )
        flags |= CV_LKFLOW_INITIAL_GUESSES;

    for ( int i=0; i < count; i++)
    {

        CvPoint2D32f tmpL = goodPoints[0][i];
        //CvPoint2D32f tmpR = right[i];//goodPoints[2][i];
        /*  cvmSet(triCoordL,0,i,tmpL.x);
                     cvmSet(triCoordL,1,i,tmpL.y);
                     cvmSet(triCoordR,0,i,tmpR.x);
                     cvmSet(triCoordR,1,i,tmpR.y);
             */

        //and we project this ONE point onto BOTH images
        double pointImg1_a[2] = { tmpL.x, tmpL.y};
        CvMat pointImg1 = cvMat(1,1,CV_64FC2,pointImg1_a);
        double pointImg2_a[2] = {0,0};
        CvMat pointImg2 = cvMat(1,1,CV_64FC2,pointImg2_a);


        /* double point_a[3] = {-0.289405, 0.323173, 2.549763};
       CvMat point = cvMat(1,1,CV_64FC3,point_a);

       //and we project this ONE point onto BOTH images
       double pointImg1_a[2] = { 0.0, 0.0};
       CvMat pointImg1 = cvMat(1,1,CV_64FC2,pointImg1_a);
       double pointImg2_a[2] = { 0.0, 0.0};
       CvMat pointImg2 = cvMat(1,1,CV_64FC2,pointImg2_a);

       cvProjectPoints2(&point, &RV1, &T1, &camInt1, &camDist1, &pointImg1);
       cvProjectPoints2(&point, &RV2, &T2, &camInt2, &camDist2, &pointImg2);
*/
        int flags = 0;



        if(zguess != AUV_NO_Z_GUESS ){
            flags |= CV_LKFLOW_INITIAL_GUESSES;

            //Convert pixel coords to normal coords
            double x_n = (pointImg1_a[0]-_calib.camera_calibs[0].ccx)/_calib.camera_calibs[0].fcx;
            double y_n = (pointImg1_a[1]-_calib.camera_calibs[0].ccy)/_calib.camera_calibs[0].fcy;
            double point_a[3] = { x_n*zguess,
                                  y_n*zguess,
                                  zguess};
            CvMat point = cvMat(1,1,CV_64FC3,point_a);
            cvProjectPoints2(&point, _RV2, _T2,_camInt2, _camDist2, &pointImg2);
            goodPoints[2][i].x=pointImg2_a[0];
            goodPoints[2][i].y=pointImg2_a[1];
        }

    }
    float error[count];
    for(int i=0; i< count; i++){
        double x= goodPoints[0][i].x;
        double y= goodPoints[0][i].y;
        if(!is_new_corner_valid( x,
                                 y,
                                 i,
                                 _min_distance,goodPoints[0],Lstatus,count ))

            Lstatus[i]=false;
        else
            Lstatus[i]=true;
    }

    int cnt1=0;
    for(int i=0; i< count; i++)
        if(Lstatus[i]){
            cnt1++;
        }
    ///           printf("%d %f %f\n",cnt1++,goodPoints[0][i].x,
    //               goodPoints[0][i].y);
    //printf("m: GFTT 3 %d %f\n",cnt1,min_distance);
    //printf("m: GFTT 4 %d\n",count);
    stats.total_init_feat=cnt1;

    cvZero(leftPyr);
    cvZero(rightPyr);
    CvSize win_size = cvSize( _track_window_x, _track_window_y );
    CvTermCriteria criteria = cvTermCriteria( CV_TERMCRIT_ITER| CV_TERMCRIT_EPS,
                                              _track_iterations,
                                              _track_epsilon  );
    cvCalcOpticalFlowPyrLK( leftGrey, rightGrey, leftPyr, rightPyr,
                            goodPoints[0], goodPoints[2], count, win_size, _pyramid_level, Rstatus, error,
                            criteria, flags );
    //printf("m: GFTT 4 %d\n",count);

    int validC=0;
    for(int i=0; i< count; i++){
        // printf("! %f %f\n",goodPoints[2][i].x,goodPoints[2][i].y);
        double x= goodPoints[2][i].x;
        double y= goodPoints[2][i].y;
        if(
                x< 0 || y< 0 || x > _calib.camera_calibs[0].width || y > _calib.camera_calibs[0].height
                ||           error[i] > _track_max_error )

            Rstatus[i]=false;
        else
            validC++;
    }

    //  printf("m: LK v %d\n",validC);

    //cvSaveImage("a1.png",leftGrey);  cvSaveImage("a2.png",rightGrey);;
    // CvMat imagePointsLeft = cvMat(1, count, CV_32FC2, &goodPoints[0][0] );

    //Assign point data to CvMat data in order to use cvUndistortPoints
    // CvMat imagePointsRight = cvMat(1, count, CV_32FC2, &goodPoints[2][0] );

    stats.total_matched_feat=triangulatePoints(count,goodPoints[0],goodPoints[2],Lstatus,Rstatus,stats,points);
    stats.total_tracking_fail=stats.total_init_feat-stats.total_matched_feat;

    //  printf("ctt %d\n",ctt);
    cvFree(&Lstatus);
    cvFree(&Rstatus);
    cvFree( &(goodPoints[0]));
    cvFree(  &(goodPoints[1]));
    cvFree(  &(goodPoints[2]));
    cvFree(  &(goodPoints[3]));

    /*  IplImage *tmpRI1=cvCreateImage(cvSize(leftGrey->width/2,leftGrey->height/2),IPL_DEPTH_8U,1);

                    IplImage *tmpLI1=cvCreateImage(cvSize(leftGrey->width/2,leftGrey->height/2),IPL_DEPTH_8U,1);

                    cvResize(leftGrey,tmpLI1);

                    cvResize(leftGrey,tmpRI1);
                    IplImage *tmpRI=cvCreateImage(cvSize(leftGrey->width/2,leftGrey->height/2),IPL_DEPTH_8U,3);

                    IplImage *tmpLI=cvCreateImage(cvSize(leftGrey->width/2,leftGrey->height/2),IPL_DEPTH_8U,3);
                    cvCvtColor(tmpLI1,tmpLI,CV_GRAY2RGB);
                    cvCvtColor(tmpRI1,tmpRI,CV_GRAY2RGB);

                    for (int i = 0; i<count; i++)
                    {
                        CvPoint p=cvPointFrom32f(goodPoints[0][i]);
                        p.x*=0.5;
                        p.y*=0.5;
                        if(!Lstatus[i])
                            cvCircle(tmpLI, p, 3, CV_RGB(255,0,0));
                        else
                            cvCircle(tmpLI, p, 3, CV_RGB(0,255,0));

                        //cvCircle(vdisp, cvPointFrom32f(goodPoints[1][i]), 3, CV_RGB(255,255,255), -1, 8,0);
                        //   cvCircle(leftGrey, cvPointFrom32f(goodPoints[0][i]), 3, CV_RGB(255,255,255), -1, 8,0);
                    }
                    int cnt=0;
                    for (int i = 0; i<count; i++)
                    {

                        CvPoint p=cvPointFrom32f(goodPoints[2][i]);
                        p.x*=0.5;
                        p.y*=0.5;
                        if(!Lstatus[i])
                            continue;
                        if(!Rstatus[i])
                            cvCircle(tmpRI, p, 3, CV_RGB(255,0,0));
                        else
                            cvCircle(tmpRI, p, 3, CV_RGB(0,255,0));

                        //cvCircle(vdisp, cvPointFrom32f(goodPoints[1][i]), 3, CV_RGB(255,255,255), -1, 8,0);
                        //   cvCircle(leftGrey, cvPointFrom32f(goodPoints[0][i]), 3, CV_RGB(255,255,255), -1, 8,0);
                    }


                */




}
int StereoEngine::triangulatePoints(int count,CvPoint2D32f*leftP,CvPoint2D32f*rightP,char* Lstatus,char* Rstatus,MatchStats &stats,std::list<osg::Vec3> &points){

    vector<CvPoint2D64f> undist_coords1;

    vector<CvPoint2D64f> undist_coords2;

    for ( int i=0; i < count; i++)
    {
        if(!Lstatus[i]||!Rstatus[i]){
            undist_coords1.push_back(cvPoint2D64f(0,
                                                  0));

            undist_coords2.push_back(cvPoint2D64f(0,
                                                  0));

        }else{

            CvPoint2D32f tmpL = leftP[i];
            CvPoint2D32f tmpR = rightP[i];


            //and we project this ONE point onto BOTH images
            double pointImg1_a[2] = { tmpL.x, tmpL.y};
            CvMat pointImg1 = cvMat(1,1,CV_64FC2,pointImg1_a);
            double pointImg2_a[2] = { tmpR.x,tmpR.y};
            CvMat pointImg2 = cvMat(1,1,CV_64FC2,pointImg2_a);


            /**
        * 3D localization
        * now we have all parameters we also get from our stereo frame
        * - intrinsic and extrinsic of cameras
        * - coordinates of a point in both images
        *
        * so we calculate the 3d position of this point in our system
        */

            //first we need do the inverse of the projective transformation to get
            //the points coordinates in the cameras normal space
            cvUndistortPoints(&pointImg1,&pointImg1,_camInt1,_camDist1);
            cvUndistortPoints(&pointImg2,&pointImg2,_camInt2,_camDist2);
            undist_coords1.push_back(cvPoint2D64f(pointImg1.data.db[0],
                                                  pointImg1.data.db[1]));

            undist_coords2.push_back(cvPoint2D64f(pointImg2.data.db[0],
                                                  pointImg2.data.db[1]));
        }
        //be sure the point's are saved in right matrix format 2xN 1 channel

    }
    stats.total_epi_fail=apply_epipolar_constraints(_F,_l_to_r_max_epipolar_dist,undist_coords1,undist_coords2,Rstatus,Lstatus,_calib);

    int ctt=0;
    for ( int i=0; i < count; i++)
    {
        if(!Rstatus[i]|| !Lstatus[i])
            continue;


        CvMat *_pointImg1 = cvCreateMat(2,1,CV_64FC1);
        CvMat *_pointImg2 = cvCreateMat(2,1,CV_64FC1);
        CV_MAT_ELEM( *_pointImg1, double, 0, 0 ) = undist_coords1[i].x;
        CV_MAT_ELEM( *_pointImg1, double, 1, 0 ) = undist_coords1[i].y;
        CV_MAT_ELEM( *_pointImg2, double, 0, 0 ) = undist_coords2[i].x;
        CV_MAT_ELEM( *_pointImg2, double, 1, 0 ) = undist_coords2[i].y;

        //triangulate both projections to find real point position
        //!!! all parameters must be double type
        CvMat *point3D = cvCreateMat(4,1,CV_64F) ;
        cvTriangulatePoints(_cam1Ext, _cam2Ext, _pointImg1, _pointImg2, point3D);

        //to get the real position we need to do also a homogeneous division
        point3D->data.db[0] /= point3D->data.db[3];
        point3D->data.db[1] /= point3D->data.db[3];
        point3D->data.db[2] /= point3D->data.db[3];
        ctt++;
        //et voila
        //  cout << point3D->data.db[0] << " " << point3D->data.db[1] << " " << point3D->data.db[2];
        //cout << " "<<tmpL.x<< " "<< tmpL.y<< " "<< tmpR.x<< " "<<tmpR.y << endl;
        osg::Vec3 pt(point3D->data.db[0],point3D->data.db[1],point3D->data.db[2]);
        if(pt.z() > 0.0 && pt.length2() < max_triangulation_len*max_triangulation_len)
            points.push_back(pt);
        cvReleaseMat(&_pointImg1);
        cvReleaseMat(&_pointImg2);
        cvReleaseMat(&point3D);


    }
    return ctt;
}

StereoStatusFlag StereoEngine::processPair(const std::string basedir,const std::string left_file_name,const std::string &right_file_name ,const osg::Matrix &mat,osg::BoundingBox &bbox,MatchStats &stats,const double feature_depth_guess,bool cache_img,bool use_cached,bool force_keypoint){

    char cachedtexdir[1024];
    char meshfilename[1024];
    char tcmeshfilename[1024];
    char texfilename[1024];
    string imgadd= ((left_file_name.size() == osgDB::getSimpleFileName(left_file_name).size())? "/img/": "/");
    string imgdir=basedir+imgadd;
    StereoStatusFlag statusFlag=STEREO_OK;
    string cached_dir=basedir+"/tmp/cache-mesh-feat";
    checkmkdir(cached_dir);

    string outputdir="tmp/mesh-agg/";
    checkmkdir(outputdir);
    if(cache_img){
        sprintf(cachedtexdir,"%s/cache-tex-%d",basedir.c_str(),tex_size);
        checkmkdir(cachedtexdir);
    }
    sprintf(meshfilename,"%s/surface-%s.ply",cached_dir.c_str(),osgDB::getSimpleFileName(osgDB::getNameLessExtension(left_file_name)).c_str());

    sprintf(tcmeshfilename,"%s/surface-%s.tc.ply",outputdir.c_str(),osgDB::getSimpleFileName(osgDB::getNameLessExtension(left_file_name)).c_str());
    sprintf(texfilename,"%s/%s.png",
            cachedtexdir,osgDB::getSimpleFileName(osgDB::getNameLessExtension(left_file_name)).c_str());
    bool cachedMesh= use_cached ? osgDB::fileExists(meshfilename) : false;
    bool cachedTex=cache_img ? osgDB::fileExists(texfilename) : true;

    IplImage *left_frame=NULL;
    IplImage *color_image=NULL;
    IplImage *right_frame=NULL;
    /*unsigned int left_frame_id=frame_id++;
    unsigned int right_frame_id=frame_id++;*/
    double load_start_time;
    double load_end_time;

    if(!cachedTex || !cachedMesh){

        // Create the stereo feature finder
        //
        load_start_time = get_time( );
        //
        // Load the images
        //
        if(verbose)
            cout << "Loading images..." << endl;

        if( !get_stereo_pair(
                    imgdir,
                    left_frame, right_frame,color_image,
                    left_file_name, right_file_name ) )
        {
            return FAIL_OTHER;
        }
        load_end_time = get_time( );
        if(verbose)
            cout << "Loaded images: " << left_file_name << " and "
                 << right_file_name << endl;

        if(!cachedTex)
            cacheImage(color_image,texfilename,tex_size);
    }

    TriMesh *mesh=NULL;
    TriMesh::verbose=0;
    bool no_depth=false;
    GtsSurface *surf=NULL;
    int mesh_verts=0;
    std::list<osg::Vec3> new_method_pts;

    if(!cachedMesh){
        //printf("Not cached creating\n");

        // if(feature_depth_guess == AUV_NO_Z_GUESS && !no_depth)
        // feature_depth_guess = name.alt;

        GPtrArray *localV=NULL;
        if(!use_dense_stereo){
            //
            // Find the features
            //
#if OLD_AUV_CODE
            list<Feature *> features;

            list<libsnapper::Stereo_Feature_Estimate> feature_positions;
            stats=finder->find( left_frame,
                                right_frame,
                                left_frame_id,
                                right_frame_id,
                                max_feature_count,
                                features,
                                feature_depth_guess );
            Stereo_Reference_Frame ref_frame = STEREO_LEFT_CAMERA;
            list<libplankton::Vector>::iterator viter;
#else
            int MAX_COUNT=100000;
            sparseDepth(left_frame,right_frame,stats,MAX_COUNT,new_method_pts,feature_depth_guess);


#endif
            double per_failed=(stats.total_epi_fail+stats.total_tracking_fail)/(double)stats.total_matched_feat;
            //            cout << thresh_per_rejected_output_debug << " " << per_failed<<endl;
            //cout <<stats.total_epi_fail<< " "<<stats.total_tracking_fail << " "<<(double)stats.total_matched_feat<<endl;
            if(thresh_per_rejected_output_debug > 0 && per_failed > thresh_per_rejected_output_debug){
                statusFlag=FAIL_FEAT_THRESH;
            }


            localV = g_ptr_array_new ();

            GtsRange r;
            gts_range_init(&r);
            TVertex *vert;
            //
            // Triangulate the features if requested
            //
            //Fallback slow matching

            if(statusFlag != STEREO_OK &&  new_method_pts.size() < min_feat_rerun || force_keypoint){
                statusFlag=FALLBACK_KEYPOINT;
                //    printf("Orig points %d\n",new_method_pts.size());
                int old_pts_cnt=new_method_pts.size();
                new_method_pts.clear();
                keypointDepth(left_frame,right_frame,left_file_name,stats,new_method_pts);
                printf("Fallback LK pts: %d LK Fail Rate %d%% Feat Pts: %d \n",old_pts_cnt,(int)round(100*per_failed),new_method_pts.size());
            }

#ifdef OLD_AUV_CODE

            /*Matrix pose_cov(4,4);
      get_cov_mat(cov_file,pose_cov);
    */
            //cout << "Cov " << pose_cov << "Pose "<< veh_pose<<endl;
            stereo_triangulate( *_auv_stereo_calib,
                                ref_frame,
                                features,
                                left_frame_id,
                                right_frame_id,
                                NULL,//image_coord_covar,
                                feature_positions );

            // printf("Feature Pos %d %d %d\n",feature_positions.size(),features.size(),new_method_pts.size());
            //  cout << "1 "<< stats <<endl<<"2 "<<stats2<<endl;


            list<Stereo_Feature_Estimate>::iterator litr;

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
                libplankton::Vector max_eig_v(3);
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

        }
#else
            std::list<osg::Vec3>::iterator litr;
            for( litr  = new_method_pts.begin( ) ;
                 litr != new_method_pts.end( ) ;
                 litr++ )
            {   vert=(TVertex*)  gts_vertex_new (t_vertex_class (),
                                                 (*litr)[0],(*litr)[1],(*litr)[2]);
                //printf("%f %f %f\n", litr->x[0],litr->x[1],litr->x[2]);

                g_ptr_array_add(localV,GTS_VERTEX(vert));


            }
#endif

            mesh_verts=localV->len;
            //   printf("Writing points %d %d\n",mesh_verts,new_method_pts.size());

            double mult=0.00;
            if(mesh_verts){
                surf = mesh_proc::auv_mesh_pts(localV,mult,0);
                FILE *fp = fopen(meshfilename, "w" );
                if(!fp){
                    fprintf(stderr,"\nWARNING - Can't open mesh file %s\n",meshfilename);
                    fflush(stderr);

                    // clean up
                    cvReleaseImage( &left_frame );
                    cvReleaseImage( &right_frame );
                    cvReleaseImage( &color_image);
                    return FAIL_OTHER;
                }
                bool have_cov_file=false;
                mesh_proc::auv_write_ply(surf, fp,have_cov_file,"test");
                fflush(fp);
                fclose(fp);
                //Destory Surf
                if(surf)
                    gts_object_destroy (GTS_OBJECT (surf));


                // delete the vertices
                g_ptr_array_free( localV, true );
            }
        }else{

            sdense->dense_stereo(left_frame,right_frame);
            std::vector<osg::Vec3> points;
            IplImage *mask =cvCreateImage(cvSize(sdense->getDisp16()->width,sdense->getDisp16()->height),IPL_DEPTH_8U,1);
            cvZero(mask);
            sdense->get_points(points,mask);
            mesh=mesh_proc::get_dense_grid(mask,points);
            cvReleaseImage(&mask);
            mesh_verts = points.size();
            if(mesh && mesh_verts)
                mesh->write(meshfilename);

        }
    }

    if(!cachedMesh){

        if( display_debug_images && pause_after_each_frame ){
            cout << left_file_name<<endl;
            cvWaitKey( 0 );
        }
        else if( display_debug_images )
            cvWaitKey( 100 );

    }


    mesh = TriMesh::read(meshfilename);
    if(!mesh)
        fprintf(stderr,"\nWARNING - mesh null after doing dense stereo\n");

    if(!mesh){
        fprintf(stderr,"\nWARNING - No cached mesh file %s\n",meshfilename);
        fflush(stderr);

        // clean up
        if(!cachedTex || !cachedMesh){
            if(left_frame){
                cvReleaseImage( &left_frame );
                left_frame=NULL;
            }
            if(right_frame){
                cvReleaseImage( &right_frame );
                right_frame=NULL;
            }
            if(color_image){
                cvReleaseImage( &color_image );
                color_image=NULL;
            }
        }
        return FAIL_OTHER;
    }
    unsigned int triFeat=mesh->vertices.size();

    mesh_proc::edge_len_thresh(mesh,edgethresh);
    unsigned int postEdgeThreshTriFeat=mesh->vertices.size();
    // printf("%d/%d %f %f\n",(triFeat-postEdgeThreshTriFeat),triFeat,(triFeat-postEdgeThreshTriFeat)/(double)triFeat,
    //   thresh_per_rejected_output_debug);
    stats.total_tri_fail=triFeat-postEdgeThreshTriFeat;
    if(( stats.total_tri_fail)/(double)triFeat > thresh_per_rejected_output_debug || postEdgeThreshTriFeat < minFeatPerFrameThresh){
        char tmp[1024];
        sprintf(tmp,"debug/%s-edgefail.txt",osgDB::getSimpleFileName(osgDB::getNameLessExtension(left_file_name)).c_str());

        FILE *fp=fopen(tmp,"w");
        fprintf(fp,"Init,Matched,PostEpipolar,Tri,TriEdge\n");
        fprintf(fp,"%d,%d,%d,%d,%d\n",stats.total_init_feat,stats.total_matched_feat,stats.total_accepted_feat,triFeat,postEdgeThreshTriFeat);
        fclose(fp);
    }

    if(!mesh->faces.size()){
        fprintf(stderr,"\nWARNING - Mesh empty after edge threshold clipping %s\n",meshfilename);
        fflush(stderr);

        // clean up
        cvReleaseImage( &left_frame );
        cvReleaseImage( &right_frame );
        cvReleaseImage( &color_image);
        return FAIL_TRI_EDGE_THRESH;
    }

    /*xform xf(mat(0,0),mat(1,0),mat(2,0),mat(3,0),
       mat(0,1),mat(1,1),mat(2,1),mat(3,1),
       mat(0,2),mat(1,2),mat(2,2),mat(3,2),
           mat(0,3),mat(1,3),mat(2,3),mat(3,3));
*/
    xform xf(mat(0,0),mat(0,1),mat(0,2),mat(0,3),
             mat(1,0),mat(1,1),mat(1,2),mat(2,3),
             mat(2,0),mat(2,1),mat(2,2),mat(2,3),
             mat(3,0),mat(3,1),mat(3,2),mat(3,3));


    stats.total_accepted_feat=mesh->vertices.size();

    apply_xform(mesh,xf);
    mesh->need_bbox();
    osg::Vec3 maxbb(mesh->bbox.max[0],mesh->bbox.max[1],mesh->bbox.max[2]);

    osg::Vec3 minbb(mesh->bbox.min[0],mesh->bbox.min[1],mesh->bbox.min[2]);
    osg::BoundingBox tbbox;
    tbbox.expandBy(maxbb);
    tbbox.expandBy(minbb);
    std::stringstream str;
    str << minbb << " " << maxbb;
    mesh->write(tcmeshfilename,str.str().c_str());
    bbox = tbbox;
    stats.total_faces=mesh->faces.size();
#ifdef OSG_TRI
    osg::ref_ptr<osg::DrawElementsUInt> tris;
    osg::ref_ptr<osg::Vec3Array> points;
    osg::ref_ptr<osg::Geometry> gm;
    osg::ref_ptr<osg::Geode> geode;
    if(cachedMesh){

        osg::ref_ptr<osg::Node> node;
        //Not thread Safe look to replacing if actually slower
        {
            OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_osgDBMutex);

            node=osgDB::readNodeFile(meshfilename);
        }
        if(!node.valid()){
            fprintf(stderr,"Can't load ply file\n");
            cachedMesh=false;
            return false;
        }else{
            geode=node->asGeode();
            if(!geode.valid()){
                fprintf(stderr,"Can't load ply file not GEODE\n");
                cachedMesh=false;
            }
            osg::Drawable *drawable = geode->getDrawable(0);
            gm = dynamic_cast< osg::Geometry*>(drawable);
            if(!gm.valid()){
                fprintf(stderr,"Can't load ply file not GEOM\n");
                cachedMesh=false;
            }
            points=static_cast<osg::Vec3Array*>(gm->getVertexArray());
            tris=static_cast<osg::DrawElementsUInt *>(gm->getPrimitiveSet(0));

            if(!points || !tris){
                fprintf(stderr,"Can't load ply file no points\n");
                cachedMesh=false;
            }
            if(points->size()< 3 || !tris->size()){
                if(!cachedTex){
                    cvReleaseImage( &left_frame );
                    cvReleaseImage( &right_frame );
                    if(color_image)
                        cvReleaseImage( &color_image );

                }
                return FAIL_FEAT_THRESH;
            }
        }
    }
    if(!cachedMesh){


        //
        // Load the stereo camera calibration
        //

        std::list<osg::Vec3> pts_exp;
        double find_start_time = get_time();
        sparseDepth(left_frame,right_frame,max_feature_count,pts_exp,feature_depth_guess);
        double find_end_time = get_time( );
        //
        // Display useful info
        //
        if(verbose){
            cout << endl;
            cout << "Left Image : " << left_file_name << endl;
            cout << "Right Image: " << right_file_name << endl;
            cout << endl;

            cout << "Image loading time     : " << load_end_time-load_start_time << endl;
            cout << "Feature finding time   : " << find_end_time-find_start_time << endl;
        }
        double tri_start_time = get_time( );
        //
        // Triangulate the features if requested
        //


        points = new osg::Vec3Array;
        unique(pts_exp.begin(),pts_exp.end(),is_same_vec3_xy());
        for(std::list<osg::Vec3>::iterator itr=pts_exp.begin(); itr!=pts_exp.end(); itr++)
            points->push_back(*itr);
        osg::ref_ptr<osgUtil::DelaunayTriangulator> trig = new osgUtil::DelaunayTriangulator();
        trig->setInputPointArray(points);
        if(points->size()>3)
            trig->triangulate();
        else{
            if(!cachedTex || !cachedMesh){
                cvReleaseImage( &left_frame );
                cvReleaseImage( &right_frame );
                if(color_image)
                    cvReleaseImage( &color_image );

            }
            return FAIL_OTHER;
        }
        geode = new osg::Geode();
        gm = new osg::Geometry;
        gm->setVertexArray(points);
        tris=trig->getTriangles();
        gm->addPrimitiveSet(tris);
        geode->addDrawable(gm);

        {
            std::ofstream f(meshfilename);
            if(!f.good()){
                fprintf(stderr,"Can't open %s",meshfilename);
                if(!cachedTex || !cachedMesh){
                    cvReleaseImage( &left_frame );
                    cvReleaseImage( &right_frame );
                    if(color_image)
                        cvReleaseImage( &color_image );

                }
                return FAIL_OTHER;
            }
            PLYWriterNodeVisitor nv(f,NULL,NULL);
            geode->accept(nv);
        }
        double tri_end_time = get_time( );

        if(verbose)
        {                cvReleaseImage( &color_frame);

            cout << "Trianulation finding time   "<< (tri_end_time-tri_start_time) <<endl;
            cout << endl;
            cout << "------------------------------------" << endl;
            cout << endl;
        }
        //
        // Pause between frames if requested.
        //
        if( display_debug_images && pause_after_each_frame )
            cvWaitKey( 0 );
        else if( display_debug_images )
            cvWaitKey( 100 );
    }

    osg::ref_ptr<osg::DrawElementsUInt> validTri=new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES);
    for(int i=0; i< (int)tris->size()-2; i+=3){
        unsigned int i0=tris->at(i);
        unsigned int i1=tris->at(i+1);
        unsigned int i2=tris->at(i+2);
        const osg::Vec3 &v0=points->at(i0);
        const osg::Vec3 &v1=points->at(i1);
        const osg::Vec3 &v2=points->at(i2);
        double thresh2=edgethresh*edgethresh;
        double d01 = (v0-v1).length2();
        float d12 = (v1 -v2).length2();
        float d20 = (v2-v0).length2();
        if(d01 < thresh2 && d12 < thresh2 &&d20 <thresh2){
            validTri->push_back(i0);
            validTri->push_back(i1);
            validTri->push_back(i2);
        }
    }

    vector<bool> unused(points->size(), true);
    for (int i = 0; i < (int)validTri->size()-2; i+=3) {
        unused[validTri->at(i)] = false;
        unused[validTri->at(i+1)] = false;
        unused[validTri->at(i+2)] = false;
    }
    osg::ref_ptr<osg::Vec3Array> validPts=new osg::Vec3Array;
    unsigned int cnt=0;
    std::map<unsigned int,unsigned int> remap;
    for(int i=0; i< (int)points->size(); i++){
        if(!unused[i]){
            validPts->push_back(points->at(i));
            remap[i]=cnt++;
        }
    }
    for (int i = 0; i < (int)validTri->size(); i++) {
        validTri->at(i)=remap[validTri->at(i)];
    }

    gm->setPrimitiveSet(0,validTri);
    gm->setVertexArray(validPts);
    osg::ref_ptr<osg::MatrixTransform>xform = new osg::MatrixTransform;
    xform->setDataVariance( osg::Object::STATIC );
    xform->setMatrix(mat);
    xform->addChild(geode);
    osgUtil::Optimizer::FlattenStaticTransformsVisitor fstv(NULL);
    xform->accept(fstv);
    fstv.removeTransforms(xform);
    osg::ComputeBoundsVisitor cbbv(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
    geode->accept(cbbv);
    bbox = cbbv.getBoundingBox();

    {
        std::stringstream str;
        str << bbox._min << " " << bbox._max;
        std::ofstream f(tcmeshfilename);
        if(!f.good()){
            fprintf(stderr,"Can't write %s\n",tcmeshfilename);
            if(!cachedTex || !cachedMesh){
                cvReleaseImage( &left_frame );
                cvReleaseImage( &right_frame );
                if(color_image)
                    cvReleaseImage( &color_image );

            }
            return FAIL_OTHER;
        }
        PLYWriterNodeVisitor nv(f,NULL,NULL,str.str());
        geode->accept(nv);
    }

#endif

    //
    // Clean-up
    //
    if(!cachedTex || !cachedMesh){
        if(left_frame){
            cvReleaseImage( &left_frame );
            left_frame=NULL;
        }
        if(right_frame){
            cvReleaseImage( &right_frame );
            right_frame=NULL;
        }
        if(color_image){
            cvReleaseImage( &color_image );
            color_image=NULL;
        }
    }
    if(mesh)
        delete mesh;
    return statusFlag;
}

// vim:ts=4:sw=4
