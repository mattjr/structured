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
#define USE_AUV_LIBS
#ifdef USE_AUV_LIBS
#include "auv_stereo_geometry_compat.hpp"
#include "auv_stereo_keypoint_finder.hpp"
#include "adt_image_norm.hpp"
#include "auv_stereo_dense.hpp"
#include "gts.h"
#include "TriMesh.h"
#include "TriMesh_algo.h"
#include "XForm.h"
#include "mesh_proc.hpp"
//#include "auv_mesh_io.hpp"
#include "auv_geometry.hpp"
#include "RobustMatcher.h"
using namespace libsnapper;
#endif
using namespace std;
using namespace cv;
//!
//! Calculate the mean and variance of the pixel intensities in an image
//!
static void calc_intensity_mean_and_var( const IplImage *image,
                                         double &mean,
                                         double &var )
{

   // FIXME: Only sample a reduced set of pixels instead of the whole image

   // Create a histogram counting the frequency of each pixel intensity value
   IplImage *image_array[] = {const_cast<IplImage*>(image)};
   int hist_size[] = {256};
   float range[] = {0,256}; // 256 not 255 is apparently correct
   float *range_array[] = { range };

   CvHistogram* hist = cvCreateHist( 1, hist_size, CV_HIST_ARRAY, range_array, 1 );
   cvCalcHist( image_array, hist, 0, 0 );


   // Use the histogram to calculate the mean and variance
   unsigned int num_samples = 0;
   mean = 0;
   for( unsigned int i=0 ; i <= 255 ; i++ )
   {
      unsigned int freq = (unsigned int)cvGetReal1D( hist->bins, i );
      mean += freq*i;
      num_samples += freq;
   }
   mean = mean/num_samples;
   var = 0;
   for( unsigned int i=0 ; i <= 255 ; i++ )
   {
      unsigned int freq = (unsigned int)cvGetReal1D( hist->bins, i );
      var += freq*pow(i-mean,2);
   }
   var = var/(num_samples-1);

   // Clean up
   cvReleaseHist( &hist );
}


//!
//! Perform linear scaling on each pixel intensity value
//!   output = scale*input + offset
//!
static void linearly_scale_intensities( double scale,
                                        double offset,
                                        IplImage *image )
{
   // Create a look-up table mapping old intensity values to new values
   unsigned char lut_data[256];
   CvMat* lut_array = cvCreateMatHeader(1, 256, CV_8UC1);
   cvSetData( lut_array, &lut_data, 0 );
   for( unsigned int i=0 ; i <= 255 ; i++ )
   {
      double value = scale*i+offset;
      if( value <= 0 )
         lut_data[i] = 0;
      else if ( value >= 255 )
         lut_data[i] = 255;
      else
         lut_data[i] = (unsigned char)value;
   }

   // Apply the look-up table to the image
   cvLUT( image, image, &lut_array[0] );
   cvReleaseMat( &lut_array );
}



//!
//! Scale the image pixel intensities to have a desired mean and variance.
//!
//! NOTE: This is currently used as a quick workaround for the fact that when
//!       using the pixel equalisation function which assumes the vehicle is at
//!       a specified altitude, and the vehicle is actually at a higher
//!       altitude, the resulting images are very dark.
//!
//! If a better colour equalisation method that takes the depth of a scene
//! into account is used, this workaround should become redundant.
//!
void normalise_image( double desired_mean,
                                      double desired_var,
                                      IplImage *image )
{
   double mean = 0;
   double var  = 0;
   calc_intensity_mean_and_var( image, mean, var );

   double scale = sqrt( desired_var/var );
   double offset = desired_mean - mean*scale;
   linearly_scale_intensities( scale, offset, image );
}

int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);
    if(argc <5){
        fprintf(stderr,"Usage : leftimage rightimage calib meshfilename\n");
        exit(-1);
    }
    vector<KeyPoint> keypoints1, keypoints2;
    IplImage *left_frame=cvLoadImage(argv[1],CV_LOAD_IMAGE_GRAYSCALE);
    IplImage *right_frame=cvLoadImage(argv[2],CV_LOAD_IMAGE_GRAYSCALE);
   int normalised_mean=128;
   int normalised_var=400;
       //  normalise_image( normalised_mean, normalised_var, left_frame  );
        // normalise_image( normalised_mean, normalised_var, right_frame );



    /*  detector.detect(left_frame, keypoints1);
detector.detect(right_frame, keypoints2);
*/
    // computing descriptors
    /*  SurfDescriptorExtractor extractor;
Mat descriptors1, descriptors2;
extractor.compute(left_frame, keypoints1, descriptors1);
extractor.compute(right_frame, keypoints2, descriptors2);*/
    IplImage *left_undist_image=cvCreateImage(cvSize(left_frame->width,left_frame->height),IPL_DEPTH_8U,1);
    IplImage *right_undist_image=cvCreateImage(cvSize(right_frame->width,right_frame->height),IPL_DEPTH_8U,1);
    IplImage *invalidMask=cvCreateImage(cvSize(right_frame->width,right_frame->height),IPL_DEPTH_8U,1);
    IplImage *tmp=cvCreateImage(cvSize(right_frame->width,right_frame->height),IPL_DEPTH_8U,1);

    int mesh_verts=0;

      list<libsnapper::Stereo_Feature_Estimate> feature_positions;
      GPtrArray *localV=NULL;
      localV = g_ptr_array_new ();


    libsnapper::Undistort_Data *undist_left;
    libsnapper::Undistort_Data *undist_right;
    libsnapper::Stereo_Calib _auv_stereo_calib(argv[3]);

        undist_left=new Undistort_Data(_auv_stereo_calib.left_calib,left_frame,true);

        undist_right=new Undistort_Data(_auv_stereo_calib.right_calib,right_frame,true);

    undistort_image(*undist_left,left_frame,left_undist_image);
    undistort_image(*undist_right,right_frame,right_undist_image);
    cvCmpS( left_undist_image, 1.0, invalidMask,  CV_CMP_LT );
    cvCmpS( right_undist_image, 1.0, tmp,  CV_CMP_LT );
    cvAdd(invalidMask,tmp,invalidMask);
    cvReleaseImage(&tmp);
    RobustMatcher matcher(5000,6000,0.65,4.0);
    vector<DMatch> matches;
    matcher.match(left_undist_image,right_undist_image,matches,keypoints1, keypoints2);
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
    std::vector<CvPoint2D64f>  left_coords;
    std::vector<CvPoint2D64f>  right_coords;
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
                //   printf("bad %f %f -- %f %f\n",l.x,l.y,r.x,r.y);
                // cvShowImage( "Good Matches", invalidMask );
                //  waitKey(0);
                if( CV_IMAGE_ELEM(invalidMask,uchar,(int)l.y,(int)l.x) ==0 &&  CV_IMAGE_ELEM(invalidMask,uchar,(int)r.y,(int)r.x) == 0){
                    left_coords.push_back(l);
                    right_coords.push_back(r);
                    good_matches.push_back( matches[i]);
                }

            }
        }
    }
    cout << "Number of matched points (after dist clean): "<<good_matches.size()<<endl;
    /*apply_epipolar_constraints(F,2.0,right_coords,left_coords,status,_calib);
 for(int i=0; i< status.size(); i++){
     if(status[i]){
         good_matches.push_back( matches[i]);

     }
 }*/
    //free(status);
    if(1){//_writeDebugImages){

        Mat img_matches;
        drawMatches( left_undist_image, keypoints1, right_undist_image, keypoints2,
                     good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                     vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        //-- Show detected matches
         imshow( "Good Matches", img_matches );

        /*string debugfilename=string("debug/"+osgDB::getSimpleFileName(left_file_name)+".surf.png");
        // cout <<debugfilename <<endl;
        if(!imwrite(debugfilename.c_str(),img_matches))
            fprintf(stderr,"Failed to write debug image %s\n",debugfilename.c_str());*/
         waitKey(0);
    }
    cvReleaseImage(&left_undist_image);
    cvReleaseImage(&right_undist_image);
    cvReleaseImage(&invalidMask);
    Stereo_Reference_Frame ref_frame = STEREO_LEFT_CAMERA;
    TVertex *vert;
    GtsSurface *surf=NULL;

    std::vector<bool>          valid;
    std::vector<libplankton::Vector> feats;
    stereo_triangulate( _auv_stereo_calib,
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

    mesh_verts=localV->len;
    double mult=0.00;
    if(mesh_verts){
        surf = mesh_proc::auv_mesh_pts(localV,mult,0);
        FILE *fp = fopen(argv[4], "w" );
        if(!fp){
            fprintf(stderr,"\nWARNING - Can't open mesh file %s\n",argv[4]);
            fflush(stderr);

            // clean up
            cvReleaseImage( &left_frame );
            cvReleaseImage( &right_frame );
            //cvReleaseImage( &color_image);
            return -1;
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
}
