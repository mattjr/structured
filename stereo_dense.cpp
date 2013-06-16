/* * structured - Tools for the Generation and Visualization of Large-scale
 * Three-dimensional Reconstructions from Image Data. This software includes
 * source code from other projects, which is subject to different licensing,
 * see COPYING for details. If this project is used for research see COPYING
 * for making the appropriate citations.
 * Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
 *
 * This file is part of structured.
 *
 * structured is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * structured is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with structured.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <iostream>
#include <iomanip>
#include "calibFile.h"
#include "image_rect.hpp"
#include "stereo_dense.hpp"

#include <osg/Vec4>


void
make_color_disp_image(IplImage *disp, int ndisp,  IplImage *outI,int skip);


#define LEFT_DEBUG_IMAGE_NAME  "Stereo Corner Finder Left"
#define RIGHT_DEBUG_IMAGE_NAME "Stereo Corner Finder Right"
#define USE_DENSE_STEREO
using namespace std;



//!
//! Create a Stereo_Dense object
//!
Stereo_Dense::Stereo_Dense( const Config_File &config_file,
                                            double image_scale,
                                            const StereoCalib *calib  )
   : 
     image_scale( image_scale ),
     calib( calib ),
     left_calib( NULL ),
     right_calib( NULL ),
     _rect(NULL)
     

{

  dense_config  = load_dense_config ( config_file );
   dense_config.width=calib->camera_calibs[0].width;
   dense_config.height=calib->camera_calibs[0].height;
   if( calib != NULL )
   {
      left_calib = &calib->camera_calibs[0];
      right_calib = &calib->camera_calibs[1];
   } 
   scaled_size.width  = (int)(dense_config.width*image_scale  );
   scaled_size.height = (int)(dense_config.height*image_scale);

   strcpy(dispname,"disp-0000.png");
   config_file.get_value( "SD_METHOD"     , dense_config.method   ,"opencvBM"   ); 
   config_file.get_value( "SD_SHOW_DEBUG_IMAGES", show_debug_images,false );
   config_file.get_value( "SD_SAVE_DEBUG_IMAGES", save_debug_images ,false);
   config_file.get_value( "SD_MINDISP", dense_config.minDisp,-64 );
  dense_config.verbose=1;
  if(dense_config.method == "opencvBM"){
     BMState = cvCreateStereoBMState(0,dense_config.num_disp);
     BMState->minDisparity=dense_config.minDisp;
     BMState->SADWindowSize=dense_config.corr_window;

     //     printf("Min Disp %d Corr Window %d Num Disp %d\n",dense_config.minDisp,dense_config.corr_window,dense_config.num_disp);
     /*   assert(BMState != 0);
     BMState->preFilterSize=41;
     BMState->preFilterCap=31;
     BMState->SADWindowSize=dense_config.corr_window;
     BMState->minDisparity=-64;
     BMState->numberOfDisparities=dense_config.num_disp;
     BMState->textureThreshold=10;
     BMState->uniquenessRatio=15;*/
   }/* else if(dense_config.method == "opencvGC"){
     GCstate = cvCreateStereoGCState( 128, 5 );  
     GCstate->minDisparity=-64;
     //assert(GCState != 0);    
   }*/
     
   
   // printf("Scaled Height %d %d \n",scaled_size.width,scaled_size.height);
   scaled_left  = cvCreateImage( scaled_size, IPL_DEPTH_8U, 1 );
   scaled_right = cvCreateImage( scaled_size, IPL_DEPTH_8U, 1 );
   
   scaled_color = cvCreateImage( scaled_size, IPL_DEPTH_8U, 3);
   scaled_mask = cvCreateImage( scaled_size, IPL_DEPTH_8U, 1 );
   tmp = cvCreateImage( scaled_size, IPL_DEPTH_8U, 1 );
   rect_left  = cvCreateImage( cvSize(dense_config.width,dense_config.height), IPL_DEPTH_8U, 1 );
   rect_right  = cvCreateImage( cvSize(dense_config.width,dense_config.height), IPL_DEPTH_8U, 1 );
 rect_color  = cvCreateImage( cvSize(dense_config.width,dense_config.height), IPL_DEPTH_8U, 3 );
   invalid_mask  = cvCreateImage( cvSize(dense_config.width,dense_config.height), IPL_DEPTH_8U, 1 );

   
   disp = cvCreateImage( scaled_size, IPL_DEPTH_32F, 1 );
   disp8 = cvCreateImage( scaled_size, IPL_DEPTH_8U, 1 );
   disp16 = cvCreateImage( scaled_size, IPL_DEPTH_16S, 1 );
   disp16R = cvCreateImage( scaled_size, IPL_DEPTH_16S, 1 );
   Ixyz=cvCreateImage( scaled_size, IPL_DEPTH_32F, 3 );
}                      



//!
//! Destroy a Stereo_Dense object
//!
Stereo_Dense::~Stereo_Dense( void )
{
  cvReleaseImage( &disp);
  cvReleaseImage( &disp8);

   cvReleaseImage( &scaled_right);
   cvReleaseImage( &scaled_left);
   if(scaled_color)
     cvReleaseImage( &scaled_color);


}

Dense_Config::Dense_Config( void )
{

  m_iHoropter=0;  

  m_iDisparityImageScale=0;
  m_iSADThreshold=0;
  
  xOffset=0;
  yOffset=0;
  m_bPropagateStereo  = false;
 

   // parameters controlled via command-line options:
     num_disp = 16;           // disparity levels (d = 0 .. num_disp-1)
     birchfield = 0;    // use Birchfield/Tomasi costs
     squaredDiffs = 0;  // use squared differences (absolute differences by default)
     truncDiffs = 255;  // truncated differences (before squaring), by default not
     MRFalg = 0;        // 0-ICM, 1-GC/expansion (default), 2-GC/swap, 3-TRWS, 4-BPS, 5-BPM, 9-all
     smoothexp = 1;     // exponent of smoothness term: 1 (default) or 2, i.e. L1 or L2 norm
     smoothmax = 2;     // maximum value of smoothness term (2 by default)
     lambda = 20;       // weight of smoothness term (20 by default)
     gradThresh = -1;   // intensity gradient cue threshold, by default none
     gradPenalty = 2;   // if grad < gradThresh, multiply smoothness cost by this
     outscale = -1;     // scale factor for disparities; -1 means full range 255.0/(num_disp-1)
     dump_timing=false;
     minDisp=-32;
}

//!
//! Load the configuration for the object used to locate corners in the 
//! left frame.
//!
Dense_Config 
Stereo_Dense::load_dense_config( const Config_File &config_file ) const
{
   Dense_Config dense_config;
  
   //   config_file.get_value( "SD_MIN_RANGE_THRESH", dense_config.min_range_thresh,
   //			  0 );
   config_file.get_value( "SD_MAX_RANGE_THRESH", dense_config.max_range_thresh,
			  0 );


   config_file.get_value( "SD_CORR_WINDOW"  , dense_config.corr_window,21   );
   config_file.get_value( "SD_NUM_DISP", dense_config.num_disp,128);
   // config_file.get_value( "SD_DUMP_TIMING", dense_config.dump_timing);
   return dense_config;
}



//!
//! Find locations of in the second image of features matching those
//! found in the first image by using the Lucas-Kanade tracker.
//!
bool Stereo_Dense::dense_stereo( const IplImage *left_frame,
				 const IplImage *right_frame,const IplImage *color_frame)
			
				
{

  if(!_rect){
    _rect = new Rect_Data(*calib,left_frame,true);
  
  }
  
  rect_image(*_rect,left_frame,rect_left,right_frame,rect_right,color_frame,rect_color);
 
  //printf("Saving...\n");
   // cvSaveImage( "real-L.bmp", rect_left);
   //cvSaveImage( "real-R.bmp", rect_right);
  /* cvSaveImage( "real2-l.bmp", left_frame);
   cvSaveImage( "real2-r.bmp", right_frame);*/
  // if(image_scale != 1.0 ){
    
  cvResize( rect_left, scaled_left );
  cvResize( rect_right, scaled_right );
  /* if(color_frame)    
    cvResize( rect_color, scaled_color );
  invalid_mask =scaled_mask;  
    // }
  cvCmpS( scaled_left, 1.0, invalid_mask,  CV_CMP_LT );
  cvCmpS( scaled_right, 1.0, tmp,  CV_CMP_LT );
  cvAdd(invalid_mask,tmp,invalid_mask);*/
  /* printf("Saving...\n");
  cvSaveImage( "scaled-l.bmp", rect_left);
  cvSaveImage( "scaled-r.bmp", rect_right);
  */


if(dense_config.method == "existing"){
    disp=cvLoadImage("disp.bmp",0);
    if(!disp )
      fprintf(stderr,"Load Failed %s not found or valid\n","disp.bmp");
  }

   else if(dense_config.method == "opencvBM"){
 const int nPixels = rect_right->width * rect_right->height;

    cvFindStereoCorrespondenceBM(scaled_left,scaled_right,disp16,
                        BMState);
    float *output = (float *)disp->imageData;
    /*  cvNamedWindow("sdsa",-1);
    cvShowImage("sdsa",disp16);
    cvWaitKey(0);*/
 for(unsigned int row=0;row<(unsigned int)disp16->height;row++){
      for(unsigned int col=0;col<(unsigned int)disp16->width;col++){
	short 	val=(CV_IMAGE_ELEM(disp16,short,row,col));
	//printf("%d\n",val);
      }
    }
 
 /*  for (int i = 0; i < nPixels; i++){
      //  printf("%d\n",disp16->imageData[i]);
       output[i] = disp16->imageData[i] ;
       }*/

  }
#if 0
else if(dense_config.method == "opencvGC"){

const int nPixels = rect_right->width * rect_right->height;
    cvFindStereoCorrespondenceGC(scaled_left,scaled_right,disp16,disp16R,
                        GCstate);
    float *output = (float *)disp->imageData;
    /*  cvNamedWindow("sdsa",-1);
    cvShowImage("sdsa",disp16);
    cvWaitKey(0);*/
   
     
    for (int i = 0; i < nPixels; i++){
      //  printf("%d\n",disp16->imageData[i]);
       output[i] = disp16->imageData[i] ;
    }

  }
#endif 
else{
    fprintf(stderr,"Unknown method %s\n",dense_config.method.c_str());
    return false;
  }


  if(show_debug_images)
    display_feature_images();
 
 return true; 
}



void Stereo_Dense::get_points( std::vector<osg::Vec3> &points,IplImage *mask){

  int pts_skip=1;//round(1/ratio);
  //  int *pcoord;
  // int y = iy;
  float cx = (float)calib->Q[(4*0)+3];
  float cy = (float)calib->Q[(4*1)+3];
  float f  = (float)calib->Q[(4*2)+3];
  float itx = (float)calib->Q[(4*3)+2];
  int dmax=INT_MAX;
  int dmin=INT_MIN;

  itx *= 1.0 / (float)16;//dense_config.dpp; // adjust for subpixel interpolation

  // set up range max/min disparity
  if (dense_config.max_range_thresh > 0.0){
  
      //Negate because we're z down
    double dm = f / ((dense_config.max_range_thresh/image_scale)*itx);
      dmax = (int)dm;
    }
   if (dense_config.min_range_thresh > 0.0)
    { //Negate because we're z down
      double dm = f / (dense_config.min_range_thresh*itx);
      dmin = (int)dm;
      }
 
    for(unsigned int row=0;row<(unsigned int)disp16->height;row++){
      for(unsigned int col=0;col<(unsigned int)disp16->width;col++){
	//Negate because we're z down
	short val=-(CV_IMAGE_ELEM(disp16,short,row,col));
    osg::Vec3 pt;
	//	printf("%d %d %d\n",val ,dmin,dmax);
	if (val < dmax && val < 0/*postive values are now invalid cause of negation above*/)
		{
		  //FLip cy
		  float ax = (float)row + cy*image_scale;;
		  //Flip cx
		  float ay = (float)col + cx*image_scale;;
		  float aw = 1.0 / (itx * (float)val);
          pt[1] = ax*aw; // X swithed Y
          pt[0] = ay*aw; // Y swithc X
          pt[2] = f*aw*image_scale;; // Z
        //  pt[3] = 0;
          if(isfinite(pt[0]) &&isfinite(pt[1]) &&isfinite(pt[2])){
		    points.push_back(pt);
		    //    printf("%f %f %f %d\n",pt(1),pt(0),pt(2),val);
			  if(mask)
			    CV_IMAGE_ELEM(mask,uchar,row,col)=1;
		  }
		}
	 
	    }
	}
 
}

//!
//! Display images showing the located corner features and the 
//! tracks between feature positions in the left and right images
//!
//! This is useful to check that this class is working correctly
//!
void Stereo_Dense::display_feature_images( ) 
{
  debug_image_scale=1.0;
   //
   // Allocate space for debug images
   //
   static CvSize debug_size = cvSize( (int)(scaled_left->width*debug_image_scale),
                                      (int)(scaled_left->height*debug_image_scale) );
   static IplImage *left_rgb  = cvCreateImage( cvGetSize(scaled_left) , 8, 3 );
   static IplImage *disp_rgb = cvCreateImage( cvGetSize(scaled_right), 8, 3 );
   static IplImage *disp = cvCreateImage( cvGetSize(scaled_right), 8, 1 );
   static IplImage *disp_display_image  = cvCreateImage( debug_size, 8, 3 );
   static IplImage *left_display_image = cvCreateImage( debug_size, 8, 3 );

   //
   // Create RGB copy of source image
   //
   cvCvtColor( scaled_left, left_rgb, CV_GRAY2BGR );
   get_disp_8(disp);
   cvCvtColor( disp, disp_rgb, CV_GRAY2BGR );
   //   printf("Num Disp %d\n",dense_config.num_disp);
   make_color_disp_image(disp16, dense_config.num_disp, disp_rgb,1);
   //
   // Scale RGB images to debug image size
   //
   cvResize( left_rgb , left_display_image  );
   cvResize( disp_rgb, disp_display_image );

   

   //
   // Display debug images
   // 
   cvNamedWindow( LEFT_DEBUG_IMAGE_NAME , 1 );
   cvNamedWindow( RIGHT_DEBUG_IMAGE_NAME, 1 );

   cvShowImage( LEFT_DEBUG_IMAGE_NAME , left_display_image  );
   cvShowImage( RIGHT_DEBUG_IMAGE_NAME, disp_display_image );

   /*   cvResizeWindow( LEFT_DEBUG_IMAGE_NAME, 
                   left_display_image->width, 
                   left_display_image->height );
  
   cvResizeWindow( RIGHT_DEBUG_IMAGE_NAME, 
                   disp_display_image->width, 
                   disp_display_image->height );

   */
   //
   // Save debug image
   //
   if( save_debug_images )
   {
      static int index = 0;
      stringstream left_file_name;
      stringstream disp_file_name;
      left_file_name << "stereo_dense_" 
                     << setfill('0') << setw(4) << index 
                     << "_left""-" << dense_config.method<<".png";
      disp_file_name << "stereo_dense_" 
		     << setfill('0') << setw(4) << index 
		     <<"_disp"<<"-" << dense_config.method<<".png";

      cvSaveImage( left_file_name.str( ).c_str( ) , left_display_image );
      cvSaveImage( disp_file_name.str( ).c_str( ), disp_display_image );
      index++;
   }
}

void Stereo_Dense::get_disp(IplImage *&retdisp){
  if(!retdisp || disp->width != retdisp->width || disp->height != retdisp->height || disp->nChannels != retdisp->nChannels || disp->depth != retdisp->depth){
    retdisp=cvCloneImage(disp);
    printf("get_disp: reallocating image invalid size depth\n");
  }else{
    cvCopy(disp,retdisp);
  }
  

}

void Stereo_Dense::get_disp_8(IplImage *&retdisp){
  if(!retdisp || disp->width != retdisp->width || disp->height != retdisp->height || disp->nChannels != retdisp->nChannels || retdisp->depth != IPL_DEPTH_8U){
  
    printf("get_disp_8: reallocating image invalid size depth local %d %d passed %d %d\n",disp->height,disp->depth,retdisp->height,retdisp->depth);
    retdisp=cvCreateImage(cvSize(disp->width,disp->height),IPL_DEPTH_8U,1);
   
}

  cvConvertScale(disp,retdisp,8.0);
  

}


void Stereo_Dense::get_rect_left(IplImage *&retdisp){
  if(!retdisp || disp->width != retdisp->width || disp->height != retdisp->height || disp->nChannels != retdisp->nChannels || retdisp->depth != IPL_DEPTH_8U){
    retdisp=cvCreateImage(cvSize(disp->width,disp->height),IPL_DEPTH_8U,1);
    printf("get_rect_left: reallocating image invalid size depth\n");
  }

  cvConvertScale(scaled_left,retdisp);
  

}
void Stereo_Dense::get_rect_color(IplImage *&retdisp){
  if(!retdisp || scaled_color->width != retdisp->width || scaled_color->height != retdisp->height || scaled_color->nChannels != retdisp->nChannels || retdisp->depth != IPL_DEPTH_8U){
    retdisp=cvCreateImage(cvSize(disp->width,disp->height),IPL_DEPTH_8U,3);
    printf("get_color_left: reallocating image invalid size depth\n");
  }

  cvConvertScale(scaled_color,retdisp);
  

}

void Stereo_Dense::get_rect_right(IplImage *&retdisp){
  if(!retdisp || disp->width != retdisp->width || disp->height != retdisp->height || disp->nChannels != retdisp->nChannels || retdisp->depth != IPL_DEPTH_8U){
    retdisp=cvCreateImage(cvSize(disp->width,disp->height),IPL_DEPTH_8U,1);
    printf("get_rect_right: reallocating image invalid size depth\n");
  }
  cvConvertScale(scaled_right,retdisp);

  

}

 static unsigned char dmap[768] = 
  { 150, 150, 150,
    107, 0, 12,
    106, 0, 18,
    105, 0, 24,
    103, 0, 30,
    102, 0, 36,
    101, 0, 42,
    99, 0, 48,
    98, 0, 54,
    97, 0, 60,
    96, 0, 66,
    94, 0, 72,
    93, 0, 78,
    92, 0, 84,
    91, 0, 90,
    89, 0, 96,
    88, 0, 102,
    87, 0, 108,
    85, 0, 114,
    84, 0, 120,
    83, 0, 126,
    82, 0, 131,
    80, 0, 137,
    79, 0, 143,
    78, 0, 149,
    77, 0, 155,
    75, 0, 161,
    74, 0, 167,
    73, 0, 173,
    71, 0, 179,
    70, 0, 185,
    69, 0, 191,
    68, 0, 197,
    66, 0, 203,
    65, 0, 209,
    64, 0, 215,
    62, 0, 221,
    61, 0, 227,
    60, 0, 233,
    59, 0, 239,
    57, 0, 245,
    56, 0, 251,
    55, 0, 255,
    54, 0, 255,
    52, 0, 255,
    51, 0, 255,
    50, 0, 255,
    48, 0, 255,
    47, 0, 255,
    46, 0, 255,
    45, 0, 255,
    43, 0, 255,
    42, 0, 255,
    41, 0, 255,
    40, 0, 255,
    38, 0, 255,
    37, 0, 255,
    36, 0, 255,
    34, 0, 255,
    33, 0, 255,
    32, 0, 255,
    31, 0, 255,
    29, 0, 255,
    28, 0, 255,
    27, 0, 255,
    26, 0, 255,
    24, 0, 255,
    23, 0, 255,
    22, 0, 255,
    20, 0, 255,
    19, 0, 255,
    18, 0, 255,
    17, 0, 255,
    15, 0, 255,
    14, 0, 255,
    13, 0, 255,
    11, 0, 255,
    10, 0, 255,
    9, 0, 255,
    8, 0, 255,
    6, 0, 255,
    5, 0, 255,
    4, 0, 255,
    3, 0, 255,
    1, 0, 255,
    0, 4, 255,
    0, 10, 255,
    0, 16, 255,
    0, 22, 255,
    0, 28, 255,
    0, 34, 255,
    0, 40, 255,
    0, 46, 255,
    0, 52, 255,
    0, 58, 255,
    0, 64, 255,
    0, 70, 255,
    0, 76, 255,
    0, 82, 255,
    0, 88, 255,
    0, 94, 255,
    0, 100, 255,
    0, 106, 255,
    0, 112, 255,
    0, 118, 255,
    0, 124, 255,
    0, 129, 255,
    0, 135, 255,
    0, 141, 255,
    0, 147, 255,
    0, 153, 255,
    0, 159, 255,
    0, 165, 255,
    0, 171, 255,
    0, 177, 255,
    0, 183, 255,
    0, 189, 255,
    0, 195, 255,
    0, 201, 255,
    0, 207, 255,
    0, 213, 255,
    0, 219, 255,
    0, 225, 255,
    0, 231, 255,
    0, 237, 255,
    0, 243, 255,
    0, 249, 255,
    0, 255, 255,
    0, 255, 249,
    0, 255, 243,
    0, 255, 237,
    0, 255, 231,
    0, 255, 225,
    0, 255, 219,
    0, 255, 213,
    0, 255, 207,
    0, 255, 201,
    0, 255, 195,
    0, 255, 189,
    0, 255, 183,
    0, 255, 177,
    0, 255, 171,
    0, 255, 165,
    0, 255, 159,
    0, 255, 153,
    0, 255, 147,
    0, 255, 141,
    0, 255, 135,
    0, 255, 129,
    0, 255, 124,
    0, 255, 118,
    0, 255, 112,
    0, 255, 106,
    0, 255, 100,
    0, 255, 94,
    0, 255, 88,
    0, 255, 82,
    0, 255, 76,
    0, 255, 70,
    0, 255, 64,
    0, 255, 58,
    0, 255, 52,
    0, 255, 46,
    0, 255, 40,
    0, 255, 34,
    0, 255, 28,
    0, 255, 22,
    0, 255, 16,
    0, 255, 10,
    0, 255, 4,
    2, 255, 0,
    8, 255, 0,
    14, 255, 0,
    20, 255, 0,
    26, 255, 0,
    32, 255, 0,
    38, 255, 0,
    44, 255, 0,
    50, 255, 0,
    56, 255, 0,
    62, 255, 0,
    68, 255, 0,
    74, 255, 0,
    80, 255, 0,
    86, 255, 0,
    92, 255, 0,
    98, 255, 0,
    104, 255, 0,
    110, 255, 0,
    116, 255, 0,
    122, 255, 0,
    128, 255, 0,
    133, 255, 0,
    139, 255, 0,
    145, 255, 0,
    151, 255, 0,
    157, 255, 0,
    163, 255, 0,
    169, 255, 0,
    175, 255, 0,
    181, 255, 0,
    187, 255, 0,
    193, 255, 0,
    199, 255, 0,
    205, 255, 0,
    211, 255, 0,
    217, 255, 0,
    223, 255, 0,
    229, 255, 0,
    235, 255, 0,
    241, 255, 0,
    247, 255, 0,
    253, 255, 0,
    255, 251, 0,
    255, 245, 0,
    255, 239, 0,
    255, 233, 0,
    255, 227, 0,
    255, 221, 0,
    255, 215, 0,
    255, 209, 0,
    255, 203, 0,
    255, 197, 0,
    255, 191, 0,
    255, 185, 0,
    255, 179, 0,
    255, 173, 0,
    255, 167, 0,
    255, 161, 0,
    255, 155, 0,
    255, 149, 0,
    255, 143, 0,
    255, 137, 0,
    255, 131, 0,
    255, 126, 0,
    255, 120, 0,
    255, 114, 0,
    255, 108, 0,
    255, 102, 0,
    255, 96, 0,
    255, 90, 0,
    255, 84, 0,
    255, 78, 0,
    255, 72, 0,
    255, 66, 0,
    255, 60, 0,
    255, 54, 0,
    255, 48, 0,
    255, 42, 0,
    255, 36, 0,
    255, 30, 0,
    255, 24, 0,
    255, 18, 0,
    255, 12, 0,
    0, 0, 0,
    0, 0, 0
  };
void
make_color_disp_image(IplImage *disp, int ndisp,  IplImage *outI,int skip)
{
  int i, v ;
  short *in;
  short v2;
  unsigned char *out=(unsigned char*)outI->imageData;
  int n= disp->width*disp->height;
  in = (short *)disp->imageData;	// disparity images are short's
  for(unsigned int row=0;row<(unsigned int)disp->height;row++){
      for(unsigned int col=0;col<(unsigned int)disp->width;col++){
	short 	val=(CV_IMAGE_ELEM(disp,short,row,col));
	//	printf("ball %d\n",val);
      }
    }

  for (i=0; i<n; i++)
    {
      if(i % skip ==0)
	in+=skip;
      v = *in;
      v2= *in;
      // printf("a:%d\n",v2);
      if (v < 0)
	{
	  *out++ = 0;
	  *out++ = 0;
	  *out++ = 0;
	}      
      else
	{
	  v = (v*255)/(ndisp*16);
	  v = v*3;
	
	  *out++ = dmap[v];
	  *out++ = dmap[v+1];
	  *out++ = dmap[v+2];
	}
    }
};

