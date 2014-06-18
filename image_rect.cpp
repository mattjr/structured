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

#include "image_rect.hpp"

using namespace std;

void PrintMat(CvMat *A, FILE *fp)
{
  int i, j;
  for (i = 0; i < A->rows; i++)
    {
      switch (CV_MAT_DEPTH(A->type))
	{
	case CV_32F:
	case CV_64F:
	  for (j = 0; j < A->cols; j++)
	    fprintf (fp,"%8.5f ", (float)cvGetReal2D(A, i, j));
	  break;
	case CV_8U:
	case CV_16U:
	  for(j = 0; j < A->cols; j++)
	    fprintf (fp,"%6d",(int)cvGetReal2D(A, i, j));
	  break;
	default:
	  break;
	}
      fprintf(fp,"\n");
    }
}

//!
//! Create an Rect_Data object
//!
//! NOTE: This function assumes that the third radial distortion parameter 
//!       kc5 is zero. A warning is printed if this is not the case.
//!
//! \param calib           Intrinsic parameters of the camera.
//! \param image        An arbitrary image from the camera.
//! \param interpolate  Should interpolation be used.
//! 
Rect_Data::Rect_Data( const StereoCalib &calib,
		      const IplImage     *image,
		      bool                undist,
              bool interpolate ):undist(undist)
  //rect_matrix_l(3,3), rect_matrix_r(3,3),left_proj(3,3),left_proj_rect(3,3)
{
  printf("Rect map created\n");
//  cv_stereo.camera[0] = convert_to_opencv(calib.camera_calibs[0]);
  //cv_stereo.camera[1] =convert_to_opencv(calib.camera_calibs[1]);

  /* rectMap[LEFT_CAM][0] = cvCreateMat(calib.camera_calibs[0].height,
                     calib.camera_calibs[0].width,
				     CV_16SC2);
  rectMap[LEFT_CAM][1] = cvCreateMat(calib.camera_calibs[0].height,
                     calib.camera_calibs[0].width,
		     CV_16SC2);*/
  double K1[3][3];
  double D1[5];

 for(int i=0; i<5; i++)
    D1[i]=0.0;
  
  D1[0]=calib.camera_calibs[0].kc1;
  D1[1]=calib.camera_calibs[0].kc2;
  D1[2]=calib.camera_calibs[0].kc3;
  D1[3]=calib.camera_calibs[0].kc4;
  D1[4]=calib.camera_calibs[0].kc5;
  cv::Mat D_left  = cv::Mat(1, 5, CV_64F, D1 );

  for(int i=0; i< 3; i++)
    for(int j=0; j<3; j++)
      K1[i][j]=0.0;

  K1[0][0] = calib.camera_calibs[0].fcx;
  K1[0][2] = calib.camera_calibs[0].ccx;
  K1[1][1] = calib.camera_calibs[0].fcy;
  K1[1][2] = calib.camera_calibs[0].ccy;
  K1[2][2] = 1;

  cv::Mat K_left  = cv::Mat(3, 3, CV_64F, K1 );

  CvMat *R_left  = cvCreateMat(3,3,CV_64F);
  CvMat *R_right  = cvCreateMat(3,3,CV_64F);
  for( unsigned int i=0 ; i < 3 ; i++ )
    for(int j=0; j < 3; j++){
      //cvmSet(Krect,i,j, calib.rectK[(4*i)+j]);
      cvmSet(R_left,i,j, calib.left_R[(4*i)+j]);
      cvmSet(R_right,i,j, calib.right_R[(4*i)+j]);
    }


  // CvMat *Krect  = cvCreateMat(3,3,CV_64F);
 

  double K2[3][3];
  double  D2[5];
  for(int i=0; i<5; i++)
    D2[i]=0.0;
  
  D2[0]=calib.camera_calibs[1].kc1;
  D2[1]=calib.camera_calibs[1].kc2;
  D2[2]=calib.camera_calibs[1].kc3;
  D2[3]=calib.camera_calibs[1].kc4;
  D2[4]=calib.camera_calibs[1].kc5;

  cv::Mat D_right = cv::Mat(1, 5, CV_64F, D2 );

  
  for(int i=0; i< 3; i++)
    for(int j=0; j<3; j++)
      K2[i][j]=0.0;
  
  K2[0][0] = calib.camera_calibs[1].fcx;
  K2[0][2] = calib.camera_calibs[1].ccx;
  K2[1][1] = calib.camera_calibs[1].fcy;
  K2[1][2] = calib.camera_calibs[1].ccy;
  K2[2][2] = 1;
  cv::Mat K_right = cv::Mat(3, 3, CV_64F, K2 );
  /*  rectMap[RIGHT_CAM][0] = cvCreateMat(calib.camera_calibs[1].height,
                      calib.camera_calibs[1].width,
				      CV_32FC1);
  rectMap[RIGHT_CAM][1] = cvCreateMat(calib.camera_calibs[1].height,
                    calib.camera_calibs[1].width,
		    CV_32FC1);*/
  double *rm2_a= (double *)calib.camera_calibs[1].rotMatr; //  = { 0,0,0, 0,0,0,  0,0,0};
  cv::Mat R = cv::Mat(3,3,CV_64F,rm2_a);                //calculate rotation matrix
  
  
  double t2_a[3] = {  calib.camera_calibs[1].transVect[0],
		      calib.camera_calibs[1].transVect[1],
		      calib.camera_calibs[1].transVect[2] };
  
  cv::Mat T = cv::Mat(3,1,CV_64F,t2_a);                //transformation vector camera 2

cv::Mat R1; // 3x3 matrix
cv::Mat R2; // 3x3 matrix
cv::Mat P1; // 3x4 matrix
cv::Mat P2; // 3x4 matrix
 CvSize imageSize;
 imageSize.height=calib.camera_calibs[1].height;
 imageSize.width=calib.camera_calibs[1].width;

 cv::stereoRectify(K_left,D_left,K_right,D_right,imageSize,R,T,R1,R2,P1,P2,Q);

 cv::initUndistortRectifyMap(K_left, D_left, R1,P1,imageSize,CV_16SC2,
			    rectMap[LEFT_CAM][0], 
			    rectMap[LEFT_CAM][1]);
 cv::initUndistortRectifyMap(K_right, D_right, R2,P2,imageSize,CV_16SC2,
			    rectMap[RIGHT_CAM][0], 
			    rectMap[RIGHT_CAM][1]);


  /* printf("\nRight camera matrix:\n");
   PrintMat(&K_right,stderr);

   printf("\nD rightmatrix:\n");
   PrintMat(&D_right,stderr);
  printf("\nR rightmatrix:\n");
   PrintMat(R_right,stderr);
   printf("\nK matrx:\n");
   PrintMat(Krect,stderr);
  */
}



//!
//! Destroy an UndistortData object
//!
Rect_Data::~Rect_Data( )
{
   //cvReleaseImage( &data );
   
}



//!
//! Rect an image. 
//!
//! This function removes the effect of the radial distortions defined by
//! kc1 and kc2, and the tangential distortions defined by kc3 and kc4.
//!
//! This function does not shift the image to center the principal point
//! (ccx and ccy). These still need to be considered along with the focal
//! length when calculating azimuth and elevation angles.
//!
//! NOTE: This function assumes that the third radial distortion parameter 
//!       kc5 is zero. A warning is printed if this is not the case.
//!
//! \param data    Precalculated image rection data.
//!
//! \param source  The distorted source image.
//!
//! \param dest    The destination for the rected image. This image 
//!                must be initialised by the called to have the same 
//!                size and format as the source image.
//!
void rect_image(  Rect_Data &data,
			     const IplImage       *sourceL,
			     IplImage             *&destL ,
			     const IplImage       *sourceR,
			      IplImage             *&destR,
			     const IplImage       *sourceC,
			      IplImage             *&destC)


{

  if(destL == NULL)
    destL=cvCloneImage(sourceL);    
//    destL=cvCreateImage(cvSize(sourceL->width,sourceL->height),sourceL->depth,sourceL->nChannels);

  if(destR == NULL)
    destR=cvCloneImage(sourceR);
  //destR=cvCreateImage(cvSize(sourceR->width,sourceR->height),sourceR->depth,sourceR->nChannels);

  if(sourceC){  
    if(destC == NULL)
      destC=cvCloneImage(sourceC);
    //  destC=cvCreateImage(cvSize(sourceC->width,sourceC->height),sourceC->depth,sourceC->nChannels);

  }

  

  cv::remap( cv::Mat(sourceL), cv::OutputArray(destL), data.rectMap[LEFT_CAM][0], 
	     data.rectMap[LEFT_CAM][1] ,CV_INTER_LINEAR);

  
 
  cv::remap( cv::Mat(sourceR), cv::OutputArray(destR), data.rectMap[RIGHT_CAM][0],
	     data.rectMap[RIGHT_CAM][1],CV_INTER_LINEAR );
  
  if(sourceC){
    cv::remap( cv::Mat(sourceC), cv::OutputArray(destC), data.rectMap[LEFT_CAM][0], 
	       data.rectMap[LEFT_CAM][1] ,CV_INTER_LINEAR);
  }

}



