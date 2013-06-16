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

#ifndef AUV_STEREO_DENSE_HPP
#define AUV_STEREO_DENSE_HPP

#include "configFile.h"

#include "cxmisc.h"
#include "highgui.h"
#include "cxmisc.h"
#include "cv.h"

#include "calibFile.h"
#include "image_rect.hpp"
#include <osg/Vec3>

class Dense_Config
{
public:

   //!
   //! Create a Dense_Config object with default configuration values
   //!
   Dense_Config( void ); 

  ~Dense_Config(){};
   //!
   //! Create a Dense_Config object using values from a configuration 
   //! file
   //!
   Dense_Config( const Config_File &config_file );
   
  std::string method ;

  int num_disp;
  int birchfield;
  int squaredDiffs;
  int truncDiffs;
  int MRFalg;
  int smoothexp;
  int smoothmax;
  int lambda;
  int gradThresh;
  int gradPenalty;
  int outscale;
  int verbose;

  int minDisp;
  double max_range_thresh;
  double min_range_thresh;
   unsigned int corr_window;
  //estereo params
  int m_iHoropter;  

  int m_iDisparityImageScale;
  int m_iSADThreshold;
  int width;
  int height;
  int xOffset;
  int yOffset;
  bool m_bPropagateStereo;
  int m_iDisparityRange;
  int svsThresh;
  int svsStep;
  bool dump_timing;
};


//!
//! A class to find dense stereo in a pair of images
//!
class Stereo_Dense 
{
public:


   //!
   //! Create a Stereo_Dense object with stereo calibration
   //!
   //! \param calib  May be NULL
   //!
  Stereo_Dense( const Config_File &config_file,
		double image_scale,
        const StereoCalib *calib  );


   //!
   //! Destroy a Stereo_Dense object
   //!
   ~Stereo_Dense( void );
  bool dense_stereo( const IplImage *left_frame,
		     const IplImage *right_frame,const IplImage *color_frame=NULL);
  void get_disp_8(IplImage *&retdisp);
  void get_rect_left(IplImage *&retdisp);
  void get_rect_color(IplImage *&retdisp);
  void get_rect_right(IplImage *&retdisp);
  void get_depth_image(IplImage *&depthI);
  void get_disp(IplImage *&disp);		
  IplImage * getDisp16(){return disp16;}
  char dispname[255];
  void get_points(std::vector<osg::Vec3> &points,IplImage *mask=NULL);
protected:
   void display_feature_images() ;
   
   //!
   //! Load the configuration for the object used to locate corners in the 
   //! left frame.
   //!
   Dense_Config 
   load_dense_config( const Config_File &config_file ) const;

   //--------------------//
   // Instance Variables //
   //--------------------//

   //
   // Configuration
   //
   bool using_undistorted_images;  //!< Have the effects of radial and 
                                   //!< tangential distortion been removed from
                                   //!< the images provided to this object.

   double image_scale;  //!< The image supplied to this object may
                        //!< have been scaled from their original size
                        //!< for which the stereo calibration is correct.
                        //!< This parameter specifes the image scaling
                        //!< factor. eg. 0.5 for half size images

   const StereoCalib *calib;  //!< Stereo camera calibration parameters

  IplImage *scaled_left,*scaled_right,*scaled_color;
  IplImage *rect_left,*rect_right,*rect_color;
  IplImage *disp;
  IplImage *disp8;
  IplImage *disp16;
  IplImage *disp16R;
  
  IplImage *invalid_mask,*scaled_mask,*tmp;
  CvSize scaled_size;
  IplImage *Ixyz;
  const CameraCalib *left_calib;   //!< Left camera calibration parameters
  const CameraCalib *right_calib;  //!< Right camera calibration parameters
  CvStereoBMState *BMState;
//  CvStereoGCState* GCstate ;
  Dense_Config dense_config;
 //
   // Debugging
   // 
   bool show_debug_images;   //!< Should images showing the located features
                             //!< (useful for debugging) be shown

   bool save_debug_images;   //!< Should the debugging images be saved to disk

   double debug_image_scale; //!< The debug images can be scaled to help fit
                             //!< them all on a screen without overlapping
  Rect_Data *_rect;


};
// number of lines to print into timing file
#define MAXITER 500



#endif //!AUV_STEREO_DENSE_HPP
