#ifndef CAMERACALIBFILE_H
#define CAMERACALIBFILE_H
#include <string>
#include <iostream>
#include <opencv/cv.h>
//! A Class to store calibration parameters for a camera
class CameraCalib
{
public:
   unsigned int width;   //! Image width  (number of pixels along image X-axis)
   unsigned int height;  //! Image height (number of pixels along image Y-axis)

   double ccx;  //!< X-axis coordinate of the central point (in pixel coords)
   double ccy;  //!< Y-axis coordinate of the central point (in pixel coords)
   double fcx;  //!< X-axis focal length (measured in pixel units)
   double fcy;  //!< Y-axis focal length (measured in pixel units)
   double kc1;  //!< Second order radial distortion coefficient
   double kc2;  //!< Fourth order radial distortion coefficient
   double kc3;  //!< Radial distortion coefficient
   double kc4;  //!< Radial distortion coefficient
   double kc5;  //!< Sixth order radial distortion coefficient (often not used)

   double   rotMatr[9];
   double   transVect[3];
};

class StereoCalib
{
public:
    StereoCalib(const std::string &calib_file_name);
    std::vector<CameraCalib> camera_calibs;  //!< Intrinsic parameters for the all cameras
    void load_from_file( const std::string &calib_file_name );
    CameraCalib extract_from_file( std::istream &in_file,
                                                 bool have_3rd_radial_param);
    bool have_3rd_radial_param;
    bool have_rect_params;


};

#endif // CALIBFILE_H
