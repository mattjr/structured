/** \file changeReferenceSystem.cpp
 * \brief Changes reference system of points and cameras
 *
 * Standalone application for changing the reference system of a point cloud
 * and a set of cameras
 * 
 * This file is part of the RoboEarth ROS WP1 package.
 * 
 * It file was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the European Union Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2011 by <a href="mailto:dorian@unizar.es">Dorian Galvez-Lopez</a>, University of Zaragoza
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * \author Dorian Galvez-Lopez
 * \version 1.0
 * \date 2011
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
***********************************************/

#include <iostream>
#include <fstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string>
#include <vector>


#include "DUtils.h"
#include "DUtilsCV.h"
#include "DVision.h"

typedef DVision::PixelPointFile::PixelPoint PixelPoint;
typedef DVision::PMVS::PatchFile PatchFile;
typedef DVision::PMVS::PatchFile::Patch Patch;
typedef DVision::PMVS::CameraFile CameraFile;
typedef DVision::PMVS::CameraFile::Camera Camera;
typedef DVision::PMVS::PLYFile PLYFile;
typedef DVision::PMVS::PLYFile::PLYPoint PLYPoint;

using namespace std;
using namespace DUtils;

// ----------------------------------------------------------------------------

/**
 * Calculates the transformation needed to describe the points in the 
 * object reference system
 * @param plypoints
 * @return a 4x4 transformation matrix
 */
cv::Mat calculateTransformation(const vector<PLYPoint> &plypoints);

/**
 * Applys a transformation to the points to move them to the object
 * reference system
 * @param plypoints
 * @param oTw transformation from (w)orld to (o)bject
 */
void transformPoints(vector<PLYPoint> &plypoints, 
  vector<Patch> &sparse_points, const cv::Mat &oTw);

/**
 * Applies the given transformation to the given 3d point
 * @param x
 * @param y
 * @param z
 * @param s scale (homogeneous coordinates)
 * @param oTw transformation from (w)orld to (o)bject
 */
void transformPoint(double &x, double &y, double &z, double s, const cv::Mat &oTw);

/**
 * Applys a transformation to the cameras so that the projection of the new
 * 3D points be the same as before
 * @param cameras
 * @param oTw
 */
void transformCameras(vector<Camera> &cameras, const cv::Mat &oTw);

// ----------------------------------------------------------------------------

void test(const Camera &camera, const PLYPoint &p)
{
  cv::Mat X = (cv::Mat_<double>(4,1) << p.x, p.y, p.z, 1.);
  cv::Mat x = camera.P * X;
  x.at<double>(0,0) /= x.at<double>(2,0);
  x.at<double>(1,0) /= x.at<double>(2,0);

  cout << "(" << p.x << " " << p.y << " " << p.z << ") --> ";
  cout << x.at<double>(0,0) << ", " << x.at<double>(1,0) << endl;
}

// ----------------------------------------------------------------------------

int main(int argc, char *argv[])
{
  if(argc < 7)
  {
    cout << "Usage: " << argv[0] << " <camera file> <ply file> <patch file>"
      " <camera out dir> <out ply file> <out patch file>" << endl;
      return 1;
  }
  
  string camera_file = argv[1];
  string ply_file = argv[2];
  string patch_file = argv[3];
  string camera_out_dir = argv[4];
  string ply_out_file = argv[5];
  string patch_out_file = argv[6];

  try 
  {

    cout << "Reading data..." << endl;

    vector<PLYPoint> plypoints;
    PLYFile::readFile(ply_file, plypoints);
    
    cout << "-- " << plypoints.size() << " PLY points read" << endl;
    
    vector<Patch> patches;
    PatchFile::readFile(patch_file, patches);
    
    cout << "-- " << patches.size() << " patches read" << endl;
    
    vector<Camera> cameras;
    CameraFile::readFile(camera_file, cameras);
    
    cout << "-- " << cameras.size() << " cameras read" << endl;

    cout << "Changing reference system..." << endl;

  #if 0
      test(cameras[0], plypoints[10]);
      test(cameras[0], plypoints[20]);
      test(cameras[0], plypoints[30]);
  #endif
    
    cv::Mat oTw = calculateTransformation(plypoints); 
    cout << "." << flush;
    transformPoints(plypoints, patches, oTw);
    cout << "." << flush;
    transformCameras(cameras, oTw);
    cout << "." << endl;
    
  #if 0
      test(cameras[0], plypoints[10]);
      test(cameras[0], plypoints[20]);
      test(cameras[0], plypoints[30]);
  #endif
    
    cout << "Saving..." << endl;
    PLYFile::saveFile(ply_out_file, plypoints);
    PatchFile::saveFile(patch_out_file, patches);
    CameraFile::saveFile(camera_out_dir, cameras);
  
  }catch(std::string ex)
  {
    std::cout << ex << std::endl;
    return 1;
  }
  
  return 0;
}

// ----------------------------------------------------------------------------

cv::Mat calculateTransformation(const vector<PLYPoint> &plypoints)
{
  // the object reference will be the centroid of the point cloud.
  // the new axes will be oriented according to the principal axis of the cloud
  if(plypoints.empty()) return cv::Mat::eye(4, 4, CV_64F);
  
  cv::Mat samples(plypoints.size(), 3, CV_64FC1);
  
  cv::Mat centroid(1, 3, CV_64FC1);
  centroid = 0.0;  
  
  /*
  double minx, maxx, miny, maxy, minz, maxz;
  minx = maxx = plypoints[0].x;
  miny = maxy = plypoints[0].y;
  minz = maxz = plypoints[0].z;
  */
  
  for(unsigned int i = 0; i < plypoints.size(); ++i)
  {
    samples.at<double>(i, 0) = plypoints[i].x;
    samples.at<double>(i, 1) = plypoints[i].y;
    samples.at<double>(i, 2) = plypoints[i].z;
    
    /*
    if(plypoints[i].x < minx) minx = plypoints[i].x;
    else if(plypoints[i].x > maxx) maxx = plypoints[i].x;
    
    if(plypoints[i].y < miny) miny = plypoints[i].y;
    else if(plypoints[i].y > maxy) maxy = plypoints[i].y;
    
    if(plypoints[i].z < minz) minz = plypoints[i].z;
    else if(plypoints[i].z > maxz) maxz = plypoints[i].z;
    */
    
    centroid += samples.rowRange(i,i+1);
  }
  centroid /= plypoints.size();

  /*
  centroid.ptr<double>()[0] = (maxx + minx)/2.0;
  centroid.ptr<double>()[1] = (maxy + miny)/2.0;
  centroid.ptr<double>()[2] = (maxz + minz)/2.0;
  */
  
  cv::Mat cov, mean;
  cv::calcCovarMatrix(samples, cov, mean, CV_COVAR_NORMAL | CV_COVAR_ROWS);
    
  cv::Mat eigenvalues, eigenvectors;
  cv::eigen(cov, eigenvalues, eigenvectors); // descending order
  // eigenvectors are normalized
  
  // rotation to set old x, y, z axes to new principal directions, in 
  // descending order
  
  // 1) rotation to align old X with new X
  // oX = [1 0 0 0]';
  const cv::Mat xX = eigenvectors.colRange(0,1); // first eigenvector == new X axis
  
  // rotation vector
  cv::Mat V = (cv::Mat_<double>(3,1) 
    << 0.0, -xX.at<double>(2,0), xX.at<double>(1,0) ); // V = cross(oX, xX)
  V /= cv::norm(V);
  
  // rotation angle
  double alpha = acos(xX.at<double>(0,0)); // xX[0] == dot(oX, xX) / norm(oX) / norm(xX)

  // rotate the old system with RotV and get the intermediate Y axis
  // oY = [0 1 0 0]';
  cv::Mat RotV = DUtilsCV::Transformations::rotvec(V, alpha);
  cv::Mat auxY = RotV.colRange(1,2).rowRange(0,3); // == RotV * oY
  
  // 2) rotation to align auxY with new Y
  const cv::Mat xY = eigenvectors.colRange(1,2);
  double beta = acos( auxY.dot(xY) );

  // calculate beta sign
  V = auxY.cross(xY);
  V /= norm(V);
  if(V.dot(xX) < 0) beta = -beta;
  
  cv::Mat RotX = DUtilsCV::Transformations::rotx(beta);

  // 3) get final transformation
  cv::Mat wTo = RotV * RotX;

  // 4) merge with the translation to the centroid (or some center)
  wTo.at<double>(0, 3) = centroid.ptr<double>()[0];
  wTo.at<double>(1, 3) = centroid.ptr<double>()[1];
  wTo.at<double>(2, 3) = centroid.ptr<double>()[2];
  
  return DUtilsCV::Transformations::inv(wTo); // == oTw
}

// ----------------------------------------------------------------------------

void transformPoints(vector<PLYPoint> &plypoints, 
  vector<Patch> &patches,
  const cv::Mat &oTw)
{  
  vector<PLYPoint>::iterator pit;
  for(pit = plypoints.begin(); pit != plypoints.end(); ++pit)
  {
    transformPoint(pit->x, pit->y, pit->z, 1., oTw);
    transformPoint(pit->nx, pit->ny, pit->nz, 0., oTw);
  }

  vector<Patch>::iterator bit;
  for(bit = patches.begin(); bit != patches.end(); ++bit)
  {
    transformPoint(bit->x, bit->y, bit->z, bit->s, oTw);
    transformPoint(bit->nx, bit->ny, bit->nz, bit->ns, oTw);
  }
}

// ---------------------------------------------------------------------------

void transformPoint(double &x, double &y, double &z, double s, const cv::Mat &oTw)
{
  cv::Mat wP = (cv::Mat_<double>(4,1) << x, y, z, s);
  cv::Mat oP = oTw * wP;
  
  if(s != 0.)
  {
    x = oP.at<double>(0,0) / oP.at<double>(3,0);
    y = oP.at<double>(1,0) / oP.at<double>(3,0);
    z = oP.at<double>(2,0) / oP.at<double>(3,0);
  }else
  {
    x = oP.at<double>(0,0);
    y = oP.at<double>(1,0);
    z = oP.at<double>(2,0);
  }
}

// ---------------------------------------------------------------------------

void transformCameras(vector<Camera> &cameras, const cv::Mat &oTw)
{  
  const cv::Mat wTo = DUtilsCV::Transformations::inv(oTw);
    
  for(unsigned int i = 0; i < cameras.size(); ++i)
  {
    // For PMVS Cameras
    cv::Mat K, R, t; // 3x3, 3x3, 4x1
    cv::decomposeProjectionMatrix(cameras[i].P, K, R, t);
    // opencv 2.1: t is not correct 
 
    assert(R.type() == CV_64F);
    assert(t.type() == CV_64F);
    
    cv::Mat Rt = K.inv() * cameras[i].P;
    cv::Mat cvTw = (cv::Mat_<double>(4,4) <<
      Rt.at<double>(0,0), Rt.at<double>(0,1), Rt.at<double>(0,2), Rt.at<double>(0,3),
      Rt.at<double>(1,0), Rt.at<double>(1,1), Rt.at<double>(1,2), Rt.at<double>(1,3), 
      Rt.at<double>(2,0), Rt.at<double>(2,1), Rt.at<double>(2,2), Rt.at<double>(2,3), 
      0, 0, 0, 1); 

    cv::Mat cvTo = cvTw * wTo;

    cvTo.at<double>(0,3) /= cvTo.at<double>(3,3);
    cvTo.at<double>(1,3) /= cvTo.at<double>(3,3);
    cvTo.at<double>(2,3) /= cvTo.at<double>(3,3);

    cameras[i].P = K * cvTo.rowRange(0,3);
    

    /*
    // For Bundle Cameras
    cv::Mat R, t;
    R = cameras[i].R;
    t = cameras[i].t;
    
    cv::Mat cvTw = ( cv::Mat_<double>(4, 4) <<
      R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0),
                    0.0,               0.0,               0.0, t.at<double>(3,0));

    cv::Mat cvTo = cvTw * wTo;

    cvTo.at<double>(0,3) /= cvTo.at<double>(3,3);
    cvTo.at<double>(1,3) /= cvTo.at<double>(3,3);
    cvTo.at<double>(2,3) /= cvTo.at<double>(3,3);

    cameras[i].R = cvTo.rowRange(0, 3).colRange(0, 3);
    cameras[i].t = cvTo.rowRange(0, 3).colRange(3, 4);
    */
  }
}

// ---------------------------------------------------------------------------


