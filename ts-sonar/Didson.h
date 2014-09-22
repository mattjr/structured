/**
 * @file Didson.h
 * @brief Projection functions for DIDSON
 * @author: Michael Kaess
 * @date: Aug 2014
 */

#pragma once

#include <vector>
#include <utility>
#include <memory>
#include <boost/shared_ptr.hpp>

#include <opencv2/opencv.hpp>

#include "Sonar.h"

namespace sonar {

class DidsonCartesian {
  std::vector<int> _map;
  std::vector<int> _invMap;
public:
  DidsonCartesian(std::vector<int> map, std::vector<int> invMap) :
      _map(map), _invMap(invMap) {
  }
  cv::Mat image;
  cv::Mat mask;

  // returns (row,col) into
  std::pair<int, int> bearingRange2Cartesian(int bearing, int range);
};

class Didson: public Sonar {
  cv::Mat _image;
  double _tiltRad;
  double _rollRad;
  isam::Pose3d _vehiclePose;
  isam::Pose3d _didsonPose;

  static int lensDistortion(int nbeams, double theta);
  static std::pair<std::vector<int>, int> createMapping(int ixsize,
      double rmax, double rmin, double halffov, int nbeams, int nbins);

public:

  // constructor taking a DIDSON frame
  Didson(int windowStart, int windowLength,
         const isam::Pose3d& vehiclePose, double pan, double tilt,
         const unsigned char* data=NULL, bool transformFrame=true);

  // provides the pose of the DIDSON sensor for the HULS3 vehicle,
  // taking into account offsets and actuator angles
  isam::Pose3d didsonHuls3Pose() const;

  // returns vector of eight points that define a bounding box of the DIDSON
  // volume. From the sonar looking into the frustum, the first
  // four points define the near plane and are ordered counterclockwise
  // starting from bottom left
  std::vector<isam::Point3d> getFrustum() const;

  // provides the projection of a point into the DIDSON; returns true if
  // projection is visible to the DIDSON
  bool project(const isam::Point3d& point, isam::Point2d& projection) const;

  // bearing/range image of raw measurements
  const cv::Mat getImage() const {
    return _image;
  }

  // objection with Cartesian image, mask, and bearingRange2Cartesian
  // conversion suitable for texture mapping
  boost::shared_ptr<DidsonCartesian> getCartesian(int numCols, int widthTmp=1000) const;
};

} /* namespace sonar */
