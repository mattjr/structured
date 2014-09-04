/**
 * @file Sonar.h
 * @brief Projection functions for imaging sonar, e.g. DIDSON
 * @author: Michael Kaess
 * @date: Aug 2014
 */

#pragma once

#include <vector>

#include <isam/isam.h>

namespace sonar {

class Sonar {
  double _rMin, _rMax;
  double _bearingFov, _elevationFov;
  int _numBearings, _numRanges;

public:

  // constructor taking sonar intrinsics
  Sonar(double rMin, double rMax, double bearingFov, double elevationFov,
      int numBearings, int numRanges);

  // returns vector of eight points that define a bounding box of the sonar
  // volume given a pose. From the sonar looking into the frustum, the first
  // four points define the near plane and are ordered counterclockwise
  // starting from bottom left
  std::vector<isam::Point3d> getFrustum(const isam::Pose3d& pose) const;

  // provides the projection of a point given a pose; returns true if
  // projection is visible to the sonar
  bool project(const isam::Pose3d& pose, const isam::Point3d& point,
      isam::Point2d& projection) const;

  // minimum range of measurements
  double minRange() const {
    return _rMin;
  }

  // maximum range of measurements
  double maxRange() const {
    return _rMax;
  }

  // number of beams / bearing measurements
  int numBearings() const {
    return _numBearings;
  }

  // number of range bins
  int numRanges() const {
    return _numRanges;
  }
};

} /* namespace sonar */
