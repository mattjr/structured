/**
 * @file Sonar.cpp
 * @brief Projection functions for imaging sonar, e.g. DIDSON
 * @author: Michael Kaess
 * @date: Aug 2014
 */

#include <iostream>
#include <vector>
#include <utility>

#include <Eigen/Dense>

#include "Sonar.h"

using namespace std;
using namespace isam;
using namespace Eigen;

namespace sonar {

Sonar::Sonar(double rMin, double rMax, double bearingFov, double elevationFov,
    int numBearings, int numRanges) :
    _rMin(rMin), _rMax(rMax), _bearingFov(bearingFov), _elevationFov(
        elevationFov), _numBearings(numBearings), _numRanges(numRanges) {
  if (rMax <= rMin) {
    cout << "ERROR: rMax must be larger than rMin" << endl;
    exit(1);
  }
  if (bearingFov <= 0 || elevationFov <= 0) {
    cout << "ERROR: bearingFov and elevationFov must be positive" << endl;
    exit(1);
  }
  if (numBearings <= 0 || numRanges <= 0) {
    cout << "ERROR: numBearing and numRanges must be positive" << endl;
    exit(1);
  }
}

vector<Point3d> Sonar::getFrustum(const isam::Pose3d& pose) const {
  vector<Point3d> frustum;
  vector<double> range(2);
  range[0] = _rMin; range[1] = _rMax;
  vector<pair<double, double> > angles;
  double yaw = 0.5 * _bearingFov;
  double pitch = 0.5 * _elevationFov;
  angles.push_back(make_pair(-yaw, -pitch));
  angles.push_back(make_pair(yaw, -pitch));
  angles.push_back(make_pair(yaw, pitch));
  angles.push_back(make_pair(-yaw, pitch));

  for (int r = 0; r < 2; r++) {
    for (int a = 0; a < 4; a++) {
      yaw = angles[a].first;
      pitch = angles[a].second;
      Point3d p_local = Point3d(range[r], 0, 0);
      Point3d p_sonar = Pose3d(0, 0, 0, yaw, pitch, 0).transform_from(p_local);
      frustum.push_back(pose.transform_from(p_sonar));
    }
  }

  return frustum;
}

bool Sonar::project(const isam::Pose3d& pose, const isam::Point3d& point,
    isam::Point2d& projection) const {
  Point3d point_sonar = pose.transform_to(point);
  double range = point_sonar.vector().norm();
  double x = point_sonar.x();
  double y = point_sonar.y();
  double z = point_sonar.z();
  Vector2d direction(point_sonar.x(), point_sonar.y());
  direction.normalize();
  Vector2d mapping = direction * range;

  double u = mapping(0);
  double v = mapping(1);
  double b = (atan2(v, u) / _bearingFov + 0.5) * _numBearings;
  double r = (range - _rMin) / (_rMax - _rMin) * _numRanges;
  double elevation = atan2(z, sqrt(x * x + y * y)) / _elevationFov;

  projection = Point2d(b, r);

  // projection within field of view of sensor?
  if (b >= 0 && b < _numBearings && r >= 0 && r < _numRanges
      && elevation >= -0.5 && elevation <= 0.5) {
    return true;
  }

  return false;
}

} /* namespace sonar */
