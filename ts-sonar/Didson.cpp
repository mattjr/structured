/**
 * @file Didson.cpp
 * @brief Projection functions for DIDSON
 * @author: Michael Kaess
 * @date: Aug 2014
 */

#include <vector>
#include <utility>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

#include "Didson.h"

using namespace boost;
using namespace std;
using namespace isam;
using namespace Eigen;

namespace sonar {

const float degtorad = M_PI / 180.;
const float radtodeg = 180. / M_PI;

class DidsonConstants {
public:
  double bearingFov;
  double elevationFov;
  int numBearings;
  int numRanges;
  double startBase;
  std::vector<double> lengths;
  DidsonConstants() {
    bearingFov = 28.8 * degtorad; // 28.8 or 29? (both are given in the specs)
    elevationFov = 28. * degtorad; // 28 degrees with spreader lens (default without lens is 14 degrees)
    numBearings = 96;
    numRanges = 512;
#if 1 // todo: unknown if DIDSON in CW or XW mode
    // classic window mode (CW)
    startBase = 0.375;
    lengths.resize (4);
    lengths[0] = 1.125; lengths[1] = 2.25; lengths[2] = 4.5; lengths[3] = 9.;
#else
    // extended window mode (XW)
    startBase = 0.42;
    lengths = {1.25, 2.5, 5., 10.};
#endif
  }
};

const DidsonConstants consts;

pair<int, int> DidsonCartesian::bearingRange2Cartesian(int bearing,
    int range) {
  int pos = _invMap[range * consts.numBearings + bearing];
  int col = pos % image.cols;
  int row = (pos - col) / image.cols;
  return make_pair(row, col);
}

// conversion of Matlab DIDSON code from Soundmetrics ---
int Didson::lensDistortion(int nbeams, double theta) {
  double* a = NULL;
  double factor = 0.0;
  double a48[4] = { 0.0015, -0.0036, 1.3351, 24.0976 };
  double a189[4] = { 0.0015, -0.0036, 1.3351, 24.0978 };
  double a96[4] = { 0.0030, -0.0055, 2.6829, 48.04 };
  double a381[4] = { 0.0030, -0.0055, 2.6829, 48.04 };

  switch (nbeams) {
  case 48:
    factor = 1.0;
    a = a48;
    break;
  case 189:
    factor = 4.026;
    a = a189;
    break;
  case 96:
    factor = 1.012;
    a = a96;
    break;
  case 381:
    factor = 4.05;
    a = a381;
    break;
  }
  return (int) round(
      factor
          * (a[0] * theta * theta * theta + a[1] * theta * theta + a[2] * theta
              + a[3] + 1));
}

// conversion of Matlab DIDSON code from Soundmetrics ---
/**
 * Calculates a map from cartesian to polar coordinates
 *
 * ixsize  - number of pixels in horizontal direction in image space
 * rmax    - maximum range in meters
 * rmin    - minimum range in meters
 * halffov - one-half of sector field of view in radians
 * nbins   - number of range bins in sample space
 */
pair<vector<int>, int> Didson::createMapping(int ixsize, double rmax,
    double rmin, double halffov, int nbeams, int nbins) {
  //double d2 = rmax*cos(halffov); // distance from point scan touches image boundary to origin
  double d3 = rmin * cos(halffov); // bottom of image frame to r,theta origin in meters
  double c1 = (consts.numRanges - 1) / (rmax - rmin); // precalculation of constants used in do loop below
  //double c2 = (nbeams-1)/(2*halffov);
  double gamma = ixsize / (2 * rmax * sin(halffov)); // Ratio of pixel number to position in meters

  int iysize = (int) (gamma * (rmax - d3) + 0.5); // number of pixels in image in vertical direction

  vector<int> map(ixsize * iysize); // Stores the index map

  // ix,iy   - coordinates of a pixel in image space
  for (int iy = 1; iy <= iysize; iy++) {
    for (int ix = 1; ix <= ixsize; ix++) {
      double x = ((ix - 1) - ixsize / 2) / gamma; // Convert from pixels to meters

      double z = 0.0;
      double y = rmax - (iy - 1) / gamma; // Convert from pixels to meters

      double r = sqrt(y * y + x * x + z * z); // Convert to polar coordinates
      double theta = radtodeg * atan2(x, y); // Theta is in degrees
      int binnum = (int) ((r - rmin) * c1 + 1.5); // the rangebin number
      int beamnum = lensDistortion(nbeams, theta); // Remove the lens distortation using empirical formula
      int pos = 0; // invalid == -1, note pos-1 below
      if ((beamnum > 0) && (beamnum <= nbeams) && (binnum > 0)
          && (binnum <= nbins)) {
        pos = (binnum - 1) * nbeams + beamnum;
      }
      map[(iy - 1) * ixsize + ix - 1] = pos - 1;
    }
  }
  return make_pair(map, iysize);
}

// create a Cartesian image suitable for texture mapping from the raw
// bearing/range measurements; also return a mask of valid image regions
shared_ptr<DidsonCartesian> Didson::getCartesian(int width, int widthTmp) const {
  // generate map for Cartesian image
  vector<int> map;
  int height;
  pair<vector<int>, int> tmp1 = createMapping(width, maxRange(), minRange(),
      consts.bearingFov * 0.5, numBearings(), numRanges());
  map = tmp1.first;
  height = tmp1.second;

  // avoid having to write out the inverse mapping function by creating
  // a map with sufficiently high resolution as a lookup table for the inverse map
  // not ideal, but works...
  vector<int> invMap(consts.numRanges * consts.numBearings);
  vector<int> mapTmp;
  int heightTmp;
  pair<vector<int>, int> tmp2 = createMapping(widthTmp, maxRange(), minRange(),
      consts.bearingFov * 0.5, numBearings(), consts.numRanges);
  mapTmp = tmp2.first;
  heightTmp = tmp2.second;

  int c = 0;
  for (int y = 0; y < heightTmp; y++) {
    for (int x = 0; x < widthTmp; x++) {
      int idx = mapTmp[c];
      if (idx != -1) {
        int icol = x * ((double) width / (double) widthTmp);
        int irow = y * ((double) height / (double) heightTmp);
        int i = irow * width + icol;
        invMap[idx] = i;
      }
      c++;
    }
  }

  shared_ptr<DidsonCartesian> cartesian(new DidsonCartesian(map, invMap));
  cartesian->image = cv::Mat(height, width, CV_8UC1);
  cartesian->mask = cv::Mat(height, width, CV_8UC1);
  for (int i = 0; i < width * height; i++) {
    if (map[i] == -1) {
      cartesian->image.data[i] = 0;
      cartesian->mask.data[i] = 0;
    } else {
      cartesian->image.data[i] = _image.data[map[i]];
      cartesian->mask.data[i] = 255;
    }
  }

  return cartesian;
}

// constructor taking a DIDSON frame
Didson::Didson(int windowStart, int windowLength, const unsigned char* data,
    const isam::Pose3d& vehiclePose, double tiltRad, double rollRad) :
    Sonar(consts.startBase * windowStart,
        consts.startBase * windowStart + consts.lengths[windowLength],
        consts.bearingFov, consts.elevationFov, consts.numBearings,
        consts.numRanges), _tiltRad(tiltRad), _rollRad(
        rollRad), _vehiclePose(vehiclePose) {
  if (windowStart < 1 || windowStart > 31) {
    cout << "ERROR: windowStart out of range" << endl;
    exit(1);
  }
  if (windowLength < 0 || windowLength > 3) {
    cout << "ERROR: windowLength out of range" << endl;
    exit(1);
  }

  _image = cv::Mat(consts.numRanges, consts.numBearings, CV_8UC1);
  memcpy(_image.data, data, consts.numRanges * consts.numBearings);

  // we assume DIDSON facing to the right (DVL in hull lock mode)
  // i.e. 90 degrees yaw and -90 degrees roll configuration + tilt and roll actuators
  Pose3d didsonOffset(0, 0.2, 0, 0, 0, 0); // DIDSON is to the right of the DVL along y  // todo: check DIDSON offset
  Pose3d didsonActuators(0, 0, 0, M_PI * 0.5, tiltRad, rollRad - M_PI * 0.5);
  _didsonPose = vehiclePose.oplus(didsonOffset).oplus(didsonActuators);
}

Pose3d Didson::didsonHuls3Pose() const {
  return _didsonPose;
}

vector<Point3d> Didson::getFrustum() const {
  return Sonar::getFrustum(didsonHuls3Pose());
}

bool Didson::project(const isam::Point3d& point,
    isam::Point2d& projection) const {
  return Sonar::project(didsonHuls3Pose(), point, projection);
}

} /* namespace sonar */
