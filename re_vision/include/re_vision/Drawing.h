/**
 * File: Drawing.h
 * Project: DUtilsCV library
 * Author: Dorian Galvez-Lopez
 * Date: September 23, 2010
 * Description: drawing functions
 *
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
 */

#ifndef __D_CV_DRAWING__
#define __D_CV_DRAWING__

#include <vector>
#include <opencv/cv.h>

namespace DUtilsCV
{

class Drawing
{
public:
  
  /**
   * Draws keypoints on an image
   * @param image
   * @param keypoints
   */
  static void drawKeyPoints(cv::Mat &image, 
    const std::vector<cv::KeyPoint> &keypoints);

  /**
   * Draws and saves keypoints on an image
   * @param filename
   * @param image
   * @param keypoints
   */
  static void saveKeyPointImage(const std::string &filename,
    const cv::Mat &image, const std::vector<cv::KeyPoint> &keypoints);
  
  /**
   * Creates an image with correspondences
   * @param image return image
   * @param im1 
   * @param im2
   * @param kp1 keypoints from im1
   * @param kp2 keypoints from im2
   * @param c1 indices of correspondences from kp1 
   * @param c2 indices of correspondences from kp2
   */
  static void drawCorrespondences(cv::Mat &image, const cv::Mat &im1,
    const cv::Mat &im2, const std::vector<cv::KeyPoint> &kp1,
    const std::vector<cv::KeyPoint> &kp2,
    const std::vector<int> &c1, const std::vector<int> &c2);

  /**
   * Creates and saves an image with correspondences
   * @param filename file to create
   * @param im1 
   * @param im2
   * @param kp1 keypoints from im1
   * @param kp2 keypoints from im2
   * @param c1 indices of correspondences from kp1 
   * @param c2 indices of correspondences from kp2
   */
  static void saveCorrespondenceImage(const std::string &filename, 
    const cv::Mat &im1,
    const cv::Mat &im2, const std::vector<cv::KeyPoint> &kp1,
    const std::vector<cv::KeyPoint> &kp2,
    const std::vector<int> &c1, const std::vector<int> &c2);

  /**
   * Draws a reference system in the given image with axes x red, y green, z
   * blue
   * @param image image to draw
   * @param cTo transformation from camera to the origin of the drawn reference
   * @param A intrinsic camera parameters 
   * @param K distortion of the camera
   * @param length length of axes
   */
  static void drawReferenceSystem(cv::Mat &image, const cv::Mat &cTo,
    const cv::Mat &A, const cv::Mat &K = cv::Mat(), 
    float length = 0.1);

  /**
   * Draws a reference system in the given image with axes x red, y green, z
   * blue
   * @param image image to draw
   * @param cRo rotation from camera to the origin of the drawn reference
   * @param cto translation from camera to the origin of the drawn reference
   * @param A intrinsic camera parameters 
   * @param K distortion of the camera
   * @param length length of axes
   */
  static void drawReferenceSystem(cv::Mat &image, const cv::Mat &cRo,
    const cv::Mat &cto, const cv::Mat &A, const cv::Mat &K = cv::Mat(), 
    float length = 0.1);

  /**
   * Draws a rectangle in the image from its location in the 3D space
   * @param image image to draw
   * @param cRo rotation from camera to the center of the rectangle
   * @param cto translation from camera to the center of the rectangle
   * @param width
   * @param height dimensions of the rectangle in metres
   * @param A intrinsic camera parameters 
   * @param K distortion of the camera
   */
  static void drawBox(cv::Mat &image, const cv::Mat &cRo,
    const cv::Mat &cto, float width, float height,
    const cv::Mat &A, const cv::Mat &K = cv::Mat());

  /** 
   * Draws a rectangle in the image from an homography
   * @param image image to draw
   * @param sHb homography from a orthonormal plane to the image
   * @param cols
   * @param rows dimensions of the plane in pixels
   */
  static void drawBox(cv::Mat &image, const cv::Mat &sHb, int cols, int rows);

};

}

#endif
