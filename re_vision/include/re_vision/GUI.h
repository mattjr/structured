/**
 * File: GUI.h
 * Project: DUtilsCV library
 * Author: Dorian Galvez
 * Date: September 24, 2010
 * Description: OpenCV-related GUI functions
 *
 */

#ifndef __D_CV_GUI__
#define __D_CV_GUI__

#include <vector>
#include <opencv/cv.h>

namespace DUtilsCV
{

class GUI
{
public:
  
  /**
   * Creates a windows showing the given image and waits untils some key
   * is pressed
   * @param image
   */
  static void showImage(const cv::Mat &image, bool autosize = true);
  
  /**
   * Saves the image in a temporary file to visualize it with a system
   * application
   * @param image
   * @param tmp_file file where the image is stored
   * @param app application invoked to visualize the image
   * @return true iff success
   */
  static bool showSystemImage(const cv::Mat &image,
    const std::string &tmp_file = "tmp.png",
    const std::string &app = "eog");

};

}

#endif
