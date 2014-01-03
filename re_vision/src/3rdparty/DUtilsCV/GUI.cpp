/**
 * File: GUI.cpp
 * Project: DUtilsCV library
 * Author: Dorian Galvez-Lopez
 * Date: September 24, 2010
 * Description: OpenCV-related GUI functions
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

#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cstdio>
#include <string>
#include <sstream>
#include "GUI.h"

using namespace std;
using namespace DUtilsCV;

// ---------------------------------------------------------------------------

void GUI::showImage(const cv::Mat &image, bool autosize)
{
  static int id = 0;
  
  char name[16];
  sprintf(name, "win%d", id++);
  
  int flags = 0;
  if(autosize) flags |= CV_WINDOW_AUTOSIZE;
  
  cv::namedWindow( name, flags );
  cv::imshow( name, image );
  cv::waitKey();
}

// ---------------------------------------------------------------------------

bool GUI::showSystemImage(const cv::Mat &image,
  const std::string &tmp_file, const std::string &app)
{
  cv::imwrite(tmp_file, image);
  
  stringstream ss;
  ss << app << " " << tmp_file;

  return(0 == system(ss.str().c_str()));
}

// ---------------------------------------------------------------------------

