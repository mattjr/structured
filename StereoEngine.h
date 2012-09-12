/*
 * StereoEngine.h
 * Copyright (C) Brandon Wampler 2009 <bwampler@purdue.edu>
 *
 * StereoEngine.h is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * StereoEngine.h is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef StereoEngine_H
#define StereoEngine_H

#include "cv.h"
#include <cvaux.h>
#include "cxmisc.h"
#include "highgui.h"
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>
#include <iostream>
#include <osg/StateAttribute>
#include <osg/BlendFunc>
#include "pthread.h"
#include <osgViewer/Viewer>
#include <list>
//#include <opencv2/legacy/legacy.hpp>
#include "calibFile.h"

#define AUV_NO_Z_GUESS -100.0


class StereoEngine
{
public:
    static const int winSize = 10;
    static const int maxCount = 20;
    static const double quality = 0.01;
    static const double minDistance = 10;
    pthread_t viewerThread;
    osgViewer::Viewer viewer;

    // Dynamically Allocated (Must Release Memory)
    IplImage *temp,*eig, /**leftImage, *rightImage, *leftGreyR, *rightGreyR,*/*leftPyr,*rightPyr;
 //*eig, , *threeD, *leftImageR;
    //CvPoint2D32f *points[2];
//    CvMat *Q, /**F,*/ *R1, *R2, *P1, *P2, *M1, *M2, *D1, *D2,
  //  *mx1, *my1, *mx2, *my2, *pair, *R, *T, *E, *disp, *vdisp, *img1r, *img2r, *realDisp;
    CvCalibFilter m_CalibFilter;
    CvStereoBMState *BMState;

    // Links (Don't Deallocate)
  //  IplImage *frameL, *frameR;
    StereoCalib _calib;

    // Statically Allocated
    int pointCount;
    CvSize imageSize;
    std::vector<std::string> imageNames[2];
public:
    StereoEngine(const StereoCalib &calib,double edgethresh,double max_triangulation_len,int max_feature_count,
                 double min_feat_dist,double feat_quality,int tex_size,OpenThreads::Mutex &mutex);
    virtual ~StereoEngine();
    void captureCalibrationImages(int number);
    static void stereoCalibrate(std::string imageList, int nx, int ny, int useUncalibrated);
    void findDisparity();
    void displayDisparity();
    void sparseDepth(IplImage *leftGrey,IplImage *rightGrey,const int MAX_COUNT, std::list<osg::Vec3> &points,double zguess);
 //   void denseDepth(const int MAX_COUNT, matrix<double> &bodyCoord, matrix<double> &colors, int &sparseFeatureCount);

    void reprojectTo3d();
    void drawPointCloud();
    osg::StateSet* makeStateSet(float size);
    void loadMatrices();
    void capture();
    void drawPoints(CvArr* image, CvPoint2D32f points[], int count);
    void calculateOpticalFlow();
    void displayOpticalFlow();
    bool processPair(const std::string basedir,const std::string left_file_name,const std::string &right_file_name ,const osg::Matrix &mat,osg::BoundingBox &bbox,const double feature_depth_guess);
    double edgethresh;
    double max_triangulation_len;
    int max_feature_count;
    double min_feat_dist;
    double feat_quality;
    int tex_size;
    bool verbose;
    bool display_debug_images;
    bool pause_after_each_frame;
    OpenThreads::Mutex &_osgDBMutex;
};

class is_same_vec3_xy
{
public:
    bool operator() (const osg::Vec3 &first, const osg::Vec3 &second)
    {
        return(first.x() ==second.x() && first.y() == second.y());
    }
};
bool get_stereo_pair(
    const std::string contents_dir_name,
    IplImage     *&left_image,
    IplImage     *&right_image,
    IplImage *&color_image,
    const std::string        &left_image_name,
    const std::string        &right_image_name );
bool checkmkdir(std::string dir);
double get_time( void );
void cacheImage(IplImage *img,std::string name,int tex_size);
#endif

// vim:ts=4:sw=4
