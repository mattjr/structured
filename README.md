# structured
Tools for the Generation and Visualization of Large-scale Three-dimensional Reconstructions from Image Data.

## Installing

structured requires:

- cmake (>=2.8)
- openscenegraph (>=3.0)
- OpenCV (>=2.2)
- OpenGL
- TCL/TK (8.4)
- vips 
- gdal (including python bindings and binaries)
- csh

For installing on Ubuntu the following apt-get install lines should pull in the requirements:

### 12.04
```sudo apt-get install cmake libcv-dev libopenscenegraph-dev gdal-bin```
```sudo apt-get install libgdal1-dev python-gdal libvips-dev libopencv-dev```
```sudo apt-get install libhighgui-dev libcvaux-dev libopencv-gpu-dev```
```sudo apt-get installlibopencv-gpu2.3 tcl8.4-dev tk8.4-dev csh imagemagick```

### 13.04
```sudo apt-get install cmake libcv-dev libopencv-dev libopenscenegraph-dev```
```sudo apt-get install gdal-bin libgdal1-dev python-gdal libvips-dev ```
```sudo apt-get install libhighgui-dev libcvaux-dev tcl8.4-dev ```
```sudo apt-get install tk8.4-dev csh imagemagick```

## Running 

Binary is not installed and must be called using the full path of the source dir.

For example:
 
```/home/$USER/git/structured/threadedStereo .```

Once data folder is setup as below.

## Data folder file setup

|Filename       |Desc           |
|:--------------|:--------------:|
|mesh.cfg|Reconstruction config file. A default file is in the structured directory|
|img|A link to the converted images|
|stereo.calib|The stereo calibration file (this should be in the renav directory)|
|localiser.cfg|The SLAM  config file (this should be in the renav directory)|
|stereo_pose_est.data	|A link to the file stereo_pose_est.data produced by seabed_slam|

## Viewing the Meshes
The meshes can be viewed using the program benthicQT also avalaible on github.

## The AUV Navigation Frame

The AUV estimates its position in a local navigation reference frame
(not for example in degrees of latitude/longitude). Positions in the
navigation estimate files are measured relative to the origin of the
navigation frame.

The orientation of the local navigation frame is defined as follows:

-   Positive X is true (not magnetic) North

-   Positive Y is East

-   Positive Z is down

The origin of the local navigation frame is defined by latitude and
longitude coordinates.


## stereo\_pose\_est.data

Each line describes the pose of the stereo-vision system relative to the
navigation frame at the time a pair of stereo images were acquired.

The stereo-rig reference frame is defined to be coincident with the left
camera..

The orientation of the stereo-rig frame is defined as follows:

-   Positive X is aligned with the X-axis of an image (towards the
    right of an image)

-   Positive Y is aligned with the Y-axis of an image (towards the
    bottom of an image)

-   Positive Z is the direction in which the camera is looking

On each line of the file are 13 items:

| Position        | Desc           | Type  |
| ------------- |:-------------:| -----:|
|1|Pose identifier|integer value                    |
|2|Timestamp|in seconds                             |
|3|X position (North)|in meters, relative to nav frame|
|4|Y position (East)|in meters, relative to nav frame|
|5|Z position (Down)|in meters, relative to nav frame|
|6|X-axis Euler angle (Roll)|in radians, relative to nav frame|
|7|Y-axis Euler angle (Pitch)|in radians, relative to nav frame|
|8|Z-axis Euler angle (Yaw/Heading) |in radians, relative to nav frame|
|9|Left image name| local filename|
|10|Right image name| local filename|
|11|Vehicle altitude |in meters|
|12|Approximate bounding image radius|in meters|
|13|Likely trajectory cross-over point|1 for true, 0 for false|


Data items 12 and 13 are used within our 3D mesh building software, and
can safely be ignored in other applications.

## stereo.calib

* First Line: number of cameras (1 number)

* For each camera a line with camera parameters
   - image size (2): [x y]
   - intrinsic parameter matrix (9): [ fx 0 cx; 0 fy cy; 0 0 1 ]
   - distortion coefficients, 2 radial then 2 tangential (4): [k1 k2 p1 p2]
   - rotation matrix (9)
   - translation vector in meters (3) 

* Rectified image quadrangle for camera1 (8)
* Rectified image quadrangle for camera2 (8)

* Transformation Coefficients for camera1 (9)
* Transformation Coefficients for camera2 (9)

Notes: for a stereo config, the number of cameras is two.
       for stefs calib, the left camera is camera1, the right is camera2

-------------------------------------------------------------------------------
```

// Number of cameras
2


// Camera 1 - Image size
1360.0000000000 1024.0000000000 

// Camera 1 - Instrinsic parameters [ fx 0 cx; 0 fy cy; 0 0 1 ]
1742.4833984375    0.0000000000     702.2943725586    
0.0000000000       1744.5247802734  505.3277282715    
0.0000000000       0.0000000000     1.0000000000    

// Camera 1 - Distortion parameters [ k1 k2 p1 p2 ] 
0.1640239209    0.6399505734    0.0019153117    0.0084038246   

// Camera 1 - Rotation matrix
-0.9471259117   -0.3171964586   -0.0483626388   
-0.3193612993    0.9173640609    0.2375952154  
-0.0309982132    0.2404777408   -0.9701595306    
 
// Camera 1 - Translation vector 
0.1716919392   -0.0959866196    1.1181910038 






// Camera 2 - Image Size
1360.0000000000 1024.0000000000 

// Camera 2 - Instrinsic parameters [ fx 0 cx; 0 fy cy; 0 0 1 ] 
1739.3259277344    0.0000000000  678.7371215820    
0.0000000000 1740.5989990234  524.8011474609    
0.0000000000    0.0000000000    1.0000000000    

// Camera 2 - Distortion parameters [ k1 k2 p1 p2 ] 
0.1479872018    0.7572427392   -0.0015344607   -0.0037570531  

// Camera 2 - Rotation matrix
-0.9497462511   -0.3011870682   -0.0852552578   
-0.3128538132    0.9222519994    0.2270985991   
 0.0102276700    0.2423584610   -0.9701328874   
  
// Camera 2 - Translation vector 
0.1418329179   -0.0863670856    1.1126987934 





// Camera 1 - coordinates of destination quadrangle after epipolar geometry rectification
-63.7492790222  -17.5878181458 1363.6937255859    0.0580182485 1360.0000000000  983.3759155273  -63.3137931824  990.2045288086 

// Camera 2 - coordinates of destination quadrangle after epipolar geometry rectification
-3.4339661598   35.0206069946 1416.6680908203   25.5735092163 1423.8615722656 1022.9803466797    0.0000000000 1023.4026489258 





// Camera 1 - Transformation coefficients
1.0745683575    0.0002419059  -63.7492790280    0.0129759424    0.9870401045  -17.5878181530    0.0000183164    0.0000028963    1.0000000000 

// Camera 2 - Transformation coefficients
1.0347662652    0.0033534826   -3.4339661738   -0.0071165600    0.9626237039   35.0206069774   -0.0000066539   -0.0000025338    1.0000000000 

```

## Converting the Stereo-rig Z-axis Euler Angle to an Image Rotation for GIS Programs

The stereo-rig Z-axis Euler angle (shown as φ in the figure below) is
the angle between the X-axis of the navigation frame (north) and the
X-axis of the camera or image. The image rotation angle (shown as α) is
the angle between East and the camera/image X-axis.

The image rotation angle can therefore be obtained from the Z-axis Euler
angle by subtracting 90 degrees:

α = φ - 90°

In the figure below, the stereo-rig's Z-axis Euler angle is 135°, and
the image rotation angle is 45°.

## 

![Gis Angle](https://github.com/mattjr/structured/raw/master/GIS_angle.png "GIS Angle")


