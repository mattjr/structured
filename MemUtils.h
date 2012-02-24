#ifndef MEMUTILS_H
#define MEMUTILS_H
#include <string>
#include <fstream>
#include <math.h>
#include <osg/Matrix>
#include <osg/Vec2>
#include <osg/Vec3>
#include <iostream>
#include <osg/io_utils>
#include <iomanip>
#include <stdlib.h>
#include <stdio.h>
#include <sys/stat.h>
void process_mem_usage(double& vm_usage, double& resident_set);
std::string get_size_string(double kb);
std::string format_elapsed(double d);
bool applyGeoTags(std::string name,osg::Vec2 geoOrigin,osg::Matrix viewproj,int width,int height,std::string ext="ppm",int jpegQuality=95);
#endif // MEMUTILS_H
