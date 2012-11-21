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
/* Hold our state in this.
 */
inline std::string format_elapsed(double d)
{
    char buf[256] = {0};

    if( d < 0.00000001 )
    {
        // show in ps with 4 digits
        sprintf(buf, "%0.4f ps", d * 1000000000000.0);
    }
    else if( d < 0.00001 )
    {
        // show in ns
        sprintf(buf, "%0.0f ns", d * 1000000000.0);
    }
    else if( d < 0.001 )
    {
        // show in us
        sprintf(buf, "%0.0f us", d * 1000000.0);
    }
    else if( d < 0.1 )
    {
        // show in ms
        sprintf(buf, "%0.0f ms", d * 1000.0);
    }
    else if( d <= 60.0 )
    {
        // show in seconds
        sprintf(buf, "%0.2f s", d);
    }
    else if( d < 3600.0 )
    {
        // show in min:sec
        sprintf(buf, "%01.0f:%02.2f", floor(d/60.0), fmod(d,60.0));
    }
    // show in h:min:sec
    else
        sprintf(buf, "%01.0f:%02.0f:%02.2f", floor(d/3600.0), floor(fmod(d,3600.0)/60.0), fmod(d,60.0));

    return buf;
}
bool applyGeoTags(std::string name,osg::Vec2 geoOrigin,osg::Matrix viewproj,int width,int height,std::string basepath,std::string ext="ppm",int jpegQuality=95);
bool genPyramid(std::string name,int pyramidHeight,std::string ext="ppm");
std::string getProj4StringForAUVFrame(double lat_origin,double lon_origin);
osg::Vec2 calcCoordReprojSimple(const osg::Vec3 &vert,const osg::Matrix &trans,const osg::Matrix &viewProj,const osg::Vec2 &size);
void getULLR(osg::Matrix viewproj,int width,int height,osg::Vec4 &ullr);
extern const char *uname;
extern const char *diced_dir;
extern const char *diced_img_dir;

extern const char *mosdir;
extern const char *aggdir;
#endif // MEMUTILS_H
