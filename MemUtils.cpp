#include "MemUtils.h"
#include <osgDB/FileNameUtils>
using namespace std;
#define KILOBYTE_FACTOR 1024.0
#define MEGABYTE_FACTOR ( 1024.0 * 1024.0 )
#define GIGABYTE_FACTOR ( 1024.0 * 1024.0 * 1024.0 )
std::string get_size_string(double kb){
    int buflen=8192;
    char buf[buflen];
    double size = kb*1024;
    double displayed_size;
    if( size < MEGABYTE_FACTOR )
    {
        displayed_size =  size / KILOBYTE_FACTOR;
        snprintf( buf, buflen,  "%'.1f KB" , displayed_size );
    }
    else if( size < GIGABYTE_FACTOR )
    {
        displayed_size = size / MEGABYTE_FACTOR;
        snprintf( buf, buflen,  "%'.1f MB" , displayed_size );
    }
    else
    {
        displayed_size =  size / GIGABYTE_FACTOR;
        snprintf( buf, buflen,  "%'.1f GB" , displayed_size );
    }
    return buf;
}

//////////////////////////////////////////////////////////////////////////////
//
// process_mem_usage(double &, double &) - takes two doubles by reference,
// attempts to read the system-dependent data for a process' virtual memory
// size and resident set size, and return the results in KB.
//
// On failure, returns 0.0, 0.0

void process_mem_usage(double& vm_usage, double& resident_set)
{
    using std::ios_base;
    using std::ifstream;
    using std::string;

    vm_usage     = 0.0;
    resident_set = 0.0;

    // 'file' stat seems to give the most reliable results
    //
    ifstream stat_stream("/proc/self/stat",ios_base::in);

    // dummy vars for leading entries in stat that we don't care about
    //
    string pid, comm, state, ppid, pgrp, session, tty_nr;
    string tpgid, flags, minflt, cminflt, majflt, cmajflt;
    string utime, stime, cutime, cstime, priority, nice;
    string O, itrealvalue, starttime;

    // the two fields we want
    //
    unsigned long vsize;
    long rss;

    stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
            >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
            >> utime >> stime >> cutime >> cstime >> priority >> nice
            >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest

    stat_stream.close();

    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
    vm_usage     = vsize / 1024.0;
    resident_set = rss * page_size_kb;
}
bool applyGeoTags(std::string name,osg::Vec2 geoOrigin,osg::Matrix viewproj,int width,int height){

    osg::Matrix modWindow =( osg::Matrix::translate(1.0,1.0,1.0)*osg::Matrix::scale(0.5*width,0.5*height,0.5f));
    osg::Matrix bottomLeftToTopLeft= (osg::Matrix::scale(1,-1,1)*osg::Matrix::translate(0,height,0));
    osg::Matrix worldtoScreen=/*viewMatrix * projMatrix*/ viewproj * modWindow*bottomLeftToTopLeft;
    osg::Matrix screenToWorld=osg::Matrix::inverse(worldtoScreen);
    osg::Vec3 tl(0,0,0);
    osg::Vec3 bl(0,height,0);
    osg::Vec3 tr(width,0,0);
    osg::Vec3 br(width,height,0);
    char szProj4[4096];
    sprintf( szProj4,
             "\"+proj=tmerc +lat_0=%.24f +lon_0=%.24f +k=%.12f +x_0=%.12f +y_0=%.12f +datum=WGS84 +ellps=WGS84 +units=m +no_defs\"",geoOrigin.x(),geoOrigin.y(),1.0,0.0,0.0);

    osg::Vec3 tlGlobal=tl*screenToWorld;
    osg::Vec3 blGlobal=bl*screenToWorld;
    osg::Vec3 trGlobal=tr*screenToWorld;
    osg::Vec3 brGlobal=br*screenToWorld;
    osg::Vec3 diff=(brGlobal-tlGlobal);
    osg::Vec2 scalePix( diff.x()/width, diff.y()/height);
    cout << "brGlobal " << brGlobal<< " tlGlobal " << tlGlobal << endl;
    cout << "trGlobal " << trGlobal<< " " << blGlobal << endl;
    string wfname=name+".tfw";
    std::ofstream worldfile(wfname.c_str());
    worldfile << std::setprecision(12) << scalePix.x() << std::endl<< 0 <<std::endl<< 0 << std::endl<<scalePix.y() << std::endl<<tlGlobal.x()<<std::endl<<tlGlobal.y()<<std::endl;
    std::cout << tlGlobal << " " << blGlobal <<" " <<trGlobal<< " " << brGlobal <<"\n";
    worldfile.close();
    char gdal_param[4096];
    sprintf(gdal_param," -of GTiff -co \"compress=packbits\" -co \"TILED=YES\" -a_ullr %.12f %.12f %.12f %.12f -a_srs %s",tlGlobal.x(),tlGlobal.y(),brGlobal.x(),brGlobal.y(),szProj4);
    char tmp[8192];
    sprintf(tmp,"%s-add_geo.sh",name.c_str());
    FILE *fp=fopen(tmp,"w");
    if(!fp)
        std::cerr << "Failed! "<< tmp <<"\n";
    fprintf(fp,"#!/bin/bash\n");
    //fprintf(fp,"vips im_vips2tiff subtile.v tex.tif:none:tile:256x256\n");


    fprintf(fp,"#gdal_translate %s %s %s\n",gdal_param,name.c_str(),name.c_str());
    fprintf(fp,"#geotifcp -e %s %s %s.tif\n",wfname.c_str(),name.c_str(),name.c_str());

    fchmod(fileno(fp),0777);
    fclose (fp);
    //gdalwarp  -t_srs '+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs' geo_tif2.tif utm.tif
    //  int res= system(("add_geo-"+name+".sh").c_str());
    sprintf(tmp,"gdal_translate %s %s-tmp.ppm %s",gdal_param,osgDB::getNameLessExtension(name).c_str(),
            ("mosaic/"+osgDB::getSimpleFileName(name)).c_str());
    int res= system(tmp);
    // printf("%s\n",tmp);
    if(res != 0){
        printf("Failed on run geotifcp\n");
        return false;
    }
    return true;
}
