#include "MemUtils.h"
#include <osgDB/FileNameUtils>
extern const char *diced_dir;
#include <stack>
#include <errno.h>
#include <string.h>

using namespace std;
#define KILOBYTE_FACTOR 1024.0
#define MEGABYTE_FACTOR ( 1024.0 * 1024.0 )
#define GIGABYTE_FACTOR ( 1024.0 * 1024.0 * 1024.0 )

//const char *uname="mesh";
const char *diced_dir="tmp/mesh-diced/";
const char *diced_img_dir="tmp/mesh-img/";

const char *mosdir="mosaic";
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
string getProj4StringForAUVFrame(double lat_origin,double lon_origin){

    char szProj4[4096];
    sprintf( szProj4,
             "+proj=tmerc +lat_0=%.24f +lon_0=%.24f +k=%.12f +x_0=%.12f +y_0=%.12f +datum=WGS84 +ellps=WGS84 +units=m +no_defs",lat_origin,lon_origin,1.0,0.0,0.0);
    return string(szProj4);
}
void getULLR(osg::Matrix viewproj,int width,int height,osg::Vec4 &ullr){
    osg::Matrix screenToWorldmitch=osg::Matrix::inverse(viewproj); // added by mitch

    // Added by Mitch
    osg::Vec4 tlmitch(-1.0,1.0,0,1.0);
    osg::Vec4 brmitch(1.0,-1.0,0,1.0);
    osg::Vec4 tlGlobalmitch=tlmitch*screenToWorldmitch;
    osg::Vec4 brGlobalmitch=brmitch*screenToWorldmitch;





    ullr[0]=tlGlobalmitch.y();
    ullr[1]=tlGlobalmitch.x();
    ullr[2]=brGlobalmitch.y();
    ullr[3]=brGlobalmitch.x();

}

bool applyGeoTags(std::string name,osg::Vec2 geoOrigin,osg::Matrix viewproj,int width,int height,string basepath,string ext,int jpegQuality){
#if 0
    osg::Matrix modWindow =( osg::Matrix::translate(1.0,1.0,1.0)*osg::Matrix::scale(0.5*width,0.5*height,0.5f));
    osg::Matrix bottomLeftToTopLeft= (osg::Matrix::scale(1,-1,1)*osg::Matrix::translate(0,height,0));
    osg::Matrix worldtoScreen=/*viewMatrix * projMatrix*/ viewproj * modWindow*bottomLeftToTopLeft;
    osg::Matrix screenToWorld=osg::Matrix::inverse(worldtoScreen);
    osg::Matrix screenToWorldmitch=osg::Matrix::inverse(viewproj); // added by mitch

    osg::Vec3 tl(0,0,0);
    osg::Vec3 bl(0,height,0);
    osg::Vec3 tr(width,0,0);
    osg::Vec3 br(width,height,0);

    string proj4=getProj4StringForAUVFrame(geoOrigin.x(),geoOrigin.y());
    osg::Vec3 tlGlobal=tl*screenToWorld;
    osg::Vec3 blGlobal=bl*screenToWorld;
    osg::Vec3 trGlobal=tr*screenToWorld;
    osg::Vec3 brGlobal=br*screenToWorld;
    
    // Added by Mitch
    osg::Vec4 tlmitch(-1.0,1.0,0,1.0);
    osg::Vec4 brmitch(1.0,-1.0,0,1.0);
    osg::Vec4 tlGlobalmitch=tlmitch*screenToWorldmitch;
    osg::Vec4 brGlobalmitch=brmitch*screenToWorldmitch;
    
    cout << "tl: " << tlGlobalmitch.x() << " " << tlGlobalmitch.y() << endl;
    cout << "br:" << brGlobalmitch.x() << " " << brGlobalmitch.y() << endl;
    
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

    /*sprintf(gdal_param," -of GTiff -co \"compress=packbits\" -co \"TILED=YES\" -a_ullr %.12f %.12f %.12f %.12f -a_srs %s",tlGlobal.x(),tlGlobal.y(),brGlobal.x(),brGlobal.y(),proj4.c_str());
    
    // added by mitch
    sprintf(gdal_param," -of GTiff -co \"compress=packbits\" -co \"TILED=YES\" -a_ullr %.12f %.12f %.12f %.12f -a_srs %s",tlGlobalmitch.y(),tlGlobalmitch.x(),brGlobalmitch.y(),brGlobalmitch.x(),szProj4);
    */
/*if(jpegQuality <0)
        sprintf(gdal_param," -of GTiff -co \"compress=packbits\" -co \"TILED=YES\" -a_ullr %.12f %.12f %.12f %.12f -a_srs \"%s\"",
              tlGlobalmitch.y(),tlGlobalmitch.x(),brGlobalmitch.y(),brGlobalmitch.x(),
                // tlGlobal.x(),tlGlobal.y(),brGlobal.x(),brGlobal.y(),
                proj4.c_str());
    else
        sprintf(gdal_param," -of GTiff -b 1 -b 2 -b 3 -mask 4 -co \"compress=JPEG\" -co \"TILED=YES\" --config GDAL_TIFF_INTERNAL_MASK YES -co \"PHOTOMETRIC=YCBCR\" -co \"JPEG_QUALITY=%d\" -a_ullr %.12f %.12f %.12f %.12f -a_srs \"%s\"",jpegQuality,
              //  tlGlobal.x(),tlGlobal.y(),brGlobal.x(),brGlobal.y(),
                tlGlobalmitch.y(),tlGlobalmitch.x(),brGlobalmitch.y(),brGlobalmitch.x(),

                proj4.c_str());*/


    sprintf(gdal_param," -a_ullr %.12f %.12f %.12f %.12f -a_srs \"%s\" ",
            tlGlobalmitch.y(),tlGlobalmitch.x(),brGlobalmitch.y(),brGlobalmitch.x(),
            proj4.c_str());
#endif
    osg::Vec4 ullr;
    char gdal_param[4096];

    getULLR(viewproj,width,height,ullr);
    string proj4=getProj4StringForAUVFrame(geoOrigin.x(),geoOrigin.y());
    sprintf(gdal_param," -a_ullr %.12f %.12f %.12f %.12f -a_srs \"%s\" ",
            ullr[0],ullr[1],ullr[2],ullr[3],
            proj4.c_str());
    char tmp[8192];
    sprintf(tmp,"%s-add_geo.sh",name.c_str());
    FILE *fp=fopen(tmp,"w");
    if(!fp)
        std::cerr << "Failed! "<< tmp <<"\n";
    fprintf(fp,"#!/bin/bash\n");
    //fprintf(fp,"vips im_vips2tiff subtile.v tex.tif:none:tile:256x256\n");


    fprintf(fp,"#gdal_translate %s %s %s\n",gdal_param,name.c_str(),name.c_str());
    //fprintf(fp,"#geotifcp -e %s %s %s.tif\n",wfname.c_str(),name.c_str(),name.c_str());

    fchmod(fileno(fp),0777);
    fclose (fp);
    //gdalwarp  -t_srs '+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs' geo_tif2.tif utm.tif
    //  int res= system(("add_geo-"+name+".sh").c_str());
  /*  sprintf(tmp,"gdal_merge.py -separate  %s-tmp.%s %s-tmp-mask.pgm -o %s.tif; gdal_translate %s %s.tif  %s",
            osgDB::getNameLessExtension(name).c_str(),ext.c_str(),
            osgDB::getNameLessExtension(name).c_str(),
            osgDB::getNameLessExtension(name).c_str(),
            gdal_param,
            osgDB::getNameLessExtension(name).c_str(),
            ("mosaic/"+osgDB::getSimpleFileName(name)).c_str());*/
    /*sprintf(tmp,"%s/create_masked.py %s  %s-tmp.%s %s-tmp-mask.pgm %s",
               basepath.c_str(),
               gdal_param,
               osgDB::getNameLessExtension(name).c_str(),ext.c_str(),
               osgDB::getNameLessExtension(name).c_str(),
               ("mosaic/"+osgDB::getSimpleFileName(name)).c_str());*/
    char nodata[1024];
    sprintf(nodata," -a_nodata 255,255,255 -co \"TILED=YES\" -co \"COMPRESS=DEFLATE\" ");
    sprintf(tmp,"gdal_translate %s %s %s-tmp.%s  %s",
            gdal_param,
            nodata,
            osgDB::getNameLessExtension(name).c_str(),ext.c_str(),
            ("mosaic/"+osgDB::getSimpleFileName(name)).c_str());
    int res= system(tmp);
    // printf("%s\n",tmp);
    if(res != 0){
        printf("Failed on run %s\n",tmp);
        return false;
    }
    return true;
}


bool genPyramid(std::string name,std::string basepath,int pyramidHeight,int sizeX,int sizeY,string ext){
    char tmp[8192];
    tmp[0]='\0';
    char tmp2[1024];
    for(int i=1; i < pyramidHeight+1; i++){
        if(i==1)
            sprintf(tmp2,"%s.%s",osgDB::getNameLessExtension(name).c_str(),ext.c_str());
        else
            sprintf(tmp2,"%s-%02d.%s",(string(diced_img_dir)+"/"+osgDB::getNameLessExtension(osgDB::getSimpleFileName(name))).c_str(),i-1,ext.c_str());

        sprintf(tmp,"%s %s/create_mipmap %s %s-%02d.%s %d %d %d;",tmp,basepath.c_str(),tmp2,
                (string(diced_img_dir)+"/"+osgDB::getNameLessExtension(osgDB::getSimpleFileName(name))).c_str(),i,ext.c_str(),sizeX,sizeY,i);
    }
    int res= system(tmp);
    // printf("%s\n",tmp);
    if(res != 0){
        printf("Failed on run %s\n",tmp);
        return false;
    }
    return true;
}

osg::Vec2 calcCoordReprojSimple(const osg::Vec3 &vert,const osg::Matrix &trans,const osg::Matrix &viewProj,const osg::Vec2 &size){
    osg::Vec4 v(vert.x(),vert.y(),vert.z(),1.0);
    v=v*trans;
    v=v*viewProj;
    v.x() /= v.w();
    v.y() /= v.w();

    v.x() /= size.x();;
    v.y() /= size.y();


    osg::Vec2 tc(v.x(),v.y());


    return tc;

}
bool makeDirectory( const std::string &path ,__mode_t maskval)
{
    if (path.empty())
    {
        osg::notify(osg::DEBUG_INFO) << "osgDB::makeDirectory(): cannot create an empty directory" << std::endl;
        return false;
    }

    struct stat64 stbuf;
    if( stat64( path.c_str(), &stbuf ) == 0 )
    {
        if( S_ISDIR(stbuf.st_mode))
            return true;
        else
        {
            osg::notify(osg::DEBUG_INFO) << "osgDB::makeDirectory(): "  <<
                    path << " already exists and is not a directory!" << std::endl;
            return false;
        }
    }

    std::string dir = path;
    std::stack<std::string> paths;
    while( true )
    {
        if( dir.empty() )
            break;

        if( stat64( dir.c_str(), &stbuf ) < 0 )
        {
            switch( errno )
            {
                case ENOENT:
                case ENOTDIR:
                    paths.push( dir );
                    break;

                default:
                    osg::notify(osg::DEBUG_INFO) << "osgDB::makeDirectory(): "  << strerror(errno) << std::endl;
                    return false;
            }
        }
        dir = osgDB::getFilePath(std::string(dir));
    }

    while( !paths.empty() )
    {
        std::string dir = paths.top();

        #if defined(WIN32)
            //catch drive name
            if (dir.size() == 2 && dir.c_str()[1] == ':') {
                paths.pop();
                continue;
            }
        #endif

        if( mkdir( dir.c_str(), maskval )< 0 )
        {
            osg::notify(osg::DEBUG_INFO) << "osgDB::makeDirectory(): "  << strerror(errno) << std::endl;
            return false;
        }
        paths.pop();
    }
    return true;
}
