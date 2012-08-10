#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/io_utils>
#include <osg/Timer>
#include <string>
#include <vips/vips.h>
#include "BuildAtlas.h"
using namespace std;
int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);
    osg::Timer_t before_computeMax = osg::Timer::instance()->tick();

    string matname;
    if(!arguments.read("-mat",matname)){
        fprintf(stderr, "need mat\n");
        exit(-1);
    }
    double scaleFactor=1.0;
    arguments.read("-scale",scaleFactor);
    string basedir="mesh";
    arguments.read("-dir",basedir);

    string mosaic_cells_fname;
    if(!arguments.read("-cells",mosaic_cells_fname)){
        fprintf(stderr, "need mat\n");
        exit(-1);
    }
    int totalX,totalY;
    std::vector<mosaic_cell> mosaic_cells;
    vips_init(argv[0]);

    loadMosaicCells(mosaic_cells_fname,totalX,totalY,mosaic_cells);

    osg::Matrix viewProj;
    readMatrixToScreen(matname,viewProj);

        bool outputPyr=true;
        string imgFile;
        if(arguments.read("-img",imgFile))
            outputPyr=false;
            createVTAtlas( viewProj, totalX, totalY,
                         mosaic_cells,
                       outputPyr,scaleFactor,basedir,imgFile);
   osg::Timer_t after_computeMax = osg::Timer::instance()->tick();

   cout << "Time for vipsAtlas = " << osg::Timer::instance()->delta_s(before_computeMax, after_computeMax) <<endl;


}
