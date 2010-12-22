#include "VPBInterface.hpp"
#include <string>
#include <osgDB/ReadFile>
#include "TexturingQuery.h"
using namespace std;
void doQuadTreeVPB(std::vector<std::vector<string> > datalist_lod,Bounds bounds,Camera_Calib &calib){

    vpb::GeospatialExtents geo(bounds.min_x, bounds.min_y, bounds.max_x,bounds.max_y,false);
    char filename[1024];
    int numlod=datalist_lod.size()-1;
    vpb::MyDataSet *m=new vpb::MyDataSet();
    m->_tq = new TexturingQuery("mesh-quad/bbox.txt",calib);

    m->setDestinationName("real.ive");

    m->setLogFileName("tmp.log");
        for(int lod=0; lod < datalist_lod.size(); lod++){
            for(int i=0; i<datalist_lod[lod].size(); i++){
        //    sprintf(filename,"/home/mattjr/data/new3/mesh-quad/quad-lod%d.ply",lod);

            vpb::Source *sourceModel=new vpb::Source(vpb::Source::MODEL,datalist_lod[lod][i]);
            sourceModel->setMaxLevel(numlod-lod);
            sourceModel->setMinLevel(numlod-lod);
            sourceModel->setCoordinateSystem(new osg::CoordinateSystemNode("WKT",""));
            osg::Node* model = osgDB::readNodeFile(sourceModel->getFileName().c_str());
            if (model)
            {
                vpb::SourceData* data = new vpb::SourceData(sourceModel);
                data->_model = model;
                data->_extents.expandBy(model->getBound());
                sourceModel->setSourceData(data);

            }


            m->addSource(sourceModel,1);
        }
    }
    m->createNewDestinationGraph(geo,256,128,numlod);
    m->_run();

}
