#include "VPBInterface.hpp"
#include <string>
#include <osgDB/ReadFile>
#include "TexturingQuery.h"
void doQuadTreeVPB(Bounds bounds){

vpb::GeospatialExtents geo(bounds.min_x, bounds.min_y, bounds.max_x,bounds.max_y,false);
char filename[1024];
int numlod=5;
vpb::MyDataSet *m=new vpb::MyDataSet();
m->_tq = new TexturingQuery("/home/mattjr/data/new2/mesh-quad/bbox.txt");

m->setDestinationName("real.ive");

    m->setLogFileName("tmp.log");

    for(int lod=0; lod < numlod+1; lod++){

        sprintf(filename,"/home/mattjr/data/new2/mesh-quad/quad-lod%d.ply",lod);

        vpb::Source *sourceModel=new vpb::Source(vpb::Source::MODEL,std::string(filename));
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
m->createNewDestinationGraph(geo,256,128,numlod);
m->_run();

}
