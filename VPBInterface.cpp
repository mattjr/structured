#include "VPBInterface.hpp"
#include <string>
#include <osgDB/ReadFile>
#include "TexturingQuery.h"
using namespace std;
void doQuadTreeVPB(std::string cacheddir,std::vector<std::vector<string> > datalist_lod,Bounds bounds,Camera_Calib &calib,bool useTextureArray){

    vpb::GeospatialExtents geo(bounds.min_x, bounds.min_y, bounds.max_x,bounds.max_y,false);
    int numlod=datalist_lod.size()-1;
    vpb::MyDataSet *m=new vpb::MyDataSet(calib,useTextureArray);
    m->setNumReadThreadsToCoresRatio(1.5);
    m->setNumWriteThreadsToCoresRatio(1.5);
    //m->setCompressionMethod(vpb::BuildOptions::RGB_S3TC_DXT1);
    m->setRadiusToMaxVisibleDistanceRatio(4.0);

    m->setDestinationName("real.ive");

    m->setLogFileName("tmp.log");
    osgDB::Registry::instance()->setBuildKdTreesHint(osgDB::ReaderWriter::Options::BUILD_KDTREES);
    for(int lod=0; lod < (int)datalist_lod.size(); lod++){
        for(int i=0; i<(int)datalist_lod[lod].size(); i++){
            if(!osgDB::fileExists(datalist_lod[lod][i]))
                continue;
            TexturedSource *sourceModel=new TexturedSource(vpb::Source::MODEL,datalist_lod[lod][i]);
            sourceModel->setMaxLevel(lod);
            sourceModel->setMinLevel(lod);
            sourceModel->tex_cache_dir=cacheddir;
            sourceModel->setCoordinateSystem(new osg::CoordinateSystemNode("WKT",""));
            osg::Node* model = osgDB::readNodeFile(sourceModel->getFileName().c_str());
            if (model)
            {
                vpb::SourceData* data = new vpb::SourceData(sourceModel);
                data->_model = model;
                data->_extents.expandBy(model->getBound());
                sourceModel->setSourceData(data);
                osg::Geode *geode= dynamic_cast<osg::Geode*>(model);
                if(geode && geode->getNumDrawables()){
                    osg::Drawable *drawable = geode->getDrawable(0);
                    sourceModel->_kdTree = dynamic_cast<osg::KdTree*>(drawable->getShape());
                }else{
                    OSG_ALWAYS << "No drawbables \n";
                }
            }


            m->addSource(sourceModel,1);
        }
    }
    m->createNewDestinationGraph(geo,256,128,numlod);
    m->_run();

}
