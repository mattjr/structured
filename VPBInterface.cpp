#include "VPBInterface.hpp"
#include <string>
#include <osgDB/ReadFile>
#include "TexturingQuery.h"
#include <osg/Vec4>
#include "vertexData.h"
using namespace std;
#if 1
bool toVert(osg::Node *node,osg::Vec2Array *texcoord,osg::Vec4Array *ids,osg::Vec2Array *newTexCoord,osg::Vec4Array *newIds){
    int vertsAdded=0;

    osg::Geode *geode=dynamic_cast<osg::Geode*>(node);
    //No cached
    if(!geode){
        OSG_ALWAYS << "Not valid geode\n";
        assert(0);
        return false;
    }

        osg::Drawable *drawable = geode->getDrawable(0);

        osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
        osg::Vec3Array *verts=static_cast<const osg::Vec3Array*>(geom->getVertexArray());
        osg::Vec2Array *texCoords=static_cast<const osg::Vec2Array*>(geom->getTexCoordArray(0));

        newIds->resize(verts->size(),osg::Vec4(-1,-1,-1,-1));
        newTexCoord->resize(verts->size());
        if(!verts || !verts->size()){
            OSG_INFO<< "Empty mesh continuing!" <<endl;
            //continue;
            return false;
        }
        OSG_INFO << "\tModel Size: "<< verts->size()<<endl;
        osg::Geometry::PrimitiveSetList& primitiveSets = geom->getPrimitiveSetList();
        osg::PrimitiveSet *prim=primitiveSets.begin()->get();
        assert(texCoords->size() ==prim->getNumIndices() );
        int numIdx=prim->getNumIndices();
        vector<unsigned int>new_list;
        idbackmap_t idmap;
        for(int i=0; i<numIdx-2; i+=3){
            for(int k=0; k <3; k++){
                int newidx=prim->index(i+k);
                if(idmap.count(newidx) == 0)
                    idmap[newidx]=ids->at(i+k)[0];
                else if(idmap[newidx] != ids->at(i+k)[0]){
                    osg::Vec3 v=verts->at(newidx);
                    newidx=verts->size();
                    verts->push_back(v);
                    vertsAdded++;
                    dynamic_cast<osg::DrawElementsUInt*>(prim)->setElement(i+k,newidx);
                    newTexCoord->push_back(texCoords->at(i+k));
                    newIds->push_back(ids->at(i+k));
                    continue;
                }
                newTexCoord->at(newidx)=texCoords->at(i+k);
                newIds->at(newidx)=ids->at(i+k);
                new_list.push_back(newidx);

            }
        }
        geom->setTexCoordArray(0,newTexCoord);

    return true;
}
#endif
void doQuadTreeVPB(std::string cacheddir,std::vector<std::vector<string> > datalist_lod,Bounds bounds,Camera_Calib &calib,texcache_t cachedDirs,bool useTextureArray){
vector<osg::KdTree*> trees;
    vpb::GeospatialExtents geo(bounds.min_x, bounds.min_y, bounds.max_x,bounds.max_y,false);
    int numlod=datalist_lod.size()-1;
    osg::ref_ptr<vpb::MyDataSet> m=new vpb::MyDataSet(calib,useTextureArray);
    m->_cachedDirs=cachedDirs;
    m->setNumReadThreadsToCoresRatio(0.5);
    m->setNumWriteThreadsToCoresRatio(0.5);
    //m->setCompressionMethod(vpb::BuildOptions::RGB_S3TC_DXT1);
    m->setRadiusToMaxVisibleDistanceRatio(4.0);

    m->setDestinationName("real.ive");

    m->setLogFileName("tmp.log");
    osgDB::Registry::instance()->setBuildKdTreesHint(osgDB::ReaderWriter::Options::BUILD_KDTREES);
    for(int lod=0; lod < (int)datalist_lod.size(); lod++){
        for(int i=0; i<(int)datalist_lod[lod].size(); i++){
            if(!osgDB::fileExists(datalist_lod[lod][i]))
                continue;
            std::string mf=datalist_lod[lod][i];
            int npos=mf.find("/");
            std::string bbox_file=std::string(mf.substr(0,npos)+"/bbox-"+mf.substr(npos+1,mf.size()-9-npos-1)+".ply.txt");
            TexturedSource *sourceModel=new TexturedSource(vpb::Source::MODEL,mf,bbox_file);
            sourceModel->setMaxLevel(lod);
            sourceModel->setMinLevel(lod);
            sourceModel->tex_cache_dir=cacheddir;
            sourceModel->setCoordinateSystem(new osg::CoordinateSystemNode("WKT",""));
            ply::VertexData vertexData;
            osg::Node* model = vertexData.readPlyFile(sourceModel->getFileName().c_str());
            sourceModel->ids=new osg::Vec4Array;
            sourceModel->tex=new osg::Vec2Array;


            toVert(model,vertexData._texCoord,vertexData._texIds,sourceModel->tex,sourceModel->ids);
            osg::ref_ptr<osg::KdTreeBuilder>  _kdTreeBuilder = osgDB::Registry::instance()->getKdTreeBuilder()->clone();
            model->accept(*_kdTreeBuilder);
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
                    trees.push_back(sourceModel->_kdTree);
                }else{
                    OSG_ALWAYS << "No drawbables \n";
                }
            }


            m->addSource(sourceModel,1);
        }
    }
    m->createNewDestinationGraph(geo,256,128,numlod);
    m->_run();
//    for(int i=0; i<trees.size(); i++)
  //    delete trees[i];


}
