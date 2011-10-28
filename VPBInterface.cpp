#include "VPBInterface.hpp"
#include <string>
#include <osgDB/ReadFile>
#include "TexturingQuery.h"
#include <osg/Vec4>
#include "vertexData.h"
using namespace std;
#if 1
bool toVert(osg::Node *node,const TexBlendCoord &texcoord,osg::Vec4Array *ids,TexBlendCoord &newTexCoord,osg::Vec4Array *newIds){
    int vertsAdded=0;

    osg::Geode *geode=dynamic_cast<osg::Geode*>(node);
    //No cached
    if(!geode){
        osg::notify(osg::ALWAYS) << "Not valid geode\n";
        assert(0);
        return false;
    }

    osg::Drawable *drawable = geode->getDrawable(0);

    osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
    osg::Vec3Array *verts=static_cast<const osg::Vec3Array*>(geom->getVertexArray());
    //osg::Vec2Array *texCoords=static_cast<const osg::Vec2Array*>(geom->getTexCoordArray(0));
    newTexCoord.resize(texcoord.size());
    newTexCoord[0]=new osg::Vec3Array;
    if(texcoord.size() > 1){
        newTexCoord[1]=new osg::Vec3Array;
        newTexCoord[2]=new osg::Vec3Array;
        newTexCoord[3]=new osg::Vec3Array;
        newIds->resize(verts->size(),osg::Vec4(-1,-1,-1,-1));

    }
    for(int f=0; f< (int)texcoord.size(); f++)
        newTexCoord[f]->resize(verts->size());
    if(!verts || !verts->size()){
        osg::notify(osg::INFO)<< "Empty mesh continuing!" <<endl;
        //continue;
        return false;
    }
    osg::notify(osg::INFO) << "\tModel Size: "<< verts->size()<<endl;
    osg::Geometry::PrimitiveSetList& primitiveSets = geom->getPrimitiveSetList();
    osg::PrimitiveSet *prim=primitiveSets.begin()->get();
    //printf("%d %d\n",texcoord[0]->size() ,prim->getNumIndices() );
    assert(texcoord[0]->size() ==prim->getNumIndices() );
    int numIdx=prim->getNumIndices();
    vector<unsigned int>new_list;
    idbackmap_t idmap;
    for(int i=0; i<numIdx-2; i+=3){
        for(int k=0; k <3; k++){
            int newidx=prim->index(i+k);
            if(texcoord.size() > 1){
                if(idmap.count(newidx) == 0){
                    idmap[newidx]=ids->at(i+k)[0];
                }
                else if(idmap[newidx] != ids->at(i+k)[0]){
                    osg::Vec3 v=verts->at(newidx);
                    newidx=verts->size();
                    verts->push_back(v);
                    vertsAdded++;
                    (*dynamic_cast<osg::DrawElementsUInt*>(prim))[i+k]=newidx;
                    for(int f=0; f< (int)texcoord.size(); f++)
                        newTexCoord[f]->push_back(texcoord[f]->at(i+k));
                    if(texcoord.size() > 1)
                        newIds->push_back(ids->at(i+k));
                    continue;
                }
            }
            for(int f=0; f< (int)texcoord.size(); f++)
                newTexCoord[f]->at(newidx)=texcoord[f]->at(i+k);
            if(texcoord.size() > 1)
                newIds->at(newidx)=ids->at(i+k);
            new_list.push_back(newidx);

        }
    }
    //geom->setTexCoordArray(0,newTexCoord);

    return true;
}
#endif
void doQuadTreeVPB(std::string basePath,std::vector<std::vector<string> > datalist_lod,Bounds bounds,CameraCalib &calib,texcache_t cachedDirs,bool useTextureArray,bool useReimage,bool useVirtualTex,const osg::BoundingBox &bbox,string dst_wkt_coord_system,string src_proj4_coord_system){
    //vector<osg::KdTree*> trees;
    vpb::GeospatialExtents geo(bounds.bbox.xMin(), bounds.bbox.yMin(), bounds.bbox.xMax(),bounds.bbox.yMax(),false);
    int numlod=datalist_lod.size()-1;
    if(useVirtualTex)
        useReimage=false;
    osg::ref_ptr<vpb::MyDataSet> m=new vpb::MyDataSet(calib,basePath,useTextureArray,useReimage,useVirtualTex);
    m->_cachedDirs=cachedDirs;
    m->_zrange=osg::Vec4(bbox.zMin(),bbox.zMax(),bbox.zMin(),bbox.zMax());
    m->setNumReadThreadsToCoresRatio(1.5);
    m->setNumWriteThreadsToCoresRatio(1.5);
    m->setDestinationCoordinateSystem(dst_wkt_coord_system);
    m->setSourceCoordinateSystemProj4(src_proj4_coord_system);
    // m->setCompressionMethod(vpb::BuildOptions::NVTT);
    //m->setCompressionMethod(vpb::BuildOptions::GL_DRIVER);
    //vpb::ImageOptions *imageOptions = new vpb::ImageOptions();
    //imageOptions->setTextureType(vpb::ImageOptions::RGBA);
    //m->setLayerImageOptions(0,imageOptions);
    //  m->setMaximumVisibleDistanceOfTopLevel(1e11);
    if(useVirtualTex)
        m->setRadiusToMaxVisibleDistanceRatio(7);
    else
        m->setRadiusToMaxVisibleDistanceRatio(7);

    m->setDestinationName("real.ive");

    m->setLogFileName("tmp.log");
    osgDB::Registry::instance()->setBuildKdTreesHint(osgDB::ReaderWriter::Options::BUILD_KDTREES);
    for(int lod=0/*datalist_lod.size()-1*/; lod < (int)datalist_lod.size(); lod++){
        for(int i=0; i<(int)datalist_lod[lod].size(); i++){
            if(!osgDB::fileExists(datalist_lod[lod][i]))
                continue;
            std::string mf=datalist_lod[lod][i];
            int npos=mf.find("/");

            std::string bbox_file;
            TexturedSource *sourceModel;
            if(!useVirtualTex && !useReimage){
                bbox_file=std::string(mf.substr(0,npos)+"/bbox-"+mf.substr(npos+1,mf.size()-9-npos-1)+".ply.txt");
                sourceModel =new TexturedSource(vpb::Source::MODEL,mf,bbox_file);
            }else{
                sourceModel=new TexturedSource(vpb::Source::MODEL,mf);
            }
            sourceModel->setMaxLevel(lod);
            sourceModel->setMinLevel(lod);
            sourceModel->setCoordinateSystem(new osg::CoordinateSystemNode("WKT",""));
            plyV::VertexData vertexData;
            osg::Node* model;
            if(!m->_useReImage){
                model= vertexData.readPlyFile(sourceModel->getFileName().c_str());
                toVert(model,vertexData._texCoord,vertexData._texIds,sourceModel->tex,sourceModel->ids);
            }else
                model = osgDB::readNodeFile(sourceModel->getFileName());
            //std::cerr << "aaa " << sourceModel->tex->at(0)->size() << " " << sourceModel->ids->size() <<endl;
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
                    osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
                    sourceModel->colors=(osg::Vec4Array*)geom->getColorArray();
                    sourceModel->_kdTree = dynamic_cast<osg::KdTree*>(drawable->getShape());
                    // trees.push_back(sourceModel->_kdTree);
                }else{
                    osg::notify(osg::INFO) << "No drawbables \n";
                }
            }


            m->addSource(sourceModel);
        }
    }
    m->createNewDestinationGraph(geo,256,128,numlod);
    m->_run();
    //    for(int i=0; i<trees.size(); i++)
    //    delete trees[i];


}



