#include "Extents.h"
#include "TexturingQuery.h"
#include <osg/Matrix>
#include <osg/Geometry>
#include <osg/PrimitiveSet>
#include <map>
#include <osg/Texture2DArray>
#include <osgDB/FileUtils>
#include <osg/io_utils>
#include <osg/TriangleIndexFunctor>
using namespace SpatialIndex;
using namespace std;




TexturingQuery::TexturingQuery(std::string bbox_file,const Camera_Calib &calib,TexPyrAtlas &atlasGen,bool useTextureArray) : _bbox_file(bbox_file),
_calib(calib),
_origImageSize(1360,1024),
_useTextureArray(useTextureArray),
_atlasGen(atlasGen),
_useAtlas(true)
{
    _vertexAlias = AttributeAlias(0, "osg_Vertex");

    baseName="tmpStore";
    char tmp[1024];
    sprintf(tmp,"%d",rand());
    utilization=0.7;
    capacity=4;
    baseName=tmp;
    memstore = StorageManager::createNewMemoryStorageManager();
    // Create a new storage manager with the provided base name and a 4K page size.

    manager = StorageManager::createNewRandomEvictionsBuffer(*memstore, 10, false);
    // applies a main memory random buffer on top of the persistent storage manager
    // (LRU buffer, etc can be created the same way).
    stream = new MyDataStream(_bbox_file,_cameras);

    // Create and bulk load a new RTree with dimensionality 3, using "file" as
    // the StorageManager and the RSTAR splitting policy.
    id_type indexIdentifier;
    tree = RTree::createAndBulkLoadNewRTree(
            RTree::BLM_STR, *stream, *manager, utilization, capacity,capacity, 3, SpatialIndex::RTree::RV_RSTAR, indexIdentifier);

    // std::cerr << *tree;
    //std::cerr << "Buffer hits: " << file->getHits() << std::endl;
    //std::cerr << "Index ID: " << indexIdentifier << std::endl;

    bool ret = tree->isIndexValid();
    if (ret == false) std::cerr << "ERROR: Structure is invalid!" << std::endl;
    //  else std::cerr << "The stucture seems O.K." << std::endl;


}
TexturingQuery::~TexturingQuery(){
    delete tree;
    delete manager;
    delete memstore;
    delete stream;
}
const CamDists TexturingQuery::getClosest(std::vector<int> tri_v,const osg::Vec3Array &verts){
    CamDists orderedProj;
    if(tri_v.size() != 3){
        fprintf(stderr,"Error not 3 verts\n");
        return orderedProj;
    }
    osg::Vec3 avgV;
    std::multimap<SpatialIndex::id_type,int> cam_counter;
    for(unsigned int i=0; i<tri_v.size(); i++){
        avgV += verts[tri_v[i]];
        pair<multimap<int, ProjectionCamera>::iterator, multimap<int, ProjectionCamera>::iterator> ppp;
        ppp=reproj.equal_range(tri_v[i]);
        for (multimap<int, ProjectionCamera>::iterator it2 = ppp.first;
             it2 != ppp.second;
             ++it2){
            //   printf("here %d %d\n",(*it2).second.id,i);
            cam_counter.insert(make_pair<long,int>((*it2).second.id,i));
        }
    }
    avgV.x()/=3.0;
    avgV.y()/=3.0;
    avgV.z()/=3.0;

    // printf("Size of original %d %d\n",cam_counter.size(),tri_v.size());
    set<SpatialIndex::id_type> seeninalltri;
    for (multimap<SpatialIndex::id_type, int>::iterator it = cam_counter.begin();
    it != cam_counter.end();
    ++it){
        if(cam_counter.count((*it).first) == 3){
            seeninalltri.insert((*it).first);
        }
    }

    for(set<SpatialIndex::id_type>::iterator it=seeninalltri.begin(); it != seeninalltri.end(); ++it){
        CamProjAndDist tmp;
        findCamProjAndDist(tmp,avgV,(*it));
        orderedProj.push_back(tmp);
    }
    std::sort(orderedProj.begin(), orderedProj.end(), sort_pred());
    /* printf("New Tri %d\n",orderedProj.size());
    for(int i=0; i < orderedProj.size(); i++)
        printf("ID: %d Dist: %f\n",orderedProj[i].first,orderedProj[i].second);
   // printf("Size of reproject %d\n",seeninalltri.size());
*/
    return orderedProj;
}
osg::Vec2 TexturingQuery::convertToUV(const osg::Vec2 &pix){
    osg::Vec2 ratioPow2;
    osg::Vec2 pow2size(osg::Image::computeNearestPowerOfTwo(_origImageSize.x()),osg::Image::computeNearestPowerOfTwo(_origImageSize.y()));
    ratioPow2.x()=(pow2size.x()/_origImageSize.x());
    ratioPow2.y()=(pow2size.y()/_origImageSize.y());
    ratioPow2.x()*=pix.x();
    ratioPow2.y()*=pix.y();
    osg::Vec2 uv;
    uv.x()=ratioPow2.x()/pow2size.x();
    uv.y()=1.0-(ratioPow2.y()/pow2size.y());
    return uv;
}

void TexturingQuery::findCamProjAndDist(CamProjAndDist &cpad,osg::Vec3 v,SpatialIndex::id_type id){
    cpad.id=id;
    osg::Vec2 texC=reprojectPt(_cameras[id].m,v);
    if(texC.x() < 0.0 || texC.x() > _origImageSize.x() || texC.y() < 0.0 || texC.y() > _origImageSize.y() )
        cpad.dist=DBL_MAX;
    else
        cpad.dist=(texC-osg::Vec2(_origImageSize.x()/2,_origImageSize.y()/2)).length2();
    cpad.uv=convertToUV(texC);

}
double TexturingQuery::getDistToCenter(osg::Vec3 v, ProjectionCamera cam){
    osg::Vec2 texC=reprojectPt(cam.m,v);
    if(texC.x() < 0.0 || texC.x() > _origImageSize.x() || texC.y() < 0.0 || texC.y() > _origImageSize.y() )
        return DBL_MAX;
    else
        return (texC-osg::Vec2(_origImageSize.x()/2.0,_origImageSize.y()/2.0)).length2();
}


bool TexturingQuery::projectAllTriangles(osg::Vec4Array* camIdxArr,osg::Vec2Array* texCoordsArray,
                                         const osg::PrimitiveSet& prset, const osg::Vec3Array &verts){
    int numIdx=prset.getNumIndices();
    osg::Timer_t before_computeMax = osg::Timer::instance()->tick();



    if(numIdx < 3 || !camIdxArr)
        return false;

    //Reproject all verticies
    for(unsigned int i=0; i<verts.size(); i++){
        double pt[3];
        pt[0]=verts[i].x();
        pt[1]=verts[i].y();
        pt[2]=verts[i].z();
        Point p = Point(pt, 3);
        ObjVisitor vis;
        // cout << p <<endl;
        tree->intersectsWithQuery(p,vis);
        if(vis.GetResultCount()){
            for(unsigned int r=0; r < vis.GetResults().size(); r++){
                if(_cameras.count(vis.GetResults()[r]))
                    reproj.insert(std::pair<int,ProjectionCamera>(i,_cameras[vis.GetResults()[r]]));
            }
        }
    }
    camIdxArr->resize(verts.size(),osg::Vec4(-1,-1,-1,-1));
    if(texCoordsArray)
        texCoordsArray->resize(verts.size());
    multimap<unsigned int, int> vert_reproj;

    for(int i=0; i<numIdx-2; i+=3){

        vector<int> tri_v;
        for(int k=0; k <3; k++){
            tri_v.push_back(prset.index(i+k));
        }
        const CamDists d=getClosest(tri_v,verts);
        osg::Vec4 camIdx(-1,-1,-1,-1);
        for(int v=0; v<(int)d.size() && v <4; v++){
            if(d[v].id >= 0)
                camIdx[v]=d[v].id;
        }
        for(int k=0; k <3; k++){
/*
            if(d[0].id <0){
                printf("REmap\n");
                camIdxArr->at(prset.index(i+k))=osg::Vec4(-1,-1,-1,-1);
                if(texCoordsArray && d.size()) {
                    texCoordsArray->at(prset.index(i+k))=osg::Vec2(-1,-1);
                }

            }else*/{
                vert_reproj.insert(make_pair<unsigned int, int >(prset.index(i+k),1));
                camIdxArr->at(prset.index(i+k))=camIdx;
                if(texCoordsArray && d.size()) {
                    texCoordsArray->at(prset.index(i+k))=convertToUV(reprojectPt(_cameras[d[0].id].m,verts[tri_v[k]]));
                }
            }
        }
    }

    map<SpatialIndex::id_type,int> allIds=calcAllIds(camIdxArr);
    addImagesToAtlasGen(allIds);
    //cout << "Number of prim" << prset.getNumIndices() << " " << prset.getNumPrimitives() << endl;

    osg::Timer_t after_computeMax = osg::Timer::instance()->tick();

    OSG_NOTICE << "Time for projectAllTriangles = " << osg::Timer::instance()->delta_s(before_computeMax, after_computeMax) <<endl;

    return true;
}
bool loadShaderSource(osg::Shader* obj, const std::string& fileName )
{
    std::string fqFileName = osgDB::findDataFile(fileName);
    if( fqFileName.length() == 0 )
    {
        std::cout << "File \"" << fileName << "\" not found." << std::endl;
        return false;
    }
    bool success = obj->loadShaderSourceFromFile( fqFileName.c_str());
    if ( !success  )
    {
        std::cout << "Couldn't load file: " << fileName << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}
void TexturingQuery::addImagesToAtlasGen(map<SpatialIndex::id_type,int> allIds){

    std::vector<std::pair<SpatialIndex::id_type,string> > files(allIds.size());
    map<SpatialIndex::id_type,int>::const_iterator end = allIds.end();
    for (map<SpatialIndex::id_type,int>::const_iterator it = allIds.begin(); it != end; ++it)
    {   if(_cameras.count(it->first))
        files[it->second]=make_pair<SpatialIndex::id_type,string> (it->first,_cameras[it->first].filename);
        else
            OSG_ALWAYS << " Failed " <<it->first << " : " << it->second << '\n';
}

    _atlasGen.addSources(files);
}


osg::Vec2 TexturingQuery::reprojectPt(const osg::Matrixf &mat,const osg::Vec3 &v){

    osg::Vec3 cam_frame=mat*v;
    osg::Vec3 und_n;
    und_n.x()=cam_frame.x()/cam_frame.z();
    und_n.y()=cam_frame.y()/cam_frame.z();
    double r_2 = pow(und_n.x(),2) + pow(und_n.y(),2);
    double k_radial = 1 + _calib.kc1*r_2 + _calib.kc2*pow(r_2,2) + _calib.kc5*pow(r_2,3);
    double delta_x = 2*_calib.kc3*und_n.x()*und_n.y() + _calib.kc4*(r_2 + 2*pow(und_n.x(),2));
    double delta_y = _calib.kc3*(r_2 + 2*pow(und_n.y(),2)) + 2*_calib.kc4*und_n.x()*und_n.y();
    osg::Matrix m1=osg::Matrix::scale(_calib.fcx,_calib.fcy,1);
    osg::Matrix m2=osg::Matrix::translate(_calib.ccx,_calib.ccy,0);
    osg::Matrix k1=osg::Matrix::scale(k_radial,k_radial,1);
    osg::Matrix delta=osg::Matrix::translate(delta_x,delta_y,0);
    osg::Vec3 norm_c=k1*und_n*delta;
    osg::Vec3 pixel_c=(m1*norm_c*m2);
    return osg::Vec2(pixel_c.x(),pixel_c.y());
}
map<SpatialIndex::id_type,int> calcAllIds(osg::Vec4Array *v){

    map<SpatialIndex::id_type,int> allIds;
    unsigned int uniqueIdCount=0;
    for(int i=0; i< (int)v->size(); i++){
        for(int j=0; j< 4; j++){
            int id=(int)((*v)[i][j]);
            if(id < 0)
                continue;
            if(allIds.count(id) == 0){
                allIds[id]=uniqueIdCount++;
            }
        }
    }
    return allIds;
}





class TriangleIndexVisitor
{
public:
    TriangleIndexVisitor(){newVertCounter=0;}
    set<int> indices_counted;
    map<int,int> indices_double_counted;
    vector<int> new_list;
    int newVertCounter;

    void operator()(const int v1, const int v2, const int v3)
    {
        new_list.push_back(v1);
        new_list.push_back(v2);
        new_list.push_back(v3);

        // toss the computed indices into the indices array
        if(indices_counted.count(v1) == 0)
            indices_counted.insert( v1 );
        else{
            indices_double_counted[v1]=newVertCounter++;
        }
        if(indices_counted.count(v2) == 0)
            indices_counted.insert( v2 );
        else
            indices_double_counted[v2]=newVertCounter++;
        if(indices_counted.count(v3) == 0)
            indices_counted.insert( v3 );
        else
            indices_double_counted[v3]=newVertCounter++;

    }
};
/*void TexturingQuery::remapSharedVert(osg::PrimitiveSet& prset, osg::Vec3Array &verts,std::map<int,int> remap){
    int origVertSize=verts.size();
    osg::DrawElementsUInt* elements
            = new osg::DrawElementsUInt(GL_TRIANGLES, taf._in_indices.begin(),
                               taf._in_indices.end());
    new_primitives.push_back(elements);
    for(int i=0; i<prset.getNumIndices()-2; i+=3){
        for(int k=0; k< 3; k++){
            if(remap.count(prset.index(i+k)))
                prset.index(i+k)=(origVertSize+remap[prset.index(i+k)]);
                verts->push_back(verts.at(prset.index(i+k)));
        }
    }
}*/

void TexturingQuery::projectModel(osg::Geode *geode,int texSizeIdx){
    if(!geode){
        fprintf(stderr,"Not valid geode\n");
        return;
    }
    for (unsigned int i=0; i<geode->getNumDrawables(); i++){
        reproj.clear();

        osg::Drawable *drawable = geode->getDrawable(i);
        osg::TriangleIndexFunctor<TriangleIndexVisitor> tif;
        drawable->accept(tif);


      //  printf("Size %d",(int)tif.indices_double_counted.size());
        osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
        geom->setUseDisplayList(false);
        osg::Vec3Array *verts=static_cast<const osg::Vec3Array*>(geom->getVertexArray());

        int origSize=tif.new_list.size();
      //  verts->resize(origSize+tif.indices_double_counted.size());
        for(int i=0; i<origSize; i++){
            if(tif.indices_double_counted.count(tif.new_list[i])){
               verts->push_back(verts->at(tif.new_list[i]));
               tif.new_list[i]=verts->size()-1;
            }
        }
        osg::DrawElementsUInt* elements
                = new osg::DrawElementsUInt(GL_TRIANGLES, tif.new_list.begin(),
                                   tif.new_list.end());
        geom->setPrimitiveSet(0,elements);

        //setVertexAttrib(*geom, _vertexAlias, geom->getVertexArray(), false, osg::Geometry::BIND_PER_VERTEX);
        // geom->setVertexArray(0);
        if(!verts || !verts->size()){
            OSG_NOTICE << "Empty mesh continuing!" <<endl;
            continue;
        }
        OSG_NOTICE << "\tModel Size: "<< verts->size()<<endl;

        osg::Geometry::PrimitiveSetList& primitiveSets = geom->getPrimitiveSetList();
        osg::Geometry::PrimitiveSetList::iterator itr;
        osg::Vec4Array *v= new osg::Vec4Array;

        osg::Vec2Array* texCoords=new osg::Vec2Array;
#warning "memory leak"
        osg::ref_ptr<osg::StateSet> stateset;
        bool projectValid=false;

        for(itr=primitiveSets.begin(); itr!=primitiveSets.end(); ++itr){
            switch((*itr)->getMode()){
            case(osg::PrimitiveSet::TRIANGLES):
                //remapSharedVert(*(*itr), *verts,tif.indices_double_counted);
                projectValid=projectAllTriangles(v,texCoords,*(*itr), *verts);
                if(!v||!projectValid){
                    OSG_FATAL << "Failed to create reprojection array" <<endl;
                    continue;
                }
                    _tile->texCoordIDIndexPerModel.push_back(v);
                    _tile->texCoordsPerModel.push_back(texCoords);

                break;
        default:
                OSG_FATAL << "Freakout shouldn't be anything but triangles\n";
            }
        }
        if(!_useTextureArray){
            vector<osg::Geometry*> geoms;

            /*generateStateAndSplitDrawables(geoms,v,*(primitiveSets.begin()->get()),texCoords,*verts,texSizeIdx);
            geode->removeDrawables(0);
            for(int i=0; i < (int)geoms.size(); i++)
                geode->addDrawable(geoms[i]);*/
            return;
        }
    }
}
