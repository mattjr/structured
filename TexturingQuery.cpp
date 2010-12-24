#include "TexturingQuery.h"
#include <osg/Matrix>
#include <osg/Geometry>
#include <osg/PrimitiveSet>
#include <map>
#include <osg/Texture2DArray>
#include <osgDB/FileUtils>
#include <osg/io_utils>
#include "libsnapper/auv_camera_geometry.hpp"
using namespace SpatialIndex;
using namespace std;
// example of a Visitor pattern.
// findes the index and leaf IO for answering the query and prints
// the resulting data IDs to stdout.

ObjVisitor::ObjVisitor(): nResults(0)
{
}

ObjVisitor::~ObjVisitor()
{
    /*std::vector<id_type>::iterator it;
    for (it = m_vector.begin(); it != m_vector.end(); it++) {
        delete *it;
    }*/

}

void ObjVisitor::visitNode(const SpatialIndex::INode& n)
{
    if (n.isLeaf()) m_leafIO++;
    else m_indexIO++;
}

void ObjVisitor::visitData(const SpatialIndex::IData& d)
{
    /*SpatialIndex::IShape* pS;
    d.getShape(&pS);
    SpatialIndex::Region *r = new SpatialIndex::Region();
    pS->getMBR(*r);
     std::cout <<"found shape: " << *r << " dimension: " <<pS->getDimension() << std::endl;*/
    /*

    // data should be an array of characters representing a Region as a string.
    uint8_t* data = 0;
    uint32_t length = 0;
    d.getData(length, &data);

    id_type item = new Item(d.getIdentifier());
    item->SetData(data, length);
    item->SetBounds(r);


    delete pS;
    delete r;
    delete[] data;
*/
    nResults += 1;

    m_vector.push_back(d.getIdentifier());
}

void ObjVisitor::visitData(std::vector<const SpatialIndex::IData*>& v)
{
    // std::cout << v[0]->getIdentifier() << " " << v[1]->getIdentifier() << std::endl;
}



class MyDataStream : public IDataStream
{
public:
    MyDataStream(std::string inputFile,TexturingQuery::CameraVector &cam) : m_pNext(0),m_camVec(cam)
    {
        m_fin.open(inputFile.c_str());

        if (! m_fin)
            throw Tools::IllegalArgumentException("Input file not found.");

        readNextEntry();
    }

    virtual ~MyDataStream()
    {
        if (m_pNext != 0) delete m_pNext;
    }

    virtual IData* getNext()
    {
        if (m_pNext == 0) return 0;

        RTree::Data* ret = m_pNext;
        m_pNext = 0;
        readNextEntry();
        return ret;
    }

    virtual bool hasNext()
    {
        return (m_pNext != 0);
    }

    virtual uint32_t size()
    {
        throw Tools::NotSupportedException("Operation not supported.");
    }

    virtual void rewind()
    {
        if (m_pNext != 0)
        {
            delete m_pNext;
            m_pNext = 0;
        }

        m_fin.seekg(0, std::ios::beg);
        readNextEntry();
    }

    void readNextEntry()
    {
        double low[3], high[3];
        TexturingQuery::ProjectionCamera cam;
        osg::Matrix m;
        m_fin >> cam.id >> cam.filename >> low[0] >> low[1] >> low[2] >> high[0] >> high[1] >> high[2]
                >> m(0,0) >>m(0,1)>>m(0,2) >>m(0,3)
                >> m(1,0) >>m(1,1)>>m(1,2) >>m(1,3)
                >> m(2,0) >>m(2,1)>>m(2,2) >>m(2,3)
                >> m(3,0) >>m(3,1)>>m(3,2) >>m(3,3);
        if (m_fin.good())
        {
            cam.m=osg::Matrix::inverse(m);
            cam.bb = osg::BoundingBox(low[0],low[1],low[2],high[0],high[1],high[2]);
            m_camVec[cam.id]=cam;
            /*if (op != INSERT)
                                throw Tools::IllegalArgumentException(
                                        "The data input should contain insertions only."
                                );*/

            Region r(low, high, 3);
            m_pNext = new RTree::Data(sizeof(double), reinterpret_cast<byte*>(low), r, cam.id);
            // Associate a bogus data array with every entry for testing purposes.
            // Once the data array is given to RTRee:Data a local copy will be created.
            // Hence, the input data array can be deleted after this operation if not
            // needed anymore.
        }
    }

    std::ifstream m_fin;
    RTree::Data* m_pNext;
    TexturingQuery::CameraVector &m_camVec;


};

TexturingQuery::TexturingQuery(std::string bbox_file,const Camera_Calib &calib) : _bbox_file(bbox_file),_calib(calib),   _origImageSize(1360,1024)
{
    _useTextureArray=false;
    _vertexAlias = AttributeAlias(0, "osg_Vertex");
    _projCoordAlias = AttributeAlias(1, "osg_ProjCoord");
    _atlasGen = new TexPyrAtlas("/home/mattjr/auvdata/r20090804_084719_scott_25_dense_repeat_auv5_deep/renav20090804/mesh/img");
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
    camIdxArr->resize(verts.size());
    if(texCoordsArray)
        texCoordsArray->resize(verts.size());

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
            camIdxArr->at(prset.index(i+k))=camIdx;
            if(texCoordsArray && d.size()) {
                texCoordsArray->at(prset.index(i+k))=convertToUV(reprojectPt(_cameras[d[0].id].m,verts[tri_v[k]]));
            }
        }
    }

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
std::vector<osg::ref_ptr<osg::Image> >TexturingQuery::loadTex(map<SpatialIndex::id_type,int> allIds,int sizeIdx){
    osg::Timer_t before_computeMax = osg::Timer::instance()->tick();

    std::vector<osg::ref_ptr<osg::Image> > images(allIds.size());
    std::vector<std::pair<SpatialIndex::id_type,string> > files(allIds.size());
    map<SpatialIndex::id_type,int>::const_iterator end = allIds.end();
    for (map<SpatialIndex::id_type,int>::const_iterator it = allIds.begin(); it != end; ++it)
    {
        files[it->second]=make_pair<SpatialIndex::id_type,string> (it->first,_cameras[it->first].filename);
        //   std::cout << " loading " <<it->first << " : " << it->second << '\n';
    }

    _atlasGen->loadSources(files);
    end = allIds.end();
    for (map<SpatialIndex::id_type,int>::const_iterator it = allIds.begin(); it != end; ++it)
    {
        images[it->second]=_atlasGen->getImage(it->second,sizeIdx);
    }
    osg::Timer_t after_computeMax = osg::Timer::instance()->tick();

    OSG_NOTICE << "Time for loadTex = " << osg::Timer::instance()->delta_s(before_computeMax, after_computeMax) <<endl;

    return images;
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

osg::StateSet *TexturingQuery::generateStateAndArray2DRemap( osg::Vec4Array *v,int texSizeIdx){
    if(!v)
        return NULL;
    map<SpatialIndex::id_type,int> allIds;
    unsigned int uniqueIdCount=0;
    vector<osg::Matrixf> arrayProjMat;
    osg::StateSet *stateset= new osg::StateSet;
    for(int i=0; i< (int)v->size(); i++){
        for(int j=0; j< 4; j++){
            int id=(int)((*v)[i][j]);
            if(id < 0)
                continue;
            if(allIds.count(id) == 0){
                allIds[id]=uniqueIdCount++;
                arrayProjMat.push_back(_cameras[id].m);
            }
        }
    }

    osg::ref_ptr<osg::Uniform> arrayProjUniform = new osg::Uniform(osg::Uniform::FLOAT_MAT4,
                                                                   "projMatrices", arrayProjMat.size());
    for (unsigned int i = 0; i < arrayProjMat.size(); ++i)
        arrayProjUniform->setElement(i, arrayProjMat[i]);
    stateset->addUniform(arrayProjUniform);

    //Remap
    for(int i=0; i< (int)v->size(); i++){
        for(int j=0; j< 4; j++){
            int id=(int)((*v)[i][j]);
            if(id >= 0 && allIds.count(id)){
                (*v)[i][j]=allIds[id];
            }
        }
    }


    int tex_size=_atlasGen->getDownsampleSize(texSizeIdx);

    std::vector<osg::ref_ptr<osg::Image> > textures= loadTex(allIds,texSizeIdx);

    OSG_ALWAYS << "\tTexture Array Size: " <<uniqueIdCount <<endl;
    OSG_ALWAYS << "\tImage Size: " <<tex_size <<endl;
    if(uniqueIdCount != textures.size())
        OSG_FATAL << "uniqueIdCount != textures.size())" <<uniqueIdCount <<" "<< textures.size()<< endl;

    osg::ref_ptr<osg::Texture2DArray> textureArray =new osg::Texture2DArray;
    osg::Program* program = new osg::Program;
    program->setName( "projective_tex" );
    osg::ref_ptr<osg::Shader> lerpF=new osg::Shader( osg::Shader::FRAGMENT);
    osg::ref_ptr<osg::Shader> lerpV=new osg::Shader( osg::Shader::VERTEX);
    loadShaderSource( lerpF, "/home/mattjr/svn/threadedStereo-vpb/projV.frag" );
    loadShaderSource( lerpV, "/home/mattjr/svn/threadedStereo-vpb/projV.vert" );
    program->addShader(  lerpF );
    program->addShader(  lerpV );
    program->addBindAttribLocation(_projCoordAlias.second,_projCoordAlias.first);
    textureArray->setTextureSize(tex_size,tex_size,uniqueIdCount);

    for(int i=0; i < (int)textures.size(); i++)
        textureArray->setImage(i,textures[i].get());

    stateset->setAttributeAndModes( program, osg::StateAttribute::ON );
    stateset->addUniform( new osg::Uniform("theTexture", TEXUNIT_ARRAY) );

    stateset->addUniform( new osg::Uniform("fc_cc", osg::Vec4(_calib.fcx,_calib.fcy,
                                                              _calib.ccx,_calib.ccy)) );


    stateset->addUniform( new osg::Uniform("kc1234", osg::Vec4(_calib.kc1,_calib.kc2,_calib.kc3,
                                                               _calib.kc4)) );

    stateset->addUniform( new osg::Uniform("kc5", (float)_calib.kc5 ));
    stateset->setTextureAttribute(TEXUNIT_ARRAY, textureArray.get());
    stateset->setDataVariance(osg::Object::STATIC);

    return stateset;
}


void TexturingQuery::generateStateAndSplitDrawables(vector<osg::Geometry*> &geoms,osg::Vec4Array *v, const osg::PrimitiveSet& prset,
                                                    osg::Vec2Array* texCoordsArray,
                                                    const osg::Vec3Array &verts,int texSizeIdx){
    if(!v)
        return;

    map<SpatialIndex::id_type,int> allIds;
    unsigned int uniqueIdCount=0;
    for(int i=0; i< (int)v->size(); i++){
        int id=(int)((*v)[i][0]);
        if(id < 0)
            continue;
        if(allIds.count(id) == 0){
            allIds[id]=uniqueIdCount++;
        }
    }

    //int tex_size=_atlasGen->getDownsampleSize(texSizeIdx);
    std::vector<osg::ref_ptr<osg::Image> > textures= loadTex(allIds,texSizeIdx);
    std::vector<osg::DrawElementsUInt *>  primsets(_atlasGen->getNumAtlases());;
    std::vector<osg::Vec3Array *>  vertSplit(_atlasGen->getNumAtlases());
    std::vector<osg::Vec2Array *> texSplit(_atlasGen->getNumAtlases());
    geoms.resize(_atlasGen->getNumAtlases());

    for(int i=0; i< (int)geoms.size(); i++){
        geoms[i]= new osg::Geometry;
        vertSplit[i]=new osg::Vec3Array;
        texSplit[i]=new osg::Vec2Array;
        geoms[i]->setVertexArray(vertSplit[i]);
        geoms[i]->setTexCoordArray(TEX_UNIT,texSplit[i]);
        primsets[i] = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES,0);
        geoms[i]->addPrimitiveSet(primsets[i]);
        geoms[i]->setUseDisplayList(false);
        osg::StateSet *stateset=geoms[i]->getOrCreateStateSet();
        osg::Image *atlasTex=_atlasGen->getAtlasByNumber(i);
        osg::ref_ptr<osg::Texture2D> texture=new osg::Texture2D(atlasTex);
        texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_BORDER);
        texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_BORDER);
        texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
        texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
        stateset->setTextureAttributeAndModes(TEX_UNIT,texture,osg::StateAttribute::ON);
        stateset->setMode( GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF );

        stateset->setDataVariance(osg::Object::STATIC);
    }
    int numIdx=prset.getNumIndices();
    printf("Num idx %d\n",numIdx);

    for(int i=0; i<numIdx-2; i+=3){
        vector<osg::Vec3> vP;
        vector<osg::Vec2> tP;
        vector<unsigned int> iP;
        int atlas=-1;
        for(int k=0; k <3; k++){
            int id=(int)((*v)[prset.index(i+k)][0]);
            if(id < 0)
                continue;
            int tmp=_atlasGen->getAtlasId(id);
            if(tmp < 0 || tmp >= (int)geoms.size()){
                OSG_ALWAYS << "Atlas mapping incorrect id: " << id << " index: " << prset.index(i+k) << endl;
                continue;
            }
            atlas=tmp;

            osg::Vec2 tc = (*texCoordsArray)[prset.index(i+k)];
            osg::Matrix matrix=_atlasGen->getTextureMatrixByID(id);
            vP.push_back((verts)[prset.index(i+k)]);
            tP.push_back(osg::Vec2(tc[0]*matrix(0,0) + tc[1]*matrix(1,0) + matrix(3,0),
                                   tc[0]*matrix(0,1) + tc[1]*matrix(1,1) + matrix(3,1)));
            iP.push_back(vertSplit[atlas]->size()+k);
        }
        if(vP.size() == 3){
            for(int k=0; k <3; k++){
                vertSplit[atlas]->push_back(vP[k]);
                texSplit[atlas]->push_back(tP[k]);
                primsets[atlas]->push_back(iP[k]);
            }
        }
    }
    /*  for(int i=0; i< (int)v->size(); i++){
        int id=(int)((*v)[i][0]);


    }*/

}
void TexturingQuery::setVertexAttrib(osg::Geometry& geom, const AttributeAlias& alias, osg::Array* array, bool normalize, osg::Geometry::AttributeBinding binding)
{
    if(!array)
        return;
    unsigned int index = alias.first;
    const std::string& name = alias.second;
    array->setName(name);
    geom.setVertexAttribArray(index, array);
    geom.setVertexAttribNormalize(index, normalize);
    geom.setVertexAttribBinding(index, binding);

    osg::notify(osg::NOTICE)<<"   vertex attrib("<<name<<", index="<<index<<", normalize="<<normalize<<" binding="<<binding<<")"<<std::endl;
}
void TexturingQuery::projectModel(osg::Geode *geode,int texSizeIdx){
    if(!geode){
        fprintf(stderr,"Not valid geode\n");
        return;
    }
    for (unsigned int i=0; i<geode->getNumDrawables(); i++){
        reproj.clear();

        osg::Drawable *drawable = geode->getDrawable(i);
        osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
        geom->setUseDisplayList(false);
        osg::Vec3Array *verts=static_cast<const osg::Vec3Array*>(geom->getVertexArray());
        //setVertexAttrib(*geom, _vertexAlias, geom->getVertexArray(), false, osg::Geometry::BIND_PER_VERTEX);
        // geom->setVertexArray(0);
        if(!verts || !verts->size()){
            OSG_NOTICE << "Empty mesh continuing!" <<endl;
            continue;
        }
        OSG_NOTICE << "\tModel Size: "<< verts->size()<<endl;

        osg::Geometry::PrimitiveSetList& primitiveSets = geom->getPrimitiveSetList();
        osg::Geometry::PrimitiveSetList::iterator itr;
        osg::ref_ptr<osg::Vec4Array> v= new osg::Vec4Array;

        osg::ref_ptr<osg::Vec2Array> texCoords;
        if(!_useTextureArray)
            texCoords=new osg::Vec2Array;

        osg::ref_ptr<osg::StateSet> stateset;
        bool projectValid=false;

        for(itr=primitiveSets.begin(); itr!=primitiveSets.end(); ++itr){
            switch((*itr)->getMode()){
            case(osg::PrimitiveSet::TRIANGLES):
                projectValid=projectAllTriangles(v,texCoords.get(),*(*itr), *verts);
                if(!v.valid()||!projectValid){
                    OSG_FATAL << "Failed to create reprojection array" <<endl;
                    continue;
                }
                if(_useTextureArray){
                    stateset=generateStateAndArray2DRemap(v,texSizeIdx);
                    setVertexAttrib(*geom,_projCoordAlias,v,false,osg::Geometry::BIND_PER_VERTEX);

                    if(stateset.valid())
                        geom->setStateSet(stateset);
                    else
                        OSG_FATAL << "Failed to create state set for texture array" <<endl;
                }
                break;
        default:
                OSG_FATAL << "Freakout shouldn't be anything but triangles\n";
            }
        }
        if(!_useTextureArray){
            vector<osg::Geometry*> geoms;

            generateStateAndSplitDrawables(geoms,v,*(primitiveSets.begin()->get()),texCoords.get(),*verts,0);
            geode->removeDrawables(0);
            for(int i=0; i < (int)geoms.size(); i++)
                geode->addDrawable(geoms[i]);
            return;
        }
    }
}
