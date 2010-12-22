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



/*class TexturingQuery::MyVisitor : public IVisitor
{
public:
        size_t m_indexIO;
        size_t m_leafIO;
        multimap<int,ProjectionCamera> reprojected;
        CameraVector &_camVec;

public:
        MyVisitor(CameraVector &cameras) : m_indexIO(0), m_leafIO(0),_camVec(cameras) {}

        void visitNode(const INode& n)
        {
                if (n.isLeaf()) m_leafIO++;
                else m_indexIO++;
        }

        void visitData(const IData& d)
        {
                IShape* pS;
                d.getShape(&pS);
                        // do something.
                delete pS;

                // data should be an array of characters representing a Region as a string.
                byte* pData = 0;
                uint32_t cLen = 0;
                d.getData(cLen, &pData);
                // do something.
                //string s = reinterpret_cast<char*>(pData);
                //cout << s << endl;
                delete[] pData;

               // cout << d.getIdentifier() << endl;
                //reprojected[d.getIdentifier()]=
                        // the ID of this data entry is an answer to the query. I will just print it to stdout.
        }

        void visitData(std::vector<const IData*>& v)
        {
                cout << v[0]->getIdentifier() << " " << v[1]->getIdentifier() << endl;
        }
};
*/
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

TexturingQuery::TexturingQuery(std::string bbox_file,Camera_Calib &calib) : _bbox_file(bbox_file),_calib(calib)
{
    _vertexAlias = AttributeAlias(0, "osg_Vertex");
    _projCoordAlias = AttributeAlias(1, "osg_ProjCoord");
    _atlasGen = new TexPyrAtlas("/home/mattjr/auvdata/r20090804_084719_scott_25_dense_repeat_auv5_deep/renav20090804/mesh/img");
    baseName="tmpStore";
    utilization=0.7;
    capacity=4;
    diskfile = StorageManager::createNewDiskStorageManager(baseName, 4096);
    // Create a new storage manager with the provided base name and a 4K page size.

    file = StorageManager::createNewRandomEvictionsBuffer(*diskfile, 10, false);
    // applies a main memory random buffer on top of the persistent storage manager
    // (LRU buffer, etc can be created the same way).
     stream = new MyDataStream(_bbox_file,_cameras);

    // Create and bulk load a new RTree with dimensionality 3, using "file" as
    // the StorageManager and the RSTAR splitting policy.
    id_type indexIdentifier;
    tree = RTree::createAndBulkLoadNewRTree(
            RTree::BLM_STR, *stream, *file, utilization, capacity,capacity, 3, SpatialIndex::RTree::RV_RSTAR, indexIdentifier);

    std::cerr << *tree;
    std::cerr << "Buffer hits: " << file->getHits() << std::endl;
    std::cerr << "Index ID: " << indexIdentifier << std::endl;

    bool ret = tree->isIndexValid();
    if (ret == false) std::cerr << "ERROR: Structure is invalid!" << std::endl;
    else std::cerr << "The stucture seems O.K." << std::endl;


}
TexturingQuery::~TexturingQuery(){
    delete tree;
    delete file;
    delete diskfile;
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
        orderedProj.push_back(make_pair<SpatialIndex::id_type,double>((*it),getDistToCenter(avgV,_cameras[(*it)])));
    }
    std::sort(orderedProj.begin(), orderedProj.end(), sort_pred());
  /* printf("New Tri %d\n",orderedProj.size());
    for(int i=0; i < orderedProj.size(); i++)
        printf("ID: %d Dist: %f\n",orderedProj[i].first,orderedProj[i].second);
   // printf("Size of reproject %d\n",seeninalltri.size());
*/
    return orderedProj;
}

double TexturingQuery::getDistToCenter(osg::Vec3 v, ProjectionCamera cam){
    //return (v-cam.bb.center()).length2();
    osg::Vec2 texC=reprojectPt(cam.m,v);
    if(texC.x() < 0.0 || texC.x() > 1360.0 || texC.y() < 0.0 || texC.y() > 1024.0 )
        return DBL_MAX;
    else
        return (texC-osg::Vec2(1360/2,1024/2)).length2();
}


osg::Vec4Array* TexturingQuery::projectAllTriangles(const osg::PrimitiveSet& prset, const osg::Vec3Array &verts){
    int numIdx=prset.getNumIndices();

    if(numIdx < 3)
        return NULL;
    osg::Vec4Array* camIdxArr = new osg::Vec4Array;

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
    for(int i=0; i<numIdx-2; i+=3){
        vector<int> tri_v;
        for(int k=0; k <3; k++){
            tri_v.push_back(prset.index(i+k));
        }
        const CamDists d=getClosest(tri_v,verts);
        osg::Vec4 camIdx(-1,-1,-1,-1);
        for(int v=0; v<(int)d.size() && v <4; v++){
            if(d[v].first >= 0)
                camIdx[v]=d[v].first;
        }
 // if(d.size())
   //    cout << "i: "<< tri_v[0] << " close "<< _cameras[d[0].first].m*verts[tri_v[0]] << " local " << d[0].first<<" " << verts[tri_v[0]] << "\n" <<_cameras[d[0].first].m <<"\n" << reprojectPt(_cameras[d[0].first].m,verts[tri_v[0]]) <<endl;
    for(int k=0; k <3; k++)
        camIdxArr->at(prset.index(i+k))=camIdx;
    }
 /*   cout << "Elements in m: " << endl;

      for (multimap<int, ProjectionCamera>::iterator it = reproj.begin();
               it != reproj.end();
           ++it)

       {
              cout << "  [" << (*it).first << ": " << reproj.count((*it).first) << "]" << endl;
       }
*/
    return camIdxArr;
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
    std::vector<osg::ref_ptr<osg::Image> > images(allIds.size());
    std::vector<string> filenames(allIds.size());
    map<SpatialIndex::id_type,int>::const_iterator end = allIds.end();
        for (map<SpatialIndex::id_type,int>::const_iterator it = allIds.begin(); it != end; ++it)
        {
           filenames[it->second]=_cameras[it->first].filename;
        //   std::cout << " loading " <<it->first << " : " << it->second << '\n';
        }

        _atlasGen->loadSources(filenames);
       end = allIds.end();
            for (map<SpatialIndex::id_type,int>::const_iterator it = allIds.begin(); it != end; ++it)
            {
              images[it->second]=_atlasGen->getImage(it->second,sizeIdx);
          }
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

osg::StateSet *TexturingQuery::generateStateAndAtlasRemap( osg::Vec4Array *v,int texSizeIdx){
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

    OSG_ALWAYS << "Texture Array Size: " <<uniqueIdCount <<endl;
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

   /* map<int,int>::const_iterator end = allIds.end();
    for (map<int,int>::const_iterator it = allIds.begin(); it != end; ++it)
    {
        std::cout << "Who(key = first): " << it->first;
        std::cout << " Score(value = second): " << it->second << '\n';
    }*/
    return stateset;
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

        osg::Geometry::PrimitiveSetList& primitiveSets = geom->getPrimitiveSetList();
        osg::Geometry::PrimitiveSetList::iterator itr;
         osg::Vec4Array *v=NULL;
         osg::StateSet *stateset=NULL;

        for(itr=primitiveSets.begin(); itr!=primitiveSets.end(); ++itr){
            switch((*itr)->getMode()){
            case(osg::PrimitiveSet::TRIANGLES):
                v=projectAllTriangles(*(*itr), *verts);
                if(v){
                    stateset=generateStateAndAtlasRemap(v,texSizeIdx);
                    setVertexAttrib(*geom,_projCoordAlias,v,false,osg::Geometry::BIND_PER_VERTEX);
                }
                geom->setStateSet(stateset);
                break;
            default:
                OSG_FATAL << "Freakout shouldn't be anything but triangles\n";
            }
        }
    }
}
