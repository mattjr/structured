#include "TexturingQuery.h"
#include <osg/Matrix>
#include <osg/Geometry>
#include <osg/PrimitiveSet>
#include <map>
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
        id_type id;
        double low[3], high[3];
        TexturingQuery::ProjectionCamera cam;

        m_fin >> id >> cam.filename >> low[0] >> low[1] >> low[2] >> high[0] >> high[1] >> high[2]
                >> cam.m(0,0) >>cam.m(0,1)>>cam.m(0,2) >>cam.m(0,3)
                >> cam.m(1,0) >>cam.m(1,1)>>cam.m(1,2) >>cam.m(1,3)
                >> cam.m(2,0) >>cam.m(2,1)>>cam.m(2,2) >>cam.m(2,3)
                >> cam.m(3,0) >>cam.m(3,1)>>cam.m(3,2) >>cam.m(3,3);
        if (m_fin.good())
        {
            m_camVec[id]=cam;
            /*if (op != INSERT)
                                throw Tools::IllegalArgumentException(
                                        "The data input should contain insertions only."
                                );*/

            Region r(low, high, 3);
            m_pNext = new RTree::Data(sizeof(double), reinterpret_cast<byte*>(low), r, id);
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

TexturingQuery::TexturingQuery(std::string bbox_file) : _bbox_file(bbox_file)
{
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
void TexturingQuery::projectAllTriangles(const osg::PrimitiveSet& prset, const osg::Vec3Array &verts,ProjectsToMap &reproj){
    int numIdx=prset.getNumIndices();
    if(numIdx < 3)
        return;
    for(int i=0; i<numIdx-2; i+=3){
        for(int k=0; k <3; k++){
            int ptIdx=i+k;
            double pt[3];
            pt[0]=verts[prset.index(ptIdx)].x();
            pt[1]=verts[prset.index(ptIdx)].y();
            pt[2]=verts[prset.index(ptIdx)].z();
            Point p = Point(pt, 3);
            ObjVisitor vis;
           // cout << p <<endl;
            tree->intersectsWithQuery(p,vis);
            if(vis.GetResultCount()){
                for(unsigned int r=0; r < vis.GetResults().size(); r++){
                    if(_cameras.count(vis.GetResults()[r]))
                        reproj.insert(std::pair<int,ProjectionCamera>(ptIdx,_cameras[vis.GetResults()[r]]));
                }
            }
        }
    }
 /*   cout << "Elements in m: " << endl;

      for (multimap<int, ProjectionCamera>::iterator it = reproj.begin();
               it != reproj.end();
           ++it)

       {
              cout << "  [" << (*it).first << ": " << reproj.count((*it).first) << "]" << endl;
       }
*/
}


void TexturingQuery::projectModel(osg::Geode *geode){
    if(!geode){
        fprintf(stderr,"Not valid geode\n");
        return;
    }
    for (unsigned int i=0; i<geode->getNumDrawables(); i++){
        ProjectsToMap reproj;

        osg::Drawable *drawable = geode->getDrawable(i);
        osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
        osg::Geometry::PrimitiveSetList& primitiveSets = geom->getPrimitiveSetList();
        osg::Geometry::PrimitiveSetList::iterator itr;
        for(itr=primitiveSets.begin(); itr!=primitiveSets.end(); ++itr){
            switch((*itr)->getMode()){
            case(osg::PrimitiveSet::TRIANGLES):
                projectAllTriangles(*(*itr), *static_cast<const osg::Vec3Array*>(geom->getVertexArray()),reproj);
            }
        }
    }
}
