#ifndef TEXTUREDSOURCE_H
#define TEXTUREDSOURCE_H
#include <vpb/System>

#include <spatialindex/SpatialIndex.h>
#include <osg/Geode>
#include <osg/BoundingBox>
#include <osg/Geometry>


class MyDataStream;

class TexturedSource : public vpb::Source
{
    friend class TexturingQuery;
public:
    TexturedSource(Type type, const std::string& filename,const std::string &bbox_file);
    ~TexturedSource();

    class ProjectionCamera{
    public:
        osg::Matrixf m;
        std::string filename;
        long id;
        osg::BoundingBox bb;
    };
    typedef std::map<SpatialIndex::id_type,ProjectionCamera>  CameraVector;
    SpatialIndex::ISpatialIndex* tree;
    void intersectsWithQuery(const SpatialIndex::IShape& query, SpatialIndex::IVisitor& v);
  osg::BoundingBox _bb;
  osg::KdTree *_kdTree;
  std::string tex_cache_dir;
  TexturedSource::CameraVector _cameras;
  osg::Vec4Array *ids;
protected:
    SpatialIndex::IStorageManager* memstore;
    SpatialIndex::IStorageManager* manager;
    double utilization;
    int capacity;
    MyDataStream *stream;
    std::string _bbox_file;
    OpenThreads::Mutex _treeMutex;



};


class CountVisitor : public SpatialIndex::IVisitor
{
private:
    uint32_t m_indexIO;
    uint32_t m_leafIO;
    uint32_t nResults;

public:
    CountVisitor(): nResults(0) {}

    ~CountVisitor() {}

    void visitNode(const SpatialIndex::INode& n)
    {
        if (n.isLeaf()) m_leafIO++;
        else m_indexIO++;
    }

    void visitData(const SpatialIndex::IData& d)
    {
        nResults += 1;

    }

    void visitData(std::vector<const SpatialIndex::IData*>& v)
    {
        // std::cout << v[0]->getIdentifier() << " " << v[1]->getIdentifier() << std::endl;
    }




    uint32_t GetResultCount() const { return nResults; }


};
// example of a Visitor pattern.
// findes the index and leaf IO for answering the query and prints
// the resulting data IDs to stdout.

class ObjVisitor : public SpatialIndex::IVisitor
{
private:
    uint32_t m_indexIO;
    uint32_t m_leafIO;
    std::vector<SpatialIndex::id_type> m_vector;
    uint32_t nResults;

public:
    ObjVisitor(): nResults(0) {}

    ~ObjVisitor() {}

    void visitNode(const SpatialIndex::INode& n)
    {
        if (n.isLeaf()) m_leafIO++;
        else m_indexIO++;
    }

    void visitData(const SpatialIndex::IData& d)
    {
        nResults += 1;

        m_vector.push_back(d.getIdentifier());
    }

    void visitData(std::vector<const SpatialIndex::IData*>& v)
    {
        // std::cout << v[0]->getIdentifier() << " " << v[1]->getIdentifier() << std::endl;
    }




    uint32_t GetResultCount() const { return nResults; }
    std::vector<SpatialIndex::id_type>& GetResults()  { return m_vector; }


};
class MyDataStream : public SpatialIndex::IDataStream
{
public:
    MyDataStream(std::string inputFile,TexturedSource::CameraVector &cam) : m_pNext(0),m_camVec(cam)
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

    virtual SpatialIndex::IData* getNext()
    {
        if (m_pNext == 0) return 0;

        SpatialIndex::RTree::Data* ret = m_pNext;
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
        TexturedSource::ProjectionCamera cam;
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

            SpatialIndex::Region r(low, high, 3);
            m_pNext = new SpatialIndex::RTree::Data(sizeof(double), reinterpret_cast<byte*>(low), r, cam.id);
            // Associate a bogus data array with every entry for testing purposes.
            // Once the data array is given to RTRee:Data a local copy will be created.
            // Hence, the input data array can be deleted after this operation if not
            // needed anymore.
        }
    }

    std::ifstream m_fin;
    SpatialIndex::RTree::Data* m_pNext;
    TexturedSource::CameraVector &m_camVec;


};

#endif // TEXTUREDSOURCE_H
