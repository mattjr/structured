#ifndef TEXTURINGQUERY_H
#define TEXTURINGQUERY_H
#include <string>
#include <spatialindex/SpatialIndex.h>
#include <osg/Geode>
class ObjVisitor : public SpatialIndex::IVisitor
{
private:
    uint32_t m_indexIO;
    uint32_t m_leafIO;
    std::vector<SpatialIndex::id_type> m_vector;
    uint32_t nResults;

public:

    ObjVisitor();
    ~ObjVisitor();

    uint32_t GetResultCount() const { return nResults; }
    std::vector<SpatialIndex::id_type>& GetResults()  { return m_vector; }

    void visitNode(const SpatialIndex::INode& n);
    void visitData(const SpatialIndex::IData& d);
    void visitData(std::vector<const SpatialIndex::IData*>& v);
};
class MyDataStream;

class TexturingQuery
{
public:
    TexturingQuery(std::string bbox_file);
    ~TexturingQuery();
    void projectModel(osg::Geode *);
    class ProjectionCamera{
    public:
        osg::Matrix m;
        std::string filename;
    };
    typedef std::map<SpatialIndex::id_type,ProjectionCamera>  CameraVector;

protected:

    std::string _bbox_file;
    SpatialIndex::ISpatialIndex* tree;
    SpatialIndex::StorageManager::IBuffer* file;
    SpatialIndex::IStorageManager* diskfile;
    std::string baseName;
    double utilization;
    int capacity;
    MyDataStream *stream;
    typedef std::multimap<int,ProjectionCamera> ProjectsToMap;
    void projectAllTriangles(const osg::PrimitiveSet& prset, const osg::Vec3Array &verts,ProjectsToMap &reproj);

    CameraVector _cameras;


};

#endif // TEXTURINGQUERY_H
