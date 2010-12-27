#ifndef TEXTURINGQUERY_H
#define TEXTURINGQUERY_H
#include <string>
#include <spatialindex/SpatialIndex.h>
#include <osg/Geode>
#include <osg/BoundingBox>
#include <osg/Geometry>
#include "TexPyrAtlas.h"
#include <libsnapper/auv_camera_calib.hpp>
using libsnapper::Camera_Calib;
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
class MyDataStream;
class CamProjAndDist{
public:
    SpatialIndex::id_type id;
    double dist;
    osg::Vec2 uv;
};

typedef std::vector<CamProjAndDist> CamDists;

class TexturingQuery
{
public:
    TexturingQuery(std::string bbox_file,const Camera_Calib &calib,bool useTextureArray);
    ~TexturingQuery();
    void projectModel(osg::Geode *,int texSizeIdx);
    class ProjectionCamera{
    public:
        osg::Matrixf m;
        std::string filename;
        long id;
        osg::BoundingBox bb;
    };
    typedef std::map<SpatialIndex::id_type,ProjectionCamera>  CameraVector;
    struct sort_pred {
        bool operator()(const CamProjAndDist &left, const CamProjAndDist &right) {
            return left.dist < right.dist;
        }
    };

protected:
    typedef std::multimap<int,ProjectionCamera> ProjectsToMap;
    double getDistToCenter(osg::Vec3 v, ProjectionCamera cam);
    void findCamProjAndDist(CamProjAndDist &cpad,osg::Vec3 v,SpatialIndex::id_type id);
    osg::Vec2 convertToUV(const osg::Vec2 &pix);
    const CamDists getClosest(std::vector<int> tri_v,const osg::Vec3Array &verts);
    std::string _bbox_file;
    const Camera_Calib _calib;
    //void remapSharedVert(osg::PrimitiveSet& prset, osg::Vec3Array &verts,std::map<int,int> remap);
    SpatialIndex::ISpatialIndex* tree;
    SpatialIndex::IStorageManager* memstore;
    SpatialIndex::IStorageManager* manager;
    std::string baseName;
    double utilization;
    int capacity;
    MyDataStream *stream;
    bool projectAllTriangles(osg::Vec4Array* camIdxArr,osg::Vec2Array* texCoordsArray,
                                             const osg::PrimitiveSet& prset, const osg::Vec3Array &verts);
    osg::StateSet *generateStateAndArray2DRemap( osg::Vec4Array *v,  osg::Vec2Array* texCoordsArray, int texSizeIdx);
    static const int TEXUNIT_ARRAY=0;
    static const int TEX_UNIT=0;
    ProjectsToMap reproj;
    osg::Vec2 _origImageSize;
    bool _useTextureArray;
    bool _useAtlas;

    typedef std::pair<unsigned int, std::string> AttributeAlias;
    void setVertexAttrib(osg::Geometry& geom, const AttributeAlias& alias, osg::Array* array, bool normalize, osg::Geometry::AttributeBinding binding);
    std::vector<osg::ref_ptr<osg::Image> >loadTex(std::map<SpatialIndex::id_type,int> allIds,int sizeIdx);
    void generateStateAndSplitDrawables(std::vector<osg::Geometry*> &geoms,osg::Vec4Array *v, const osg::PrimitiveSet& prset,
                                        osg::Vec2Array* texCoordsArray,
                                        const osg::Vec3Array &verts,int texSizeIdx);
    osg::Vec2 reprojectPt(const osg::Matrixf &mat,const osg::Vec3 &v);
    AttributeAlias _vertexAlias;
    AttributeAlias _projCoordAlias;
    AttributeAlias _texCoordsAlias;
    TexPyrAtlas *_atlasGen;
    CameraVector _cameras;


};
// example of a Visitor pattern.
// findes the index and leaf IO for answering the query and prints
// the resulting data IDs to stdout.


class MyDataStream : public SpatialIndex::IDataStream
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
    TexturingQuery::CameraVector &m_camVec;


};
#endif // TEXTURINGQUERY_H
