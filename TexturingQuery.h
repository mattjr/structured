#ifndef TEXTURINGQUERY_H
#define TEXTURINGQUERY_H
#include <string>
#include <spatialindex/SpatialIndex.h>
#include <osg/Geode>
#include <osg/BoundingBox>
#include <osg/Geometry>
#include "TexPyrAtlas.h"
#include "TexturedSource.h"
#include <libsnapper/auv_camera_calib.hpp>
using libsnapper::Camera_Calib;
void addDups(osg::Geode *geode);
namespace vpb{
    class MyDestinationTile;
};
class CamProjAndDist{
public:
    SpatialIndex::id_type id;
    double dist;
    osg::Vec2 uv;
};

typedef std::vector<CamProjAndDist> CamDists;
int checkCached(std::string mf,std::string cachedloc,std::string &sha2hash);
bool loadCached(const std::string &file,osg::Vec4Array *ids,osg::Vec2Array *texCoords);
bool writeCached(const std::string &outfilename,const std::string  sha2hash,osg::Vec4Array *ids,osg::Vec2Array *texCoords);
std::string getHash(std::string mf);

class TexturingQuery
{
    friend class TexturedSource;
public:
    TexturingQuery(TexturedSource *source,const Camera_Calib &calib,TexPyrAtlas &atlasGen,bool useTextureArray);
    ~TexturingQuery();
    bool projectModel(osg::Geode *);

    struct sort_pred {
        bool operator()(const CamProjAndDist &left, const CamProjAndDist &right) {
            return left.dist < right.dist;
        }
    };
    vpb::MyDestinationTile *_tile;
    bool checkAndLoadCache(osg::Vec4Array *ids,osg::Vec2Array *texCoords);
    void addImagesToAtlasGen(std::map<SpatialIndex::id_type,int> allIds);

protected:
    typedef std::multimap<int,TexturedSource::ProjectionCamera> ProjectsToMap;
    double getDistToCenter(osg::Vec3 v, TexturedSource::ProjectionCamera cam);
    void findCamProjAndDist(CamProjAndDist &cpad,osg::Vec3 v,SpatialIndex::id_type id);
    osg::Vec2 convertToUV(const osg::Vec2 &pix);
    const CamDists getClosest(std::vector<int> tri_v,const osg::Vec3Array &verts);
    std::string _bbox_file;
    const Camera_Calib _calib;
    //void remapSharedVert(osg::PrimitiveSet& prset, osg::Vec3Array &verts,std::map<int,int> remap);

    std::string baseName;

    bool projectAllTriangles(osg::Vec4Array* camIdxArr,osg::Vec2Array* texCoordsArray,
                                             const osg::PrimitiveSet& prset, const osg::Vec3Array &verts);
    osg::StateSet *generateStateAndArray2DRemap( osg::Vec4Array *v,  osg::Vec2Array* texCoordsArray, int texSizeIdx);
    ProjectsToMap reproj;
    osg::Vec2 _origImageSize;
    bool _useTextureArray;
    TexPyrAtlas &_atlasGen;

    bool _useAtlas;
    typedef std::pair<unsigned int, std::string> AttributeAlias;
    std::vector<osg::ref_ptr<osg::Image> >getRemappedImages(std::map<SpatialIndex::id_type,int> allIds,int sizeIdx);
    void generateStateAndSplitDrawables(std::vector<osg::Geometry*> &geoms,osg::Vec4Array *v, const osg::PrimitiveSet& prset,
                                        osg::Vec2Array* texCoordsArray,
                                        const osg::Vec3Array &verts,int texSizeIdx);
    osg::Vec2 reprojectPt(const osg::Matrixf &mat,const osg::Vec3 &v);
    AttributeAlias _vertexAlias;

    TexturedSource *_source;

};

typedef std::map<SpatialIndex::id_type,int> idmap_t;
typedef std::map<int,SpatialIndex::id_type> idbackmap_t;

void calcAllIdsBack(osg::Vec4Array *v,std::map<SpatialIndex::id_type,int> &allIds,std::map<int,SpatialIndex::id_type>  &backMap);

std::map<SpatialIndex::id_type,int> calcAllIds(osg::Vec4Array *v);
bool loadShaderSource(osg::Shader* obj, const std::string& fileName );

#endif // TEXTURINGQUERY_H
