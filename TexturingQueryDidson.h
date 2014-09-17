#include "TexturingQuery.h"

class TexturingQueryDidson : public TexturingQuery {

public:
    TexturingQueryDidson(TexturedSource *source,const CameraCalib &calib,TexPyrAtlas &atlasGen,bool useTextureArray, 
                         int _windowStart, int _windowLength);

    
protected:
    virtual double getDistToCenter(osg::Vec3 v, TexturedSource::ProjectionCamera cam);
    virtual void findCamProjAndDist(CamProjAndDist &cpad,osg::Vec3 v,SpatialIndex::id_type id);
    virtual osg::Vec2 reprojectPt(const osg::Matrixf &mat,const osg::Vec3 &v, bool *is_visible=NULL);

    void setDidsonParams (int windowStart, int windowLength);

    int windowStart;
    int windowLength;
};
