#include "TexturingQueryDidson.h"

using namespace std;

const static int IMAGE_WIDTH = 240;

static Eigen::MatrixXd _osgToEigen (const osg::Matrixf &mat)
{
    Eigen::MatrixXd eig (4, 4);
    for (int row=0; row<4; ++row)
        for (int col=0; col<4; ++col)
            eig (row, col) = mat (row, col);
    return eig;
}

TexturingQueryDidson::TexturingQueryDidson(TexturedSource *source,const CameraCalib &calib,TexPyrAtlas &atlasGen,bool useTextureArray,
                                           int _windowStart, int _windowLength)
    : TexturingQuery (source, calib, atlasGen, useTextureArray), windowStart (0), windowLength (0)
{
    this->setDidsonParams (_windowStart, _windowLength);
    this->tmp = new sonar::Didson (this->windowStart, this->windowLength, isam::Pose3d (), 0, 0, NULL, false);
    // pre-allocate conversion from bearing/range to image coordinates
    this->cartesian = this->tmp->getCartesian (IMAGE_WIDTH);
}

void TexturingQueryDidson::setDidsonParams (int _windowStart, int _windowLength)
{
    this->windowStart = _windowStart;
    this->windowLength = _windowLength;
}

osg::Vec2 TexturingQueryDidson::reprojectPt(const osg::Matrixf &mat,const osg::Vec3 &v, bool *is_visible)
{
    osg::Vec2 ret;

    // decompose matrix
    Eigen::MatrixXd _mat = _osgToEigen (osg::Matrixf::inverse (mat));
    isam::Pose3d pose (_mat);

    // these will be ignored
    double tilt = 0., pan = 0.;
    sonar::Didson didson (this->windowStart, this->windowLength, pose, pan, tilt, NULL, false);

    isam::Point3d p (v[0], v[1], v[2]);
    isam::Point2d br;
    bool is_visible_flag = didson.project (p, br);
    if (is_visible)
        *is_visible = is_visible_flag;

    if (is_visible_flag) {
        pair<int, int> rowcol = this->cartesian->bearingRange2Cartesian (br.x (), br.y ());
        ret[0] = rowcol.second; ret[1] = rowcol.first;
    } else {
        ret[0] = -1;
        ret[1] = -1;
    }
    return ret;
}

void TexturingQueryDidson::findCamProjAndDist(CamProjAndDist &cpad,osg::Vec3 v,SpatialIndex::id_type id)
{
    cpad.id=id;
    //  std::cout << "Id "<< id << _source->_cameras[id].m << " " << v<<std::endl;

    bool is_visible = false;
    osg::Vec2 texC=reprojectPt(_source->_cameras[id].m,v,&is_visible);
    
    if(!is_visible)
        cpad.dist=DBL_MAX;
    else
        cpad.dist=(texC-osg::Vec2(_calib.width/2,_calib.height/2)).length2();
    cpad.uv=convertToUV(texC);
    
}

double TexturingQueryDidson::getDistToCenter(osg::Vec3 v, TexturedSource::ProjectionCamera cam)
{
    // not sure what to do here since the "center" of a DIDSON cone isn't trivial to
    // compute.  I'll just use width/2, height/2, since it should be a good proxy for a
    // distortion-corrected image...
    bool is_visible = false;
    osg::Vec2 texC=reprojectPt(cam.m,v,&is_visible);
    if(!is_visible)
        return DBL_MAX;
    else
        return (texC-osg::Vec2(_calib.width/2.0,_calib.height/2.0)).length2();
}
