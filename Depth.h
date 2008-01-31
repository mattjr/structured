#ifndef DEPTH_H
#define DEPTH_H
#include "OSGExport.h"
#include "auv_ransac_plane.hpp"
#include "TriMesh.h"
#include <osg/TextureRectangle>
using namespace std;
using namespace libpolyp;
using namespace auv_data_tools;
osg::TextureRectangle*  getPlaneTex( vector<Plane3D> planes,int size);
osg::Texture2D* newColorTexture2D(unsigned width, unsigned height, unsigned accuracy);
osg::TextureRectangle* newColorTextureRectangle(unsigned width, unsigned height, unsigned accuracy);
osg::Vec3Array* displayPlane(Plane3D m_BiggerPlane3D,Point3D center);
class DepthStats{

public:
  DepthStats(TriMesh *mesh);
  vector<int> * getPlaneFits(vector<Plane3D> &planes, vector<TriMesh::BBox> &bounds,
		    int widthSplits,int heightSplits);
  TriMesh *_mesh;
};
#endif
