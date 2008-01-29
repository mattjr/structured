#include "OSGExport.h"
#include "auv_ransac_plane.hpp"
#include "TriMesh.h"
using namespace std;

using namespace auv_data_tools;
class DepthStats{

public:
  DepthStats(TriMesh *mesh);
  void getPlaneFits(vector<Plane3D> &planes, vector<GtsBBox> &bounds,
		    int widthSplits,int heightSplits);
  TriMesh *_mesh;
};
