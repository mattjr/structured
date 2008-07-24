#ifndef PATH_H
#define PATH_H
#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osg/AnimationPath>
#include <osg/Geometry>
#include <vector>
typedef struct _bbox{
  double x1,x2,y1,y2,z1,z2;
}bbox;

struct script_element{
  double timestamp;
  double height;
  double speed;
};

class PathLoader{
public:
  PathLoader(){};
  ~PathLoader();
  
  void createPath(const char *path, std::vector<osg::Matrixd> &camMat,std::vector<bbox> &bboxes  );
};
#endif
