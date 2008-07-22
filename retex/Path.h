#ifndef PATH_H
#define PATH_H
#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osg/AnimationPath>
#include <osg/Geometry>

struct script_element{
  double timestamp;
  double height;
  double speed;
};

class PathLoader{
public:
  PathLoader(){};
  ~PathLoader();
  std::vector<osg::Matrixd> createPath(const char *path);

};
#endif
