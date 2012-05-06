#ifndef GENPARAM_H
#define GENPARAM_H
#include <osg/Vec3>
#include <string>
osg::Vec3Array* CGALreparam(osg::ref_ptr<osg::Vec3Array> verts,osg::ref_ptr<osg::DrawElementsUInt> triangles);

#endif // GENPARAM_H
