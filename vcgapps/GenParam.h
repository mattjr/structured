#ifndef GENPARAM_H
#define GENPARAM_H
#include <osg/Vec3>
#include <string>
//osg::Vec3Array* CGALreparam(osg::ref_ptr<osg::Vec3Array> verts,osg::ref_ptr<osg::DrawElementsUInt> triangles);
osg::Vec3Array* OGFreparam(osg::ref_ptr<osg::Vec3Array> verts,osg::ref_ptr<osg::DrawElementsUInt> triangles);
int calcOptimalImageSize(const osg::Vec2 imageSize,osg::Vec3Array *verts,osg::DrawElementsUInt* triangles,std::vector<osg::Vec3Array *>   &texCoord);

#endif // GENPARAM_H
