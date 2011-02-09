#ifndef IMAGENODE_H
#define IMAGENODE_H
#include "PosterPrinter.h"
void readCameraMatrix( std::ifstream &file,osg::Matrix &view,osg::Matrix &proj);
int render(osg::Node *scene,osg::ref_ptr<osg::Image> &image,osg::GraphicsContext &gc,osg::Matrix &toScreen,const osg::Vec4 &sizes);
osg::Geode *convertModel(osg::Group *group);
int renderAll(osg::Node *scene, std::vector<osg::Matrixd> &views,std::vector<osg::Matrixd> &projs,std::vector<std::string> &fnames,const osg::Vec4 &sizes);
osg::Vec2 calcCoordReproj(const osg::Vec3 &vert,const osg::Matrix &toScreen,const osg::Vec2 &size);
#endif // IMAGENODE_H
