#ifndef VPBINTERFACE_HPP
#define VPBINTERFACE_HPP
#include "stereo_cells.hpp"
#include "Extents.h"
#include <osg/BoundingBox>
#include <vpb/Commandline>
#include <vector>
#include <string>

void doQuadTreeVPB(std::string basePath,std::string cacheddir,std::vector<std::vector<std::string> > datalist_lod,Bounds bounds,Camera_Calib &calib,texcache_t cachedDirs,bool useTextureArray,bool useReimage,bool useVirtualTex,const osg::BoundingBox &bbox);
bool toVert(osg::Node *node,const TexBlendCoord &texcoord,osg::Vec4Array *ids,TexBlendCoord &newTexCoord,osg::Vec4Array *newIds);
#endif // VPBINTERFACE_HPP
