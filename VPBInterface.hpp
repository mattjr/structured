#ifndef VPBINTERFACE_HPP
#define VPBINTERFACE_HPP
#include "stereo_cells.hpp"
#include "Extents.h"
#include <osg/BoundingBox>
#include <vpb/Commandline>
#include <vector>
#include <string>

void doQuadTreeVPB(std::string basePath,std::vector<std::vector<std::string> > datalist_lod,Bounds bounds,CameraCalib &calib,texcache_t cachedDirs,bool useTextureArray,bool useReimage,bool useVirtualTex,const osg::BoundingBox &bbox,std::string dst_wkt_coord_system,std::string src_proj4_coord_system,bool no_hw_context=false);
bool toVert(osg::Node *node,const TexBlendCoord &texcoord,osg::Vec4Array *ids,TexBlendCoord &newTexCoord,osg::Vec4Array *newIds);
#endif // VPBINTERFACE_HPP
