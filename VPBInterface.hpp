//
// structured - Tools for the Generation and Visualization of Large-scale
// Three-dimensional Reconstructions from Image Data. This software includes
// source code from other projects, which is subject to different licensing,
// see COPYING for details. If this project is used for research see COPYING
// for making the appropriate citations.
// Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
//
// This file is part of structured.
//
// structured is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// structured is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with structured.  If not, see <http://www.gnu.org/licenses/>.
//

#ifndef VPBINTERFACE_HPP
#define VPBINTERFACE_HPP
#include "stereo_cells.hpp"
#include "Extents.h"
#include <osg/BoundingBox>
#include <vpb/Commandline>
#include <vector>
#include <string>

void doQuadTreeVPB(std::string basePath,std::vector<std::vector<std::string> > datalist_lod,Bounds bounds,CameraCalib &calib,texcache_t cachedDirs,int POTAtlasSize,bool useTextureArray,bool useReimage,bool useVirtualTex,const osg::BoundingBox &bbox,std::string src_proj4_coord_system,std::string dst_proj4_coord_system,double sparseRatio,bool no_hw_context=false,bool no_atlas=false);
bool toVert(osg::Node *node,const TexBlendCoord &texcoord,osg::Vec4Array *ids,TexBlendCoord &newTexCoord,osg::Vec4Array *newIds);
#endif // VPBINTERFACE_HPP
