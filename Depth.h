/* * structured - Tools for the Generation and Visualization of Large-scale
 * Three-dimensional Reconstructions from Image Data. This software includes
 * source code from other projects, which is subject to different licensing,
 * see COPYING for details. If this project is used for research see COPYING
 * for making the appropriate citations.
 * Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
 *
 * This file is part of structured.
 *
 * structured is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * structured is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with structured.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DEPTH_H
#define DEPTH_H
#include "OSGExport.h"
#include "auv_ransac_plane.hpp"
#include "auv_best_fit.hpp"
#include "TriMesh.h"
#include <osg/TextureRectangle>
using namespace std;
using namespace libpolyp;
using namespace auv_data_tools;
osg::TextureRectangle*  getPlaneTex( vector<Plane3D> planes,int size);
osg::Texture2D* newColorTexture2D(unsigned width, unsigned height, unsigned accuracy);
osg::TextureRectangle* newColorTextureRectangle(unsigned width, unsigned height, unsigned accuracy);
osg::TextureRectangle* ass(unsigned width, unsigned height, unsigned accuracy);
osg::Vec3Array* displayPlane(Plane3D m_BiggerPlane3D,Point3D center);
class DepthStats{

public:
  DepthStats(TriMesh *mesh);
  vector<int> * getPlaneFits(vector<Plane3D> &planes, vector<TriMesh::BBox> &bounds,osg::Matrix *rot,
			    float widthTargetSize,float heightTargetSize,unsigned int minPts);
  TriMesh *_mesh;
};
#endif
