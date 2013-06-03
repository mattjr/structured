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

#ifndef IMAGENODE_H
#define IMAGENODE_H
#include "PosterPrinter.h"
void readCameraMatrix( std::ifstream &file,osg::Matrix &view,osg::Matrix &proj);
int render(osg::Node *scene,osg::ref_ptr<osg::Image> &image,osg::GraphicsContext &gc,osg::Matrix &toScreen,const osg::Vec4 &sizes);
osg::Geode *convertModel(osg::Group *group);
int renderAll(osg::Node *scene, std::vector<osg::Matrixd> &views,std::vector<osg::Matrixd> &projs,std::vector<std::string> &fnames,const osg::Vec4 &sizes);
osg::Vec2 calcCoordReproj(const osg::Vec3 &vert,const osg::Matrix &viewProj,const osg::Matrix &screen,const osg::Vec2 &size,const osg::Vec4 &ratio);
#endif // IMAGENODE_H
