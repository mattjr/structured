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

#ifndef BUILDATLAS_H
#define BUILDATLAS_H
#include <osg/BoundingBox>
#include <vips/vips>
#include <vector>
#include <OpenThreads/Mutex>
#include <osg/Matrix>
#include <osg/Vec4>
#include <osg/Vec2>
#include <set>
#include "SpatialIndex.h"
#include "TightFit.h"
class mosaic_cell{
public:
   osg::BoundingBoxd bbox;
   std::string name;
   vips::VImage *img;
   std::vector<vips::VImage *> img_ds;
   OpenThreads::Mutex *mutex;
   int levels;
   std::vector<std::string> name_ds;
};
bool readMatrixToScreen(std::string fname,osg::Matrixd &viewProj);
bool readMatrix(std::string fname,osg::Matrix &viewProj);
void loadMosaicCells(std::string fname,int &totalX,int &totalY,std::vector<mosaic_cell> &mosaic_cells);
class TightFitAtlasBuilder;
void  generateAtlasAndTexCoordMappingFromExtents(const std::vector<mosaic_cell> &mosaic_cells,
                                             const osg::Vec2 minT,
                                     const osg::Vec2 maxT,int origX,int origY,
                                     const osg::Matrix &toTex,
                                                 TightFitAtlasBuilder* atlas,
                                     osg::Vec4 &ratio,
                                     int level);
VipsAtlasBuilder* createVTAtlas(const osg::Matrix &viewProj,int totalX,int totalY,int POTAtlasSize,
                                const std::vector<mosaic_cell> &mosaic_cells,
                                bool writeAtlas,double scaleFactor=1.0,std::string basedir="mesh",std::string imgOutput="",
                                bool flat=false,double maxRes=-1,
                                int levelRun=-1,int rowRun=-1);
void generateImageFromExtents(const std::vector<mosaic_cell> &mosaic_cells,
                                             const osg::Vec2 minT,
                                     const osg::Vec2 maxT,int origX,int origY,
                                     osg::Vec4 &texsize,
                                     const osg::Matrix &toTex,
                                     osg::ref_ptr<osg::Image> &image,
                                     osg::Vec4 &ratio,
                                     int level);
void generateAtlasAndTexCoordMappingFromExtentsVips(const std::vector<mosaic_cell> &mosaic_cells,

                                                       VipsAtlasBuilder* atlas,bool flat=false,int level=-1
                                                       );
bool shouldUseSparseMode(const std::vector<mosaic_cell> &mosaic_cells,double sparseRatio, const osg::Vec2 minT,
                         const osg::Vec2 maxT, osg::ref_ptr<TightFitAtlasBuilder> &atlas,int leveloffset,        const std::set<int> &seenids);
SpatialIndex::ISpatialIndex* createTree(const std::vector<mosaic_cell> &mosaic_cells);

#endif // BUILDATLAS_H
