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
VipsAtlasBuilder* createVTAtlas(const osg::Matrix &viewProj,int totalX,int totalY,
                                const std::vector<mosaic_cell> &mosaic_cells,
                                bool writeAtlas,double scaleFactor=1.0,std::string basedir="mesh",std::string imgOutput="");
void generateImageFromExtents(const std::vector<mosaic_cell> &mosaic_cells,
                                             const osg::Vec2 minT,
                                     const osg::Vec2 maxT,int origX,int origY,
                                     osg::Vec4 &texsize,
                                     const osg::Matrix &toTex,
                                     osg::ref_ptr<osg::Image> &image,
                                     osg::Vec4 &ratio,
                                     int level);
bool shouldUseSparseMode(const std::vector<mosaic_cell> &mosaic_cells,double sparseRatio, const osg::Vec2 minT,
                         const osg::Vec2 maxT, osg::ref_ptr<TightFitAtlasBuilder> &atlas,int leveloffset,        const std::set<int> &seenids);
SpatialIndex::ISpatialIndex* createTree(const std::vector<mosaic_cell> &mosaic_cells);

#endif // BUILDATLAS_H
