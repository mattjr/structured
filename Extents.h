#ifndef EXTENTS_H
#define EXTENTS_H
/* -*-c++-*- VirtualPlanetBuilder - Copyright (C) 1998-2007 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/


#ifdef WIN32

/////////////////////////////////////////////////////////////////////////////
// Disable unavoidable warning messages:
//
// C4503 - decorated name length exceeded, name was truncated
//
#pragma warning(disable : 4503)

#endif // WIN32


#include <osg/CoordinateSystemNode>

#include <osgTerrain/TerrainTile>

#include <osgDB/Archive>
#include <osgDB/DatabaseRevisions>

#include <set>

#include <vpb/SpatialProperties>
#include <vpb/Source>
#include <vpb/Destination>
#include <vpb/BuildOptions>
#include <vpb/BuildLog>
#include <vpb/ObjectPlacer>
#include <vpb/ThreadPool>
#include <vpb/DataSet>
#include <osgUtil/MeshOptimizers>
#include "TexturingQuery.h"
#include <OpenThreads/ScopedLock>

namespace vpb
{
    class MyDestinationTile : public DestinationTile{
    public:
        MyDestinationTile(std::string imageDir):_atlasGen(imageDir){
        _projCoordAlias = AttributeAlias(1, "osg_ProjCoord");
        _texCoordsAlias = AttributeAlias(15, "osg_texCoord");
        levelToTextureLevel[0]=2;
        levelToTextureLevel[1]=2;
        levelToTextureLevel[2]=1;
        levelToTextureLevel[3]=0;
        levelToTextureLevel[4]=0;
        levelToTextureLevel[5]=0;

    }
        typedef std::map<SpatialIndex::id_type,int> idmap_t;
        osg::StateSet *generateStateAndArray2DRemap( osg::Vec4Array *v,  osg::Vec2Array* texCoordsArray,int texSizeIdx);
        std::vector<osg::ref_ptr<osg::Image> >getRemappedImages(idmap_t allIds,int sizeIdx);
        static const int TEXUNIT_ARRAY=0;
        typedef std::pair<unsigned int, std::string> AttributeAlias;
        std::vector<osg::ref_ptr<osg::Vec4Array > > texCoordIDIndexPerModel;
        std::vector<osg::ref_ptr<osg::Vec2Array > > texCoordsPerModel;
        void setVertexAttrib(osg::Geometry& geom, const AttributeAlias& alias, osg::Array* array, bool normalize, osg::Geometry::AttributeBinding binding);
        void remapArrayForTexturing(osg::Vec4Array *v,osg::Vec2Array *texCoordsArray,idmap_t allIds);

        void generateStateAndSplitDrawables(std::vector<osg::Geometry*> &geoms,osg::Vec4Array *v, const osg::PrimitiveSet& prset,
                                                            osg::Vec2Array* texCoordsArray,
                                                            const osg::Vec3Array &verts,int texSizeIdx);
        static const int TEX_UNIT=0;

        AttributeAlias _projCoordAlias;
        AttributeAlias _texCoordsAlias;
         osg::Node * createScene(void);
         OpenThreads::Mutex _tileMutex;
         TexPyrAtlas _atlasGen;
         std::map<int,int> levelToTextureLevel;

    };
    class ClippedCopy{
    public:
        ClippedCopy(osg::BoundingBox bbox):_bbox(bbox){}
        osg::Geode* makeCopy(osg::Geode *geode);
        osg::BoundingBox _bbox;
        void AnalyzePrimSet(const osg::PrimitiveSet& prset, const osg::Vec3Array &verts);
        osg::ref_ptr<osg::DrawElementsUInt> _triangles;
        osg::ref_ptr<osg::Vec3Array>   _vertices;
    };

    class MyCompositeDestination : public CompositeDestination{
    public:
        MyCompositeDestination(osg::CoordinateSystemNode* cs, const GeospatialExtents& extents):
                CompositeDestination(cs, extents){}
        osg::Node* createPagedLODScene();
        osg::Node* createSubTileScene();

        typedef std::vector< osg::ref_ptr<MyDestinationTile> > MyTileList;
    };


class MyDataSet :  public DataSet
{
    public:
        MyDataSet(const Camera_Calib &calib,bool useTextureArray);
        void createNewDestinationGraph(
                                       const GeospatialExtents& extents,
                                       unsigned int maxImageSize,
                                       unsigned int maxTerrainSize,
                                       unsigned int maxNumLevels);

        void buildDestination() { _buildDestination(false); }

        void writeDestination() { _buildDestination(true); }

        void _buildDestination(bool writeToDisk);
        int _run();
        TexturingQuery *_tq;
        void processTile(MyDestinationTile *tile,Source *src);

    protected:
        virtual ~MyDataSet() {}

        void _readRow(Row& row);
         void _writeRow(Row& row);
         void init();
         const Camera_Calib &_calib;
         MyCompositeDestination* createDestinationTile(int level, int tileX, int tileY);
         bool _useTextureArray;
};

}


#endif // EXTENTS_H
