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


namespace vpb
{
    class MyDestinationTile : public DestinationTile{
    public:
         osg::Node * createScene(void);
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
        MyDataSet(const Camera_Calib &calib);
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
        void processTile(DestinationTile *tile,Source *src);

    protected:
        virtual ~MyDataSet() {}

        void _readRow(Row& row);
         void _writeRow(Row& row);
         void init();
         const Camera_Calib &_calib;
         MyCompositeDestination* createDestinationTile(int level, int tileX, int tileY);

};

}


#endif // EXTENTS_H
