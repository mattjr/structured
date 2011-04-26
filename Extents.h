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
#include <vips/vips>
namespace vpb
{

class MyDataSet;
    class MyDestinationTile : public DestinationTile{
    public:
        MyDestinationTile(texcache_t imageDir){
            _atlasGen=new TexPyrAtlas(imageDir);
        _texCoordsAlias1 = AttributeAlias(8, "osg_texCoord1");
        _texCoordsAlias2 = AttributeAlias(9, "osg_texCoord2");
        _texCoordsAlias3 = AttributeAlias(10, "osg_texCoord3");
        _texCoordsAlias4 = AttributeAlias(11, "osg_texCoord4");
        _colorAlias = AttributeAlias(3, "osg_Color");

        levelToTextureLevel[0]=2;
        levelToTextureLevel[1]=2;
        levelToTextureLevel[2]=1;
        levelToTextureLevel[3]=0;
        levelToTextureLevel[4]=0;
        levelToTextureLevel[5]=0;
        _hintTextureNumber=0;
        _hintNumLevels=0;

    }

        osg::StateSet *generateStateAndArray2DRemap( osg::Vec4Array *v,  const TexBlendCoord &texCoordsArray,int texSizeIdx);
        std::vector<osg::ref_ptr<osg::Image> >getRemappedImages(idmap_t allIds,int sizeIdx);
        static const int TEXUNIT_ARRAY=0;
        int getTextureSizeForLevel(int level){
            if(levelToTextureLevel.count(level))
                return _atlasGen->getDownsampleSize(levelToTextureLevel[level]);
            return -1;
        }
        void addToHintTextureNumber(int numberTextures){
            _hintTextureNumber+=numberTextures;
        }
        void setHintNumLevels(int levels){
            _hintNumLevels=levels;
        }

        void addSourceWithHint(TexturedSource *source,const vpb::GeospatialExtents extents){
            _sources.push_back(source);
           /* const double minR[]={extents.xMin(),extents.yMin(),DBL_MIN};
            const double maxR[]={extents.xMax(),extents.yMax(),DBL_MAX};


            SpatialIndex::Region r = SpatialIndex::Region(minR,maxR,3);
            CountVisitor c;
            source->tree->intersectsWithQuery(r,c);
            //OSG_NOTICE << "Source has " << c.GetResultCount() <<std::endl;
            addToHintTextureNumber(c.GetResultCount());*/
        }

        typedef std::pair<unsigned int, std::string> AttributeAlias;
        std::map< osg::Node * , osg::ref_ptr<osg::Vec4Array > > texCoordIDIndexPerModel;
        std::map< osg::Node * ,TexBlendCoord> texCoordsPerModel;
        void setVertexAttrib(osg::Geometry& geom, const AttributeAlias& alias, osg::Array* array, bool normalize, osg::Geometry::AttributeBinding binding);
        void remapArrayForTexturing(osg::Vec4Array *v,const TexBlendCoord &texCoordsArray,idmap_t allIds);

        void generateStateAndSplitDrawables(std::vector<osg::Geometry*> &geoms,osg::Vec4Array *v, const osg::Vec4Array &colors,const osg::PrimitiveSet& prset,
                                                            const TexBlendCoord  &texCoordsArray,
                                                            const osg::Vec3Array &verts,int tex_size);
        static const int TEX_UNIT=0;
        void unrefData();

        AttributeAlias _texCoordsAlias1;
        AttributeAlias _texCoordsAlias2;
        AttributeAlias _texCoordsAlias3;
        AttributeAlias _texCoordsAlias4;
        AttributeAlias _colorAlias;

         osg::Node * createScene(void);
         OpenThreads::Mutex _texCoordMutex;
         OpenThreads::Mutex _modelMutex;
         osg::ref_ptr<TexPyrAtlas>  _atlasGen;
         std::map<int,int> levelToTextureLevel;
         int _hintTextureNumber;
         int _hintNumLevels;
         MyDataSet *_mydataSet;
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
        MyCompositeDestination(osg::CoordinateSystemNode* cs, const GeospatialExtents& extents,bool useReImage,bool useVBO,bool useDisplayLists,std::ofstream &file,OpenThreads::Mutex &fileMutex):
                CompositeDestination(cs, extents),_useReImage(useReImage),_useVBO(useVBO),_useDisplayLists(useDisplayLists),_file(file),_fileMutex(fileMutex){}
        osg::Node* createPagedLODScene();
        osg::Node* createSubTileScene();
        void unrefSubTileData();
        void unrefLocalData();
        bool _useReImage;
        bool _useVBO;
        bool _useDisplayLists;
        osg::Group *convertModel(osg::Group *group);
        std::ofstream &_file;
        OpenThreads::Mutex &_fileMutex;
        typedef std::vector< osg::ref_ptr<MyDestinationTile> > MyTileList;
        void writeCameraMatrix(osg::Node *scene);
int _numLevels;
    };


class MyDataSet :  public DataSet
{
    public:
        MyDataSet(const Camera_Calib &calib,std::string basePath,bool useTextureArray,bool useReImage,bool useVirtualTex);
        void createNewDestinationGraph(
                                       const GeospatialExtents& extents,
                                       unsigned int maxImageSize,
                                       unsigned int maxTerrainSize,
                                       unsigned int maxNumLevels);

        void buildDestination() { _buildDestination(false); }

        void writeDestination() { _buildDestination(true); }

        void _buildDestination(bool writeToDisk);
        void _equalizeRow(Row& row);
        int _run();
        void processTile(MyDestinationTile *tile,TexturedSource *src);
        texcache_t _cachedDirs;
        bool _useDisplayLists;
        bool _useAtlas;
        bool _useBlending;
        bool _useVBO;
        osg::Matrix viewProj;
        osg::Matrix rotMat;

        vips::VImage *in;
        osg::Matrix getImageSection(vips::VImage &in,const osg::Vec2 minT, const osg::Vec2 maxT,int origX,int origY,osg::Vec4 &texsize,const osg::Matrix &toTex,osg::ref_ptr<osg::Image> &image,osg::Vec4 &ratio,int level);
        void loadShaderSourcePrelude(osg::Shader* obj, const std::string& fileName );
        osg::Vec4 _zrange;
        bool _useStub;
    protected:
        virtual ~MyDataSet() {if(in) delete in;}
        void _readRow(Row& row);
         void _writeRow(Row& row);
         void init();
         const Camera_Calib &_calib;

         MyCompositeDestination* createDestinationTile(int level, int tileX, int tileY);
         std::ofstream _file;
         OpenThreads::Mutex _fileMutex;
         OpenThreads::Mutex _imageMutex;
public:
         std::string _basePath;
         bool _useTextureArray;
         bool _useReImage;
         bool _useVirtualTex;



};

}
bool readMatrixToScreen(std::string fname,osg::Matrix &viewProj);
bool readMatrix(std::string fname,osg::Matrix &viewProj);


#endif // EXTENTS_H
