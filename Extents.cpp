
/* -*-c++-*- VirtualPlanetBuilder - Copyright (C) 1998-2009 Robert Osfield
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


#include <osg/Texture2D>
#include <osg/ComputeBoundsVisitor>
#include <osg/io_utils>

#include <osg/GLU>
#include <osgUtil/SmoothingVisitor>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>

#include <osgFX/MultiTextureControl>

#include <osgViewer/GraphicsWindow>
#include <osgViewer/Viewer>
#include <osgViewer/Version>
#include "Clipper.h"
#include "Extents.h"
#include <vpb/DatabaseBuilder>
#include <vpb/TaskManager>
#include <vpb/System>
#include <vpb/FileUtils>
#include <vpb/FilePathManager>

#include <vpb/ShapeFilePlacer>

// GDAL includes
#include <gdal_priv.h>
#include <ogr_spatialref.h>

// standard library includes
#include <sstream>
#include <iostream>
#include <algorithm>


using namespace vpb;
using vpb::log;

MyDataSet::MyDataSet()
{
    init();
}

void MyDataSet::init()
{
    // make sure GDAL etc. are initialized
    System::instance();
    _C1 = 0;
    _R1 = 0;

    _numTextureLevels = 1;

    _newDestinationGraph = false;

}
class CollectClusterCullingCallbacks : public osg::NodeVisitor
{
public:


    struct Triple
    {
        Triple():
                _object(0),
                _callback(0) {}

        Triple(osg::NodePath nodePath, osg::Object* object, osg::ClusterCullingCallback* callback):
                _nodePath(nodePath),
                _object(object),
                _callback(callback) {}

        Triple(const Triple& t):
                _nodePath(t._nodePath),
                _object(t._object),
                _callback(t._callback) {}

        Triple& operator = (const Triple& t)
                           {
            _nodePath = t._nodePath;
            _object = t._object;
            _callback = t._callback;
            return *this;
        }

        osg::NodePath                   _nodePath;
        osg::Object*                    _object;
        osg::ClusterCullingCallback*    _callback;
    };

    typedef std::vector<Triple> ClusterCullingCallbackList;

    CollectClusterCullingCallbacks():
            osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}

    virtual void apply(osg::Group& group)
    {
        osgTerrain::Terrain* terrain = dynamic_cast<osgTerrain::Terrain*>(&group);
        if (terrain)
        {
            osg::ClusterCullingCallback* callback = dynamic_cast<osg::ClusterCullingCallback*>(terrain->getCullCallback());
            if (callback)
            {
                _callbackList.push_back(Triple(getNodePath(),terrain,callback));
            }
            return;
        }

        osgTerrain::TerrainTile* terrainTile = dynamic_cast<osgTerrain::TerrainTile*>(&group);
        if (terrainTile)
        {
            osg::ClusterCullingCallback* callback = dynamic_cast<osg::ClusterCullingCallback*>(terrainTile->getCullCallback());
            if (callback)
            {
                _callbackList.push_back(Triple(getNodePath(),terrain,callback));
            }
            return;
        }
        else
        {
            osg::NodeVisitor::apply(group);
        }
    }

    virtual void apply(osg::Geode& geode)
    {
        for(unsigned int i=0; i<geode.getNumDrawables();++i)
        {
            osg::ClusterCullingCallback* callback = dynamic_cast<osg::ClusterCullingCallback*>(geode.getDrawable(i)->getCullCallback());
            if (callback)
            {
                _callbackList.push_back(Triple(getNodePath(),geode.getDrawable(i),callback));
            }
        }
    }

    ClusterCullingCallbackList _callbackList;

};

osg::Node* MyCompositeDestination::createPagedLODScene()
{
    if (_children.empty() && _tiles.empty()) return 0;

    if (_children.empty() && _tiles.size()==1) { DestinationTile *t= _tiles.front();

        MyDestinationTile *myt=dynamic_cast<MyDestinationTile*>(t);

        return myt?myt->createScene(): 0;
    }
    if (_tiles.empty() && _children.size()==1)  {
        CompositeDestination *c=_children.front();
        MyCompositeDestination *child=dynamic_cast<MyCompositeDestination*>(c);
        if(child)return child->createPagedLODScene();
        else return 0;}

    if (_type==GROUP)
    {
        osg::Group* group = new osg::Group;
        for(TileList::iterator titr=_tiles.begin();
        titr!=_tiles.end();
        ++titr)
        {
            DestinationTile *t=*titr;

            MyDestinationTile *myt=dynamic_cast<MyDestinationTile*>(t);

            osg::Node* node = myt?myt->createScene(): 0;

            if (node) group->addChild(node);
        }

        // handle chilren
        for(ChildList::iterator citr=_children.begin();
        citr!=_children.end();
        ++citr)
        {
            CompositeDestination *c=*citr;
            MyCompositeDestination *child=dynamic_cast<MyCompositeDestination*>(c);
            osg::Node* node = child ? child->createScene() : 0;
            if (node) group->addChild(node);
        }
        return group;
    }

    // must be either a LOD or a PagedLOD

    typedef std::vector<osg::Node*>  NodeList;

    // collect all the local tiles
    NodeList tileNodes;
    for(TileList::iterator titr=_tiles.begin();
    titr!=_tiles.end();
    ++titr)
    {
        DestinationTile *t=*titr;

        MyDestinationTile *myt=dynamic_cast<MyDestinationTile*>(t);

        osg::Node* node = myt?myt->createScene(): 0;
        if (node) tileNodes.push_back(node);
    }

    float cutOffDistance = -FLT_MAX;
    for(ChildList::iterator citr=_children.begin();
    citr!=_children.end();
    ++citr)
    {
        cutOffDistance = osg::maximum(cutOffDistance,(*citr)->_maxVisibleDistance);
    }


    osg::PagedLOD* pagedLOD = new osg::PagedLOD;

    float farDistance = _dataSet->getMaximumVisibleDistanceOfTopLevel();
    if (tileNodes.size()==1)
    {
        pagedLOD->addChild(tileNodes.front());
    }
    else if (tileNodes.size()>1)
    {
        osg::Group* group = new osg::Group;
        for(NodeList::iterator itr=tileNodes.begin();
        itr != tileNodes.end();
        ++itr)
        {
            group->addChild(*itr);
        }
        pagedLOD->addChild(group);
    }


    // find cluster culling callbacks on drawables and move them to the PagedLOD level.
    {
        CollectClusterCullingCallbacks collect;
        pagedLOD->accept(collect);

        if (!collect._callbackList.empty())
        {
            if (collect._callbackList.size()==1)
            {
                CollectClusterCullingCallbacks::Triple& triple = collect._callbackList.front();

                osg::Matrixd matrix = osg::computeLocalToWorld(triple._nodePath);

                triple._callback->transform(matrix);

                log(osg::INFO,"cluster culling matrix %f\t%f\t%f\t%f",matrix(0,0),matrix(0,1),matrix(0,2),matrix(0,3));
                log(osg::INFO,"                       %f\t%f\t%f\t%f",matrix(1,0),matrix(1,1),matrix(1,2),matrix(1,3));
                log(osg::INFO,"                       %f\t%f\t%f\t%f",matrix(2,0),matrix(2,1),matrix(2,2),matrix(2,3));
                log(osg::INFO,"                       %f\t%f\t%f\t%f",matrix(3,0),matrix(3,1),matrix(3,2),matrix(3,3));

                // moving cluster culling callback pagedLOD node.
                pagedLOD->setCullCallback(triple._callback);

                osg::Drawable* drawable = dynamic_cast<osg::Drawable*>(triple._object);
                if (drawable) drawable->setCullCallback(0);

                osg::Node* node = dynamic_cast<osg::Node*>(triple._object);
                if (node) node->setCullCallback(0);
            }
        }
    }

    cutOffDistance = osg::maximum(cutOffDistance,(float)(pagedLOD->getBound().radius()*_dataSet->getRadiusToMaxVisibleDistanceRatio()));

    pagedLOD->setRange(0,cutOffDistance,farDistance);

    pagedLOD->setFileName(1,getExternalSubTileName());
    pagedLOD->setRange(1,0,cutOffDistance);

    if (pagedLOD->getNumChildren()>0)
        pagedLOD->setCenter(pagedLOD->getBound().center());

    return pagedLOD;
}
void MyDataSet::_buildDestination(bool writeToDisk)
{
    if (!_state) _state = new osg::State;

    osg::ref_ptr<osgDB::ReaderWriter::Options> previous_options = osgDB::Registry::instance()->getOptions();
    if(previous_options.get())
    {
        log(osg::NOTICE, "vpb: adding optionstring %s",previous_options->getOptionString().c_str());
        osgDB::Registry::instance()->setOptions(new osgDB::ReaderWriter::Options(std::string("precision 16") + std::string(" ") + previous_options->getOptionString()) );
    }
    else
    {
        osgDB::Registry::instance()->setOptions(new osgDB::ReaderWriter::Options("precision 16"));
    }

    if (!_archive && !_archiveName.empty())
    {
        unsigned int indexBlockSizeHint=4096;
        _archive = osgDB::openArchive(_archiveName, osgDB::Archive::CREATE, indexBlockSizeHint);
    }

    if (_destinationGraph.valid())
    {
#ifdef NEW_NAMING
        std::string filename = _destinationGraph->getTileFileName();
#else
        std::string filename = _directory+_tileBasename+_tileExtension;
#endif

        if (_archive.valid())
        {
            log(osg::NOTICE, "started MyDataSet::writeDestination(%s)",_archiveName.c_str());
            log(osg::NOTICE, "        archive file = %s",_archiveName.c_str());
            log(osg::NOTICE, "        archive master file = %s",filename.c_str());
        }
        else
        {
            log(osg::NOTICE, "started MyDataSet::writeDestination(%s)",filename.c_str());
        }

        /* if (_databaseType==LOD_DATABASE)
        {
            populateDestinationGraphFromSources();
            _rootNode = _destinationGraph->createScene();

            if (_decorateWithMultiTextureControl)
            {
                _rootNode = decorateWithMultiTextureControl(_rootNode.get());
            }

            if (getGeometryType()==TERRAIN)
            {
                _rootNode = decorateWithTerrain(_rootNode.get());
            }
            else if (_decorateWithCoordinateSystemNode)
            {
                _rootNode = decorateWithCoordinateSystemNode(_rootNode.get());
            }

            if (!_comment.empty())
            {
                _rootNode->addDescription(_comment);
            }

            if (writeToDisk)
            {
                _writeNodeFileAndImages(*_rootNode,filename);
            }
        }
        else  // _databaseType==PagedLOD_DATABASE*/
        {

            // for each level build read and write the rows.
            for(QuadMap::iterator qitr=_quadMap.begin();
                qitr!=_quadMap.end();
                ++qitr)
            {
                Level& level = qitr->second;

                // skip is level is empty.
                if (level.empty()) continue;

                // skip lower levels if we are generating subtiles
                if (getGenerateSubtile() && qitr->first<=getSubtileLevel()) continue;

                if (getRecordSubtileFileNamesOnLeafTile() && qitr->first>=getMaximumNumOfLevels()) continue;

                log(osg::INFO, "New level");

                Level::iterator prev_itr = level.begin();
                _readRow(prev_itr->second);
                Level::iterator curr_itr = prev_itr;
                ++curr_itr;
                for(;
                    curr_itr!=level.end();
                    ++curr_itr)
                {
                    _readRow(curr_itr->second);

                    _equalizeRow(prev_itr->second);
                    if (writeToDisk) _writeRow(prev_itr->second);

                    prev_itr = curr_itr;
                }

                _equalizeRow(prev_itr->second);

                if (writeToDisk)
                {
                    if (writeToDisk) _writeRow(prev_itr->second);
                }

#if 0
                if (_writeThreadPool.valid()) _writeThreadPool->waitForCompletion();
#endif
            }

        }

        if (_archive.valid())
        {
            log(osg::NOTICE, "completed MyDataSet::writeDestination(%s)",_archiveName.c_str());
            log(osg::NOTICE, "          archive file = %s",_archiveName.c_str());
            log(osg::NOTICE, "          archive master file = %s",filename.c_str());
        }
        else
        {
            log(osg::NOTICE, "completed MyDataSet::writeDestination(%s)",filename.c_str());
        }

        if (_writeThreadPool.valid()) _writeThreadPool->waitForCompletion();

    }
    else
    {
        log(osg::WARN, "Error: no scene graph to output, no file written.");
    }

    if (_archive.valid()) _archive->close();

    osgDB::Registry::instance()->setOptions(previous_options.get());

}
osg::Node* MyDestinationTile::createScene()
{
    if (_createdScene.valid()) return _createdScene.get();
/*
    if (_dataSet->getGeometryType()==DataSet::HEIGHT_FIELD)
    {
        _createdScene = createHeightField();
    }
    else if (_dataSet->getGeometryType()==DataSet::TERRAIN)
    {
        _createdScene = createTerrainTile();
    }
    else
    {*/
    // _createdScene = createPolygonal();
    //}

    if (_models.valid())
    {
        /*  if (_dataSet->getModelPlacer())
        {
            for(ModelList::iterator itr = _models->_models.begin();
                itr != _models->_models.end();
                ++itr)
            {
                _dataSet->getModelPlacer()->place(*this, itr->get());
            }
        }
        else*/
        {
            for(ModelList::iterator itr = _models->_models.begin();
            itr != _models->_models.end();
            ++itr)
            {
                addNodeToScene(itr->get());
            }
        }

        /*  if (_dataSet->getShapeFilePlacer())
        {
            for(ModelList::iterator itr = _models->_shapeFiles.begin();
                itr != _models->_shapeFiles.end();
                ++itr)
            {
                _dataSet->getShapeFilePlacer()->place(*this, itr->get());
            }
        }
        else
        {
            for(ModelList::iterator itr = _models->_shapeFiles.begin();
                itr != _models->_shapeFiles.end();
                ++itr)
            {
                addNodeToScene(itr->get());
            }
        }*/
    }else {
        if (!_createdScene) _createdScene = new osg::Group;
    }

    return _createdScene.get();
}

MyCompositeDestination* MyDataSet::createDestinationTile(int currentLevel, int currentX, int currentY)
{
    CompositeDestination* parent = 0;
    GeospatialExtents extents;

    if (currentLevel==0)
    {
        extents = _destinationExtents;
    }
    else
    {
        // compute the extents
        double destination_xRange = _destinationExtents.xMax()-_destinationExtents.xMin();
        double destination_yRange = _destinationExtents.yMax()-_destinationExtents.yMin();

        int Ck = int(pow(2.0, double(currentLevel-1))) * _C1;
        int Rk = int(pow(2.0, double(currentLevel-1))) * _R1;

        extents.xMin() = _destinationExtents.xMin() + (double(currentX)/double(Ck)) * destination_xRange;
        extents.xMax() = _destinationExtents.xMin() + (double(currentX+1)/double(Ck)) * destination_xRange;

        extents.yMin() = _destinationExtents.yMin() + (double(currentY)/double(Rk)) * destination_yRange;
        extents.yMax() = _destinationExtents.yMin() + (double(currentY+1)/double(Rk)) * destination_yRange;

        // compute the parent
        if (currentLevel == 1)
        {
            parent = _destinationGraph.get();
        }
        else
        {
            parent = getComposite(currentLevel-1,currentX/2,currentY/2);

            if (!parent)
            {
                log(osg::NOTICE,"Warning: getComposite(%i,%i,%i) return 0",currentLevel-1,currentX/2,currentY/2);
            }
        }

    }

    MyCompositeDestination* destinationGraph = new MyCompositeDestination(_intermediateCoordinateSystem.get(),extents);

    if (currentLevel==0) _destinationGraph = destinationGraph;

    /*if (mapLatLongsToXYZ())
    {
        // we need to project the extents into world coords to get the appropriate size to use for control max visible distance
        float max_range = osg::maximum(extents.xMax()-extents.xMin(),extents.yMax()-extents.yMin());
        float projected_radius =  osg::DegreesToRadians(max_range) * getEllipsoidModel()->getRadiusEquator();
        float center_offset = (max_range/360.0f) * getEllipsoidModel()->getRadiusEquator();
        destinationGraph->_maxVisibleDistance = projected_radius * getRadiusToMaxVisibleDistanceRatio() + center_offset;
    }
    else*/
    {
        destinationGraph->_maxVisibleDistance = extents.radius()*getRadiusToMaxVisibleDistanceRatio();
    }

    // first create the topmost tile

    // create the name
    std::ostringstream os;
    os << _tileBasename << "_L"<<currentLevel<<"_X"<<currentX<<"_Y"<<currentY;
    //std::cout << os.str() << "extents " << extents._min  << " " << extents._max<<std::endl;
    destinationGraph->_parent = parent;
    destinationGraph->_name = os.str();
    destinationGraph->_level = currentLevel;
    destinationGraph->_tileX = currentX;
    destinationGraph->_tileY = currentY;
    destinationGraph->_dataSet = this;


    MyDestinationTile* tile = new MyDestinationTile;
    tile->_name = destinationGraph->_name;
    tile->_level = currentLevel;
    tile->_tileX = currentX;
    tile->_tileY = currentY;
    tile->_dataSet = this;
    tile->_cs = destinationGraph->_cs;
    tile->_extents = extents;
    tile->_parent = destinationGraph;

    // set to NONE as the tile is a mix of RASTER and VECTOR
    // that way the default of RASTER for image and VECTOR for height is maintained
    tile->_dataType = SpatialProperties::NONE;

    tile->setMaximumTerrainSize(_maximumTileTerrainSize,_maximumTileTerrainSize);

    destinationGraph->_tiles.push_back(tile);

    if (parent)
    {
        parent->_type = LOD;
        parent->addChild(destinationGraph);
    }


    insertTileToQuadMap(destinationGraph);

    return destinationGraph;
}

void MyDataSet::createNewDestinationGraph(
        const GeospatialExtents& extents,
        unsigned int maxImageSize,
        unsigned int maxTerrainSize,
        unsigned int maxNumLevels)
{


    //int highestLevelFound = 0;
    _destinationExtents=extents;
    computeOptimumTileSystemDimensions(_C1,_R1);

    // first populate the destination graph from imagery and DEM sources extents/resolution
    //for(CompositeSource::source_iterator itr(_sourceGraph.get());itr.valid();++itr)
    {
        /*  Source* source = (*itr).get();



        SourceData* sd = (*itr)->getSourceData();
        if (!sd)
        {
            log(osg::NOTICE,"Skipping source %s as no data loaded from it.",source->getFileName().c_str());
            continue;
        }

        const SpatialProperties& sp = sd->computeSpatialProperties(cs);

        if (!sp._extents.intersects(extents))
        {
            // skip this source since it doesn't overlap this tile.
            log(osg::NOTICE,"Skipping source %s as its extents don't overlap destination extents.",source->getFileName().c_str());
            continue;
        }


        if (source->getType()!=Source::IMAGE && source->getType()!=Source::HEIGHT_FIELD)
        {
            continue;
        }

        int k = 0;
        if (!computeOptimumLevel(source, maxNumLevels-1, k)) continue;


        if (k>highestLevelFound) highestLevelFound = k;
*/
        int k=maxNumLevels;
        int startLevel = 0; // getGenerateSubtile() ? getSubtileLevel() : 0;

        for(int l=startLevel; l<=k; l++)
        {
            int i_min, i_max, j_min, j_max;
            if (computeCoverage(extents, l, i_min, j_min, i_max, j_max))
            {
                printf("     level=%i i_min=%i i_max=%i j_min=%i j_max=%i\n",l, i_min, i_max, j_min, j_max);

                if (getGenerateSubtile())
                {
                    int i_lower, i_upper, j_lower, j_upper;

                    if (l<static_cast<int>(getSubtileLevel()))
                    {
                        // divide by 2 to the power of ((getSubtileLevel()-l);
                        int delta = getSubtileLevel()-l;
                        i_lower = getSubtileX() >> delta;
                        j_lower = getSubtileY() >> delta;
                        i_upper = i_lower + 1;
                        j_upper = j_lower + 1;
                    }
                    else
                    {
                        // multiply 2 to the power of ((l-getSubtileLevel());
                        int f = 1 << (l-getSubtileLevel());
                        i_lower = getSubtileX() * f;
                        j_lower = getSubtileY() * f;
                        i_upper = i_lower + f;
                        j_upper = j_lower + f;
                    }

                    if (i_min<i_lower) i_min = i_lower;
                    if (i_max>i_upper) i_max = i_upper;
                    if (j_min<j_lower) j_min = j_lower;
                    if (j_max>j_upper) j_max = j_upper;
                }

                for(int j=j_min; j<j_max;++j)
                {
                    for(int i=i_min; i<i_max;++i)
                    {
                        CompositeDestination* cd = getComposite(l,i,j);
                        if (!cd)
                        {
                            cd = createDestinationTile(l,i,j);
                        }
                    }
                }
            }
        }
    }
   // printf("Total Size %d\n",(int)_quadMap.size());
    // now extend the sources upwards where required.
    for(QuadMap::iterator qitr = _quadMap.begin();
    qitr != _quadMap.end();
    ++qitr)
    {
        QuadMap::iterator temp_itr = qitr;
        ++temp_itr;
        if (temp_itr==_quadMap.end()) continue;

        int l = qitr->first;

        Level& level = qitr->second;
        for(Level::iterator litr = level.begin();
        litr != level.end();
        ++litr)
        {
            Row& row = litr->second;
            for(Row::iterator ritr = row.begin();
            ritr != row.end();
            ++ritr)
            {
                CompositeDestination* cd = ritr->second;

                int numChildren = cd->_children.size();
                int numChildrenExpected = (l==0) ? (_C1*_R1) : 4;
                if (numChildren!=0 && numChildren!=numChildrenExpected)
                {
#if 0
                    printf("  tile (%i,%i,%i) numTiles=%i numChildren=%i\n",
                           cd->_level, cd->_tileX, cd->_tileY, cd->_tiles.size(), cd->_children.size());
#endif
                    int i_min = (l==0) ? 0   : (cd->_tileX * 2);
                    int j_min = (l==0) ? 0   : (cd->_tileY * 2);
                    int i_max = (l==0) ? _C1 : i_min + 2;
                    int j_max = (l==0) ? _R1 : j_min + 2;
                    int new_l = l+1;

                    if (getGenerateSubtile())
                    {
                        int i_lower, i_upper, j_lower, j_upper;


                        if (l<static_cast<int>(getSubtileLevel()))
                        {
                            // divide by 2 to the power of ((getSubtileLevel()-new_l);
                            int delta = getSubtileLevel()-new_l;
                            i_lower = getSubtileX() >> delta;
                            j_lower = getSubtileY() >> delta;
                            i_upper = i_lower + 1;
                            j_upper = j_lower + 1;
                        }
                        else
                        {
                            // multiply 2 to the power of ((new_l-getSubtileLevel());
                            int f = 1 << (new_l-getSubtileLevel());
                            i_lower = getSubtileX() * f;
                            j_lower = getSubtileY() * f;
                            i_upper = i_lower + f;
                            j_upper = j_lower + f;
                        }

                        if (i_min<i_lower) i_min = i_lower;
                        if (i_max>i_upper) i_max = i_upper;
                        if (j_min<j_lower) j_min = j_lower;
                        if (j_max>j_upper) j_max = j_upper;
                    }

                    for(int j=j_min; j<j_max;++j)
                    {
                        for(int i=i_min; i<i_max;++i)
                        {
                            CompositeDestination* cd = getComposite(new_l,i,j);
                            if (!cd)
                            {
                                cd = createDestinationTile(new_l,i,j);
                            }
                        }
                    }

                }
            }
        }
    }

    // now insert the sources into the destination graph
    for(CompositeSource::source_iterator itr(_sourceGraph.get());itr.valid();++itr)
    {
        Source* source = (*itr).get();

        if (source->getMinLevel()>maxNumLevels)
        {
            log(osg::ALWAYS,"Skipping source %s as its min level excees destination max level.",source->getFileName().c_str());
            continue;
        }

        if (getGenerateSubtile() && source->getMaxLevel()<getSubtileLevel())
        {
            log(osg::ALWAYS,"Skipping source %s as its max level is lower than the subtile level.",source->getFileName().c_str());
            continue;
        }

        SourceData* sd = (*itr)->getSourceData();
        if (!sd)
        {
            log(osg::ALWAYS,"Skipping source %s as no data loaded from it.",source->getFileName().c_str());
            continue;
        }
        osg::CoordinateSystemNode* cs = _intermediateCoordinateSystem.get();

        const SpatialProperties& sp = sd->computeSpatialProperties(cs);
        //std::cout << sp._extents._min << " " << sp._extents._max << " " << source->getFileName() <<std::endl;
        if (!sp._extents.intersects(extents))
        {
            // skip this source since it doesn't overlap this tile.
            log(osg::ALWAYS,"Skipping source %s as its extents don't overlap destination extents.",source->getFileName().c_str());
            continue;
        }

        int k = maxNumLevels;

        /* if (source->getType()==Source::IMAGE || source->getType()==Source::HEIGHT_FIELD)
        {
            if (!computeOptimumLevel(source, maxNumLevels-1, k)) continue;
        }
        else
        {
            k = highestLevelFound;
        }
*/

       // log(osg::ALWAYS,"     opt level = %i",k);

        int startLevel = 0; // getGenerateSubtile() ? getSubtileLevel() : 0;

        for(int l=startLevel; l<=k; l++)
        {
            int i_min, i_max, j_min, j_max;
            if (computeCoverage(extents, l, i_min, j_min, i_max, j_max))
            {
                // log(osg::NOTICE,"     level=%i i_min=%i i_max=%i j_min=%i j_max=%i",l, i_min, i_max, j_min, j_max);

                if (getGenerateSubtile())
                {
                    int i_lower, i_upper, j_lower, j_upper;

                    if (l<static_cast<int>(getSubtileLevel()))
                    {
                        // divide by 2 to the power of ((getSubtileLevel()-l);
                        int delta = getSubtileLevel()-l;
                        i_lower = getSubtileX() >> delta;
                        j_lower = getSubtileY() >> delta;
                        i_upper = i_lower + 1;
                        j_upper = j_lower + 1;
                    }
                    else
                    {
                        // multiply 2 to the power of ((l-getSubtileLevel());
                        int f = 1 << (l-getSubtileLevel());
                        i_lower = getSubtileX() * f;
                        j_lower = getSubtileY() * f;
                        i_upper = i_lower + f;
                        j_upper = j_lower + f;
                    }

                    if (i_min<i_lower) i_min = i_lower;
                    if (i_max>i_upper) i_max = i_upper;
                    if (j_min<j_lower) j_min = j_lower;
                    if (j_max>j_upper) j_max = j_upper;
                }

                for(int j=j_min; j<j_max;++j)
                {
                    for(int i=i_min; i<i_max;++i)
                    {
                        CompositeDestination* cd = getComposite(l,i,j);
                        if (!cd || !cd->intersects(sp) || (l <(int)source->getMinLevel() || l > (int)source->getMaxLevel())) continue;

                       // printf("Tile %d %d_%d %s\n",l ,i,j,source->getFileName().c_str());
                        /*  if (l==k)
                        {
                            cd->addSource(source);
                        }
                        else*/
                        {
                            for(CompositeDestination::TileList::iterator titr = cd->_tiles.begin();
                            titr != cd->_tiles.end();
                            ++titr)
                            {
                                DestinationTile* tile = titr->get();
                                tile->_sources.push_back(source);
                            }
                        }
                    }
                }
            }
        }
    }

    osg::Timer_t before_computeMax = osg::Timer::instance()->tick();

    if (_destinationGraph.valid()) _destinationGraph->computeMaximumSourceResolution();

    osg::Timer_t after_computeMax = osg::Timer::instance()->tick();

    log(osg::NOTICE,"Time for _destinationGraph->computeMaximumSourceResolution() = %f", osg::Timer::instance()->delta_s(before_computeMax, after_computeMax));
}

void MyDataSet::_readRow(Row& row)
{
    log(osg::NOTICE, "_readRow %u",row.size());

    //CompositeSource* sourceGraph = _newDestinationGraph ? 0 : _sourceGraph.get();

    /*  if (_readThreadPool.valid())
    {
        for(Row::iterator citr=row.begin();
            citr!=row.end();
            ++citr)
        {
            CompositeDestination* cd = citr->second;
            for(CompositeDestination::TileList::iterator titr=cd->_tiles.begin();
                titr!=cd->_tiles.end();
                ++titr)
            {
                _readThreadPool->run(new vpb::DataSet::ReadFromOperation(_readThreadPool.get(), getBuildLog(), titr->get(), sourceGraph));
            }
        }

        // wait for the threads to complete.
        _readThreadPool->waitForCompletion();

    }
    else*/
    {
        for(Row::iterator citr=row.begin();
        citr!=row.end();
        ++citr)
        {
            CompositeDestination* cd = citr->second;
            for(CompositeDestination::TileList::iterator titr=cd->_tiles.begin();
            titr!=cd->_tiles.end();
            ++titr)
            {
                DestinationTile* tile = titr->get();
                log(osg::NOTICE, "   reading tile level=%u X=%u Y=%u",tile->_level,tile->_tileX,tile->_tileY);
                //tile->readFrom(sourceGraph);
                for(DestinationTile::Sources::iterator itr = tile->_sources.begin();
                itr != tile->_sources.end();
                ++itr)
                {
                    log(osg::NOTICE,"   source:%s",(*itr)->getFileName().c_str());
                 //   log(osg::NOTICE,"    %x",(*itr)->getSourceData()->_model.get());
                    osg::BoundingBox ext_bbox(osg::Vec3d(tile->_extents._min.x(),
                                                         tile->_extents._min.y(),
                                                         DBL_MIN),osg::Vec3d(tile->_extents._max.x(),
                                                                             tile->_extents._max.y(),
                                                                             DBL_MAX));
                    //Clipper clipper(ext_bbox);
                    int texSizeIdx=0;

                    if(tile->_level == 1){
                        texSizeIdx=1;
          //              clipper.setColor(osg::Vec4(1,0,0,1));
                    }
                    else if(tile->_level == 2){
                        texSizeIdx=0;
        //                clipper.setColor(osg::Vec4(0,1,0,1));
                    }
                    else if(tile->_level == 3){
                        texSizeIdx=0;
      //                  clipper.setColor(osg::Vec4(0,0.5,0.5,1));
                    }
                    else if(tile->_level == 0){
                        texSizeIdx=2;
    //                    clipper.setColor(osg::Vec4(0,0,1,1));
                    }else
                        texSizeIdx=0;

  //                  clipper.setApplyColor(true);
                    //  osg::ref_ptr<osg::Node> root = (osg::Node*)(*itr)->getSourceData()->_model.get()->clone(osg::CopyOp::DEEP_COPY_ALL);

                    //   root->accept(clipper);
                    ClippedCopy cl_cp(ext_bbox);
                    osg::ref_ptr<osg::Node> root =cl_cp.makeCopy((osg::Geode*)(*itr)->getSourceData()->_model.get());
                    _tq->projectModel(dynamic_cast<osg::Geode*>(root.get()),texSizeIdx);
                    //(*itr)->getSourceData()->
                    //    tile->_models->
                    if(!tile->_models){
                        tile->_models = new DestinationData(NULL);
                    }
                    tile->_models->_models.push_back(root.get());



                }
            }
        }
    }
}

void ClippedCopy::AnalyzePrimSet(const osg::PrimitiveSet& prset, const osg::Vec3Array &verts)
{
    //std::cout  << "Prim set type "<< prset.getMode() << std::endl;
    std::vector<int> vertRemap(verts.size(),-1);
    if(!_triangles.valid())
        _triangles = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    if(1)
    {
        unsigned int ic;
        //unsigned int nprim=0;
        //unsigned int numValid=0;
        for (ic=0; ic < prset.getNumIndices(); ic++)
        {
            // NB the vertices are held in the drawable -
            /*std::cout  <<  "vertex "<< ic << " is index "<<prset.index(ic) << " at " <<
           (verts)[prset.index(ic)].x() << "," <<
           (verts)[prset.index(ic)].y() << "," <<
           (verts)[prset.index(ic)].z() << std::endl;*/
            if(_bbox.contains( (verts)[prset.index(ic)])){
                vertRemap[prset.index(ic)]=_vertices->size();
                _vertices->push_back((verts)[prset.index(ic)]);
            }



        }
        // you might want to handle each type of primset differently: such as:

        switch (prset.getMode())
        {
        case osg::PrimitiveSet::TRIANGLES: // get vertices of triangle
            {
               // std::cout << "Triangles "<< nprim << " is index "<<prset.index(ic) << std::endl;
                for(unsigned int i2=0; i2<prset.getNumIndices()-2; i2+=3)
                {
                    std::vector<bool> outside(3,false);
                    for(int k=0; k <3; k++){
                        outside[k]=(vertRemap[prset.index(i2+k)]>= 0);
                    }

                    if(outside[0] && outside[1] && outside[2]){
                        for(int k=0; k <3; k++){
                            _triangles->push_back(vertRemap[prset.index(i2+k)]);
                        }
                    }
                }
            }
            break;

        default:
            break;
        }
    }
}

osg::Geode* ClippedCopy::makeCopy(osg::Geode *geode){
    osg::Geode *newGeode=new osg::Geode;
    osg::Geometry *new_geom=new osg::Geometry;
    _vertices=new osg::Vec3Array;
    for (unsigned int i=0; i<geode->getNumDrawables(); i++)
    {
        osg::Drawable *drawable = geode->getDrawable(i);
        osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
        osg::Geometry::PrimitiveSetList& primitiveSets = geom->getPrimitiveSetList();
        osg::Geometry::PrimitiveSetList::iterator itr;


        for(itr=primitiveSets.begin(); itr!=primitiveSets.end(); ++itr){
            switch((*itr)->getMode()){

            case(osg::PrimitiveSet::TRIANGLES):
                //  for( unsigned int j = 0; j < (*itr)->getNumPrimitives(); j++ )
                // {
                //   const osg::PrimitiveSet* prset = (*itr);//geom->getPrimitiveSet(j);
                AnalyzePrimSet(*(*itr), *static_cast<const osg::Vec3Array*>(geom->getVertexArray()));

                break;

            default:
                fprintf(stderr,"ERRROR Only supported for TRIANGLES\n");
            }
        }
    }
    new_geom->addPrimitiveSet(_triangles);
    new_geom->setVertexArray(_vertices.get());
    //osgUtil::SmoothingVisitor::smooth(*new_geom);
    newGeode->addDrawable(new_geom);
    return newGeode;
}


class MyGraphicsContext : public osg::Referenced {
public:
    MyGraphicsContext(BuildLog* buildLog)
    {
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->readDISPLAY();
        traits->x = 0;
        traits->y = 0;
        traits->width = 1;
        traits->height = 1;
        traits->windowDecoration = false;
        traits->doubleBuffer = false;
        traits->sharedContext = 0;
        traits->pbuffer = true;

        _gc = osg::GraphicsContext::createGraphicsContext(traits.get());

        if (!_gc)
        {
            if (buildLog) buildLog->log(osg::NOTICE,"Failed to create pbuffer, failing back to normal graphics window.");

            traits->pbuffer = false;
            _gc = osg::GraphicsContext::createGraphicsContext(traits.get());
        }

        if (_gc.valid())


        {
            _gc->realize();
            _gc->makeCurrent();

            if (buildLog) buildLog->log(osg::NOTICE,"Realized window");
        }
    }

    bool valid() const { return _gc.valid() && _gc->isRealized(); }

private:
    osg::ref_ptr<osg::GraphicsContext> _gc;
};

int MyDataSet::_run()
{

    log(osg::NOTICE,"MyDataSet::_run() %i %i",getDistributedBuildSplitLevel(),getDistributedBuildSecondarySplitLevel());
/*
#ifdef HAVE_NVTT
    bool requiresGraphicsContextInMainThread = (getCompressionMethod() == vpb::BuildOptions::GL_DRIVER);
    bool requiresGraphicsContextInWritingThread = (getCompressionMethod() == vpb::BuildOptions::GL_DRIVER);
#else
    bool requiresGraphicsContextInMainThread = true;
    bool requiresGraphicsContextInWritingThread = true;
#endif

    int numProcessors = OpenThreads::GetNumberOfProcessors();
    #if 0
    if (numProcessors>1)
#endif
    {
        int numReadThreads = int(ceilf(getNumReadThreadsToCoresRatio() * float(numProcessors)));
        if (numReadThreads>=1)
        {
            log(osg::NOTICE,"Starting %i read threads.",numReadThreads);
            _readThreadPool = new ThreadPool(numReadThreads, false);
            _readThreadPool->startThreads();
        }

        int numWriteThreads = int(ceilf(getNumWriteThreadsToCoresRatio() * float(numProcessors)));
        if (numWriteThreads>=1)
        {
            log(osg::NOTICE,"Starting %i write threads.",numWriteThreads);
            _writeThreadPool = new ThreadPool(numWriteThreads, requiresGraphicsContextInWritingThread);
            _writeThreadPool->startThreads();

            //requiresGraphicsContextInMainThread = false;
        }
    }


    //loadSources();

    int numLevels = getMaximumNumOfLevels();
    if (getRecordSubtileFileNamesOnLeafTile()) ++numLevels;

    createDestination(numLevels);

    if (!_destinationGraph)
    {
        log(osg::WARN, "Error: no destination graph built, cannot proceed with build.");
        return 1;
    }

*/
    bool requiresGenerationOfTiles = getGenerateTiles();

    /*  if (!getIntermediateBuildName().empty())
    {
        osg::ref_ptr<osgTerrain::TerrainTile> terrainTile = createTerrainRepresentation();
        if (terrainTile.valid())
        {
            DatabaseBuilder* db = dynamic_cast<DatabaseBuilder*>(terrainTile->getTerrainTechnique());
            if (db && db->getBuildOptions())
            {
                db->getBuildOptions()->setIntermediateBuildName("");
            }
            _writeNodeFile(*terrainTile,getIntermediateBuildName());
            requiresGenerationOfTiles = false;
        }
    }


    bool printOutContributingSources = true;
    if (printOutContributingSources)
    {
        CompositeDestination* startPoint = _destinationGraph.get();
        if (getGenerateSubtile())
        {
            startPoint = getComposite(getSubtileLevel(), getSubtileX(), getSubtileY());
        }

        if (startPoint)
        {
            DestinationTile::Sources sources = startPoint->getAllContributingSources();
            log(osg::NOTICE,"There are %d contributing source files:",sources.size());

            for(DestinationTile::Sources::iterator itr = sources.begin();
                itr != sources.end();
                ++itr)
            {
                log(osg::NOTICE,"    %s",(*itr)->getFileName().c_str());
            }
        }
        else
        {
            log(osg::NOTICE,"Warning: No destination graph generated.");
        }
    }
*/
    if (requiresGenerationOfTiles)
    {
        // dummy Viewer to get round silly Windows autoregistration problem for GraphicsWindowWin32.cpp
        //osgViewer::Viewer viewer;
        /*
        osg::ref_ptr<MyGraphicsContext> context;

        if (requiresGraphicsContextInMainThread)
        {
            context  = new MyGraphicsContext(getBuildLog());
            if (!context || !context->valid())
            {
                log(osg::NOTICE,"Error: Unable to create graphis context, problem with running osgViewer-%s, cannot run compression.",osgViewerGetVersion());
                return 1;
            }
        }
*/
        int result = 0;
        osgDB::FileType type = osgDB::fileType(getDirectory());
        if (type==osgDB::DIRECTORY)
        {
            log(osg::NOTICE,"   Base Directory already created");
        }
        else if (type==osgDB::REGULAR_FILE)
        {
            log(osg::NOTICE,"   Error cannot create directory as a conventional file already exists with that name");
            return 1;
        }
        else // FILE_NOT_FOUND
        {
            // need to create directory.
            result = vpb::mkpath(getDirectory().c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
        }

        if (result)
        {
            log(osg::NOTICE,"Error: could not create directory, errorno=%i",errno);
            return 1;
        }


        if (getOutputTaskDirectories())
        {
            _taskOutputDirectory = getDirectory() + getTaskName(getSubtileLevel(), getSubtileX(), getSubtileY());
            log(osg::NOTICE,"Need to create output task directory = %s", _taskOutputDirectory.c_str());
            result = 0;
            type = osgDB::fileType(_taskOutputDirectory);
            if (type==osgDB::DIRECTORY)
            {
                log(osg::NOTICE,"   Directory already created");
            }
            else if (type==osgDB::REGULAR_FILE)
            {
                log(osg::NOTICE,"   Error cannot create directory as a conventional file already exists with that name");
                return 1;
            }
            else // FILE_NOT_FOUND
            {
                // need to create directory.
                result = vpb::mkpath(_taskOutputDirectory.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
            }

            if (result)
            {
                log(osg::NOTICE,"Error: could not create directory, errorno=%i",errno);
                return 1;
            }

#ifdef WIN32
            _taskOutputDirectory.push_back('\\');
#else
            _taskOutputDirectory.push_back('/');
#endif

            if (getGenerateSubtile()) log(osg::NOTICE,"We are a subtile");
            if (getRecordSubtileFileNamesOnLeafTile()) log(osg::NOTICE,"We have to record ../task/name");
        }
        else
        {
            _taskOutputDirectory = getDirectory();
        }

        log(osg::NOTICE,"Task output directory = %s", _taskOutputDirectory.c_str());

        writeDestination();
    }

    return 0;
}

osg::Node* MyCompositeDestination::createSubTileScene()
{
    if (_type==GROUP ||
        _children.empty() ||
        _tiles.empty()) return 0;

    // handle chilren
    typedef std::vector<osg::Node*>  NodeList;
    NodeList nodeList;
    for(ChildList::iterator citr=_children.begin();
    citr!=_children.end();
    ++citr)
    {
        CompositeDestination *c=*citr;
        MyCompositeDestination *child=dynamic_cast<MyCompositeDestination*>(c);
        osg::Node* node = child ? child->createPagedLODScene() : 0;
        if (node) nodeList.push_back(node);
    }

    if (nodeList.size()==1)
    {
        return nodeList.front();
    }
    else if (nodeList.size()>1)
    {
        osg::Group* group = new osg::Group;
        for(NodeList::iterator itr=nodeList.begin();
        itr!=nodeList.end();
        ++itr)
        {
            group->addChild(*itr);
        }
        return group;
    }
    else
    {
        return 0;
    }
}
void MyDataSet::_writeRow(Row& row)
{
    log(osg::NOTICE, "_writeRow %u",row.size());
    for(Row::iterator citr=row.begin();
    citr!=row.end();
    ++citr)
    {
        MyCompositeDestination* cd = dynamic_cast<MyCompositeDestination*>(citr->second);
        MyCompositeDestination* parent =  dynamic_cast<MyCompositeDestination*>(cd->_parent);

        if (parent)
        {
            if (!parent->getSubTilesGenerated() && parent->areSubTilesComplete())
            {
                parent->setSubTilesGenerated(true);

#ifdef NEW_NAMING
                std::string filename = cd->getTileFileName();
#else
                std::string filename = _taskOutputDirectory+parent->getSubTileName();
#endif
                log(osg::NOTICE, "       _taskOutputDirectory= %s",_taskOutputDirectory.c_str());

                /*  if (_writeThreadPool.valid())
                {
                    _writeThreadPool->run(new WriteOperation(_writeThreadPool.get(), this, parent, filename));
                }
                else*/
                {
                    osg::ref_ptr<osg::Node> node = parent->createSubTileScene();
                    if (node.valid())
                    {
                        log(osg::NOTICE, "   writeSubTile filename= %s",filename.c_str());
                        _writeNodeFileAndImages(*node,filename);


                        parent->setSubTilesGenerated(true);
                        parent->unrefSubTileData();

                    }
                    else
                    {
                        log(osg::WARN, "   failed to writeSubTile node for tile, filename=%s",filename.c_str());
                    }
                }
            }
        }
        else
        {
            osg::ref_ptr<osg::Node> node = cd->createPagedLODScene();

#ifdef NEW_NAMING
            std::string filename = cd->getTileFileName();
#else
            std::string filename;
#endif

            if (cd->_level==0)
            {

#ifndef NEW_NAMING
                filename = getDirectory() + _tileBasename + _tileExtension;
#endif
                if (_decorateWithMultiTextureControl)
                {
                    node = decorateWithMultiTextureControl(node.get());
                }

                if (getGeometryType()==TERRAIN)
                {
                    node = decorateWithTerrain(node.get());
                }
                else if (_decorateWithCoordinateSystemNode)
                {
                    node = decorateWithCoordinateSystemNode(node.get());
                }

                if (!_comment.empty())
                {
                    node->addDescription(_comment);
                }

                log(osg::NOTICE, "       getDirectory()= %s",getDirectory().c_str());
            }
            else
            {
#ifndef NEW_NAMING
                filename = _taskOutputDirectory + _tileBasename + _tileExtension;
#endif

                log(osg::NOTICE, "       _taskOutputDirectory= %s",_taskOutputDirectory.c_str());
            }

            if (node.valid())
            {
                log(osg::NOTICE, "   writeNodeFile = %u X=%u Y=%u filename=%s",cd->_level,cd->_tileX,cd->_tileY,filename.c_str());

                _writeNodeFileAndImages(*node,filename);
            }
            else
            {
                log(osg::WARN, "   faild to write node for tile = %u X=%u Y=%u filename=%s",cd->_level,cd->_tileX,cd->_tileY,filename.c_str());
            }

            // record the top nodes as the rootNode of the database
            _rootNode = node;

        }
    }

#if 0
    if (_writeThreadPool.valid()) _writeThreadPool->waitForCompletion();
#endif

}
