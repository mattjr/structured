/* -*-c++-*- OpenSceneGraph example, osgposter.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
*/

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <iostream>
#include <sstream>
#include <osg/io_utils>
#include "PosterPrinter.h"
#include<osgUtil/LineSegmentIntersector>
#include <string.h>
//Include the stuff for the geotifs
#include <geotiff/geotiff.h>
#include <geotiff/xtiffio.h>
#include <GeographicConversions/ufRedfearn.h>

#include <adt_write_gtiff.hpp>
#include <adt_file_utils.hpp>
/* PagedLoadingCallback: Callback for loading paged nodes while doing intersecting test */
struct PagedLoadingCallback : public osgUtil::IntersectionVisitor::ReadCallback
{
    virtual osg::Node* readNodeFile( const std::string& filename )
    {
        return osgDB::readNodeFile( filename );
    }
};
static osg::ref_ptr<PagedLoadingCallback> g_pagedLoadingCallback = new PagedLoadingCallback;

/* LodCullingCallback: Callback for culling LODs and selecting the highest level */
class LodCullingCallback : public osg::NodeCallback
{
public:
    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv )
    {
        osg::LOD* lod = static_cast<osg::LOD*>(node);
        if ( lod && lod->getNumChildren()>0 )
            lod->getChild(lod->getNumChildren()-1)->accept(*nv);
    }
};
static osg::ref_ptr<LodCullingCallback> g_lodCullingCallback = new LodCullingCallback;
/* PagedCullingCallback: Callback for culling paged nodes and selecting the highest level */
class PagedCullingCallback : public osg::NodeCallback
{
public:
    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv )
    {
        osg::PagedLOD* pagedLOD = static_cast<osg::PagedLOD*>(node);
        if ( pagedLOD && pagedLOD->getNumChildren()>0 )
        {
            unsigned int numChildren = pagedLOD->getNumChildren();
            bool updateTimeStamp = nv->getVisitorType()==osg::NodeVisitor::CULL_VISITOR;
            if ( nv->getFrameStamp() && updateTimeStamp )
            {
                double timeStamp = nv->getFrameStamp()?nv->getFrameStamp()->getReferenceTime():0.0;
                int frameNumber = nv->getFrameStamp()?nv->getFrameStamp()->getFrameNumber():0;
                
                pagedLOD->setFrameNumberOfLastTraversal( frameNumber );
                pagedLOD->setTimeStamp( numChildren-1, timeStamp );
                pagedLOD->setFrameNumber( numChildren-1, frameNumber );
                pagedLOD->getChild(numChildren-1)->accept(*nv);
            }
            
            // Request for new child
            bool ret=false;
#if OPENSCENEGRAPH_MAJOR_VERSION>2 || OPENSCENEGRAPH_MAJOR_VERSION==2&&OPENSCENEGRAPH_MINOR_VERSION>=9
            ret=pagedLOD->getDisableExternalChildrenPaging();
#endif
            if ( !ret &&
                 nv->getDatabaseRequestHandler() &&
                 numChildren<pagedLOD->getNumRanges() )
            {
                if ( pagedLOD->getDatabasePath().empty() )
                {
#if OPENSCENEGRAPH_MAJOR_VERSION>2 || OPENSCENEGRAPH_MAJOR_VERSION==2&&OPENSCENEGRAPH_MINOR_VERSION>=9

                    nv->getDatabaseRequestHandler()->requestNodeFile(
                        pagedLOD->getFileName(numChildren), pagedLOD,
                        1.0, nv->getFrameStamp(),
                        pagedLOD->getDatabaseRequest(numChildren), pagedLOD->getDatabaseOptions() );
#else
                       nv->getDatabaseRequestHandler()->requestNodeFile(
                        pagedLOD->getFileName(numChildren), pagedLOD,
                        1.0, nv->getFrameStamp(),
                        pagedLOD->getDatabaseRequest(numChildren));
#endif

                }
                else
                {
#if OPENSCENEGRAPH_MAJOR_VERSION>2 || OPENSCENEGRAPH_MAJOR_VERSION==2&&OPENSCENEGRAPH_MINOR_VERSION>=9

                    nv->getDatabaseRequestHandler()->requestNodeFile(
                        pagedLOD->getDatabasePath()+pagedLOD->getFileName(numChildren), pagedLOD,
                        1.0, nv->getFrameStamp(),
                        pagedLOD->getDatabaseRequest(numChildren), pagedLOD->getDatabaseOptions() );
#else
                     nv->getDatabaseRequestHandler()->requestNodeFile(
                        pagedLOD->getDatabasePath()+pagedLOD->getFileName(numChildren), pagedLOD,
                        1.0, nv->getFrameStamp(),
                        pagedLOD->getDatabaseRequest(numChildren) );
#endif
                }
            }
        }
        else
            node->traverse(*nv);
    }
};
static osg::ref_ptr<PagedCullingCallback> g_pagedCullingCallback = new PagedCullingCallback;

/* PosterVisitor: A visitor for adding culling callbacks to newly allocated paged nodes */
PosterVisitor::PosterVisitor()
:   osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
    _appliedCount(0), _needToApplyCount(0),
    _addingCallbacks(true)
{
}

void PosterVisitor::apply( osg::LOD& node )
{
    /*if ( !hasCullCallback(node.getCullCallback(), g_lodCullingCallback.get()) )
    {
        if ( !node.getName().empty() )
        {
            PagedNodeNameSet::iterator itr = _pagedNodeNames.find( node.getName() );
            if ( itr!=_pagedNodeNames.end() )
            {
                insertCullCallback( node, g_lodCullingCallback.get() );
                _appliedCount++;
            }
        }
    }
    else if ( !_addingCallbacks )
    {
        node.removeCullCallback( g_lodCullingCallback.get() );
        _appliedCount--;
    }*/
    traverse( node );
}
osg::Vec3 PosterPrinter::unprojectFromScreen( const osg::Vec3 screen, osg::ref_ptr< osg::Camera > camera )
{
    return screen * osg::Matrix::inverse( camera->getViewMatrix() * camera->getProjectionMatrix() * camera->getViewport()->computeWindowMatrix() );
}

osg::Matrix PosterPrinter::getFromScreenMatrix(  osg::ref_ptr< osg::Camera > camera )
{
    return osg::Matrix::inverse( camera->getViewMatrix() * camera->getProjectionMatrix() * camera->getViewport()->computeWindowMatrix() );
}
void PosterVisitor::apply( osg::PagedLOD& node )
{
    if ( !hasCullCallback(node.getCullCallback(), g_pagedCullingCallback.get()) )
    {
        for ( unsigned int i=0; i<node.getNumFileNames(); ++i )
        {
            if ( node.getFileName(i).empty() ) continue;
            
            PagedNodeNameSet::iterator itr = _pagedNodeNames.find( node.getFileName(i) );
            if ( itr!=_pagedNodeNames.end() )
            {
                insertCullCallback( node, g_pagedCullingCallback.get() );
                _appliedCount++;
            }
            break;
        }
    }
    else if ( !_addingCallbacks )
    {
        node.removeCullCallback( g_pagedCullingCallback.get() );
        if ( _appliedCount>0 ) _appliedCount--;
    }
    traverse( node );
}

/* PosterIntersector: A simple polytope intersector for updating pagedLODs in each image-tile */
PosterIntersector::PosterIntersector( const osg::Polytope& polytope )
:   _parent(0), _polytope(polytope)
{}

PosterIntersector::PosterIntersector( double xMin, double yMin, double xMax, double yMax )
:   Intersector(osgUtil::Intersector::PROJECTION),
    _parent(0)
{
    _polytope.add( osg::Plane( 1.0, 0.0, 0.0,-xMin) );
    _polytope.add( osg::Plane(-1.0, 0.0, 0.0, xMax) );
    _polytope.add( osg::Plane( 0.0, 1.0, 0.0,-yMin) );
    _polytope.add( osg::Plane( 0.0,-1.0, 0.0, yMax) );
}

osgUtil::Intersector* PosterIntersector::clone( osgUtil::IntersectionVisitor& iv )
{
    osg::Matrix matrix;
    if ( iv.getProjectionMatrix() ) matrix.preMult( *iv.getProjectionMatrix() );
    if ( iv.getViewMatrix() ) matrix.preMult( *iv.getViewMatrix() );
    if ( iv.getModelMatrix() ) matrix.preMult( *iv.getModelMatrix() );
    
    osg::Polytope transformedPolytope;
    transformedPolytope.setAndTransformProvidingInverse( _polytope, matrix );
    
    osg::ref_ptr<PosterIntersector> pi = new PosterIntersector( transformedPolytope );
    pi->_parent = this;
    return pi.release();
}

void PosterIntersector::reset()
{
    Intersector::reset();
}

void PosterIntersector::intersect( osgUtil::IntersectionVisitor& iv, osg::Drawable* drawable )
{
    if ( !_polytope.contains(drawable->getBound()) ) return;
    if ( iv.getDoDummyTraversal() ) return;
    
    // Find and collect all paged LODs in the node path
    osg::NodePath& nodePath = iv.getNodePath();
    for ( osg::NodePath::iterator itr=nodePath.begin(); itr!=nodePath.end(); ++itr )
    {
        osg::PagedLOD* pagedLOD = dynamic_cast<osg::PagedLOD*>(*itr);
        if ( pagedLOD )
        {
            // FIXME: The first non-empty getFileName() is used as the identity of this paged node.
            // This should work with VPB-generated terrains but maybe unusable with others.
            for ( unsigned int i=0; i<pagedLOD->getNumFileNames(); ++i )
            {
                if ( pagedLOD->getFileName(i).empty() ) continue;
                if ( _parent->_visitor.valid() )
                    _parent->_visitor->insertName( pagedLOD->getFileName(i) );
                break;
            }
            continue;
        }
        
        /*osg::LOD* lod = dynamic_cast<osg::LOD*>(*itr);
        if ( lod )
        {
            if ( !lod->getName().empty() && _parent->_visitor.valid() )
                _parent->_visitor->insertName( lod->getName() );
        }*/
    }
}

/* PosterPrinter: The implementation class of high-res rendering */
PosterPrinter::PosterPrinter()
    :_outputTileExt("bmp"),_isRunning(false), _isFinishing(false), _useDepth(false),
    _outputTiles(false),_lastBindingFrame(0),

    _currentRow(0), _currentColumn(0),
    _camera(0), _finalPoster(0)
{
    _intersector = new PosterIntersector(-1.0, -1.0, 1.0, 1.0);
    _visitor = new PosterVisitor;
    _intersector->setPosterVisitor( _visitor.get() );
}

void PosterPrinter::init( const osg::Camera* camera )
{
    if ( _isRunning || !_camera.valid() )
        return;
    
    _images.clear();
    _depthImages.clear();
    _visitor->clearNames();
    _tileRows = (int)(_posterSize.y() / _tileSize.y());
    _tileColumns = (int)(_posterSize.x() / _tileSize.x());
    _currentRow = 0;
    _currentColumn = 0;
    _currentViewMatrix = camera->getViewMatrix();
    _currentProjectionMatrix = camera->getProjectionMatrix();
    _lastBindingFrame = 0;
    _isRunning = true;
    _isFinishing = false;
}

void PosterPrinter::frame( const osg::FrameStamp* fs, osg::Node* node )
{
    // Add cull callbacks to all existing paged nodes,
    // and advance frame when all callbacks are dispatched.
    if ( addCullCallbacks(fs, node) )
        return;
    
    if ( _isFinishing )
    {
        if ( (fs->getFrameNumber()-_lastBindingFrame)>2 )
        {
            // Record images and the final poster
            recordImages();
			if ( _finalPoster.valid() )
			{
			    std::cout << "Writing final result to file..." << std::endl;
			    osgDB::writeImageFile( *_finalPoster, _outputPosterName );
                            applyGeoTags(_outputPosterName.c_str(),_currentViewMatrix,_currentProjectionMatrix,_camera);

                        }
                        if ( _finalHeightMap.valid() )
                        {
                            std::cout << "Writing final result to file..." << std::endl;
                            osgDB::writeImageFile( *_finalHeightMap, _outputHeightMapName );
                            applyGeoTags(_outputHeightMapName.c_str(),_currentViewMatrix,_currentProjectionMatrix,_camera);

                        }

			// Release all cull callbacks to free unused paged nodes
            removeCullCallbacks( node );
            _visitor->clearNames();
			
			_isFinishing = false;
			std::cout << "Recording images finished." << std::endl;
        }
    }
    
    if ( _isRunning )
    {
        // Every "copy-to-image" process seems to be finished in 2 frames.
        // So record them and dispatch camera to next tiles.
        if ( (fs->getFrameNumber()-_lastBindingFrame)>2 )
        {
            // Record images and unref them to free memory
            recordImages();
            
            // Release all cull callbacks to free unused paged nodes
            removeCullCallbacks( node );
            _visitor->clearNames();
            
            if ( _camera.valid() )
            {
             //   std::cout << "Binding sub-camera " << _currentRow << "_" << _currentColumn
                       //   << " to image..." << std::endl;
                bindCameraToImage( _camera.get(), _currentRow, _currentColumn );
 			printf("\r%d,%d / (%d/%d)",_currentColumn,_currentRow,_tileRows,_tileColumns);
                    fflush(stdout);
                if ( _currentColumn<_tileColumns-1 )
                {
                    _currentColumn++;
                }
                else
                {
                    if ( _currentRow<_tileRows-1 )
                    {
                        _currentRow++;
                        _currentColumn = 0;
                    }
                    else
                    {
                        _isRunning = false;
                        _isFinishing = true;
                        printf("\n");
                    }
                }
            }
            _lastBindingFrame = fs->getFrameNumber();
        }
    }
}
bool PosterPrinter::getClosestZValue(int x,int y,float &z)
{
        osg::ref_ptr<osgUtil::LineSegmentIntersector> picker = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, x,y);
        osgUtil::IntersectionVisitor iv( picker );
        _camera->accept(iv);


    if (picker->containsIntersections()) {
        osgUtil::LineSegmentIntersector::Intersection intersection = picker->getFirstIntersection();
        z=intersection.getLocalIntersectPoint()[2];
        return true;
      }

    return false;

}
bool PosterPrinter::addCullCallbacks( const osg::FrameStamp* fs, osg::Node* node )
{
    if ( !_visitor->inQueue() || done() )
        return false;
    
    _visitor->setAddingCallbacks( true );
    _camera->accept( *_visitor );
    _lastBindingFrame = fs->getFrameNumber();
    
    //std::cout << "Dispatching callbacks to paged nodes... "
      //        << _visitor->inQueue() << std::endl;
    return true;
}

void PosterPrinter::removeCullCallbacks( osg::Node* node )
{
    _visitor->setAddingCallbacks( false );
    _camera->accept( *_visitor );
}

void PosterPrinter::bindCameraToImage( osg::Camera* camera, int row, int col )
{
    std::stringstream stream;
    stream << "image_" << row << "_" << col;
    
    osg::ref_ptr<osg::Image> image = new osg::Image;
    image->setName( stream.str() );
    image->allocateImage( (int)_tileSize.x(), (int)_tileSize.y(), 1, GL_RGBA, GL_UNSIGNED_BYTE );
    _images[TilePosition(row,col)] = image.get();

    osg::ref_ptr<osg::Image> depthImage;
    if(_useDepth){
        depthImage=new osg::Image;;
        depthImage->setName( stream.str() );
        depthImage->allocateImage( (int)_tileSize.x(), (int)_tileSize.y(), 1,GL_DEPTH_COMPONENT, GL_FLOAT );
        _depthImages[TilePosition(row,col)] = depthImage.get();
    }
    // Calculate projection matrix offset of each tile
    osg::Matrix offsetMatrix =
        osg::Matrix::scale(_tileColumns, _tileRows, 1.0) *
        osg::Matrix::translate(_tileColumns-1-2*col, _tileRows-1-2*row, 0.0);
    camera->setViewMatrix( _currentViewMatrix );
    camera->setProjectionMatrix( _currentProjectionMatrix * offsetMatrix );
    
    // Check intersections between the image-tile box and the model
    osgUtil::IntersectionVisitor iv( _intersector.get() );
    iv.setReadCallback( g_pagedLoadingCallback.get() );
    _intersector->reset();
    camera->accept( iv );
    if ( _intersector->containsIntersections() )
    {
        // Apply a cull calback to every paged node obtained, to force the highest level displaying.
        // This will be done by the PosterVisitor, who already records all the paged nodes.
    }
    
    // Reattach cameras and new allocated images
    camera->setRenderingCache( NULL );  // FIXME: Uses for reattaching camera with image, maybe inefficient?
    camera->detach( osg::Camera::COLOR_BUFFER );
    camera->attach( osg::Camera::COLOR_BUFFER, image.get(), 0, 0 );
    if(_useDepth){
        camera->detach(osg::Camera::DEPTH_BUFFER);
        camera->attach(osg::Camera::DEPTH_BUFFER, depthImage.get(),0,0);
        camera->setComputeNearFarMode(osg::Camera::DO_NOT_COMPUTE_NEAR_FAR);
        _depthMats[TilePosition(row,col)]=getFromScreenMatrix(camera);
    }

}

void PosterPrinter::recordImages()
{

    for ( TileImages::iterator itr=_images.begin(); itr!=_images.end(); ++itr )
    {
        osg::Image* image = (itr->second).get();
        if ( _finalPoster.valid() )
        {
            // FIXME: A stupid way to combine tile images to final result. Any better ideas?
            unsigned int row = itr->first.first, col = itr->first.second;
			for ( int t=0; t<image->t(); ++t ) 
			{
				unsigned char* source = image->data( 0, t );
				unsigned char* target = _finalPoster->data( col*(int)_tileSize.x(), t + row*(int)_tileSize.y() );
				memcpy( target, source, image->s() * 4 * sizeof(unsigned char) ); 
			}
        }
        
        if ( _outputTiles )
            osgDB::writeImageFile( *image, image->getName()+"."+_outputTileExt );
    }
    _images.clear();

    if(_useDepth){
        for ( TileImages::iterator itr=_depthImages.begin(); itr!=_depthImages.end(); ++itr )
        {
            osg::Image* image = (itr->second).get();
            if ( _finalHeightMap.valid() ){

                unsigned int row = itr->first.first, col = itr->first.second;
                osg::Matrix screenToWorld=_depthMats[TilePosition(row,col)];
                //std::cout << row << " " << col << " " <<screenToWorld<<std::endl;
                for ( int t=0; t<image->t(); ++t )
                {
                    float* source = (float*)image->data( 0, t );
                    float* target = (float*)_finalHeightMap->data( col*(int)_tileSize.x(), t + row*(int)_tileSize.y() );
                    for(int i=0; i <image->s(); i++){
                        float depth=*(source+i);
                        osg::Vec3 screen(t,i,depth);
                        osg::Vec3 world=(screen*screenToWorld);
                       // if(depth != 1.0)
                        //    printf("V: %f %f %f depth %f\n",world[0],world[1],world[2],depth);
                        *(target+i)=world[1];
                    }
                }
            }
        }

    }
    _depthImages.clear();
}
void Transpose( osg::Matrix& dest, const osg::Matrix& src )
{
   for( int i = 0; i < 4; i++ )
      for( int j = 0; j < 4; j++ )
         dest(i,j) = src(j,i);
}
void PosterPrinter::applyGeoTags(const char* filename,osg::Matrix viewMatrix,osg::Matrix projMatrix,osg::Camera *camera){
    gtiffspace::geo_info pos;
    libplankton::Local_WGS84_TM_Projection *map_projection = new libplankton::Local_WGS84_TM_Projection( _geoOrigin.x(),
                                                                                                         _geoOrigin.y());//glong

    osg::Matrix trans(
            osg::Matrix::rotate(osg::inDegrees(-90.0f),
                                1.0f,0.0f,0.0f)*
            osg::Matrix::rotate(osg::inDegrees(-90.0f),0.0f,
                                1.0f,0.0f));
    osg::Matrix modWindow=osg::Matrix( osg::Matrix::scale(_tileColumns, _tileRows, 1.0) *camera->getViewport()->computeWindowMatrix());
    modWindow(3,0)*=_tileColumns;
    modWindow(3,1)*=_tileRows;
    osg::Matrix bottomLeftToTopLeft= (osg::Matrix::scale(1,-1,1)*osg::Matrix::translate(0,_tileRows*_tileSize.y(),0));
    //std::cout << bottomLeftToTopLeft<<std::endl;
    //std::cout <<modWindow<<std::endl;
    osg::Matrix worldtoScreen=trans*viewMatrix * projMatrix * modWindow*bottomLeftToTopLeft;
    osg::Matrix screenToWorld=osg::Matrix::inverse(worldtoScreen);
    osg::Vec3 tl(0,0,0);
    osg::Vec3 bl(0,_posterSize.y(),0);
    osg::Vec3 tr(_posterSize.x(),0,0);
    osg::Vec3 br(_posterSize.x(),_posterSize.y(),0);

    osg::Vec3 tlGlobal=tl*screenToWorld;
    // cout << "TL Global" << tlGlobal<<endl;
    double latTL,longTL;
    map_projection->calc_geo_coords(tlGlobal.x(),tlGlobal.y(),latTL,longTL);


    osg::Vec3 blGlobal=bl*screenToWorld;
    //cout << "bl Global" << blGlobal<<endl;
    double latbl,longbl;
    map_projection->calc_geo_coords(blGlobal.x(),blGlobal.y(),latbl,longbl);

    osg::Vec3 trGlobal=tr*screenToWorld;
    //cout << "tr Global" << trGlobal<<endl;
    double lattr,longtr;
    map_projection->calc_geo_coords(trGlobal.x(),trGlobal.y(),lattr,longtr);

    osg::Vec3 brGlobal=br*screenToWorld;
    //cout << "br Global" << brGlobal<<endl;
    double latbr,longbr;
    map_projection->calc_geo_coords(brGlobal.x(),brGlobal.y(),latbr,longbr);

    //osg::Vec3 test(1956.17,3937.951,47.6057);
    //double testLat,testLong;
    //map_projection->calc_geo_coords(test.x(),test.y(),testLat,testLong);
    // std::cout << "LAt LONG " <<testLat << " "<< testLong<<"\n";

    /* std::cout << "good " <<test*worldtoScreen<<"\n";
    std::cout << "LAt LONG " <<testLat << " "<< testLong<<"\n";
    cout << "brGlobal " << brGlobal<< " " << tlGlobal << endl;*/
    osg::Vec3 diff=(brGlobal-tlGlobal);
    //cout << "Diff " << diff<<endl;
    osg::Vec2 scalePix( fabs(diff.x()/_posterSize.x()), fabs(diff.y()/_posterSize.y()));
    //cout << "Pix Scale "<<scalePix<<endl;
    UF::GeographicConversions::Redfearn gpsConversion("WGS84","UTM");
    string zone;
    double easting_ctr, northing_ctr;

    double gridConvergence, pointScale;

    gpsConversion.GetGridCoordinates(latTL, longTL
                                     , zone, easting_ctr, northing_ctr, gridConvergence, pointScale);
    pos.UTMZone=zone;
    gtiffspace::tie_point tie;
    tie.pt[0]=tl.x();
    tie.pt[1]=tl.y();
    tie.pt[2]=0;
    tie.pt[3]=easting_ctr;
    tie.pt[4]=northing_ctr;
    tie.pt[5]=0;
    pos.tie_points.push_back(tie);
    pos.pixscale[0]=scalePix.x();
    pos.pixscale[1]=scalePix.y();
    pos.pixscale[2]=0;

    /*std::cout << "world " <<(test*worldtoScreen)<<"Grab Iron\n";
    osg::Matrix transScreen;
    Transpose(transScreen,screenToWorld);
      pos.transform[0] = transScreen(0,0);
    pos.transform[1] = transScreen(0,1);
    pos.transform[2] = 0;
    pos.transform[3] = transScreen(0,3);
    pos.transform[4] = transScreen(1,0);
    pos.transform[5] = transScreen(1,1);
    pos.transform[6] = 0;
    pos.transform[7] = transScreen(1,3);
    pos.transform[8] = 0;
    pos.transform[9] = 0;
    pos.transform[10] = 0;
    pos.transform[11] = 0;
    pos.transform[12] = 0;
    pos.transform[13] = 0;
    pos.transform[14] = 0;
    pos.transform[15] = 1;
    pos.UTMZone = "51S";*/


    /*printf("%f %f %f %f\n",pos.transform[0],pos.transform[1],pos.transform[2],pos.transform[3]);
    printf("%f %f %f %f\n",pos.transform[4],pos.transform[5],pos.transform[6],pos.transform[7]);
    std::cout<<"Screen to World" << screenToWorld << std::endl;
    std::cout<<"Trans Scren" << (transScreen*test) << std::endl;
    std::cout<<"View" << transScreen << std::endl;
    std::cout<<"CAm" << camera->getViewport()->computeWindowMatrix() << std::endl;*/
    gtiffspace::append_gtiff_tags(filename,pos);
    // std::cout<<"Screen to World" << screenToWorlds << std::endl;

}
