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
#include "PosterPrinter.h"
//#include <GL/glu.h>
#include <osg/io_utils>
#include <osg/ComputeBoundsVisitor>
#include <osgShadow/ConvexPolyhedron>
#include <osg/Version>
using std::cout;
using std::endl;
class MyGraphicsContext {
    public:
        MyGraphicsContext()
        {
            osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
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
                traits->pbuffer = false;
                _gc = osg::GraphicsContext::createGraphicsContext(traits.get());
            }

            if (_gc.valid())
            {
                _gc->realize();
                _gc->makeCurrent();
            }
        }

        bool valid() const { return _gc.valid() && _gc->isRealized(); }

    private:
        osg::ref_ptr<osg::GraphicsContext> _gc;
};


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
PosterIntersector::PosterIntersector( const osg::Polytope& polytope,osg::Camera *camera,osg::Matrix &modelMatrix )
:   _parent(0), _camera(camera),_polytope(polytope),_modelMatrix(modelMatrix)
{
}

PosterIntersector::PosterIntersector( double xMin, double yMin, double xMax, double yMax,osg::Camera *camera,osg::Matrix &modelMatrix)
:   Intersector(osgUtil::Intersector::PROJECTION),_camera(camera),
    _parent(0),_modelMatrix(modelMatrix)
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
    
    osg::ref_ptr<PosterIntersector> pi = new PosterIntersector( transformedPolytope,_camera,_modelMatrix );
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
    osg::Matrix matrix;//=osg::computeLocalToWorld(nodePath) *_camera->getViewMatrix() *_camera->getProjectionMatrix();

    if (iv.getWindowMatrix()) matrix.preMult( *iv.getWindowMatrix());
    if (iv.getProjectionMatrix()) matrix.preMult(*iv.getProjectionMatrix() );
    if (iv.getViewMatrix()) matrix.preMult( *iv.getViewMatrix() );
    if (iv.getModelMatrix()) matrix.preMult( *iv.getModelMatrix() );
     if (iv.getModelMatrix())
            _modelMatrix=*iv.getModelMatrix();

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
:   _isRunning(false), _isFinishing(false), _lastBindingFrame(0),
    _outputTiles(true), _tmpTileExt("png"), _outputTileExt("png"),_tileOverlap(1),_tileSize(254,254),
    _currentRow(0), _currentColumn(0),_emptyImage(0),
    _camera(0), _outputEmpty(false),_haveSavedFirstEmpty(false),_baseName("mesh"),_finalPoster(0)
{

_visitor = new PosterVisitor;
}

void PosterPrinter::init( const osg::Camera* camera,std::vector<TilePosition> &valid,osg::BoundingBox bbox,std::pair<unsigned int,unsigned int> empty)
{
    _intersector = new PosterIntersector(-1.0, -1.0, 1.0, 1.0,_camera,_model);
    _intersector->setPosterVisitor( _visitor.get() );
    if ( _isRunning || !_camera.valid() )
        return;
    _images.clear();
    _bbox=bbox;
    _visitor->clearNames();
   _tileRows = (int)ceil(_posterSize.y() / _tileSize.y());
    _tileColumns = (int)ceil(_posterSize.x() / _tileSize.x());
    if(valid.size() && !_outputEmpty){
        _emptyPair =empty;
        _validTiles=valid;
        _validCurrent=0;
        _validMats.resize(valid.size());
        _validBbox.resize(valid.size());

        _currentRow=_validTiles[_validCurrent].second;
        _currentColumn=_validTiles[_validCurrent].first;
    }else{
        _validCurrent=0;
        _currentRow = 0;
        _currentColumn = 0;
	_validMats.resize(_tileRows*_tileColumns);
        _validBbox.resize(_tileRows*_tileColumns);
    }
 
    _bboxMatrix = new osg::BoundingBox *[_tileColumns*_tileRows];
    for(int i=0; i < _tileColumns*_tileRows; i++)
        _bboxMatrix[i]=NULL;
    _currentViewMatrix = camera->getViewMatrix();
    _currentProjectionMatrix = camera->getProjectionMatrix();
    _lastBindingFrame = 0;
    _isRunning = true;
    _isFinishing = false;
    _maxLevel=ceil(log2(std::max(	_posterSize.x(),_posterSize.y())));
    std::stringstream ss;
    ss << "/tmp_"<< _posterSize.x() << "_"<<_tileSize.x()<<"/";
    _tmpbase=ss.str();
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
            if ( _finalPoster )
            {
                std::cout << "Writing final result to file..." << std::endl;
           //     osgDB::writeImageFile( *_finalPoster, _outputPosterName );

                cvSaveImage(_outputPosterName.c_str(),_finalPoster);
            }

            // Release all cull callbacks to free unused paged nodes
            removeCullCallbacks( node );
            _visitor->clearNames();

            _isFinishing = false;
            std::cout << "Recording images finished. Max Level: " << _maxLevel << " Rows: "<< _tileRows << " Cols: "<<_tileColumns<<std::endl;
            if(_outputTiles){

                doDeepZoom();
                writeMats();

            }

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
               // std::cout << "Binding sub-camera " << _currentRow << "_" << _currentColumn
                 //         << " to image..." << std::endl;
                bindCameraToImage( _camera.get(), _currentRow, _currentColumn,_validCurrent );
                if(_validTiles.size() == 0 || _outputEmpty){
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
                            printf("\n");
                            _isRunning = false;
                            _isFinishing = true;
                        }
                    }
                }else{
                    if ( _validCurrent < (int)_validTiles.size()-1){
                        printf("\r%d/%d",_validCurrent+1,(int)_validTiles.size());
                        fflush(stdout);
                        _validCurrent++;

                        _currentRow=_validTiles[_validCurrent].second;
                        _currentColumn=_validTiles[_validCurrent].first;

                    }else if(!_haveSavedFirstEmpty && _emptyPair.second >= 0 && _emptyPair.first >= 0){
                        _haveSavedFirstEmpty=true;
                        _currentRow=_emptyPair.second;
                        _currentColumn=_emptyPair.first;
                        printf("\r%d/%d",(int)_validTiles.size(),(int)_validTiles.size());

                    }else{
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

bool PosterPrinter::addCullCallbacks( const osg::FrameStamp* fs, osg::Node* node )
{
    if ( !_visitor->inQueue() || done() )
        return false;
    
    _visitor->setAddingCallbacks( true );
    _camera->accept( *_visitor );
    _lastBindingFrame = fs->getFrameNumber();
    
    //std::cout << "Dispatching callbacks to paged nodes... "
       //       << _visitor->inQueue() << std::endl;
    return true;
}

void PosterPrinter::removeCullCallbacks( osg::Node* node )
{
    _visitor->setAddingCallbacks( false );
    _camera->accept( *_visitor );
}

void PosterPrinter::bindCameraToImage( osg::Camera* camera, int row, int col, int idx)
{

    // Calculate projection matrix offset of each tile
    osg::Matrix offsetMatrix =
        osg::Matrix::scale(_tileColumns, _tileRows, 1.0) *
             osg::Matrix::translate(_tileColumns-1-2*col, /*flip rows*/-1*(_tileRows-1-2*row), 0.0);
          //    cout << "Level "<< _maxLevel << " Col " << col << " Row : "<<row<<endl;
            //         cout <<              osg::Matrix::translate(_tileColumns-1-2*col, /*flip rows*/-1*(_tileRows-1-2*row), 0.0)<<endl;
    camera->setViewMatrix( _currentViewMatrix );
    camera->setProjectionMatrix( _currentProjectionMatrix * offsetMatrix );
    osg::Polytope  frustum;

    frustum.setToUnitFrustum();
    frustum.transformProvidingInverse(
            camera->getViewMatrix() *
            camera->getProjectionMatrix());

    osgShadow::ConvexPolyhedron shaved;

    // Check intersections between the image-tile box and the model
    osgUtil::IntersectionVisitor iv( _intersector.get() );
    iv.setReadCallback( g_pagedLoadingCallback.get() );
    _intersector->reset();
    camera->accept( iv );
    if(!_outputEmpty){
        if (! _intersector->containsIntersections() || idx <0)
        {
            if(!_outputEmpty && _emptyImage)
            printf("image %d_%d empty but thought to be valid\n",col,row);

            if(!_emptyImage){

                _emptyImage = new osg::Image;
                _emptyImage->allocateImage( (int)_tileSize.x(), (int)_tileSize.y(), 1, GL_RGBA, GL_UNSIGNED_BYTE );
                _emptyImage->setName( "empty" );

                // Reattach cameras and new allocated images
                camera->setRenderingCache( NULL );  // FIXME: Uses for reattaching camera with image, maybe inefficient?
                camera->detach( osg::Camera::COLOR_BUFFER );
                camera->attach( osg::Camera::COLOR_BUFFER, _emptyImage.get(), 0, 0 );
                _haveSavedFirstEmpty=true;
            }
            // Apply a cull calback to every paged node obtained, to force the highest level displaying.
            // This will be done by the PosterVisitor, who already records all the paged nodes.
            return ;
        }
    }

    osg::Matrix matrix;

    matrix.preMult(camera->getProjectionMatrix() );
    matrix.preMult( camera->getViewMatrix() );
    matrix.preMult(_model);
osg::Matrix win=camera->getViewport()->computeWindowMatrix();
    //if(camera->getViewport())
//printf("Neede %f %f\n,",camera->getViewport()->x(),camera->getViewport()->y());
    int offsetL = (col == 0 ? 0 : _tileOverlap);
    int offsetT = (row == 0 ? 0 : _tileOverlap);
    int offsetR = (col == (_tileColumns-1) ? 0 : _tileOverlap);
    int offsetB = (row == (_tileRows-1) ? 0 : _tileOverlap);
    double offsetW=_tileSize.x()+offsetL+offsetR;
    double offsetH=_tileSize.y()+offsetT+offsetB;
  //  printf("offsetW %f offsetH %f\n",offsetW,offsetH);
//    printf("Width Hiehgt %d %d %d %d\n",offsetL,offsetT,offsetR,offsetB);
 //   osg::Matrix win=osg::Matrix::translate(1.0,1.0,1.0)*osg::Matrix::scale(0.5*offsetW,0.5*offsetH,0.5f)*osg::Matrix::translate(camera->getViewport()->x(),camera->getViewport()->y(),0.0f);

//printf("MFFER %f \n",offsetW/_tileSize.x());
    matrix.postMult(win);
    matrix.postMult(osg::Matrix::translate(offsetL,offsetB,0)*osg::Matrix::scale(_tileSize.x()/offsetW,
                                        _tileSize.y()/offsetH,0.0));
//cout <<win <<endl;
/*    double scaledOffsetL=offsetL *((_tileSize.x()+offsetL+offsetR)/(double)_tileSize.x());
        double scaledOffsetB=offsetB *((_tileSize.y()+offsetT+offsetB)/(double)_tileSize.y());*/
//double scaledOffsetL=offsetL;//(_tileSize.x()/(double)(_tileSize.x()+offsetL+offsetR));
  //  double scaledOffsetB=offsetB ;
    //printf("%f CGECJ\n",(_tileSize.y()/(double)(_tileSize.y()+offsetT+offsetB)));
  //  printf("%f CGECJ\n",(_tileSize.x()/(double)(_tileSize.x()+offsetL+offsetR)));

//printf("col %d row %d %d %d %f %f\n",col,row ,offsetL,offsetB,scaledOffsetL,scaledOffsetB);

  //  matrix.postMult(osg::Matrix::translate(scaledOffsetL,
    //                                     scaledOffsetB,0.0));
    //  matrix.postMult(osg::Matrix::translate(20,
       //                                  40,0.0));
osg::Vec3 v1(-39.7 ,45, 31.0);
//cout << "Heter "<< v1*matrix <<endl;
osg::Matrix texScale(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
    texScale(0,0)=1.0/(double)(_tileSize.x());
    texScale(1,1)=1.0/(double)(_tileSize.y());
    matrix.postMult(texScale);
 // osg::Vec3 v1(-133.381,74.1,15.0319);
 //  osg::Vec3 v1(-7.381,-8.0,26.0319);

   // cout << texScale<<endl;
   // cout <<matrix<<endl;
/*if(!(row == 0 && col == 0)){
matrix=osg::Matrix::identity();
printf("ADASDAS\n");
}*/
//    cout << "current "<<idx<<endl;
    _validMats[idx]=matrix;
//cout << matrix<<endl;
    /* cout <<"Min "<< minV << " Max "<< maxV <<endl;
cout << "In World "<<_bbox.contains(vt)<<endl;*/
    shaved.setToBoundingBox(_bbox);
    shaved.cut( frustum );
    osg::BoundingBox bb2 =shaved.computeBoundingBox();
    bb2._min=_model*bb2._min;
    bb2._max=_model*bb2._max;
    //cout <<"BBS Min "<< bb2._min << " Max "<< bb2._max <<endl;
    _bboxMatrix[col * _tileColumns + row]=new osg::BoundingBox(bb2);
    //printf ("ADDED %d\n",col * _tileColumns + row);
    _validBbox[idx]=bb2;

    //  cout << "Ouput pixel "<<vt*matrix<< "IDX "<<idx <<"Working " <<    bb2.contains(vt)<<endl;

    //printf("Has %d %d valid node\n",row,col);
    std::stringstream stream;
    stream << "image_" << col << "_" << row;

    osg::ref_ptr<osg::Image> image = new osg::Image;
    image->setName( stream.str() );
    image->allocateImage( (int)_tileSize.x(), (int)_tileSize.y(), 1, GL_RGBA, GL_UNSIGNED_BYTE );
    _images[TilePosition(row,col)] = image.get();


    // Reattach cameras and new allocated images
    camera->setRenderingCache( NULL );  // FIXME: Uses for reattaching camera with image, maybe inefficient?
    camera->detach( osg::Camera::COLOR_BUFFER );
    camera->attach( osg::Camera::COLOR_BUFFER, image.get(), 0, 0 );
}
void PosterPrinter::recordImages()
{
    MyGraphicsContext gc;

    for ( TileImages::iterator itr=_images.begin(); itr!=_images.end(); ++itr )
    {
        osg::Image* image = (itr->second).get();
        unsigned int row = itr->first.first, col = itr->first.second;

        if ( _outputTiles ){

            std::stringstream ss;
            ss<<_dir<<"/"<<_baseName<<"_files/";

            osgDB::makeDirectory(ss.str());
            ss << _maxLevel;
            osgDB::makeDirectory(ss.str());


            std::stringstream ssquare;
            ssquare<<_dir<<"/tmpsquare/";

            osgDB::makeDirectory(ssquare.str());
            ssquare << _maxLevel;
            osgDB::makeDirectory(ssquare.str());
            //ss<<"/"<<col<<"_"<<(row) <<"."<<_tmpTileExt;

            std::stringstream  tmpss;
            tmpss<<_dir<<"/"<<_tmpbase<<_baseName<<"_files/"<<_maxLevel;
            osgDB::makeDirectory(tmpss.str());

            tmpss<<"/"<<col<<"_"<<(row) <<"."<<_tmpTileExt;
            osgDB::writeImageFile( *image,  tmpss.str());
            // _emptyImage =osgDB::readImageFile("/home/mattjr/pattern.png");// new osg::Image;
            // _emptyImage->allocateImage( (int)_tileSize.x(), (int)_tileSize.y(), 1, GL_RGBA, GL_UNSIGNED_BYTE );
            //      osgDB::writeImageFile( *_emptyImage,  ss.str());

            //cout << "Writing "<<ss.str();
        }
        if(_finalPoster && _finalPoster->imageData){
            int w=_tileSize.x();
            int h=_tileSize.y();

            int x = col * _tileSize.x() ;
            int y = row * _tileSize.y() ;
            if (x + w > _posterSize.x())
                w = _posterSize.x() - x;
            if (y + h > _posterSize.y())
                h = _posterSize.y() - y;
            image->flipVertical();
            IplImage *localImage=  cvCreateImage(cvSize(_tileSize.x(),_tileSize.y()),IPL_DEPTH_8U,4);
	    memcpy(localImage->imageData,image->data(),image->getImageSizeInBytes());
	    /*  int cnt=0;
          for(y=0; y<localImage->height; y++){
		for(x=0; x<localImage->width; x++){
		  CV_IMAGE_ELEM(localImage,uchar,y,x*4)=255;
		  CV_IMAGE_ELEM(localImage,uchar,y,x*4+1)=0;
		  CV_IMAGE_ELEM(localImage,uchar,y,x*4+2)=0;
		  CV_IMAGE_ELEM(localImage,uchar,y,x*4+3)=255;

		}
		}*/
    IplImage *localImage2=  cvCreateImage(cvSize(_tileSize.x(),_tileSize.y()),IPL_DEPTH_8U,3);
IplImage *localImage3=  cvCreateImage(cvSize(_tileSize.x(),_tileSize.y()),IPL_DEPTH_8U,1);
   cvCvtColor(localImage,localImage2,CV_BGRA2BGR);
   cvCvtColor(localImage2,localImage2,CV_BGR2RGB);
   
   cvSetImageCOI(localImage,4);
   cvCopy(localImage,localImage3);
   cvCvtColor(localImage2,localImage,CV_BGR2BGRA);
   cvSetImageCOI(localImage,4);
   cvCopy(localImage3,localImage);
   cvSetImageCOI(localImage,0);
            CvRect srcrect = cvRect(0, 0, w ,h);
            CvRect dstrect = cvRect(x, y, w, h);
            cvSetImageROI(localImage,srcrect);
            cvSetImageROI(_finalPoster,dstrect);
            cvCopy(localImage,_finalPoster);

            cvReleaseImage(&localImage);
            cvReleaseImage(&localImage2);
            cvReleaseImage(&localImage3);

            cvResetImageROI(_finalPoster);

        }
        

    }
    _images.clear();
}
osg::BoundingBox PosterPrinter::computeBoundingBox(int level, int col,int row){
    osg::BoundingBox *parent=NULL;
    osg::BoundingBox includeAll;
    bool setAny=false;
    int mult=_maxLevel-level;
int range=(int)pow(2,mult);
    for(int i=0; i<range; i++){
        for(int j=0; j<range; j++){
        //  printf("range %d Col %d Row %d , %d %d\n",range,col,row,col*range,row*range);
            int idx=(((col*range)+i) * _tileColumns) + ((row*range)+j);
            if(((col*range)+i) < _tileColumns &&  ((row*range)+j) < _tileRows)
                parent=_bboxMatrix[idx];
            else
                parent=NULL;

            if(parent){
               // printf(" %d_%d include %d_%d base col %d base row %d\n",col,row,(col*range)+i,((row*range)+j),(row*range),col*range);
                if(!setAny){
                    includeAll.set(parent->_min,parent->_max);
                    setAny=true;

                }else{
                    includeAll.expandBy(*parent);

                }
            }
        }
    }
//cout << "Computing bounding box col: " << col << " row: " << row << " level: " << level <<endl;
     //       cout << "Using: col: "<< saved_i << " row: "<< saved_j <<endl;
    if(setAny){
          //  cout << "orig: " << parent->_min << " -- " << parent->_max << "\nnow : " << includeAll._min << " -- "<<includeAll._max <<endl;
        return includeAll;
    }else{
     //  fprintf(stdout,"Not found %d + 1 %d +1\n",row*(2*mult),col*(2*mult));
          // printf ("ADDED %d 0x%x\n",saved_i * _tileColumns + saved_j, _bboxMatrix[saved_i * _tileColumns + saved_j]);

    }
return osg::BoundingBox(0,0,0,0,0,0);
}
void PosterPrinter::writeMats(){
    const int lodSkip[]={0,2,2};
    double width = _posterSize.x();
    double height = _posterSize.y();
    int level=_maxLevel;
    for(int lod=0; lod< 3; lod++){
        std::stringstream bname;
        bname << "re-bbox-"<<lod <<".txt";

        std::string fname=(_dir+"/"+bname.str());
        std::ofstream mfile(fname.c_str(),std::ios_base::trunc);
        if(!mfile.is_open()){
            std::cerr<< "Couldn't open meshes txt  "<<fname<<std::endl;
            return;
        }

        if(lod == 0){
            for(unsigned int i=0; i<_validTiles.size(); i++){
                mfile<< i << " ";
                int row=_validTiles[i].second;
                int col=_validTiles[i].first;
                //  std::stringstream ss;

                //ss<<col<<"_"<<row <<"."<<_tmpTileExt;
                std::stringstream ss;
                ss<<"/tmpsquare/"<<level<<"/s";
                ss<<col<<"_"<<row <<"."<<_outputTileExt;
                //   ss<<_tmpbase<<_baseName<<"_files/"<<_maxLevel<<"/"<<col<<"_"<<row <<"."<<_tmpTileExt;

                mfile << ss.str() << " ";
                mfile << _validBbox[i]._min[0] << " " << _validBbox[i]._min[1] << " "<<_validBbox[i]._min[2] << " ";
                mfile << _validBbox[i]._max[0] << " " << _validBbox[i]._max[1] << " "<<_validBbox[i]._max[2] << " ";

                //cout << _validMats[i]<<endl;
                for(int j=0; j < 4; j++){
                    for(int k=0; k <4; k++){
                        mfile << _validMats[i](j,k) << " ";
                    }
                }
                mfile<<std::endl;

            }
        }else{

            for(int i=0; i<lodSkip[lod] ; i++){
                width/=2.0;
                height/=2.0;
                level--;
            }

//printf("New Level %d %f %f\n",level,width,height);
            int cnt=0;

            int nCols = (int)ceil(width / _tileSize.x());
            int nRows = (int)ceil(height / _tileSize.y());
            double dCols=width / _tileSize.x();
            double dRows =height / _tileSize.y();
            for (int col = 0; col < nCols; col++) {
                for (int row = 0; row < nRows; row++) {
                    osg::BoundingBox bbox=computeBoundingBox(level,col,row);
                    if(bbox.radius() == 0)
                        continue;

                    mfile<< cnt++ << " ";
                    std::stringstream ss;
                    ss<<"/tmpsquare/"<<level<<"/s";
                    ss<<col<<"_"<<row <<"."<<_outputTileExt;
                    mfile << ss.str() << " ";
                    mfile << bbox._min[0] << " " << bbox._min[1] << " "<<bbox._min[2] << " ";
                    mfile << bbox._max[0] << " " << bbox._max[1] << " "<<bbox._max[2] << " ";
                    osg::Matrix matrix;
                     osg::Matrix offsetMatrix =
                             osg::Matrix::scale(dCols, dRows, 1.0) *
                             osg::Matrix::translate(dCols-1-2*col, /*flip rows*/-1*(dRows-1-2*row), 0.0);

//printf("Level %d trans %d %d\n",level,nCols-1-2*col,-1*(nRows-1-2*row));
                  //   cout << "Level "<< level << " Col " << col << " Row : "<<row<<endl;
                    // cout << osg::Matrix::translate(nCols-1-2*col, /*flip rows*/-1*(nRows-1-2*row),0.0)<<endl;
                     matrix.preMult(_currentProjectionMatrix * offsetMatrix );
                     matrix.preMult( _currentViewMatrix );
                     matrix.preMult(_model);
                     osg::Matrix win=_camera->getViewport()->computeWindowMatrix();

                     int offsetL = (col == 0 ? 0 : _tileOverlap);
                     int offsetT = (row == 0 ? 0 : _tileOverlap);
                     int offsetR = (col == (nCols-1) ? 0 : _tileOverlap);
                     int offsetB = (row == (nRows-1) ? 0 : _tileOverlap);
                     double offsetW=(_tileSize.x())+offsetL+offsetR;
                     double offsetH=(_tileSize.y())+offsetT+offsetB;
                     double w = _tileSize.x() + offsetL + offsetR;
                     double h = _tileSize.y() + offsetT + offsetB;


                     int x = col * _tileSize.x() - (col == 0 ? 0 : _tileOverlap);
                     int y = row * _tileSize.y() - ((nRows-1) == 0 ? 0 : _tileOverlap);
                     if (x + w > width)
                         w = width - x;
                     if (y + h > height)
                         h = height - y;

                     matrix.postMult(win);


                     //Top left vs. Bottom left scaling must flip y coord orgin to get correct scaling
                     matrix.postMult(osg::Matrix::scale(1.0,-1.0,1.0));
                     matrix.postMult(osg::Matrix::translate(0,_tileSize.y(),0));
                     matrix.postMult(osg::Matrix::translate(offsetL,offsetT,0)*osg::Matrix::scale(_tileSize.x()/w,
                                                                                                  _tileSize.y()/h,1.0));
                     //Flip back
                     matrix.postMult(osg::Matrix::scale(1.0,-1.0,1.0));
                     matrix.postMult(osg::Matrix::translate(0,_tileSize.y(),0));

                     osg::Matrix texScale(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
                     texScale(0,0)=1.0/(double)(_tileSize.x());
                     texScale(1,1)=1.0/(double)(_tileSize.y());
                     matrix.postMult(texScale);

                     //cout << _validMats[i]<<endl;
                     for(int j=0; j < 4; j++){
                         for(int k=0; k <4; k++){
                             mfile << matrix(j,k) << " ";
                         }
                     }
                     mfile<<std::endl;
                }
            }

        }
        mfile.close();
    }
}

void PosterPrinter::copyNeigborPixels(IplImage *img_overlap,int level, int col, int row,CvRect &srcrect,CvRect &dstrect){
    std::stringstream L;
    L<<_dir<<"/"<<_tmpbase<<_baseName<<"_files/"<<level<<"/"<<col<<"_"<<row<<"."<<_tmpTileExt;
    IplImage *imgsrc =NULL;
    bool exists=osgDB::fileExists(L.str());
    cvSetImageROI(img_overlap, dstrect);

    if(exists){
        imgsrc = cvLoadImage(L.str().c_str(), -1);
        cvSetImageROI(imgsrc, srcrect);
        cvCopy(imgsrc,img_overlap);
        cvReleaseImage(&imgsrc);

    }else if(!_outputEmpty){
        unsigned char color[3];
        memset(&color[0],255,3*sizeof(unsigned char));
        if(_emptyImage && _emptyImage->s() && _emptyImage->t()){
            unsigned char *ptr=_emptyImage->data(0,0);
            //Get osg/opengl bgr and set rgb opencv
            color[0]= *(ptr+2);
            color[1]= *(ptr+1);
            color[2]= *(ptr);
        }else{
            fprintf(stderr,"Empty image null\n");
        }

        cvSet(img_overlap,cvScalar(color[0],color[1],color[2]));
   //     printf("using empty image for %d %d level %d\n",col,row,level);
    }else{
    printf("Can't find %s\n",L.str().c_str());

    }
}
void PosterPrinter::addOverlap(int level,int row, int col,double width,double height){
    int levelColumns = (int)ceil(width / _tileSize.x());
    int levelRows = (int)ceil(height / _tileSize.y());
    std::stringstream origName;
    origName<<_dir<<"/"<<_tmpbase<<_baseName<<"_files/"<<level<<"/"<<col<<"_"<<row<<"."<<_tmpTileExt;
    if(!osgDB::fileExists(origName.str())){
         // cout << "Doesnt " << origName.str()<<endl;
        return;
    }



    int offsetL = (col == 0 ? 0 : _tileOverlap);
    int offsetT = (row == 0 ? 0 : _tileOverlap);

    int offsetR = (col == (levelColumns-1) ? 0 : _tileOverlap);
    int offsetB = (row == (levelRows-1) ? 0 : _tileOverlap);

    int offsetTL = ((offsetL == 0 || offsetT == 0) ? 0 : _tileOverlap);
    int offsetTR = ((offsetR == 0 || offsetT == 0) ? 0 : _tileOverlap);

    int offsetBL = ((offsetL == 0 || offsetB == 0) ? 0 : _tileOverlap);
    int offsetBR = ((offsetR == 0 || offsetB == 0) ? 0 : _tileOverlap);


    //printf("col  %d row %d L %d R %d T %d B%d TL %d TR %d BL %d BR %d\n",col,row,offsetL,offsetR,offsetT,offsetB,
    //offsetTL,offsetTR,offsetBL,offsetBR);

   int w = _tileSize.x() + offsetL + offsetR;
    int h = _tileSize.y() + offsetT + offsetB;
 //      int w = _tileSize.x() + (col == 0 ? 1 : 2) * _tileOverlap;
   //     int h = _tileSize.y() + (row == 0 ? 1 : 2) * _tileOverlap;


    int x = col * _tileSize.x() - (col == 0 ? 0 : _tileOverlap);
    int y = row * _tileSize.y() - (row == 0 ? 0 : _tileOverlap);
    if (x + w > width)
        w = width - x;
    if (y + h > height)
        h = height - y;
//printf("getTile: row=%d, col=%d, x=%d, y=%d, w=%d, h=%d\n",row,col,x,y,w,h);

//    cout << "Adding offset level" << level<< " " << origName.str() << " h " << h << " w " <<w<<endl;
    IplImage *img1 = cvLoadImage(origName.str().c_str(), -1);
     IplImage *img_overlap = cvCreateImage(cvSize(w,h),IPL_DEPTH_8U,3);
    cvSet(img_overlap,cvScalar(rand()%255,rand()%255,rand()%255));

    if(offsetL > 0 ){
        int pullh=std::min((int)_tileSize.y(),h-offsetT);

        CvRect srcrect = cvRect(_tileSize.x()-offsetL, 0, offsetL, pullh);
        CvRect dstrect = cvRect(0, offsetT, offsetL, pullh);
        copyNeigborPixels(img_overlap,level,col-1,row,srcrect,dstrect);
    }
    if(offsetR > 0 ){
        int pullh=std::min((int)_tileSize.y(),h-offsetT);
        CvRect srcrect = cvRect(0, 0, offsetR, pullh);
        CvRect dstrect = cvRect(img_overlap->width-offsetR, offsetT, offsetR, pullh);
        copyNeigborPixels(img_overlap,level,col+1,row,srcrect,dstrect);
    }
    if(offsetT > 0 ){
        int pullw=std::min((int)_tileSize.x(),w-offsetL);

        CvRect srcrect = cvRect(0, _tileSize.x()-offsetT, pullw, offsetT);
        CvRect dstrect = cvRect(offsetL, 0, pullw, offsetT);
        copyNeigborPixels(img_overlap,level,col,row-1,srcrect,dstrect);
    }
    if(offsetB > 0 ){
                int pullw=std::min((int)_tileSize.y(),w-offsetL);

        CvRect srcrect = cvRect(0,0 , pullw, offsetB);
        CvRect dstrect = cvRect(offsetL, img_overlap->height-offsetB, pullw, offsetB);
        copyNeigborPixels(img_overlap,level,col,row+1,srcrect,dstrect);
    }

    if(offsetBL > 0 ){
        CvRect srcrect = cvRect(_tileSize.y()-offsetBL,0 , offsetBL, offsetBL);
        CvRect dstrect = cvRect(0, img_overlap->height-offsetBL, offsetBL, offsetBL);
        copyNeigborPixels(img_overlap,level,col-1,row+1,srcrect,dstrect);
    }
    if(offsetBR > 0 ){
        CvRect srcrect = cvRect(0,0 , offsetBR, offsetBR);
        CvRect dstrect = cvRect(img_overlap->width-offsetBR, img_overlap->height-offsetBR, offsetBR, offsetBR);
        copyNeigborPixels(img_overlap,level,col+1,row+1,srcrect,dstrect);
    }
    if(offsetTL > 0 ){
        CvRect srcrect = cvRect(_tileSize.y()-offsetTL,_tileSize.x()-offsetTL , offsetTL, offsetTL);
        CvRect dstrect = cvRect(0, 0, offsetTL, offsetTL);
        copyNeigborPixels(img_overlap,level,col-1,row-1,srcrect,dstrect);
    }
    if(offsetTR > 0 ){
        CvRect srcrect = cvRect(0,_tileSize.x()-offsetTR , offsetTR, offsetTR);
        CvRect dstrect = cvRect(img_overlap->width-offsetTR, 0, offsetTR, offsetTR);
        copyNeigborPixels(img_overlap,level,col+1,row-1,srcrect,dstrect);
    }

    std::stringstream newName;
    newName<<_dir<<"/"<<_baseName<<"_files/"<<level<<"/"<<col<<"_"<<row<<"."<<_outputTileExt;

    std::stringstream nameSquare;
    nameSquare<<_dir<<"/tmpsquare/"<<level<<"/s"<<col<<"_"<<row<<"."<<_outputTileExt;
    int pasteH=std::min((int)_tileSize.y(),h-offsetT);

        int pasteW=std::min((int)_tileSize.x(),w-offsetL);

    CvRect srcrect = cvRect(0, 0, pasteW,pasteH);
    CvRect dstrect = cvRect(offsetL, offsetT, pasteW,pasteH);
    cvSetImageROI(img1, srcrect);
    cvSetImageROI(img_overlap, dstrect);
    cvCopy(img1,img_overlap);
    cvResetImageROI(img_overlap);
    cvResetImageROI(img1);
    cvSaveImage(newName.str().c_str(),img_overlap);
         IplImage *img_square= cvCreateImage(cvSize(_tileSize.x(),_tileSize.y()),IPL_DEPTH_8U,3);
cvResize(img_overlap,img_square);
    cvSaveImage(nameSquare.str().c_str(),img_square);

    cvReleaseImage(&img_square);

    cvReleaseImage(&img_overlap);
    cvReleaseImage(&img1);
   // cout << newName.str() <<endl;
}

void PosterPrinter::doDeepZoom(){
    MyGraphicsContext gc;
    double width = _posterSize.x();
    double height = _posterSize.y();
     _tileSizeLevel=-1;

    int nCols = (int)ceil(width / _tileSize.x());
    int nRows = (int)ceil(height / _tileSize.y());
    if(_validTiles.size()){
        for(int i=0; i<(int)_validTiles.size();i++){
            addOverlap(_maxLevel,_validTiles[i].second,_validTiles[i].first,width,height);
        }
    }else{
        for (int col = 0; col < nCols; col++) {
            for (int row = 0; row < nRows; row++) {
            //    printf("call addoverlap %d %d\n",row,col);
                addOverlap(_maxLevel,row,col,width,height);
            }
        }
    }
    width/=2.0;
    height/=2.0;
    for (int level = _maxLevel-1; level >= 0; level--) {
        nCols = (int)ceil(width / _tileSize.x());
        nRows = (int)ceil(height / _tileSize.y());
        cout << "Level " << level << " rows: " << nRows << " cols: " << nCols<< " processing.. \n";
        std::stringstream ssquare;
        ssquare<<_dir<<"/tmpsquare/"<<level;
        osgDB::makeDirectory(ssquare.str());

        std::stringstream ss;
        ss<<_dir<<"/"<<_baseName<<"_files/"<<level;
        osgDB::makeDirectory(ss.str());


        std::stringstream tmpss;
        tmpss<<_dir<<"/"<<_tmpbase<<_baseName<<"_files/"<<level;
        osgDB::makeDirectory(tmpss.str());

         for (int col = 0; col < nCols; col++) {
            for (int row = 0; row < nRows; row++) {
                //Combine four smaller images into this one
                int tParents=(int)std::min(ceil(width*2.0 / _tileSize.x()),2.0);
                int sParents= (int)std::min(ceil(height*2.0 / _tileSize.y()),2.0);
                int parentWidth=_tileSize.x();//(int)std::min((int)_tileSize.x()),(int)width);
                int parentHeight=_tileSize.y();//(int)std::min((int)_tileSize.y(),(int)height);

                IplImage *parentImage=cvCreateImage(cvSize(parentWidth,parentHeight),IPL_DEPTH_8U,3);//parentImage->allocateImage(parentWidth,parentHeight,1,GL_RGBA,GL_UNSIGNED_BYTE);
                IplImage *downSized=cvCreateImage(cvSize(parentWidth/(float)tParents,parentHeight/(float)sParents),IPL_DEPTH_8U,3);
                bool validForOutput=_outputEmpty;
             //   printf("Width %f PArnet Width %d downWidth %d\n",width,parentWidth,downSized->width);
                cvZero(parentImage);
                for(int t=0; t<tParents; t++){
                    for(int s=0; s<sParents; s++){
                        std::stringstream ssmall;
                        ssmall<<_dir<<"/"<<_tmpbase<<_baseName<<"_files/"<<(level+1)<<"/"<<((col*2)+t)<<"_"<<((row*2)+s)<<"."<<_tmpTileExt;
                        //osg::ref_ptr<osg::Image> image;
                        IplImage *image=NULL;
                        unsigned char color[3];
                        memset(&color[0],255,3*sizeof(unsigned char));
                        int down_w = downSized->width;
                        int down_h = downSized->height;
                        int x = t*(parentImage->width/tParents);
                        int y = s*(parentImage->height/sParents);
                        if (x + down_w > width)
                            down_w = width - x;
                        if (y + down_h > height)
                            down_h = height - y;
                        bool thisImageHere=false;
                        if(osgDB::fileExists(ssmall.str())){
                            validForOutput=true;
                            thisImageHere=true;
                            //image = osgDB::readImageFile(ssmall.str());
                            image=cvLoadImage(ssmall.str().c_str(),-1);
                            if(!image){
                                std::cout <<"Can't open corrupt image or loader doesn't work"<<ssmall.str()<<std::endl;
                                exit(-1);
                            }
                        }else{
                            if(_outputEmpty){
                                int cc=ceil(width*2.0 / _tileSize.x());
                                int rc=ceil(height*2.0 / _tileSize.y());
                                if(((col*2)+t) < cc && ((row*2)+s) < cc){
                                std::cout <<"Can't open "<<((col*2)+t)<<"_"<<((row*2)+s)<< "Should be there columns above " <<cc <<" rows above "<< rc<<" "<<ssmall.str()<<std::endl;
                                exit(-1);
                            }
                            }
                         if(_emptyImage && _emptyImage->s() && _emptyImage->t()){
                                unsigned char *ptr=_emptyImage->data(0,0);
                                //Get osg/opengl bgr and set rgb opencv
                                color[0]= *(ptr+2);
                                color[1]= *(ptr+1);
                                color[2]= *(ptr);
                            }
                        }

                        CvRect srcrect = cvRect(0,0, down_w,down_h);
                        CvRect dstrect = cvRect(x,y, down_w,down_h);
                        cvSetImageROI(parentImage,dstrect);
                        //printf("Copyind x %d y %d w %d h %d\n",x,y,down_w,down_h);
                        // printf("Level %d Tparents %d/%d Sparents %d/%d parent size %d %d child offsset %d %d \n",
                        //    level,tParents,t,sParents,s,parentImage->width,parentImage->height,t*(parentImage->width/tParents),s*(parentImage->height/sParents));
                        if(image){

                            cvResize(image,downSized);

                            cvSetImageROI(downSized,srcrect);

                            cvCopy(downSized,parentImage);
                            cvResetImageROI(downSized);

                        }else{
                            cvSet(parentImage,cvScalar(color[0],color[1],color[2]));
                        }

                        cvResetImageROI(parentImage);


                    }
                }
                if(validForOutput){
                    std::stringstream par_ss;
                    par_ss<<_dir<<"/"<<_tmpbase<<_baseName<<"_files/"<<level<<"/"<<col<<"_"<<row <<"."<<_tmpTileExt;
                    //osgDB::writeImageFile( *parentImage,  par_ss.str());
                    cvSaveImage(  par_ss.str().c_str(),parentImage);
                    //cout << "Writing out level " << level << "h " << parentImage->s() << " w" <<parentImage->t()<< par_ss.str()<<endl;
                }
                cvReleaseImage(&parentImage);
                cvReleaseImage(&downSized);
            }
        }

        //Add overlap for level just completed if needed
        for (int col = 0; col < nCols; col++) {
            for (int row = 0; row < nRows; row++) {
                    addOverlap(level,row,col,width,height);
                }
            }

        if(width <= _tileSize.x() && height <= _tileSize.y()){
            _tileSizeLevel=level;
            break;
        }
        // Scale down image for next level
        width = ceil(width / 2);
        height = ceil(height / 2);
    }
    printf("Exit level %d\n",_tileSizeLevel);
    for (int level = _tileSizeLevel; level >= 0; level--) {
        std::stringstream origName;
        origName<<_dir<<"/"<<_tmpbase<<_baseName<<"_files/"<<level<<"/"<<0<<"_"<<0<<"."<<_tmpTileExt;
        std::stringstream newName;
        newName<<_dir<<"/"<<_baseName<<"_files/"<<level;
        osgDB::makeDirectory(newName.str());
        newName<<"/"<<0<<"_"<<0<<"."<<_outputTileExt;
        IplImage *output_image=cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
        IplImage *imgsrc = cvLoadImage(origName.str().c_str(), -1);
        if(level==_tileSizeLevel){
            CvRect srcrect = cvRect(0, 0, width,height);
            cvSetImageROI(imgsrc,srcrect);
        }
        cvResize(imgsrc,output_image);
        if(level > 0){
            std::stringstream newTmp;
            newTmp<<_dir<<"/"<<_tmpbase<<_baseName<<"_files/"<<level-1;
        osgDB::makeDirectory(newTmp.str());
            newTmp<<"/"<<0<<"_"<<0<<"."<<_tmpTileExt;
            cvSaveImage(newTmp.str().c_str(),output_image);
        }
        cvSaveImage(newName.str().c_str(),output_image);
        cvReleaseImage(&imgsrc);
         width = ceil(width / 2);
        height = ceil(height / 2);
    }
    RecordXML();
}
void PosterPrinter::RecordXML(){
    std::string xmlHeader = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
    std::string schemaName = "http://schemas.microsoft.com/deepzoom/2009";
    std::string fname=(_dir+"/"+_baseName+".xml");
    std::ofstream dzifile(fname.c_str());
    if(!dzifile.is_open()){
        std::cerr<< "Couldn't open dzi "<<fname<<std::endl;
        return;
    }
    dzifile <<xmlHeader;
    dzifile <<"<Image TileSize=\"" << _tileSize.x() << "\" Overlap=\"" << _tileOverlap <<
            "\" Format=\"" << _outputTileExt << "\" ServerFormat=\"Default\" xmnls=\"" <<
            schemaName << "\">" <<std::endl;
    dzifile << "<Size Width=\"" << _posterSize.x() << "\" Height=\"" << _posterSize.y() << "\" />"<<std::endl;
    dzifile << "</Image>"<<std::endl;
    dzifile.close();
}
