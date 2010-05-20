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
#include <GL/glu.h>
#include <osg/io_utils>
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
            if ( !pagedLOD->getDisableExternalChildrenPaging() &&
                 nv->getDatabaseRequestHandler() &&
                 numChildren<pagedLOD->getNumRanges() )
            {
                if ( pagedLOD->getDatabasePath().empty() )
                {
                    nv->getDatabaseRequestHandler()->requestNodeFile(
                        pagedLOD->getFileName(numChildren), pagedLOD,
                        1.0, nv->getFrameStamp(),
                        pagedLOD->getDatabaseRequest(numChildren), pagedLOD->getDatabaseOptions() );
                }
                else
                {
                    nv->getDatabaseRequestHandler()->requestNodeFile(
                        pagedLOD->getDatabasePath()+pagedLOD->getFileName(numChildren), pagedLOD,
                        1.0, nv->getFrameStamp(),
                        pagedLOD->getDatabaseRequest(numChildren), pagedLOD->getDatabaseOptions() );
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
:   _parent(0), _polytope(polytope),_camera(camera),_modelMatrix(modelMatrix)
{printf("RESET\n");
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
    cout << "GOOD "<<matrix;
    cout <<"needed " << *iv.getModelMatrix()<<endl;
    cout <<"WHY "<< _modelMatrix;
/*    osg::Matrix inverse;
  inverse.invert(matrix);

      osg::Vec3 v1( -133.401 ,72.6815 ,14.4479);

      cout <<     "SDASD"<<v1 << " "<<v1*matrix<<endl;

      cout <<     "SDASD"<<v1 << " "<<v1*inverse<<endl;

      cout <<     "SDASD"<<v1 << " "<<inverse*v1<<endl;
*/

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
    _outputTiles(true), _tmpTileExt("png"), _outputTileExt("jpg"),_tileOverlap(1),_tileSize(254,254),
    _currentRow(0), _currentColumn(0),
    _camera(0), _finalPoster(0),_baseName("mesh"),_outputEmpty(false),_haveSavedFirstEmpty(false)
{

_visitor = new PosterVisitor;
}

void PosterPrinter::init( const osg::Camera* camera,std::vector<TilePosition> &valid,osg::Matrix model )
{
        _intersector = new PosterIntersector(-1.0, -1.0, 1.0, 1.0,_camera,_model);
//_model=model;
    _intersector->setPosterVisitor( _visitor.get() );
    if ( _isRunning || !_camera.valid() )
        return;
    _images.clear();
    _visitor->clearNames();
    if(valid.size() && !_outputEmpty){
        _validTiles=valid;
        _validCurrent=0;
        _validMats.resize(valid.size());
        _validBbox.resize(valid.size());

        _currentRow=_validTiles[_validCurrent].second;
        _currentColumn=_validTiles[_validCurrent].first;
        printf("Here %d %d\n",_currentRow,_currentColumn);
    }else{
        _currentRow = 0;
        _currentColumn = 0;
    }
    _tileRows = (int)ceil(_posterSize.y() / _tileSize.y());
    _tileColumns = (int)ceil(_posterSize.x() / _tileSize.x());

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
            std::cout << "Recording images finished." << std::endl;
            if(_outputTiles)
                writeMats();

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
                std::cout << "Binding sub-camera " << _currentRow << "_" << _currentColumn
                          << " to image..." << std::endl;
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
                    if ( _validCurrent < _validTiles.size()-1){
                        printf("\r%d/%d",_validCurrent,(int)_validTiles.size());
                        fflush(stdout);
                        _validCurrent++;

                        _currentRow=_validTiles[_validCurrent].second;
                        _currentColumn=_validTiles[_validCurrent].first;

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

void PosterPrinter::bindCameraToImage( osg::Camera* camera, int row, int col, int idx )
{

    // Calculate projection matrix offset of each tile
    osg::Matrix offsetMatrix =
        osg::Matrix::scale(_tileColumns, _tileRows, 1.0) *
             osg::Matrix::translate(_tileColumns-1-2*col, /*flip rows*/-1*(_tileRows-1-2*row), 0.0);
    camera->setViewMatrix( _currentViewMatrix );
    camera->setProjectionMatrix( _currentProjectionMatrix * offsetMatrix );

    _validBbox[idx]=osg::BoundingBox(-1000,-1000,-1000,1000,1000,1000);
    // Check intersections between the image-tile box and the model
    osgUtil::IntersectionVisitor iv( _intersector.get() );
    iv.setReadCallback( g_pagedLoadingCallback.get() );
    _intersector->reset();
    camera->accept( iv );
    if(!_outputEmpty){
        if (! _intersector->containsIntersections())
        {
            if(!_outputEmpty)
            printf("image %d_%d empty but thought to be valid\n",col,row);

            if(!_haveSavedFirstEmpty){

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
   //matrix.preMult( camera->getWindowMatrix());

          cout <<"Inter"<<_model;
   matrix.preMult(camera->getProjectionMatrix() );
    matrix.preMult( camera->getViewMatrix() );
              matrix.preMult(_model);

    if(camera->getViewport())
        matrix.postMult(camera->getViewport()->computeWindowMatrix());
 //    camera->getmatrix.preMult( camera->getModelMatrix() );

    cout <<"BAD "<<matrix<<endl;
    _validMats[idx]=matrix;//*osg::Matrix(1,0,0,0,0,0,1,0,0,1,0,0,0,0,0,1); * camera->getProjectionMatrix()//osg::Matrix::rotate(osg::DegreesToRadians(90.0f),0,0,-1);//

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
            printf("herer\n");
            std::stringstream ss;
            ss<<_dir<<"/";//<<_baseName<<"_files/";

           // osgDB::makeDirectory(ss.str());

            ss<<"/"<<col<<"_"<<(row) <<"."<<_tmpTileExt;
            osgDB::writeImageFile( *image,  ss.str());
            cout << "Writing "<<ss.str();
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

            CvRect srcrect = cvRect(0, 0, w ,h);
            CvRect dstrect = cvRect(x, y, w, h);
            cvSetImageROI(localImage,srcrect);
            cvSetImageROI(_finalPoster,dstrect);
            cvCopy(localImage,_finalPoster);

            cvReleaseImage(&localImage);
            cvResetImageROI(_finalPoster);

        }
        

    }
    _images.clear();


}

void PosterPrinter::writeMats(){


    std::string fname=(_dir+"/"+"re-bbox.txt");
    std::ofstream mfile(fname.c_str(),std::ios_base::trunc);
    if(!mfile.is_open()){
        std::cerr<< "Couldn't open meshes txt  "<<fname<<std::endl;
        return;
    }
    for(int i=0; i<_validTiles.size(); i++){
        mfile<< i << " ";
        int row=_validTiles[i].second;
        int col=_validTiles[i].first;
        std::stringstream ss;

        ss<<col<<"_"<<row <<"."<<_tmpTileExt;
        mfile << ss.str() << " ";
        mfile << _validBbox[i]._min[0] << " " << _validBbox[i]._min[1] << " "<<_validBbox[i]._min[2] << " ";
                mfile << _validBbox[i]._max[0] << " " << _validBbox[i]._max[1] << " "<<_validBbox[i]._max[2] << " ";

cout << _validMats[i]<<endl;
        for(int j=0; j < 4; j++){
            for(int k=0; k <4; k++){
                mfile << _validMats[i](j,k) << " ";
            }
        }
         mfile<<std::endl;


    }
    mfile.close();
}
