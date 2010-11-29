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

#include <osg/ArgumentParser>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Renderer>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <iostream>
#include <osg/io_utils>
#include "PosterPrinter.h"
#if __APPLE__
//Broken Pbuffer cocoa implemtation
const bool pBufferWorks=false;
#else
const bool pBufferWorks=true;
#endif
/* Computing view matrix helpers */
template<class T>
class FindTopMostNodeOfTypeVisitor : public osg::NodeVisitor
{
public:
    FindTopMostNodeOfTypeVisitor()
        :   osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
        _foundNode(0)
    {}
    
    void apply( osg::Node& node )
    {
        T* result = dynamic_cast<T*>( &node );
        if ( result ) _foundNode = result;
        else traverse( node );
    }
    
    T* _foundNode;
};

template<class T>
T* findTopMostNodeOfType( osg::Node* node )
{
    if ( !node ) return 0;
    
    FindTopMostNodeOfTypeVisitor<T> fnotv;
    node->accept( fnotv );
    return fnotv._foundNode;
}


void  checkIntersection(osgViewer::View *viewer,int numC,int numR,std::vector<std::pair<unsigned int,unsigned int> > &valid,std::pair<unsigned int,unsigned int> &empty){
    osgUtil::PolytopeIntersector* picker;
    // use window coordinates
    // remap the mouse x,y into viewport coordinates.
    osg::Viewport* viewport = viewer->getCamera()->getViewport();
    double width= viewport->width();
    double height=viewport->height();
    double deltaW=width/numC;
    double deltaH=height/numR;
    //printf("here %d %d\n",numC,numR);
    bool gotempty=false;
    for(int c=0; c < numC; c++){
        for(int r=0; r<numR; r++){
            picker = new osgUtil::PolytopeIntersector( osgUtil::Intersector::WINDOW, deltaW*c, deltaH*r, deltaW*(c+1), deltaH*(r+1) );
            //  printf("checking %f %f %f %f\n",deltaW*c, deltaH*r, deltaW*(c+1), deltaH*(r+1));
            osgUtil::IntersectionVisitor iv(picker);

            viewer->getCamera()->accept(iv);
            if(picker->containsIntersections()){
                //Get one empty
                valid.push_back(std::make_pair(c,/*flip row*/numR-r-1));
                //  printf("%d_%d\n",valid.back().second-1,valid.back()    .first);
                //  std::cout <<"haveit" << iv.getModelMatrix();
            }else if(!gotempty){
                gotempty=true;
                empty=std::make_pair(c,/*flip row*/numR-r-1);

            }
        }
        // printf("\n");
    }
    if(!gotempty)
        empty=std::make_pair(-1,-1);

    printf("Done checking valid size %d/%d\n",(int)valid.size(),numC*numR);
    //  exit(0);
}


/* Computing view matrix functions */
void computeViewMatrix( osg::Camera* camera, const osg::Vec3d& eye, const osg::Vec3d& hpr )
{
    osg::Matrixd matrix;
    matrix.makeTranslate( eye );
    matrix.preMult( osg::Matrixd::rotate( hpr[0], 0.0, 1.0, 0.0) );
    matrix.preMult( osg::Matrixd::rotate( hpr[1], 1.0, 0.0, 0.0) );
    matrix.preMult( osg::Matrixd::rotate( hpr[2], 0.0, 0.0, 1.0) );
    camera->setViewMatrix( osg::Matrixd::inverse(matrix) );
}

void computeViewMatrixOnEarth( osg::Camera* camera, osg::Node* scene,
                               const osg::Vec3d& latLongHeight, const osg::Vec3d& hpr )
{
    osg::CoordinateSystemNode* csn = findTopMostNodeOfType<osg::CoordinateSystemNode>(scene);
    if ( !csn ) return;
    
    // Compute eye point in world coordiantes
    osg::Vec3d eye;
    csn->getEllipsoidModel()->convertLatLongHeightToXYZ(
            latLongHeight.x(), latLongHeight.y(), latLongHeight.z(), eye.x(), eye.y(), eye.z() );
    
    // Build matrix for computing target vector
    osg::Matrixd target_matrix =
            osg::Matrixd::rotate( -hpr.x(), osg::Vec3d(1,0,0),
                                  -latLongHeight.x(), osg::Vec3d(0,1,0),
                                  latLongHeight.y(), osg::Vec3d(0,0,1) );
    
    // Compute tangent vector
    osg::Vec3d tangent = target_matrix.preMult( osg::Vec3d(0,0,1) );
    
    // Compute non-inclined, non-rolled up vector
    osg::Vec3d up( eye );
    up.normalize();
    
    // Incline by rotating the target- and up vector around the tangent/up-vector
    // cross-product
    osg::Vec3d up_cross_tangent = up ^ tangent;
    osg::Matrixd incline_matrix = osg::Matrixd::rotate( hpr.y(), up_cross_tangent );
    osg::Vec3d target = incline_matrix.preMult( tangent );
    
    // Roll by rotating the up vector around the target vector
    osg::Matrixd roll_matrix = incline_matrix * osg::Matrixd::rotate( hpr.z(), target );
    up = roll_matrix.preMult( up );
    camera->setViewMatrixAsLookAt( eye, eye+target, up );
}

/* CustomRenderer: Do culling only while loading PagedLODs */
class CustomRenderer : public osgViewer::Renderer
{
public:
    CustomRenderer( osg::Camera* camera )
        : osgViewer::Renderer(camera), _cullOnly(true)
    {
        setTargetFrameRate(1);
        setMinimumTimeAvailableForGLCompileAndDeletePerFrame(1);
    }
    
    void setCullOnly(bool on) { _cullOnly = on; }

    virtual void operator ()( osg::GraphicsContext* )
    {
        if ( _graphicsThreadDoesCull )
        {
            if (_cullOnly) cull();
            else cull_draw();
        }
    }
    
    virtual void cull()
    {
        osgUtil::SceneView* sceneView = _sceneView[0].get();
        if ( !sceneView || _done || _graphicsThreadDoesCull )
            return;
        
        updateSceneView( sceneView );
        
        osgViewer::View* view = dynamic_cast<osgViewer::View*>( _camera->getView() );
        if ( view )
            sceneView->setFusionDistance( view->getFusionDistanceMode(), view->getFusionDistanceValue() );
        sceneView->inheritCullSettings( *(sceneView->getCamera()) );
        sceneView->cull();
    }
    
    bool _cullOnly;
};

/* PrintPosterHandler: A gui handler for interactive high-res capturing */
class PrintPosterHandler : public osgGA::GUIEventHandler
{
public:
    PrintPosterHandler( PosterPrinter* printer )
        : _printer(printer) {}
    
    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        osgViewer::View* view = dynamic_cast<osgViewer::View*>( &aa );
        if ( !view ) return false;
        
        switch( ea.getEventType() )
        {
        case osgGA::GUIEventAdapter::FRAME:
            if ( view->getDatabasePager() )
            {
                // Wait until all paged nodes are processed
                if ( view->getDatabasePager()->getRequestsInProgress() )
                    break;
            }
            
            if ( _printer.valid() )
                _printer->frame( view->getFrameStamp(), view->getSceneData() );
            break;

        case osgGA::GUIEventAdapter::KEYDOWN:
            if ( ea.getKey()=='p' || ea.getKey()=='P' )
            {
                if ( _printer.valid() )
                    _printer->init( view->getCamera() );
                return true;
            }
            break;

        default:
            break;
        }
        return false;
    }

protected:
    osg::ref_ptr<PosterPrinter> _printer;
};

/* The main entry */
int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );
    osg::ApplicationUsage* usage = arguments.getApplicationUsage();
    usage->setDescription( arguments.getApplicationName() +
                           " is the example which demonstrates how to render high-resolution images (posters).");
    usage->setCommandLineUsage( arguments.getApplicationName() + " [options] scene_file" );
    usage->addCommandLineOption( "-h or --help", "Display this information." );
    usage->addCommandLineOption( "--color <r> <g> <b>", "The background color." );
    usage->addCommandLineOption( "--ext <ext>", "The output tiles' extension (Default: bmp)." );
    usage->addCommandLineOption( "--poster <filename>", "The output poster's name (Default: poster.bmp)." );
    usage->addCommandLineOption( "--tilesize <w> <h>", "Size of each image tile (Default: 640 480)." );
    usage->addCommandLineOption( "--finalsize <w> <h>", "Size of the poster (Default: 6400 4800)." );
    usage->addCommandLineOption( "--enable-output-poster", "Output the final poster file (Default)." );
    usage->addCommandLineOption( "--disable-output-poster", "Don't output the final poster file." );
    usage->addCommandLineOption( "--enable-output-tiles", "Output all tile files." );
    usage->addCommandLineOption( "--disable-output-tiles", "Don't output all tile files (Default)." );
    usage->addCommandLineOption( "--use-fb", "Use Frame Buffer for rendering tiles (Default, recommended).");
    usage->addCommandLineOption( "--use-fbo", "Use Frame Buffer Object for rendering tiles.");
    usage->addCommandLineOption( "--use-pbuffer","Use Pixel Buffer for rendering tiles.");
    usage->addCommandLineOption( "--use-pbuffer-rtt","Use Pixel Buffer RTT for rendering tiles.");
    usage->addCommandLineOption( "--inactive", "Inactive capturing mode." );
    usage->addCommandLineOption( "--camera-eye <x> <y> <z>", "Set eye position in inactive mode." );
    usage->addCommandLineOption( "--camera-latlongheight <lat> <lon> <h>", "Set eye position on earth in inactive mode." );
    usage->addCommandLineOption( "--camera-hpr <h> <p> <r>", "Set eye rotation in inactive mode." );
    
    if ( arguments.read("-h") || arguments.read("--help") )
    {
        usage->write( std::cout );
        return 1;
    }
    
    // Poster arguments
    bool activeMode = true;
    bool untex=false;

    bool outputPoster = true, outputTiles = false,outputHeightMap=false;
    int tileWidth = 512, tileHeight = 512;
    int posterWidth = 512*10, posterHeight = 512*10;
    std::string posterName = "poster.tif", extName = "tif",heightMapName="height.tif";
    osg::Vec4 bgColor(0.2f, 0.2f, 0.6f, 1.0f);
    osg::Camera::RenderTargetImplementation renderImplementation = osg::Camera::FRAME_BUFFER;
    osg::Vec2d origin(0,0);
    while ( arguments.read("--inactive") ) { activeMode = false; }
    while ( arguments.read("--color", bgColor.r(), bgColor.g(), bgColor.b()) ) {}
    while ( arguments.read("--tilesize", tileWidth, tileHeight) ) {}
    while ( arguments.read("--finalsize", posterWidth, posterHeight) ) {}
    while ( arguments.read("--poster", posterName) ) {}
    while ( arguments.read("--heightmap", heightMapName) ) {}
    while ( arguments.read("--origin", origin.x(),origin.y()) ) {}

    while ( arguments.read("--ext", extName) ) {}
    while ( arguments.read("--enable-output-heightmap") ) { outputHeightMap = true; }

    while ( arguments.read("--enable-output-poster") ) { outputPoster = true; }
    while ( arguments.read("--disable-output-poster") ) { outputPoster = false; }
    while ( arguments.read("--enable-output-tiles") ) { outputTiles = true; }
    while ( arguments.read("--disable-output-tiles") ) { outputTiles = false; }
    while ( arguments.read("--use-fbo")) { renderImplementation = osg::Camera::FRAME_BUFFER_OBJECT; }
    while ( arguments.read("--use-pbuffer")) { renderImplementation = osg::Camera::PIXEL_BUFFER; }
    while ( arguments.read("--use-pbuffer-rtt")) { renderImplementation = osg::Camera::PIXEL_BUFFER_RTT; }
    while ( arguments.read("--use-fb")) { renderImplementation = osg::Camera::FRAME_BUFFER; }
    while ( arguments.read("--notex") ) {untex=true;}
    if(origin.x() == 0 && origin.y() == 0)
        fprintf(stderr,"Warning origin not defined but generating geotiffs!\n");

    // Camera settings for inactive screenshot
    bool useLatLongHeight = true;
    osg::Vec3d eye;
    osg::Vec3d latLongHeight( 50.0, 10.0, 2000.0 );
    osg::Vec3d hpr( 0.0, 0.0, 0.0 );
    if ( arguments.read("--camera-eye", eye.x(), eye.y(), eye.z()) )
    {
        useLatLongHeight = false;
        activeMode = false;
    }
    else if ( arguments.read("--camera-latlongheight", latLongHeight.x(), latLongHeight.y(), latLongHeight.z()) )
    {
        activeMode = false;
        latLongHeight.x() = osg::DegreesToRadians( latLongHeight.x() );
        latLongHeight.y() = osg::DegreesToRadians( latLongHeight.y() );
    }
    if ( arguments.read("--camera-hpr", hpr.x(), hpr.y(), hpr.z()) )
    {
        activeMode = false;
        hpr.x() = osg::DegreesToRadians( hpr.x() );
        hpr.y() = osg::DegreesToRadians( hpr.y() );
        hpr.z() = osg::DegreesToRadians( hpr.z() );
    }
    
    // Construct scene graph
    osg::Node* scene = osgDB::readNodeFiles( arguments );
    if ( !scene ) scene = osgDB::readNodeFile( "cow.osg" );
    if ( !scene )
    {
        std::cout << arguments.getApplicationName() <<": No data loaded" << std::endl;
        return 1;
    }
    if(untex){
        //Set shader out to 3 untex
        osg::Uniform* shared_shader_out = new osg::Uniform("shaderOut",3);
        scene->getOrCreateStateSet()->addUniform(shared_shader_out);
        //   osg::Uniform* shared_shader_out2 = new osg::Uniform("zrange",3.444950,1.817355
        //							 ,0);
        //scene->getOrCreateStateSet()->addUniform(shared_shader_out2);
    }
    // Create camera for rendering tiles offscreen. FrameBuffer is recommended because it requires less memory.
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setClearColor( bgColor );
    camera->setClearMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
    camera->setRenderOrder( osg::Camera::PRE_RENDER );
    camera->setRenderTargetImplementation( renderImplementation );
    camera->setViewport( 0, 0, tileWidth, tileHeight );
    camera->addChild( scene );
  //  camera->setComputeNearFarMode(osg::Camera::DO_NOT_COMPUTE_NEAR_FAR);

    const osg::BoundingSphere& bs = scene->getBound();
    float znear = 1.0f*bs.radius();
    float zfar  = 3.0f*bs.radius();


    camera->setProjectionMatrixAsOrtho(-bs.radius(),bs.radius(),-bs.radius(),bs.radius(),znear,zfar);
    camera->setViewMatrixAsLookAt(bs.center(), osg::Vec3d(0.0f,0.0f,0.0f),osg::Vec3(0.0,0.0,1.0));



    // Set the printer
    osg::ref_ptr<PosterPrinter> printer = new PosterPrinter;
    printer->setTileSize( tileWidth, tileHeight );
    printer->setPosterSize( posterWidth, posterHeight );
    printer->setCamera( camera.get() );
    printer->setUseDepth(outputHeightMap);
    printer->setGeoOrigin(origin);
    osg::ref_ptr<osg::Image> posterImage = 0;
    osg::ref_ptr<osg::Image> heightmapImage = 0;

    if ( outputPoster )
    {
        posterImage = new osg::Image;
        posterImage->allocateImage( posterWidth, posterHeight, 1, GL_RGBA, GL_UNSIGNED_BYTE );
        printer->setFinalPoster( posterImage.get() );
        printer->setOutputPosterName( posterName );
    }
    if ( outputHeightMap )
    {
        heightmapImage = new osg::Image;
        heightmapImage->allocateImage( posterWidth, posterHeight, 1, GL_DEPTH_COMPONENT, GL_FLOAT );
        printer->setFinalHeightMap( heightmapImage.get() );
        printer->setOutputHeightMapName( heightMapName );
    }

    
    // Create root and start the viewer
    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild( scene );
    root->addChild( camera.get() );

    osgViewer::Viewer viewer;
    if(!activeMode && pBufferWorks){
        int x=0;
        int y=0;

        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->x =x;
        traits->y = y;
        traits->width = tileWidth;
        traits->height = tileHeight;
        traits->windowDecoration = false;
        traits->doubleBuffer = false;
        traits->sharedContext = 0;
        traits->pbuffer = true;

        osg::GraphicsContext* _gc= osg::GraphicsContext::createGraphicsContext(traits.get());

        if (!_gc)
        {
            osg::notify(osg::NOTICE)<<"Failed to create pbuffer, failing back to normal graphics window."<<std::endl;

            traits->pbuffer = false;
            _gc = osg::GraphicsContext::createGraphicsContext(traits.get());
        }
        viewer.getCamera()->setGraphicsContext(_gc);
        /*double fovy, aspectRatio, zNear, zFar;
        viewer.getCamera()->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);

        double newAspectRatio = double(traits->width) / double(traits->height);
        double aspectRatioChange = newAspectRatio / aspectRatio;
        if (aspectRatioChange != 1.0)
        {
            viewer.getCamera()->getProjectionMatrix() *= osg::Matrix::scale(1.0/aspectRatioChange,1.0,1.0);
        }*/
        viewer.getCamera()->setViewport(new osg::Viewport(x,y,tileWidth,tileHeight));
    }else{
        viewer.setUpViewInWindow( 100, 100, tileWidth, tileHeight );
    }
    viewer.setSceneData( root.get() );
    viewer.getDatabasePager()->setDoPreCompile( false );
    viewer.getDatabasePager()->setTargetMaximumNumberOfPageLOD(1);
    viewer.getCamera()->setProjectionMatrixAsOrtho(-bs.radius(),bs.radius(),-bs.radius(),bs.radius(),znear,zfar);
    viewer.getCamera()->setViewMatrixAsLookAt(bs.center(), osg::Vec3d(0.0f,0.0f,0.0f), osg::Vec3(0.0,0.0,1.0));
    if ( activeMode )
    {
        viewer.addEventHandler( new PrintPosterHandler(printer.get()) );
        viewer.addEventHandler( new osgViewer::StatsHandler );
        viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
     //   viewer.setCameraManipulator( new osgGA::TrackballManipulator );
        viewer.run();
    }
    else
    {
        osg::Camera* camera = viewer.getCamera();
        viewer.addEventHandler( new osgViewer::StatsHandler );
        viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
      //  viewer.setCameraManipulator( new osgGA::TrackballManipulator );
        // if ( !useLatLongHeight ) computeViewMatrix( camera, eye, hpr );
        // else computeViewMatrixOnEarth( camera, scene, latLongHeight, hpr );

        osg::ref_ptr<CustomRenderer> renderer = new CustomRenderer( camera );
        camera->setRenderer( renderer.get() );
        viewer.setThreadingModel( osgViewer::Viewer::SingleThreaded );
        
        // Realize and initiate the first PagedLOD request
        viewer.realize();
        viewer.frame();

        printer->init( camera );
        while ( !printer->done() )
        {
            viewer.advance();
            
            // Keep updating and culling until full level of detail is reached
            renderer->setCullOnly( true );
            while ( viewer.getDatabasePager()->getRequestsInProgress() )
            {
                viewer.updateTraversal();
                viewer.renderingTraversals();
            }
            
            renderer->setCullOnly( false );
            printer->frame( viewer.getFrameStamp(), viewer.getSceneData() );
            viewer.renderingTraversals();
        }
    }
    return 0;
}
