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
#include <osg/Switch>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Renderer>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <iostream>
#include "PosterPrinter.h"


/* CustomRenderer: Do culling only while loading PagedLODs */
class CustomRenderer : public osgViewer::Renderer
{
public:
    CustomRenderer( osg::Camera* camera )
    : osgViewer::Renderer(camera), _cullOnly(true)
    {
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
osg::Matrix getToScreenMatrix(  osg::ref_ptr< osg::Camera > camera )
{
    return osg::Matrix( camera->getViewMatrix() * camera->getProjectionMatrix() * camera->getViewport()->computeWindowMatrix() );
}
osg::Matrix getFromScreenMatrix(  osg::ref_ptr< osg::Camera > camera )
{
    return osg::Matrix::inverse( camera->getViewMatrix() * camera->getProjectionMatrix() * camera->getViewport()->computeWindowMatrix() );
}
/* The main entry */
int render(osg::Node *scene,osg::ref_ptr<osg::Image> &image,osg::Matrix &toScreen,const osg::Vec2 &texSize)
{
    // Poster arguments
    bool activeMode = false;
    bool outputPoster = true, outputTiles = false;
    int tileWidth = 512, tileHeight = 512;
    int posterWidth = texSize.x(), posterHeight = texSize.y();
    std::string posterName = "poster.bmp", extName = "bmp";
    osg::Vec4 bgColor(0.2f, 0.2f, 0.6f, 1.0f);
    osg::Camera::RenderTargetImplementation renderImplementation =  osg::Camera::FRAME_BUFFER_OBJECT;



    // Camera settings for inactive screenshot
    osg::Vec3d latLongHeight( 50.0, 10.0, 2000.0 );
    osg::Vec3d hpr( 0.0, 0.0, 0.0 );
    const osg::BoundingSphere &bs=scene->getBound();
    osg::Vec3d eye(bs.center()+osg::Vec3(0,0,3.5*bs.radius()));


    if ( !scene )
    {
        std::cout << "No data loaded" << std::endl;
        return 1;
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

    // Set the printer
    osg::ref_ptr<PosterPrinter> printer = new PosterPrinter;
    printer->setTileSize( tileWidth, tileHeight );
    printer->setPosterSize( posterWidth, posterHeight );
    printer->setCamera( camera.get() );

    if ( outputPoster )
    {
        image = new osg::Image;
        image->allocateImage( posterWidth, posterHeight, 1, GL_RGBA, GL_UNSIGNED_BYTE );
        printer->setFinalPoster( image.get() );
        printer->setOutputPosterName( posterName );
    }

#if 0
    // While recording sub-images of the poster, the scene will always be traversed twice, from its two
    // parent node: root and camera. Sometimes this may not be so comfortable.
    // To prevent this behaviour, we can use a switch node to enable one parent and disable the other.
    // However, the solution also needs to be used with care, as the window will go blank while taking
    // snapshots and recover later.
    osg::ref_ptr<osg::Switch> root = new osg::Switch;
    root->addChild( scene, true );
    root->addChild( camera.get(), false );
#else
    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild( scene );
    root->addChild( camera.get() );
#endif

    osgViewer::Viewer viewer;
    viewer.setSceneData( root.get() );
    viewer.getDatabasePager()->setDoPreCompile( false );

    if ( renderImplementation==osg::Camera::FRAME_BUFFER )
    {
        // FRAME_BUFFER requires the window resolution equal or greater than the to-be-copied size
        viewer.setUpViewInWindow( 200, 200, tileWidth, tileHeight );
    }
    else
    {
        // We want to see the console output, so just render in a window
        viewer.setUpViewInWindow( 200, 200, tileWidth, tileHeight );
    }

    if ( activeMode )
    {
        //viewer.addEventHandler( new PrintPosterHandler(printer.get()) );
        viewer.addEventHandler( new osgViewer::StatsHandler );
        viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
        viewer.setCameraManipulator( new osgGA::TrackballManipulator );
        viewer.run();
    }
    else
    {
        osg::Camera* camera = viewer.getCamera();
        computeViewMatrix( camera, eye, hpr );
        camera->setProjectionMatrixAsOrtho2D(-bs.radius(),bs.radius(),-bs.radius(),bs.radius());
        toScreen=getToScreenMatrix(camera);
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

