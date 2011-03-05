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
#include "imageNode.h"
#include "Extents.h"
#include <vpb/TextureUtils>
#include <osg/ComputeBoundsVisitor>
#include <osg/io_utils>
#include <iostream>
#include <vips/vips.h>
#include <vips/vips>
#include <math.h>
#include <osgUtil/ShaderGen>
#include "Extents.h"
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
osg::Matrix getToScreenMatrix(  osg::ref_ptr< osg::Camera > camera,osg::Vec2 size )
{
    return osg::Matrix( camera->getViewMatrix() * camera->getProjectionMatrix() *( osg::Matrix::translate(1.0,1.0,1.0)*osg::Matrix::scale(0.5*size.x(),0.5*size.y(),0.5f)));//camera->getViewport()->computeWindowMatrix() );
}
osg::Matrix getFromScreenMatrix(  osg::ref_ptr< osg::Camera > camera,osg::Vec2 size  )
{
    return osg::Matrix::inverse( camera->getViewMatrix() * camera->getProjectionMatrix() *( osg::Matrix::translate(1.0,1.0,1.0)*osg::Matrix::scale(0.5*size.x(),0.5*size.y(),0.5f)));
}
void vpb::MyCompositeDestination::writeCameraMatrix(osg::Node *scene){
    const osg::BoundingSphere &bs=scene->getBound();
    osg::Vec3d eye(bs.center()+osg::Vec3(0,0,3.5*bs.radius()));
    osg::ComputeBoundsVisitor cbbv(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
    scene->traverse(cbbv);
    osg::BoundingBox bb = cbbv.getBoundingBox();
    osg::Matrixd matrix;
    matrix.makeTranslate( eye );
    osg::Matrix view=osg::Matrix::inverse(matrix);
    osg::Vec3 centeredMin,centeredMax;
    centeredMin=(bb._min-bb.center());
    centeredMax=(bb._max-bb.center());
    std::cout << "radus "<<bb.radius() <<std::endl;
    double radius=bb.radius();
    if(!std::isfinite(radius))
        return;
    osg::Matrix proj= osg::Matrixd::ortho2D(centeredMin[0],centeredMax[0],centeredMin[1],centeredMax[1]);
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_fileMutex);
        for(int i=0; i<4; i++)
            for(int j=0; j<4; j++)
                _file << view(i,j) << " ";
        _file << "\n";
        for(int i=0; i<4; i++)
            for(int j=0; j<4; j++)
                _file << proj(i,j) << " ";
        _file << "\n";
    }
}

void readCameraMatrix( std::ifstream &file,osg::Matrix &view,osg::Matrix &proj){


    // std::ifstream file(filename.c_str());
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            file >> view(i,j);
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            file >> proj(i,j);
}

/* The main entry */
int render(osg::Node *scene,osg::ref_ptr<osg::Image> &image,osg::GraphicsContext &gc,osg::Matrix &toScreen,const osg::Vec4 &sizes)
{
    // Poster arguments
    bool activeMode = false;
    bool outputPoster = true;
    assert(sizes[2]== sizes[3]);
    int tileWidth = sizes[0], tileHeight = sizes[1];
    int posterWidth = sizes[2], posterHeight = sizes[3];
    std::string posterName = "poster.bmp", extName = "bmp";
    osg::Vec4 bgColor(0.2f, 0.2f, 0.6f, 1.0f);
    osg::Camera::RenderTargetImplementation renderImplementation =  osg::Camera::FRAME_BUFFER_OBJECT;



    // Camera settings for inactive screenshot
    osg::Vec3d latLongHeight( 50.0, 10.0, 2000.0 );
    osg::Vec3d hpr( 0.0, 0.0, 0.0 );
    const osg::BoundingSphere &bs=scene->getBound();
    osg::Vec3d eye(bs.center()+osg::Vec3(0,0,3.5*bs.radius()));

    osg::ComputeBoundsVisitor cbbv(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
    scene->traverse(cbbv);
    osg::BoundingBox bb = cbbv.getBoundingBox();

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
    viewer.getCamera()->setGraphicsContext(&gc);

    if ( renderImplementation==osg::Camera::FRAME_BUFFER )
    {
        // FRAME_BUFFER requires the window resolution equal or greater than the to-be-copied size
        viewer.setUpViewInWindow( 200, 200, tileWidth, tileHeight );
    }
    else
    {
        // We want to see the console output, so just render in a window
        //   viewer.setUpViewInWindow( 200, 200, tileWidth, tileHeight );
        viewer.getCamera()->setViewport(new osg::Viewport(0,0,tileWidth,tileHeight));

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
        osg::Vec3 centeredMin,centeredMax;
        centeredMin=(bb._min-bb.center());
        centeredMax=(bb._max-bb.center());
        // camera->setProjectionMatrixAsOrtho2D(-bs.radius(),bs.radius(),-bs.radius(),bs.radius());
        camera->setProjectionMatrixAsOrtho2D(centeredMin[0],centeredMax[0],centeredMin[1],centeredMax[1]);
        toScreen=getToScreenMatrix(camera,osg::Vec2(posterWidth,posterHeight));
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

/* The main entry */
int renderAll(osg::Node *scene, std::vector<osg::Matrixd> &views,std::vector<osg::Matrixd> &projs,std::vector<std::string> &fnames,const osg::Vec4 &sizes)
{

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x =0;
    traits->y = 0;
    traits->width = 1;
    traits->height = 1;
    traits->windowDecoration = false;
    traits->doubleBuffer = false;
    traits->sharedContext = 0;
    traits->pbuffer = true;

    osg::ref_ptr<osg::GraphicsContext> _gc= osg::GraphicsContext::createGraphicsContext(traits.get());

    if (!_gc)
    {
        osg::notify(osg::NOTICE)<<"Failed to create pbuffer, failing back to normal graphics window."<<std::endl;

        traits->pbuffer = false;
        _gc = osg::GraphicsContext::createGraphicsContext(traits.get());
    }
    // Poster arguments
    bool activeMode = false;
    bool outputPoster = true;
    assert(sizes[2]== sizes[3]);
    int tileWidth = sizes[0], tileHeight = sizes[1];
    int posterWidth = sizes[2], posterHeight = sizes[3];
    std::string posterName = "poster.bmp", extName = "bmp";
    osg::Vec4 bgColor(0.2f, 0.2f, 0.6f, 1.0f);
    osg::Camera::RenderTargetImplementation renderImplementation =  osg::Camera::FRAME_BUFFER_OBJECT;



    // Camera settings for inactive screenshot
    osg::Vec3d hpr( 0.0, 0.0, 0.0 );
    const osg::BoundingSphere &bs=scene->getBound();
    osg::Vec3d eye(bs.center()+osg::Vec3(0,0,3.5*bs.radius()));

    osg::ComputeBoundsVisitor cbbv(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
    scene->traverse(cbbv);
    osg::BoundingBox bb = cbbv.getBoundingBox();

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
    osg::ref_ptr<CameraVectorPrinter> printer = new CameraVectorPrinter;
    printer->setTileSize( tileWidth, tileHeight );
    printer->setPosterSize( posterWidth, posterHeight );
    printer->setCamera( camera.get() );



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
    viewer.getCamera()->setGraphicsContext(_gc.get());

    if ( renderImplementation==osg::Camera::FRAME_BUFFER )
    {
        // FRAME_BUFFER requires the window resolution equal or greater than the to-be-copied size
        viewer.setUpViewInWindow( 200, 200, tileWidth, tileHeight );
    }
    else
    {
        // We want to see the console output, so just render in a window
        //   viewer.setUpViewInWindow( 200, 200, tileWidth, tileHeight );
        viewer.getCamera()->setViewport(new osg::Viewport(0,0,tileWidth,tileHeight));

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
        osg::Vec3 centeredMin,centeredMax;
        centeredMin=(bb._min-bb.center());
        centeredMax=(bb._max-bb.center());
        camera->setProjectionMatrixAsOrtho2D(-bs.radius(),bs.radius(),-bs.radius(),bs.radius());
        //  camera->setProjectionMatrixAsOrtho2D(centeredMin[0],centeredMax[0],centeredMin[1],centeredMax[1]);
        osg::ref_ptr<CustomRenderer> renderer = new CustomRenderer( camera );
        camera->setRenderer( renderer.get() );
        viewer.setThreadingModel( osgViewer::Viewer::SingleThreaded );

        // Realize and initiate the first PagedLOD request
        viewer.realize();
        viewer.frame();

        printer->init( views,projs,fnames );
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

osg::Vec2 calcCoordReproj(const osg::Vec3 &vert,const osg::Matrix &viewProj,const osg::Matrix &screen,const osg::Vec2 &size,const osg::Vec4 &ratio){
    osg::Vec4 v(vert.x(),vert.y(),vert.z(),1.0);
    v=v*viewProj;
    v.x() /= v.w();
    v.y() /= v.w();
    v.z() /= v.w();
    v.w() /= v.w();
    v= v*screen;
    //std::cout << "Pre shift " << v << std::endl;
    v.x() /= size.x();;
    v.y() /= size.y();


    v.x() -= (ratio.x()/size.x());
    v.y() -= (ratio.y()/size.y());
    //std::cout << "Post shift " << v << std::endl;


    //  std::cout << "PP shift " << v << std::endl;


    osg::Vec2 tc(v.x(),v.y());
    tc.x() *= ratio.z();
    tc.y() *=ratio.w();
    //tc.x()*=ratio.x();
    //tc.y()*=ratio.y();
    tc.x()/=(ratio.z());
    tc.y()/=(ratio.w());


    return tc;

}
osg::Matrix vpb::MyDataSet::getImageSection(vips::VImage &in,const osg::Vec2 minT, const osg::Vec2 maxT,int origX,int origY,osg::Vec4 &texsize,const osg::Matrix &toTex,osg::ref_ptr<osg::Image> &image,osg::Vec4 &ratio,int level){

    double downsampleFactor=pow(2,level);
    double downsampleRatio=1.0/downsampleFactor;

    printf("%f %f\n",downsampleRatio,downsampleRatio);
    int x=(int)std::max((int)floor(minT.x()),0);
    int y=(int)std::max((int)floor(minT.y()),0);
    int xMax=(int)std::min((int)ceil(maxT.x()),origX);
    int yMax=(int)std::min((int)ceil(maxT.y()),origY);
    int xRange=(xMax-x);
    int yRange=(yMax-y);
    //printf("X:%f -- %f Y:%f -- %f\n",vMin.x(),vMax.x(),vMin.y(),vMax.y());

    //Need bias of 1.0 or will round down
    double maxSide=std::max(osg::Image::computeNearestPowerOfTwo(xRange,1.0),osg::Image::computeNearestPowerOfTwo(yRange,1.0));

    osg::Vec2 subSize=osg::Vec2(maxSide,maxSide);
    if(downsampleRatio*maxSide < 1.0){
        printf("Clipping %f %f to",downsampleRatio,downsampleFactor);
        downsampleRatio=1.0/maxSide;
        downsampleFactor=maxSide;
        printf("%f %f\n",downsampleRatio,downsampleFactor);

    }
    texsize[0]=origX;
    texsize[1]=origY;
    texsize[2]=subSize.x();
    texsize[3]=subSize.y();
    osg::Vec2 downsampleSize(subSize.x(),subSize.y());
    downsampleSize.x()*=downsampleRatio;
    downsampleSize.y()*=downsampleRatio;

    // printf("Range %d %d\n",xRange,yRange);
    // printf("%f %f %f %f\n",subSize.x(),subSize.y(),downsampleSize.x(),downsampleSize.y());
    image = new osg::Image;
    image->allocateImage(downsampleSize.x(),downsampleSize.y(), 1, GL_RGBA,GL_UNSIGNED_BYTE);
    if(image->data() == 0 ){
        fprintf(stderr,"Failed to allocate\n");
        exit(-1);
    }
    {
     //   OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_imageMutex);
        vips::VImage *osgImage = new vips::VImage(image->data(),downsampleSize.x(),downsampleSize.y(),4,vips::VImage::FMTUCHAR);
#warning "memleak"
        in.extract_area(x,y,xRange,yRange).embed(1,0,0,subSize.x(),subSize.y())./*shrink(downsampleFactor,downsampleFactor)*/affine(downsampleRatio,0,0,downsampleRatio,0,0,0,0,downsampleSize.x(),downsampleSize.y())
                                                                                                                                    .write(*osgImage);
       // delete osgImage;
    }
    ratio=osg::Vec4(x,y,subSize.x(),subSize.y());
    //osg::Vec2 f(xRange-subSize.x(),yRange-subSize.y());
    //std::cout << f<<std::endl;
    return toTex;
}

osg::Group *vpb::MyCompositeDestination::convertModel(osg::Group *group){
    if(!group)
        return NULL;
    //
    if(group->getNumChildren() == 0)
        return group;
    if(!std::isfinite(group->getBound().radius()) || group->getBound().radius() < 0.0)
        return group;
    //double l=group->getBound().radius();
    //printf("Brav %f\n",l);
    /* if(_level!=_numLevels)
        return group;
    writeCameraMatrix(group);
    return group;
*/
    if(!_useReImage)
        return group;
    osg::ref_ptr<osg::Image> image;


    osg::Matrix toScreen;
    osg::Vec2Array *texCoord=new osg::Vec2Array();
    osg::Geometry *newGeom = new osg::Geometry;
    // osg::Group *group= findTopMostNodeOfType<osg::Group>(model);
    osg::Vec3Array *newVerts= new osg::Vec3Array;
    osg::DrawElementsUInt* newPrimitiveSet = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES,0);
    osg::Geode *newGeode=new osg::Geode;
    osg::Vec2 minT(DBL_MAX,DBL_MAX),maxT(-DBL_MAX,-DBL_MAX);
    // ("subtile.tif");

    int origX=dynamic_cast<MyDataSet*>(_dataSet)->in->Xsize(),origY=dynamic_cast<MyDataSet*>(_dataSet)->in->Ysize();
    osg::Matrix toTex=dynamic_cast<MyDataSet*>(_dataSet)->viewProj*( osg::Matrix::translate(1.0,1.0,1.0)*osg::Matrix::scale(0.5*origX,0.5*origY,0.5f));

    for(int i=0; i< (int)group->getNumChildren(); i++){

        osg::Group *group2  = dynamic_cast< osg::Group*>(group->getChild(i));
        osg::Geode *geode;
        if(group2)
            geode=group2->getChild(0)->asGeode();
        else
            geode = dynamic_cast< osg::Geode*>(group->getChild(i));

        osg::Drawable *drawable=geode->getDrawable(0);
        osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
        osg::Vec3Array *verts=static_cast<const osg::Vec3Array*>(geom->getVertexArray());
        osg::DrawElementsUInt* primitiveSet = dynamic_cast<osg::DrawElementsUInt*>(geom->getPrimitiveSet(0));
        int offset=newVerts->size();
        if(!verts || !primitiveSet)
            continue;
        for(int j=0; j< (int)verts->size(); j++){
            osg::Vec4 pt(verts->at(j)[0],verts->at(j)[1],verts->at(j)[2],1.0);
            osg::Vec4 proj=pt*toTex;
            proj.x() /= proj.w();
            proj.y() /= proj.w();
            for(int k=0; k <2; k++){
                if(proj[k]< minT[k])
                    minT[k]=proj[k];
                if(proj[k]> maxT[k])
                    maxT[k]=proj[k];
            }
            newVerts->push_back(verts->at(j));
        }
        for(int j=0; j< (int)primitiveSet->getNumIndices(); j++){
            newPrimitiveSet->addElement(offset+primitiveSet->getElement(j));
        }

    }
    osg::Vec4 texSizes;
    osg::Vec4 ratio(0.0,0.0,0,0);

    if(0){
        ///OLD slow render
        texSizes=osg::Vec4(1024,1024,1024,1024);
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->x =0;
        traits->y = 0;
        traits->width = 1024;
        traits->height = 1024;
        traits->windowDecoration = false;
        traits->doubleBuffer = false;
        traits->sharedContext = 0;
        traits->pbuffer = true;

        osg::ref_ptr<osg::GraphicsContext> _gc= osg::GraphicsContext::createGraphicsContext(traits.get());

        if (!_gc)
        {
            osg::notify(osg::NOTICE)<<"Failed to create pbuffer, failing back to normal graphics window."<<std::endl;

            traits->pbuffer = false;
            _gc = osg::GraphicsContext::createGraphicsContext(traits.get());
        }


        //printf("%d\n",newVerts->size());
        render(group,image,*_gc,toScreen,texSizes);
    }else{
        //std::cout << minV << " "<<maxV<<std::endl;
        int start_pow=9;
        //tex_size=1024;//log2 = 10
        int leveloffset=(_numLevels-_level);
        leveloffset=std::min(leveloffset,5);

        toScreen=dynamic_cast<MyDataSet*>(_dataSet)->getImageSection(*(dynamic_cast<MyDataSet*>(_dataSet)->in),minT,maxT,origX,origY,texSizes, toTex,image,ratio,leveloffset);
    }

    for(int j=0; j< (int)newVerts->size(); j++){

        texCoord->push_back(calcCoordReproj(newVerts->at(j),toScreen,osg::Matrix::identity(),osg::Vec2(texSizes[2],texSizes[3]),ratio));
        //  std::cout <<texCoord->back() << std::endl;

    }
    for(int j=0; j< (int)newPrimitiveSet->getNumIndices(); j++){
        if(newPrimitiveSet->getElement(j) < 0 || newPrimitiveSet->getElement(j) > newVerts->size() ){
            printf("ASDADASDASDASDADS\n");
            exit(-1);
        }
    }
    //printf("%d %d\n",newPrimitiveSet->getNumIndices(),        newPrimitiveSet->getNumIndices()/3);


    newGeom->setTexCoordArray(0,texCoord);
    newGeom->setVertexArray(newVerts);
    newGeom->addPrimitiveSet(newPrimitiveSet);
    newGeom->setUseDisplayList(_useDisplayLists);
    newGeom->setUseVertexBufferObjects(_useVBO);

    newGeode->addDrawable(newGeom);
    char tmp[128];
    // if(image->s() != image->t()){
    sprintf(tmp,"%d-%d-%d.png",image->s(),image->t(),rand());
    // osgDB::writeImageFile(*image.get(),tmp);
    //}
    osg::ref_ptr<osg::Texture2D> texture=new osg::Texture2D(image);
    texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_BORDER);
    texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_BORDER);
    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    texture->setTextureSize(image->s(),image->t());
    //std::cout <<  "Check it "<<texture->getTextureWidth() << " "<< texture->getTextureHeight()<<"\n";

    osg::Texture::InternalFormatMode internalFormatMode = osg::Texture::USE_IMAGE_DATA_FORMAT;
    internalFormatMode = osg::Texture::USE_S3TC_DXT1_COMPRESSION;
    /*  switch(getImageOptions(layerNum)->getTextureType())
    {
    case(BuildOptions::RGB_S3TC_DXT1): internalFormatMode = osg::Texture::USE_S3TC_DXT1_COMPRESSION; break;
    case(BuildOptions::RGBA_S3TC_DXT1): internalFormatMode = osg::Texture::USE_S3TC_DXT1_COMPRESSION; break;
    case(BuildOptions::RGBA_S3TC_DXT3): internalFormatMode = osg::Texture::USE_S3TC_DXT3_COMPRESSION; break;
    case(BuildOptions::RGBA_S3TC_DXT5): internalFormatMode = osg::Texture::USE_S3TC_DXT5_COMPRESSION; break;
    case(BuildOptions::ARB_COMPRESSED): internalFormatMode = osg::Texture::USE_ARB_COMPRESSION; break;
    case(BuildOptions::COMPRESSED_TEXTURE): internalFormatMode = osg::Texture::USE_S3TC_DXT1_COMPRESSION; break;
    case(BuildOptions::COMPRESSED_RGBA_TEXTURE): internalFormatMode = osg::Texture::USE_S3TC_DXT3_COMPRESSION; break;
    default: break;
    }
*/
    bool compressedImageRequired = (internalFormatMode != osg::Texture::USE_IMAGE_DATA_FORMAT);
    //  image->s()>=minumCompressedTextureSize && image->t()>=minumCompressedTextureSize &&

    if (0 &&/*compressedImageSupported && */compressedImageRequired )
    {
        log(osg::NOTICE,"Compressed image");

        bool generateMiMap = true;//getImageOptions(layerNum)->getMipMappingMode()==DataSet::MIP_MAPPING_IMAGERY;
        bool resizePowerOfTwo = true;//getImageOptions(layerNum)->getPowerOfTwoImages();
        vpb::compress(*_dataSet->getState(),*texture,internalFormatMode,generateMiMap,resizePowerOfTwo,_dataSet->getCompressionMethod(),_dataSet->getCompressionQuality());

        log(osg::INFO,">>>>>>>>>>>>>>>compressed image.<<<<<<<<<<<<<<");

    }

    osg::StateSet *stateset=newGeode->getOrCreateStateSet();

    stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
    stateset->setMode( GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF );

    stateset->setDataVariance(osg::Object::STATIC);
 //   osgUtil::ShaderGenVisitor sgv;
   // newGeode->accept(sgv);

    // osg::Vec3 v(1972.38,3932.55,0);
    //osg::Vec3 v(302.3,334.3,0);
    //  std::cout << v*toScreen << " " << toScreen<<std::endl;
    // osgDB::Registry::instance()->writeImage( *image,"ass.png",NULL);
    osg::Group *newGroup=new osg::Group;
    newGroup->addChild(newGeode);
    return newGroup;

}

