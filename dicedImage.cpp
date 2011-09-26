#include "GLImaging.h"
#include "MemUtils.h"
int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] filename ...");
    bool pbufferOnly = !arguments.read("--show");


    unsigned int width=512;
    unsigned int height=512;
    arguments.read("--pbuffer-only",width,height);
    int _tileColumns=0;
    int _tileRows=0;

    osg::BoundingBox totalbb;

    double lat=0,lon=0;
    arguments.read("-lat",lat);
    arguments.read("-lon",lon);
    bool untex= arguments.read("-untex");
    string ext;
    bool nogfx= arguments.read("-nogfx",ext);

    std::vector<picture_cell> cells;
    FILE *fp=fopen(argv[1],"r");
    if(!fp){
        fprintf(stderr,"Can't open %s\n",argv[1]);
        exit(-1);
    }
    int cnt=0;
    while(!feof(fp)){
        char fname[1024];
        float minx,maxx,miny,maxy,minz,maxz;
        int row,col;
        int res=fscanf(fp,"%f %f %f %f %f %f %d %d %s\n",&minx,&maxx,&miny,&maxy,&minz,&maxz,&col,&row,fname);
        if(res != 9){
            fprintf(stderr,"Bad parse\n");
            exit(-1);
        }
        if(cnt==0){
            totalbb=osg::BoundingBox(minx,miny,minz,maxx,maxy,maxz);
            _tileColumns=col;
            _tileRows=row;

        }else{
            picture_cell cell;
            cell.bbox=osg::BoundingBox(minx,miny,minz,maxx,maxy,maxz);
            cell.col=col;
            cell.row=row;
            if(std::string(fname) != "null")
                cell.name=std::string(argv[2])+"/"+std::string(fname);
            else
                cell.name=std::string(fname);
            cells.push_back(cell);

        }
        cnt++;

    }





    osg::Matrixd view,proj;


    std::stringstream os2;
    os2<< "view.mat";

    std::fstream _file(os2.str().c_str(),std::ios::binary|std::ios::in);
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            _file.read(reinterpret_cast<char*>(&(view(i,j))),sizeof(double));
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            _file.read(reinterpret_cast<char*>(&(proj(i,j))),sizeof(double));
    _file.close();

    // std::ostringstream os;
    // os <<"subtile.ppm";//<<":deflate";
    vips::VImage raw;
    vips::VImage raw_untex;
    printf("X: %d Y: %d\n",width*_tileColumns,height*_tileRows);
    raw.initdesc(width*_tileColumns,height*_tileRows,3,vips::VImage::FMTUCHAR,vips::VImage::NOCODING,vips::VImage::sRGB,1.0,1.0,0,0);
    if(untex)
        raw_untex.initdesc(width*_tileColumns,height*_tileRows,3,vips::VImage::FMTUCHAR,vips::VImage::NOCODING,vips::VImage::sRGB,1.0,1.0,0,0);
    double vm, rss;
    process_mem_usage(vm, rss);
    cout << "VM: " << get_size_string(vm) << "; RSS: " << get_size_string(rss) << endl;
    osg::Matrix win;
    unsigned int validCount=0;
    for(int i=0; i < (int)cells.size(); i++){
        if(cells[i].name != "null")
            validCount++;
    }
    osg::Timer_t startTick = osg::Timer::instance()->tick();
    formatBar("Img",startTick,0,validCount);
    FILE *logfp=fopen("DItiming.txt","w");

    int count=0;
    for(int i=0; i < (int)cells.size(); i++)
    {
        osg::Timer_t loopTick = osg::Timer::instance()->tick();
        if(cells[i].name == "null" )
            continue;

        if(nogfx){
            char tmp[1024];
            {
                sprintf(tmp,"mesh-diced/image_r%04d_c%04d_rs%04d_cs%04d.%s",cells[i].row,cells[i].col,_tileRows,_tileColumns,ext.c_str());
                if(osgDB::fileExists(tmp)){
                    vips::VImage tmpI(tmp);
                    raw.insertplace(tmpI.extract_bands(0,3),width*cells[i].col,height*(_tileRows-cells[i].row-1));
                }else{
                    vips::VImage tmpI;
                    tmpI.initdesc(width,height,3,vips::VImage::FMTUCHAR,vips::VImage::NOCODING,vips::VImage::sRGB,1.0,1.0,0,0);
                    memset(tmpI.data(),255,width*height*3);
                    raw.insertplace(tmpI,width*cells[i].col,height*(_tileRows-cells[i].row-1));
                    fprintf(stderr,"Can't find %s\n",tmp);
                }
            }
            if(untex){
                sprintf(tmp,"mesh-diced/image_r%04d_c%04d_rs%04d_cs%04d_untex.%s",cells[i].row,cells[i].col,_tileRows,_tileColumns,ext.c_str());
                if(osgDB::fileExists(tmp)){
                    vips::VImage tmpI(tmp);
                    raw_untex.insertplace(tmpI.extract_bands(0,3),width*cells[i].col,height*(_tileRows-cells[i].row-1));
                }else{
                    vips::VImage tmpI;
                    tmpI.initdesc(width,height,3,vips::VImage::FMTUCHAR,vips::VImage::NOCODING,vips::VImage::sRGB,1.0,1.0,0,0);
                    memset(tmpI.data(),255,width*height*3);
                    raw.insertplace(tmpI,width*cells[i].col,height*(_tileRows-cells[i].row-1));
                    fprintf(stderr,"Can't find %s\n",tmp);
                }
            }
        }else{


            osgViewer::Viewer viewer(arguments);



            GLenum readBuffer = GL_BACK;
            WindowCaptureCallback::FramePosition position = WindowCaptureCallback::END_FRAME;
            WindowCaptureCallback::Mode mode = WindowCaptureCallback::SINGLE_PBO;


            osg::ref_ptr<osg::GraphicsContext> pbuffer;
            {
                osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
                traits->x = 0;
                traits->y = 0;
                traits->width = width;
                traits->height = height;
                traits->red = 8;
                traits->green = 8;
                traits->blue = 8;
                traits->alpha = 8;
                traits->windowDecoration = false;
                traits->pbuffer = true;
                traits->doubleBuffer = true;
                traits->sharedContext = 0;

                pbuffer = osg::GraphicsContext::createGraphicsContext(traits.get());
                //std::cout << "Buffer obj "<< pbuffer->getState()->getMaxBufferObjectPoolSize() << " tex "<<  pbuffer->getState()->getMaxBufferObjectPoolSize() <<std::endl;
                if (pbuffer.valid())
                {
                    //   osg::notify(osg::INFO)<<"Pixel buffer has been created successfully."<<std::endl;
                }
                else
                {
                    osg::notify(osg::INFO)<<"Pixel buffer has not been created successfully."<<std::endl;
                }

            }



            osg::ref_ptr<WindowCaptureCallback> wcc=new WindowCaptureCallback(mode, position, readBuffer);
            osg::ref_ptr<osg::Camera> camera;

            if (pbuffer.valid())
            {camera = new osg::Camera;
                camera->setGraphicsContext(pbuffer.get());
                camera->setViewport(new osg::Viewport(0,0,width,height));
                GLenum buffer = pbuffer->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
                camera->setDrawBuffer(buffer);
                camera->setReadBuffer(buffer);
                camera->setFinalDrawCallback(wcc);

                if (pbufferOnly)
                {
                    viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd());

                    viewer.realize();
                }
                else
                {
                    viewer.realize();

                    viewer.stopThreading();

                    pbuffer->realize();

                    viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd());

                    viewer.startThreading();
                }
            }
            else
            {
                viewer.realize();

                addCallbackToViewer(viewer, wcc);
            }
            osg::Timer_t timeEndSetup = osg::Timer::instance()->tick();
            //double setupTime = osg::Timer::instance()->delta_s(loopTick, timeEndSetup);

            // load the data
            osg::Matrixd offsetMatrix=osg::Matrixd::scale(_tileColumns, _tileRows, 1.0)*osg::Matrixd::translate(_tileColumns-1-2*cells[i].col, _tileRows-1-2*cells[i].row, 0.0);
            /* printf("\r%03d/%03d",i,(int)cells.size());
        fflush(stdout);
*/       // std::cout <<"row: "<<cells[i].row << " col: "<<cells[i].col<<" tc: "<<_tileColumns << " "<<_tileRows<<" "<<"\n"<<osg::Matrix::scale(_tileColumns, _tileRows, 1.0) << "\n"<<osg::Matrix::translate(_tileColumns-1-2*cells[i].col, _tileRows-1-2*cells[i].row, 0.0) <<"\n"<<offsetMatrix<<endl;

            osg::ref_ptr<osg::Node> node=osgDB::readNodeFile(cells[i].name);
            osg::Timer_t timeEndLoad = osg::Timer::instance()->tick();

            double loadTime = osg::Timer::instance()->delta_s(timeEndSetup, timeEndLoad);

            int mem=0;
            if (node.valid() )
            {
                viewer.setSceneData( node );
                viewer.getCamera()->setProjectionMatrix(proj*offsetMatrix);
                viewer.getCamera()->setViewMatrix(view);
                viewer.frame();
                viewer.advance();
                viewer.updateTraversal();
                viewer.renderingTraversals();
                osg::Timer_t timeEndRender = osg::Timer::instance()->tick();
                double renderTime = osg::Timer::instance()->delta_s(timeEndLoad, timeEndRender);

                gpuUsage(1,mem);
                fprintf(logfp,"cnt: %d load %.1fs render: %.1fs mem1: %d MB",count,loadTime,renderTime,mem);
                osg::Image *img=(wcc->getContextData(pbuffer)->_imageBuffer[wcc->getContextData(pbuffer)->_currentImageIndex]);
                vips::VImage tmp(img->data(),img->s(),img->t(),4,vips::VImage::FMTUCHAR);
                raw.insertplace(tmp.flipver().extract_bands(0,3),width*cells[i].col,height*(_tileRows-cells[i].row-1));
                if(untex){
                    node->getOrCreateStateSet()->addUniform(new osg::Uniform("shaderOut",3));
                    viewer.setSceneData( node );
                    viewer.frame();
                    viewer.advance();
                    viewer.updateTraversal();
                    viewer.renderingTraversals();
                    osg::Timer_t timeEndRender2 = osg::Timer::instance()->tick();
                    double renderTime2 = osg::Timer::instance()->delta_s(timeEndRender, timeEndRender2);

                    gpuUsage(1,mem);
                    osg::Image *img=(wcc->getContextData(pbuffer)->_imageBuffer[wcc->getContextData(pbuffer)->_currentImageIndex]);
                    vips::VImage tmp(img->data(),img->s(),img->t(),4,vips::VImage::FMTUCHAR);
                    raw_untex.insertplace(tmp.flipver().extract_bands(0,3),width*cells[i].col,height*(_tileRows-cells[i].row-1));
                    fprintf(logfp," render2: %.1fs mem2: %d",renderTime2,mem);

                }
            }else{
                std::cout << "Invalid " << cells[i].name << "\n";
            }
        }
        formatBar("Img",startTick,++count,validCount);

        osg::Timer_t timeEndLoop = osg::Timer::instance()->tick();
        double loopTime = osg::Timer::instance()->delta_s(loopTick, timeEndLoop);
        fprintf(logfp," loop: %.1fs\n",loopTime);
        fflush(logfp);


    }
    formatBar("Img",startTick,validCount,validCount);
    osg::Timer_t writeStart = osg::Timer::instance()->tick();
    process_mem_usage(vm, rss);
    cout << "VM: " << get_size_string(vm) << "; RSS: " << get_size_string(rss) << endl;
    raw.write("subtile.v");
    double writeTime = osg::Timer::instance()->delta_s(writeStart, osg::Timer::instance()->tick());
    fprintf(logfp,"Write Time %.1fs\n",writeTime);

    if(untex){
        osg::Timer_t writeStart = osg::Timer::instance()->tick();

        raw_untex.write("subtile_untex.v");
        double writeTime = osg::Timer::instance()->delta_s(writeStart, osg::Timer::instance()->tick());
        fprintf(logfp,"Write Time 2 %.1fs\n",writeTime);

    }
    printf("Done\n");
    fclose(logfp);

    applyGeoTags(osg::Vec2(lat,lon),view,proj,raw.Xsize(),raw.Ysize());

}
