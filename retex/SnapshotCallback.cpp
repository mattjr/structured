#include "SnapshotCallback.h"

#include "highgui.h"

using namespace auv_data_tools;

SnapImageDrawCallback::SnapImageDrawCallback(std::vector<osg::Matrixd> passedM,std::vector<bbox> bboxes,osgDB::DatabasePager *dp,osgViewer::Viewer *viewer,osg::Node *root) :_dp(dp),_viewer(viewer),_root(root),_passedM(passedM),_bboxes(bboxes){
  _snapImageOnNextFrame=false;
  takeSnapTile=false;
  tileBuffer=NULL;
 
  premove=MOVE;
  firstFrame=false;
  imgAllocated=false;
  depthFrameBuffer=NULL;
  snapcount=0;
  use_png=true;
  //_expDelay=_dp->getExpiryDelay();
  bool ch,onoff;
  coarseDepthMethod=true;
 osg::Texture::setMinimumNumberOfTextureObjectsToRetainInCache(2);
 osg::Drawable::setMinimumNumberOfDisplayListsToRetainInCache(2);
  _dp-> 	getUnrefImageDataAfterApplyPolicy (ch,onoff);

  _root->accept(cmplsv);
  depthFP=NULL;
  getDepth=false;
  bboxfp=fopen("re-bbox.txt","w");
  // if(batch)
  // setSnapImageOnNextFrame(snapRes);
}

void erode(IplImage* image, int dilateSize = 3)
{
  IplConvKernel* square = cvCreateStructuringElementEx(10,10,5,5,CV_SHAPE_ELLIPSE);
	
	cvErode(image, image, square, dilateSize);
	
	return; 
}
void SnapImageDrawCallback:: setSnapImageOnNextFrame(int resolution) { 
  usleep(5000);
  snapBuffer=NULL;
 
  tileRow=tileCol=0;
  takeSnapTile=true;
  imgAllocated=false;
  totalCols=totalRows=resolution;
  _resolution=resolution;
  premove=MOVE;
  _index=0;
  firstFrame=false;
  if(_fullscreen){
    osgViewer::Viewer::Windows    windows;
   
    _viewer->getWindows(windows);
    for(osgViewer::Viewer::Windows::iterator itr = windows.begin();
	itr != windows.end();
	++itr)
      {
	osg::GraphicsContext::WindowingSystemInterface    *wsi = osg::GraphicsContext::getWindowingSystemInterface();
	
	if (wsi == NULL) 
	  {
	    osg::notify(osg::NOTICE) << "Error, no WindowSystemInterface available, cannot toggle window fullscreen." << std::endl;
	    return;
	  }
	
	unsigned int    screenWidth;
	unsigned int    screenHeight;
	
	wsi->getScreenResolution(*((*itr)->getTraits()), screenWidth, screenHeight);
	
       int x;
       int y;
       int width;
       int height;
       
       (*itr)->getWindowRectangle(x, y, width, height);
       
       bool    isFullScreen = x == 0 && y == 0 && width == (int)screenWidth && height == (int)screenHeight;
       
       if (isFullScreen)
       {
	   osg::Vec2    resolution(640,480);
	   
	   
	   (*itr)->setWindowDecoration(true);
	   (*itr)->setWindowRectangle((screenWidth - (int)resolution.x()) / 2, (screenHeight - (int)resolution.y()) / 2, (int)resolution.x(), (int)resolution.y());
	   osg::notify(osg::INFO) << "Screen resolution = " << (int)resolution.x() << "x" << (int)resolution.y() << std::endl;  
	 }
   
       
       (*itr)->grabFocusIfPointerInWindow();
       
       /* osgViewer::Viewer::Windows    windows;
   
   _viewer->getWindows(windows);
   for(osgViewer::Viewer::Windows::iterator itr = windows.begin();
       itr != windows.end();
       ++itr)
     {
       _wsh->toggleFullscreen(*itr);
     }
    */
    //    _wsh->setToggleFullscreen(true);
     }
  }
}

bool SnapImageDrawCallback:: getSnapImageOnNextFrame() const { 
  return _snapImageOnNextFrame; }

 void SnapImageDrawCallback::operator () (const osg::CameraNode& camera) const {
   int numToCompile,fileRequests;
    numToCompile=fileRequests=0;

    
  
  if (!takeSnapTile) return;
  
  /*Have to have one callback that is called pre and post move */
  if(premove == MOVE ){
    //printf("Move\n");
    self = &const_cast<osg::Camera &>(camera);
    if(!firstFrame){
      glGetFloatv(GL_DEPTH_RANGE, depthRange);
      cmplsv.changeToPixel();
      //self->getProjectionMatrixAsPerspective(fov,aspectRatio,zNear,zFar);
      firstFrame=true;
    }
    mod_view=osg::Matrixd::rotate(M_PI,1,0,0)*_passedM[_index++];
    _viewer->getCameraManipulator()->setByMatrix(mod_view);

    /*
    //0.336517 -0.137927 2.08631,
    osg::Vec3 worldPt(-18.8754, -3.25458 ,64.7271);
   
    osg::Matrix invP;
    invP=invP.inverse(_passedM[_index-1]);
    //  std::cout << invP <<std::endl;
    osg::Vec3 p1=worldPt*invP;
    std::cout << p1 <<" get on my level\n";
    double v1X,v1Y;
    libplankton::Vector sp1(4);
    sp1[0]=p1[0];
    sp1[1]=p1[1];
    sp1[2]=p1[2];
    sp1[3]=1;
    libsnapper::Stereo_Calib *calib = new libsnapper::Stereo_Calib( "s.calib" );
    calib->left_calib.ccx /= 2.0;
    calib->left_calib.ccy /= 2.0;
    calib->left_calib.fcx /= 2.0;
    calib->left_calib.fcy /= 2.0;
    
 libsnapper::camera_frame_to_undist_pixel_coords(calib->left_calib,sp1[0],sp1[1],sp1[2],v1X,v1Y);


    std::cout <<"Comapre " << v1X << " " << v1Y << " local  "<< std::endl;
    // exit(0);
//    osg::Matrix inverseVPW;
  //  inverseVPW.invert(VPW);
     
    //osg::Vec3d world = windowPt * inverseVPW;*/
    
    premove=UPDATE;
    
  }else if(premove == UPDATE){
    // printf("Update %d \n",_index);
    if(_dp){
      numToCompile=_dp->getDataToCompileListSize();
      fileRequests=_dp->getFileRequestListSize() ;
      // printf("\r%d %d",numToCompile,fileRequests);
    }
    
    if(numToCompile == 0 && fileRequests == 0){
      premove=SAVE;
    }else{
      return;
    }
  }else if(premove==SAVE){
    /*
 osg::Matrix VPW = camera.getViewMatrix() *
      camera.getProjectionMatrix() *
      camera.getViewport()->computeWindowMatrix();
    //0.336517 -0.137927 2.08631,
    osg::Vec3 worldPt(-18.8754, -3.25458 ,64.7271);
    osg::Vec3 window = worldPt * VPW;

    std::cout <<"World Pt " << worldPt << " " << "window " << window <<std::endl;
 sleep(1);
 exit(0);*/
    //   printf("Save\n");
    int x,y,width,height;
    const osg::Viewport *vp=camera.getViewport();
    x=(int)vp->x();
    y=(int)vp->y();
    width=(int)vp->width();
    height=(int)vp->height();
    if(!tileBuffer){
    
      tileBuffer=cvCreateImage(cvSize(width,height),
				     IPL_DEPTH_8U, 3);
    }
    glReadPixels(x, y, width,height, GL_BGR, GL_UNSIGNED_BYTE, tileBuffer->imageData);
    cvFlip(tileBuffer,tileBuffer,0);    
   
    if(getDepth){
      if(!depthFrameBuffer){
	depthFrameBuffer=cvCreateImage(cvSize(width,height),
				       IPL_DEPTH_8U, 1);
      }
  
      glReadPixels(0, 0, depthFrameBuffer->width,depthFrameBuffer->height, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, depthFrameBuffer->imageData);
      erode(depthFrameBuffer,4);
    }

      pasteTile();
   if(_dp){
      numToCompile=_dp->getDataToCompileListSize();
      fileRequests=_dp->getFileRequestListSize() ;
   }
   premove=MOVE;
   
  }
 
  
}


void SnapImageDrawCallback::pasteTile() const {
    const char * basename = "regen-tex";
    char ext[4],tmp[255];
    if(use_png)
      sprintf(ext,"png");
    else
      sprintf(ext,"tif");
    sprintf(tmp,"%s-%04d.%s",basename,snapcount,ext);
    _filename=std::string(tmp);
    snapcount++;
  if (!imgAllocated){
   


  

    if(getDepth){
      sprintf(tmp,"%s-%04d.%s",basename,snapcount,"hd5");
      depthImg=cvCreateImage(cvSize(tileBuffer->width * _resolution, 
				    tileBuffer->height * _resolution),
			     IPL_DEPTH_32F, 1);
    }


    if(use_png){
      
      snapBuffer=cvCreateImage(cvSize(tileBuffer->width * _resolution, 
				    tileBuffer->height * _resolution),
			       tileBuffer->depth, tileBuffer->nChannels);
   
      printf("Created Image %d x %d %.1fM\n",snapBuffer->width,snapBuffer->height,
	     (snapBuffer->width*snapBuffer->height*3.0)/1024.0/1024.0);
      texImage=cvCreateImage(cvSize(512,512),
			   tileBuffer->depth, tileBuffer->nChannels);
    }
    imgAllocated=true;
  }

  
  int x=tileBuffer->width*((totalCols-1)-tileCol) ;
   int y=tileBuffer->height*((totalRows-1)-tileRow);
   //printf("x %d ,y%d\n",x,y);
  if(use_png){
    CvRect roi;
    roi.width=tileBuffer->width;
    roi.height=tileBuffer->height;
    roi.x=x;
    roi.y=y;
    cvSetImageROI(snapBuffer,roi);
    cvCopy(tileBuffer,snapBuffer);
    cvResetImageROI(snapBuffer);

  }
  if(getDepth){
 
    
    for(int i=0; i < tileBuffer->width; i++){
      for(int j=0; j < tileBuffer->height; j++){
	
        float z=(-FLT_MAX);
	bool good=getClosestZValue(i,j,z);
	if(good)
	  CV_IMAGE_ELEM(depthImg,float,y+j,x+i)=z;
	else
          CV_IMAGE_ELEM(depthImg,float,y+j,x+i)=(-FLT_MAX);
      }
    }
  }
  
  

  tileCol++;
  
  if (tileCol >= totalCols) {
    tileCol=0;
    tileRow++;
       
    if (tileRow >= totalRows){				
      //printf("Outputting Snapshot\n");
      CvRect rec;
      if(getDepth){
	int maxIOn=INT_MIN;
	int minIOn=INT_MAX;

	int maxJOn=INT_MIN;
	int minJOn=INT_MAX;

	for(int i=0; i <depthImg->height; i++){
	  for(int j=0; j < depthImg->width; j++){
	    if(CV_IMAGE_ELEM(depthImg,float,i,j) != FLT_MIN){
	      if(i < minIOn)
		minIOn=i;
	      if(j < minJOn)
		minJOn=j;

	      if(i > maxIOn)
		maxIOn=i;
	      if(j > maxJOn)
		maxJOn=j;


	    }
	      
	  }
	}
	maxIOn=(maxIOn-(maxIOn %2));
	minIOn=(minIOn-(minIOn %2));
	maxJOn=(maxJOn-(maxJOn %2));
	minJOn=(minJOn-(minJOn %2));

	printf("%d %d %d %d\n",minIOn,maxIOn,minJOn,maxJOn);

       	rec.x=minJOn;
	rec.y=minIOn;
	rec.height=maxIOn-minIOn;
	rec.width=maxJOn-minJOn;
	printf("%d %d %d %d\n",rec.x,rec.y,rec.height,rec.width);
	IplImage *croppedDepthImg=cv_crop(depthImg,rec);
	cvReleaseImage(&depthImg);
	
	hid_t file_id=init_h5_datafile("depth.hd5","/img");
	save_cvimg_h5(croppedDepthImg,"/img/d1",file_id);
	close_h5_datafile(file_id);
	IplImage *color=cvNewColor(croppedDepthImg);
	//IplImage *tmpI=cvNewSame(croppedDepthImg);
	IplImage *mask=cvNewGray(croppedDepthImg);

       	cvCmpS(croppedDepthImg,FLT_MIN,mask,CV_CMP_NE);
	scaleJetImage(croppedDepthImg,color,mask);
	IplImage *small= cvCreateImage(cvSize((int)color->width/_resolution,(int)color->height/_resolution),color->depth,color->nChannels);
	cvResize(color,small);	
	cvSaveImage("nikes.png",small);
      }

      tileRow=0;
      tileCol=0;
      cvResize(snapBuffer,texImage);
      cvSaveImage(_filename.c_str(),texImage);
      const int count=_index-1;
      //write_pose_mat(count);

      char tmp[255];
      sprintf(tmp,"regen-tex-%04d.png",count);
      fprintf(bboxfp,"%d %s %lf %lf %lf %lf %lf %lf" ,count, tmp,
	      _bboxes[count].x1,_bboxes[count].y1,_bboxes[count].z1,
	      _bboxes[count].x2,_bboxes[count].y2,_bboxes[count].z2);
      for(int i=0; i < 4; i++)
	for(int j=0; j < 4; j++)
	  fprintf(bboxfp," %lf", _passedM[count](j,i));
      fprintf(bboxfp,"\n");


      //printf("Saved %s\n",_filename.c_str());
      premove=MOVE;
      if( _index == (int)_passedM.size()){
	fclose(bboxfp);
	_viewer->setDone(true);
	takeSnapTile=false;
      }
      
    }
  }
}
   


void SnapImageDrawCallback::myGluPerspective(GLdouble fovy, GLdouble aspect, GLdouble zNear, GLdouble zFar,osg::Camera *camera) const {
  GLdouble fLeft, fRight, fBottom, fTop, left, right, bottom, top, xDim, yDim, xOff, yOff, tDimX, tDimY;
  
  fTop = zNear * tan(fovy * M_PI / 360.0);
  fLeft = -fTop * aspect;
  fBottom = -fTop;
  fRight = -fLeft;
  
  // Dimensione totale
  xDim = fabs(fLeft * 2);
  yDim = fabs(fTop * 2);
  
  // Dimensione di un tile
  tDimX = xDim / totalCols;
  tDimY = yDim / totalRows; 
  
  // Offset del tile
  yOff = tDimY * tileRow;
  xOff = tDimX *tileCol;
  
  // Nuovo frustum
  left = fLeft + xOff;
  right = fLeft + xOff + tDimX;
  bottom = fTop - yOff - tDimY;
  top = fTop - yOff;
  
  camera->setProjectionMatrixAsFrustum (left, right, bottom, top,
					zNear, zFar);
  
}

bool SnapImageDrawCallback::getClosestZValue(int x,int y,float &z) const
{
  /*  if(coarseDepthMethod){
      float nRange = depthRange[0];
      float fRange = depthRange[1];
      // compute range based on the resulting
      // depth and the near and far clipping planes
      
      const float depthRange = fRange - nRange;
      const float clipRange = zFar - zNear;
      
      
      
      const float f1 = float(fabs((zFar * zNear			   
				   * depthRange) / clipRange));
     
      float depth=(*((float *)depthBuffer->data(x,y)));
      if(depth == zFar)
	return false;
      
      // figure range in Data Base Units
      
      z = float(fabs(f1 / (depth - (((zNear + zFar) * depthRange)
				    / (2.0f * clipRange))
			   - ((fRange + nRange) / 2.0f))));
      
  }else*/
  
  {
    unsigned char bdepth=CV_IMAGE_ELEM(depthFrameBuffer,unsigned char,y,x);
  
    if(bdepth == 255)
      return false;
   
    osgUtil::LineSegmentIntersector::Intersections intersections;

 
    if (_viewer->computeIntersections(x,y,intersections)) {
      
      osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
      if(hitr != intersections.end()) {
	z=hitr->getWorldIntersectPoint()[1];
	return true;
      }
    }
      return false;
  }
}
