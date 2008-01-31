#include "Depth.h"

DepthStats::DepthStats(TriMesh *mesh):_mesh(mesh){


}


osg::Texture2D* newColorTexture2D(unsigned width, unsigned height, unsigned accuracy)
{
  osg::Texture2D* texture2D = new osg::Texture2D;
  
  texture2D->setTextureSize(width, height);
  if(accuracy == 32)
    {
      texture2D->setInternalFormat(GL_RGBA32F_ARB);
      texture2D->setSourceFormat(GL_RGBA);
    }
  else if(accuracy == 8)
    {
      texture2D->setInternalFormat(GL_RGBA);
    }
  texture2D->setSourceType(GL_FLOAT);
  texture2D->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
  texture2D->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);

  osg::Image* image = new osg::Image;
  float* data = new float[width*height*4];
  
  for(unsigned y=0; y < height; y++)
    for(unsigned x=0; x < width; x++){
     
      for(int i=0; i<4; i++)
	data[(y*width*4) + (x*4) +i] = 0.66666;;

    }
  image->setImage(width,height,1,GL_RGBA32F_ARB, GL_RGBA, GL_FLOAT, (unsigned char*)&data[0], osg::Image::USE_NEW_DELETE);
  texture2D->setImage(image);
  return texture2D;
}

 
osg::TextureRectangle* ass(unsigned width, unsigned height, unsigned accuracy)
{
  osg::TextureRectangle* textureRectangle = new osg::TextureRectangle;
  
  textureRectangle->setTextureSize(width, height);
  if(accuracy == 32)
    {
      textureRectangle->setInternalFormat(GL_RGBA32F_ARB);
      textureRectangle->setSourceFormat(GL_RGBA);
    }
  else if(accuracy == 8)
    {
      textureRectangle->setInternalFormat(GL_RGBA);
    }
  textureRectangle->setSourceType(GL_FLOAT);
  

  osg::Image* image = new osg::Image;
  float* data = new float[width*height*4];
  
  for(unsigned y=0; y < height; y++)
    for(unsigned x=0; x < width; x++){
      /*    if(y==0){
	for(int i=0; i<4; i++)
	if(i==3)
	  data[(y*width*4) + (x*4) +i] = 0.66600;
	else
	  data[(y*width*4) + (x*4) +i] = 0.33330;
      }else{
for(int i=0; i<4; i++)
	if(i==3)
	  data[(y*width*4) + (x*4) +i] = 0.111111;
	else
	  data[(y*width*4) + (x*4) +i] = 0.888888;
	  }*/
      for(int i=0; i<4; i++)
	data[(y*width*4) + (x*4) +i] = -299.666f;

    }
  image->setImage(width,height,1,GL_RGB32F_ARB, GL_RGB, GL_FLOAT, (unsigned char*)&data[0], osg::Image::USE_NEW_DELETE);
  textureRectangle->setImage(image);
  return textureRectangle;
}

osg::TextureRectangle*  getPlaneTex( vector<Plane3D> planes,int size){
 
  
  if(size*size < (int)planes.size()){
    fprintf(stderr,"Not enought tex space for planes gonna fail\n");
    return NULL;
  }
   
  osg::TextureRectangle *textureRectangle=newColorTextureRectangle(size,size,32);
  //printf("planes Size %d tex  size  %d x %d = %d\n",planes.size(),_planeTexSize,_planeTexSize,_planeTexSize*_planeTexSize);
  osg::Image* image = new osg::Image;
  float* data = new float[size*size*4];
  /*
  for(unsigned y=0; y < height; y++)
    for(unsigned x=0; x < width; x++){
      
      for(int i=0; i<4; i++)
	data[(y*width*4) + (x*4) +i] = -207.66666;

	}*/
  int height=size;
  int width=size;
  bzero(data,height*width*sizeof(float));
 
  float *ptr=data;
  int counter=0;
  for(int i=0; i< (int)planes.size(); i++){
    *ptr=planes[i].u[0];
    ptr++;
    *ptr=planes[i].u[1];
    ptr++;
    *ptr=planes[i].u[2];
    ptr++;
    *ptr=planes[i].d0;
    ptr++;
    cout << counter++ << "ss ";
    
    planes[i].info(); 
  }
   
 image->setImage(size,size,1,GL_RGBA32F_ARB, GL_RGBA, GL_FLOAT, (unsigned char*)&data[0], osg::Image::USE_NEW_DELETE);
  textureRectangle->setImage(image);
  return textureRectangle;

}
osg::TextureRectangle* newColorTextureRectangle(unsigned width, unsigned height, unsigned accuracy)
{
  osg::TextureRectangle* textureRectangle = new osg::TextureRectangle;
  
  textureRectangle->setTextureSize(width, height);
  if(accuracy == 32)
    {
      textureRectangle->setInternalFormat(GL_RGBA32F_ARB);
      textureRectangle->setSourceFormat(GL_RGBA);
    }
  else if(accuracy == 8)
    {
      textureRectangle->setInternalFormat(GL_RGBA);
    }
  textureRectangle->setSourceType(GL_FLOAT);



  return textureRectangle;
}

osg::Vec3Array* displayPlane(Plane3D m_BiggerPlane3D,Point3D center)
{
  /*  int nSize = m_pPoints->size();
  Point3D* ppoint;
 /
  
  for (int i=0; i< nSize; i++) {
    ppoint = &((*m_pPoints)[i]);
    glColor3f(0.0,0.0,1.0);// blue points
    glBegin(GL_POINTS);
        if (ppoint->isGoodPoint())
	  glVertex3f(ppoint->x(), ppoint->y(), ppoint->z());
    glEnd();
  }
 
  
  
  float x,y,z;
  float w = 5;
  int nSize2=1000;*/
  int type = -1;
  double u0=fabs(m_BiggerPlane3D.u[0]);
  double u1=fabs(m_BiggerPlane3D.u[1]);
  double u2=fabs(m_BiggerPlane3D.u[2]);
 
  if (u0>=u1 && u0>=u2)
    type = 0;
  else if (u1>=u0 && u1>=u2)
    type = 1;
  else if (u2>=u0 && u2>=u1)
    type = 2;
  else {
    std::cout << " ERRROORORORORROROROROORORRORORORO"
	      << std::endl;
    return NULL;
  }
 
  double x_min=10e10, x_max=-10e10;
  double y_min=10e10, y_max=-10e10;
  double z_min=10e10, z_max=-10e10;
 
  /*for (int i=0; i< nSize; i++) {
    ppoint = &((*m_pPoints)[i]);
    if (ppoint->isGoodPoint()) {
      x = ppoint->x();
      y = ppoint->y();
      z = ppoint->z();
 
      x_min = MIN(x_min,x);
      y_min = MIN(y_min,y);
      z_min = MIN(z_min,z);
      x_max = MAX(x_max,x);
      y_max = MAX(y_max,y);
      z_max = MAX(z_max,z);
    }
  }
  */
  float margin=0.1;
  
  x_min=center[0]-margin;
  y_min=center[1]-margin;
  z_min=center[2]-margin;

  x_max=center[0]+margin;
  y_max=center[1]+margin;
  z_max=center[2]+margin;

 
  Point3D p1,p2,p3,p4;
  double aux;
  switch(type) {
  case 0://x
    aux = -(m_BiggerPlane3D.u[1]*y_min+m_BiggerPlane3D.u[2]*z_min+
	    m_BiggerPlane3D.d0)/m_BiggerPlane3D.u[0];
    p1.setCoord(aux,y_min,z_min);
    aux = -(m_BiggerPlane3D.u[1]*y_max+m_BiggerPlane3D.u[2]*z_min+
	    m_BiggerPlane3D.d0)/m_BiggerPlane3D.u[0];
    p2.setCoord(aux,y_max,z_min);
    aux = -(m_BiggerPlane3D.u[1]*y_max+m_BiggerPlane3D.u[2]*z_max+
	    m_BiggerPlane3D.d0)/m_BiggerPlane3D.u[0];
    p3.setCoord(aux,y_max,z_max);
    aux = -(m_BiggerPlane3D.u[1]*y_min+m_BiggerPlane3D.u[2]*z_max+
	    m_BiggerPlane3D.d0)/m_BiggerPlane3D.u[0];
    p4.setCoord(aux,y_min,z_max);
    break;
  case 1:
    aux = -(m_BiggerPlane3D.u[0]*x_min+m_BiggerPlane3D.u[2]*z_min+
	    m_BiggerPlane3D.d0)/m_BiggerPlane3D.u[1];
    p1.setCoord(x_min,aux,z_min);
    aux = -(m_BiggerPlane3D.u[0]*x_max+m_BiggerPlane3D.u[2]*z_min+
	    m_BiggerPlane3D.d0)/m_BiggerPlane3D.u[1];
    p2.setCoord(x_max,aux,z_min);
    aux = -(m_BiggerPlane3D.u[0]*x_max+m_BiggerPlane3D.u[2]*z_max+
	    m_BiggerPlane3D.d0)/m_BiggerPlane3D.u[1];
    p3.setCoord(x_max,aux,z_max);
    aux = -(m_BiggerPlane3D.u[0]*x_min+m_BiggerPlane3D.u[2]*z_max+
	    m_BiggerPlane3D.d0)/m_BiggerPlane3D.u[1];
    p4.setCoord(x_min,aux,z_max);
    break;
  case 2:
    aux = -(m_BiggerPlane3D.u[0]*x_min+m_BiggerPlane3D.u[1]*y_min+
	    m_BiggerPlane3D.d0)/m_BiggerPlane3D.u[2];
    p1.setCoord(x_min,y_min,aux);
    aux = -(m_BiggerPlane3D.u[0]*x_max+m_BiggerPlane3D.u[1]*y_min+
	    m_BiggerPlane3D.d0)/m_BiggerPlane3D.u[2];
    p2.setCoord(x_max,y_min,aux);
    aux = -(m_BiggerPlane3D.u[0]*x_max+m_BiggerPlane3D.u[1]*y_max+
	    m_BiggerPlane3D.d0)/m_BiggerPlane3D.u[2];
    p3.setCoord(x_max,y_max,aux);
    aux = -(m_BiggerPlane3D.u[0]*x_min+m_BiggerPlane3D.u[1]*y_max+
	    m_BiggerPlane3D.d0)/m_BiggerPlane3D.u[2];
    p4.setCoord(x_min,y_max,aux);
    break;
  }
  osg::Vec3Array* vec =new osg::Vec3Array;
  vec->push_back(osg::Vec3(p1.x(),
			  p1.y(),
			  p1.z()));
  vec->push_back(osg::Vec3(p2.x(),
			  p2.y(),
			  p2.z()));
  vec->push_back(osg::Vec3(p3.x(),
			  p3.y(),
			  p3.z()));
  vec->push_back(osg::Vec3(p4.x(),
			  p4.y(),
			  p4.z()));
  
  return vec;
}
vector<int> * DepthStats::getPlaneFits(vector<Plane3D> &planes, vector<TriMesh::BBox> &bounds,int widthSplits,int heightSplits){
  int nv = _mesh->vertices.size();
  _mesh->need_bbox();
  vec v=_mesh->bbox.size();
  cout << v << endl;
  cout << _mesh->bbox.min << " " << _mesh->bbox.max << endl;

  double xstep = v[0] /widthSplits;
  double ystep = v[1]/ heightSplits;
  vector<int> *planeIdx = new vector<int>;
  planeIdx->resize(nv);

  for(int wS=0; wS < widthSplits; wS++){
    for(int hS=0; hS <heightSplits; hS++){
      TriMesh::BBox stepbbox;
     Plane3D plane3D,plane3D_2;
     RansacPlane m_RansacPlane;
     std::vector<bool> inliers;
     unsigned int nInlierCount = 0;
     double dbModelScore;
     m_RansacPlane.setMaxIterationNumber(25);
     m_RansacPlane.setInlierDistanceThreshold(0.001);
     plane3D.clear();
     std::vector<Point3D> m_pPoints;
     std::vector<int> pointIndex;
     int count=0;
     stepbbox.min[0] = _mesh->bbox.min[0] + (xstep * wS);
     stepbbox.min[1]= _mesh->bbox.min[1] + (ystep * hS);
     
     stepbbox.max[0] = _mesh->bbox.min[0] + (xstep*(wS+1));
     stepbbox.max[1]= _mesh->bbox.min[1] + (ystep*(hS+1));
     
     stepbbox.min[2]= _mesh->bbox.min[2];
     stepbbox.max[2]= _mesh->bbox.max[2];
     //   cout << stepbbox.min << stepbbox.max<< " " <<(xstep * wS) << " " <<(ystep * hS) <<  endl;
     for (int i = 0; i < nv; i++){
       if (_mesh->vertices[i][0] >= stepbbox.min[0] &&
	   _mesh->vertices[i][0] <= stepbbox.max[0] &&
	   _mesh->vertices[i][1] >= stepbbox.min[1] &&
	   _mesh->vertices[i][1] <= stepbbox.max[1] ){
	 Point3D pt(_mesh->vertices[i][0],
		    _mesh->vertices[i][1],
		    _mesh->vertices[i][2]);
	 m_pPoints.push_back(pt);
	 pointIndex.push_back(count++);
       }
     }
     if(!pointIndex.size())
       continue;
     // --- Ransac ---------
     // --------------------
     
     nInlierCount = 0;
     inliers.clear();
     m_RansacPlane.setData(&m_pPoints, &pointIndex);
     int nReturn = m_RansacPlane.Algorithm(0.5, plane3D, inliers,
					   nInlierCount, dbModelScore);
     if (nReturn <= 0) // No model or error
       continue;
     
     
     //cout << std::endl << std::endl
     // << "New cell : return of ransac " << nReturn << std::endl;
     if (nInlierCount==0) {
       std::cout << std::endl << std::endl
		 << "no plane was found " << std::endl;
       continue;
     }
     else {
       // std::cout << "N# of inliers " << nInlierCount
       //     << "  N# of points " << pointIndex.size()
       //     << std::endl;
     }
     
     //std::cout << "Plane Score " << dbModelScore << std::endl;
     
     cout << planes.size() << " "; plane3D.info();
    
     // --- Min Least Square  -------
     // -----------------------------
     m_RansacPlane.bestPlaneMinLeastSquares(pointIndex, plane3D_2);
     double dbScore_2 = 0;
     int nCount = 0;
     double d;
     for (unsigned int k=0; k< nInlierCount; k++) {
       d = plane3D_2.signedDistance(m_pPoints[pointIndex[k]]);
       dbScore_2 += fabs(d);
       
       if (fabs(d) < m_RansacPlane.getInlierDistanceThreshold())
	 nCount++;
     }
     dbScore_2 /= nInlierCount;
     if (dbScore_2 < dbModelScore) {
       // MinLeastSquare Plane is better
       dbModelScore = dbScore_2;
       plane3D = plane3D_2;
       //std::cout << "--------------------------------MLQE" << std::endl;
     }

     
     planes.push_back(plane3D);
     bounds.push_back(stepbbox);
     for (int i = 0; i < nv; i++){
       if (_mesh->vertices[i][0] >= stepbbox.min[0] &&
	   _mesh->vertices[i][0] <= stepbbox.max[0] &&
	   _mesh->vertices[i][1] >= stepbbox.min[1] &&
	   _mesh->vertices[i][1] <= stepbbox.max[1] ){
	 (*planeIdx)[i]=(planes.size()-1);
       }
     }
     
    }
  }
  return planeIdx;
}
