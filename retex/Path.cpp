#include "Path.h"
#include <fstream>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osg/Point>
#include <iostream>
using namespace std;

void PathLoader::createPath(const char *path, std::vector<osg::Matrixd> &camMat,std::vector<bbox> &bboxes  )
{
 

  std::fstream camfile;

  

  camfile.open(path);

  
  if(!camfile){
    fprintf(stderr,"Error Open file %s\n",path);
    
   
  }
  int skip=2;
  int count=skip;

  bool first=true;
  
  while (!camfile.fail()){
    osg::Matrixd m;
    double timestamp;
    bbox b;
    int idx;
      string ln,rn;
    camfile >> idx ;
    camfile >> timestamp ;
    camfile >> ln;
    camfile >> rn;
    camfile >> b.x1;
    camfile >> b.y1;
    camfile >> b.z1;
    camfile >> b.x2;
    camfile >> b.y2;
    camfile >> b.z2;

    for(int i=0; i < 4; i++){
      for(int j=0; j <4; j++){
	camfile >> m(j,i);
      }
    }
  
  if (!camfile.fail()){ 
       //      GOOD
      

      if(count != skip){
	count ++;
	continue;
      }
      count=0;
       camMat.push_back(m);
       bboxes.push_back(b);
     
    }   
  }
  printf("Number cam poses %d\n",camMat.size());

 
 
  
    
}
