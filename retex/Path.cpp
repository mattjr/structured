#include "Path.h"
#include <fstream>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osg/Point>
#include <iostream>
using namespace std;

std::vector<osg::Matrixd>   PathLoader::createPath(const char *path )
{
 

  std::fstream camfile;

  

  camfile.open(path);
  std::vector<osg::Matrixd> camMat;
  
  if(!camfile){
    fprintf(stderr,"Error Open file %s\n",path);
    
    return camMat;
  }
 


  
  while (!camfile.fail()){
    osg::Matrixd m;
    double timestamp;
    camfile >> timestamp ;
    for(int i=0; i < 4; i++){
      for(int j=0; j <4; j++){
	camfile >> m(j,i);
      }
    }
  
    if (!camfile.fail()){ 
       //      GOOD
       camMat.push_back(m);
    }   
  }
  printf("Number cam poses %d\n",camMat.size());

 
 
  return camMat; 
    
}
