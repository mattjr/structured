//
// stereo_feature_finder_test.cpp
//
// A program to test the feature finding classes by running through a set of
// images loaded from a contents file.
//
// Each line of the contents file should have the following format:
//    <timestamp> <left_image_name> <right_image_name>
//
// Ian Mahon
// 06-03-05
//

#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <sys/time.h>
#include <time.h>

#include "cv.h"
#include "highgui.h"
#include "auv_image_distortion.hpp"
#include "auv_stereo_geometry.hpp"

#include "auv_stereo_corner_finder.hpp"
#include "auv_stereo_keypoint_finder.hpp"
#include "adt_raw_file.hpp"
#include "OSGExport.h"
#include  <boost/thread/xtime.hpp> 
#include "auv_mesh_utils.hpp"
#include "auv_mesh_io.hpp"
#include "auv_lod.hpp"
using namespace std;
using namespace libplankton;
using namespace libsnapper;


//
// Command-line arguments
//

static bool have_max_mesh_count = false;
static unsigned int max_mesh_count;

static int lodNum=3;
char dicedir[255];
char mdir[255];

//
// Parse command line arguments into global variables
//
static bool parse_args( int argc, char *argv[ ] )
{

  strcpy(mdir,"mesh");
   
  int i=1;
  while( i < argc )
    {
      if( strcmp( argv[i], "-n" ) == 0 )
        {
          if( i == argc-1 ) return false;
          have_max_mesh_count = true;
          max_mesh_count = atoi( argv[i+1] );
          i+=2;
        }else if( strcmp( argv[i], "--dicedir" ) == 0 )
        {
          if( i == argc-1 ) return false;
	  strcpy(dicedir, argv[i+1]);
          i+=2;
        }else
      if( strcmp( argv[i], "--mdir" ) == 0 )
        {
          if( i == argc-1 ) return false;
	  strcpy(mdir, argv[i+1]);
          i+=2;
        }
    
      else
	{
	  cerr << "Error - unknown parameter: " << argv[i] << endl;
	  return false;
	}
    }




  return ( true);
}



// Display information on how to use this program
//
static void print_usage( void )
{
  cout << "USAGE:" << endl;
  cout << "   loggen [OPTIONS] dir" << endl; 
  cout << endl;
  cout << "OPTIONS:" << endl;
 
 
  cout << "   -n <max_frame_count>    Set the maximum number of frames to be processed." << endl;
  cout << endl;
}

int main( int argc, char *argv[ ] )
{
  //
  // Parse command line arguments
  //
  if( !parse_args( argc, argv ) )
    {
      print_usage( );
      exit( 1 );
    }
  std::vector<vector<string > > outNames;
  std::vector<osg::ref_ptr<osg::Node>  > outNodes;
  if(!have_max_mesh_count){
    char tmp[255];
    sprintf(tmp,"%s/valid.txt",dicedir);
    string dicefile(tmp);
    std::vector<string> meshNames;
    cout << tmp <<endl;
    
    struct stat BUF;
    bool have_dice=(stat(dicefile.c_str(),&BUF)!=-1);
    if(!have_dice){
      printf("Dice can't be found\n");
      exit(0);
    
    }else{
      
      FILE *dicefp=fopen(dicefile.c_str(),"r");
      char tmp[255];
      int eof=0;
      while(eof != EOF){
	eof=fscanf(dicefp,"%s\n",tmp);
	if(eof != EOF){
	  printf("Diced files %s\n",tmp);
	  meshNames.push_back("mesh-diced/"+string(tmp));
	 
	}
      }
      max_mesh_count = meshNames.size();
    }
  }

  osg::Node * lod0Node[2];

  printf("Loading %d bbox files\n",max_mesh_count);
  for(int i=0; i <  (int)max_mesh_count; i++){
  
  
    lod0Node[0]=NULL;
    lod0Node[1]=NULL;   
    char out_name[255];
    sprintf(out_name,"%s/blended-%02d-lod2-t.ive",mdir,i);
    lod0Node[0]=   osgDB::readNodeFile(string(out_name));
    sprintf(out_name,"%s/blended-%02d-lod2-u.ive",mdir,i);
    lod0Node[1]=osgDB::readNodeFile(string(out_name));
  
    sprintf(out_name,"%s/blended-%02d-lod2-t.ive",mdir,i);
 
  
  
    char text[2][255];
    strcpy(text[0],"-t.ive");
    strcpy(text[1],"-u.ive");
  
    for(int k=0; k <2; k++){
      if(lod0Node[k]){  
    
	int filecount=0;
	char out_name[255];
	vector<string> setnames;
	for(int  j=0; j < lodNum; j++){
	  sprintf(out_name,"%s/blended-%02d-lod%d%s",mdir,i,j,text[k]);

	  if(FileExists(out_name)){
	    setnames.push_back(osgDB::getSimpleFileName(out_name));
	  }
	}
	if(setnames.size() != 3 || lod0Node[k] == NULL){
	  if(i==0)
	    printf("Textured ");
	  else
	    printf("UnTextured ");
	  printf(" nodes for %d not found num found : %d\n",i,filecount);
	}else{
	  outNodes.push_back(lod0Node[k]);
	  outNames.push_back(setnames);
	}
      }
    }
  }
  
  genPagedLod(outNodes,outNames,mdir);
  
 
  return 0;
}
