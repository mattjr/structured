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
#include "auv_stereo_ncc_corner_finder.hpp"
#include "auv_stereo_keypoint_finder.hpp"
#include "adt_raw_file.hpp"
#include "OSGExport.h"
#include  <boost/thread/xtime.hpp> 
#include "auv_mesh_utils.hpp"
#include "auv_mesh_io.hpp"

using namespace std;
using namespace libplankton;
using namespace libsnapper;


//
// Command-line arguments
//

static bool have_max_mesh_count = false;
static unsigned int max_mesh_count;

static int lodNum=3;

//
// Parse command line arguments into global variables
//
static bool parse_args( int argc, char *argv[ ] )
{

  
   
  int i=1;
  while( i < argc )
    {
      if( strcmp( argv[i], "-n" ) == 0 )
        {
          if( i == argc-1 ) return false;
          have_max_mesh_count = true;
          max_mesh_count = atoi( argv[i+1] );
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
  vector< std::vector<vector<string > > >outNames;
   std::vector<osg::ref_ptr<osg::Group>  > outNodes;
  if(!have_max_mesh_count){
    string dicefile("mesh-agg/diced.txt");
    std::vector<string> meshNames;
 
    
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
	  meshNames.push_back("mesh-agg/"+string(tmp));
	 
	}
      }
     max_mesh_count = meshNames.size();
    }
  }

  vector <string> texname,untexname;

  printf("Loading %d bbox files\n",max_mesh_count);
  for(int i=0; i <  (int)max_mesh_count; i++){
    vector <std::vector<string > > lodnames;
    for(int j=0; j < lodNum; j++){
      
      char out_name[255];
      
      sprintf(out_name,"mesh/blended-%02d-lod%d.ive",i,j);
      char outtex[255];
      char outuntex[255];
      string outname_str(out_name);
      strcpy(outtex,(outname_str.substr(0,outname_str.length()-4)+string("-t.ive")).c_str());
      
      texname.push_back(osgDB::getSimpleFileName(outtex));
      strcpy(outuntex,(outname_str.substr(0,outname_str.length()-4)+string("-u.ive")).c_str());
      untexname.push_back(osgDB::getSimpleFileName(outuntex));
      osg::Group * group=new osg::Group;
      // if(FileExists(out_name)){
      {


	if(j == (lodNum -1)){
	  osg::Group *group1=dynamic_cast<osg::Group*>(osgDB::readNodeFile(string(outtex)));

	    osg::Group *group2=dynamic_cast<osg::Group*>(osgDB::readNodeFile(string(outuntex)));
	    if(group1->getNumChildren())
	      group->addChild(group1->getChild(0));
	    if(group2->getNumChildren())
	      group->addChild(group2->getChild(0));
	  outNodes.push_back(group);
	}
      }
      
    }
    if(outNodes.size()){
      lodnames.push_back(untexname);
	lodnames.push_back(texname);	

	outNames.push_back(lodnames);
    }
    
  }
  
  
 
  genPagedLod(outNodes,outNames);
  
 
  return 0;
}
