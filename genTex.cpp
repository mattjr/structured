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
static string stereo_config_file_name;
static string contents_file_name;
static string dir_name;
static int verbose=false;
static bool no_simp=false;

static bool have_max_mesh_count = false;
static unsigned int max_mesh_count;
static int num_threads=1;
static bool display_debug_images = true;

static bool compress_textures = true;

static int lodNum=3;
static string stereo_calib_file_name;


static std::map<int,std::string> texture_file_names;
static bool hardware_compress=true;
static float tex_scale=1.0;

extern std::vector<GtsBBox *> bboxes_all;

//
// Parse command line arguments into global variables
//
static bool parse_args( int argc, char *argv[ ] )
{
  bool have_stereo_config_file_name = false;
  
   
  int i=1;
  while( i < argc )
    {
      if( strcmp( argv[i], "-r" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  tex_scale = atof( argv[i+1] );
	  i+=2;
	}
      else if( strcmp( argv[i], "-f" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  dir_name=string( argv[i+1]) ;
	  i+=2;
	}
      else if( strcmp( argv[i], "--nosimp" ) == 0 )
	{
	  no_simp=true;
	  i+=1;
	}
      else if( strcmp( argv[i], "--no-compress-tex" ) == 0 )
	{
	  compress_textures=false;
	  i+=1;
	}
      else if( strcmp( argv[i], "--no-hardware-compress" ) == 0 )
	{
	  hardware_compress=false;
	  i+=1;
	}
      else if( strcmp( argv[i], "-v" ) == 0 )
	{
	  verbose=true;
	  i+=1;
	}
      else if( strcmp( argv[i], "-vv" ) == 0 )
	{
	  verbose=true;
	  osg::setNotifyLevel(osg::INFO);
	  i+=1;
	}
      else if( strcmp( argv[i], "-n" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  have_max_mesh_count = true;
	  max_mesh_count = atoi( argv[i+1] );
	  i+=2;
	}
      else if( strcmp( argv[i], "-t" ) == 0 )
      {
         if( i == argc-1 ) return false;
         num_threads = atoi( argv[i+1] );
	 i+=2;
      }
      else if( !have_stereo_config_file_name )
	{
	  stereo_config_file_name = argv[i];
	  have_stereo_config_file_name = true;
	  i++;
	}
   
      else
	{
	  cerr << "Error - unknown parameter: " << argv[i] << endl;
	  return false;
	}
    }




  return ( have_stereo_config_file_name);
}
osg::Node *create_paged_lod(osg::Node * model,vector<string> lod_file_names){

  float cut_off_distance = 25.0f;
    float max_visible_distance = 150.0f;
    float max_dist=1e7;

  const osg::BoundingSphere& bs = model->getBound();
  if (bs.valid()){

    printf("%s dist: %g - %g\n\t%s dist: %g - %g\n\t%s dist: %g - %g\n",lod_file_names[0].c_str(),max_visible_distance,max_dist,lod_file_names[1].c_str(),cut_off_distance,max_visible_distance,lod_file_names[2].c_str(),0.0,cut_off_distance);  
    
    osg::PagedLOD* pagedlod = new osg::PagedLOD;

    pagedlod->setDatabasePath("");
    pagedlod->setCenter(bs.center());
    pagedlod->setRadius(bs.radius());
    pagedlod->setNumChildrenThatCannotBeExpired(2);
    
    pagedlod->setRange(0,max_visible_distance,max_dist);
    pagedlod->addChild(model);
    
    pagedlod->setRange(1,cut_off_distance,max_visible_distance);
    pagedlod->setFileName(1,lod_file_names[1]);
 
    pagedlod->setRange(2,0.0f,cut_off_distance);
    pagedlod->setFileName(2,lod_file_names[0]);
   
   
    return pagedlod;
  }
  return NULL;
}
void genPagedLod(vector< osg::ref_ptr <osg::Group> > nodes, vector< vector<string> > lodnames){
  osg::Group *total=new osg::Group;
  printf("Final Paged LOD Hierarchy\n");
  for(int i=0; i < (int)nodes.size(); i++){
  
    osg::Node *tmp=create_paged_lod(nodes[i].get(),lodnames[i]);
    total->addChild(tmp);
  }
  osgDB::ReaderWriter::WriteResult result = osgDB::Registry::instance()->writeNode(*total,"mesh/final.ive",osgDB::Registry::instance()->getOptions());

 if (result.success())	{
    osg::notify(osg::NOTICE)<<"Data written to '"<<"mesh/final.ive" <<"'."<< std::endl;

     
   
  }
  else if  (result.message().empty()){
    osg::notify(osg::NOTICE)<<"Warning: file write to '"<<"mesh/final.ive" <<"' no supported."<< std::endl;
  }

}
GNode *loadBBox(int num,std::map<int,GtsMatrix *> &gts_trans_map){
  char conf_name[255];
  
  sprintf(conf_name,"mesh-agg/bbox-%08d.txt",num);
  
  FILE *bboxfp = fopen(conf_name,"r");
  int count;
  GNode *bboxTree=NULL;
  GSList * bboxes = NULL;
  if(bboxfp){
    char name[255];
    double x1,x2,y1,y2,z1,z2;
    GtsMatrix *mtmp=gts_matrix_identity(NULL);
    int eof0, eof1,eof2,frame_count=0;
    while (eof0 != EOF && eof1 != EOF && eof2 != EOF){
      
      eof0 = fscanf(bboxfp,"%d %s %lf %lf %lf %lf %lf %lf" ,&count, name,
	     &x1,&y1,&z1,&x2,&y2,&z2);
      if(eof0 == EOF)
	break;
      for(int i=0; i < 4; i++)
	for(int j=0; j < 4; j++)
	 eof1 = fscanf(bboxfp," %lf",&mtmp[i][j]);
     eof2 = fscanf(bboxfp,"\n");
      
     
      
      texture_file_names[count]=(name);
      GtsBBox *bbox= gts_bbox_new(gts_bbox_class(),NULL,x1,y1,z1,x2,y2,z2);
      bbox->bounded=(void *)count;
      bboxes= g_slist_prepend (bboxes,bbox);
      bboxes_all.push_back(bbox);
      gts_trans_map[count]=gts_matrix_inverse(mtmp);
      frame_count++;
    }
    gts_matrix_destroy(mtmp);
    fclose(bboxfp);
    if(bboxes)
      bboxTree=gts_bb_tree_new(bboxes);
    return bboxTree;
  }
    printf("No bbox file bailing...\n");
    return NULL;


}
gboolean mesh_count ( int number, int total,int lod,int maxlod,int coarseper,int tex,int textotal){
  if (number == total) {
    boost::call_once(&timer_destroy, once2);
    fprintf (stderr, "\n");
    fflush (stderr);
    return TRUE;
  }
  
  if (timer == NULL) {
    boost::call_once(&timer_init, once);
    nmax = nold = number;
  }
  
  {// && number % 1 == 0 ){//&& number < nmax && nmax > total) {
    gdouble total_elapsed = g_timer_elapsed (total_timer, NULL);
    gdouble remaining;
    gdouble hours, mins, secs;
    gdouble hours1, mins1, secs1;
    
    g_timer_stop (timer);
    
    hours = floor (total_elapsed/3600.);
    mins = floor ((total_elapsed - 3600.*hours)/60.);
    secs = floor (total_elapsed - 3600.*hours - 60.*mins);
    
    remaining = ((total_elapsed/(gdouble)number) *((gdouble)total-number));
    hours1 = floor (remaining/3600.);
    mins1 = floor ((remaining - 3600.*hours1)/60.);
    secs1 = floor (remaining - 3600.*hours1 - 60.*mins1);
    
    fprintf (stderr, 
	     "\rMesh %2d/%2d LOD %d/%d, Simp: %3d%% Tex %d/%d "
	     "Elap: %02.0f:%02.0f:%02.0f "
	     "Rem: %02.0f:%02.0f:%02.0f",
	     number+1, total,lod+1,maxlod, coarseper,tex,textotal,
	     //   100.*( number)/( total),
	     //g_timer_elapsed (timer, NULL)/ (number - nold  ),
	     hours, mins, secs,
	     hours1, mins1, secs1);
    fflush (stderr);

    nold = number;
    g_timer_start (timer);
  }
 
  return FALSE;
}
//
// Display information on how to use this program
//
static void print_usage( void )
{
  cout << "USAGE:" << endl;
  cout << "   stereo_feature_finder_test [OPTIONS] <stereo_cfg> " << endl; 
  cout << endl;
  cout << "OPTIONS:" << endl;
  cout << "   -r <resize_scale>       Resize the images by a scaling factor." << endl;
 
  cout << "   -n <max_frame_count>    Set the maximum number of frames to be processed." << endl;
  cout << "    --no-compress-tex           Don't Compress Textures" << endl;
  cout << "    --no-hardware-compress      Software Texture Compress" << endl;
  cout << "    --nosimp      Don't Simplify" << endl;
  cout << "    -v      Verbose" << endl;
  cout << "    -vv      Very Verbose" << endl;
  cout << "    -f <imagedir> Image dir prefix" << endl;
  cout << "    -t <num_threads> Num threads" << endl;
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

  //
  // Figure out the directory that contains the config file 
  //
  string config_dir_name;
  int slash_pos = stereo_config_file_name.rfind( "/" );
  if( slash_pos != -1 )
    config_dir_name = stereo_config_file_name.substr( 0, slash_pos+1 );


  //
  // Open the config file
  // Ensure the option to display the feature finding debug images is on.
  //
  Config_File *config_file;
  try
    {
      config_file = new Config_File( stereo_config_file_name );
    }
  catch( string error )
    {
      cerr << "ERROR - " << error << endl;
      exit( 1 );
    }
  config_file->set_value( "SKF_SHOW_DEBUG_IMAGES"    , display_debug_images );
  config_file->set_value( "SCF_SHOW_DEBUG_IMAGES"    , display_debug_images );
  config_file->set_value( "NCC_SCF_SHOW_DEBUG_IMAGES", display_debug_images );

  


  //
  // Load the stereo camera calibration 
  //
  Stereo_Calib *calib = NULL;
  bool have_stereo_calib = false;
   
  if( config_file->get_value( "STEREO_CALIB_FILE", stereo_calib_file_name) )
    {
      stereo_calib_file_name = config_dir_name+stereo_calib_file_name;
      try
	{
	  calib = new Stereo_Calib( stereo_calib_file_name );
	}
      catch( string error )
	{
	  cerr << "ERROR - " << error << endl;
	  exit( 1 );
	}
      have_stereo_calib = true;
    }      

 

 


  auv_data_tools::makedir("mesh");
  if(num_threads > 1)
    g_thread_init (NULL);
 
  float zrange[2];
  std::map<int,GtsMatrix *> gts_trans_map;
  FILE *fp =fopen("mesh-agg/range.txt","r");
  fscanf(fp,"%*f %*f %f\n%*f %*f %f\n",&(zrange[0]),&(zrange[1]));
  fclose(fp);

  string dicefile("mesh-agg/diced.txt");
  std::vector<string> meshNames;
  std::vector<vector<string > > outNames;
  std::vector<osg::ref_ptr<osg::Group>  > outNodes;

  struct stat BUF;
  bool have_dice=(stat(dicefile.c_str(),&BUF)!=-1);
  if(!have_dice){
   
    meshNames.push_back( "mesh-agg/out.ply");
    
  }else{
    
    FILE *dicefp=fopen(dicefile.c_str(),"r");
    char tmp[255];
    int eof=0;
    while(eof != EOF){
      eof=fscanf(dicefp,"%s\n",tmp);
      if(eof != EOF){
	if(verbose)
	  printf("Diced files %s\n",tmp);
	meshNames.push_back("mesh-agg/"+string(tmp));
      }
    }
  }

 

  int totalMeshCount;
  if(have_max_mesh_count )
    totalMeshCount=max_mesh_count;
  else
    totalMeshCount=meshNames.size();
  int i;
  for( i=5; i < (int) totalMeshCount; i++){
    
   
    if(verbose)
      printf("Loading Surface %s ...\n",meshNames[i].c_str());
  
    GtsSurface *s = gts_surface_new(gts_surface_class(),
					(GtsFaceClass *) t_face_class(),
					 gts_edge_class(), t_vertex_class());
    bool res=read_ply(meshNames[i].c_str(),s,verbose);
    GNode *bboxTree=loadBBox(i,gts_trans_map);
    if(!res ){
      printf("Failed to load surface %s\n",
	     meshNames[i].c_str());
      exit(-1);
    }
    if(verbose)
      printf("Done Loaded %d Verts %d Edges\n",gts_surface_vertex_number(s),
	     gts_surface_edge_number(s));
    int initialEdges=gts_surface_edge_number(s);

    int lodTexSize[]={max((int)(512*tex_scale),32),max((int)(256*tex_scale),32),max((int)(32*tex_scale),32)};
    float simpRatio[]={0.5,0.1,0.01};
    std::vector<string> lodnames;
    OSGExporter *osgExp=new OSGExporter(dir_name,false,compress_textures,
					num_threads,verbose,hardware_compress);    
   
    for(int j=0; j < lodNum; j++){
       boost::function< bool(int) > coarsecallback = boost::bind(mesh_count,i,totalMeshCount,j,lodNum,_1,0,0);

    
       mesh_count(i,totalMeshCount,j,lodNum,0,0,0);
      GtsSurface *surf = gts_surface_new(gts_surface_class(),
					 (GtsFaceClass*)t_face_class(),
					 gts_edge_class(), t_vertex_class());
      
      int currentEdges=gts_surface_edge_number(s);
      if(!no_simp){
	int targetEdges=(int)(initialEdges*simpRatio[j]);

	if(verbose){
	  printf("LOD %d Surface has %d edges downsampling to %d\n",
		 j,currentEdges,targetEdges);
	  printf("Coarsen...\n");
	}
	
	coarsen(s,targetEdges,verbose,coarsecallback);
	
	if(verbose)
	  printf("Done\n");
	gts_surface_copy(surf,s);
      }
      boost::xtime xt, xt2;
      long time;
      double secs;
      boost::function<bool(int,int)> texcallback = boost::bind(mesh_count,i,totalMeshCount,j,lodNum,100,_1,_2);
      if(bboxTree){

	
	if(verbose)
	  printf("Gen texture coordinates\n");

      
	boost::xtime_get(&xt, boost::TIME_UTC);
    
	gen_mesh_tex_coord(surf,&calib->left_calib,gts_trans_map,bboxTree,
			   lodTexSize[j],num_threads,verbose);
	boost::xtime_get(&xt2, boost::TIME_UTC);
	time = (xt2.sec*1000000000+xt2.nsec - xt.sec*1000000000 - xt.nsec) / 1000000;
	secs=time/1000.0;
	if(verbose)
	  printf("Done Took %.2f secs\n",secs);
      }
      if(verbose)
	printf("Converting to model for export\n");
      
      boost::xtime_get(&xt, boost::TIME_UTC);
      char out_name[255];
      
      sprintf(out_name,"mesh/blended-%02d-lod%d.ive",i,j);
      
      osg::ref_ptr<osg::Group> node =osgExp->convertModelOSG(surf,texture_file_names,
							     out_name,lodTexSize[j],texcallback,zrange);
      
      lodnames.push_back(osgDB::getSimpleFileName(string(out_name)).c_str());
      if(j == (lodNum -1))
	outNodes.push_back(node);

      boost::xtime_get(&xt2, boost::TIME_UTC);
      time = (xt2.sec*1000000000+xt2.nsec - xt.sec*1000000000 - xt.nsec) / 1000000;
      secs=time/1000.0;
      //Destory Surf
      if(surf)
	gts_object_destroy (GTS_OBJECT (surf)); 
      if(verbose)
	printf("Done Took %.2f secs\n",secs);
    }
    //Destory Surf
    if(s)
      gts_object_destroy (GTS_OBJECT (s)); 
    outNames.push_back(lodnames);
    delete osgExp;
  }
  mesh_count(i,totalMeshCount,lodNum,lodNum,0,0,0);

  if(have_dice && lodNum > 1){
    genPagedLod(outNodes,outNames);
  }
  // 
  // Clean-up
  //
  //for(int i=0; i < (int)gts_trans.size(); i++)
  // gts_matrix_destroy(gts_trans[i]);
  delete config_file;
  delete calib;

  return 0;
}
