//
// stereo_feature_finder_test.cpp
//
// A program to test the feature finding classes by running through a set of
// images loaded from a contents file.
//
// Each line of the contents file should have the following format:
//    <timestamp> <left_image_name> <right_image_name>
//
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
#include "adt_file_utils.hpp"
#include "OSGExport.h"
#include  <boost/thread/xtime.hpp> 
#include "auv_mesh_utils.hpp"
#include "auv_mesh_io.hpp"
#include "Depth.h"

using namespace std;
using namespace libplankton;
using namespace libsnapper;

static bool tex_array_blend=false;
//
// Command-line arguments
//
static bool applyNonVisMat=false;
static string stereo_config_file_name;
static string recon_config_file_name;
static bool shader_color=false;
static string contents_file_name;
static string dir_name;
static int verbose=false;
static bool no_simp=false;
static     bool output_3ds=false;
static bool have_max_mesh_count = false;
static unsigned int max_mesh_count;
static int num_threads=1;
static bool display_debug_images = true;
static int nonvisidx=0;
static bool compress_textures = true;
static bool single_run=false;
static int single_run_index=0;
static string classes_file;
static bool do_classes=false;
static int max_class_id=0;
static std::map<string,int> classes;
static int lodNum=3;
static bool do_atlas=true;
static string stereo_calib_file_name;
static std::map<int,std::string> texture_file_names;
static bool hardware_compress=true;
static float tex_scale=1.0;
static  char diceddir[255];
extern std::vector<GtsBBox *> bboxes_all;
static bool do_novelty=false;
static std::vector<GtsMatrix *>gts_inv_ptr;
static bool usePlaneDist=false;
int margins[]={10,500,INT_MAX};
static bool use_proj_tex=false;
static bool use_regen_tex=false;
static bool range_run=false;
static int endRun=0;
static bool no_tex=false;
static bool use_dist_coords=true;
int lodTexSize[3];
int lodStart=0;
bool passed_calib=false;
//
// Parse command line arguments into global variables
//
static bool parse_args( int argc, char *argv[ ] )
{
  bool have_stereo_config_file_name = false;
  bool have_recon_config_file_name = false;


  strcpy(diceddir,"mesh-diced/");
     stereo_calib_file_name = "stereo.calib";
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
  else if( strcmp( argv[i], "--shader" ) == 0 )
	{
     
	  shader_color=true;
	  i+=1;
	}
      else if( strcmp( argv[i], "--notex" ) == 0 )
	{
     
	  no_tex=true;
	  i+=1;
	}
      else if( strcmp( argv[i], "--margins" ) == 0 )
	{
	  if( i == argc-3 ) return false;
	  margins[0]=atoi( argv[i+1]);
	  margins[1]=atoi( argv[i+2]);
	  margins[2]=atoi( argv[i+3]);
	  i+=4;
	}
      else if( strcmp( argv[i], "--nosimp" ) == 0 )
	{
	  no_simp=true;
	  i+=1;
	}
      else if( strcmp( argv[i], "--no-atlas" ) == 0 )
	{
	  do_atlas=false;
	  i+=1;
	}
      else if( strcmp( argv[i], "--projtex" ) == 0 )
	{
	  use_proj_tex=true;
	  do_atlas=false;
	  i+=1;
	}
      else if( strcmp( argv[i], "--classes" ) == 0 )
	{
	  do_classes=true;
	  classes_file=argv[i+1];
	  i+=2;
	}

      else if( strcmp( argv[i], "--novelty" ) == 0 )
	{
	  do_novelty=true;
	  i+=1;
	}
      else if( strcmp( argv[i], "--planedist" ) == 0 )
	{
	  usePlaneDist=true;
	  i+=1;
	}
      else if( strcmp( argv[i], "--3ds" ) == 0 )
	{
	  output_3ds=true;
	  i+=1;
	}
      else if( strcmp( argv[i], "--blend" ) == 0 )
	{
	  tex_array_blend=true;
	  i+=1;
	}
      else if( strcmp( argv[i], "--no-compress-tex" ) == 0 )
	{
	  compress_textures=false;
	  i+=1;
	}
      else if( strcmp( argv[i], "--stereo-calib" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  passed_calib=true;
	  stereo_calib_file_name = argv[i+1];
	  i+=2;
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
      else if( strcmp( argv[i], "--nonvis" ) == 0 )
	{
	  applyNonVisMat=true;
	  if( i == argc-1 ) return false;
	  nonvisidx=atoi( argv[i+1] );
	  i+=2;
	}
      else if( strcmp( argv[i], "--regen" ) == 0 )
	{
	  use_regen_tex=true;
	  use_dist_coords=false;
	  i++;
	}
      else if( strcmp( argv[i], "--single-run" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  single_run_index = atoi( argv[i+1] );
	  i+=2;
	  single_run=true;
	}
      else if( strcmp( argv[i], "--range-run" ) == 0 )
	{
	  if( i == argc-2 ) return false;
	  single_run_index = atoi( argv[i+1] );
	  endRun = atoi( argv[i+2] );
	  i+=3;
	  range_run=true;
	}
      else if( strcmp( argv[i], "--lod-start" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  lodStart= atoi( argv[i+1] );
	  i+=2;
	}
      else if( strcmp( argv[i], "--lods" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	  lodNum= atoi( argv[i+1] );
	  i+=2;
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
      else if( strcmp( argv[i], "--dicedir" ) == 0 )
	{
	  if( i == argc-1 ) return false;
	 
	  strcpy(diceddir,argv[i+1] );
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
      else if( !have_recon_config_file_name )
	{
	  recon_config_file_name = argv[i];
	  have_recon_config_file_name = true;
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

GNode *loadBBox(const char *str,std::map<int,GtsMatrix *> &gts_trans_map){
  char conf_name[255];
  if(use_regen_tex)
    sprintf(conf_name,"mesh-regen-tex/re-bbox.txt");
  // sprintf(conf_name,"mesh-regen-tex/re-bbox-%s.txt",str);
  else
    sprintf(conf_name,"%s/bbox-%s.txt",diceddir,str);

  FILE *bboxfp = fopen(conf_name,"r");
  int count;
  GNode *bboxTree=NULL;
  GSList * bboxes = NULL;
  if(bboxfp){
    char name[255];
    double x1,x2,y1,y2,z1,z2;
    GtsMatrix *mtmp=gts_matrix_identity(NULL);
    int eof0=1, eof1=1,eof2=1,frame_count=0;
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
  //printf("No bbox file bailing...\n");
  return NULL;


}
gboolean mesh_count ( int number, int total,int lod,int maxlod,int coarseper,int tex,int textotal){
  if(verbose)
    return true;
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
  cout << "    --no-atlas           Don't Do  Texture Atlas" << endl;
  cout << "    --no-hardware-compress      Software Texture Compress" << endl;
  cout << "    --nosimp      Don't Simplify" << endl;
  cout << "    -v      Verbose" << endl;
  cout << "    -vv      Very Verbose" << endl;
  cout << "    --3ds      3ds output" << endl;
  cout << "    --lods <num>       Num of lods to output" << endl;
  cout << "    -f <imagedir> Image dir prefix" << endl;
  cout << "    -t <num_threads> Num threads" << endl;
  cout << "    --regen Use regen tex" << endl;
  
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
  Config_File *config_file,*recon_config_file;
  try
    {
      config_file = new Config_File( stereo_config_file_name );
    }
  catch( string error )
    {
      cerr << "ERROR - " << error << endl;
      exit( 1 );
    }

  try {
    recon_config_file= new Config_File(recon_config_file_name.c_str());
  }   catch( string error ) {
    cerr << "ERROR - " << error << endl;
    exit( 1 );
  }

  recon_config_file->get_value( "TEX_SIZE_LOD0", lodTexSize[0],
				max((int)(512*tex_scale),4) );

  recon_config_file->get_value( "TEX_SIZE_LOD1", lodTexSize[1],
				max((int)(256*tex_scale),4) );
  
  recon_config_file->get_value( "TEX_SIZE_LOD2", lodTexSize[2] ,
				max((int)(16*tex_scale),4));
  



  recon_config_file->set_value( "TEX_MARGIN_LOD0" , margins[0] );
  recon_config_file->set_value( "TEX_MARGIN_LOD1" , margins[1] );
  recon_config_file->set_value( "TEX_MARGIN_LOD2" , margins[2] );


  config_file->set_value( "SKF_SHOW_DEBUG_IMAGES"    , display_debug_images );
  config_file->set_value( "SCF_SHOW_DEBUG_IMAGES"    , display_debug_images );

  


  //
  // Load the stereo camera calibration 
  //
  Stereo_Calib *calib = NULL;
  bool have_stereo_calib = false;

  //if( config_file->get_value( "STEREO_CALIB_FILE", stereo_calib_file_name) )
  {

  if(!passed_calib)
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

 

 

  char mdir[255];
  char subdir[255];
  if(use_regen_tex)
    strcpy(mdir,"mesh-blend");
  else
    strcpy(mdir,"mesh");
  auv_data_tools::makedir(mdir);
  chmod(mdir,   0777);
  strcpy(subdir,"lod");
  FILE *fpp=fopen(string(string(mdir)+"/shaderout.txt").c_str(),"w");
  if(fpp){
    if(do_novelty)
      fprintf(fpp,"%d\n",6);
    else
      fprintf(fpp,"%d\n",4);
    
    fclose(fpp);
  }


  string fulllodpath=string(mdir) + "/"+string(subdir);
  auv_data_tools::makedir(fulllodpath.c_str());
  chmod(fulllodpath.c_str(),   0777);
  if(num_threads > 1)
    g_thread_init (NULL);
 
  float zrange[2];
  std::map<int,GtsMatrix *> gts_trans_map;
  char rangefname[255];
  sprintf(rangefname,"%s/range.txt",diceddir);
  FILE *fp =fopen(rangefname,"r");
  if(!fp){
    fprintf(stderr,"No range.txt file Quitting\n");
    exit(-1);
  }
  fscanf(fp,"%*f %*f %f\n%*f %*f %f\n",&(zrange[0]),&(zrange[1]));
  fclose(fp);
 fpp=fopen(string(string(mdir)+"/scalarbar.txt").c_str(),"w");
  if(fpp){
    fprintf(fpp,"%d %d Depth_[m]\n",(int)zrange[0],(int)zrange[1]);
    
    fclose(fpp);
  }
  if(do_classes){
    FILE *fp =fopen(classes_file.c_str(),"r");
    if(!fp){
      fprintf(stderr,"No classes file %s file Quitting\n",classes_file.c_str());
      exit(-1);
    }
    int class_id;
   
    char img_name[2048];
    while(!feof(fp)){
      fscanf(fp,"%*f %s %d\n",img_name,&class_id);
      classes[string(img_name)]=class_id;
      if(class_id > max_class_id)
	max_class_id=class_id;
    }
    fclose(fp);
    printf("Loaded %d class labels with max id %d\n",classes.size(),max_class_id);
  }
  char dicefname[255];
  sprintf(dicefname,"%s/valid.txt",diceddir);
  string dicefile(dicefname);
  std::vector<string> meshNames;
  std::vector<vector<string >   > outNames;
  std::vector<osg::ref_ptr<osg::Node>  > outNodes;

  struct stat BUF;
  bool have_dice=(stat(dicefile.c_str(),&BUF)!=-1);
  if(!have_dice && single_run){
    fprintf(stderr,"Error need dice for single run\n");
    return(-1);
  }
    
  if(!have_dice){
    fprintf(stderr,"No dice file\n");
    return -1;
    
  }else{
    
    FILE *dicefp=fopen(dicefile.c_str(),"r");
    char tmp[255];
    int eof=0;
    while(eof != EOF){
      eof=fscanf(dicefp,"%s\n",tmp);
      if(eof != EOF){
	if(verbose)
	  printf("Diced files %s\n",tmp);
	meshNames.push_back(diceddir+string(tmp));
      }
    }
  }

 

  int totalMeshCount,startRun;
  if(single_run){
    verbose=true;
    startRun=single_run_index;
    totalMeshCount=single_run_index+1;
  }
  else if(range_run){
    startRun=single_run_index;
    totalMeshCount=min((int)meshNames.size(),endRun);
  }else{
    startRun=0;
    if(have_max_mesh_count )
      totalMeshCount=max_mesh_count;
    else
      totalMeshCount=meshNames.size();
  }
  
  int i;
  for( i=startRun; i < (int) totalMeshCount; i++){
    GNode *bboxTree=NULL;
    if(!no_tex)
      bboxTree=loadBBox(osgDB::getSimpleFileName(meshNames[i]).c_str(),
			gts_trans_map);
    else 
      bboxTree=NULL;   

    /*if(!bboxTree){ 
      fprintf(stderr,"Failed to load bboxtree\n");
      exit(-1);
      }
    */
   
    //int initialEdges=gts_surface_edge_number(s);

 
    //float simpRatio[]={0.5,0.1,0.01};
    
    std::vector<string > lodnames;
    string path=string(argv[0]);
    unsigned int loc=path.rfind("/");
   
    string basepath= loc == string::npos ? "./" : path.substr(0,loc+1);
    basepath= osgDB::getRealPath (basepath) +"/";
    if(!mgc)
      mgc = new MyGraphicsContext();
    OSGExporter *osgExp=new OSGExporter(dir_name,false,compress_textures,
					num_threads,verbose,hardware_compress,tex_array_blend,do_novelty,basepath,usePlaneDist,(applyNonVisMat && i == nonvisidx),use_proj_tex,do_atlas,shader_color);

    osgExp->setCalib(&calib->left_calib);
    osg::Node * lod0Node[2];
    lod0Node[0]=NULL;
    lod0Node[1]=NULL;
    char out_name[255];

    for(int j=lodStart; j < lodNum; j++){
      boost::function< bool(int) > coarsecallback = boost::bind(mesh_count,i,totalMeshCount,j,lodNum,_1,0,0);
      string str=meshNames[i];
      if(!no_simp){
	char tmp[255];
	sprintf(tmp,"-lod%d.ply",j);
	int pos=str.find(".ply");
	str.replace(pos, 4, string(tmp) );
      }
      if(verbose)
	printf("Loading Surface %s ...\n",str.c_str());
       
      GtsSurface *surf = gts_surface_new(gts_surface_class(),
					 (GtsFaceClass *) t_face_class(),
					 gts_edge_class(), t_vertex_class());
      TriMesh::verbose=verbose;
      if(!file_exists(str))
	continue;
      TriMesh *mesh = TriMesh::read(str.c_str());
      if(!mesh){
	fprintf(stderr,"Empty mesh skipping\n");
	continue;
      }
      vector<int> *planeIdx=NULL;
      vector<Plane3D> planes;
      osg::Matrix *rot=NULL;
      vector<TriMesh::BBox> bounds;
      double width_target=1.5;
      double height_target=2.0;


      if(usePlaneDist){
	DepthStats ds(mesh);
	rot=new osg::Matrixd();
	//	planeIdx=ds.getPlaneFits(planes,bounds,rot,5,3);
	if(lodNum ==2)
	  planeIdx=new vector<int>;
	else
	  planeIdx=ds.getPlaneFits(planes,bounds,rot,width_target,height_target,10);
      }
      bool res=convert_ply(mesh,surf,verbose,planeIdx);
      mesh_count(i,totalMeshCount,j,lodNum,0,0,0);
      delete planeIdx;
      if(!res ){
	printf("Failed to load surface %s\n",
	       meshNames[i].c_str());
	exit(-1);
      }
      if(verbose)
	printf("Done Loaded %d Verts %d Edges\n",gts_surface_vertex_number(surf),
	       gts_surface_edge_number(surf));
      /* GtsSurface *surf = gts_surface_new(gts_surface_class(),
	 (GtsFaceClass*)t_face_class(),
	 gts_edge_class(), t_vertex_class());
      
	 int currentEdges=gts_surface_edge_number(s);
	 if(!no_simp && bboxTree){
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
	 }*/
       
      boost::xtime xt, xt2;
      long time;
      double secs;
      boost::function<bool(int,int)> texcallback = boost::bind(mesh_count,i,totalMeshCount,j,lodNum,100,_1,_2);
      if(bboxTree){

	
	if(verbose)
	  printf("Gen texture coordinates\n");

      
	boost::xtime_get(&xt, boost::TIME_UTC);
    
		gen_mesh_tex_coord(surf,&calib->left_calib,gts_trans_map,bboxTree,
		   lodTexSize[j],num_threads,verbose,tex_array_blend,margins[j],use_dist_coords);
	boost::xtime_get(&xt2, boost::TIME_UTC);
	time = (xt2.sec*1000000000+xt2.nsec - xt.sec*1000000000 - xt.nsec) / 1000000;
	secs=time/1000.0;
	//	if(verbose)
	  printf("Done Took %.2f secs\n",secs);
      }
      if(verbose)
	printf("Converting to model for export\n");
      
      boost::xtime_get(&xt, boost::TIME_UTC);
      
      char ext[5];
  
      if(output_3ds)
	strcpy(ext,"3ds");
      else
	strcpy(ext,"ive");
      sprintf(out_name,"%s/%s/blended-%04d-lod%d.%s",mdir,subdir,i,j,ext);
      osg::ref_ptr<osg::Geode> group[2];
      ClippingMap cm;
      std::map<int,GtsMatrix *>::iterator iter;
      std::map<int,osg::Matrixd> *camMatrices=NULL;
      if(use_proj_tex){
	camMatrices= new  std::map<int,osg::Matrixd>;
	for(iter=gts_trans_map.begin(); iter!=gts_trans_map.end(); iter++){
	  GtsMatrix *m=iter->second;
	  osg::Matrixd osgm;
	  for(int i=0; i < 4; i++){
	    for(int j=0; j <4; j++){
	      osgm(i,j)=m[i][j];
	    }
	  }
	  (*camMatrices)[iter->first]=osgm;
	
	}
      }
      osg::Group *toggle_ptr=NULL;
      std::map<string,int> *	class_ptr=NULL;
      if(do_classes)
       	class_ptr=&classes;
      if(usePlaneDist)
	toggle_ptr=new osg::Group();
      osgExp->convertGtsSurfListToGeometry(surf,texture_file_names,&cm,
					   lodTexSize[j],group,planes,bounds,rot,
					   texcallback,zrange,camMatrices,
					   class_ptr,max_class_id,toggle_ptr);
    

      
      osgExp->outputModelOSG(out_name,group,toggle_ptr);

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
    //if(s)
    //gts_object_destroy (GTS_OBJECT (s)); 
    // if(bboxTree)
    //gts_bb_tree_destroy(bboxTree,true);
   

    bboxes_all.clear();
    delete osgExp;
    
  }
  mesh_count(i,totalMeshCount,lodNum,lodNum,0,0,0);

  // 
  // Clean-up
  //
  std::map<int,GtsMatrix *>::iterator iter;
  for(iter=gts_trans_map.begin(); iter!=gts_trans_map.end(); iter++){
    if(iter->second)
      gts_matrix_destroy(iter->second);
  }

  delete config_file;
  delete recon_config_file;
  delete calib;

  return 0;
}
