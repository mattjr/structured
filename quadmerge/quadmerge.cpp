// quadtest.cpp	-thatcher 1/8/2000 Copyright Thatcher Ulrich

// Test program for quadtree adaptive heightfield.

// This code may be freely modified and redistributed.  I make no
// warrantees about it; use at your own risk.  If you do incorporate
// this code into a project, I'd appreciate a mention in the credits.
//
// Thatcher Ulrich <tu@tulrich.com>


#include <stdio.h>
#include <math.h>
#include "geometry.hpp"
#include "clip.hpp"
#include "uquadtree.hpp"
#include  <stdio.h>
#include  <stdlib.h>
#include "auv_mesh_utils.hpp" 
#include "envelope.hpp"
#include "raster.hpp"
#include "TriMesh_algo.h"
#include <osgViewer/Viewer>
#include <sys/time.h>
#include <stdlib.h> 
#include <string.h>
#include <vector>
#include <libplankton/auv_config_file.hpp>
#include "resample.hpp"
#include <GeographicConversions/ufRedfearn.h>
#include "auv_args.hpp"
#include <math.h>
#include "fileio.hpp"
#include "shadowmap.hpp"
#include "highgui.h"
#include "sample.hpp"
#include "../mesh2hmap/mesh2hmap.h"
#include "../mesh2hmap/parser.h"
using mapnik::Envelope;
using namespace ul;
using std::cout;
double edge_thresh;

using  std::cout;
using  std::endl;

#define PI 3.141592654


bool dump_stats=false;
std::vector<mesh_input> meshes;
void	LoadData(std::vector<mesh_input> &meshes);
void	LoadData(char  *filename);
bool compute_shadows=false;
quadsquare*	root = NULL;
quadcornerdata	RootCornerData = { NULL, NULL, 0, 0, 0, 0, { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } } };
int	TriangleCounter = 0;
global_extents ge;
bool lod=false;
double min_cell_size=DBL_MAX;
std::string total_stat_file;
bool have_geoconf=false;
void bound_xyz( mesh_input &m,double &zmin, double &zmax,bool ascii){
  float data[3];
  bool first=false;
  FILE *fp=fopen(m.name.c_str(),"r");
  while(!feof(fp)){

    if(!ascii){
      if(fread(data,3,sizeof(float),fp) != 3)
      break;
    }else{
      if(fscanf(fp,"%f %f %f\n",&data[0],&data[1],&data[2]) != 3)
	break;
    }
    
    if(!first){
      m.envelope=Envelope<double>(data[0],data[1],data[0],data[1]);
      first=true;
      m.count=3;
    }else{
      m.envelope.expand_to_include(data[0],data[1]);
      m.count+=3;
    }

    if( data[2] < zmin)
      zmin= data[2];
    
    if( data[2] > zmax)
      zmax= data[2];
  }

}


void bound_mesh( mesh_input &m,double &zmin, double &zmax){

  TriMesh::verbose=0;
  TriMesh *mesh = TriMesh::read(m.name.c_str());
  if(!mesh){
      fprintf(stderr, "Quadmerge: %s cannot be opened\n",m.name.c_str());
      return ;
  }
  edge_len_thresh(mesh,edge_thresh);
  mesh->need_bbox();
  m.envelope=Envelope<double>(mesh->bbox.min[0],
			      mesh->bbox.min[1],
			      mesh->bbox.max[0],
			      mesh->bbox.max[1]);
  // cout << meshes[i].envelope;
  if( mesh->bbox.min[2] < zmin)
    zmin= mesh->bbox.min[2];
  
  if( mesh->bbox.min[2] > zmax)
    zmax= mesh->bbox.max[2];
  delete mesh;

}
int	main(int argc, char *argv[])
{
  libplankton::ArgumentParser argp(&argc,argv);

  if(argp.argc() < 4){
    fprintf(stderr,"Usage quadmerge meshlistfile.txt edgethreshold outfile.ply\n");
    exit(-1);
  }
  std::string tmp,input;
  if(  argp.read("-output",tmp ));
  wf_fname = (char *)malloc(255);
  strcpy(  wf_fname,tmp.c_str());;
  ge.max_Level=12;
  RootCornerData.Level= ge.max_Level;
  edge_thresh=0.5;
  argp.read("-edgethresh",edge_thresh);	  
  argp.read("-input",input);
  std::string rangefile;
  bool range= argp.read("-range",rangefile);
  bool stat=false;
  std::string statfile;
  std::string input_statfile;
  bool input_stat=false;
  if(  argp.read("-lod"))
    lod=true;

  if(  argp.read("-shadow"))
    compute_shadows=true;
  if(  argp.read("-color"))
    apply_color_wf=true;
  if(argp.read("-stat",statfile))
    stat=true;

  if(argp.read("-istat",input_statfile)){
    input_stat=true;
  }
  if(argp.read("-zerr"))
    color_metric=Z_ERR;

  if(argp.read("-signederr"))
    color_metric=SIGNED_ERR;

  if(argp.read("-zsamples"))
    color_metric=Z_SAMPLES;

  if(argp.read("-zvar"))
    color_metric=Z_VAR;

  if(argp.read("-dumpstat",total_stat_file)){
    save_stats=true;
    dump_stats=true;
  }
  printf("Color Metric: ");
  switch(color_metric){
      case Z_SAMPLES:
	printf("Z Samples\n");
	break;
      case Z_ERR:
	printf("Z STD Err\n");
       	break;
  case Z_VAR:
    printf("Z Var\n");
    break;
      case SIGNED_ERR:
	printf("Signed Err\n");
       	break;
      }
  std::string config_file_name;
  double local_easting, local_northing;
  if(argp.read("-geoconf",config_file_name)){
 
    have_geoconf=true;
    double lat_orig,lon_orig;
   
     try {

    libplankton::Config_File config_file( config_file_name ); 
  if( config_file.get_value( "LATITUDE", lat_orig) == 0 )      {
      fprintf(stderr,"Couldn't get geoconf gloabal params\n");
      have_geoconf=false;
    }
    if( config_file.get_value( "LONGITUDE", lon_orig) == 0 ){
      fprintf(stderr,"Couldn't get geoconf gloabal params\n");
      have_geoconf=false;
    }  
     }   catch( std::string error ) {
       std::cerr << "ERROR - " << error << endl;
     exit( 1 );
   }



    cout << "Lat Origin "<<lat_orig << " Long Ori " << lon_orig<<endl;
    UF::GeographicConversions::Redfearn gpsConversion;

    double gridConvergence, pointScale;
    std::string zone;
    gpsConversion.GetGridCoordinates( lat_orig, lon_orig,
				      zone, local_easting, local_northing,
				      gridConvergence, pointScale);
  }
  for(int i=0; i <3; i++){
    ge.min[i]=DBL_MAX;
    ge.max[i]=DBL_MIN;
    ge.range[i]=1.0;
  }



  FILE* fp;
  char meshname[255];
  float res;
  int interp;
  fp=fopen(input.c_str(),"rb");
  if(!fp){
    fprintf(stderr,"Can't open %s\n",input.c_str());
    exit(0);
  }
  int idx=0;
  while(1){
    if(fscanf(fp,"%s %f %d",meshname,&res,&interp)!=3 || feof(fp))
      break;
    mesh_input m;
    m.name=meshname;
    m.res=res;
    m.interp=interp;
    m.index=idx++;
    meshes.push_back(m);
  }
  double zmin=DBL_MAX;
  double zmax=DBL_MIN;
	
  mapnik::Envelope<double> tree_bounds;
  for(unsigned int i=0; i< meshes.size(); i++){
    if(meshes[i].name.substr(meshes[i].name.size()-3) == "ply")
      bound_mesh(meshes[i],zmin,zmax);
    else if(meshes[i].name.substr(meshes[i].name.size()-3) == "grd")
      bound_grd(meshes[i],zmin,zmax,local_easting,local_northing);
    else if(meshes[i].name.substr(meshes[i].name.size()-3) == "txt")
      bound_xyz(meshes[i],zmin,zmax,true);
    else
      bound_xyz(meshes[i],zmin,zmax,false);

    if(i == 0)
      tree_bounds.init(meshes[i].envelope.minx(),
		       meshes[i].envelope.miny(),
		       meshes[i].envelope.maxx(),
		       meshes[i].envelope.maxy());
    else
      tree_bounds.expand_to_include(meshes[i].envelope);
  
  }
	 
  cout << tree_bounds<<endl;
  int	whole_cell_int_size = 2 << ge.max_Level;
  double tree_max_size=max(tree_bounds.width(),tree_bounds.height());
  ge.cell_size=tree_max_size/whole_cell_int_size;
  //	  ge.cell_size=0.1;
  ge.range[2]=zmax-zmin;
  ge.range[0]=tree_bounds.width();
  ge.range[1]=tree_bounds.height();
  ge.min[0]=tree_bounds.minx();
  ge.min[1]=tree_bounds.miny();
  ge.min[2]=zmin;

  ge.max[0]=tree_bounds.maxx();
  ge.max[1]=tree_bounds.maxy();
  ge.max[2]=zmax;
  if(range){
    FILE *rangefp=fopen(rangefile.c_str(),"w");
    if(rangefp){fprintf(rangefp,"%f %f %f\n%f %f %f\n",ge.min[0],ge.min[1],ge.min[2],ge.max[0],ge.max[1],ge.max[2]);
      fclose(rangefp);
    }else
      fprintf(stderr,"Couldn't open %s\n",rangefile.c_str());
  }
  std::cout << ge;

  RootCornerData.xorg=ge.get_in_cells(tree_bounds.minx()-tree_bounds.minx(),ge.max_Level);
  RootCornerData.yorg=ge.get_in_cells(tree_bounds.miny()-tree_bounds.miny(),ge.max_Level);
  //printf("Root Corner xorg %d yorg %d\n",RootCornerData.xorg,RootCornerData.yorg);
  root = new quadsquare(&RootCornerData);
  render_no_data=false;

  LoadData(meshes);
	
	
  // Debug info.


  printf("nodes = %d\n", root->CountNodes());
  printf("node size %d\n",(int)sizeof(quadsquare));
  printf("Memory %.2fM\n",sizeof(quadsquare)*root->CountNodes()/1024.0/1024.0);
  printf("max error = %g\n", root->RecomputeErrorAndLighting(RootCornerData));
  printf("Cell Size %f\n",ge.cell_size);
  // Get rid of unnecessary nodes in flat-ish areas.
  //printf("Culling unnecessary nodes (detail factor = 25)...\n");
  //root->StaticCullData(RootCornerData, 25);
  
  root->UpdateStats(RootCornerData);
  if(dump_stats){
 
    double meanV=mean(stat_vals);
    double medianV=median(stat_vals);

    FILE *fp=fopen("statfile.txt","w");
    //    fprintf(fp,"%f %f\n",min_stat_val,max_stat_val);
    fprintf(fp,"%f %f\n",meanV,medianV);
    fclose(fp);
    fp=fopen(total_stat_file.c_str(),"w");
    for(int i=0; i< (int)stat_vals.size(); i++)
      fprintf(fp,"%f\n",stat_vals[i]);
    fclose(fp);

  }

  // Post-cull debug info.
  /*printf("nodes = %d\n", root->CountNodes());
    printf("max error = %g\n", root->RecomputeErrorAndLighting(RootCornerData));


    // Run the update function a few times before we start rendering
    // to disable unnecessary quadsquares, so the first frame won't
    // be overloaded with tons of triangles.
    for (i = 0; i < 10; i++) {
    root->Update(RootCornerData, (const float*) ViewerLoc, Detail);
    }
  */
  if(compute_shadows){
    color_metric=SHADOWED;   
    int lmsize=2048;//4096;
    int level;
    double actual_res;
    double res=ge.range[0]/lmsize;
    ge.get_closest_res_level(res,level,actual_res);
    int nny=ge.range[0]/actual_res;
    int nnx=ge.range[1]/actual_res;
    //    int size=max(nny,nnx);
    float *heightmap= new float[nnx*nny];
    unsigned char *heightmap2= new unsigned char[nnx*nny];

    int xstart=  0;
    int xend=  2 << RootCornerData.Level;
   
    int xstep = max((xend-xstart)/max(nny,nnx),1);
    
    int ystart=0;
    int yend= 2 << RootCornerData.Level;
    
    
    int ystep = max((yend-ystart)/max(nny,nnx),1);
    int ct=0;
    for(int i=0,x=xstart; i< nnx; i++,x+=xstep){
      for(int j=0,y=ystart; j<nny; j++,y+=ystep){
	float height=root->GetHeight(RootCornerData, x,y);
	
	
	heightmap[i*nny+j]=height;
	heightmap2[i*nny+j]=(height/(float)UINT16_MAX_MINUS_ONE) *255;
	if(height!=0.0){
	  ct++;
	} 
      }
    }
    printf("Count %d\n",ct);
    HeightMapInfo	hm;
    hm.x_origin = 0;
    hm.y_origin = 0;

    hm.XSize = nnx;
    hm.YSize = nny;

    hm.RowWidth = hm.XSize;
    hm.Scale =level;
    hm.Data = new float[hm.XSize * hm.YSize];
    
    IplImage *lmimg=cvCreateImageHeader(cvSize(nny,nnx),IPL_DEPTH_8U,1);
    unsigned char *lightmap= new unsigned char[nnx*nny];
    ul::vector Sun(128.0f, 512.0f, 256.0f);
    calc_shadow(heightmap2,lightmap,nny,nnx,Sun);
    int pt=0;
    for(int i =0; i < nnx; i++)
      for(int j=0; j < nny; j++){
	hm.Data[j*nnx +i]=lightmap[pt];
	pt++;
      }
  
 
    lmimg->imageData=(char *) lightmap;
    cvSaveImage("lightmap.png",lmimg);
    root->AddShadowMap(RootCornerData, hm);

  }
  const float detail[]={FLT_MAX,800000.0,100000.0};
  std::stringstream title;
   int discrete=0;
  if (root) {
    if(input_stat){

      FILE *fp = fopen(input_statfile.c_str(),"r");
      fscanf(fp,"%lf %lf %*d\n%*s\n",&min_stat_val,
	      &max_stat_val);
      fclose(fp);  
    }
    if( stat){
   
      min_cell_size=ge.cell_size;
     
      title.setf(std::ios::fixed, std::ios::floatfield);
      title.precision(5);
      switch(color_metric){
      case Z_SAMPLES:
	discrete=1;

	title<<"Number of samples";
	break;
      case Z_ERR:
	title << "Standard Dev Err [m]";

	break;
	case SIGNED_ERR:
	  title << "Signed Err [m]";
	  break;

	case Z_VAR:
	  title << "Var Z [m]";
	  break;
	  
      }
      FILE *fp = fopen(statfile.c_str(),"w");
      fprintf(fp,"%f %f %d\n%s\n",min_stat_val,max_stat_val,discrete,title.str().c_str());
      fclose(fp);  
    }
    if(input_stat)
      printf("Loading previously saved stat file for color range %f %f\n",min_stat_val,max_stat_val);  
    //		root->Update(RootCornerData, (const float*) ViewerLoc, Detail);
    if(lod){  // Draw the quadtree.
      std::string name(wf_fname);
      for(int i=0; i < 3; i++){
	char tmp[255];
	sprintf(tmp,"-lod%d.ply",i);
	std::string str(name);
	int pos=str.find(".ply");
	str.replace(pos, 4, std::string(tmp) );
	strcpy(wf_fname,str.c_str());
	std::cout << "Writing " << wf_fname << std::endl;
	//	      root->StaticUpdate(RootCornerData,detail[i]);
	if(i > 0){
	  for(int j=0; j< 20; j++)
	    //	  root->Update(RootCornerData, (const float*) ViewerLoc, detail[i]);  
	    root->StaticUpdate(RootCornerData,detail[i]);
	}
	root->RenderToWF(RootCornerData);
      }
    }
    else
      root->RenderToWF(RootCornerData);
  }
  return 0;

}
void load_xyz( mesh_input &m,bool ascii){
  float *xyzdata=new float[m.count];
  float *ptr=xyzdata;
  FILE *fp=fopen(m.name.c_str(),"r");
  float data[3];
  while(!feof(fp)){
    if(!ascii){
      if(fread(data,3,sizeof(float),fp) != 3)
      break;
    }else{
      if(fscanf(fp,"%f %f %f\n",&data[0],&data[1],&data[2]) != 3)
	break;
    }
    memcpy(ptr,data,sizeof(float)*3);
    ptr+=3;
  }
  point_nn*pout;
  int nout;
  int nx,ny;
  float cx,cy;
  int level;
  double actual_res;

 interpolate_grid(xyzdata,m,pout,nout,nx,ny,cx,cy,actual_res,level);
HeightMapInfo	hm;
  hm.x_origin = ge.get_in_cells(m.envelope.minx()-ge.min[0],ge.max_Level);
  hm.y_origin = ge.get_in_cells(m.envelope.miny()-ge.min[1],ge.max_Level);
  //printf("Xorigin %d Yorigin %d\n",hm.x_origin,hm.y_origin);
  hm.XSize = nx;
  hm.YSize = ny;
  hm.RowWidth = hm.XSize;
  hm.Scale =level;
  hm.Data = new float[hm.XSize * hm.YSize];
  hm.index=m.index;
   
  for(int i=0; i < hm.XSize * hm.YSize; i++){
    // printf("%f ",pout[i].z);
    if(std::isnan(pout[i].z))
      hm.Data[i]=0;
    else
      hm.Data[i]= ge.toUINTz(pout[i].z);
    //  printf("%d %d\n",hm.Data[i],UINT16_MAX_MINUS_ONE);
    //  printf("Final %d Source %f Rescaled %f\n",hm.Data[i],pout[i].z,(pout[i].z-zmin)/(zmax-zmin));
  }
  
   
  //  root->AddHeightMap(RootCornerData, hm);
  root->AddHeightMap(RootCornerData, hm);
  delete [] hm.Data;
  
  free(pout);
  delete xyzdata;
}
void load_mesh( mesh_input &m){

  TriMesh::verbose=0;
  TriMesh *mesh = TriMesh::read(m.name.c_str());
  if(!mesh){
    fprintf(stderr, "Quadmerge: %s cannot be opened\n",m.name.c_str());
    return;
  }
  edge_len_thresh(mesh,edge_thresh);
  point_nn*pout;
  int nout;
  int nx,ny;
  float cx,cy;
  int level;
  double actual_res;
  interpolate_grid(mesh,m,pout,nout,nx,ny,cx,cy,actual_res,level);
  if(actual_res < min_cell_size)
    min_cell_size=actual_res;
  char fname[255];
  sprintf(fname,"tmp/%s",m.name.c_str());
  //  write_mesh(pout,nout,fname,true);
   

  // points_to_quadtree(nout,pout,qt);
  //free(&pout);
  //   printf("Nx %d Ny %d Cx %f Cy %f\n",nx,ny,cx,cy);
  HeightMapInfo	hm;
  hm.x_origin = ge.get_in_cells(m.envelope.minx()-ge.min[0],ge.max_Level);
  hm.y_origin = ge.get_in_cells(m.envelope.miny()-ge.min[1],ge.max_Level);
  //printf("Xorigin %d Yorigin %d\n",hm.x_origin,hm.y_origin);
  hm.XSize = nx;
  hm.YSize = ny;
  hm.RowWidth = hm.XSize;
  hm.Scale =level;
  hm.Data = new float[hm.XSize * hm.YSize];
  hm.index=m.index;
  for(int i=0; i < hm.XSize * hm.YSize; i++){
    // printf("%f ",pout[i].z);
    if(std::isnan(pout[i].z))
      hm.Data[i]=0;
    else
      hm.Data[i]= ge.toUINTz(pout[i].z);
    //  printf("%d %d\n",hm.Data[i],UINT16_MAX_MINUS_ONE);
    //  printf("Final %d Source %f Rescaled %f\n",hm.Data[i],pout[i].z,(pout[i].z-zmin)/(zmax-zmin));
  }
  
   
  //  root->AddHeightMap(RootCornerData, hm);
  root->AddHeightMap(RootCornerData, hm);
  delete [] hm.Data;
  delete mesh;  
  free(pout);
}


void load_mesh_nointerp( mesh_input &m){

  TriMesh::verbose=0;
  TriMesh *mesh = TriMesh::read(m.name.c_str());
  if(!mesh){
    fprintf(stderr, "Quadmerge: %s cannot be opened\n",m.name.c_str());
    return;
  }
  edge_len_thresh(mesh,edge_thresh);
  
  mesh_t hmmesh;
  trimesh2mesh(&hmmesh, mesh);
  int x, y, z, invert;
  float  x_m_pix, y_m_pix;
  int width, height;
  width = height = 0;
  
  invert = 0;
  z = 2; x = 1; y = 0;

  int level;
  double actual_res;

  ge.get_closest_res_level(m.res,level,actual_res);
  x_m_pix = y_m_pix = actual_res;
  
  if(actual_res < min_cell_size)
    min_cell_size=actual_res;
  hmap_t hmap;
  
  hmap.rows = height;
  hmap.cols = width;
  mesh2hmap(&hmap, &hmmesh, x, y, z, invert, x_m_pix, y_m_pix);
  fmatrix_free(hmmesh.vert);
  irowarray_free(hmmesh.poly, hmmesh.num_poly);
  
  HeightMapInfo	hm;
  hm.x_origin = ge.get_in_cells(m.envelope.minx()-ge.min[0],ge.max_Level);
  hm.y_origin = ge.get_in_cells(m.envelope.miny()-ge.min[1],ge.max_Level);
  //printf("Xorigin %d Yorigin %d\n",hm.x_origin,hm.y_origin);
  hm.XSize = hmap.cols;
  hm.YSize = hmap.rows;
  hm.RowWidth = hm.XSize;
  hm.Scale =level;
  hm.Data = new float[hm.XSize * hm.YSize];
  hm.index=m.index;
  int cnt=0;
  for(int i=0; i < hm.YSize; i++){
    for(int j=0; j <  hm.XSize; j++){
    // printf("%f ",pout[i].z);
    if(hmap.map[i][j] == -HUGE_VAL)
      hm.Data[cnt]=0;
    else
      hm.Data[cnt]= (hmap.map[i][j] + hmap.z_min);
    //     printf("%f\n",hm.Data[cnt]);
    //  printf("Final %d Source %f Rescaled %f\n",hm.Data[i],pout[i].z,(pout[i].z-zmin)/(zmax-zmin));
    cnt++;
    }
  }
  
   
  //  root->AddHeightMap(RootCornerData, hm);
  root->AddHeightMap(RootCornerData, hm);
  delete [] hm.Data;
  delete mesh;  

}
void load_grd( mesh_input &m){

  if(!have_geoconf){
    fprintf(stderr,"Config file with origin needed for grd processing\n");
    exit(-1);
  }
  short nx,ny;
  int level;
  double actual_res;
  float *data_grd;
  if(!read_grd_data(m.name.c_str(),nx,ny,data_grd))
  return;
  ge.get_closest_res_level(m.res,level,actual_res);
  if(actual_res < min_cell_size)
    min_cell_size=actual_res;
  /*Flip X and Y*/
  int nnx=m.envelope.height()/actual_res;
  int nny=m.envelope.width()/actual_res;
  //Flip x and y
 if(nnx> nx || nny > ny){
    fprintf(stderr,"Supposed to be downsampling AHHHHH!\n");
  }

  float *data =new float[nnx*nny];
  //printf("New/Old  X: %d/%d Y: %d/%d  %f/%f\n",nx,nnx,ny,nny,m.res,actual_res);
 
  bool flipx=true;
  
  downsampleArray(data_grd,nx,ny,data,nnx,nny,flipx);
  GMT_free ((void *)data_grd);
 
  HeightMapInfo	hm;
  if(flipx){
    hm.x_origin = ge.get_in_cells(m.envelope.minx()-ge.min[0],ge.max_Level);
    hm.y_origin = ge.get_in_cells(m.envelope.miny()-ge.min[1],ge.max_Level);
  }
  else{
    hm.x_origin = ge.get_in_cells(m.envelope.minx()-ge.min[0],ge.max_Level);
    hm.y_origin = ge.get_in_cells(m.envelope.miny()-ge.min[1],ge.max_Level);
  }
  //   printf("Xorigin %d Yorigin %d\n",hm.x_origin,hm.y_origin);
  if(flipx){
    hm.XSize = nny;
    hm.YSize = nnx;
  }else{
    hm.XSize = nnx;
    hm.YSize = nny;
  }
  hm.RowWidth = hm.XSize;
  hm.Scale =level;
  hm.Data = new float[hm.XSize * hm.YSize];
  hm.index=m.index;

    for(int i=0; i < hm.XSize * hm.YSize; i++){
    if(std::isnan(data[i]))
      hm.Data[i]=0;
    else
      hm.Data[i]= ge.toUINTz(data[i]);
  }
    
    root->AddHeightMap(RootCornerData, hm,true); 
  delete [] hm.Data;
  delete data;
}
void	LoadData(std::vector<mesh_input> &meshes)
// Load some data and put it into the quadtree.
{
 
  for(unsigned int i=0; i< meshes.size(); i++){
    printf("\r %03d/%03d",i,(int)meshes.size());
    fflush(stdout);
    if(meshes[i].name.substr(meshes[i].name.size()-3) == "ply"){
      if(meshes[i].interp)
	load_mesh(meshes[i]);
      else
	load_mesh_nointerp(meshes[i]);
    }
    else    if(meshes[i].name.substr(meshes[i].name.size()-3) == "grd")
      load_grd(meshes[i]);
    else   if(meshes[i].name.substr(meshes[i].name.size()-3) == "txt")
      load_xyz(meshes[i],true);
    else
      load_xyz(meshes[i],false);

  }
 
  printf("\r %03d/%03d\n",(int)meshes.size(),(int)meshes.size());
	

}



