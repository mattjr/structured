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
#include "auv_args.hpp"
#include <math.h>
using mapnik::Envelope;
using namespace ul;
using std::cout;
double edge_thresh;

#define PI 3.141592654

std::vector<mesh_input> meshes;
void	LoadData(std::vector<mesh_input> &meshes);
void	LoadData(char  *filename);

quadsquare*	root = NULL;
quadcornerdata	RootCornerData = { NULL, NULL, 0, 0, 0, 0, { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } } };
int	TriangleCounter = 0;
global_extents ge;
bool lod=false;

void bound_xyz( mesh_input &m,double &zmin, double &zmax){
  float data[3];
  bool first=false;
  FILE *fp=fopen(m.name.c_str(),"r");
  while(!feof(fp)){
    if(fread(data,3,sizeof(float),fp) != 3)
      break;
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
  ge.max_Level=15;
  RootCornerData.Level= ge.max_Level;
  edge_thresh=0.5;
  argp.read("-edgethresh",edge_thresh);	  
  argp.read("-input",input);
  std::string rangefile;
  bool range= argp.read("-range",rangefile);
  
  if(  argp.read("-lod"))
    lod=true;

  for(int i=0; i <3; i++){
    ge.min[i]=DBL_MAX;
    ge.max[i]=DBL_MIN;
    ge.range[i]=1.0;
  }



  FILE* fp;
  char meshname[255];
  float res;
  fp=fopen(input.c_str(),"rb");
  if(!fp){
    fprintf(stderr,"Can't open %s\n",input.c_str());
    exit(0);
  }
	  
  while(1){
    if(fscanf(fp,"%s %f %*d",meshname,&res)!=2 || feof(fp))
      break;
    mesh_input m;
    m.name=meshname;
    m.res=res;
    meshes.push_back(m);
  }
  double zmin=DBL_MAX;
  double zmax=DBL_MIN;
	
  mapnik::Envelope<double> tree_bounds;
  for(unsigned int i=0; i< meshes.size(); i++){
    if(meshes[i].name.substr(meshes[i].name.size()-3) == "ply")
      bound_mesh(meshes[i],zmin,zmax);
    else
      bound_xyz(meshes[i],zmin,zmax);

    if(i == 0)
      tree_bounds=meshes[i].envelope;
    else
      tree_bounds.expand_to_include(meshes[i].envelope);
	    
  }
	 

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
  render_no_data=true;
  LoadData(meshes);
	
	
  // Debug info.
  printf("nodes = %d\n", root->CountNodes());
  printf("max error = %g\n", root->RecomputeErrorAndLighting(RootCornerData));
  printf("Cell Size %f\n",ge.cell_size);
  // Get rid of unnecessary nodes in flat-ish areas.
  //printf("Culling unnecessary nodes (detail factor = 25)...\n");
  //root->StaticCullData(RootCornerData, 25);

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
  const float detail[]={FLT_MAX,20.0,3.0};
  // Draw the quadtree.
  if (root) {
    //		root->Update(RootCornerData, (const float*) ViewerLoc, Detail);
    if(lod){
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
void load_xyz( mesh_input &m){
  float *xyzdata=new float[m.count];
  float *ptr=xyzdata;
  FILE *fp=fopen(m.name.c_str(),"r");
  while(!feof(fp)){
    if(fread(ptr,3,sizeof(float),fp) != 3)
      break;
    ptr+=3;
  }

  HeightMapInfo	hm;
  hm.x_origin = ge.get_in_cells(m.envelope.minx()-ge.min[0],ge.max_Level);
  hm.y_origin = ge.get_in_cells(m.envelope.miny()-ge.min[1],ge.max_Level);
  /* hm.XSize = nx;
     hm.YSize = ny;
     hm.RowWidth = hm.XSize;
     hm.Scale =level;*/
  hm.Data = new uint16[hm.XSize * hm.YSize];
   
  for(int i=0; i < hm.XSize * hm.YSize; i++){
    // if(std::isnan(pout[i].z))
    if(1) hm.Data[i]=0;
    else
      ;//hm.Data[i]= ge.toUINTz(pout[i].z);
    //  printf("%d %d\n",hm.Data[i],UINT16_MAX_MINUS_ONE);
    //  printf("Final %d Source %f Rescaled %f\n",hm.Data[i],pout[i].z,(pout[i].z-zmin)/(zmax-zmin));
  }
 
 
  root->AddHeightMap(RootCornerData, hm);
  delete [] hm.Data;
  delete xyzdata;
}
void load_mesh( mesh_input &m){
  TriMesh::verbose=0;
  TriMesh *mesh = TriMesh::read(m.name.c_str());
  edge_len_thresh(mesh,edge_thresh);
  point_nn*pout;
  int nout;
  int nx,ny;
  float cx,cy;
  int level;
  double actual_res;
  interpolate_grid(mesh,m,pout,nout,nx,ny,cx,cy,actual_res,level);
  char fname[255];
  sprintf(fname,"tmp/%s",m.name.c_str());
  //  write_mesh(pout,nout,fname,true);
   
  fflush(stdout);
  // points_to_quadtree(nout,pout,qt);
  //free(&pout);
  //   printf("Nx %d Ny %d Cx %f Cy %f\n",nx,ny,cx,cy);
  HeightMapInfo	hm;
  hm.x_origin = ge.get_in_cells(m.envelope.minx()-ge.min[0],ge.max_Level);
  hm.y_origin = ge.get_in_cells(m.envelope.miny()-ge.min[1],ge.max_Level);
  //   printf("Xorigin %d Yorigin %d\n",hm.x_origin,hm.y_origin);
  hm.XSize = nx;
  hm.YSize = ny;
  hm.RowWidth = hm.XSize;
  hm.Scale =level;
  hm.Data = new uint16[hm.XSize * hm.YSize];
   
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
}
void	LoadData(std::vector<mesh_input> &meshes)
// Load some data and put it into the quadtree.
{
 
  for(unsigned int i=0; i< meshes.size(); i++){
    printf("\r %03d/%03d",i,(int)meshes.size());
    if(meshes[i].name.substr(meshes[i].name.size()-3) == "ply")
      load_mesh(meshes[i]);
    else
      load_xyz(meshes[i]);

  }
 
  printf("\r %03d/%03d\n",(int)meshes.size(),(int)meshes.size());
	

}



