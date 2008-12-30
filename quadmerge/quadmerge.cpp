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


using mapnik::Envelope;
using namespace ul;
using std::cout;
double edge_thresh;

#define PI 3.141592654

std::vector<mesh_input> meshes;
void	LoadData(std::vector<mesh_input> &meshes);

void	LoadData(char  *filename);

//int max_Level=15;
quadsquare*	root = NULL;
quadcornerdata	RootCornerData = { NULL, NULL, 0, 0, 0, 0, { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } } };

ul::vector	ViewerDir(1, 0, 0);
ul::vector	ViewerUp(0, 1, 0);
float	ViewerTheta = 0;
float	ViewerPhi = 0;
float	ViewerHeight = 0;
ul::vector	ViewerLoc(0, 40, 0);
float	Speed = (1 << 5);
bool	PinToGround = false;
bool	MoveForward = false;

int	TriangleCounter = 0;
global_extents ge;

float	Detail = 100.0;// FLT_MAX;


int	main(int argc, char *argv[])
{

  ge.max_Level=15;
  RootCornerData.Level= ge.max_Level;

  edge_thresh=atof(argv[2]);
  
	for(int i=0; i <3; i++){
	  ge.min[i]=DBL_MAX;
	  ge.max[i]=DBL_MIN;
	  ge.range[i]=1.0;
	}


	if(argc > 1){
	  FILE* fp;
	  char meshname[255];
	  float res;
	  fp=fopen(argv[1],"rb");
	  if(!fp){
	    fprintf(stderr,"Can't open %s\n",argv[1]);
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
	    TriMesh::verbose=0;
	    TriMesh *mesh = TriMesh::read(meshes[i].name.c_str());
	    edge_len_thresh(mesh,edge_thresh);
	    mesh->need_bbox();
	    meshes[i].envelope=Envelope<double>(mesh->bbox.min[0],
						mesh->bbox.min[1],
						mesh->bbox.max[0],
						mesh->bbox.max[1]);
	    // cout << meshes[i].envelope;
	    if( mesh->bbox.min[2] < zmin)
	      zmin= mesh->bbox.min[2];
	    
	    if( mesh->bbox.min[2] > zmax)
	      zmax= mesh->bbox.max[2];
	    if(i == 0)
	      tree_bounds=meshes[i].envelope;
	    else
	      tree_bounds.expand_to_include(meshes[i].envelope);
	    
	    delete mesh;
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

	  std::cout << ge;

	  RootCornerData.xorg=ge.get_in_cells(tree_bounds.minx()-tree_bounds.minx(),ge.max_Level);
	  RootCornerData.yorg=ge.get_in_cells(tree_bounds.miny()-tree_bounds.miny(),ge.max_Level);
	  printf("Root Corner xorg %d yorg %d\n",RootCornerData.xorg,RootCornerData.yorg);
	  root = new quadsquare(&RootCornerData);

	  LoadData(meshes);
	}
	
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
	// Draw the quadtree.
	if (root) {
	  //		root->Update(RootCornerData, (const float*) ViewerLoc, Detail);
		root->RenderToWF(RootCornerData);
	}
	return 0;

}
void load_hm_file(HeightMapInfo *hm,const char *filename){
  FILE *fp= fopen(filename,"rb");
  if(!fp){
    fprintf(stderr,"Cannot open %s\n",filename);
    return;
  }
	float data[2];
	int idata[2];
	
	fread((char *)data,sizeof(float),2,fp);
	hm->x_origin=data[0];
	hm->y_origin=data[1];

	fread((char *)data,sizeof(float),2,fp);
	float min=data[0];
	float max=data[1];
	float zrange = max-min;
	fread((char *)idata,sizeof(int),2,fp);
	hm->YSize=idata[0];
	hm->XSize=idata[1];

	hm->RowWidth=hm->XSize;
	hm->Scale=5;
	hm->Data = new uint16[hm->XSize * hm->YSize];
	float tmp;
	int range = (int) pow(2,8) - 1;
	for(int i=0; i < hm->XSize * hm->YSize; i++){
	  fread((char *)&tmp,sizeof(float),1,fp);
	  //if(isinf(tmp))
	    //	    hm->Data[i]=100;//	    
	    tmp=min;
	    //	    hm->Data[i]=0;
	    // else
	    //	  else
	    // hm->Data[i]=0;
	    hm->Data[i]=((uint16)(((tmp-min)/zrange)*(range)));
	      //printf("%f %f %f %d %d\n",tmp, (tmp-min)/zrange,zrange,range,hm->Data[i]);
	}
	hm->x_origin=24576;
	  hm->y_origin=24576;

}
void	LoadData(std::vector<mesh_input> &meshes)
// Load some data and put it into the quadtree.
{
 
  for(unsigned int i=0; i< meshes.size(); i++){
 

   TriMesh::verbose=0;
   TriMesh *mesh = TriMesh::read(meshes[i].name.c_str());
   edge_len_thresh(mesh,edge_thresh);
   point_nn*pout;
   int nout;
   int nx,ny;
   float cx,cy;
   int level;
   double actual_res;
   interpolate_grid(mesh,meshes[i],pout,nout,nx,ny,cx,cy,actual_res,level);
   char fname[255];
   sprintf(fname,"tmp/%s",meshes[i].name.c_str());
   write_mesh(pout,nout,fname);
   printf("\r %03d/%03d",i,(int)meshes.size());
   fflush(stdout);
   // points_to_quadtree(nout,pout,qt);
   //free(&pout);
   //   printf("Nx %d Ny %d Cx %f Cy %f\n",nx,ny,cx,cy);
   HeightMapInfo	hm;
   hm.x_origin = ge.get_in_cells(meshes[i].envelope.minx()-ge.min[0],ge.max_Level);
   hm.y_origin = ge.get_in_cells(meshes[i].envelope.miny()-ge.min[1],ge.max_Level);
   //   printf("Xorigin %d Yorigin %d\n",hm.x_origin,hm.y_origin);
   hm.XSize = nx;
   hm.YSize = ny;
   hm.RowWidth = hm.XSize;
   hm.Scale =level;
   hm.Data = new uint16[hm.XSize * hm.YSize];
   
   for(int i=0; i < hm.XSize * hm.YSize; i++){
     // printf("%f ",pout[i].z);
     hm.Data[i]= ((UINT16_MAX_MINUS_ONE)* ((pout[i].z-ge.min[2])/(ge.range[2]))) +1;
     //  printf("Final %d Source %f Rescaled %f\n",hm.Data[i],pout[i].z,(pout[i].z-zmin)/(zmax-zmin));
   }
   //zmin=0.0;
   root->AddHeightMap(RootCornerData, hm);
   delete [] hm.Data;
   delete mesh;  
 }
 
 
	

}

void	LoadData(char *filename)
// Load some data and put it into the quadtree.
{

  HeightMapInfo	hm;
  load_hm_file(&hm,filename);
  root->AddHeightMap(RootCornerData, hm);
	

}
