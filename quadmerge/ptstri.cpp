#include  <stdio.h>
#include  <stdlib.h>
#include "terrainnode.hpp"
#include <osgViewer/Viewer>
using namespace std;
int main( int argc, char **argv ) {



  FILE* fp;
  int i,cnt=0;
#define DIMENSION 3
	float c[2*DIMENSION];
	fp=fopen(argv[1],"rb");
	double min[3],max[3];


	while(1){
	  if(fread(c,sizeof(float),2*DIMENSION,fp)!=6){break;}
	  for(i=0;i<DIMENSION;i++){
	    if(!cnt || c[i]<min[i]){min[i]=c[i];}
	    if(!cnt || c[i]>max[i]){max[i]=c[i];}
	  }
	  cnt++;
	}
	printf("Number of Pts %d\n",cnt);
	fseek(fp,SEEK_SET,0);
	terrain_data *pts = new terrain_data[cnt*DIMENSION];
      
	//float *norms= new terrain_data[cnt*DIMENSION];
	int ptcount=0;
	while(1){
	  if(fread(c,sizeof(float),2*DIMENSION,fp)!=6){break;}
	  pts[ptcount].x=(double)((float *)c)[0];
	  pts[ptcount].y=(double)((float *)c)[1];
	  pts[ptcount].z=(double)((float *)c)[2];
	  ptcount++;
	}
	fclose(fp);
	printf("%d count %f %f %f %f %f %f\n",ptcount,min[0],min[1],min[2],max[0],max[1],max[2] );
	mapnik::Envelope<double> tree_bounds(min[0],min[1],max[0],max[1]);
	terrain_tree qt(tree_bounds,20,0.5,atof(argv[2]));
	for(i=0; i < ptcount; i++){

	  Envelope<double> pt_ext(pts[i].x,pts[i].y,pts[i].x,pts[i].y);
	  qt.insert(pts[i],pt_ext);

	  //	  cout <<tree_bounds.contains(pt_ext)<< " "<< pt_ext << " total " << tree_bounds<<endl;
	}

	  delete pts;
	  //delete norms;
	//	qt.print();
	//qt.draw();
	printf("Num items %d\n",qt.count_items());
// construct the viewer.
//	qt.trim();
//	qt.balance();
	qt.render_terrain();
	//qt.render_tree();
    osgViewer::Viewer viewer;
    viewer.setSceneData(qt.osg_root);
    return viewer.run();
}
