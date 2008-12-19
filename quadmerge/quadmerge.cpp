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
	fseek(fp,SEEK_SET,0);
	float pts[cnt*DIMENSION];
	float norms[cnt*DIMENSION];
	int idx=0;
	while(1){
	  if(fread(c,sizeof(float),2*DIMENSION,fp)!=6){break;}
	  memcpy(&pts[idx],c,sizeof(float)*3);
	  memcpy(&norms[idx],c,sizeof(float)*3);
	  idx+=DIMENSION;
	  
	}
	fclose(fp);
	printf("%d count %f %f %f %f %f %f\n",cnt,min[0],min[1],min[2],max[0],max[1],max[2] );
	mapnik::Envelope<double> tree_bounds(min[0],min[1],max[0],max[1]);
	terrain_tree qt(tree_bounds,atoi(argv[2]),0.5);
	for(i=0; i < cnt*DIMENSION; i+=DIMENSION){
	  //	  cout << "pt : " << pts[i] << " " << pts[i+1] << " "<< pts[i +2]<<endl;
	  Envelope<double> pt_ext((double)pts[i+0],(double)pts[i+1],(double)pts[i+0],(double)pts[i+1]);
	  terrain_data data;
	  data.x=pts[i];
	  data.y=pts[i+1];
	  data.z=pts[i+2];

	  qt.insert(data,pt_ext);
	  //	  cout <<tree_bounds.contains(pt_ext)<< " "<< pt_ext << " total " << tree_bounds<<endl;
	}
	//	qt.print();
	//qt.draw();
	printf("Num items %d\n",qt.count_items());
// construct the viewer.
//	qt.trim();
	qt.balance();
	qt.render_tree();
    osgViewer::Viewer viewer;
    viewer.setSceneData(qt.osg_root);
    return viewer.run();
}
