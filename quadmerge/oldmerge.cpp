#include  <stdio.h>
#include  <stdlib.h>
#include "terrainnode.hpp"
#include "TriMesh_algo.h"
#include <osgViewer/Viewer>
#include <osgDB/WriteFile>
#include <osgDB/Registry>

#include "uquadtree.hpp"
#include "raster.hpp"

using namespace std;
static void points_to_quadtree(int n, point_nn* points, terrain_tree &qt)
{
    int i;

    for (i = 0; i < n; ++i) {
        point_nn* p = &points[i];
	terrain_data data;
	data.z=p->z;
	data.x=p->x;
	data.y=p->y;
	//	printf("%f %f %f\n",p->z,p->x,p->y);
        if (!isnan(p->z) && p->z !=-DBL_MAX)
          qt.insert(data,p->x, p->y);
    }
}

std::vector<mesh_input> meshes;

int main( int argc, char **argv ) {
  ge.max_Level=15;


  FILE* fp;
  char meshname[255];
  float res;
  fp=fopen(argv[1],"rb");
  if(!fp){
    fprintf(stderr,"Can't open %s\n",argv[1]);
    exit(0);
  }
  
  while(1){
    if(fscanf(fp,"%s %f %*d",meshname,&res)!=2)
      break;
    mesh_input m;
    m.name=meshname;
    m.res=res;
    meshes.push_back(m);
  }
 for(int i=0; i <3; i++){
    ge.min[i]=DBL_MAX;
    ge.max[i]=DBL_MIN;
    ge.range[i]=1.0;
  }


  mapnik::Envelope<double> tree_bounds;
  for(unsigned int i=0; i< meshes.size(); i++){
    TriMesh::verbose=0;
    TriMesh *mesh = TriMesh::read(meshes[i].name.c_str());
   
    mesh->need_bbox();
    meshes[i].envelope=Envelope<double>(mesh->bbox.min[0],
						   mesh->bbox.min[1],
						   mesh->bbox.max[0],
				     mesh->bbox.max[1]);
    if(i==0)
      tree_bounds=meshes[i].envelope;
    else
    tree_bounds.expand_to_include(meshes[i].envelope);
    delete mesh;
  }
  cout <<"Number of meshes " << meshes.size() <<endl; 
  cout << "Bounding Box " << tree_bounds<<endl;
 int	whole_cell_int_size = 2 << ge.max_Level;
 double tree_max_size=max(tree_bounds.width(),tree_bounds.height());
  ge.cell_size=tree_max_size/whole_cell_int_size;
  //	  ge.cell_size=0.1;
  //ge.range[2]=zmax-zmin;
  ge.range[0]=tree_bounds.width();
  ge.range[1]=tree_bounds.height();
  ge.min[0]=tree_bounds.minx();
  ge.min[1]=tree_bounds.miny();
  // ge.min[2]=zmin;

  ge.max[0]=tree_bounds.maxx();
  ge.max[1]=tree_bounds.maxy();
  //ge.max[2]=zmax;

  terrain_tree qt(tree_bounds,20,0.5,atof(argv[2]));
  cout <<"Created Tree\n";
 for(unsigned int i=0; i< meshes.size(); i++){
 

   TriMesh::verbose=0;
   TriMesh *mesh = TriMesh::read(meshes[i].name.c_str());

   point_nn*pout;
   int nout;
   int nx,ny;
   float cx,cy;
   double actual_res;
   int level;
   interpolate_grid(mesh,meshes[i],pout,nout,ny,ny,cx,cy,actual_res,level,true);
   printf("\r %03d number of points: %d",i,nout);
   fflush(stdout);
   points_to_quadtree(nout,pout,qt);
   //free(&pout);
 
   delete mesh;  
 }
 printf("Iterpolation Done\n");
  /* for(i=0; i < ptcount; i++){
    
    Envelope<double> pt_ext(pts[i].x,pts[i].y,pts[i].x,pts[i].y);
    qt.insert(pts[i],pt_ext);
    
    //	  cout <<tree_bounds.contains(pt_ext)<< " "<< pt_ext << " total " << tree_bounds<<endl;
  }
  */
  //delete pts;
  //delete norms;
  //	qt.print();
  //qt.draw();
 //printf("Num items %d %f mem\n",qt.count_items(),qt.count_items()*sizeof(terrain_node)/1024.0/1024.0);
  // construct the viewer.
  //	qt.trim();
  //	qt.balance();
  //qt.render_terrain();
  qt.render_tree();
  osgDB::Registry::instance()->writeNode( *qt.osg_root,"tree.ive",osgDB::Registry::instance()->getOptions() );

  //   osgViewer::Viewer viewer;
  //viewer.getCamera()->setClearColor(osg::Vec4(1.0,1.0,1.0,1.0));
  // viewer.setSceneData(qt.osg_root);
  //return viewer.run();

}
