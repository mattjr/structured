#include  <stdio.h>
#include  <stdlib.h>
#include "terrainnode.hpp"
#include "TriMesh_algo.h"
#include <osgViewer/Viewer>

#include "raster.hpp"
using namespace std;
typedef struct _mesh_input{
  string name;
  float res;
}mesh_input;
std::vector<mesh_input> meshes;
int main( int argc, char **argv ) {



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
  mapnik::Envelope<double> tree_bounds;
  for(unsigned int i=0; i< meshes.size(); i++){
    TriMesh::verbose=0;
    TriMesh *mesh = TriMesh::read(meshes[i].name.c_str());
    mesh->need_bbox();
    tree_bounds.expand_to_include(Envelope<double>(mesh->bbox.min[0],
						   mesh->bbox.min[1],
						   mesh->bbox.max[0],
						   mesh->bbox.max[1]));
    delete mesh;
  }
  cout <<"Number of meshes " << meshes.size() <<endl; 
  cout << "Bounding Box " << tree_bounds<<endl;

  terrain_tree qt(tree_bounds,20,0.5,atof(argv[2]));
  cout <<"Created Tree\n";
 for(unsigned int i=0; i< meshes.size(); i++){
   TriMesh::verbose=0;
   TriMesh *mesh = TriMesh::read(meshes[i].name.c_str());
   int nf = mesh->faces.size();
   for (int i = 0; i < nf; i++){
     raster_triangle(mesh->vertices[mesh->faces[i][0]],
		     mesh->vertices[mesh->faces[i][1]],
		     mesh->vertices[mesh->faces[i][2]]);
   }

   delete mesh;  
 }
 
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
  printf("Num items %d\n",qt.count_items());
  // construct the viewer.
  //	qt.trim();
  //	qt.balance();
  //qt.render_terrain();
  qt.render_tree();
  // osgViewer::Viewer viewer;
  //viewer.setSceneData(qt.osg_root);
  // return viewer.run();
}
