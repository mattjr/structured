#include "TriMesh.h"
#include "TriMesh_algo.h"
void usage(char *str){
  printf("%s : input.ply output.bnpts [-flip]\n");
  exit(-1);
}
int main(int argc, char **argv){
  bool flip=false;
  if (argc < 3)
    usage(argv[0]);

  if(argc == 4){
    flip=true;
    fprintf(stderr, "Flipping!\n");
  }
  TriMesh *mesh = TriMesh::read(argv[1]);
  if (!mesh) {
    fprintf(stderr, "Couldn't read file %s\n", argv[1]);
    exit(-1);
  }
  FILE *pos_fp=fopen(argv[2],"wb");
  if(!pos_fp){
    fprintf(stderr,"Can't open %s\n",argv[2]);
    exit(-1);
  }
  if(flip)
    faceflip(mesh);

  mesh->need_normals();
  int nv = mesh->vertices.size();
  float buf[6];
  for (int i = 0; i < nv; i++) {
    for(int j=0; j<3; j++)
      buf[j]=(float)mesh->vertices[i][j];
    
    for(int j=0; j<3; j++)
      buf[j+3]=(float)mesh->normals[i][j];
    fwrite(buf,sizeof(float),6,pos_fp);
  }

  return 0;
}

