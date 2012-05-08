#ifndef _GRAPH_GENERATOR_H_
#define _GRAPH_GENERATOR_H_



void makeCircle(int nvtxs, char * filename);
void makeStar(int nvtxs, char * filename);
void makeTorus(int * nvtxs, int dim1, int dim2, char * filename);
void makeCylinder(int * nvtxs, int dim1, int dim2, char * filename);
void makeSquareGrid(int * nvtxs, int dim1, int dim2, char * filename, int connect_corners,int partial);
void makeEiffel(int * nvtxs, char * filename);
void makeBinaryTree(int * nvtxs, int depth, char * filename);
void makeSierpinski(int * nvtxs, int depth, char * filename);
void make_hypercube(int * nvtxs, int dim, char * filename);


#endif
