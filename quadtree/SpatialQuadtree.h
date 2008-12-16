/*

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish and distribute.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
  AUTHOR BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
  AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

  Copyright 1999 -   Laurent Balmelli
                     Laboratory For Audio-Visual Communications 
		     Ecole Polytechnique Federale de Lausanne, Switzerland

		     also:
		     Mathematics for Communication, Bell Laboratories
		     Lucent Technologies, USA

  contact, bugs:     Laurent.Balmelli@epfl.ch
                     balmelli@acm.org
		     
  PLEASE DO NOT REMOVE THIS COPYRIGHT NOTICE. 
		     
 */

#ifndef SpatialQuadtree_h  
#define SpatialQuadtree_h       1

#include "fast/Quadtree.h"

#include <stdio.h>
#include <vector>
#include "QTElement.h"

class SpatialQuadtree : public Quadtree<QTElement> {
 
 public:

  SpatialQuadtree( int depth ): Quadtree<QTElement>::Quadtree(depth){
    init();
  };
  ~SpatialQuadtree(){};

  inline void myTraversal(int index) {

    //int coords[2],
    int    current;
    //   level;

    current = getTRChildNode(index);
    current = getNeighbor(current,_EAST);
    printf("@ node %d\n",current);
    current = getNeighbor(current,_NORTH);
    printf("@ node %d\n",current);
    current = getNeighbor(current,_NW);
    printf("@ node %d\n",current);

  }
private:
  void init(void);
  void setNode(int, QTElement );
  vector<int> base4(int i);
   vector<int>lbase4(int i);
  vector<int>  slbase4(int i);
  vector<int> locate(int i);
  int dh(int i);
  int dht(int i);
  int dv(int i);
  int dvt(int i);
  int switchbit(int i);
  vector<int> fliplr(vector<int>ai);
  int getNeighbor(int i, int j);
  void computeDistanceVector(int i);
    int **DistancesH;
     int **DistancesV;
    const static int MAX_DEPTH = 20;
    const static int NORTH = 0;
    const static int SOUTH = 1;
    const static int EAST = 2;
    const static int WEST = 3;
    const static int NW = 4;
    const static int NE = 5;
    const static int SE = 6;
    const static int SW = 7;
    const static int NW_OL = 8;
    const static int NE_OL = 9;
    const static int SE_OL = 10;
    const static int SW_OL = 11;
    const static int W_OL_DOWN = 12;
    const static int W_OL_UP = 13;
    const static int E_OL_DOWN = 14;
    const static int E_OL_UP = 15;
};

#endif
