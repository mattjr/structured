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
                     balmelli@research.bell-labs.com
		     
  PLEASE DO NOT REMOVE THIS COPYRIGHT NOTICE. 
		     
 */

#ifndef Mytree_h  
#define Mytree_h       1

#include "Quadtree.h"
#include "Mynode.h"
#include <stdio.h>
 
class Mytree : public Quadtree<Mynode> {

 public:

  Mytree( int depth );
  ~Mytree();


  inline void myTraversal(int index) {

    int coords[2],
        current,
        level;

    getNodeCoordinates(index,coords,&level);
    
    getTRChildNodeCoords(coords);
    current = getTRChildNode(index,&level);
    current = gotoNextNode(current,_EAST,&level,coords);
    printf("@ node %d\n",current);
    current = gotoNextNode(current,_NORTH,&level,coords);
    printf("@ node %d\n",current);
    current = gotoNextNode(current,_NW,&level,coords);
    printf("@ node %d\n",current);
  }

};

#endif
