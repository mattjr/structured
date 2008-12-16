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

#include "fast/Quadtree.h"
#include "fast/Quadnode.h"
#include "SpatialQuadtree.h"
#include  <stdio.h>
#include  <stdlib.h>

main( int argc, char **argv ) {

  SpatialQuadtree *q; 
  int trees;
  int neib;
  
  if (argc < 2 ) {
    printf("format: %s <departure node>\n",argv[0]);
    exit(0);
  }

  q = new SpatialQuadtree(6);
  trees = q->treeSize();

  q->myTraversal(atoi(argv[1]));

  delete q;
}
