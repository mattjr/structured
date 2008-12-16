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

#ifndef Quadtree_h  
#define Quadtree_h       1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/*******************************************************************

  Quadtree structure
 
  *****************************************************************/
 
#include "Mynode.h"

template < class nd > 
class Quadtree {

 public:
  
  Quadtree( int depth );
  ~Quadtree();

  inline nd* getNode(int index) {
    return &(Nodes[index]);
  }

  inline nd* getHeadNodes() {
    return Nodes;
  } 

  inline int treeDepth() { 
    return Depth;
  }

  inline int treeSize() { 
    return Size;
  }
  inline int local(int index){
    int j=nodeLevel(index);
    return (int)((double)index- (pow(4.0,j)-1.0)/3.0);
  }
  
  inline int nodeLevel(int index) {
    int l = 0,tmp;
    tmp = index-1;
    while ( tmp >= 0 ) {
      tmp = (tmp >> 2 ) - 1;
      l++;
    }
    return l;
  }
  
  /******************************************/
  /*       CHILD NODES COORDINATES          */
  /******************************************/

  /******* TOP LEFT CHILD *******/
  inline void getTLChildNodeCoords(int *coords) {
    coords[0] = coords[0] << 1;
    coords[1] = coords[1] << 1;
  }
    
  /******* TOP RIGHT CHILD *******/
  inline void getTRChildNodeCoords(int *coords) {
    coords[0] = (coords[0] << 1) + 1;
    coords[1] = coords[1] << 1;
  }
  
  /******* BOTTOM LEFT CHILD *******/
  inline void getBLChildNodeCoords(int *coords) {
    coords[0] = coords[0] << 1;
    coords[1] = (coords[1] << 1) + 1;
  }
    
  /******* BOTTOM RIGHT CHILD *******/
  inline void getBRChildNodeCoords(int *coords) {
    coords[0] = (coords[0] << 1) + 1;
    coords[1] = (coords[1] << 1) + 1;
  }

  /*******    FATHER NODE  **********/
  inline void getFatherNodeCoords( int *coords ) {
    coords[0] = coords[0] >> 1;
    coords[1] = coords[1] >> 1;
  }
  
  /******************************************/
  /*       CHILDREN NODES INDICES           */
  /******************************************/
   
  /******* TOP LEFT CHILD *******/
  inline int getTLChildNode(int index) {
    int level = nodeLevel(index);
    return getTLChildNode(index,&level);
  }
  
  inline int getTLChildNode(int index,int *level) {
    int v = *level;
    (*level)++;
    return (v & 1)?((index << 2)+1):((index << 2)+2); 
  }
  
  /******* TOP RIGHT CHILD *******/
  inline int getTRChildNode(int index) {
    int level = nodeLevel(index);
    return getTRChildNode(index,&level);
  } 

  inline int getTRChildNode(int index,int *level) {
    int v = *level;
    (*level)++;
    return (v & 1)?((index << 2)+2):((index << 2)+1); 
  }

  /******* BOTTOM LEFT CHILD *******/
  inline int getBLChildNode(int index) {
    int level = nodeLevel(index);
    return getBLChildNode(index,&level);
  }
 
  inline int getBLChildNode(int index,int *level) {
    int v = *level;
    (*level)++;
    return (v & 1)?((index << 2)+3):((index << 2)+4); 
  }
    
  /******* BOTTOM RIGHT CHILD *******/
  inline int getBRChildNode(int index) {
    int level = nodeLevel(index);
    return getBRChildNode(index,&level); 
  }  

  inline int getBRChildNode(int index,int *level) {
    int v = *level;
    (*level)++;
    return (v & 1)?((index << 2)+4):((index << 2)+3); 
  }

  /******************************************/
  /*          FATHER NODE INDEX             */
  /******************************************/

  inline int getFatherNode(int index) {
    return (index>0)?(((index-1) >> 2 )):0;
  }

  inline int getFatherNode(int index, int *level ) {
    ((*level)--);
    return (index>0)?(((index-1) >> 2 )):0;
  }

  /******************************************/
  /*    NODE LOCATION INSIDE THE QUADTREE   */
  /******************************************/

  const static char _INSIDE_NODE       = 0;
  const static char _TOP_BORDER_NODE   = 1;
  const static char _BOT_BORDER_NODE   = 2;
  const static char _LEFT_BORDER_NODE  = 3;
  const static char _RIGHT_BORDER_NODE = 4;
  const static char _TR_CORNER_NODE    = 5;
  const static char _TL_CORNER_NODE    = 6;
  const static char _BR_CORNER_NODE    = 7;
  const static char _BL_CORNER_NODE    = 8;
  const static char _ROOT_NODE         = 9;

  inline char nodeLocation( nd *n, int level ) {
    int gs = (1 << level ) - 1;
    return nodeLocationCoords(n->Coords[0],n->Coords[1],gs);
  }
 
  inline char nodeLocation( int index ) {
    int gs,level,coords;
    nd *n;

    level = nodeLevel(index);
    gs    = (1 << level ) - 1;
    n     = Nodes+index;
    return nodeLocationCoords(n->Coords[0],n->Coords[1],gs);
  }

  inline char nodeLocationCoords( int c0, int c1, int gs ) {

    if ( gs == 0 ) return _ROOT_NODE;

    if ( ( c0 > 0 ) && ( c0 < gs) &&
	 ( c1 > 0 ) && ( c1 < gs) ) 
      return _INSIDE_NODE;
    else if ( ( c0 == 0 ) && ( c1 > 0 ) && ( c1 < gs) )
      return _LEFT_BORDER_NODE;
    else if ( ( c0 == gs ) && ( c1 > 0 ) && ( c1 < gs) )
      return _RIGHT_BORDER_NODE;
    else if ( ( c1 == 0 ) && ( c0 > 0 ) && ( c0 < gs) )
      return _TOP_BORDER_NODE;
    else if ( ( c1 == gs ) && ( c0 > 0 ) && ( c0 < gs) )
      return _BOT_BORDER_NODE;
    else if ( ( c0 == 0 ) && ( c1 == 0 ) )
      return _TL_CORNER_NODE;
    else if ( ( c0 == gs ) && ( c1 == 0 ) )
      return _TR_CORNER_NODE;
    else if ( ( c0 == 0 ) && ( c1 == gs ) )
      return _BL_CORNER_NODE;
    else if ( ( c0 == gs ) && ( c1 == gs ) )
      return _BR_CORNER_NODE;
    else {
      printf("Quadtree: error in nodeLocationCoords(). exiting.\n");
      exit(0);
    }
  }

  void printLocation(char l);

  inline void getNodeCoordinates( int index , int *coords, int *level );

  inline int getNeighbor(int index, int neib) {
    
    int coords[2]; 
    
    coords[0] = (Nodes+index)->Coords[0];
    coords[1] = (Nodes+index)->Coords[1];
    return getNextNode(index, neib, nodeLevel(index),coords);
  }
 
  inline int getNextNode(int index, int neib, int level, int *coords);
  inline int gotoNextNode(int index, int neib, int *level, int *coords);

  inline int gotoFatherNode(int index, int *level, int *coords) {
    getFatherNodeCoords(coords);
    return getFatherNode(index,level); 
  }
  inline int gotoTLChildNode(int index, int *level, int *coords) {
    getTLChildNodeCoords(coords);
    return getTLChildNode(index,level);
  }
  inline int gotoTRChildNode(int index, int *level, int *coords) {
    getTLChildNodeCoords(coords);
    return getTRChildNode(index,level);
  } 
  inline int gotoBLChildNode(int index, int *level, int *coords) {
    getTLChildNodeCoords(coords);
    return getBLChildNode(index,level);
  }
  inline int gotoBRChildNode(int index, int *level, int *coords) {
    getTLChildNodeCoords(coords);
    return getBRChildNode(index,level);
  }

  /*************************************************************************/
  /*                  PRE-COMPUTING THE NODE COORDINATES                   */
  /*    FOR EACH NODE, ITS POSITION ON A LOCAL GRID IS STORED IN THE QT    */
  /*************************************************************************/

  void precomputeCoords();

  /********* NEIGHBORS DIRECTIONS ************/

  static const char  _LEFT      = 0;
  static const char  _RIGHT     = 1;
  static const char  _NORTH     = 2;
  static const char  _SOUTH     = 3;
  static const char  _WEST      = 4;
  static const char  _EAST      = 5;
  static const char  _NW        = 6;
  static const char  _NE        = 7;
  static const char  _SE        = 8;
  static const char  _SW        = 9;
  static const char  _NW_OL     = 10;
  static const char  _NE_OL     = 11;
  static const char  _SE_OL     = 12;
  static const char  _SW_OL     = 13;

  /******** QUADTREE CONSTANTS ***************/

  static const char _DEFAULT_DEPTH           = 3;
  static const char _MAX_DEPTH               = 15; 
  static const char _FALSE                   = 0;
  static const char _TRUE                    = 1;
  

 private:

  inline int   dh(int n );  
  inline int   dht(int n );
  inline int   dv(int n );
  inline int   dvt(int n );  
  void  computeDistanceVector(int level);
  

  /******* DISTANCE VECTORS *********/

  int **DistancesH;
  int **DistancesV;
  int DistanceLevelCreated[15];  /** MAX_DEPTH **/

  /******* POW, LOG LOOK-UP TABLES ***************/

 public:

  int           *_POW4;
  int           *_POW2;
  int           *_POWM1;
  long double    _LOG_4;
  long double    _ONETHIRD;
  long double    _TWOTHIRD;
  
 protected:
  
  int Size;          /* total tree size (number of nodes) */
  int Depth;
  
  nd *Nodes;



};

#endif 
