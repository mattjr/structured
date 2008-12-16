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

#include "Quadtree.h"
#include "../SpatialQuadtree.h"
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>

/*********************************************************/
/***                 CONSTRUCTOR                       ***/
/*********************************************************/

template < class nd >
Quadtree<nd>::Quadtree( int depth ) {


  /***** Constant initialization ******/
   
  int size,
    i; 
  

  for ( i = 0; i < _MAX_DEPTH; i++ ) DistanceLevelCreated[i] = 0;
  
  _POW2  = new int[_MAX_DEPTH];
  _POW4  = new int[_MAX_DEPTH];
  _POWM1 = new int[_MAX_DEPTH];

  for ( i = 0 ; i < _MAX_DEPTH; i++ ) {
    
    _POW2[i]  = (int)pow(2.0,(double)i);
    _POW4[i]  = (int)pow(4.0,(double)i);
    _POWM1[i] = (int)pow(-1.0,(double)i);
   
  } 
  
  _LOG_4    = log(4.0);
  _ONETHIRD = (long double)1.0/(long double)3.0;
  _TWOTHIRD = (long double)2.0/(long double)3.0;
  
  
  size = (int)(((double)((double)(_POW4[depth])-1.0)*_ONETHIRD)+0.5);
 
  Depth   = depth;
  Size    = size;

  Nodes   = new nd[size];

  DistancesH = new int*[_MAX_DEPTH];
  DistancesV = new int*[_MAX_DEPTH];
 
  for (i = 0; i < _MAX_DEPTH; i++ ) {
    DistancesH[i]              = NULL;
    DistancesV[i]              = NULL;
  }
  
  precomputeCoords();

}


/*********************************************************/
/***                 DESTRUCTOR                        ***/
/*********************************************************/

template < class nd >
Quadtree<nd>::~Quadtree() {

  delete[] Nodes;

  for (int i = 0; i < _MAX_DEPTH; i++ ) {
    
    if ( DistancesH[i] != NULL )
      delete[] DistancesH[i];
    if ( DistancesV[i] != NULL )
      delete[] DistancesV[i];
  }

  delete[] DistancesH;
  delete[] DistancesV;
  delete[] _POW2;
  delete[] _POW4;
  delete[] _POWM1;

}


template < class nd >
void Quadtree<nd>::getNodeCoordinates( int index, int *coords, int *level ) {

  /***************************************/
  /**               LEVEL               **/
  /***************************************/

  int l = 0;
  int tmp;
  int pl;
 
  tmp = index-1;
  while ( tmp >= 0 ) {
    tmp = (tmp >> 2 ) - 1;
    l++;
  }

  /***************************************/
  /**            LOCAL INDEX            **/
  /***************************************/
 
  pl = (int)(index - ((1 << 2*l)/3));

  coords[0] = 0;
  coords[1] = 0;

  /***************************************/
  /**   GENERATE BITCODE AND COORDS     **/
  /***************************************/
  
  if ( l & 1 ) {     /*** ODD LEVEL ***/

    for ( int i = 0; i < l; i++ ) {

      if ( i & 1 ) {   /** ODD **/

	tmp = ( ((pl >> (i << 1)) & 1 ) + 
		(((pl >> ((i << 1)+1)) & 1 ) << 1));
		
      } else {

	/*** SWITCH BIT ***/

	tmp = ( (!((pl >> (i << 1)) & 1 )) + 
		(((pl >> ((i << 1)+1)) & 1 ) << 1)); 
      }

      (tmp==1)?(coords[0]+=(1<<i)):0;
      (tmp==2)?(coords[1]+=(1<<i)):0;
      (tmp==3)?(coords[1]+=(1<<i),coords[0]+=(1<<i)):0;

    }

  } else {    /*** EVEN LEVEL ***/

    for ( int i = 0; i < l; i++ ) {
      
      if ( i & 1 ) {   
	
	/*** SWITCH BIT ***/

	tmp = ( (!((pl >> (i << 1)) & 1 )) + 
		(((pl >> ((i << 1)+1)) & 1 ) << 1)); 
      } else {

	tmp = ( ((pl >> (i << 1)) & 1 ) + 
		(((pl >> ((i << 1)+1)) & 1 ) << 1));
      }

      (tmp==1)?(coords[0]+=(1<<i)):0;
      (tmp==2)?(coords[1]+=(1<<i)):0;
      (tmp==3)?(coords[1]+=(1<<i),coords[0]+=(1<<i)):0;
    }
  }
 
  *level = l;

}

/************************************************************
  
  RELATIONS OF THE CLOSED FORM

  WARNING: due to the use of int to compute the distance
           of indices, the maximum possible value of n is 15.
	   This corresponds to a terrain of size 32768x32768.

  **********************************************************/

template < class nd >
int Quadtree<nd>::dh(int n ) {

  return (int)((((double)(6*_POW4[n]+_POWM1[n+1]))*0.2)+0.5);
}

template < class nd >
int Quadtree<nd>::dht(int n ) {

  return (int)((((double)(_POW4[n+1]+_POWM1[n]))*0.2)+0.5);
}

template < class nd >
int Quadtree<nd>::dv(int n ) {

  return (int)(((_POW4[n+1]+2)*_ONETHIRD)+0.5);
}

template < class nd >
int Quadtree<nd>::dvt(int n ) {
  
  return (int)(((2*_POW4[n+1]-2)*_ONETHIRD)+0.5);
}


template < class nd >
void Quadtree<nd>::computeDistanceVector(int level) {

   int n = level,
       j,i,
     phi1_length,
     phi2_length,
     buffer_length,
     DH_length, DV_length;

   int *buffer,
        *phi1,      /* two buffers of phi to swap during the */
        *phi2;      /* recursion.*/
   
   /*********************************************************/
   /** CONSTRUCT THE DISTANCE VECTOR IF IT DOES NOT EXISTS **/
   /*********************************************************/

   
   
   buffer = (int*)malloc(3*sizeof(int));
   buffer_length = 3;
   phi1 = (int*)malloc(1*sizeof(int));
   phi1_length = 1;
   phi2 = (int*)malloc(3*sizeof(int));
   phi2_length = 3;
   
   if ( level > 0 ) {
     
     if ( DistancesH[level] == NULL ) {
       
       DistancesH[level] = new int[(_POW2[level]+1)];
       
       n -= 1; 
       buffer[0] = dht(0);
       buffer[1] = -dh(0);
       buffer[2] = dht(0);
       phi1[0]   = -dh(0);
       
       for ( i = 1; i <= n; i++ ) {
	 
	 buffer = (int*)malloc((_POW2[i+1]+1)*sizeof(int));
	 
	 buffer_length = _POW2[i+1]+1;
	 
	 
	 if ( (i%2) == 1 ) {
	   
	   phi2 = (int*)malloc((_POW2[i+1]-1)*sizeof(int));
	   phi2_length = _POW2[i+1]-1;
	   
	   for ( j = 0; j < phi1_length; j++ ) 
	     phi2[j]=-phi1[j];
	   
	   phi2[phi1_length] = -dh(i);
	   
	   for ( j = phi1_length+1; j < 2*(phi1_length)+1; j++ ) 
	     phi2[j]=-phi1[j-(phi1_length+1)];
	   
	   buffer[0] = dht(i);
	   for ( j = 1; j <= phi2_length; j++ )
	     buffer[j] = phi2[j-1];
	   buffer[phi2_length+1] = dht(i);
	   
	 }
	 else {
	   
	   phi1 = (int*)malloc((_POW2[i+1]-1)*sizeof(int));
	   phi1_length = _POW2[i+1]-1;
	   
	   for ( j = 0; j < phi2_length; j++ ) 
	     phi1[j]=-phi2[j];
	   
	   phi1[phi2_length] = -dh(i);
	   
	   for ( j = phi2_length+1; j < 2*(phi2_length)+1; j++ ) 
	     phi1[j]=-phi2[j-(phi2_length+1)]; 
	   
	   buffer[0] = dht(i);
	   for ( j = 1; j <= phi1_length; j++ )
	     buffer[j] = phi1[j-1];
	   buffer[phi1_length+1] = dht(i);
	   
	 }
	 
       }
       memcpy(DistancesH[level],buffer,buffer_length*sizeof(int));
       DH_length = buffer_length;
       
     }
     
     free(buffer);
     free(phi1);
     free(phi2);
     
     buffer = (int*)malloc(3*sizeof(int));
     buffer_length = 3;
     phi1 = (int*)malloc(1*sizeof(int));
     phi1_length = 1;
     phi2 = (int*)malloc(3*sizeof(int));
     phi2_length = 3;
     
     n = level;
       
     if ( DistancesV[level] == NULL ) {
       
       DistancesV[level] = new int[(_POW2[level]+1)];
       
       n -= 1;
       buffer[0] = -dvt(0);
       buffer[1] = dv(0);
       buffer[2] = -dvt(0);
       phi1[0]   = dv(0);
       
       for ( i = 1; i <= n; i++ ) {
	 
	 buffer = (int*)(malloc((_POW2[i+1]+1)*sizeof(int))); 
	 buffer_length = _POW2[i+1]+1;
	 
	 if ( i%2 == 1 ) {
	   
	   phi2 = (int*)malloc((_POW2[i+1]-1)*sizeof(int));           
	   phi2_length = _POW2[i+1]-1;
	   
	   for ( j = 0; j < phi1_length; j++ ) 
	     phi2[j]=phi1[j];
	   
	   phi2[phi1_length] = dv(i);
	   
	   for ( j = phi1_length+1; j < 2*(phi1_length)+1; j++ ) 
	     phi2[j]=phi1[j-(phi1_length+1)];
	   
	   buffer[0] = -dvt(i);
	   for ( j = 1; j <= phi2_length; j++ )
	     buffer[j] = phi2[j-1];
	   buffer[phi2_length+1] = -dvt(i);
	   
	 }
	 else {
	   
	   phi1 = (int*)malloc((_POW2[i+1]-1)*sizeof(int)); 
	   phi1_length = _POW2[i+1]-1; 
	   
	   for ( j = 0; j < phi2_length; j++ ) 
	     phi1[j]=phi2[j];
	   
	   phi1[phi2_length] = dv(i);
	   
	   for ( j = phi2_length+1; j < 2*(phi2_length)+1; j++ ) 
	     phi1[j]=phi2[j-(phi2_length+1)]; 
	   
	   buffer[0] = -dvt(i);
	   for ( j = 1; j <= phi1_length; j++ )
	     buffer[j] = phi1[j-1];
	   buffer[phi1_length+1] = -dvt(i);
	 }
       }
       
       memcpy(DistancesV[level],buffer,buffer_length*sizeof(int));
       DV_length = buffer_length;
     }
   } else {
     printf("Cannot generate a distance vector for level 0.");
     return;
   }
   
   DistanceLevelCreated[level] = 1; 
   
}  
 

template < class nd >
int Quadtree<nd>::getNextNode(int index, int neib, int level, int *coords) {

  if ( !DistanceLevelCreated[level] )
    computeDistanceVector(level);
    
  switch (neib) {
    
  case _NORTH:
    return index - DistancesV[level][coords[1]];
    break; 

  case _SOUTH:
    return index + DistancesV[level][coords[1]+1];
    break; 

  case _EAST:
    return index + DistancesH[level][coords[0]+1];
    break; 
    
  case _WEST:
    return index - DistancesH[level][coords[0]];
    break; 
    
  case _NW:
    return index - DistancesH[level][coords[0]] - 
      DistancesV[level][coords[1]];
    break;
 
  case _NE:
    return index + DistancesH[level][coords[0]+1] - 
      DistancesV[level][coords[1]];
    break; 
    
  case _SE:
    return index + DistancesH[level][coords[0]+1] + 
      DistancesV[level][coords[1]+1];
    break; 
    
  case _SW:
    return index - DistancesH[level][coords[0]] + 
      DistancesV[level][coords[1]+1];
    break; 

    
  case _NW_OL:

    if ( !DistanceLevelCreated[level+1] )
      computeDistanceVector(level+1);

    if ( level%2 == 0 )
      return 4*index+2 - DistancesH[level+1][2*coords[0]] - 
	DistancesV[level+1][2*coords[1]];
    else 
      return 4*index+1 - DistancesH[level+1][2*coords[0]] - 
	DistancesV[level+1][2*coords[1]];
    break; 
    
  case _NE_OL:

    if ( !DistanceLevelCreated[level+1] )
      computeDistanceVector(level+1);

    if ( level%2 == 0 )
      return 4*index+1+ DistancesH[level+1][2*coords[0]+2] - 
	DistancesV[level+1][2*coords[1]];
    else
      return 4*index+2 + DistancesH[level+1][2*coords[0]+2] - 
	DistancesV[level+1][2*coords[1]];
    break; 
    
  case _SE_OL:

    if ( !DistanceLevelCreated[level+1] )
      computeDistanceVector(level+1);

    if ( level%2 == 0 )
      return 4*index+3 + DistancesH[level+1][2*coords[0]+2] + 
	DistancesV[level+1][2*coords[1]+2];
    else
      return 4*index+4 + DistancesH[level+1][2*coords[0]+2] + 
	DistancesV[level+1][2*coords[1]+2];
    break; 
    
  case _SW_OL:

    if ( !DistanceLevelCreated[level+1] )
      computeDistanceVector(level+1);

    if ( level%2 == 0 )
      return 4*index+4 - DistancesH[level+1][2*coords[0]] + 
	DistancesV[level+1][2*coords[1]+2];
    else
      return 4*index+3 - DistancesH[level+1][2*coords[0]] + 
	DistancesV[level+1][2*coords[1]+2];
    break; 
    
  }
  
  return 0;
}

template < class nd >
int Quadtree<nd>::gotoNextNode(int index, int neib, int *level, int *coords) {

  int vlevel = *level,
      next = 0,
      grid_size;

  grid_size = (1 << vlevel) - 1;

  if ( !DistanceLevelCreated[vlevel] )
    computeDistanceVector(vlevel);
  
  switch (neib) {
 
  
  case _NORTH:
    next = index - DistancesV[vlevel][coords[1]];
    if ( (--coords[1]) < 0 ) coords[1]=grid_size;
  
    break; 

  case _SOUTH:
    
    next = index + DistancesV[vlevel][coords[1]+1];
    if ( (++coords[1]) > grid_size ) coords[1]=0; 
   
    break; 

  case _EAST:
    
    next = index + DistancesH[vlevel][coords[0]+1];    
    if ( (++coords[0]) > grid_size ) coords[0]=0;

    break; 
    
  case _WEST:
    
    next = index - DistancesH[vlevel][coords[0]];
    if ( (--coords[0]) < 0 ) coords[0]=grid_size; 
   
    break; 
     
  case _NW:
    
    next = index - DistancesH[vlevel][coords[0]] - 
      DistancesV[vlevel][coords[1]];

    if ( (--coords[0]) < 0 ) coords[0]=grid_size; 
    if ( (--coords[1]) < 0 ) coords[1]=grid_size; 
    
    break;
 
  case _NE:
    
    next = index + DistancesH[vlevel][coords[0]+1] - 
      DistancesV[vlevel][coords[1]];

    if ( (--coords[1]) < 0 ) coords[1]=grid_size; 
    if ( (++coords[0]) > grid_size ) coords[0]=0;

    break; 
    
  case _SE:
    
    next = index + DistancesH[vlevel][coords[0]+1] + 
      DistancesV[vlevel][coords[1]+1];

    if ( (++coords[0]) > grid_size ) coords[0]=0;
    if ( (++coords[1]) > grid_size ) coords[1]=0;
    
    break; 
    
  case _SW:
    
    next = index - DistancesH[vlevel][coords[0]] + 
      DistancesV[vlevel][coords[1]+1];

    if ( (--coords[0]) < 0 ) coords[0] =grid_size; 
    if ( (++coords[1]) > grid_size ) coords[1]=0;

    break; 

     
  case _NW_OL:
     
    grid_size = (1 << (vlevel+1)) - 1;

    if ( !DistanceLevelCreated[vlevel+1] )
      computeDistanceVector(vlevel+1);

    if ( vlevel%2 == 0 )
      next = 4*index+2 - DistancesH[vlevel+1][2*coords[0]] - 
	DistancesV[vlevel+1][2*coords[1]];
    else 
      next = 4*index+1 - DistancesH[vlevel+1][2*coords[0]] - 
	DistancesV[vlevel+1][2*coords[1]];

    coords[0] = (coords[0] << 1) - 1;
    coords[1] = (coords[1] << 1) - 1;
    if ( coords[0] < 0 ) coords[0] = grid_size;
    if ( coords[1] < 0 ) coords[1] = grid_size;

    (*level)++;
    break; 
    
  case _NE_OL:
    
    grid_size = (1 << (vlevel+1)) - 1;

    if ( !DistanceLevelCreated[vlevel+1] )
      computeDistanceVector(vlevel+1);

    if ( vlevel%2 == 0 )
      next = 4*index+1+ DistancesH[vlevel+1][2*coords[0]+2] - 
	DistancesV[vlevel+1][2*coords[1]];
    else
      next = 4*index+2 + DistancesH[vlevel+1][2*coords[0]+2] - 
	DistancesV[vlevel+1][2*coords[1]];
    coords[0] = (coords[0] << 1) + 2;
    coords[1] = (coords[1] << 1) - 1;
    if ( coords[0] > grid_size ) coords[0] = 0;
    if ( coords[1] < 0 ) coords[1] = grid_size;

    (*level)++; 
    break; 
    
  case _SE_OL:
    
    grid_size = (1 << (vlevel+1)) - 1;

    if ( !DistanceLevelCreated[vlevel+1] )
      computeDistanceVector(vlevel+1);

    if ( vlevel%2 == 0 )
      next = 4*index+3 + DistancesH[vlevel+1][2*coords[0]+2] + 
	DistancesV[vlevel+1][2*coords[1]+2];
    else
      next = 4*index+4 + DistancesH[vlevel+1][2*coords[0]+2] + 
	DistancesV[vlevel+1][2*coords[1]+2];
    coords[0] = (coords[0] << 1) + 2;
    coords[1] = (coords[1] << 1) + 2;
    if ( coords[0] > grid_size ) coords[0] = 0;
    if ( coords[1] > grid_size ) coords[1] = 0;

    (*level)++; 
    break; 
    
  case _SW_OL:
    
    grid_size = (1 << (vlevel+1)) - 1;

    if ( !DistanceLevelCreated[vlevel+1] )
      computeDistanceVector(vlevel+1);

    if ( vlevel%2 == 0 )
      next = 4*index+4 - DistancesH[vlevel+1][2*coords[0]] + 
	DistancesV[vlevel+1][2*coords[1]+2];
    else
      next = 4*index+3 - DistancesH[vlevel+1][2*coords[0]] + 
	DistancesV[vlevel+1][2*coords[1]+2];
    coords[0] = (coords[0] << 1) - 1;
    coords[1] = (coords[1] << 1) + 2;
    if ( coords[0] < 0 ) coords[0] = grid_size;
    if ( coords[1] > grid_size ) coords[1] = 0;

    (*level)++; 
    break; 

  }
 
  return next;   
}

template < class nd >
void Quadtree<nd>::precomputeCoords() {

  int i, j,
      depthm,
      index,
      findex,
      coords0,
      coords1,
      size;

  depthm = Depth - 1;

  /*********************************************************/
  /**     FIRST NODE: (0,0) -> START FROM 1 AND COMPUTE   **/
  /**  THE COORDINATES UNTIL LEVEL DEPTH-1                **/
  /*********************************************************/
  
  (*(Nodes)).Coords[0] = 0;   /** ROOT NODE COORDS **/
  (*(Nodes)).Coords[1] = 0;
 
  index = 0;                  /** CURRENT NODE **/

  for ( i = 0; i < depthm; i++ ) {
    
    size = _POW4[i];

    if ( i%2 == 0 ) {   /*** EVEN LEVEL ***/

      for ( j = 0; j < size; j++ ) {

	coords0 = (*(Nodes+index)).Coords[0];
	coords1 = (*(Nodes+index)).Coords[1];
	findex  = 4*index+1;
	  
	(*(Nodes+findex)).Coords[0] = 2*coords0+1;
	(*(Nodes+findex)).Coords[1] = 2*coords1;
	
	findex++;

	(*(Nodes+findex)).Coords[0] = 2*coords0;
	(*(Nodes+findex)).Coords[1] = 2*coords1;

	findex++;

	(*(Nodes+findex)).Coords[0] = 2*coords0+1;
	(*(Nodes+findex)).Coords[1] = 2*coords1+1;

	findex++;

	(*(Nodes+findex)).Coords[0] = 2*coords0;
	(*(Nodes+findex)).Coords[1] = 2*coords1+1;
	
	index++;
      }
      
    } else {            /*** ODD LEVEL ***/
      
      for ( j = 0; j < size; j++ ) {

	coords0 = (*(Nodes+index)).Coords[0];
	coords1 = (*(Nodes+index)).Coords[1];
	findex  = 4*index+1;
	  
	(*(Nodes+findex)).Coords[0] = 2*coords0;
	(*(Nodes+findex)).Coords[1] = 2*coords1;
	
	findex++;

	(*(Nodes+findex)).Coords[0] = 2*coords0+1;
	(*(Nodes+findex)).Coords[1] = 2*coords1;

	findex++;

	(*(Nodes+findex)).Coords[0] = 2*coords0;
	(*(Nodes+findex)).Coords[1] = 2*coords1+1;

	findex++;

	(*(Nodes+findex)).Coords[0] = 2*coords0+1;
	(*(Nodes+findex)).Coords[1] = 2*coords1+1;
	
	index++;
      }
    }
  }
}

template < class nd >
void Quadtree<nd>::printLocation(char l) {
  
  switch ( l ) {

  case _INSIDE_NODE:
    printf("_INSIDE_NODE\n");
    break;
  case _TOP_BORDER_NODE:
    printf("_TOP_BORDER_NODE\n");
    break;
  case _BOT_BORDER_NODE:
    printf("_BOT_BORDER_NODE\n");
    break;
  case _LEFT_BORDER_NODE:
    printf("_LEFT_BORDER_NODE\n");
    break;
  case _RIGHT_BORDER_NODE:
    printf("_RIGHT_BORDER_NODE\n");
    break;
  case _TR_CORNER_NODE:
    printf("_TR_CORNER_NODE\n");
    break;
  case _TL_CORNER_NODE:
    printf("_TL_CORNER_NODE\n");
    break;
  case _BR_CORNER_NODE:
    printf("_BR_CORNER_NODE\n");
    break;
  case _BL_CORNER_NODE:
    printf("_BL_CORNER_NODE\n");
    break;
  case _ROOT_NODE:
    printf("_ROOT_NODE\n");
    break;
  }
}



/**********************************************************************************/
/**                 INSTANCIATIONS OF THE TEMPLATE CLASS                        ***/
/**********************************************************************************/                  

template class Quadtree<QTElement>;



