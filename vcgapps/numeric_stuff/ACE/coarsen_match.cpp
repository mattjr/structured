/****************************************************************
*																*
* This file contains functions for coarsening and interpolation *
* based on edge-contraction. Due to the simplicity of this kind *
* of coarsening, it is implemented without constructing an      *
* interpolation matrix.											*
*																*
****************************************************************/

/****************************************************************
 * This file is based on the code of Chaco by Bruce Hendrickson *
 * and Robert Leland at Sandia National Laboratories            *				
 ****************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include "defs.h"
#include "structs.h"
#include "utils.h"

void      makev2cv(
/* Construct mapping from original graph vtxs to coarsened graph vtxs. */
				int      *mflag,		/* flag indicating vtx selected or not */
				int       nvtxs,		/* number of vtxs in original graph */
				int      *v2cv			/* mapping from vtxs to coarsened vtxs */
				);


int       maxmatch3(
					struct vtx_data **graph,	/* array of vtx data for graph */
					int       nvtxs,		/* number of vertices in graph */
					int      *mflag,		/* flag indicating vtx selected or not */
					int       using_ewgts		/* are edge weights being used? */
					);

void      makefgraph(
						struct vtx_data **graph,	/* array of vtx data for graph */
						int       nvtxs,		/* number of vertices in graph */
						int       nedges,		/* number of edges in graph */
						struct vtx_data ***pcgraph,	/* coarsened version of graph */
						int       cnvtxs,		/* number of vtxs in coarsened graph */
						int      *pcnedges,		/* number of edges in coarsened graph */
						int      *v2cv,			/* mapping from vtxs to coarsened vtxs */
						int       using_ewgts		/* are edge weights being used? */
			       );

void      coarsen_match(
						struct vtx_data **graph,	/* array of vtx data for graph */
						int       nvtxs,			/* number of vertices in graph */
						int       nedges,			/* number of edges in graph */
						struct vtx_data ***pcgraph,	/* coarsened version of graph */
						int      *pcnvtxs,			/* number of vtxs in coarsened graph */
						int      *pcnedges,			/* number of edges in coarsened graph */
						int     **pv2cv,			/* pointer to v2cv */
						int       using_ewgts,		/* are edge weights being used? */
						double * masses,
						double ** pcmasses
						)
{
    extern int COARSEN_VWGTS;	/* turn off vertex weights in coarse graph? */
    extern int COARSEN_EWGTS;	/* turn off edge weights in coarse graph? */
	int old_COARSEN_VWGTS, old_COARSEN_EWGTS;
    extern double coarsen_time;
    extern double match_time;
    int      *v2cv;		/* maps from vtxs to cvtxs */
    int      *mflag;		/* flag indicating vtx matched or not */
    int       cnvtxs;		/* number of vtxs in coarse graph */
    int       nmerged;		/* number of edges contracted */
	double * cmasses;
	int i;
    
    /* Allocate and initialize space. */
    v2cv = new int[nvtxs + 1];
    mflag = new int[nvtxs + 1];

    /* Find a maximal matching in the graph. */
	nmerged = maxmatch3(graph, nvtxs, mflag, using_ewgts);
    
    /* Now construct coarser graph by contracting along matching edges. */
    /* Pairs of values in mflag array indicate matched vertices. */
    /* A zero value indicates that vertex is unmatched. */

    makev2cv(mflag, nvtxs, v2cv);

	
    delete [] mflag;

    cnvtxs = nvtxs - nmerged;
	cmasses = new double[cnvtxs+1];
	for (i=1; i<=cnvtxs; i++)
		cmasses[i]=0; ///(10/7/2002, I changed it from "=1" to "=0")
	for (i=1; i<=nvtxs; i++)
		cmasses[v2cv[i]]+=masses[i];

    old_COARSEN_VWGTS = COARSEN_VWGTS;
	old_COARSEN_EWGTS = COARSEN_EWGTS;
	COARSEN_EWGTS = TRUE;
	COARSEN_VWGTS = FALSE;
	
	makefgraph(graph, nvtxs, nedges, pcgraph, cnvtxs, pcnedges, v2cv, using_ewgts);
    
	COARSEN_EWGTS = old_COARSEN_EWGTS;
	COARSEN_VWGTS = old_COARSEN_VWGTS;
	

    *pcnvtxs = cnvtxs;
    *pv2cv = v2cv;
	*pcmasses = cmasses;
}


/* Interpolate the eigenvector from the coarsened to the original graph.
   This may require regenerating mflag and v2cv arrays from merged array.

   I also need to undo the merged edges in the reverse order from that in
   which they were collapsed.
*/
void      interpolate_match(
			double  **vecs,			/* approximate eigenvectors for graph */
			double  **cvecs,		/* exact eigenvectors for coarse graph */
			int       ndims,		/* number of vectors to interpolate */
			struct vtx_data **graph,	/* array of vtx data for graph */
			int       nvtxs,		/* number of vertices in graph */
			int      *v2cv			/* mapping from vtxs to cvtxs */
			)
{
    double   *vec, *cvec;	/* pointers into vecs and vecs */
    int       i, j;		/* loop counters */

    /* Uncompress the coarse eigenvector by replicating matched values. */
    for (j = 1; j <= ndims; j++) {
		vec = vecs[j];
		cvec = cvecs[j];
		for (i = 1; i <= nvtxs; i++)
			vec[i] = cvec[v2cv[i]];
		}
		
}



void      makev2cv(
/* Construct mapping from original graph vtxs to coarsened graph vtxs. */
				int      *mflag,		/* flag indicating vtx selected or not */
				int       nvtxs,		/* number of vtxs in original graph */
				int      *v2cv			/* mapping from vtxs to coarsened vtxs */
				)
{
    int       i, j;		/* loop counters */

    j = 1;
    for (i = 1; i <= nvtxs; i++) {
	if (mflag[i] == 0 || mflag[i] > i)
	    v2cv[i] = j++;
	else
	    v2cv[i] = v2cv[mflag[i]];
    }
}

inline double    drandom()
{
    return (((double) rand()) / (RAND_MAX+1));
}


/* Randomly permute elements of an array. */

void      randomize(
					int      *array,		// array of integer values  
					int       n			    // number of values  
					)
{
    int       temp;		// holds value being swapped 
    int       i, j;		// loop counter  
    
    for (i=1; i<n; i++) {
		j=i+long_rand()%(n-i+1); // must use long_rand() (not rand()), since n may be greater than RAND_MAX
		temp=array[i];
		array[i]=array[j];
		array[j]=temp;
	}	
}
   
/* Find a maximal matching in a graph using simple greedy algorithm. */
/* Randomly permute vertices, and then have each select an unmatched */
/* neighbor. */

int       maxmatch3(
					struct vtx_data **graph,	/* array of vtx data for graph */
					int       nvtxs,		/* number of vertices in graph */
					int      *mflag,		/* flag indicating vtx selected or not */
					int       using_ewgts		/* are edge weights being used? */
					)
{
    extern int HEAVY_MATCH;	/* pick heavy edges in matching? */
    int      *order;		/* random ordering of vertices */
    int      *iptr, *jptr;	/* loops through integer arrays */
    double    prob_sum;		/* sum of probabilities to select from */
    double    val;		/* random value for selecting neighbor */
    float     ewgt;		/* edge weight */
    int       save;		/* neighbor vertex if only one active */
    int       vtx;		/* vertex to process next */
    int       neighbor;		/* neighbor of a vertex */
    int       nmerged;		/* number of edges in matching */
    int       i, j;		/* loop counters */
    
    /* First, randomly permute the vertices. */
    iptr = order = new int[nvtxs + 1];
    jptr = mflag;
    for (i = 1; i <= nvtxs; i++) {
		*(++iptr) = i;
		*(++jptr) = 0;
    }
    
	randomize(order, nvtxs);

    nmerged = 0;
    if (!using_ewgts || !HEAVY_MATCH) {	/* All edges equal. */
	for (i = 1; i <= nvtxs; i++) {
	    vtx = order[i];
	    if (mflag[vtx] == 0) {	/* Not already matched. */
		/* Add up sum of edge weights of neighbors. */
		prob_sum = 0;
		save = 0;
		for (j = 1; j < graph[vtx]->nedges; j++) {
		    neighbor = graph[vtx]->edges[j];
		    if (mflag[neighbor] == 0) {
			/* Set flag for single possible neighbor. */
			if (prob_sum == 0)
			    save = neighbor;
			else
			    save = 0;
			prob_sum += 1.0;
		    }
		}

		if (prob_sum != 0) {	/* Does vertex have contractible edges? */
		    nmerged++;
		    if (save != 0) {	/* Only one neighbor, special case. */
				mflag[vtx] = save;
				mflag[save] = vtx;
		    }
		    else {	/* Pick randomly neighbor. */
				val = drandom() * prob_sum * .999999;
				prob_sum = 0;
				for (j = 1; !mflag[vtx]; j++) {
					neighbor = graph[vtx]->edges[j];
					if (mflag[neighbor] == 0) {
						prob_sum += 1.0;
						if (prob_sum >= val) {
							mflag[vtx] = neighbor;
							mflag[neighbor] = vtx;
						}
					}
				}
		    }
		}
	    }
	}
    }

    else {			/* Choose heavy edges preferentially. */
	for (i = 1; i <= nvtxs; i++) {
	    vtx = order[i];
	    if (mflag[vtx] == 0) {	/* Not already matched. */
		/* Add up sum of edge weights of neighbors. */
		prob_sum = 0;
		save = 0;
		for (j = 1; j < graph[vtx]->nedges; j++) {
		    neighbor = graph[vtx]->edges[j];
		    if (mflag[neighbor] == 0) {
			/* Set flag for single possible neighbor. */
			if (prob_sum == 0)
			    save = neighbor;
			else
			    save = 0;
			ewgt = graph[vtx]->ewgts[j];
			prob_sum += ewgt;
		    }
		}

		if (prob_sum != 0) {	/* Does vertex have contractible edges? */
		    nmerged++;
		    if (save != 0) {	/* Only one neighbor, special case. */
			mflag[vtx] = save;
			mflag[save] = vtx;
		    }
		    else {	/* Pick randomly neighbor, skewed by edge weights. */
			val = drandom() * prob_sum * .999999;
			prob_sum = 0;
			for (j = 1; !mflag[vtx]; j++) {
			    neighbor = graph[vtx]->edges[j];
			    if (mflag[neighbor] == 0) {
				ewgt = graph[vtx]->ewgts[j];
				prob_sum += ewgt;
				if (prob_sum >= val) {
				    mflag[vtx] = neighbor;
				    mflag[neighbor] = vtx;
				}
			    }
			}
		    }
		}
	    }
	}
    }

    delete [] order;
    return nmerged;
}


static void makecv2v(
	int       nvtxs,		/* number of vertices in graph */
	int       cnvtxs,		/* number of vtxs in coarsened graph */
	int      *v2cv,			/* mapping from vtxs to coarsened vtxs */
	int      *cv2v_vals,		/* vtxs corresponding to each cvtx */
	int      *cv2v_ptrs		/* indices into cv2c_vals */
	)

{
    int       sum;		/* cumulative offests into vals array */
    int       i;		/* loop counter */

    /* First find number of vtxs associated with each coarse graph vtx. */

    for (i = 1; i <= cnvtxs + 1; i++) {
	cv2v_ptrs[i] = 0;
    }

    for (i = 1; i <= nvtxs; i++) {
	++cv2v_ptrs[v2cv[i] + 1];	/* +1 offsets and simplifies next loop. */
    }
    cv2v_ptrs[1] = 0;

    /* Now make this a cumulative total to index into cv2v_vals. */
    sum = 0;
    for (i = 2; i <= cnvtxs + 1; i++) {
	cv2v_ptrs[i] += sum;
	sum = cv2v_ptrs[i];
    }

    /* Now go ahead and set the cv2v_vals. */
    for (i = 1; i <= nvtxs; i++) {
	cv2v_vals[cv2v_ptrs[v2cv[i]]] = i;
	++cv2v_ptrs[v2cv[i]];
    }

    /* Finally, reset the cv2v_ptrs values. */
    for (i = cnvtxs; i; i--) {
	cv2v_ptrs[i] = cv2v_ptrs[i - 1];
    }
    cv2v_ptrs[1] = 0;
}




void      makefgraph(
						struct vtx_data **graph,	/* array of vtx data for graph */
						int       nvtxs,		/* number of vertices in graph */
						int       nedges,		/* number of edges in graph */
						struct vtx_data ***pcgraph,	/* coarsened version of graph */
						int       cnvtxs,		/* number of vtxs in coarsened graph */
						int      *pcnedges,		/* number of edges in coarsened graph */
						int      *v2cv,			/* mapping from vtxs to coarsened vtxs */
						int       using_ewgts		/* are edge weights being used? */
			       )
{
    extern int COARSEN_VWGTS;	/* turn off vertex weights in coarse graph? */
    extern int COARSEN_EWGTS;	/* turn off edge weights in coarse graph? */
    struct vtx_data **cgraph;	/* coarsened version of graph */
    struct vtx_data *links;	/* space for all the vertex data */
    struct vtx_data **gptr;	/* loops through cgraph */
    struct vtx_data *cgptr;	/* loops through cgraph */
    int      *iptr;		/* loops through integer arrays */
    int      *seenflag;		/* flags for vtxs already put in edge list */
    int      *sptr;		/* loops through seenflags */
    int      *cv2v_vals;	/* vtxs corresponding to each cvtx */
    int      *cv2v_ptrs;	/* indices into cv2v_vals */
    float    *eweights;		/* space for edge weights in coarsened graph */
    float    *ewptr;		/* loops through eweights */
    float    *fptr;		/* loops through eweights */
    float     ewgt;		/* edge weight */
    double    ewgt_sum;		/* sum of edge weights */
    int       nseen;		/* number of edges of coarse graph seen so far */
    int       vtx;		/* vertex in original graph */
    int       cvtx;		/* vertex in coarse graph */
    int       cnedges;		/* twice number of edges in coarsened graph */
    int       neighbor;		/* neighboring vertex */
    int       size;		/* space needed for coarsened graph */
    int      *edges;		/* space for edges in coarsened graph */
    int      *eptr;		/* loops through edges data structure */
    int       cneighbor;	/* neighboring vertex number in coarsened graph */
    int       i, j;		/* loop counters */
    
    /* Compute the number of vertices and edges in the coarsened graph, */
    /* and construct start pointers into coarsened edge array. */
    
    /* Construct mapping from original graph vtxs to coarsened graph vtxs. */
    cv2v_vals = new int[nvtxs];
    cv2v_ptrs = new int[cnvtxs + 2];
    makecv2v(nvtxs, cnvtxs, v2cv, cv2v_vals, cv2v_ptrs);

    /* Compute an upper bound on the number of coarse graph edges. */
    cnedges = nedges - (nvtxs - cnvtxs);

	typedef struct vtx_data * vtx_data_ptr;

    /* Now allocate space for the new graph.  Overallocate and realloc later. */
    *pcgraph = cgraph = new vtx_data_ptr[cnvtxs + 1];
    links = new struct vtx_data[cnvtxs];

    size = 2 * cnedges + cnvtxs;
    edges = new int[size];
    if (COARSEN_EWGTS) {
		ewptr = eweights = new float[size];
    }

    /* Zero all the seen flags. */
    seenflag = new int[cnvtxs + 1];
    sptr = seenflag;
    for (i = cnvtxs; i; i--) {
		*(++sptr) = 0;
    }

    /* Use the renumbering to fill in the edge lists for the new graph. */
    cnedges = 0;
    eptr = edges;
    ewgt = 1;

    sptr = cv2v_vals;
    for (cvtx = 1; cvtx <= cnvtxs; cvtx++) {
		nseen = 1;

		cgptr = cgraph[cvtx] = links++;

		if (COARSEN_VWGTS)
			cgptr->vwgt = 0;
		else
			cgptr->vwgt = 1;

		eptr[0] = cvtx;
		cgptr->edges = eptr;
		if (COARSEN_EWGTS) {
			cgptr->ewgts = ewptr;
		}
		else
			cgptr->ewgts = NULL;

		ewgt_sum = 0;
		for (i = cv2v_ptrs[cvtx + 1] - cv2v_ptrs[cvtx]; i; i--) {
			vtx = *sptr++;

			iptr = graph[vtx]->edges;
			if (using_ewgts)
				fptr = graph[vtx]->ewgts;
			for (j = graph[vtx]->nedges - 1; j; j--) {
			neighbor = *(++iptr);
			cneighbor = v2cv[neighbor];
			if (cneighbor != cvtx) {
				if (using_ewgts)
					ewgt = *(++fptr);
				ewgt_sum += ewgt;

				/* Seenflags being used as map from cvtx to index. */
				if (seenflag[cneighbor] == 0) {	/* New neighbor. */
				cgptr->edges[nseen] = cneighbor;
				if (COARSEN_EWGTS)
					cgptr->ewgts[nseen] = ewgt;
				seenflag[cneighbor] = nseen++;
				}
				else {	/* Already seen neighbor. */
				if (COARSEN_EWGTS)
					cgptr->ewgts[seenflag[cneighbor]] += ewgt;
				}
			}
			else if (using_ewgts)
				++fptr;
			}
		}

		/* Now clear the seenflag values. */
		iptr = cgptr->edges;
		for (j = nseen - 1; j; j--) {
			seenflag[*(++iptr)] = 0;
		}

		if (COARSEN_EWGTS)
			cgptr->ewgts[0] = -ewgt_sum;
		/* Increment pointers into edges list. */
		cgptr->nedges = nseen;
		eptr += nseen;
		if (COARSEN_EWGTS) {
			ewptr += nseen;
		}

		cnedges += nseen - 1;
    }

    delete [] seenflag;

    /* Form new vertex weights by adding those from contracted edges. */
    if (COARSEN_VWGTS) {
	gptr = graph;
	for (i = 1; i <= nvtxs; i++) {
	    cgraph[v2cv[i]]->vwgt += (*(++gptr))->vwgt;
	}
    }

    /* Reduce arrays to actual sizes */
    cnedges /= 2;
    size = 2 * cnedges + cnvtxs;
    eptr = edges;
    edges = (int *) realloc((char *) edges, (unsigned) size * sizeof(int));
    if (eptr != edges) {        /* Need to reset pointers in graph. */
        for (i = 1; i <= cnvtxs; i++) {
            cgraph[i]->edges = edges;
            edges += cgraph[i]->nedges;
        }
    }

    if (COARSEN_EWGTS) {
	ewptr = eweights;
	eweights = (float *) realloc((char *) eweights,
				      (unsigned) size * sizeof(float));
        if (ewptr != eweights) {        /* Need to reset pointers in graph. */
            for (i = 1; i <= cnvtxs; i++) {
                cgraph[i]->ewgts = eweights;
                eweights += cgraph[i]->nedges;
            }
        }
    }

    *pcnedges = cnedges;

    delete [] cv2v_ptrs;
    delete [] cv2v_vals;

 }


