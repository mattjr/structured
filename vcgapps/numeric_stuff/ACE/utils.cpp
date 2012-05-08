#include <malloc.h>
#include <stdlib.h>
#include <stdio.h>
#include "defs.h"
#include "structs.h"
#include "read_params.h"
#
void      free_graph(struct vtx_data **graph)
{
    
    if (graph != NULL) { 
		if (graph[1] != NULL) {
			if (graph[1]->ewgts != NULL)
					delete [] graph[1]->ewgts;
			if (graph[1]->edges != NULL) {
					delete [] graph[1]->edges;
			}
			delete [] graph[1];
		}
        delete [] graph;
    }
}


/* Free a graph data structure. For graphs where the adjacency
list of some nodes was allocated alone, so we cannot use the
function: 'free_graph'											 */
void      free_graph_ex(struct vtx_data **graph, int n)
{
    int       sfree();
	int i;

    if (graph != NULL) { 
		if (graph[1] != NULL) {
			for (i=1; i<=n; i++) {
				// don't use the costly 'sfree' here
				if (graph[i]->vwgt==-1) { 
					free(graph[i]->ewgts);
					free(graph[i]->edges);
				}
			}
			free((char *) graph[1]);
		}
		free((char *) graph);
    }
}





/* Change from a FORTRAN graph style to our graph data structure.     */
/* This function is taken from the code of Chaco by Bruce Hendrickson *
 * and Robert Leland at Sandia National Laboratories                  */


int       reformat(
					int      *start,		/* start of edge list for each vertex */
					int      *adjacency,		/* edge list data */
					int       nvtxs,		/* number of vertices in graph */
					int      *pnedges,		/* ptr to number of edges in graph */
					int      *vwgts,		/* weights for all vertices */
					float    *ewgts,		/* weights for all edges */
					struct vtx_data ***pgraph	/* ptr to array of vtx data for graph */
					)
{
    struct vtx_data **graph = NULL;	/* array of vtx data for graph */
    struct vtx_data *links = NULL;	/* space for data for all vtxs */
    int      *edges = NULL;	/* space for all adjacency lists */
    float    *eweights = NULL;	/* space for all edge weights */
    int      *eptr;		/* steps through adjacency list */
    int      *eptr_save;	/* saved index into adjacency list */
    float    *wptr;		/* steps through edge weights list */
    int       self_edge;	/* number of self loops detected */
    int       size;		/* length of all edge lists */
    double    sum;		/* sum of edge weights for a vtx */
    int       using_ewgts;	/* are edge weights being used? */
    int       using_vwgts;	/* are vertex weights being used? */
    int       i, j;		/* loop counters */
   
    using_ewgts = (ewgts != NULL);
    using_vwgts = (vwgts != NULL);

    graph = (struct vtx_data **) malloc((unsigned) (nvtxs + 1) * sizeof(struct vtx_data *));
    *pgraph = graph;
    if (graph == NULL) return(1);

    graph[1] = NULL;

    /* Set up all the basic data structure for the vertices. */
    /* Replace many small mallocs by a few large ones. */
    links = (struct vtx_data *) malloc((unsigned) (nvtxs) * sizeof(struct vtx_data));
    if (links == NULL) return(1);

    for (i = 1; i <= nvtxs; i++) {
	graph[i] = links++;
    }

    graph[1]->edges = NULL;
    graph[1]->ewgts = NULL;

    /* Now fill in all the data fields. */
    if (start != NULL)
	*pnedges = start[nvtxs] / 2;
    else
	*pnedges = 0;
    size = 2 * (*pnedges) + nvtxs;
    edges = (int *) malloc((unsigned) size * sizeof(int));
    if (edges == NULL) return(1);

    if (using_ewgts) {
	eweights = (float *) malloc((unsigned) size * sizeof(float));
        if (eweights == NULL) return(1);
    }

    if (start != NULL) {
        eptr = adjacency + start[0];
        wptr = ewgts;
    }
    self_edge = 0;

    for (i = 1; i <= nvtxs; i++) {
	if (using_vwgts)
	    graph[i]->vwgt = *(vwgts++);
	else
	    graph[i]->vwgt = 1;
	if (start != NULL)
	    size = start[i] - start[i - 1];
	else
	    size = 0;
	graph[i]->nedges = size + 1;
	graph[i]->edges = edges;
	*edges++ = i;
	eptr_save = eptr;
	for (j = size; j; j--) {
	    if (*eptr != i)
		*edges++ = *eptr++;
	    else {		/* Self edge, skip it. */
		if (!self_edge) {
		    printf("WARNING: Self edge (%d,%d) being ignored\n", i, i);
    		    if (fpStatsFile != NULL) {
		        fprintf(fpStatsFile,
			    "WARNING: Self edge (%d,%d) being ignored\n", i, i);
		    }
		}
		++self_edge;
		eptr++;
		--(graph[i]->nedges);
		--(*pnedges);
	    }
	}
	if (using_ewgts) {
	    graph[i]->ewgts = eweights;
	    eweights++;
	    sum = 0;
	    for (j = size; j; j--) {
		if (*eptr_save++ != i) {
		    sum += *wptr;
		    *eweights++ = *wptr++;
		}
		else
		    wptr++;
	    }
	    graph[i]->ewgts[0] = -sum;
	}
	else
	    graph[i]->ewgts = NULL;
    }
    if (self_edge > 1) {
	printf("WARNING: %d self edges were detected and ignored\n", self_edge);
        if (fpStatsFile != NULL) {
	    fprintf(fpStatsFile,
		"WARNING: %d self edges were detected and ignored\n", self_edge);
	}
    }

    return(0);
}

