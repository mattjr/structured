#ifndef _STRUCTS_H_
#define _STRUCTS_H_ 

/* An array of these stores all the data for the graph/matrix. */
struct vtx_data {
	int vwgt;		/* weight of vertex */
	int nedges;		/* number of neighbors of vertex in subgraph */
				/* Note: above always includes self-edge first */
	int *edges;		/* neighbor list in subgraph numbering scheme */
	float *ewgts;		/* weights of all the edges */
				/* Note: above 2 fields have self-edge first */
};


#endif
