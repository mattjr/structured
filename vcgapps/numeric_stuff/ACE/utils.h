#ifndef _UTILS_H_
#define _UTILS_H_

#include <stdlib.h>

void      free_graph(struct vtx_data **graph);
void      free_graph_ex(struct vtx_data **graph, int n);

inline unsigned long long_rand() {
   unsigned long n = (unsigned long)(rand()) | ((unsigned long)(rand())<<16);
   return n;
}

int       reformat(
					int      *start,		/* start of edge list for each vertex */
					int      *adjacency,		/* edge list data */
					int       nvtxs,		/* number of vertices in graph */
					int      *pnedges,		/* ptr to number of edges in graph */
					int      *vwgts,		/* weights for all vertices */
					float    *ewgts,		/* weights for all edges */
					struct vtx_data ***pgraph	/* ptr to array of vtx data for graph */
					);

#endif
