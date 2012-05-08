#ifndef _COARSEN_MATCH_H_
#define _COARSEN_MATCH_H_

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
						);

void      interpolate_match(
			double  **vecs,			/* interpolated approximate eigenvectors for graph */
			double  **cvecs,		/* exact eigenvectors for coarse graph */
			int       ndims,		/* number of vectors to interpolate */
			struct vtx_data **graph,/* array of vtx data for graph */
			int       nvtxs,		/* number of vertices in graph */
			int      *v2cv			/* mapping from vtxs to cvtxs */
			);
#endif
