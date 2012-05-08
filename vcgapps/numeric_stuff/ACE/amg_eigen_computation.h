#ifndef _amg_eigen_computation_H_
#define _amg_eigen_computation_H_

void   amg_eigen_computation(
						struct vtx_data **graph,	/* array of vtx data for graph */
						int       nvtxs,		/* number of vertices in graph */
						int       nedges,		/* number of edges in graph - needed only for debug info*/
						double  **evecs,		/* eigenvectors returned */
						int       ndims,		/* number of eigenvectors to calculate */
						int       vmax,			/* largest subgraph to stop coarsening */
						int       step,			/* current step number */
						double *masses,          /* 'masses' of the fine nodes */
						int       nstep		/* number of coarsenings between PI steps */
					);

#endif
