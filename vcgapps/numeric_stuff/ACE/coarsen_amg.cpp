#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include "defs.h"
#include "structs.h"
#include "read_params.h"
#include "sort.h"
#include "utils.h"


int negativeMajority=0, positiveMajority=0;



typedef float weights_type;


/*
This function gets an n*n fine matrix (L) and produces
an n*m interpolation matrix (A), using "weighted_interpolation"
Method:
We perform several sweeps through the variables. At each sweep 
we add a variable to the representatives, if its connectivity to
the current representatives is too low
Notice:
The first entry in every line of the matrix must be a self loop
We assume that the energy to minimize is -x^T*L*x (or in other
words that L is the minus of the laplacian, so we treat the 
non-diagonal entries as the edge weights when interpolating
*/

int construct_weighted_interpolation(
		struct vtx_data ** matrix, // the fine matrix
		struct vtx_data *** interpolation_matrix,
		int n,
		int *coarse_size, // number of coarse var's (what's called 'm')
		double min_connectivity, // each fine variable should be connected to
								 // the coarse variables with at least 'min_connectivity'
		double connectivity_growth, // in next passes increase the min_connectivity by 
									// this amount
		int max_nodes,  // max number of coarse var's (one kind of stopping condition)
		int max_passes  // max number of passes (another kind of stopping condition)
		)
{
	int * nodes; // a permutation of nodes 1..n, nodes[last+1..n]
	int * inv_nodes;
	int i,j,k;
	int temp;
	int * isrepresentative; // a 0/1 vector 
	int m; // number of coarse var's
	int passes;
	int last; // last non-representative var in the 'nodes' vector
	double total_connectivity, connectivity2representatives;
	struct vtx_data  *matrix_line, *A_line, *links;
	int node;
	struct vtx_data ** A; // The interpolation matrix (*interpolation_matrix)
	double * degrees;
	int num_edges;
	weights_type * edge_weights, * edge_weights_ptr;
	int * edges, * edges_ptr;
	double pos_weights, neg_weights;
	int pos_nedges, neg_nedges;
	
	

    nodes = new int[n+1];
	for (i=1; i<=n; i++)
		nodes[i]=i;
	// sort the nodes by degrees:
	// quicksort(matrix,nodes,1,n);
	
	// Order the nodes randomly:
	for (i=1; i<n; i++) {
		j=i+long_rand()%(n-i+1); // note that n may be larger than RAND_MAX, so we are not using rand()
		temp=nodes[i];
		nodes[i]=nodes[j];
		nodes[j]=temp;
	}

	// initialize the isrepresentative vector
	isrepresentative = new int[n+1];
	for (i=0; i<=n; i++)
		isrepresentative[i]=0;

	
	isrepresentative[nodes[n]]=1; // the *last* node is always a representative
	m=1;
	passes=0;
	last=n-1;

    // Loop through the nodes and choose the representatives:
	while (m<=max_nodes && passes<max_passes) {
		passes++;
		for (i=1; i<=last && m<=max_nodes;) {
			node=nodes[i];
			total_connectivity=0;
			connectivity2representatives=0;
			matrix_line=matrix[node];
			for (j=1; j<matrix_line->nedges; j++) { // skip the first entry which is a self loop
				total_connectivity+=fabs(matrix_line->ewgts[j]);
				if (isrepresentative[matrix_line->edges[j]]) 
					connectivity2representatives+=fabs(matrix_line->ewgts[j]);
			}
			if (connectivity2representatives/total_connectivity < min_connectivity){
				// add to representatives:
				m++;
				isrepresentative[node]=1;
				nodes[i]=nodes[last];
				nodes[last]=node;
				last--;
			}
			else
				i++;

		}
		min_connectivity+=connectivity_growth;
	}

	// Allocating space for the interpolation matrix:

	A = (struct vtx_data **) malloc((unsigned) (n + 1) * sizeof(struct vtx_data *));
	*interpolation_matrix = A;
	if (A == NULL) return(1);
	
	// Set up all the basic data structure for the vertices. 
    // Replace many small mallocs by a few large ones. 
    links = (struct vtx_data *) malloc((unsigned) n * sizeof(struct vtx_data));
    if (links == NULL) return(1);

	for (i=1; i<=n; i++) {
		A[i] = links++;
		A[i]->nedges = 0;
	}

	
	degrees = (double *) malloc((unsigned) (n + 1) * sizeof(double));
	if (degrees == NULL) return(1);
	 for (i=1; i<=n; i++)
		degrees[i]=0;

	positiveMajority=negativeMajority=0;
	num_edges=0;
	for (i=1; i<=last; i++) {
		node=nodes[i];
		matrix_line=matrix[node];
		pos_weights=neg_weights=0.0;
		pos_nedges=neg_nedges=0;
		for (j=0; j<matrix_line->nedges; j++) {
			if (isrepresentative[matrix_line->edges[j]]) {
				if (matrix_line->ewgts[j]>0) {
					pos_weights+=matrix_line->ewgts[j];
					pos_nedges++;
				}
				else if (matrix_line->ewgts[j]<0) {
					neg_weights+=matrix_line->ewgts[j];
					neg_nedges++;
				}
			}
		}
		if (pos_weights>(-neg_weights)) {
			degrees[node]=pos_weights;
			A[node]->nedges=pos_nedges;
			num_edges+=pos_nedges;
			A[node]->vwgt=1; // 1 indicates positive
			positiveMajority++;
		}
		else {
			degrees[node]=neg_weights;
			A[node]->nedges=neg_nedges;
			num_edges+=neg_nedges;
			A[node]->vwgt=0; // 0 indicates negative
			negativeMajority++;
		}
	}
	

	// allocate space for 'edges' (we add 'm' units for the 'm' representatives):
	edges_ptr = edges = (int *) malloc((unsigned) (num_edges+m) * sizeof(int));
	edge_weights_ptr = edge_weights = (weights_type *) malloc((unsigned) (num_edges+m) * sizeof(weights_type));
	
	// The name of a coarse node related the representative nodes[i] is i-last, so we
	// need the index of nodes[i] (which is i), for each i>last
	inv_nodes = (int *) malloc((unsigned) (1+n) * sizeof(int));
	for (i=last+1; i<=n; i++)
		inv_nodes[nodes[i]]=i-last;
	
	
	for (node=1; node<=n; node++) {
		if (isrepresentative[node]) {
			A_line = A[node];
			A_line->edges = edges_ptr;
			A_line->ewgts = edge_weights_ptr;
			A_line->edges[0] = inv_nodes[node];
			A_line->ewgts[0] = 1.0;
			A_line->nedges = 1;
			edges_ptr++;
			edge_weights_ptr++;
		}
		else {
			matrix_line = matrix[node];
			A_line = A[node];
			A_line->edges = edges_ptr;
			A_line->ewgts = edge_weights_ptr;
			k = 0;
			if (A_line->vwgt==1) { // positive majority
				for (j=0; j<matrix_line->nedges; j++)
					if (isrepresentative[matrix_line->edges[j]] && matrix_line->ewgts[j]>0) { 
						A_line->edges[k] = inv_nodes[matrix_line->edges[j]];
						A_line->ewgts[k] = (weights_type) (matrix_line->ewgts[j]/degrees[node]);
						k++;
					}
			}
			else {	// negative majority
				for (j=0; j<matrix_line->nedges; j++)
					if (isrepresentative[matrix_line->edges[j]] && matrix_line->ewgts[j]<0) { 
						A_line->edges[k] = inv_nodes[matrix_line->edges[j]];
						A_line->ewgts[k] = (weights_type) (matrix_line->ewgts[j]/degrees[node]);
						k++;
					}
			}

			edges_ptr += A_line->nedges;
			edge_weights_ptr += A_line->nedges; 
		}
	}
	
	

	
 	delete [] degrees;
	delete [] nodes;
	delete [] isrepresentative;
	delete [] inv_nodes;	
	*coarse_size=m;
	return 0;
}


/***************************************************************
 Construct interpolation matrix using the edge-contraction.
 In practice, for this kind of interpolation it would be much 
 faster to use the functions in the file "coarsen_match", which
 do not build the interpolation matrix explicitly
 ***************************************************************/


int construct_interpolation_matching(
		struct vtx_data ** matrix, // the fine matrix
		struct vtx_data *** interpolation_matrix,
		int n,
		int *coarse_size // number of coarse var's (what's called 'm')
		)
{
	extern int HEAVY_MATCH;	/* pick heavy edges in matching? */	
	int old_HEAVY_MATCH=HEAVY_MATCH;
	int * mflag = (int *) malloc((unsigned)(n+1)*sizeof(int));
	int i,j,neighbor;
	struct vtx_data ** A;
	struct vtx_data * links;
	weights_type  * edge_weights_ptr;
	int  * edges_ptr;
	extern int MATCH_TYPE;

	int maxmatch3(struct vtx_data **graph,int  nvtxs, int  *mflag, int using_ewgts); 

	
	HEAVY_MATCH=0; // That's because I till have a problem with negative weights!!!!!!!!!!!!
	maxmatch3(matrix,n,mflag,1); 
	HEAVY_MATCH=old_HEAVY_MATCH;
	
	// rename vertices:
	for (j=-1,i=1; i<=n; i++) {
		if (mflag[i]<0) // already renamed
			continue;
		else if (mflag[i]==0) { // unmatched
			mflag[i]=j; 
			j--;
		}
		else { // matched
			neighbor=mflag[i];
			mflag[i]=mflag[neighbor]=j;
			j--;
		}
	}

	*coarse_size=-j-1;

	// build A:
	A = (struct vtx_data **) malloc((unsigned) (n + 1) * sizeof(struct vtx_data *));
	*interpolation_matrix = A;
	if (A == NULL) return(1);
	
	// Set up all the basic data structure for the vertices. 
    // Replace many small mallocs by a few large ones. 
    links = (struct vtx_data *) malloc((unsigned) n * sizeof(struct vtx_data));
    if (links == NULL) return(1);
	// allocate space for 'edges' 
	edges_ptr = (int *) malloc((unsigned) (n) * sizeof(int));
	edge_weights_ptr = (weights_type *) malloc((unsigned) (n) * sizeof(weights_type));
	
	for (i=1; i<=n; i++) {
		A[i] = links++;
		A[i]->nedges = 1;
		A[i]->edges = edges_ptr++;
		A[i]->edges[0] = -mflag[i];
		A[i]->ewgts = edge_weights_ptr++;
		A[i]->ewgts[0] = 1.0;
	}


	delete [] mflag;

	return 0;
}	


/*************************************************************************
A third kind of interpolation not mentioned in the paper.

The idea is to first make edge-contraction, and to lower the energy
using gauss-seidel sweeps. The result is a kind of weighted-interpolation
that does not have any representatives, but *all* nodes get their val's
from several coarse nodes.

AT THIS STAGE WE DO NOR RECOMMEND USING THIS METHOD!!!

**************************************************************************/

#define SELF_LOOP_WEIGHT 1.0
int construct_interpolation_matching_gs(
		struct vtx_data ** matrix, // the fine matrix
		struct vtx_data *** interpolation_matrix,
		int n,
		int *coarse_size, // number of coarse var's (what's called 'm')
		int matrix_edges
		)
{
	extern int HEAVY_MATCH;	/* pick heavy edges in matching? */	
	extern int MATCH_TYPE;
	int old_HEAVY_MATCH=HEAVY_MATCH;
	int * cluster = (int *) malloc((unsigned)(n+1)*sizeof(int));
	int i,j,neighbor;
	struct vtx_data ** A;
	struct vtx_data * links;
	struct vtx_data * Aline, * Mline;
	weights_type  * edge_weights_ptr;
	int  * edges_ptr;
	int * place;
	int csize; // num of coarse nodes
	int num_edges;
	double pos_weights, neg_weights;
	int left,right;
	int tmp_edge;

	int maxmatch3(struct vtx_data **graph,int  nvtxs, int  *mflag, int using_ewgts); 

		
	HEAVY_MATCH=0; // That's because I till have a problem with negative weights!!!!!!!!!!!!
		maxmatch3(matrix,n,cluster,1); 
	HEAVY_MATCH=old_HEAVY_MATCH;

	// rename vertices:
	for (j=-1,i=1; i<=n; i++) {
		if (cluster[i]<0) // already renamed
			continue;
		else if (cluster[i]==0) { // unmatched
			cluster[i]=j; 
			j--;
		}
		else { // matched
			neighbor=cluster[i];
			cluster[i]=cluster[neighbor]=j;
			j--;
		}
	}

	for (i=1; i<=n; i++)
		cluster[i] = -cluster[i];

	csize = -j-1;
	*coarse_size = csize;

	// build A:
	A = (struct vtx_data **) malloc((unsigned) (n + 1) * sizeof(struct vtx_data *));
	*interpolation_matrix = A;
	if (A == NULL) return(1);
	
	// Set up all the basic data structure for the vertices. 
    // Replace many small mallocs by a few large ones. 
    links = (struct vtx_data *) malloc((unsigned) n * sizeof(struct vtx_data));
    if (links == NULL) return(1);
	// allocate space for 'edges'
	// Notice that because we should allocte at once (for using "free_graph()), 
	// we allocate extra (wasted) memory 
	edges_ptr = (int *) malloc((unsigned) (2*matrix_edges+n) * sizeof(int));
	edge_weights_ptr = (weights_type *) malloc((unsigned) (2*matrix_edges+n) * sizeof(weights_type));
	
	place = (int *) malloc((unsigned) (csize+1) * sizeof(int));
	for (i=1; i<=csize; i++)
		place[i]=-1;

	positiveMajority=negativeMajority=0;
	for (i=1; i<=n; i++) {
		Mline = matrix[i];
		A[i] = links++;
		Aline = A[i];
		num_edges=0;
		Aline->edges = edges_ptr;
		Aline->ewgts = edge_weights_ptr;
		for (j=1; j<Mline->nedges; j++) { // skip the first edge which is a self loop
			if (place[cluster[Mline->edges[j]]]==-1) { // new neighbor
				Aline->edges[num_edges] = cluster[Mline->edges[j]];
				Aline->ewgts[num_edges] = Mline->ewgts[j]; 
				place[cluster[Mline->edges[j]]] = num_edges;
				num_edges++;
			}
			else  // already found neighbor
				Aline->ewgts[place[cluster[Mline->edges[j]]]] += Mline->ewgts[j];
		}
		// add the self loop:
		if (SELF_LOOP_WEIGHT!=0) {
			if (place[cluster[i]]==-1) {
				Aline->edges[num_edges] = cluster[i];
				Aline->ewgts[num_edges] = SELF_LOOP_WEIGHT; 
				num_edges++;
			}
			else
				Aline->ewgts[place[cluster[i]]] += SELF_LOOP_WEIGHT;
		}
		
		
		pos_weights = neg_weights = 0.0;
		for (j=0; j<num_edges; j++) {
			if (Aline->ewgts[j]>0)
				pos_weights += Aline->ewgts[j];
			else
				neg_weights += Aline->ewgts[j];
		}

		
	    // 	move the positive (or negative) edges to the head and normalize the weights
		if (pos_weights>=(-neg_weights)) {
			positiveMajority++;
			left=0; right=num_edges-1;
			while (left<right) {
				if (Aline->ewgts[left]>0) {
					left++;
					Aline->ewgts[left] /= (weights_type)pos_weights;
				}
				else if (Aline->ewgts[right]<=0) 
					right--;
				else {
					Aline->ewgts[left]=Aline->ewgts[right]/(weights_type)pos_weights;
					tmp_edge = Aline->edges[left];
					Aline->edges[left] = Aline->edges[right];
					Aline->edges[right] = tmp_edge; // we need this for initializing 'place'
					Aline->ewgts[right] = -1.0; // put here any negative value for (*)
					left++; right--;
				}
			}
			if (Aline->ewgts[left]>0) { // (*)
				left++;
				Aline->ewgts[left] /= (weights_type)pos_weights;
			}
		}
		else {
			negativeMajority++;
			left=0; right=num_edges-1;
			while (left<right)  {
				if (Aline->ewgts[left]<0) {
					Aline->ewgts[left] /= (weights_type)neg_weights;
					left++;
				}
				else if (Aline->ewgts[right]>=0) 
					right--;
				else {
					Aline->ewgts[left]=Aline->ewgts[right]/(weights_type)neg_weights;
					tmp_edge = Aline->edges[left];
					Aline->edges[left] = Aline->edges[right];
					Aline->edges[right] = tmp_edge; // we need this for initializing 'place'
					Aline->ewgts[right] = 1.0; // put here any positive value for (**)
					left++; right--;
				}
			}
			if (Aline->ewgts[left]<0) { // (**)
				Aline->ewgts[left] /= (weights_type)neg_weights;
				left++;
			}
		}

		Aline->nedges = left;
			
		// reinitialize the vector "place":
		for (j=0; j<num_edges; j++) 
			place[Aline->edges[j]] = -1;
		
		edges_ptr += left;
		edge_weights_ptr += left;
	}

	

	delete [] cluster;
	delete [] place;

	return 0;


}	



/* 
This function gets an n*n fine matrix (L) and an 
n*m interpolation matrix (A), 
and computes an m*m coarse matrix: A^T * L * A.
Space for the coarse matrix is allocated inside the function.

Notice that matrix dimensions start with 1 (not 0)

Another function that does the same job probably faster
is "compute_coarse_matrix_2steps" which appears below

*/

//#define MIN_WEIGHT 0
#define MIN_WEIGHT 0.001
int compute_coarse_matrix(
		struct vtx_data ** fine_matrix,
		struct vtx_data *** coarse_matrix,
		struct vtx_data ** interpolation_matrix,
		int n,
		int m,
		int * num_coarse_edges // not including self loops
		) 
{
	extern int DEBUG_COARSEN;	/* debug flag for coarsening */
    struct vtx_data ** mat; // pointer to coarse_matrix
	struct vtx_data ** A; // pointer to the interpolation matrix
	struct vtx_data ** A_t; // the transpose of the interpolation matrix
	weights_type * storage; // an auxiliary array
	int * visited;
	int * neighbors;
	struct vtx_data * links;
	int i,j,k,l;
	int neighbor, max_neighbor;
	int num_edges;
	weights_type * edge_weights, * edge_weights_ptr;
	int * edges, * edges_ptr;
	int index1,index2,index3;
	weights_type weight1,weight2;
	weights_type weight;
	int negative_edges;
	
	A=interpolation_matrix;

	// Computation of A's transpose:
	//----------------------------
	A_t = (struct vtx_data **) malloc((unsigned) (m + 1) * sizeof(struct vtx_data *));
	// Set up all the basic data structure for the vertices. 
    // Replace many small mallocs by a few large ones. 
    links = (struct vtx_data *) malloc((unsigned) m * sizeof(struct vtx_data));
    if (links == NULL) return(1);

	for (i=1; i<=m; i++) {
		A_t[i] = links++;
		A_t[i]->nedges = 0;
	}

	// first compute the 'degree' of each entry:
	num_edges=0;
	for (i=1; i<=n; i++) 
		for (j=0; j<A[i]->nedges; j++) {
			A_t[A[i]->edges[j]]->nedges++;
			num_edges++;
		}
	// allocate space for 'edges':
	edges_ptr = edges = (int *) malloc((unsigned) num_edges * sizeof(int));
	edge_weights_ptr = edge_weights = (weights_type *) malloc((unsigned) num_edges * sizeof(weights_type));
	for (i=1; i<=m; i++) {
		A_t[i]->edges=edges_ptr;
		A_t[i]->ewgts=edge_weights_ptr;
		edges_ptr+=A_t[i]->nedges;
		edge_weights_ptr+=A_t[i]->nedges;
	}

	// fill the A_T matrix:
	for (i=1; i<=m; i++) 
		A_t[i]->nedges = 0;

	for (i=1; i<=n; i++) 
		for (j=0; j<A[i]->nedges; j++) {
			neighbor=A[i]->edges[j];
			A_t[neighbor]->edges[A_t[neighbor]->nedges]=i;
			A_t[neighbor]->ewgts[A_t[neighbor]->nedges]=A[i]->ewgts[j];
			A_t[neighbor]->nedges++;
		}
	


	// Allocating space for the coarse matrix:

    mat = (struct vtx_data **) malloc((unsigned) (m + 1) * sizeof(struct vtx_data *));
    *coarse_matrix = mat;
    if (mat == NULL) return(1);
	
	// Set up all the basic data structure for the vertices. 
    // Replace many small mallocs by a large one. 
    links = (struct vtx_data *) malloc((unsigned) m * sizeof(struct vtx_data));
    if (links == NULL) return(1);

	for (i=1; i<=m; i++) {
		mat[i] = links++;
		mat[i]->nedges = 0;
	}
  
	
    storage = (weights_type *) malloc((unsigned) (m+1) * sizeof(weights_type));
	visited = (int *) malloc((unsigned) (m+1) * sizeof(int));
	neighbors = (int *) malloc((unsigned) (m+1) * sizeof(int)); 
	for (i=1; i<=m; i++)
		visited[i]=0; 
	
	
	// filling the coarse matrix. The (i,l) entry is the sum of: A_t_ij * L_jk * A_kl
	negative_edges=0;
	*num_coarse_edges=0;
	for (i=1; i<=m; i++) {
		max_neighbor=0;
		for (j=0; j<A_t[i]->nedges; j++) {
			index1=A_t[i]->edges[j];
			weight1=A_t[i]->ewgts[j];
			for (k=0; k<fine_matrix[index1]->nedges; k++) {
				index2=fine_matrix[index1]->edges[k];
				weight2=fine_matrix[index1]->ewgts[k];
				for (l=0; l<A[index2]->nedges; l++) {
					index3=A[index2]->edges[l];
					if (!visited[index3]) { // a new entry (should be really -10, but for safety...)
						neighbors[max_neighbor]=index3;
						storage[index3]=weight1*weight2*A[index2]->ewgts[l]; // notice that this may be NEGATIVE
						visited[index3]=1;
						max_neighbor++;
					}
					else
						storage[index3]+=weight1*weight2*A[index2]->ewgts[l];
				}
			}
		}
		
		if (max_neighbor == 0) return(1); // some error

		// don't use the costly smalloc here
		mat[i]->edges = (int *) malloc((unsigned) max_neighbor * sizeof(int));
		mat[i]->ewgts = (weights_type *) malloc((unsigned) max_neighbor * sizeof(weights_type));
		
		
		// put self loop as the first edge for compability with Chaco
		mat[i]->edges[0] = i;
		//mat[i]->ewgts[0] = storage[i]; // because we round to zero and we want the line sum 
										 // to be 0, we will sum all weights
		mat[i]->ewgts[0] = 0;
		visited[i] = 0;
		
		for (k=1,j=0; j<max_neighbor; j++) { 
			if (neighbors[j] != i) {
				weight = storage[neighbors[j]];
				if (fabs(weight)>MIN_WEIGHT) {
					mat[i]->ewgts[0] -= weight;
					mat[i]->edges[k] = neighbors[j];
					mat[i]->ewgts[k] = weight;
					k++;
					if (weight<0)
						negative_edges++;
				}
				visited[neighbors[j]] = 0;
			}
		}
		mat[i]->nedges = k;
		mat[i]->vwgt=-1; // should be freed
		(*num_coarse_edges)+=k-1; // notice that we are not counting self loops
	}


	(*num_coarse_edges)/=2; // we have count each edge twice
	negative_edges/=2;

	if (showStats) {
			fprintf(fpStatsFile,"NegMajority: %d, PosMajority: %d ",negativeMajority,positiveMajority);
			fprintf(fpStatsFile,"Neg%2.2f%% \n",(float)negative_edges*100/(*num_coarse_edges));
	}
	
	// free auxiliary allocated data:
	delete [] neighbors;
	delete [] storage;
	delete [] visited;
	free_graph(A_t);

	return 0;
	}
	


/* 
This function gets an n*n fine matrix (L) and an 
n*m interpolation matrix (A), 
and computes an m*m coarse matrix: A^T * L * A.
Space for the coarse matrix is allocated inside the function.

The computation is done in two steps: first compute A^T * L
and then (A^T * L) * A

This found to be faster in some cases than the above
 'compute_coarse_matrix'


Notice that matrix dimensions start with 1 (not 0)

*/

//#define MIN_WEIGHT 0.01
#define MIN_WEIGHT 0.001
int compute_coarse_matrix_2steps(
		struct vtx_data ** fine_matrix,
		struct vtx_data *** coarse_matrix,
		struct vtx_data ** interpolation_matrix,
		int n,
		int m,
		int num_fine_edges,
		int * num_coarse_edges // not including self loops
		) 
{
	extern int DEBUG_COARSEN;	/* debug flag for coarsening */
    struct vtx_data ** mat; // pointer to coarse_matrix
	struct vtx_data ** A; // pointer to the interpolation matrix
	struct vtx_data ** A_t; // the transpose of the interpolation matrix
	struct vtx_data ** At_L; // A_t*L
	struct vtx_data * mat_i;
	int * place; // an auxiliary array
	struct vtx_data * links;
	int i,j,k;
	int neighbor, max_neighbor;
	int num_edges;
	weights_type * edge_weights, * edge_weights_ptr;
	int * edges, * edges_ptr;
	int index1,index2;
	weights_type weight1,weight2;
	weights_type weight;
	int negative_edges;
	int nedges_allocate;
	int reserved_space;

	extern int total_negative_edges;
        extern int total_coarse_edges;
	extern double max_neg_percentage;



	void      free_graph_ex(struct vtx_data **graph,int n);

	A=interpolation_matrix;

	// Computation of A's transpose:
	//----------------------------
	A_t = (struct vtx_data **) malloc((unsigned) (m + 1) * sizeof(struct vtx_data *));
	// Set up all the basic data structure for the vertices. 
    // Replace many small mallocs by a few large ones. 
    links = (struct vtx_data *) malloc((unsigned) m * sizeof(struct vtx_data));
    if (links == NULL) return(1);

	for (i=1; i<=m; i++) {
		A_t[i] = links++;
		A_t[i]->nedges = 0;
	}

	// first compute the 'degree' of each entry:
	num_edges=0;
	for (i=1; i<=n; i++) 
		for (j=0; j<A[i]->nedges; j++) {
			A_t[A[i]->edges[j]]->nedges++;
			num_edges++;
		}
	// allocate space for 'edges':
	edges_ptr = edges = (int *) malloc((unsigned) num_edges * sizeof(int));
	edge_weights_ptr = edge_weights = (weights_type *) malloc((unsigned) num_edges * sizeof(weights_type));
	for (i=1; i<=m; i++) {
		A_t[i]->edges=edges_ptr;
		A_t[i]->ewgts=edge_weights_ptr;
		edges_ptr+=A_t[i]->nedges;
		edge_weights_ptr+=A_t[i]->nedges;
	}

	// fill the A_T matrix:
	for (i=1; i<=m; i++) 
		A_t[i]->nedges = 0;

	for (i=1; i<=n; i++) 
		for (j=0; j<A[i]->nedges; j++) {
			neighbor=A[i]->edges[j];
			A_t[neighbor]->edges[A_t[neighbor]->nedges]=i;
			A_t[neighbor]->ewgts[A_t[neighbor]->nedges]=A[i]->ewgts[j];
			A_t[neighbor]->nedges++;
		}
	

	// compute A_t*L (At_L)
	
	// Allocating space:	
	At_L = (struct vtx_data **) malloc((unsigned) (m + 1) * sizeof(struct vtx_data *));
	links = (struct vtx_data *) malloc((unsigned) m * sizeof(struct vtx_data));
	
	for (i=1; i<=m; i++) 
		At_L[i] = links++;
	
	place = (int *) malloc((unsigned) (n + 1) * sizeof(int));
	for (i=1; i<=n; i++)
		place[i] = -1;

	nedges_allocate = 2*num_fine_edges+m;
	reserved_space = 0;
	for (i=1; i<=m; i++) {
		// fill the i'th row
		mat_i = At_L[i];
		if (reserved_space<n) { // maybe there will be not enough space (max degree is n)
			edges = (int *) malloc((unsigned) nedges_allocate * sizeof(int));
			edge_weights = (weights_type *) malloc((unsigned) nedges_allocate * sizeof(weights_type));
			mat_i->vwgt = -1; // the edge-list of node i should be freed
			reserved_space = nedges_allocate;
		}
		else
			mat_i->vwgt = 1; // the edge-list of node i should not be freed

		mat_i->edges = edges;
		mat_i->ewgts = edge_weights;
		max_neighbor = 0;
		for (j=0; j<A_t[i]->nedges; j++) {
			index1 = A_t[i]->edges[j];
			weight1 = A_t[i]->ewgts[j];
			for (k=0; k<fine_matrix[index1]->nedges; k++) {
				index2 = fine_matrix[index1]->edges[k];
				weight2 = fine_matrix[index1]->ewgts[k];
				if (place[index2]==-1) { // a new neighbor
					*edges = index2;
					*edge_weights = weight1*weight2;
					place[index2] = max_neighbor;
					edges++;
					edge_weights++;
					max_neighbor++;
				}
				else  // an existing neighbor
					mat_i->ewgts[place[index2]]+=weight1*weight2;
			}
		}

		// reinitialize place
		for (j=0; j<max_neighbor; j++)
			place[mat_i->edges[j]] = -1;
		
		mat_i->nedges = max_neighbor;
		reserved_space-=max_neighbor;
	}
		
			
	// compute (A_t*L)*A (mat)
	
	// Allocating space:	
	mat = (struct vtx_data **) malloc((unsigned) (m + 1) * sizeof(struct vtx_data *));
	*coarse_matrix = mat;
	links = (struct vtx_data *) malloc((unsigned) m * sizeof(struct vtx_data));
	
	for (i=1; i<=m; i++) 
		mat[i] = links++;
	
	
	nedges_allocate = 2*num_fine_edges+m;
	reserved_space = 0;
	negative_edges=0;
	*num_coarse_edges=0;
	for (i=1; i<=m; i++) {
		// fill the i'th row
		mat_i = mat[i];
		if (reserved_space<m) { // maybe there will be not enough space (max degree is m)
			edges = (int *) malloc((unsigned) nedges_allocate * sizeof(int));
			edge_weights = (weights_type *) malloc((unsigned) nedges_allocate * sizeof(weights_type));
			mat_i->vwgt = -1; // the edge-list of node i should be freed
			reserved_space = nedges_allocate;
		}
		else
			mat_i->vwgt = 1; // the edge-list of node i should not be freed

		mat_i->edges = edges;
		mat_i->ewgts = edge_weights;
		max_neighbor = 0;
		for (j=0; j<At_L[i]->nedges; j++) {
			index1 = At_L[i]->edges[j];
			weight1 = At_L[i]->ewgts[j];
			for (k=0; k<A[index1]->nedges; k++) {
				index2 = A[index1]->edges[k];
				weight2 = A[index1]->ewgts[k];
				if (place[index2]==-1) { // a new neighbor
					mat_i->edges[max_neighbor] = index2;
					mat_i->ewgts[max_neighbor] = weight1*weight2;
					place[index2] = max_neighbor;
					max_neighbor++;
				}
				else  // an existing neighbor
					mat_i->ewgts[place[index2]]+=weight1*weight2;
			}
		}
	
		// put self loop as the edge number 0, for compability with other functions
		mat_i->edges[place[i]] = mat_i->edges[0];
		mat_i->ewgts[place[i]] = mat_i->ewgts[0];
		mat_i->edges[0] = i;
	
		mat_i->ewgts[0] = 0; // since we eliminate small edges, and we want the sum of the line
							  // to be zero, we will recompute mat[i]->ewgts[0]
		
			// reinitialize place
		for (j=0; j<max_neighbor; j++)
			place[mat_i->edges[j]] = -1;


		for (j=1; j<max_neighbor; ) { 
			weight = mat_i->ewgts[j];
			if (weight>MIN_WEIGHT) {
				mat_i->ewgts[0] -= weight;
				j++;
			}
			else if (weight<-MIN_WEIGHT) {
				mat_i->ewgts[0] -= weight;
				j++;
				negative_edges++;
			}
			else { // skip a small edge
				max_neighbor--;
				mat_i->edges[j] = mat_i->edges[max_neighbor];
				mat_i->ewgts[j] = mat_i->ewgts[max_neighbor];
			}
		}
		
		mat_i->nedges = max_neighbor;
		reserved_space-=max_neighbor;
		edges += max_neighbor;
		edge_weights += max_neighbor;
		*num_coarse_edges += max_neighbor;
	}
		
			
	(*num_coarse_edges)/=2; // we have count each edge twice
	negative_edges/=2;

	if (showStats) {
			fprintf(fpStatsFile,"NegMajority: %d, PosMajority: %d ",negativeMajority,positiveMajority);
			fprintf(fpStatsFile,"Neg%2.2f%% \n",(float)negative_edges*100/(*num_coarse_edges));
	}
	total_coarse_edges += *num_coarse_edges;
	total_negative_edges+=negative_edges;
	if ((float)negative_edges*100/(*num_coarse_edges)>max_neg_percentage)
		max_neg_percentage=(float)negative_edges*100/(*num_coarse_edges);

	
	
	// free auxiliary allocated data:
	delete [] place;
	free_graph(A_t);
	free_graph_ex(At_L,m);

	return 0;
}
	

// Compute x = Ay
// space for x is allocated outside the function
void interpolate_amg(
				struct vtx_data ** A, // interpolation matrix
				double * x, // n components vector
				double * y, // m components vector
				int n,
				int m,
				struct vtx_data ** graph // needed only for gs refinement
				) {
	int i,j;
	double sum;
	struct vtx_data * A_line;

	

	for (i=1; i<=n; i++) {
		sum = 0;
		A_line = A[i];
		for (j=0; j<A_line->nedges; j++)
			sum += A_line->ewgts[j]*y[A_line->edges[j]];
		x[i] = sum;
	}
	
}




// compute the 'mass' of each coarse node, which is the sum of its 
// respected column in the interpolation matrix - A, weighted with 
// masses of the fine nodes.
// the vector coarse_masses should be allocated for at least m+1 entries outside
// this function.
void compute_masses(struct vtx_data ** A, double * fine_masses, double * coarse_masses, int n, int m) {
	int i,j;
	struct vtx_data * A_line;
	double mass_i;

	for (i=1; i<=m; i++)
		coarse_masses[i]=0;

	for (i=1; i<=n; i++) {
		A_line = A[i];
		mass_i = fine_masses[i];
		for (j=0; j<A_line->nedges; j++) {
			coarse_masses[A_line->edges[j]]+=A_line->ewgts[j]*mass_i;
		}
	}
	
}







