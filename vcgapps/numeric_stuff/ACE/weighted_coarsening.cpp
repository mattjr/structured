#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include <time.h>
#include "defs.h"
#include "structs.h"
#include "sort.h"
#include "read_params.h"

extern int negativeMajority, positiveMajority;
int SortByDegrees = 0;


typedef float weights_type;


void      free_graph(struct vtx_data **graph);
	


void choose_representatives(
		struct vtx_data ** matrix, // the fine matrix
		int n,
		int * fine2coarse, // if v is a representative then fine2coarse[v] is its new name, otherwise
						// fine2coarse[v]==0
		double * degrees, // amount of connectivity of each fine node to rep's
		int *num_representatives, // number of coarse var's (what's called 'm')
		double min_connectivity, // each fine variable should be connected to
								 // the coarse variables with at least 'min_connectivity'
		double connectivity_growth, // in next passes increase the min_connectivity by 
									// this amount
		int max_nodes,  // max number of coarse var's (one kind of stopping condition)
		int max_passes  // max number of passes (another kind of stopping condition)
		)
{
	int * nodes; // a permutation of nodes 1..n, nodes[last+1..n]
	int i,j;
	int temp;
	int m; // number of coarse var's
	int passes;
	int last; // last non-representative var in the 'nodes' vector
	static int first_call=1;
	double total_connectivity, connectivity2representatives;
	struct vtx_data  *matrix_line;
	int node;
	int num_edges;
	double pos_weights, neg_weights;
	

    nodes = (int *) malloc((unsigned) (n+1)*sizeof(int));
	for (i=1; i<=n; i++)
		nodes[i]=i;
	// sort the nodes by degrees:
	if (!first_call && SortByDegrees)
		quicksort(matrix,nodes,1,n);
	// Order the nodes randomly:
	else {
		first_call=0;
		for (i=1; i<n; i++) {
			j=i+rand()%(n-i+1);
			temp=nodes[i];
			nodes[i]=nodes[j];
			nodes[j]=temp;
		}
	}
	//nodes[1]=3; nodes[2]=1; nodes[3]=2; nodes[4]=4; nodes[5]=5;
	
	// initialize the fine2coarse vector
	for (i=0; i<=n; i++)
		fine2coarse[i]=0;

	
	fine2coarse[nodes[n]]=1; // the *last* node is always a representative
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
				if (fine2coarse[matrix_line->edges[j]]) 
					connectivity2representatives+=fabs(matrix_line->ewgts[j]);
			}
			if (connectivity2representatives/total_connectivity < min_connectivity){
				// add to representatives:
				m++;
				fine2coarse[node]=m;
				nodes[i]=nodes[last];
				nodes[last]=node;
				last--;
			}
			else
				i++;
		}
		min_connectivity+=connectivity_growth;
	}

	*num_representatives = m; // should be equal to n-last
	
	
	positiveMajority=negativeMajority=0;
	num_edges=0;
	for (i=1; i<=last; i++) {
		node=nodes[i];
		matrix_line=matrix[node];
		pos_weights=neg_weights=0.0;
		for (j=0; j<matrix_line->nedges; j++) {
			if (fine2coarse[matrix_line->edges[j]]) {
				if (matrix_line->ewgts[j]>0) {
					pos_weights+=matrix_line->ewgts[j];
				}
				else if (matrix_line->ewgts[j]<0) {
					neg_weights+=matrix_line->ewgts[j];
					}
			}
		}
		if (pos_weights>(-neg_weights)) {
			degrees[node]=pos_weights;
			positiveMajority++;
		}
		else {
			degrees[node]=neg_weights;
			negativeMajority++;
		}
	}

	delete [] nodes;
	
}

/////////////////////////////////////////////////////////////////////////
// Till the end of the file, we provide an experimental implementation //
// that does all the steps of weighted interpolation at once, without  //
// constructing an interpolation matrix.                               //
// I DO NOT recommend using this version, as it proved to be slower    //
/////////////////////////////////////////////////////////////////////////


/* 
This function gets an n*n fine matrix (L) and 
and computes an m*m coarse matrix: A^T * L * A.
A is not created explicitly, but it is treated as 
a weighted interpolation matrix.
Space for the coarse matrix is allocated inside the function.


Notice that matrix dimensions start with 1 (not 0)

*/

//#define MIN_WEIGHT 0.01
#define MIN_WEIGHT 0.001
// Notice: coarse_matrix should be freed using 'free_graph_ex1(..)'
void compute_coarse_matrix_weighted_interpolation(
		struct vtx_data ** fine_matrix,
		struct vtx_data *** coarse_matrix,
		double ** pdegrees,
		int ** pfine2coarse,
		int n,
		int *pncvtxs,
		int num_fine_edges,
		int * num_coarse_edges, // not including self loops
		double * masses,
		double ** pcmasses
		) 
{
	extern int DEBUG_COARSEN;	/* debug flag for coarsening */
    struct vtx_data ** mat; // pointer to coarse_matrix
	int * place; // an auxiliary array
	struct vtx_data * links;
	int i,j,k,l;
	int m; // number of coarse nodes
	double * degrees;
	int * fine2coarse;
	int neighbor, num_neighbors;
	weights_type * ewgts;
	int * edges;
	int * coarse2fine;
	int node;
	int index1,index2,index3;
	weights_type weight1,weight2;
	double interpolation_weight1,interpolation_weight3;
	weights_type weight;
	int negative_edges;
	int nedges_allocate;
	int reserved_space;
	double * cmasses;
	struct vtx_data  *mat_i;
		
	
	fine2coarse = (int*) malloc((unsigned) (n + 1) * sizeof(int));
	*pfine2coarse = fine2coarse;
	
	degrees = (double*) malloc((unsigned) (n + 1) * sizeof(double));
	*pdegrees = degrees;
	
	choose_representatives(fine_matrix, n, fine2coarse, degrees, pncvtxs, 
						  0.04/*min_connectivity*/, 0.04 /*connectivity_growth*/,n,2/*max_passes*/);
		
	m = *pncvtxs; 
	
	// Allocating space for the coarse matrix:
	cmasses = (double*) malloc((unsigned)(m+1)*sizeof(double));
	*pcmasses = cmasses;

    mat = (struct vtx_data **) malloc((unsigned) (m + 1) * sizeof(struct vtx_data *));
    *coarse_matrix = mat;
    if (mat == NULL) return;
	
	// Set up all the basic data structure for the vertices. 
    // Replace many small mallocs by a large one. 
    links = (struct vtx_data *) malloc((unsigned) m * sizeof(struct vtx_data));
    if (links == NULL) return;

	reserved_space = nedges_allocate = 2*num_fine_edges + m; // each edge has two endpoints and 
															 // there are self loops (actually may 
															 // need more or less)
	edges  = (int*) malloc(nedges_allocate*sizeof(int));
	ewgts  = (weights_type*) malloc(nedges_allocate*sizeof(weights_type));

	if (edges==NULL || ewgts==NULL) {
		printf("MEMORY Problem");
		return;
	}

	for (i=1; i<=m; i++) {
		mat[i] = links++;
		mat[i]->nedges = 0;
		mat[i]->vwgt = 1; // don't free() i's edge lists
	}

	mat[1]->vwgt=-1; // it denotes that we have to free() the 1's edge lists
  
	
    place = (int *) malloc((unsigned) (m+1) * sizeof(int));
	for (i=1; i<=m; i++)
		place[i]=-1; 

	coarse2fine = (int *) malloc((unsigned) (m+1) * sizeof(int));
	for (i=1; i<=n; i++)
		if (fine2coarse[i]) //i is a representative
			coarse2fine[fine2coarse[i]]=i;
	
	
	// filling the coarse matrix. The (i,l) entry is the sum of: A_t_ij * L_jk * A_kl
	negative_edges=0;
	*num_coarse_edges=0;
	for (i=1; i<=m; i++) {
		mat_i = mat[i];
		if (reserved_space<m) { // there is a dangerous of running out of space
			reserved_space = nedges_allocate;
			edges  = (int*) malloc(nedges_allocate*sizeof(int));
			ewgts  = (weights_type*) malloc(nedges_allocate*sizeof(weights_type));
			mat_i->vwgt = -1; // mat[i]'s lists should be freed
			if (edges==NULL || ewgts==NULL) {
				printf("MEMORY Problem");
				return;
			}
		}
		node = coarse2fine[i];
		mat_i->edges=edges;
		mat_i->ewgts=ewgts;
		cmasses[i] = masses[node];
		num_neighbors=0;
		for (j=0; j<fine_matrix[node]->nedges; j++) {
			index1=fine_matrix[node]->edges[j];
			weight1=fine_matrix[node]->ewgts[j];
			if (fine2coarse[index1]) { // //index1 is a representative, build a direct (length 1) edge
				neighbor = fine2coarse[index1];
				if (place[neighbor] == -1) { // a new neighbor
					mat_i->edges[num_neighbors] = neighbor;
					mat_i->ewgts[num_neighbors] = weight1;
					place[neighbor] = num_neighbors;
					num_neighbors++;
				} 
				else {
					mat_i->ewgts[place[neighbor]] += weight1;
				}
				continue;
			}
			// if we are here, then index1 is not a representative
			
			interpolation_weight1=weight1/degrees[index1];

						
			if (interpolation_weight1>0) // we don't use negative interpolations
				cmasses[i] += masses[index1]*interpolation_weight1;


			for (k=0; k<fine_matrix[index1]->nedges; k++) {
				index2=fine_matrix[index1]->edges[k];
				weight2=fine_matrix[index1]->ewgts[k];
				
				if (fine2coarse[index2] && //index2 is a representative, build a length 2 edge
					(interpolation_weight1>0 || weight2*degrees[index1]>0)) { 
					neighbor = fine2coarse[index2];
					if (place[neighbor] == -1) { // a new neighbor
						mat_i->edges[num_neighbors] = neighbor;
						mat_i->ewgts[num_neighbors] = (weights_type) 
							((interpolation_weight1>0 ? interpolation_weight1*weight2 : 0) + 
							(weight2*degrees[index1] > 0 ? interpolation_weight1*weight2 : 0)); 
											
						place[neighbor] = num_neighbors;
						num_neighbors++;
					} 
					else {
						mat_i->ewgts[place[neighbor]] += (weights_type)
							((interpolation_weight1>0 ? interpolation_weight1*weight2 : 0) + 
							(weight2*degrees[index1] > 0 ? interpolation_weight1*weight2 : 0)); 					}
					continue;
				}

				if (interpolation_weight1<0)
					continue; // no reason to progress through a third edge, since
							  // we don't use negative interpolations
	
				
				for (l=0; l<fine_matrix[index2]->nedges; l++) {
					index3=fine_matrix[index2]->edges[l];
					if (fine2coarse[index3]) { // //index3 is a representative, build a length 3 edge
						interpolation_weight3 = fine_matrix[index2]->ewgts[l]/degrees[index2];
						if (interpolation_weight3<0)
							continue; // we don't use negative interpolations
						neighbor = fine2coarse[index3];
						if (place[neighbor] == -1) { // a new neighbor
							mat_i->edges[num_neighbors] = neighbor;
							mat_i->ewgts[num_neighbors] = 
								(weights_type) (interpolation_weight1*weight2*interpolation_weight3);
							place[neighbor] = num_neighbors;
							num_neighbors++;
						} 
						else {
							mat_i->ewgts[place[neighbor]] += 
								(weights_type)(interpolation_weight1*weight2*interpolation_weight3);
						}
					}
				} // for( l..
			} // for (k..
		} // for (j..
		
				
		// reinitialize 'place'
		for (j=0; j<num_neighbors; j++) 
			place[mat_i->edges[j]] = -1;
		
		
		if (mat_i->edges[0] != i) { // this condition cannot satisfied, so we can delete this block
			// put self loop as the edge number 0, for compability with other functions
			mat_i->edges[place[i]] = mat_i->edges[0];
			mat_i->ewgts[place[i]] = mat_i->ewgts[0];
			mat_i->edges[0] = i;
		}
		mat_i->ewgts[0] = 0; // since we eliminate small edges, and we want the sum of the line
							  // to be zero, we will recompute mat[i]->ewgts[0]


		for (j=1; j<num_neighbors; ) { 
			weight = mat_i->ewgts[j];
			if (fabs(weight)>MIN_WEIGHT) {
				mat_i->ewgts[0] -= weight;
				j++;					
				if (weight<0)
					negative_edges++;
			}
			else {
				num_neighbors--;
				mat_i->edges[j] = mat_i->edges[num_neighbors];
				mat_i->ewgts[j] = mat_i->ewgts[num_neighbors];
			}
		}
		
		mat_i->nedges = num_neighbors;
		(*num_coarse_edges)+=num_neighbors-1; // notice that we are not counting self loops
		edges+=num_neighbors;
		ewgts+=num_neighbors;
		reserved_space-=num_neighbors;
	} // for (i..


	(*num_coarse_edges)/=2; // we have count each edge twice
	negative_edges/=2;

	if (showStats) {
			fprintf(fpStatsFile,"NegMajority: %d, PosMajority: %d ",negativeMajority,positiveMajority);
			fprintf(fpStatsFile,"Neg%2.2f%% ",(float)negative_edges*100/(*num_coarse_edges));
	}
	
	// free auxiliary allocated data:
	delete [] coarse2fine;
	delete [] place;

	return;
	}
	

// Compute x = Ay
// space for x is allocated outside the function
// HERE, INTERPOLATION IS DONE WITHOUT USING AN INTERPOLATION MATRIX
// In general, when you have the interpolation matrix, you can use 
// the function 'interpolate_amg'

void interpolate_vector_weighted(
				double * x, // n components vector
				double * y, // m components vector
				int n,
				int m,
				struct vtx_data ** graph, // needed only for gs refinement
				double *degrees,
				int *fine2coarse		
				) {
	int i,j;
	double sum;
	struct vtx_data * g_line;

	double weight,tot_weights;


	for (i=1; i<=n; i++) {
		if (fine2coarse[i]) { // a representative
			x[i] = y[fine2coarse[i]];
		}
		else {
			sum = 0;
			g_line = graph[i];
			tot_weights = degrees[i];
			for (j=1; j<g_line->nedges; j++) {
				if (fine2coarse[g_line->edges[j]]) { // a representative neighbor
					weight = g_line->ewgts[j]/tot_weights;
					if (weight<0)
						continue; // don't use negative interpolations
					sum += weight*y[fine2coarse[g_line->edges[j]]];
				}
			}
			x[i] = sum;
		}
	}

}



