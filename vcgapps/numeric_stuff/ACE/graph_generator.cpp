#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#undef min
#undef max
#include "defs.h"
#include "structs.h"



void makeCircle(int nvtxs, char * filename) {
	FILE * fp;
	int i,n;

	
	n=nvtxs;

	fp = fopen(filename, "w");
	
	fprintf(fp, "%% %d vertices circle\n", n);
	fprintf(fp,"%d %d\n",n,n);
	for (i=1; i<=n; i++) 
		fprintf(fp,"%d %d\n", (i==1 ? n : i-1), (i==n ? 1 : i+1));

	fclose(fp);

}

void makeStar(int nvtxs, char * filename) {
	FILE * fp;
	int i,n;

	
	n=nvtxs;

	fp = fopen(filename, "w");
	
	fprintf(fp, "%% %d vertices circle\n", n);
	fprintf(fp,"%d %d\n",n,2*n-2);
	for (i=2; i<=n; i++) 
		fprintf(fp,"%d ", i);
	fprintf(fp,"\n");

	for (i=2; i<=n; i++) 
		fprintf(fp,"%d %d %d\n", (i==2 ? n : i-1), (i==n ? 2 : i+1), 1);

	fclose(fp);

}

void makeTorus(int * nvtxs, int dim1, int dim2, char * filename) {
	FILE * fp;
	int i,j,n;
	
	*nvtxs=dim1*dim2;
	n=*nvtxs;

	fp = fopen(filename, "w");
	
	fprintf(fp, "%% %d x %d torus (%d vertices)\n", dim1,dim2,n);
	fprintf(fp,"%d %d\n",n,2*n);
	for (i=0; i<dim1; i++)
		for (j=0; j<dim2; j++) 
			// build the edges of the j-th node in the i-th circle:
			fprintf(fp, "%d %d %d %d\n",
			i*dim2+((j+1)%dim2)+1,i*dim2+((dim2+j-1)%dim2)+1,((i+1)%dim1)*dim2+j+1,((dim1+i-1)%dim1)*dim2+j+1);
	
	fclose(fp);

}

void makeCylinder(int * nvtxs, int dim1, int dim2, char * filename) {
	FILE * fp;
	int i,j,n;
	
	*nvtxs=dim1*dim2;
	n=*nvtxs;

	fp = fopen(filename, "w");
	
	fprintf(fp, "%% %d x %d cylinder (%d vertices)\n", dim1,dim2,n);
	fprintf(fp,"%d %d\n",n,2*n-dim2);
	
	for (j=0; j<dim2; j++) 
			// build the edges of the j-th node in the first circle:
			fprintf(fp, "%d %d %d\n",
			((j+1)%dim2)+1,((dim2+j-1)%dim2)+1,dim2+j+1);
	
	for (i=1; i<dim1-1; i++)
		for (j=0; j<dim2; j++) 
			// build the edges of the j-th node in the i-th circle:
			fprintf(fp, "%d %d %d %d\n",
			i*dim2+((j+1)%dim2)+1,i*dim2+((dim2+j-1)%dim2)+1,((i+1)%dim1)*dim2+j+1,((dim1+i-1)%dim1)*dim2+j+1);
	
	for (j=0; j<dim2; j++)  
			// build the edges of the j-th node in the last circle:
			fprintf(fp, "%d %d %d\n",
			(dim1-1)*dim2+((j+1)%dim2)+1,(dim1-1)*dim2+((dim2+j-1)%dim2)+1,(dim1-2)*dim2+j+1);
	

	fclose(fp);

}

void makeSquareGrid(int * nvtxs, int dim1, int dim2, char * filename, int connect_corners,int partial) {
	FILE * fp;
	int i,j,n;
	

	*nvtxs=dim1*dim2;
	n=*nvtxs;

	fp = fopen(filename, "w");
	
	fprintf(fp, "%% %d x %d square grid (%d vertices)\n", dim1,dim2,n);
	fprintf(fp,"%d %d \n",n,2*n-dim2-dim1+(connect_corners ? 2 : 0) - (partial ? (4*dim1/6-2*dim1/6)*(4*dim2/6-2*dim2/6) : 0));
	
	for (i=0; i<dim1; i++)
		for (j=0; j<dim2; j++) {
			// write the neighbors of the node i*dim2+j+1
			if (j>0 && (!partial || j<=2*dim2/6 || j>4*dim2/6 || i<=2*dim1/6 || i>4*dim1/6))
				fprintf(fp, "%d ", i*dim2+j);
			if (j<dim2-1 && (!partial || j<2*dim2/6 || j>=4*dim2/6 || i<=2*dim1/6 || i>4*dim1/6))
				fprintf(fp, "%d ", i*dim2+j+2);			
			if (i>0)
				fprintf(fp, "%d ", (i-1)*dim2+j+1);
			if (i<dim1-1)
				fprintf(fp, "%d ", (i+1)*dim2+j+1);
			if (connect_corners==1) {
				if (i==0 && j==0) // upper left
					fprintf(fp, "%d ", (dim1-1)*dim2+dim2);
				else if (i==(dim1-1) && j==0) // lower left
					fprintf(fp, "%d ", dim2);
				else if (i==0 && j==(dim2-1)) // upper right
					fprintf(fp, "%d ", (dim1-1)*dim2+1);
				else if (i==(dim1-1) && j==(dim2-1)) // lower right
					fprintf(fp, "%d ", 1);
			}
			else if (connect_corners==2) {
				if (i==0 && j==0) // upper left
					fprintf(fp, "%d ", dim2);
				else if (i==(dim1-1) && j==0) // lower left
					fprintf(fp, "%d ", (dim1-1)*dim2+dim2);
				else if (i==0 && j==(dim2-1)) // upper right
					fprintf(fp, "%d ", 1);
				else if (i==(dim1-1) && j==(dim2-1)) // lower right
					fprintf(fp, "%d ", (dim1-1)*dim2+1);
			}
			fprintf(fp, "\n");
		}

	fclose(fp);
}

void makeEiffel(int * nvtxs, char * filename) {
	int n;
	FILE * fp;
	
	*nvtxs = 5;
	n = *nvtxs;

	fp = fopen(filename, "w");
	
	fprintf(fp, "%%Eiffel tower - 5 vertices\n");
	fprintf(fp, "5 7\n");

	// neighbors of 1:
	fprintf(fp, "2 4\n");
	// neighbors of 2:
	fprintf(fp, "1 3 4 5\n");
	// neighbors of 3:
	fprintf(fp, "2 4\n");
	// neighbors of 4:
	fprintf(fp, "1 2 3 5\n");
	// neighbors of 5:
	fprintf(fp, "2 4\n");

	fclose(fp);
}

void makeBinaryTree(int * nvtxs, int depth, char * filename) {
	FILE * fp;
	int i,n;
	
	*nvtxs=(int)(pow(2.0,depth)-1);
	n=*nvtxs;

	fp = fopen(filename, "w");
	
	fprintf(fp, "%% Depth %d binary tree (%d vertices)\n", depth,n);
	fprintf(fp,"%d %d\n",n,n-1);
	for (i=0; i<n; i++) {
		if (2*i+2<n) // not a leaf
			fprintf(fp, "%d %d ",2*i+2,2*i+3);
		if (i>0) // not a root
			fprintf(fp, "%d ",(i-1)/2+1);
		fprintf(fp,"\n");
	}
	fclose(fp);

}

void constructSierpinski(int v1, int v2, int v3, int depth, struct vtx_data ** graph);
void makeSierpinski(int * nvtxs, int depth, char * filename) {
	struct vtx_data ** graph;
	struct vtx_data * links;
	int * edges;
	int n;
	int nedges;
	int i,j;
	FILE * fp;
	void  free_graph(struct vtx_data **graph);


	n=3*(1+((int)(pow(3.0,(double)depth)+0.5)-1)/2);
	*nvtxs=n;
	
	nedges = (int)(pow(3.0,depth+1.0)+0.5);

	graph = (struct vtx_data **) malloc((n+1)*sizeof(struct vtx_data *));
	links = (struct vtx_data *) malloc((n+1)*sizeof(struct vtx_data));
	edges = (int *) malloc((4*n)*sizeof(int));

	for (i=1; i<=n; i++) {
		graph[i] = links++;
		graph[i]->edges = edges;
		edges+=4;
		graph[i]->nedges=0;
	}

	constructSierpinski(1, 2, 3, depth, graph);

	fp = fopen(filename, "w");
		
	fprintf(fp, "%% Sierpinski graph: Depth %d, %d vertices, %d edges\n", depth,n,nedges);
	fprintf(fp,"%d %d \n",n,nedges); 
	for (i=1; i<=n; i++) {
		// write the neighbors of the node i
		for (j=0; j<graph[i]->nedges; j++) {
			fprintf(fp, "%d ", graph[i]->edges[j]);
		}
		fprintf(fp, "\n");
	}

	fclose(fp);

	free(graph[1]->edges);
	free(graph[1]);
	free(graph);
}

void constructSierpinski(int v1, int v2, int v3, int depth, struct vtx_data ** graph) {
	static int last_used_node_name=3;
	int v4,v5,v6;

	int nedges;

	if (depth>0) {
		v4=++last_used_node_name;
		v5=++last_used_node_name;
		v6=++last_used_node_name;
		constructSierpinski(v1, v4, v5, depth-1, graph);
		constructSierpinski(v2, v5, v6, depth-1, graph);
		constructSierpinski(v3, v4, v6, depth-1, graph);
		return;
	}

	// depth==0, Construct graph:

	nedges = graph[v1]->nedges;
	graph[v1]->edges[nedges++]=v2;
	graph[v1]->edges[nedges++]=v3;
	graph[v1]->nedges = nedges;

	nedges = graph[v2]->nedges;
	graph[v2]->edges[nedges++]=v1;
	graph[v2]->edges[nedges++]=v3;
	graph[v2]->nedges = nedges;
	
	nedges = graph[v3]->nedges;
	graph[v3]->edges[nedges++]=v1;
	graph[v3]->edges[nedges++]=v2;
	graph[v3]->nedges = nedges;

	return;

}


void make_hypercube(int * nvtxs, int dim, char * filename) {
	FILE * fp;
	int i,j,n;
	int neighbor;
	
	*nvtxs=1<<dim;
	n=*nvtxs;

	fp = fopen(filename, "w");
	
	fprintf(fp, "%% Dimension %d hyper-cube (%d vertices)\n", dim,n);
	fprintf(fp,"%d %d\n",n,dim*n/2);
	for (i=0; i<n; i++) {
		for (j=0; j<dim; j++) {
			neighbor = (i^(1<<j))+1;
			fprintf(fp, "%d ",neighbor);
		}
		fprintf(fp,"\n");
	}
	fclose(fp);
}





