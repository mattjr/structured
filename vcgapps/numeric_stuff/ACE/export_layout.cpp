#include <string.h>
#include <stdio.h>

extern int draw_edges;

void export_layout(int  *start, int *adjacency, int n, double ** coords, FILE *fp, 
				   char * filename, double coarsen_time, double pi_time, 
				   double running_time, int * dims2show, int ndims) {

	int i,j;
	double max_coordinate=-9e10;
	double min_coordinate=9e10;
	double scale;
	int dim1=-1,dim2=-1;

	if (dims2show==NULL) {
		dim1=1; 
		dim2=2;
	}
	else {
		for (i=1; i<=ndims; i++)
			if (dims2show[i]) {
				if (dim1==-1)
					dim1=i;
				else if (dim2==-1)
				    dim2=i;
			}
	}

	
	// graph name:
	fprintf(fp,"%s\n",filename);
	// layout algorithm:
	fprintf(fp,"Spectral\\AMG\n");
	// number of nodes:
	fprintf(fp, "%d\n", -n);
	// vertex type - 2 means 'dimensionless'
	fprintf(fp, "%d\n", 2);
	// stats - coarsen_time, pi_time, running_time:
	fprintf(fp, "%lf %lf %d\n",coarsen_time,pi_time,(int)running_time);

	for (i=1; i<=n; i++) {
		if (coords[dim1][i]>max_coordinate)
			max_coordinate=coords[dim1][i];
		if (coords[dim2][i]>max_coordinate)
			max_coordinate=coords[dim2][i];
		if (coords[dim1][i]<min_coordinate)
			min_coordinate=coords[dim1][i];
		if (coords[dim2][i]<min_coordinate)
			min_coordinate=coords[dim2][i];
	}

	scale = 10000.0 / (max_coordinate-min_coordinate);
	
	
	// print coordinates:
	for (i=1; i<=n; i++) { 
		fprintf(fp, "%d %d ", (int)(scale*(coords[dim1][i]-min_coordinate)), 
							  (int)(scale*(coords[dim2][i]-min_coordinate)));
	}



	// sizes of nodes 
	for (i=0; i<n; i++) 
		fprintf(fp, "1 1 ");
	
	// edges:
	if (draw_edges) {
		for (i=1; i<=n; i++) 
			for (j=start[i-1]; j<start[i]; j++)
				if (i<adjacency[j])
					fprintf(fp, "%ld-%ld ", (i-1),adjacency[j]-1);
	}
	else
		printf("Not drawing edges\n");
		

	fclose(fp);
}
