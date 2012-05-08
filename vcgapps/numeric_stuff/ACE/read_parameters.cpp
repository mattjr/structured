
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "defs.h"


typedef enum{unknown,grid,folded_grid,partial_grid,partial_folded_grid,
		     circle,tree,torus,cylinder,sierpinski,hypercube,star} GraphType;

typedef enum{weighted, contraction, non_amg} CoarseningMethod;



CoarseningMethod  coarseningMethod = weighted ; // contraction ;
int numPasses = 2 ;
double minConnectivity = 0.05 ;
double connectivityGrowth = 0.05 ;
int showStats = FALSE ;
FILE * fpStatsFile = NULL ;
int draw_edges = 1 ;
double power_iteration_tol = 1e-7 ;
double rqi_tol = 1e-3 ;
int relative_tolerance = 1;

int levels_between_refinement=1;
int MAKE_VWGTS=FALSE;
int HEAVY_MATCH = FALSE;	        /* Encourage heavy match edges? (TRUE/FALSE) */
int COARSEN_VWGTS = FALSE;//TRUE;   /* Sum vtx weights in coarsening? (TRUE/FALSE) */
int COARSEN_EWGTS = FALSE;//TRUE;   /* Sum edge weights in coarsening? (TRUE/FALSE) */

	


void tolower_str(char *str) {
	while (*str) {
		if (isupper(*str))
			*str=tolower(*str);
		str++;
	}
}



#define MAX_LEN 100
int read_parameters(char * filename, GraphType *graphType, int *graphSize1, int *graphSize2,
				char * graphFilename, char * coordinatesFilename, int *useGdrawLayoutFormat,
				int * ndims, int ** pdims2show, int *minSizeToCoarsen) 
{
	
	FILE * fp=fopen(filename,"r");

	int * dimensions2show;
	
	int i,dimension;

	char statsFilename[100];

	char graph_kind[100];
	
	char str[MAX_LEN+1];
	char * pstr;
	
	if (fp==NULL) {
		printf("Error. Cannot open parametes file: %s\n",filename);
		//getchar();
		exit(1);
	}

	// first initalize all vals with values that
	// tell us that no value was read
	*graphType=unknown;
	*graphSize1=*graphSize2=-1;
	strcpy(graphFilename,"graph.graph");
	*coordinatesFilename=0;
	*useGdrawLayoutFormat=0;
	draw_edges = 1;
	showStats=0;
	*statsFilename=0;
	power_iteration_tol=1e-7;
	rqi_tol=-1;
	relative_tolerance=1;	
	*minSizeToCoarsen=50;
	coarseningMethod=weighted;
	numPasses=2;
	minConnectivity=0.05;
	connectivityGrowth=0.05;
	dimensions2show = NULL;
	*ndims=2;
		
	while (fgets(str,MAX_LEN,fp)!=NULL) {
		// skip blanks:
		pstr = str + strspn(str, " \t");
		tolower_str(pstr);
		if (strncmp(pstr,"graphkind",9)==0) {
			pstr += 9;
			pstr = pstr + strspn(pstr, ":- \t");
			sscanf(pstr,"%s",graph_kind);
			if (strncmp(pstr,"grid",4)==0) 
				*graphType = grid;
			else if (strncmp(pstr,"folded grid",11)==0 || strncmp(pstr,"folded_grid",11)==0) 
				*graphType = folded_grid;
			else if (strncmp(pstr,"partial grid",12)==0 || strncmp(pstr,"partial_grid",12)==0) 
				*graphType = partial_grid;
			else if (strncmp(pstr,"partial folded grid",19)==0 || strncmp(pstr,"partial_folded_grid",19)==0) 
				*graphType = partial_folded_grid;
			else if (strncmp(pstr,"circle",6)==0 || strncmp(pstr,"cycle",5)==0) 
				*graphType = circle;
			else if (strncmp(pstr,"tree",4)==0) 
				*graphType = tree;
			else if (strncmp(pstr,"torus",4)==0) 
				*graphType = torus;
			else if (strncmp(pstr,"cylinder",8)==0) 
				*graphType = cylinder;
			else if (strncmp(pstr,"sierpinski",10)==0) 
				*graphType = sierpinski;
			else if (strncmp(pstr,"hypercube",9)==0) 
				*graphType = hypercube;
			else if (strncmp(pstr,"star",4)==0) 
				*graphType = star;
			else
				*graphType = unknown;
		}
		else if (strncmp(pstr,"graphsize1",10)==0) {
			pstr += 10;
			pstr = pstr + strspn(pstr, ":- \t");
			sscanf(pstr,"%d",graphSize1);
		}
		else if (strncmp(pstr,"graphsize2",10)==0) {
			pstr += 10;
			pstr = pstr + strspn(pstr, ":- \t");
			sscanf(pstr,"%d",graphSize2);
		}
		else if (strncmp(pstr,"inputfile",9)==0) {
			pstr += 9;
			pstr = pstr + strspn(pstr, ":- \t");
			sscanf(pstr,"%s",graphFilename);			
		}
		else if (strncmp(pstr,"outputfile",10)==0) {
			pstr += 10;
			pstr = pstr + strspn(pstr, ":- \t");
			sscanf(pstr,"%s",coordinatesFilename);			
		}
		else if (strncmp(pstr,"outputlayoutformat",18)==0) {
			pstr += 18;
			pstr = pstr + strspn(pstr, ":- \t");
			if (strncmp(pstr,"gdraw",5)==0)	
				*useGdrawLayoutFormat = 1;
			else
				*useGdrawLayoutFormat = 0;
		}
		else if (strncmp(pstr,"drawedges",9)==0) {
			pstr += 9;
			pstr = pstr + strspn(pstr, ":- \t");
			if (strncmp(pstr,"true",4)==0 || strncmp(pstr,"yes",3)==0)
				draw_edges = 1;
			else
				draw_edges = 0;
		}
		else if (strncmp(pstr,"weightverticesbydegrees",23)==0) {
			pstr += 23;
			pstr = pstr + strspn(pstr, ":- \t");
			if (strncmp(pstr,"true",4)==0 || strncmp(pstr,"yes",3)==0)	
				MAKE_VWGTS = 1;
			else
				MAKE_VWGTS = 0;
		}
		else if (strncmp(pstr,"showstatistics",14)==0) {
			pstr += 14;
			pstr = pstr + strspn(pstr, ":- \t");
			if (strncmp(pstr,"true",4)==0 || strncmp(pstr,"yes",3)==0)	
				showStats = 1;
			else
				showStats = 0;
		}
		
		else if (strncmp(pstr,"statisticsfile",14)==0) {
			pstr += 14;
			pstr = pstr + strspn(pstr, ":- \t");
			sscanf(pstr,"%s",statsFilename);
		}
		else if (strncmp(pstr,"dimensions2show",15)==0) {
			pstr += 15;
			pstr = pstr + strspn(pstr, ":- \t");
			if (dimensions2show!=NULL) 
				free(dimensions2show);
			dimensions2show = (int*) malloc((*ndims+1)*sizeof(int));
			for (i=1; i<=*ndims; i++)
				dimensions2show[i]=0;
			while (sscanf(pstr,"%d",&dimension)!=EOF) {
				if (dimension>=1)
					dimensions2show[dimension]=1;
				*ndims = max(*ndims,dimension);
				pstr = strpbrk(pstr," \t\n");
				pstr = pstr + strspn(pstr, ":- \t");			
			}
		}
		else if (strncmp(pstr,"poweriterationtolerance",23)==0) {
			pstr += 23;
			pstr = pstr + strspn(pstr, ": \t");
			sscanf(pstr,"%lf",&power_iteration_tol);
		}
		else if (strncmp(pstr,"rqitolerance",12)==0) {
			pstr += 12;
			pstr = pstr + strspn(pstr, ": \t");
			sscanf(pstr,"%lf",&rqi_tol);
		}
		else if (strncmp(pstr,"relativethreshold",17)==0 || strncmp(pstr,"relativetolerance",17)==0) { 
			pstr += 17;
			pstr = pstr + strspn(pstr, ":- \t");
			sscanf(pstr,"%d",&relative_tolerance);
		}
		else if (strncmp(pstr,"minsizetocoarsen",16)==0) {
			pstr += 16;
			pstr = pstr + strspn(pstr, ":- \t");
			sscanf(pstr,"%d",minSizeToCoarsen);
		}
		else if (strncmp(pstr,"coarseningmethod",16)==0) {
			pstr += 16;
			pstr = pstr + strspn(pstr, ":- \t");
			if (strncmp(pstr,"weight",6)==0)	
				coarseningMethod = weighted;
			else if (strncmp(pstr,"contract",8)==0)
				coarseningMethod = contraction;
		}
		else if (strncmp(pstr,"levelsbetweenrefinements",24)==0) {
			pstr += 24;
			pstr = pstr + strspn(pstr, ":- \t");
			sscanf(pstr,"%d",&levels_between_refinement);
		}

		else if (strncmp(pstr,"numpasses",9)==0) {
			pstr += 9;
			pstr = pstr + strspn(pstr, ":- \t");
			sscanf(pstr,"%d",&numPasses);
		}
		else if (strncmp(pstr,"minconnectivity",15)==0) {
			pstr += 15;
			pstr = pstr + strspn(pstr, ":- \t");
			sscanf(pstr,"%lf",&minConnectivity);
		}
		else if (strncmp(pstr,"connectivitygrowth",18)==0) {
			pstr += 18;
			pstr = pstr + strspn(pstr, ":- \t");
			sscanf(pstr,"%lf",&connectivityGrowth);
		}	
	}
	
	*pdims2show = dimensions2show;


	if (*coordinatesFilename==0) {
		strcpy(coordinatesFilename,graphFilename);
		coordinatesFilename[strlen(coordinatesFilename)]='.'; // just for being sure...
		coordinatesFilename[strcspn(coordinatesFilename,".")]='\0';
		strcat(coordinatesFilename,(*useGdrawLayoutFormat ? "layout" :".coords"));//replace the old extension
	}

	if (showStats) {
		if( *statsFilename) {
			if (strncmp(statsFilename,"stdout",6)==0)
				fpStatsFile=stdout;
			else
				fpStatsFile = fopen(statsFilename,"w");
			fprintf(fpStatsFile, "Graph kind: %s\nSize1: %d \t Size2: %d\nGraph file: %s\n\n",
				    graph_kind, *graphSize1,*graphSize1, graphFilename);
		}
		else
			showStats = 0;
	}
	else
		fpStatsFile = NULL;

	fclose(fp);

	return 1;
}

