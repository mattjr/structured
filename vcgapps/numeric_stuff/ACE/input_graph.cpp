/***********************************************************************

		Utilities for reading a graph in Chaco format

***********************************************************************/

/* This file is part of Chaco by Bruce Hendrickson and Robert Leland   *
 * at Sandia National Laboratories .								   */

#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>
#include <ctype.h>
#include	"defs.h"
#include "read_params.h"

int       read_int(
					FILE     *infile,		/* file to read value from */
					int      *end_flag		/* 0 => OK, 1 => EOL, -1 => EOF */
					);

double    read_val(
					FILE     *infile,		/* file to read value from */
					int      *end_flag		/* 0 => OK, 1 => EOL, -1 => EOF */
					);


int       input_graph(FILE     *fin,			// input file 
	char     *inname,		// name of input file 
	int     **start,		// start of edge list for each vertex 
	int     **adjacency,	// edge list data 
	int      *nvtxs,		// number of vertices in graph 
	int     **vweights,		// vertex weight list data 
	float   **eweights		// edge weight list data 
)
{
    int CHECK_INPUT=1;	/* print warnings or not? */
    int      *adjptr;		/* loops through adjacency data */
    float    *ewptr;		/* loops through edge weight data */
    int       narcs;		/* number of edges expected in graph */
    int       nedges;		/* twice number of edges really in graph */
    int       nedge;		/* loops through edges for each vertex */
    int       flag;		/* condition indicator */
    int       found_flag;	/* is vertex found in adjacency list? */
    int       skip_flag;	/* should this edge be ignored? */
    int       end_flag;		/* indicates end of line or file */
    int       vtx;		/* vertex in graph */
    int       line_num;		/* line number in input file */
    int       sum_edges;	/* total number of edges read so far */
    int       option;		/* input option */
    int       using_ewgts;	/* are edge weights in input file? */
    int       using_vwgts;	/* are vertex weights in input file? */
    int       vtxnums;		/* are vertex numbers in input file? */
    int       vertex;		/* current vertex being read */
    int       new_vertex;	/* new vertex being read */
    int       weight;		/* weight being read */
    float     eweight;		/* edge weight being read */
    int       neighbor;		/* neighbor of current vertex */
    int       self_edge;	/* is a self edge encountered? */
    int       ignore_me;	/* is this edge being ignored? */
    int       ignored;		/* how many edges are ignored? */
    int       error_flag;	/* error reading input? */
    int       j;		/* loop counters */
    
    
    /* Read first line  of input (= nvtxs, narcs, option). */
    /* The (decimal) digits of the option variable mean: 1's digit not zero => input
       edge weights 10's digit not zero => input vertex weights 100's digit not zero
       => include vertex numbers */

    *start = NULL;
    *adjacency = NULL;
    *vweights = NULL;
    *eweights = NULL;

    error_flag = 0;
    line_num = 0;

    /* Read any leading comment lines */
    end_flag = 1;
    while (end_flag == 1) {
	*nvtxs = read_int(fin, &end_flag);
	++line_num;
    }
    if (*nvtxs <= 0) {
	printf("ERROR in graph file `%s':", inname);
	printf(" Invalid number of vertices (%d).\n", *nvtxs);
	if (fpStatsFile != NULL) {
	    fprintf(fpStatsFile, "ERROR in graph file `%s':", inname);
	    fprintf(fpStatsFile, " Invalid number of vertices (%d).\n", *nvtxs);
	}
	fclose(fin);
	return(1);
    }

    narcs = read_int(fin, &end_flag);
    if (narcs < 0) {
	printf("ERROR in graph file `%s':", inname);
	printf(" Invalid number of expected edges (%d).\n", narcs);
	if (fpStatsFile != NULL) {
	    fprintf(fpStatsFile, "ERROR in graph file `%s':", inname);
	    fprintf(fpStatsFile, " Invalid number of expected edges (%d).\n", narcs);
	}
	fclose(fin);
	return(1);
    }

    if (!end_flag) {
	option = read_int(fin, &end_flag);
    }
    while (!end_flag)
	j = read_int(fin, &end_flag);

    using_ewgts = option - 10 * (option / 10);
    option /= 10;
    using_vwgts = option - 10 * (option / 10);
    option /= 10;
    vtxnums = option - 10 * (option / 10);

    /* Allocate space for rows and columns. */
    *start = (int *) malloc((unsigned) (*nvtxs + 1) * sizeof(int));
    if (narcs != 0)
	*adjacency = (int *) malloc((unsigned) (2 * narcs + 1) * sizeof(int));
    else
	*adjacency = NULL;

    if (using_vwgts)
	*vweights = (int *) malloc((unsigned) (*nvtxs) * sizeof(int));
    else
	*vweights = NULL;

    if (using_ewgts && narcs != 0)
	*eweights = (float *) malloc((unsigned) (2 * narcs + 1) * sizeof(float));
    else
	*eweights = NULL;

    adjptr = *adjacency;
    ewptr = *eweights;
    self_edge = 0;
    ignored = 0;

    sum_edges = 0;
    nedges = 0;
    (*start)[0] = 0;
    vertex = 0;
    vtx = 0;
    new_vertex = TRUE;
    while ((narcs || using_vwgts || vtxnums) && end_flag != -1) {
	++line_num;

	/* If multiple input lines per vertex, read vertex number. */
	if (vtxnums) {
	    j = read_int(fin, &end_flag);
	    if (end_flag) {
		if (vertex == *nvtxs)
		    break;
		printf("ERROR in graph file `%s':", inname);
		printf(" no vertex number in line %d.\n", line_num);
		if (fpStatsFile != NULL) {
		    fprintf(fpStatsFile, "ERROR in graph file `%s':", inname);
		    fprintf(fpStatsFile, " no vertex number in line %d.\n", line_num);
		}
		fclose(fin);
		return (1);
	    }
	    if (j != vertex && j != vertex + 1) {
		printf("ERROR in graph file `%s':", inname);
		printf(" out-of-order vertex number in line %d.\n", line_num);
		if (fpStatsFile != NULL) {
		    fprintf(fpStatsFile, "ERROR in graph file `%s':", inname);
		    fprintf(fpStatsFile,
			" out-of-order vertex number in line %d.\n", line_num);
		}
		fclose(fin);
		return (1);
	    }
	    if (j != vertex) {
		new_vertex = TRUE;
		vertex = j;
	    }
	    else
		new_vertex = FALSE;
	}
	else
	    vertex = ++vtx;

	if (vertex > *nvtxs)
	    break;

	/* If vertices are weighted, read vertex weight. */
	if (using_vwgts && new_vertex) {
	    weight = read_int(fin, &end_flag);
	    if (end_flag) {
		printf("ERROR in graph file `%s':", inname);
		printf(" no weight for vertex %d.\n", vertex);
		if (fpStatsFile != NULL) {
		    fprintf(fpStatsFile, "ERROR in graph file `%s':", inname);
		    fprintf(fpStatsFile, " no weight for vertex %d.\n", vertex);
		}
		fclose(fin);
		return (1);
	    }
	    if (weight <= 0) {
		printf("ERROR in graph file `%s':", inname);
		printf(" zero or negative weight entered for vertex %d.\n", vertex);
		if (fpStatsFile != NULL) {
		    fprintf(fpStatsFile, "ERROR in graph file `%s':", inname);
		    fprintf(fpStatsFile,
			" zero or negative weight entered for vertex %d.\n", vertex);
		}
		fclose(fin);
		return (1);
	    }
	    (*vweights)[vertex-1] = weight;
	}

	nedge = 0;

	/* Read number of adjacent vertex. */
	neighbor = read_int(fin, &end_flag);

	while (!end_flag) {
	    skip_flag = FALSE;
	    ignore_me = FALSE;

	    if (neighbor > *nvtxs) {
		printf("ERROR in graph file `%s':", inname);
		printf(" nvtxs=%d, but edge (%d,%d) was input.\n",
		       *nvtxs, vertex, neighbor);
		if (fpStatsFile != NULL) {
		    fprintf(fpStatsFile, "ERROR in graph file `%s':", inname);
		    fprintf(fpStatsFile,
			" nvtxs=%d, but edge (%d,%d) was input.\n",
			*nvtxs, vertex, neighbor);
		}
		fclose(fin);
		return (1);
	    }
	    if (neighbor <= 0) {
		printf("ERROR in graph file `%s':", inname);
		printf(" zero or negative vertex in edge (%d,%d).\n",
		       vertex, neighbor);
		if (fpStatsFile != NULL) {
		    fprintf(fpStatsFile, "ERROR in graph file `%s':", inname);
		    fprintf(fpStatsFile,
		        " zero or negative vertex in edge (%d,%d).\n",
			vertex, neighbor);
		}
		fclose(fin);
		return (1);
	    }

	    if (neighbor == vertex) {
		if (!self_edge && CHECK_INPUT) {
		    printf("WARNING: Self edge (%d, %d) being ignored.\n",
			   vertex, vertex);
    		    if (fpStatsFile != NULL) {
		        fprintf(fpStatsFile,
			    "WARNING: Self edge (%d, %d) being ignored.\n", vertex, vertex);
		    }
		}
		skip_flag = TRUE;
		++self_edge;
	    }

	    /* Check if adjacency is repeated. */
	    if (!skip_flag) {
		found_flag = FALSE;
		for (j = (*start)[vertex - 1]; !found_flag && j < sum_edges + nedge; j++) {
		    if ((*adjacency)[j] == neighbor)
			found_flag = TRUE;
		}
		if (found_flag) {
		    printf("WARNING: Multiple occurences of edge (%d,%d) ignored\n",
			vertex, neighbor);
		    if (fpStatsFile != NULL) {
			fprintf(fpStatsFile,
			    "WARNING: Multiple occurences of edge (%d,%d) ignored\n",
			    vertex, neighbor);
		    }
		    skip_flag = TRUE;
		    if (!ignore_me) {
			ignore_me = TRUE;
			++ignored;
		    }
		}
	    }

	    if (using_ewgts) {	/* Read edge weight if it's being input. */
		eweight = read_val(fin, &end_flag);

		if (end_flag) {
		    printf("ERROR in graph file `%s':", inname);
		    printf(" no weight for edge (%d,%d).\n", vertex, neighbor);
		    if (fpStatsFile != NULL) {
		        fprintf(fpStatsFile, "ERROR in graph file `%s':", inname);
		        fprintf(fpStatsFile,
			    " no weight for edge (%d,%d).\n", vertex, neighbor);
		    }
		    fclose(fin);
		    return (1);
		}

		if (eweight <= 0 && CHECK_INPUT) {
		    printf("WARNING: Bad weight entered for edge (%d,%d).  Edge ignored.\n",
			   vertex, neighbor);
    		    if (fpStatsFile != NULL) {
		        fprintf(fpStatsFile,
			    "WARNING: Bad weight entered for edge (%d,%d).  Edge ignored.\n",
			    vertex, neighbor);
		    }
		    skip_flag = TRUE;
		    if (!ignore_me) {
			ignore_me = TRUE;
			++ignored;
		    }
		}
		else {
		    *ewptr++ = eweight;
		}
	    }

	    /* Check for edge only entered once. */
	    if (neighbor < vertex && !skip_flag) {
		found_flag = FALSE;
		for (j = (*start)[neighbor - 1]; !found_flag && j < (*start)[neighbor]; j++) {
		    if ((*adjacency)[j] == vertex)
			found_flag = TRUE;
		}
		if (!found_flag) {
		    printf("ERROR in graph file `%s':", inname);
		    printf(" Edge (%d, %d) entered but not (%d, %d)\n",
			   vertex, neighbor, neighbor, vertex);
		    if (fpStatsFile != NULL) {
		        fprintf(fpStatsFile, "ERROR in graph file `%s':", inname);
		        fprintf(fpStatsFile,
			    " Edge (%d, %d) entered but not (%d, %d)\n",
			    vertex, neighbor, neighbor, vertex);
		    }
		    error_flag = 1;
		}
	    }

	    /* Add edge to data structure. */
	    if (!skip_flag) {
		if (++nedges > 2*narcs) {
		    printf("ERROR in graph file `%s':", inname);
		    printf(" at least %d adjacencies entered, but nedges = %d\n",
			nedges, narcs);
		    if (fpStatsFile != NULL) {
		        fprintf(fpStatsFile, "ERROR in graph file `%s':", inname);
		        fprintf(fpStatsFile,
			    " at least %d adjacencies entered, but nedges = %d\n",
			    nedges, narcs);
		    }
		    fclose(fin);
		    return (1);
		}
		*adjptr++ = neighbor;
		nedge++;
	    }

	    /* Read number of next adjacent vertex. */
	    neighbor = read_int(fin, &end_flag);
	}

	sum_edges += nedge;
	(*start)[vertex] = sum_edges;
    }

    /* Make sure there's nothing else in file. */
    flag = FALSE;
    while (!flag && end_flag != -1) {
	read_int(fin, &end_flag);
	if (!end_flag)
	    flag = TRUE;
    }
    if (flag && CHECK_INPUT) {
	printf("WARNING: Possible error in graph file `%s'\n", inname);
	printf("         Data found after expected end of file\n");
        if (fpStatsFile != NULL) {
	    fprintf(fpStatsFile,
		"WARNING: Possible error in graph file `%s'\n", inname);
	    fprintf(fpStatsFile,
		"         Data found after expected end of file\n");
	}
    }

    (*start)[*nvtxs] = sum_edges;

    if (self_edge > 1 && CHECK_INPUT) {
	printf("WARNING: %d self edges were read and ignored.\n", self_edge);
        if (fpStatsFile != NULL) {
	    fprintf(fpStatsFile,
		"WARNING: %d self edges were read and ignored.\n", self_edge);
	}
    }

    if (vertex != 0) {		/* Normal file was read. */
	/* Make sure narcs was reasonable. */
		if (nedges + 2 * self_edge != 2 * narcs &&
			nedges + 2 * self_edge + ignored != 2 * narcs &&
			nedges + self_edge != 2 * narcs && 
			nedges + self_edge + ignored != 2 * narcs && 
			nedges != 2 * narcs &&
			nedges + ignored != 2 * narcs) {
			printf("WARNING: I expected %d edges entered twice, but I only count %d.\n", narcs, nedges);
    		if (fpStatsFile != NULL) 
				fprintf(fpStatsFile,
				"WARNING: I expected %d edges entered twice, but I only count %d.\n",
					narcs, nedges);
		}
    }

    else {
	/* Graph was empty => must be using inertial method. */
	free( *start);
	if (*adjacency != NULL)
	    free( *adjacency);
	if (*eweights != NULL)
	    free( *eweights);
	*start = NULL;
	*adjacency = NULL;
    }

    fclose(fin);

    return (error_flag);
}



#define NAME_LENGTH	80	/* Maximum length of file name */
#define LINE_LENGTH	200	/* Length of input files read at once */


static char line[LINE_LENGTH];	/* space to hold values */
static int offset = 0;		/* offset into line for next data */
static int break_pnt = LINE_LENGTH;	/* place in sequence to pause */
static int save_pnt;		/* place in sequence to save */



static void flush_line(FILE     *infile /* file to read value from */)		
{
    char      c;		/* character being read */

    c = getc(infile);
    while (c != '\n' && c != '\f')
		c = getc(infile);
}

double    read_val(
					FILE     *infile,		/* file to read value from */
					int      *end_flag		/* 0 => OK, 1 => EOL, -1 => EOF */
					)
{
    double    val;		/* return value */
    char     *ptr;		/* ptr to next string to read */
    char     *ptr2;		/* ptr to next string to read */
    int       length;		/* length of line to read */
    int       length_left;	/* length of line still around */
    int       white_seen;	/* have I detected white space yet? */
    int       done;		/* checking for end of scan */
    int       i;		/* loop counter */
    
    *end_flag = 0;

    if (offset == 0 || offset >= break_pnt) {
	if (offset >= break_pnt) { /* Copy rest of line back to beginning. */ 
	    length_left = LINE_LENGTH - save_pnt - 1;
	    ptr2 = line;
	    ptr = &line[save_pnt];
	    for (i=length_left; i; i--) *ptr2++ = *ptr++;
	    length = save_pnt + 1;
	}
	else {
	    length = LINE_LENGTH;
	    length_left = 0;
	}

	line[LINE_LENGTH - 1] = ' ';
	line[LINE_LENGTH - 2] = ' ';
	/* Now read next line, or next segment of current one. */
	ptr2 = fgets(&line[length_left], length, infile);

	if (ptr2 == (char *) NULL) {	/* We've hit end of file. */
	    *end_flag = -1;
	    return((double) 0.0);
	}
	
	if (line[LINE_LENGTH - 1] == '\0' && line[LINE_LENGTH - 2] != '\0' &&
	    line[LINE_LENGTH - 2] != '\n' && line[LINE_LENGTH - 2] != '\f') {
	    /* Line too long.  Find last safe place in line. */
	    break_pnt = LINE_LENGTH - 1;
	    save_pnt = break_pnt;
	    white_seen = FALSE;
	    done = FALSE;
	    while (!done) {
		--break_pnt;
		if (line[break_pnt] != '\0') {
		    if (isspace(line[break_pnt])) {
			if (!white_seen) {
			    save_pnt = break_pnt + 1;
		            white_seen = TRUE;
			}
		    }
		    else if (white_seen) {
		        done= TRUE;
		    }
		}
	    }
	}
	else {
	    break_pnt = LINE_LENGTH;
	}

	offset = 0;
    }

    while (isspace(line[offset]) && offset < LINE_LENGTH) offset++;
    if (line[offset] == '%' || line[offset] == '#') {
	*end_flag = 1;
	if (break_pnt < LINE_LENGTH) {
	    flush_line(infile);
	}
	return((double) 0.0);
    }

    ptr = &(line[offset]);
    val = strtod(ptr, &ptr2);

    if (ptr2 == ptr) {	/* End of input line. */
	offset = 0;
	*end_flag = 1;
	return((double) 0.0);
    }
    else {
	offset = (int) (ptr2 - line) / sizeof(char);
    }

    return(val);
}


int       read_int(
					FILE     *infile,		/* file to read value from */
					int      *end_flag		/* 0 => OK, 1 => EOL, -1 => EOF */
					)
{
    int       val;		/* return value */
    char     *ptr;		/* ptr to next string to read */
    char     *ptr2;		/* ptr to next string to read */
    int       length;		/* length of line to read */
    int       length_left;	/* length of line still around */
    int       white_seen;	/* have I detected white space yet? */
    int       done;		/* checking for end of scan */
    int       i;		/* loop counter */
   
	*end_flag = 0;

    if (offset == 0 || offset >= break_pnt) {
	if (offset >= break_pnt) { /* Copy rest of line back to beginning. */ 
	    length_left = LINE_LENGTH - save_pnt - 1;
	    ptr2 = line;
	    ptr = &line[save_pnt];
	    for (i=length_left; i; i--) *ptr2++ = *ptr++;
	    length = save_pnt + 1;
	}
	else {
	    length = LINE_LENGTH;
	    length_left = 0;
	}

	line[LINE_LENGTH - 1] = ' ';
	line[LINE_LENGTH - 2] = ' ';
	/* Now read next line, or next segment of current one. */
	ptr2 = fgets(&line[length_left], length, infile);

	if (ptr2 == (char *) NULL) {	/* We've hit end of file. */
	    *end_flag = -1;
	    return(0);
	}
	
	if (line[LINE_LENGTH - 1] == '\0' && line[LINE_LENGTH - 2] != '\0' &&
	    line[LINE_LENGTH - 2] != '\n' && line[LINE_LENGTH - 2] != '\f') {
	    /* Line too long.  Find last safe place in line. */
	    break_pnt = LINE_LENGTH - 1;
	    save_pnt = break_pnt;
	    white_seen = FALSE;
	    done = FALSE;
	    while (!done) {
		--break_pnt;
		if (line[break_pnt] != '\0') {
		    if (isspace(line[break_pnt])) {
			if (!white_seen) {
			    save_pnt = break_pnt + 1;
		            white_seen = TRUE;
			}
		    }
		    else if (white_seen) {
		        done= TRUE;
		    }
		}
	    }
	}
	else {
	    break_pnt = LINE_LENGTH;
	}

	offset = 0;
    }

    while (isspace(line[offset]) && offset < LINE_LENGTH) offset++;
    if (line[offset] == '%' || line[offset] == '#') {
	*end_flag = 1;
	if (break_pnt < LINE_LENGTH) {
	    flush_line(infile);
	}
	return(0);
    }

    ptr = &(line[offset]);
    val = (int) strtol(ptr, &ptr2, 10);

    if (ptr2 == ptr) {	/* End of input line. */
	offset = 0;
	*end_flag = 1;
	return(0);
    }
    else {
	offset = (int) (ptr2 - line) / sizeof(char);
    }

    return(val);
}
