// -*- linux-c -*-

// Copyright 2006, Tim Hessint

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <stdio.h> // fscanf(), sscanf(),fprintf(), stdout, FILE, fgetc
#include <string.h> // sscanf
#include <assert.h> // assert()
#include <stdlib.h> // exit()
#include <ctype.h> // isspace(), isalnum()
#include <search.h> // hcreate(), hsearch(), hdestroy()

#include "parser.h"
#include "matrix.h" // fmatrix_resize(), fmatrix_create(), ...
#include "mesh2hmap.h" // mesh_t

void trimesh2mesh(mesh_t *mesh,	TriMesh *tmesh )
{
  if(tmesh == NULL){
    fprintf(stderr, "Mesh null trimesh2mesh\n");
    return;
    
  }
	mesh->num_vert=tmesh->vertices.size();
	mesh->vert = fmatrix_create(mesh->num_vert, 3);
	
	int i;
	for(i = 0; i < mesh->num_vert; i++) {
	  /*reverse x and y */
	  mesh->vert[i][0]= tmesh->vertices[i][1];
	  mesh->vert[i][1]= tmesh->vertices[i][0];
	  mesh->vert[i][2]= tmesh->vertices[i][2];
	
	}

	mesh->num_poly= tmesh->faces.size();

	// FIXME: change this to use grab_polys()
	mesh->poly = irowarray_create(mesh->num_poly,4);
    



	  for(int i = 0; i <mesh->num_poly; i++){   
	     mesh->poly[i][0] = tmesh->faces[i][0];
	     mesh->poly[i][1] = tmesh->faces[i][1];
	     mesh->poly[i][2] = tmesh->faces[i][2];
	     mesh->poly[i][3]= -1;
	  }
	 
}

void grab_vertices(FILE *fp, mesh_t *m) {
	int vert_inc = 1;
	int row = m->num_vert;
	int col = 0;
	int length = 30;
	char str[length+1]; // plus 1 for the '\0' char
	int index = 0;
	char ch;

	while( (ch = fgetc(fp)) != ']' && (ch != '}') && (ch != EOF) ) {
		// # = comment - chew to newline
		if ( ch == '#' ) while( (ch = fgetc(fp)) != '\n');
		if ( (! isspace(ch) ) && (ch != ',') && (ch != ']') ) {
			if (index < length) str[index++] = ch;
			else {
				puts("Error: number had more than 30 chars");
				exit(1);
			}
		} else if (index > 0) { // we parsed a number
			str[index] = '\0'; index = 0;
			sscanf(str, "%f", &m->vert[row][col++]);
			if (col >= 3) { // reset col, increment row
				col = 0;
				if (++row >= m->num_vert) {
					m->num_vert = row;
					m->num_vert += vert_inc;
					fmatrix_resize(&m->vert,m->num_vert,3);
				}
			}
		}
	}
	m->num_vert = row;
}


void grab_polys(FILE *fp, mesh_t *m, int vert_offset) {
        int poly_inc = 1;
	int row = m->num_poly;
	//int row = 0;
	int col = 0;
	int length = 30;
	char str[length+1]; // plus 1 for the '\0' char
	int index = 0;
	char ch;


	while( (ch = fgetc(fp)) != ']' && (ch != '}') && (ch != EOF)) {
		if ( ch == '#' ) while( (ch = fgetc(fp)) != '\n');
		if ( (! isspace(ch) ) && (ch != ',') && (ch != ']') ) {
			if (index < length) str[index++] = ch;
			else {
				puts("Error: number had more than 30 chars");
				exit(1);
			}
		} else if (index > 0) { // we parsed a number
			str[index] = '\0'; index = 0;
			m->poly[row][col++] = atoi(str) + vert_offset;
			if (atoi(str) == -1) {
				m->poly[row][col-1] = -1;
				col = 0;
				if (++row >= m->num_poly) {
					m->num_poly = row;
					irowarray_add_rows(
						&m->poly,m->num_poly,
						m->num_poly+poly_inc,1);
					m->num_poly += poly_inc;
				}
			} else {
				irowarray_resize_row(m->poly, row, col+1);
			}
		}
	}
	m->num_poly = row;
}


int getword(FILE *fp, char *str, int len)
{
	assert(fp != NULL);
	
	int i = 0;
	char ch;
	while ( (ch = fgetc(fp)) != EOF ) {
		if ( isspace(ch) ) {
			if ( i > 0 ) break; // End of word
			else continue; // Chew leading spaces
		}

		if ( i >= len - 2 ) {
			perror("Warning: `getword' - not enough space in "
				"`str' to store word");
			return 0;
		}
		str[i++] = ch;
	}
	str[i] = '\0';
	if ( i == 0 ) return 0;
	else return 1;
}

#if 0
// Parse a plain-text VRML file into internal mesh format.
// This is the part of the VRML we are interested in
void vrml2mesh(mesh_t *mesh, const char *vrmlfilename)
{
	FILE *fp = fopen(vrmlfilename, "r");
	if (fp == NULL) {
		printf("Error: Failed to open '%s'...exiting\n", 
		       vrmlfilename);
		exit(1);
	}
	int len = 100;
	char word[len];

	// Mesh must have at least 3 vertices and one poly
	mesh->vert = fmatrix_create(1,3);
	mesh->num_vert = 0; // THIS HAS TO BE ZERO
	mesh->poly = irowarray_create(1,1);
	mesh->num_poly = 0; // THIS HAS TO BE ZERO!!!

	int v_offset = 0;

	char *coord = "Coordinate";
	int is_coord = 0;

	char *point = "point";
	char *index = "coordIndex";
	
	// DEF variables
	int is_def = 0;
	char def_name[40];
	char use_name[40];
	char def_array[100][40];
	int i = 0;
	ENTRY e, *ep;
	if( hcreate(40) == 0 ) {
		perror("vrml2mesh: Insufficient memory for hash table");
		exit(1);
	}

	// Brace counter
	int brace_count = 1;

	while ( getword(fp, word, len) ) {
		if( strcmp(word, "DEF") == 0 ) {
			is_def = brace_count;
			getword(fp, def_name, 40);
		} else if ( strncmp(word, coord, strlen(coord)) == 0 ) {
			is_coord = brace_count;
		} else if ( strcmp(word, "point") == 0 ) {
			if( is_coord ) {
				// Chew opening bracket
				getword(fp,word,40);
				if( word[0] != '[' ) {
					perror("Incorrect VRML format");
					exit(1);
				}
				grab_vertices(fp, mesh);
			}
		} else if ( strcmp(word, index) == 0 ) {
			// Chew opening bracket
			getword(fp,word,40);
			if( word[0] != '[' ) {
				perror("Incorrect VRML format");
				exit(1);
			}
			grab_polys(fp, mesh, v_offset);
			v_offset = mesh->num_vert;
		} else if ( strcmp(word, "USE") == 0 ) {
			getword(fp, use_name, 40);
			e.key = use_name;
			if( (ep = hsearch(e, FIND)) != NULL ) {
				v_offset = (int) ep->data;
			}
		} else if ( strchr(word, '{') ) {
			brace_count++;
		} else if ( strchr(word, '}') ) {
			if( --brace_count < 1 ) {
				perror("Warning: brace count went below '1'");
			}
			if( brace_count == is_def ) {
				is_def = 0;
				if( brace_count == is_coord ) {
					// DEF was a Coordinate
					if( i >= 99 ) {
						perror("vrml2mesh: Too many "
							"DEFs in vrml file");
						exit(1);
					}
					strcpy(def_array[i],def_name);
					e.key = def_array[i++];
					e.data = (void *) v_offset;
					ep = hsearch(e, ENTER);
					if (ep == NULL) {
						perror("vrml2mesh: Couldn't"
							"insert entry into"
							"hash table");
						exit(1);
					}
				}
			} else if( brace_count == is_coord ) is_coord = 0;
		}
	}
	hdestroy();
}

#if 0
// Parse a plain-text VRML file into internal mesh format.
// This is the part of the VRML we are interested in
void vrml2mesh(mesh_t *mesh, const char *vrmlfilename)
{
	FILE *fp = fopen(vrmlfilename, "r");
	if (fp == NULL) {
		printf("Error: Failed to open '%s'...exiting\n", 
		       vrmlfilename);
		exit(1);
	}
	int buf_size = 100;
	char line[buf_size];

	// Mesh must have at least 3 vertices and one poly
	mesh->vert = fmatrix_create(1,3);
	mesh->num_vert = 0; // THIS HAS TO BE ZERO
	mesh->poly = irowarray_create(1,1);
	mesh->num_poly = 0; // THIS HAS TO BE ZERO!!!

	int v_offset = 0;

	char *coord = "Coordinate";
	char *point = "point";
	char *index = "coordIndex";
	char *current = coord;
	char delim = '{';
	// parser searches for 'coord', then 'point', then 'index',
	// then 'coord', then back to 'coord'.

	int i = 0; char ch;
	int space;
	while ( (ch = fgetc(fp)) != EOF ) {
		if ( ch != current[i] ) { // no match, reset
			i = 0;
			space = isspace(ch); // check for leading space
		} else if ( i == strlen(current) - 1 ) { // Match!!!
			// If 'coord', chew trailing '3'
			if ( current == coord ) {
				ch = fgetc(fp);
				if ( isspace(ch) || ch == delim ) {
					ungetc(ch, fp);
				} else if (ch != '3') continue;
			}

			i = 0;
			// Check for whitespace followed by delimiter
			while( isspace(ch = fgetc(fp)) );
			if (ch != delim) {
				puts("Error: VRML is incorrect...exiting");
				exit(1);
			}

			// increment current and delim. perform functions.
			if (current == coord) {
				current = point; delim = '[';
			} else if (current == point) {
				grab_vertices(fp,mesh);
				current = index; delim = '[';
			} else if (current == index) {
				grab_polys(fp,mesh,v_offset);
				v_offset = mesh->num_vert;
				current = coord; delim = '{';
			}
		} else if ( space ) { 
			// Matched char and has leading space.
			// Now check 'current[i++]'
			i++;
		}
			
	}

}
#endif

#if 0
void vrml2mesh(mesh_t *mesh, const char *vrmlfilename)
{
	FILE *fp = fopen(vrmlfilename, "r");
	if (fp == NULL) {
		printf("Error: Failed to open '%s'...exiting\n", 
		       vrmlfilename);
		exit(1);
	}

	char str[100];
	char *vrml1 = "#VRML V1.0";
	char *vrml2 = "#VRML V2.0";

	char **vrml1_fmt = { "Coordinate3", "point", "IndexedFaceSet",
		"coordIndex" };
	char **vrml2_fmt = { "Coordinate", "point", "coordIndex" };


	fgets(str, 100, fp);

	if (strncmp(str, vrml1, strlen(vrml1)) == 0) {
		vrml1_to_mesh(mesh, fp);
	} else if (strncmp(str, vrml1, strlen(vrml1)) == 0) {
		vrml2_to_mesh(mesh, fp);
	}

}
#endif


// Parse a mesh in the native text file format into internal mesh format.
// Format of native text file is: (where 'N' is number of vertices and
// 'P' is number of polygons).
//
//    Vertices N
//    v1x v1y v1z
//    v2x v2y v2z
//    v3x v3y v3z
//    ...
//    Polys P
//    p11 p12 p13
//    p21 p22 p23
//    ...
//
void native2mesh(mesh_t *mesh, const char *textfilename)
{
	assert(textfilename != NULL);

	FILE *fp = fopen(textfilename, "r");
	if(fp == NULL) {
		printf("Failed to open '%s', aborting\n", textfilename);
		abort();
	}
	fscanf(fp, "Vertices %d\n", &mesh->num_vert);
	mesh->vert = fmatrix_create(mesh->num_vert, 3);
	
	int i;
	for(i = 0; i < mesh->num_vert; i++) {
		fscanf(fp, " %f %f %f \n", &mesh->vert[i][0],
		       &mesh->vert[i][1], &mesh->vert[i][2]);
	}

	fscanf(fp, "Polys %d \n", &mesh->num_poly);

	// FIXME: change this to use grab_polys()
	mesh->poly = irowarray_create(mesh->num_poly,1);
        int poly_inc = 1;
	int row = 0;
	int col = 0;
	int length = 30;
	char str[length+1]; // plus 1 for the '\0' char
	int index = 0;
	char ch;


	while( (ch = fgetc(fp)) != EOF ) {
		if ( ! isspace(ch) ) {
			if (index < length) str[index++] = ch;
			else {
				puts("Error: number had more than 30 chars");
				exit(1);
			}
		} else if (index > 0) { // we parsed a number
			str[index] = '\0'; index = 0;
			mesh->poly[row][col++] = atoi(str);
			irowarray_resize_row(mesh->poly, row, col+1);
		}

		if ( ch == '\n'  && col > 0 ) {
			if ( col < 3 ) {
				puts("Error; need more than 3 vertices "
					"for a polygon...exiting");
				exit(1);
			}
			mesh->poly[row][col] = -1;
			col = 0;
			if (++row >= mesh->num_poly) {
				mesh->num_poly = row;
				irowarray_add_rows(
					&mesh->poly,mesh->num_poly,
					mesh->num_poly+poly_inc,1);
				mesh->num_poly += poly_inc;
			}
		}
	}
	mesh->num_poly = row;
	 
}

#endif

#if 0
int main(int argc, char** argv)
{

	char *vrmlfilename = argv[1];
	puts(vrmlfilename);
	FILE *fp = fopen(vrmlfilename, "r");
	if (fp == NULL) {
		printf("Error: Failed to open '%s'...exiting\n", 
		       vrmlfilename);
		exit(1);
	}
	int len = 100;
	char word[len];

	while ( getword(fp, &word[0], len) ) {
		if( strlen(word) ) printf("word is: %s\n", word);
	}
}
#endif
