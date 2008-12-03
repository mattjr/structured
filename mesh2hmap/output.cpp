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

#include <stdio.h> // fopen(), fprintf(), printf(), putchar(), FILE
#include <stdlib.h> // exit()
#include <assert.h> // assert()
#include <math.h> // fmax(), fmin()

#include "output.h"
#include "mesh2hmap.h"



// Set background (-Inf) to 'val'
void background_to_val(hmap_t *hmap, float val) 
{
	assert( hmap != NULL );

	int i;
	for( i = 0; i < hmap->rows*hmap->cols; i++ ) {
		if (hmap->map[0][i] == -HUGE_VAL) { // background pixel
			hmap->map[0][i] = val;
		}
	}
}

// Set all values between 'min' and 'threshold' equal to 'threshold'
void threshold_min(hmap_t *hmap, float threshold)
{
	assert( hmap != NULL );
	assert( threshold >= hmap->min );

	int i;
	for( i = 0; i < hmap->rows*hmap->cols; i++ ) {
		// if not background, perform threshold
		if (hmap->map[0][i] != -HUGE_VAL) { 
			if (hmap->map[0][i] < threshold) {
				hmap->map[0][i] = threshold;
			}
		}
	}
	

}

// Set all values between 'max' and 'threshold' equal to 'threshold'
void threshold_max(hmap_t *hmap, float threshold)
{
	assert( hmap != NULL );
	assert( threshold <= hmap->max );

	int i;
	for( i = 0; i < hmap->rows*hmap->cols; i++ ) {
		// if not background, perform threshold
		if (hmap->map[0][i] != -HUGE_VAL) { 
			if (hmap->map[0][i] > threshold) {
				hmap->map[0][i] = threshold;
			}
		}
	}
	

}


// intmap structure
typedef struct intmap {
	int rows, cols;
	int min, max, range;
	imatrix_t map;
} intmap_t;


// Convert heightmap to an integer map.
// 'range' - resolution of integer map (default 255)
// 'min' - integer value corresponding to lowest value in heightmap
// 'max' - integer value corresponding to max value in heightmap
// Background is set to -1
void hmap2intmap(const hmap_t *hmap, intmap_t *intmap, const int range,
		const int min, const int max)
{
	assert( hmap != NULL );
	assert( min >= 0 );
	assert( max > min );
	assert( range >= max );

	intmap->rows = hmap->rows;
	intmap->cols = hmap->cols;
	intmap->min = min;
	intmap->max = max;
	intmap->range = range;


	float map_range = hmap->max - hmap->min;
	if (map_range == 0.0) map_range = 1;
	
	int i;
	int val;
	for(i = 0; i < hmap->rows*hmap->cols; i++) {
		if(hmap->map[0][i] == -HUGE_VAL) {
			intmap->map[0][i] = -1;
		} else {
			intmap->map[0][i] = (int) 
				( ((hmap->map[0][i] - hmap->min)/
				   map_range)*(max-min) )  + min;
		}
	}
}


// Write intmap to Portable Grey Map
// - filename: name of file to write pgm to, owverwrites if already exitst
// - imap: integer height map to print
// - bg: value of background. '-2' means min of imap, '-1' is max of imap
void intmap2pgm(const char *filename, const intmap_t *imap, int bg)
{
	assert( imap != NULL );
	assert( bg <= imap->range );

	FILE *fp;
	if (filename == NULL) fp = stdout;
	else fp = fopen(filename, "w");

	// Print the PGM header - binary type.
	fprintf(fp, "P2\n%d %d %d\n", imap->cols, imap->rows, imap->range );

	if( bg == -2 ) bg = imap->min;
	else if ( bg == -1 ) bg = imap->max;

	
	int i,j;
	for ( i = 0; i < imap->rows; i++ ) {
		for ( j = 0; j < imap->cols; j++ ) {
			if( imap->map[i][j] == -1 ) {
				fprintf(fp, "%d ", bg);
			} else {
				fprintf(fp, "%d ", imap->map[i][j]);
			}
		}
		fputc('\n', fp);
	}
	fputc('\n', fp);
	fclose(fp);
	
}

// Write intmap to Portable Pixel Map
// - max_colour: colour of highest elevation
// - min_colour: colour of lowerst elevation
// - bground: colour of background
// Format of colours is a string with three words. Each word is a decimal
// between 0 and 255 that represent red, green, and blue colour values.
void intmap2ppm(const char *filename, const intmap_t *imap,
		const char *max_colour, const char *min_colour, 
		const char *bg_colour)
{
	assert(imap != NULL);
	
	int max_red, max_green, max_blue;
	int min_red, min_green, min_blue;
	int bg_red, bg_green, bg_blue;
	sscanf(max_colour, "%d %d %d", &max_red, &max_green, &max_blue);
	sscanf(min_colour, "%d %d %d", &min_red, &min_green, &min_blue);
	sscanf(bg_colour,"%d %d %d", &bg_red, &bg_green, &bg_blue);

	int colours[9] = {max_red, max_green, max_blue,
			min_red, min_green, min_blue,
			bg_red, bg_green, bg_blue};
	int n;
	for ( n = 0; n < sizeof(colours)/sizeof(int); n++ ) {
		if ( colours[n] > imap->range ) {
			fprintf(stderr, "Error: intmap2ppm - colour %d "
					"was %d, which is greater than "
					"range (=%d)\n", n, colours[n],
					imap->range);
			exit(EXIT_FAILURE);
		}
	}


	FILE *fp;
	if (filename == NULL) fp = stdout;
	else fp = fopen(filename, "w");

	// Print the PGM header - binary type.
	fprintf(fp, "P3\n%d %d %d\n", imap->cols, imap->rows, imap->range );

	int i,j;
	int red, green, blue;
	int rgb[3];
	for ( i = 0; i < imap->rows; i++ ) {
		for ( j = 0; j < imap->cols; j++ ) {
			for ( n = 0; n < 3; n++ ) {
				rgb[n] = (int) (colours[n] - colours[3+n])*
					((float) imap->map[i][j]/imap->range)
					+ colours[3+n];
			}
			if (imap->map[i][j] == -1) {
				rgb[0] = bg_red;
				rgb[1] = bg_green;
				rgb[2] = bg_blue;
			}
			fprintf(fp, "%d %d %d  ", rgb[0], rgb[1], rgb[2]);
		}
		fputc('\n', fp);
	}
	fputc('\n', fp);
	fclose(fp);
	
}

// Write heightmap to a Portable Grey Map.
// - filename: file to print PGM to. If NULL print to stdout
// - hmap: heightmap to convert to PGM
// - range: pgm values are from '0' to 'range'
// - min: lowest height is mapped to 'min' in PGM
// - max: heighest height is mapped to 'max' in PGM
// - bg: background is mapped to 'bg' in PGM
void hmap2pgm(const char *filename, const hmap_t *hmap, const int range,
	      const int min, const int max, const int bg)
{
	assert(min >= 0);
	assert(max > min);
	assert(range >= max);

	intmap_t imap;
	imap.map = imatrix_create(hmap->rows, hmap->cols);
	hmap2intmap(hmap, &imap, range, min, max);
	intmap2pgm(filename, &imap, bg);
	imatrix_free(imap.map);

}

// Write heightmap to a Portable Pixel Map.
// If no filename provided, print to stdout
void hmap2ppm(const char *filename, const hmap_t *hmap, const int range,
	      const int min, const int max, const char *min_colour,
	      const char *max_colour, const char* background)
{
	intmap_t imap;
	imap.map = imatrix_create(hmap->rows, hmap->cols);
	hmap2intmap(hmap, &imap, range, min, max);
	intmap2ppm(filename, &imap, max_colour, min_colour, background);
	imatrix_free(imap.map);
}

void print_raw_hmap(const char *filename, const hmap_t *hmap)
{
	assert( hmap != NULL );

	FILE *fp;
	if ( filename == NULL ) fp = stdout;
	else fp = fopen(filename, "w");
	
	int i,j;
	fprintf(fp, "Rows: %d Cols: %d Min: %f Max: %f\n",
		hmap->rows, hmap->cols, hmap->min, hmap->max);
	for( i = 0; i < hmap->rows; i++) {
		for(j = 0; j < hmap->cols; j++) {
			fprintf(fp, "%f ", hmap->map[i][j]);
		}
		fputc('\n', fp);
	}
}


void print_mesh(mesh_t *mesh) 
{
	int i;
	printf("Vertices %d\n\n", mesh->num_vert);
	for(i = 0; i < mesh->num_vert; i++) {
		printf("%f %f %f\n", mesh->vert[i][0],
		       mesh->vert[i][1], mesh->vert[i][2]);
	}
	
	printf("\n\nPolys %d\n\n", mesh->num_poly);
	int j = 0;
	for(i = 0; i < mesh->num_poly; i++) {
		while( mesh->poly[i][j] != -1 ) {
			printf("%d ", mesh->poly[i][j]);
			j++;
		}
		putchar('\n');
		j = 0;
	}

}


void print_hmap(hmap_t *hmap) 
{
	int i,j;
	printf("rows: %d, cols: %d\n", hmap->rows, hmap->cols);
	for(i = 0; i < hmap->rows; i++) {
		for(j = 0; j < hmap->rows; j++) {
			printf("%f ", hmap->map[i][j]);
		}
		putchar('\n');
	}

}
// ************************************************************
