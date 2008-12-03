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

#include <assert.h> // asert()
#include <math.h> // fmax, fmin, ceilf, floorf
#include <stdlib.h> // malloc, free, exit
#include <stdio.h> // perror()

#include "mesh2hmap.h" // mesh_t, hmap_t
#include "matrix.h" // fmatrix_create(), fmatrix_free, fmatrix_resize

typedef struct sub_grid {
	int gridl, gridr, gridb, gridt;
	float *grid_row, *grid_col;
} sub_grid_t;

// 2D cross product of difference vectors.
// returns: cross(v1-v0, v2-v0)
float sub_cross2d(float *v0, float *v1, float *v2) 
{
	return ( (v1[0]-v0[0])*(v2[1]-v0[1]) - 
		 (v1[1]-v0[1])*(v2[0]-v0[0]) );
}

void get_grid_height(hmap_t *hmap, const sub_grid_t *sub_grid,
		     const fmatrix_t verts, const int num_verts) 
{
	assert(num_verts > 0);

	float p0[2], n[3];
	float last, val;
	float height;
	int same_sign = 1;
	int k;

	// Cross product of (p2-p1) and (p3-p1)...the dodgy way.
	// Give the normal to the plane.
	n[0] = (verts[1][1]-verts[0][1])*(verts[2][2]-verts[0][2]) -
		(verts[1][2]-verts[0][2])*(verts[2][1]-verts[0][1]);
	n[1] = (verts[1][2]-verts[0][2])*(verts[2][0]-verts[0][0]) -
		(verts[1][0]-verts[0][0])*(verts[2][2]-verts[0][2]);
	n[2] = (verts[1][0]-verts[0][0])*(verts[2][1]-verts[0][1]) -
		(verts[1][1]-verts[0][1])*(verts[2][0]-verts[0][0]);

	if ( n[2] == 0.0 ) { 
		// Plane is vertical. Don't worry about it. If there
		// is a grid point on this plane then it lies on the
		// edge of another polygon that connects to this
		// vertical one, so it will get picked up elsewhere.
		return;
	}

	// Loop over bounding box, determine if grid point is within
	// triangle, if so calculate it's height
	int i,j;
	float poly_max, poly_min; // max/min height of polygon
	for(i = sub_grid->gridb; i <= sub_grid->gridt ; i++) {
		p0[1] = sub_grid->grid_row[i];
		for(j = sub_grid->gridl; j <= sub_grid->gridr; j++) {
			p0[0] = sub_grid->grid_col[j];

			k = num_verts;
			last = sub_cross2d(verts[--k], verts[0], p0);
			while( (k-- > 0) && (same_sign) ) {
				val = sub_cross2d(verts[k], verts[k+1], p0);
				if (last*val >= 0.0) last = val;
				else same_sign = 0;
			}
			

			// If p0 was always left or always right of
			// each edge, as we followed a loop around
			// the polygon, then p0 is inside triangle
			if( same_sign ) {
				height = (n[0]*(verts[0][0]-p0[0]) +
					  n[1]*(verts[0][1]-p0[1]))/n[2] 
					+ verts[0][2];
				// ensure height is within range of polygon
				poly_max = fmax(verts[0][2], verts[1][2]);
				poly_max = fmax(poly_max, verts[2][2]);
				poly_min = fmin(verts[0][2], verts[1][2]);
				poly_min = fmin(poly_min, verts[2][2]);
				height = fmin(height,poly_max);
				height = fmax(height,poly_min);

				// set to hmap is greater than current value
				hmap->map[i][j] = fmax(hmap->map[i][j],height);
			} else same_sign = 1;
		}
	}
}


// Find max and min of heightmap and set background equal to min
void set_hmap_min_max(hmap_t *hmap)
{
	assert( hmap != NULL );

        float map_max, map_min;
        map_max = -HUGE_VAL;
	map_min = HUGE_VAL;
        int i;
	for(i = 0; i < hmap->rows*hmap->cols; i++) {
		if (hmap->map[0][i] == -HUGE_VAL) continue;
		map_max = fmax(map_max, hmap->map[0][i]);
		map_min = fmin(map_min, hmap->map[0][i]);
	}

	hmap->min = map_min;
	hmap->max = map_max;
}

// Convert a mesh into a height map
void mesh2hmap(hmap_t *hmap, const mesh_t *mesh, 
	       const int x, const int y, const int z,
	       const char invert_z, float x_m_pix, float y_m_pix)
{
	assert((x >= 0) && (x <= 2));
	assert((y >= 0) && (y <= 2));
	assert((z >= 0) && (z <= 2));
	assert(x_m_pix > 0.0);
	assert(y_m_pix > 0.0);

	float x_min, y_min, z_min, x_max, y_max, z_max;
	x_min = x_max = mesh->vert[0][x];
	y_min = y_max = mesh->vert[0][y];
	z_min = z_max = mesh->vert[0][z];

	int i;
	for (i = 0; i < mesh->num_vert ; i++) {
		x_min = fmin(x_min, mesh->vert[i][x]);
		y_min = fmin(y_min, mesh->vert[i][y]);
		z_min = fmin(z_min, mesh->vert[i][z]);
		x_max = fmax(x_max, mesh->vert[i][x]);
		y_max = fmax(y_max, mesh->vert[i][y]);
		z_max = fmax(z_max, mesh->vert[i][z]);
	}

	for (i = 0; i < mesh->num_vert ; i++) {
		mesh->vert[i][x] -= x_min;
		mesh->vert[i][y] -= y_min;
		if (invert_z) mesh->vert[i][z] = z_max - mesh->vert[i][z];
		else mesh->vert[i][z] -= z_min;
	}
	x_max -= x_min;
	y_max -= y_min;
	z_max -= z_min;

	if (hmap->cols > 0) x_m_pix = x_max/hmap->cols;
	else hmap->cols = ceilf(x_max/x_m_pix);
	
	if (hmap->rows > 0) y_m_pix = y_max/hmap->rows;
	else hmap->rows = ceilf(y_max/y_m_pix);

	// limit matrix to 40Mb
	if ( hmap->cols*hmap->rows > 10*1024*1024) {
		perror("Error: requested image resolution too large. "
		     "Maximum is 40Mb");
		exit(1);
	}
	hmap->map = fmatrix_create(hmap->rows,hmap->cols);


	// Make grid and zero height map
	sub_grid_t sub_grid;
	sub_grid.grid_row=(float*) malloc((size_t) (hmap->rows)*sizeof(float));
	sub_grid.grid_col=(float*) malloc((size_t) (hmap->cols)*sizeof(float));


	int j;
	for(i = 0; i < hmap->rows; i++) {
		sub_grid.grid_row[i] = (0.5+i)*y_m_pix;
		for(j = 0; j < hmap->cols; j++) {
			hmap->map[i][j] = -HUGE_VAL;
		}
	}

	for(j = 0; j < hmap->cols; j++) {
		sub_grid.grid_col[j] = (0.5+j)*x_m_pix;
	}
	

	int p, v;
	int num_verts = 3;
	fmatrix_t verts = fmatrix_create(num_verts,3);
	float polyl, polyr, polyb, polyt;
	for(p = 0; p < mesh->num_poly; p++) {
		v = 0;
		polyl = polyr = mesh->vert[mesh->poly[p][0]][x];
		polyb = polyt = mesh->vert[mesh->poly[p][0]][y];
		do {
			verts[v][0] = mesh->vert[mesh->poly[p][v]][x];
			verts[v][1] = mesh->vert[mesh->poly[p][v]][y];
			verts[v][2] = mesh->vert[mesh->poly[p][v]][z];
			polyl = fmin(polyl,verts[v][0]);
			polyr = fmax(polyr,verts[v][0]);
			polyb = fmin(polyb,verts[v][1]);
			polyt = fmax(polyt,verts[v][1]);
			if (++v >= num_verts) {
				fmatrix_resize(&verts,++num_verts,3);
			}
		} while ( mesh->poly[p][v] != -1 );

		sub_grid.gridl = ceilf((polyl - (0.5*x_m_pix))/x_m_pix);
		sub_grid.gridr = ceilf((polyr - (0.5*x_m_pix))/x_m_pix) - 1;
		sub_grid.gridb = ceilf((polyb - (0.5*y_m_pix))/y_m_pix);  
		sub_grid.gridt = ceilf((polyt - (0.5*y_m_pix))/y_m_pix) - 1;

		// Check to make sure bounding box contains a grid point
		if ( (sub_grid.gridl <= sub_grid.gridr) && 
		     (sub_grid.gridb <= sub_grid.gridt) ) {
			get_grid_height(hmap, &sub_grid, verts, v);
		}
			
	}

	set_hmap_min_max(hmap);

	fmatrix_free(verts);
	free(sub_grid.grid_row);
	free(sub_grid.grid_col);
}
