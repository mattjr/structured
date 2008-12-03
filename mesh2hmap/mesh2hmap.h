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


#ifndef _HMAP_H_
#define _HMAP_H_

#include "matrix.h"

typedef struct hmap {
	int rows, cols; // size of hmap in pixels
	float m_per_pix; // meters per pixel
	float min, max; // min and max values in heightmap
	fmatrix_t map; // floating point height map
} hmap_t;


typedef struct mesh {
	int num_vert;
	fmatrix_t vert;
	int num_poly;
	irowarray_t poly;
} mesh_t;


void mesh2hmap(hmap_t *hmap, const mesh_t *mesh, 
               const int x, const int y, const int z,
               const char invert_z, float x_m_pix, float y_m_pix);

#endif
