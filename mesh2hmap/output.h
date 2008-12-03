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


#ifndef _OUTPUT_H_
#define _OUTPUT_H_

#include "mesh2hmap.h"

void hmap2pgm(const char *filename, const hmap_t *hmap, const int range,
	      const int min, const int max, const int bg);
void hmap2ppm(const char *filename, const hmap_t *hmap, const int range,
		const int min, const int max, const char *min_colour,
		const char *max_colour, const char * background);
void print_raw_hmap(const char *filename, const hmap_t *hmap);
void print_mesh(mesh_t *mesh);
void print_hmap(hmap_t *hmap);

#endif
