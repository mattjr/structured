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

#ifndef _MATRIX_H_
#define _MATRIX_H_

typedef float** fmatrix_t;
fmatrix_t fmatrix_create(const int rows, const int cols);
void fmatrix_resize(fmatrix_t *m, const int rows, const int cols);
void fmatrix_free(fmatrix_t m);

typedef int** imatrix_t;
imatrix_t imatrix_create(const int rows, const int cols);
imatrix_t imatrix_resize(imatrix_t m, const int rows, const int cols);
void imatrix_free(imatrix_t m);

typedef int** irowarray_t;
irowarray_t irowarray_create(const int rows, const int cols);
void irowarray_resize_row(irowarray_t r, const int row, 
			  const int new_col);
void irowarray_add_rows(irowarray_t *r, const int old_rows,
                        const int new_rows, const int cols);
void irowarray_free(irowarray_t r, const int rows);

#endif
