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

#include <stdlib.h> // malloc, realloc, free
#include <assert.h> // assert

#include "matrix.h"


// ********************************************************************
// FLOATING POINT MATRIX
// ********************************************************************
fmatrix_t fmatrix_create(const int rows, const int cols)
{
	assert(rows >= 1);
	assert(cols >= 1);
	
        fmatrix_t m = (float**) malloc((size_t) (rows)*sizeof(float*));
	m[0] = (float*) malloc((size_t) (rows*cols)*sizeof(float));
	int i; for(i = 1; i < rows; i++) m[i] = m[i-1] + cols;

        return m;
}

void fmatrix_resize(fmatrix_t *m, const int rows, const int cols)
{
	assert(rows >= 1);
	assert(cols >= 1);

	*m = (float**) realloc(*m, (size_t) rows*sizeof(float*));
	(*m)[0] = (float*) realloc((*m)[0],(size_t) (rows*cols)*sizeof(float));
	int i; for(i = 1; i < rows; i++) (*m)[i] = (*m)[i-1] + cols;
}


void fmatrix_free(fmatrix_t m) 
{
	free(m[0]);
	free(m);
}


// ********************************************************************
// INTEGER MATRIX
// ********************************************************************
imatrix_t imatrix_create(const int rows, const int cols)
{
	assert(rows >= 1);
	assert(cols >= 1);
	
        imatrix_t m = (int**) malloc((size_t) (rows)*sizeof(int*));
	m[0] = (int*) malloc((size_t) (rows*cols)*sizeof(int));
	int i; for(i = 1; i < rows; i++) m[i] = m[i-1] + cols;

        return m;
}


imatrix_t imatrix_resize(imatrix_t m, const int rows, const int cols)
{
	assert(rows >= 1);
	assert(cols >= 1);

	m = (int**) realloc(m, (size_t) rows*sizeof(int*));
	m[0] = (int*) realloc(m[0], (size_t) (rows*cols)*sizeof(int));
	int i; for(i = 1; i < rows; i++) m[i] = m[i-1] + cols;

	return m;
}


void imatrix_free(imatrix_t m) 
{
	free(m[0]);
	free(m);
}


// ********************************************************************
// ROW ARRAY
// ********************************************************************
irowarray_t irowarray_create(const int rows, const int cols)
{
	assert(rows >= 1);
	assert(cols >= 1);
	
	irowarray_t r = (int**) malloc((size_t) rows*sizeof(int*));
	int i; for(i = 0; i < rows; i++) {
		r[i] = (int*) malloc((size_t) cols*sizeof(int));
	}
	return r;
}

void irowarray_resize_row(irowarray_t r, const int row, 
			  const int new_col)
{
	assert(row >= 0);
	assert(new_col >= 1);
	assert(r != NULL);

	r[row] = (int*) realloc(r[row], (size_t) new_col*sizeof(int));
}

void irowarray_add_rows(irowarray_t *r, const int old_rows, 
			const int new_rows, const int cols)
{
	assert(cols >= 1);
	assert(*r != NULL);
	
	*r = (irowarray_t) realloc(*r, (size_t) new_rows*sizeof(int*));
	int i; for(i = old_rows; i < new_rows; i++) {
		(*r)[i] = (int*) malloc((size_t) cols*sizeof(int));
	}
}

void irowarray_free(irowarray_t r, const int rows) 
{
	assert(rows >= 1);
	int i; for(i = 0; i < rows; i++) free(r[i]);
	free(r);
}	
