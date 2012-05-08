
/*
 * taucs_ccs_ooc_ldlt.c 
 *
 * Out-of-core sparse Indefinite factorization
 *
 * authors: Omer Meshar & Sivan Toledo 
 *
 * Copyright, 2003.
 */

/*************************************************************/
/*                                                           */
/*************************************************************/

#include <stdio.h>
#include <stdlib.h>


/*#include <unistd.h>*/
/*#include <sys/uio.h>*/

#include <assert.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <numeric_stuff/TAUCS/taucs.h>

#ifdef TAUCS_CORE_DOUBLE
#define taucs_cmp(x,y) ((x)==(y))
#define taucs_real_zero_const taucs_dzero_const
#define taucs_real_abs(x) taucs_abs(x)
#endif
#ifdef TAUCS_CORE_SINGLE
#define taucs_cmp(x,y) ((x)==(y))
#define taucs_real_zero_const taucs_szero_const
#define taucs_real_abs(x) taucs_abs(x)
#endif
#ifdef TAUCS_CORE_DCOMPLEX
#define taucs_cmp(x,y) (((taucs_im(x))==(taucs_im(y))) && ((taucs_re(x))==(taucs_re(y))))
#define taucs_real_zero_const taucs_dzero_const
#define taucs_real_abs(x) taucs_abs(x)
#endif
#ifdef TAUCS_CORE_SCOMPLEX
#define taucs_cmp(x,y) (((taucs_im(x))==(taucs_im(y))) && ((taucs_re(x))==(taucs_re(y))))
#define taucs_real_zero_const taucs_szero_const
#define taucs_real_abs(x) taucs_abs(x)
#endif


#define FALSE 0
#define TRUE  1

/* #define BLAS_FLOPS_CUTOFF  1000.0 */

#define BLAS_FLOPS_CUTOFF  -1.0
#define SOLVE_DENSE_CUTOFF 5

/* number of matrices in the header of the file */
#define IO_BASE    9 /*7*/
/* multiple files of at most 1 GB or single file */
#define MULTIFILE 0

/* how much to reduce the size of the panel */
#define PANEL_RESIZE 0.75

#ifndef TAUCS_CORE_GENERAL
/* globals */
static float gl_alpha = (float)0.001;
static int gl_cramer_rule = 0;
static float gl_beta = (float)0.0000000000000000000000001;
typedef enum { fa_right_looking, fa_blocked, fa_lapack } factor_alg_enum;
static int gl_factor_alg = fa_blocked;
static int gl_block_size = 20;

#ifdef TAUCS_HACK_LDLT_TESTING
extern char * gl_parameters;

static void
get_input_parameters( char* filename )
{
	int factor_alg = 0;
  FILE* f = fopen(filename,"r");
	if (!f) {/* no file found - leave defaults */
		taucs_printf("\t\tUsing default parameters\n");
    return;
	}
  fscanf(f,"%e,%d,%d,%e,%d",&gl_alpha,&gl_cramer_rule,
						&factor_alg,&gl_beta,&gl_block_size);
	fclose(f);
	taucs_printf("\t\tUsing parameters: %e %d %d %e %d\n",gl_alpha,gl_cramer_rule,
						factor_alg,gl_beta,gl_block_size);

	switch(factor_alg) {
		case 0:
			gl_factor_alg = fa_right_looking;
			break;
		case 1:
			gl_factor_alg = fa_blocked;
			break;
		case 2:
			gl_factor_alg = fa_lapack;
	}
}
#endif

static int
taucs_is_zero( taucs_datatype v )
{
	if ( taucs_real_abs(v) < gl_beta )
	{
	  /*printf("\t\t\tzero - %e\n",v);*/
	  return TRUE;
	}
	return FALSE;
}
#endif

/*************************************************************/
/* structures                                                */
/*************************************************************/

typedef struct {
  char    uplo;     /* 'u' for upper, 'l' for lower, ' ' don't know; prefer lower. */
  int     n;        /* size of matrix */
  int     n_sn;     /* number of supernodes */

  int* parent;      /* supernodal elimination tree */
  int* first_child; 
  int* next_child;
  int* ipostorder;
  int* col_to_sn_map;     

  int* sn_size;     /* size of supernodes (diagonal block) */
  int* orig_sn_size;/* the original size of supernodes, before changes
											 made by rejected columns */
  int* sn_up_size;  /* size of subdiagonal update blocks   */
  int** sn_struct;  /* row structure of supernodes         */
  int* sn_max_ind;  /* the max index of the supernode			 */
  int* sn_rej_size;	/* the rejected size of each supernode */

  int** db_size;		/* sizes of diagonal blocks for each column in each supernode.
				   there are sn_size sizes for each supernode */

  taucs_datatype** sn_blocks; /* supernode blocks        */
  taucs_datatype** up_blocks; /* update blocks           */
  taucs_datatype** d_blocks;	/* diagonal blocks				 */
} supernodal_factor_matrix_ooc_ldlt;

typedef struct {
  int n_pn;		/* number of the panel */
  int n_sn;		/* number of supernodes */
  int n_postorder;	/* current number in postorder */
  int used;	        /* was there a panel that was used */
  
  int max_size;		/* max size of supernode in panel */
	
  int* sn_to_panel_map; /* maps each supernode to its panel */
  int* sn_in_core;	/* sn in core */
  int* postorder;	/* post order of supernodes */
	
  double avail_mem;     /* available memory in panel */
  double max_mem;     	/* max memory allowed in panel */
} supernodal_panel;

typedef struct {
  int rejected_size;		/* size of the supernode that was rejected */
  int* rowind;	       		/* To which column was this column moved to */
  int* indices;	       		/* Which column now resides in this column */
} supernodal_ldlt_factor;

#ifndef TAUCS_CORE_GENERAL
/*
static void writeFactorFacts( supernodal_factor_matrix_ooc_ldlt* L, char * str )
{
	FILE * f;
	char s[40];
	int i,j;

	sprintf( s, "C:/taucs/results/L_%s.ijv",str);
	f = fopen(s,"w");

	if (!f || !L)
		return;

	fprintf(f,"sn:\t\t\t\t\t\t");
	for (i=0; i<L->n_sn; i++)
		if (L->sn_up_size[i])
			fprintf(f,"%3d ",i);
	fprintf(f,"\nsn_size:\t\t\t");
	for (i=0; i<L->n_sn; i++)
		if (L->sn_up_size[i])
			fprintf(f,"%3d ",L->sn_size[i]);
	fprintf(f,"\nup_size:\t\t\t");
	for (i=0; i<L->n_sn; i++)
		if (L->sn_up_size[i])
			fprintf(f,"%3d ",L->sn_up_size[i]);
	fprintf(f,"\n");
	for (i=0; i<L->n_sn; i++) {
		if (L->sn_up_size[i] && L->sn_struct[i] ) {
			fprintf(f,"struct for %d: \t",i);
			for (j=0; j<L->sn_size[i]; j++)
				fprintf(f,"%d ",L->sn_struct[i][j]);
			fprintf(f,"|");
			for (j=L->sn_size[i]; j<L->sn_up_size[i]; j++)
				fprintf(f,"%d ",L->sn_struct[i][j]);
			fprintf(f,"\n");
			if ( L->db_size[i] ) {
				fprintf(f,"db_size for %d:\t",i);
				for (j=0; j<L->sn_size[i]; j++)
					fprintf(f,"%d ",L->db_size[i][j]);
				fprintf(f,"\n");
			}
		}
	}
	fclose(f);
}*/

/*static void
writeMatToFile(taucs_datatype* mat,int nrows,int ncols,FILE* f)
{
	int i,j;
	taucs_datatype v;
	for (i=0;i<ncols;i++)
		for (j=0;j<nrows;j++) {
			v = mat[i*nrows + j];
#ifdef TAUCS_CORE_DOUBLE
			fprintf(f,"%d %d %1.6e\n",j+1,i+1,v);
#endif
		}
}


static
void writeSNLToFile(supernodal_factor_matrix_ooc_ldlt* snL,int sn, char * str)
{
	FILE * fF1, *fF2, *fD, *fst;
	char sF1[55], sF2[55], sD[55], sst[55];
	int i;
	taucs_datatype v;

	return;

	sprintf( sF1, "../../results/%d_F1_%s.ijv",sn,str);
	sprintf( sF2, "../../results/%d_F2_%s.ijv",sn,str);
	sprintf( sD, "../../results/%d_D_%s.ijv",sn,str);
	sprintf( sst, "../../results/%d_st_%s.ijv",sn,str);
	fF1 = fopen(sF1,"w");
	fF2 = fopen(sF2,"w");
	fD = fopen(sD,"w");
	fst = fopen(sst,"w");

	if ( !fF1 || !fF2 || !fD ||!fst)
		return;

	writeMatToFile(snL->sn_blocks[sn],snL->sn_size[sn],snL->sn_size[sn],fF1);
	writeMatToFile(snL->up_blocks[sn],snL->sn_up_size[sn] - snL->sn_size[sn],snL->sn_size[sn],fF2);
	for (i=0;i<snL->sn_size[sn];i++) {
		v = snL->d_blocks[sn][i*2];
#ifdef TAUCS_CORE_DOUBLE
		fprintf(fD,"%d %d %1.6e\n",i+1,i+1,v);
		if ( i+1 < snL->sn_size[sn] ) {
			v = snL->d_blocks[sn][i*2 + 1];
			fprintf(fD,"%d %d %1.6e\n",i+2,i+1,v);
			fprintf(fD,"%d %d %1.6e\n",i+1,i+2,v);
		}
#endif
	}
	for (i=0;i<snL->sn_size[sn];i++) {
		fprintf(fst,"%d ",snL->sn_struct[sn][i]);
	}
	fprintf(fst,"|");
	for (i=snL->sn_size[sn];i<snL->sn_up_size[sn];i++) {
		fprintf(fst,"%d ",snL->sn_struct[sn][i]);
	}
	fclose(fF1);
	fclose(fF2);
	fclose(fD);
	fclose(fst);

}*/

#endif
/*
taucs_ccs_matrix*
taucs_dsupernodal_factor_ldlt_ooc_to_ccs(void* vL)
{
  supernodal_factor_matrix_ooc_ldlt* L = (supernodal_factor_matrix_ooc_ldlt*) vL;
  taucs_ccs_matrix* C;
  int n,nnz;
  int i,j,ip,jp,sn,next,db_size;
  taucs_datatype v;
  int* len;

  n = L->n;

  len = (int*) taucs_malloc(n*sizeof(int));
  if (!len) return NULL;

  nnz = n;	

	for	(sn=0; sn<L->n_sn; sn++) {
		for	(jp=0; jp<(L->sn_size)[sn];	jp++)	{
			j	=	(L->sn_struct)[sn][jp];
			len[j] = 1; 

			db_size = (L->db_size)[sn][jp];

			for	(ip=jp+db_size;	ip<(L->sn_size)[sn]; ip++) {
				i	=	(L->sn_struct)[sn][	ip ];
				v	=	(L->sn_blocks)[sn][	jp*((L->sn_size)[sn]) + ip	];

				if (taucs_re(v)	|| taucs_im(v))	{
					len[j] ++;
					nnz	++;
				}
      }
      for (ip=(L->sn_size)[sn]; ip<(L->sn_up_size)[sn]; ip++) {
				i = (L->sn_struct)[sn][ ip ];
				v = (L->up_blocks)[sn][ jp*((L->sn_up_size)[sn] - L->sn_size[sn]) + 
																(ip-(L->sn_size)[sn]) ];

				if (taucs_re(v) || taucs_im(v)) {
					len[j] ++;
					nnz ++;
				}
			}
    }
  }


  C = taucs_dtl(ccs_create)(n,n,nnz);
  if (!C) {
    taucs_free(len);
    return NULL;
  }

#ifdef TAUCS_CORE_SINGLE
  C->flags = TAUCS_SINGLE;
#endif

#ifdef TAUCS_CORE_DOUBLE
  C->flags = TAUCS_DOUBLE;
#endif

#ifdef TAUCS_CORE_SCOMPLEX
  C->flags = TAUCS_SCOMPLEX;
#endif

#ifdef TAUCS_CORE_DCOMPLEX
  C->flags = TAUCS_DCOMPLEX;
#endif

  C->flags |= TAUCS_TRIANGULAR | TAUCS_LOWER;

  (C->colptr)[0] = 0;
	for (j=1; j<=n; j++) (C->colptr)[j] = (C->colptr)[j-1] + len[j-1];

  taucs_free(len);

  for (sn=0; sn<L->n_sn; sn++) {
    for (jp=0; jp<(L->sn_size)[sn]; jp++) {
      j = (L->sn_struct)[sn][jp];

      next = (C->colptr)[j];

			i = (L->sn_struct)[sn][ jp ];
			v = taucs_one_const;

			(C->rowind)[next] = i;
			(C->taucs_values)[next] = v;
			next++;

			db_size = (L->db_size)[sn][jp];

			for (ip=jp+db_size; ip<(L->sn_size)[sn]; ip++) {
				i = (L->sn_struct)[sn][ ip ];
				v = (L->sn_blocks)[sn][ jp*(L->sn_size)[sn] + ip ];

				if (!taucs_re(v) && !taucs_im(v)) continue;
				
				(C->rowind)[next] = i;
				(C->taucs_values)[next] = v;
				next++;
			}
      for (ip=(L->sn_size)[sn]; ip<(L->sn_up_size)[sn]; ip++) {
				i = (L->sn_struct)[sn][ ip ];
				v = (L->up_blocks)[sn][ jp*((L->sn_up_size)[sn]-L->sn_size[sn]) + (ip-(L->sn_size)[sn]) 
];

				if (!taucs_re(v) && !taucs_im(v)) continue;

				(C->rowind)[next] = i;
				(C->taucs_values)[next] = v;
				next++;
      }
    }
	}

  return C;
}


taucs_ccs_matrix*
taucs_dsupernodal_factor_diagonal_ooc_to_ccs(void* vL)
{
  supernodal_factor_matrix_ooc_ldlt* L = (supernodal_factor_matrix_ooc_ldlt*) vL;
  taucs_ccs_matrix* C;
  int n,nnz;
  int i,j,ip,jp,sn,next,db_size;
  taucs_datatype v;
	int* len;

  n = L->n;

	len = (int*) taucs_malloc(n*sizeof(int));
  if (!len) return NULL;

  nnz = 0;

	for	(sn=0; sn<L->n_sn; sn++) {
		for	(jp=0; jp<(L->sn_size)[sn];	jp++)	{
			j	=	(L->sn_struct)[sn][jp];
			len[j] = 0;
			db_size = L->db_size[sn][jp];

			for	(ip=0;	ip<db_size; ip++) {
				v	=	(L->d_blocks)[sn][	jp*2 + ip	];

				if (taucs_re(v)	|| taucs_im(v))	{
					nnz	++;
					len[j] ++;
				}
      }
    }
  }

  C = taucs_dtl(ccs_create)(n,n,nnz);
  if (!C) {
    return NULL;
  }

#ifdef TAUCS_CORE_SINGLE
  C->flags = TAUCS_SINGLE |	TAUCS_SYMMETRIC;
#endif

#ifdef TAUCS_CORE_DOUBLE
  C->flags = TAUCS_DOUBLE |	TAUCS_SYMMETRIC;
#endif

#ifdef TAUCS_CORE_SCOMPLEX
  C->flags = TAUCS_SCOMPLEX |	TAUCS_HERMITIAN;
#endif

#ifdef TAUCS_CORE_DCOMPLEX
  C->flags = TAUCS_DCOMPLEX |	TAUCS_HERMITIAN;
#endif

  C->flags |= TAUCS_LOWER;

  (C->colptr)[0] = 0;
  for (j=1; j<=n; j++) (C->colptr)[j] = (C->colptr)[j-1] + len[j-1];

  taucs_free(len);

  for (sn=0; sn<L->n_sn; sn++) {
    for (jp=0; jp<(L->sn_size)[sn]; jp++) {
      j = (L->sn_struct)[sn][jp];

      next = (C->colptr)[j];
			db_size = L->db_size[sn][jp];

      for (ip=0; ip<db_size; ip++) {
				i	=	(L->sn_struct)[sn][	ip+jp ];
				v = (L->d_blocks)[sn][ jp*2 + ip ];

				if (!taucs_re(v) && !taucs_im(v)) continue;

				(C->rowind)[next] = i;
				(C->taucs_values)[next] = v;
				next++;
			}
    }
  }

  return C;
}*/

/*************************************************************/
/* for qsort                                                 */
/*************************************************************/

#if 0

static int compare_ints(void* vx, void* vy)
{
  int* ix = (int*)vx;
  int* iy = (int*)vy;
  if (*ix < *iy) return -1;
  if (*ix > *iy) return  1;
  return 0;
}

#endif /* 0 */

#ifndef TAUCS_CORE_GENERAL

static int* compare_indirect_map;
static int compare_indirect_ints(const void* vx, const void* vy)/*(void* vx, void* vy) omer*/
{
  int* ix = (int*)vx;
  int* iy = (int*)vy;
  if (compare_indirect_map[*ix] < compare_indirect_map[*iy]) return -1;
  if (compare_indirect_map[*ix] > compare_indirect_map[*iy]) return  1;
  return 0;
}



/*************************************************************/
/* create and free the panel object                          */
/*************************************************************/
static supernodal_panel* 
taucs_panel_create( double memory, int n_sn )
{
  int i;
  supernodal_panel* P;
  
  P = (supernodal_panel*) taucs_malloc(sizeof(supernodal_panel));
  if (!P) return NULL;
  
  P->max_mem = memory;
  P->avail_mem = memory;
  P->max_size = 0;
  P->n_sn = 0;
  P->n_pn = 0;
  P->n_postorder = 0;
  P->used = 0;
  P->sn_to_panel_map = taucs_malloc(sizeof(int)*(n_sn+1));
  if (!P->sn_to_panel_map) {
    taucs_free(P);
    return NULL;
  }
  P->sn_in_core = taucs_malloc(sizeof(int)*(n_sn+1));
  if (!P->sn_in_core) {
    taucs_free(P);
    return NULL;
  }
  P->postorder = taucs_malloc(sizeof(int)*(n_sn+1));
  if (!P->postorder) {
    taucs_free(P);
    return NULL;
  }
  for (i=0; i<n_sn+1; i++) {
    P->sn_in_core[i] = 0;
    P->sn_to_panel_map[i] = -1;
    P->postorder[i] = -1;
  }
  
  return P;
}

static void
taucs_panel_free( supernodal_panel* P )
{
  taucs_free( P->sn_to_panel_map );
  taucs_free( P->sn_in_core );
  taucs_free( P->postorder );
  taucs_free( P );
}


/*************************************************************/
/* create and free the factor object                         */
/*************************************************************/

static supernodal_factor_matrix_ooc_ldlt*
multifrontal_supernodal_create()
{
  supernodal_factor_matrix_ooc_ldlt* L;
  
  L = (supernodal_factor_matrix_ooc_ldlt*) taucs_malloc(sizeof(supernodal_factor_matrix_ooc_ldlt));
  if (!L) return NULL;
  L->uplo      = 'l';
  L->n         = -1; /* unused */

  L->sn_struct   = NULL;
  L->sn_size     = NULL;
  L->sn_up_size  = NULL;
  L->parent      = NULL;
  L->col_to_sn_map = NULL;
  L->first_child = NULL;
  L->next_child  = NULL;
  L->ipostorder  = NULL;
  L->sn_blocks     = NULL;
  L->up_blocks     = NULL;
  L->db_size			 = NULL;
  L->d_blocks			 = NULL;
  L->orig_sn_size	 = NULL;
  L->sn_max_ind		 = NULL;
  L->sn_rej_size	 = NULL;
  
  return L;
}

static void
ooc_supernodal_free(supernodal_factor_matrix_ooc_ldlt* L,
										int sn)
{
  taucs_free(L->sn_struct[sn]);
  taucs_free(L->sn_blocks[sn]);
  taucs_free(L->up_blocks[sn]);
  if (L->d_blocks[sn]) taucs_free(L->d_blocks[sn]);
  if (L->db_size[sn]) taucs_free(L->db_size[sn]);
  L->sn_struct[sn] = NULL;
  L->sn_blocks[sn] = NULL;
  L->up_blocks[sn] = NULL;
  L->d_blocks[sn] = NULL;
  L->db_size[sn] = NULL;
}

static void 
ooc_supernodal_factor_free(void* vL)
{
  supernodal_factor_matrix_ooc_ldlt* L = (supernodal_factor_matrix_ooc_ldlt*) vL;
  int sn;
  
  taucs_free(L->parent);
  taucs_free(L->first_child);
  taucs_free(L->next_child);
  taucs_free(L->col_to_sn_map);

  taucs_free(L->sn_size);
  taucs_free(L->sn_up_size);
  for (sn=0; sn<L->n_sn; sn++) {
    ooc_supernodal_free(L,sn);
  }

  taucs_free(L->sn_struct);
  taucs_free(L->sn_blocks);
  taucs_free(L->up_blocks);
  taucs_free(L->db_size);
  taucs_free(L->orig_sn_size);
  taucs_free(L->d_blocks);
  taucs_free(L->sn_max_ind);
  taucs_free(L->sn_rej_size);

  taucs_free(L);
}

/* this method tries to read the block from the second
   writing place, and if fails, gets it from the first
   writing place. */
static int
taucs_io_read_block(taucs_io_handle* handle,
		    int base_index,
		    int m,
		    int n,
		    int flags,
		    void* block)
{
  int ret = 0;
  int index = base_index + 4;
  
  ret = taucs_io_read_try(handle,index,m,n,flags,block);
  if ( ret == -1 )
    ret = taucs_io_read(handle,base_index,m,n,flags,block);
  else
    return ret;
  return ret;
}

static void
recursive_symbolic_elimination(int            j,
			       taucs_ccs_matrix* A,
			       int            first_child[],
			       int            next_child[],
			       int*           n_sn,
			       int            sn_size[],
			       int            sn_up_size[],
			       int*           sn_rowind[],
			       int            sn_first_child[], 
			       int            sn_next_child[], 
			       int	      sn_parent[],
			       int            rowind[],
			       int            column_to_sn_map[],
			       int            map[],
			       int            do_order,
			       int            ipostorder[],
			       double         given_mem,
			       void           (*sn_struct_handler)(),
			       void*          sn_struct_handler_arg
			       )
{
  int  i,ip,c,c_sn;
  int  in_previous_sn;
  int  nnz = 0;

  for (c=first_child[j]; c != -1; c = next_child[c]) {
    recursive_symbolic_elimination(c,A,
				   first_child,next_child,
				   n_sn,
				   sn_size,sn_up_size,sn_rowind,
				   sn_first_child,sn_next_child,sn_parent,
				   rowind, /* temporary */
				   column_to_sn_map,
				   map,
				   do_order,ipostorder,given_mem,
				   sn_struct_handler,sn_struct_handler_arg
				   );
  }

  
  in_previous_sn = 1;
  if (j == A->n) 
    in_previous_sn = 0; /* this is not a real column */
  else if (first_child[j] == -1) 
    in_previous_sn = 0; /* this is a leaf */
  else if (next_child[first_child[j]] != -1) 
    in_previous_sn = 0; /* more than 1 child */
  else if ((double)sn_up_size[column_to_sn_map[first_child[j]]]
	   *(double)(sn_size[column_to_sn_map[first_child[j]]]+1)
	   *sizeof(taucs_datatype) > given_mem)
    in_previous_sn = 0; /* size of supernode great than given memory */
  else { 
    /* check that the structure is nested */
    /* map contains child markers         */

    c=first_child[j];
    for (ip=(A->colptr)[j]; ip<(A->colptr)[j+1]; ip++) {
      i = (A->rowind)[ip];
      in_previous_sn = in_previous_sn && (map[i] == c);
    }
  }

  if (in_previous_sn) {
    c = first_child[j];
    c_sn = column_to_sn_map[c];
    column_to_sn_map[j] = c_sn;

    /* swap row indices so j is at the end of the */
    /* supernode, not in the update indices       */
    for (ip=sn_size[c_sn]; ip<sn_up_size[c_sn]; ip++) 
      if (sn_rowind[c_sn][ip] == j) break;
    assert(ip<sn_up_size[c_sn]);
    sn_rowind[c_sn][ip] = sn_rowind[c_sn][sn_size[c_sn]];
    sn_rowind[c_sn][sn_size[c_sn]] = j;

    /* mark the nonzeros in the map */
    for (ip=sn_size[c_sn]; ip<sn_up_size[c_sn]; ip++) 
      map[ sn_rowind[c_sn][ip] ] = j;

    sn_size   [c_sn]++;
    if((double)sn_size[c_sn]*(double)sn_up_size[c_sn]>=(1024.0*1024.0*1024.0))
      taucs_printf("debug!!!: sn_size[%d] = %d sn_up_size[%d] = %d\n ",c_sn,sn_size[c_sn],c_sn,sn_up_size[c_sn]);
    /* return c_sn; */
    return;
  }

  /* we are in a new supernode */
  if (j < A->n) {
    nnz = 1;
    rowind[0] = j;
    map[j]    = j;
    
    for (c=first_child[j]; c != -1; c = next_child[c]) {
      c_sn = column_to_sn_map[c];
      for (ip=sn_size[c_sn]; ip<sn_up_size[c_sn]; ip++) {
	i = sn_rowind[c_sn][ip];
	if (i > j && map[i] != j) { /* new row index */
	  map[i] = j;
	  rowind[nnz] = i;
	  nnz++;
	}
      }
      if (sn_struct_handler)
	(*sn_struct_handler)(sn_struct_handler_arg,
			     c_sn,sn_up_size[c_sn],&(sn_rowind[c_sn]));
    }

    for (ip=(A->colptr)[j]; ip<(A->colptr)[j+1]; ip++) {
      i = (A->rowind)[ip];
      if (map[i] != j) { /* new row index */
	map[i] = j;
	rowind[nnz] = i;
	nnz++;
      }
    }
  }

  /* append childs from root*/
  if (j == A->n) {
    for (c=first_child[j]; c != -1; c = next_child[c]) {
      c_sn = column_to_sn_map[c];
      if (sn_struct_handler)
	(*sn_struct_handler)(sn_struct_handler_arg,
			     c_sn,sn_up_size[c_sn],&(sn_rowind[c_sn]));
    }
  }

  /*printf("children of sn %d: ",*n_sn);*/
  for (c=first_child[j]; c != -1; c = next_child[c]) {
    c_sn = column_to_sn_map[c];

    /* omer - added parent info */
    sn_parent[c_sn] = *n_sn;

    /*printf("%d ",c_sn);*/
    if (c==first_child[j]) 
      sn_first_child[*n_sn] = c_sn;
    else {
      sn_next_child[ c_sn ] = sn_first_child[*n_sn];
      sn_first_child[*n_sn] = c_sn;
    }
  }
  /*printf("\n");*/

  if (j < A->n) {
    column_to_sn_map[j] = *n_sn;
    sn_size   [*n_sn] = 1;
    sn_up_size[*n_sn] = nnz;
    sn_rowind [*n_sn] = (int*) taucs_malloc(nnz * sizeof(int));
    for (ip=0; ip<nnz; ip++) sn_rowind[*n_sn][ip] = rowind[ip];
    if (do_order) {
      /* Sivan and Vladimir: we think that we can sort in */
      /* column order, not only in etree postorder.       */
      /*
				radix_sort(sn_rowind [*n_sn],nnz);
				qsort(sn_rowind [*n_sn],nnz,sizeof(int),compare_ints);
      */
      compare_indirect_map = ipostorder;
      qsort(sn_rowind [*n_sn],nnz,sizeof(int),compare_indirect_ints);
    }
    assert(sn_rowind [*n_sn][0] == j);
    (*n_sn)++;
  }
}

static
void recursive_postorder(int  j,
			 int  first_child[],
			 int  next_child[],
			 int  postorder[],
			 int  ipostorder[],
			 int* next
			 )
{
  int c;


  for (c=first_child[j]; c != -1; c = next_child[c]) {
    /*printf("*** %d is child of %d\n",c,j);*/
    recursive_postorder(c,first_child,next_child,
			postorder,ipostorder,next
			);
  }
  /*  printf(">>> j=%d next=%d\n",j,*next);*/
  if (postorder)  postorder [*next] = j;
  if (ipostorder) ipostorder[j] = *next;
  (*next)++;
}

static void
taucs_ccs_ooc_symbolic_elimination(taucs_ccs_matrix* A,
				   void* vL,
				   int do_order,
				   int do_column_to_sn_map,
				   double given_mem,
				   void           (*sn_struct_handler)(),
				   void*          sn_struct_handler_arg
				   )
{
  supernodal_factor_matrix_ooc_ldlt* L = (supernodal_factor_matrix_ooc_ldlt*) vL;
  int* first_child;
  int* next_child;
  int j;
  int* column_to_sn_map;
  int* map;
  int* rowind;
  int* parent;
  int* ipostorder;

  L->n         = A->n;
  L->sn_struct = (int**)taucs_malloc((A->n  )*sizeof(int*));
  L->sn_size   = (int*) taucs_malloc((A->n+1)*sizeof(int));
  L->sn_up_size   = (int*) taucs_malloc((A->n+1)*sizeof(int));
  L->first_child = (int*) taucs_malloc((A->n+1)*sizeof(int));
  L->next_child  = (int*) taucs_malloc((A->n+1)*sizeof(int));
  L->sn_max_ind = (int*) taucs_malloc((A->n+1)*sizeof(int));
  L->sn_rej_size = (int*) taucs_malloc((A->n+1)*sizeof(int));
  L->orig_sn_size = (int*) taucs_malloc((A->n+1)*sizeof(int));
  L->parent = (int*) taucs_malloc((A->n+1)*sizeof(int));
	
  column_to_sn_map = (int*)taucs_malloc((A->n+1)*sizeof(int));
  map              = (int*) taucs_malloc((A->n+1)*sizeof(int));

  first_child = (int*) taucs_malloc(((A->n)+1)*sizeof(int));
  next_child  = (int*) taucs_malloc(((A->n)+1)*sizeof(int));
    
  rowind      = (int*) taucs_malloc((A->n)*sizeof(int));

  taucs_printf("STARTING SYMB 1\n");

  /* compute the vertex elimination tree */
  parent      = (int*)taucs_malloc((A->n+1)*sizeof(int));
	
  taucs_ccs_etree(A,parent,NULL,NULL,NULL);
  for (j=0; j <= (A->n); j++) {
    first_child[j] = -1;
    L->parent[j] = -1;
  }
  for (j = (A->n)-1; j >= 0; j--) {
    int p = parent[j];
    next_child[j] = first_child[p];
    first_child[p] = j;
  }
  taucs_free(parent);

  taucs_printf("STARTING SYMB 2\n");

  ipostorder = (int*)taucs_malloc((A->n+1)*sizeof(int));
  { 
    int next = 0;
    /*int* postorder = (int*)taucs_malloc((A->n+1)*sizeof(int));*/
    recursive_postorder(A->n,first_child,next_child,
			NULL,
			ipostorder,&next);
    /*
    printf("ipostorder ");
    for (j=0; j <= (A->n); j++) printf("%d ",ipostorder[j]);
    printf("\n");
    printf(" postorder ");
    for (j=0; j <= (A->n); j++) printf("%d ",postorder[j]);
    printf("\n");
    */
  }

  taucs_printf("STARTING SYMB 3\n");

  L->n_sn = 0;
  for (j=0; j < (A->n); j++) map[j] = -1;
  for (j=0; j <= (A->n); j++) (L->first_child)[j] = (L->next_child)[j] = -1;

  taucs_printf("STARTING SYMB\n");

  recursive_symbolic_elimination(A->n,
				 A,
				 first_child,next_child,
				 &(L->n_sn),
				 L->sn_size,L->sn_up_size,L->sn_struct,
				 L->first_child,L->next_child,L->parent,
				 rowind,
				 column_to_sn_map,
				 map,
				 do_order,ipostorder,given_mem,
				 sn_struct_handler,sn_struct_handler_arg
				 );

  taucs_printf("AFTER SYMB\n");

  {
    double nnz   = 0.0;
    double flops = 0.0;
    taucs_get_statistics_ooc(A->flags,&flops,&nnz,L);
    taucs_printf("\t\tSymbolic Analysis of LDL^T: %.2e nonzeros, %.2e flops\n",
		 nnz, flops);
  }

  /*
  {
    int i;
    printf("c2sn: ");
    for (i=0; i<A->n; i++) printf("%d ",column_to_sn_map[i]);
    printf("\n");
  }
  */
  
  L->sn_struct = (int**)taucs_realloc( L->sn_struct,(L->n_sn  )*sizeof(int*));
  L->sn_size   = (int*) taucs_realloc( L->sn_size,(L->n_sn+1)*sizeof(int));
  L->sn_up_size   = (int*) taucs_realloc(L->sn_up_size,(L->n_sn+1)*sizeof(int));
  L->first_child = (int*) taucs_realloc(L->first_child,(L->n_sn+1)*sizeof(int));
  L->next_child  = (int*) taucs_realloc(L->next_child,(L->n_sn+1)*sizeof(int));
  L->sn_max_ind = (int*) taucs_realloc(L->sn_max_ind,(L->n_sn+1)*sizeof(int));
  L->sn_rej_size = (int*) taucs_realloc(L->sn_rej_size,(L->n_sn+1)*sizeof(int));
  L->orig_sn_size = (int*) taucs_realloc(L->orig_sn_size,(L->n_sn+1)*sizeof(int));
  L->parent = (int*) taucs_realloc(L->parent,(L->n_sn+1)*sizeof(int));
	
  L->sn_blocks     = taucs_calloc((L->n_sn), sizeof(taucs_datatype*)); /* so we can free before allocation */
  L->up_blocks     = taucs_calloc((L->n_sn), sizeof(taucs_datatype*));
  L->db_size = (int**) taucs_calloc((L->n_sn),sizeof(int*));
  L->d_blocks = (taucs_datatype**) taucs_calloc((L->n_sn),sizeof(taucs_datatype*));

  taucs_free(rowind);
  taucs_free(map);

  if(do_column_to_sn_map)
    L->col_to_sn_map = column_to_sn_map;
  else
    taucs_free(column_to_sn_map);
  
  for(j=0;j<L->n_sn+1;j++)
    L->orig_sn_size[j] = L->sn_size[j];
  
  taucs_free(next_child);
  taucs_free(first_child);
  taucs_free(ipostorder);
}

/*************************************************************/
/* left-looking factor routines                              */
/*************************************************************/

/*********************************************/
/* additional methods for LDLT factorization */
/*********************************************/

static int
build_LU_from_D( taucs_datatype D[],
		 taucs_datatype L[],
		 taucs_datatype U[] )
{
  /* D= [0] [1]		D=LU where L= 1	       	0 and U= [0]	[1]
        [2] [3]		              [2/0]	1      	  0    	[3-(1*2)/0] */
  
  int need_switch = 0;
  int map_D[4];		/* map of indices, for the switch if needed */
  
  /* check which of D11 and D21 is bigger to take as pivot */
  if ( taucs_abs(D[0]) >= taucs_abs(D[2]) ) {
    /* no need to switch */
    map_D[0] = 0; map_D[1] = 1; map_D[2] = 2; map_D[3] = 3;
  }
  else	{
    map_D[0] = 2; map_D[1] = 3; map_D[2] = 0; map_D[3] = 1;
    need_switch = 1;
  }
  
  L[0] = taucs_one_const;
  L[1] = taucs_zero_const;
  L[3] = taucs_one_const;
  L[2] = taucs_div(D[map_D[2]], D[map_D[0]]);
  
  U[0] = D[map_D[0]];
  U[1] = D[map_D[1]];
  U[2] = taucs_zero_const;
  U[3] = taucs_sub( D[map_D[3]],
		    taucs_div( taucs_mul(D[map_D[2]],D[map_D[1]]),
			       D[map_D[0]] ));
  
  return need_switch;
  
}

/* This method solves either D*x = b or x*D = b, where D is a 2*2 pivot 
   block from the LDLt factorization. x and b are vectors of size 2. (or their 
   transforms).
   D is symmetric, and of size 4,
   where [0] = D11, [1] = D21, [2] = D21 and [3] = D22
   2 ways to solve: either use GEPP or cramer's rule.
   We implement both but use only one.
   is_left indicates which of the systems is to be solved.
   (left is the x*D = b system)*/
static void
two_times_two_diagonal_block_solve( taucs_datatype D[],
				    taucs_datatype x[],
				    taucs_datatype b[],
				    int use_cramers_rule,
				    int is_left,
				    taucs_datatype L[],
				    taucs_datatype U[],
				    int need_switch)
{
  int map_b[2]; /* map of indices, for the switch if needed */
  int map_x[2]; /* map of indices, for the switch if needed */
  
  /* defualt - no switch */
  map_b[0] = 0;		map_b[1] = 1;
  map_x[0] = 0;		map_x[1] = 1;
  
  /* simple case b=[0,0] */
  if ( taucs_cmp(b[0],b[1]) && taucs_cmp(b[0],taucs_zero_const) ) {
    x[0] = x[1] = taucs_zero_const;
    return;
  }
  
  if ( need_switch ) {
    /* we switch b only if it is a column, not when is_left = 1*/
    if (!is_left) {
      map_b[0] = 1;		map_b[1] = 0;	/* switch */
    }
    map_x[0] = 1;		map_x[1] = 0;	/* switch */
  }
  
  if ( use_cramers_rule ) {
    
    /* cramer's rule */
    taucs_datatype D21,D11_D21,D22_D21,devider,B1_D21,B2_D21;
    
    /*	simply multiply by the inverse of the diagonal block,
	same as in lapack's dsytrs.
	we do not switch in cramer's rule.
	doesn't matter if its left or right. */
    D21 = D[2];
    assert(taucs_re(D21) || taucs_im(D21));
    
    D11_D21 = taucs_div( D[0],D21 );
    D22_D21 = taucs_div( D[3],D21 );
    devider = taucs_sub( taucs_mul( D11_D21, D22_D21), taucs_one_const );
    assert(taucs_re(devider) || taucs_im(devider));
    
    B1_D21 = taucs_div( b[0]/*xdense[i]*/, D21 );
    B2_D21 = taucs_div( b[1]/*xdense[i+1]*/, D21 );
    
    
    /* x[i] = (D22_D21*B1_D21 - B2_D21) / devider
       x[i+1] = (D11_D21*B2_D21 - B1_D21) / devider */
    x[0] = taucs_div( taucs_sub( taucs_mul( D22_D21, B1_D21 ), B2_D21 ),
		      devider );
    x[1] = taucs_div( taucs_sub( taucs_mul( D11_D21, B2_D21 ), B1_D21 ),
		      devider );
  }
  else {
    /* GEPP */
    taucs_datatype temp[2]; /* temporary */
    
    /* D= [0] [1]		D=LU where L= 1	       	0 and U= [0]	[1]
          [2] [3]			      [2/0]	1      	 0     	[3-(1*2)/0] */
   
    if ( is_left ) {
      /* First calculate temp: temp*U=b */
      temp[map_b[0]] = taucs_div( b[map_b[0]], U[0] );
      temp[map_b[1]] = taucs_div( taucs_sub( b[map_b[1]],
					     taucs_mul( temp[map_b[0]], U[1] )),
				  U[3]);
      
      /* Now we are ready for x: x*L = temp */
			x[map_x[1]] = temp[map_b[1]];
			x[map_x[0]] = taucs_sub( temp[map_b[0]],
						 taucs_mul( x[map_x[1]], L[2]));
    } else {
      /* First calculate temp: L*temp=b */
      temp[map_b[0]] = b[map_b[0]];
      temp[map_b[1]] = taucs_sub( b[map_b[1]],
				  taucs_mul( b[map_b[0]], L[2] ));
      
      
      /* Now we are ready for x: U*x = temp */
      x[1] = taucs_div( temp[map_b[1]],U[3] );
      x[0] = taucs_div( taucs_sub( temp[map_b[0]],
				   taucs_mul( x[1], U[1] ) ),
			U[0] );
    }
  }
}



static void
free_ldlt_factor(supernodal_ldlt_factor* fct)
{
  if (fct->rowind) taucs_free(fct->rowind);
  if (fct->indices) taucs_free(fct->indices);
}

static supernodal_ldlt_factor*
create_ldlt_factor(int sn_size)
{
  int i;
  supernodal_ldlt_factor* fct = 
    (supernodal_ldlt_factor*)taucs_malloc(sizeof(supernodal_ldlt_factor));
  if ( !fct ) return NULL;
  fct->rowind = (int*)taucs_malloc(sn_size*sizeof(int));
  fct->indices = (int*)taucs_malloc(sn_size*sizeof(int));
  fct->rejected_size = 0;
  
  if ( !fct->rowind || !fct->indices ) {
    free_ldlt_factor(fct);
    return NULL;
  }
  
  for (i=0; i<sn_size; i++)
    fct->rowind[i] = fct->indices[i] = i;
  
  return fct;
}



static int
ll_sn_reorder_ldlt(int sn,
		   supernodal_factor_matrix_ooc_ldlt* snL,
		   supernodal_ldlt_factor* fct,
		   int parent,
		   int new_sn_size,
		   int new_up_size,
		   taucs_io_handle* handle)
{
  int INFO = 0;
  int i,rej_size;
  int* current_struct = NULL;
  int old_sn_size = snL->sn_size[sn];
  
  rej_size = fct->rejected_size;
	
  if ( rej_size ) {
    
    /* copying the structure of the parent to a new bigger struct */
    if ( !snL->sn_struct[parent] ) { 
      /* doesn't exist yet - allocated and read */
      snL->sn_struct[parent] = (int*)taucs_malloc((rej_size+snL->sn_up_size[parent])*sizeof(int));
      if ( !snL->sn_struct[parent] ) {
	taucs_printf("Could not allocate memory (for structure) %d\n",(fct->rejected_size) + snL->sn_size[parent]);
	return -1;
      }
      
      taucs_io_read(handle,IO_BASE+parent,1,
		    snL->sn_up_size[parent],TAUCS_INT,snL->sn_struct[parent]);
    } else { 
      /* exists - just realloc */
      snL->sn_struct[parent] = (int*)taucs_realloc(snL->sn_struct[parent],
						   (rej_size+snL->sn_up_size[parent])*sizeof(int));
      if ( !snL->sn_struct[parent] ) {
	taucs_printf("Could not allocate memory (for structure) %d\n",(fct->rejected_size) + snL->sn_size[parent]);
	return -1;
      }
    }
    
    /* copying the rejected rows to the start of the parent. 
       and updating col_to_sn_map */
    for (i=snL->sn_up_size[parent]-1; i>=0; i--)
      snL->sn_struct[parent][i+rej_size] =
	snL->sn_struct[parent][i];
    for (i=0; i<rej_size; i++) {
      snL->sn_struct[parent][i] =
	snL->sn_struct[sn][fct->indices[i+new_sn_size]];
      snL->col_to_sn_map[snL->sn_struct[parent][i]] = parent;
    }
    
    snL->sn_size[parent]+=rej_size;
    snL->sn_up_size[parent]+=rej_size;
  }
  
  /* rebuild the structure of the current supernode */
  current_struct = (int*)taucs_malloc((new_sn_size+new_up_size)*sizeof(int));
  if ( !current_struct ) {
    taucs_printf("Could not allocate memory (for structure) %d\n",(new_sn_size+new_up_size));
    return -1;
  }
  
  /* put the rejected rows in the beginning of the update rows (F2) */
  for (i=0; i<new_sn_size; i++)
    current_struct[i] = snL->sn_struct[sn][fct->indices[i]];
  for (i=0; i<rej_size; i++ )
    current_struct[i+new_sn_size] = 
      snL->sn_struct[sn][fct->indices[i+new_sn_size]];
  for (i=0; i<new_up_size - rej_size; i++ )
    current_struct[i+new_sn_size+rej_size] =
      snL->sn_struct[sn][i+old_sn_size];
  
  taucs_free(snL->sn_struct[sn]);
  snL->sn_struct[sn] = current_struct;
  
  snL->sn_rej_size[sn] = rej_size;
  
  return INFO;
}



static int
supernodal_ldlt_modify_parent(	supernodal_factor_matrix_ooc_ldlt* snL,
				int sn,
				int rejected)
{
  int sn_size = snL->sn_size[sn];
  int old_sn_size = sn_size - rejected;
  int up_size = snL->sn_up_size[sn] - sn_size;
  int i,j;
  taucs_datatype* temp;
  
  if ( rejected ) {
    
    if ( snL->sn_blocks[sn] ) {
      temp = 
	(taucs_datatype*)taucs_calloc(sn_size*sn_size,(sizeof(taucs_datatype)));
      if (!temp) return -1;
      for (	i=0;i<old_sn_size;i++ )
	for ( j=0; j<old_sn_size; j++ )
	  temp[(i+rejected)*sn_size+j+rejected] = snL->sn_blocks[sn][i*old_sn_size+j];
      
      taucs_free(snL->sn_blocks[sn]);
      snL->sn_blocks[sn] = temp;
    }
    
    if ( snL->up_blocks[sn] ) {
      temp = 
	(taucs_datatype*)taucs_calloc(sn_size*up_size,(sizeof(taucs_datatype)));
      if (!temp) return -1;
      for (	i=0;i<old_sn_size;i++ )
	for ( j=0;j<up_size;j++ )
	  temp[(i+rejected)*up_size+j] = snL->up_blocks[sn][i*up_size+j];
      
      taucs_free(snL->up_blocks[sn]);
      snL->up_blocks[sn] = temp;
    }
    
    snL->d_blocks[sn] = (taucs_datatype*)taucs_realloc(snL->d_blocks[sn],
						       sn_size*2*(sizeof(taucs_datatype)));
    if (!snL->d_blocks[sn]) return -1;
    for (	i=old_sn_size*2;
		i<sn_size*2;
		i++ )
      snL->d_blocks[sn][i] = taucs_zero_const;
    
    snL->db_size[sn] = (int*)taucs_realloc(snL->db_size[sn],
					   sn_size*(sizeof(int)));
    if (!snL->db_size[sn]) return -1;
    
    /* adjust the max index */
    for(i=0; i<sn_size-old_sn_size; i++)
      snL->sn_max_ind[sn] = max( snL->sn_max_ind[sn], snL->sn_struct[sn][i+old_sn_size] );
    
  }
  return 0;
}

static int
supernodal_ldlt_modify_child(	supernodal_factor_matrix_ooc_ldlt* snL,
				int sn,
				int rejected)
{
  int old_sn_size = snL->sn_size[sn];
  int new_sn_size = old_sn_size - rejected;
  int old_up_size = snL->sn_up_size[sn] - old_sn_size;
  int new_up_size = old_up_size + rejected;
  int i,j;
  int* db_size_temp = NULL;
  taucs_datatype * tempF2 = NULL,*temp = NULL;
  
  if ( !rejected ) return 0;

  /* we need to reallocate in the correct size */
  if ( new_sn_size ) {
    db_size_temp = (int*)taucs_calloc(new_sn_size,sizeof(int));
    if ( !db_size_temp ) {
      taucs_printf("Could not allocate memory (for db_size) %d\n",new_sn_size);
      return -1;
    }
    temp = (taucs_datatype*)taucs_calloc(2*new_sn_size,sizeof(taucs_datatype));
    if ( !temp ) {
      taucs_printf("Could not allocate memory (for diagonal blocks) %d\n",2*new_sn_size);
      return -1;
    }
    for (i=0; i<new_sn_size; i++) {
      db_size_temp[i] = snL->db_size[sn][i];
      temp[i*2] = snL->d_blocks[sn][i*2];
      temp[i*2+1] = snL->d_blocks[sn][i*2+1];
    }
  }
  taucs_free(snL->d_blocks[sn]);
  taucs_free(snL->db_size[sn]);
  snL->d_blocks[sn] = temp;
  snL->db_size[sn] = db_size_temp;
  
  /* prepare memory for new sized F2 */
  if ( new_sn_size && new_up_size ) {
    tempF2 = 
      (taucs_datatype*)taucs_calloc(new_sn_size*new_up_size,sizeof(taucs_datatype));
    if ( !tempF2 ) {
      taucs_printf("Could not allocate memory (for F2) %d\n",new_sn_size*new_up_size);
      return -1;
    }
  }	else {
    tempF2 = NULL;
  }
  
  /* copy from F1 the rejected rows to tempF2 */
  for (i=0; i<new_sn_size && new_up_size; i++) {
    for (j=0; j<rejected; j++) {
      tempF2[i*(new_up_size)+j] = snL->sn_blocks[sn][i*old_sn_size+j+new_sn_size];
    }
  }
  
  /* copy from F2 to tempF2 */
  for (i=0; i<new_sn_size && new_up_size; i++) {
    for (j=rejected; j<new_up_size; j++) {
      tempF2[i*(new_up_size)+j] = snL->up_blocks[sn][i*old_up_size+j-rejected];
    }
  }
  
  taucs_free(snL->up_blocks[sn]);
  snL->up_blocks[sn] = tempF2;
  
  
  snL->sn_size[sn] = new_sn_size;
  
  if ( new_sn_size ) {
    temp = 
      (taucs_datatype*)taucs_calloc(new_sn_size*new_sn_size,sizeof(taucs_datatype));
    if (!temp) return -1;
    for (i=0; i<new_sn_size; i++) {
      for (j=0; j<new_sn_size; j++) {
	temp[i*new_sn_size+j] = snL->sn_blocks[sn][i*old_sn_size+j];
      }
    }
  } else {
    temp = NULL;
  }
  
  taucs_free(snL->sn_blocks[sn]);
  snL->sn_blocks[sn] = temp;
  
  return 0;
}

static int
leftlooking_supernodal_update_ldlt(int J,int K,
				   int bitmap[],
				   taucs_datatype* dense_update_matrix,
				   taucs_ccs_matrix* A,
				   supernodal_factor_matrix_ooc_ldlt* L,
				   int rejected, int* p_exist_upd, taucs_io_handle* handle)
{
  int i,j,ir,k,index;
  int sn_size_father = (L->sn_size)[J];
  int up_size_father = (L->sn_up_size)[J] - sn_size_father;
  int LD_sn_J = (L->sn_size)[J];
  int LD_up_J = (L->sn_up_size)[J] - sn_size_father;
  int sn_size_child = (L->sn_size)[K] - rejected;
  /* the up_size_child in the direct update does not include the rejected rows */
  int up_size_child = (L->sn_up_size)[K] - (L->sn_size)[K];
  int LD_sn_K = (L->sn_size)[K];
  int LD_up_K = (L->sn_up_size)[K] - (L->sn_size)[K];
  int exist_upd=0;
  int first_row=0, ind_first_row=0, ind_last_row=0;
  int row_count=0;
  int LDA,LDB,LDC,M,N;
  int max_index_snJ;
  taucs_datatype* F2D11 = NULL;
  taucs_datatype v,D[4],b[2];
  int* rejected_list = NULL;


  /* NOTE: when this is a direct update (from a son to its parent),
     and there are rejected rows, and this can be seen by the value
     of 'rejected', we are in a state in which the sn_struct is already
     reordered to the way the son is suppose to be, but it has not been
     modified yet. we first update the parent using the updated rows that
     involve the rejected rows, and only than we modify the son to the
     correct structure. This means that here we do not have the sn_struct
     and the actual structure aligned. */
  
  /* because of modifications to the supernode from its children,
     we need to find out who is the biggest index in the sn part of J
     (it is not ordered anymore) */
  /* i am not sure we need this!! */
  max_index_snJ = L->sn_max_ind[J];
  
  /* is this row index included in the columns of sn J? */
  /* we do not look at the rejected rows in K and we do
     not concider updates to rows in the rejected_list*/
  i = rejected ? 0 : L->sn_rej_size[K];
  for(;i<up_size_child;i++) {
    if(bitmap[L->sn_struct[K][i+sn_size_child+rejected]]
       && L->col_to_sn_map[L->sn_struct[K][i+sn_size_child+rejected]]==J) {
      if(!exist_upd) ind_first_row = i;
      row_count++;
      exist_upd = 1;
      ind_last_row = i+1;
    } 
  }
  
  
  /* Here we allocate K's supernode when needed */
  if (rejected && LD_sn_K) {
    if ( !(L->sn_blocks)[K] ) {
      (L->sn_blocks)[K] = taucs_calloc(LD_sn_K*LD_sn_K,sizeof(taucs_datatype));
			if (!(L->sn_blocks)[K]) {
				taucs_printf("taucs_calloc (L->sn_blocks)[%d]\n",K);
				return -1;
			}
      taucs_io_read_block(handle,IO_BASE+2*L->n_sn+6*K,
			  LD_sn_K,LD_sn_K,TAUCS_CORE_DATATYPE,(L->sn_blocks)[K]);
    }
    if ( !(L->up_blocks)[K] && LD_up_K ) {
      (L->up_blocks)[K] = taucs_calloc(LD_up_K*LD_sn_K,sizeof(taucs_datatype));
      if (!(L->up_blocks)[K]) {
				taucs_printf("taucs_calloc (L->up_blocks)[%d]\n",K);
				return -1;
			}
			taucs_io_read_block(handle,IO_BASE+2*L->n_sn+6*K+1,
			  LD_up_K,LD_sn_K,TAUCS_CORE_DATATYPE,(L->up_blocks)[K]);
    }
	}
  if (row_count && LD_sn_K) {
    if ( !(L->d_blocks)[K] ) {
      (L->d_blocks)[K] = taucs_calloc(sn_size_child*2,sizeof(taucs_datatype));
			if (!(L->d_blocks)[K]) {
				taucs_printf("taucs_calloc (L->d_blocks)[%d]\n",K);
				return -1;
			}
      taucs_io_read(handle,IO_BASE+2*L->n_sn+6*K+2,
		    sn_size_child,2,TAUCS_CORE_DATATYPE,(L->d_blocks)[K]);
    }
    if ( !(L->db_size)[K] ) {
      (L->db_size)[K] = taucs_calloc(sn_size_child,sizeof(int));
			if (!(L->db_size)[K]) {
				taucs_printf("taucs_calloc (L->db_size)[%d]\n",K);
				return -1;
			}
      taucs_io_read(handle,IO_BASE+2*L->n_sn+6*K+3,
		    sn_size_child,1,TAUCS_INT,(L->db_size)[K]);
    }
    if ( !(L->up_blocks)[K] && LD_up_K ) {
      (L->up_blocks)[K] = taucs_calloc(LD_up_K*LD_sn_K,sizeof(taucs_datatype));
			if (!(L->up_blocks)[K]) {
				taucs_printf("taucs_calloc (L->up_blocks)[%d]\n",K);
				return -1;
			}
      taucs_io_read_block(handle,IO_BASE+2*L->n_sn+6*K+1,
			  LD_up_K,LD_sn_K,TAUCS_CORE_DATATYPE,(L->up_blocks)[K]);
    }
  }
  
  
  if ( rejected ) {
    /* we put the rejected rows in the rejected_list, so that children
       of this child do not update the supernode for these rows */
    rejected_list = (int*)taucs_malloc(rejected*sizeof(int));
		if ( !rejected_list ) {
			taucs_printf("rejected_list %d\n",rejected);
			return -1;
		}
    
    /* we need to copy the updates of the rejected rows from the child
       to the parent. we put the rejected rows in the beginning of the 
       parent's supernode, and that's where we will put the updates too. */
    
    for (i=0; i<rejected; i++) {
      k = bitmap[L->sn_struct[K][i+sn_size_child]]-1;
      rejected_list[i] = L->sn_struct[K][i+sn_size_child];
      /* copy from F1 of the child the rejected updates to F1 of the parent*/
      for (j=i; j<rejected; j++) {
	index = bitmap[L->sn_struct[K][j+sn_size_child]]-1;
	ir = k;
	if (index < ir) {
	  ir = index;
	  index = k;
	}
	L->sn_blocks[J][ir*LD_sn_J + index] = 
	  taucs_add( L->sn_blocks[J][ir*LD_sn_J + index],
		     L->sn_blocks[K][(i+sn_size_child)*LD_sn_K + j+sn_size_child] );
      }
      /* copy the updates from F2 of the child to F1 of the parent */
      for (j=0; j<ind_last_row; j++) {
	index = bitmap[L->sn_struct[K][j+sn_size_child+rejected]]-1;
	assert(k < index && index<sn_size_father);
	L->sn_blocks[J][k*LD_sn_J + index] = 
	  taucs_add( L->sn_blocks[J][k*LD_sn_J + index],
		     L->up_blocks[K][(i+sn_size_child)*LD_up_K + j] );
      }
      /* copy the updates from F2 of the child to F2 of the parent */
      for (j=0; j<up_size_child-ind_last_row; j++) {
	index = bitmap[L->sn_struct[K][j+sn_size_child+rejected+ind_last_row]]-1;
	L->up_blocks[J][k*LD_up_J + index] = 
	  taucs_add( L->up_blocks[J][k*LD_up_J + index],
		     L->up_blocks[K][(i+sn_size_child)*LD_up_K + j+ind_last_row] );
      }
    }
  }
  
  if(row_count){
    
    first_row  = ind_first_row + sn_size_child + rejected;	
    LDA = M  = up_size_child - ind_first_row ;
    LDB = up_size_child;
    LDC =  up_size_father + sn_size_father;
    /* if LDC is bigger than M, we don't need it, and it might
       hurt us if M*N is the size of the dense matrix. */
    if ( LDC > M ) LDC = M;
    
    N = ind_last_row - ind_first_row;/* was row_count */
    assert( row_count <= N );
    
    
    /* we now need to allocate another dense matrix, which will hold
       the F2*D11 multiplication.
       This means that the update part of each supernode should be sorted, 
       and they are because we don't mess with the order of the update part, 
       only add more columns that were moved to the father, 
       and therefor we know that they are included in J */
    F2D11 = (taucs_datatype*)taucs_calloc(sn_size_child*M,sizeof(taucs_datatype));
		if (!F2D11) {
			taucs_printf("taucs_calloc F2D11\n");
			return -1;
		}
    
    /* Now we calculate the F2D11 */
    for (i=0;i<sn_size_child;i++) {
      if ( L->db_size[K][i] == 1 ) { /* 1*1 block */
	for (j=0; j<M; j++) {
	  v = L->up_blocks[K][i*LD_up_K + j+ind_first_row];
	  if ( taucs_re(v) || taucs_im(v) )
	    F2D11[i*M + j] = taucs_mul( v, L->d_blocks[K][i*2] );
	}
      }	else if ( L->db_size[K][i] == 2) {
	
	/* make the diagonal block for the 2*2 solution */
	D[0] = L->d_blocks[K][i*2];
	D[1] = D[2] = L->d_blocks[K][i*2 + 1];
	D[3] = L->d_blocks[K][(i+1)*2];
	
	for (j=0; j<M; j++) {
	  
	  /* make the vectors for the 2*2 solution */
	  b[0] = L->up_blocks[K][i*LD_up_K + j+ind_first_row];
	  b[1] = L->up_blocks[K][(i+1)*LD_up_K + j+ind_first_row];
	  
	  /* solve the equations */
	  /*two_times_two_diagonal_block_solve(D,x,b,FALSE,TRUE);*/
	  
	  F2D11[i*M + j] = 
	    taucs_add(taucs_mul(b[0],D[0]),
		      taucs_mul(b[1],D[2]));
	  F2D11[(i+1)*M + j] = 
	    taucs_add(taucs_mul(b[0],D[1]),
		      taucs_mul(b[1],D[3]));

	}
	
	i++; /* we need to skip the next column, we already did her */
	
      }	else {/* shouldn't get here */
	taucs_printf("db_size[%d] = %d\n",i,L->db_size[K][i] );
	assert(0);
      }
    }
    /* instead of having row_count as the number of rows in the
       calculated matrix, we put ind_last_row - ind_first_row.
       they should be the same except when the parent was modified
       to have a rejected row from a different child, which is in
       the update rows of this child, but not in the correct order
       (there are rows in between first_row and last_row that do
       not update the parent). we do not update the parent from 
       the results of these rows in the middle.*/
    
    taucs_gemm ("No Conjugate",
		"Conjugate",
		&M,&N,&sn_size_child,
		&taucs_one_const,
		F2D11,&LDA,
		&(L->up_blocks[K][ind_first_row]),&LDB,
		&taucs_zero_const,
		dense_update_matrix,&LDC);
    
    
    for(j=0;j<N;j++) {
      i = bitmap[L->sn_struct[K][first_row+j]]-1;
      if ( i == -1 ) continue;
      for(ir=j;ir<N;ir++){
	index = i;
	k = bitmap[L->sn_struct[K][first_row+ir]]-1;
	if ( k == -1 ) continue;
	if ( k < index ) {
	  /* the update will go to a place we don't look at ( the upper part 
	     of the triangle */
	  index = k; k = i;
	}
	assert(k>=0 && k<sn_size_father && index>=0 && index<sn_size_father);
	L->sn_blocks[J][index*LD_sn_J + k] =
	  taucs_sub( L->sn_blocks[J][index*LD_sn_J + k] , dense_update_matrix[j*LDC+ir]);
      }
      
      for(ir=N;ir<M;ir++){
	k = bitmap[L->sn_struct[K][ir+first_row]]-1;
	if (k == -1 ) continue;
	assert(k>=0 && k<LD_up_J && i>=0 && i<sn_size_father);
	L->up_blocks[J][i*LD_up_J + k] 	=
	  taucs_sub( L->up_blocks[J][i*LD_up_J + k] , dense_update_matrix[j*LDC+ir]);
      }
    }
    
    
    /* here we 'silence' the rejected rows from the list, 
       so that grand children do not update them ( they already
       updated them in the child */
    for (i=0; i<rejected; i++) {
      bitmap[rejected_list[i]] = 0;
		}
    taucs_free(rejected_list);
		taucs_free(F2D11);
    
    *p_exist_upd = TRUE;
  }
  
  return 0;
}


static int
recursive_leftlooking_supernodal_update_ldlt(int J,int K,
					     int bitmap[], int direct_update,
					     taucs_datatype* dense_update_matrix,
					     taucs_ccs_matrix* A,
					     supernodal_factor_matrix_ooc_ldlt* L,
					     int rejected, int* exist_upd, taucs_io_handle* handle)
{
  int  child,i;
  
  if ( direct_update ) {
    for(i=0;i<L->sn_size[J];i++) bitmap[L->sn_struct[J][i]]=i+1;
    for(i=L->sn_size[J];i<L->sn_up_size[J];i++)	bitmap[L->sn_struct[J][i]] = i-L->sn_size[J]+1;
  }
  
  if (leftlooking_supernodal_update_ldlt(J,K,bitmap,dense_update_matrix,
					 A,L,rejected,exist_upd,handle) ) {
    taucs_printf("leftlooking_supernodal_update_ldlt\n");
		return -1;
  }
  
  for (child = L->first_child[K]; child != -1; child = L->next_child[child]) {
    if ( recursive_leftlooking_supernodal_update_ldlt(	J,child, bitmap,
							FALSE, dense_update_matrix,
							A,L,0,exist_upd,handle)) {
      taucs_printf("recursive_leftlooking_supernodal_update_ldlt\n");
			return -1;
    }
  }
  
  if ( direct_update ) {
    for(i=0;i<L->sn_up_size[J];i++) bitmap[L->sn_struct[J][i]]=0;
  }
  
  return 0;
}

static int
leftlooking_supernodal_front_factor_ldlt(int sn,
					 int* indmap,
					 taucs_ccs_matrix* A,
					 supernodal_factor_matrix_ooc_ldlt* L,
					 int parent,
					 int* p_rejected,
					 taucs_io_handle* handle)
{
  int ip,jp;
  int*    ind;
  taucs_datatype* re;
  int INFO;
  int new_sn_size, new_up_size;
  supernodal_ldlt_factor* fct = NULL;
  
  int sn_size = (L->sn_size)[sn];
  int up_size = (L->sn_up_size)[sn] - (L->sn_size)[sn];
  
  if (!sn_size) return 0;
	
  /* creating transform for real indices */
  for(ip=0;ip<(L->sn_up_size)[sn];ip++) 
    indmap[(L->sn_struct)[sn][ip]] = ip;
  
  /* take only original rows from A, the rejected are updated from
     the child that rejected them */
  for(	jp=L->sn_size[sn] - L->orig_sn_size[sn];
	jp<L->sn_size[sn];
	jp++ ) {
    ind = &(A->rowind[A->colptr[ (L->sn_struct)[sn][jp] ]]);
    re  = &(A->taucs_values[A->colptr[ (L->sn_struct)[sn][jp] ]]);
    for(ip=0;
	ip < A->colptr[ (L->sn_struct)[sn][jp] + 1 ]
	  - A->colptr[ (L->sn_struct)[sn][jp] ];
	ip++) {
      if (indmap[ind[ip]] < sn_size)
	(L->sn_blocks)[sn][ sn_size*jp + indmap[ind[ip]]] =
	  taucs_add( (L->sn_blocks)[sn][ sn_size*jp + 
					 indmap[ind[ip]]] , re[ip] );
      else
	(L->up_blocks)[sn][ up_size*jp + indmap[ind[ip]] - sn_size] =
	  taucs_add( (L->up_blocks)[sn][ up_size*jp + indmap[ind[ip]] - 
					 sn_size] , re[ip] );
    }
  }
  
  /* solving of lower triangular system for L */
  fct = create_ldlt_factor(sn_size);

  /*writeSNLToFile(L,sn,"lla");*/
  
  INFO = taucs_dtl(internal_supernodal_front_factor_ldlt)(fct,L->sn_blocks[sn],L->up_blocks[sn], 
				      L->d_blocks[sn], L->db_size[sn],
				      L->sn_size[sn], L->sn_up_size[sn] - L->sn_size[sn],sn,gl_factor_alg);
  
  if (INFO) {
    taucs_printf("\t\tLDL^T Factorization: Matrix is singular.\n");
    taucs_printf("\t\t                    problematic column: %d\n",
		 (L->sn_struct)[sn][INFO]);
    return -1;
  }
  
  /* any columns that were not factored are moved to the parent's supernode. */
  new_sn_size = sn_size - fct->rejected_size;
  new_up_size = up_size + fct->rejected_size;/* not needed anymore */
  
  /* here we only reorder the rows, the actual trimming is done in
     supernodal_ldlt_modify_child after the direct update is done. 
     added here the change in col_to_sn_map.*/
  INFO = ll_sn_reorder_ldlt(sn,L,fct,parent,new_sn_size,new_up_size,handle);
  
  if (INFO) {
    taucs_printf("\t\tLDL^T Factorization: Problem reordering columns.\n");
    return -1;
  }
  
  *p_rejected = fct->rejected_size;
  /*writeSNLToFile(L,sn,"llb");*/
  
  free_ldlt_factor(fct);
  taucs_free(fct);
  
  return 0;
}


/* This is the ooc version. the second one is the in core version.
   i don't see why not to have the incore version for the in core factorization. 
   static int
   recursive_leftlooking_supernodal_factor_ldlt(int sn,       
   int is_root,  
   int* bitmap,
   int* indmap,
   taucs_ccs_matrix* A,
   supernodal_factor_matrix_ooc_ldlt* L,
   int parent,
   int* p_rejected)
   {
   int  child,maxIndex,i;
   int sn_size = L->sn_size[sn];
   int sn_up_size = L->sn_up_size[sn];
   taucs_datatype* dense_update_matrix = NULL;
   int rejected_from_child = 0;
   
   for (child = L->first_child[sn]; child != -1; child = L->next_child[child]) {
   if (recursive_leftlooking_supernodal_factor_ldlt(child,
   FALSE,
   bitmap,indmap,
   A,L,sn,&rejected_from_child)) {
   return -1;
   }
   }    
   
   if (!is_root) {
   if (sn_size) {
   (L->sn_blocks)[sn] = (taucs_datatype*)
   taucs_calloc((sn_size*sn_size),sizeof(taucs_datatype));
   (L->d_blocks)[sn] = (taucs_datatype*)
   taucs_calloc((sn_size*2),sizeof(taucs_datatype));
   (L->db_size)[sn] = (int*)
   taucs_calloc(sn_size,sizeof(int));
   }
   if (sn_up_size-sn_size && sn_size) {
   (L->up_blocks)[sn] = (taucs_datatype*)
   taucs_calloc(((sn_up_size-sn_size)*sn_size),sizeof(taucs_datatype));
   }
   (L->orig_sn_size)[sn] = L->sn_size[sn];
   
   maxIndex = 0;
   for ( i=0; i<L->sn_size[sn]; i++ ) {
   maxIndex  = max(L->sn_struct[sn][i],maxIndex);
   }
   L->sn_max_ind[sn] = maxIndex;
   
   if (!dense_update_matrix && (L->first_child[sn] != -1)) 
   dense_update_matrix = (taucs_datatype*) 
   taucs_calloc(sn_up_size*sn_size,sizeof(taucs_datatype));
   
   for (child = L->first_child[sn]; child != -1; child = L->next_child[child])
   recursive_leftlooking_supernodal_update_ldlt(sn,child,
   bitmap,dense_update_matrix,
   A,L,L->sn_rej_size[child]);
   
   taucs_free(dense_update_matrix);
   if (leftlooking_supernodal_front_factor_ldlt(sn,indmap,bitmap,
   A,L,parent,p_rejected)) {
   return -1;
   }
   }
   
   return 0;
   }*/

static int
recursive_leftlooking_supernodal_factor_ldlt(	int sn,       /* this supernode */
						int is_root,  /* is v the root? */
						int* indmap,
						int* bitmap,
						taucs_ccs_matrix* A,
						supernodal_factor_matrix_ooc_ldlt* L,
						int parent,
						int* p_rejected,
						taucs_io_handle* handle)
{
  int  child, i,j, maxIndex;
  int  sn_size = -1;
  int sn_up_size = -1;
  taucs_datatype* dense_update_matrix = NULL,* temp = NULL;
  int rejected_from_child = 0, exist_upd = FALSE;
  int dense_sn_size = 0, dense_up_size = 0;
  
  if (!is_root) {
    sn_size = L->sn_size[sn];
    sn_up_size = L->sn_up_size[sn];
  }
  
  
  if (!is_root) {
    (L->sn_blocks)[sn] = (L->up_blocks)[sn] = NULL;
    if (sn_size) {
      (L->sn_blocks)[sn] = (taucs_datatype*)
	taucs_calloc(sn_size*sn_size,sizeof(taucs_datatype));
      if (!((L->sn_blocks)[sn])) {
				taucs_printf("taucs_calloc (L->sn_blocks) %d\n",sn);
				return -1; /* the caller will free L */
			}
      (L->d_blocks)[sn] = (taucs_datatype*)
	taucs_calloc(2*sn_size,sizeof(taucs_datatype));
      if (!((L->d_blocks)[sn])) {
				taucs_printf("taucs_calloc (L->d_blocks) %d\n",sn);
				return -1; /* the caller will free L */
			}
      (L->db_size)[sn] = (int*)
	taucs_calloc((sn_size),sizeof(int));
      if (!((L->db_size)[sn])) {
				taucs_printf("taucs_calloc (L->db_size) %d\n",sn);
				return -1; /* the caller will free L */
			}
    }
    
    if ((sn_up_size - sn_size) && sn_size) {
      (L->up_blocks)[sn] = (taucs_datatype*)
	taucs_calloc((sn_up_size-sn_size)*sn_size,sizeof(taucs_datatype));
			if (!((L->up_blocks)[sn])) {
				taucs_printf("taucs_calloc (L->up_blocks) %d\n",sn);
				return -1; /* the caller will free L */
			}
    }
    
    /* put the max index */
    maxIndex = 0;
    for ( i=0; i<sn_size; i++ ) {
      maxIndex  = max(L->sn_struct[sn][i],maxIndex);
    }
    L->sn_max_ind[sn] = maxIndex;
  }
  
  for (child = L->first_child[sn]; child != -1; child = L->next_child[child]) {
    if (recursive_leftlooking_supernodal_factor_ldlt(child,
						     FALSE,
						     indmap,
						     bitmap,
						     A,L,sn,&rejected_from_child,handle)
	== -1 ) {
		taucs_printf("recursive_leftlooking_supernodal_factor_ldlt %d\n",child);
      taucs_free(dense_update_matrix);
      return -1;
    }
    
    if (!is_root) {
      if (!dense_update_matrix) {
	dense_update_matrix =	(taucs_datatype*)
	  taucs_calloc((L->sn_up_size[sn])*(L->sn_size[sn]),sizeof(taucs_datatype));
	if (!dense_update_matrix) return -1; /* caller will free L */
	dense_sn_size = L->sn_size[sn];
	dense_up_size = L->sn_up_size[sn];
      } else if ( dense_sn_size != L->sn_size[sn] ||
		  dense_up_size != L->sn_up_size[sn] ) {
	/* we need to resize the dense matrix */
	temp =  (taucs_datatype*)
	  taucs_calloc((L->sn_up_size)[sn]*(L->sn_size)[sn],sizeof(taucs_datatype));
	if (!temp) {
	  taucs_free(dense_update_matrix);
		taucs_printf("taucs_calloc temp %d\n",sn);
	  return -1; /* caller will free L */
	}
	for (i=0;i<dense_sn_size;i++)
	  for (j=0;j<dense_up_size;j++)
	    temp[i*(L->sn_up_size)[sn]+j] = dense_update_matrix[i*dense_up_size+j];
	
	taucs_free(dense_update_matrix);
	dense_update_matrix = temp;
      }
      
      
      if ( supernodal_ldlt_modify_parent(L,sn,rejected_from_child) ) {
				taucs_printf("supernodal_ldlt_modify_parent %d\n",sn);
	taucs_free(dense_update_matrix);
	return -1;
      }
      
      if ( recursive_leftlooking_supernodal_update_ldlt(sn,child,
							bitmap, TRUE, dense_update_matrix,
							A,L,rejected_from_child,&exist_upd,handle) ) {
	taucs_printf("recursive_leftlooking_supernodal_update_ldlt %d\n",sn);
	taucs_free(dense_update_matrix);
	return -1;
      }
      
      if ( supernodal_ldlt_modify_child(L,child,rejected_from_child) ) {
	taucs_printf("supernodal_ldlt_modify_child %d\n",child);
	taucs_free(dense_update_matrix);
	return -1;
      }
      
    }
  }
  taucs_free(dense_update_matrix);
  
  if(!is_root) {
    if (leftlooking_supernodal_front_factor_ldlt(sn,
						 indmap,A,L,
						 parent,p_rejected,handle)) {
			taucs_printf("leftlooking_supernodal_front_factor_ldlt %d\n",sn);
      return -1; /* nonpositive pivot */
    }
  }
  
  return 0;
}


/*************************************************************/
/* left-looking ooc factor routines                          */
/*************************************************************/

static void
ooc_sn_struct_handler(void* argument, 
		      int sn, int sn_up_size, int* sn_struct_ptr[])
{
  taucs_io_handle* handle = (taucs_io_handle*) argument;
  taucs_io_append(handle,
		  IO_BASE+sn, /* matrix written in postorder */
		  1,
		  sn_up_size,
		  TAUCS_INT,
		  *sn_struct_ptr);
  taucs_free(*sn_struct_ptr);
  *sn_struct_ptr = NULL;
}


static int
taucs_io_append_supernode(int sn,
			  taucs_io_handle* handle,
			  supernodal_factor_matrix_ooc_ldlt* L)
{
  int ret;
  int  sn_size = L->sn_size[sn];
  int	 sn_up_size = L->sn_up_size[sn];
  
  ret = taucs_io_append(handle,IO_BASE+2*(L->n_sn)+6*sn,
			sn_size,sn_size,
			TAUCS_CORE_DATATYPE,L->sn_blocks[sn]);
  
  if (ret) return ret; /*failure*/
  
  ret = taucs_io_append(handle,IO_BASE+2*(L->n_sn)+6*sn+1,
			sn_up_size - sn_size,sn_size,
			TAUCS_CORE_DATATYPE,L->up_blocks[sn]); 
  
  if (ret) return ret; /*failure*/
  
  ret = taucs_io_append(handle,IO_BASE+2*(L->n_sn)+6*sn+2,
			sn_size,2,
			TAUCS_CORE_DATATYPE,L->d_blocks[sn]); 
  
  if (ret) return ret; /*failure*/
  
  ret = taucs_io_append(handle,IO_BASE+2*(L->n_sn)+6*sn+3,
			sn_size,1,
			TAUCS_INT,L->db_size[sn]); 
  
  if (ret) return ret; /*failure*/
  
  ret = taucs_io_append(handle,IO_BASE+L->n_sn+sn,
			1,L->sn_up_size[sn],
			TAUCS_INT,L->sn_struct[sn]);
  
  return ret;
}

static int
taucs_io_read_supernode(int sn,
			taucs_io_handle* handle,
			supernodal_factor_matrix_ooc_ldlt* L)
{
  int ret;
  int  sn_size = L->sn_size[sn];
  int	 sn_up_size = L->sn_up_size[sn];
  
  ret = taucs_io_read_block(handle,IO_BASE+2*(L->n_sn)+6*sn,
			    sn_size,sn_size,
			    TAUCS_CORE_DATATYPE,L->sn_blocks[sn]);
  
  if (ret == -1) return ret; /*failure*/
  
  ret = taucs_io_read_block(handle,IO_BASE+2*(L->n_sn)+6*sn+1,
			    sn_up_size - sn_size,sn_size,
			    TAUCS_CORE_DATATYPE,L->up_blocks[sn]); 
  
  if (ret == -1) return ret; /*failure*/
  
  ret = taucs_io_read(handle,IO_BASE+2*(L->n_sn)+6*sn+2,
		      sn_size,2,
		      TAUCS_CORE_DATATYPE,L->d_blocks[sn]); 
  
  if (ret == -1) return ret; /*failure*/
  
  ret = taucs_io_read(handle,IO_BASE+2*(L->n_sn)+6*sn+3,
		      sn_size,1,
		      TAUCS_CORE_DATATYPE,L->db_size[sn]); 

  return ret;
}
														

static int
recursive_append_L(	int sn,   /* this supernode */
			int is_root,/* is v the root?*/
			taucs_io_handle* handle,
			supernodal_factor_matrix_ooc_ldlt* L)
{
  int  child;
  
  for (child = L->first_child[sn]; child != -1; child = L->next_child[child]) {
    if (recursive_append_L(child,FALSE,handle,L)) {
      /* failure */
      return -1;
    }
  }
  
  if (!is_root) { 
    taucs_io_append_supernode(sn,handle,L);
    ooc_supernodal_free(L,sn);
  }
  
  return 0;
}

static int
recursive_read_L(int sn,   /* this supernode */
		 int is_root,/* is v the root?*/
		 taucs_io_handle* handle,
		 supernodal_factor_matrix_ooc_ldlt* L)
{
  int  child;
  /*int  sn_size;*/
  
  for (child = L->first_child[sn]; child != -1; child = L->next_child[child]) {
    if (recursive_read_L(child,FALSE,handle,L)) {
      /* failure */
      return -1;
    }
  }
    
  if (!is_root) { 
    (L->sn_blocks)[sn] = (taucs_datatype*)taucs_calloc(((L->sn_size)[sn])
						       *((L->sn_size)[sn]),
						       sizeof(taucs_datatype));
    (L->up_blocks)[sn] = (taucs_datatype*)taucs_calloc(((L->sn_up_size)[sn]-(L->sn_size)[sn])
						       *((L->sn_size)[sn]),
						       sizeof(taucs_datatype));
    (L->d_blocks)[sn] = (taucs_datatype*)taucs_calloc(((L->sn_size)[sn]*2),
						      sizeof(taucs_datatype));
    (L->db_size)[sn] = (int*)taucs_calloc((L->sn_size)[sn],sizeof(int));
    
    taucs_io_read_supernode(sn,handle,L);
  }
  
  return 0;
}

static int
recursive_read_L_cols(int sn,   /* this supernode */
		      int is_root,/* is v the root?*/
		      taucs_io_handle* handle,
		      supernodal_factor_matrix_ooc_ldlt* L)
{
  int  child;
  int  sn_up_size = L->sn_up_size[sn];
  
  for (child = L->first_child[sn]; child != -1; child = L->next_child[child]) {
    if (recursive_read_L_cols(child,FALSE,handle,L)) {
      /* failure */
      return -1;
    }
  }
    
  if (!is_root) { 
    L->sn_struct[sn] = (int*)taucs_malloc(sn_up_size*sizeof(int));
    taucs_io_read(handle,IO_BASE+sn,1,sn_up_size,TAUCS_INT,L->sn_struct[sn]);
  }

  return 0;
}

static double
get_memory_needed_by_supernode( int sn_size, int sn_up_size )
{
	int multiplier = (gl_block_size > sn_size) ? sn_size : gl_block_size;
  double memory;
  memory = 
		(double)(sn_size*sn_up_size*sizeof(taucs_datatype)) + /*actual blocks size*/
		(double)(multiplier*sn_up_size*sizeof(taucs_datatype)) + /*working blocks size*/
    (double)(sn_up_size*sizeof(int)); /*struct size*/ 
  return memory;
}


static double
recursive_compute_supernodes_ll_in_core(int sn,       /* this supernode */
					int is_root,  /* is v the root? */
					double avail_mem,
					int* sn_in_core,
					supernodal_factor_matrix_ooc_ldlt* L)
{
  int  child;
  int  sn_size = L->sn_size[sn];
  int  sn_up_size = L->sn_up_size[sn];
  /*double curr_avail_mem = avail_mem;*/
  double child_mem = 0.0;
  double children_mem = 0.0;
  double total_mem = 0.0;
  
  /*new_panel = 0;*/
  for (child = L->first_child[sn]; child != -1; child = L->next_child[child]) {
    child_mem = recursive_compute_supernodes_ll_in_core(child,FALSE,avail_mem,sn_in_core,L);
    /*if (child_mem == 0) new_panel = 1; */
    children_mem += child_mem;
  }
  
  /*if (first_child[sn] == -1) 
    total_mem = (double)(L->sn_size)[sn]*(double)(L->sn_up_size)[sn]*sizeof(taucs_datatype)+(double)(L->sn_up_size)[sn]*sizeof(int);
    else
    total_mem = children_mem + 2*(double)(L->sn_size)[sn]*(double)(L->sn_up_size)[sn]*sizeof(taucs_datatype)+(double)(L->sn_up_size)[sn]*sizeof(int);
    
    if (total_mem <= avail_mem || first_child[sn] == -1) 
    sn_in_core[sn] = 1;
    else 
    sn_in_core[sn] = 0;
    return total_mem;
  */
  if (!is_root) {
    total_mem = children_mem + get_memory_needed_by_supernode(sn_size,sn_up_size);
  } else {
    total_mem = children_mem;
  }
  
  if ((total_mem + (double)sn_size*(double)sn_up_size*sizeof(taucs_datatype)) <= avail_mem || 
      L->first_child[sn] == -1) {
    sn_in_core[sn] = 1;
    return total_mem;
  } else {
    sn_in_core[sn] = 0;
    return (total_mem + (double)sn_size*(double)sn_up_size*sizeof(taucs_datatype));
  }
  
}



/******************** OOC SOLVE **************************/

static int
recursive_supernodal_solve_d_ooc(int sn,       /* this supernode */
				 int is_root,  /* is v the root? */
				 taucs_io_handle* handle,
				 int n_sn,
				 int* first_child, int* next_child,
				 int** sn_struct, int* sn_sizes, int* sn_up_sizes,
				 taucs_datatype x[], taucs_datatype b[],
				 taucs_datatype t[])
{
  int child;
  int  sn_size; /* number of rows/columns in the supernode    */
  int  up_size;
  taucs_datatype* d_block;
  int * db_size;
	int ret = 0;
  
  taucs_datatype *xdense;
  taucs_datatype D[4], y[2], z[2], L[4], U[4]; /* D*y=z */
  int i, need_switch = 0;
  
  for (child = first_child[sn]; child != -1; child = next_child[child]) {
    ret = recursive_supernodal_solve_d_ooc(child,
				     FALSE, handle, n_sn,
				     first_child,next_child,
				     sn_struct,sn_sizes,sn_up_sizes,
				     x,b,t);
		if (ret) return ret;
  }
  
  if(!is_root) {
    
    sn_size = sn_sizes[sn];
    up_size = sn_up_sizes[sn];
    
    sn_struct[sn] = (int*)taucs_malloc((sn_size+up_size)*sizeof(int));
    if ( sn_struct[sn] == NULL ) {
      return -1;
    }
    taucs_io_read(handle,IO_BASE+n_sn+sn,1,sn_size+up_size,TAUCS_INT,sn_struct[sn]);
    
    d_block = (taucs_datatype*)taucs_calloc(2*sn_size,sizeof(taucs_datatype));
    if ( d_block == NULL ) {
      return -1;
    }
    taucs_io_read(handle,IO_BASE+2*n_sn+6*sn+2,
		  sn_size, 2, TAUCS_CORE_DATATYPE,d_block);
    
    db_size = (int*)taucs_malloc(sn_size*sizeof(int));
    if ( db_size == NULL ) {
      return -1;
    }
    taucs_io_read(handle,IO_BASE+2*n_sn+6*sn+3,
		  sn_size, 1, TAUCS_INT,db_size);
    
    xdense = t;
    
    for (i=0; i<sn_size; i++)
      xdense[i] = b[ sn_struct[ sn ][ i ] ];
    
    for (i=0; i<sn_size; i++) {
      
      if ( db_size[i] == 1 ) {/* a 1*1 block x = b/Aii */
	
	y[0] = xdense[i];
	xdense[i] = taucs_div( xdense[i], d_block[i*2] );
	
	/* check that if the diagonal was inf, then y[0] was 0.
				if not - there isn't a solution.*/
				if (isinf(taucs_re(d_block[i*2])) &&
						!taucs_is_zero(y[0]) )
				 return -1;
      }
      
      else if ( db_size[i] == 2 ) {/* a 2*2 block */   
	D[0] = d_block[2*i];
	D[1] = D[2] = d_block[2*i+1];
	D[3] = d_block[2*i+2];
	
	z[0] = xdense[i];
	z[1] = xdense[i+1];
	
	need_switch = build_LU_from_D(D,L,U);
	
	two_times_two_diagonal_block_solve( D,y,z,gl_cramer_rule, FALSE,L,U,need_switch );
	
	xdense[i] = y[0];
	xdense[i+1] = y[1];
	
	i++;
      }
      else {/* shouldn't get here */
	taucs_printf("db_size[%d][%d] = %d\n",sn,i,db_size[i] );
	assert(0);
      }
    }
    
    for (i=0; i<sn_size; i++)
      x[ sn_struct[ sn][ i ] ]  = xdense[i];
    
    taucs_free(sn_struct[sn]);
    sn_struct[sn] = NULL;
    taucs_free(d_block);
    taucs_free(db_size);
  }
	return ret;
}


static int 
recursive_supernodal_solve_l_ooc(int sn,       /* this supernode */
				 int is_root,  /* is v the root? */
				 taucs_io_handle* handle,
				 int n_sn,
				 int* first_child, int* next_child,
				 int** sn_struct, int* sn_sizes, int* sn_up_sizes,
				 taucs_datatype x[], taucs_datatype b[],
				 taucs_datatype t[])
{
  int child;
  int  sn_size; /* number of rows/columns in the supernode    */
  int  up_size; /* number of rows that this supernode updates */
  int    ione = 1;
  taucs_datatype* xdense = NULL;
  taucs_datatype* bdense = NULL;
  double  flops;
  int i,j,ip,jp;
  taucs_datatype* sn_block = NULL;
  taucs_datatype* up_block = NULL;
	int ret = 0;
  
  for (child = first_child[sn]; child != -1; child = next_child[child]) {
    ret = recursive_supernodal_solve_l_ooc(child,
				     FALSE,
				     handle,
				     n_sn,
				     first_child,next_child,
				     sn_struct,sn_sizes,sn_up_sizes,
				     x,b,t);
		if (ret) return ret;
  }

  if(!is_root) {
    
    sn_size = sn_sizes[sn];
    up_size = sn_up_sizes[sn] - sn_sizes[sn];
    
    sn_struct[sn] = (int*)taucs_malloc((sn_size+up_size)*sizeof(int));
    if ( !sn_struct[sn] ) {
      return -1;
    }
    taucs_io_read(handle,IO_BASE+n_sn+sn,1,sn_size+up_size,TAUCS_INT,sn_struct[sn]);
    
    sn_block = (taucs_datatype*)taucs_calloc(sn_size*sn_size,sizeof(taucs_datatype));
    /* We use taucs_io_read_block, that tries to find the block
       in the second writing, and if fails, gets the first one.*/
    if ( !sn_block ) {
      return -1;
    }
    taucs_io_read_block(handle,IO_BASE+2*n_sn+6*sn,
			sn_size, sn_size, TAUCS_CORE_DATATYPE,sn_block);
    
    if (up_size > 0 && sn_size > 0) {
      up_block = (taucs_datatype*)taucs_calloc(up_size*sn_size,sizeof(taucs_datatype));
      if ( !up_block ) {
	return -1;
      }
      taucs_io_read_block(handle,IO_BASE+2*n_sn+6*sn+1,
			  up_size, sn_size , TAUCS_CORE_DATATYPE,up_block);
    }

       
    flops = (double)(sn_size*sn_size + 2*sn_size*up_size);

     
    if (flops > BLAS_FLOPS_CUTOFF) {
      xdense = t;
      bdense = t + sn_size;
      
      for (i=0; i<sn_size; i++)
	xdense[i] = b[ sn_struct[ sn ][ i ] ];
      for (i=0; i<up_size; i++)
	bdense[i] = taucs_zero;

      
      if ( sn_size != 0 )       
	taucs_trsm ("Left",
		    "Lower",
		    "No Conjugate",
		    "No unit diagonal",
		    &sn_size,&ione,
		    &taucs_one_const,
		    sn_block,&sn_size,
		    xdense , &sn_size);

      
      if (up_size > 0 && sn_size > 0) 
	taucs_gemm ("No Conjugate","No Conjugate",
		    &up_size, &ione, &sn_size,
		    &taucs_one_const,
		    up_block,&up_size,
		    xdense       ,&sn_size,
		    &taucs_zero_const,
		    bdense       ,&up_size);

           
      for (i=0; i<sn_size; i++)
	x[ sn_struct[ sn][ i ] ]  = xdense[i];
      for (i=0; i<up_size; i++)
	b[ sn_struct[ sn ][ sn_size + i ] ] =
	  taucs_sub(b[ sn_struct[ sn ][ sn_size + i ] ] , bdense[i]);

      
    } else if (sn_size > SOLVE_DENSE_CUTOFF) {
      
      xdense = t;
      bdense = t + sn_size;
      
      for (i=0; i<sn_size; i++)
	xdense[i] = b[ sn_struct[ sn ][ i ] ];
      for (i=0; i<up_size; i++)
	bdense[i] = taucs_zero;
      
      for (jp=0; jp<sn_size; jp++) {
	xdense[jp] = taucs_div(xdense[jp] , sn_block[ sn_size*jp + jp]);
	
	for (ip=jp+1; ip<sn_size; ip++) {
	  xdense[ip] = taucs_sub(xdense[i],
				 taucs_mul(xdense[jp] , sn_block[ sn_size*jp + ip]));
	}
      }
      
      for (jp=0; jp<sn_size; jp++) {
	for (ip=0; ip<up_size; ip++) {
	  bdense[ip] = taucs_add(bdense[ip],
				 taucs_mul(xdense[jp] , up_block[ up_size*jp + ip]));
	}
      }
      
      for (i=0; i<sn_size; i++)
	x[ sn_struct[ sn][ i ] ]  = xdense[i];
      for (i=0; i<up_size; i++)
	b[ sn_struct[ sn ][ sn_size + i ] ] =
	  taucs_sub(b[ sn_struct[ sn ][ sn_size + i ] ] , bdense[i]);
      
    } else {
      for (jp=0; jp<sn_size; jp++) {
	j = sn_struct[sn][jp];
	x[j] = taucs_div(b[j] , sn_block[ sn_size*jp + jp]);
	for (ip=jp+1; ip<sn_size; ip++) {
	  i = sn_struct[sn][ip];
	  b[i] = taucs_sub(b[i],
			   taucs_mul(x[j] , sn_block[ sn_size*jp + ip]));
	}
	
	for (ip=0; ip<up_size; ip++) {
	  i = sn_struct[sn][sn_size + ip];
	  b[i] = taucs_sub(b[i],
			   taucs_mul(x[j] , up_block[ up_size*jp + ip]));
	}
      }
    }
    taucs_free(sn_struct[sn]);
    sn_struct[sn] = NULL;
    taucs_free(sn_block);
    if (up_size > 0 && sn_size > 0) taucs_free(up_block);
    
    sn_block = NULL;
    up_block = NULL;
  }

	return 0;
}

static int 
recursive_supernodal_solve_lt_ooc(int sn,       /* this supernode */
				  int is_root,  /* is v the root? */
				  taucs_io_handle* handle,
				  int n_sn,
				  int* first_child, int* next_child,
				  int** sn_struct, int* sn_sizes, int* sn_up_sizes,
				  taucs_datatype x[], taucs_datatype b[],
				  taucs_datatype t[])
{
  int child;
  int  sn_size; /* number of rows/columns in the supernode    */
  int  up_size; /* number of rows that this supernode updates */
  int    ione = 1;
  taucs_datatype* xdense = NULL;
  taucs_datatype* bdense = NULL;
  double  flops;
  int i,j,ip,jp;
  taucs_datatype* sn_block = NULL;
  taucs_datatype* up_block = NULL;
	int ret = 0;

  if(!is_root) {
    
    sn_size = sn_sizes[sn];
    up_size = sn_up_sizes[sn]-sn_sizes[sn];
    
    sn_struct[sn] = (int*)taucs_malloc((sn_size+up_size)*sizeof(int));
    if ( sn_struct[sn] == NULL ) {
      return -1;
    }
    taucs_io_read(handle,IO_BASE+n_sn+sn,1,sn_size+up_size,TAUCS_INT,sn_struct[sn]);
    
    sn_block = (taucs_datatype*)taucs_calloc(sn_size*sn_size,sizeof(taucs_datatype));
    if ( sn_block == NULL ) {
      return -1;
    }
    taucs_io_read_block(handle,IO_BASE+2*n_sn+6*sn,
			sn_size, sn_size , TAUCS_CORE_DATATYPE,sn_block);
    
    if (up_size > 0 && sn_size > 0){
      up_block = (taucs_datatype*)taucs_calloc(up_size*sn_size,sizeof(taucs_datatype));
      if ( up_block == NULL ) {
	return -1;
      }
      taucs_io_read_block(handle,IO_BASE+2*n_sn+6*sn+1,
			  up_size, sn_size , TAUCS_CORE_DATATYPE,up_block);
    }
    
    flops = ((double)sn_size)*((double)sn_size) 
      + 2.0*((double)sn_size)*((double)up_size);
    
    if (flops > BLAS_FLOPS_CUTOFF) {
      bdense = t;
      xdense = t + sn_size;
      
      for (i=0; i<sn_size; i++)
	bdense[i] = b[ sn_struct[ sn][ i ] ];
      for (i=0; i<up_size; i++)
	xdense[i] = x[ sn_struct[sn][sn_size+i] ];
      
      if (up_size > 0 && sn_size > 0)
	taucs_gemm ("Conjugate","No Conjugate",
		    &sn_size, &ione, &up_size,
		    &taucs_minusone_const,
		    up_block,&up_size,
		    xdense       ,&up_size,
		    &taucs_one_const,
		    bdense       ,&sn_size);
      
      if ( sn_size != 0 )       
	taucs_trsm ("Left",
		    "Lower",
		    "Conjugate",
		    "No unit diagonal",
		    &sn_size,&ione,
		    &taucs_one_const,
		    sn_block,&sn_size,
		    bdense       ,&sn_size);
      
      for (i=0; i<sn_size; i++)
	x[ sn_struct[ sn][ i ] ]  = bdense[i];
      
    } else if (sn_size > SOLVE_DENSE_CUTOFF) {
      bdense = t;
      xdense = t + sn_size;
      
      for (i=0; i<sn_size; i++)
	bdense[i] = b[ sn_struct[ sn][ i ] ];
      
      for (i=0; i<up_size; i++)
	xdense[i] = x[ sn_struct[sn][sn_size+i] ];
      
      for (ip=sn_size-1; ip>=0; ip--) {
	for (jp=0; jp<up_size; jp++) {
	  bdense[ip] = taucs_sub(bdense[ip],
				 taucs_mul(xdense[jp] , up_block[ up_size*ip + jp]));
	}
      }
      
      for (ip=sn_size-1; ip>=0; ip--) {
	for (jp=sn_size-1; jp>ip; jp--) {
	  bdense[ip] = taucs_sub(bdense[ip],
				 taucs_mul(bdense[jp] , sn_block[ sn_size*ip + jp]));
	}
	bdense[ip] = taucs_div(bdense[ip] , sn_block[ sn_size*ip + ip]);
      }
      
      for (i=0; i<sn_size; i++)
	x[ sn_struct[ sn][ i ] ]  = bdense[i];
      
    } else {
      for (ip=sn_size-1; ip>=0; ip--) {
	i = sn_struct[sn][ip];
	
	for (jp=0; jp<up_size; jp++) {
	  j = sn_struct[sn][sn_size + jp];
	  b[i] = taucs_sub(b[i],
			   taucs_mul(x[j] , up_block[ up_size*ip + jp]));
	}
	
	for (jp=sn_size-1; jp>ip; jp--) {
	  j = sn_struct[sn][jp];
	  b[i] = taucs_sub(b[i],
			   taucs_mul(x[j] , sn_block[ sn_size*ip + jp]));
	}
	x[i] = taucs_div(b[i] , sn_block[ sn_size*ip + ip]);
      }
      
    }
    taucs_free(sn_struct[sn]);
    sn_struct[sn] = NULL;
    taucs_free(sn_block);
    if (up_size > 0 && sn_size > 0) taucs_free(up_block);
    
    sn_block = NULL;
    up_block = NULL;
  }

  for (child = first_child[sn]; child != -1; child = next_child[child]) {
    ret = recursive_supernodal_solve_lt_ooc(child,
				      FALSE,
				      handle,
				      n_sn,
				      first_child,next_child,
				      sn_struct,sn_sizes,sn_up_sizes,
				      x,b,t);
		if (ret) return ret;
  }
	return ret;
}


int taucs_dtl(ooc_solve_ldlt_many) 	(void* vL ,
					 int n,void* X, int ld_X, void* B, int ld_B)
{
  taucs_io_handle* handle = (taucs_io_handle*) vL;
  supernodal_factor_matrix_ooc_ldlt* L;
  
  taucs_datatype* y;
  taucs_datatype* t; /* temporary vector */
  int   j,i, ret = 0;

  L = multifrontal_supernodal_create();
  if ( !L ) {
    taucs_printf("supernodal_solve_ldlt: out of memory\n");
    return -1;
  }
  
  taucs_io_read(handle,5,1,1,TAUCS_INT,&(L->n));
  taucs_io_read(handle,0,1,1,TAUCS_INT,&(L->n_sn));
  L->sn_struct = (int**)taucs_malloc((L->n_sn  )*sizeof(int*));
  L->sn_blocks = (taucs_datatype**)taucs_malloc((L->n_sn  )*sizeof(taucs_datatype*));
  L->up_blocks = (taucs_datatype**)taucs_malloc((L->n_sn  )*sizeof(taucs_datatype*));
  L->d_blocks = (taucs_datatype**)taucs_malloc((L->n_sn  )*sizeof(taucs_datatype*));
  L->db_size = (int**)taucs_malloc((L->n_sn  )*sizeof(int*));
  L->sn_size   = (int*) taucs_malloc((L->n_sn+1)*sizeof(int));
  L->sn_up_size   = (int*) taucs_malloc((L->n_sn+1)*sizeof(int));
  L->first_child = (int*) taucs_malloc((L->n_sn+1)*sizeof(int));
  L->next_child  = (int*) taucs_malloc((L->n_sn+1)*sizeof(int));
  
  if ( !L->sn_struct || !L->sn_blocks || !L->up_blocks || !L->d_blocks ||
       !L->db_size || !L->sn_size || !L->sn_up_size || !L->first_child ||
       !L->next_child ) {
    taucs_printf("supernodal_solve_ldlt: out of memory\n");
    return -1;
  }

  taucs_io_read(handle,1,1,L->n_sn+1,TAUCS_INT,L->first_child);
  taucs_io_read(handle,2,1,L->n_sn+1,TAUCS_INT,L->next_child);
  taucs_io_read(handle,7,1,L->n_sn,TAUCS_INT,L->sn_size);
  taucs_io_read(handle,8,1,L->n_sn,TAUCS_INT,L->sn_up_size);
  /*  for (i=0; i<L->n_sn; i++) {
      L->sn_struct[i] = (int*)taucs_malloc(L->sn_up_size[i]*sizeof(int));
      taucs_io_read(handle,IO_BASE+i,1,L->sn_up_size[i],TAUCS_INT,L->sn_struct[i]);
      }*/
 

  for(i=0;i<L->n_sn;i++){
    L->sn_struct[i] = NULL;
    L->sn_blocks[i] = NULL;
    L->up_blocks[i] = NULL;
    L->d_blocks[i] = NULL;
    L->db_size[i] = NULL;
  }
  
  y = (taucs_datatype*) taucs_malloc((L->n) * sizeof(taucs_datatype));
  t = (taucs_datatype*) taucs_malloc((L->n) * sizeof(taucs_datatype));
  if (!y || !t) {
    taucs_free(y);
    taucs_free(t);
    taucs_printf("supernodal_solve_ldlt: out of memory\n");
    return -1;
  }

  for (j=0; j<n; j++) {
    
    taucs_datatype* x = (taucs_datatype*) ((taucs_datatype*)X+j*ld_X);
    taucs_datatype* b = (taucs_datatype*) ((taucs_datatype*)B+j*ld_B);
   
    
    /* before we start we need:
       1. put b into y (for the l solve) */
    for (i=0; i<L->n; i++) {
      y[i] = b[i];
    }
    
    /*for(i=0;i<L->n;i++)
      taucs_printf("%.4e ",y[i]);
      taucs_printf("\n");*/
    
    ret = recursive_supernodal_solve_l_ooc (L->n_sn,
					    TRUE,  /* this is the root */
					    handle,
					    L->n_sn,
					    L->first_child, L->next_child,
					    L->sn_struct,L->sn_size,L->sn_up_size,
					    x, y, t);
    
    if ( ret ) {
      taucs_free(y);
      taucs_free(t);
      taucs_printf("supernodal_solve_ldlt: No solution found.\n");
      return -1;
    }
    
    /*for(i=0;i<L->n;i++)
      taucs_printf("%.4e ",x[i]);
      taucs_printf("\n");*/
    
    /* solve the Diagonal block */
    ret = recursive_supernodal_solve_d_ooc(L->n_sn,
					   TRUE,
					   handle,
					   L->n_sn,
					   L->first_child, L->next_child,
					   L->sn_struct,L->sn_size,L->sn_up_size,
					   y, x, t);
    
    if ( ret ) {
      taucs_free(y);
      taucs_free(t);
      taucs_printf("supernodal_solve_ldlt: No solution found.\n");
      return -1;
    }
    
    /*for(i=0;i<L->n;i++)
      taucs_printf("%.4e ",y[i]);
      taucs_printf("\n");*/
    
    ret = recursive_supernodal_solve_lt_ooc(L->n_sn,
					    TRUE,  /* this is the root */
					    handle,
					    L->n_sn,
					    L->first_child, L->next_child,
					    L->sn_struct,L->sn_size,L->sn_up_size,
					    x, y, t);
    
    if ( ret ) {
      taucs_free(y);
      taucs_free(t);
      taucs_printf("supernodal_solve_ldlt: No solution found.\n");
      return -1;
    }
    
    /*for(i=0;i<L->n;i++)
      taucs_printf("%.4e ",x[i]);
      taucs_printf("\n");*/
    
    /* TEST 
       writeFactorFacts(L,"oocc");
       M = taucs_dsupernodal_factor_ldlt_ooc_to_ccs((void*)L);
       taucs_ccs_write_ijv(M,"C:/taucs/Results/Looc.ijv");
       M = taucs_dsupernodal_factor_diagonal_ooc_to_ccs((void*)L);
       taucs_ccs_write_ijv(M,"C:/taucs/Results/Dooc.ijv");*/
  }
  
  taucs_free(y);
  taucs_free(t);
  ooc_supernodal_factor_free(L);
  return 0;
}

/****** new solve many implementation ******/
static int
recursive_supernodal_solve_d_ooc2(int sn,       /* this supernode */
				 int is_root,  /* is v the root? */
				 taucs_io_handle* handle,
				 int n_sn,
				 int* first_child, int* next_child,
				 int** sn_struct, int* sn_sizes, int* sn_up_sizes,
				 taucs_datatype X[], taucs_datatype B[],
				 taucs_datatype T[], int ld_X, int n)
{
  int child;
  int  sn_size; /* number of rows/columns in the supernode    */
  int  up_size;
  taucs_datatype* d_block;
  int * db_size;
	int ret = 0;
	int loop1;
	taucs_datatype* x;
	taucs_datatype* b;
	taucs_datatype* t;	
  
  taucs_datatype *xdense;
  taucs_datatype D[4], y[2], z[2], L[4], U[4]; /* D*y=z */
  int i, need_switch = 0;
  
  for (child = first_child[sn]; child != -1; child = next_child[child]) {
    ret = recursive_supernodal_solve_d_ooc2(child,
				     FALSE, handle, n_sn,
				     first_child,next_child,
				     sn_struct,sn_sizes,sn_up_sizes,
				     X,B,T,ld_X,n);
		if (ret) return ret;
  }
  
  if(!is_root) {
    
    sn_size = sn_sizes[sn];
    up_size = sn_up_sizes[sn];
    
    sn_struct[sn] = (int*)taucs_malloc((sn_size+up_size)*sizeof(int));
    if ( sn_struct[sn] == NULL ) {
      return -1;
    }
    taucs_io_read(handle,IO_BASE+n_sn+sn,1,sn_size+up_size,TAUCS_INT,sn_struct[sn]);
    
    d_block = (taucs_datatype*)taucs_calloc(2*sn_size,sizeof(taucs_datatype));
    if ( d_block == NULL ) {
      return -1;
    }
    taucs_io_read(handle,IO_BASE+2*n_sn+6*sn+2,
		  sn_size, 2, TAUCS_CORE_DATATYPE,d_block);
    
    db_size = (int*)taucs_malloc(sn_size*sizeof(int));
    if ( db_size == NULL ) {
      return -1;
    }
    taucs_io_read(handle,IO_BASE+2*n_sn+6*sn+3,
		  sn_size, 1, TAUCS_INT,db_size);

		for ( loop1 = 0; loop1<n; loop1++ ) {
				
			x = X+(loop1*ld_X);
			b = B+(loop1*ld_X);
			t = T+(loop1*ld_X);
			
			xdense = t;
    
			for (i=0; i<sn_size; i++)
	      xdense[i] = b[ sn_struct[ sn ][ i ] ];
    
		  for (i=0; i<sn_size; i++) {
      
			  if ( db_size[i] == 1 ) {/* a 1*1 block x = b/Aii */
	
					y[0] = xdense[i];
					xdense[i] = taucs_div( xdense[i], d_block[i*2] );
	
				/* check that if the diagonal was inf, then y[0] was 0.
					if not - there isn't a solution.*/
					if (isinf(taucs_re(d_block[i*2])) &&
								!taucs_is_zero(y[0]) )
						return -1;
					}
      
					else if ( db_size[i] == 2 ) {/* a 2*2 block */   
						D[0] = d_block[2*i];
						D[1] = D[2] = d_block[2*i+1];
						D[3] = d_block[2*i+2];
	
						z[0] = xdense[i];
						z[1] = xdense[i+1];
	
						need_switch = build_LU_from_D(D,L,U);
	
						two_times_two_diagonal_block_solve( D,y,z,gl_cramer_rule, FALSE,L,U,need_switch );
	
						xdense[i] = y[0];
						xdense[i+1] = y[1];
	
						i++;
					}	else {/* shouldn't get here */
						taucs_printf("db_size[%d][%d] = %d\n",sn,i,db_size[i] );
						assert(0);
					}
				}
    
				for (i=0; i<sn_size; i++)
					x[ sn_struct[ sn][ i ] ]  = xdense[i];
		}
    
    taucs_free(sn_struct[sn]);
    sn_struct[sn] = NULL;
    taucs_free(d_block);
    taucs_free(db_size);
  }
	return ret;
}


static int 
recursive_supernodal_solve_l_ooc2(int sn,       /* this supernode */
				 int is_root,  /* is v the root? */
				 taucs_io_handle* handle,
				 int n_sn,
				 int* first_child, int* next_child,
				 int** sn_struct, int* sn_sizes, int* sn_up_sizes,
				 taucs_datatype X[], taucs_datatype B[],
				 taucs_datatype T[], int ld_X, int n)
{
  int child;
  int  sn_size; /* number of rows/columns in the supernode    */
  int  up_size; /* number of rows that this supernode updates */
  int    ione = 1;
  taucs_datatype* xdense = NULL;
  taucs_datatype* bdense = NULL;
  double  flops;
  int i,j,ip,jp,loop1;
  taucs_datatype* sn_block = NULL;
  taucs_datatype* up_block = NULL;
	int ret = 0;
	taucs_datatype* x;
	taucs_datatype* b;
	taucs_datatype* t;
  
  for (child = first_child[sn]; child != -1; child = next_child[child]) {
    ret = recursive_supernodal_solve_l_ooc2(child,
				     FALSE,
				     handle,
				     n_sn,
				     first_child,next_child,
				     sn_struct,sn_sizes,sn_up_sizes,
				     X,B,T,ld_X,n);
		if (ret) return ret;
  }

  if(!is_root) {
    
    sn_size = sn_sizes[sn];
    up_size = sn_up_sizes[sn] - sn_sizes[sn];
    
    sn_struct[sn] = (int*)taucs_malloc((sn_size+up_size)*sizeof(int));
    if ( !sn_struct[sn] ) {
      return -1;
    }
    taucs_io_read(handle,IO_BASE+n_sn+sn,1,sn_size+up_size,TAUCS_INT,sn_struct[sn]);
    
    sn_block = (taucs_datatype*)taucs_calloc(sn_size*sn_size,sizeof(taucs_datatype));
    /* We use taucs_io_read_block, that tries to find the block
       in the second writing, and if fails, gets the first one.*/
    if ( !sn_block ) {
      return -1;
    }
    taucs_io_read_block(handle,IO_BASE+2*n_sn+6*sn,
			sn_size, sn_size, TAUCS_CORE_DATATYPE,sn_block);
    
    if (up_size > 0 && sn_size > 0) {
      up_block = (taucs_datatype*)taucs_calloc(up_size*sn_size,sizeof(taucs_datatype));
      if ( !up_block ) {
	return -1;
      }
      taucs_io_read_block(handle,IO_BASE+2*n_sn+6*sn+1,
			  up_size, sn_size , TAUCS_CORE_DATATYPE,up_block);
    }

       
    flops = (double)(sn_size*sn_size + 2*sn_size*up_size);

		for ( loop1 = 0; loop1<n; loop1++ ) {
				
			x = X+(loop1*ld_X);
			b = B+(loop1*ld_X);
			t = T+(loop1*ld_X);
			
			if (flops > BLAS_FLOPS_CUTOFF) {
				xdense = t;
				bdense = t + sn_size;
     
				for (i=0; i<sn_size; i++)
					xdense[i] = b[ sn_struct[ sn ][ i ] ];
				for (i=0; i<up_size; i++)
					bdense[i] = taucs_zero;

      
			if ( sn_size != 0 )       
				taucs_trsm ("Left",
					"Lower",
					"No Conjugate",
					"No unit diagonal",
					&sn_size,&ione,
					&taucs_one_const,
					sn_block,&sn_size,
					xdense , &sn_size);

      
			if (up_size > 0 && sn_size > 0) 
				taucs_gemm ("No Conjugate","No Conjugate",
					&up_size, &ione, &sn_size,
					&taucs_one_const,
					up_block,&up_size,
					xdense       ,&sn_size,
					&taucs_zero_const,
					bdense       ,&up_size);

          
			for (i=0; i<sn_size; i++)
				x[ sn_struct[ sn][ i ] ]  = xdense[i];
			for (i=0; i<up_size; i++)
				b[ sn_struct[ sn ][ sn_size + i ] ] =
					taucs_sub(b[ sn_struct[ sn ][ sn_size + i ] ] , bdense[i]);
      
			} else if (sn_size > SOLVE_DENSE_CUTOFF) {
     
				xdense = t;
				bdense = t + sn_size;
     
	      for (i=0; i<sn_size; i++)
					xdense[i] = b[ sn_struct[ sn ][ i ] ];
				for (i=0; i<up_size; i++)
					bdense[i] = taucs_zero;
     
				for (jp=0; jp<sn_size; jp++) {
					xdense[jp] = taucs_div(xdense[jp] , sn_block[ sn_size*jp + jp]);
	
					for (ip=jp+1; ip<sn_size; ip++) {
						xdense[ip] = taucs_sub(xdense[i],
							taucs_mul(xdense[jp] , sn_block[ sn_size*jp + ip]));
					}
				}
      
				for (jp=0; jp<sn_size; jp++) {
					for (ip=0; ip<up_size; ip++) {
						bdense[ip] = taucs_add(bdense[ip],
							taucs_mul(xdense[jp] , up_block[ up_size*jp + ip]));
					}
				}
      
	      for (i=0; i<sn_size; i++)
					x[ sn_struct[ sn][ i ] ]  = xdense[i];
	      for (i=0; i<up_size; i++)
					b[ sn_struct[ sn ][ sn_size + i ] ] =
						taucs_sub(b[ sn_struct[ sn ][ sn_size + i ] ] , bdense[i]);
      
			} else {
				for (jp=0; jp<sn_size; jp++) {
					j = sn_struct[sn][jp];
					x[j] = taucs_div(b[j] , sn_block[ sn_size*jp + jp]);
					for (ip=jp+1; ip<sn_size; ip++) {
						i = sn_struct[sn][ip];
						b[i] = taucs_sub(b[i],
							taucs_mul(x[j] , sn_block[ sn_size*jp + ip]));
					}
		
					for (ip=0; ip<up_size; ip++) {
						i = sn_struct[sn][sn_size + ip];
						b[i] = taucs_sub(b[i],
							taucs_mul(x[j] , up_block[ up_size*jp + ip]));
					}
				}
			}
		}
	
  taucs_free(sn_struct[sn]);
  sn_struct[sn] = NULL;
  taucs_free(sn_block);
  if (up_size > 0 && sn_size > 0) taucs_free(up_block);
    
    sn_block = NULL;
    up_block = NULL;
  }

	return 0;
}

static int 
recursive_supernodal_solve_lt_ooc2(int sn,       /* this supernode */
				  int is_root,  /* is v the root? */
				  taucs_io_handle* handle,
				  int n_sn,
				  int* first_child, int* next_child,
				  int** sn_struct, int* sn_sizes, int* sn_up_sizes,
				  taucs_datatype X[], taucs_datatype B[],
				  taucs_datatype T[], int ld_X, int n)
{
  int child;
  int  sn_size; /* number of rows/columns in the supernode    */
  int  up_size; /* number of rows that this supernode updates */
  int    ione = 1;
  taucs_datatype* xdense = NULL;
  taucs_datatype* bdense = NULL;
  double  flops;
  int i,j,ip,jp,loop1;
  taucs_datatype* sn_block = NULL;
  taucs_datatype* up_block = NULL;
	int ret = 0;
	taucs_datatype* x;
	taucs_datatype* b;
	taucs_datatype* t;

  if(!is_root) {
    
    sn_size = sn_sizes[sn];
    up_size = sn_up_sizes[sn]-sn_sizes[sn];
    
    sn_struct[sn] = (int*)taucs_malloc((sn_size+up_size)*sizeof(int));
    if ( sn_struct[sn] == NULL ) {
      return -1;
    }
    taucs_io_read(handle,IO_BASE+n_sn+sn,1,sn_size+up_size,TAUCS_INT,sn_struct[sn]);
    
    sn_block = (taucs_datatype*)taucs_calloc(sn_size*sn_size,sizeof(taucs_datatype));
    if ( sn_block == NULL ) {
      return -1;
    }
    taucs_io_read_block(handle,IO_BASE+2*n_sn+6*sn,
			sn_size, sn_size , TAUCS_CORE_DATATYPE,sn_block);
    
    if (up_size > 0 && sn_size > 0){
      up_block = (taucs_datatype*)taucs_calloc(up_size*sn_size,sizeof(taucs_datatype));
      if ( up_block == NULL ) {
				return -1;
			}	
			taucs_io_read_block(handle,IO_BASE+2*n_sn+6*sn+1,
					up_size, sn_size , TAUCS_CORE_DATATYPE,up_block);
		}
    
    flops = ((double)sn_size)*((double)sn_size) 
      + 2.0*((double)sn_size)*((double)up_size);

		for ( loop1 = 0; loop1<n; loop1++ ) {
				
			x = X+(loop1*ld_X);
			b = B+(loop1*ld_X);
			t = T+(loop1*ld_X);
			
			if (flops > BLAS_FLOPS_CUTOFF) {
				bdense = t;
				xdense = t + sn_size;
		      
				for (i=0; i<sn_size; i++)
					bdense[i] = b[ sn_struct[ sn][ i ] ];
				for (i=0; i<up_size; i++)
					xdense[i] = x[ sn_struct[sn][sn_size+i] ];
		      
				if (up_size > 0 && sn_size > 0)
					taucs_gemm ("Conjugate","No Conjugate",
						&sn_size, &ione, &up_size,
						&taucs_minusone_const,
						up_block,&up_size,
						xdense       ,&up_size,
						&taucs_one_const,
						bdense       ,&sn_size);
		     
				if ( sn_size != 0 )       
					taucs_trsm ("Left",
						"Lower",
						"Conjugate",
						"No unit diagonal",
						&sn_size,&ione,
						&taucs_one_const,
						sn_block,&sn_size,
						bdense       ,&sn_size);
		     
				for (i=0; i<sn_size; i++)
					x[ sn_struct[ sn][ i ] ]  = bdense[i];
		     
			} else if (sn_size > SOLVE_DENSE_CUTOFF) {
				bdense = t;
				xdense = t + sn_size;
		     
				for (i=0; i<sn_size; i++)
					bdense[i] = b[ sn_struct[ sn][ i ] ];
		     
				for (i=0; i<up_size; i++)
					xdense[i] = x[ sn_struct[sn][sn_size+i] ];
		     
				for (ip=sn_size-1; ip>=0; ip--) {
					for (jp=0; jp<up_size; jp++) {
						bdense[ip] = taucs_sub(bdense[ip],
							taucs_mul(xdense[jp] , up_block[ up_size*ip + jp]));
					}
				}
		     
				for (ip=sn_size-1; ip>=0; ip--) {
					for (jp=sn_size-1; jp>ip; jp--) {
						bdense[ip] = taucs_sub(bdense[ip],
							taucs_mul(bdense[jp] , sn_block[ sn_size*ip + jp]));
					}
					bdense[ip] = taucs_div(bdense[ip] , sn_block[ sn_size*ip + ip]);
				}
		     
				for (i=0; i<sn_size; i++)
					x[ sn_struct[ sn][ i ] ]  = bdense[i];
		     
			} else {
				for (ip=sn_size-1; ip>=0; ip--) {
					i = sn_struct[sn][ip];
					for (jp=0; jp<up_size; jp++) {
						j = sn_struct[sn][sn_size + jp];
						b[i] = taucs_sub(b[i],
							taucs_mul(x[j] , up_block[ up_size*ip + jp]));
					}
					for (jp=sn_size-1; jp>ip; jp--) {
						j = sn_struct[sn][jp];
						b[i] = taucs_sub(b[i],
								taucs_mul(x[j] , sn_block[ sn_size*ip + jp]));
					}
					x[i] = taucs_div(b[i] , sn_block[ sn_size*ip + ip]);
				}
			}
		}
		  
    taucs_free(sn_struct[sn]);
    sn_struct[sn] = NULL;
    taucs_free(sn_block);
    if (up_size > 0 && sn_size > 0) taucs_free(up_block);
    
    sn_block = NULL;
    up_block = NULL;
  }

  for (child = first_child[sn]; child != -1; child = next_child[child]) {
    ret = recursive_supernodal_solve_lt_ooc2(child,
				      FALSE,
				      handle,
				      n_sn,
				      first_child,next_child,
				      sn_struct,sn_sizes,sn_up_sizes,
				      X,B,T,ld_X,n);
		if (ret) return ret;
  }
	return ret;
}


int taucs_dtl(ooc_solve_ldlt_many2) 	(void* vL ,
					 int n,void* vX, int ld_X, void* vB, int ld_B)
{
  taucs_io_handle* handle = (taucs_io_handle*) vL;
  supernodal_factor_matrix_ooc_ldlt* L;
  
	taucs_datatype* X = (taucs_datatype*)vX;
	taucs_datatype* B = (taucs_datatype*)vB;
  taucs_datatype* Y;
  taucs_datatype* T; /* temporary vector */
  int   j,i, ret = 0;

  L = multifrontal_supernodal_create();
  if ( !L ) {
    taucs_printf("supernodal_solve_ldlt2: out of memory\n");
    return -1;
  }
  
  taucs_io_read(handle,5,1,1,TAUCS_INT,&(L->n));
  taucs_io_read(handle,0,1,1,TAUCS_INT,&(L->n_sn));
  L->sn_struct = (int**)taucs_malloc((L->n_sn  )*sizeof(int*));
  L->sn_blocks = (taucs_datatype**)taucs_malloc((L->n_sn  )*sizeof(taucs_datatype*));
  L->up_blocks = (taucs_datatype**)taucs_malloc((L->n_sn  )*sizeof(taucs_datatype*));
  L->d_blocks = (taucs_datatype**)taucs_malloc((L->n_sn  )*sizeof(taucs_datatype*));
  L->db_size = (int**)taucs_malloc((L->n_sn  )*sizeof(int*));
  L->sn_size   = (int*) taucs_malloc((L->n_sn+1)*sizeof(int));
  L->sn_up_size   = (int*) taucs_malloc((L->n_sn+1)*sizeof(int));
  L->first_child = (int*) taucs_malloc((L->n_sn+1)*sizeof(int));
  L->next_child  = (int*) taucs_malloc((L->n_sn+1)*sizeof(int));
  
  if ( !L->sn_struct || !L->sn_blocks || !L->up_blocks || !L->d_blocks ||
       !L->db_size || !L->sn_size || !L->sn_up_size || !L->first_child ||
       !L->next_child ) {
    taucs_printf("supernodal_solve_ldlt2: out of memory\n");
    return -1;
  }

  taucs_io_read(handle,1,1,L->n_sn+1,TAUCS_INT,L->first_child);
  taucs_io_read(handle,2,1,L->n_sn+1,TAUCS_INT,L->next_child);
  taucs_io_read(handle,7,1,L->n_sn,TAUCS_INT,L->sn_size);
  taucs_io_read(handle,8,1,L->n_sn,TAUCS_INT,L->sn_up_size);
 
  for(i=0;i<L->n_sn;i++){
    L->sn_struct[i] = NULL;
    L->sn_blocks[i] = NULL;
    L->up_blocks[i] = NULL;
    L->d_blocks[i] = NULL;
    L->db_size[i] = NULL;
  }
  
  Y = (taucs_datatype*) taucs_malloc(ld_X * n * sizeof(taucs_datatype));
  T = (taucs_datatype*) taucs_malloc(ld_X * n * sizeof(taucs_datatype));
  if (!Y || !T) {
    taucs_free(Y);
    taucs_free(T);
    taucs_printf("supernodal_solve_ldlt2: out of memory\n");
    return -1;
  }

  /*taucs_datatype* x = (taucs_datatype*) ((taucs_datatype*)X+j*ld_X);
    taucs_datatype* b = (taucs_datatype*) ((taucs_datatype*)B+j*ld_B);*/
   
    
    /* before we start we need:
       1. put b into y (for the l solve) */
	for (j=0; j<n; j++) {
	  for (i=0; i<ld_X; i++) {
      Y[i+j*(ld_X)] = ((taucs_datatype*)B)[i+j*(ld_X)];
    }
	}
    
    /*for(i=0;i<L->n;i++)
      taucs_printf("%.4e ",y[i]);
      taucs_printf("\n");*/
    
    ret = recursive_supernodal_solve_l_ooc2 (L->n_sn,
					    TRUE,  /* this is the root */
					    handle,
					    L->n_sn,
					    L->first_child, L->next_child,
					    L->sn_struct,L->sn_size,L->sn_up_size,
					    X, Y, T, ld_X,n);
    
    if ( ret ) {
      taucs_free(Y);
      taucs_free(T);
      taucs_printf("supernodal_solve_ldlt2: No solution found.\n");
      return -1;
    }
    
    /*for(i=0;i<L->n;i++)
      taucs_printf("%.4e ",x[i]);
      taucs_printf("\n");*/
    
    /* solve the Diagonal block */
    ret = recursive_supernodal_solve_d_ooc2(L->n_sn,
					   TRUE,
					   handle,
					   L->n_sn,
					   L->first_child, L->next_child,
					   L->sn_struct,L->sn_size,L->sn_up_size,
					   Y, X, T,ld_X,n);
    
    if ( ret ) {
      taucs_free(Y);
      taucs_free(T);
      taucs_printf("supernodal_solve_ldlt2: No solution found.\n");
      return -1;
    }
    
    /*for(i=0;i<L->n;i++)
      taucs_printf("%.4e ",y[i]);
      taucs_printf("\n");*/
    
    ret = recursive_supernodal_solve_lt_ooc2(L->n_sn,
					    TRUE,  /* this is the root */
					    handle,
					    L->n_sn,
					    L->first_child, L->next_child,
					    L->sn_struct,L->sn_size,L->sn_up_size,
					    X, Y, T,ld_X,n);
    
    if ( ret ) {
      taucs_free(Y);
      taucs_free(T);
      taucs_printf("supernodal_solve_ldlt2: No solution found.\n");
      return -1;
    }
    
  
  taucs_free(Y);
  taucs_free(T);
  ooc_supernodal_factor_free(L);
  return 0;
}
/*******************************************/

int taucs_dtl(ooc_solve_ldlt)(void* vL,
			      void* vx, void* vb)
{
  taucs_io_handle* handle = (taucs_io_handle*) vL;
  taucs_datatype* x = (taucs_datatype*) vx;
  taucs_datatype* b = (taucs_datatype*) vb;
  /*  char* filename = (char*) vL; */
  supernodal_factor_matrix_ooc_ldlt* L;
  
  taucs_datatype* y;
  taucs_datatype* t; /* temporary vector */
  int     i, ret = 0;

  L = multifrontal_supernodal_create();
  
  taucs_io_read(handle,5,1,1,TAUCS_INT,&(L->n));
  taucs_io_read(handle,0,1,1,TAUCS_INT,&(L->n_sn));
  L->sn_struct = (int**)taucs_malloc((L->n_sn  )*sizeof(int*));
  L->sn_blocks = (taucs_datatype**)taucs_malloc((L->n_sn  )*sizeof(taucs_datatype*));
  L->up_blocks = (taucs_datatype**)taucs_malloc((L->n_sn  )*sizeof(taucs_datatype*));
  L->d_blocks = (taucs_datatype**)taucs_malloc((L->n_sn  )*sizeof(taucs_datatype*));
  L->db_size = (int**)taucs_malloc((L->n_sn  )*sizeof(int*));
  L->sn_size   = (int*) taucs_malloc((L->n_sn+1)*sizeof(int));
  L->sn_up_size   = (int*) taucs_malloc((L->n_sn+1)*sizeof(int));
  L->first_child = (int*) taucs_malloc((L->n_sn+1)*sizeof(int));
  L->next_child  = (int*) taucs_malloc((L->n_sn+1)*sizeof(int));
  taucs_io_read(handle,1,1,L->n_sn+1,TAUCS_INT,L->first_child);
  taucs_io_read(handle,2,1,L->n_sn+1,TAUCS_INT,L->next_child);
  taucs_io_read(handle,7,1,L->n_sn,TAUCS_INT,L->sn_size);
  taucs_io_read(handle,8,1,L->n_sn,TAUCS_INT,L->sn_up_size);
  /*  for (i=0; i<L->n_sn; i++) {
      L->sn_struct[i] = (int*)taucs_malloc(L->sn_up_size[i]*sizeof(int));
      taucs_io_read(handle,IO_BASE+i,1,L->sn_up_size[i],TAUCS_INT,L->sn_struct[i]);
      }*/
  for(i=0;i<L->n_sn;i++){
    L->sn_struct[i] = NULL;
    L->sn_blocks[i] = NULL;
    L->up_blocks[i] = NULL;
    L->d_blocks[i] = NULL;
    L->db_size[i] = NULL;
  }
  
  y = (taucs_datatype*) taucs_malloc((L->n) * sizeof(taucs_datatype));
  t = (taucs_datatype*) taucs_malloc((L->n) * sizeof(taucs_datatype));
  if (!y || !t) {
    taucs_free(y);
    taucs_free(t);
    taucs_printf("supernodal_solve_ldlt: out of memory\n");
    return -1;
  }
  
  /* before we start we need:
     1. put b into y (for the l solve) */
  for (i=0; i<L->n; i++) {
    y[i] = b[i];
  }
  
  /*for(i=0;i<L->n;i++)
    taucs_printf("%.4e ",y[i]);
    taucs_printf("\n");*/
  
  ret = recursive_supernodal_solve_l_ooc (L->n_sn,
				    TRUE,  /* this is the root */
				    handle,
				    L->n_sn,
				    L->first_child, L->next_child,
				    L->sn_struct,L->sn_size,L->sn_up_size,
				    x, y, t);

	if ( ret ) {
		taucs_free(y);
    taucs_free(t);
		taucs_printf("supernodal_solve_ldlt: No solution found.\n");
    return -1;
	}

  /*for(i=0;i<L->n;i++)
    taucs_printf("%.4e ",x[i]);
    taucs_printf("\n");*/
  
  /* solve the Diagonal block */
  ret = recursive_supernodal_solve_d_ooc(L->n_sn,
				   TRUE,
				   handle,
				   L->n_sn,
				   L->first_child, L->next_child,
				   L->sn_struct,L->sn_size,L->sn_up_size,
				   y, x, t);

	if ( ret ) {
		taucs_free(y);
    taucs_free(t);
		taucs_printf("supernodal_solve_ldlt: No solution found.\n");
    return -1;
	}

  /*for(i=0;i<L->n;i++)
    taucs_printf("%.4e ",y[i]);
    taucs_printf("\n");*/
  
  ret = recursive_supernodal_solve_lt_ooc(L->n_sn,
				    TRUE,  /* this is the root */
				    handle,
				    L->n_sn,
				    L->first_child, L->next_child,
				    L->sn_struct,L->sn_size,L->sn_up_size,
				    x, y, t);

	if ( ret ) {
		taucs_free(y);
    taucs_free(t);
		taucs_printf("supernodal_solve_ldlt: No solution found.\n");
    return -1;
	}

  /*for(i=0;i<L->n;i++)
    taucs_printf("%.4e ",x[i]);
    taucs_printf("\n");*/
  
  /* TEST 
     writeFactorFacts(L,"oocc");
     M = taucs_dsupernodal_factor_ldlt_ooc_to_ccs((void*)L);
     taucs_ccs_write_ijv(M,"C:/taucs/Results/Looc.ijv");
     M = taucs_dsupernodal_factor_diagonal_ooc_to_ccs((void*)L);
     taucs_ccs_write_ijv(M,"C:/taucs/Results/Dooc.ijv");*/
  
  taucs_free(y);
  taucs_free(t);
  ooc_supernodal_factor_free(L);
  return 0;
}

#if defined(TAUCS_CORE_DOUBLE) || defined(TAUCS_CORE_SINGLE)
static void
recursive_inertia_calc_ooc(int sn,       /* this supernode */
														int is_root,  /* is v the root? */
														int* first_child,
														int* next_child,
														int* sn_sizes,	
														int n_sn,
														taucs_io_handle* handle,
														int *inertia)
{
	int child;
  int  sn_size; /* number of rows/columns in the supernode    */
  taucs_datatype D[4];
  int i;
	taucs_datatype* d_block;
	int * db_size;
	 
   for (child = first_child[sn]; child != -1; child = next_child[child]) {
  	   recursive_inertia_calc_ooc(child,
				  FALSE,
				  first_child,next_child,
				  sn_sizes, n_sn,
				  handle, inertia);
   }

	if(!is_root) {

		sn_size = sn_sizes[sn];
		
		d_block = (taucs_datatype*)taucs_calloc(2*sn_size,sizeof(taucs_datatype));
    taucs_io_read(handle,IO_BASE+2*n_sn+6*sn+2,
		  sn_size, 2, TAUCS_CORE_DATATYPE,d_block);
    
    db_size = (int*)taucs_malloc(sn_size*sizeof(int));
    taucs_io_read(handle,IO_BASE+2*n_sn+6*sn+3,
		  sn_size, 1, TAUCS_INT,db_size);


		for (i=0; i<sn_size; i++)	{
		 if ( db_size[i] == 1 ) {/* a 1*1 block x = b/Aii */

			 D[0] = d_block[2*i];
       if(D[0] > 0)
         inertia[1]++;
       else if(D[0] < 0)
         inertia[2]++;
       else
         inertia[0]++;
		 }

		 else if ( db_size[i] == 2 ) /* a 2*2 block */
		 {
			 D[0] = d_block[2*i];
			 D[1] = D[2] = d_block[2*i+1];
			 D[3] = d_block[2*i+2];

       if( D[0] >= -D[3] ){
         inertia[1]++;
         if( taucs_mul(D[0],D[3]) > taucs_mul(D[1],D[1]) )
             inertia[1]++;
         else
             inertia[2]++;
       } else {
         inertia[2]++;
         if( taucs_mul(D[0],D[3]) > taucs_mul(D[1],D[1]) )
           inertia[2]++;
         else
           inertia[1]++;
         }
			 i++;
		 }
		 else {/* shouldn't get here */
			 taucs_printf("db_size[%d][%d] = %d\n",sn,i,db_size[i] );
  		 assert(0);
     }
		}
		taucs_free(d_block);
    taucs_free(db_size);
	}
}

void	taucs_dtl(inertia_calc_ooc)(taucs_io_handle* handle,int* inertia)
{
	int n_sn;
	int* sn_size = NULL;
	int* first_child= NULL;
	int* next_child = NULL;

	taucs_io_read(handle,0,1,1,TAUCS_INT,&n_sn);
  sn_size   = (int*) taucs_malloc(n_sn*sizeof(int));
  first_child = (int*) taucs_malloc((n_sn+1)*sizeof(int));
  next_child  = (int*) taucs_malloc((n_sn+1)*sizeof(int));
  taucs_io_read(handle,1,1,n_sn+1,TAUCS_INT,first_child);
  taucs_io_read(handle,2,1,n_sn+1,TAUCS_INT,next_child);
  taucs_io_read(handle,7,1,n_sn,TAUCS_INT,sn_size);

	recursive_inertia_calc_ooc(n_sn,TRUE,first_child,next_child,
														sn_size,n_sn, handle, inertia);

	taucs_free(sn_size);
	taucs_free(first_child);
	taucs_free(next_child);

}
#endif

void taucs_dtl(get_statistics_ooc)( double* pflops,
																double* pnnz,
																void* vL)
{
  double nnz   = 0.0;
  double flops = 0.0;
  int sn,i,colnnz;
  
	supernodal_factor_matrix_ooc_ldlt* L = (supernodal_factor_matrix_ooc_ldlt*) vL;
  
  for (sn=0; sn<(L->n_sn); sn++) {
    for (	i=0, colnnz = (L->sn_up_size)[sn];
		i<(L->sn_size)[sn];
		i++, colnnz--) {
      /* There was a bug here. I did not count muliply-adds in the
				 update part of the computation as 2 flops but one. */
      /*flops += ((double)(colnnz) - 1.0) * ((double)(colnnz) + 2.0) / 2.0;*/
      flops += 1.0 + ((double)(colnnz)) * ((double)(colnnz));
      nnz   += (double) (colnnz);
    }
  }
  *pflops = flops;
  *pnnz = nnz;
}
/*******************************************************************/
/**                     OOC Panelize Factor                       **/
/*******************************************************************/


/*static void
recursive_find_parent(int sn,
supernodal_panel* P,
supernodal_factor_matrix_ooc_ldlt* L,
int* first_leaf)
{
int child;

if ( L->first_child[sn] == -1 && *first_leaf == -1 )
*first_leaf = sn;

for ( child = L->first_child[sn];child != -1;child = L->next_child[child] )
{
  recursive_find_parent(child,P,L,first_leaf);
		P->parent[child] = sn;
}
}*/

/* NOTE: not used */
/*static int
  leftlooking_supernodal_update_ooc(int J,
  int K,
  int bitmap[],
  taucs_datatype* dense_update_matrix,
  taucs_io_handle* handle,
  taucs_ccs_matrix* A,
  supernodal_factor_matrix_ooc_ldlt* L)
  {
  int i,j,ir;
  int sn_size_child = (L->sn_size)[K];
  int sn_up_size_child = (L->sn_up_size)[K];
  int exist_upd=0;
  int first_row=0;
  int row_count=0;
  int PK,M,N,LDA,LDB,LDC;
  
  for(i=sn_size_child;i<sn_up_size_child;i++) {
  if(L->col_to_sn_map[L->sn_struct[K][i]]<J)
  continue;
  
  if(L->col_to_sn_map[L->sn_struct[K][i]]==J) {
  if(!exist_upd) first_row = i;
  row_count++;
  exist_upd = 1;
  }
  }
  
  if(exist_upd) { 
  if(!(L->up_blocks)[K]) {
  (L->up_blocks)[K] = (taucs_datatype*)
  taucs_calloc((sn_up_size_child-sn_size_child)*sn_size_child,
  sizeof(taucs_datatype));
  taucs_io_read_try(handle,IO_BASE+2*(L->n_sn)+6*K+1,
  sn_up_size_child-sn_size_child,sn_size_child ,
  TAUCS_CORE_DATATYPE,(L->up_blocks)[K]);
  }
  LDA = LDB = sn_up_size_child-sn_size_child;
  M  = sn_up_size_child - first_row ;
  LDC =  M;
  N  = row_count; 
  PK = sn_size_child; 	 
  
  taucs_gemm ("No Conjugate",
  "Conjugate",
  &M,&N,&PK,
  &taucs_one_const,
  &(L->up_blocks[K][first_row-sn_size_child]),&LDA,
  &(L->up_blocks[K][first_row-sn_size_child]),&LDB,
  &taucs_zero_const,
  dense_update_matrix,&LDC);
  
  
  
  assert((double)row_count*(double)LDC < 2048.0*1024.0*1024.0);

  for(j=0;j<row_count;j++) {
  for(ir=j;ir<row_count;ir++){
  L->sn_blocks[J][(bitmap[L->sn_struct[K][first_row+j]]-1)*L->sn_size[J] +
  (bitmap[L->sn_struct[K][first_row+ir]]-1)] =
  taucs_sub(L->sn_blocks[J][(bitmap[L->sn_struct[K][first_row+j]]-1)*
  L->sn_size[J]+(bitmap[L->sn_struct[K][first_row+ir]]-1)] , 
  dense_update_matrix[j*LDC+ir]);
  
  assert((double)(bitmap[L->sn_struct[K][first_row+j]]-1)*(double)L->sn_size[J] < 2048.0*1024.0*1024.0);
  }
  }
  for(j=0;j<row_count;j++) {
  for(ir=row_count;ir<M;ir++){
  L->up_blocks[J][(bitmap[L->sn_struct[K][first_row+j]]-1)*
  (L->sn_up_size[J]-L->sn_size[J]) +
  (bitmap[L->sn_struct[K][ir+first_row]]-1)] =
  taucs_sub(L->up_blocks[J][(bitmap[L->sn_struct[K][first_row+j]]-1)*
  (L->sn_up_size[J]-L->sn_size[J])+
  (bitmap[L->sn_struct[K][ir+first_row]]-1)] ,
  dense_update_matrix[j*LDC+ir]);
  
  assert((double)(bitmap[L->sn_struct[K][first_row+j]]-1)*(double)(L->sn_up_size[J]-L->sn_size[J]) < 2048.0*1024.0*1024.0);
  } 
  }
  
  }
  
  return exist_upd;
  }*/

static int
leftlooking_supernodal_update_panel_ooc(int J,
					int K,
					int bitmap[],
					int direct_update,
					taucs_datatype* dense_update_matrix,
					taucs_io_handle* handle,
					taucs_ccs_matrix* A,
					supernodal_factor_matrix_ooc_ldlt* L,
					supernodal_panel* P)
{
  int parent,i;
  int exist_upd = 0;
  int rej_size;
  int* old_struct = NULL;
  int sn_size = 0, sn_up_size = 0, old_size = 0;

  sn_size = L->sn_size[J];
  sn_up_size = L->sn_up_size[J];
  old_size = L->orig_sn_size[J];
  
  /* we only want to inform of rejected if its
     a direct update */
  rej_size = direct_update ? L->sn_rej_size[K] : 0;
  
  /* read the old structure, without the rejected columns,
     and put only them in the bitmap.
     we do that in 3 stages: 1) read the old structure.
     2) put the old sturcture in the bitmap.
     3) put the new sturcture in the bitmap, only if
     it was already in the bitmap. this way we have
     the correct sturcture in the bitmap, without the new
     rejected rows.*/
  old_struct = (int*)taucs_malloc((old_size+sn_up_size-sn_size)*sizeof(int));
  taucs_io_read(handle,IO_BASE+J,1,old_size+sn_up_size-sn_size,TAUCS_INT,old_struct);
  
  for(i=0;i<old_size+sn_up_size-sn_size;i++) bitmap[old_struct[i]]=i+1;
  
  for(i=0;i<L->sn_size[J];i++) 
    if ( direct_update || bitmap[L->sn_struct[J][i]] )
      bitmap[L->sn_struct[J][i]]=i+1;
  for(i=L->sn_size[J];i<L->sn_up_size[J];i++)	
    if ( direct_update || bitmap[L->sn_struct[J][i]] )
      bitmap[L->sn_struct[J][i]] = i - L->sn_size[J] + 1;
  
  taucs_free(old_struct);
  
  
  leftlooking_supernodal_update_ldlt(J,K,bitmap,dense_update_matrix,A,L,
				     rej_size,&exist_upd,handle);
  
  if (direct_update) {
    
    if ( supernodal_ldlt_modify_child(L,K,L->sn_rej_size[K]) == -1 )
      return -1;
    /* write the child again */
    if (L->sn_rej_size[K]) {
      taucs_io_append(handle,IO_BASE+L->n_sn*2+6*K+4,
		      L->sn_size[K],L->sn_size[K],
		      TAUCS_CORE_DATATYPE,L->sn_blocks[K]);
      taucs_io_append(handle,IO_BASE+L->n_sn*2+6*K+5,
		      L->sn_up_size[K]-L->sn_size[K],L->sn_size[K],
		      TAUCS_CORE_DATATYPE,L->up_blocks[K]);
    }
  }
  
  for(i=0;i<L->sn_up_size[J];i++)
    bitmap[L->sn_struct[J][i]] = 0;
  
  /*exist_upd = leftlooking_supernodal_update_ooc(J,K,bitmap,dense_update_matrix,
    handle,A,L);*/
  
  /* update the supernode's parents in the panel */
  for (parent=L->parent[J];parent!=-1;parent=L->parent[parent]) {
    if ( P->sn_to_panel_map[J] == P->sn_to_panel_map[parent] ) {
      
      sn_size = L->sn_size[parent];
      sn_up_size = L->sn_up_size[parent];
      old_size = L->orig_sn_size[parent];
      
      /* again - read the old structure first */
      old_struct = (int*)taucs_malloc((old_size+sn_up_size-sn_size)*sizeof(int));
      taucs_io_read(handle,IO_BASE+parent,1,old_size+sn_up_size-sn_size,TAUCS_INT,old_struct);
      
      for(i=0;i<sn_up_size-sn_size+old_size;i++) bitmap[old_struct[i]]=i+1;
      
      for(i=0;i<L->sn_size[parent];i++)
	if (bitmap[L->sn_struct[parent][i]])
	  bitmap[L->sn_struct[parent][i]]=i+1;
      for(i=L->sn_size[parent];i<L->sn_up_size[parent];i++)
	if ( bitmap[L->sn_struct[parent][i]] )
	  bitmap[L->sn_struct[parent][i]] = i - L->sn_size[parent] + 1;
      
      leftlooking_supernodal_update_ldlt(parent,K,bitmap,dense_update_matrix,A,L,
					 0,&exist_upd,handle);
      
      for(i=0;i<L->sn_up_size[parent];i++)
	bitmap[L->sn_struct[parent][i]] = 0;
      /*if (leftlooking_supernodal_update_ooc(parent,K,bitmap,dense_update_matrix,
	handle,A,L) )
	if (!exist_upd)
	exist_upd = 1;*/
      
      taucs_free(old_struct);
    }
  }
  
  return exist_upd;
}



static int
recursive_leftlooking_supernodal_update_panel_ooc(int J,int K, 
						  int bitmap[], int direct_update,
						  taucs_datatype* dense_update_matrix,
						  taucs_io_handle* handle,
						  taucs_ccs_matrix* A,
						  supernodal_factor_matrix_ooc_ldlt* L,
						  supernodal_panel* P)
{
  int  child;
  int sn_size_child = (L->sn_size)[K];
  int sn_up_size_child = (L->sn_up_size)[K];
  int exist_upd = 0;
  
  if(sn_up_size_child-sn_size_child>0) {
    
    if(!(L->sn_struct)[K]) {
      L->sn_struct[K] = (int*)taucs_malloc(sn_up_size_child*sizeof(int));
      taucs_io_read(handle,IO_BASE+L->n_sn+K,1,sn_up_size_child,TAUCS_INT,L->sn_struct[K]);
      /*taucs_printf("reading struct of %d\n",K);*/
    }
    
    /* if end of update to up_sn or edge condition */
    /*	if( L->col_to_sn_map[L->sn_struct[K][i]]!=up_sn ||
	i==sn_up_size_child-1) {*/
    
    /* is this row index included in the columns of sn in the same panel? */
    /*	if(L->col_to_sn_map[L->sn_struct[K][i]]!=up_sn) {
	if(sn_to_panel_map[L->col_to_sn_map[L->sn_struct[K][i]]]==updated_panel) {
	up_sn = L->col_to_sn_map[L->sn_struct[K][i]];
	if(!exist_upd) first_row = i;
	row_count++;
	exist_upd = 1;
	if( i==sn_up_size_child-1) { */
    
    exist_upd = leftlooking_supernodal_update_panel_ooc(J,K,bitmap,direct_update,
							dense_update_matrix,
							handle,A,L,P);
    if ( exist_upd == -1 )
      return -1;
    
    
  }
  
  if ( direct_update ) {
    /*	if ( supernodal_ldlt_modify_child(L,K,L->sn_rej_size[K]) )
	return -1;		
	if (L->sn_rej_size[K]) {
	taucs_io_append(handle,IO_BASE+L->n_sn*2+6*K+4,
	L->sn_size[K],L->sn_size[K],
	TAUCS_CORE_DATATYPE,L->sn_blocks[K]);
	taucs_io_append(handle,IO_BASE+L->n_sn*2+6*K+5,
	L->sn_up_size[K]-L->sn_size[K],L->sn_size[K],
	TAUCS_CORE_DATATYPE,L->up_blocks[K]);
	}*/
  }
  
  
  
  /* free update sn from memory */
  ooc_supernodal_free(L,K);	
  
  if(exist_upd &&
     P->sn_to_panel_map[J]!=P->sn_to_panel_map[K]) {	  
    
    for (child = L->first_child[K]; child != -1; child = L->next_child[child]) {
      if (recursive_leftlooking_supernodal_update_panel_ooc(J,child,
							    bitmap, FALSE,
							    dense_update_matrix,
							    handle,A,L,P) == -1 )
	return -1;
    }
  } 
  return 0;
}

static double
get_memory_overhead( int n )
{
  double memory_overhead;
  memory_overhead = 
    10.0*(double)(n*sizeof(int)) + /* integer vectors in L */
    5.0*(double)(n*sizeof(int)) + /* pointer arrays in L  */
    2.0*(double)(n*sizeof(int)) + /* integer vectors in program  */
    4.0*3.0*(double)(n*sizeof(int));  /* singlefile matrix arrays */
  return memory_overhead;
}




/* this method is called when a panel is full and allocated,
it factors each supernode in the panel, in postorder, and updates
and factors it. the update is done so that each supernode in the panel
is updated from each supernode child braught from the disk. */
static int
leftlooking_supernodal_factor_panel_ldlt_ooc(int postorder,
					     int max_postorder,
					     int* indmap,
					     int* bitmap,
					     taucs_io_handle* handle,
					     taucs_ccs_matrix* A,
					     supernodal_factor_matrix_ooc_ldlt* L,
					     supernodal_panel* P,
					     int parent,
					     int* p_rejected)
{ 
  int i;
  taucs_datatype* dense_update_matrix = NULL;
  
  if ( postorder == max_postorder ) /* no supernodes to factor */
    return 0;
  
  P->used = TRUE;
  
  if (!dense_update_matrix) 
    dense_update_matrix = (taucs_datatype*) 
      taucs_calloc(P->max_size,sizeof(taucs_datatype));
  
  /* we go through each of the supernodes in the panel,
     in postorder, and factor them */
  for (i=postorder; i<max_postorder; i++) {
    
    int sn = P->postorder[i];
    int child;
    parent = L->parent[sn];
    
    for (child = L->first_child[sn]; child != -1; child = L->next_child[child]) {
      if(P->sn_to_panel_map[sn]!=P->sn_to_panel_map[child]) {
	
	/* NOTE: no need to modify,
	   because what was rejected in another panel was already included
	   in the parent. 
	   if ( supernodal_ldlt_modify_parent(L,sn,L->sn_rej_size[child]) ) {
	   taucs_free(dense_update_matrix);
	   return -1;
	   }*/
	
	/* this method should update all supernodes in panel!! */
	if (recursive_leftlooking_supernodal_update_panel_ooc(sn,child,
							      bitmap, TRUE,
							      dense_update_matrix,
							      handle,A,L,P) == -1 ) {
	  
	  taucs_free(dense_update_matrix);
	  return -1;
	}
	/* NOTE: the modify_child is done inside the update_panel */
	
      }
    }
    
    if (leftlooking_supernodal_front_factor_ldlt(sn,indmap,
						 A,L,parent,p_rejected,handle) == -1) {
      /* nonpositive pivot */
      return -1;
    }																						
    
    taucs_io_append_supernode(sn,handle,L);
    
    if ( P->sn_to_panel_map[sn] == P->sn_to_panel_map[L->parent[sn]] ) {
      
      if ( supernodal_ldlt_modify_parent(L,L->parent[sn],L->sn_rej_size[sn]) ) {
	taucs_free(dense_update_matrix);
	return -1;
      }
      
      if ( leftlooking_supernodal_update_panel_ooc(L->parent[sn],sn,bitmap, TRUE,
						   dense_update_matrix,
						   handle,A,L,P) == -1 ) {
	taucs_free(dense_update_matrix);
	return -1;
      }
      
    }
    
    ooc_supernodal_free(L,sn);
  }
  
  taucs_free(dense_update_matrix);
  return 0;
}

static int
recursive_leftlooking_supernodal_factor_panel_ldlt_ooc(int sn,    /* this supernode */
						       int father_sn,
						       int is_root,/* is sn the root?*/
						       int* indmap,
						       int* bitmap,
						       taucs_io_handle* handle,
						       taucs_ccs_matrix* A,
						       supernodal_factor_matrix_ooc_ldlt* L,
						       supernodal_panel* P,
						       int* first_sn_in_panel,
						       int* p_rejected)
{
  int  child,maxIndex;
  int failed = 0, i;
  double this_sn_mem = 0.0;
  int rejected_from_child = 0;
  
  for (child = L->first_child[sn]; child != -1; child = L->next_child[child]) {
    if(P->sn_in_core[child]){
      if (recursive_read_L_cols(child,FALSE,handle,L)) {
				taucs_printf("recursive_read_L_cols\n");
	return -1;
      }
      if (recursive_leftlooking_supernodal_factor_ldlt(child,FALSE,indmap,bitmap,
						       A,L,sn,&rejected_from_child,handle)) {
	/* failure */
	taucs_printf("recursive_leftlooking_supernodal_factor_ldlt %d\n",child);
	return -1;
      }
      if (recursive_append_L(child,FALSE,handle,L)) {
				taucs_printf("recursive_append_L\n");
	return -1;
      }
    } else {
      if (recursive_leftlooking_supernodal_factor_panel_ldlt_ooc(child,
								 sn,
								 FALSE,
								 indmap,
								 bitmap,
								 handle,
								 A,L,P,
								 first_sn_in_panel,&rejected_from_child)) {
	/* failure */
	taucs_printf("recursive_leftlooking_supernodal_factor_panel_ldlt_ooc\n");
	return -1;
      }
    }
  }
  
  if (!is_root) { 
    
    int sn_size = L->sn_size[sn];
    int sn_up_size = L->sn_up_size[sn];
    
    /* check if this supernode can be inside current panel */
    this_sn_mem = get_memory_needed_by_supernode(sn_size,sn_up_size);
		assert( this_sn_mem < P->max_mem ); /*this means we need to split the supernode*/

    if( P->avail_mem - this_sn_mem < 0.0 ) {
      /* this supernode cannot fit into the current panel,
	 factor the panel and start a new one with this sn */
      
      failed = leftlooking_supernodal_factor_panel_ldlt_ooc(*first_sn_in_panel,
							    P->n_postorder,
							    indmap,
							    bitmap,
							    handle,
							    A,L,P,father_sn,p_rejected);
      
			if ( failed ) {
				taucs_printf("leftlooking_supernodal_factor_panel_ldlt_ooc\n");
				return -1;
			}
      
      P->n_pn++;
      P->avail_mem = P->max_mem;
      P->max_size = 0;
      *first_sn_in_panel = P->n_postorder;
      /* get the sizes again - they might have changed by now */
      sn_size = L->sn_size[sn];
      sn_up_size = L->sn_up_size[sn];
      
    } 
    
    /* this supernode is part of the currect panel.
       add him to the panel and continue */
    P->sn_to_panel_map[sn] = P->n_pn;
    P->avail_mem -= this_sn_mem;
    P->max_size = max(sn_size*sn_up_size,P->max_size);
    
    /* put this supernode in its place in the postorder */
    P->postorder[P->n_postorder] = sn;
    P->n_postorder++;
    
    /* allocate the supernode */
    if (sn_size) {
      if(!(L->sn_blocks)[sn])
	(L->sn_blocks)[sn] = (taucs_datatype*) 
	  taucs_calloc(sn_size*sn_size,sizeof(taucs_datatype));
      if(!(L->d_blocks)[sn])
	(L->d_blocks)[sn] = (taucs_datatype*) 
	  taucs_calloc(sn_size*2,sizeof(taucs_datatype));
      if(!(L->db_size)[sn])
	(L->db_size)[sn] = (int*)
	  taucs_calloc(sn_size,sizeof(taucs_datatype));
      
      if(sn_up_size-sn_size>0 && !(L->up_blocks)[sn])
	(L->up_blocks)[sn] = (taucs_datatype*)
	  taucs_calloc((sn_up_size-sn_size) * sn_size,sizeof(taucs_datatype));
      
    }
    
    if(!L->sn_struct[sn]){
      L->sn_struct[sn] = (int*)taucs_malloc(sn_up_size*sizeof(int));
      taucs_io_read(handle,IO_BASE+sn,1,sn_up_size,TAUCS_INT,L->sn_struct[sn]);
      /*taucs_printf("reading struct of %d\n",sn);*/
    }
    
    /* put the max index */
    maxIndex = 0;
    for ( i=0; i<sn_size; i++ ) {
      maxIndex  = max(L->sn_struct[sn][i],maxIndex);
    }
    L->sn_max_ind[sn] = maxIndex;
    
  } else {
    /* we need to factor the last panel */
    leftlooking_supernodal_factor_panel_ldlt_ooc(*first_sn_in_panel,
						 P->n_postorder,
						 indmap,
						 bitmap,
						 handle,
						 A,L,P,
						 father_sn,p_rejected);
  }

  return 0;
}	




int taucs_dtl(ooc_factor_ldlt)(taucs_ccs_matrix* A, 
			       taucs_io_handle* handle,
			       double memory)
{
  supernodal_factor_matrix_ooc_ldlt* L;
  supernodal_panel* P;
  int i,first_sn_in_panel = 0;
  int* indmap,* bitmap;
  int rejected = 0;
  double wtime, ctime;
  double memory_overhead, reduced_memory;
  
  /* compute fixed memory overhead */
  
  memory_overhead = get_memory_overhead(A->n);
  
  taucs_printf("\t\tOOC memory overhead bound %.0lf MB (out of %.0lf MB available)\n",
	       memory_overhead/1048576.0,memory/1048576.0);
  
  taucs_printf(">>> 1\n");
  
  if ( memory - memory_overhead < 
       2.0*(double)((A->n)*sizeof(taucs_datatype)) + 
       2.0*(double)((A->n)*sizeof(int)) ) {
    taucs_printf("\t\ttaucs_ccs_factor_ldlt_ll_ooc: not enough memory\n");
    return -1;
  }

	reduced_memory = ((memory - memory_overhead)/3.0) * PANEL_RESIZE;

  wtime = taucs_wtime();
  ctime = taucs_ctime();

  L = multifrontal_supernodal_create();
  
  taucs_io_append(handle,5,1,1,TAUCS_INT,&(A->n));
  
  taucs_ccs_ooc_symbolic_elimination(A,L,
				     TRUE /* sort row indices */,
				     TRUE /* return col_to_sn_map */,
				     reduced_memory,
				     ooc_sn_struct_handler,handle);

  /*writeFactorFacts(L,"ooca");*/
  
  wtime = taucs_wtime()-wtime;
  ctime = taucs_ctime()-ctime;
  taucs_printf("\t\tSymbolic Analysis            = % 10.3f seconds (%.3f cpu)\n",
	       wtime,ctime);

  /* we now compute an exact memory overhead bound using n_sn */
  memory_overhead = get_memory_overhead(L->n_sn);
    
  taucs_printf("\t\tOOC actual memory overhead %.0lf MB (out of %.0lf MB available)\n",
	       memory_overhead/1048576.0,memory/1048576.0);

	reduced_memory = ((memory - memory_overhead)/3.0) * PANEL_RESIZE;
 
  wtime = taucs_wtime();
  ctime = taucs_ctime();

  taucs_io_append(handle,0,1,1,TAUCS_INT,&(L->n_sn));
  taucs_io_append(handle,1,1,L->n_sn+1,TAUCS_INT,L->first_child);
  taucs_io_append(handle,2,1,L->n_sn+1,TAUCS_INT,L->next_child);
  taucs_io_append(handle,3,1,L->n_sn,TAUCS_INT,L->sn_size);
  taucs_io_append(handle,4,1,L->n_sn,TAUCS_INT,L->sn_up_size);
  /*taucs_io_append(handle,5,1,1,TAUCS_INT,&(L->n));*/
  taucs_io_append(handle,6,1,1,TAUCS_INT,&(A->flags));

  wtime = taucs_wtime()-wtime;
  ctime = taucs_ctime()-ctime;
  taucs_printf("\t\tOOC Supernodal Left-Looking Prepare L = % 10.3f seconds (%.3f cpu)\n",
	       wtime,ctime);

  wtime = taucs_wtime();
  ctime = taucs_ctime();

  P = taucs_panel_create( reduced_memory, L->n_sn );
	
  indmap  = (int*)taucs_malloc((A->n+1)*sizeof(int));
  bitmap	= (int*)taucs_calloc((A->n+1),sizeof(int));

  for(i=0;i<L->n_sn;i++){
    (L->sn_blocks)[i] = NULL;
    (L->up_blocks)[i] = NULL;
    (L->sn_struct)[i] = NULL;
  }

  #ifdef TAUCS_HACK_LDLT_TESTING
	get_input_parameters(gl_parameters);
  #endif
	switch(gl_factor_alg)	{
		case fa_right_looking:
			taucs_printf("\t\tUsing RightLooking update in dense factorization.\n");
			break;
		case fa_blocked:
			taucs_printf("\t\tUsing blocked update in dense factorization.\n");
			break;
		case fa_lapack:
			taucs_printf("\t\tUsing lapack dense factorization.\n");
			break;
	}

  wtime = taucs_wtime();
  ctime = taucs_ctime();
  if(recursive_compute_supernodes_ll_in_core(L->n_sn,
					     TRUE,
					     P->max_mem,
					     P->sn_in_core,
					     L)<0.0) {
    ooc_supernodal_factor_free(L);
    taucs_free(indmap);
    taucs_free(bitmap);
    taucs_panel_free(P);
    return -1;
  }
  
  
  wtime = taucs_wtime()-wtime;
  ctime = taucs_ctime()-ctime;
  taucs_printf("\t\tOOC Supernodal Left-Looking Scheduling = % 10.3f seconds (%.3f cpu)\n",
	       wtime,ctime);

  wtime = taucs_wtime();
  ctime = taucs_ctime();
  
  if (recursive_leftlooking_supernodal_factor_panel_ldlt_ooc(L->n_sn,
							     -1,  
							     TRUE, 
							     indmap,
							     bitmap,
							     handle,
							     A,L,P,
							     &first_sn_in_panel,&rejected)) {
    taucs_printf("recursive_leftlooking_supernodal_factor_panel_ldlt_ooc\n");
		ooc_supernodal_factor_free(L);
    taucs_free(bitmap);
    taucs_free(indmap);
    taucs_panel_free(P);
    return -1;
  }
  
	{
		double nnz   = 0.0;
    double flops = 0.0;
    taucs_get_statistics_ooc(A->flags,&flops,&nnz,L);
    taucs_printf("\t\tPost Analysis of LDL^T: %.2e nonzeros, %.2e flops\n",
		 nnz, flops);
	}
  /*writeFactorFacts(L,"oocb");*/
  
  taucs_printf("\t\tOOC Supernodal Left-Looking:\n");
  taucs_printf("\t\t\tread count           = %.0f \n",handle->nreads);
  taucs_printf("\t\t\tread volume (bytes)  = %.2e \n",handle->bytes_read);
  taucs_printf("\t\t\tread time (seconds)  = %.0f \n",handle->read_time);
  taucs_printf("\t\t\twrite count          = %.0f \n",handle->nwrites);
  taucs_printf("\t\t\twrite volume (bytes) = %.2e \n",handle->bytes_written);
  taucs_printf("\t\t\twrite time (seconds) = %.0f \n",handle->write_time);
  if (P->used)
    taucs_printf("\t\t\tnumber of panels %d\n",P->n_pn + 1);
  else
    taucs_printf("\t\t\tnumber of panels %d\n",P->n_pn);
  
  wtime = taucs_wtime()-wtime;
  ctime = taucs_ctime()-ctime;
  taucs_printf("\t\tOOC Supernodal Left-Looking LDL^T = % 10.3f seconds (%.3f cpu)\n",
	       wtime,ctime);
  
  wtime = taucs_wtime();
  ctime = taucs_ctime();
  
  /*for(i=0;i<L->n_sn;i++)
    if ( P->sn_to_panel_map[i] != -1 )
    taucs_printf("sn\t%d\t,panel\t%d\n",i,P->sn_to_panel_map[i]);*/
  
  taucs_free(bitmap);
  taucs_free(indmap);
  taucs_panel_free(P);
  
  /* we now write the new sizes to the disk */
  taucs_io_append(handle,7,1,L->n_sn,TAUCS_INT,L->sn_size);
  taucs_io_append(handle,8,1,L->n_sn,TAUCS_INT,L->sn_up_size);
  
  ooc_supernodal_factor_free(L);
  
  wtime = taucs_wtime()-wtime;
  ctime = taucs_ctime()-ctime;
  
  taucs_printf("\t\tOOC Supernodal Left-Looking Cleanup = % 10.3f seconds (%.3f cpu)\n",
	       wtime,ctime);
  
  return 0;
}

#endif

/*************************************************************/
/* generic interfaces to user-callable routines              */
/*************************************************************/

#ifdef TAUCS_CORE_GENERAL/*DOUBLE*/

int taucs_ooc_factor_ldlt(taucs_ccs_matrix* A,
			  taucs_io_handle*  L,
			  double memory)
{
#ifdef TAUCS_DOUBLE_IN_BUILD
  if (A->flags & TAUCS_DOUBLE)
    return taucs_dooc_factor_ldlt(A,L,memory);
#endif
  
#ifdef TAUCS_SINGLE_IN_BUILD
  if (A->flags & TAUCS_SINGLE)
    return taucs_sooc_factor_ldlt(A,L,memory);
#endif
  
#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_DCOMPLEX)
    return taucs_zooc_factor_ldlt(A,L,memory);
#endif
  
#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_SCOMPLEX)
    return taucs_cooc_factor_ldlt(A,L,memory);
#endif


  assert(0);
  return -1;
}


/* 
   this generic function retrieves the data type
   from the file and uses it to call a specialized 
   function.
*/

int taucs_ooc_solve_ldlt_many(void *L,int n,void* X, int ld_X,void* B, int ld_B)
{
	int flags;
  
  taucs_io_read((taucs_io_handle*)L,
		6,1,1,TAUCS_INT,
		&flags);

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (flags & TAUCS_DOUBLE)
    return taucs_dooc_solve_ldlt_many2(L,n,X,ld_X,B,ld_B);
#endif
  
#ifdef TAUCS_SINGLE_IN_BUILD
  if (flags & TAUCS_SINGLE)
    return taucs_sooc_solve_ldlt_many2(L,n,X,ld_X,B,ld_B);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (flags & TAUCS_DCOMPLEX)
    return taucs_zooc_solve_ldlt_many2(L,n,X,ld_X,B,ld_B);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (flags & TAUCS_SCOMPLEX)
    return taucs_cooc_solve_ldlt_many2(L,n,X,ld_X,B,ld_B);
#endif
  
  assert(0);
  return -1;
}

int taucs_ooc_solve_ldlt (void* L /* actual type: taucs_io_handle* */,
			  void* x, void* b)
{
  int flags;
  
  taucs_io_read((taucs_io_handle*)L,
		6,1,1,TAUCS_INT,
		&flags);

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (flags & TAUCS_DOUBLE)
    return taucs_dooc_solve_ldlt(L,x,b);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (flags & TAUCS_SINGLE)
    return taucs_sooc_solve_ldlt(L,x,b);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (flags & TAUCS_DCOMPLEX)
    return taucs_zooc_solve_ldlt(L,x,b);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (flags & TAUCS_SCOMPLEX)
    return taucs_cooc_solve_ldlt(L,x,b);
#endif
  
  assert(0);
  return -1;
}

void	taucs_get_statistics_ooc(int flags, double* pflops,double* pnnz,void* vL)
{
	
#ifdef TAUCS_DOUBLE_IN_BUILD
	if (flags & TAUCS_DOUBLE) {
    taucs_dget_statistics_ooc(pflops,pnnz,vL);
		return;
	}
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
	if (flags & TAUCS_SINGLE) {
    taucs_sget_statistics_ooc(pflops,pnnz,vL);
		return;
	}
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
	if (flags & TAUCS_DCOMPLEX) {
    taucs_zget_statistics_ooc(pflops,pnnz,vL);
		return;
	}
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
	if (flags & TAUCS_SCOMPLEX) {
    taucs_cget_statistics_ooc(pflops,pnnz,vL);
		return;
	}
#endif

  assert(0);
}

void	taucs_inertia_calc_ooc(int flags, taucs_io_handle* h, int* inertia)
{
#ifdef TAUCS_DOUBLE_IN_BUILD
	if (flags & TAUCS_DOUBLE) {
    taucs_dinertia_calc_ooc(h,inertia);
		return;
	}
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
	if (flags & TAUCS_SINGLE) {
    taucs_sinertia_calc_ooc(h,inertia);
		return;
	}
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (flags & TAUCS_DCOMPLEX)
    return;
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (flags & TAUCS_SCOMPLEX)
    return;
#endif

  assert(0);
}




#endif

/*************************************************************/
/* end of file                                               */
/*************************************************************/




