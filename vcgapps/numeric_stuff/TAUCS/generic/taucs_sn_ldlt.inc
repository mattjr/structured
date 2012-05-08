/*************************************************************/
/*                                                           */
/*************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
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

#define BLAS_FLOPS_CUTOFF  -1.0
#define SOLVE_DENSE_CUTOFF 5

/*******************/
/* Test Utilities  */
/*******************/

#ifndef TAUCS_CORE_GENERAL

/* globals */
static float gl_alpha = (float)0.001;
static int gl_cramer_rule = 0;
static float gl_beta = (float)0.000000000000000000000000000000000000001;
typedef enum { fa_right_looking, fa_blocked, fa_sytrf, fa_potrf } factor_alg_enum;
static int gl_factor_alg = fa_right_looking ;
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
			gl_factor_alg = fa_sytrf;
			break;
		case 3:
			gl_factor_alg = fa_potrf;
			break;
	}
}
#endif

static int
taucs_is_zero_real( taucs_real_datatype v )
{
	if ( fabs(v) < gl_beta )
	  {
	    /*taucs_printf("zero - %e beta - %e\n",v,gl_beta);*/
		return TRUE;
	  }
	return FALSE;
}
static int
taucs_is_zero( taucs_datatype v )
{
	if ( taucs_real_abs(v) < fabs(gl_beta) )
	     {
	       /*taucs_printf("zero - %e beta - %e\n",v,gl_beta);*/
		return TRUE;
	  }
	return FALSE;
}
#endif



/*************************************************************/
/* structures                                                */
/*************************************************************/



typedef struct {
  int     sn_size;		/* size of super node */
  int     n;					/* size of the frontal matrix (sn+up) */
  int*    rowind;			/* row indices to the ordered matrix */

  int     up_size;		/* number of update columns */
  int*    sn_vertices;
  int*    up_vertices;

  taucs_datatype* f1; /* the first block matrix */
  taucs_datatype* f2; /* the second block matrix */
  taucs_datatype* u;	/* the update block matrix */
  /*	F = |--|---|
	|f1| f2|
	|--|---|
	|f2| u |
	|--|---|*/
  int* db_size;				/* sizes of diagonal blocks */
  taucs_datatype* d;	/* the diagonal entries of f1 */
  
} supernodal_frontal_matrix_ldlt;

#define SFM_F1 f1
#define SFM_F2 f2
#define SFM_U   u
#define SFM_D		d

typedef struct {
	int rejected_size;		/* size of the supernode that was rejected */
	int* rowind;					/* To which column was this column moved to */
	int* indices;					/* Which column now resides in this column */
} supernodal_ldlt_factor;


typedef struct {
  int     flags;

  char    uplo;     /* 'u' for upper, 'l' for lower, ' ' don't know; prefer lower. */
  int     n;        /* size of matrix */
  int     n_sn;     /* number of supernodes */

  int* parent;      /* supernodal elimination tree */
  int* first_child;
  int* next_child;

  int* sn_size;     /* size of supernodes (diagonal block) */
  int* orig_sn_size;/* the original size of supernodes, before changes
											 made by rejected columns */
  int* sn_up_size;  /* size of subdiagonal update blocks   */
  int** sn_struct;  /* row structure of supernodes         */
  int* sn_max_ind;  /* the max index of the supernode			 */
  int* sn_rej_size;	/* the rejected size of each supernode */
  
  int** db_size;		/* sizes of diagonal blocks for each column in each supernode.
				   there are sn_size sizes for each supernode */
  
  int* sn_blocks_ld;  /* lda of supernode blocks */
  taucs_datatype** sn_blocks; /* supernode blocks        */
  
  int* up_blocks_ld;  /* lda of update blocks    */
  taucs_datatype** up_blocks; /* update blocks           */
  
  taucs_datatype** d_blocks;	/* diagonal blocks				 */
} supernodal_factor_matrix_ldlt;




/*************************************************************/
/* for qsort                                                 */
/*************************************************************/


static int* compare_indirect_map;
static int compare_indirect_ints( const void* vx, const void* vy)
{
  int* ix = (int*)vx;
  int* iy = (int*)vy;
  if (compare_indirect_map[*ix] < compare_indirect_map[*iy]) return -1;
  if (compare_indirect_map[*ix] > compare_indirect_map[*iy]) return  1;
  return 0;
}


/*************************************************************/
/* create and free the factor object                         */
/*************************************************************/

#ifndef TAUCS_CORE_GENERAL

static supernodal_factor_matrix_ldlt*
multifrontal_supernodal_create()
{
  supernodal_factor_matrix_ldlt* L;

  L = (supernodal_factor_matrix_ldlt*)
		taucs_malloc(sizeof(supernodal_factor_matrix_ldlt));
  if (!L) return NULL;

#ifdef TAUCS_CORE_SINGLE
  L->flags = TAUCS_SINGLE;
#endif

#ifdef TAUCS_CORE_DOUBLE
  L->flags = TAUCS_DOUBLE;
#endif

#ifdef TAUCS_CORE_SCOMPLEX
  L->flags = TAUCS_SCOMPLEX;
#endif

#ifdef TAUCS_CORE_DCOMPLEX
  L->flags = TAUCS_DCOMPLEX;
#endif

  L->uplo      = 'l';
  L->n         = -1; /* unused */

  L->sn_struct   = NULL;
  L->sn_size     = NULL;
	L->sn_up_size  = NULL;
  L->parent      = NULL;
  L->first_child = NULL;
  L->next_child  = NULL;
  L->sn_blocks_ld  = NULL;
  L->sn_blocks     = NULL;
  L->up_blocks_ld  = NULL;
  L->up_blocks     = NULL;
	L->db_size			 = NULL;
	L->d_blocks			 = NULL;
	L->orig_sn_size	 = NULL;
	L->sn_max_ind		 = NULL;
	L->sn_rej_size	 = NULL;

  return L;
}

void taucs_dtl(supernodal_factor_ldlt_free)(void* vL)
{
  supernodal_factor_matrix_ldlt* L = (supernodal_factor_matrix_ldlt*) vL;
  int sn;

  if (!L) return;

  taucs_free(L->parent);
  taucs_free(L->first_child);
  taucs_free(L->next_child);

  taucs_free(L->sn_size);
  taucs_free(L->sn_up_size);
  taucs_free(L->sn_blocks_ld);
  taucs_free(L->up_blocks_ld);

  if (L->sn_struct)
    for (sn=0; sn<L->n_sn; sn++)
      taucs_free(L->sn_struct[sn]);

  if (L->sn_blocks)
    for (sn=0; sn<L->n_sn; sn++)
      taucs_free(L->sn_blocks[sn]);

  if (L->up_blocks)
    for (sn=0; sn<L->n_sn; sn++)
      taucs_free(L->up_blocks[sn]);

	if (L->db_size)
    for (sn=0; sn<L->n_sn; sn++)
			if (L->db_size[sn])
				taucs_free(L->db_size[sn]);

	if (L->d_blocks)
		for (sn=0; sn<L->n_sn; sn++)
      taucs_free(L->d_blocks[sn]);

	
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

void taucs_dtl(supernodal_factor_ldlt_free_numeric)(void* vL)
{
  supernodal_factor_matrix_ldlt* L = (supernodal_factor_matrix_ldlt*) vL;
  int sn;

  for (sn=0; sn<L->n_sn; sn++) {
    taucs_free(L->sn_blocks[sn]);
    L->sn_blocks[sn] = NULL;
    taucs_free(L->up_blocks[sn]);
    L->up_blocks[sn] = NULL;
		if ( L->db_size ) {
			taucs_free(L->db_size[sn]);
			L->db_size[sn] = NULL;
		}
		taucs_free(L->d_blocks[sn]);
		L->d_blocks[sn] = NULL;
  }
}


taucs_ccs_matrix*
taucs_dtl(supernodal_factor_ldlt_to_ccs)(void* vL)
{
  supernodal_factor_matrix_ldlt* L = (supernodal_factor_matrix_ldlt*) vL;
  taucs_ccs_matrix* C;
  int n,nnz;
  int i,j,ip,jp,sn,next,db_size;
  taucs_datatype v;
  int* len;

  n = L->n;

  len = (int*) taucs_malloc(n*sizeof(int));
  if (!len) return NULL;

  nnz = n;	/* n nonzeros on the diagonal */

	for	(sn=0; sn<L->n_sn; sn++) {
		for	(jp=0; jp<(L->sn_size)[sn];	jp++)	{
			j	=	(L->sn_struct)[sn][jp];
			len[j] = 1; /* changed from 0 to 1, to include the diagonal 1 entry */

			/* find the beginning of the non diagonal entries */
			db_size = (L->db_size)[sn][jp];

			for	(ip=jp+db_size;	ip<(L->sn_size)[sn]; ip++) {
				i	=	(L->sn_struct)[sn][	ip ];
				v	=	(L->sn_blocks)[sn][	jp*((L->sn_blocks_ld)[sn]) + ip	];

				if (taucs_re(v)	|| taucs_im(v))	{
					len[j] ++;
					nnz	++;
				}
      }
      for (ip=(L->sn_size)[sn]; ip<(L->sn_up_size)[sn]; ip++) {
				i = (L->sn_struct)[sn][ ip ];
				v = (L->up_blocks)[sn][ jp*(L->up_blocks_ld)[sn] + 
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

			/* put diagonal 1 in beginning of each column */
			i = (L->sn_struct)[sn][ jp ];
			v = taucs_one_const;

			(C->rowind)[next] = i;
			(C->taucs_values)[next] = v;
			next++;

			/* skip the diagonal block numbers */
			db_size = (L->db_size)[sn][jp];

			for (ip=jp+db_size; ip<(L->sn_size)[sn]; ip++) {
				i = (L->sn_struct)[sn][ ip ];
				v = (L->sn_blocks)[sn][ jp*(L->sn_blocks_ld)[sn] + ip ];

				if (!taucs_re(v) && !taucs_im(v)) continue;
				/*if (v == 0.0) continue;*/

				(C->rowind)[next] = i;
				(C->taucs_values)[next] = v;
				next++;
			}
      for (ip=(L->sn_size)[sn]; ip<(L->sn_up_size)[sn]; ip++) {
				i = (L->sn_struct)[sn][ ip ];
				v = (L->up_blocks)[sn][ jp*(L->up_blocks_ld)[sn] + (ip-(L->sn_size)[sn]) 
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

/* just get the diagonal of a supernodal factor, for Penny */

taucs_ccs_matrix*
taucs_dtl(supernodal_factor_diagonal_to_ccs)(void* vL)
{
  supernodal_factor_matrix_ldlt* L = (supernodal_factor_matrix_ldlt*) vL;
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
}

#if defined(TAUCS_CORE_DOUBLE) || defined(TAUCS_CORE_SINGLE)
static void
recursive_inertia_calc(int sn,       /* this supernode */
				 int is_root,  /* is v the root? */
			     int* first_child, int* next_child,
			     int** sn_struct, int* sn_sizes,
				 taucs_datatype* diagonal[], int ** db_size,
                 int *inertia)
{
   int child;
   int  sn_size; /* number of rows/columns in the supernode    */

   taucs_datatype D[4];

   int i;
	 
   for (child = first_child[sn]; child != -1; child = next_child[child]) {
  	   recursive_inertia_calc(child,
				  FALSE,
				  first_child,next_child,
				  sn_struct,sn_sizes,
				  diagonal, db_size,
				  inertia);
   }

   if(!is_root) {

     sn_size = sn_sizes[sn];


	 for (i=0; i<sn_size; i++)
	 {
		 if ( db_size[sn][i] == 1 ) {/* a 1*1 block x = b/Aii */

			 D[0] = diagonal[sn][2*i];
             if(D[0] > 0)
                inertia[1]++;
             else if(D[0] < 0)
                inertia[2]++;
             else
                inertia[0]++;
		 }

		 else if ( db_size[sn][i] == 2 ) /* a 2*2 block */
		 {
			 D[0] = diagonal[sn][2*i];
			 D[1] = D[2] = diagonal[sn][2*i+1];
			 D[3] = diagonal[sn][2*i+2];

             if( D[0] >= -D[3] ){
                inertia[1]++;
                if( taucs_mul(D[0],D[3]) > taucs_mul(D[1],D[1]) )
                   inertia[1]++;
                else
                   inertia[2]++;
             }
             else{
                inertia[2]++;
                if( taucs_mul(D[0],D[3]) > taucs_mul(D[1],D[1]) )
                   inertia[2]++;
                else
                   inertia[1]++;
             }

			 i++;
		 }
		 else {/* shouldn't get here */
			 taucs_printf("db_size[%d][%d] = %d\n",sn,i,db_size[sn][i] );
  		 	 assert(0);
         }
      }
   }
}

void	taucs_dtl(inertia_calc)(void* vL, int* inertia)
{
	supernodal_factor_matrix_ldlt* L = (supernodal_factor_matrix_ldlt*) vL;
	recursive_inertia_calc(L->n_sn,TRUE,L->first_child,L->next_child,
												 L->sn_struct,L->sn_size,L->d_blocks,L->db_size,
												 inertia);
}
#endif

void taucs_dtl(get_statistics)( int* pbytes,
																double* pflops,
																double* pnnz,
																void* vL)
{
  double nnz   = 0.0;
  double flops = 0.0;
  int sn,i,colnnz;
  int bytes;

	supernodal_factor_matrix_ldlt* L = (supernodal_factor_matrix_ldlt*) vL;
  
  bytes =
    1*sizeof(char)                /* uplo             */
    + 2*sizeof(int)               /* n, n_sn          */
    + 3*(L->n_sn)*sizeof(int)     /* etree            */
    + 5*(L->n_sn)*sizeof(int)     /* block sizes, lda */
    + 2*(L->n_sn)*sizeof(int*)    /* row/col indices , db_size */
    + 3*(L->n_sn)*sizeof(taucs_datatype*) /* actual blocks    */
    ;
  
  for (sn=0; sn<(L->n_sn); sn++) {
    bytes += (L->sn_up_size)[sn] * sizeof(int);
    bytes += (L->sn_size)[sn] * sizeof(int); /* db_size */
    bytes += ((L->sn_size)[sn]*(L->sn_up_size)[sn]) * sizeof(taucs_datatype);
    bytes += ((L->sn_size)[sn]* 2 * sizeof(taucs_datatype));/* d_blocks*/
    
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
  *pbytes = bytes;
  *pflops = flops;
  *pnnz = nnz;
}

static int
build_LU_from_D( taucs_datatype D[],
		 taucs_datatype L[],
		 taucs_datatype U[] )
{
  /* D= [0] [1]		D=LU where L= 1			0 and U= [0]	[1]
     [2] [3]									[2/0]	1				 0		[3-(1*2)/0] */
  
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
block
from the LDLt factorization. x and b are vectors of size 2. (or their 
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

		/* D= [0] [1]		D=LU where L= 1			0 and U= [0]	[1]
					[2] [3]									[2/0]	1				 0		[3-(1*2)/0] */

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
void writeSNLToFile(supernodal_factor_matrix_ldlt* snL,int sn, char * str)
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

}


static void writeToFile(supernodal_frontal_matrix_ldlt* mtr,int sn, char * str)
{
	FILE * fF1, *fF2, *fU, *fD, *fst;
	char sF1[55], sF2[55], sU[55], sD[55], sst[55];
	int i;
	taucs_datatype v;

	return;

	sprintf( sF1, "../../results/%d_F1_%s.ijv",sn,str);
	sprintf( sF2, "../../results/%d_F2_%s.ijv",sn,str);
	sprintf( sU, "../../results/%d_U_%s.ijv",sn,str);
	sprintf( sD, "../../results/%d_D_%s.ijv",sn,str);
	sprintf( sst, "../../results/%d_st_%s.ijv",sn,str);
	fF1 = fopen(sF1,"w");
	fF2 = fopen(sF2,"w");
	fU = fopen(sU,"w");
	fD = fopen(sD,"w");
	fst = fopen(sst,"w");

	if ( !fF1 || !fF2 || !fU || !fD || !fst)
		return;

	writeMatToFile(mtr->SFM_F1,mtr->sn_size,mtr->sn_size,fF1);
	writeMatToFile(mtr->SFM_F2,mtr->up_size,mtr->sn_size,fF2);
	writeMatToFile(mtr->SFM_U,mtr->up_size,mtr->up_size,fU);
	for (i=0;i<mtr->sn_size;i++) {
		v = mtr->SFM_D[i*2];
#ifdef TAUCS_CORE_DOUBLE
		fprintf(fD,"%d %d %1.6e\n",i+1,i+1,v);
		if ( i+1 < mtr->sn_size ) {
			v = mtr->SFM_D[i*2 + 1];
			fprintf(fD,"%d %d %1.6e\n",i+2,i+1,v);
			fprintf(fD,"%d %d %1.6e\n",i+1,i+2,v);
		}
#endif
	}
	for (i=0;i<mtr->sn_size;i++) {
		fprintf(fst,"%d ",mtr->sn_vertices[i]);
	}
	fprintf(fst,"|");
	for (i=0;i<mtr->up_size;i++) {
		fprintf(fst,"%d ",mtr->up_vertices[i]);
	}
	fclose(fF1);
	fclose(fF2);
	fclose(fU);
	fclose(fD);
	fclose(fst);

}

static void writeFactorFacts( supernodal_factor_matrix_ldlt* L, char * str )
{
	FILE * f;
	char s[40];
	int i,j;

	sprintf( s, "./results/L_%s.ijv",str);
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
		if (L->sn_up_size[i]) {
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




/*************************************************************/
/* create and free frontal matrices                          */
/*************************************************************/

static supernodal_frontal_matrix_ldlt*
supernodal_frontal_ldlt_create(int* firstcol_in_supernode,
			  int sn_size,
			  int n,
			  int* rowind)
{
  supernodal_frontal_matrix_ldlt* tmp;

  tmp =
		(supernodal_frontal_matrix_ldlt*)taucs_malloc(sizeof(supernodal_frontal_matrix_ldlt));
  if(tmp==NULL) return NULL;

  tmp->rowind = rowind;

  tmp->n = n;
  tmp->sn_size = sn_size;
  tmp->up_size = n-sn_size;

  tmp->sn_vertices = rowind;
  tmp->up_vertices = rowind + sn_size;

	/* on some platforms, malloc(0) fails, so we avoid such calls */

  tmp->SFM_F1 = tmp->SFM_F2 = tmp->SFM_U = NULL;
	tmp->SFM_D = NULL;
	tmp->db_size = NULL;

  if (tmp->sn_size) {
    tmp->SFM_F1 =
			(taucs_datatype*)taucs_calloc((tmp->sn_size)*(tmp->sn_size),sizeof(taucs_datatype));
		tmp->db_size = (int*)taucs_calloc(tmp->sn_size,sizeof(int));
		tmp->d = 
(taucs_datatype*)taucs_calloc((tmp->sn_size)*2,sizeof(taucs_datatype));
	}

  if (tmp->sn_size && tmp->up_size)
    tmp->SFM_F2 =
			(taucs_datatype*)taucs_calloc((tmp->up_size)*(tmp->sn_size),sizeof(taucs_datatype));

  if (tmp->up_size)
    tmp->SFM_U  =
			(taucs_datatype*)taucs_calloc((tmp->up_size)*(tmp->up_size),sizeof(taucs_datatype));


  if ((   tmp->SFM_F1==NULL && tmp->sn_size)
			|| (tmp->SFM_F2==NULL && tmp->sn_size && tmp->up_size)
			|| (tmp->SFM_U ==NULL && tmp->up_size)
			|| (tmp->SFM_D == NULL && tmp->sn_size)
			|| (tmp->db_size == NULL && tmp->sn_size)) {
    taucs_free(tmp->SFM_U);
    taucs_free(tmp->SFM_F1);
    taucs_free(tmp->SFM_F2);
		taucs_free(tmp->SFM_D);
		taucs_free(tmp->db_size);
    taucs_free(tmp);
    return NULL;
  }

  assert(tmp);
  return tmp;
}

static int
supernodal_frontal_ldlt_modify( supernodal_frontal_matrix_ldlt* mtr,
															  supernodal_factor_matrix_ldlt* snL,
																int sn )
{
	int sn_size = snL->sn_size[sn];
	int i,j;
	taucs_datatype* temp;

	assert( mtr->sn_size <= sn_size );

	if ( mtr->sn_size != sn_size ) {

		if ( mtr->SFM_F1 ) {
			temp = (taucs_datatype*)taucs_calloc(sn_size*sn_size,(sizeof(taucs_datatype)));
			if (!temp) return -1;
			for (	i=0;i<mtr->sn_size;i++ )
				for ( j=0; j<mtr->sn_size; j++ )
					temp[i*sn_size+j] = mtr->SFM_F1[i*(mtr->sn_size)+j];

			taucs_free(mtr->SFM_F1);
			mtr->SFM_F1 = temp;
		}

		if ( mtr->SFM_F2 ) {
			mtr->SFM_F2 = (taucs_datatype*)taucs_realloc(mtr->SFM_F2,
						sn_size*(mtr->up_size)*(sizeof(taucs_datatype)));
			if (!mtr->SFM_F2) return -1;
			for (	i=(mtr->sn_size)*(mtr->up_size);
						i<sn_size*(mtr->up_size);
						i++ )
				mtr->SFM_F2[i] = taucs_zero_const;
		}

		mtr->SFM_D = (taucs_datatype*)taucs_realloc(mtr->SFM_D,
					sn_size*2*(sizeof(taucs_datatype)));
		if (!mtr->SFM_D) return -1;
		for (	i=(mtr->sn_size)*2;
					i<sn_size*2;
					i++ )
			mtr->SFM_D[i] = taucs_zero_const;

		mtr->db_size = (int*)taucs_realloc(mtr->db_size,
					sn_size*(sizeof(int)));
		if (!mtr->db_size) return -1;

		mtr->rowind = snL->sn_struct[sn];
		mtr->sn_vertices = mtr->rowind;
		mtr->up_vertices = mtr->rowind + sn_size;

		mtr->sn_size = sn_size;
		mtr->n = sn_size + mtr->up_size;
	}
	return 0;
}

static void supernodal_frontal_free(supernodal_frontal_matrix_ldlt* to_del)
{
  /*
     SFM_F1 and SFM_F2 are moved to the factor,
     but this function may be called before they are
     moved.
  */


  if (to_del) {
    taucs_free(to_del->SFM_F1);
    taucs_free(to_del->SFM_F2);
    taucs_free(to_del->SFM_U);
		if (to_del->db_size) taucs_free(to_del->db_size);
		if (to_del->SFM_D) taucs_free(to_del->SFM_D);
    taucs_free(to_del);
  }
}

/*************************************************************/
/* factor a frontal matrix                                   */
/*************************************************************/

static int
mf_sn_front_reorder_ldlt(int sn,
												supernodal_frontal_matrix_ldlt* mtr,
												supernodal_factor_matrix_ldlt* snL,
												taucs_datatype* F2D11,
												supernodal_ldlt_factor* fct,
												int parent,
												int new_sn_size,
												int new_up_size)
{
	int INFO = 0, old_sn_size;
	int *rejected_cols = NULL;
	int i,j,k,rej_size;
	taucs_datatype * tempF2 = NULL,*temp = NULL;
	int* db_size_temp = NULL,*current_struct = NULL;

	/* if the actual size is smaller than allocated, we need
	to reallocate in the correct size */
	rej_size = fct->rejected_size;
	old_sn_size = mtr->sn_size;

	if ( new_sn_size < old_sn_size ) {
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
				db_size_temp[i] = mtr->db_size[i];
				temp[i*2] = mtr->SFM_D[i*2];
				temp[i*2+1] = mtr->SFM_D[i*2+1];
			}
		}
		taucs_free(mtr->SFM_D);
		taucs_free(mtr->db_size);
		mtr->SFM_D = temp;
		mtr->db_size= db_size_temp;

		/* prepare memory for new sized SFM_F2 */
		if ( new_sn_size && new_up_size ) {
			tempF2 = (taucs_datatype*)taucs_calloc(new_sn_size*new_up_size,sizeof(taucs_datatype));
			if ( !tempF2 ) {
				taucs_printf("Could not allocate memory (for diagonal blocks) %d\n",new_sn_size*new_up_size);
				return -1;
			}
		}	else {
			tempF2 = NULL;
		}
	} else { /* no need for new memory for SFM_F2 */
		tempF2 = NULL;
	}

	if ( !rej_size ) { /* no rejected columns */
		/* simply copy from SFM_F2 to F2D11 */
		for (i=0; i<new_sn_size && mtr->up_size ; i++)
			for (j=0; j<mtr->up_size; j++) {
				F2D11[i*(mtr->up_size)+j] = mtr->SFM_F2[i*(mtr->up_size)+j];
				if ( tempF2 )
					tempF2[i*(mtr->up_size)+j] = mtr->SFM_F2[i*(mtr->up_size)+j];
			}
	} else {
		rejected_cols = (int*)taucs_malloc(rej_size*sizeof(int));
		if ( !rejected_cols ) return -1;
		/* copy from F2 to temp and F2D11 */
		for (i=0; i<new_sn_size && new_up_size; i++) {
			for (j=0; j<mtr->up_size; j++) {
				F2D11[i*(new_up_size)+j] =
					mtr->SFM_F2[i*(mtr->up_size)+j];
				if ( tempF2 )
					tempF2[i*(new_up_size)+j] = mtr->SFM_F2[i*(mtr->up_size)+j];
			}
		}
		/* copy from F1 the rejected columns to temp and F2D11 */
		for (i=0; i<new_sn_size && new_up_size; i++) {
			for (j=new_sn_size; j<old_sn_size; j++) {
				k = j + mtr->up_size - new_sn_size;
				F2D11[i*(new_up_size)+k] =
					mtr->SFM_F1[i*old_sn_size+j];
				if ( tempF2 )
					tempF2[i*(new_up_size)+k] = mtr->SFM_F1[i*old_sn_size+j];
			}
		}
		/* put the rejected columns in the array */
		for (i=0; i<rej_size; i++)
			rejected_cols[i] = i+new_sn_size;

		/* now we enlarge SFM_U and put the new data in it */
		temp = (taucs_datatype*)taucs_calloc(new_up_size*new_up_size,sizeof(taucs_datatype));
		if ( !temp ) return -1;
		/* copy from SFM_U */
		for (i=0; i<mtr->up_size; i++)
			for (j=0; j<mtr->up_size; j++)
				temp[i*new_up_size+j] = mtr->SFM_U[i*(mtr->up_size)+j];

		/* copy from F1 */
		for (i=0; i<rej_size; i++)
			for (j=i; j<rej_size; j++)
				temp[(i+mtr->up_size)*new_up_size + j+mtr->up_size] =
					mtr->SFM_F1[(i+new_sn_size)*old_sn_size + j+new_sn_size];

		/* copy from F2 */
		for (i=0; i<rej_size; i++)
			for (j=0; j<mtr->up_size; j++)
				temp[j*new_up_size + i+mtr->up_size] =
					mtr->SFM_F2[(i+new_sn_size)*(mtr->up_size) + j];

		taucs_free(mtr->SFM_U);
		mtr->SFM_U = temp;

		if ( tempF2 || !new_sn_size ) {
			taucs_free(mtr->SFM_F2);
			mtr->SFM_F2 = tempF2;
		}

		/* copying the structure of the parent to a new bigger struct */
		snL->sn_struct[parent] = (int*)taucs_realloc(snL->sn_struct[parent],
				(rej_size+snL->sn_up_size[parent])*sizeof(int));
		if ( !snL->sn_struct[parent] ) {
			taucs_printf("Could not allocate memory (for structure) %d\n",(fct->rejected_size) + snL->sn_size[parent]);
			return -1;
		}
		k = snL->sn_up_size[parent] - snL->sn_size[parent];
		for (i=k-1; i>=0; i--)
			snL->sn_struct[parent][i+snL->sn_size[parent]+rej_size] =
				snL->sn_struct[parent][i+snL->sn_size[parent]];
		for (i=0; i<rej_size; i++)
			snL->sn_struct[parent][i+snL->sn_size[parent]] =
				snL->sn_struct[sn][fct->indices[rejected_cols[i]]];

		snL->sn_size[parent]+=rej_size;
		snL->sn_up_size[parent]+=rej_size;
		snL->sn_blocks_ld[parent]+=rej_size;
		/*snL->up_blocks_ld[parent]+=rej_size;*/
	}

	/* rebuild the structure of the current supernode */
	current_struct = (int*)taucs_malloc((mtr->n)*sizeof(int));
	if ( !current_struct ) {
		taucs_printf("Could not allocate memory (for structure) %d\n",mtr->n);
		return -1;
	}

	for (i=0; i<new_sn_size; i++)
		current_struct[i] = snL->sn_struct[sn][fct->indices[i]];
	for (i=0; i<mtr->up_size; i++)
		current_struct[i+new_sn_size] = snL->sn_struct[sn][i+old_sn_size];
	for (i=0; i<rej_size; i++ )
		current_struct[i+mtr->up_size+new_sn_size] =
			snL->sn_struct[sn][fct->indices[rejected_cols[i]]];

	/* changes to mtr needed only if there are size changes */
	mtr->sn_size = new_sn_size;
	mtr->up_size = new_up_size;

	taucs_free(snL->sn_struct[sn]);
	snL->sn_struct[sn] = current_struct;

	/* changes to the mtr
	currently i think we do not need to put the rejected cols in
	the update matrix, just move them to the parent*/
	mtr->rowind = snL->sn_struct[sn];
	mtr->sn_vertices = mtr->rowind;
	mtr->up_vertices = mtr->rowind + new_sn_size;

	if ( new_sn_size < old_sn_size ) {
		if ( new_sn_size ) {
			temp = 
(taucs_datatype*)taucs_calloc(new_sn_size*new_sn_size,sizeof(taucs_datatype));
			if (!temp) return -1;
			for (i=0; i<new_sn_size; i++) {
				for (j=0; j<new_sn_size; j++) {
					temp[i*new_sn_size+j] = mtr->SFM_F1[i*old_sn_size+j];
				}
			}
		} else {
			temp = NULL;
		}

		taucs_free(mtr->SFM_F1);
		mtr->SFM_F1 = temp;
	}

	/* end of changes to mtr */

	if (rejected_cols)
		taucs_free(rejected_cols);

	return INFO;
}

static int
supernodal_find_max_new(taucs_datatype* data,
												int size,
												int incremental,
												int* index,
												taucs_real_datatype* max)
{
	int ret = 0;
	assert(data != NULL);

	if ( size <= 0 ) {
		*index = size-1;
		*max = taucs_real_zero_const;
		return 1;
	}

	*index = taucs_iamax(&size,data,&incremental);
	*index -= 1;
	*max = taucs_abs(data[(*index)*incremental]);
	return ret;
}

static int
supernodal_find_max(taucs_datatype * column,
							 int size,
							 int start,
							 int * index,
							 taucs_real_datatype * max)
{
	int ret = 0;
	
	assert(column != NULL);

	/*
	if ( start >= size ) {
		*index = size - 1;
		*max = taucs_real_zero_const;
		return 1;
	}
	

	*index = start;
	*max = taucs_abs(column[*index]);

	for (i=start+1; i<size; i++) {
		if (taucs_abs(column[i]) > *max) {
			*index = i;
			*max = taucs_abs(column[i]);
		}
	}
	*/

	supernodal_find_max_new(&(column[start]),size-start,1,index,max); 
	*index += start;
	/*supernodal_find_max_new(&(column[start]),size-start,1,&test1,&test2);
	assert(test1+start == *index);
	assert(test2 == *max);*/
	
	return ret;
}

static int
supernodal_find_row_max(taucs_datatype * row,
									int size,
									int loop,
									int start,
									int * index,
									taucs_real_datatype * max)
{
	int ret = 0;
	
	assert(row != NULL);

	/*
	if ( start >= loop ) {
		*index = loop - 1;
		*max = taucs_real_zero_const;
		return 1;
	}

	
	*index = start;*/
	/**max = taucs_abs(row[*index]);*/
	/**max = taucs_abs(row[start*size]);

	for (i=start+1; i<loop; i++) {
		if (taucs_abs(row[i*size]) > *max) {
			*index = i;
			*max = taucs_abs(row[i*size]);
		}
	}
	*/

	supernodal_find_max_new(&(row[start*size]),loop-start,size,index,max);
	*index += start;
	/*supernodal_find_max_new(&(row[start*size]),loop-start,size,&test1,&test2);
	assert(test1+start == *index);
	assert(test2 == *max);*/

	return ret;
}


static int
supernodal_find_pivot( int* pivot,
											int* secondpivot,
											int k,
											int sn_size,
											int LDF1,
											int up_size,
											taucs_datatype* F1,
											taucs_datatype* F2)
{
	int INFO = 0;
	int max_k1Ind, max_k2Ind, max_q1Ind, max_q2Ind;
	taucs_real_datatype max_k1, max_k2 = taucs_real_zero_const, max_k;
	taucs_real_datatype max_q1, max_q2 = taucs_real_zero_const, max_q;
	taucs_real_datatype akk, aqq, aq1;

	/* defualt - hasn't found */
	*secondpivot = -1;
	*pivot = -1;

	akk = taucs_abs(F1[k*LDF1+k]);

	assert( sn_size );

	/* find max of column k in SFM_F1 and SFM_F2 */
	supernodal_find_max( &(F1[k*LDF1]), sn_size, k+1, &max_k1Ind, &max_k1);
	if ( up_size )
		supernodal_find_max( &(F2[k*up_size]), up_size, 0, &max_k2Ind, &max_k2);

	max_k = max(max_k1,max_k2);

	if ( taucs_is_zero_real(max_k)) {
		if ( taucs_is_zero_real(akk)) {
			/* if max = 0 and akk = 0 then this is a singular matrix */
			return k + 1;
		}	else {
			*pivot = -2; /* no need to do anything for this column */
			return INFO;
		}
	}

	/* check if akk can be a pivot */
	if ( akk >= gl_alpha*max_k ) {
		*pivot = k;
		return INFO;
	}

	/* find max of column q (maxF1Ind) */
	/*supernodal_find_row_max( &(F1[max_k1Ind]), sn_size, max_k1Ind, 0, &max_q1Ind, &max_q1);*/
	supernodal_find_row_max( &(F1[max_k1Ind]), sn_size, max_k1Ind+1, k+1, &max_q1Ind, &max_q1);
	supernodal_find_max( &(F1[max_k1Ind*LDF1]), sn_size, max_k1Ind+1, &max_q2Ind, &max_q2);
	if ( max_q1 < max_q2 ) {
		max_q1 = max_q2;
		max_q1Ind = max_q2Ind;
	}
	if ( up_size )
		supernodal_find_max( &(F2[max_k1Ind*up_size]), up_size, 0, &max_q2Ind, &max_q2);

	/* check that q is not k
	if ( k != max_q1Ind )
		return INFO;*/

	/* check if aqq can be a pivot */
	aqq = taucs_abs(F1[max_k1Ind*LDF1+max_k1Ind]);
	/* the max in q column inside F1 can be the F1(q,k) we found */
	max_q1 = max(max_q1, max_k1);
	max_q = max(max_q1, max_q2);
	if ( aqq >= gl_alpha*max_q ) {
		*pivot = max_k1Ind;
		return INFO;
	}
	/* check if akk, aqq can be 2*2 pivot */
	aq1 = taucs_abs(F1[k*LDF1+max_k1Ind]);

	if ( aq1 == taucs_real_zero_const ) /* no chance for 2*2 */
		return INFO;

	if (  fabs(max( aqq*max_k + aq1*max_q, akk*max_q + aq1*max_k ))  <=
				( fabs(akk * aqq - (aq1*aq1) /  gl_alpha ) ))  {
		*pivot = k;
		*secondpivot = max_k1Ind;
		return INFO;
	}
	return INFO;
}

static int
blocked_find_pivot( int* pivot,
										int* secondpivot,
										int k,
										int sn_size,
										int up_size,
										taucs_datatype* F1,
										taucs_datatype* F2,
										taucs_datatype* W1,
										taucs_datatype* W2,
										int size_remained,
										int index)
{
	int INFO = 0;
	int max_k1Ind, max_k2Ind, max_q1Ind, max_q2Ind;
	int vector_size;
	taucs_real_datatype max_k1, max_k2 = taucs_real_zero_const, max_k;
	taucs_real_datatype max_q1, max_q2 = taucs_real_zero_const, max_q;
	taucs_real_datatype akk, aqq, aq1;
	int one=1;
	int i = k - index;
	taucs_datatype temp;

	/* defualt - hasn't found */
	*secondpivot = -1;
	*pivot = -1;

	akk = taucs_abs(W1[i*sn_size+i]);

	/* find max of column i (k) in W1 and W2 */
	supernodal_find_max( &(W1[i*sn_size]), size_remained, i+1, &max_k1Ind, &max_k1);
	if ( up_size )
		supernodal_find_max( &(W2[i*up_size]), up_size, 0, &max_k2Ind, &max_k2);

	max_k = max(max_k1,max_k2);

	if ( taucs_is_zero_real(max_k)) {
		if ( taucs_is_zero_real(akk)) {
			/* if max = 0 and akk = 0 then this is a singular matrix */
			return k + 1;
		}	else {
			*pivot = -2; /* no need to do anything for this column */
			return INFO;
		}
	}

	/* check if akk can be a pivot */
	if ( akk >= gl_alpha*max_k ) {
		*pivot = k;
		return INFO;
	}

	/* copy column max_k1Ind of F1 to column i+1 of W1 and update */
	vector_size = max_k1Ind-i;
	taucs_copy( &vector_size, &(F1[k*sn_size+max_k1Ind+index]), &sn_size,
							&(W1[(i+1)*sn_size+i]), &one );
	/*CALL DCOPY( IMAX-K, A( IMAX, K ), LDA, W( K, K+1 ), 1 )*/
	vector_size = size_remained-max_k1Ind;
	taucs_copy( &vector_size, &(F1[(max_k1Ind+index)*sn_size+max_k1Ind+index]), &one,
							&(W1[(i+1)*sn_size+max_k1Ind]), &one );
	/*CALL DCOPY( N-IMAX+1, A( IMAX, IMAX ), 1, W( IMAX, K+1 ),1)*/
	vector_size = size_remained-i;
	taucs_gemv( "No transpose", &vector_size,
							&i, &taucs_minusone_const, &(F1[index*sn_size+k]),
							&sn_size, &(W1[max_k1Ind]), &sn_size,
							&taucs_one_const, &(W1[(i+1)*sn_size+i]), &one );
	/*CALL DGEMV( 'No transpose', N-K+1, K-1, -ONE, A( K, 1 ),
                          LDA, W( IMAX, 1 ), LDW, ONE, W( K, K+1 ), 1 )*/

  /* copy column max_k1Ind of F2 to column i+1 of W2 and update */
	if ( up_size > 0 ) {
		taucs_copy( &up_size, &(F2[(max_k1Ind+index)*up_size]), &one,
								&(W2[(i+1)*up_size]), &one );
		taucs_gemv( "No transpose", &up_size,
								&i, &taucs_minusone_const, &F2[index*up_size],
								&up_size, &(W1[max_k1Ind]), &sn_size,
								&taucs_one_const, &(W2[(i+1)*up_size]), &one );
	}

	/* find max of column i+1 (q) (maxF1Ind) */
	supernodal_find_max( &(W1[(i+1)*sn_size]), size_remained, i, &max_q1Ind, &max_q1);
	if ( up_size )
		supernodal_find_max( &(W2[(i+1)*up_size]), up_size, 0, &max_q2Ind, &max_q2);

	/* check if aqq can be a pivot */
	aqq = taucs_abs(W1[(i+1)*sn_size+max_k1Ind]);

	/* the max in q column inside F1 can be the F1(q,k) we found */
	max_q1 = max(max_q1, max_k1);
	max_q = max(max_q1, max_q2);
	if ( aqq >= gl_alpha*max_q ) {
		*pivot = max_k1Ind + index;

		/* we need to copy from W[i+1] to W[i] 
		in the correct order */
		vector_size = size_remained-i;
		taucs_copy(&vector_size, &(W1[(i+1)*sn_size+i]), &one,
								 &(W1[i*sn_size+i]), &one);
		taucs_copy(&up_size, &(W2[(i+1)*up_size]), &one,
								&(W2[i*up_size]), &one );
		/* switch between the value in i and in max_k1Ind*/
		temp = W1[i*sn_size+i];
		W1[i*sn_size+i] = W1[i*sn_size+max_k1Ind];
		W1[i*sn_size+max_k1Ind] = temp;

		return INFO;
	}

	/* check if akk, aqq can be 2*2 pivot */
	aq1 = taucs_abs(W1[i*sn_size+max_k1Ind]);

	if ( aq1 == taucs_real_zero_const ) /* no chance for 2*2 */
		return INFO;

	if ( max( aqq*max_k + aq1*max_q, akk*max_q + aq1*max_k )  <=
				( fabs(akk * aqq - (aq1*aq1) /  gl_alpha ) ) ) {
		*pivot = k;
		*secondpivot = max_k1Ind + index;

		/* switch the values in W1[i+1] between i+1 and max_k1Ind*/
		temp = W1[(i+1)*sn_size+i+1];
		W1[(i+1)*sn_size+i+1] = W1[(i+1)*sn_size+max_k1Ind];
		W1[(i+1)*sn_size+max_k1Ind] = temp;
		
		/* switch the values in W1[i] between i+1 and max_k1Ind*/
		/*done outside by the swap
		aq1 = W1[i*sn_size+i+1];
		W1[i*sn_size+i+1] = W1[i*sn_size+max_k1Ind];
		W1[i*sn_size+max_k1Ind] = aq1;*/

		return INFO;
	}
	
	return INFO;
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

static void
supernodal_columns_copy(taucs_datatype* F1,
												taucs_datatype* F2,
												int sn_size,
												int up_size,
												int i,
												int pivot )
{
	int size,j;

	assert(i<pivot);

	/* copy column i to pivot */
	/* we have 4 parts - copy the diagonal,
	both going in rows, then one row the other column 
	and then both columns. */

	F1[pivot*sn_size+pivot] = F1[i*sn_size+i];
	
	/* we do this outside */
	/*for (j=0; j<i; j++) {
		F1[j*sn_size+pivot] = F1[j*sn_size+i];
	}*/

	size = pivot-i-1;
	/*taucs_copy(&size,&(F1[i*sn_size+i+1]),&one,&(F1[(i+1)*sn_size+pivot]),&sn_size);*/
	for (j=i+1; j<pivot; j++) {
		F1[j*sn_size+pivot] = F1[i*sn_size+j];
	}
	size = sn_size-pivot-1;
	/*taucs_copy(&size,&(F1[i*sn_size+pivot+1]),&one,&(F1[pivot*sn_size+pivot+1]),&one);*/
	for (j=pivot+1; j<sn_size; j++) {
		F1[pivot*sn_size+j] = F1[i*sn_size+j];
	}
	/*taucs_copy(&up_size,&(F2[i*up_size]),&one,&(F2[pivot*up_size]),&one);*/
	for (j=0; j<up_size; j++) {
		F2[pivot*up_size+j] = F2[i*up_size+j];
	}
}


static void
supernodal_columns_switch(taucs_datatype* F1,
													taucs_datatype* F2,
													int sn_size,
													int up_size,
													int i,
													int pivot )
{
	taucs_datatype temp;
	int j;

	assert(i<pivot);

	/* switch columns */
	/* we have 4 parts - switch the diagonal,
	both going in rows, then one row the other column 
	and then both columns. */

	temp = F1[i*sn_size+i];
	F1[i*sn_size+i] = F1[pivot*sn_size+pivot];
	F1[pivot*sn_size+pivot] = temp;
	
	for (j=0; j<i; j++) {
		/* F1 */
		temp = F1[j*sn_size+i];
		F1[j*sn_size+i] = F1[j*sn_size+pivot];
		F1[j*sn_size+pivot] = temp;
	}
	for (j=i+1; j<pivot; j++) {
		/* F1 */
		temp = F1[i*sn_size+j];
		F1[i*sn_size+j] = F1[j*sn_size+pivot];
		F1[j*sn_size+pivot] = temp;
	}
	for (j=pivot+1; j<sn_size; j++) {
		/* F1 */
		temp = F1[i*sn_size+j];
		F1[i*sn_size+j] = F1[pivot*sn_size+j];
		F1[pivot*sn_size+j] = temp;
	}
	for (j=0; j<up_size; j++) {
		/* F2 */
		temp = F2[i*up_size+j];
		F2[i*up_size+j] = F2[pivot*up_size+j];
		F2[pivot*up_size+j] = temp;
	}
	/* switch rows */
	/*for (j=0; j<sn_size; j++) {
		temp = F1[j*sn_size+i];
		F1[j*sn_size+i] = F1[j*sn_size+pivot];
		F1[j*sn_size+pivot] = temp;
	}*/
	/* adjust lower triangular */
	/*for (j=i+1; j<=pivot; j++) {
		temp = F1[j*sn_size+i];
		F1[j*sn_size+i] = taucs_zero_const;
		F1[i*sn_size+j] = temp;
		temp = F1[pivot*sn_size+j];
		F1[pivot*sn_size+j] =  taucs_zero_const;
		F1[j*sn_size+pivot] = temp;
	}*/
}

static void
supernodal_indices_switch( supernodal_ldlt_factor* fct,
										 int i,
										 int pivot )
{
	int indexp, indexi, temp;

	/* switch indices */
	indexp = fct->indices[pivot];
	indexi = fct->indices[i];

	fct->indices[pivot] = indexi;
	fct->indices[i] = indexp;

	/* switch row indices */
	temp = fct->rowind[indexp]; /* its row index */
	fct->rowind[indexp] = fct->rowind[indexi];
	fct->rowind[indexi] = temp;
}


static void
supernodal_switch( taucs_datatype* F1, taucs_datatype* F2,
						 int sn_size, int up_size,
						 int i, int pivot,
						 supernodal_ldlt_factor* fct)
{
	supernodal_columns_switch(F1, F2, sn_size, up_size, i, pivot);
	supernodal_indices_switch( fct, i , pivot );
}






/*************************************************************/
/* extend-add                                                */
/*************************************************************/

static void
multifrontal_supernodal_front_extend_add(
					 supernodal_frontal_matrix_ldlt* parent_mtr,
					 supernodal_frontal_matrix_ldlt* my_mtr,
					 int* bitmap)
{
  int j,i,parent_i,parent_j;
  taucs_datatype v;

  for(i=0;i<parent_mtr->sn_size;i++) bitmap[parent_mtr->sn_vertices[i]] = i;
  for(i=0;i<parent_mtr->up_size;i++) bitmap[parent_mtr->up_vertices[i]] =
		(parent_mtr->sn_size)+i;

  /* extend add operation for update matrix */
  for(j=0;j<my_mtr->up_size;j++) {
    for(i=j;i<my_mtr->up_size;i++) {
      parent_j = bitmap[ my_mtr->up_vertices[j] ];
      parent_i = bitmap[ my_mtr->up_vertices[i] ];
      /* we could skip this if indices were sorted */
      if (parent_j>parent_i) {
				int tmp = parent_j;
				parent_j = parent_i;
				parent_i = tmp;
      }

      v = (my_mtr->SFM_U)[(my_mtr->up_size)*j+i];

      if (parent_j < parent_mtr->sn_size) {
				if (parent_i < parent_mtr->sn_size) {
					(parent_mtr->SFM_F1)[ (parent_mtr->sn_size)*parent_j + parent_i] =
						taucs_add( (parent_mtr->SFM_F1)[ (parent_mtr->sn_size)*parent_j +
							parent_i] , v );
				} else {
					(parent_mtr->SFM_F2)[ (parent_mtr->up_size)*parent_j +
					(parent_i-parent_mtr->sn_size)] =
						taucs_add( (parent_mtr->SFM_F2)[ (parent_mtr->up_size)*parent_j +
							(parent_i-parent_mtr->sn_size)] , v );
				}
			}	else {
				(parent_mtr->SFM_U)[
					(parent_mtr->up_size)*(parent_j-parent_mtr->sn_size) +
					(parent_i-parent_mtr->sn_size)] =
						taucs_add( (parent_mtr->SFM_U)[
							(parent_mtr->up_size)*(parent_j-parent_mtr->sn_size) +
							(parent_i-parent_mtr->sn_size)] , v);
			}
    }
  }
}

#endif /*#ifndef TAUCS_CORE_GENERAL*/

/*************************************************************/
/* symbolic elimination                                      */
/*************************************************************/

/* UNION FIND ROUTINES */


static int uf_find   (int* uf, int i)        { if (uf[i] != i)
                                                 uf[i] = uf_find(uf,uf[i]);
                                               return uf[i]; }
static
void recursive_postorder(int  j,
			 int  first_child[],
			 int  next_child[],
			 int  postorder[],
			 int  ipostorder[],
			 int* next)
{
  int c;
  for (c=first_child[j]; c != -1; c = next_child[c]) {
    /*printf("*** %d is child of %d\n",c,j);*/
    recursive_postorder(c,first_child,next_child,
			postorder,ipostorder,next);
  }
  /*printf(">>> j=%d next=%d\n",j,*next);*/
  if (postorder)  postorder [*next] = j;
  if (ipostorder) ipostorder[j] = *next;
  (*next)++;
}

#define GILBERT_NG_PEYTON_ANALYSIS_SUP

/* in a few tests the supernodal version seemed slower */
#undef GILBERT_NG_PEYTON_ANALYSIS_SUP


static void
tree_level(int j,
	   int isroot,
	   int first_child[],
	   int next_child[],
	   int level[],
	   int level_j)
{
  int c;
  if (!isroot) level[j] = level_j;
  for (c=first_child[j]; c != -1; c = next_child[c]) {
    tree_level(c,
	       FALSE,
	       first_child,
	       next_child,
	       level,
	       level_j+1);
  }
}

static void
tree_first_descendant(int j,
		      int isroot,
		      int first_child[],
		      int next_child[],
		      int ipostorder[],
		      int first_descendant[])
{
  int c;
  int fd = ipostorder[j];
  for (c=first_child[j]; c != -1; c = next_child[c]) {
    tree_first_descendant(c,
			  FALSE,
			  first_child,
			  next_child,
			  ipostorder,
			  first_descendant);
    if (first_descendant[c] < fd) fd = first_descendant[c];
  }
  if (!isroot) first_descendant[j] = fd;
}

int
taucs_ccs_etree(taucs_ccs_matrix* A,
		int* parent,
		int* l_colcount,
		int* l_rowcount,
		int* l_nnz);

int
taucs_ccs_etree_liu(taucs_ccs_matrix* A,
		    int* parent,
		    int* l_colcount,
		    int* l_rowcount,
		    int* l_nnz);




static int
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
			       int            rowind[],
			       int            column_to_sn_map[],
			       int            map[],
			       int            do_order,
			       int            ipostorder[]
			       )
{
  int  i,ip,c,c_sn;
  int  in_previous_sn;
  int  nnz = 0; /* just to suppress the warning */

  for (c=first_child[j]; c != -1; c = next_child[c]) {
    if (recursive_symbolic_elimination(c,A,
				       first_child,next_child,
				       n_sn,
				       sn_size,sn_up_size,sn_rowind,
				       sn_first_child,sn_next_child,
				       rowind, /* temporary */
				       column_to_sn_map,
				       map,
				       do_order,ipostorder
				       )
	== -1) return -1;
  }

  in_previous_sn = 1;
  if (j == A->n)
    in_previous_sn = 0; /* this is not a real column */
  else if (first_child[j] == -1)
    in_previous_sn = 0; /* this is a leaf */
  else if (next_child[first_child[j]] != -1)
    in_previous_sn = 0; /* more than 1 child */
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

    return 0;
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

  /*printf("children of sn %d: ",*n_sn);*/
  for (c=first_child[j]; c != -1; c = next_child[c]) {
    c_sn = column_to_sn_map[c];
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
    if (!( sn_rowind [*n_sn] )) return -1;
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

  return 0;
}

/* count zeros and nonzeros in a supernode to compute the */
/* utility of merging fundamental supernodes.             */

typedef struct {
  double zeros;
  double nonzeros;
} znz;

static znz
recursive_amalgamate_supernodes(int           sn,
				int*           n_sn,
				int            sn_size[],
				int            sn_up_size[],
				int*           sn_rowind[],
				int            sn_first_child[],
				int            sn_next_child[],
				int            rowind[],
				int            column_to_sn_map[],
				int            map[],
				int            do_order,
				int            ipostorder[]
				)
{
  int  i,ip,c_sn,gc_sn;
  /*int  i,ip,c,c_sn,gc_sn;*/
  int  nnz;
  int  nchildren /*, ichild*/; /* number of children, child index */
  znz* c_znz = NULL;
  znz  sn_znz, merged_znz;
  /*int zero_count = 0;*/
  int new_sn_size, new_sn_up_size;

  sn_znz.zeros    = 0.0;
  sn_znz.nonzeros = (double) (((sn_up_size[sn] - sn_size[sn]) * sn_size[sn])
                              + (sn_size[sn] * (sn_size[sn] + 1))/2);

  if (sn_first_child[sn] == -1) { /* leaf */
    return sn_znz;
  }

  nchildren = 0;
  for (c_sn=sn_first_child[sn]; c_sn != -1; c_sn = sn_next_child[c_sn])
    nchildren++;

  /*  c_znz = (znz*) alloca(nchildren * sizeof(znz));*/
  c_znz = (znz*) taucs_malloc(nchildren * sizeof(znz));
  assert(c_znz);

  /*printf("supernode %d out of %d\n",sn,*n_sn);*/

  /* merge the supernode with its children! */

  i = 0;
  for (c_sn=sn_first_child[sn]; c_sn != -1; c_sn = sn_next_child[c_sn]) {
    c_znz[i] =
      recursive_amalgamate_supernodes(c_sn,
				      n_sn,
				      sn_size,sn_up_size,sn_rowind,
				      sn_first_child,sn_next_child,
				      rowind, /* temporary */
				      column_to_sn_map,
				      map,
				      do_order,ipostorder
				      );
    assert(c_znz[i].zeros + c_znz[i].nonzeros ==
	   (double) (((sn_up_size[c_sn] - sn_size[c_sn]) * sn_size[c_sn])
		     + (sn_size[c_sn] * (sn_size[c_sn] + 1))/2 ));
    i++;
  }

  merged_znz.nonzeros = sn_znz.nonzeros;
  merged_znz.zeros    = sn_znz.zeros;

  for (i=0; i<nchildren; i++) {
    merged_znz.nonzeros += (c_znz[i]).nonzeros;
    merged_znz.zeros    += (c_znz[i]).zeros;
  }

  taucs_free(c_znz);

  /*  printf("supernode %d out of %d (continuing)\n",sn,*n_sn);*/

  /* should we merge the supernode with its children? */

  nnz = 0;
  for (c_sn=sn_first_child[sn]; c_sn != -1; c_sn = sn_next_child[c_sn]) {
    for (ip=0; ip<sn_size[c_sn]; ip++) {
      i = sn_rowind[c_sn][ip];
      assert( map[i] != sn );
      map[i] = sn;
      rowind[nnz] = i;
      nnz++;
    }
  }

  for (ip=0; ip<sn_size[sn]; ip++) {
    i = sn_rowind[sn][ip];
    assert( map[i] != sn );
    map[i] = sn;
    rowind[nnz] = i;
    nnz++;
  }

  new_sn_size = nnz;

  for (c_sn=sn_first_child[sn]; c_sn != -1; c_sn = sn_next_child[c_sn]) {
    for (ip=sn_size[c_sn]; ip<sn_up_size[c_sn]; ip++) {
      i = sn_rowind[c_sn][ip];
      if (map[i] != sn) { /* new row index */
	map[i] = sn;
	rowind[nnz] = i;
	nnz++;
      }
    }
  }

  for (ip=sn_size[sn]; ip<sn_up_size[sn]; ip++) {
    i = sn_rowind[sn][ip];
    if (map[i] != sn) { /* new row index */
      map[i] = sn;
      rowind[nnz] = i;
      nnz++;
    }
  }

  new_sn_up_size = nnz;

  if (do_order) {
    compare_indirect_map = ipostorder;
    qsort(rowind,nnz,sizeof(int),compare_indirect_ints);
  }

  /* determine whether we should merge the supernode and its children */

  {
    int n;
    double* zcount = NULL;

    n = 0;
    for (ip=0; ip<nnz; ip++) {
      i = rowind[ip];
      if (i >= n) n = i+1;
    }

    /*zcount = (double*) alloca(n * sizeof(double));*/
    zcount = (double*) taucs_malloc(n * sizeof(double));
    assert(zcount);

    for (ip=0; ip<new_sn_size; ip++) {
      i = rowind[ip]; assert(i<n);
      zcount[i] = (double) (ip+1);
    }
    for (ip=new_sn_size; ip<new_sn_up_size; ip++) {
      i = rowind[ip]; assert(i<n);
      zcount[i] = (double) new_sn_size;
    }

    /*
    for (ip=0; ip<new_sn_up_size; ip++)
      printf("row %d zcount = %.0f\n",rowind[ip],zcount[rowind[ip]]);
    */

    for (c_sn=sn_first_child[sn]; c_sn != -1; c_sn = sn_next_child[c_sn]) {
      for (ip=0; ip<sn_size[c_sn]; ip++) {
	i = sn_rowind[c_sn][ip]; assert(i<n);
	zcount[i] -= (double) (ip+1);
      }
      for (ip=sn_size[c_sn]; ip<sn_up_size[c_sn]; ip++) {
	i = sn_rowind[c_sn][ip]; assert(i<n);
	zcount[i] -= (double) sn_size[c_sn];
      }
    }

    for (ip=0; ip<sn_size[sn]; ip++) {
      i = sn_rowind[sn][ip]; assert(i<n);
      zcount[i] -= (double) (ip+1);
    }
    for (ip=sn_size[sn]; ip<sn_up_size[sn]; ip++) {
      i = sn_rowind[sn][ip]; assert(i<n);
      zcount[i] -= (double) sn_size[sn];
    }

    /*
    for (ip=0; ip<new_sn_up_size; ip++)
      printf("ROW %d zcount = %.0f\n",rowind[ip],zcount[rowind[ip]]);
    printf("zeros before merging %.0f\n",merged_znz.zeros);
    */

    for (ip=0; ip<new_sn_up_size; ip++) {
      i = rowind[ip]; assert(i<n);
      assert(zcount[i] >= 0.0);
      merged_znz.zeros += zcount[i];
    }

    /*printf("zeros after merging %.0f\n",merged_znz.zeros);*/

    /* voodoo constants (need some kind of a utility function */
    if ((new_sn_size < 16)
	||
	((sn_size[sn] < 50) && (merged_znz.zeros < 0.5 * merged_znz.nonzeros))
	||
	((sn_size[sn] < 250) && (merged_znz.zeros < 0.25 * merged_znz.nonzeros))
	||
	((sn_size[sn] < 500) && (merged_znz.zeros < 0.10 * merged_znz.nonzeros))
	||
	(merged_znz.zeros < 0.05 * merged_znz.nonzeros)
	) {
      /*
      taucs_printf("merging sn %d, zeros (%f) vs nonzeros (%f)\n",
		   sn,merged_znz.zeros,merged_znz.nonzeros);
      */
    } else {
      /*
      taucs_printf("sn %d, too many zeros (%f) vs nonzeros (%f)\n",
		   sn,merged_znz.zeros,merged_znz.nonzeros);
      printf("returning without merging\n");
      */
      taucs_free(zcount);
      return sn_znz;
    }

    taucs_free(zcount);
  }

  /* now merge the children lists */

  sn_size[sn]    = new_sn_size;
  sn_up_size[sn] = new_sn_up_size;
  sn_rowind[sn]  = (int*) taucs_realloc(sn_rowind[sn],
				  new_sn_up_size * sizeof(int));
  for (ip=0; ip<new_sn_up_size; ip++) sn_rowind[sn][ip] = rowind[ip];

  /*  printf("supernode %d out of %d (merging)\n",sn,*n_sn);*/

  nchildren = 0;
  for (c_sn=sn_first_child[sn]; c_sn != -1; c_sn = sn_next_child[c_sn]) {
    for (ip=0; ip<sn_size[c_sn]; ip++) {
      i = (sn_rowind[c_sn])[ip];
      assert(column_to_sn_map[i] == c_sn);
      column_to_sn_map[i] = sn;
    }

    for (gc_sn=sn_first_child[c_sn]; gc_sn != -1; gc_sn =
sn_next_child[gc_sn]) {
      rowind[nchildren] = gc_sn;
      nchildren++;
    }
  }

  /* free the children's rowind vectors */
  for (c_sn=sn_first_child[sn]; c_sn != -1; c_sn = sn_next_child[c_sn]) {
    taucs_free( sn_rowind[c_sn] );
    sn_rowind[c_sn]  = NULL;
    sn_size[c_sn]    = 0;
    sn_up_size[c_sn] = 0;
  }

  sn_first_child[sn] = -1;
  for (i=0; i<nchildren; i++) {
    sn_next_child[ rowind[i] ] = sn_first_child[sn];
    sn_first_child[sn] = rowind[i];
  }

  /*
  printf("supernode %d out of %d (done)\n",sn,*n_sn);
  printf("returning, merging\n");
  */
  return merged_znz;
}

#ifndef TAUCS_CORE_GENERAL


/*************************************************************/
/* factor routines                                           */
/*************************************************************/


static int
RL_supernodal_front_factor_ldlt(supernodal_ldlt_factor* fct,
																taucs_datatype* F1,
																taucs_datatype* F2,
																taucs_datatype* DB,
																int* db_size,
																int sn_size,
																int up_size,
																int sn)
{
	int INFO = 0,i,k,j,one,two;
	int size_left,size_L21;
	int pivot, secondpivot;
	taucs_datatype diagonal,temp,D[4],x[2],b[2],L[4],U[4];
	taucs_datatype* L21 = NULL;
	int rejected_tried = 0, need_switch = 0;
	
	for (i=0; i<sn_size; i++) {
		INFO = supernodal_find_pivot(&pivot, &secondpivot, i, sn_size, sn_size, up_size, F1, F2);
		if ( INFO ) {
			/*The matrix is singular.*/
			/*return INFO;*/
			/* we put INF in D and zeros in L and continue */
			db_size[i] = 1;
#if defined (TAUCS_CORE_DOUBLE) || defined (TAUCS_CORE_SINGLE)
			taucs_re(DB[i * 2]) = taucs_get_inf();
#endif
			F1[i * sn_size + i] = taucs_one_const;
			INFO = 0;
			taucs_printf("\t\tFound a singular column in sn=%d. Putting INf in D.\n",sn);
			continue;
		}

		/* in case we already rejected this column, yet we succeeded
		to factor it this time */
		if (pivot!=-1 && fct->rowind[fct->indices[i]] == -1) {
			fct->rowind[fct->indices[i]] = i;
			fct->rejected_size--;
		}

		if ( pivot == -2 ) {
			/* no need to do factoring for this column */
			db_size[i] = 1;
			diagonal = F1[i*sn_size + i];

			DB[i * 2] = diagonal;
			F1[i * sn_size + i] = taucs_one_const;
		}
		else if ( pivot >= 0 && secondpivot == -1 ) {
			/* a 1*1 pivot of column 'pivot' */
			if ( pivot != i )
				supernodal_switch( F1, F2, sn_size, up_size, i, pivot, fct );

			db_size[i] = 1;
			diagonal = F1[i*sn_size + i];

			DB[i * 2] = diagonal;
			F1[i * sn_size + i] = taucs_one_const;

			/* update the rest of F1 and F2 */
			size_left = sn_size - i - 1;
			one = 1;
			temp = taucs_div( taucs_minusone_const, diagonal );
			if ( size_left ) {
				/* update F1 */
				taucs_her("Lower", &size_left, &temp,
								&(F1[i*sn_size+i+1]),
								&one,
								&(F1[(i+1)*sn_size + i+1 ]),
								&sn_size);
				/* update F2 */
				if ( up_size )
					taucs_ger(&up_size, &size_left, &temp,
								&(F2[i*up_size]), &one,
								&(F1[i*sn_size+i+1]),&one,
								&(F2[(i+1)*up_size]), &up_size);
			}

			/* update the column in F1 */
			temp = taucs_div( taucs_one_const, diagonal );
			taucs_scal(&size_left,
								&temp,
								&(F1[i*sn_size+i+1]),
								&one);
			/* update the column in F2 */
			taucs_scal(&up_size,
								&temp,
								&(F2[i*up_size]),
								&one);

		}
		else  if ( pivot >= 0 ) { /* 2*2 pivot found */
			/* check that we didn't already reject the pivots,
			and if so - remove it from the rejected list */
			if ( fct->rejected_size ) {
				if ( fct->rowind[fct->indices[pivot]] == -1 ) {
          fct->rejected_size--;
					fct->rowind[fct->indices[pivot]] = pivot;
				}
				if ( fct->rowind[fct->indices[secondpivot]] == -1 ) {
					fct->rejected_size--;
					fct->rowind[fct->indices[secondpivot]] = secondpivot;
				}
			}
			if ( pivot != i )
				supernodal_switch( F1, F2, sn_size, up_size, i, pivot, fct );
			if ( secondpivot != i+1 )
				supernodal_switch( F1, F2, sn_size, up_size, i+1, secondpivot, fct );

			db_size[i] = 2;
			db_size[i+1] = 1;

			
			D[0] = F1[i * sn_size + i];
			D[1] = D[2] = F1[i * sn_size + i+1];
			D[3] = F1[(i+1) * sn_size + i+1];

			size_left = sn_size -(i+2);
			size_L21 = size_left;/* + mtr->up_size;*/

			if ( size_L21 ) {
				L21 = (taucs_datatype*)taucs_calloc(2*size_L21, sizeof(taucs_datatype));
				if (!L21) return -1;
				/* copy to L21 */
				for (k=0; k<size_L21; k++) {
					L21[k] = F1[i*sn_size+i+2+k];
					L21[(size_L21)+k] =
						F1[(i+1)*sn_size+i+2+k];
				}
				/*for (k=0; k<mtr->up_size; k++) {
					L21[size_left+k] = mtr->SFM_F2[i*(mtr->up_size)+k];
					L21[(size_L21)+size_left+k] =
						mtr->SFM_F2[(i+1)*(mtr->up_size)+k];
				}*/
			}

			DB[i*2] = D[0];
			DB[(i*2)+1] = D[1];
			DB[(i*2)+2] = D[3];
			F1[i * sn_size + i] = taucs_one_const;
			F1[i * sn_size + i+1] = taucs_zero_const;
			F1[(i+1) * sn_size + i+1] = taucs_one_const;

			need_switch = build_LU_from_D(D,L,U);


			/* factor the two columns of F1 and F2 */
			for (k=0; k<size_left; k++) {
				b[0] = F1[sn_size*i +i+2+k];
				b[1] = F1[sn_size*(i+1) +i+2+k];
				two_times_two_diagonal_block_solve(D,x,b,gl_cramer_rule,TRUE,L,U,need_switch);
				F1[i*sn_size +i+2+k] = x[0];
				F1[(i+1)*sn_size +i+2+k] = x[1];
			}
			for (k=0; k<up_size ; k++) {
				b[0] = F2[up_size*i +k];
				b[1] = F2[up_size*(i+1) +k];
				two_times_two_diagonal_block_solve(D,x,b,gl_cramer_rule,TRUE,L,U,need_switch);
				F2[i*up_size +k] = x[0];
				F2[(i+1)*up_size +k] = x[1];
			}

			two = 2;

			/* GEMM : C := A*B + C*/
			if ( size_left ) {
				taucs_gemm("No conjugate",
								"Conjugate",
								&size_left, &size_left,	&two,
								&taucs_minusone_const,
								&(F1[i*sn_size+i+2]),
								&sn_size,
								L21,&size_L21,
								&taucs_one_const,
								&(F1[(i+2)*sn_size + i+2]),&sn_size);

				if ( up_size )
					taucs_gemm("No conjugate",
								"Conjugate",
								&up_size, &size_left,	&two,
								&taucs_minusone_const,
								&(F2[i*up_size]),
								&up_size,
								L21,&size_L21,
								&taucs_one_const,
								&(F2[(i+2)*up_size]),&up_size);
			}

			/* we do not need the upper part of SFM_F1
			for it is symmetric */
			for (k=i+2; k<sn_size; k++)
				for (j=0; j<k; j++)
					F1[k*sn_size+j] = taucs_zero_const;

			if ( size_L21 )
				taucs_free(L21);

			i++;
		}
		else { /* no pivot found */
      if ( fct->rowind[fct->indices[i]] == -1 ) {
	/* this column was already rejected */
	if ( rejected_tried ) {
	  /* we already tried to factor the rejected columns
	     just finish */
	  i = sn_size;
	} else {
	  /* set all rejected as unrejected and try them again */
	  rejected_tried = 1;
	  for (j=i;j<sn_size;j++)
	    fct->rowind[fct->indices[j]] = 0;
	  fct->rejected_size = 0;
	}
	i--;
      } else {
				/* we move this column to the end of F1 */
				/* k is the last place in F1 which has not been replaced
				with a rejected column */
				k = sn_size - fct->rejected_size - 1;
				if ( i != k ) {
					supernodal_switch( F1,F2,sn_size, up_size,i,k,fct);
					fct->rowind[fct->indices[k]] = -1;
					fct->rejected_size++;
				} else {
					fct->rowind[fct->indices[i]] = -1;
					fct->rejected_size++;
				}
				i--; /* try again the new column in place i */
			}
		}
	}

	return INFO;
}

static int
block_factor(int index,
						 int* actual_block_size,
						 taucs_datatype* F1,
						 taucs_datatype* F2,
						 taucs_datatype* DB,
						 int* db_size,
						 int sn_size,
						 int up_size, 
						 supernodal_ldlt_factor* fct,
						 taucs_datatype* W1,
						 taucs_datatype* W2,
						 int* rejected_tried,
						 int sn)
{
	int size_remained = sn_size - index;
	int INFO = 0;
	int i = 0, vector_size, k,j;
	int one = 1;
	int pivot, secondpivot, need_switch;
	taucs_datatype diagonal,D[4],x[2],b[2],L[4],U[4];
	
	while ( i < gl_block_size-1 && i < size_remained ) {
		k = i+index;

		/* copy column k of F1 to column k of W1 and update */
		vector_size = size_remained - i;
		taucs_copy( &vector_size, &(F1[k*sn_size+k]), &one,
								&(W1[i*sn_size+i]), &one );
		taucs_gemv( "No transpose", &vector_size,
								&i, &taucs_minusone_const, &(F1[index*sn_size+k]),
								&sn_size, &(W1[i]), &sn_size,
								&taucs_one_const, &(W1[i*sn_size+i]), &one );
		/*   CALL DCOPY( N-K+1, A( K, K ), 1, W( K, K ), 1 )
         CALL DGEMV( 'No transpose', N-K+1, K-1, -ONE, A( K, 1 ), LDA,
                    W( K, 1 ), LDW, ONE, W( K, K ), 1 )*/


		if ( up_size > 0 ) {
			/* copy column k of F2 to column k of W2 and update */
			taucs_copy( &up_size, &(F2[k*up_size]), &one,
									&(W2[i*up_size]), &one );
			taucs_gemv( "No transpose", &up_size,
									&i, &taucs_minusone_const, &F2[index*up_size],
									&up_size, &(W1[i]), &sn_size,
									&taucs_one_const, &(W2[i*up_size]), &one );
		}

		INFO = blocked_find_pivot(&pivot,&secondpivot,k,
															sn_size,up_size,F1,
															F2,W1,W2,size_remained,index);

		if ( INFO ) {
			/*The matrix is singular.*/
			/* we put INF in D and zeros in L and continue */
			db_size[k] = 1;
#if defined (TAUCS_CORE_DOUBLE) || defined (TAUCS_CORE_SINGLE)
			taucs_re(DB[k * 2]) = taucs_get_inf();
#endif
			F1[k * sn_size + k] = taucs_one_const;
			INFO = 0;
			taucs_printf("\t\tFound a singular column in sn=%d. Putting INf in D.\n",sn);
			i++;
			continue;
		}

		/* in case we already rejected this column, yet we succeeded
		to factor it this time */
		if (pivot!=-1 && fct->rowind[fct->indices[k]] == -1) {
			fct->rowind[fct->indices[k]] = k;
			fct->rejected_size--;
		}

		if ( pivot == -2 ) {
			/* no need to do factoring for this column */
			vector_size = size_remained - i;
			taucs_copy( &vector_size, &(W1[i*sn_size+i]), &one,
									&(F1[k*sn_size+k]), &one );

			db_size[k] = 1;
			diagonal = F1[k*sn_size + k];

			DB[k * 2] = diagonal;
			F1[k * sn_size + k] = taucs_one_const;
			i++;
			continue;
		}

		if ( (secondpivot != -1 && secondpivot != k+1) ||
				 (pivot != -2 && pivot != k && pivot != -1) ) {
			/* need to put non updated column inplace of the 
			one we put into W */
			int KK = (secondpivot!=-1) ? k+1 : k;
			int KP = (secondpivot!=-1) ? secondpivot : pivot;
			int KK_index = KK - index;
			
			supernodal_columns_copy(F1,F2,sn_size,up_size,KK,KP);
			/*
			vector_size = KP-KK-1;
			taucs_copy(&vector_size, &(F1[KK*sn_size+KK+1]),
								 &one, &(F1[(KK+1)*sn_size+KP]), &sn_size);
			vector_size = size_remained-KP;
			taucs_copy(&vector_size, &(F1[KK*sn_size+KP]),
								 &one, &(F1[KP*sn_size+KP]), &one);
			F1[KK*sn_size+KP] = F1[KP*sn_size+KP];
			F1[KP*sn_size+KP] = F1[KK*sn_size+KK];
			*/
			/*
			 A( KP, K ) = A( KK, K )
       CALL DCOPY( KP-K-1, A( K+1, KK ), 1, A( KP, K+1 ), LDA )
       CALL DCOPY( N-KP+1, A( KP, KK ), 1, A( KP, KP ), 1 )
			*/

			taucs_swap(&KK, &(F1[KK]), &sn_size, &(F1[KP]), &sn_size);
			taucs_swap(&KK_index, &(W1[KK-index]), &sn_size, &(W1[KP-index]), &sn_size);
			/*
			CALL DSWAP( KK, A( KK, 1 ), LDA, A( KP, 1 ), LDA )
      CALL DSWAP( KK, W( KK, 1 ), LDW, W( KP, 1 ), LDW )
			*/
			/*
			if ( up_size > 0 ) {
				taucs_copy(&up_size, &(F2[KK*up_size]),
									&one, &(F2[KP*sn_size]), &one);
			}
			*/
			/*taucs_swap(&KK, &(F2[KK]), &up_size, &(F2[KP]), &up_size);
				taucs_swap(&KK, &(W2[KK]), &up_size, &(W2[KP]), &up_size);*/
			


		}
		
		if ( pivot >= 0 && secondpivot == -1 ) {
			
			if ( pivot != k ) {
				/* a 1*1 pivot of column 'pivot' */
				/*supernodal_switch( F1, F2, sn_size, up_size, i, pivot, fct );*/
				supernodal_indices_switch( fct, k , pivot );
			}
			
			vector_size = size_remained - i;
			taucs_copy( &vector_size, &(W1[i*sn_size+i]), &one,
									&(F1[k*sn_size+k]), &one );
			taucs_copy( &up_size, &(W2[i*up_size]), &one,
									&(F2[k*up_size]), &one );


			db_size[k] = 1;
			diagonal = F1[k*sn_size + k];

			DB[k * 2] = diagonal;
			F1[k * sn_size + k] = taucs_one_const;

			/* update the column in F1 */
			vector_size = size_remained-i-1;
			diagonal = taucs_div( taucs_one_const, diagonal );
			taucs_scal(&vector_size,
								&diagonal,
								&(F1[k*sn_size+k+1]),
								&one);
			/* update the column in F2 */
			taucs_scal(&up_size,
								&diagonal,
								&(F2[k*up_size]),
								&one);


			i++;
		}
		else  if ( pivot >= 0 ) { /* 2*2 pivot found */
			/* check that we didn't already reject the pivots,
			and if so - remove it from the rejected list */
			if ( fct->rejected_size ) {
				if ( fct->rowind[fct->indices[pivot]] == -1 ) {
          fct->rejected_size--;
					fct->rowind[fct->indices[pivot]] = pivot;
				}
				if ( fct->rowind[fct->indices[secondpivot]] == -1 ) {
					fct->rejected_size--;
					fct->rowind[fct->indices[secondpivot]] = secondpivot;
				}
			}
			if ( pivot != k )
				/*supernodal_switch( F1, F2, sn_size, up_size, i, pivot, fct );*/
				supernodal_indices_switch( fct, k , pivot );
			if ( secondpivot != k+1 )
				/*supernodal_switch( F1, F2, sn_size, up_size, i+1, secondpivot, fct );*/
				supernodal_indices_switch( fct, k+1 , secondpivot );

			vector_size = size_remained - i;
			taucs_copy( &vector_size, &(W1[i*sn_size+i]), &one,
									&(F1[k*sn_size+k]), &one );
			taucs_copy( &up_size, &(W2[i*up_size]), &one,
									&(F2[k*up_size]), &one );
			vector_size = size_remained - i - 1;
			taucs_copy( &vector_size, &(W1[(i+1)*sn_size+i+1]), &one,
									&(F1[(k+1)*sn_size+k+1]), &one );
			taucs_copy( &up_size, &(W2[(i+1)*up_size]), &one,
									&(F2[(k+1)*up_size]), &one );

			db_size[k] = 2;
			db_size[k+1] = 1;

			
			D[0] = F1[k * sn_size + k];
			D[1] = D[2] = F1[k * sn_size + k+1];
			D[3] = F1[(k+1) * sn_size + k+1];

			
			DB[k*2] = D[0];
			DB[(k*2)+1] = D[1];
			DB[(k*2)+2] = D[3];
			F1[k * sn_size + k] = taucs_one_const;
			F1[k * sn_size + k+1] = taucs_zero_const;
			F1[(k+1) * sn_size + k+1] = taucs_one_const;

			need_switch = build_LU_from_D(D,L,U);


			/* factor the two columns of F1 and F2 */
			vector_size = size_remained-i-2;
			for (j=0; j<vector_size; j++) {
				b[0] = F1[sn_size*k +k+2+j];
				b[1] = F1[sn_size*(k+1) +k+2+j];
				two_times_two_diagonal_block_solve(D,x,b,gl_cramer_rule,TRUE,L,U,need_switch);
				F1[k*sn_size +k+2+j] = x[0];
				F1[(k+1)*sn_size +k+2+j] = x[1];
			}
			for (j=0; j<up_size ; j++) {
				b[0] = F2[up_size*k +j];
				b[1] = F2[up_size*(k+1) +j];
				two_times_two_diagonal_block_solve(D,x,b,gl_cramer_rule,TRUE,L,U,need_switch);
				F2[k*up_size +j] = x[0];
				F2[(k+1)*up_size +j] = x[1];
			}

			
			i+=2;
		}
		else { /* no pivot found */
      if ( fct->rowind[fct->indices[k]] == -1 ) {
				/* this column was already rejected */
				if ( *rejected_tried ) {
					/* we already tried to factor the rejected columns
						just finish */
					*rejected_tried = 2;
					break;
				} else {
					/* set all rejected as unrejected and try them again */
					*rejected_tried = 1;
					for (j=i;j<sn_size;j++)
						fct->rowind[fct->indices[j]] = 0;
					fct->rejected_size = 0;
				}
      } else {
				/* we move this column to the end of F1 */
				/* j is the last place in F1 which has not been replaced
				with a rejected column */
				j = sn_size - fct->rejected_size - 1;
				if ( k != j ) {
					/* need to switch both in F1 and in W1 */
					int k_index = k-index;
					int j_index = j-index;
					supernodal_switch( F1,F2,sn_size, up_size,k,j,fct);
					taucs_swap(&k_index, &(W1[k_index]), &sn_size, &(W1[j_index]), &sn_size);
					fct->rowind[fct->indices[j]] = -1;
					fct->rejected_size++;
				} else {
					fct->rowind[fct->indices[k]] = -1;
					fct->rejected_size++;
				}
			}
		}
	}
	
	/* now we update the rest of the matrix, block by block */
	k = i+index;
	for (j=k; j<sn_size; j+=gl_block_size) {
		int JB = min(gl_block_size, sn_size-j);
		int JJ = j;
		int sum;
		for ( ;JJ<j+JB; JJ++) {
			sum = j+JB-JJ;
			taucs_gemv("No Transpose", &sum, &i,
								 &taucs_minusone_const, &(F1[index*sn_size+JJ]),
								 &sn_size, &(W1[JJ-index]), &sn_size, &taucs_one_const,
								 &(F1[JJ*sn_size+JJ]), &one);
			/* not needed
			if ( up_size > 0 )
				taucs_gemv("No Transpose", &sum, &i,
								 &taucs_minusone_const, F2,
								 &up_size, W2, &up_size, &taucs_one_const,
								 &(F2[JJ*up_size+JJ]), &one);*/
			/*
			      DO 100 JJ = J, J + JB - 1
               CALL DGEMV( 'No transpose', J+JB-JJ, K-1, -ONE,
     $                     A( JJ, 1 ), LDA, W( JJ, 1 ), LDW, ONE,
     $                     A( JJ, JJ ), 1 )
			*/
		}
		if ( j+JB < sn_size ) {
			sum = sn_size-j-JB;
			taucs_gemm( "No transpose", "Transpose", &sum, &JB,
									&i, &taucs_minusone_const, &(F1[index*sn_size+j+JB]),
									&sn_size, &(W1[j-index]), &sn_size, &taucs_one_const,
									&(F1[j*sn_size+j+JB]), &sn_size );
		}
		if ( up_size > 0 ) {
			taucs_gemm( "No transpose", "Transpose", &up_size, &JB,
								&i, &taucs_minusone_const, &(F2[index*up_size]),
								&up_size, &(W1[j-index]),
								&sn_size, &taucs_one_const,
								&(F2[j*up_size]), &up_size );
		}
		
		/*
		        IF( J+JB.LE.N )
     $         CALL DGEMM( 'No transpose', 'Transpose', N-J-JB+1, JB,
     $                     K-1, -ONE, A( J+JB, 1 ), LDA, W( J, 1 ), LDW,
     $                     ONE, A( J+JB, J ), LDA )
		*/
	}

	*actual_block_size = i;

	return INFO;
}

static int
unblock_factor(	int index,
		taucs_datatype* F1,
		taucs_datatype* F2,
		taucs_datatype* DB,
		int* db_size,
		int sn_size,
		int up_size, 
		supernodal_ldlt_factor* fct,
		int* rejected_tried,
		int sn)
{
  int INFO = 0,i,k,j,one,two;
  int size_left,size_L21;
  int pivot, secondpivot;
  taucs_datatype diagonal,temp,D[4],x[2],b[2],L[4],U[4];
  taucs_datatype* L21 = NULL;
  int need_switch = 0;
  
  for (i=index; i<sn_size; i++) {
    INFO = supernodal_find_pivot(&pivot, &secondpivot, i, sn_size, sn_size, up_size, F1, F2);
    if ( INFO ) {
      /*The matrix is singular.*/
      /*return INFO;*/
      /* we put INF in D and zeros in L and continue */
      db_size[i] = 1;
#if defined (TAUCS_CORE_DOUBLE) || defined (TAUCS_CORE_SINGLE)
      taucs_re(DB[i * 2]) = taucs_get_inf();
#endif
      F1[i * sn_size + i] = taucs_one_const;
      INFO = 0;
      taucs_printf("\t\tFound a singular column in sn=%d. Putting INf in D.\n",sn);
      continue;
    }
    
    /* in case we already rejected this column, yet we succeeded
       to factor it this time */
    if (pivot!=-1 && fct->rowind[fct->indices[i]] == -1) {
      fct->rowind[fct->indices[i]] = i;
      fct->rejected_size--;
    }
    
    if ( pivot == -2 ) {
      /* no need to do factoring for this column */
      db_size[i] = 1;
      diagonal = F1[i*sn_size + i];
      
      DB[i * 2] = diagonal;
      F1[i * sn_size + i] = taucs_one_const;
    }
    else if ( pivot >= 0 && secondpivot == -1 ) {
      /* a 1*1 pivot of column 'pivot' */
      if ( pivot != i )
	supernodal_switch( F1, F2, sn_size, up_size, i, pivot, fct );
      
      db_size[i] = 1;
      diagonal = F1[i*sn_size + i];
      
      DB[i * 2] = diagonal;
      F1[i * sn_size + i] = taucs_one_const;
      
      /* update the rest of F1 and F2 */
      size_left = sn_size - i - 1;
      one = 1;
      temp = taucs_div( taucs_minusone_const, diagonal );
      if ( size_left ) {
	/* update F1 */
	taucs_her("Lower", &size_left, &temp,
		  &(F1[i*sn_size+i+1]),
		  &one,
		  &(F1[(i+1)*sn_size + i+1 ]),
		  &sn_size);
	/* update F2 */
	if ( up_size )
	  taucs_ger(&up_size, &size_left, &temp,
		    &(F2[i*up_size]), &one,
		    &(F1[i*sn_size+i+1]),&one,
		    &(F2[(i+1)*up_size]), &up_size);
      }
      
      /* update the column in F1 */
      temp = taucs_div( taucs_one_const, diagonal );
      taucs_scal(&size_left,
		 &temp,
		 &(F1[i*sn_size+i+1]),
		 &one);
      /* update the column in F2 */
      taucs_scal(&up_size,
		 &temp,
		 &(F2[i*up_size]),
		 &one);
      
    }
    else  if ( pivot >= 0 ) { /* 2*2 pivot found */
      /* check that we didn't already reject the pivots,
	 and if so - remove it from the rejected list */
      if ( fct->rejected_size ) {
	if ( fct->rowind[fct->indices[pivot]] == -1 ) {
          fct->rejected_size--;
	  fct->rowind[fct->indices[pivot]] = pivot;
	}
	if ( fct->rowind[fct->indices[secondpivot]] == -1 ) {
	  fct->rejected_size--;
	  fct->rowind[fct->indices[secondpivot]] = secondpivot;
	}
      }
      if ( pivot != i )
	supernodal_switch( F1, F2, sn_size, up_size, i, pivot, fct );
      if ( secondpivot != i+1 )
	supernodal_switch( F1, F2, sn_size, up_size, i+1, secondpivot, fct );
      
      db_size[i] = 2;
      db_size[i+1] = 1;
      
      
      D[0] = F1[i * sn_size + i];
      D[1] = D[2] = F1[i * sn_size + i+1];
      D[3] = F1[(i+1) * sn_size + i+1];
      
      size_left = sn_size -(i+2);
      size_L21 = size_left;/* + mtr->up_size;*/
      
      if ( size_L21 ) {
	L21 = (taucs_datatype*)taucs_calloc(2*size_L21, sizeof(taucs_datatype));
	if (!L21) return -1;
	/* copy to L21 */
	for (k=0; k<size_L21; k++) {
	  L21[k] = F1[i*sn_size+i+2+k];
	  L21[(size_L21)+k] =
	    F1[(i+1)*sn_size+i+2+k];
	}
	/*for (k=0; k<mtr->up_size; k++) {
	  L21[size_left+k] = mtr->SFM_F2[i*(mtr->up_size)+k];
	  L21[(size_L21)+size_left+k] =
	  mtr->SFM_F2[(i+1)*(mtr->up_size)+k];
	  }*/
      }
      
      DB[i*2] = D[0];
      DB[(i*2)+1] = D[1];
      DB[(i*2)+2] = D[3];
      F1[i * sn_size + i] = taucs_one_const;
      F1[i * sn_size + i+1] = taucs_zero_const;
      F1[(i+1) * sn_size + i+1] = taucs_one_const;
      
      need_switch = build_LU_from_D(D,L,U);
      
      
      /* factor the two columns of F1 and F2 */
      for (k=0; k<size_left; k++) {
	b[0] = F1[sn_size*i +i+2+k];
	b[1] = F1[sn_size*(i+1) +i+2+k];
	two_times_two_diagonal_block_solve(D,x,b,gl_cramer_rule,TRUE,L,U,need_switch);
	F1[i*sn_size +i+2+k] = x[0];
	F1[(i+1)*sn_size +i+2+k] = x[1];
      }
      for (k=0; k<up_size ; k++) {
	b[0] = F2[up_size*i +k];
	b[1] = F2[up_size*(i+1) +k];
	two_times_two_diagonal_block_solve(D,x,b,gl_cramer_rule,TRUE,L,U,need_switch);
	F2[i*up_size +k] = x[0];
	F2[(i+1)*up_size +k] = x[1];
      }
      
      two = 2;
      
      /* GEMM : C := A*B + C*/
      if ( size_left ) {
	taucs_gemm("No conjugate",
		   "Conjugate",
		   &size_left, &size_left,	&two,
		   &taucs_minusone_const,
		   &(F1[i*sn_size+i+2]),
		   &sn_size,
		   L21,&size_L21,
		   &taucs_one_const,
		   &(F1[(i+2)*sn_size + i+2]),&sn_size);
	
	if ( up_size )
	  taucs_gemm("No conjugate",
		     "Conjugate",
		     &up_size, &size_left,	&two,
		     &taucs_minusone_const,
		     &(F2[i*up_size]),
		     &up_size,
		     L21,&size_L21,
		     &taucs_one_const,
		     &(F2[(i+2)*up_size]),&up_size);
      }
      
      /* we do not need the upper part of SFM_F1
	 for it is symmetric */
      for (k=i+2; k<sn_size; k++)
	for (j=0; j<k; j++)
	  F1[k*sn_size+j] = taucs_zero_const;
      
      if ( size_L21 )
	taucs_free(L21);
      
      i++;
    }
    else { /* no pivot found */
      if ( fct->rowind[fct->indices[i]] == -1 ) {
	/* this column was already rejected */
	if ( *rejected_tried ) {
	  /* we already tried to factor the rejected columns
	     just finish */
	  *rejected_tried = 2;
	  i = sn_size;
	} else {
	  /* set all rejected as unrejected and try them again */
	  *rejected_tried = 1;
	  for (j=i;j<sn_size;j++)
	    fct->rowind[fct->indices[j]] = 0;
	  fct->rejected_size = 0;
	}
	i--;
      } else {
	/* we move this column to the end of F1 */
	/* k is the last place in F1 which has not been replaced
	   with a rejected column */
	k = sn_size - fct->rejected_size - 1;
	if ( i != k ) {
	  supernodal_switch( F1,F2,sn_size, up_size,i,k,fct);
	  fct->rowind[fct->indices[k]] = -1;
	  fct->rejected_size++;
	} else {
	  fct->rowind[fct->indices[i]] = -1;
	  fct->rejected_size++;
	}
	i--; /* try again the new column in place i */
      }
    }
  }
  
  return INFO;
}


static int
Blocked_supernodal_front_factor_ldlt(supernodal_ldlt_factor* fct,
				     taucs_datatype* F1,
				     taucs_datatype* F2,
				     taucs_datatype* DB,
				     int* db_size,
				     int sn_size,
				     int up_size,
				     int sn)
{
  int INFO = 0;
  int actual_block_size = gl_block_size;
  taucs_datatype* W1 = NULL;
  taucs_datatype* W2 = NULL;
  int i, rejected_tried = 0;
  
  W1 = (taucs_datatype*)
    taucs_calloc(gl_block_size*sn_size,sizeof(taucs_datatype));
  W2 = (taucs_datatype*)
    taucs_calloc(gl_block_size*up_size,sizeof(taucs_datatype));
  if ( !W1 || !W2 ) {
    return -1;
  }
  
  i = 0;
  while ( i < sn_size ) {
    if ( i <= sn_size - gl_block_size ) {
      INFO = block_factor(i, &actual_block_size,
			  F1,F2,DB,db_size,
			  sn_size, up_size, fct, W1, W2, &rejected_tried, sn ); 
      /*CALL DLASYF( UPLO, N-K+1, NB, KB, A( K, K ), LDA, IPIV( K ),
	WORK, LDWORK, IINFO )*/
    } else {
      INFO = unblock_factor(i, F1, F2, DB, db_size,
			    sn_size, up_size, fct, &rejected_tried, sn ); 
      
      actual_block_size = sn_size-i;
      /*   CALL DSYTF2( UPLO, N-K+1, A( K, K ), LDA, IPIV( K ), IINFO )
	   KB = N - K + 1*/
    }
    i+= actual_block_size;
    if ( rejected_tried == 2 ) break;
  }
  taucs_free(W1);
  taucs_free(W2);
  
  return INFO;
}

static int
Sytrf_supernodal_front_factor_ldlt(supernodal_ldlt_factor* fct,
				   taucs_datatype* F1,
				   taucs_datatype* F2,
				   taucs_datatype* DB,
				   int* db_size,
				   int sn_size,
				   int up_size,
				   int sn)
{
  int INFO = 0, i;
  int* IPIV = NULL;
  taucs_datatype* WORK = NULL;
  int LWORK = sn_size * 20;
  
  assert( up_size == 0 ); /* can only be called on the last supernode */
  
  IPIV = taucs_calloc(sn_size,sizeof(int));
  WORK =(taucs_datatype*)taucs_calloc(LWORK,sizeof(taucs_datatype));
  if (!IPIV || !WORK) {
    taucs_free(IPIV);
    taucs_free(WORK);
    return -1;
  }
  
  assert(0);
  /*  taucs_sytrf("Lower", &sn_size, 
	      F1, &sn_size,
	      IPIV, WORK,
	      &LWORK, &INFO );
  */
  /* taucs_printf("XXX Sytrf info = %d\n",INFO); */
  
  if (!INFO) {
    /* we now make the adjustements for our code */
    for (i=0; i<sn_size; i++ ) {
      //      taucs_printf("XXX ipiv[%d] = %d\n",i,IPIV[i]);
      if (IPIV[i] > 0) IPIV[i] -= 1;
      else             IPIV[i] += 1;
    }
    
    for (i=0; i<sn_size; i++ ) {
      taucs_printf("XXX ipiv[%d] = %d\n",i,IPIV[i]);
      if (IPIV[i] >= 0) {
	
	if ( i != IPIV[i] ) {
	  taucs_printf("XXX indices switch %d %d\n",i,IPIV[i]);
	  supernodal_indices_switch( fct, i , IPIV[i] );
	  /*supernodal_switch(F1,F2,sn_size,up_size, i,IPIV[i]-1,fct);*/
	}
	
	db_size[i] = 1;
	
	DB[2*i] = F1[i*sn_size+i];
	F1[i*sn_size+i] = taucs_one_const;
	
      } else if (IPIV[i] < 0 && IPIV[i+1] == IPIV[i]) {
	//	assert(0);
	if ( i+1 != -(IPIV[i]) )
	  supernodal_indices_switch( fct, i+1 , -(IPIV[i]) );
	/*supernodal_switch(F1,F2,sn_size,up_size,
	  i+1,-(IPIV[i]+1),fct);*/
	
	db_size[i] = 2;
	db_size[i+1] = 1;
	
	DB[2*i] = F1[i*sn_size+i];
	DB[2*i+1] = F1[i*sn_size+i+1];
	DB[2*i+2] = F1[(i+1)*sn_size+i+1];
	F1[i*sn_size+i] = taucs_one_const;
	F1[i*sn_size+i+1] = taucs_zero_const;
	F1[(i+1)*sn_size+i+1] = taucs_one_const;
	
	i++;
	
      }
    }
  }
  
  taucs_free(IPIV);
  taucs_free(WORK);
  
  return INFO;
}

static int
Potrf_supernodal_front_factor_ldlt(supernodal_ldlt_factor* fct,
																		taucs_datatype* F1,
																		taucs_datatype* F2,
																		taucs_datatype* DB,
																		int* db_size,
																		int sn_size,
																		int up_size,
																		int sn)
{
	int INFO = 0, i;
	taucs_datatype temp,diagonal;
	int k;
	
	assert( up_size == 0 );

	  /*taucs_potrf ("LOWER",
							&sn_size,
							F1,&sn_size,
							&INFO);*/
	if (!INFO) {
		/* we now make the adjustements for our code */
		for (i=0; i<sn_size; i++) {
			
			db_size[i] = 1;
			diagonal = F1[i*sn_size+i];
			DB[2*i] = taucs_mul(diagonal,diagonal);
			F1[i*sn_size+i] = taucs_one_const;

			for (k=i+1; k<sn_size; k++) {
				temp = F1[sn_size*i + k];
				if ( taucs_re(temp)  || taucs_im(temp) )
					F1[sn_size*i + k] = taucs_div( temp, diagonal );
			}
		}
	}
	
	return INFO;
}

int taucs_dtl(internal_supernodal_front_factor_ldlt)(void* vfct,taucs_datatype* F1,
						     taucs_datatype* F2,taucs_datatype* DB,
						     int* db_size,int sn_size,
						     int up_size,int sn,int alg)
{
  supernodal_ldlt_factor* fct = (supernodal_ldlt_factor*)vfct;
  int ret = 0;
  switch(alg)	{
  case fa_right_looking:
    ret = RL_supernodal_front_factor_ldlt(fct,F1,F2,DB,
					  db_size,sn_size,up_size,sn);
    break;
  case fa_blocked:
    ret = Blocked_supernodal_front_factor_ldlt(fct,F1,F2,DB,
					       db_size,sn_size,up_size,sn);
    break;
  case fa_sytrf:
    ret = Sytrf_supernodal_front_factor_ldlt(fct,F1,F2,DB,
					     db_size,sn_size,up_size,sn);
    break;
  case fa_potrf:
    ret = Potrf_supernodal_front_factor_ldlt(fct,F1,F2,DB,
					     db_size,sn_size,up_size,sn);
    break;
  }
  return ret;
}


static int
multifrontal_supernodal_front_factor_ldlt(int sn,
				     int* firstcol_in_supernode,
				     int sn_size,
				     taucs_ccs_matrix* A,
				     supernodal_frontal_matrix_ldlt* mtr,
				     int* bitmap,
				     supernodal_factor_matrix_ldlt* snL,
						 int parent)
{
  int i,j,db_size,index;
	int new_sn_size,new_up_size,old_up_size;
  int* ind;
  taucs_datatype* re;
	taucs_datatype* F2D11 = NULL; /* This is the product of F2 and D11 */
	taucs_datatype D[4],b[2];
  int INFO;
	supernodal_ldlt_factor* fct;

	/* creating transform for real indices */
  for(i=0;i<mtr->sn_size;i++) bitmap[mtr->sn_vertices[i]] = i;
  for(i=0;i<mtr->up_size;i++) bitmap[mtr->up_vertices[i]] = mtr->sn_size + i;

	/* adding sn_size column of A to first sn_size column of frontal matrix */
	/* we add only the orig sn_size of the frontal matrix, because all the rest
	was already contibuted in rejected columns */
	for(j=0;j<(snL->orig_sn_size[sn]);j++) {
    ind = &(A->rowind[A->colptr[*(firstcol_in_supernode+j)]]);
    re  = &(A->taucs_values[A->colptr[*(firstcol_in_supernode+j)]]);
    for(i=0;
				i < A->colptr[*(firstcol_in_supernode+j)+1]
										  - A->colptr[*(firstcol_in_supernode+j)];
				i++) {
			if (bitmap[ind[i]] == -1 ) continue;
      if (bitmap[ind[i]] < mtr->sn_size)
				mtr->SFM_F1[ (mtr->sn_size)*j + bitmap[ind[i]]] =
						taucs_add( mtr->SFM_F1[ (mtr->sn_size)*j + bitmap[ind[i]]] , re[i] );
      else {
				mtr->SFM_F2[ (mtr->up_size)*j + bitmap[ind[i]] - mtr->sn_size] =
						taucs_add( mtr->SFM_F2[ (mtr->up_size)*j + bitmap[ind[i]] -
							mtr->sn_size] , re[i] );
			}
    }
  }
	/* this is the structure that gives us the info about the factorization
	of the F1 matrix */
	fct = create_ldlt_factor(mtr->sn_size);
	if (!fct)
		return -1;

	/*writeToFile(mtr,sn,"mfa");*/
	
	/* factor the F1 part of the front matrix */
	{
	  int algorithm;
	  
/*	  
	  
	  if ( sn == snL->n_sn - 1 ) algorithm = fa_sytrf;
	  else                       algorithm = fa_blocked;
*/
          algorithm = gl_factor_alg ; /* Bruno: fa_sytrf seems to be partially
                                      unimplemented in this preliminary version */

	  INFO = taucs_dtl(internal_supernodal_front_factor_ldlt)(fct,mtr->SFM_F1,mtr->SFM_F2, 
								  mtr->SFM_D, mtr->db_size, mtr->sn_size, mtr->up_size,sn,
								  algorithm /* was gl_factor_alg, but 
									     this caused delays even in the last supernode */);
	}

	if (INFO) {
    taucs_printf("\t\tLDL^T Factorization: Matrix is singular.\n");
    taucs_printf("\t\t                     Encountered problem in column %d\n",
		 mtr->sn_vertices[INFO-2]);
    return -1;
  }

	/* any columns that were not factored are moved to F2 and the supernode's parent
		 we put F2 inside F2D11 (sized as needed)*/
	new_sn_size = mtr->sn_size - fct->rejected_size;
	new_up_size = mtr->up_size + fct->rejected_size;
	old_up_size = mtr->up_size;
	if ( new_sn_size && new_up_size ) {
		F2D11 = (taucs_datatype*)taucs_calloc(new_sn_size*new_up_size,sizeof(taucs_datatype));
		if ( !F2D11 ) {
			taucs_printf("Could not allocate memory (for structure) %d\n",new_sn_size*new_up_size);
			return -1;
		}
	}
	INFO = mf_sn_front_reorder_ldlt(sn,mtr,snL,F2D11,fct,parent,new_sn_size,new_up_size);
	
	if (INFO) {
    taucs_printf("\t\tLDL^T Factorization: Problem reordering columns.\n");
    return -1;
  }
	/*writeToFile(mtr,sn,"mfb");*/

	free_ldlt_factor(fct);
	taucs_free(fct);

  /* getting completion for found columns of L */
	/* changed here for LDLt computations. First we find F2*D11, then
		 F2:=F2*D11^-1 */
	if (new_up_size && new_sn_size) {

		/* find F2D11 from F2 */
		for (i=0; i<new_sn_size; i++){
			db_size = mtr->db_size[i];

			if ( db_size == 1 ) { /* 1*1 block */
				for (j=0; j<new_up_size; j++) {
					index = i*(new_up_size) + j;
					if ( taucs_re(F2D11[index]) || taucs_im(F2D11[index]) )
						F2D11[index] = taucs_mul( F2D11[index], mtr->SFM_D[i*2] );
				}
			}	else if ( db_size == 2) {

				/* make the diagonal block for the 2*2 solution */
				D[0] = mtr->SFM_D[i*2];
				D[1] = D[2] = mtr->SFM_D[i*2 + 1];
				D[3] = mtr->SFM_D[(i+1)*2];

				for (j=0; j<new_up_size; j++) {

					/* make the vectors for the 2*2 solution */
					b[0] = F2D11[i*(new_up_size) + j];
					b[1] = F2D11[(i+1)*(new_up_size) + j];

					/* solve the equations */
					/*two_times_two_diagonal_block_solve(D,x,b,FALSE,TRUE);*/

					/*F2D11[i*(new_up_size) + j] = x[0];
					F2D11[(i+1)*(new_up_size) + j] = x[1];*/
					F2D11[i*new_up_size + j] = 
						taucs_add(taucs_mul(b[0],D[0]),
											taucs_mul(b[1],D[2]));
					F2D11[(i+1)*new_up_size + j] = 
						taucs_add(taucs_mul(b[0],D[1]),
											taucs_mul(b[1],D[3]));

				}

				i++; /* we need to skip the next column, we already did her */

			}	else {/* shouldn't get here */
				taucs_printf("db_size[%d] = %d in sn=%d\n",i,mtr->db_size[i],sn );
				assert(0);
			}

		}

		/* computation of updated part of frontal matrix */
		/* GEMM : C := A*B + C*/
		/* we use here the old_up_size so that we update only the non-updated
		part of U. The rest was already updated in F1-2, and moved here.*/
		taucs_gemm("No conjugate",
			"Conjugate",
			&old_up_size, &old_up_size,
			&new_sn_size,
			&taucs_minusone_const,
			F2D11,&new_up_size,
			mtr->SFM_F2,&new_up_size,
			&taucs_one_const,
			mtr->SFM_U,&new_up_size);

		/* we do not need the upper part of SFM_U
		for it is symmetric */
		for (i=0; i<new_up_size; i++)
			for (j=0; j<i; j++)
				mtr->SFM_U[i*new_up_size+j] = taucs_zero_const;
	}

					  /*writeToFile(mtr,sn,"mfc");*/



	(snL->sn_blocks   )[sn] = mtr->SFM_F1;
  (snL->sn_blocks_ld)[sn] = mtr->sn_size;
	(snL->sn_size)[sn]			= mtr->sn_size;

  (snL->up_blocks   )[sn] = mtr->SFM_F2;
  (snL->up_blocks_ld)[sn] = mtr->up_size;
	(snL->sn_up_size)[sn]		= mtr->up_size+mtr->sn_size;

	(snL->d_blocks		)[sn] = mtr->SFM_D;
	(snL->db_size			)[sn] = mtr->db_size;

  mtr->SFM_F1 = NULL; /* so we don't free twice */
  mtr->SFM_F2 = NULL; /* so we don't free twice */
	mtr->SFM_D = NULL;	/* so we don't free twice */
	mtr->db_size = NULL;/* so we don't free twice */

	if ( F2D11 )
		taucs_free(F2D11);

  return 0;

}

static supernodal_frontal_matrix_ldlt*
recursive_multifrontal_supernodal_factor_ldlt(int sn,       /* this 
supernode */
					     int is_root,  /* is v the root? */
					     int* bitmap,
					     taucs_ccs_matrix* A,
					     supernodal_factor_matrix_ldlt* snL,
					     int* fail,
							 int parent /* the parent */)
{
  supernodal_frontal_matrix_ldlt* my_matrix=NULL;
  supernodal_frontal_matrix_ldlt* child_matrix=NULL;
  int child;
  int* v;
  int  sn_size;
  int* first_child   = snL->first_child;
  int* next_child    = snL->next_child;

  /* Sivan fixed a bug 25/2/2003: v was set even at the root, */
  /* but this element of sn_struct was not allocated.         */


  /*if (!is_root) {
    sn_size = snL->sn_size[sn];
    v = &( snL->sn_struct[sn][0] );
  } else {
    sn_size = -1;
    v = NULL;
  }*/
	/* record the original supernode size */
	snL->orig_sn_size[sn] = snL->sn_size[sn];

  for (child = first_child[sn]; child != -1; child = next_child[child]) {
    child_matrix =
      recursive_multifrontal_supernodal_factor_ldlt(child,
						   FALSE,
						   bitmap,
						   A,snL,fail,sn);
    if (*fail) {
      if (my_matrix) supernodal_frontal_free(my_matrix);
      return NULL;
    }

    if (!is_root) {
      if (!my_matrix) {
				sn_size = snL->sn_size[sn];
				v = &( snL->sn_struct[sn][0] );
				my_matrix =  supernodal_frontal_ldlt_create(v,sn_size,
					       snL->sn_up_size[sn],
					       snL->sn_struct[sn]);
				if (!my_matrix) {
					*fail = TRUE;
					supernodal_frontal_free(child_matrix);
					return NULL;
				}
			} else {
				if ( supernodal_frontal_ldlt_modify( my_matrix, snL, sn ) != 0 ) {
					supernodal_frontal_free(my_matrix);
					*fail = TRUE;
					return NULL;
				}
			}

			multifrontal_supernodal_front_extend_add(my_matrix,child_matrix,bitmap);
    }
    /* moved outside "if !is_root"; Sivan 27 Feb 2002 */
    supernodal_frontal_free(child_matrix);
  }

  /* in case we have no children, we allocate now */
  if (!is_root && !my_matrix) {
		sn_size = snL->sn_size[sn];
    v = &( snL->sn_struct[sn][0] );
    my_matrix =  supernodal_frontal_ldlt_create(v,sn_size,
					   snL->sn_up_size[sn],
					   snL->sn_struct[sn]);
    if (!my_matrix) {
      *fail = TRUE;
      return NULL;
    }
  }

  if(!is_root) {
		sn_size = snL->sn_size[sn];
    v = &( snL->sn_struct[sn][0] );
    if (multifrontal_supernodal_front_factor_ldlt(sn,
					     v,
							 sn_size,
					     A,
					     my_matrix,
					     bitmap,
					     snL,
							 parent)) {
      /* nonpositive pivot */
      *fail = TRUE;
      supernodal_frontal_free(my_matrix);
      return NULL;
    }
  }
  return my_matrix;
}

void*
taucs_dtl(ccs_factor_ldlt_mf)(taucs_ccs_matrix* A)
{
  return taucs_dtl(ccs_factor_ldlt_mf_maxdepth)(A,0);
}

void*
taucs_dtl(ccs_factor_ldlt_mf_maxdepth)(taucs_ccs_matrix* A,int max_depth)
{
  supernodal_factor_matrix_ldlt* L;
  int* map;
  int fail;
  double wtime, ctime;

  wtime = taucs_wtime();
  ctime = taucs_ctime();

	L = multifrontal_supernodal_create();
  if (!L) return NULL;

#ifdef TAUCS_CORE_COMPLEX
  fail = taucs_ccs_ldlt_symbolic_elimination(A,L,
					TRUE /* sort, to avoid complex conjuation */,
					max_depth);
#else
  fail = taucs_ccs_ldlt_symbolic_elimination(A,L,
					FALSE /* don't sort row indices */          ,
					max_depth);
#endif
  if (fail == -1) {
    taucs_supernodal_factor_ldlt_free(L);
    return NULL;
  }

	wtime = taucs_wtime()-wtime;
  ctime = taucs_ctime()-ctime;
  taucs_printf("\t\tSymbolic Analysis            = % 10.3f seconds (%.3f cpu)\n",
	       wtime,ctime);

  map = (int*)taucs_malloc((A->n+1)*sizeof(int));
  if (!map) {
    taucs_supernodal_factor_ldlt_free(L);
    return NULL;

  }


  /*writeFactorFacts(L,"mfa");*/
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
		case fa_sytrf:
			taucs_printf("\t\tUsing sytrf lapack dense factorization.\n");
			break;
		case fa_potrf:
			taucs_printf("\t\tUsing potrf lapack dense factorization.\n");
			break;
	}
	
  wtime = taucs_wtime();
  ctime = taucs_ctime();

  fail = FALSE;
	recursive_multifrontal_supernodal_factor_ldlt((L->n_sn),
					       TRUE,
					       map,
					       A,L,&fail,-1);

	wtime = taucs_wtime()-wtime;
  ctime = taucs_ctime()-ctime;
  taucs_printf("\t\tSupernodal Multifrontal LDL^T = % 10.3f seconds (%.3f cpu)\n",
	       wtime,ctime);

  taucs_free(map);

  if (fail) {
    taucs_supernodal_factor_ldlt_free(L);
    return NULL;
  }

  {
    double nnz   = 0.0;
    double flops = 0.0;
    int bytes;
    
    taucs_get_statistics(&bytes,&flops,&nnz,L);
    
    taucs_printf("\t\tPost     Analysis of LDL^T: %.2e nonzeros, %.2e flops, %.2e bytes in L\n",
		 nnz, flops, (float) bytes);
  }
  
	/*writeFactorFacts(L,"mfb");*/

  return (void*) L;
}

/*************************************************************/
/* symbolic-numeric routines                                 */
/*************************************************************/

void*
taucs_dtl(ccs_factor_ldlt_symbolic)(taucs_ccs_matrix* A)
{
  return taucs_dtl(ccs_factor_ldlt_symbolic_maxdepth)(A,0);
}

void*
taucs_dtl(ccs_factor_ldlt_symbolic_maxdepth)(taucs_ccs_matrix* A, int 
max_depth)
{
  supernodal_factor_matrix_ldlt* L;
  int fail;
  double wtime, ctime;

  wtime = taucs_wtime();
  ctime = taucs_ctime();

  L = multifrontal_supernodal_create();
  if (!L) return NULL;

#ifdef TAUCS_CORE_COMPLEX
  fail = taucs_ccs_ldlt_symbolic_elimination(A,L,
					TRUE /* sort, to avoid complex conjuation */,
					max_depth);
#else
  fail = taucs_ccs_ldlt_symbolic_elimination(A,L,
					FALSE /* don't sort row indices */          ,
					max_depth);
	#endif

  if (fail == -1) {
    taucs_supernodal_factor_ldlt_free(L);
    return NULL;
  }

  wtime = taucs_wtime()-wtime;
  ctime = taucs_ctime()-ctime;
  taucs_printf("\t\tSymbolic Analysis            = % 10.3f seconds (%.3f cpu)\n",
	       wtime,ctime);
  return L;
}

int
taucs_dtl(ccs_factor_ldlt_numeric)(taucs_ccs_matrix* A,void* vL)
{
  supernodal_factor_matrix_ldlt* L = (supernodal_factor_matrix_ldlt*) vL;
  int* map;
  int fail;
  double wtime, ctime;

  map = (int*)taucs_malloc((A->n+1)*sizeof(int));

  wtime = taucs_wtime();
  ctime = taucs_ctime();

  fail = FALSE;
  recursive_multifrontal_supernodal_factor_ldlt((L->n_sn),
					       TRUE,
					       map,
					       A,L,&fail,-1);

  wtime = taucs_wtime()-wtime;
  ctime = taucs_ctime()-ctime;
  taucs_printf("\t\tSupernodal Multifrontal LDL^T = % 10.3f seconds (%.3f cpu)\n",
	       wtime,ctime);

  taucs_free(map);

  if (fail) {
    taucs_supernodal_factor_ldlt_free_numeric(L);
    return -1;
  }

  return 0;
}

/*************************************************************/
/* left-looking factor routines                              */
/*************************************************************/

static int
recursive_leftlooking_supernodal_update_ldlt(int J,int K,
					int bitmap[],
					taucs_datatype* dense_update_matrix,
					taucs_ccs_matrix* A,
					supernodal_factor_matrix_ldlt* L,
					int rejected)
{
  int i,j,ir,k,index;
  int  child;
  int sn_size_father = (L->sn_size)[J];
  int up_size_father = (L->sn_up_size)[J] - sn_size_father;
	int LD_sn_J = (L->sn_blocks_ld)[J];
	int LD_up_J = (L->up_blocks_ld)[J];
  int sn_size_child = (L->sn_size)[K] - rejected;
	/* the up_size_child in the direct update does not include the rejected rows */
	int up_size_child = (L->sn_up_size)[K] - (L->sn_size)[K];
  int LD_sn_K = (L->sn_blocks_ld)[K];
	int LD_up_K = (L->up_blocks_ld)[K];
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
       && L->sn_struct[K][i+sn_size_child+rejected] <= max_index_snJ) {
			if(!exist_upd) ind_first_row = i;
      row_count++;
      exist_upd = 1;
			ind_last_row = i+1;
		} 
	}

	
	if ( rejected ) {
		/* we put the rejected rows in the rejected_list, so that children
		of this child do not update the supernode for these rows */
		rejected_list = (int*)taucs_malloc(rejected*sizeof(int));
		if ( !rejected_list ) return -1;
		
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
		LDB = (L->up_blocks_ld)[K];
    LDC =  up_size_father + sn_size_father;
		N = ind_last_row - ind_first_row;/* was row_count */
		assert( row_count <= N );
		
    
		/* we now need to allocate another dense matrix, which will hold
			 the F2*D11 multiplication.
			 This means that the update part of each supernode should be sorted, 
			 and they are because we don't mess with the order of the update part, 
			 only add more columns that were moved to the father, 
			 and therefor we know that they are included in J */
		F2D11 = (taucs_datatype*)taucs_calloc(sn_size_child*M,sizeof(taucs_datatype));
		if (!F2D11) return -1;
		
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

		/*sprintf(str,"a_%d",K);
		  writeSNLToFile(L,J,str);*/
    
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

		/*sprintf(str,"b_%d",K);
		writeSNLToFile(L,J,str);*/

		/* here we 'silence' the rejected rows from the list, 
		so that grand children do not update them ( they already
		updated them in the child */
		for (i=0; i<rejected; i++) {
			bitmap[rejected_list[i]] = 0;
		}
		taucs_free(rejected_list);
		taucs_free(F2D11);
    
    for (child = L->first_child[K]; child != -1; child = L->next_child[child]) {
      if ( recursive_leftlooking_supernodal_update_ldlt(	J,child,
																											bitmap,dense_update_matrix,
																											A,L,0)) {
				return -1;
			}
    }
  }
	return 0;
}

static int
ll_sn_reorder_ldlt(int sn,
										supernodal_factor_matrix_ldlt* snL,
										supernodal_ldlt_factor* fct,
										int parent,
										int new_sn_size,
										int new_up_size)
{
	int INFO = 0;
	int i,rej_size;
	int* current_struct = NULL;
	int old_sn_size = snL->sn_size[sn];
	
	rej_size = fct->rejected_size;
	
	if ( rej_size ) { 
		
		/* copying the structure of the parent to a new bigger struct */
		snL->sn_struct[parent] = (int*)taucs_realloc(snL->sn_struct[parent],
				(rej_size+snL->sn_up_size[parent])*sizeof(int));
		if ( !snL->sn_struct[parent] ) {
			taucs_printf("Could not allocate memory (for structure, 1) %d\n",(fct->rejected_size) + snL->sn_size[parent]);
			taucs_printf("Requested size: %d\n",(rej_size+snL->sn_up_size[parent]));
			return -1;
		}
		/* copying the rejected rows to the start of the parent. */
		for (i=snL->sn_up_size[parent]-1; i>=0; i--)
			snL->sn_struct[parent][i+rej_size] =
				snL->sn_struct[parent][i];
		for (i=0; i<rej_size; i++)
			snL->sn_struct[parent][i] =
				snL->sn_struct[sn][fct->indices[i+new_sn_size]];

		snL->sn_size[parent]+=rej_size;
		snL->sn_up_size[parent]+=rej_size;
		snL->sn_blocks_ld[parent]+=rej_size;
	}

	/* rebuild the structure of the current supernode */
	current_struct = (int*)taucs_malloc((new_sn_size+new_up_size)*sizeof(int));
	if ( !current_struct ) {
		taucs_printf("Could not allocate memory (for structure, 2) %d\n",(new_sn_size+new_up_size));
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
supernodal_ldlt_modify_parent(	supernodal_factor_matrix_ldlt* snL,
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
supernodal_ldlt_modify_child(	supernodal_factor_matrix_ldlt* snL,
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
	
	snL->up_blocks_ld[sn]+=rejected;
	snL->sn_blocks_ld[sn]-=rejected;
		
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
leftlooking_supernodal_front_factor_ldlt(int sn,
							int* indmap,
							int* bitmap,
							taucs_ccs_matrix* A,
							supernodal_factor_matrix_ldlt* L,
							int parent,
							int* p_rejected)
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
	(L->sn_blocks)[sn][ (L->sn_blocks_ld)[sn]*jp + indmap[ind[ip]]] =
	  taucs_add( (L->sn_blocks)[sn][ (L->sn_blocks_ld)[sn]*jp + 
					 indmap[ind[ip]]] , re[ip] );
      else
	(L->up_blocks)[sn][ (L->up_blocks_ld)[sn]*jp + indmap[ind[ip]] - sn_size] =
	  taucs_add( (L->up_blocks)[sn][ (L->up_blocks_ld)[sn]*jp + indmap[ind[ip]] - 
					 sn_size] , re[ip] );
    }
  }
  
  /* solving of lower triangular system for L */
  fct = create_ldlt_factor(sn_size);
  
  
  /*writeSNLToFile(L,sn,"lla");*/
  
	{
	  int algorithm;
	  if ( sn == L->n_sn - 1 ) algorithm = fa_sytrf;
	  else                     algorithm = fa_blocked;
	  INFO = taucs_dtl(internal_supernodal_front_factor_ldlt)(fct,L->sn_blocks[sn],L->up_blocks[sn], 
								  L->d_blocks[sn], L->db_size[sn],
								  L->sn_size[sn], L->sn_up_size[sn] - L->sn_size[sn],sn,
								  algorithm  
								  /* was gl_factor_alg, but 
								     this caused delays even in the last supernode */);
	}

  
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
     supernodal_ldlt_modify_child after the direct update is done. */
  INFO = ll_sn_reorder_ldlt(sn,L,fct,parent,new_sn_size,new_up_size);
  
  if (INFO) {
    taucs_printf("\t\tLDL^T Factorization: Problem reordering columns (left-looking).\n");
    return -1;
  }
  /*writeSNLToFile(L,sn,"llb");*/
  *p_rejected = fct->rejected_size;
  
  free_ldlt_factor(fct);
  taucs_free(fct);
  
  return 0;
}



static int
recursive_leftlooking_supernodal_factor_ldlt(int sn,       /* this supernode */
					     int is_root,  /* is v the root? */
					     int* bitmap,
					     int* indmap,
					     taucs_ccs_matrix* A,
					     supernodal_factor_matrix_ldlt* L,
					     int parent,
					     int* p_rejected)
{
  int  child, i,j, maxIndex;
  int  sn_size;
  taucs_datatype* dense_update_matrix = NULL,* temp = NULL;
  int rejected_from_child = 0;
  int dense_sn_size = 0, dense_up_size = 0;

  if (!is_root)
    sn_size = L->sn_size[sn];
  else
    sn_size = -1;

  if (!is_root) {
    (L->sn_blocks   )[sn] = (L->up_blocks   )[sn] = NULL;
    if (L->sn_size[sn]) {
      L->orig_sn_size[sn] = L->sn_size[sn];
      (L->sn_blocks   )[sn] = 
	(taucs_datatype*)taucs_calloc(((L->sn_size)[sn])*((L->sn_size)[sn]),
				      sizeof(taucs_datatype));
      if (!((L->sn_blocks)[sn])) return -1; /* the caller will free L */
      (L->d_blocks)[sn] = 
	(taucs_datatype*)taucs_calloc(2*(L->sn_size[sn]),sizeof(taucs_datatype));
      if (!((L->d_blocks)[sn])) return -1; /* the caller will free L */
      (L->db_size)[sn] = 
	(int*)taucs_calloc((L->sn_size[sn]),sizeof(int));
      if (!((L->db_size)[sn])) return -1; /* the caller will free L */
    }
    (L->sn_blocks_ld)[sn] = (L->sn_size   )[sn];
    
    if (((L->sn_up_size)[sn] - (L->sn_size)[sn]) && (L->sn_size)[sn]) {
      (L->up_blocks   )[sn] = 
	(taucs_datatype*)taucs_calloc(((L->sn_up_size)[sn]-(L->sn_size)[sn])
				      *((L->sn_size)[sn]),sizeof(taucs_datatype));
      if (!((L->up_blocks)[sn])) return -1; /* the caller will free L */
    }
    (L->up_blocks_ld)[sn] = (L->sn_up_size)[sn]-(L->sn_size)[sn];
  }
  
  /* put the max index */
  if (!is_root) {
    maxIndex = 0;
    for ( i=0; i<sn_size; i++ ) {
      maxIndex  = max(L->sn_struct[sn][i],maxIndex);
    }
    L->sn_max_ind[sn] = maxIndex;
  }
  
  for (child = L->first_child[sn]; child != -1; child = L->next_child[child]) {
    if (recursive_leftlooking_supernodal_factor_ldlt(child,
						     FALSE,
						     bitmap,
						     indmap,
						     A,L,sn,&rejected_from_child)
	== -1 ) {
      taucs_free(dense_update_matrix);
      return -1;
    }
    
    if (!is_root) {
      if (!dense_update_matrix) {
	dense_update_matrix =	(taucs_datatype*)
					taucs_calloc((L->sn_up_size)[sn]*(L->sn_size)[sn],sizeof(taucs_datatype));
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
	  return -1; /* caller will free L */
	}
	for (i=0;i<dense_sn_size;i++)
	  for (j=0;j<dense_up_size;j++)
	    temp[i*(L->sn_up_size)[sn]+j] = dense_update_matrix[i*dense_up_size+j];
	
	taucs_free(dense_update_matrix);
	dense_update_matrix = temp;

	dense_sn_size = L->sn_size[sn];
	dense_up_size = L->sn_up_size[sn];
      }
      
      
      /* prepare the bitmap. Moved out of the recusive
	 update procedure 20/1/2003. Sivan and Elad */
      
      {
	int i;
	int J = sn;
	int sn_size_father = (L->sn_size)[J];
	int sn_up_size_father = (L->sn_up_size)[J];
	
	for(i=0;i<sn_size_father;i++)
	  bitmap[L->sn_struct[J][i]]=i+1;
	for(i=sn_size_father;i<sn_up_size_father;i++)
	  bitmap[L->sn_struct[J][i]] = i - sn_size_father + 1;
      }
      
      if ( supernodal_ldlt_modify_parent(L,sn,rejected_from_child) ) {
	taucs_free(dense_update_matrix);
	return -1;
			}
      
      if ( recursive_leftlooking_supernodal_update_ldlt(sn,child,
							bitmap,dense_update_matrix,
							A,L,rejected_from_child) ) {
	taucs_free(dense_update_matrix);
	return -1;
      }
      
      if ( supernodal_ldlt_modify_child(L,child,rejected_from_child) ) {
				taucs_free(dense_update_matrix);
				return -1;
      }
      
      
      {
	int i;
	int J = sn;
	int sn_up_size_father = (L->sn_up_size)[J];
	
	for(i=0;i<sn_up_size_father;i++)
	  bitmap[L->sn_struct[J][i]]=0;
      }
      
    }
  }
  taucs_free(dense_update_matrix);
  
  if(!is_root) {
    if (leftlooking_supernodal_front_factor_ldlt(sn,
						 indmap,
						 bitmap,
						 A,
						 L,
						 parent,p_rejected)) {
      return -1; /* nonpositive pivot */
    }
  }
  
  return 0;
}


void* taucs_dtl(ccs_factor_ldlt_ll)(taucs_ccs_matrix* A)
{
  return taucs_dtl(ccs_factor_ldlt_ll_maxdepth)(A,0);
}

void*
taucs_dtl(ccs_factor_ldlt_ll_maxdepth)(taucs_ccs_matrix* A,int max_depth)
{
  supernodal_factor_matrix_ldlt* L;
  int* map;
  int *map2;
  double wtime, ctime;
  int fail,rejected;

  wtime = taucs_wtime();
  ctime = taucs_ctime();

  L = multifrontal_supernodal_create();
  if (!L) return NULL;

  fail = taucs_ccs_ldlt_symbolic_elimination(A,L,
					TRUE /* sort row indices */,
					max_depth);

  wtime = taucs_wtime()-wtime;
  ctime = taucs_ctime()-ctime;
  taucs_printf("\t\tSymbolic Analysis            = % 10.3f seconds (%.3f cpu)\n",
	       wtime,ctime);

  map  = (int*)taucs_malloc((A->n+1)*sizeof(int));
  map2 = (int*)taucs_calloc((A->n+1),sizeof(int));

  if (fail == -1 || !map || !map2) {
    taucs_supernodal_factor_ldlt_free(L);
    taucs_free(map2);
    taucs_free(map);
    return NULL;
  }

  /*writeFactorFacts(L,"lla");*/
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
		case fa_sytrf:
			taucs_printf("\t\tUsing sytrf lapack dense factorization.\n");
			break;
		case fa_potrf:
			taucs_printf("\t\tUsing potrf lapack dense factorization.\n");
			break;
	}

  wtime = taucs_wtime();
  ctime = taucs_ctime();

  if (recursive_leftlooking_supernodal_factor_ldlt((L->n_sn),
						  TRUE,
						  map2,
						  map,
						  A,L,-1,&rejected)
      == -1) {
    taucs_supernodal_factor_ldlt_free(L);
    taucs_free(map);
    taucs_free(map2);
    return NULL;
  }

  wtime = taucs_wtime()-wtime;
  ctime = taucs_ctime()-ctime;
  taucs_printf("\t\tSupernodal Left-Looking LDL^T = % 10.3f seconds (%.3f cpu)\n",
	       wtime,ctime);

	{
		double nnz   = 0.0;
		double flops = 0.0;
    int bytes;

		taucs_get_statistics(&bytes,&flops,&nnz,L);

    taucs_printf("\t\tPost     Analysis of LDL^T: %.2e nonzeros, %.2e flops, %.2e bytes in L\n",
		 nnz, flops, (float) bytes);
	}

  taucs_free(map);
  taucs_free(map2);

  /*writeFactorFacts(L,"llb");*/

  return (void*) L;
}



/*************************************************************/
/* supernodal solve routines                                 */
/*************************************************************/



static int
recursive_supernodal_solve_d(int sn,       /* this supernode */
					 int is_root,  /* is v the root? */
			     int* first_child, int* next_child,
			     int** sn_struct, int* sn_sizes,
					 taucs_datatype* diagonal[], int ** db_size,
			     taucs_datatype x[], taucs_datatype b[],
			     taucs_datatype t[])
{
	int child;
  int  sn_size; /* number of rows/columns in the supernode    */
	int ret = 0;

  taucs_datatype *xdense;
	taucs_datatype D[4], y[2], z[2], L[4], U[4]; /* D*y=z */
  int i, need_switch = 0;

	for (child = first_child[sn]; child != -1; child = next_child[child]) {
		ret = recursive_supernodal_solve_d(child,
						FALSE,
						first_child,next_child,
						sn_struct,sn_sizes,
						diagonal, db_size,
						x,b,t);
		if ( ret ) return ret;
  }

  if(!is_root) {

    sn_size = sn_sizes[sn];

		xdense = t;

		for (i=0; i<sn_size; i++)
			xdense[i] = b[ sn_struct[ sn ][ i ] ];

		for (i=0; i<sn_size; i++)
		{

			if ( db_size[sn][i] == 1 ) {/* a 1*1 block x = b/Aii */

				y[0] = xdense[i];
				xdense[i] = taucs_div( xdense[i], diagonal[sn][i*2] );

				/* check that if the diagonal was inf, then y[0] was 0.
				if not - there isn't a solution.*/
				if (isinf(taucs_re(diagonal[sn][i*2])) &&
						!taucs_is_zero(y[0]) )
				 return -1;
			}

			else if ( db_size[sn][i] == 2 ) /* a 2*2 block */
			{
				D[0] = diagonal[sn][2*i];
				D[1] = D[2] = diagonal[sn][2*i+1];
				D[3] = diagonal[sn][2*i+2];

				z[0] = xdense[i];
				z[1] = xdense[i+1];

				need_switch = build_LU_from_D(D,L,U);

				two_times_two_diagonal_block_solve( D,y,z,gl_cramer_rule, FALSE,L,U,need_switch );

				xdense[i] = y[0];
				xdense[i+1] = y[1];

				i++;
			}
			else {/* shouldn't get here */
				taucs_printf("db_size[%d][%d] = %d\n",sn,i,db_size[sn][i] );
				assert(0);
				return -1;
			}
		}

    for (i=0; i<sn_size; i++)
			x[ sn_struct[ sn][ i ] ]  = xdense[i];
  }

	return ret;
}




static int
recursive_supernodal_solve_l(int sn,       /* this supernode */
			     int is_root,  /* is v the root? */
			     int* first_child, int* next_child,
			     int** sn_struct, int* sn_sizes, int* sn_up_sizes,
			     int* sn_blocks_ld,taucs_datatype* sn_blocks[],
			     int* up_blocks_ld,taucs_datatype* up_blocks[],
			     taucs_datatype x[], taucs_datatype b[],
			     taucs_datatype t[])
{
  int child;
  int  sn_size; /* number of rows/columns in the supernode    */
  int  up_size; /* number of rows that this supernode updates */

  int    ione = 1;

  taucs_datatype* xdense;
  taucs_datatype* bdense;
  double  flops;
  int i;/*ip,j,jp omer*/
	int ret = 0;

  for (child = first_child[sn]; child != -1; child = next_child[child]) {
    ret  = recursive_supernodal_solve_l(child,
						FALSE,
						first_child,next_child,
						sn_struct,sn_sizes,sn_up_sizes,
						sn_blocks_ld,sn_blocks,
						up_blocks_ld,up_blocks,
						x,b,t);
		if ( ret ) return ret;
  }

  if(!is_root) {

    sn_size = sn_sizes[sn];
    up_size = sn_up_sizes[sn] - sn_sizes[sn];


    flops = ((double)sn_size)*((double)sn_size)
      + 2.0*((double)sn_size)*((double)up_size);

    if (flops > BLAS_FLOPS_CUTOFF) {
      xdense = t;
      bdense = t + sn_size;

      for (i=0; i<sn_size; i++)
				xdense[i] = b[ sn_struct[ sn ][ i ] ];
      for (i=0; i<up_size; i++)
				bdense[i] = taucs_zero;

			if ( sn_size ) {
				taucs_trsm ("Left",
					"Lower",
					"No Conjugate",
					"No unit diagonal",
					&sn_size,&ione,
					&taucs_one_const,
					sn_blocks[sn],&(sn_blocks_ld[sn]),
					xdense       ,&sn_size);
			}

      if (up_size > 0 && sn_size > 0) {
				taucs_gemm ("No Conjugate","No Conjugate",
			    &up_size, &ione, &sn_size,
					&taucs_one_const,
					up_blocks[sn],&(up_blocks_ld[sn]),
					xdense       ,&sn_size,
					&taucs_zero_const,
					bdense       ,&up_size);
      }

      for (i=0; i<sn_size; i++)
				x[ sn_struct[ sn][ i ] ]  = xdense[i];
      for (i=0; i<up_size; i++)
				b[ sn_struct[ sn ][ sn_size + i ] ] =
					taucs_sub( b[ sn_struct[ sn ][ sn_size + i ] ] , bdense[i] );
		}
  }
	return ret;
}

static int
recursive_supernodal_solve_lt(int sn,       /* this supernode */
			      int is_root,  /* is v the root? */
			      int* first_child, int* next_child,
			      int** sn_struct, int* sn_sizes, int* sn_up_sizes,
			      int* sn_blocks_ld,taucs_datatype* sn_blocks[],
			      int* up_blocks_ld,taucs_datatype* up_blocks[],
			      taucs_datatype x[], taucs_datatype b[],
			      taucs_datatype t[])
{
	int child;
	int  sn_size; /* number of rows/columns in the supernode		*/
	int  up_size; /* number of rows that this supernode updates */
	int 	 ione = 1;
	int ret = 0;

	taucs_datatype* xdense;
	taucs_datatype* bdense;
	double	flops;
	int i;/*ip,j,jp omer*/

	if(!is_root) {

		sn_size = sn_sizes[sn];
		up_size = sn_up_sizes[sn]-sn_sizes[sn];

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
					up_blocks[sn],&(up_blocks_ld[sn]),
					xdense 			,&up_size,
					&taucs_one_const,
					bdense 			,&sn_size);

			if (sn_size)
				taucs_trsm ("Left",
					"Lower",
					"Conjugate",
					"No unit diagonal",
					&sn_size,&ione,
					&taucs_one_const,
					sn_blocks[sn],&(sn_blocks_ld[sn]),
					bdense			 ,&sn_size);

			for (i=0; i<sn_size; i++)
				x[ sn_struct[ sn][ i ] ]	= bdense[i];
		}
	}

	for (child = first_child[sn]; child != -1; child = next_child[child]) {
		ret  = recursive_supernodal_solve_lt(child,
						FALSE,
						first_child,next_child,
						sn_struct,sn_sizes,sn_up_sizes,
						sn_blocks_ld,sn_blocks,
						up_blocks_ld,up_blocks,
						x,b,t);
		if ( ret ) return ret;
	}

	return ret;
}


int
taucs_dtl(supernodal_solve_ldlt_many)(void* vL, int n,
																			void* X, int ld_X,
																			void* B, int ld_B)
{
  supernodal_factor_matrix_ldlt* L = (supernodal_factor_matrix_ldlt*) vL;
  taucs_datatype* y;
  taucs_datatype* t; /* temporary vector */
	int     j=0,i,ret = 0;
	
  y = taucs_malloc((L->n) * sizeof(taucs_datatype));
  t = taucs_malloc((L->n) * sizeof(taucs_datatype));
	if (!y || !t ) {
    taucs_free(y);
    taucs_free(t);
		taucs_printf("supernodal_solve_ldlt: out of memory\n");
    return -1;
  }

	for (j=0; j<n; j++) {
		taucs_datatype* x = (taucs_datatype*) ((taucs_datatype*)X+(j*ld_X));
		taucs_datatype* b = (taucs_datatype*) ((taucs_datatype*)B+(j*ld_B));

		/*for(i=0;i<L->n;i++)
			taucs_printf("%.4e ",b[i]);
		taucs_printf("\n");*/

		/* before we start we need:
		1. put b into y (for the l solve) */
		for (i=0; i<L->n; i++) {
			y[i] = b[i];
		}

		/*for(i=0;i<L->n;i++)
			taucs_printf("%.4e ",y[i]);
		taucs_printf("\n");*/

		ret = recursive_supernodal_solve_l (L->n_sn,
					TRUE,  /* this is the root */
					L->first_child, L->next_child,
					L->sn_struct,L->sn_size,L->sn_up_size,
					L->sn_blocks_ld, L->sn_blocks,
					L->up_blocks_ld, L->up_blocks,
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
		ret = recursive_supernodal_solve_d(L->n_sn,
					TRUE,
					L->first_child, L->next_child,
					L->sn_struct,L->sn_size,L->d_blocks, L->db_size,
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


		ret = recursive_supernodal_solve_lt(L->n_sn,
					TRUE,  /* this is the root */
					L->first_child, L->next_child,
					L->sn_struct,L->sn_size,L->sn_up_size,
					L->sn_blocks_ld, L->sn_blocks,
					L->up_blocks_ld, L->up_blocks,
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
	}
	
	taucs_free(y);
  taucs_free(t);

	return 0;
}


int
taucs_dtl(supernodal_solve_ldlt)(void* vL, void* vx, void* vb)
{
  supernodal_factor_matrix_ldlt* L = (supernodal_factor_matrix_ldlt*) vL;
  taucs_datatype* x = (taucs_datatype*) vx;
  taucs_datatype* b = (taucs_datatype*) vb;
  taucs_datatype* y;
  taucs_datatype* t; /* temporary vector */
	int     i,ret = 0;
	
  y = taucs_malloc((L->n) * sizeof(taucs_datatype));
  t = taucs_malloc((L->n) * sizeof(taucs_datatype));
	if (!y || !t ) {
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

	ret = recursive_supernodal_solve_l (L->n_sn,
				TRUE,  /* this is the root */
				L->first_child, L->next_child,
				L->sn_struct,L->sn_size,L->sn_up_size,
				L->sn_blocks_ld, L->sn_blocks,
				L->up_blocks_ld, L->up_blocks,
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
	ret = recursive_supernodal_solve_d(L->n_sn,
				TRUE,
				L->first_child, L->next_child,
				L->sn_struct,L->sn_size,L->d_blocks, L->db_size,
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


	ret = recursive_supernodal_solve_lt(L->n_sn,
				TRUE,  /* this is the root */
				L->first_child, L->next_child,
				L->sn_struct,L->sn_size,L->sn_up_size,
				L->sn_blocks_ld, L->sn_blocks,
				L->up_blocks_ld, L->up_blocks,
				x, y, t);

	if ( ret ) {
		taucs_free(y);
    taucs_free(t);
		taucs_printf("multifrontal_supernodal_solve_ldlt: No solution found.\n");
    return -1;
	}

	/*for(i=0;i<L->n;i++)
		taucs_printf("%.4e ",x[i]);
	taucs_printf("\n");*/

	
	taucs_free(y);
  taucs_free(t);

  return 0;
}
#endif /*#ifndef TAUCS_CORE_GENERAL*/

/*************************************************************/
/* generic interfaces to user-callable routines              */
/*************************************************************/

#ifdef TAUCS_CORE_DOUBLE/*GENERAL*/

void* taucs_ccs_factor_ldlt_mf_maxdepth(taucs_ccs_matrix* A,int max_depth)
{

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (A->flags & TAUCS_DOUBLE)
    return taucs_dccs_factor_ldlt_mf_maxdepth(A,max_depth);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (A->flags & TAUCS_SINGLE)
    return taucs_sccs_factor_ldlt_mf_maxdepth(A,max_depth);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_DCOMPLEX)
    return taucs_zccs_factor_ldlt_mf_maxdepth(A,max_depth);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_SCOMPLEX)
    return taucs_cccs_factor_ldlt_mf_maxdepth(A,max_depth);
#endif

  assert(0);
  return NULL;
}
void* taucs_ccs_factor_ldlt_symbolic_maxdepth(taucs_ccs_matrix* A,int
max_depth)
{

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (A->flags & TAUCS_DOUBLE)
    return taucs_dccs_factor_ldlt_symbolic_maxdepth(A,max_depth);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (A->flags & TAUCS_SINGLE)
    return taucs_sccs_factor_ldlt_symbolic_maxdepth(A,max_depth);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_DCOMPLEX)
    return taucs_zccs_factor_ldlt_symbolic_maxdepth(A,max_depth);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_SCOMPLEX)
    return taucs_cccs_factor_ldlt_symbolic_maxdepth(A,max_depth);
#endif

  assert(0);
  return NULL;
}

void* taucs_ccs_factor_ldlt_ll_maxdepth(taucs_ccs_matrix* A,int max_depth)
{
#ifdef TAUCS_DOUBLE_IN_BUILD
  if (A->flags & TAUCS_DOUBLE)
    return taucs_dccs_factor_ldlt_ll_maxdepth(A,max_depth);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (A->flags & TAUCS_SINGLE)
    return taucs_sccs_factor_ldlt_ll_maxdepth(A,max_depth);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_DCOMPLEX)
    return taucs_zccs_factor_ldlt_ll_maxdepth(A,max_depth);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_SCOMPLEX)
    return taucs_cccs_factor_ldlt_ll_maxdepth(A,max_depth);
#endif

	assert(0);
  return NULL;
}

void* taucs_ccs_factor_ldlt_ll(taucs_ccs_matrix* A)
{

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (A->flags & TAUCS_DOUBLE)
    return taucs_dccs_factor_ldlt_ll(A);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (A->flags & TAUCS_SINGLE)
    return taucs_sccs_factor_ldlt_ll(A);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_DCOMPLEX)
    return taucs_zccs_factor_ldlt_ll(A);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_SCOMPLEX)
    return taucs_cccs_factor_ldlt_ll(A);
#endif

  assert(0);
  return NULL;
}




void* taucs_ccs_factor_ldlt_mf(taucs_ccs_matrix* A)
{

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (A->flags & TAUCS_DOUBLE)
    return taucs_dccs_factor_ldlt_mf(A);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (A->flags & TAUCS_SINGLE)
    return taucs_sccs_factor_ldlt_mf(A);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_DCOMPLEX)
    return taucs_zccs_factor_ldlt_mf(A);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_SCOMPLEX)
    return taucs_cccs_factor_ldlt_mf(A);
#endif

  assert(0);
  return NULL;
}




int taucs_ccs_factor_ldlt_numeric(taucs_ccs_matrix* A, void* L)
{

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (A->flags & TAUCS_DOUBLE)
    return taucs_dccs_factor_ldlt_numeric(A,L);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (A->flags & TAUCS_SINGLE)
    return taucs_sccs_factor_ldlt_numeric(A,L);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_DCOMPLEX)
    return taucs_zccs_factor_ldlt_numeric(A,L);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_SCOMPLEX)
    return taucs_cccs_factor_ldlt_numeric(A,L);
#endif

  assert(0);
  return -1;
}

int taucs_supernodal_solve_ldlt_many(void* L, int n,
																void* X, int ld_X,
																void* B, int ld_B)
{

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_DOUBLE)
    return taucs_dsupernodal_solve_ldlt_many(L,n,X,ld_X,B,ld_B);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_SINGLE)
    return taucs_ssupernodal_solve_ldlt_many(L,n,X,ld_X,B,ld_B);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_DCOMPLEX)
    return taucs_zsupernodal_solve_ldlt_many(L,n,X,ld_X,B,ld_B);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_SCOMPLEX)
    return taucs_csupernodal_solve_ldlt_many(L,n,X,ld_X,B,ld_B);
#endif

  assert(0);
  return -1;
}


int taucs_supernodal_solve_ldlt(void* L, void* x, void* b)
{

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_DOUBLE)
    return taucs_dsupernodal_solve_ldlt(L,x,b);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_SINGLE)
    return taucs_ssupernodal_solve_ldlt(L,x,b);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_DCOMPLEX)
    return taucs_zsupernodal_solve_ldlt(L,x,b);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_SCOMPLEX)
    return taucs_csupernodal_solve_ldlt(L,x,b);
#endif

  assert(0);
  return -1;
}

void taucs_supernodal_factor_ldlt_free(void* L)
{

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_DOUBLE) {
    taucs_dsupernodal_factor_ldlt_free(L);
    return;
  }
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_SINGLE) {
    taucs_ssupernodal_factor_ldlt_free(L);
    return;
  }
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_DCOMPLEX) {
    taucs_zsupernodal_factor_ldlt_free(L);
    return;
  }
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_SCOMPLEX) {
    taucs_csupernodal_factor_ldlt_free(L);
    return;
  }
#endif

  assert(0);
}

void taucs_supernodal_factor_ldlt_free_numeric(void* L)
{


#ifdef TAUCS_DOUBLE_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_DOUBLE) {
    taucs_dsupernodal_factor_ldlt_free_numeric(L);
    return;
  }
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_SINGLE) {
    taucs_ssupernodal_factor_ldlt_free_numeric(L);
    return;
  }
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_DCOMPLEX) {
    taucs_zsupernodal_factor_ldlt_free_numeric(L);
    return;
  }
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_SCOMPLEX) {
    taucs_csupernodal_factor_ldlt_free_numeric(L);
    return;
  }
#endif

  assert(0);
}



taucs_ccs_matrix*
taucs_supernodal_factor_ldlt_to_ccs(void* L)
{

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_DOUBLE)
    return taucs_dsupernodal_factor_ldlt_to_ccs(L);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_SINGLE)
    return taucs_ssupernodal_factor_ldlt_to_ccs(L);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_DCOMPLEX)
    return taucs_zsupernodal_factor_ldlt_to_ccs(L);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_SCOMPLEX)
    return taucs_csupernodal_factor_ldlt_to_ccs(L);
#endif

  assert(0);
  return NULL;
}



taucs_ccs_matrix*
taucs_supernodal_factor_diagonal_to_ccs(void* L)
{

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_DOUBLE)
    return taucs_dsupernodal_factor_diagonal_to_ccs(L);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_SINGLE)
    return taucs_ssupernodal_factor_diagonal_to_ccs(L);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_DCOMPLEX)
    return taucs_zsupernodal_factor_diagonal_to_ccs(L);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) L)->flags & TAUCS_SCOMPLEX)
    return taucs_csupernodal_factor_diagonal_to_ccs(L);
#endif

  assert(0);
  return NULL;
}

void	taucs_get_statistics(int* pbytes,double* pflops,double* pnnz,void* vL)
{
#ifdef TAUCS_DOUBLE_IN_BUILD
	if (((supernodal_factor_matrix_ldlt*) vL)->flags & TAUCS_DOUBLE) {
    taucs_dget_statistics(pbytes,pflops,pnnz,vL);
		return;
	}
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
	if (((supernodal_factor_matrix_ldlt*) vL)->flags & TAUCS_SINGLE) {
    taucs_sget_statistics(pbytes,pflops,pnnz,vL);
		return;
	}
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
	if (((supernodal_factor_matrix_ldlt*) vL)->flags & TAUCS_DCOMPLEX) {
    taucs_zget_statistics(pbytes,pflops,pnnz,vL);
		return;
	}
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
	if (((supernodal_factor_matrix_ldlt*) vL)->flags & TAUCS_SCOMPLEX) {
    taucs_cget_statistics(pbytes,pflops,pnnz,vL);
		return;
	}
#endif

  assert(0);
}
void	taucs_inertia_calc(void* vL, int* inertia)
{
#ifdef TAUCS_DOUBLE_IN_BUILD
	if (((supernodal_factor_matrix_ldlt*) vL)->flags & TAUCS_DOUBLE) {
    taucs_dinertia_calc(vL,inertia);
		return;
	}
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
	if (((supernodal_factor_matrix_ldlt*) vL)->flags & TAUCS_SINGLE) {
    taucs_sinertia_calc(vL,inertia);
		return;
	}
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) vL)->flags & TAUCS_DCOMPLEX)
    return;
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (((supernodal_factor_matrix_ldlt*) vL)->flags & TAUCS_SCOMPLEX)
    return;
#endif

  assert(0);
}



int
taucs_ccs_ldlt_symbolic_elimination(taucs_ccs_matrix* A,
			       void* vL,
			       int do_order,
			       int max_depth
			       )
{
  supernodal_factor_matrix_ldlt* L = (supernodal_factor_matrix_ldlt*) vL;
  int* first_child;
  int* next_child;
  int j;
  int* column_to_sn_map;
  int* map;
  int* rowind;
  int* parent;
  int* ipostorder;

  int depth;

  L->n           = A->n;
  /* use calloc so we can deallocate unallocated entries */
  L->sn_struct   = (int**)taucs_calloc((A->n  ),sizeof(int*));
  L->sn_size     = (int*) taucs_malloc((A->n+1)*sizeof(int));
	L->orig_sn_size = (int*) taucs_malloc((A->n+1)*sizeof(int));
  L->sn_up_size  = (int*) taucs_malloc((A->n+1)*sizeof(int));
  L->first_child = (int*) taucs_malloc((A->n+1)*sizeof(int));
  L->next_child  = (int*) taucs_malloc((A->n+1)*sizeof(int));
	L->sn_max_ind = (int*) taucs_malloc((A->n+1)*sizeof(int));
	L->sn_rej_size = (int*) taucs_malloc((A->n+1)*sizeof(int));

  column_to_sn_map = (int*) taucs_malloc((A->n+1)*sizeof(int));
  map              = (int*) taucs_malloc((A->n+1)*sizeof(int));
  first_child      = (int*) taucs_malloc((A->n+1)*sizeof(int));
  next_child       = (int*) taucs_malloc((A->n+1)*sizeof(int));
  parent           = (int*) taucs_malloc((A->n+1)*sizeof(int));
  rowind           = (int*) taucs_malloc((A->n  )*sizeof(int));

  if (!(L->sn_struct) || !(L->sn_size) || !(L->sn_up_size) || !(L->orig_sn_size) ||
      !(L->first_child) || !(L->next_child) || !column_to_sn_map
      || !map || !first_child || !next_child || !rowind || !parent ||
			!L->sn_max_ind || !L->sn_rej_size ) {
    taucs_free(parent);
    taucs_free(rowind);
    taucs_free(next_child);
    taucs_free(first_child);
    taucs_free(map);
    taucs_free(column_to_sn_map);
    taucs_free(L->next_child);
    taucs_free(L->first_child);
    taucs_free(L->sn_up_size);
    taucs_free(L->sn_size);
    taucs_free(L->sn_struct);
		taucs_free(L->orig_sn_size);
		taucs_free(L->sn_max_ind);
		taucs_free(L->sn_rej_size);
    L->sn_struct = NULL;
    L->sn_size = L->sn_up_size = L->first_child = L->next_child = NULL;
		L->orig_sn_size = NULL;
    return -1;
  }

  if (taucs_ccs_etree(A,parent,NULL,NULL,NULL) == -1) {
    taucs_free(parent);
    taucs_free(rowind);
    taucs_free(next_child);
    taucs_free(first_child);
    taucs_free(map);
    taucs_free(column_to_sn_map);
    taucs_free(L->next_child);
    taucs_free(L->first_child);
    taucs_free(L->sn_up_size);
    taucs_free(L->sn_size);
    taucs_free(L->sn_struct);
		taucs_free(L->orig_sn_size);
		taucs_free(L->sn_max_ind);
		taucs_free(L->sn_rej_size);
		L->orig_sn_size = L->sn_max_ind = L->sn_rej_size = NULL;
    L->sn_struct = NULL;
    L->sn_size = L->sn_up_size = L->first_child = L->next_child = NULL;
    return -1;
  }

  if (0) {
    double wtime;
    int *cc1,*cc2,*rc1,*rc2;
    int *p1;
    int nnz1,nnz2;

    cc1=(int*)taucs_malloc((A->n)*sizeof(int));
    cc2=(int*)taucs_malloc((A->n)*sizeof(int));
    rc1=(int*)taucs_malloc((A->n)*sizeof(int));
    rc2=(int*)taucs_malloc((A->n)*sizeof(int));
    p1 =(int*)taucs_malloc((A->n)*sizeof(int));

    wtime = taucs_wtime();
    taucs_ccs_etree_liu(A,parent,cc1,rc1,&nnz1);
    wtime = taucs_wtime() - wtime;
    printf("\t\t\tLiu Analysis = %.3f seconds\n",wtime);

    wtime = taucs_wtime();
    taucs_ccs_etree(A,p1,cc2,rc2,&nnz2);
    wtime = taucs_wtime() - wtime;
    printf("\t\t\tGNP Analysis = %.3f seconds\n",wtime);

    for (j=0; j<(A->n); j++) assert(parent[j]==p1[j]);
    for (j=0; j<(A->n); j++) {
      if (cc1[j]!=cc2[j]) printf("j=%d cc1=%d cc2=%d\n",j,cc1[j],cc2[j]);
      assert(cc1[j]==cc2[j]);
    }

    for (j=0; j<(A->n); j++) {
      if (rc1[j]!=rc2[j]) printf("j=%d rc1=%d rc2=%d\n",j,rc1[j],rc2[j]);
      assert(rc1[j]==rc2[j]);
    }

    if (nnz1!=nnz2) printf("nnz1=%d nnz2=%d\n",nnz1,nnz2);

    taucs_free(cc1); taucs_free(cc2); taucs_free(rc1); taucs_free(rc2);
  }

  for (j=0; j <= (A->n); j++) first_child[j] = -1;
  for (j = (A->n)-1; j >= 0; j--) {
    int p = parent[j];
    next_child[j] = first_child[p];
    first_child[p] = j;
  }

  /* let's compute the depth of the etree, to bail out if it is too deep */
  /* the whole thing will work better if we compute supernodal etrees    */

  {
    int next_depth_count;
    int this_depth_count;
    int child,i;

    int* this_depth = rowind; /* we alias rowind */
    int* next_depth = map;    /* and map         */
    int* tmp;

    this_depth[0] = A->n;
    this_depth_count = 1;
    next_depth_count = 0;
    depth = -1;

    while (this_depth_count) {
      for (i=0; i<this_depth_count; i++) {
	child = first_child[ this_depth[i] ];
	while (child != -1) {
	  next_depth[ next_depth_count ] = child;
	  next_depth_count++;
	  child = next_child[ child ];
	}
      }

      tmp = this_depth;
      this_depth = next_depth;
      next_depth = tmp;

      this_depth_count = next_depth_count;
      next_depth_count = 0;
      depth++;
    }
  }

  taucs_printf("\t\tElimination tree depth is %d\n",depth);

  if (max_depth && depth > max_depth) {
    taucs_printf("taucs_ccs_ldlt_symbolic_elimination: etree depth %d, maximum allowed is %d\n",
		 depth, max_depth);
    taucs_free(parent);
    taucs_free(rowind);
    taucs_free(next_child);
    taucs_free(first_child);
    taucs_free(map);
    taucs_free(column_to_sn_map);
    taucs_free(L->next_child);
    taucs_free(L->first_child);
    taucs_free(L->sn_up_size);
    taucs_free(L->sn_size);
    taucs_free(L->sn_struct);
		taucs_free(L->orig_sn_size);
		taucs_free(L->sn_max_ind);
		taucs_free(L->sn_rej_size);
		L->orig_sn_size = L->sn_max_ind = L->sn_rej_size = NULL;
    L->sn_struct = NULL;
    L->sn_size = L->sn_up_size = L->first_child = L->next_child = NULL;
    return -1;
  }

  /*
  taucs_free(parent);
  ipostorder = (int*)taucs_malloc((A->n+1)*sizeof(int));
  */

  ipostorder = parent;
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

  L->n_sn = 0;
  for (j=0; j < (A->n); j++) map[j] = -1;
  for (j=0; j <= (A->n); j++) (L->first_child)[j] = (L->next_child)[j] = -1;

  if (recursive_symbolic_elimination(A->n,
				     A,
				     first_child,next_child,
				     &(L->n_sn),
				     L->sn_size,L->sn_up_size,L->sn_struct,
				     L->first_child,L->next_child,
				     rowind,
				     column_to_sn_map,
				     map,
				     do_order,ipostorder
				     )
      == -1) {
    for (j=0; j < (A->n); j++) taucs_free((L->sn_struct)[j]);

    taucs_free(parent);
    taucs_free(rowind);
    taucs_free(next_child);
    taucs_free(first_child);
    taucs_free(map);
    taucs_free(column_to_sn_map);
    taucs_free(L->next_child);
    taucs_free(L->first_child);
    taucs_free(L->sn_up_size);
    taucs_free(L->sn_size);
    taucs_free(L->sn_struct);
		taucs_free(L->orig_sn_size);
		taucs_free(L->sn_max_ind);
		taucs_free(L->sn_rej_size);
		L->orig_sn_size = L->sn_max_ind = L->sn_rej_size = NULL;
    L->sn_struct = NULL;
    L->sn_size = L->sn_up_size = L->first_child = L->next_child = NULL;
    return -1;
  }

  {
    double nnz   = 0.0;
    double flops = 0.0;
    int bytes;

		taucs_get_statistics(&bytes,&flops,&nnz,L);

    taucs_printf("\t\tSymbolic Analysis of LDL^T: %.2e nonzeros, %.2e flops, %.2e bytes in L\n",
		 nnz, flops, (float) bytes);
  }

  for (j=0; j < (A->n); j++) map[j] = -1;
  if (1)
  (void) recursive_amalgamate_supernodes((L->n_sn) - 1,
					 &(L->n_sn),
					 L->sn_size,L->sn_up_size,L->sn_struct,
					 L->first_child,L->next_child,
					 rowind,
					 column_to_sn_map,
					 map,
					 do_order,ipostorder
					 );


  {
    double nnz   = 0.0;
    double flops = 0.0;
    int bytes;

		taucs_get_statistics(&bytes,&flops,&nnz,L);

    taucs_printf("\t\tRelaxed  Analysis of LDL^T: %.2e nonzeros, %.2e flops, %.2e bytes in L\n",
		 nnz, flops, (float) bytes);
  }

  /*
  {
    int i;
    printf("c2sn: ");
    for (i=0; i<A->n; i++) printf("%d ",column_to_sn_map[i]);
    printf("\n");
  }
  */

  taucs_free(parent);
  taucs_free(rowind);
  taucs_free(map);
  taucs_free(column_to_sn_map);
  taucs_free(next_child);
  taucs_free(first_child);

  L->sn_blocks_ld  = taucs_malloc((L->n_sn) * sizeof(int));
  L->sn_blocks     = taucs_calloc((L->n_sn), sizeof(taucs_datatype*)); /* so
we can free before allocation */

  L->up_blocks_ld  = taucs_malloc((L->n_sn) * sizeof(int));
  L->up_blocks     = taucs_calloc((L->n_sn), sizeof(taucs_datatype*));

	L->db_size = (int**) taucs_calloc((L->n_sn),sizeof(int*));
	L->d_blocks = (taucs_datatype**) taucs_calloc((L->n_sn),sizeof(taucs_datatype*));

  if (!(L->sn_blocks_ld)
      || !(L->sn_blocks_ld)
      || !(L->sn_blocks)
      || !(L->up_blocks_ld)
      || !(L->up_blocks)
			|| !(L->db_size)
			|| !(L->d_blocks ))
    return -1; /* the caller will free L */

  return 0;
}

#endif


/*************************************************************/
/* end of file                                               */
/*************************************************************/



















