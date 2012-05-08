/*************************************************************************************
 * TAUCS                                                                             
 * Author: Sivan Toledo                                                              
 *
 * MULTILU - TAUCS's unsymmetric multifrontal linear solver 
 * Contributed by Haim Avron
 *
 *************************************************************************************/

#include <assert.h>
#include <string.h>

#define TAUCS_CORE_CILK
#include <numeric_stuff/TAUCS/taucs.h>

#ifndef TAUCS_CORE_GENERAL
#include <numeric_stuff/TAUCS/taucs_dense.h>
#endif

#ifdef TAUCS_CILK
#pragma lang -C
#endif

/*************************************************************************************
 *************************************************************************************
 * Internal decleartions
 *************************************************************************************
 *************************************************************************************/
#define MULTILU_SYMBOLIC_NONE -1

#define TRUE  1
#define FALSE 0

#define OK       1
#define FAILURE -1

/*************************************************************************************
 *************************************************************************************
 * COMPILE-TIME PARAMETERS
 *************************************************************************************
 *************************************************************************************/

/*************************************************************************************
 * Compile-time parameters for symbolic phase
 *************************************************************************************/

/* 
 * MULTILU_MAX_SUPERCOL_SIZE: Maximum size of supercolumn. -1 to deactivate 
 */
#define MULTILU_MAX_SUPERCOL_SIZE      -1


/* 
 * MULTILU_MAX_OVERFILL_RATIO: When we do the symbolic elimination we calculate an 
 *                             upper bound on the number of non-zeros of L and U. 
 *                             Later we build supercolumns by uniting chains of 
 *                             one-childed columns. This process enlarges the
 *                             upperbound on the non-zeros of L and U. We call this 
 *                             overfill. We allow a maximum size of the new upperbound
 *                             as a factor of the original upperbound.
 *                          
 */
#define MULTILU_MAX_OVERFILL_RATIO      2

/* 
 * MULTILU_RELAX_RULE_SIZE: When doing the relax phase (attempting to unite leaf  
 *                          supercolumns) we unite supercolumn i with its parent p if  
 *                          the last column of p has at most MULTIUL_RELAX_RULE_SIZE 
 *                          descendents.  
 */
#define MULTILU_RELAX_RULE_SIZE        20

/*
 * MULTILU_EAN_BUFFER: When doing symbolic analysis we have an extra buffer for holding 
 *                     rows. This defines how big this buffer is (MULTILU_EAN_BUFFER times 
 *                     the number of columns).
 */
#define MULTILU_EAN_BUFFER              2

/*
 * _UF_UNION_BY_RANK: Define it if we want union by rank in the union-find library
 */
/*#define _UF_UNION_BY_RANK */

/*************************************************************************************
 * Compile-time parameters for numeric phase
 *************************************************************************************/

/*
 * MULTILU_MIN_COVER_SPRS_SPAWN: Minimum number of covered columns a supercolumn must 
 *                               have inorder to have it childs spawned recursively 
 */
#define MULTILU_MIN_COVER_SPRS_SPAWN   -1

/*
 * MULTILU_MIN_SIZE_DENSE_SPAWN: Minimum supercolumn size for calling parallel dense
 *                               function for factoring the block.
 */
#define MULTILU_MIN_SIZE_DENSE_SPAWN   -1

/*
 * MULTILU_ALIGN_ADD_SMALL: TODO!
 */
#define MULTILU_ALIGN_ADD_SMALL        80

/*************************************************************************************
 *************************************************************************************
 * STRUCTURES AND TYPES
 *************************************************************************************
 *************************************************************************************/
#if defined(TAUCS_CORE_GENERAL) || !defined(TAUCS_CORE_GENERAL)

/*************************************************************************************
 * Structure: multilu_etree
 *
 * Description: Defines the etree structure that is calculated on the preordered matrix
 * Members:
 *   first_root - index of the first root. all other roots are brothers of this root.
 *   parent - given for each column it's parent in the etree
 *   first_child - for each node gives it's first child. none is -1
 *   next_child - for each node gives the next child of the same parent
 *   first_desc_index - index of the first descendnt in the column order
 *   last_desc_index - index of the last descendnt in the column order
 *
 *************************************************************************************/
struct taucs_multilu_etree_st
{
  int first_root;      
  int *parent;
  int *first_child;
  int *next_child;
  int *first_desc_index;
  int *last_desc_index;
};

/*************************************************************************************
 * Structure: taucs_multilu_symbolic
 *
 * Description: Symbolic information describing the breakage of the matrix into 
 *              supercolumns and the structure of the resultent factorization using
 *              that structure.
 *
 *************************************************************************************/
struct taucs_multilu_symbolic_st
{
  int n;
	
  /* Super column description */
  int *columns;
  int number_supercolumns;
  int *start_supercolumn;
  int *end_supercolumn;
  int *supercolumn_size;
  int *supercolumn_covered_columns;  

  /* Symbolic information */
  int *l_size;
  int *u_size;
  taucs_multilu_etree etree;
};

/* Contribution blocks. */
/* Not preallocated to avoid extreme memory useage */
/*************************************************************************************
 * Structure: multilu_contrib_block
 *
 * Description: During factorization we keep the matrix left to factor in a series
 *              of contribution blocks. This structure defines them
 *
 *************************************************************************************/
typedef struct
{
  /* rows and columns hold the relevent row/column number. If none then -1 */
  /* The _loc members are the columns densed. They point to the rows/columns array */
  int m, n, ld;
  int *columns, *col_loc;
  int *rows, *row_loc;
  taucs_datatype *values;        /* Dense! */

  /* When factorizing an only-child we will put the parents columns on the start and
     the rest following. Here we will mark how many are in the parents */
  int num_cols_in_parent;

  /* During calculation every assembeled contrib block is a L_member and/or U_member */
  /* To avoid recalculation (and to help with other things) we keep it on on the block */
  /* The structure of parallelism will ensure that there is no collision... */
  /* A bit ugly but will work very good. */
  int L_member, U_member;

} multilu_contrib_block;

/*************************************************************************************
 * Structure: multilu_factor_block
 *
 * Description: This sturcture defines a section of the LU factor. For L it defines
 *              a the values of a set of columns (that are one after the other in 
 *              the column ordering). For U it defines the values of the corresponding
 *              rows (those that are pivotal at that columns).
 *
 *              Thus we have the set of columns in L (call pivot_cols) and the set 
 *              set of rows of U. The columns of L have indices of rows that are 
 *              not in U, these are the non_pivot_rows. Likewise rows of U have
 *              non_pivot_cols. 
 *
 *              The L supercolumn and U the superrow are stored in dense form, but
 *              in three matrices.  LU1 stores the pivotal parts of L and U. The lower
 *              triangle for L and upper triangle for U. L2 is the non-pivotal part of 
 *              the supercolumn. Ut2 is the non-pivotal part of U in row-major format.
 * 
 *               +-------+---------------+
 *               |\      |               |
 *               | \ U1  |               |
 *               |  \    |    (Ut2)'     | 
 *               |   \   |               |
 *               | L1 \  |               |
 *               |     \ |               |
 *               +-------+---------------+
 *               |       |
 *               |       |
 *               |  L2   |
 *               |       |
 *               |       |
 *               |       |
 *               +-------+ 
 *      
 *************************************************************************************/
typedef struct _multilu_factor_block
{
  /* Valid flag */
  int valid;

  /* Pivot rows and column numbers */
  int row_pivots_number, col_pivots_number;
  int *pivot_rows, *pivot_cols;
  
  /* Non-pivot rows and column numbers */
  int non_pivot_rows_number, non_pivot_cols_number;
  int *non_pivot_rows, *non_pivot_cols;
  
  /* Total L size */
  int l_size;

  /* The actual matrices */
  taucs_datatype *LU1, *L2, *Ut2;

  /* 
   * This is the associated factor block. At the end of factorization this empty!
   * It is inside the factor block structure to enable quick association at 
   * factor-time
   */
  multilu_contrib_block *contrib_block;

} multilu_factor_block;

/*************************************************************************************
 * Structure: multilu_blocked_factor
 *
 * Description: Result of the factorization in "blocked" form. This is basically a
 *              series of factor blocks that are stored "in-order". Yet all the blocks
 *              are allocated in big worksspaces, this workspaces pointers are stored
 *              here too.
 *              
 *************************************************************************************/
struct taucs_multilu_factor_st
{
  /* Factor blocks */
  int num_blocks;
  multilu_factor_block **blocks;
  
  /* Size of matrices */
  int m, n;

  /* Number type */
  int type;
};

#endif /* both with core general and not we define the structure. not that they will
          differ if compiled in different mode (but only in pointers). */

/*************************************************************************************
 *************************************************************************************
 * SYMBOLIC FACTORIZATION
 *************************************************************************************
 *************************************************************************************/

#ifdef TAUCS_CORE_GENERAL

/*************************************************************************************
 * Function prototypes 
 *************************************************************************************/
static taucs_multilu_symbolic *allocate_symbolic(int n);
static void complete_symbolic(taucs_multilu_symbolic *symbolic);
static int elimination_analysis(taucs_ccs_matrix *A, int *column_order, 
				 int *parent, int *l_size, int *u_size); 
static int detect_supercol(taucs_ccs_matrix *A, int *column_order, int *one_child, 
			   int *l_size, int *u_size, int *postorder,
 			   int *desc_count, int *sc_num, int *sc_size, int *sc_parent);
static int df_postorder(int *first_child, int *next_child, int root, int *postorder, int *desc_count);
static int garbage_collect(int *workspace, int *el_start, int *el_sizem, int *el_cleared, int el_num);

/*************************************************************************************
 * Union-find functions prototypes, defined later 
 *************************************************************************************/
static int uf_find(void *sets, int x);
static int uf_union(void *sets, int x, int y);
static void *uf_make_sets(int sets_num);

/*************************************************************************************
 * Sub-system API functions 
 *************************************************************************************/

/*************************************************************************************
 * Function: taucs_ccs_factor_lu_symbolic
 *
 * Description: Calculate the symbolic information need for multilu, and thus 
 *              taucs, inorder to factorize A when the row order is given.
 *
 *************************************************************************************/
taucs_multilu_symbolic *taucs_ccs_factor_lu_symbolic(taucs_ccs_matrix *A, int *column_order)
{
  int *parent, *l_size, *u_size, *postorder, *desc_count_org, *desc_count; 
  int *first_child, *next_child, *one_child;
  taucs_multilu_symbolic *symbolic;
  int i, j, firstcol_ind, t;
  
  TAUCS_PROFILE_START(taucs_profile_multilu_sym_anlz);
  
  /* TODO: Make this smarter (memory + time) */
  
  /* Preallocate memory */
  symbolic = allocate_symbolic(A->n);
  l_size = taucs_malloc((A->n + 1) * sizeof(int));
  u_size = taucs_malloc((A->n + 1) * sizeof(int));
  postorder = taucs_malloc(A->n * sizeof(int));
  desc_count_org = taucs_malloc(A->n * sizeof(int));
  one_child = taucs_calloc(A->n, sizeof(int));
  desc_count = taucs_malloc(A->n * sizeof(int));
  if (symbolic == NULL || 
      l_size == NULL || u_size == NULL || 
      postorder == NULL || desc_count_org == NULL || desc_count == NULL ||
      one_child == NULL)
    {
      taucs_printf("ttt: out of memory\n");
      taucs_multilu_symbolic_free(symbolic);
      symbolic = NULL;
      goto free_and_exit;
    }

  /* Do elimination analysis */
  parent = symbolic->etree.parent;
  t = elimination_analysis(A, column_order, parent, l_size, u_size);
  if (t == FAILURE)
    {
      taucs_multilu_symbolic_free(symbolic);
      symbolic = NULL;
      goto free_and_exit;
    }

  /* Create the tree in first_child, next_child form */
  first_child = symbolic->etree.first_child;
  next_child = symbolic->etree.next_child;
  for (i = 0; i < A->n + 1; i++)
    first_child[i] = MULTILU_SYMBOLIC_NONE;
  memset(first_child, MULTILU_SYMBOLIC_NONE, (A->n + 1) * sizeof(int));
  for (i = A->n - 1; i >= 0; i--) 
    {
      int p;
    
      p = parent[i];
      next_child[i] = first_child[p];
      first_child[p] = i;
    }
  
  /* Reorder the columns by depth-first postorder */
  /* A current upperbound on u_size is the depth of the cetree */
  t = df_postorder(first_child, next_child, A->n, postorder, desc_count_org);
  if (t == FAILURE)
    {
      taucs_multilu_symbolic_free(symbolic);
      symbolic = NULL;
      goto free_and_exit;
    }
  
  /* Determine for each node whether it has one child exactly and wheter it is leaf */
  for(i = 0; i < A->n; i++)
    {
      int col;
    
      col = postorder[i];
      if (first_child[col] != MULTILU_SYMBOLIC_NONE && next_child[first_child[col]] == MULTILU_SYMBOLIC_NONE)
	one_child[i] = TRUE;
    }
  
  /* Apply the ordering to columns and desc_count */
  /* TODO: do desc_count in one pass, no allocation twice */	
  for(i = 0; i < A->n; i++)
    {
      symbolic->columns[i] = column_order[postorder[i]];
      desc_count[i] = desc_count_org[postorder[i]];
    }
  
  /* Detect supercols by redoing the elimination process */
  t = detect_supercol(A, symbolic->columns, one_child, desc_count, l_size, u_size, postorder,
		      &symbolic->number_supercolumns, symbolic->supercolumn_size,
		      symbolic->etree.parent);
  if (t == FAILURE)
    {
      taucs_multilu_symbolic_free(symbolic);
      symbolic = NULL;
      goto free_and_exit;
    }

  /* Find l_size and u_size of supercolumns */
  firstcol_ind = 0;
  for(i = 0; i < symbolic->number_supercolumns; i++)
    {
      if (MULTILU_RELAX_RULE_SIZE == 0 && MULTILU_MAX_OVERFILL_RATIO == 1)
	{
	  symbolic->l_size[i] = l_size[postorder[firstcol_ind]];
	  symbolic->u_size[i] = u_size[postorder[firstcol_ind]];
	  firstcol_ind += symbolic->supercolumn_size[i];
	} else
	  {
	    symbolic->l_size[i] = 0;
	    symbolic->u_size[i] = 0;
	    for(j = 0; j < symbolic->supercolumn_size[i]; j++)
	      {
		symbolic->l_size[i] = max(symbolic->l_size[i], l_size[postorder[firstcol_ind + j]] + j);
		symbolic->u_size[i] = max(symbolic->u_size[i], u_size[postorder[firstcol_ind + j]] + j);
	      }
	    firstcol_ind += symbolic->supercolumn_size[i];
	  }
    }
  
  /* Complete the rest of symbolic data */
  complete_symbolic(symbolic);

 free_and_exit:
  taucs_free(l_size);
  taucs_free(u_size);
  taucs_free(postorder);
  taucs_free(desc_count_org);
  taucs_free(one_child);

  TAUCS_PROFILE_STOP(taucs_profile_multilu_sym_anlz);
  
  return symbolic;
}

/*************************************************************************************
 * Function: taucs_multilu_symbolic_free
 *
 * Description: Free the memory associated with the symbolic data
 *
 *************************************************************************************/
void taucs_multilu_symbolic_free(taucs_multilu_symbolic *symbolic)
{
  if (symbolic == NULL) 
    return;

  taucs_free(symbolic->etree.parent);
  taucs_free(symbolic->etree.first_child);
  taucs_free(symbolic->etree.next_child);
  taucs_free(symbolic->etree.first_desc_index);
  taucs_free(symbolic->etree.last_desc_index);
  taucs_free(symbolic->start_supercolumn);
  taucs_free(symbolic->end_supercolumn);
  taucs_free(symbolic->supercolumn_size);
  taucs_free(symbolic->columns);
  taucs_free(symbolic->l_size);
  taucs_free(symbolic->u_size);
  taucs_free(symbolic);
}

/*************************************************************************************
 * Sub-system Internal functions 
 *************************************************************************************/

/*************************************************************************************
 * Function: elimination_analysis
 *
 * Description: Do the column elimination analysis on A given the supplied order.
 *              Eliminition analysis simulates the factoring of the matrix using
 *              the row-merge matrix. Using this method it finds the column etree
 *              and an upperbound on the L-colcount. l_size[i] gives the column count
 *              when factoring the i'th column in factorization phase, aka factoring
 *              column column_order[i]. u_size[i] gives the row count when factoring
 *              the i'th pivot
 *
 * Algorithm:   Gilbert + Ng 
 *              "Predicting Structure in Nonsymmetric Sparse Matrix Factorization"
 *              Davis + Gilbert + Larimore + Ng
 *              "A column approximate minimum degree ordering algorithm"
 *
 *************************************************************************************/
static int elimination_analysis(taucs_ccs_matrix *A, 
				int *column_order, int *parent, 
				int *l_size, int *u_size) 
{
  int *firstcol, *root, *rdegs, *rnums, *row_workspace, *rows_start, *rows_size;
  int *col_cleared, *row_cleared, *col_mmb;
  void *sets;
  int i, j, col, next_row;
  int status = OK;

  /* Preallocate memory */
  firstcol = taucs_malloc(A->m * sizeof(int));
  root = taucs_malloc(A->n * sizeof(int));
  rdegs = taucs_malloc(A->n * sizeof(int));
  rnums = taucs_malloc(A->n * sizeof(int));
  sets = uf_make_sets(A->n);
  col_cleared = taucs_calloc(A->n, sizeof(int));
  row_cleared = taucs_calloc(A->m, sizeof(int));
  col_mmb = taucs_calloc(A->n, sizeof(int));
  row_workspace = taucs_malloc((A->colptr[A->n] + MULTILU_EAN_BUFFER * A->n) * sizeof(int));
  rows_start = taucs_malloc(A->m * sizeof(int));
  rows_size = taucs_calloc(A->m, sizeof(int));
  if (firstcol == NULL || root == NULL || rdegs == NULL ||
      rnums == NULL || sets == NULL || col_cleared == NULL ||
      row_cleared == NULL || col_mmb == NULL || row_workspace == NULL ||
      rows_start == NULL || rows_size == NULL)
    {
      status = FAILURE;
      goto free_and_exit;
    }

  /* Initilizations */
  for(i = 0; i < A->m; i++)
    firstcol[i] = A->n;

  for(i = 0; i < A->colptr[A->n]; i++)
    rows_size[A->rowind[i]]++;

  rows_start[0] = 0;
  for(i = 1; i < A->m; i++)
    rows_start[i] = rows_start[i - 1] + rows_size[i - 1];

  /* Now put values and indexs into the new matrix. For growing row index use row size array */
  memset(rows_size, 0, A->m * sizeof(int));
  for(i = 0; i < A->n; i++)
    for(j = A->colptr[i]; j < A->colptr[i + 1]; j++)
      {
	int row = A->rowind[j];
	int index = rows_start[row] + rows_size[row];

	row_workspace[index] = i;
	rows_size[row]++;
      }
  next_row = A->colptr[ A->n ] /* A->nnz */;

  /* Go over columns in order... */
  for(col = 0; col < A->n; col++) 
    {
      int *rowind_column;
      int nnz_column;
      int org_col, cset;
      int row_start, row_size;

      /* Do garbage collection if needed */
      if (next_row + A->n - col > A->colptr[ A->n ] /* A->nnz */ + MULTILU_EAN_BUFFER * A->n)
	next_row = garbage_collect(row_workspace, rows_start, rows_size, row_cleared, A->m);
      row_start = next_row;;
      row_size = 0;

      org_col = column_order[col];

      /* TODO: Handle the case of empty column. */
      assert(A->colptr[org_col + 1] > A->colptr[org_col]);

      cset = col;

      /* This is the actual values for this column */
      nnz_column = A->colptr[org_col + 1] - A->colptr[org_col];
      rowind_column = A->rowind + A->colptr[org_col];

      /* Initilization for this column */
      root[cset] = col;
      parent[col] = A->n;
      rdegs[cset] = 0;
    		
      /* Go over non-zeros of this column */
	       
      for(i = 0; i < nnz_column; i++) 
	{     
	  int row, fcol;

	  row = rowind_column[i];
	  fcol = firstcol[row];
      
	  /* If this is the first appearance of this row then write column */
	  /* Otherwise unite this row (if not already done so) */
	  if (fcol == A->n) 
	    {
	      firstcol[row] = col;
	      rdegs[cset]++;

	      /* Add this row to the structure */
	      for(j = 0; j < rows_size[row]; j++)
		{
		  int col;

		  col = row_workspace[rows_start[row] + j];
		  if (!col_cleared[col] && !col_mmb[col])
		    {
		      row_workspace[row_start + row_size] = col;
		      col_mmb[col] = TRUE;
		      row_size++;
		    }
		}

	      /* Mark row as cleared */
	      row_cleared[row] = TRUE;
	    } 
	  else 
	    {
	      int rset, rroot;

	      rset = uf_find(sets, fcol);
	      rroot = root[rset]; 
	      if (rroot != col) 
		{
		  int cset_old, rnum;

		  /* Merge row pattern */
		  rnum = rnums[rset];
		  for(j = 0; j < rows_size[rnum]; j++)
		    {
		      int col;

		      col = row_workspace[rows_start[rnum] + j];
		      if (!col_cleared[col] && !col_mmb[col])
			{
			  row_workspace[row_start + row_size] = col;
			  col_mmb[col] = TRUE;
			  row_size++;
			}
		    }
		  row_cleared[rnum] = TRUE;					

		  /* Now do merge in the groups */
		  parent[rroot] = col;
		  cset_old = cset;
		  cset = uf_union(sets, cset, rset);
		  rdegs[cset] = rdegs[cset_old] + rdegs[rset];
		  root[cset] = col;
		}
	    }
	}
		

      /* l_size is the number of rows overall and u_size is the size of the united row */
      /* Also update the inner degrees of the rows */
      l_size[col] = rdegs[cset];
      assert(row_size > 0);
      u_size[col] = row_size;
      rdegs[cset] = max(0, rdegs[cset] - 1); /* Because we eliminate one row */

      /* Give a "real" number to the united row */
      rnums[cset] = rowind_column[0];
      rows_start[rnums[cset]] = row_start;
      rows_size[rnums[cset]] = row_size;
      row_cleared[rnums[cset]] = FALSE;
		
      /* Clear the column member indication */
      for (j = 0; j < row_size; j++)
	col_mmb[row_workspace[row_start + j]] = FALSE;

      /* Set where to put the next row */
      next_row = row_start + row_size;

      /* Mark this column as cleared (using orginal col numbers) */
      col_cleared[org_col] = TRUE;
    }

  /* Free memory */
 free_and_exit:
  taucs_free(root);
  taucs_free(firstcol);
  taucs_free(sets);
  taucs_free(rdegs);
  taucs_free(col_cleared);
  taucs_free(row_cleared);
  taucs_free(rnums);
  taucs_free(row_workspace);
  taucs_free(rows_size);
  taucs_free(col_mmb);

  return status;
}

/*************************************************************************************
 * Function: df_postorder
 *
 * Description: Finds the depth-first traversal postorder of the tree given by
 *              the parent array. Also returns the number of descendents each 
 *              vertex has (vertices indexed by original number)
 *
 *************************************************************************************/
static int df_postorder(int *first_child, int *next_child, int root, int *postorder, int *desc_count)
{
  int *stack_vertex, *stack_child, child;
  int postnum, depth;
  int status = OK;

  stack_vertex = taucs_malloc((root + 1) * sizeof(int));
  stack_child = taucs_malloc((root + 1) * sizeof(int));
  if (stack_vertex == NULL || stack_child == NULL)
    {
      status = FAILURE;
      goto free_and_exit;
    }

  /* We do dfs in a loop, instead of recursively. This is why we use a stack */
  postnum = 0;
  depth = 0;
  stack_vertex[depth] = root; /* This is the "root" */
  stack_child[depth] = first_child[stack_vertex[depth]];
  while (depth >= 0) 
    {
      if (stack_child[depth] != MULTILU_SYMBOLIC_NONE) 
	{
	  stack_vertex[depth + 1] = stack_child[depth];
	  stack_child[depth + 1] = first_child[stack_vertex[depth+1]];
	  depth++;
	} 
      else 
	{
	  /* If not "root" then we put it in the postorder */
	  if (stack_vertex[depth] != root) 
	    { 
	      int vertex = stack_vertex[depth];

	      assert(stack_vertex[depth] < root);
	      postorder[postnum] = vertex;
	      desc_count[vertex] = 1;
	      for(child = first_child[vertex]; child != MULTILU_SYMBOLIC_NONE; child = next_child[child])
		desc_count[vertex] += desc_count[child];
	      postnum++;
	    }

	  /* OK, we finished this node but we need to replace it with it's brother  */
	  /* (if there is one). */
	  depth--;
	  if (depth >= 0) 
	    stack_child[depth] = next_child[stack_child[depth]];
	}
    }

  /* Free memory */
 free_and_exit:
  taucs_free(stack_vertex);
  taucs_free(stack_child);

  return status;
}

/*************************************************************************************
 * Function: detect_supercol
 *
 * Description: Given the matrix and the column order, and partial cetree data this
 *              function find supercolumns. This supercolumns are actually upperbound
 *              on fundemental supercolumns that will be generated when factoring.
 *
 *************************************************************************************/
static int detect_supercol(taucs_ccs_matrix *A, int *column_order, int *one_child, int *desc_count, 
			   int *l_size, int *u_size, int *postorder,
			   int *sc_num, int *sc_size, int *sc_parent)
{
  int *firstcol, *root, *map_col_supercol, *lastcol, *map_fsc_rsc;
  int i, fsc_num, col, flag, new_supercol, cscs;
  int sc_lsize, sc_usize, max_lsize, max_usize;
  void *sets;
  int status = OK;

  /* Preallocate memory */
  firstcol = taucs_malloc(A->m * sizeof(int));
  map_col_supercol = taucs_malloc(A->n * sizeof(int));
  lastcol = taucs_malloc(A->n * sizeof(int));
  root = taucs_malloc(A->n * sizeof(int));
  sets = uf_make_sets(A->n);
  if (firstcol == NULL || map_col_supercol == NULL ||
      lastcol == NULL || root == NULL || sets == NULL)
    {
      status = FAILURE;
      goto free_and_exit;
    }
  
  /* Initilizations */
  for(i = 0; i < A->m; i++)
    firstcol[i] = A->n;
	
  fsc_num = -1;

  memset(sc_size, 0, A->n * sizeof(int));
  for(i = 0; i < A->n; i++)
    sc_parent[i] = MULTILU_SYMBOLIC_NONE;

  max_lsize = 0; max_usize = 0;
  sc_lsize = 0; sc_usize = 0;

  /* Go over columns in order... */
  for(col = 0; col < A->n; col++) 
    {
      int *rowind_column;
      int nnz_column;
      int org_col, cset;

      org_col = column_order[col];

      cset = col;
      new_supercol = FALSE;
      flag = FALSE;

      /* If not one child then automatically new supercol */
      if (!one_child[col] || sc_size[fsc_num] == MULTILU_MAX_SUPERCOL_SIZE)
	new_supercol = TRUE;

      /* This is the actual values for this column */
      nnz_column = A->colptr[org_col + 1] - A->colptr[org_col];
      rowind_column = A->rowind + A->colptr[org_col];

      /* Initilization for this column */
      root[cset] = col;
    		
      /* Go over non-zeros of this column */
      for(i = 0; i < nnz_column; i++) 
	{     
	  int row, fcol;

	  row = rowind_column[i];
	  fcol = firstcol[row];
      
	  /* If this is the first appearance of this row then write column */
	  /* Otherwise unite this row (if not already done so) */
	  if (fcol == A->n)
	      firstcol[row] = col;
	  else 
	    {
	      int rset, rroot;

	      rset = uf_find(sets, fcol);
	      rroot = root[rset]; 
	      if (rroot != col) 
		{
		  sc_parent[map_col_supercol[rroot]] = col;
		  cset = uf_union(sets, cset, rset);
		  root[cset] = col;
		}
	    }
	}

      /* If we are in a chain check to see if we break supercolumn */
      if (!new_supercol)
	{
	  int inc_sc_size = sc_size[fsc_num] + 1;

	  max_lsize += l_size[postorder[col]];
	  max_usize += u_size[postorder[col]];
	  sc_lsize = max(sc_lsize, l_size[postorder[col]] + sc_size[fsc_num]);
	  sc_usize = max(sc_usize, u_size[postorder[col]] + sc_size[fsc_num]);

	  /* Now we are ready to check condition */
	  if (sc_lsize * inc_sc_size > MULTILU_MAX_OVERFILL_RATIO * max_lsize || 
	      sc_usize * inc_sc_size > MULTILU_MAX_OVERFILL_RATIO * max_usize)
	    new_supercol = TRUE;
	}

      /* Take care of supercolumns */
      if (new_supercol)
	{
	  fsc_num++;
	  sc_size[fsc_num] = 1;
	  lastcol[fsc_num] = col;
	  map_col_supercol[col] = fsc_num;
	  max_lsize = l_size[postorder[col]]; max_usize = u_size[postorder[col]];
	  sc_lsize = l_size[postorder[col]]; sc_usize = u_size[postorder[col]];
	}
      else
	{
	  sc_size[fsc_num]++;
	  lastcol[fsc_num] = col;
	  map_col_supercol[col] = fsc_num;
	}
    }

  /* Close last supercolumn */
  fsc_num++;

  /* Correct mapping of sc_parent from columns to supercolumns */
  for(i = 0; i < fsc_num; i++)
    {
      if (sc_parent[i] != MULTILU_SYMBOLIC_NONE)
	sc_parent[i] = map_col_supercol[sc_parent[i]];

      /* Second way that it is a root - parent is itself */
      if (sc_parent[i] == i)
	sc_parent[i] = MULTILU_SYMBOLIC_NONE;
    }

  /* Relax supernodes (if need to by parameter) */
#if MULTILU_RELAX_RULE_SIZE > 1
  map_fsc_rsc = map_col_supercol;
  *sc_num = 0;
  cscs = 0;
  for(i = 0; i < fsc_num; i++)
    {
      cscs += sc_size[i];
      map_fsc_rsc[i] = *sc_num;
      lastcol[*sc_num] = i;
      if (sc_parent[i] != MULTILU_SYMBOLIC_NONE && desc_count[lastcol[sc_parent[i]]] >= MULTILU_RELAX_RULE_SIZE)
	{
	  sc_size[*sc_num] = cscs;
	  cscs = 0;
	  (*sc_num)++;
	}
    }
  sc_size[*sc_num] = cscs;
  (*sc_num)++;
  
  /* Correct parent again */
  for(i = 0; i < *sc_num; i++)
    {
      int org_parent;
      
      org_parent = sc_parent[lastcol[i]];
      if (org_parent != MULTILU_SYMBOLIC_NONE)
	sc_parent[i] = map_fsc_rsc[org_parent];
      else
	sc_parent[i] = MULTILU_SYMBOLIC_NONE;
    }
#else
  *sc_num = fsc_num;
#endif

  /* Free memory */
 free_and_exit:
  taucs_free(root);
  taucs_free(firstcol);
  taucs_free(sets);
  taucs_free(map_col_supercol);
  taucs_free(lastcol);

  return status;
}


/*************************************************************************************
 * Function: complete_symbolic
 *
 * Description: Fill-in the rest of the symbolic data that colamd didn't calculate
 *              (the etree)
 *
 *************************************************************************************/
void complete_symbolic(taucs_multilu_symbolic *symbolic)
{
  taucs_multilu_etree *etree = &symbolic->etree;
  int i;
  
  int s = symbolic->number_supercolumns;
  
  /* Fill in supercolumn start and end */
  symbolic->start_supercolumn[0] = 0;
  symbolic->end_supercolumn[0] = symbolic->supercolumn_size[0] - 1;
  for(i = 1; i < s; i++)
    {
      symbolic->start_supercolumn[i] = symbolic->end_supercolumn[i - 1] + 1;
      symbolic->end_supercolumn[i] = symbolic->start_supercolumn[i] + symbolic->supercolumn_size[i] - 1;
    }
  
  assert(symbolic->end_supercolumn[s - 1] == symbolic->n - 1);
  
  /* Complete the etree */
  
  /* Initlize all elements before filling them */
  etree->first_root = MULTILU_SYMBOLIC_NONE;
  for(i = 0; i < s; i++)
    {
      etree->first_child[i] = MULTILU_SYMBOLIC_NONE;
      etree->next_child[i] = MULTILU_SYMBOLIC_NONE;
      etree->first_desc_index[i] = MULTILU_SYMBOLIC_NONE;
      etree->last_desc_index[i] = MULTILU_SYMBOLIC_NONE;
    }
  
  /* Build child list and find root */
  for(i = 0; i < s; i++)
    {
      int child = i;
      int parent = etree->parent[child];
      
      /* Check if a new root. Otherwise fill it as child */
      if (parent == MULTILU_SYMBOLIC_NONE)
	{
	  etree->next_child[child] = etree->first_root; 
	  etree->first_root = child;
	}
      else
	{
	  etree->next_child[child] = etree->first_child[parent];
	  etree->first_child[parent] = child;
	}
    }
  
  /* The postorder is simple: 1, 2, ..., root */
  
  /* Make the desc sets using indexes in the order. */
  /* This is a bit complex but it works */
  /* The idea is to use the postorder and update only parent. Because of the postorder all children */
  /* will be ready and have updated once we get to the parent so it is ok to update it's parent */
 for(i = 0; i < s; i++)
   {
     int parent = etree->parent[i];
      
     /* If the column has descendents then end one column before */
     if (etree->first_desc_index[i] != MULTILU_SYMBOLIC_NONE)
       etree->last_desc_index[i] = i - 1;
      
     /* We update the first desc of the parent. Diffrent if we have desc. or not... */
     if (parent != MULTILU_SYMBOLIC_NONE)
       {
	 if (etree->first_desc_index[parent] == MULTILU_SYMBOLIC_NONE && etree->first_desc_index[i] == MULTILU_SYMBOLIC_NONE)
	   etree->first_desc_index[parent] = i;
	 if (etree->first_desc_index[parent] == MULTILU_SYMBOLIC_NONE && etree->first_desc_index[i] != MULTILU_SYMBOLIC_NONE)
	   etree->first_desc_index[parent] = etree->first_desc_index[i];
       }
   }
  
 /* Calculate the number of covered columns at each supercolumn */
 memset(symbolic->supercolumn_covered_columns, 0, s * sizeof(int));
 for(i = 0; i < s; i++)
   {
     int parent = etree->parent[i];
      
     symbolic->supercolumn_covered_columns[i] += symbolic->supercolumn_size[i];
     if (parent != MULTILU_SYMBOLIC_NONE)
       symbolic->supercolumn_covered_columns[parent] += symbolic->supercolumn_covered_columns[i];
   }
}

/*************************************************************************************
 * Function: allocate_symbolic
 *
 * Description: allocate_memory for use with the symbolic data
 *
 *************************************************************************************/
static taucs_multilu_symbolic *allocate_symbolic(int n)
{
  taucs_multilu_symbolic *symbolic = taucs_malloc(sizeof(taucs_multilu_symbolic));
  if (symbolic == NULL)
    return NULL;

  symbolic->n = n;
  symbolic->etree.parent = taucs_malloc((n + 1) * sizeof(int));
  symbolic->etree.first_child = taucs_malloc((n + 1) * sizeof(int));
  symbolic->etree.next_child = taucs_malloc((n + 1) * sizeof(int));
  symbolic->etree.first_desc_index = taucs_malloc((n + 1) * sizeof(int));
  symbolic->etree.last_desc_index = taucs_malloc((n + 1) * sizeof(int));
  symbolic->columns = taucs_malloc((n + 1) * sizeof(int));
  symbolic->start_supercolumn = taucs_malloc((n + 1) * sizeof(int));
  symbolic->end_supercolumn = taucs_malloc((n + 1) * sizeof(int));
  symbolic->supercolumn_size = taucs_malloc((n + 1) * sizeof(int));
  symbolic->supercolumn_covered_columns = taucs_malloc((n + 1) * sizeof(int));
  symbolic->l_size = taucs_malloc((n + 1) * sizeof(int));
  symbolic->u_size = taucs_malloc((n + 1) * sizeof(int));

  if (symbolic->etree.parent == NULL ||
      symbolic->etree.first_child == NULL ||
      symbolic->etree.next_child == NULL ||
      symbolic->etree.first_desc_index == NULL ||
      symbolic->etree.last_desc_index == NULL ||
      symbolic->columns == NULL ||
      symbolic->start_supercolumn == NULL ||
      symbolic->end_supercolumn == NULL ||
      symbolic->supercolumn_size == NULL ||
      symbolic->supercolumn_covered_columns == NULL ||
      symbolic->l_size == NULL ||
      symbolic->u_size == NULL)
    {
      taucs_multilu_symbolic_free(symbolic);
      return NULL;
    }

  return symbolic;
}

/*************************************************************************************
 * Function: garbage_collect
 *
 * Description: When doing elimination analysis we keep superrows, i.e. supersets of
 *              the rows created during the real factorization. All this rows are kept
 *              in a big pool. For each created superrow atleast one row dies. 
 *              One has to manage this pool of memory. The following function does - it 
 *              garbage collect and defragment the memory pool.
 *   
 *
 *************************************************************************************/
typedef struct
{
  int el_start;
  int el_number;
} el_location;

static int cmp_el_location(const void *_a, const void *_b)
{
  el_location *a = (el_location *)_a;
  el_location *b = (el_location *)_b;
  return(a->el_start - b->el_start);
}

static int garbage_collect(int *workspace, int *el_start, int *el_size, int *el_cleared, int el_num)
{
  int i, act_el_num, loc;
  el_location *el_loc;
  
  /* Set order of elements */
  el_loc = taucs_malloc(el_num * sizeof(el_location));
  act_el_num = 0;
  for(i = 0; i < el_num; i++)
    if (!el_cleared[i])
      {
	el_loc[act_el_num].el_start = el_start[i];
	el_loc[act_el_num].el_number = i;
	act_el_num++;
      }
  qsort(el_loc, act_el_num, sizeof(el_location), cmp_el_location);

  /* Now do the defragment */
  loc = 0;
  for(i = 0; i < act_el_num; i++)
    {
      memcpy(workspace + loc, workspace + el_loc[i].el_start, el_size[el_loc[i].el_number] * sizeof(int));
      el_loc[i].el_start = loc;
      loc += el_size[el_loc[i].el_number];
    }

  /* Now correct locations */
  for(i = 0; i < act_el_num; i++)
    el_start[el_loc[i].el_number] = el_loc[i].el_start;

  return loc;
}

#endif /* TAUCS_CORE_GENERAL for the entire symbolic phase */

/*************************************************************************************
 *************************************************************************************
 * NUMERIC FACTORIZATION
 *************************************************************************************
 *************************************************************************************/

#ifndef TAUCS_CORE_GENERAL

/*************************************************************************************
 * Factor Context
 *
 * When running the algorithm we need several data structures. This are needed
 * by all subfunctions. Instead of passing them one by one we pass a context structure.
 * When running a function the context is basically the instance parameters of the 
 * function (every function is called as a part of the algorithm so the run is part
 * of an instance of the appliance of the algorithm).
 *
 * We could have defined globals, but that is not thread safe...
 *************************************************************************************/
typedef struct multilu_context_st 
{
  /* This is the result factor that is built throughout the process. */
  taucs_multilu_factor* F;

  /* 
   * The matrix and the symbolic data 
   * Why is the matrix kept in At form too? Inorder to more efficently focus on rows 
   */
  taucs_ccs_matrix* A;
  taucs_ccs_matrix* At;
  int* row_cleared; 
  int* column_cleared;
  taucs_multilu_symbolic *symbolic;
  taucs_double thresh;

  /*
   * Workspace for mapping rows to location. 
   * This maps to inside the non-pivotal par, i.e. the sturcture of the contribution block 
   * Desc alway use diffrent rows because they can be chosen as pivots... 
   * So we can use this map in parallel 
   */
  int *map_rows;

  /*
   * Workspace for mapping columns to location 
   * Global to avoid reallocation or passing as parameter 
   * In parallel mode we need a batch of these (one for every processor) 
   * so we have additional data for maintaining that.
   * One can ask: why bother in the parallel mode? Just reallocate each time!
   * The answer is simple: the values must be set to -1 so it will be a waste to
   * allocate and set for each supercolumn...
   * TODO: Check if that is true? Maybe there are few enough supercolumns...
   */
  int *map_cols;

#ifdef TAUCS_CILK
  /* Next is in the first item of every workspace */
  int          start_free_map_cols;
  Cilk_lockvar lock_select_map_cols;
#else
  /*
   * Workspaces that cannot be used in parallel. 
   * Since they cannot be used in parallel we do not define them in cilked mode. 
   * Instead we allocate them when needed. This is not a big overhead
   * because we don't have to memset them. 
   */

  /*
   * This is a scartch workspace for the LU factorization routine 
   * Global to avoid reallocation or passing as parameter 
   */
  int *LU_rows_scratch;

  /*
   * This is a scartch workspace for putting row degrees for the LU factorization routine 
   * Global to avoid reallocation or passing as parameter 
   */
  int *LU_degrees_scratch;
#endif
} multilu_context;

static void multilu_context_free(multilu_context* context)
{
  taucs_ccs_free(context->At);
  taucs_free(context->row_cleared);
  taucs_free(context->column_cleared);
  taucs_free(context->map_rows);
  taucs_free(context->map_cols);
    
#ifndef TAUCS_CILK

  taucs_free(context->LU_rows_scratch);
  taucs_free(context->LU_degrees_scratch);
    
#endif
    
  taucs_free(context);
}

cilk static multilu_context* multilu_context_create(taucs_ccs_matrix *A, 
					       taucs_multilu_symbolic *symbolic, 
					       taucs_double thresh)
{
#ifdef TAUCS_CILK
  int i;
#endif

  multilu_context* context;

  context = (multilu_context*)taucs_malloc(sizeof(multilu_context));
  if (context == NULL) 
    return NULL;

  context->symbolic = symbolic;
  context->thresh = thresh;
  context->A = A;
  context->At = taucs_ccs_transpose(A);
  context->row_cleared = (int*)taucs_calloc(A->m, sizeof(int));
  context->column_cleared = (int*)taucs_calloc(A->n, sizeof(int));
  context->map_rows = (int*)taucs_malloc(A->m * sizeof(int));
  memset(context->map_rows, -1, A->m * sizeof(int));

#ifndef TAUCS_CILK

  context->map_cols = (int*)taucs_malloc(A->n * sizeof(int));
  memset(context->map_cols, -1, A->n * sizeof(int));
  context->LU_rows_scratch = (int*)taucs_malloc(A->m * sizeof(int));
  context->LU_degrees_scratch = (int*)taucs_malloc(A->m * sizeof(int));

#else
  /* 
   * In cilk mode: allocate a few buffers. Make a free list were
   * the [0] element points to the start of the next one.
   * Allocate buffers of n+1 size so that the [n] element will point
   * to the start of the current buffer
   */
  context->map_cols = taucs_malloc((A->n + 1) * sizeof(int) * Cilk_active_size);
  memset(context->map_cols, -1, (A->n + 1) * sizeof(int) * Cilk_active_size);
  context->start_free_map_cols = 0;
  for(i = 0; i < Cilk_active_size; i++)
    {
      if (i + 1 < Cilk_active_size)
	context->map_cols[i * (A->n + 1)] = (i + 1) * (A->n + 1);
      context->map_cols[i * (A->n + 1) + A->n] = i * (A->n + 1);
    }
  Cilk_lock_init(context->lock_select_map_cols);

#endif

  /* Check allocation */
  if (context->At == NULL || context->row_cleared == NULL || context->map_rows == NULL ||
      context->map_cols == NULL 
#ifndef TAUCS_CILK
      || context->LU_rows_scratch == NULL || context->LU_degrees_scratch == NULL
#endif 
      )
  {
    multilu_context_free(context);
    context = NULL;
  }

  return context;
}

/*************************************************************************************
 * Function prototypes 
 *************************************************************************************/
/* TODO: use spawn on factor-blocks only in cilked blas mode */
static void allocate_factor(multilu_context* context, int m, int n, int supercolumns_number, int type);
cilk static void factorize_supercolumn(multilu_context* mcontext, int pivot_supercol, int *map_cols);
#ifdef TAUCS_CILK
cilk static int recursive_factorize_supercolumn(multilu_context* mcontext,
						int pivot_supercol, int depth, int max_depth);
#endif
static void focus_supercolumn_from_child(multilu_context* context, int supercol, int child);
static void focus_supercolumn_from_contrib(multilu_context* context, int supercol, int contrib);
static void focus_supercolumn_from_A(multilu_context* context, int supercol);
static int focus_rows(multilu_context* context,
		      int *rows, int number, int pivot_supercol, 
		      int *ind, taucs_datatype *values, int max_size, int *map_cols);
#ifdef TAUCS_CILK
cilk static int rearrange_non_pivot_cols(multilu_context* mcontext, int pivot_supercol, int row_b_size, int ru_size, int *map_cols, int *scratch);
cilk static void align_add_subtree(multilu_context* mcontext, multilu_contrib_block *addto, int subtree_root, int *map_cols);
#endif
cilk static void align_add_from(multilu_context* context, multilu_contrib_block *addto, multilu_factor_block *desc_factor_block, int *map_cols);
cilk static void align_add(multilu_context* context, multilu_contrib_block *addto, multilu_contrib_block *addfrom, int *map_cols,
			   int m0, int m1, int n0, int n1);
cilk static void align_add_rows(multilu_context* context, multilu_contrib_block *addto, multilu_contrib_block *addfrom, int *map_col,
				int m0, int m1, int n0, int n1);
cilk static void align_add_cols(multilu_context* context, multilu_contrib_block *addto, multilu_contrib_block *addfrom, int *map_cols,
				int m0, int m1, int n0, int n1);
static int is_member(int x, int *S, int n);
static void allocate_factor_block(multilu_context* mcontext, int supercol);
static multilu_contrib_block *allocate_contrib_block(int l_size, int u_size);
static void free_contrib_block(multilu_contrib_block *block);
static void prepare_degree_array(multilu_context* context, int supercol, int *rows, int size, int *degrees);
static void compress_values_block(taucs_datatype **values, int m, int n, int ld);
cilk static int *get_map_cols(multilu_context *mcontext);
cilk static void release_map_cols(multilu_context *mcontext, int *map_cols); 

#endif /* not TAUCS_CORE_GENERAL for the data structures and prototypes part */

/*************************************************************************************
 * Sub-system CORE_GENERAL API functions 
 *************************************************************************************/
#ifdef TAUCS_CORE_GENERAL

/*************************************************************************************
 * Function: taucs_ccs_factor_lu
 *
 * Description: Factorize matrix A using thresh as threshold for selecting pivot rows.
 *
 *************************************************************************************/
cilk taucs_multilu_factor* taucs_ccs_factor_lu(taucs_ccs_matrix* A, int *column_order, 
					       taucs_double thresh)
{
  taucs_multilu_factor *r = NULL;

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (A->flags & TAUCS_DOUBLE)
    r = spawn taucs_dccs_factor_lu(A, column_order, thresh);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (A->flags & TAUCS_SINGLE)
    r = taucs_sccs_factor_lu(A, column_order, thresh);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_DCOMPLEX)
    r = taucs_zccs_factor_lu(A, column_order, thresh);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_SCOMPLEX)
    r = taucs_cccs_factor_lu(A, column_order, thresh);
#endif

  sync;
  return r;
}

/*************************************************************************************
 * Function: taucs_ccs_factor_lu_maxdepth
 *
 * Description: Factorize matrix A using thresh as threshold for selecting pivot rows.
 *              This function recives a preorder on the columns (zeros based). 
 *              Here a maxdepth is given. It is ignored if we do not use a recursive
 *              algorithm (like, when we don't compile with cilk). If the maxdepth
 *              is achieved then we cut a sequential algorithm.
 *
 *************************************************************************************/
cilk taucs_multilu_factor* taucs_ccs_factor_lu_maxdepth(taucs_ccs_matrix* A, int *column_order, 
							taucs_double thresh, int max_depth)
{
  taucs_multilu_factor *r = NULL;

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (A->flags & TAUCS_DOUBLE)
    r = spawn taucs_dccs_factor_lu_maxdepth(A, column_order, thresh, max_depth);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (A->flags & TAUCS_SINGLE)
    r = spawn taucs_sccs_factor_lu_maxdepth(A, column_order, thresh, max_depth);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_DCOMPLEX)
    r = spawn taucs_zccs_factor_lu_maxdepth(A, column_order, thresh, max_depth);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_SCOMPLEX)
    r = spawn taucs_cccs_factor_lu_maxdepth(A, column_order, thresh, max_depth);
#endif
  
  sync;
  return r;
}

/*************************************************************************************
 * Function: taucs_ccs_factor_lu_numeric
 *
 * Description: Factorizes the matrix using the supercolumn symbolic data given. 
 *
 *************************************************************************************/
cilk taucs_multilu_factor* taucs_ccs_factor_lu_numeric(taucs_ccs_matrix *A, 
						       taucs_multilu_symbolic *symbolic, 
						       taucs_double thresh)
{
  taucs_multilu_factor *r = NULL;

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (A->flags & TAUCS_DOUBLE)
    r = spawn taucs_dccs_factor_lu_numeric(A, symbolic, thresh);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (A->flags & TAUCS_SINGLE)
    r = spawn taucs_sccs_factor_lu_numeric(A, symbolic, thresh);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_DCOMPLEX)
    r = spawn taucs_zccs_factor_lu_numeric(A, symbolic, thresh);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_SCOMPLEX)
    r = spawn taucs_cccs_factor_lu_numeric(A, symbolic, thresh);
#endif
  
  sync;
  return r;
}

/*************************************************************************************
 * Function: taucs_ccs_factor_lu_numeric_maxdepth
 *
 * Description: Factorizes the matrix using the supercolumn symbolic data given. 
 *              Here a maxdepth is given. It is ignored if we do not use a recursive
 *              algorithm (like, when we don't compile with cilk). If the maxdepth
 *              is achieved then we cut a sequential algorithm.
 *
 *************************************************************************************/
cilk taucs_multilu_factor* taucs_ccs_factor_lu_numeric_maxdepth(taucs_ccs_matrix *A, 
								taucs_multilu_symbolic *symbolic, 
								taucs_double thresh, int max_depth)
{
  taucs_multilu_factor *r = NULL;

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (A->flags & TAUCS_DOUBLE)
    r = spawn taucs_dccs_factor_lu_numeric_maxdepth(A, symbolic, thresh, max_depth);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (A->flags & TAUCS_SINGLE)
    r = spawn taucs_sccs_factor_lu_numeric_maxdepth(A, symbolic, thresh, max_depth);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_DCOMPLEX)
    r = spawn taucs_zccs_factor_lu_numeric_maxdepth(A, symbolic, thresh, max_depth);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (A->flags & TAUCS_SCOMPLEX)
    r = spawn taucs_cccs_factor_lu_numeric_maxdepth(A, symbolic, thresh, max_depth);
#endif
  
  sync;
  return r;
}

#endif /* TAUCS_CORE_GENERAL for the CORE_GENERAL API functions */

/*************************************************************************************
 * Sub-system DATATYPE API functions 
 *************************************************************************************/
#ifndef TAUCS_CORE_GENERAL

/*************************************************************************************
 * Function: taucs_dtl(ccs_factor_lu)
 *
 * Description: Datatype version of taucs_ccs_factor_lu
 *
 *************************************************************************************/
cilk taucs_multilu_factor* taucs_dtl(ccs_factor_lu)(taucs_ccs_matrix* A, int *column_order, 
					       taucs_double thresh)
{
  taucs_multilu_factor *r;
  r = spawn taucs_dtl(ccs_factor_lu_maxdepth)(A, column_order, thresh, 0);
  sync;

  return r;
}

/*************************************************************************************
 * Function: taucs_dtl(ccs_factor_lu_maxdepth)
 *
 * Description: Datatype version of taucs_ccs_factor_lu_maxdepth
 *
 *************************************************************************************/
cilk taucs_multilu_factor* taucs_dtl(ccs_factor_lu_maxdepth)(taucs_ccs_matrix* A, int *column_order, 
							     taucs_double thresh, int max_depth)
{
  taucs_multilu_symbolic *symbolic;
  taucs_multilu_factor *F;
  
  /* TODO: Work for non equal number of rows and columns */
  assert(A->m == A->n);
  
  /* Make symbolic anaylsis and fill in etree */
  symbolic = taucs_ccs_factor_lu_symbolic(A, column_order);
  F = spawn taucs_ccs_factor_lu_numeric_maxdepth(A, symbolic, thresh, max_depth);
  sync;
  
  /* Free memory */
  taucs_multilu_symbolic_free(symbolic);
  
  return F;
}

/*************************************************************************************
 * Function: taucs_dtl(ccs_factor_lu_numeric)
 *
 * Description: Datatype version of taucs_ccs_factor_lu_numeric
 *
 *************************************************************************************/
cilk taucs_multilu_factor* taucs_dtl(ccs_factor_lu_numeric)(taucs_ccs_matrix *A, 
							    taucs_multilu_symbolic *symbolic, 
							    taucs_double thresh)
{
  taucs_multilu_factor *r;
  r = spawn taucs_dtl(ccs_factor_lu_numeric_maxdepth)(A, symbolic, thresh, 0);
  sync;

  return r;
}

/*************************************************************************************
 * Function: taucs_dtl(ccs_factor_lu_numeric_maxdepth)
 *
 * Description: Datatype version of taucs_ccs_factor_lu_numeric
 *
 *************************************************************************************/
cilk taucs_multilu_factor* taucs_dtl(ccs_factor_lu_numeric_maxdepth)(taucs_ccs_matrix *A, 
								     taucs_multilu_symbolic *symbolic, 
								     taucs_double thresh, int max_depth)
{
  int i;
  multilu_context* context;
  taucs_multilu_factor *F;
  
  /* Basic sanity check for the symbolic structure because called internal to multilu */
  if (symbolic == NULL)
    return NULL;

  /* Data preparation  */
  context = spawn multilu_context_create(A, symbolic, thresh);
  sync;
  if (context == NULL)
    return NULL;
  allocate_factor(context, context->A->m, context->A->n, context->symbolic->number_supercolumns, 
		  A->flags & (TAUCS_DOUBLE | TAUCS_SINGLE | TAUCS_SCOMPLEX | TAUCS_DCOMPLEX));
  if (context->F->blocks == NULL)
    {
      multilu_context_free(context);
      return NULL;
    }

  /* 
   * If we are compiling cilk code and there is more the one processor then
   * use are recursive algorithm 
   */
  #ifdef TAUCS_CILK
  if (Cilk_active_size > 1)
    {
      int dummy;

      /* Factorize each root. Can do in parallel */
      assert(context->symbolic->etree.first_root != MULTILU_SYMBOLIC_NONE); 
      for(i = context->symbolic->etree.first_root; i != MULTILU_SYMBOLIC_NONE; i = context->symbolic->etree.next_child[i])
	dummy = spawn recursive_factorize_supercolumn(context, i, 0, max_depth);
      sync;  
    }
  else
    {
  #endif

      /* Otherewise use a sequential algorithm - factorize each node by order */
      assert(context->symbolic->etree.first_root != MULTILU_SYMBOLIC_NONE); 
      for(i = 0; i < context->symbolic->number_supercolumns; i++)
	{
	  int *map_cols, child;

	  /* Maybe we know that this supercolumn is empty beforehand and can skip it? */
	  /* TODO: Is this good? */
	  if (context->symbolic->l_size[i] == 0)
	    continue;

	  /* Prepare the factor block and assemble the supercolumn to it */ 
	  allocate_factor_block(context, i);
	  for(child = context->symbolic->etree.first_child[i]; child != MULTILU_SYMBOLIC_NONE; child = context->symbolic->etree.next_child[child])
	    focus_supercolumn_from_child(context, i, child);  
	  focus_supercolumn_from_A(context, i);
	  
	  /* Now we can factorize the supercolumn */
	  map_cols = spawn get_map_cols(context);
	  sync;
	  
	  spawn factorize_supercolumn(context, i, map_cols);
	  sync;

	  spawn release_map_cols(context, map_cols);
	  sync;
	}	

  #ifdef TAUCS_CILK
    }
  #endif

  /* Keep factor and free rest of context */  
  F = context->F;
  multilu_context_free(context);

  /* Make sure that all the factor blocks are valid (if not we failed) */
  for(i = 0; i < F->num_blocks; i++)
    if (F->blocks[i] == NULL || !F->blocks[i]->valid)
      {
	taucs_multilu_factor_free(F);
	return NULL;
      }

  return F;
}

#endif /* not TAUCS_CORE_GENERAL for the DATATYPE API functions */

/*************************************************************************************
 * Sub-system Internal functions 
 *************************************************************************************/
#ifndef TAUCS_CORE_GENERAL

/*************************************************************************************
 * Inline functions first
 *************************************************************************************/

/*************************************************************************************
 * Function: is_contrib_block_empty
 *
 * Description: Tests of the contrib block is empty
 *
 *************************************************************************************/
#define is_contrib_block_empty(block) (block == NULL)

/*************************************************************************************
 * Function: is_member
 *
 * Description: Checks if x is a member of the array pointed by S with n elements.
 *              Returns the location if found or -1 if not found.
 *
 *************************************************************************************/
int is_member(int x, int *S, int n)
{
  int i;
  for(i = 0; i < n; i++)
    if (S[i] == x)
      return i;
  
  return -1;
}

/*************************************************************************************
 * Regular functions
 *************************************************************************************/

/*************************************************************************************
 * Function: recursive_factorize_supercolumn
 *
 * Description: Recursively factorizes the given supercolumn. 
 *              To factorize it we need first to factorize the children. 
 *              After that we can factorize this column, knowing that the children are
 *              ready to update.  
 *
 *************************************************************************************/
#ifdef TAUCS_CILK
cilk static int recursive_factorize_supercolumn(multilu_context *mcontext, 
						int pivot_supercol, int depth, int max_depth)
{
  taucs_multilu_etree *etree = &mcontext->symbolic->etree;
  const int no_child_spawn = mcontext->symbolic->supercolumn_covered_columns[pivot_supercol] < MULTILU_MIN_COVER_SPRS_SPAWN;  
  int child, desc, have_job_here, one_child;
  int *map_cols;

  /* Inlet for pipelinning the focus operation withother childs */
  inlet void focus_from_spawned(int child)
  { 
    if (!have_job_here || one_child) 
      return;

    /* If this is the first to return then allocate (cannot do beforhand beacuse of memory explosion */
    if (mcontext->F->blocks[pivot_supercol] == NULL)
      allocate_factor_block(mcontext, pivot_supercol);
	
    focus_supercolumn_from_child(mcontext, pivot_supercol, child);
  }
    
  /* 
   * If the first child will not cause us to reach the maximum depth then 
   * we can call recursively. Otherwise we have to cut to a sequential code
   */
  if ((max_depth == 0 || depth + 1 < max_depth) && !no_child_spawn)
    {
      have_job_here = (mcontext->symbolic->l_size[pivot_supercol] != 0);
      one_child = 
	etree->first_child[pivot_supercol] != MULTILU_SYMBOLIC_NONE && 
	etree->next_child[etree->first_child[pivot_supercol]] == MULTILU_SYMBOLIC_NONE;

      /* We need to first factorize the children */
      for(child = etree->first_child[pivot_supercol]; child != MULTILU_SYMBOLIC_NONE; child = etree->next_child[child])
	focus_from_spawned(spawn recursive_factorize_supercolumn(mcontext, child, depth + 1, max_depth));
      sync;

      /* Maybe we know that this supercolumn is empty beforehand and can skip it? */
      /* TODO: Is this good? */
      if (!have_job_here)
	return pivot_supercol;

      /* Prepare the factor block and assemble the supercolumn to it */ 
      if (!one_child)
	{
	  if (mcontext->F->blocks[pivot_supercol] == NULL)
	    allocate_factor_block(mcontext, pivot_supercol);
	  focus_supercolumn_from_A(mcontext, pivot_supercol);
	}

      /* Now we can factorize this column */
      map_cols = spawn get_map_cols(mcontext);
      sync;
      
      spawn factorize_supercolumn(mcontext, pivot_supercol, map_cols);
      sync;

      spawn release_map_cols(mcontext, map_cols);
      sync;
    }
  else
    {
      map_cols = spawn get_map_cols(mcontext);
      sync;

      if (etree->first_desc_index[pivot_supercol] != MULTILU_SYMBOLIC_NONE)
	for(desc = etree->first_desc_index[pivot_supercol]; desc <= etree->last_desc_index[pivot_supercol]; desc++) 
	  {
	    if (mcontext->symbolic->l_size[desc] == 0)
	      continue;

	    one_child =
	      etree->first_child[desc] != MULTILU_SYMBOLIC_NONE && 
	      etree->next_child[etree->first_child[desc]] == MULTILU_SYMBOLIC_NONE;

	    
	    /* Focus for the supercolumn */
	    if (!one_child)
	      {
		allocate_factor_block(mcontext, desc);
		for(child = mcontext->symbolic->etree.first_child[desc]; child != MULTILU_SYMBOLIC_NONE; child = mcontext->symbolic->etree.next_child[child])
		  focus_supercolumn_from_child(mcontext, desc, child);  
		focus_supercolumn_from_A(mcontext, desc);
	      }
		
	    spawn factorize_supercolumn(mcontext, desc, map_cols);
	    sync;
	  }

      if (mcontext->symbolic->l_size[pivot_supercol] == 0)
	{
	  spawn release_map_cols(mcontext, map_cols);
	  sync;
	  
	  return pivot_supercol;
	}

      one_child = 
	etree->first_child[pivot_supercol] != MULTILU_SYMBOLIC_NONE && 
	etree->next_child[etree->first_child[pivot_supercol]] == MULTILU_SYMBOLIC_NONE;

      if (!one_child)
	{
	  /* Prepare the factor block and assemble the supercolumn to it */ 
	  allocate_factor_block(mcontext, pivot_supercol);
	  for(child = mcontext->symbolic->etree.first_child[pivot_supercol]; child != MULTILU_SYMBOLIC_NONE; child = mcontext->symbolic->etree.next_child[child])
	    focus_supercolumn_from_child(mcontext, pivot_supercol, child);  
	  focus_supercolumn_from_A(mcontext, pivot_supercol);
	}

      spawn factorize_supercolumn(mcontext, pivot_supercol, map_cols);
      sync;

      spawn release_map_cols(mcontext, map_cols);
      sync;
    }

  return pivot_supercol;
}
#endif 

/*************************************************************************************
 * Function: factorize_l_portion
 *
 *
 *************************************************************************************/
cilk static void factorize_l_portion(multilu_context* mcontext, int pivot_supercol, int *map_cols, int dense_spawn, int one_child)
{
  int *LU_rows_scratch = NULL, *LU_degrees_scratch = NULL; 
  multilu_factor_block *factor_block;
  int ml_size, l_size, col_b_size, row_b_size;

  #ifdef TAUCS_CILK
  taucs_multilu_etree *etree = &mcontext->symbolic->etree;
  #endif

  /* If one_child need to complete the focused part and do the focus */
  #ifdef TAUCS_CILK
  if (one_child && Cilk_active_size > 1)
    {
      int child;
      multilu_factor_block *child_factor_block;
      multilu_contrib_block *child_contrib_block;

      child = etree->first_child[pivot_supercol];
      child_factor_block = mcontext->F->blocks[child];
      child_contrib_block = child_factor_block->contrib_block;

      if (child_contrib_block != NULL && child_contrib_block->num_cols_in_parent > 0)
	{
	  TAUCS_DENSE_SPAWN taucs_dtl(C_CaddMABT)(child_contrib_block->m, child_contrib_block->num_cols_in_parent, child_factor_block->col_pivots_number, 
						  child_factor_block->L2, child_factor_block->row_pivots_number + child_factor_block->non_pivot_rows_number,
						  child_factor_block->Ut2, child_factor_block->non_pivot_cols_number,
						  child_contrib_block->values, child_contrib_block->ld);
	  TAUCS_DENSE_SYNC;
	}


      allocate_factor_block(mcontext, pivot_supercol);
      focus_supercolumn_from_child(mcontext, pivot_supercol, child);
      focus_supercolumn_from_A(mcontext, pivot_supercol);
    }
  #endif

  factor_block = mcontext->F->blocks[pivot_supercol];
  if (!factor_block->valid)
    return;

  /* Take the sizes */
  ml_size = mcontext->symbolic->l_size[pivot_supercol];
  l_size = factor_block->l_size;
  assert(l_size > 0);
  col_b_size = mcontext->symbolic->supercolumn_size[pivot_supercol];
  row_b_size = min(l_size, col_b_size);

  if (l_size > 0)
    {
      /* Compress the memory beacuse we have redundent space. Don't forget to set the non-pivotal part */
      factor_block->pivot_rows = (int*)taucs_realloc(factor_block->pivot_rows, l_size * sizeof(int));
      factor_block->non_pivot_rows = factor_block->pivot_rows + row_b_size;
      compress_values_block(&factor_block->LU1, l_size, col_b_size, ml_size);
      factor_block->L2 = factor_block->LU1 + row_b_size;
      
      /* The following blocks are for dense LU */
      
      /* Define scratchs, preallocated or not */
      #ifdef TAUCS_CILK
      if (mcontext->thresh < 1.0)
	LU_degrees_scratch = taucs_malloc(l_size * sizeof(int));
      LU_rows_scratch = taucs_malloc(l_size * sizeof(int));
      #else
      LU_degrees_scratch = mcontext->LU_degrees_scratch;
      LU_rows_scratch = mcontext->LU_rows_scratch;
      #endif
  
      /* If threshold is bellow 1.0 then prepare degrees array. */
      if (mcontext->thresh < 1.0)
	prepare_degree_array(mcontext, pivot_supercol, factor_block->pivot_rows, l_size, LU_degrees_scratch);
      
      /* Do the LU factorization of the upper part */
      #ifdef TAUCS_CILK
      if (dense_spawn)
	{
	  TAUCS_PROFILE_START(taucs_profile_multilu_dense_fctr);
	  TAUCS_DENSE_SPAWN taucs_dtl(C_LU)(factor_block->LU1, l_size, col_b_size, l_size, mcontext->thresh, 
					    LU_degrees_scratch, factor_block->pivot_rows, LU_rows_scratch);
	  TAUCS_DENSE_SYNC;
	  TAUCS_PROFILE_STOP(taucs_profile_multilu_dense_fctr);
	}
      else 
      #endif
	{
	  TAUCS_PROFILE_START(taucs_profile_multilu_dense_fctr);
	  taucs_dtl(S_LU)(factor_block->LU1, l_size, col_b_size, l_size, mcontext->thresh, 
			  LU_degrees_scratch, factor_block->pivot_rows, LU_rows_scratch);
	  TAUCS_PROFILE_STOP(taucs_profile_multilu_dense_fctr);
	}
      
      /* Rearrange columns so that parent's columns are first */
      #ifdef TAUCS_CILK
      taucs_free(LU_rows_scratch);
      if (mcontext->thresh < 1.0)
	taucs_free(LU_degrees_scratch);
      #endif
    }
}

cilk void factorize_supercolumn(multilu_context* mcontext, int pivot_supercol, int *map_cols)
{
  multilu_factor_block *factor_block;
  multilu_contrib_block *new_contrib_block = NULL;
  taucs_multilu_etree *etree = &mcontext->symbolic->etree;
  int row_b_size, col_b_size, l_size, mu_size, ru_size;
  int i, desc;

  /* dummies for non-cilk case */
  int dense_spawn = 0, one_child = 0;

  #ifdef TAUCS_CILK 

  int *rows_scratch;

  int only_child = 
    etree->parent[pivot_supercol] != MULTILU_SYMBOLIC_NONE && 
    etree->first_child[etree->parent[pivot_supercol]] == pivot_supercol &&
    etree->next_child[pivot_supercol] == MULTILU_SYMBOLIC_NONE;


  int parent_has_job =
    etree->parent[pivot_supercol] != MULTILU_SYMBOLIC_NONE && 
    mcontext->symbolic->l_size[ etree->parent[pivot_supercol]] > 0;

  one_child =
    etree->first_child[pivot_supercol] != MULTILU_SYMBOLIC_NONE &&
    etree->next_child[etree->first_child[pivot_supercol]] == MULTILU_SYMBOLIC_NONE; 

  dense_spawn = mcontext->symbolic->supercolumn_size[pivot_supercol] >= MULTILU_MIN_SIZE_DENSE_SPAWN && Cilk_active_size > 1;

  #endif


  /* Factorize the L porition */
  spawn factorize_l_portion(mcontext, pivot_supercol, map_cols, dense_spawn, one_child);

  /* If one child then we must now compelte a portion of the child's multiply */
  #ifdef TAUCS_CILK
  if (one_child && Cilk_active_size > 1)
    {
      int child;
      multilu_factor_block *child_factor_block;
      multilu_contrib_block *child_contrib_block;

      child = etree->first_child[pivot_supercol];
      child_factor_block = mcontext->F->blocks[child];
      child_contrib_block = child_factor_block->contrib_block;

      if (child_contrib_block != NULL && child_factor_block->non_pivot_cols_number - child_contrib_block->num_cols_in_parent > 0)
	{
	  int num_inparent = child_contrib_block->num_cols_in_parent;
	  TAUCS_DENSE_SPAWN taucs_dtl(C_CaddMABT)(child_contrib_block->m, child_factor_block->non_pivot_cols_number - num_inparent, child_factor_block->col_pivots_number,
						  child_factor_block->L2, child_factor_block->row_pivots_number + child_factor_block->non_pivot_rows_number,
						  child_factor_block->Ut2 + num_inparent, child_factor_block->non_pivot_cols_number,
						  child_contrib_block->values + num_inparent * child_contrib_block->ld, child_contrib_block->ld);
	}
    }
  #endif

  /* Here is the sync for the l_portion and compeleting of the parent */
  sync;

  /* Intilize varaibles */
  factor_block = mcontext->F->blocks[pivot_supercol];
  if (!factor_block->valid)
    return;
  mu_size = mcontext->symbolic->u_size[pivot_supercol]; 
  l_size = factor_block->l_size;
  col_b_size =  mcontext->symbolic->supercolumn_size[pivot_supercol];
  row_b_size = min(l_size, col_b_size);

  /* The rest... */
  if (l_size > 0)
    {
      /* Focus on the remainning part of the U block */
      ru_size = focus_rows(mcontext,
			   factor_block->pivot_rows, row_b_size,  pivot_supercol, 
			   factor_block->non_pivot_cols, factor_block->Ut2,
			   mu_size, map_cols);

      /* Compress the memory beacuse we have redundent space. We have to correct the non-pivotal part  */
      factor_block->pivot_cols = (int*)taucs_realloc(factor_block->pivot_cols, (col_b_size + ru_size) * sizeof(int));
      factor_block->non_pivot_cols = factor_block->pivot_cols + mcontext->symbolic->supercolumn_size[pivot_supercol];
      compress_values_block(&factor_block->Ut2, ru_size, row_b_size, mu_size);      

      /* OK we have a U part. Two things left: apply the pivots and create contribution block */
      if (ru_size > 0)
	{
	  int num_cols_in_parent = 0;

	  /* In cilk mode and when only child - rearrange columns so that parent's columns are first */
          #ifdef TAUCS_CILK
	  if (only_child && parent_has_job && Cilk_active_size > 1)
	    {
	     rows_scratch = taucs_malloc(ru_size * sizeof(int));

	      num_cols_in_parent = spawn rearrange_non_pivot_cols(mcontext, pivot_supercol, 
								  row_b_size, ru_size, map_cols, 
								  rows_scratch);
	      sync;
	      
	      taucs_free(rows_scratch);
	    }
          #endif

	  /* Creating the combined block from descendents and applying pivots are done in parallel (cilk) */

	  /* Here is the applying of the pivots. */
          #ifdef TAUCS_CILK
	  if (dense_spawn)
	      TAUCS_DENSE_SPAWN taucs_dtl(C_UnitLowerRightTriSolve)(ru_size, row_b_size, 
								    factor_block->LU1, l_size, 
								    factor_block->Ut2, ru_size);
	  else
	  #endif
	    {
	      TAUCS_PROFILE_START(taucs_profile_multilu_dense_fctr);
	      taucs_dtl(S_UnitLowerRightTriSolve)(ru_size, row_b_size, 
						  factor_block->LU1, l_size, 
						  factor_block->Ut2, ru_size);
	      TAUCS_PROFILE_STOP(taucs_profile_multilu_dense_fctr);
	    }

	  /* Create contribution block if it is not of zero size. */
      
	  /* 
	   * Correct row mapping, for two reasons: 
	   *   a) only interested in ones inside the contribution block 
	   *   b) we reordered them. 
	   * Notice: we do not map for pivots. Only for non-pivots and their relative location 
	   * This is because of the useage in align add to the contribution blocks. 
	   */
	  for(i = 0; i < l_size - row_b_size; i++)
	    mcontext->map_rows[factor_block->non_pivot_rows[i]] = i;

	  /* 
	   * If l_size is biger the row_b_size we have a contribution block.
	   * Here we build it, including the contribution merging.
	   * If it is not we don't have a contirbution block but we still need to zero L_memember and U_member.
	   */
	  if (l_size - row_b_size > 0)
	    {
	      /* Initilize contribution block */
	      factor_block->contrib_block = allocate_contrib_block(l_size - row_b_size, ru_size);
	      new_contrib_block = factor_block->contrib_block;
	      new_contrib_block->num_cols_in_parent = num_cols_in_parent;
	      memset(new_contrib_block->values, 0, new_contrib_block->n * new_contrib_block->m * sizeof(taucs_datatype));
	      memcpy(new_contrib_block->rows, factor_block->non_pivot_rows, (l_size - row_b_size) * sizeof(int));
	      for(i = 0; i < (l_size - row_b_size); i++)
		new_contrib_block->row_loc[i] = i;	
	      memcpy(new_contrib_block->columns, factor_block->non_pivot_cols, ru_size * sizeof(int));
	      for(i = 0; i < ru_size; i++)
		new_contrib_block->col_loc[i] = i;
	      
	      /* Experience has shown that it is better to first add from previous blcoks and the add our block */
	      
	      /* Add contributions from descendent */
	      if (etree->first_desc_index[pivot_supercol] != MULTILU_SYMBOLIC_NONE)
                
                #ifndef TAUCS_CILK
		for(desc = etree->first_desc_index[pivot_supercol]; desc < pivot_supercol; desc++)
		  {
		    multilu_factor_block *desc_factor_block = mcontext->F->blocks[desc];
		    multilu_contrib_block *desc_contrib_block = desc_factor_block->contrib_block;
		    
		    if (!is_contrib_block_empty(desc_contrib_block) && (desc_contrib_block->L_member || desc_contrib_block->U_member))
		      align_add_from(mcontext, new_contrib_block, desc_factor_block, map_cols);
		  }
                #else
		for(desc = etree->first_child[pivot_supercol]; desc != MULTILU_SYMBOLIC_NONE; desc = etree->next_child[desc])
		  spawn align_add_subtree(mcontext, new_contrib_block, desc, map_cols);
		#endif
	      
	      /* Here is the sync for both the align add and the triangular solve */
	      sync;
	      
	      /* Now we can add current contribution (if not only child, if so we delay)*/
	      TAUCS_PROFILE_START(taucs_profile_multilu_dense_fctr);
              #ifdef TAUCS_CILK
	      if (!only_child || !parent_has_job || Cilk_active_size == 1)
		{
		  if (dense_spawn)
		    {
		      TAUCS_DENSE_SPAWN taucs_dtl(C_CaddMABT)(new_contrib_block->m, new_contrib_block->n, col_b_size, 
							      factor_block->L2, l_size,
							      factor_block->Ut2, ru_size,
							      new_contrib_block->values, new_contrib_block->m);


		      TAUCS_DENSE_SYNC;
		    }
		  else
              #endif
		    taucs_dtl(S_CaddMABT)(new_contrib_block->m, new_contrib_block->n, col_b_size, 
					  factor_block->L2, l_size,
					  factor_block->Ut2, ru_size,
					  new_contrib_block->values, new_contrib_block->m);
	      #ifdef TAUCS_CILK
		}
	      #endif
	      TAUCS_PROFILE_STOP(taucs_profile_multilu_dense_fctr);
	    }  
	}
    }
  else
    ru_size = 0;  

  /* Write sizes into factor block */
  factor_block->col_pivots_number = col_b_size;
  factor_block->row_pivots_number = row_b_size;
  factor_block->non_pivot_rows_number = l_size - row_b_size;
  factor_block->non_pivot_cols_number  = ru_size;

  /* Remove _member indication when no contribution block (because of zero-sized L-portion or U-portion) */
  if (factor_block->non_pivot_rows_number == 0 || factor_block->non_pivot_cols_number == 0)
    if (etree->first_desc_index[pivot_supercol] != MULTILU_SYMBOLIC_NONE)
      for(desc = etree->first_desc_index[pivot_supercol]; desc < pivot_supercol; desc++)
	{
	  multilu_factor_block *desc_factor_block = mcontext->F->blocks[desc];
	  multilu_contrib_block *desc_contrib_block = desc_factor_block->contrib_block;
	  
	  if (desc_contrib_block != NULL)
	    {
	      desc_contrib_block->L_member = FALSE;
	      desc_contrib_block->U_member = FALSE;
	    }
	}

  /* Clear the row and col indication */
  for(i = 0; i < factor_block->row_pivots_number; i++)
    mcontext->map_rows[factor_block->pivot_rows[i]] = -1;
  for(i = 0; i < factor_block->non_pivot_rows_number; i++)
    mcontext->map_rows[factor_block->non_pivot_rows[i]] = -1;
  for(i = 0; i < factor_block->non_pivot_cols_number; i++)
    map_cols[factor_block->non_pivot_cols[i]] = -1;
}

/*************************************************************************************
 * Function: allocate_factor_block 
 *
 * Description: Allocate the factor block space, including spaces for L and U (initial)
 *
 *************************************************************************************/
void allocate_factor_block(multilu_context* mcontext, int pivot_supercol)
{
  int s, ml_size, mu_size, ru_size;
  multilu_factor_block *factor_block;

  ru_size = 0;
      	
  /* Define various sizes */
  s = mcontext->symbolic->supercolumn_size[pivot_supercol];
  mu_size = mcontext->symbolic->u_size[pivot_supercol]; 
  ml_size = mcontext->symbolic->l_size[pivot_supercol];

  /* Allocate factor block. For now we allocate by max possible */
  assert(mcontext->F->blocks[pivot_supercol] == NULL);
  factor_block =  mcontext->F->blocks[pivot_supercol] = (multilu_factor_block*)taucs_malloc(sizeof(multilu_factor_block));
  factor_block->pivot_cols = (mu_size > 0) ? (int*)taucs_malloc(sizeof(int) * mu_size) : NULL;
  factor_block->non_pivot_cols = factor_block->pivot_cols + s;
  memcpy(factor_block->pivot_cols, mcontext->symbolic->columns + mcontext->symbolic->start_supercolumn[pivot_supercol], sizeof(int) * s);
  factor_block->pivot_rows = (ml_size > 0) ? (int*)taucs_malloc(sizeof(int) * ml_size) : NULL;
  factor_block->LU1 = (ml_size > 0) ? (taucs_datatype*)taucs_malloc(sizeof(taucs_datatype) * ml_size * s) : NULL;
  memset(factor_block->LU1, 0, sizeof(taucs_datatype) * ml_size * s);
  factor_block->Ut2 = (mu_size > 0) ? (taucs_datatype*)taucs_malloc(sizeof(taucs_datatype) * mu_size * s) : NULL;
  factor_block->contrib_block = NULL;
  factor_block->l_size = 0;

  /* Check that allocation were successful */
  if ((mu_size > 0 && factor_block->pivot_cols == NULL) ||
      (ml_size > 0 && factor_block->pivot_rows == NULL) ||
      (ml_size > 0 && factor_block->LU1 == NULL) ||
      (mu_size > 0 && factor_block->Ut2 == NULL))
    factor_block->valid = FALSE;
  else
    factor_block->valid = TRUE;
}

/*************************************************************************************
 * Function: focus_supercolumn_from_child
 *
 * Description: Focuses the supercolumns (assembles it from the scattered matrix) 
 *              from all the contributions in the child subtree
 *
 *************************************************************************************/
static void focus_supercolumn_from_child(multilu_context* context, int supercol, int child)
{
  taucs_multilu_etree *etree = &context->symbolic->etree;
  int i;
 
  TAUCS_PROFILE_START(taucs_profile_multilu_focus_cols);

  /* Assemble from each contribution block */
  if (etree->first_desc_index[child] != MULTILU_SYMBOLIC_NONE)
    for(i = etree->first_desc_index[child]; i < child; i++)
      focus_supercolumn_from_contrib(context, supercol, i);
  focus_supercolumn_from_contrib(context, supercol, child);

  /* Do not kill modified row mapping because we will use it later, in align add */
  /* Ofcourse, before it's useage it will be modified because of new assemblies and data movements */

  TAUCS_PROFILE_STOP(taucs_profile_multilu_focus_cols);
}

/*************************************************************************************
 * Function: focus_supercolumn_from_contrib
 *
 * Description: Focuses the supercolumn part that is in the specified contribution
 *
 *************************************************************************************/
static void focus_supercolumn_from_contrib(multilu_context* context, int supercol, int contrib)
{
  int j, size, max_size;
  int column, col_c;
  taucs_datatype *values;
  multilu_factor_block *factor_block, *desc_factor_block;
  multilu_contrib_block *desc_contrib_block;
 
  desc_factor_block = context->F->blocks[contrib];
  desc_contrib_block = desc_factor_block->contrib_block;
	
  if (is_contrib_block_empty(desc_contrib_block))
    return;

  max_size = context->symbolic->l_size[supercol]; 
  factor_block = context->F->blocks[supercol];
  size = factor_block->l_size;

  /* Go over each column of the supercolumn and assemble from contribution blocks */ 	
  values = factor_block->LU1;	
  for(col_c = 0; col_c < context->symbolic->supercolumn_size[supercol]; col_c++)
    {
      int loc_arr;
      
      column = context->symbolic->columns[context->symbolic->start_supercolumn[supercol] + col_c];	      
      
      loc_arr = is_member(column, desc_contrib_block->columns, desc_contrib_block->n);
      if (loc_arr != -1)
	{
	  /* This location in columns array maps to location in values array... */
	  int loc_val = desc_contrib_block->col_loc[loc_arr];

	  for(j = 0; j < desc_contrib_block->m; j++)
	    {
	      int j_loc, row;
	      taucs_datatype val;
	      
	      row = desc_contrib_block->rows[j];
	      assert(!context->row_cleared[row]);
	      
	      /* Map it to real value */
	      j_loc = desc_contrib_block->row_loc[j];
	      val = desc_contrib_block->values[loc_val * desc_contrib_block->ld + j_loc];
	      
	      /* If we can we assemble to know row, otherwise we add the new row */
	      /*if (val != 0.0)
		{*/
	      if (context->map_rows[row] != -1)
		values[context->map_rows[row]] = taucs_add(values[context->map_rows[row]], val);
	      else
		{
		  factor_block->pivot_rows[size] = row;
		  values[size] = val;
		  context->map_rows[row] = size;
		  size++;
		}
	      /*  } */
	    }
	  
	  /* Make contribution block smaller. If we can - kill it */
	  desc_contrib_block->n--;
	  if (desc_contrib_block->n == 0)
	    {
	      free_contrib_block(desc_contrib_block);
	      desc_factor_block->contrib_block = NULL;
	      desc_contrib_block = NULL;
	      break;
	    }
	  else
	    {
	      desc_contrib_block->columns[loc_arr] = desc_contrib_block->columns[desc_contrib_block->n];
	      desc_contrib_block->col_loc[loc_arr] = desc_contrib_block->col_loc[desc_contrib_block->n];
	    }
	  
	  /* Mark as U_member */
		desc_contrib_block->U_member = TRUE;
	}
	    
      values += max_size;
      assert(size <= max_size);
    }

  factor_block->l_size = size;
  
  /* Do not kill modified row mapping because we will use it later, in align add */
  /* Ofcourse, before it's useage it will be modified because of new assemblies and data movements */
}

/*************************************************************************************
 * Function: focus_supercolumn_from_A
 *
 * Description: Focuses the supercolumn's part that is in the original matrix (A) 
 *
 *************************************************************************************/
static void focus_supercolumn_from_A(multilu_context* context, int supercol)
{
  int i, size, max_size;
  int column, col_c;
  taucs_datatype *values;
  multilu_factor_block *factor_block;
 
  TAUCS_PROFILE_START(taucs_profile_multilu_focus_cols);

  max_size = context->symbolic->l_size[supercol]; 
  factor_block = context->F->blocks[supercol];
  size = factor_block->l_size;

  /* Go over each column of the supercolumn and assemble from original matrix */
  values = factor_block->LU1;
  for(col_c = 0; col_c < context->symbolic->supercolumn_size[supercol]; col_c++)
    {
      column = factor_block->pivot_cols[col_c];
      assert(!context->column_cleared[column]);
      
      /* First assemble from the matrix */
      for(i = context->A->colptr[column]; i < context->A->colptr[column + 1]; i++)
	{
	  int row = context->A->rowind[i];
	  
	  /* Check if row cleared */
	  if (context->row_cleared[row])
	    continue;
	  
	  /* If a previous column had this row then we use the same index. Otherwise we reallocate index */
	  if (context->map_rows[row] != -1)
	    values[context->map_rows[row]] = taucs_add(values[context->map_rows[row]], (context->A->taucs_values)[i]);
	  else
	    {
	      factor_block->pivot_rows[size] = row;
	      values[size] = (context->A->taucs_values)[i];
	      context->map_rows[row] = size;
	      size++;
	    }		  
	}
      
      /* Mark column as cleared */
      context->column_cleared[column] = TRUE;
      
      assert(size <= max_size);
      values += max_size;
    }
    
  factor_block->l_size = size;

  /* Do not kill modified row mapping because we will use it later, in align add */
  /* Ofcourse, before it's useage it will be modified because of new assemblies and data movements */

  TAUCS_PROFILE_STOP(taucs_profile_multilu_focus_cols);
}

/*************************************************************************************
 * Function: focus_rows
 *
 * Description: Assembles the values of the given rows. "Focuses on it" since it's
 *              values are scattered around the datastructures. One has to remember
 *              that we run this function after the focus on the supercolumn thus the pivot
 *              columns are not member of the rows.
 *
 *              Why do we pass the pivot_supercol? Because then we know that we need only 
 *              look at ancestors of the pivot_supercol for values in the row. And we
 *              have to assemble only from descendents.
 *
 *************************************************************************************/
static int focus_rows(multilu_context* context, 
		      int *rows, int number, 
		      int pivot_supercol, int *ind, 
		      taucs_datatype *values, int max_size, 
		      int *map_cols)
{
  int c, i, row, row_ind, size;
  taucs_multilu_etree *etree = &context->symbolic->etree;
  taucs_datatype *original_values = values;

  TAUCS_PROFILE_START(taucs_profile_multilu_focus_rows);

  /* Initilze to zero, because will have holes... */
  /* TODO: Check if good to avoid memset by finding size beforehand. */
  memset(values, 0, sizeof(taucs_datatype) * max_size * number);

  size = 0;

  /* Go over each row. */
  /* TODO: Do it smarter by using row mapping */
  for(row_ind = 0; row_ind < number; row_ind++)
    {
      row = rows[row_ind];
      assert(!context->row_cleared[row]);

      /* First assemble from the matrix (use matrix transpose) */
      for(i = context->At->colptr[row]; i < context->At->colptr[row + 1]; i++)
	{
	  int column = context->At->rowind[i];

	  /* Check if row cleared */
	  if (context->column_cleared[column])
	    continue;

	  /* If a previous column had this row then we use the same index. Otherwise we reallocate index */
	  if (map_cols[column] != -1)
	    values[map_cols[column]] = (context->At->taucs_values)[i];
	  else
	    {
	      ind[size] = column;
	      values[size] = (context->At->taucs_values)[i];
	      map_cols[column] = size;
	      size++;
	    }
	}

      /* Mark row as cleared */
      context->row_cleared[row] = TRUE;

      /* Increase pointers to point to next column location */
      assert(size <= max_size);
      values += max_size;

    }

  /* Assemble from contribution blocks */
  /* Go over only descendents, but of pivot column ofcourse. This gives the contrib blocks */
  /* of children of that pivot column (so they were factored before) */
  /* We start from the first descendent till our pivot supercolumn. This because every supercolumn 
     can contain many subsupercolumns */
  if (etree->first_desc_index[pivot_supercol] != MULTILU_SYMBOLIC_NONE)
    for(c = etree->first_desc_index[pivot_supercol]; c < pivot_supercol; c++)
      {
	multilu_factor_block *desc_factor_block = context->F->blocks[c];
	multilu_contrib_block *desc_contrib_block = desc_factor_block->contrib_block;
	
	if (is_contrib_block_empty(desc_contrib_block))
	  continue;
	  
	/* Go over each row and assemble from contribution blocks */
	values = original_values;
	for(row_ind = 0; row_ind < number; row_ind++)
	  {
	    int loc_arr;
	    
	    row = rows[row_ind];
	    
	    loc_arr = is_member(row, desc_contrib_block->rows, desc_contrib_block->m);
	    
	    if (loc_arr != -1)
	      {
		/* This location in rows array maps to location in values array... */
		int loc_val = desc_contrib_block->row_loc[loc_arr];
		
		for(i = 0; i < desc_contrib_block->n; i++)
		  {
		    int i_loc, col;
		    taucs_datatype val;
		    
		    col = desc_contrib_block->columns[i];
		    assert(!context->column_cleared[col]);
		    
		    /* Map it to a real value */
		    i_loc = desc_contrib_block->col_loc[i];
		    val = desc_contrib_block->values[i_loc * desc_contrib_block->ld + loc_val];
		    
		    /* if (val != 0.0) 
		       {*/
		    if (map_cols[col] != -1)
		      values[map_cols[col]] = taucs_add(values[map_cols[col]], val);
		    else
		      {	
			ind[size] = col;
			values[size] = val;
			      map_cols[col] = size;
			      size++;
		      }
		    /*}	*/
		  }
		
		/* Make contribution block smaller. If we can - kill it */
		desc_contrib_block->m--;
		if (desc_contrib_block->m == 0)
		  {
		    free_contrib_block(desc_contrib_block);
		    desc_factor_block->contrib_block = NULL;
		    desc_contrib_block = NULL;
		    break;
		  }
		else
		  {
		    desc_contrib_block->rows[loc_arr] = desc_contrib_block->rows[desc_contrib_block->m];
		    desc_contrib_block->row_loc[loc_arr] = desc_contrib_block->row_loc[desc_contrib_block->m];
		  }
		
		/* Mark as L_member */
		desc_contrib_block->L_member = TRUE;
	      }
	    
	    /* Increase pointers to point to next column location */
	    assert(size <= max_size);
	    values += max_size;
	  }
      }

  /* Do not kill modified column mapping because we will use it later, in align add */
  
  TAUCS_PROFILE_STOP(taucs_profile_multilu_focus_rows);
  return size;
}

/*************************************************************************************
 * Function: rearrange_non_pivot_cols
 *
 * Description: rearrange the order of the non-pivot cols so that parent's columns
 *              are first
 *
 *************************************************************************************/
#ifdef TAUCS_CILK
cilk static int rearrange_non_pivot_cols(multilu_context* mcontext, int pivot_supercol, int row_b_size, int ru_size, int *map_cols, int *scratch)
{
  int column, parent_supercol, num_inparent, col_c, switched_column, column_location;
  multilu_factor_block *factor_block;

  factor_block = mcontext->F->blocks[pivot_supercol];
  parent_supercol = mcontext->symbolic->etree.parent[pivot_supercol];

  if (parent_supercol == MULTILU_SYMBOLIC_NONE)
    return 0;

  /* Find the rearrangements that need to be done and do them in the non_pivot_cols and map_cols */
  num_inparent = 0;
  for(col_c = 0; col_c < mcontext->symbolic->supercolumn_size[parent_supercol]; col_c++)
    {
      column = mcontext->symbolic->columns[mcontext->symbolic->start_supercolumn[parent_supercol] + col_c];
      if (map_cols[column] != -1)
	{
	  scratch[num_inparent] = map_cols[column] + 1;
	
	  switched_column = factor_block->non_pivot_cols[num_inparent];
	  column_location = map_cols[column]; 

	  /* Switch in non_pivot_cols */
	  factor_block->non_pivot_cols[num_inparent] = column;
	  factor_block->non_pivot_cols[column_location] = switched_column;

	  /* Swithc in map_cols */
	  map_cols[switched_column] = column_location;
	  map_cols[column] = num_inparent;

	  num_inparent++;
	 
	}
    }

  /* Do swap lines */
  if (num_inparent > 0)
    {
      spawn taucs_dtl(SwapLines)(factor_block->Ut2, row_b_size, ru_size, scratch, 0, num_inparent);
      sync;
    } 

  return num_inparent;
}
#endif

/*************************************************************************************
 * Function: allocate_factor
 *
 * Description: Allocate space for the result factor. Do not allocate space
 *              for the inside of the actual L and U parts.
 *
 *************************************************************************************/
static void allocate_factor(multilu_context* context, int m, int n, int supercolumns_number, int type)
{
  context->F = (taucs_multilu_factor*)taucs_malloc(sizeof(taucs_multilu_factor));

  context->F->num_blocks = supercolumns_number;
  context->F->m = m;
  context->F->n = n;
  context->F->blocks = (multilu_factor_block**)taucs_calloc(supercolumns_number, sizeof(multilu_factor_block *));
  context->F->type = type;
}

#ifdef TAUCS_CILK

/*************************************************************************************
 * Function: align_add_subtree
 *
 * Description: Align add to the contribution block all contribution blocks from the 
 *              subtree rooted at subtree_root.
 *
 *************************************************************************************/
cilk static void align_add_subtree(multilu_context* mcontext, multilu_contrib_block *addto, int subtree_root, int *map_cols)
{
  multilu_factor_block *desc_factor_block = mcontext->F->blocks[subtree_root];
  int child;

  /* First assemble from children's subtree then assemble from me */
  for(child = mcontext->symbolic->etree.first_child[subtree_root]; child != MULTILU_SYMBOLIC_NONE; child = mcontext->symbolic->etree.next_child[child])
    spawn align_add_subtree(mcontext, addto, child, map_cols);
  sync;
      
  
  if (!is_contrib_block_empty(desc_factor_block->contrib_block))
    {
      spawn align_add_from(mcontext, addto, desc_factor_block, map_cols);
      sync;
    }
}

#endif

/*************************************************************************************
 * Function: align_add_from
 *
 * Description: Align adds to addto the contribution from the node given
 *
 *************************************************************************************/
cilk void align_add_from(multilu_context* mcontext,multilu_contrib_block *addto, multilu_factor_block *desc_factor_block, int *map_cols)
{
  multilu_contrib_block *desc_contrib_block = desc_factor_block->contrib_block;
  int i;

  TAUCS_PROFILE_START(taucs_profile_multilu_align_add);
      
  /* LUSon */
  if (desc_contrib_block->L_member && desc_contrib_block->U_member)
    {
      spawn align_add(mcontext, addto, desc_contrib_block, map_cols, 0, desc_contrib_block->m, 0, desc_contrib_block->n);
      sync;
    }

  /* Lson */
  if (desc_contrib_block->L_member && !desc_contrib_block->U_member)
    {
      spawn align_add_rows(mcontext, addto, desc_contrib_block, map_cols, 0, desc_contrib_block->m, 0, desc_contrib_block->n);
      sync;

      /* Correct contribution block's row mapping (take off removed) */
      for (i = 0; i < desc_contrib_block->m; i++)
	if (mcontext->map_rows[desc_contrib_block->rows[i]] != -1)
	  {
	    desc_contrib_block->m--;
	    desc_contrib_block->rows[i] = desc_contrib_block->rows[desc_contrib_block->m];
	    desc_contrib_block->row_loc[i] = desc_contrib_block->row_loc[desc_contrib_block->m];	    

	    /* We deleted one so take i back */
	    i--;
	  }
    }
  
  /* Uson */
  if (!desc_contrib_block->L_member && desc_contrib_block->U_member)				
    {
      spawn align_add_cols(mcontext, addto, desc_contrib_block, map_cols, 0, desc_contrib_block->m, 0, desc_contrib_block->n);
      sync;

      /* Correct contribution block's col mapping (take off removed) */
      for (i = 0; i < desc_contrib_block->n; i++)
	if (map_cols[desc_contrib_block->columns[i]] != -1)
	  {
	    desc_contrib_block->n--;
	    desc_contrib_block->columns[i] = desc_contrib_block->columns[desc_contrib_block->n];
	    desc_contrib_block->col_loc[i] = desc_contrib_block->col_loc[desc_contrib_block->n];	    

	    /* We deleted one so take i back */
	    i--;
	  }
    }
  
  /* If we can kill the contribution block. Otherwise unmark it (reset L_member and U_member) */
  if (desc_contrib_block->n == 0 || desc_contrib_block->m == 0)
    {
      free_contrib_block(desc_contrib_block);
      desc_factor_block->contrib_block = NULL;
    }
  else
    {
      desc_contrib_block->L_member = FALSE;
      desc_contrib_block->U_member = FALSE;
    }
  
  TAUCS_PROFILE_STOP(taucs_profile_multilu_align_add);
}

/*************************************************************************************
 * Function: align_add
 *
 * Description: Align adds the addfrom contribution block to addto. This function 
 *              assumes that addfrom is fully contained inside addto. At the end addfrom
 *              is discarded.
 *
 *************************************************************************************/
cilk void align_add(multilu_context* mcontext ,multilu_contrib_block *addto, multilu_contrib_block *addfrom, int *map_cols, 
		    int m0, int m1, int n0, int n1)
{
  int i, j, i_from, i_loc, j_from, j_loc, i_to_ind, j_to_ind;
  taucs_datatype *v, *v2;

  #ifdef TAUCS_CILK
  if (Cilk_active_size > 1)
    {
      if (n1 - n0 > MULTILU_ALIGN_ADD_SMALL)
	{
	  int nhalf = (n0 + n1) / 2;
	  
	  spawn align_add(mcontext, addto, addfrom, map_cols, m0, m1, n0, nhalf);
	  spawn align_add(mcontext, addto, addfrom, map_cols, m0, m1, nhalf, n1);
	  sync;
	
	  return;
	}
      
      if (m1 - m0 > MULTILU_ALIGN_ADD_SMALL)
	{
	  int mhalf = (m0 + m1) / 2;
	  
	  spawn align_add(mcontext, addto, addfrom, map_cols, m0, mhalf, n0, n1);
	  spawn align_add(mcontext, addto, addfrom, map_cols, mhalf, m1, n0, n1);
	  sync;
	  
	  return;
	}
    }
  #endif

  /* Use the mapping. We know that there is a mapping so utilize it */
  for(j = n0; j < n1; j++)
    {
      /* Get column that we are adding and map it to column in new block */
      j_from = addfrom->columns[j];
      j_loc = addfrom->col_loc[j];
      j_to_ind = map_cols[j_from];
	  
      v =  &(addto->values[j_to_ind * addto->ld]);
      v2 = &(addfrom->values[j_loc * addfrom->ld]);
      for(i = m0; i < m1; i++)
	{
	  /* Get row that we are adding and map it to row in new block */
	  i_from = addfrom->rows[i];
	  i_loc = addfrom->row_loc[i];
	  i_to_ind = mcontext->map_rows[i_from];
	    
	  v[i_to_ind] = taucs_add(v[i_to_ind], v2[i_loc]);
	}
    }

  /* Mark the contrib block as empty */
  addfrom->n = 0;
}

/*************************************************************************************
 * Function: align_add_rows
 *
 * Description: Align adds the addfrom contribution block to addto. This function 
 *              assumes that addfrom columns are fully contained in addto's.
 *
 *************************************************************************************/
cilk void align_add_rows(multilu_context* mcontext, multilu_contrib_block *addto, multilu_contrib_block *addfrom, int *map_cols,
			 int m0, int m1, int n0, int n1)
{
  int i, j, i_loc, j_loc, i_from, j_from, i_to_ind, j_to_ind;
  
  #ifdef TAUCS_CILK
  if (Cilk_active_size > 1)
    { 
      if (n1 - n0 > MULTILU_ALIGN_ADD_SMALL)
	{
	  int nhalf = (n0 + n1) / 2;
	  
	  spawn align_add_rows(mcontext, addto, addfrom, map_cols, m0, m1, n0, nhalf);
	  spawn align_add_rows(mcontext, addto, addfrom, map_cols, m0, m1, nhalf, n1);
	  sync;
	  
	  return;
	}
      
      if (m1 - m0 > MULTILU_ALIGN_ADD_SMALL)
	{
	  int mhalf = (m0 + m1) / 2;
	  
	  spawn align_add_rows(mcontext, addto, addfrom, map_cols, m0, mhalf, n0, n1);
	  spawn align_add_rows(mcontext, addto, addfrom, map_cols, mhalf, m1, n0, n1);
	  sync;
	  
	  return;
	}
    }
  #endif

  /* Use the mapping. We know that there is a mapping so utilize it */
  /* TODO: Notice the i,j order for better speed */
  for(i = m0; i < m1; i++)
    {
      /* Get row that we are adding */
      i_from = addfrom->rows[i];
      i_loc = addfrom->row_loc[i];
      
      i_to_ind = mcontext->map_rows[i_from];
      
      /* Check if not a member */
      if (i_to_ind == -1)
	continue; 
      assert(addto->rows[i_to_ind] == i_from);
      
      /* Now do the actual assembly of the row */
      for(j = n0; j < n1; j++)
	{
	  /* Get column that we are adding and map it to column in new block */
	  j_from = addfrom->columns[j];
	  j_loc = addfrom->col_loc[j];
	  j_to_ind = map_cols[j_from];
	  /*	 if (j_to_ind == -1)
		 continue;*/
	  assert(addto->columns[j_to_ind] == j_from);
	  
	  addto->values[j_to_ind * addto->ld + i_to_ind] = taucs_add(
								     addto->values[j_to_ind * addto->ld + i_to_ind],
								     addfrom->values[j_loc * addfrom->ld + i_loc]
								     );
	}
    }
} 


/*************************************************************************************
 * Function: align_add_cols
 *
 * Description: Align adds the addfrom contribution block to addto. This function 
 *              assumes that addfrom rows are fully contained in addto's.
 *
 *************************************************************************************/
cilk void align_add_cols(multilu_context* mcontext, multilu_contrib_block *addto, multilu_contrib_block *addfrom, int *map_cols,
			 int m0, int m1, int n0, int n1)
{
  int i, j, i_loc, j_loc, i_from, j_from, i_to_ind, j_to_ind;
  taucs_datatype *v, *v2;

  #ifdef TAUCS_CILK
  if (Cilk_active_size > 1)
    {
      if (n1 - n0 > MULTILU_ALIGN_ADD_SMALL)
	{
	  int nhalf = (n0 + n1) / 2;
	  
	  spawn align_add_cols(mcontext, addto, addfrom, map_cols, m0, m1, n0, nhalf);
	  spawn align_add_cols(mcontext, addto, addfrom, map_cols, m0, m1, nhalf, n1);
	  sync;
	  
	  return;
	}
      
      if (m1 - m0 > MULTILU_ALIGN_ADD_SMALL)
	{
	  int mhalf = (m0 + m1) / 2;
      
	  spawn align_add_cols(mcontext, addto, addfrom, map_cols, m0, mhalf, n0, n1);
	  spawn align_add_cols(mcontext, addto, addfrom, map_cols, mhalf, m1, n0, n1);
	  sync;
	  
	  return;
	}
    }
  #endif

  /* Use the mapping. We know that there is a mapping so utilize it */
  /* TODO: Notice the i,j order for better speed */
  for(j = n0; j < n1; j++)
    {
      /* Get column that we are adding */
      j_from = addfrom->columns[j];
      j_loc = addfrom->col_loc[j];

      j_to_ind = map_cols[j_from];

      /* Check if not a member */
      if (j_to_ind == -1)
	continue;

      v =  &(addto->values[j_to_ind * addto->ld]);
      v2 = &(addfrom->values[j_loc * addfrom->ld]);

      /* Now do the actual assembly of the column */
      for(i = m0; i < m1; i++)		
	{
	  /* Get column that we are adding and map it to column in new block */
	  i_from = addfrom->rows[i];
	  i_loc = addfrom->row_loc[i];
	  i_to_ind = mcontext->map_rows[i_from];
	
	  v[i_to_ind] = taucs_add(v[i_to_ind], v2[i_loc]);
	}
    }
}

/*************************************************************************************
 * Function: allocate_contrib_block
 *
 * Description: Allocate a contrib block of the given size.
 *
 *************************************************************************************/
static multilu_contrib_block *allocate_contrib_block(int l_size, int u_size)
{
  multilu_contrib_block *block;
  
  block = (multilu_contrib_block*)taucs_malloc(sizeof(multilu_contrib_block));
  if (block == NULL)
    return NULL;

  block->m = l_size;
  block->ld = l_size;
  block->n = u_size;
  block->rows = (int*)taucs_malloc(l_size * sizeof(int));
  block->row_loc = (int*)taucs_malloc(l_size * sizeof(int));
  block->columns = (int*)taucs_malloc(u_size * sizeof(int));
  block->col_loc = (int*)taucs_malloc(u_size * sizeof(int));
  block->values = (taucs_datatype*)taucs_malloc(l_size * u_size * sizeof(taucs_datatype));
  block->num_cols_in_parent = 0;
  block->L_member = FALSE;
  block->U_member = FALSE;

  /* Free factor block if failed */
  if (block->rows == NULL || block->row_loc == NULL || 
      block->columns == NULL || block->col_loc == NULL ||
      block->values == NULL)
    {
      free_contrib_block(block);

      return NULL;
    }

  return block;
}

/*************************************************************************************
 * Function: free_contrib_block
 *
 * Description: Free the memory associated with the given contribution block.
 *
 *************************************************************************************/
static void free_contrib_block(multilu_contrib_block *block)
{
  block->n = 0;
  block->m = 0;
  taucs_free(block->rows);
  taucs_free(block->row_loc);
  taucs_free(block->columns);
  taucs_free(block->col_loc);
  taucs_free(block->values);

  taucs_free(block);
}

/*************************************************************************************
 * Function: prepare_degree_array
 *
 * Description: When factorization supercol write an upper estimate of the row degrees
 *              of rows into degrees.
 *
 *************************************************************************************/
void prepare_degree_array(multilu_context* context,int supercol, int *rows, int size, int *degrees)
{
  taucs_multilu_etree *etree = &context->symbolic->etree;
  int i, j;

  TAUCS_PROFILE_START(taucs_profile_multilu_degrees);

  /* For now - degrees are original row sizes + sum of the updates */
  /* TODO: Use better degree estimates? */
  memset(degrees, 0, size * sizeof(int));

  /* Add leftover from original rows */
  for(i = 0; i < size; i++)
    for (j = context->At->colptr[rows[i]]; j < context->At->colptr[rows[i] + 1]; j++)
      if (!context->column_cleared[context->At->rowind[j]])
	degrees[i]++;

  /* Add size of updates */
  /* We start from the first descendent till our pivot supercolumn. This because every supercolumn 
     can contain many subsupercolumns */
  if (etree->first_desc_index[supercol] != MULTILU_SYMBOLIC_NONE)
    for(i = etree->first_desc_index[supercol]; i < supercol; i++)
    {
      multilu_factor_block *desc_factor_block = context->F->blocks[i];
      multilu_contrib_block *desc_contrib_block = desc_factor_block->contrib_block;
      
      if (is_contrib_block_empty(desc_contrib_block))
	continue;
      
      for(j = 0; j < desc_contrib_block->m; j++)
	{
	  int row = desc_contrib_block->rows[j];
	  if (context->map_rows[row] != -1)
	    degrees[context->map_rows[row]] += desc_contrib_block->n;
	}
    }
  
  TAUCS_PROFILE_STOP(taucs_profile_multilu_degrees);
}

/*************************************************************************************
 * Function: compress_values_blocks
 *
 * Description: *values points to a mxn matrix with ld load. This functions compresses
 *              the matrix so that *values points to mxn matrix with m load
 *
 *************************************************************************************/
static void compress_values_block(taucs_datatype **values, int m, int n, int ld)
{
  int i;
  taucs_datatype *original_values;
  
  TAUCS_PROFILE_START(taucs_profile_multilu_compress);
  
  /* Handle the case we are compressing to zero sized block */
  if (m == 0 || n == 0)
    {
      taucs_free(*values);
      *values = NULL;
      return;
    }

  /* Move the values to the upper-left of the block */
  original_values = *values;
  for(i = 1; i < n; i++)
    memcpy(original_values + i * m, original_values + i * ld, m * sizeof(taucs_datatype));

  /* Realloc memory */
  *values = (taucs_datatype*)taucs_realloc(*values, m * n * sizeof(taucs_datatype));
  
  TAUCS_PROFILE_STOP(taucs_profile_multilu_compress);
}

/*************************************************************************************
 * Function: get_map_cols
 *
 * Description: Gets from the context a pre-allocated map_cols array. If this is 
 *              a non-cilked version of the code then we just have to return the 
 *              array. But if it is cilked we must manage the map_cols array 
 *              linked-list. The idea is to avoid the memset to -1.
 *
 *************************************************************************************/
cilk static int *get_map_cols(multilu_context *mcontext)
{
#ifndef TAUCS_CILK
  return mcontext->map_cols;
#else
  if (Cilk_active_size == 1)
    return mcontext->map_cols;
  else
    {
      int *map_cols;

      Cilk_lock(mcontext->lock_select_map_cols);
      assert(mcontext->start_free_map_cols != -1);
      map_cols = mcontext->map_cols + mcontext->start_free_map_cols;
      mcontext->start_free_map_cols = map_cols[0];
      map_cols[0] = -1;
      Cilk_unlock(mcontext->lock_select_map_cols);

      return map_cols;
    }
#endif
}

/*************************************************************************************
 * Function: release_map_cols
 *
 * Description: Counter to get_map_cols this function releases the lock.
 *              Before returning it the values must be set to -1.
 *
 *************************************************************************************/
cilk static void release_map_cols(multilu_context *mcontext, int *map_cols)
{
#ifdef TAUCS_CILK
  if (Cilk_active_size > 1)
    {
      Cilk_lock(mcontext->lock_select_map_cols);
      assert(map_cols[mcontext->A->n] != -1);
      map_cols[0] = mcontext->start_free_map_cols;
      mcontext->start_free_map_cols = map_cols[mcontext->A->n];
      Cilk_unlock(mcontext->lock_select_map_cols);
    }
#endif
}

#endif /* not TAUCS_CORE_GENERAL for local static functions */


/*************************************************************************************
 *************************************************************************************
 * SOLVE PHASE
 *************************************************************************************
 *************************************************************************************/

/*************************************************************************************
 * Function prototypes 
 *************************************************************************************/
#ifndef TAUCS_CORE_GENERAL

cilk static void solve_blocked_L(taucs_multilu_factor *F, 
				 taucs_datatype *X, taucs_datatype *B, taucs_datatype *T, 
				 int n, int ld_B, int ld_X);
cilk static void solve_blocked_U(taucs_multilu_factor *F, 
				 taucs_datatype *X, taucs_datatype *B, taucs_datatype *T, 
				 int n, int ld_B, int ld_X);

#endif /* not TAUCS_CORE_GENERAL for the  function prototypes */ 

/*************************************************************************************
 * Sub-system CORE_GENERAL API functions 
 *************************************************************************************/
#ifdef TAUCS_CORE_GENERAL

/*************************************************************************************
 * Function: taucs_multilu_solve
 *
 * Description: Solves the system Ax = b when A is given in blocked format 
 *              
 *************************************************************************************/
cilk int taucs_multilu_solve(taucs_multilu_factor *F, void *x, void *b)
{
  int r = TAUCS_ERROR;

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (F->type == TAUCS_DOUBLE)
    r = spawn taucs_dmultilu_solve(F, x, b);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (F->type == TAUCS_SINGLE)
    r = spawn taucs_smultilu_solve(F, x, b);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (F->type == TAUCS_DCOMPLEX)
    r = spawn taucs_zmultilu_solve(F, x, b);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (F->type == TAUCS_SCOMPLEX)
    r = spawn taucs_cmultilu_solve(F, x, b);
#endif
  
  sync;
  return r;
}

/*************************************************************************************
 * Function: taucs_multilu_solve_many
 *
 * Description: Solves the system AX=B, returnning X (and allocating it). A is given
 *              as an blocked LU factor. Here B has as many rows has A has columns, 
 *              and it has n columns. It is given in column major mode with load ld_B.
 *              Output will have a load of ld_X.
 *              
 *************************************************************************************/
cilk int taucs_multilu_solve_many(taucs_multilu_factor *F,
				  int n, 
				  void* X, int ld_X,
				  void* B, int ld_B)
{
  int r = TAUCS_ERROR;

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (F->type == TAUCS_DOUBLE)
    r = spawn taucs_dmultilu_solve_many(F, n, X, ld_X, B, ld_B);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (F->type == TAUCS_SINGLE)
    r = spawn taucs_smultilu_solve_many(F, n, X, ld_X, B, ld_B);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (F->type == TAUCS_DCOMPLEX)
    r = spawn taucs_zmultilu_solve_many(F, n, X, ld_X, B, ld_B);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (F->type == TAUCS_SCOMPLEX)
    r = spawn taucs_cmultilu_solve_many(F, n, X, ld_X, B, ld_B);
#endif
  
  sync;
  return r;
}

/*************************************************************************************
 * Function: taucs_lu_solve_many
 *
 * Description: Solves the system AX=B, returnning x (and allocating it). A is given
 *              as an LU factor. Here B has as many rows has A has columns, 
 *              and it has n columns. It is given in column major mode with load ld_B.
 *              Output will have a load of ld_X.
 *
 *************************************************************************************/
cilk int taucs_lu_solve_many(taucs_lu_factor *F,
			     int n, 
			     void* X, int ld_X,
			     void* B, int ld_B)
{
  int r = TAUCS_ERROR; 

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (F->L->flags & TAUCS_DOUBLE)
    r = spawn taucs_dlu_solve_many(F, n, X, ld_X, B, ld_B);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (F->L->flags & TAUCS_SINGLE)
    r = spawn taucs_slu_solve_many(F, n, X, ld_X, B, ld_B);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (F->L->flags &  TAUCS_DCOMPLEX)
    r = spawn taucs_zlu_solve_many(F, n, X, ld_X, B, ld_B);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (F->L->flags == TAUCS_SCOMPLEX)
    r = spawn taucs_clu_solve_many(F, n, X, ld_X, B, ld_B);
#endif
  
  sync;
  return r;
}

/*************************************************************************************
 * Function: taucs_lu_solve
 *
 * Description: Solves the system Ax = b when A is given in LU format 
 *              
 *************************************************************************************/
cilk int taucs_lu_solve(taucs_lu_factor *F, void *x, void *b)
{
  int r = TAUCS_ERROR;

#ifdef TAUCS_DOUBLE_IN_BUILD
  if (F->L->flags & TAUCS_DOUBLE)
    r = spawn taucs_dlu_solve(F, x, b);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (F->L->flags & TAUCS_SINGLE)
    r = spawn taucs_slu_solve(F, x, b);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (F->L->flags & TAUCS_DCOMPLEX)
    r = spawn taucs_zlu_solve(F, x, b);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (F->L->flags & TAUCS_SCOMPLEX)
    r = spawn taucs_clu_solve(F, x, b);
#endif
  
  sync;
  return r;
}

#endif /* TAUCS_CORE_GENERAL for the CORE GENERAL API functions */

/*************************************************************************************
 * Sub-system DATATYPE API functions 
 *************************************************************************************/
#ifndef TAUCS_CORE_GENERAL

/*************************************************************************************
 * Function: taucs_dtl(multilu_solve)
 *
 * Description: Datatype version of taucs_multilu_solve
 *              
 *************************************************************************************/
cilk int taucs_dtl(multilu_solve)(taucs_multilu_factor *F, taucs_datatype *x, taucs_datatype *b)
{
  int r;
  r = spawn taucs_multilu_solve_many(F, 1, x, F->m, b, F->m);
  sync;

  return r;
}

/*************************************************************************************
 * Function: taucs_dtl(multilu_solve_many)
 *
 * Description: Datatype version of taucs_multilu_solve_many
 *              
 *************************************************************************************/
cilk int taucs_dtl(multilu_solve_many)(taucs_multilu_factor *F,
				       int n, 
				       taucs_datatype* X, int ld_X,
				       taucs_datatype* B, int ld_B)
{
  taucs_datatype *B_Copy, *Y, *T;
  
  /* TODO: Make this more memory efficent */

  /* Allocate memory */
  B_Copy = (taucs_datatype*)taucs_malloc(sizeof(taucs_datatype) * n * ld_B);
  Y = (taucs_datatype*)taucs_malloc(sizeof(taucs_datatype) * n * F->n);
  T = (taucs_datatype*)taucs_malloc(sizeof(taucs_datatype) * n * F->n);
  if (B_Copy == NULL || Y == NULL || T == NULL)
    {
      taucs_free(B_Copy);
      taucs_free(Y);
      taucs_free(T);
      return TAUCS_ERROR_NOMEM;
    }

  /* B_Copy will hold a copy of B */
  memcpy(B_Copy, B, sizeof(taucs_datatype) * n * ld_B);

  /* Solve LY = PB */
  spawn solve_blocked_L(F, Y, B_Copy, T, n, ld_B, F->n);
  sync;
  
  /* Solve Uinv(Q)X = Y */
  spawn solve_blocked_U(F, X, Y, T, n, F->n, ld_X);
  sync;
  
  /* Free spaces */
  taucs_free(T);
  taucs_free(Y);
  taucs_free(B_Copy);
  
  return TAUCS_SUCCESS;
}

/*************************************************************************************
 * Function: taucs_dtl(lu_solve_many)
 *
 * Description: Datatype version taucs_lu_solve_many
 *
 *************************************************************************************/
cilk int taucs_dtl(lu_solve_many)(taucs_lu_factor *F,
				  int n, 
				  taucs_datatype* X, int ld_X,
				  taucs_datatype* B, int ld_B)
{
  int i, r;
  
  /* TODO: check success for each one */
  for(i = 0; i < n; i++)
    r = spawn taucs_lu_solve(F, X + i * ld_X, B + i * ld_B);
  sync;
  
  return TAUCS_SUCCESS;
}

/*************************************************************************************
 * Function: taucs_dtl(lu_solve)
 *
 * Description: Datatype version of taucs_lu_solve
 *              
 *************************************************************************************/
cilk int taucs_dtl(lu_solve)(taucs_lu_factor *F, taucs_datatype *x, taucs_datatype *b)
{
  taucs_ccs_matrix *L, *U;
  taucs_datatype *Pb, *y, *x1;
  int n, i, col;

  n = F->n;

  /* Premute b - P*b */
  Pb = (taucs_datatype*)taucs_malloc(n * sizeof(taucs_datatype));
  if (Pb == NULL) 
    return TAUCS_ERROR_NOMEM;

  for(i = 0; i < n; i++)
    Pb[i] = b[F->r[i]];

  /* Solve Ly = Pb */
  L = F->L;
  y = Pb;
  col = 0;
  y[0] = taucs_div(y[0], (L->taucs_values)[0]);
  for(i = 1; i < L->colptr[L->n]; i++)
    {
      /* Check when to advance column */
      if (i == L->colptr[col + 1])
	{
	  col++;
	  y[col] = taucs_div(y[col], (L->taucs_values)[i]);
	} 
      else
	y[L->rowind[i]] = taucs_sub(y[L->rowind[i]], 
				    taucs_mul((L->taucs_values)[i], y[col]));
    }

  /* Solve Ux1 = y */
  U = F->U;
  x1 = y;
  col = n - 1;
  x1[col] = taucs_div(x1[col], (U->taucs_values)[U->colptr[U->n] - 1]);
  for(i = U->colptr[ U->n ] - 2; i >= 0; i--)
    {
      /* Check when to advance column */
      if (i < U->colptr[col])
	{
	  col--;
	  x1[col] = taucs_div(x1[col], (U->taucs_values)[i]);
	} 
      else
	x1[U->rowind[i]] = taucs_sub(x1[U->rowind[i]],
				     taucs_mul((U->taucs_values)[i], x1[col]));
    }

  /* Premute result back to x */
  for(i = 0; i < n; i++)
    x[F->c[i]] = x1[i];

  taucs_free(x1);

  return TAUCS_SUCCESS;
}

#endif /* not TAUCS_CORE_GENERAL for the DATATYPE API functions */

/*************************************************************************************
 * Internal functions
 *************************************************************************************/
#ifndef TAUCS_CORE_GENERAL

/*************************************************************************************
 * Function: solve_blocked_L
 *
 * Description: Solves the system LX = PB when L is the L part of a blocked format 
 *              factor and P is given by the factor block. T is a temporary workspace
 *              
 *************************************************************************************/
cilk void solve_blocked_L(taucs_multilu_factor *F, 
			  taucs_datatype *X, taucs_datatype *B, taucs_datatype *T, 
			  int n, int ld_B, int ld_X)
{
  int i, j, c;
  int ld_T;
  
  ld_T = F->n;
  for (i = 0; i < F->num_blocks; i++)
    {
      multilu_factor_block *block = F->blocks[i];

      /* Copy to X the corresponding part of B */
      for(c = 0; c < n; c++)
	for(j = 0; j < block->row_pivots_number; j++)
	  X[j + c * ld_X] = B[block->pivot_rows[j] + c * ld_B];
      
      /* Solve L1X0 = B0 (X0 and B0 are the relevent parts of X and B) */
      TAUCS_PROFILE_START(taucs_profile_multilu_dense_solve);
      TAUCS_DENSE_SPAWN taucs_dtl(C_UnitLowerLeftTriSolve)(block->row_pivots_number, n,  
							   block->LU1, block->row_pivots_number + block->non_pivot_rows_number,  
							   X, ld_X);
      TAUCS_DENSE_SYNC;
      TAUCS_PROFILE_STOP(taucs_profile_multilu_dense_solve);
      
      /* Updates to the rest of the solution vector */
      if (block->non_pivot_rows_number > 0)
	{
	  /* Copy to T the relevent parts of B */
	  for(c = 0; c < n; c++)
	    for(j = 0; j < block->non_pivot_rows_number; j++)
	      T[j + c * ld_T] = B[block->non_pivot_rows[j] + c * ld_B];
	  
	  /* T = T - L2X */
	  TAUCS_PROFILE_START(taucs_profile_multilu_dense_solve);
	  TAUCS_DENSE_SPAWN taucs_dtl(C_CaddMAB)(block->non_pivot_rows_number, n, block->row_pivots_number,
						 block->L2, block->row_pivots_number + block->non_pivot_rows_number,
						 X, ld_X,
						 T, ld_T);
	  TAUCS_DENSE_SYNC;
	  TAUCS_PROFILE_STOP(taucs_profile_multilu_dense_solve);
	  
	  /* Copy back from T to B */
	  for(c = 0; c < n; c++)
	    for(j = 0; j < block->non_pivot_rows_number; j++)
	      B[block->non_pivot_rows[j] + c * ld_B] = T[j + c * ld_T];
	}
      
      X += block->row_pivots_number;
    }
}

/*************************************************************************************
 * Function: solve_blocked_U
 *
 * Description: Solves the system Uinv(Q)X = B when U is the U part of a blocked format 
 *              factor and Q is given by the factor block. T is a temporary workspace
 *              
 *************************************************************************************/
cilk void solve_blocked_U(taucs_multilu_factor *F, 
			  taucs_datatype *X, taucs_datatype *B, taucs_datatype *T, 
			  int n, int ld_B, int ld_X)
{
  int i, j, c;
  int ld_T;
  
  ld_T = F->n;
  
  /* We advance in B from the end so we put the pointer of it to the end */
  B += F->n;
  
  for (i = F->num_blocks - 1;  i >= 0; i--)
    {
      multilu_factor_block *block = F->blocks[i];
      
      B -= block->col_pivots_number;
	  
      /* Update B if need to */
      if (block->non_pivot_cols_number > 0)
	{
	  for(c = 0; c < n; c++)
	    for(j = 0; j < block->non_pivot_cols_number; j++)
	      T[j + c * ld_T] = X[block->non_pivot_cols[j] + c * ld_X];
	  
	  TAUCS_PROFILE_START(taucs_profile_multilu_dense_solve);
	  TAUCS_DENSE_SPAWN taucs_dtl(C_CaddMATB)(block->col_pivots_number, n, block->non_pivot_cols_number,
						  block->Ut2, block->non_pivot_cols_number,
						  T, ld_T,
						  B, ld_B);
	  TAUCS_DENSE_SYNC;
	  TAUCS_PROFILE_STOP(taucs_profile_multilu_dense_solve);
	}
      
      /* Find the solution for this part of X */
      TAUCS_PROFILE_START(taucs_profile_multilu_dense_solve);
      TAUCS_DENSE_SPAWN taucs_dtl(C_UpperLeftTriSolve)(block->col_pivots_number, n,  
						       block->LU1, block->row_pivots_number + block->non_pivot_rows_number,  
						       B, ld_B); 
      TAUCS_DENSE_SYNC;
      TAUCS_PROFILE_STOP(taucs_profile_multilu_dense_solve);      
      
      /* Distribute the results in X */
      for(c = 0; c < n; c++)
	for(j = 0; j < block->col_pivots_number; j++)
	  X[block->pivot_cols[j] + c * ld_X] = B[j + c * ld_B];
    }
}

#endif /* not TAUCS_CORE_GENERAL for the internal functions */

/*************************************************************************************
 *************************************************************************************
 * FACTOR MANIPULATIONS
 *************************************************************************************
 *************************************************************************************/

/*************************************************************************************
 * Function prototypes 
 *************************************************************************************/
#ifdef TAUCS_CORE_GENERAL
static void free_factor_block(multilu_factor_block *block);
#endif /* TAUCS_CORE_GENERAL on function prototypes */

/*************************************************************************************
 * Function: taucs_multilu_factor_to_lu_factor
 *
 * Description: Converts from multilu's internal blocked LU factor to inefficent
 *              general formace (two ccs matrices)
 *              
 *************************************************************************************/
#ifdef TAUCS_CORE_GENERAL
taucs_lu_factor *taucs_multilu_factor_to_lu_factor(taucs_multilu_factor *F)
{
#ifdef TAUCS_DOUBLE_IN_BUILD
  if (F->type == TAUCS_DOUBLE)
    return taucs_dmultilu_factor_to_lu_factor(F);
#endif

#ifdef TAUCS_SINGLE_IN_BUILD
  if (F->type == TAUCS_SINGLE)
    return taucs_smultilu_factor_to_lu_factor(F);
#endif

#ifdef TAUCS_DCOMPLEX_IN_BUILD
  if (F->type == TAUCS_DCOMPLEX)
    return taucs_zmultilu_factor_to_lu_factor(F);
#endif

#ifdef TAUCS_SCOMPLEX_IN_BUILD
  if (F->type == TAUCS_SCOMPLEX)
    return taucs_cmultilu_factor_to_lu_factor(F);
#endif
  
  assert(0);
  return NULL;
}

#endif /* TAUCS_CORE_GENERAL for general factor convertor */

/*************************************************************************************
 * Function: multilu_get_lu
 *
 * Description: Convert the blocked factor to LU format
 *              
 *************************************************************************************/
#ifndef TAUCS_CORE_GENERAL
taucs_lu_factor *taucs_dtl(multilu_factor_to_lu_factor)(taucs_multilu_factor *F)
{
  /* TODO: Consider avoiding to return explicit zeros */
  /* TODO: Make this work for m != n */

  taucs_lu_factor *LU;
  taucs_ccs_matrix *Ut;
  int i, j, n, m, k, col, row, loc_L, loc_U;
  int L_nnz, Ut_nnz;
  
  TAUCS_PROFILE_START(taucs_profile_multilu_cnv_lu);
  
  n = F->n;
  m = F->m;

  /* Allocate LU and set it's sizes */
  LU = (taucs_lu_factor*)taucs_malloc(sizeof(taucs_lu_factor));
  LU->n = n;
  LU->m = m;
  
  /* Create column ordering */
  LU->c = (int*)taucs_malloc(n * sizeof(int));
  col = 0;
  for(i = 0; i < F->num_blocks; i++)
    {
      multilu_factor_block *factor_block = F->blocks[i];
      for(j = 0; j < factor_block->col_pivots_number; j++)
	{
	  LU->c[col] = factor_block->pivot_cols[j];
	  col++;
	}
    }
  assert(col == n);
  
  /* Create row ordering */
  LU->r = (int*)taucs_malloc(m * sizeof(int));
  row = 0;
  for(i = 0; i < F->num_blocks; i++)
    {
      multilu_factor_block *factor_block = F->blocks[i];
      for(j = 0; j < factor_block->row_pivots_number; j++)
	{
	  LU->r[row] = factor_block->pivot_rows[j];
	  row++;
	}
    }
  
  /* TODO: Fill in the rest of the rows. */
  assert(row == m);
  
  /* Calculate L and U sizes */
  L_nnz = Ut_nnz = 0;
  for(i = 0; i < F->num_blocks; i++)
    {
      multilu_factor_block *factor_block = F->blocks[i];
      int pl_size = factor_block->row_pivots_number;
      int pu_size = factor_block->col_pivots_number;
      int rl_size = factor_block->non_pivot_rows_number;
      int ru_size = factor_block->non_pivot_cols_number;
      
      L_nnz += ((1 + pl_size) * pl_size / 2) + (pu_size - pl_size) + (rl_size * pu_size);
      Ut_nnz += ((1 + 2 * pu_size - pl_size) * pl_size / 2) + (ru_size * pl_size);
    }
  
  /* Create matrices (allocate space */
  LU->L = taucs_ccs_create(m, n, L_nnz, F->type | TAUCS_TRIANGULAR | TAUCS_LOWER);
  Ut = taucs_ccs_create(n, m, Ut_nnz, F->type | TAUCS_TRIANGULAR | TAUCS_LOWER);

  /* Set Ut values */
  col = 0; 
  loc_U = 0;
  for(i = 0; i < F->num_blocks; i++)
    {
      /* Init some values */
      multilu_factor_block *factor_block = F->blocks[i];
      int u_size = factor_block->col_pivots_number + factor_block->non_pivot_cols_number;
      int ld_L = factor_block->row_pivots_number + factor_block->non_pivot_rows_number; 
      int ld_U = factor_block->non_pivot_cols_number;
    
      /* Create indexes */
      Ut->colptr[col] = loc_U;
      for(j = 1; j < factor_block->row_pivots_number; j++)
	Ut->colptr[col + j] = Ut->colptr[col + j - 1] + u_size - j + 1;
    
      /* Copy values in LU1 */
      for(j = 0; j < factor_block->row_pivots_number; j++)
	for(k = 0; k <= j; k++) 
	  *((Ut->taucs_values) + Ut->colptr[col + k] + j - k) = *(factor_block->LU1 + j * ld_L + k);
    
      /* Copy values in Ut2 */
      for(j = 0; j < factor_block->row_pivots_number; j++)
	{
	  loc_U = Ut->colptr[col + j];
      
	  memcpy(Ut->rowind + loc_U, factor_block->pivot_cols + j, (u_size - j) * sizeof(int));
	  memcpy((Ut->taucs_values) + loc_U + factor_block->col_pivots_number - j,
		 factor_block->Ut2 + j * ld_U, factor_block->non_pivot_cols_number * sizeof(taucs_datatype));
      
	}
      loc_U += factor_block->non_pivot_cols_number + 1;
    
      col += factor_block->row_pivots_number;
    }
  assert(loc_U == Ut_nnz);
  assert(col == m);
  
  /* Correct Ut with row order and transpose (for U) */
  Ut->colptr[n] = Ut_nnz;
  taucs_ccs_permute_rows_inplace(Ut, LU->c);
  LU->U = taucs_ccs_transpose(Ut);
  taucs_ccs_free(Ut);
  
  /* Create L values */
  col = 0;
  loc_L = 0;
  for(i = 0; i < F->num_blocks; i++)
    {
      /* Init some values */
      multilu_factor_block *factor_block = F->blocks[i];
      int l_size = factor_block->row_pivots_number + factor_block->non_pivot_rows_number;
      int ld_L = l_size;
    
      /* Copy actual pivots columns, i.e. ones that have pivot rows */
      for(j = 0; j < factor_block->row_pivots_number; j++)
	{
	  LU->L->colptr[col + j] = loc_L;
	  memcpy(LU->L->rowind + loc_L, factor_block->pivot_rows + j, l_size * sizeof(int));
	  *(((LU->L)->taucs_values) + loc_L) = taucs_one;
	  memcpy(((LU->L)->taucs_values) + loc_L + 1, factor_block->LU1 + j * (ld_L + 1) + 1, (l_size - 1) * sizeof(taucs_datatype));
	  
	  loc_L += l_size;
	  l_size--;
	}
      
      /* Create null columns for the columns that didn't get pivots */
      /* TODO: Is this correct? */
      for(j = 0; j < factor_block->col_pivots_number - factor_block->row_pivots_number; j++)
	{
	  assert(FALSE);
	  LU->L->colptr[col + j] = loc_L;
	  /* TODO: What to put here? */
	  *(((LU->L)->taucs_values) + loc_L) = taucs_one;
	  loc_L++;
	}		  
      
      col += factor_block->col_pivots_number;
    }
  assert(loc_L == L_nnz);
  assert(col == n);
  
  /* Correct row and column ordering */
  LU->L->colptr[n] = L_nnz;
  taucs_ccs_permute_rows_inplace(LU->L, LU->r);
  
  TAUCS_PROFILE_STOP(taucs_profile_multilu_cnv_lu);
  
  return LU;
}
#endif /* not TAUCS_CORE_GENERAL on DATATYPE factor conversion */

#ifdef TAUCS_CORE_GENERAL

/*************************************************************************************
 * Function: multilu_free_blocked_factor
 *
 * Description: Free blocked factor format
 *              
 *************************************************************************************/
void taucs_multilu_factor_free(taucs_multilu_factor* F)
{
  int i;

  if (F == NULL) 
    return;

  for(i = 0; i < F->num_blocks; i++)
    free_factor_block(F->blocks[i]);
  taucs_free(F->blocks);
  taucs_free(F);
}

/*************************************************************************************
 * Function: multilu_free_lu_factor
 *
 * Description: Free lu factor format
 *              
 *************************************************************************************/
void taucs_lu_factor_free(taucs_lu_factor* F)
{
  if (F == NULL) 
    return;
  taucs_ccs_free(F->L);
  taucs_ccs_free(F->U);
  taucs_free(F->c);
  taucs_free(F->r);
  taucs_free(F);
}

/*************************************************************************************
 * Sub-system Internal functions 
 *************************************************************************************/

/*************************************************************************************
 * Function: free_factor_block
 *
 * Description: Free a single factor block
 *              
 *************************************************************************************/
static void free_factor_block(multilu_factor_block *block)
{
  taucs_free(block->pivot_rows);
  taucs_free(block->pivot_cols);
  taucs_free(block->LU1);
  taucs_free(block->Ut2);
  taucs_free(block);
}

#endif /* TAUCS_CORE_GENERAL on all free functions */

/*************************************************************************************
 *************************************************************************************
 * UNION-FIND library
 *************************************************************************************
 *************************************************************************************/

#ifdef TAUCS_CORE_GENERAL

/*************************************************************************************
 * Structure: uf_setnode
 *
 * Description: A union-find set. We work with set-groups which is an array of sets.
 *
 *************************************************************************************/

#ifdef _UF_UNION_BY_RANK

typedef struct
{
  int parent;
  int rank;
} uf_setnode;

#else

/* Only parent */
typedef int uf_setnode;

#endif

/*************************************************************************************
 * Function: uf_make_sets
 *
 * Description: Create a union-find group of sets. if r = uf_make_sets(n) the 
 *              r is a n-element array of sets. r[i-1] is i'th set. In the functions
 *              (uf_union and uf_find) we give sets by index and the sets.
 *
 *************************************************************************************/
static void *uf_make_sets(int sets_num)
{
  uf_setnode *sets;
  int i;
  
  sets = taucs_malloc(sets_num * sizeof(uf_setnode));
  if (sets == NULL)
    return NULL;

  for(i = 0; i < sets_num; i++)
    {
#ifdef _UF_UNION_BY_RANK
      
      sets[i].rank = 0;
      sets[i].parent = i;
      
#else
      
      sets[i] = i;
      
#endif
    }
  
  return (void *)sets;
}

/*************************************************************************************
 * Function: uf_union
 *
 * Description: In the group sets unite x and y and return the rep of the united group
 *
 *************************************************************************************/
static int uf_union(void *_sets, int x, int y)
{
  uf_setnode *sets = (uf_setnode *)_sets;

  /* By rank */
#ifdef _UF_UNION_BY_RANK
  
  /* Find rep to unite */
  x = uf_find(sets, x);
  y = uf_find(sets, y);

  if (sets[x].rank > sets[y].rank)
    {
      sets[y].parent = x;
      return x;
    } 
  else
    {
      sets[x].parent = y;
      if (sets[x].rank == sets[y].rank)
	sets[y].rank++;
    
      return y;
    }
  
  /* Not by rank */
#else
  
  sets[x] = y;
  return y;
  
#endif
}

/*************************************************************************************
 * Function: uf_find
 *
 * Description: Find rep of x in the group sets.
 *
 *************************************************************************************/
static int uf_find(void *_sets, int x)
{
  uf_setnode *sets = (uf_setnode *)_sets;

  /* parent_x is defined otherwise if by rank or not */
  #ifdef _UF_UNION_BY_RANK
  #define parent_x sets[x].parent
  #else
  #define parent_x sets[x]
  #endif
  
  if (x != parent_x)
    parent_x = uf_find(sets, parent_x);
  return parent_x;
}

#endif /* TAUCS_CORE_GENERAL for the UNION-FIND library */

/*************************************************************************************
 *************************************************************************************
 * END OF FILE
 *************************************************************************************
 *************************************************************************************/

