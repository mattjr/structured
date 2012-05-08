#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>

#include "defs.h"
#include "structs.h"
#include "read_params.h"

typedef double * double_ptr;

void      cpvec(double   *copy,int  beg, int end, double   *vec);
double    dot(double   *vec1, int beg, int end, double *vec2); 
void      scadd(double   *vec1,int beg, int end, double  fac, double *vec2);
void      vecscale(double   *vec1,int beg, int end, double alpha, double   *vec2);
double    norm(double   *vec, int beg, int end);


double compute_gershgorin_estimate(struct vtx_data ** graph, int n, double * masses) {
    double sum, max_sum=-1;
    int i,j;
	
    if (graph[1]->ewgts==NULL) { // no edge weights
        sum=0;
        for (i=1; i<=n; i++) 
            if (graph[i]->nedges/masses[i]>sum)
                sum=graph[i]->nedges/masses[i];

        max_sum = 2*sum;
    }
    else {	
        for (i=1; i<=n; i++) {
            sum=0;
            for (j=0; j<graph[i]->nedges; j++)
                sum+=fabs(graph[i]->ewgts[j]);
            if (sum/masses[i]>max_sum)
                max_sum=sum/masses[i];
        }
    }

    return max_sum;
}

void refine_generalized_eigs(
    struct vtx_data ** graph, double * masses, int n, int dim, 
    double ** eigs, double tol, bool initialize
) {

//    std::cerr << "refine_generalized_eigs()" << std::endl ;
	
    int i,j,k;

    // allocate space for eigenvalues:
    double * evals = (double *) malloc((n+1)*sizeof(double));

    double *tmp_vec = (double *) malloc((n+1)*sizeof(double));
    double *curr_vector;

    double sum;
    double len;
    double angle;
    double alpha;

    double gersh_estimate = compute_gershgorin_estimate(graph, n, masses);
	
    int iterations;

    int largest_index;
    double largest_eval;

    double * normalized_masses = (double *) malloc((n+1)*sizeof(double));


    if (showStats) 
        fprintf(fpStatsFile, "Entering power-iteration, tol: %e, gersh est.: %e\n", 1-tol, gersh_estimate);	


    for (i=1; i<=n; i++)
        normalized_masses[i] = masses[i];
    len = norm(normalized_masses, 1, n);
    vecscale(normalized_masses, 1, n, 1.0 / len, normalized_masses);	

		
    for (i=1; i<=dim; i++) {
//        std::cerr << "---> current eigen=" << i << std::endl ;
        curr_vector = eigs[i];

        if (initialize)
            // init randomly the i-th eigen vector
            for (j=0; j<n; j++)
//                curr_vector[j] = rand();		
                curr_vector[j] = 1.0 ;
	
        // orthogonalize against degrees (the principal eigenvector)
        alpha = -dot(normalized_masses, 1, n, curr_vector);
        scadd(curr_vector, 1, n, alpha, normalized_masses);		
        // orthogonalize against higher eigenvectors
        for (j=1; j<i; j++) {
            alpha = -dot(eigs[j], 1, n, curr_vector);
            scadd(curr_vector, 1, n, alpha, eigs[j]);
        }
        len = norm(curr_vector, 1, n);
        vecscale(curr_vector, 1, n, 1.0 / len, curr_vector);	

        iterations=0;
        do {
			
            // put each node at the weighted average of its neighbors
            // which is equivaqlent to power iteration:
            cpvec(tmp_vec,1, n,curr_vector);
			
            if (graph[1]->ewgts==NULL) {
                for (j=1; j<=n; j++) {
                    sum = 0;
                    for (k=1; k<graph[j]->nedges; k++) 
                        sum += tmp_vec[graph[j]->edges[k]];
                    sum-=graph[j]->nedges*tmp_vec[j];
                    curr_vector[j] = (gersh_estimate*tmp_vec[j] + sum / masses[j]);
                }
            }
            else {			
                for (j=1; j<=n; j++) {
                    sum = 0;
                    for (k=0; k<graph[j]->nedges; k++) 
                        sum += graph[j]->ewgts[k]*tmp_vec[graph[j]->edges[k]];

                    //sum-=0.25*graph[j]->ewgts[0]*tmp_vec[j]; // change (Ulrik's idea)!!
					
                    curr_vector[j] = (gersh_estimate*tmp_vec[j] + sum / masses[j]);
                }
            }
            // M-orthogonalize against 1_n (the principal eigenvector)
            alpha = -dot(normalized_masses, 1, n, curr_vector);
            scadd(curr_vector, 1, n, alpha, normalized_masses);		
            // orthogonalize against higher eigenvectors
            for (j=1; j<i; j++) {
                alpha = -dot(eigs[j], 1, n, curr_vector);
                scadd(curr_vector, 1, n, alpha, eigs[j]);
            }
            len = norm(curr_vector, 1, n);
            vecscale(curr_vector, 1, n, 1.0 / len, curr_vector);
			
            angle = dot(curr_vector, 1, n, tmp_vec);
            ++iterations;
//            if(!(iterations % 127)) {
//                std::cerr << "iterations: " << iterations << " angle=" << angle << " tol=" << tol << std::endl ;
//            }
        } while (angle<tol);
        evals[i]=angle*len; // this is the Rayleigh quotient (up to errors due to orthogonalization):
        // u*(A*u)/||A*u||)*||A*u||, where u=last_vec, and ||u||=1

        if (showStats) 
            fprintf(fpStatsFile, "\tEigenvector #%d: #iterations: %d, eval: %e\n", i, iterations, gersh_estimate-evals[i]);

		
        // Now, to ease M-orthogonalization, distort the vector by multiplying by M:
        for (j=1; j<=n; j++) 
            curr_vector[j] *= masses[j];
        len = norm(curr_vector, 1, n);
        vecscale(curr_vector, 1, n, 1.0 / len, curr_vector);
			
    }

    // restore true eigenvectors by multiplying by M^{-1}:
    for (i=1; i<=dim; i++) {
        curr_vector = eigs[i];
        for (j=1; j<=n; j++)
            curr_vector[j] /= masses[j];
    }

    // sort vectors by their evals, for overcoming possible mis-convergence:
//    std::cerr << "sorting ...." << std::endl ;
    for (i=1; i<dim; i++) {
        largest_index=i;
        largest_eval=evals[largest_index];
        for (j=i+1; j<=dim; j++) {
            if (largest_eval<evals[j]) {
                largest_index=j;
                largest_eval=evals[largest_index];
            }
        }
        if (largest_index!=i) { // exchange eigenvectors:
            cpvec(tmp_vec,1, n,eigs[i]);
            cpvec(eigs[i],1, n,eigs[largest_index]);
            cpvec(eigs[largest_index],1, n,tmp_vec);

            evals[largest_index]=evals[i];
            evals[i]=largest_eval;
        }
    }
//    std::cerr << "sorted." << std::endl ;

    if (showStats) 
        fprintf(fpStatsFile, "Ending power-iteration\n");

    free(tmp_vec); free(evals); free(normalized_masses);
}



/* Copy a range of a double vector to a double vector */
void      cpvec(double   *copy,int  beg, int end, double   *vec)
{
    int       i;

    copy = copy + beg;
    vec = vec + beg;
    for (i = end - beg + 1; i; i--) {
	*copy++ = *vec++;
    }
}

/* Returns scalar product of two double n-vectors. */
double    dot(double   *vec1, int beg, int end, double *vec2)
{
    int       i;
    double    sum;

    sum = 0.0;
    vec1 = vec1 + beg;
    vec2 = vec2 + beg;
    for (i = end - beg + 1; i; i--) {
	sum += (*vec1++) * (*vec2++);
    }
    return (sum);
}


/* Scaled add - fills double vec1 with vec1 + alpha*vec2 over range*/
void      scadd(double   *vec1,int beg, int end, double  fac, double *vec2)
{
    int       i;

    vec1 = vec1 + beg;
    vec2 = vec2 + beg;
    for (i = end - beg + 1; i; i--) {
	(*vec1++) += fac * (*vec2++);
    }
}

/* Scale - fills vec1 with alpha*vec2 over range, double version */
void      vecscale(double   *vec1,int beg, int end, double alpha, double   *vec2)
{
    int       i;

    vec1 += beg;
    vec2 += beg;
    for (i = end - beg + 1; i; i--) {
	(*vec1++) = alpha * (*vec2++);
    }
}

/* Returns 2-norm of a double n-vector over range. */
double    norm(double   *vec, int beg, int end)
{
    return (sqrt(dot(vec, beg, end, vec)));
}

