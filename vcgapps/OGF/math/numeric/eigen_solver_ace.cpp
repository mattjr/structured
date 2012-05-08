/*
 *  OGF/Graphite: Geometry and Graphics Programming Library + Utilities
 *  Copyright (C) 2000-2005 INRIA - Project ALICE
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  If you modify this software, you should include a notice giving the
 *  name of the person performing the modification, the date of modification,
 *  and the reason for such modification.
 *
 *  Contact: Bruno Levy - levy@loria.fr
 *
 *     Project ALICE
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs. 
 *
 * As an exception to the GPL, Graphite can be linked with the following (non-GPL) libraries:
 *     Qt, SuperLU, WildMagic and CGAL
 */
 

#include <OGF/math/numeric/eigen_solver_ace.h>
#include <numeric_stuff/ACE/structs.h>
#include <numeric_stuff/ACE/amg_eigen_computation.h>
#include <fstream>

extern unsigned long coarsening_time, power_iteration_time, rqi_time;
extern int total_negative_edges, total_coarse_edges ;
extern double max_neg_percentage ;
extern int IsWeightedGraph ;
extern int levels_between_refinement ;

typedef enum{weighted, contraction, non_amg} CoarseningMethod;
extern CoarseningMethod coarseningMethod ;


namespace OGF {

    static void print_graph(vtx_data** graph, unsigned int n) {
        std::ofstream out("graph.out") ;
        for(unsigned int i=1; i<=n; i++) {
            for(int j=0; j<graph[i]->nedges; j++) {
                out << graph[i]->edges[j] << " " ;
            }
            out << std::endl ;
        }
    }

    static vtx_data** graphite_to_ace(const SparseMatrix& M) {

        ogf_assert(!M.has_symmetric_storage()) ;
        ogf_assert(M.rows_are_stored()) ;

        bool using_edge_weights = true ;
        vtx_data** graph = new vtx_data*[M.m() + 1] ;
        vtx_data* links = new vtx_data[M.m()] ;
        graph[0] = nil ;
        for(unsigned int i=0; i<(unsigned int)M.m(); i++) {
            graph[i+1] = &links[i] ;
        }
        int size = M.nnz() ;
        int* edges = new int[size] ;
        float* eweights = nil ;
        if(using_edge_weights) {
            eweights = new float[size] ;
        }

        int* edge_ptr = edges ;
        float* eweight_ptr = eweights ;

        for(unsigned int i=0; i<(unsigned int)M.m(); i++) {
            const SparseMatrix::Row& Ri = M.row(i) ;
            graph[i+1]->vwgt = 1 ;
            graph[i+1]->nedges = Ri.nb_coeffs() ;
            graph[i+1]->edges = edge_ptr ;
            
            // Self-edge first
            *edge_ptr = i+1 ;
            edge_ptr++ ;

            if(using_edge_weights) {
                graph[i+1]->ewgts = eweight_ptr ;
                *eweight_ptr = (float) M.diag(i) ;
                eweight_ptr++ ;
            } else {
                graph[i+1]->ewgts = nil ;
            }

            for(unsigned int jj=0; jj<(unsigned int)Ri.nb_coeffs(); jj++) {
                unsigned int j = Ri.coeff(jj).index ;
                double aij = Ri.coeff(jj).a ;
                if(j != i) {
                    if(using_edge_weights) {
                        *eweight_ptr = (float) aij ;
                        eweight_ptr++ ;
                    }
                    *edge_ptr = j+1 ;
                    edge_ptr++ ;
                }
            }
        }
        return graph ;
    }

    EigenSolver_ACE::EigenSolver_ACE() {
        nb_eigens_ = 0 ;
        eigen_vectors_ = nil ;
        eigen_values_ = nil ;
    }

    EigenSolver_ACE::~EigenSolver_ACE() {
        cleanup() ;
    }
        
    bool EigenSolver_ACE::solve() {
        const SparseMatrix& M = *A_ ;
        cleanup() ;
        eigen_values_ = new double[nb_eigens_ + 1] ;
        eigen_vectors_ = new double*[nb_eigens_ + 1];
        eigen_vectors_[0] = nil ;
        for(int i=0; i<nb_eigens_; i++) {
            eigen_vectors_[i+1] = new double[M.n() + 1] ;
        }
        vtx_data** graph = graphite_to_ace(M) ;
//        print_graph(graph, M.m()) ;
        unsigned int nedges = (M.nnz() - M.n())/2 ;
        int min_size_to_coarsen = 50 ;
        double* vmasses = new double[M.n() + 1] ;
        for(int i=0; i<M.n(); i++) {
            vmasses[i+1] = 1.0 ;
        }

        coarseningMethod = contraction ;

/*
        std::cerr << "calling amg_eigen_computation(" 
                  << graph << "," << M.n() << "," << nedges << "," << eigen_vectors_ << "," 
                  << nb_eigens_ << "," << min_size_to_coarsen << "," << 0 << "," << vmasses << "," 
                  << levels_between_refinement << ")" << std::endl ;
*/

        amg_eigen_computation(
            graph, M.n(),  nedges, 
            eigen_vectors_, nb_eigens_, 
            min_size_to_coarsen,
            0, vmasses, 
            levels_between_refinement
        ) ;

        std::cerr << "returned from ACE" << std::endl ;
        delete[] vmasses ;
        return true ;
    }
    
    double* EigenSolver_ACE::get_eigen_vector(int index) {
        ogf_assert(index > 0) ;
        index-- ;
        ogf_assert(index < nb_eigens_) ;
        return eigen_vectors_[index+1] ;
    }

    void EigenSolver_ACE::get_eigen_vector(int index, Vector& v) {
        if(index == 0) {
            double s = 1.0 / ::sqrt(double(v.size())) ;
            for(unsigned int i=0; i<v.size(); i++) {
                v[i] = s ;
            }
            return ;
        }
        index-- ;
        ogf_assert(index < nb_eigens_) ;
        for(unsigned int i=0; i<v.size(); i++) {
            v[i] = eigen_vectors_[index+1][i+1] ;
        }
    }
    
    double EigenSolver_ACE::get_eigen_value(int index) {
        if(index == 0) { return 0.0 ; }
        index-- ;
        ogf_assert(index < nb_eigens_) ;
        return eigen_values_[index+1] ;
    }
        

    void EigenSolver_ACE::cleanup() {
        if(eigen_vectors_ != nil) {
            for(int i=0; i<nb_eigens_; i++) {
                delete[] eigen_vectors_[i+1] ;
            }
        }
        delete[] eigen_vectors_ ;
        delete[] eigen_values_ ;
        eigen_vectors_ = nil ;
        eigen_values_ = nil ;
    }

}

