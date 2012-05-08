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
 
#include <OGF/math/numeric/sparse_matrix_crs.h>

#include <deque>
#include <algorithm>

namespace OGF {

    void Permutation::invert(Permutation& result) const {
        // Rem: this is coherent with the convention 
        // size()==0 <=> this is the identity permutation
        result.allocate(size()) ;
        for(unsigned int i=0; i<size(); i++) {
            result[(*this)[i]] = i ;
        }
    }

    //_______________________________________________________________________________________

    void SparseMatrixPatternCRS::expand_symmetric_pattern(SparseMatrixPatternCRS& rhs) const {
        rhs.N = N ;
        rhs.symmetric_storage = false ; 

        // If matrix pattern does not use symmetric storage, just
        // copy the pattern.
        if(!symmetric_storage) {
            std::cerr << "SparseMatrixPatternCRS::expand_symmetric_pattern()" 
                      << " called for matrix without symmetric storage"
                      << std::endl ;
            rhs.colind.allocate(colind.size()) ;
            for(unsigned int i=0; i<colind.size(); i++) {
                rhs.colind[i] = colind[i] ;
            }
            rhs.rowptr.allocate(rowptr.size()) ;
            for(unsigned int i=0; i<rowptr.size(); i++) {
                rhs.rowptr[i] = rowptr[i] ;
            }
            return ;
        }

        // Step1: compute row sizes and total number of non-zero coefficients
        unsigned int new_nnz = 0 ;
        IndexArray row_size(m()) ;
        row_size.set_all(0) ;
        for(unsigned int i=0; i<m(); i++) {
            row_size[i] += rowptr[i+1] - rowptr[i] ;
            new_nnz += rowptr[i+1] - rowptr[i] ;
            for(unsigned int jj= rowptr[i] ; jj < rowptr[i+1]; jj++) {
                unsigned int j = colind[jj] ;
                // Do not count diagonal elements twice
                if(j != i) { row_size[j]++ ; new_nnz++ ; }
            }
        }

        // Step 2: allocate target matrix pattern and compute row pointers.
        rhs.rowptr.allocate(m() + 1) ;
        rhs.colind.allocate(new_nnz) ;
        rhs.rowptr[0] = 0 ;
        for(unsigned int i=0; i<m(); i++) {
            rhs.rowptr[i+1] = rhs.rowptr[i] + row_size[i] ;
        }

        // Step 3: compute column indices.
        row_size.set_all(0) ;
        for(unsigned int i=0; i<m(); i++) {
            for(unsigned int jj= rowptr[i] ; jj < rowptr[i+1]; jj++) {
                unsigned int j = colind[jj] ;
                rhs.colind[  rhs.rowptr[i] + row_size[i] ] = j ;
                row_size[i]++ ;
                if(j != i) { 
                    rhs.colind[ rhs.rowptr[j] + row_size[j] ] = i ;
                    row_size[j]++ ; 
                }
            }
        }
    }

    unsigned int SparseMatrixPatternCRS::bandwidth() const {
        unsigned int result = 0 ;
        for(unsigned int i=0; i<m(); i++) {
            for(unsigned int jj=rowptr[i]; jj < rowptr[i+1]; jj++) {
                unsigned int j = colind[jj] ;
                result = ogf_max(result, (unsigned int)ogf_abs(int(i) - int(j))) ;
            }
        }
        return result ;
    }
    
    int SparseMatrixPatternCRS::find_furthest_node(unsigned int x, IndexArray& dist) {

        std::deque<unsigned int> Q ;
        for(unsigned int i=0; i<m(); i++) {  dist[i] = NO_INDEX ; }
        unsigned int r = x ; unsigned int ex = 0 ; unsigned int dr = row_nnz(r) ;
            
        dist[x] = 0 ;
        Q.push_back(x) ;
        
        while(!Q.empty()) {
            unsigned int cur = Q.front() ;
            Q.pop_front() ;
            
            if(dist[cur] > ex) {
                r = cur ; dr = row_nnz(r) ; ex = dist[r] ;
            } else if(dist[cur] == ex) {
                unsigned int d = row_nnz(cur) ;
                if(d < dr) {   r = cur ; dr = d ;  }
            }
            
            for(unsigned int jj=rowptr[cur]; jj < rowptr[cur+1]; jj++) {
                unsigned int j = colind[jj] ;
                if(dist[j] == NO_INDEX) {
                    dist[j] = dist[cur] + 1 ;
                    Q.push_back(j) ;
                }
            }
        }
             
        unsigned int er = 0 ;
        for(unsigned int i=0; i<m(); i++) {  dist[i] = NO_INDEX ; }
        dist[r] = 0 ;
        Q.push_back(r) ;
        while(!Q.empty()) {
            unsigned int cur = Q.front() ;
            Q.pop_front() ;
            for(unsigned int jj=rowptr[cur]; jj < rowptr[cur+1]; jj++) {
                unsigned int j = colind[jj] ;
                if(dist[j] == NO_INDEX) {
                    dist[j] = dist[cur] + 1 ;
                    er = ogf_max(er, dist[j]) ;
                    Q.push_back(j) ;
                }
            }
        }
        std::cerr << "   ex = " << ex << "   er = " << er << std::endl ;
        return (er > ex) ? int(r) : -1 ;
    }

    class CompareDegree {
    public:
        CompareDegree(SparseMatrixPatternCRS* M) : matrix_(*M) {  }
        bool operator()(unsigned int i, unsigned int j) const {
            return matrix_.row_nnz(i) < matrix_.row_nnz(j) ;
        }
    private:
        SparseMatrixPatternCRS& matrix_ ;
    } ;

    void SparseMatrixPatternCRS::compute_RCMK_ordering(Permutation& result) {
        ogf_assert(m() == n()) ;
        
        if(symmetric_storage) {
            SparseMatrixPatternCRS temp ;
            expand_symmetric_pattern(temp) ;
            temp.compute_RCMK_ordering(result) ;
            return ;
        }

        unsigned int cur_id = n()-1 ;
        result.allocate(n()) ;
        result.set_all(NO_INDEX) ;
        Permutation dist ;
        dist.allocate(n()) ;

        // Note: the graph of the matrix may
        // have multiple connected component,
        // therefore we need this outer loop.
        for(unsigned int x=0; x<n(); x++) {
            if(result[x] != NO_INDEX) { continue ; }

            std::deque<unsigned int> Q ;

            int r ;
            while((r = find_furthest_node(x, dist)) != -1) {  x = r ;  }

            unsigned int seed = x ;

            Q.push_back(seed) ;
            result[seed] = cur_id-- ;

            while(!Q.empty()) {
                unsigned int cur = Q.front() ;
                Q.pop_front() ;
                
                std::vector<unsigned int> neighbors ;
                for(unsigned int jj=rowptr[cur]; jj < rowptr[cur+1]; jj++) {
                    unsigned int j = colind[jj] ;
                    if(result[j] == NO_INDEX) {
                        neighbors.push_back(j) ;
                    }
                }
                
                std::sort(neighbors.begin(), neighbors.end(), CompareDegree(this)) ;
                for(unsigned int k=0; k<neighbors.size(); k++) {
                    unsigned int n = neighbors[k] ;
                    result[n] = cur_id-- ; 
                    Q.push_back(n) ;
                }
            }
        }
    }

//--------------------------------------------------------------------------------------------------------------------------

}

