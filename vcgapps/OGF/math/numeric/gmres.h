/*
 *  GXML/Graphite: Geometry and Graphics Programming Library + Utilities
 *  Copyright (C) 2000 Bruno Levy
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
 *  Contact: Bruno Levy
 *
 *     levy@loria.fr
 *
 *     ISA Project
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs. 
 */
 
#ifndef __OGF_MATH_NUMERIC_GMRES__
#define __OGF_MATH_NUMERIC_GMRES__

#include <OGF/math/common/common.h>
#include <OGF/math/numeric/convergence_logger.h>
#include <OGF/math/numeric/blas.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/debug/progress.h>

namespace OGF {

//_________________________________________________________

    template< class Matrix > inline int
    solve_gmres( 
        int m, int n, const Matrix &A, const double *b, 
        double *x, double eps, int max_iter, 
        bool create_log_file = false,
        Progress* progress = nil,
        std::string task_name = "sls_gmres",
        bool quiet = false
    ) {
        if ( n<=0 ) {
            return -1 ;
        }

        ConvergenceLogger logger(task_name, create_log_file, quiet) ;

        typedef double *doubleP;
        double *V  = new double[n*(m+1)];
        double *U  = new double[m*(m+1)/2];
        double *r  = new double[n];
        double *y  = new double[m+1];
        double *c  = new double[m];
        double *s  = new double[m];
        double **v = new doubleP[m+1];
        for ( int i=0; i<=m; ++i ) v[i]=V+i*n;
        int its=-1;
        {
            double beta, h, rd, dd, nrm2b;
            int j, io, uij, u0j;
            nrm2b=BLAS::dnrm2(n,b,1);
    
            io=0;
            do  { // outer loop
                ++io;
                mult(A,x,r);
                BLAS::daxpy(n,-1.,b,1,r,1);
                beta=BLAS::dnrm2(n,r,1);
                BLAS::dcopy(n,r,1,v[0],1);
                BLAS::dscal(n,1./beta,v[0],1);

                y[0]=beta;
                j=0;
                uij=0;
                do { // inner loop: j=0,...,m-1
                    u0j=uij;
                    mult(A,v[j],v[j+1]);
                    BLAS::dgemv(
                        BLAS::Transpose,n,j+1,1.,V,n,v[j+1],1,0.,U+u0j,1
                    );
                    BLAS::dgemv(
                        BLAS::NoTranspose,n,j+1,-1.,V,n,U+u0j,1,1.,v[j+1],1
                    );
                    h=BLAS::dnrm2(n,v[j+1],1);
                    BLAS::dscal(n,1./h,v[j+1],1);
                    for ( int i=0; i<j; ++i ) { // rotiere neue Spalte
                        double tmp = c[i]*U[uij]-s[i]*U[uij+1];
                        U[uij+1]   = s[i]*U[uij]+c[i]*U[uij+1];
                        U[uij]     = tmp;
                        ++uij;
                    }
                    { // berechne neue Rotation
                        rd     = U[uij];
                        dd     = sqrt(rd*rd+h*h);
                        c[j]   = rd/dd;
                        s[j]   = -h/dd;
                        U[uij] = dd;
                        ++uij;
                    }
                    { // rotiere rechte Seite y (vorher: y[j+1]=0)
                        y[j+1] = s[j]*y[j];
                        y[j]   = c[j]*y[j];
                    }
                    ++j;
                    if ( create_log_file ) {
                        std::cout <<"gmres("<<m<<")\t"
                                  <<io<<"\t"<<j<<"\t"<<y[j]
                                  << std::endl;
                    }
                } while ( 
                    j<m && ::fabs(y[j])>=eps*nrm2b 
                ) ;
                { // minimiere bzgl Y
                    BLAS::dtpsv(
                        BLAS::UpperTriangle,
                        BLAS::NoTranspose,
                        BLAS::NotUnitTriangular,
                        j,U,y,1
                    );
                    // correct X
                    BLAS::dgemv(BLAS::NoTranspose,n,j,-1.,V,n,y,1,1.,x,1);
                }
                if(progress != nil) {
                    progress-> notify(m*(io-1)+j) ;
                }
            } while ( ::fabs(y[j])>=eps*nrm2b && (m*(io-1)+j) < max_iter);
                
            // Count the inner iterations
            its = m*(io-1)+j;


        }

        delete[] V;
        delete[] U;
        delete[] r;
        delete[] y;
        delete[] c;
        delete[] s;
        delete[] v;

        if(progress != nil) {
            progress-> notify(0) ;
        }

        return its;
    }
}

#endif
