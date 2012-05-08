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

#ifndef __OGF_MATH_NUMERIC_BICGSTAB__
#define __OGF_MATH_NUMERIC_BICGSTAB__

#include <OGF/math/common/common.h>
#include <OGF/math/numeric/convergence_logger.h>
#include <OGF/math/numeric/blas.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/debug/progress.h>
#include <OGF/basic/debug/assert.h>

namespace OGF {

//_________________________________________________________

    /**
     *  The BICGSTAB algorithm without preconditioner:
     *  Ashby, Manteuffel, Saylor
     *     A taxononmy for conjugate gradient methods
     *     SIAM J Numer Anal 27, 1542-1568 (1990)
     *
     * This implementation is inspired by the lsolver library,
     * by Christian Badura, available from:
     * http://www.mathematik.uni-freiburg.de
     * /IAM/Research/projectskr/lin_solver/
     *
     * @param N dimension of the problem.
     * @param A generic matrix, a function
     *   mult(const MATRIX& M, const double* x, double* y)
     *   needs to be defined.
     * @param b right hand side of the system.
     * @param x initial value.
     * @param eps threshold for the residual.
     * @param max_iter maximum number of iterations.
     * @param create_log_file if set, creates a gnuplot 
     *    compatible file showing the convergence.
     * @param progress if specified, will be notified during
     *    the iterations. This can be used to update a progress
     *    bar.
     * @param task_name if specified, will be used to construct
     *    the names of the log files. Default is "sls_bicgstab"
     */    


    template< class MATRIX > inline int solve_bicgstab( 
        unsigned N, const MATRIX &A,
        const double *b, double *x, double eps, int max_iter,
        bool create_log_file = false, Progress* progress = nil,
        std::string task_name = "sls_bicgstab",
        bool quiet = false
    ) {
        double *rT  = new double[N];
        double *d   = new double[N];
        double *h   = new double[N];
        double *u   = new double[N];
        double *Ad  = new double[N];
        double *t   = new double[N];
        double *s   = h;
        double rTh, rTAd, rTr, alpha, beta, omega, st, tt;
        int its=0;
        double err=eps*eps*BLAS::ddot(N,b,1,b,1);
        double *r = new double[N];

        ConvergenceLogger logger(task_name, create_log_file, quiet) ;

        mult(A,x,r);
        BLAS::daxpy(N,-1.,b,1,r,1);
        BLAS::dcopy(N,r,1,d,1);
        BLAS::dcopy(N,d,1,h,1);
        BLAS::dcopy(N,h,1,rT,1);
        ogf_assert( BLAS::ddot(N,rT,1,rT,1)>1e-40 );
        rTh=BLAS::ddot(N,rT,1,h,1);
        rTr=BLAS::ddot(N,r,1,r,1);
        while ( rTr>err && its < max_iter) {
            mult(A,d,Ad);
            rTAd=BLAS::ddot(N,rT,1,Ad,1);
            ogf_assert( ::fabs(rTAd)>1e-40 );
            alpha=rTh/rTAd;
            BLAS::daxpy(N,-alpha,Ad,1,r,1);
            BLAS::dcopy(N,h,1,s,1);
            BLAS::daxpy(N,-alpha,Ad,1,s,1);
            mult(A,s,t);
            BLAS::daxpy(N,1.,t,1,u,1);
            BLAS::dscal(N,alpha,u,1);
            st=BLAS::ddot(N,s,1,t,1);
            tt=BLAS::ddot(N,t,1,t,1);
            if ( fabs(st)<1e-40 || fabs(tt)<1e-40 ) {
                omega = 0.;
            } else {
                omega = st/tt;
            }
            BLAS::daxpy(N,-omega,t,1,r,1);
            BLAS::daxpy(N,-alpha,d,1,x,1);
            BLAS::daxpy(N,-omega,s,1,x,1);
            BLAS::dcopy(N,s,1,h,1);
            BLAS::daxpy(N,-omega,t,1,h,1);
            beta=(alpha/omega)/rTh; rTh=BLAS::ddot(N,rT,1,h,1); beta*=rTh;
            BLAS::dscal(N,beta,d,1);
            BLAS::daxpy(N,1.,h,1,d,1);
            BLAS::daxpy(N,-beta*omega,Ad,1,d,1);
            rTr=BLAS::ddot(N,r,1,r,1);
            if ( create_log_file ) {
                logger.log(its, ::sqrt(rTr)) ;
            }
            if(progress != nil) {
                progress-> notify(its) ;
            }
            ++its;
        }

        delete[] r;
        delete[] rT;
        delete[] d;
        delete[] h;
        delete[] u;
        delete[] Ad;
        delete[] t;

        if(progress != nil) {
            progress-> notify(0) ;
        }

        return its;
    }

    /**
     *  The BICGSTAB with preconditioner:
     *  Ashby, Manteuffel, Saylor
     *     A taxononmy for conjugate gradient methods
     *     SIAM J Numer Anal 27, 1542-1568 (1990)
     *
     * This implementation is inspired by the lsolver library,
     * by Christian Badura, available from:
     * http://www.mathematik.uni-freiburg.de
     * /IAM/Research/projectskr/lin_solver/
     *
     * @param N dimension of the problem.
     * @param A generic matrix, a function
     *   mult(const MATRIX& M, const double* x, double* y)
     *   needs to be defined.
     * @param C preconditioner, a function
     *   mult(const PC_MATRIX& C, const double* x, double* y)
     *   needs to be defined.
     * @param b right hand side of the system.
     * @param x initial value.
     * @param eps threshold for the residual.
     * @param max_iter maximum number of iterations.
     * @param create_log_file if set, creates a gnuplot 
     *    compatible file showing the convergence.
     * @param progress if specified, will be notified during
     *    the iterations. This can be used to update a progress
     *    bar.
     * @param task_name if specified, will be used to construct
     *    the names of the log files. Default is "sls_bicgstab"
     */    

    template< class MATRIX, class PC_MATRIX > inline
    int solve_preconditioned_bicgstab( 
        unsigned N, const MATRIX &A, const PC_MATRIX &C,
        const double *b, double *x, double eps, int max_iter,
        bool create_log_file = false, Progress* progress = nil,
        std::string task_name = "sls_pre_bicgstab",
        bool quiet = false
    ) {
        double *rT  = new double[N];
        double *d   = new double[N];
        double *h   = new double[N];
        double *u   = new double[N];
        double *Sd  = new double[N];
        double *t   = new double[N];
        double *aux = new double[N];
        double *s   = h;
        double rTh, rTSd, rTr, alpha, beta, omega, st, tt;
        int its=0;
        double err=eps*eps*BLAS::ddot(N,b,1,b,1);
        double *r = new double[N];

        ConvergenceLogger logger(task_name, create_log_file, quiet) ;

        mult(A,x,r);
        BLAS::daxpy(N,-1.,b,1,r,1);
        mult(C,r,d);
        BLAS::dcopy(N,d,1,h,1);
        BLAS::dcopy(N,h,1,rT,1);
        ogf_assert( BLAS::ddot(N,rT,1,rT,1)>1e-40 );
        rTh=BLAS::ddot(N,rT,1,h,1);
        rTr=BLAS::ddot(N,r,1,r,1);
        while ( rTr>err && its < max_iter) {
            mult(A,d,aux);
            mult(C,aux,Sd);
            rTSd=BLAS::ddot(N,rT,1,Sd,1);
            ogf_assert( ::fabs(rTSd)>1e-40 );
            alpha=rTh/rTSd;
            BLAS::daxpy(N,-alpha,aux,1,r,1);
            BLAS::dcopy(N,h,1,s,1);
            BLAS::daxpy(N,-alpha,Sd,1,s,1);
            mult(A,s,aux);
            mult(C,aux,t);
            BLAS::daxpy(N,1.,t,1,u,1);
            BLAS::dscal(N,alpha,u,1);
            st=BLAS::ddot(N,s,1,t,1);
            tt=BLAS::ddot(N,t,1,t,1);
            if ( fabs(st)<1e-40 || fabs(tt)<1e-40 ) {
                omega = 0.;
            } else {
                omega = st/tt;
            }
            BLAS::daxpy(N,-omega,aux,1,r,1);
            BLAS::daxpy(N,-alpha,d,1,x,1);
            BLAS::daxpy(N,-omega,s,1,x,1);
            BLAS::dcopy(N,s,1,h,1);
            BLAS::daxpy(N,-omega,t,1,h,1);
            beta=(alpha/omega)/rTh; rTh=BLAS::ddot(N,rT,1,h,1); beta*=rTh;
            BLAS::dscal(N,beta,d,1);
            BLAS::daxpy(N,1.,h,1,d,1);
            BLAS::daxpy(N,-beta*omega,Sd,1,d,1);
            rTr=BLAS::ddot(N,r,1,r,1);
            if ( create_log_file ) {
                logger.log(its, ::sqrt(rTr)) ;
            }
            if(progress != nil) {
                progress-> notify(its) ;
            }
            ++its;
        }
        delete[] r;
        delete[] rT;
        delete[] d;
        delete[] h;
        delete[] u;
        delete[] Sd;
        delete[] t;
        delete[] aux;
        if(progress != nil) {
            progress-> notify(0) ;
        }
        return its;
    }
}

#endif 
