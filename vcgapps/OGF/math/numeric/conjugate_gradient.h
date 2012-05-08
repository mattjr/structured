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
 
#ifndef __OGF_MATH_NUMERIC_CONJ_GRAD__
#define __OGF_MATH_NUMERIC_CONJ_GRAD__

#include <OGF/math/common/common.h>
#include <OGF/math/numeric/convergence_logger.h>
#include <OGF/math/numeric/blas.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/debug/progress.h>

namespace OGF {

//_________________________________________________________

    /**
     *  The Conjugate Gradient algorithm without preconditioner:
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
     *    the names of the log files. Default is "sls_cghs"
     */

    template< class MATRIX >
    inline int solve_conjugate_gradient(
        int N, 
        const MATRIX &A, 
        const double* b, double* x, 
        double eps, int max_iter, bool create_log_file = false,
        Progress* progress = nil,
        std::string task_name = "sls_cghs",
        bool quiet = false
    ) {
        if ( N==0 ) {
            return -1;
        }

        ConvergenceLogger logger(task_name, create_log_file, quiet) ;
        ProgressLogger progress_logger(max_iter, task_name, quiet) ;

        double *g = new double[N];
        double *r = new double[N];
        double *p = new double[N];
        int its=0;
        double t, tau, sig, rho, gam;
        double bnorm2 = BLAS::ddot(N,b,1,b,1) ; 
        double err=eps*eps*bnorm2 ;
            
        mult(A,x,g);
        BLAS::daxpy(N,-1.,b,1,g,1);
        BLAS::dscal(N,-1.,g,1);
        BLAS::dcopy(N,g,1,r,1);
        while ( BLAS::ddot(N,g,1,g,1)>err && its < max_iter) {
            if(has_nan(x,N)) {
                std::cerr << "Encountered NAN" << std::endl ;
                return 0 ;
            }
            if(progress != nil) {
                progress-> notify(its) ;
            }

            if(progress_logger.is_canceled()) {
                return false ;
            }
            progress_logger.next() ;

            mult(A,r,p);
            rho=BLAS::ddot(N,p,1,p,1);
            sig=BLAS::ddot(N,r,1,p,1);
            tau=BLAS::ddot(N,g,1,r,1);
            t=tau/sig;
            BLAS::daxpy(N,t,r,1,x,1);
            BLAS::daxpy(N,-t,p,1,g,1);
            gam=(t*t*rho-tau)/tau;
            BLAS::dscal(N,gam,r,1);
            BLAS::daxpy(N,1.,g,1,r,1);

            if ( create_log_file ) {
                logger.begin_bias() ;
                double residual = BLAS::dnrm2(N,g,1) ;
                logger.end_bias() ;
                logger.log(its, residual) ;
            }

            ++its;
        }
        logger.log(its, sqrt(BLAS::ddot(N,r,1,r,1)/bnorm2)) ;
        delete[] g;
        delete[] r;
        delete[] p;
        if(progress != nil) {
            progress-> notify(0) ;
        }
        progress_logger.notify(max_iter) ;
        return its;
    } 

    template< class MATRIX >
    inline int solve_conjugate_gradient_quiet(
        int N, 
        const MATRIX &A, 
        const double* b, double* x, 
        double eps, int max_iter
    ) {
        if ( N==0 ) {
            return -1;
        }
        double *g = new double[N];
        double *r = new double[N];
        double *p = new double[N];
        int its=0;
        double t, tau, sig, rho, gam;
        double bnorm2 = BLAS::ddot(N,b,1,b,1) ; 
        double err=eps*eps*bnorm2 ;
        mult(A,x,g);
        BLAS::daxpy(N,-1.,b,1,g,1);
        BLAS::dscal(N,-1.,g,1);
        BLAS::dcopy(N,g,1,r,1);
        while ( BLAS::ddot(N,g,1,g,1)>err && its < max_iter) {
            if(has_nan(x,N)) {
                std::cerr << "Encountered NAN" << std::endl ;
                return 0 ;
            }
            mult(A,r,p);
            rho=BLAS::ddot(N,p,1,p,1);
            sig=BLAS::ddot(N,r,1,p,1);
            tau=BLAS::ddot(N,g,1,r,1);
            t=tau/sig;
            BLAS::daxpy(N,t,r,1,x,1);
            BLAS::daxpy(N,-t,p,1,g,1);
            gam=(t*t*rho-tau)/tau;
            BLAS::dscal(N,gam,r,1);
            BLAS::daxpy(N,1.,g,1,r,1);
            ++its;
        }
        delete[] g;
        delete[] r;
        delete[] p;
        return its;
    } 

    //_______________________________________________________________
    
    /**
     *  the Conjugate Gradient algorithm with preconditioner:
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
     * @param C generic preconditioning matrix, a function
     *   mult(const PC_MATRIX& M, const double* x, double* y)
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
     *    the names of the log files. Default is "sls_pre_cghs"
     */
        
    template< class MATRIX, class PC_MATRIX > inline int
    solve_preconditioned_conjugate_gradient( 
        int N,
        const MATRIX &A, const PC_MATRIX &C,
        const double* b, double* x, double eps, 
        int max_iter, bool create_log_file = false,
        Progress* progress = nil,
        std::string task_name = "sls_pre_cghs",
        bool quiet = false
    ) {


        if ( N==0 ) {
            return 0;
        }

        ConvergenceLogger logger(task_name, create_log_file, quiet) ;
        ProgressLogger progress_logger(max_iter, task_name, quiet) ;

        double* g = nil ;
        if(create_log_file) {
            g = new double[N] ;
        }

        double *r = new double[N];
        double *d = new double[N];
        double *h = new double[N];
        double *Ad = h;
        int its=0;
        double rh, alpha, beta;
        double bnorm2 = BLAS::ddot(N,b,1,b,1) ;
        double err = eps * eps * bnorm2 ;
            
        mult(A,x,r);
        BLAS::daxpy(N,-1.,b,1,r,1);
        mult(C,r,d);
        BLAS::dcopy(N,d,1,h,1);
        rh=BLAS::ddot(N,r,1,h,1);
        while ( BLAS::ddot(N,r,1,r,1)>err && its < max_iter) {
            if(has_nan(x,N)) {
                std::cerr << "Encountered NAN" << std::endl ;
                return 0 ;
            }
            if(progress != nil) {
                progress-> notify(its) ;
            }
            if(progress_logger.is_canceled()) {
                return false ;
            }
            progress_logger.next() ;
            mult(A,d,Ad);
            alpha=rh/BLAS::ddot(N,d,1,Ad,1);
            BLAS::daxpy(N,-alpha,d,1,x,1);
            BLAS::daxpy(N,-alpha,Ad,1,r,1);
            mult(C,r,h);
            beta=1./rh; rh=BLAS::ddot(N,r,1,h,1); beta*=rh;
            BLAS::dscal(N,beta,d,1);
            BLAS::daxpy(N,1.,h,1,d,1);
            if ( create_log_file ) {
                logger.begin_bias() ;
                mult(A,x,g);
                BLAS::daxpy(N,-1.,b,1,g,1);
                double conv = BLAS::dnrm2(N,g,1) ;
                logger.end_bias() ;
                logger.log(its, conv) ;
            }
            ++its;
        }
        logger.log(its, sqrt(BLAS::ddot(N,r,1,r,1)/bnorm2)) ;
        delete[] r;
        delete[] d;
        delete[] h;
        delete[] g ;

        if(progress != nil) {
            progress-> notify(0) ;
        }
        progress_logger.notify(max_iter) ;

        return its;
    }


//_________________________________________________________

}

#endif

