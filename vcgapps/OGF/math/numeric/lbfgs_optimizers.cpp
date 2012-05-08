/*
 *  OGF/Graphite: Geometry and Graphics Programming Library + Utilities
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


#include <OGF/math/numeric/lbfgs_optimizers.h>

#include <OGF/math/third_party/LBFGSB/lbfgs.h>
#include <OGF/math/third_party/LBFGSB/lbfgsb.h>
#include <OGF/math/third_party/HLBFGS/HLBFGS.h>
#include <iostream>




//************* for LBFGS ***********

void funcgrad(ap::real_1d_array& x, double& f, ap::real_1d_array& g);

void lbfgsnewiteration(
    const ap::real_1d_array& x,
    double f,
    const ap::real_1d_array& g
);

//************* for LBFGSB ***********
// uses also funcgrad()

void lbfgsbnewiteration(
    const ap::real_1d_array& x,
    double f,
    const ap::real_1d_array& g
);

//************* for HLBFGS *************

void evalfunc(int N, double* x, double *prev_x, double* f, double* g) ;

void evalfunction_h(int N, double* x, double* prev_x, double* f, double* g, HESSIAN_MATRIX& m_hessian);

void hlbfgsnewiteration(int iter, int call_iter, double *x, double* f, double *g,  double* gnorm);

namespace OGF {
    
    class GlobalOptimizerConfig {
    public:
        
        static void set_funcgrad_callback(funcgrad_fp fp){
            funcgrad_callback = fp;
        }
        
        static void set_newiteration_callback(newiteration_fp fp){
            newiteration_callback = fp;
        }
        
        static void set_evalhessian_callback(evalhessian_fp fp){
            evalhessian_callback = fp;
        }
        
        static void newiteration( const double* x, double f, const double* g, double gnorm ){
            (*newiteration_callback)(N, x, f, g, gnorm);
        }
        
        static void funcgrad(double* x, double& f, double* g ){
            (*funcgrad_callback)(N, x, f, g);
        }
        
        static void evalhessian(int N, double *x, double &f, double *g, HESSIAN_MATRIX& hessian){
            (*evalhessian_callback)(N, x, f, g, hessian);
        }
        
        static void set_N(int n){
            N = n;
        }
        
    private:
        static funcgrad_fp funcgrad_callback;
        static newiteration_fp newiteration_callback;
        static evalhessian_fp evalhessian_callback;
        static int N;
    };

    newiteration_fp GlobalOptimizerConfig::newiteration_callback = nil;
    funcgrad_fp	    GlobalOptimizerConfig::funcgrad_callback	 = nil;
    evalhessian_fp  GlobalOptimizerConfig::evalhessian_callback  = nil;
    int GlobalOptimizerConfig::N = 0;
    
    void show_bfgs_message(int info) {
        switch(info)  {
        case -1: std::cout <<"wrong parameters were specified " << std::endl;
            break;
        case 0: std::cout <<"interrupted by user " << std::endl;
            break;
        case 1:  std::cout <<"relative function decreasing is less or equal to EpsF " << std::endl;
            break;
        case 2:  std::cout <<" step is less or equal to EpsX" << std::endl;
            break;
        case 4:  std::cout <<"gradient norm is less or equal to EpsG, " << std::endl;
            break;
        case 5:  std::cout <<"number of iterations exceeds MaxIts. " << std::endl;
            break;
        default:
            break;
        }
    }



    void LBFGSOptimizer::optimize( double* x ) {
        ogf_assert( newiteration_callback != nil ) ;
        ogf_assert( funcgrad_callback != nil ) ;
        ogf_assert( n>0 ) ;
        ogf_assert( x!=nil ) ;
        
        
        GlobalOptimizerConfig::set_newiteration_callback(newiteration_callback);
        GlobalOptimizerConfig::set_funcgrad_callback(funcgrad_callback);
        GlobalOptimizerConfig::set_N(n);
        
        int info;
        ap::real_1d_array x_ap;
        x_ap.setcontent(1, n, x); //duplicates x
        
        lbfgsminimize(n,m,x_ap,epsg,epsf,epsx,max_iter,info);
        
        // copy back the solution
        memcpy(x, x_ap.getcontent(), n*sizeof(double));
        
        if(verbose){
            show_bfgs_message(info);
        }
    }


    void LBFGSBOptimizer::set_nbd(int nb, int* rhs){
        ogf_assert (nb>0 && rhs != nil);
        delete [] nbd;
        nbd = new int[nb] ;
        memcpy(nbd,rhs,nb*sizeof(int));
    }
	
    void LBFGSBOptimizer::set_l(int nb, double* rhs){
        ogf_assert (nb>0 && rhs != nil);
        delete [] l;
        l = new double[nb] ;
        memcpy(l,rhs,nb*sizeof(double));
    }
    
    void LBFGSBOptimizer::set_u(int nb, double* rhs){
        ogf_assert (nb>0 && rhs != nil);
        delete [] u;
        u = new double[nb] ;
        memcpy(u,rhs,nb*sizeof(double));
    }
		

    void LBFGSBOptimizer::optimize( double* x ) {
        ogf_assert( newiteration_callback != nil ) ;
        ogf_assert( funcgrad_callback != nil ) ;
        ogf_assert( n>0 ) ;
        ogf_assert( nbd != nil ) ;
        ogf_assert( l != nil ) ;
        ogf_assert( u != nil ) ;
        ogf_assert( x != nil ) ;
        
        GlobalOptimizerConfig::set_newiteration_callback(newiteration_callback);
        GlobalOptimizerConfig::set_funcgrad_callback(funcgrad_callback);
        GlobalOptimizerConfig::set_N(n);
        
        int info;
        ap::real_1d_array x_ap;
        ap::integer_1d_array nbd_ap;
        ap::real_1d_array l_ap, u_ap;
        

        x_ap.setcontent(1, n, x);
        nbd_ap.setcontent(1, n, nbd);
        l_ap.setcontent(1, n, l);
        u_ap.setcontent(1, n, u);
        
        lbfgsbminimize(n, m, x_ap, epsg, epsf, epsx, max_iter, nbd_ap, l_ap, u_ap, info);
        
        memcpy(x, x_ap.getcontent(), n*sizeof(double));
        
        if(verbose){
            show_bfgs_message(info);
        }
    }

    HLBFGSOptimizer::HLBFGSOptimizer() {
        b_m1qn3 = false; b_cg = false;
    }

    void HLBFGSOptimizer::optimize( double* x ) {
        ogf_assert( newiteration_callback != nil ) ;
        ogf_assert( funcgrad_callback != nil ) ;
        ogf_assert( n>0 ) ;
        ogf_assert( x != nil ) ;
        
        GlobalOptimizerConfig::set_newiteration_callback(newiteration_callback);
        GlobalOptimizerConfig::set_funcgrad_callback(funcgrad_callback);
        GlobalOptimizerConfig::set_N(n);
        
        double parameter[20] ;
        int hlbfgs_info[20] ;
        
        //initialize parameters and infos
        INIT_HLBFGS(parameter, hlbfgs_info) ; 
        hlbfgs_info[3] = b_m1qn3?1:0; //determines whether we use m1qn3
        hlbfgs_info[4] = max_iter ; // max iterations
        hlbfgs_info[10] = b_cg?1:0; //determines whether we use cg
        parameter[5] = 0; // disabled
        parameter[6] = epsg;
        HLBFGS(
            n,
            m,
            x, 
            evalfunc,
            0, 
            HLBFGS_UPDATE_Hessian,
            hlbfgsnewiteration,
            parameter,
            hlbfgs_info
        );
    }

    HLBFGS_HessOptimizer::HLBFGS_HessOptimizer() {
        b_m1qn3 = false; b_cg = false; T = 0;
    }

    void HLBFGS_HessOptimizer::optimize( double* x ) {
        ogf_assert( newiteration_callback != nil ) ;
        ogf_assert( funcgrad_callback != nil ) ;
        ogf_assert( evalhessian_callback != nil);
        ogf_assert( n>0 ) ;
        ogf_assert( x != nil ) ;
        
        GlobalOptimizerConfig::set_newiteration_callback(newiteration_callback);
        GlobalOptimizerConfig::set_funcgrad_callback(funcgrad_callback);
        GlobalOptimizerConfig::set_evalhessian_callback(evalhessian_callback);
        GlobalOptimizerConfig::set_N(n);

        double parameter[20] ;
        int hlbfgs_info[20] ;

        //initialize parameters and infos
        INIT_HLBFGS(parameter, hlbfgs_info) ; 
        hlbfgs_info[4] = max_iter ; // max iterations
        hlbfgs_info[6] = T ; // update interval of hessian
        hlbfgs_info[7] = 1 ;  // 0: without hessian, 1: with accurate hessian

        HLBFGS(
            n,
            m,
            x, 
            evalfunc,
            evalfunction_h, 
            HLBFGS_UPDATE_Hessian,
            hlbfgsnewiteration,
            parameter,
            hlbfgs_info
        );
    }
}

//-------- global functions for optimizers ----------


//----------LBFGS -------------

void funcgrad(ap::real_1d_array& x, double& f, ap::real_1d_array& g) {
    OGF::GlobalOptimizerConfig::funcgrad( x.getcontent(), f, g.getcontent() );
}


void lbfgsnewiteration(
    const ap::real_1d_array& x,
    double f,
    const ap::real_1d_array& g
){
    double gnorm2 = 0.0 ;
    for(int i=g.getlowbound(); i<=g.gethighbound(); i++) {
        gnorm2 += g(i)*g(i) ;
    }
    gnorm2 = ::sqrt(gnorm2) ;
    OGF::GlobalOptimizerConfig::newiteration( x.getcontent(), f, g.getcontent(), gnorm2);
}

void lbfgsbnewiteration(
    const ap::real_1d_array& x,
    double f,
    const ap::real_1d_array& g
){
    double gnorm2 = 0.0 ;
    for(int i=g.getlowbound(); i<=g.gethighbound(); i++) {
        gnorm2 += g(i)*g(i) ;
    }
    gnorm2 = ::sqrt(gnorm2) ;
    OGF::GlobalOptimizerConfig::newiteration( x.getcontent(), f, g.getcontent(), gnorm2);
}

//--------- HLBFGS ------

void evalfunc(int N, double* x, double *prev_x, double* f, double* g){
    OGF::GlobalOptimizerConfig::funcgrad(x, *f, g);
}

void hlbfgsnewiteration(int iter, int call_iter, double *x, double* f, double *g,  double* gnorm){
    OGF::GlobalOptimizerConfig::newiteration(x, *f, g, *gnorm);
}

void evalfunction_h(int N, double* x, double* prev_x, double* f, double* g, HESSIAN_MATRIX& m_hessian){
    OGF::GlobalOptimizerConfig::evalhessian(N, x, *f, g, m_hessian);
}

