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

#ifndef __OGF_MATH_NUMERIC_OPTIMIZER__
#define __OGF_MATH_NUMERIC_OPTIMIZER__

#include <OGF/math/common/common.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/types/smart_pointer.h>
#include <OGF/basic/types/counted.h>
#include <OGF/basic/debug/assert.h>

class HESSIAN_MATRIX;

namespace OGF {
    
    typedef void (*funcgrad_fp)(int N, double* x, double& f, double* g);
    typedef void (*newiteration_fp)(int N, const double* x, double f, const double* g, double gnorm);
    typedef void (*evalhessian_fp)(int N, double *x, double &f, double *g, HESSIAN_MATRIX& hessian);
    
    class MATH_API Optimizer : public Counted {
    public:
        Optimizer();
        ~Optimizer();
        
        // x should have N elements, where n is set for the optimizer with set_n().
        virtual void optimize(double* x) = 0;
	
        virtual int get_optimizer_id() = 0;
        
        
        void set_N(int N){ n = N; }
        int get_N(){ return n; }
        void set_M(int M){ m = M; }
        int get_M(){ return m; }
        void set_max_iter(int maxiter){ max_iter = maxiter; }
        int get_max_iter(){ return max_iter; }
        
        void set_funcgrad_callback(funcgrad_fp fp){
            funcgrad_callback = fp;
        }
        
        void set_newiteration_callback(newiteration_fp fp){
            newiteration_callback = fp;
        }
        
        void set_evalhessian_callback(evalhessian_fp fp){
            evalhessian_callback = fp;
        }


        void set_epsg(double eg){epsg = eg;}
        void set_epsf(double ef){epsf = ef;}
        void set_epsx(double ex){epsx = ex;}
	
        void set_verbose(bool verb){verbose = verb;}
        
    protected:
        int n; // size of the problem
        int m; // number of  corrections  in  the  BFGS  scheme  of  Hessian approximation  update
        int max_iter; // max iterations
        
        funcgrad_fp funcgrad_callback;
        newiteration_fp newiteration_callback;
        evalhessian_fp evalhessian_callback;
	
        double epsg, epsf, epsx; // error tolerance on x, f and g
        
        bool verbose;
    } ;

    typedef SmartPointer<Optimizer> Optimizer_var ;


} ;


#endif 
