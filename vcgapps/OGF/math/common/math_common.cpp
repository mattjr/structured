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
 

#include <OGF/math/common/common.h>
#include <OGF/math/types/math_library.h>

#include <OGF/math/numeric/quick_blas.h>
#include <OGF/math/numeric/sparse_matrix_sse.h>

#include <OGF/math/numeric/system_solver.h>
#include <OGF/math/numeric/system_solver_taucs.h>
#include <OGF/math/numeric/system_solver_cholmod.h>
#include <OGF/math/numeric/system_solver_fastcg.h>

#include <OGF/math/numeric/eigen_solver_arpack.h>
#include <OGF/math/numeric/eigen_solver_ace.h>

#include <OGF/basic/debug/logger.h>
#include <OGF/basic/modules/module.h>
#include <OGF/basic/attributes/attribute_serializer.h>
#include <OGF/basic/attributes/attribute_interpolator.h>

#include <OGF/math/geometry/types.h>
#include <OGF/math/geometry/complex.h>
#include <OGF/math/geometry/quaternion.h>
#include <OGF/math/geometry/shewchuk.h>

#include <OGF/math/functions/bivariate.h>
#include <OGF/math/functions/pn.h>

#include <OGF/math/attributes/point_attribute_interpolator.h>

namespace OGF {
    
/****************************************************************/
    
    void math_libinit::initialize() {
        Logger::out("Init") << "Initializing library \'" 
                            << "math" << "\'" << std::endl ; 
        //_____________________________________________________________

        Module* module_info = new Module ;
        module_info->set_name("math") ;
        module_info->set_vendor("OGF") ;
        module_info->set_version("1.0-a4") ;
        module_info->set_info("Mathematical objects and numerical solvers") ;
        Module::bind_module("math", module_info) ;

        MathLibrary::initialize() ;
        ogf_declare_system_solver<SystemSolver_CG>("CG") ;
        ogf_declare_system_solver<SystemSolver_BICGSTAB>("BICGSTAB") ;
        ogf_declare_system_solver<SystemSolver_GMRES>("GMRES") ;
        ogf_declare_system_solver<SystemSolver_SUPERLU>("SUPERLU") ;
        ogf_declare_system_solver<SystemSolverTaucs_LLT>("TAUCS_LLT") ;
        ogf_declare_system_solver<SystemSolverTaucs_LU>("TAUCS_LU") ;
        ogf_declare_system_solver<SystemSolverTaucs_LDLT>("TAUCS_LDLT") ;
        ogf_declare_system_solver<SystemSolverCholmod>("CHOLMOD") ;
        ogf_declare_system_solver<SystemSolver_FASTCG>("FASTCG") ;

        ogf_declare_eigen_solver<EigenSolver_ARPACK>("ARPACK") ;        
        ogf_declare_eigen_solver<EigenSolver_ACE>("ACE") ;        

        // Initializes Shewchuck's exact predicates.
        exactinit();

        QuickBLAS::initialize() ;
        sparse_matrix_SSE2_initialize() ;        
        
        ogf_register_attribute_type<Point2d>("Point2d") ;
        ogf_register_attribute_type<Point3d>("Point3d") ;
        ogf_register_attribute_type<Vector2d>("Vector2d") ;
        ogf_register_attribute_type<Vector3d>("Vector3d") ;
        ogf_register_attribute_type<Complex>("Complex") ;

        ogf_register_numeric_attribute_interpolator<Complex>() ;
        ogf_register_numeric_attribute_interpolator<Vector2d>() ;
        ogf_register_numeric_attribute_interpolator<Vector3d>() ;
        ogf_register_attribute_interpolator<Point2d>(new Point2dAttributeInterpolator) ;
        ogf_register_attribute_interpolator<Point3d>(new Point3dAttributeInterpolator) ;

        ogf_register_attribute_type<BivariateLinearFunction>("BivariateLinearFuncion") ;
        ogf_register_attribute_type<BivariateQuadraticFunction>("BivariateQuadraticFuncion") ;
        ogf_register_attribute_type<BivariateCubicFunction>("BivariateCubicFuncion") ;

        ogf_register_attribute_type<P0BivariateFunction>("P0BivariateFunction") ;
        ogf_register_attribute_type<P1BivariateFunction>("P1BivariateFunction") ;
        ogf_register_attribute_type<P2BivariateFunction>("P2BivariateFunction") ;
        ogf_register_attribute_type<P3BivariateFunction>("P3BivariateFunction") ;

        Logger::out("Init") << "Initialized library \'" 
                            << "math" << "\'" << std::endl ; 
    }
    
    void math_libinit::terminate() {
        Logger::out("Init") << "Terminating library \'" 
                            << "math" << "\'" << std::endl ; 

        //_____________________________________________________________

        MathLibrary::terminate() ;

        //_____________________________________________________________

        Module::unbind_module("math") ;
        
        Logger::out("Init") << "Terminated library \'" 
                            << "math" << "\'" << std::endl ; 
    }
    
// You should not need to modify this file below that point.
    
/****************************************************************/
    
    math_libinit::math_libinit() {
        increment_users() ;
    }

    math_libinit::~math_libinit() {
        decrement_users() ;
    }
    
    void math_libinit::increment_users() {
        // Note that count_ is incremented before calling
        // initialize, else it would still be equal to
        // zero at module initialization time, which 
        // may cause duplicate initialization of libraries.
        count_++ ;
        if(count_ == 1) {
            initialize() ;
        }
    }
    
    void math_libinit::decrement_users() {
        count_-- ;
        if(count_ == 0) {
            terminate() ;
        }
    }
    
    int math_libinit::count_ = 0 ;
    
}

// The initialization and termination functions
// are also declared using C linkage in order to 
// enable dynamic linking of modules.

extern "C" void MATH_API OGF_math_initialize() {
    OGF::math_libinit::increment_users() ;
}

extern "C" void MATH_API OGF_math_terminate() {
    OGF::math_libinit::decrement_users() ;
}


