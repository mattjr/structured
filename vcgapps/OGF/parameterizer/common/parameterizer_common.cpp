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
 

#include <OGF/parameterizer/common/common.h>
/*#include <OGF/parameterizer/commands/surface_parameterizer_commands.h>
#include <OGF/parameterizer/commands/surface_periodic_commands.h>
#include <OGF/parameterizer/commands/surface_baking_commands.h>
#include <OGF/parameterizer/commands/line_trimming_commands.h>
*/
#include <OGF/parameterizer/algos/abf.h>
#include <OGF/parameterizer/algos/dpbf.h>
#include <OGF/parameterizer/algos/hlscm.h>

#include <OGF/parameterizer/io/map_serializer_abf.h>
#include <OGF/parameterizer/io/map_serializer_jmd.h>
#include <OGF/parameterizer/io/map_serializer_m.h>

/*#include <OGF/parameterizer/shaders/scalar_topo_surface_shader3d.h>
#include <OGF/parameterizer/shaders/pgp_surface_shader3d.h>

#include <OGF/scene_graph/types/scene_graph_library.h>
*/
#include <OGF/cells/types/cells_library.h>

#include <OGF/basic/debug/logger.h>
#include <OGF/basic/modules/module.h>

#include <OGF/basic/modules/modmgr.h>
#include <OGF/basic/os/file_manager.h>

namespace OGF {

/****************************************************************/
    
    void parameterizer_libinit::initialize() {

/*        gom_package_initialize(parameterizer) ;

        ogf_register_grob_commands<Surface, SurfaceParameterizerCommands>() ;
        ogf_register_grob_commands<Surface, SurfacePeriodicCommands>() ;
        ogf_register_grob_commands<Surface, SurfaceBakingCommands>() ;
        ogf_register_grob_commands<Line, LineTrimmingCommands>() ;
*/
        ogf_declare_map_parameterizer<ABFMapParameterizer>("ABF") ;
        ogf_declare_map_parameterizer<MapParameterizerDPBF>("DPBF") ;
        ogf_declare_map_parameterizer<MapParameterizerHLSCM>("HLSCM") ;

  /*      ogf_declare_map_serializer<MapSerializer_abf>("abf") ;
        ogf_declare_map_serializer<MapSerializer_fl>("fl") ;
        ogf_declare_map_serializer<MapSerializer_fl>("in") ;
        ogf_declare_map_serializer<MapSerializer_JMD>("jmd") ;
        ogf_declare_map_serializer<MapSerializer_m>("m") ;

        Surface::register_map_serializers() ;

        ogf_register_grob_shader3d<Surface, ScalarTopoSurfaceShader3d>() ;
        ogf_register_grob_shader3d<Surface, PGPSurfaceShader3d>() ;
*/
        //_____________________________________________________________

        Module* module_info = new Module ;
        module_info->set_name("parameterizer") ;
        module_info->set_vendor("OGF") ;
        module_info->set_version("1.0-a4") ;
        module_info->set_info("Experimental parameterization algorithms") ;
        Module::bind_module("parameterizer", module_info) ;
        
        Logger::out("Init") << "Initialized library \'" 
                            << "parameterizer" << "\'" << std::endl ; 
    }
    
    void parameterizer_libinit::terminate() {
        Logger::out("Init") << "Terminating library \'" 
                            << "parameterizer" << "\'" << std::endl ; 

        //_____________________________________________________________

        
        //_____________________________________________________________
		
		Module::unbind_module("parameterizer") ;

        Logger::out("Init") << "Terminated library \'" 
                            << "parameterizer" << "\'" << std::endl ; 
    }
    
// You should not need to modify this file below that point.
    
/****************************************************************/
    
    parameterizer_libinit::parameterizer_libinit() {
        increment_users() ;
    }

    parameterizer_libinit::~parameterizer_libinit() {
        decrement_users() ;
    }
    
    void parameterizer_libinit::increment_users() {
        // Note that count_ is incremented before calling
        // initialize, else it would still be equal to
        // zero at module initialization time, which 
        // may cause duplicate initialization of libraries.
        count_++ ;
        if(count_ == 1) {
            initialize() ;
        }
    }
    
    void parameterizer_libinit::decrement_users() {
        count_-- ;
        if(count_ == 0) {
            terminate() ;
        }
    }
    
    int parameterizer_libinit::count_ = 0 ;
    
}

// The initialization and termination functions
// are also declared using C linkage in order to 
// enable dynamic linking of modules.

extern "C" void PARAMETERIZER_API OGF_parameterizer_initialize() {
    OGF::parameterizer_libinit::increment_users() ;
}

extern "C" void PARAMETERIZER_API OGF_parameterizer_terminate() {
    OGF::parameterizer_libinit::decrement_users() ;
}


