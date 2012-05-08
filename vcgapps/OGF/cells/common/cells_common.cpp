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
 

#include <OGF/cells/common/common.h>
 
#include <OGF/basic/debug/logger.h>
#include <OGF/basic/modules/module.h>

#include <OGF/cells/types/cells_library.h>
#include <OGF/cells/map/map_editor.h>

/*#include <OGF/cells/io/map_serializer_obj.h>
#include <OGF/cells/io/map_serializer_off.h>
#include <OGF/cells/io/map_serializer_geom.h>
#include <OGF/cells/io/map_serializer_inventor.h>
#include <OGF/cells/io/map_serializer_vrml.h>
#include <OGF/cells/io/map_serializer_x3d.h>
#include <OGF/cells/io/map_serializer_3ds.h>
#include <OGF/cells/io/map_serializer_gocad.h>
#include <OGF/cells/io/map_serializer_ply.h>
#include <OGF/cells/io/map_serializer_tri.h>
#include <OGF/cells/io/map_serializer_lwo.h>
*/
#include <OGF/cells/map_algos/lscm.h>
#include <OGF/cells/map_algos/abf_plus_plus.h>
#include <OGF/cells/map_algos/floater.h>
#include <OGF/cells/map_algos/stretch_map_parameterizer.h>
#include <OGF/cells/map_algos/map_projection.h>

#include <OGF/cells/map_algos/variational_map_splitter.h>
#include <OGF/cells/points_algos/kdtree_search_ann.h>

#include <OGF/cells/triangulation/delaunay.h>
#include <OGF/cells/triangulation/delaunay_shewchuk.h>


namespace OGF {
    
/****************************************************************/

    static update_map_graphics_func update_map_graphics = nil ;
    
    void CELLS_API set_update_map_graphics_func(update_map_graphics_func f) {
        update_map_graphics = f ;
    }

    void CELLS_API update_graphics(Map* map) {
        if(update_map_graphics != nil) {
            update_map_graphics(map) ;
        }
    }
   
    void cells_libinit::initialize() {
        Logger::out("Init") << "Initializing library \'" 
                            << "cells" << "\'" << std::endl ; 
        //_____________________________________________________________


        CellsLibrary::initialize() ;

       /* ogf_declare_map_serializer<MapSerializer_obj>("obj") ;
        ogf_declare_map_serializer<MapSerializer_eobj>("eobj") ;
        ogf_declare_map_serializer<MapSerializer_off>("off") ;
        ogf_declare_map_serializer<MapSerializer_geom>("geom") ;
        ogf_declare_map_serializer<MapSerializer_Inventor>("iv") ;
        ogf_declare_map_serializer<MapSerializer_VRML>("wrl") ;
        ogf_declare_map_serializer<MapSerializer_X3D>("x3d") ;
        ogf_declare_map_serializer<MapSerializer_lib3DS>("3ds") ;
        ogf_declare_map_serializer<MapSerializer_Gocad>("ts") ;
        //ogf_declare_map_serializer<MapSerializer_ply>("ply") ;
        //ogf_declare_map_serializer<MapSerializer_ply2>("ply2") ;
        ogf_declare_map_serializer<MapSerializer_tri>("tri") ;
        ogf_declare_map_serializer<MapSerializer_lwo>("lwo") ;*/

        ogf_declare_map_parameterizer<MapParameterizerLSCM>("LSCM") ;
        ogf_declare_map_parameterizer<ABFPlusPlusMapParameterizer>("ABF++") ; 
        ogf_declare_map_parameterizer<MapParameterizerFloater>("Barycentric") ;
        ogf_declare_map_parameterizer<MapParameterizerMVC>("MeanValueCoordinates") ;
        ogf_declare_map_parameterizer<StretchMapParameterizer>("stretch") ;
        ogf_declare_map_parameterizer<MapParameterizerProjection>("Projection") ;

        ogf_declare_SpatialSearch<KdTreeSearchANN>("ANN");

        ogf_declare_map_component_splitter<VariationalMapComponentSplitter>("VSA") ;
        ogf_declare_map_component_splitter<SmoothVariationalMapComponentSplitter>("VSASmooth") ;

//        ogf_declare_Delaunay<DelaunayShewchuk>("ShewchukTriangle") ;        

        //_____________________________________________________________

        Module* module_info = new Module ;
        module_info->set_name("cells") ;
        module_info->set_vendor("OGF") ;
        module_info->set_version("1.0-a4") ;
        module_info->set_info("Polygonal surfaces and algorithms") ;
        Module::bind_module("cells", module_info) ;

        Logger::out("Init") << "Initialized library \'" 
                            << "cells" << "\'" << std::endl ; 
    }
    
    void cells_libinit::terminate() {
        Logger::out("Init") << "Terminating library \'" 
                            << "cells" << "\'" << std::endl ; 

        //_____________________________________________________________

        CellsLibrary::terminate() ;

        annClose();

        //_____________________________________________________________

        Module::unbind_module("cells") ;
        
        Logger::out("Init") << "Terminated library \'" 
                            << "cells" << "\'" << std::endl ; 
    }
    
// You should not need to modify this file below that point.
    
/****************************************************************/
    
    cells_libinit::cells_libinit() {
        increment_users() ;
    }

    cells_libinit::~cells_libinit() {
        decrement_users() ;
    }
    
    void cells_libinit::increment_users() {
        // Note that count_ is incremented before calling
        // initialize, else it would still be equal to
        // zero at module initialization time, which 
        // may cause duplicate initialization of libraries.
        count_++ ;
        if(count_ == 1) {
            initialize() ;
        }
    }
    
    void cells_libinit::decrement_users() {
        count_-- ;
        if(count_ == 0) {
            terminate() ;
        }
    }
    
    int cells_libinit::count_ = 0 ;
    
}

// The initialization and termination functions
// are also declared using C linkage in order to 
// enable dynamic linking of modules.

extern "C" void CELLS_API OGF_cells_initialize() {
    OGF::cells_libinit::increment_users() ;
}

extern "C" void CELLS_API OGF_cells_terminate() {
    OGF::cells_libinit::decrement_users() ;
}


